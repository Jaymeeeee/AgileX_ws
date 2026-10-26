"""Observation-only process transport for optional ROS collection.

This module has no ROS imports. Replies acknowledge collection operations; no
worker response is a robot command or is passed to the hardware executor.
"""
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

from piper_control.ipc import _owned, read_message, write_message


class CollectionError(RuntimeError):
    pass


class CollectorClient:
    def __init__(self, config, directory):
        self.config = config
        self.options = config["record"]
        self.directory = Path(directory) if directory is not None else None
        self.process = None
        self.queue = queue.Queue(maxsize=int(self.options.get("queue_size", 256)))
        self.replies = queue.Queue(maxsize=2)
        self.timeout = float(self.options.get("command_timeout_sec", 5.))
        self.shutdown_timeout = float(self.options.get("shutdown_timeout_sec", 20.))
        self._sequence = 0
        self._error = None
        self._closing = False
        self._closed = False
        self._close_ack = threading.Event()
        self._writer_thread = None
        self._reader_thread = None
        self._image_stamps = {}
        self._active_since = None
        self._active_kind = None

    def _argv(self):
        python = self.options.get("python") or sys.executable
        setup = self.options.get("ros_setup")
        if setup:
            if not Path(setup).is_file():
                raise CollectionError("ROS setup script does not exist: " + str(setup))
            # Arguments are positional; neither user paths nor configuration
            # values are interpolated into executable shell source.
            return ["bash", "--noprofile", "--norc", "-c",
                    'source "$1" && exec "$2" -m piper_control.recording.ros_worker',
                    "piper-collection", str(setup), str(python)]
        return [str(python), "-m", "piper_control.recording.ros_worker"]

    def start(self):
        if self.process is not None or self._closed:
            raise CollectionError("Collection client cannot be restarted")
        env = dict(os.environ)
        env["PYTHONNOUSERSITE"] = "1"
        if self.options.get("ros_setup"):
            # Source only the selected ROS installation, never an inherited
            # catkin workspace or the control interpreter's Python packages.
            for key in ("PYTHONPATH", "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH"):
                env.pop(key, None)
        self.process = subprocess.Popen(self._argv(), cwd=str(Path(__file__).resolve().parents[2]),
                                        env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self._reader_thread = threading.Thread(target=self._read, name="collection-ack", daemon=True)
        self._reader_thread.start()
        try:
            write_message(self.process.stdin, {"kind": "start", "seq": 0,
                          "config": self.config,
                          "directory": str(self.directory) if self.directory is not None else None})
            timeout = float(self.options.get("startup_timeout_sec", 20.))
            reply = self._reply(timeout)
            if reply.get("kind") != "ready" or reply.get("seq") != 0:
                raise CollectionError("Collection worker did not acknowledge startup")
            self._writer_thread = threading.Thread(target=self._write, name="collection-input", daemon=True)
            self._writer_thread.start()
        except BaseException:
            self._terminate()
            self._closed = True
            raise
        return self

    def _read(self):
        try:
            while not self._closed:
                reply = read_message(self.process.stdout)
                self.replies.put_nowait(reply)
                if reply.get("kind") == "closed":
                    return
        except Exception as exc:
            if not self._closed and not self._close_ack.is_set():
                self._error = CollectionError("Collection worker connection failed: {}".format(exc))

    def _reply(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                reply = self.replies.get(timeout=min(.1, max(.001, deadline - time.monotonic())))
            except queue.Empty:
                if self._error is not None:
                    raise self._error
                if self.process.poll() is not None:
                    raise CollectionError("Collection worker exited before acknowledging input")
                continue
            if reply.get("error"):
                raise CollectionError("Collection worker: " + str(reply["error"]))
            return reply
        raise CollectionError("Collection worker acknowledgement timed out")

    def _write(self):
        try:
            while True:
                message = self.queue.get()
                self._active_kind = message["kind"]
                self._active_since = time.monotonic()
                write_message(self.process.stdin, message)
                closing = message["kind"] == "close"
                reply = self._reply(self.shutdown_timeout if closing else self.timeout)
                if reply.get("seq") != message["seq"] or reply.get("kind") != ("closed" if closing else "ack"):
                    raise CollectionError("Collection acknowledgement does not match submitted input")
                self._active_since = None
                self._active_kind = None
                if closing:
                    self._close_ack.set()
                    return
        except Exception as exc:
            self._error = exc

    def _enqueue(self, kind, payload):
        if self.process is None or self._closed or self._closing:
            raise CollectionError("Collection process is not accepting input")
        self.check_health()
        self._sequence += 1
        message = {"kind": kind, "seq": self._sequence, "payload": _owned(payload),
                   "captured_epoch": time.time(), "captured_monotonic": time.monotonic()}
        try:
            self.queue.put_nowait(message)
        except queue.Full as exc:
            self._error = CollectionError("Collection queue is full; recording cannot silently drop samples")
            raise self._error from exc

    def publish_state(self, snapshot):
        self._enqueue("state", snapshot)

    def publish_images(self, frames):
        stamp_keys = ("stamp", "depth_stamp", "aligned_depth_stamp")
        fresh = {
            name: frame
            for name, frame in frames.items()
            if tuple(frame.get(key) for key in stamp_keys)
            != self._image_stamps.get(name)
        }
        if fresh:
            self._enqueue("images", fresh)
            for name, frame in fresh.items():
                self._image_stamps[name] = tuple(frame.get(key) for key in stamp_keys)

    def publish_action(self, action, source, metadata=None):
        self._enqueue("action", {"action": action, "source": source, "metadata": metadata or {}})

    def publish_event(self, event):
        self._enqueue("event", event)

    def check_health(self):
        if self._error is not None:
            raise CollectionError(str(self._error)) from self._error
        if self._closed:
            return
        if self._active_since is not None:
            limit = self.shutdown_timeout if self._active_kind == "close" else self.timeout
            if time.monotonic() - self._active_since > limit:
                raise CollectionError("Collection worker input stalled")
        if self.process is not None and self.process.poll() is not None and not self._close_ack.is_set():
            raise CollectionError("Collection worker exited unexpectedly")

    def _terminate(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            # Worker SIGTERM handler closes its ROS bag in finally.
            self.process.terminate()
            try:
                self.process.wait(timeout=self.shutdown_timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.)
        for stream in (self.process.stdin, self.process.stdout):
            if stream is not None:
                stream.close()

    def close(self):
        if self._closed:
            return
        self._closing = True
        try:
            if self.process is not None and self._error is None and self._writer_thread is not None:
                self._sequence += 1
                deadline = time.monotonic() + self.shutdown_timeout
                while True:
                    self.check_health()
                    try:
                        self.queue.put({"kind": "close", "seq": self._sequence}, timeout=.1)
                        break
                    except queue.Full:
                        if time.monotonic() > deadline:
                            raise CollectionError("Collection shutdown queue timed out")
                while not self._close_ack.wait(.05):
                    self.check_health()
                    if time.monotonic() > deadline:
                        raise CollectionError("Collection did not finalize its bag before shutdown")
                self._writer_thread.join(timeout=1.)
                self.process.wait(timeout=3.)
                if self.process.returncode != 0:
                    raise CollectionError("Collection worker exited with an error after closing")
            self.check_health()
        finally:
            self._closed = True
            self._terminate()
