"""Asynchronous local policy process transport, independent of the model library."""
from __future__ import annotations

import collections
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

from ..ipc import _owned, read_message, write_message
from .interfaces import PolicyError


class PolicyClient:
    """Bounded asynchronous subprocess client; never perform inference in control loop.

    One inference may run and at most one further snapshot waits. Commands are
    serialized with inference so replay records and learner mutations stay ordered.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.generation = 0
        self.process = None
        self.responses = queue.Queue()
        self.commands = collections.deque()
        self.condition = threading.Condition()
        self.closed = False
        self.active_since = None
        self.active_seq = None
        self.busy = False
        self.command_since = None
        self.command_kind = None
        self._snapshot_workspace = None
        self.timeout = float(cfg["policy"].get("inference_timeout_sec", 2))

    def load(self):
        if self.process is not None:
            return self
        python = self.cfg["policy"].get("python") or sys.executable
        env = dict(os.environ)
        # Avoid host Python paths leaking into a different interpreter environment.
        env.pop("PYTHONPATH", None)
        env["PYTHONNOUSERSITE"] = "1"
        env["HF_HUB_OFFLINE"] = "1"
        env["TRANSFORMERS_OFFLINE"] = "1"
        worker_cfg = _owned(self.cfg)
        if worker_cfg.get("workflow", {}).get("mode") == "online_rl":
            rlt_options = worker_cfg["workflow"].setdefault("rlt", {})
            scratch_parent = rlt_options.get("snapshot_dir")
            if scratch_parent:
                Path(scratch_parent).mkdir(parents=True, exist_ok=True)
            self._snapshot_workspace = tempfile.TemporaryDirectory(prefix="piper-rlt-worker-", dir=scratch_parent)
            rlt_options["snapshot_dir"] = self._snapshot_workspace.name
        self.process = subprocess.Popen([python, "-m", "piper_control.policies.worker"],
            cwd=str(Path(__file__).resolve().parents[2]), env=env,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        threading.Thread(target=self._reader, daemon=True).start()
        write_message(self.process.stdin, {"kind": "load", "config": worker_cfg})
        try:
            reply = self.responses.get(timeout=float(self.cfg["policy"].get("startup_timeout_sec", 120)))
        except queue.Empty as exc:
            self.close()
            raise PolicyError("Policy worker startup timed out") from exc
        if reply.get("error") or reply.get("kind") != "ready":
            self.close()
            raise PolicyError(reply.get("error", "Policy worker failed to start"))
        self.capabilities = reply
        threading.Thread(target=self._writer, daemon=True).start()
        return self

    def _reader(self):
        try:
            while not self.closed:
                reply = read_message(self.process.stdout)
                with self.condition:
                    self.busy = False
                    self.command_since = None
                    self.command_kind = None
                    if reply.get("kind") == "infer":
                        self.active_since = None
                        self.active_seq = None
                    self.condition.notify_all()
                self.responses.put(reply)
        except Exception as exc:
            if not self.closed:
                self.responses.put({"kind": "error", "error": str(exc)})

    def _writer(self):
        try:
            while True:
                with self.condition:
                    while (not self.commands or self.busy) and not self.closed:
                        self.condition.wait()
                    if self.closed:
                        return
                    command = self.commands.popleft()
                    self.busy = True
                    if command["kind"] == "infer":
                        if command["generation"] != self.generation:
                            self.busy = False
                            continue
                        self.active_since = time.monotonic()
                        self.active_seq = command["seq"]
                    self.command_since = time.monotonic()
                    self.command_kind = command["kind"]
                write_message(self.process.stdin, command)
        except Exception as exc:
            if not self.closed:
                self.responses.put({"kind": "error", "error": str(exc)})

    def _enqueue(self, message):
        if self.closed or self.process is None:
            raise PolicyError("Policy client is not running")
        with self.condition:
            if len(self.commands) >= int(self.cfg["policy"].get("max_pending_commands", 64)):
                raise PolicyError("Policy command queue overflow; stop collecting rather than lose executed actions")
            self.commands.append(_owned(message))
            self.condition.notify_all()

    def submit(self, observation, generation, seq):
        self.generation = generation
        with self.condition:
            self.commands = collections.deque(m for m in self.commands if m["kind"] != "infer")
            self._enqueue({"kind": "infer", "observation": observation, "generation": generation, "seq": seq})

    def poll(self):
        with self.condition:
            timeout = (float(self.cfg["policy"].get("update_timeout_sec",
                       self.cfg.get("workflow", {}).get("rlt", {}).get("update_timeout_sec", 600)))
                       if self.command_kind in ("end_episode", "update", "save") else self.timeout)
            if self.command_since is not None and time.monotonic() - self.command_since > timeout:
                raise PolicyError("Policy worker %s timed out (seq=%s); invalidate control and restart worker" %
                                  (self.command_kind, self.active_seq))
        while True:
            try:
                reply = self.responses.get_nowait()
            except queue.Empty:
                return None
            if reply.get("error"):
                raise PolicyError(reply["error"])
            if reply.get("kind") == "infer" and reply.get("generation") != self.generation:
                continue
            return reply

    def invalidate(self, generation):
        with self.condition:
            self.generation = generation
            self.commands = collections.deque(m for m in self.commands if m["kind"] != "infer")
            self._enqueue({"kind": "reset", "generation": generation})

    def reset(self):
        self.invalidate(self.generation + 1)

    def record_step(self, action, source, intervention=False, reward=0,
                    boundary=None, next_observation=None, ref_row=None):
        self._enqueue({"kind": "record_step", "action": np.asarray(action, dtype=np.float32),
                       "source": source, "intervention": intervention, "reward": reward,
                       "boundary": boundary, "next_observation": next_observation, "ref_row": ref_row})

    def boundary(self, observation, episode_id=None, step_id=None, segment_start=False):
        self._enqueue({"kind": "boundary", "observation": observation,
                       "episode_id": episode_id, "step_id": step_id, "segment_start": segment_start})

    def end_episode(self, observation, terminal=False, reward=0., success=False, truncated=False):
        self._enqueue({"kind": "end_episode", "observation": observation, "terminal": terminal,
                       "reward": reward, "success": success, "truncated": truncated})

    def update(self):
        self._enqueue({"kind": "update"})

    def save(self, path):
        self._enqueue({"kind": "save", "path": str(path)})

    def close(self):
        self.closed = True
        with self.condition:
            self.condition.notify_all()
        try:
            if self.process is not None:
                if self.process.poll() is None:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=3)
                for stream in (self.process.stdin, self.process.stdout):
                    if stream is not None:
                        stream.close()
        finally:
            if self._snapshot_workspace is not None:
                self._snapshot_workspace.cleanup()
                self._snapshot_workspace = None
