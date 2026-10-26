"""Tutorial-style one-way leader CAN forwarding, with no motion filtering.

Only the joint mode/target and gripper command IDs belong to this route. Local
SocketCAN echoes are excluded. The lock makes pause wait for a current send,
so a model command and a forwarded human command cannot share the writer.
"""
import errno
import math
import struct
import threading
import time

import numpy as np


JOINT_IDS = (0x155, 0x156, 0x157)
TARGET_IDS = JOINT_IDS + (0x159,)
FORWARD_IDS = (0x151,) + TARGET_IDS
JOINT_UNIT = math.pi / 180000.0
SEND_RETRY_TIMEOUT_SEC = 0.01
SEND_RETRY_INTERVAL_SEC = 0.0005


def open_bus(channel):
    import can
    return can.Bus(interface="socketcan", channel=channel,
                   receive_own_messages=False)


class CANForwarder:
    """Forward command bytes unchanged; decode complete sets only for recording."""

    def __init__(self, source, destination, bus_factory=None):
        self.source, self.destination = source, destination
        self.bus_factory = bus_factory or open_bus
        self.rx = self.tx = None
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.thread = None
        self.active = False
        self.generation = 0
        self.error = None
        self.parts = {}
        self.sequence = 0
        self._target = None
        self.sent_frames = 0
        self.send_retries = 0

    def start(self):
        self.rx = self.bus_factory(self.source)
        try:
            self.tx = self.bus_factory(self.destination)
        except Exception:
            self.rx.shutdown()
            raise
        self.thread = threading.Thread(target=self._run, daemon=True,
                                       name="piper-forward-" + self.source)
        self.thread.start()

    def activate(self):
        with self.lock:
            self.generation += 1
            self.parts.clear()
            self._target = None
            self.active = True

    def deactivate(self):
        with self.lock:
            self.active = False
            self.generation += 1
            self.parts.clear()
            self._target = None

    def forward(self, frame, generation=None):
        with self.lock:
            if (not self.active or (generation is not None and generation != self.generation)
                    or not frame.is_rx or frame.is_extended_id
                    or frame.is_remote_frame or frame.is_error_frame
                    or frame.arbitration_id not in FORWARD_IDS):
                return False
            if not self._send(frame):
                return False
            self.sent_frames += 1
            if frame.arbitration_id in TARGET_IDS:
                self.parts[frame.arbitration_id] = (
                    float(frame.timestamp), bytes(frame.data), time.monotonic())
                if all(ident in self.parts for ident in TARGET_IDS):
                    self._assemble()
            return True

    def _send(self, frame):
        """Retry a transient full kernel queue within one short send budget.

        SocketCAN can report ENOBUFS even after select() says it is writable;
        python-can's send(timeout=...) does not retry that syscall failure.
        Keep frame order and bytes, without an extra application-level queue.
        The caller holds the mode lock, so pause waits at most this send budget
        plus OS scheduling time before it can deactivate the route.
        """
        deadline = time.monotonic() + SEND_RETRY_TIMEOUT_SEC
        attempted = False
        while True:
            if self.closed.is_set():
                return False
            try:
                if attempted:
                    self.send_retries += 1
                attempted = True
                self.tx.send(frame, timeout=max(0.0, deadline - time.monotonic()))
                return True
            except Exception as exc:
                code = getattr(exc, "error_code", None)
                if code is None:
                    code = getattr(exc, "errno", None)
                if code not in (errno.ENOBUFS, errno.EAGAIN):
                    raise
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        "CAN TX queue on %s remained congested for %.0f ms: %s"
                        % (self.destination, SEND_RETRY_TIMEOUT_SEC * 1000, exc)
                    ) from exc
                time.sleep(min(SEND_RETRY_INTERVAL_SEC, remaining))
                # Do not send a delayed command once the retry budget expires.
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "CAN TX queue on %s remained congested for %.0f ms: %s"
                        % (self.destination, SEND_RETRY_TIMEOUT_SEC * 1000, exc)
                    ) from exc

    def _assemble(self):
        entries = [self.parts[ident] for ident in TARGET_IDS]
        joints = [value * JOINT_UNIT for entry in entries[:3]
                  for value in struct.unpack(">ii", entry[1])]
        gripper = struct.unpack(">i", entries[3][1][:4])[0] * 1e-6
        self.sequence += 1
        self._target = dict(position=np.asarray(joints + [gripper], dtype=float),
                            stamp=min(entry[0] for entry in entries),
                            monotonic=min(entry[2] for entry in entries),
                            sequence=self.sequence, source="forwarded_command")
        self.parts.clear()

    def target(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError("CAN forwarding failed: " + str(self.error)) from self.error
            if self._target is None or not self.active:
                return None
            return dict(self._target, position=self._target["position"].copy())

    def _run(self):
        try:
            while not self.closed.is_set():
                with self.lock:
                    generation = self.generation
                frame = self.rx.recv(timeout=.02)
                if frame is not None:
                    self.forward(frame, generation)
        except Exception as exc:
            if not self.closed.is_set():
                self.error = exc
            self.deactivate()

    def close(self):
        self.closed.set()
        self.deactivate()
        if self.thread is not None:
            self.thread.join(timeout=.2)
        for bus in (self.rx, self.tx):
            if bus is not None:
                bus.shutdown()
        self.rx = self.tx = None
