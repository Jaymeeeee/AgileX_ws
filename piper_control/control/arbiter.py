"""Choose model or human control and pass physical targets directly to the SDK."""

from collections import deque
import threading
import time

import numpy as np


class ControlError(RuntimeError):
    pass


class Controller:
    HOLD = "HOLD"
    POLICY = "POLICY"
    HUMAN = "HUMAN"
    TRANSITION = "TRANSITION"
    FAULT = "FAULT"

    def __init__(self, config, robot, event_sink=None):
        self.config, self.robot = config, robot
        self.state, self.generation = self.HOLD, 0
        self.dimension = 7 * len(config["robot"]["arms"])
        self.actions = deque()
        self.target = None
        self.pending_seq = None
        self.last_source = "HOLD"
        self.last_rlt_ref_row = None
        self.policy_started = False
        self.event_sink = event_sink or (lambda event: None)
        self.abort = threading.Event()

    def _event(self, kind, **fields):
        self.event_sink(
            dict(
                type=kind,
                timestamp=time.time(),
                generation=self.generation,
                control_state=self.state,
                **fields,
            )
        )

    def _vector(self, value):
        value = np.asarray(value, dtype=np.float64)
        if value.shape != (self.dimension,) or not np.isfinite(value).all():
            raise ControlError(
                "Expected a finite %d-dimensional action/state" % self.dimension
            )
        return value.copy()

    def invalidate(self, reason):
        self.generation += 1
        self.actions.clear()
        self.pending_seq = None
        self.policy_started = False
        self.last_rlt_ref_row = None
        self._event("invalidate", reason=reason)

    def start(self):
        if self.abort.is_set():
            raise ControlError("Stop requested before enabling")
        self.robot.enable()
        if self.abort.is_set():
            raise ControlError("Stop requested during enabling")
        self.target = self._vector(self.robot.snapshot()["state"])
        if self.abort.is_set():
            raise ControlError("Stop requested before startup alignment")
        self.robot.command(self.target)
        self._event("ready")

    def request_stop(self):
        """May run on input thread; independent of model inference/mode polling."""
        self.abort.set()
        request = getattr(self.robot, "request_stop", None)
        if request:
            request()

    def _transition(self, destination):
        if self.abort.is_set() or self.state == self.FAULT:
            raise ControlError(
                "Controller is stopped; restart after inspecting the cause"
            )
        if destination == self.HUMAN and not self.robot.supports_human:
            raise ControlError("Human control requires a leader for every selected arm")
        old = self.state
        if old == destination:
            return
        self.invalidate("%s->%s" % (old, destination))
        self.state = self.TRANSITION
        if old == self.HUMAN:
            self.robot.exit_human()
        if destination == self.HUMAN:
            self.robot.enter_human()
        else:
            self.target = self._vector(self.robot.snapshot()["state"])
            self.robot.command(self.target)
        self.state = destination
        self._event("mode", previous=old)

    def enter_policy(self):
        self._transition(self.POLICY)

    def enter_human(self):
        self._transition(self.HUMAN)

    def pause(self):
        self._transition(self.HOLD)

    def expect(self, seq):
        if self.state != self.POLICY or self.pending_seq is not None:
            return False
        self.pending_seq = seq
        return True

    def offer(self, response):
        """Accept only the outstanding result in the current control epoch."""
        if (
            self.state != self.POLICY
            or response.get("generation") != self.generation
            or response.get("seq") != self.pending_seq
            or self.pending_seq is None
        ):
            return False
        self.pending_seq = None
        if response.get("error"):
            raise ControlError("Policy failed: %s" % response["error"])
        values = np.asarray(
            response.get("actions", response.get("action")), dtype=np.float64
        )
        if values.ndim == 1:
            values = values[None, :]
        if (
            values.ndim != 2
            or values.shape[1] != self.dimension
            or not 0 < len(values) <= 4096
        ):
            raise ControlError("Policy returned an invalid action trajectory")
        if not np.isfinite(values).all():
            raise ControlError("Policy returned nonfinite actions")
        metadata = dict(response.get("metadata") or {})
        refs = (metadata.get("rlt_boundary") or {}).get("ref_chunk")
        if refs is not None:
            refs = np.asarray(refs, dtype=np.float32)
            if (
                refs.ndim != 2
                or refs.shape[1] != self.dimension
                or len(refs) < len(values)
                or not np.isfinite(refs).all()
            ):
                raise ControlError("Policy returned an invalid RLT reference chunk")
        for index, value in enumerate(values):
            item_meta = dict(metadata)
            if refs is not None:
                # Normalized model-order reference from the prediction that
                # actually supplied this target, even across a later replan.
                item_meta["rlt_ref_row"] = refs[index].copy()
            if index:
                item_meta.pop("rlt_boundary", None)
            self.actions.append((value.copy(), item_meta, response["seq"], index))
        return True

    def step(self, snapshot):
        """One policy-rate interval; target remains constant between these calls."""
        if self.abort.is_set():
            raise ControlError("Stop requested")
        if self.state == self.HUMAN:
            action = snapshot.get("human_action")
            if action is None:
                getter = getattr(self.robot, "human_action", None)
                action = getter() if getter else None
            if action is None:
                return None
            action = self._vector(action)
            self.last_source = "HUMAN"
            return {
                "action": action,
                "predicted_action": None,
                "source": "HUMAN",
                "metadata": {"human_action_seq": snapshot.get("human_action_seq")},
                "generation": self.generation,
                "control_state": self.state,
            }
        if self.state != self.POLICY:
            return None
        if not self.actions:
            if not self.policy_started:
                return None
            self.robot.command(self.target)
            return {
                "action": self.target.copy(),
                "predicted_action": None,
                "source": self.last_source,
                "metadata": (
                    {"rlt_ref_row": self.last_rlt_ref_row.copy()}
                    if self.last_rlt_ref_row is not None
                    else {}
                ),
                "generation": self.generation,
                "control_state": self.state,
                "waiting_for_policy": True,
            }
        predicted, metadata, seq, index = self.actions.popleft()
        source = metadata.get("source", "BASE")
        if source not in ("BASE", "RL"):
            raise ControlError("Unexpected model action source: %s" % source)
        self.robot.command(predicted)
        sent = predicted.copy()
        self.target = sent
        self.policy_started = True
        self.last_source = source
        self.last_rlt_ref_row = metadata.get("rlt_ref_row")
        return {
            "action": sent.copy(),
            "predicted_action": predicted,
            "source": self.last_source,
            "metadata": metadata,
            "seq": seq,
            "chunk_index": index,
            "generation": self.generation,
            "control_state": self.state,
        }

    def tick(self, snapshot=None):
        if self.abort.is_set():
            raise ControlError("Stop requested")

    def fail(self, reason):
        self.state = self.FAULT
        self.abort.set()
        self.generation += 1
        self.actions.clear()
        self.pending_seq = None
        self.policy_started = False
        try:
            self.robot.stop()
        finally:
            self._event("fault", reason=reason)

    def close(self):
        try:
            self.robot.stop()
        finally:
            self.invalidate("close")
