"""Public contracts shared by control-side callers and the isolated model process.

Observations use named float32 state components and RGB HWC uint8 camera arrays.
Returned actions are physical, absolute robot targets after checkpoint processors:
joint radians and gripper width in metres, in the configured canonical order.
"""
from __future__ import annotations

from typing import Any, Dict, Protocol, TypedDict

import numpy as np


class PolicyError(RuntimeError):
    """A model contract, dependency, transport or inference failure."""


Observation = Dict[str, Any]


class Prediction(TypedDict, total=False):
    """Exactly one of action/actions is present; metadata contains provenance."""
    action: np.ndarray
    actions: np.ndarray
    metadata: Dict[str, Any]


class PolicyBackend(Protocol):
    """Exclusive owner of model weights, processors and temporal state."""
    def load(self) -> "PolicyBackend": ...
    def infer(self, observation: Observation) -> Prediction: ...
    def reset(self) -> None: ...
    def invalidate(self) -> None: ...
    def close(self) -> None: ...


class PolicyTransport(Protocol):
    """Nonblocking control-loop interface; generation changes invalidate old work."""
    def load(self) -> "PolicyTransport": ...
    def submit(self, observation: Observation, generation: int, seq: int) -> None: ...
    def poll(self): ...
    def invalidate(self, generation: int) -> None: ...
    def close(self) -> None: ...
