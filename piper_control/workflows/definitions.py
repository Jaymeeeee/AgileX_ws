"""Workflow decisions are independent of hardware, policy families and ROS."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Workflow:
    name: str
    uses_policy: bool
    requires_recording: bool
    learns: bool
    initial_command: str


WORKFLOWS = {
    "deploy": Workflow("deploy", True, False, False, "p"),
    "teleop": Workflow("teleop", False, False, False, "h"),
    "collect": Workflow("collect", False, True, False, "h"),
    "online_rl": Workflow("online_rl", True, True, True, "p"),
}


def get_workflow(name):
    try:
        return WORKFLOWS[name]
    except KeyError as exc:
        raise ValueError("Unknown workflow: %s" % name) from exc
