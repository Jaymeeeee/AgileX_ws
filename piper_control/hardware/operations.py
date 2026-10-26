"""Send explicit joint/gripper targets directly through the SDK.

Home sends one target. Replay sends the listed targets at their timestamps.
After the last target, --hold-seconds controls the wait before stop/disconnect.
Run without --execute/--mock to print the commands without connecting.
"""
import argparse
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np
import yaml

from ..config import component_names, load_config
from .robot import MockRobot, SDKRobot


ACTION_SEMANTICS = "absolute_joint_position_rad_gripper_m"


def _read(path):
    path = Path(path).expanduser()
    if path.suffix.lower() == ".npz":
        with np.load(str(path), allow_pickle=False) as values:
            return {key: values[key].tolist() for key in values.files}
    with path.open() as stream:
        return json.load(stream) if path.suffix.lower() == ".json" else yaml.safe_load(stream)


def _trajectory(values, config):
    if not isinstance(values, dict):
        raise ValueError("Trajectory must be a mapping")
    names = component_names(config["arms"])
    if values.get("action_names") != names:
        raise ValueError("action_names must match selected arms in canonical joint/gripper order")
    if values.get("action_semantics") != ACTION_SEMANTICS:
        raise ValueError("Trajectory must explicitly use absolute joint radians and gripper metres")
    try:
        actions = np.asarray(values["actions"], dtype=float)
        stamps = np.asarray(values["timestamps"], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Trajectory needs numeric actions[N,D] and timestamps[N]") from exc
    if actions.ndim != 2 or actions.shape[1] != len(names) or not len(actions):
        raise ValueError("Trajectory actions must have nonempty shape [N, {}]".format(len(names)))
    if stamps.shape != (len(actions),) or not np.isfinite(stamps).all():
        raise ValueError("timestamps must contain one finite value per action")
    if stamps[0] != 0 or np.any(np.diff(stamps) <= 0):
        raise ValueError("timestamps must start at zero and increase strictly in seconds")
    if not np.isfinite(actions).all():
        raise ValueError("Trajectory targets must contain finite values")
    return dict(action_names=names, action_semantics=ACTION_SEMANTICS,
                actions=actions.copy(), timestamps=stamps.copy())


def load_trajectory(path, robot_cfg):
    """Load an explicit JSON/YAML/NPZ action schema; no implicit unit conversion."""
    return _trajectory(_read(path), robot_cfg)


def home_trajectory(robot_cfg, path=None):
    """Use configured per-arm home targets or a schema-checked one-waypoint file."""
    if path is not None:
        result = load_trajectory(path, robot_cfg)
        if len(result["actions"]) != 1:
            raise ValueError("Home target file must contain exactly one waypoint")
        return result
    home = robot_cfg.get("home_position", {})
    if not isinstance(home, dict) or any(side not in home for side in robot_cfg["arms"]):
        raise ValueError("Provide robot.home_position for every selected arm or --target-file")
    if any(not isinstance(home[side], (list, tuple)) or len(home[side]) != 7
           for side in robot_cfg["arms"]):
        raise ValueError("Each home_position must contain six joint radians and gripper metres")
    return _trajectory(dict(action_names=component_names(robot_cfg["arms"]),
                            action_semantics=ACTION_SEMANTICS,
                            actions=[[value for side in robot_cfg["arms"] for value in home[side]]],
                            timestamps=[0.]), robot_cfg)


def execute_trajectory(robot_cfg, trajectory, robot=None, hold_seconds=2.):
    """Send each target once, wait a fixed duration, then stop/disconnect.

    The final wait is elapsed time only; it does not verify arrival.
    """
    trajectory = _trajectory(trajectory, robot_cfg)
    if not np.isfinite(hold_seconds) or hold_seconds < 0:
        raise ValueError("hold_seconds must be a finite nonnegative duration")
    robot = robot if robot is not None else SDKRobot(robot_cfg)
    commands = 0
    try:
        robot.connect()
        robot.enable()
        started = time.monotonic()
        for stamp, target in zip(trajectory["timestamps"], trajectory["actions"]):
            delay = started + float(stamp) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            robot.command(target)
            commands += 1
        if hold_seconds:
            time.sleep(hold_seconds)
        return dict(operation="trajectory", waypoints=len(trajectory["actions"]),
                    commands=commands, elapsed_sec=time.monotonic() - started,
                    final_target=trajectory["actions"][-1].tolist())
    finally:
        try:
            robot.stop()
        finally:
            robot.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("home", "replay"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--target-file", help="One-waypoint home target with explicit action schema")
    parser.add_argument("--trajectory", help="JSON/YAML/NPZ trajectory with explicit action schema")
    parser.add_argument("--hold-seconds", type=float, default=2.,
                        help="Fixed wait after the final command before stop/disconnect (default: 2)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="Allow real SDK motion")
    mode.add_argument("--mock", action="store_true", help="Execute using the simulated robot")
    args = parser.parse_args(argv)
    if args.operation == "replay" and not args.trajectory:
        parser.error("replay requires --trajectory")
    if args.operation == "home" and args.trajectory:
        parser.error("home uses --target-file, not --trajectory")
    if args.operation == "replay" and args.target_file:
        parser.error("--target-file applies only to home")
    handlers = {}
    try:
        if not np.isfinite(args.hold_seconds) or args.hold_seconds < 0:
            raise ValueError("--hold-seconds must be a finite nonnegative duration")
        cfg = load_config(args.config)["robot"]
        trajectory = (home_trajectory(cfg, args.target_file) if args.operation == "home"
                      else load_trajectory(args.trajectory, cfg))
        print(json.dumps(dict(operation=args.operation, execute=args.execute, mock=args.mock,
                              arms=cfg["arms"], action_names=trajectory["action_names"],
                              action_semantics=ACTION_SEMANTICS, waypoints=len(trajectory["actions"]),
                              duration_sec=float(trajectory["timestamps"][-1]),
                              hold_seconds=args.hold_seconds,
                              first_target=trajectory["actions"][0].tolist(),
                              final_target=trajectory["actions"][-1].tolist()), indent=2))
        if not (args.execute or args.mock):
            return 0
        def interrupt(signum, frame):
            raise KeyboardInterrupt("Trajectory interrupted")
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt)
        print(json.dumps(execute_trajectory(cfg, trajectory, MockRobot(cfg) if args.mock else None,
                                            hold_seconds=args.hold_seconds)))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("{} failed: {}".format(args.operation, exc), file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
