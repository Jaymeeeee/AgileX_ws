"""Manually send joint and gripper disable commands to the selected arms.

Run without --execute to preview the devices. This does not change the normal
workflow shutdown behavior or issue enable, reset, mode or position commands.
"""

import argparse
import json
import signal
import sys

from ..config import load_config
from .interfaces import configured_devices
from .robot import RobotError, SDKRobot


def disable_motors(robot):
    """Send each disable command once; SDK return values are cached feedback."""
    joints, grippers, errors = [], [], []
    for key, device in robot.devices.items():
        name = "{}/{}".format(*key)
        for kind, command, sent in (
            ("joints", device["sdk"].disable, joints),
            ("gripper", device["gripper"].disable_gripper, grippers),
        ):
            try:
                command()
                sent.append(name)
            except Exception as exc:
                errors.append("{} {}: {}".format(name, kind, exc))
    if errors:
        raise RobotError("Motor disable failed: " + "; ".join(errors))
    return {
        "joint_disable_commands": joints,
        "gripper_disable_commands": grippers,
        "feedback_verified": False,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--execute", action="store_true", help="Connect and send disable commands"
    )
    args = parser.parse_args(argv)
    handlers, robot, exit_code = {}, None, 0
    try:
        cfg = load_config(args.config)
        devices = {}
        for side, role, spec in configured_devices(cfg["robot"]):
            devices.setdefault(side, {})[role] = spec
        plan = {
            "operation": "disable",
            "arms": cfg["robot"]["arms"],
            "execute": args.execute,
            "devices": devices,
        }
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        if args.execute:
            robot = SDKRobot(cfg["robot"])

            def interrupt(signum, frame):
                raise KeyboardInterrupt("Disable interrupted")

            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, interrupt)
            robot.connect()
            print(json.dumps(disable_motors(robot), ensure_ascii=False, indent=2))
    except (Exception, KeyboardInterrupt) as exc:
        print("Disable failed: {}".format(exc), file=sys.stderr)
        exit_code = 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        try:
            # This instance never enables control, so close only disconnects.
            if robot is not None:
                robot.close()
        except Exception as exc:
            print("Disconnect failed: {}".format(exc), file=sys.stderr)
            exit_code = exit_code or 1
        finally:
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
