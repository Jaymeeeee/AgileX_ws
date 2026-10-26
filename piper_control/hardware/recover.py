"""Send the SDK reset command to each configured leader and follower.

Run without --execute to print the selected channels without connecting.
"""

import argparse
import json
import signal
import sys

from ..config import load_config
from .interfaces import configured_devices
from .robot import SDKRobot


def recover_software_stop(robot):
    """Issue reset once per connected arm; no status polling or motion commands."""
    reset = []
    for key, device in robot.devices.items():
        device["sdk"].reset()
        reset.append("{}/{}".format(*key))
    return {"reset_commands": reset}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--execute", action="store_true", help="Connect and send SDK reset commands"
    )
    args = parser.parse_args(argv)
    handlers, robot = {}, None
    try:
        cfg = load_config(args.config)
        devices = {}
        for side, role, spec in configured_devices(cfg["robot"]):
            devices.setdefault(side, {})[role] = spec
        plan = {
            "operation": "reset",
            "arms": cfg["robot"]["arms"],
            "firmware": cfg["robot"]["firmware"],
            "execute": args.execute,
            "devices": devices,
        }
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        if not args.execute:
            return 0
        robot = SDKRobot(cfg["robot"])

        def interrupt(signum, frame):
            raise KeyboardInterrupt("Recovery interrupted")

        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt)
        robot.connect()
        print(json.dumps(recover_software_stop(robot), ensure_ascii=False, indent=2))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("Recovery failed: {}".format(exc), file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        try:
            if robot is not None:
                robot.close()
        finally:
            for signum, handler in handlers.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())
