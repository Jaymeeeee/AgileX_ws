"""Discover SocketCAN identities or preview setup; apply only with --execute."""

import argparse
import json
import os
import subprocess
import sys

from ..config import load_config
from .interfaces import configured_devices, discover_interfaces, resolve_robot_config


def configuration_plan(config, interfaces=None):
    bitrate = config.get("bitrate", 1000000)
    if isinstance(bitrate, bool) or not isinstance(bitrate, int) or bitrate <= 0:
        raise ValueError("robot.bitrate must be a positive integer")
    tx_queue_len = config.get("can_tx_queue_len", 64)
    if type(tx_queue_len) is not int or tx_queue_len <= 0:
        raise ValueError("robot.can_tx_queue_len must be a positive integer")
    resolved = resolve_robot_config(config, interfaces=interfaces)
    commands, channels = [], {}
    for side, role, spec in configured_devices(resolved):
        channel = spec["channel"]
        channels.setdefault(side, {})[role] = channel
        commands.extend(
            [
                ["ip", "link", "set", "dev", channel, "down"],
                [
                    "ip",
                    "link",
                    "set",
                    "dev",
                    channel,
                    "type",
                    "can",
                    "bitrate",
                    str(bitrate),
                ],
                ["ip", "link", "set", "dev", channel, "txqueuelen", str(tx_queue_len)],
                ["ip", "link", "set", "dev", channel, "up"],
            ]
        )
    return dict(commands=commands, resolved=channels)


def configure_interfaces(config, plan=None):
    """Apply one resolved plan, without a shell or a second identity lookup."""
    if plan is None:
        plan = configuration_plan(config)
    prefix = [] if os.geteuid() == 0 else ["sudo", "-n"]
    for command in plan["commands"]:
        subprocess.run(prefix + command, check=True)
    return plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config")
    parser.add_argument(
        "--list", action="store_true", help="List CAN and USB identities"
    )
    parser.add_argument(
        "--execute", action="store_true", help="Run the ip link setup commands"
    )
    args = parser.parse_args(argv)
    if args.list and args.execute:
        parser.error("--list cannot be combined with --execute")
    if not args.list and not args.config:
        parser.error("--config is required unless --list is used")
    try:
        if args.list:
            print(json.dumps(discover_interfaces(), ensure_ascii=False, indent=2))
            return 0
        config = load_config(args.config)["robot"]
        plan = configuration_plan(config)
        print(
            json.dumps(dict(plan, execute=args.execute), ensure_ascii=False, indent=2)
        )
        if args.execute:
            configure_interfaces(config, plan=plan)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("CAN preparation failed: {}".format(exc), file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == "__main__":
    sys.exit(main())
