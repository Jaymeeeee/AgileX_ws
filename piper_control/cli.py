#!/usr/bin/env python3
"""Standalone configuration-driven SDK workflows."""
import argparse
import json
import math
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from piper_control.config import load_config, describe


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check-config", "run"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--execute", action="store_true", help="allow real SDK mode/enable/motion commands")
    parser.add_argument("--mock", action="store_true", help="fake robot, cameras and policy; no ROS, CAN or model needed")
    parser.add_argument("--steps", type=int, help="stop after this many recorded steps")
    parser.add_argument("--seconds", type=float, help="maximum session duration (excluding startup)")
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--output", help="recording parent directory")
    parser.add_argument("--schedule", help="mock-only JSON array [[seconds,command], ...]")
    args = parser.parse_args(argv)
    if args.mock and args.execute:
        parser.error("--mock and --execute are mutually exclusive")
    if args.command == "run" and not (args.mock or args.execute):
        parser.error("select --mock for offline validation or --execute for real hardware")
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be positive")
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    if args.schedule and not args.mock:
        parser.error("--schedule is for offline mock verification only")
    overrides = {}
    if args.output:
        overrides["record"] = {"output_dir": str(Path(args.output).expanduser().resolve())}
    if args.auto_start:
        overrides["workflow"] = {"auto_start": True}
    if args.mock:
        overrides["policy"] = {"backend": "mock", "python": sys.executable, "device": "cpu"}
    try:
        config = load_config(args.config, overrides)
        if args.mock and config["record"]["backend"] == "ros":
            config["record"]["backend"] = "files"
        elif args.mock and config["record"]["backend"] == "ros_publish":
            config["record"]["backend"] = "none"
        if args.mock and config["workflow"]["mode"] == "online_rl":
            parser.error("Mock hardware smoke uses deploy/collect; learner tests use synthetic RLT fixtures")
        if args.command == "check-config":
            print(json.dumps(describe(config), indent=2, ensure_ascii=False))
            return 0
        if not args.mock and config["policy"]["backend"] == "mock":
            parser.error("Mock policy cannot be used with real hardware")
        schedule = json.loads(args.schedule) if args.schedule else []
        if not isinstance(schedule, list) or any(
                not isinstance(row, list) or len(row) != 2 or isinstance(row[0], bool)
                or not isinstance(row[0], (int, float)) or not math.isfinite(row[0])
                or row[0] < 0 or not isinstance(row[1], str) for row in schedule):
            parser.error("Schedule must contain [nonnegative_seconds, command] pairs")
        from piper_control.workflows.session import Session
        result = Session(config, mock=args.mock, command_schedule=schedule).run(
            max_steps=args.steps, max_seconds=args.seconds)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("piper_control: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
