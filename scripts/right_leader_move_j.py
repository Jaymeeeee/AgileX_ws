#!/usr/bin/env python3
"""Right leader only: enable, send one J5 relative target via move_j, observe."""

import argparse
import copy
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from piper_control.config import load_config
from piper_control.hardware.interfaces import discover_interfaces


FAULT_FIELDS = (
    "voltage_too_low", "motor_overheating", "driver_overcurrent",
    "driver_overheating", "collision_status", "driver_error_status", "stall_status",
)


def read_feedback(sdk):
    joints = copy.deepcopy(sdk.get_joint_angles())
    drivers = [copy.deepcopy(sdk.get_driver_states(i)) for i in range(1, 7)]
    arm = copy.deepcopy(sdk.get_arm_status())
    faults = {}
    for index, driver in enumerate(drivers, 1):
        if driver is not None:
            flags = [key for key in FAULT_FIELDS if getattr(driver.msg.foc_status, key, False)]
            if flags:
                faults["joint_%d" % index] = flags
    return dict(
        joints_rad=list(joints.msg) if joints is not None else None,
        joints_stamp=joints.timestamp if joints is not None else 0,
        enabled=[bool(d.msg.foc_status.driver_enable_status) if d else None for d in drivers],
        driver_stamps=[d.timestamp if d else 0 for d in drivers],
        driver_faults=faults,
        arm_status=arm.msg.arm_status if arm is not None else None,
    )


def report(stage, state):
    print(json.dumps(dict(time=time.time(), stage=stage, **state), ensure_ascii=False), flush=True)


def fresh(state, since):
    joints = state["joints_rad"]
    return (
        joints is not None and len(joints) == 6
        and all(math.isfinite(value) for value in joints)
        and state["joints_stamp"] > since
        and all(stamp > since for stamp in state["driver_stamps"])
    )


def wait_feedback(sdk, stage, predicate, timeout=5.0, retry=None):
    deadline = time.monotonic() + timeout
    while True:
        state = read_feedback(sdk)
        if state["driver_faults"]:
            report(stage + ":fault", state)
            raise RuntimeError("Driver fault; no further enable/reset/move_j commands")
        if predicate(state):
            report(stage, state)
            return state
        if time.monotonic() >= deadline:
            report(stage + ":timeout", state)
            raise RuntimeError("Timed out waiting for " + stage)
        if retry is not None:
            retry()
        time.sleep(0.02)


def run_test(sdk, delta_deg=2.0, speed=10, reset=False):
    """No followers, gripper commands, target repeats or automatic retries of motion."""
    touched = False
    try:
        connected_at = time.time()
        sdk.connect()
        state = wait_feedback(sdk, "before_enable", lambda s: fresh(s, connected_at))
        if state["arm_status"] == 1 and not reset:
            raise RuntimeError("Electronic stop is active. --reset requests one reset before enabling.")
        if reset:
            # Only reached after reading fault-free feedback; never clear a driver fault.
            touched = True
            sent_at = time.time()
            sdk.reset()
            wait_feedback(sdk, "reset_confirmed", lambda s: fresh(s, sent_at) and not any(s["enabled"]))

        touched = True
        sdk.set_follower_mode()  # The physical right leader must accept SDK position targets.
        time.sleep(0.1)
        sent_at = time.time()
        report("enable_call", {})
        sdk.enable()
        wait_feedback(
            sdk, "enabled", lambda s: fresh(s, sent_at) and all(s["enabled"]),
            timeout=10.0, retry=sdk.enable,
        )
        speed_sent_at = time.time()
        sdk.set_speed_percent(speed)
        state = wait_feedback(sdk, "before_move_j", lambda s: fresh(s, speed_sent_at))
        if not all(state["enabled"]):
            raise RuntimeError("Motor enable lost before move_j")
        target = list(state["joints_rad"])
        target[4] += math.radians(delta_deg)
        report("move_j_call", dict(start_joints_rad=state["joints_rad"], target_joints_rad=target, speed_percent=speed))
        sdk.move_j(target)  # Exactly one motion target; the other five targets stay unchanged.

        deadline, next_report = time.monotonic() + 3.0, 0.0
        while time.monotonic() < deadline:
            state = read_feedback(sdk)
            if state["driver_faults"] or not all(state["enabled"]) or not fresh(state, time.time() - 0.5):
                report("after_move_j:fault", state)
                raise RuntimeError("Fault, enable loss or missing fresh feedback after move_j")
            if time.monotonic() >= next_report:
                report("after_move_j", state)
                next_report = time.monotonic() + 0.5
            time.sleep(0.02)
        report("observation_finished", dict(joint_5_error_rad=target[4] - state["joints_rad"][4]))
    finally:
        try:
            if touched:
                sdk.electronic_emergency_stop()
        finally:
            sdk.disconnect()  # Normal exit retains enable state; no disable or return motion.


def right_leader_channel(spec):
    for selector in ("usb_serial", "usb_path"):
        if spec.get(selector):
            matches = [
                item["channel"] for item in discover_interfaces()
                if item.get(selector) == spec[selector]
                or (selector == "usb_path" and item.get("usb_interface") == spec[selector])
            ]
            if len(matches) != 1:
                raise ValueError("right_leader must resolve to one CAN interface: %r" % matches)
            return matches[0]
    return spec["channel"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/collect_right.yaml"))
    parser.add_argument("--delta-deg", type=float, default=2.0, help="Signed J5 increment; maximum magnitude 5 degrees")
    parser.add_argument("--speed", type=int, default=10, help="SDK speed percent (1..100)")
    parser.add_argument("--reset", action="store_true", help="Request one motion reset after verifying no driver fault")
    parser.add_argument("--execute", action="store_true", help="Connect and execute; otherwise preview only")
    args = parser.parse_args(argv)
    if not math.isfinite(args.delta_deg) or not 0 < abs(args.delta_deg) <= 5:
        parser.error("--delta-deg must be nonzero and within -5..5 degrees")
    if not 1 <= args.speed <= 100:
        parser.error("--speed must be within 1..100")
    handlers = {}
    try:
        robot_cfg = load_config(args.config)["robot"]
        spec = robot_cfg["pairs"]["right"]["leader"]
        if not isinstance(spec, dict):
            raise ValueError("Enable right.leader in the selected configuration")
        report("plan", dict(device="right/leader", adapter=spec, delta_deg=args.delta_deg, speed_percent=args.speed, reset=args.reset, execute=args.execute))
        if not args.execute:
            return 0
        channel = right_leader_channel(spec)
        from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile

        sdk = AgxArmFactory.create_arm(create_agx_arm_config(
            robot="piper",
            firmeware_version=resolve_firmware_profile("piper", robot_cfg["firmware"]),
            interface="socketcan", channel=channel, bitrate=robot_cfg.get("bitrate", 1000000),
        ))

        def interrupt(signum, frame):
            raise KeyboardInterrupt("Test interrupted")

        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt)
        report("selected_device", dict(device="right/leader", channel=channel))
        run_test(sdk, args.delta_deg, args.speed, args.reset)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("right_leader_move_j: %s" % exc, file=sys.stderr, flush=True)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
