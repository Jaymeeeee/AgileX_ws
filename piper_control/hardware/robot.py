"""Direct public pyAgxArm commands and observations; no ROS dependency.

Commands are submitted in radians/metres without framework clipping, slew
limits or wire-grid rounding. The SDK retains its normal validation. Returning
a command records what was submitted, not an acknowledgement of execution.
"""

import copy
import time

import numpy as np

from .can import CANForwarder
from .interfaces import configured_devices, resolve_robot_config


class RobotError(RuntimeError):
    pass


def _action(action, arms):
    value = np.asarray(action, dtype=float)
    if value.shape != (7 * len(arms),):
        raise ValueError("Expected seven action values per selected arm")
    return value.copy()


class SDKRobot:
    def __init__(self, config, sdk_factory=None, bus_factory=None):
        self.config = copy.deepcopy(config)
        self.arms = list(config["arms"])
        self.sdk_factory, self.bus_factory = sdk_factory, bus_factory
        self.devices, self.forwarders = {}, {}
        self.connected = self.enabled = self.touched = False
        self.mode = "DISCONNECTED"
        self.fault = None

    @property
    def supports_human(self):
        return all(
            self.config["pairs"][side].get("leader") is not None for side in self.arms
        )

    def _make_sdk(self, spec):
        if self.sdk_factory is not None:
            return self.sdk_factory(spec, self.config)
        from pyAgxArm import (
            AgxArmFactory,
            create_agx_arm_config,
            resolve_firmware_profile,
        )

        sdk = AgxArmFactory.create_arm(
            create_agx_arm_config(
                robot="piper",
                firmeware_version=resolve_firmware_profile(
                    "piper", self.config["firmware"]
                ),
                interface="socketcan",
                channel=spec["channel"],
                bitrate=self.config.get("bitrate", 1000000),
            )
        )
        return sdk, sdk.init_effector(sdk.OPTIONS.EFFECTOR.AGX_GRIPPER)

    def connect(self):
        if self.connected:
            return
        try:
            self.config = resolve_robot_config(self.config)
            for side, role, spec in configured_devices(self.config):
                sdk, gripper = self._make_sdk(spec)
                self.devices[(side, role)] = dict(
                    sdk=sdk,
                    gripper=gripper,
                    side=side,
                    role=role,
                    mode="follower",
                )
                sdk.connect()
            for side in self.arms:
                pair = self.config["pairs"][side]
                if pair.get("leader") is None:
                    continue
                forwarder = CANForwarder(
                    pair["leader"]["channel"],
                    pair["follower"]["channel"],
                    bus_factory=self.bus_factory,
                )
                self.forwarders[side] = forwarder
                forwarder.start()
            self.connected, self.mode = True, "OBSERVE"
        except Exception:
            self.close()
            raise

    def _set_mode(self, devices, mode):
        for device in devices:
            getattr(device["sdk"], "set_" + mode + "_mode")()
            device["mode"] = mode
        time.sleep(float(self.config.get("mode_settle_sec", 0.1)))

    @staticmethod
    def _driver_enable_feedback(sdk, sent_at):
        """Read six enable bits and identify missing/pre-command feedback."""
        flags, missing, stale = [], [], []
        for index in range(1, 7):
            state = copy.deepcopy(sdk.get_driver_states(index))
            if state is None:
                flags.append(False)
                missing.append(index)
                continue
            flags.append(bool(state.msg.foc_status.driver_enable_status))
            if not state.timestamp > sent_at:
                stale.append(index)
        return flags, missing, stale

    def enable(self):
        if not self.connected:
            raise RobotError("Connect before enabling")

        def check_stop():
            if self.mode == "STOPPED":
                raise RobotError("Stop requested while enabling")

        def check_driver_faults():
            for key, device in self.devices.items():
                faults = self._driver_faults(device["sdk"])
                if faults:
                    raise RobotError(
                        "Driver fault during startup enable: %s driver_faults=%s"
                        % ("/".join(key), faults)
                    )

        check_stop()
        check_driver_faults()
        self.touched = True
        self.enabled = False
        self._set_mode(self.devices.values(), "follower")
        check_stop()
        timeout = float(self.config.get("startup_enable_timeout_sec", 10.0))
        deadline = time.monotonic() + timeout
        sent_at = {}
        for key, device in self.devices.items():
            check_stop()
            sent_at[key] = time.time()
            device["sdk"].enable()
            check_stop()
            device["sdk"].set_speed_percent(self.config.get("move_speed_percent", 100))
        while True:
            check_stop()
            check_driver_faults()
            pending = []
            for key, device in self.devices.items():
                flags, missing, stale = self._driver_enable_feedback(
                    device["sdk"], sent_at[key]
                )
                if missing or stale or not all(flags):
                    pending.append((key, device, flags, missing, stale))
            check_stop()
            if not pending:
                self.enabled, self.mode = True, "HOLD"
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                details = []
                for key, device, flags, missing, stale in pending:
                    sdk = device["sdk"]
                    detail = (
                        "%s enabled=%s; missing_driver_feedback=%s; "
                        "stale_driver_feedback=%s"
                    ) % (
                        "/".join(key), flags, missing, stale
                    )
                    status = sdk.get_arm_status()
                    if status is not None:
                        detail += "; arm_status=%s; err_code=%s" % (
                            status.msg.arm_status, status.msg.err_code
                        )
                    details.append(detail)
                raise RobotError(
                    "Startup enable timed out after %.1fs: %s"
                    % (timeout, "; ".join(details))
                )
            # enable() sends once and returns cached feedback. Repeat only for
            # arms still awaiting six enabled motors, as in the SDK example.
            for _, device, _, _, _ in pending:
                check_stop()
                device["sdk"].enable()
                check_stop()
            time.sleep(min(0.02, remaining))

    @staticmethod
    def _driver_faults(sdk):
        fault_fields = (
            "voltage_too_low", "motor_overheating", "driver_overcurrent",
            "driver_overheating", "collision_status", "driver_error_status",
            "stall_status",
        )
        faults = {}
        for index in range(1, 7):
            driver = copy.deepcopy(sdk.get_driver_states(index))
            if driver is None:
                continue
            flags = [
                field for field in fault_fields
                if getattr(driver.msg.foc_status, field, False)
            ]
            if flags:
                faults["joint_%d" % index] = flags
        return faults

    def prepare_human_start(self, stop_requested=None):
        """Restore feedback mode, then recover electronic stops without clearing faults."""
        if not self.connected:
            raise RobotError("Connect before preparing human control")

        def check_stop():
            if (stop_requested is not None and stop_requested()) or self.mode == "STOPPED":
                raise RobotError("Stop requested before startup recovery")

        check_stop()
        faults = []
        for key, device in self.devices.items():
            driver_faults = self._driver_faults(device["sdk"])
            if driver_faults:
                faults.append("%s driver_faults=%s" % ("/".join(key), driver_faults))
        if faults:
            raise RobotError(
                "Driver fault before startup recovery; no reset/enable/motion sent: "
                + "; ".join(faults)
            )
        # Ctrl+C or a previous process can leave the physical leader in leader
        # mode, which does not emit ordinary arm/driver feedback. The local mode
        # assigned in connect() does not establish the hardware mode. Switch
        # before waiting for feedback; this command does not enable or reset.
        leaders = [device for (_, role), device in self.devices.items() if role == "leader"]
        if leaders:
            check_stop()
            self.touched = True
            self._set_mode(leaders, "follower")
        check_stop()
        timeout = float(self.config.get("startup_enable_timeout_sec", 10.0))
        deadline = time.monotonic() + timeout
        while True:
            check_stop()
            pending, stopped, faults = [], {}, []
            for key, device in self.devices.items():
                sdk = device["sdk"]
                label = "/".join(key)
                driver_faults = self._driver_faults(sdk)
                if driver_faults:
                    faults.append("%s driver_faults=%s" % (label, driver_faults))
                status = copy.deepcopy(sdk.get_arm_status())
                drivers = [copy.deepcopy(sdk.get_driver_states(i)) for i in range(1, 7)]
                missing_drivers = [
                    i for i, state in enumerate(drivers, 1)
                    if state is None or state.timestamp <= 0
                ]
                missing_status = status is None or status.timestamp <= 0
                if missing_status or missing_drivers:
                    pending.append(
                        "%s missing_arm_status=%s; missing_driver_feedback=%s"
                        % (label, missing_status, missing_drivers)
                    )
                elif status.msg.arm_status == 1:
                    stopped[key] = device
            check_stop()
            if faults:
                raise RobotError(
                    "Driver fault before startup recovery; no reset/enable/motion sent: "
                    + "; ".join(faults)
                )
            if not pending:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RobotError(
                    "Startup recovery feedback timed out after %.1fs: %s"
                    % (timeout, "; ".join(pending))
                )
            time.sleep(min(0.02, remaining))
        if not stopped:
            return

        deadline = time.monotonic() + timeout
        sent_at = {}
        for key, device in stopped.items():
            check_stop()
            self.touched = True
            sent_at[key] = time.time()
            device["sdk"].reset()
        time.sleep(float(self.config.get("mode_settle_sec", 0.1)))
        # reset() is asynchronous and drops motor power. Do not let the next
        # enable stage accept cached bits from before that reset took effect.
        while True:
            check_stop()
            pending = []
            for key, device in stopped.items():
                flags, missing, stale = self._driver_enable_feedback(
                    device["sdk"], sent_at[key]
                )
                if missing or stale or any(flags):
                    pending.append(
                        "%s enabled=%s; missing_driver_feedback=%s; "
                        "stale_driver_feedback=%s"
                        % ("/".join(key), flags, missing, stale)
                    )
            check_stop()
            if not pending:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RobotError(
                    "Startup reset timed out after %.1fs: %s"
                    % (timeout, "; ".join(pending))
                )
            time.sleep(min(0.02, remaining))

    def wait_for_alignment(self, target, stop_requested):
        """Wait for the initial JS motion to settle before enabling human drag.

        The target is the follower pose already submitted by Controller.start.
        Only observe feedback here: Controller.start already sent one JS target,
        just as it does during deployment. Do not reissue or change motion modes.
        JS may settle with position residuals. Report pair/target errors without
        requiring precision alignment. Use new feedback over a stable window;
        repeated cached samples must not count as completed motion.
        """
        target = _action(target, self.arms)
        settle_delta = np.tile([0.002] * 6 + [0.001], len(self.arms) * 2)
        settle_since = None
        window_min = window_max = None
        last_stamp = None
        timeout = float(self.config.get("startup_alignment_timeout_sec", 10.0))
        deadline = time.monotonic() + timeout

        def check_stop():
            if stop_requested() or self.mode == "STOPPED":
                raise RobotError("Stop requested during startup alignment")

        def describe_error(errors):
            return dict(
                max_joint_error_rad=round(float(np.max(errors[:6])), 6),
                max_joint_error_joint=int(np.argmax(errors[:6])) + 1,
                gripper_error_m=round(float(errors[6]), 6),
            )

        while True:
            check_stop()
            snapshot = self.snapshot()
            pending = []
            disabled = []
            pair_errors = {}
            target_errors = {}
            samples = []
            for index, side in enumerate(self.arms):
                part = slice(index * 7, (index + 1) * 7)
                for role in ("follower", "leader"):
                    device = self.devices.get((side, role))
                    label = side + "/" + role
                    if device is None:
                        pending.append(label + " unavailable")
                        continue
                    flags = device["sdk"].get_joints_enable_status_list()
                    if flags is None or len(flags) != 6 or not all(flags):
                        disabled.append(
                            self._alignment_detail(side, role, device, target[part])
                        )
                    elif self._driver_faults(device["sdk"]):
                        raise RobotError(
                            "Driver fault during startup alignment: "
                            + self._alignment_detail(side, role, device, target[part])
                        )
                pair = snapshot["arms"][side]
                follower, leader = pair.get("follower"), pair.get("leader")
                if not all(
                    sample is not None and sample["valid"]
                    and sample["source"] == "feedback"
                    and np.all(np.isfinite(sample["position"]))
                    for sample in (follower, leader)
                ):
                    pending.append(side + ": pair feedback unavailable")
                    pair_errors[side] = None
                    continue
                errors = np.abs(leader["position"] - follower["position"])
                pair_errors[side] = describe_error(errors)
                target_errors[side] = {
                    role: describe_error(np.abs(sample["position"] - target[part]))
                    for role, sample in (("follower", follower), ("leader", leader))
                }
                samples.extend((follower, leader))
            check_stop()
            if disabled:
                raise RobotError(
                    "Motor enable lost during startup alignment: " + "; ".join(disabled)
                )
            if not pending:
                stamp = min(sample["stamp"] for sample in samples)
                if not np.isfinite(stamp) or stamp <= 0:
                    pending.append("position feedback has no valid timestamp")
                elif last_stamp is None or stamp > last_stamp:
                    positions = np.concatenate([sample["position"] for sample in samples])
                    if settle_since is None:
                        settle_since = stamp
                        window_min, window_max = positions.copy(), positions.copy()
                    else:
                        window_min = np.minimum(window_min, positions)
                        window_max = np.maximum(window_max, positions)
                        if np.any(window_max - window_min > settle_delta):
                            settle_since = stamp
                            window_min, window_max = positions.copy(), positions.copy()
                    last_stamp = stamp
                    if stamp - settle_since >= 0.8:
                        return dict(
                            result="settled", settle_time_sec=round(stamp - settle_since, 3),
                            reference="current_follower", pair_errors=pair_errors,
                            target_errors=target_errors,
                        )
                    pending.append("JS motion not settled over 0.8s of new feedback")
                else:
                    pending.append("waiting for new position feedback")
            else:
                settle_since = None
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                details = []
                for index, side in enumerate(self.arms):
                    part = target[index * 7 : (index + 1) * 7]
                    for role in ("follower", "leader"):
                        device = self.devices.get((side, role))
                        if device is not None:
                            details.append(self._alignment_detail(side, role, device, part))
                raise RobotError(
                    "Startup motion settle timed out after %.1fs: %s; "
                    "reference=current_follower, pair_errors=%s; feedback: %s"
                    % (timeout, "; ".join(pending), pair_errors, "; ".join(details))
                )
            time.sleep(min(0.02, remaining))

    @staticmethod
    def _alignment_detail(side, role, device, target):
        sdk, gripper = device["sdk"], device["gripper"]
        joints = copy.deepcopy(sdk.get_joint_angles())
        grip = copy.deepcopy(gripper.get_gripper_status())
        drivers = [copy.deepcopy(sdk.get_driver_states(i)) for i in range(1, 7)]
        fault_fields = (
            "voltage_too_low", "motor_overheating", "driver_overcurrent",
            "driver_overheating", "collision_status", "driver_error_status",
            "stall_status",
        )
        driver_faults = {}
        for index, driver in enumerate(drivers, 1):
            if driver is None:
                continue
            flags = [
                field for field in fault_fields
                if getattr(driver.msg.foc_status, field, False)
            ]
            if flags:
                driver_faults["joint_%d" % index] = flags
        current = np.asarray(list(joints.msg), dtype=float) if joints is not None else None
        errors = np.abs(current - target[:6]) if current is not None else None
        missing = []
        if joints is None:
            missing.append("joint_angles")
        if grip is None:
            missing.append("gripper_status")
        return (
            "%s/%s target_joints_rad=%s, current_joints_rad=%s, "
            "max_joint_error_rad=%s, max_joint_error_joint=%s, "
            "gripper_target_m=%.6f, gripper_current_m=%s, gripper_error_m=%s, "
            "enabled=%s, missing_feedback=%s, missing_driver_feedback=%s, "
            "driver_faults=%s"
            % (
                side, role, np.round(target[:6], 6).tolist(),
                np.round(current, 6).tolist() if current is not None else None,
                round(float(np.max(errors)), 6) if errors is not None else None,
                int(np.argmax(errors)) + 1 if errors is not None else None,
                target[6], round(float(grip.msg.value), 6) if grip is not None else None,
                round(abs(float(grip.msg.value) - target[6]), 6) if grip is not None else None,
                sdk.get_joints_enable_status_list(), missing,
                [i for i, driver in enumerate(drivers, 1) if driver is None],
                driver_faults,
            )
        )

    def command(self, action):
        """Use the same SDK JS mode for startup, model and hold targets."""
        if not self.enabled or self.mode not in ("HOLD", "MODEL"):
            raise RobotError("Model commands require enabled model/hold mode")
        submitted = _action(action, self.arms)
        for index, side in enumerate(self.arms):
            part = submitted[index * 7 : (index + 1) * 7]
            for role in ("follower", "leader"):
                if (side, role) not in self.devices:
                    continue
                device = self.devices[(side, role)]
                device["sdk"].move_js(part[:6].tolist())
                device["gripper"].move_gripper_m(
                    value=float(part[6]),
                    force=self.config.get("gripper_force", 1.0),
                )
        self.mode = "MODEL"
        return submitted

    def enter_human(self):
        if not self.supports_human:
            raise RobotError("Human control requires a leader for every selected arm")
        if not self.enabled:
            raise RobotError("Enable before human control")
        self.mode = "TRANSITION"
        self._set_mode([self.devices[(side, "leader")] for side in self.arms], "leader")
        for forwarder in self.forwarders.values():
            forwarder.activate()
        self.mode = "HUMAN"

    def exit_human(self):
        self.mode = "TRANSITION"
        for forwarder in self.forwarders.values():
            forwarder.deactivate()
        self._set_mode(
            [device for (_, role), device in self.devices.items() if role == "leader"],
            "follower",
        )
        self.mode = "HOLD"
        return self.snapshot()

    @staticmethod
    def _detail(sample):
        if sample is None:
            return dict(stamp=None, valid=False)
        sample = copy.deepcopy(sample)
        value = (
            sample.msg.to_dict()
            if hasattr(sample.msg, "to_dict")
            else vars(sample.msg).copy()
        )
        return dict(value, stamp=float(sample.timestamp), valid=True)

    def _sample(self, device):
        """Read the SDK's latest aggregate; ``valid`` means available, not fresh.

        SDK joint timestamps describe its cached aggregate, not a synchronized
        six-joint read. No parser internals or independent RX monitor are used.
        A missing sample has None position/timestamps, never fabricated zeros.
        """
        sdk, grip = device["sdk"], device["gripper"]
        human = device["mode"] == "leader"
        joints = copy.deepcopy(
            sdk.get_leader_joint_angles() if human else sdk.get_joint_angles()
        )
        gripper = copy.deepcopy(
            grip.get_gripper_ctrl_states() if human else grip.get_gripper_status()
        )
        available = joints is not None and gripper is not None
        stamp = (
            min(float(joints.timestamp), float(gripper.timestamp))
            if available
            else None
        )
        result = dict(
            position=(
                np.asarray(list(joints.msg) + [gripper.msg.value], dtype=float)
                if available
                else None
            ),
            stamp=stamp,
            monotonic=time.monotonic() - (time.time() - stamp) if available else None,
            valid=available,
            source="command" if human else "feedback",
            joint_timestamp=float(joints.timestamp) if joints is not None else None,
            gripper_timestamp=float(gripper.timestamp) if gripper is not None else None,
        )
        result["motor_feedback"] = [
            self._detail(sdk.get_motor_states(i)) for i in range(1, 7)
        ]
        result["driver_feedback"] = [
            self._detail(sdk.get_driver_states(i)) for i in range(1, 7)
        ]
        result["gripper_feedback"] = self._detail(gripper)
        result["arm_status"] = self._detail(sdk.get_arm_status())
        return result

    def snapshot(self):
        pairs = {
            side: {
                role: self._sample(self.devices[(side, role)])
                for role in ("follower", "leader")
                if (side, role) in self.devices
            }
            for side in self.arms
        }
        for side in self.arms:
            if not pairs[side]["follower"]["valid"]:
                raise RobotError(
                    "SDK joint/gripper feedback unavailable: " + side + "/follower"
                )
        targets = [
            self.forwarders[side].target()
            for side in self.arms
            if side in self.forwarders
        ]
        human = (
            self.mode == "HUMAN"
            and self.supports_human
            and all(target is not None for target in targets)
        )
        leaders = all(
            "leader" in pair
            and pair["leader"]["valid"]
            and pair["leader"]["source"] == "feedback"
            for pair in pairs.values()
        )
        return dict(
            stamp=min(pair["follower"]["stamp"] for pair in pairs.values()),
            monotonic=min(pair["follower"]["monotonic"] for pair in pairs.values()),
            sampled_at=time.time(),
            state=np.concatenate(
                [pairs[side]["follower"]["position"] for side in self.arms]
            ),
            leaders=(
                np.concatenate(
                    [pairs[side]["leader"]["position"] for side in self.arms]
                )
                if leaders
                else None
            ),
            arms=pairs,
            human_action=(
                np.concatenate([target["position"] for target in targets])
                if human
                else None
            ),
            human_action_seq=(
                tuple(target["sequence"] for target in targets) if human else None
            ),
            human_action_stamp=(
                min(target["stamp"] for target in targets) if human else None
            ),
            mode=self.mode,
            fault=self.fault,
        )

    def human_action(self):
        return self.snapshot()["human_action"]

    def stop(self):
        if self.mode == "STOPPED":
            return
        self.enabled, self.mode = False, "STOPPED"
        errors = []
        for action in [
            forwarder.deactivate for forwarder in self.forwarders.values()
        ] + (
            [
                device["sdk"].electronic_emergency_stop
                for device in self.devices.values()
            ]
            if self.touched
            else []
        ):
            try:
                action()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RobotError("Robot stop failed: " + "; ".join(errors))

    request_stop = stop

    def close(self):
        errors = []
        actions = [self.stop] if self.touched else []
        actions += [forwarder.close for forwarder in self.forwarders.values()]
        actions += [device["sdk"].disconnect for device in self.devices.values()]
        for action in actions:
            try:
                action()
            except Exception as exc:
                errors.append(str(exc))
        self.devices.clear()
        self.forwarders.clear()
        self.connected = self.enabled = False
        if errors:
            raise RobotError("Robot close failed: " + "; ".join(errors))


class MockRobot:
    """Instantaneous in-memory plant with the same direct command interface."""

    def __init__(self, config):
        self.config, self.arms = copy.deepcopy(config), list(config["arms"])
        self.state = np.zeros(7 * len(self.arms))
        self.connected = self.enabled = False
        self.mode, self.fault, self.sequence = "DISCONNECTED", None, 0

    @property
    def supports_human(self):
        return all(
            self.config["pairs"][side].get("leader") is not None for side in self.arms
        )

    def connect(self):
        self.connected, self.mode = True, "OBSERVE"

    def enable(self):
        if not self.connected:
            raise RobotError("Connect before enabling")
        self.enabled, self.mode = True, "HOLD"

    def prepare_human_start(self, stop_requested=None):
        if (stop_requested is not None and stop_requested()) or self.mode == "STOPPED":
            raise RobotError("Stop requested before startup recovery")

    def wait_for_alignment(self, target, stop_requested):
        # The mock plant reaches command() targets immediately.
        if stop_requested() or self.mode == "STOPPED":
            raise RobotError("Stop requested during startup alignment")

    def snapshot(self):
        stamp, mono = time.time(), time.monotonic()
        human = self.mode == "HUMAN"
        pairs = {}
        for index, side in enumerate(self.arms):
            sample = dict(
                position=self.state[index * 7 : (index + 1) * 7].copy(),
                stamp=stamp,
                monotonic=mono,
                valid=True,
                source="feedback",
            )
            pairs[side] = dict(follower=sample)
            if self.config["pairs"][side].get("leader") is not None:
                pairs[side]["leader"] = dict(
                    sample, source="command" if human else "feedback"
                )
        return dict(
            stamp=stamp,
            monotonic=mono,
            state=self.state.copy(),
            leaders=self.state.copy() if self.supports_human and not human else None,
            arms=pairs,
            human_action=self.state.copy() if human else None,
            human_action_seq=(self.sequence,) * len(self.arms) if human else None,
            human_action_stamp=stamp if human else None,
            mode=self.mode,
            fault=self.fault,
        )

    def command(self, action):
        if not self.enabled or self.mode not in ("HOLD", "MODEL"):
            raise RobotError("Model commands require enabled model/hold mode")
        self.state = _action(action, self.arms)
        self.mode = "MODEL"
        return self.state.copy()

    def enter_human(self):
        if not self.supports_human:
            raise RobotError("Human control requires a leader for every selected arm")
        if not self.enabled:
            raise RobotError("Enable before human control")
        self.mode = "HUMAN"
        self.sequence += 1

    def set_human_action(self, action):
        self.state = _action(action, self.arms)
        self.sequence += 1

    def human_action(self):
        return self.snapshot()["human_action"]

    def exit_human(self):
        self.mode = "HOLD"
        return self.snapshot()

    def stop(self):
        self.enabled, self.mode = False, "STOPPED"

    request_stop = stop

    def close(self):
        self.stop()
        self.connected = False
