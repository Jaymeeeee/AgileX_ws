import threading
import time
import unittest
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from piper_control.hardware.robot import MockRobot, RobotError, SDKRobot
from piper_control.hardware.cameras import CameraRig
from piper_control.control.arbiter import Controller


def config(arms=None):
    return dict(
        arms=arms or ["left"],
        firmware="S-V1.9-0",
        mode_settle_sec=0.0,
        move_speed_percent=73,
        gripper_force=1.2,
        pairs=dict(
            left=dict(leader=dict(channel="can30"), follower=dict(channel="can31")),
            right=dict(leader=dict(channel="can32"), follower=dict(channel="can33")),
        ),
    )


class FakeSDK:
    """Only the public SDK surface is implemented; no parser or transport hooks."""

    def __init__(self, name, calls):
        self.name, self.calls = name, calls
        stamp = time.time()
        self.joints = SimpleNamespace(msg=[0.1] * 6, timestamp=stamp)
        self.gripper = SimpleNamespace(msg=SimpleNamespace(value=0.02), timestamp=stamp)
        self.enable_flags = [True] * 6
        self.arm_status = 0
        self.drivers = [
            SimpleNamespace(
                timestamp=stamp,
                msg=SimpleNamespace(
                    foc_status=SimpleNamespace(driver_enable_status=True)
                ),
            )
            for _ in range(6)
        ]
        self.sent = []

    def receive_enable_feedback(self, flags=None, stamp=None):
        if flags is not None:
            self.enable_flags = list(flags)
        for index, driver in enumerate(self.drivers):
            driver.timestamp = time.time() if stamp is None else stamp
            driver.msg.foc_status.driver_enable_status = self.enable_flags[index]

    def connect(self):
        self.calls.append((self.name, "connect"))

    def disconnect(self):
        self.calls.append((self.name, "disconnect"))

    def enable(self):
        self.calls.append((self.name, "enable"))
        self.receive_enable_feedback([True] * 6)

    def reset(self):
        self.calls.append((self.name, "reset"))
        self.receive_enable_feedback([False] * 6)

    def get_joints_enable_status_list(self):
        return self.enable_flags

    def set_follower_mode(self):
        self.calls.append((self.name, "follower"))

    def set_leader_mode(self):
        self.calls.append((self.name, "leader"))

    def electronic_emergency_stop(self):
        self.calls.append((self.name, "stop"))

    def set_speed_percent(self, value):
        self.calls.append((self.name, "speed", value))

    def get_joint_angles(self):
        return self.joints

    def get_leader_joint_angles(self):
        return self.joints

    def get_gripper_status(self):
        return self.gripper

    def get_gripper_ctrl_states(self):
        return self.gripper

    def get_motor_states(self, index):
        return None

    def get_driver_states(self, index):
        driver = self.drivers[index - 1]
        driver.msg.foc_status.driver_enable_status = self.enable_flags[index - 1]
        return driver

    def get_arm_status(self):
        return SimpleNamespace(
            timestamp=time.time(), msg=SimpleNamespace(arm_status=self.arm_status, err_code=0)
        )

    def move_js(self, value):
        self.sent.append(value)
        self.calls.append((self.name, "joints"))

    def move_j(self, value):
        self.sent.append(value)
        self.calls.append((self.name, "alignment_joints"))

    def move_gripper_m(self, value, force):
        self.sent.append((value, force))
        self.calls.append((self.name, "gripper"))


class FakeForwarder:
    def __init__(self, *args, **kwargs):
        self.active, self.error, self.value = False, None, None

    def start(self):
        pass

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def close(self):
        self.active = False

    def target(self):
        return self.value if self.active else None


def fake_robot(arms=None, robot_config=None):
    calls, made = [], []

    def factory(spec, cfg):
        sdk = FakeSDK(spec["channel"], calls)
        made.append(sdk)
        return sdk, sdk

    robot = SDKRobot(robot_config or config(arms), sdk_factory=factory)
    with patch("piper_control.hardware.robot.CANForwarder", FakeForwarder):
        robot.connect()
    return robot, calls, made


@contextmanager
def advancing_position_feedback(made):
    clock = [100.0]

    def advance(seconds):
        clock[0] += seconds
        for sdk in made:
            if sdk.joints is not None:
                sdk.joints.timestamp = clock[0]
            if sdk.gripper is not None:
                sdk.gripper.timestamp = clock[0]

    advance(0)
    with patch("piper_control.hardware.robot.time.monotonic", side_effect=lambda: clock[0]), patch(
        "piper_control.hardware.robot.time.sleep", side_effect=advance
    ):
        yield


class SDKTests(unittest.TestCase):
    def test_collection_and_deployment_start_use_identical_motion_commands(self):
        traces = []
        for manual in (False, True):
            robot, calls, made = fake_robot(["right"])
            controller = Controller({"robot": robot.config}, robot)
            try:
                if manual:
                    robot.prepare_human_start()
                controller.start()
                if manual:
                    with advancing_position_feedback(made):
                        robot.wait_for_alignment(controller.target, lambda: False)
                controller.enter_human()
                # Collection may restore a previously stopped leader's feedback
                # mode before preflight. The actual enable and JS motion path
                # remains shared with deployment.
                traces.append([call for call in calls if call[1] != "follower"])
                self.assertEqual(sum(call[1] == "joints" for call in calls), 2)
                self.assertFalse(any(call[1] == "alignment_joints" for call in calls))
                self.assertTrue(robot.forwarders["right"].active)
            finally:
                robot.close()
        self.assertEqual(traces[0], traces[1])

    def test_driver_fault_blocks_enable_and_enable_retries(self):
        for stage in ("before_enable", "after_enable"):
            with self.subTest(stage=stage):
                robot, calls, made = fake_robot(["right"])
                leader = made[1]

                def fault():
                    leader.drivers[4].msg.foc_status.driver_overcurrent = True

                if stage == "before_enable":
                    fault()
                else:
                    enable = leader.enable

                    def enable_then_fault():
                        enable()
                        fault()

                    leader.enable = enable_then_fault
                before = list(calls)
                try:
                    with self.assertRaisesRegex(RobotError, "Driver fault during startup enable"):
                        Controller({"robot": robot.config}, robot).start()
                    self.assertTrue(all(not sdk.sent for sdk in made))
                    if stage == "before_enable":
                        self.assertEqual(calls, before)
                    else:
                        self.assertEqual(sum(call[1] == "enable" for call in calls), 2)
                finally:
                    robot.close()

    def test_start_and_runtime_share_js_without_switching_to_j(self):
        robot, calls, made = fake_robot(["right"])
        controller = Controller({"robot": robot.config}, robot)
        try:
            controller.start()
            self.assertEqual(
                [call for call in calls if call[1] == "joints"],
                [("can33", "joints"), ("can32", "joints")],
            )
            self.assertFalse(any(call[1] == "alignment_joints" for call in calls))
            for sdk in made:
                np.testing.assert_array_equal(sdk.sent[0], controller.target[:6])
                self.assertEqual(sdk.sent[1], (controller.target[6], 1.2))
            before = len(calls)
            robot.command(controller.target)
            self.assertEqual(
                calls[before:],
                [("can33", "joints"), ("can33", "gripper"),
                 ("can32", "joints"), ("can32", "gripper")],
            )
        finally:
            robot.close()

    def test_shared_command_preserves_targets_and_requires_enabled_control(self):
        robot, calls, made = fake_robot(["left", "right"])
        target = np.arange(14, dtype=float) / 100.0
        try:
            with self.assertRaisesRegex(RobotError, "enabled model/hold"):
                robot.command(target)
            self.assertTrue(all(not sdk.sent for sdk in made))
            robot.enable()
            np.testing.assert_array_equal(robot.command(target), target)
            for index, side in enumerate(robot.arms):
                part = target[index * 7 : (index + 1) * 7]
                for role in ("follower", "leader"):
                    sdk = robot.devices[(side, role)]["sdk"]
                    np.testing.assert_array_equal(sdk.sent[0], part[:6])
                    self.assertEqual(sdk.sent[1], (part[6], 1.2))
            self.assertFalse(any(call[1] == "alignment_joints" for call in calls))
            robot.enter_human()
            before = list(calls)
            with self.assertRaisesRegex(RobotError, "enabled model/hold"):
                robot.command(target)
            self.assertEqual(calls, before)
        finally:
            robot.close()

    def test_start_retries_only_pending_arm_before_submitting_alignment_target(self):
        robot, calls, made = fake_robot(["right"])
        follower, leader = made
        follower.enable_flags = [False] * 6
        attempts = [0]

        def enable_follower():
            calls.append((follower.name, "enable"))
            attempts[0] += 1
            if attempts[0] == 3:
                follower.receive_enable_feedback([True] * 6)
            # A return value alone must not stand in for six motor feedback bits.
            return True

        follower.enable = enable_follower

        def settle(seconds):
            self.assertFalse(robot.enabled)
            self.assertTrue(all(not sdk.sent for sdk in made))

        controller = Controller({"robot": robot.config}, robot)
        try:
            with patch("piper_control.hardware.robot.time.sleep", side_effect=settle):
                controller.start()
            self.assertEqual(attempts[0], 3)
            self.assertEqual(calls.count((leader.name, "enable")), 1)
            self.assertTrue(robot.enabled)
            self.assertEqual(robot.mode, "MODEL")
            self.assertIsNotNone(controller.target)
            self.assertTrue(all(len(sdk.sent) == 2 for sdk in made))
            first_move = next(i for i, call in enumerate(calls) if call[1] == "joints")
            self.assertFalse(any(call[1] == "enable" for call in calls[first_move:]))
        finally:
            robot.close()

    def test_enable_timeout_distinguishes_disabled_and_missing_driver_feedback(self):
        cfg = config(["right"])
        cfg["startup_enable_timeout_sec"] = 0.04
        robot, calls, made = fake_robot(robot_config=cfg)
        follower, leader = made
        follower.enable_flags = [True, False, False, True, True, True]
        follower.enable = lambda: (
            calls.append((follower.name, "enable")), follower.receive_enable_feedback()
        )
        get_driver = follower.get_driver_states
        follower.get_driver_states = lambda i: None if i == 3 else get_driver(i)
        follower.get_arm_status = lambda: SimpleNamespace(
            msg=SimpleNamespace(arm_status=0, err_code=0)
        )
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        controller = Controller({"robot": robot.config}, robot)
        try:
            with patch("piper_control.hardware.robot.time.monotonic",
                       side_effect=lambda: clock[0]), patch(
                "piper_control.hardware.robot.time.sleep", side_effect=sleep
            ):
                with self.assertRaisesRegex(RobotError, "Startup enable timed out") as caught:
                    controller.start()
            message = str(caught.exception)
            self.assertIn("right/follower enabled=[True, False, False, True, True, True]", message)
            self.assertIn("missing_driver_feedback=[3]", message)
            self.assertIn("arm_status=0; err_code=0", message)
            self.assertNotIn("right/leader", message)
            self.assertEqual(calls.count((leader.name, "enable")), 1)
            self.assertGreater(calls.count((follower.name, "enable")), 1)
            self.assertFalse(robot.enabled)
            self.assertIsNone(controller.target)
            self.assertTrue(all(not sdk.sent for sdk in made))
            self.assertFalse(robot.forwarders["right"].active)
        finally:
            robot.close()

    def test_stop_interrupts_enable_retry_without_more_commands(self):
        for interruption in ("retry", "sleep"):
            with self.subTest(interruption=interruption):
                robot, calls, made = fake_robot(["right"])
                for sdk in made:
                    sdk.enable_flags = [False] * 6
                attempts = [0]

                def enable_then_stop():
                    calls.append((made[0].name, "enable"))
                    attempts[0] += 1
                    if attempts[0] == 2 and interruption == "retry":
                        robot.stop()

                def settle(seconds):
                    if seconds > 0 and interruption == "sleep":
                        robot.stop()

                made[0].enable = enable_then_stop
                try:
                    with patch("piper_control.hardware.robot.time.sleep", side_effect=settle):
                        with self.assertRaisesRegex(RobotError, "Stop requested while enabling"):
                            robot.enable()
                    first_stop = next(i for i, call in enumerate(calls) if call[1] == "stop")
                    self.assertTrue(all(call[1] == "stop" for call in calls[first_stop:]))
                    self.assertEqual(calls.count((made[1].name, "enable")),
                                     1)
                    self.assertFalse(robot.enabled)
                    self.assertEqual(robot.mode, "STOPPED")
                    self.assertTrue(all(not sdk.sent for sdk in made))
                finally:
                    robot.close()

    def test_enable_does_not_resume_commands_after_stop(self):
        for interruption in ("before_enable", "mode_settle", "first_enable", "last_speed"):
            with self.subTest(interruption=interruption):
                robot, calls, made = fake_robot(["right"])
                if interruption == "before_enable":
                    robot.stop()
                elif interruption == "first_enable":
                    def enable_then_stop():
                        calls.append((made[0].name, "enable"))
                        robot.stop()

                    made[0].enable = enable_then_stop
                elif interruption == "last_speed":
                    def speed_then_stop(value):
                        calls.append((made[-1].name, "speed", value))
                        robot.stop()

                    made[-1].set_speed_percent = speed_then_stop

                def settle(seconds):
                    if interruption == "mode_settle":
                        robot.stop()

                before = len(calls)
                try:
                    with patch("piper_control.hardware.robot.time.sleep", side_effect=settle):
                        with self.assertRaisesRegex(RobotError, "Stop requested while enabling"):
                            robot.enable()
                    after = calls[before:]
                    self.assertFalse(robot.enabled)
                    self.assertEqual(robot.mode, "STOPPED")
                    self.assertTrue(all(not sdk.sent for sdk in made))
                    if interruption in ("before_enable", "mode_settle"):
                        self.assertFalse(any(call[1] in ("enable", "speed") for call in after))
                    elif interruption == "first_enable":
                        self.assertEqual([call for call in after if call[1] == "enable"],
                                         [(made[0].name, "enable")])
                        self.assertFalse(any(call[1] == "speed" for call in after))
                    with self.assertRaisesRegex(RobotError, "enabled model/hold"):
                        robot.command([0.0] * 7)
                    self.assertTrue(all(not sdk.sent for sdk in made))
                finally:
                    robot.close()

    def test_human_start_recovery_only_resets_selected_right_devices(self):
        robot, calls, made = fake_robot(["right"])
        for sdk in made:
            sdk.arm_status = 1
        try:
            robot.prepare_human_start()
            self.assertEqual(
                calls,
                [("can33", "connect"), ("can32", "connect"),
                 ("can32", "follower"),
                 ("can33", "reset"), ("can32", "reset")],
            )
            self.assertFalse(robot.enabled)
            self.assertEqual(robot.mode, "OBSERVE")
            self.assertTrue(all(not sdk.sent for sdk in made))
        finally:
            robot.close()

    def test_reset_and_enable_wait_for_their_own_fresh_feedback_before_position(self):
        robot, calls, made = fake_robot(["right"])
        clock = [101.0]
        phase = ["reset"]
        for sdk in made:
            sdk.arm_status = 1
            sdk.receive_enable_feedback([True] * 6, stamp=100.0)
            sdk.reset = lambda sdk=sdk: calls.append((sdk.name, "reset"))
            sdk.enable = lambda sdk=sdk: calls.append((sdk.name, "enable"))

        def receive_feedback(seconds):
            self.assertTrue(all(not sdk.sent for sdk in made))
            self.assertFalse(robot.enabled)
            if seconds == 0:
                return
            clock[0] += seconds
            enabled = phase[0] == "enable"
            for sdk in made:
                sdk.receive_enable_feedback([enabled] * 6, stamp=clock[0])

        try:
            with patch("piper_control.hardware.robot.time.time", side_effect=lambda: clock[0]), patch(
                "piper_control.hardware.robot.time.monotonic", side_effect=lambda: clock[0]
            ), patch("piper_control.hardware.robot.time.sleep", side_effect=receive_feedback):
                robot.prepare_human_start()
                self.assertEqual(clock[0], 101.02)
                self.assertFalse(any(call[1] == "enable" for call in calls))
                self.assertTrue(all(sdk.enable_flags == [False] * 6 for sdk in made))
                phase[0] = "enable"
                controller = Controller({"robot": robot.config}, robot)
                controller.start()
            self.assertAlmostEqual(clock[0], 101.04)
            self.assertTrue(robot.enabled)
            self.assertTrue(all(len(sdk.sent) == 2 for sdk in made))
            first_position = next(i for i, call in enumerate(calls)
                                  if call[1] == "joints")
            self.assertTrue(all(call[1] in ("joints", "gripper")
                                for call in calls[first_position:]))
        finally:
            robot.close()

    def test_reset_does_not_accept_stale_false_or_missing_driver_feedback(self):
        for missing in (False, True):
            with self.subTest(missing=missing):
                cfg = config(["right"])
                cfg["startup_enable_timeout_sec"] = 0.04
                robot, calls, made = fake_robot(robot_config=cfg)
                clock = [101.0]
                for sdk in made:
                    sdk.arm_status = 1
                    sdk.receive_enable_feedback([False] * 6, stamp=100.0)
                    sdk.reset = lambda sdk=sdk: calls.append((sdk.name, "reset"))
                    if missing:
                        def reset_with_missing_feedback(sdk=sdk):
                            calls.append((sdk.name, "reset"))
                            sdk.get_driver_states = lambda index: None
                        sdk.reset = reset_with_missing_feedback

                def sleep(seconds):
                    clock[0] += seconds

                try:
                    with patch("piper_control.hardware.robot.time.time", side_effect=lambda: clock[0]), patch(
                        "piper_control.hardware.robot.time.monotonic", side_effect=lambda: clock[0]
                    ), patch("piper_control.hardware.robot.time.sleep", side_effect=sleep):
                        with self.assertRaisesRegex(RobotError, "Startup reset timed out") as caught:
                            robot.prepare_human_start()
                    field = "missing_driver_feedback" if missing else "stale_driver_feedback"
                    self.assertIn(field + "=[1, 2, 3, 4, 5, 6]", str(caught.exception))
                    self.assertFalse(any(call[1] == "enable" for call in calls))
                    self.assertTrue(all(not sdk.sent for sdk in made))
                finally:
                    robot.close()

    def test_enable_does_not_accept_cached_true_or_incomplete_new_feedback(self):
        for stale in (list(range(1, 7)), [6]):
            with self.subTest(stale=stale):
                cfg = config(["right"])
                cfg["startup_enable_timeout_sec"] = 0.04
                robot, calls, made = fake_robot(robot_config=cfg)
                follower = made[0]
                clock = [101.0]
                follower.receive_enable_feedback([True] * 6, stamp=100.0)

                def enable():
                    calls.append((follower.name, "enable"))
                    for index, driver in enumerate(follower.drivers, 1):
                        if index not in stale:
                            driver.timestamp = clock[0] + 0.001

                follower.enable = enable

                def sleep(seconds):
                    clock[0] += seconds

                try:
                    with patch("piper_control.hardware.robot.time.time", side_effect=lambda: clock[0]), patch(
                        "piper_control.hardware.robot.time.monotonic", side_effect=lambda: clock[0]
                    ), patch("piper_control.hardware.robot.time.sleep", side_effect=sleep):
                        with self.assertRaisesRegex(RobotError, "Startup enable timed out") as caught:
                            robot.enable()
                    self.assertIn("stale_driver_feedback=" + str(stale), str(caught.exception))
                    self.assertGreater(calls.count((follower.name, "enable")), 1)
                    self.assertFalse(robot.enabled)
                    self.assertTrue(all(not sdk.sent for sdk in made))
                finally:
                    robot.close()

    def test_stop_interrupts_wait_for_reset_feedback(self):
        robot, calls, made = fake_robot(["right"])
        stopped = threading.Event()
        for sdk in made:
            sdk.arm_status = 1
            sdk.reset = lambda sdk=sdk: calls.append((sdk.name, "reset"))

        def sleep(seconds):
            if seconds > 0:
                stopped.set()

        try:
            with patch("piper_control.hardware.robot.time.sleep", side_effect=sleep):
                with self.assertRaisesRegex(RobotError, "Stop requested"):
                    robot.prepare_human_start(stopped.is_set)
            self.assertFalse(any(call[1] == "enable" for call in calls))
            self.assertTrue(all(not sdk.sent for sdk in made))
        finally:
            robot.close()

    def test_alignment_enable_loss_stops_before_another_target(self):
        for role in ("follower", "leader"):
            with self.subTest(role=role):
                robot, calls, made = fake_robot(["right"])
                robot.enable()
                target = robot.snapshot()["state"].copy()
                sdk = robot.devices[("right", role)]["sdk"]
                sdk.receive_enable_feedback([True, True, False, True, True, True])
                before = list(calls)
                try:
                    with self.assertRaisesRegex(
                        RobotError, "Motor enable lost during startup alignment: right/" + role
                    ) as caught:
                        robot.wait_for_alignment(target, lambda: False)
                    self.assertIn("enabled=[True, True, False, True, True, True]", str(caught.exception))
                    self.assertIn("missing_driver_feedback=[]", str(caught.exception))
                    self.assertIn("driver_faults={}", str(caught.exception))
                    self.assertEqual(calls, before)
                    self.assertTrue(all(not sdk.sent for sdk in made))
                    self.assertFalse(robot.forwarders["right"].active)
                finally:
                    robot.close()

    def test_alignment_reports_joint_driver_faults_without_retrying_motion(self):
        robot, calls, made = fake_robot(["right"])
        try:
            robot.enable()
            target = robot.snapshot()["state"].copy()
            leader = robot.devices[("right", "leader")]["sdk"]
            leader.receive_enable_feedback([True, True, True, True, False, True])
            leader.joints.msg[4] += 0.13
            status = leader.drivers[4].msg.foc_status
            status.driver_overcurrent = True
            status.driver_error_status = True
            status.stall_status = True
            status.motor_overheating = False
            before = list(calls)
            with self.assertRaisesRegex(
                RobotError, "Motor enable lost during startup alignment: right/leader"
            ) as caught:
                robot.wait_for_alignment(target, lambda: False)
            self.assertIn(
                "driver_faults={'joint_5': ['driver_overcurrent', "
                "'driver_error_status', 'stall_status']}",
                str(caught.exception),
            )
            self.assertEqual(calls, before)
            self.assertTrue(all(not sdk.sent for sdk in made))
            self.assertFalse(robot.forwarders["right"].active)
        finally:
            robot.close()

    def test_alignment_driver_fault_stops_even_if_enable_bits_remain_true(self):
        robot, calls, made = fake_robot(["right"])
        try:
            robot.enable()
            target = robot.snapshot()["state"].copy()
            made[1].drivers[4].msg.foc_status.stall_status = True
            before = list(calls)
            with self.assertRaisesRegex(RobotError, "Driver fault during startup alignment"):
                robot.wait_for_alignment(target, lambda: False)
            self.assertEqual(calls, before)
            self.assertTrue(all(not sdk.sent for sdk in made))
        finally:
            robot.close()

    def test_startup_recovery_does_not_reset_more_devices_after_stop(self):
        robot, calls, made = fake_robot(["right"])
        for sdk in made:
            sdk.arm_status = 1
        stopped = threading.Event()

        def reset_then_stop():
            calls.append((made[0].name, "reset"))
            stopped.set()

        made[0].reset = reset_then_stop
        try:
            with self.assertRaisesRegex(RobotError, "Stop requested"):
                robot.prepare_human_start(stopped.is_set)
            self.assertEqual([call for call in calls if call[1] == "reset"],
                             [("can33", "reset")])
        finally:
            robot.close()

    def test_alignment_waits_for_joint_and_gripper_feedback(self):
        for role in ("follower", "leader"):
            for mismatch in ("joint", "gripper"):
                with self.subTest(role=role, mismatch=mismatch):
                    robot, calls, made = fake_robot(["right"])
                    robot.enable()
                    target = robot.snapshot()["state"].copy()
                    robot.command(target)
                    device = robot.devices[("right", role)]["sdk"]
                    if mismatch == "joint":
                        device.joints.msg[-1] += 0.04
                    else:
                        device.gripper.msg.value += 0.004
                    before = len(calls)
                    clock = [100.0]
                    for sdk in made:
                        sdk.joints.timestamp = sdk.gripper.timestamp = clock[0]

                    def receive_feedback(seconds):
                        self.assertFalse(robot.forwarders["right"].active)
                        self.assertGreater(seconds, 0.0)
                        self.assertLessEqual(seconds, 0.02)
                        clock[0] += seconds
                        device.enable_flags = [True] * 6
                        device.joints.msg = target[:6].tolist()
                        device.gripper.msg.value = target[6]
                        for sdk in made:
                            sdk.joints.timestamp = sdk.gripper.timestamp = clock[0]

                    try:
                        with patch("piper_control.hardware.robot.time.monotonic",
                                   side_effect=lambda: clock[0]), patch(
                            "piper_control.hardware.robot.time.sleep",
                            side_effect=receive_feedback,
                        ) as sleep:
                            robot.wait_for_alignment(target, lambda: False)
                        self.assertGreaterEqual(sleep.call_count, 40)
                        self.assertEqual(robot.mode, "MODEL")
                        self.assertEqual(calls[before:], [])
                        for sdk in made:
                            np.testing.assert_array_equal(sdk.sent[0], target[:6])
                            self.assertEqual(sdk.sent[1], (target[6], 1.2))
                    finally:
                        robot.close()

    def test_alignment_aborts_before_sending_another_target(self):
        for interruption in ("already_requested", "during_snapshot", "stopped_robot"):
            with self.subTest(interruption=interruption):
                robot, calls, made = fake_robot(["right"])
                robot.enable()
                target = robot.snapshot()["state"].copy()
                made[1].joints.msg = [0.3] * 6
                stopped = threading.Event()
                snapshot = robot.snapshot
                if interruption == "already_requested":
                    stopped.set()
                elif interruption == "stopped_robot":
                    robot.stop()

                def read_feedback():
                    result = snapshot()
                    if interruption == "during_snapshot":
                        stopped.set()
                    return result

                before = list(calls)
                try:
                    with patch.object(robot, "snapshot", side_effect=read_feedback):
                        with self.assertRaisesRegex(RobotError, "Stop requested"):
                            robot.wait_for_alignment(target, stopped.is_set)
                    self.assertEqual(calls, before)
                    self.assertTrue(all(not sdk.sent for sdk in made))
                finally:
                    robot.close()

    def test_settle_timeout_reports_target_errors_without_resending(self):
        cfg = config(["right"])
        cfg["startup_alignment_timeout_sec"] = 0.04
        robot, calls, made = fake_robot(robot_config=cfg)
        robot.enable()
        target = robot.snapshot()["state"].copy()
        made[1].joints.msg = [0.4] * 6
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        try:
            with patch("piper_control.hardware.robot.time.monotonic",
                       side_effect=lambda: clock[0]), patch(
                "piper_control.hardware.robot.time.sleep", side_effect=sleep
            ):
                with self.assertRaisesRegex(
                    RobotError,
                    "Startup motion settle timed out",
                ) as caught:
                    robot.wait_for_alignment(target, lambda: False)
            message = str(caught.exception)
            self.assertIn("right/follower target_joints_rad=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]", message)
            self.assertIn("current_joints_rad=[0.4, 0.4, 0.4, 0.4, 0.4, 0.4]", message)
            self.assertIn("max_joint_error_rad=0.3, max_joint_error_joint=1", message)
            self.assertIn("gripper_target_m=0.020000, gripper_current_m=0.02, gripper_error_m=0.0", message)
            self.assertIn("enabled=[True, True, True, True, True, True]", message)
            self.assertIn("missing_feedback=[]", message)
            self.assertIn("missing_driver_feedback=[]", message)
            self.assertFalse(robot.forwarders["right"].active)
            self.assertNotIn(("can32", "leader"), calls)
            self.assertFalse(any(call[1] == "joints" for call in calls))
            for sdk in made:
                self.assertEqual(sdk.sent, [])
        finally:
            robot.close()

    def test_alignment_timeout_reports_missing_feedback_without_fabricating_positions(self):
        cfg = config(["right"])
        cfg["startup_alignment_timeout_sec"] = 0.02
        robot, calls, made = fake_robot(robot_config=cfg)
        robot.enable()
        target = robot.snapshot()["state"].copy()
        made[1].joints = None
        made[1].gripper = None
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        try:
            with patch("piper_control.hardware.robot.time.monotonic",
                       side_effect=lambda: clock[0]), patch(
                "piper_control.hardware.robot.time.sleep", side_effect=sleep
            ):
                with self.assertRaisesRegex(RobotError, "pair feedback unavailable") as caught:
                    robot.wait_for_alignment(target, lambda: False)
            message = str(caught.exception)
            self.assertIn("current_joints_rad=None, max_joint_error_rad=None", message)
            self.assertIn("gripper_current_m=None, gripper_error_m=None", message)
            self.assertIn("missing_feedback=['joint_angles', 'gripper_status']", message)
            self.assertFalse(any(call[1] == "joints" for call in calls))
        finally:
            robot.close()

    def test_mock_startup_alignment_has_no_extra_motion_and_obeys_stop(self):
        robot = MockRobot(config(["right"]))
        robot.connect()
        robot.prepare_human_start()
        self.assertFalse(robot.enabled)
        robot.enable()
        target = np.arange(7) / 100.0
        robot.command(target)
        robot.wait_for_alignment(target, lambda: False)
        np.testing.assert_array_equal(robot.state, target)
        with self.assertRaisesRegex(RobotError, "Stop requested"):
            robot.wait_for_alignment(target, lambda: True)
        robot.stop()
        with self.assertRaisesRegex(RobotError, "Stop requested"):
            robot.wait_for_alignment(target, lambda: False)
        robot.close()

    def test_usb_identity_is_resolved_again_before_reconnecting_sdk(self):
        cfg = config()
        cfg["pairs"]["left"] = dict(follower=dict(usb_serial="arm-left"), leader=None)
        calls = []

        def factory(spec, config):
            sdk = FakeSDK(spec["channel"], calls)
            return sdk, sdk

        robot = SDKRobot(cfg, sdk_factory=factory)
        with patch(
            "piper_control.hardware.interfaces.discover_interfaces",
            side_effect=[
                [dict(channel="can41", usb_serial="arm-left")],
                [dict(channel="can42", usb_serial="arm-left")],
            ],
        ) as discover, patch("piper_control.hardware.robot.CANForwarder") as forwarders:
            robot.connect()
            robot.close()
            robot.connect()
            robot.close()
        forwarders.assert_not_called()
        self.assertEqual(discover.call_count, 2)
        self.assertEqual(
            calls,
            [
                ("can41", "connect"),
                ("can41", "disconnect"),
                ("can42", "connect"),
                ("can42", "disconnect"),
            ],
        )
        self.assertEqual(cfg["pairs"]["left"]["follower"], {"usb_serial": "arm-left"})

    def test_usb_identity_resolves_sdk_and_forwarder_channels_together(self):
        cfg = config()
        cfg["pairs"]["left"] = dict(
            follower=dict(usb_serial="left-follower"),
            leader=dict(usb_serial="left-leader"),
        )
        calls = []

        def factory(spec, config):
            sdk = FakeSDK(spec["channel"], calls)
            return sdk, sdk

        robot = SDKRobot(cfg, sdk_factory=factory)
        with patch(
            "piper_control.hardware.interfaces.discover_interfaces",
            return_value=[
                dict(channel="can41", usb_serial="left-follower"),
                dict(channel="can17", usb_serial="left-leader"),
            ],
        ), patch(
            "piper_control.hardware.robot.CANForwarder", side_effect=FakeForwarder
        ) as forwarders:
            robot.connect()
            self.assertEqual(calls, [("can41", "connect"), ("can17", "connect")])
            forwarders.assert_called_once_with("can17", "can41", bus_factory=None)
            robot.close()

    def test_follower_only_single_and_dual_deployment(self):
        for arms in (["left"], ["right"], ["left", "right"]):
            for omitted in (False, True):
                with self.subTest(arms=arms, omitted=omitted):
                    cfg = config(arms)
                    for side in arms:
                        if omitted:
                            cfg["pairs"][side].pop("leader")
                        else:
                            cfg["pairs"][side]["leader"] = None
                    robot, calls, made = fake_robot(robot_config=cfg)
                    self.assertFalse(robot.supports_human)
                    self.assertEqual(
                        set(robot.devices), {(s, "follower") for s in arms}
                    )
                    self.assertFalse(robot.forwarders)
                    expected = [
                        cfg["pairs"][side]["follower"]["channel"] for side in arms
                    ]
                    self.assertEqual(
                        calls, [(channel, "connect") for channel in expected]
                    )
                    try:
                        robot.enable()
                        action = np.arange(7 * len(arms), dtype=float) / 100
                        np.testing.assert_array_equal(robot.command(action), action)
                        for index, sdk in enumerate(made):
                            part = action[index * 7 : (index + 1) * 7]
                            np.testing.assert_array_equal(sdk.sent[0], part[:6])
                            self.assertEqual(sdk.sent[1], (part[6], 1.2))
                            sdk.joints.msg = part[:6].tolist()
                            sdk.gripper.msg.value = part[6]
                        snapshot = robot.snapshot()
                        np.testing.assert_array_equal(snapshot["state"], action)
                        self.assertIsNone(snapshot["leaders"])
                        self.assertIsNone(snapshot["human_action"])
                        for pair in snapshot["arms"].values():
                            self.assertEqual(set(pair), {"follower"})
                    finally:
                        robot.close()
                    self.assertEqual(
                        sum(call[1] == "stop" for call in calls), len(arms)
                    )
                    self.assertEqual(
                        sum(call[1] == "disconnect" for call in calls), len(arms)
                    )

    def test_partial_leader_configuration_cannot_begin_handover(self):
        cfg = config(["left", "right"])
        cfg["pairs"]["right"]["leader"] = None
        robot, calls, made = fake_robot(robot_config=cfg)
        try:
            robot.enable()
            robot.command([0.0] * 14)
            before = list(calls)
            with self.assertRaisesRegex(RobotError, "leader for every selected arm"):
                robot.enter_human()
            self.assertEqual(robot.mode, "MODEL")
            self.assertEqual(calls, before)
            self.assertFalse(robot.forwarders["left"].active)
            self.assertIsNone(robot.snapshot()["leaders"])
            self.assertEqual(len(made), 3)
        finally:
            robot.close()

    def test_follower_only_mock_has_no_fabricated_leader(self):
        for arms in (["left"], ["left", "right"]):
            with self.subTest(arms=arms):
                cfg = config(arms)
                for side in arms:
                    cfg["pairs"][side]["leader"] = None
                robot = MockRobot(cfg)
                robot.connect()
                robot.enable()
                target = np.arange(7 * len(arms), dtype=float)
                robot.command(target)
                with self.assertRaisesRegex(
                    RobotError, "leader for every selected arm"
                ):
                    robot.enter_human()
                snapshot = robot.snapshot()
                self.assertEqual(robot.mode, "MODEL")
                np.testing.assert_array_equal(snapshot["state"], target)
                self.assertIsNone(snapshot["leaders"])
                self.assertIsNone(snapshot["human_action"])
                for pair in snapshot["arms"].values():
                    self.assertEqual(set(pair), {"follower"})
                robot.close()

    def test_connect_is_read_only_and_close_without_enable_only_disconnects(self):
        robot, calls, made = fake_robot()
        self.assertEqual(calls, [("can31", "connect"), ("can30", "connect")])
        robot.close()
        self.assertEqual(calls[2:], [("can31", "disconnect"), ("can30", "disconnect")])
        robot.close()

    def test_enable_and_action_use_public_commands_in_order_without_transforming_targets(
        self,
    ):
        robot, calls, made = fake_robot(["left", "right"])
        robot.enable()
        for sdk in made:
            events = [call[1:] for call in calls if call[0] == sdk.name]
            self.assertEqual(
                events, [("connect",), ("follower",), ("enable",), ("speed", 73)]
            )
        action = np.asarray([10.0, -10.0, 0.123456789, 0.4, -0.7, 1.0, 0.123456789] * 2)
        sent = robot.command(action)
        np.testing.assert_array_equal(sent, action)
        for sdk in made:
            np.testing.assert_array_equal(sdk.sent[0], action[:6])
            self.assertEqual(sdk.sent[1], (action[6], 1.2))
        robot.close()
        self.assertEqual(sum(call[1] == "stop" for call in calls), 4)

    def test_handover_changes_leader_modes_and_routes_commands_exclusively(self):
        robot, calls, made = fake_robot()
        robot.enable()
        with patch.object(robot, "prepare_human_start") as prepare, patch.object(
            robot, "wait_for_alignment"
        ) as align:
            robot.enter_human()
        prepare.assert_not_called()
        align.assert_not_called()
        self.assertTrue(robot.forwarders["left"].active)
        self.assertEqual(calls[-1], ("can30", "leader"))
        snapshot = robot.snapshot()
        self.assertEqual(snapshot["arms"]["left"]["leader"]["source"], "command")
        self.assertIsNone(snapshot["leaders"])
        self.assertIsNone(snapshot["human_action"])
        with self.assertRaisesRegex(RobotError, "model/hold"):
            robot.command([0.0] * 7)
        robot.forwarders["left"].value = dict(
            position=np.arange(7.0), stamp=123.0, sequence=9
        )
        np.testing.assert_array_equal(robot.human_action(), np.arange(7.0))
        self.assertEqual(robot.snapshot()["human_action_stamp"], 123.0)
        self.assertEqual(robot.snapshot()["human_action_seq"], (9,))
        snapshot = robot.exit_human()
        self.assertFalse(robot.forwarders["left"].active)
        self.assertEqual(calls[-1], ("can30", "follower"))
        self.assertIsNotNone(snapshot["leaders"])
        robot.command([0.0] * 7)
        robot.close()

    def test_old_sdk_sample_is_returned_with_its_original_timestamp(self):
        robot, _, made = fake_robot()
        made[0].joints.timestamp = 100.0
        made[0].gripper.timestamp = 101.0
        sample = robot.snapshot()["arms"]["left"]["follower"]
        self.assertTrue(sample["valid"])
        self.assertEqual(sample["stamp"], 100.0)
        self.assertEqual(sample["joint_timestamp"], 100.0)
        self.assertEqual(sample["gripper_timestamp"], 101.0)
        np.testing.assert_array_equal(sample["position"], [0.1] * 6 + [0.02])
        robot.close()

    def test_missing_feedback_is_not_replaced_by_zeros_or_current_time(self):
        robot, _, made = fake_robot()
        made[1].joints = None
        snapshot = robot.snapshot()
        self.assertIsNone(snapshot["leaders"])
        leader = snapshot["arms"]["left"]["leader"]
        self.assertFalse(leader["valid"])
        self.assertIsNone(leader["position"])
        self.assertIsNone(leader["stamp"])
        made[0].gripper = None
        with self.assertRaisesRegex(RobotError, "feedback unavailable: left/follower"):
            robot.snapshot()
        robot.close()

    def test_sdk_exceptions_reach_caller_and_close_still_cleans_other_devices(self):
        robot, calls, made = fake_robot()
        robot.enable()

        def failed(*args):
            raise OSError("SDK failed")

        made[0].move_js = failed
        with self.assertRaisesRegex(OSError, "SDK failed"):
            robot.command([0.0] * 7)
        made[0].disconnect = failed
        with self.assertRaisesRegex(RobotError, "SDK failed"):
            robot.close()
        self.assertIn(("can30", "disconnect"), calls)
        self.assertFalse(robot.connected)

    def test_single_and_dual_mock_preserve_command_values(self):
        for arms in (["left"], ["right"], ["left", "right"]):
            robot = MockRobot(config(arms))
            robot.connect()
            robot.enable()
            action = np.full(7 * len(arms), 0.123456789)
            np.testing.assert_array_equal(robot.command(action), action)
            robot.enter_human()
            with self.assertRaises(RobotError):
                robot.command(action)
            robot.set_human_action(action * 2)
            np.testing.assert_array_equal(robot.human_action(), action * 2)
            robot.exit_human()
            robot.close()


class CameraCleanupTests(unittest.TestCase):
    def test_pipeline_stop_error_is_reported_after_capture_thread_exits(self):
        rig = CameraRig({"camera_map": {}, "cameras": {}})

        def join(timeout):
            rig._errors["top"] = RuntimeError("pipeline stop failed")

        rig._threads = [
            SimpleNamespace(join=join, is_alive=lambda: False, name="camera-top")
        ]
        with self.assertRaisesRegex(RuntimeError, "pipeline stop failed"):
            rig.close()
        self.assertTrue(rig._stop.is_set())


if __name__ == "__main__":
    unittest.main()
