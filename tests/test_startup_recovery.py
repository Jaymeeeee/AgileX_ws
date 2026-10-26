import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from piper_control.hardware.robot import RobotError
from tests.test_sdk import config, fake_robot


def arm_status(sdk, value, timestamp=None):
    sample = SimpleNamespace(
        timestamp=time.time() if timestamp is None else timestamp,
        msg=SimpleNamespace(arm_status=value, err_code=0),
    )
    sdk.get_arm_status = lambda: sample


class StartupRecoveryTests(unittest.TestCase):
    def test_normal_arms_need_no_reset_even_when_already_enabled(self):
        robot, calls, made = fake_robot(["right"])
        try:
            for sdk in made:
                arm_status(sdk, 0, timestamp=10.0)
                sdk.receive_enable_feedback([True] * 6, stamp=10.0)
            before = list(calls)
            robot.prepare_human_start()
            self.assertEqual(calls, before + [(made[1].name, "follower")])
            self.assertTrue(robot.touched)
            self.assertFalse(robot.enabled)
            self.assertTrue(all(sdk.enable_flags == [True] * 6 for sdk in made))
        finally:
            robot.close()

    def test_only_stopped_arm_is_reset_and_waited_for(self):
        robot, calls, made = fake_robot(["right"])
        follower, leader = made
        try:
            arm_status(follower, 0)
            arm_status(leader, 1)
            before = len(calls)
            robot.prepare_human_start()
            self.assertEqual(calls[before:], [(leader.name, "follower"), (leader.name, "reset")])
            self.assertEqual(follower.enable_flags, [True] * 6)
            self.assertEqual(leader.enable_flags, [False] * 6)
            self.assertTrue(robot.touched)
            self.assertTrue(all(not sdk.sent for sdk in made))
        finally:
            robot.close()

    def test_fault_on_second_arm_blocks_reset_of_every_arm(self):
        robot, calls, made = fake_robot(["right"])
        try:
            for sdk in made:
                arm_status(sdk, 1)
            faults = made[1].drivers[4].msg.foc_status
            faults.driver_overcurrent = True
            faults.driver_error_status = True
            faults.stall_status = True
            before = list(calls)
            with self.assertRaisesRegex(RobotError, "Driver fault before startup recovery") as caught:
                robot.prepare_human_start()
            message = str(caught.exception)
            for field in ("right/leader", "joint_5", "driver_overcurrent",
                          "driver_error_status", "stall_status"):
                self.assertIn(field, message)
            self.assertEqual(calls, before)
            self.assertFalse(robot.touched)
        finally:
            robot.close()

    def test_quiet_leader_feedback_returns_only_after_follower_mode(self):
        for electronically_stopped in (False, True):
            with self.subTest(electronically_stopped=electronically_stopped):
                robot, calls, made = fake_robot(["right"])
                follower, leader = made
                leader.arm_status = int(electronically_stopped)
                restored = [False]
                original_status = leader.get_arm_status
                original_driver = leader.get_driver_states
                original_mode = leader.set_follower_mode
                leader.get_arm_status = lambda: original_status() if restored[0] else None
                leader.get_driver_states = lambda i: original_driver(i) if restored[0] else None

                def restore_feedback():
                    original_mode()
                    restored[0] = True

                leader.set_follower_mode = restore_feedback
                try:
                    # connect()'s local label is not evidence of the actual mode.
                    self.assertEqual(robot.devices[("right", "leader")]["mode"], "follower")
                    self.assertIsNone(leader.get_arm_status())
                    before = len(calls)
                    robot.prepare_human_start()
                    expected = [(leader.name, "follower")]
                    if electronically_stopped:
                        expected.append((leader.name, "reset"))
                    self.assertEqual(calls[before:], expected)
                    self.assertTrue(restored[0])
                    self.assertTrue(robot.touched)
                    self.assertFalse(robot.enabled)
                    self.assertTrue(all(not sdk.sent for sdk in made))
                    self.assertFalse(robot.forwarders["right"].active)
                finally:
                    robot.close()

    def test_fault_revealed_by_follower_mode_blocks_reset_enable_and_motion(self):
        robot, calls, made = fake_robot(["right"])
        follower, leader = made
        for sdk in made:
            sdk.arm_status = 1
        leader.drivers[4].msg.foc_status.driver_overcurrent = True
        restored = [False]
        original_status = leader.get_arm_status
        original_driver = leader.get_driver_states
        original_mode = leader.set_follower_mode
        leader.get_arm_status = lambda: original_status() if restored[0] else None
        leader.get_driver_states = lambda i: original_driver(i) if restored[0] else None

        def restore_feedback():
            original_mode()
            restored[0] = True

        leader.set_follower_mode = restore_feedback
        before = len(calls)
        try:
            with self.assertRaisesRegex(RobotError, "Driver fault before startup recovery") as caught:
                robot.prepare_human_start()
            self.assertIn("right/leader", str(caught.exception))
            self.assertIn("joint_5", str(caught.exception))
            self.assertIn("driver_overcurrent", str(caught.exception))
            self.assertEqual(calls[before:], [(leader.name, "follower")])
            self.assertTrue(all(not sdk.sent for sdk in made))
            self.assertFalse(robot.enabled)
        finally:
            robot.close()

    def test_already_requested_stop_prevents_mode_restoration(self):
        robot, calls, made = fake_robot(["right"])
        try:
            before = list(calls)
            with self.assertRaisesRegex(RobotError, "Stop requested"):
                robot.prepare_human_start(lambda: True)
            self.assertEqual(calls, before)
            self.assertFalse(robot.touched)
        finally:
            robot.close()

    def test_missing_status_or_driver_feedback_times_out_without_reset(self):
        for missing in ("arm", "driver", "driver_timestamp"):
            with self.subTest(missing=missing):
                cfg = config(["right"])
                cfg["startup_enable_timeout_sec"] = 0.04
                robot, calls, made = fake_robot(robot_config=cfg)
                for sdk in made:
                    arm_status(sdk, 1)
                leader = made[1]
                if missing == "arm":
                    leader.get_arm_status = lambda: None
                elif missing == "driver":
                    original = leader.get_driver_states
                    leader.get_driver_states = lambda index: None if index == 5 else original(index)
                else:
                    leader.drivers[4].timestamp = 0.0
                clock = [100.0]

                def sleep(seconds):
                    clock[0] += seconds

                try:
                    before = list(calls)
                    with patch("piper_control.hardware.robot.time.monotonic", side_effect=lambda: clock[0]), patch(
                        "piper_control.hardware.robot.time.sleep", side_effect=sleep
                    ):
                        with self.assertRaisesRegex(RobotError, "Startup recovery feedback timed out") as caught:
                            robot.prepare_human_start()
                    self.assertIn("right/leader", str(caught.exception))
                    field = "missing_arm_status=True" if missing == "arm" else "missing_driver_feedback=[5]"
                    self.assertIn(field, str(caught.exception))
                    self.assertEqual(calls, before + [(leader.name, "follower")])
                finally:
                    robot.close()

    def test_waits_for_status_before_deciding_whether_to_reset(self):
        robot, calls, made = fake_robot(["right"])
        follower, leader = made
        arm_status(follower, 0)
        leader.get_arm_status = lambda: None

        def receive_status(seconds):
            self.assertFalse(any(call[1] == "reset" for call in calls))
            arm_status(leader, 0)

        try:
            before = list(calls)
            with patch("piper_control.hardware.robot.time.sleep", side_effect=receive_status) as sleep:
                robot.prepare_human_start()
            sleep.assert_called_once()
            self.assertEqual(calls, before + [(leader.name, "follower")])
        finally:
            robot.close()

    def test_stop_interrupts_wait_for_initial_feedback_without_reset(self):
        robot, calls, made = fake_robot(["right"])
        for sdk in made:
            sdk.get_arm_status = lambda: None
        stopped = threading.Event()
        try:
            before = list(calls)
            with patch("piper_control.hardware.robot.time.sleep", side_effect=lambda seconds: stopped.set()):
                with self.assertRaisesRegex(RobotError, "Stop requested"):
                    robot.prepare_human_start(stopped.is_set)
            self.assertEqual(calls, before + [(made[1].name, "follower")])
        finally:
            robot.close()


if __name__ == "__main__":
    unittest.main()
