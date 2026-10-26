"""Offline checks that initial JS motion settles before human handover."""

import unittest
from contextlib import contextmanager
from unittest.mock import patch

import numpy as np

from piper_control.hardware.robot import RobotError
from tests.test_sdk import config, fake_robot


class PairAlignmentTests(unittest.TestCase):
    def robot(self, arms=None, initial_target=None):
        cfg = config(arms or ["right"])
        cfg["startup_alignment_timeout_sec"] = 2.0
        robot, calls, made = fake_robot(robot_config=cfg)
        self.addCleanup(robot.close)
        robot.enable()
        if initial_target is not None:
            for index, side in enumerate(robot.arms):
                part = initial_target[index * 7 : (index + 1) * 7]
                for role in ("follower", "leader"):
                    self.pose(robot, side, role, part[:6], part[6])
        target = robot.snapshot()["state"].copy()
        robot.command(target)
        return robot, calls, made, target

    @staticmethod
    def pose(robot, side, role, joints, gripper=0.02):
        sdk = robot.devices[(side, role)]["sdk"]
        sdk.joints.msg = list(joints)
        sdk.gripper.msg.value = gripper

    @contextmanager
    def feedback(self, robot, update=None, frozen=()):
        """Advance fake time and publish fresh position samples on every sleep."""
        clock = [100.0]
        for device in robot.devices.values():
            for field in ("joints", "gripper"):
                sample = getattr(device["sdk"], field)
                if sample is not None:
                    sample.timestamp = clock[0]

        def sleep(seconds):
            clock[0] += seconds
            if update is not None:
                update(seconds, clock[0])
            for (side, role), device in robot.devices.items():
                for field in ("joints", "gripper"):
                    sample = getattr(device["sdk"], field)
                    if sample is not None and (side, role, field) not in frozen:
                        sample.timestamp = clock[0]

        with patch("piper_control.hardware.robot.time.monotonic", side_effect=lambda: clock[0]), patch(
            "piper_control.hardware.robot.time.time", side_effect=lambda: clock[0]
        ), patch("piper_control.hardware.robot.time.sleep", side_effect=sleep):
            yield clock

    def timeout(self, robot, target, **feedback_options):
        with self.feedback(robot, **feedback_options):
            with self.assertRaisesRegex(RobotError, "Startup motion settle timed out") as caught:
                robot.wait_for_alignment(target, lambda: False)
        return str(caught.exception)

    def test_reported_live_poses_settle_despite_offset_from_initial_target(self):
        target = np.asarray([0.049602, -0.034767, 0.033894, -0.054297, 0.401304, 0.277141, 0.0])
        robot, calls, made, target = self.robot(initial_target=target)
        follower = [0.049602, -0.004224, 0.005812, -0.052691, 0.401670, 0.276931]
        leader = [0.052430, -0.003560, 0.004067, -0.054192, 0.390325, 0.261642]
        self.pose(robot, "right", "follower", follower, 0.0)
        self.pose(robot, "right", "leader", leader, 0.0002)
        before = list(calls)
        with self.feedback(robot) as clock:
            result = robot.wait_for_alignment(target, lambda: False)
            self.assertGreaterEqual(clock[0] - 100.0, 0.8)
        self.assertEqual(result["result"], "settled")
        self.assertGreaterEqual(result["settle_time_sec"], 0.8)
        self.assertEqual(result["reference"], "current_follower")
        self.assertEqual(result["pair_errors"]["right"], {
            "max_joint_error_rad": 0.015289,
            "max_joint_error_joint": 6,
            "gripper_error_m": 0.0002,
        })
        for role in ("follower", "leader"):
            self.assertGreater(result["target_errors"]["right"][role]["max_joint_error_rad"], 0.03)
        self.assertEqual(calls, before)
        self.assertTrue(all(len(sdk.sent) == 2 for sdk in made))
        self.assertFalse(robot.forwarders["right"].active)

    def test_stable_pair_residuals_are_reported_without_precision_gate(self):
        for joint_index, joint_error, gripper_error in ((3, 0.05, 0.0), (4, 0.13, 0.004)):
            with self.subTest(joint_index=joint_index):
                robot, calls, made, target = self.robot()
                sdk = robot.devices[("right", "leader")]["sdk"]
                sdk.joints.msg[joint_index] += joint_error
                sdk.gripper.msg.value += gripper_error
                before = list(calls)
                with self.feedback(robot):
                    result = robot.wait_for_alignment(target, lambda: False)
                self.assertEqual(result["result"], "settled")
                self.assertEqual(result["pair_errors"]["right"], {
                    "max_joint_error_rad": joint_error,
                    "max_joint_error_joint": joint_index + 1,
                    "gripper_error_m": gripper_error,
                })
                self.assertEqual(calls, before)
                self.assertFalse(robot.forwarders["right"].active)

    def test_matching_poses_do_not_override_fault_or_disable(self):
        for state in ("driver_fault", "disabled"):
            with self.subTest(state=state):
                robot, calls, made, target = self.robot()
                sdk = robot.devices[("right", "leader")]["sdk"]
                if state == "driver_fault":
                    sdk.drivers[4].msg.foc_status.stall_status = True
                    expected = "Driver fault during startup alignment"
                else:
                    sdk.enable_flags[4] = False
                    expected = "Motor enable lost during startup alignment"
                before = list(calls)
                with self.assertRaisesRegex(RobotError, expected):
                    robot.wait_for_alignment(target, lambda: False)
                self.assertEqual(calls, before)
                self.assertFalse(robot.forwarders["right"].active)

    def test_joint_motion_is_checked_across_the_whole_window(self):
        for increment in (0.003, 0.0005):
            with self.subTest(increment=increment):
                robot, calls, made, target = self.robot()

                def moving(seconds, now):
                    # Both poses match, including during slow per-frame drift.
                    for role in ("follower", "leader"):
                        robot.devices[("right", role)]["sdk"].joints.msg[3] += increment

                before = list(calls)
                message = self.timeout(robot, target, update=moving)
                self.assertIn("JS motion not settled", message)
                self.assertEqual(calls, before)

    def test_gripper_motion_prevents_handover(self):
        robot, calls, made, target = self.robot()

        def moving(seconds, now):
            robot.devices[("right", "leader")]["sdk"].gripper.msg.value += 0.0002

        before = list(calls)
        self.assertIn("JS motion not settled", self.timeout(robot, target, update=moving))
        self.assertEqual(calls, before)

    def test_settle_window_starts_after_motion_finishes(self):
        robot, calls, made, target = self.robot()

        def moving_then_stopped(seconds, now):
            if now < 100.5:
                robot.devices[("right", "leader")]["sdk"].joints.msg[3] += 0.003

        with self.feedback(robot, update=moving_then_stopped) as clock:
            result = robot.wait_for_alignment(target, lambda: False)
            self.assertGreaterEqual(clock[0] - 100.0, 1.25)
            self.assertLess(clock[0] - 100.0, 1.4)
        self.assertEqual(result["result"], "settled")

    def test_cached_samples_do_not_count_as_elapsed_settling_time(self):
        for stalled in ("all", "leader_gripper"):
            with self.subTest(stalled=stalled):
                robot, calls, made, target = self.robot()
                frozen = (
                    [(side, role, field) for side, role in robot.devices for field in ("joints", "gripper")]
                    if stalled == "all" else [("right", "leader", "gripper")]
                )
                before = list(calls)
                message = self.timeout(robot, target, frozen=frozen)
                self.assertIn("waiting for new position feedback", message)
                self.assertEqual(calls, before)

    def test_all_selected_pairs_must_settle(self):
        robot, calls, made, target = self.robot(["left", "right"])
        for role in ("follower", "leader"):
            self.pose(robot, "left", role, [0.1] * 6, 0.02)
            self.pose(robot, "right", role, [0.8] * 6, 0.07)
        before = list(calls)
        with self.feedback(robot):
            result = robot.wait_for_alignment(target, lambda: False)
        self.assertEqual(set(result["pair_errors"]), {"left", "right"})
        self.assertEqual(result["pair_errors"]["left"]["max_joint_error_rad"], 0.0)
        self.assertEqual(result["pair_errors"]["right"]["max_joint_error_rad"], 0.0)

        def moving(seconds, now):
            robot.devices[("right", "leader")]["sdk"].joints.msg[1] += 0.003

        self.assertIn("JS motion not settled", self.timeout(robot, target, update=moving))
        self.assertEqual(calls, before)

    def test_nonfinite_joint_or_gripper_feedback_never_counts_as_settled(self):
        for role, field, value in (
            ("leader", "joint", float("nan")),
            ("follower", "joint", float("inf")),
            ("leader", "gripper", float("nan")),
        ):
            with self.subTest(role=role, field=field, value=value):
                robot, calls, made, target = self.robot()
                sdk = robot.devices[("right", role)]["sdk"]
                if field == "joint":
                    sdk.joints.msg[4] = value
                else:
                    sdk.gripper.msg.value = value
                before = list(calls)
                message = self.timeout(robot, target)
                self.assertIn("pair feedback unavailable", message)
                self.assertIn("pair_errors={'right': None}", message)
                self.assertEqual(calls, before)

    def test_missing_leader_feedback_prevents_handover(self):
        robot, calls, made, target = self.robot()
        robot.devices[("right", "leader")]["sdk"].joints = None
        before = list(calls)
        message = self.timeout(robot, target)
        self.assertIn("pair feedback unavailable", message)
        self.assertIn("missing_feedback=['joint_angles']", message)
        self.assertEqual(calls, before)


if __name__ == "__main__":
    unittest.main()
