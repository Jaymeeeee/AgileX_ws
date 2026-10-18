"""Offline bag parsing and SDK playback tests; never connect to real CAN."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import time
import unittest
from unittest.mock import Mock

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import JointState

from model_deploy.core import ActionGuard, load_config
from model_deploy.controller import Controller
from model_deploy.replay import (
    INPUT_NAMES, KEYS, SDKReplayFeedback, check_start, load_trajectory,
    move_to_start, play_trajectory, replay_config,
)

ROOT = Path(__file__).resolve().parents[1]


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.base = load_config(ROOT/'config/act.yaml')
        self.cfg = replay_config(self.base, ROOT/'config/replay.yaml')
        self.pose = np.tile([0., .5, -.5, 0., 0., 0., .04], 2)
        self.tmp = TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'data.bag'

    def bag(self, omit=None, stamps=(10., 10.02, 10.04), mutate=None):
        with rosbag.Bag(str(self.path), 'w') as bag:
            for index, stamp in enumerate(stamps):
                for k, key in enumerate(KEYS):
                    if key == omit:
                        continue
                    msg = JointState()
                    msg.header.stamp = rospy.Time.from_sec(stamp)
                    msg.name = INPUT_NAMES[k][::-1]
                    values = self.pose[[slice(0, 6), slice(6, 7), slice(7, 13), slice(13, 14)][k]].copy()
                    values[0] += index*(.01 if k % 2 == 0 else .001)
                    msg.position = values[::-1].tolist()
                    if mutate:
                        mutate(key, index, msg)
                    bag.write(self.cfg['replay']['topics'][key], msg, rospy.Time.from_sec(stamp))
        return load_trajectory(self.path, self.cfg)

    def test_named_joints_interpolation_and_both_grippers(self):
        grid, points, plan = self.bag()
        np.testing.assert_allclose(grid, [0., .01, .02, .03, .04])
        np.testing.assert_allclose(points[0], self.pose)
        np.testing.assert_allclose(points[1, [0, 6, 7, 13]], [.005, .0405, .005, .0405])
        np.testing.assert_allclose(points[-1, [0, 6, 7, 13]], [.02, .042, .02, .042])
        self.assertEqual(plan['source'], 'follower_actual')
        self.assertAlmostEqual(plan['nominal_playback_sec'], .08)
        self.assertEqual(plan['source_counts'], dict.fromkeys(KEYS, 3))
        # Directory input has the same semantics as data.bag.
        np.testing.assert_equal(load_trajectory(self.path.parent, self.cfg)[1], points)

    def test_rate_resampling_and_final_endpoint(self):
        self.cfg = replay_config(self.base, ROOT/'config/replay.yaml', rate=1)
        grid, points, plan = self.bag(stamps=(10., 10.02, 10.045))
        np.testing.assert_allclose(grid, [0., .02, .04, .045])
        self.assertEqual(points.shape, (4, 14))
        self.assertAlmostEqual(plan['nominal_playback_sec'], .045)

    def test_config_does_not_modify_act_and_rejects_invalid_rate(self):
        self.assertEqual(self.base['policy']['fps'], 15)
        self.assertEqual(self.cfg['policy']['fps'], 50)
        for rate in (0, -1, float('nan'), float('inf'), 3, True):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                replay_config(self.base, ROOT/'config/replay.yaml', rate=rate)

    def test_missing_stream_rejected(self):
        with self.assertRaisesRegex(ValueError, 'right_gripper.*two messages'):
            self.bag(omit='right_gripper')

    def test_gap_rejected(self):
        with self.assertRaisesRegex(ValueError, 'source gap'):
            self.bag(stamps=(10., 10.02, 10.3))

    def test_bad_timestamps_rejected(self):
        for bad_stamp, message in ((0, 'invalid header'), (20, 'invalid header'), (10., 'non-increasing')):
            def mutate(key, index, msg):
                if key == 'left_joints' and index == 1:
                    msg.header.stamp = rospy.Time.from_sec(bad_stamp)
            with self.subTest(stamp=bad_stamp), self.assertRaisesRegex(ValueError, message):
                self.bag(mutate=mutate)

    def test_missing_joint_and_nonfinite_rejected(self):
        for value in ('name', float('nan')):
            def mutate(key, index, msg):
                if key == 'left_joints' and index == 1:
                    if value == 'name':
                        msg.name[0] = 'unrelated_joint'
                    else:
                        msg.position[0] = value
            with self.subTest(value=value), self.assertRaises((ValueError, KeyError)):
                self.bag(mutate=mutate)

    def test_out_of_range_rejected_before_motion(self):
        def mutate(key, index, msg):
            if key == 'right_joints':
                msg.position[0] = 4.0
        with self.assertRaisesRegex(ValueError, 'recorded feedback exceeds physical range'):
            self.bag(mutate=mutate)

    def test_small_negative_gripper_feedback_is_reported_and_projected(self):
        def mutate(key, index, msg):
            if key == 'left_gripper':
                msg.position[0] = -.0001
        _, points, plan = self.bag(mutate=mutate)
        np.testing.assert_equal(points[:, 6], 0)
        self.assertEqual(plan['projected_source_samples']['left_gripper'], 3)
        self.assertAlmostEqual(plan['max_projection_per_source']['left_gripper'], .0001)

    def test_active_or_incomplete_recording_rejected(self):
        self.bag()
        active = Path(str(self.path)+'.active')
        active.touch()
        with self.assertRaisesRegex(ValueError, 'completed .bag'):
            load_trajectory(self.path, self.cfg)
        active.unlink()
        (self.path.parent/'manifest.json').write_text('{"state":"recording"}')
        with self.assertRaisesRegex(ValueError, 'manifest is not complete'):
            load_trajectory(self.path, self.cfg)

    def test_common_interval_does_not_extrapolate(self):
        def mutate(key, index, msg):
            if key == 'right_gripper':
                msg.header.stamp += rospy.Duration.from_sec(.005)
        grid, points, plan = self.bag(mutate=mutate)
        self.assertEqual(plan['common_start_ns'], 10005000000)
        self.assertEqual(plan['common_end_ns'], rospy.Time.from_sec(10.04).to_nsec())
        self.assertAlmostEqual(grid[-1], .035)
        self.assertAlmostEqual(points[0, 0], .0025)
        self.assertAlmostEqual(points[0, 13], .04)

    def test_start_check_independent_of_disabled_model_gates(self):
        check_start(self.cfg, self.pose, self.pose)
        target = self.pose.copy()
        target[5] = .16
        with self.assertRaisesRegex(ValueError, 'left_joint_6'):
            check_start(self.cfg, target, self.pose)
        target = self.pose.copy()
        target[13] += .02
        with self.assertRaisesRegex(ValueError, 'right_gripper_width'):
            check_start(self.cfg, target, self.pose)

    def simulation(self, follow=True):
        elapsed = [0.]
        actual = self.pose.copy()
        guard = ActionGuard(self.cfg['control'], 1/self.cfg['replay']['command_hz'])
        guard.commit(actual)
        obs = NS(actual=lambda: actual.copy())
        def send(target):
            sent = guard.prepare(target, actual)
            guard.commit(sent)
            if follow:
                actual[:] += np.clip(sent-actual, -.003, .003)
            return sent
        ctrl = NS(send=send, check_fault=lambda: None, guard=guard)
        def sleep(dt):
            elapsed[0] += dt
        return ctrl, obs, lambda: elapsed[0], sleep

    def test_approach_mismatched_start_is_slow_and_confirmed(self):
        ctrl, obs, clock, sleep = self.simulation()
        target = self.pose.copy()
        target[10], target[12], target[6] = .1702, -.1803, .0
        with self.assertRaisesRegex(ValueError, 'Replay start pose mismatch'):
            check_start(self.cfg, target, obs.actual())
        old_speed = ctrl.guard.speed.copy()
        events = []
        result = move_to_start(self.cfg, target, ctrl, obs, clock=clock, sleep=sleep, report=events.append)
        self.assertEqual(result['state'], 'complete')
        check_start(self.cfg, target, obs.actual())
        np.testing.assert_allclose(obs.actual(), target, atol=.001)
        commands = np.asarray([self.pose]+[e['commanded'] for e in events])
        max_step = np.tile([.2/50]*6+[.01/50], 2)
        self.assertTrue(np.all(np.abs(np.diff(commands, axis=0)) <= max_step+1e-10))
        self.assertGreaterEqual(result['elapsed_sec'], 4.)
        np.testing.assert_equal(ctrl.guard.speed, old_speed)

    def test_approach_timeout_restores_controller_limits(self):
        ctrl, obs, clock, sleep = self.simulation(follow=False)
        old_speed = ctrl.guard.speed.copy()
        self.cfg['replay']['approach_timeout_sec'] = .2
        target = self.pose.copy()
        target[0] = .4
        with self.assertRaisesRegex(RuntimeError, 'Move to bag start timed out'):
            move_to_start(self.cfg, target, ctrl, obs, clock=clock, sleep=sleep)
        np.testing.assert_equal(ctrl.guard.speed, old_speed)

    def test_approach_cancel_and_invalid_target_do_not_send(self):
        ctrl, obs, clock, sleep = self.simulation()
        ctrl.send = Mock(wraps=ctrl.send)
        old_speed = ctrl.guard.speed.copy()
        with self.assertRaises(InterruptedError):
            move_to_start(self.cfg, self.pose, ctrl, obs, clock=clock, sleep=sleep, cancelled=lambda: True)
        target = self.pose.copy()
        target[5] = 4.
        with self.assertRaisesRegex(ValueError, 'exceeds configured limits'):
            move_to_start(self.cfg, target, ctrl, obs, clock=clock, sleep=sleep)
        ctrl.send.assert_not_called()
        np.testing.assert_equal(ctrl.guard.speed, old_speed)

    def test_slow_following_waits_visits_every_point_and_settles(self):
        ctrl, obs, clock, sleep = self.simulation()
        points = np.tile(self.pose, (4, 1))
        points[:, 0] += [0., .05, .10, .15]
        grid = np.array([0., .01, .02, .03])
        events = []
        result = play_trajectory(self.cfg, grid, points, ctrl, obs,
                                 clock=clock, sleep=sleep, report=events.append)
        self.assertEqual(result['state'], 'complete')
        self.assertGreater(result['elapsed_sec'], .03/.5+.5)
        self.assertEqual(sorted(set(e['index'] for e in events)), list(range(4)))
        self.assertTrue(all(b['index']-a['index'] in (0, 1) for a, b in zip(events, events[1:])))
        self.assertTrue(all(b['elapsed_sec']-a['elapsed_sec'] >= .019999 for a, b in zip(events, events[1:])))
        np.testing.assert_allclose(result['final_actual'], points[-1], atol=.001)

    def test_stalled_gripper_times_out_before_advancing(self):
        ctrl, obs, clock, sleep = self.simulation(follow=False)
        self.cfg['replay']['waypoint_timeout_sec'] = .6
        points = np.tile(self.pose, (3, 1))
        points[1:, 6] += .015
        events = []
        with self.assertRaisesRegex(RuntimeError, 'waypoint timed out'):
            play_trajectory(self.cfg, np.array([0., .01, .02]), points, ctrl, obs,
                            clock=clock, sleep=sleep, report=events.append)
        self.assertEqual(max(e['index'] for e in events), 1)

    def test_cancellation_sends_no_next_command(self):
        ctrl, obs, clock, sleep = self.simulation()
        ctrl.send = Mock(wraps=ctrl.send)
        with self.assertRaises(InterruptedError):
            play_trajectory(self.cfg, [0.], [self.pose], ctrl, obs,
                            clock=clock, sleep=sleep, cancelled=lambda: True)
        ctrl.send.assert_not_called()

    def sdk_feedback(self):
        hardware = Mock()
        hardware.actual.side_effect = AssertionError('Do not use replayed ROS state')
        obs = SDKReplayFeedback(self.cfg, hardware)
        def msg(value):
            return NS(msg=value, timestamp=time.time())
        arms, grippers = {}, {}
        for side in ('left', 'right'):
            arms[side] = Mock()
            arms[side].get_joint_angles.side_effect = lambda: msg(self.pose[:6].tolist())
            arms[side].get_arm_status.side_effect = lambda: msg(NS(arm_status=0, err_code=0))
            grippers[side] = Mock()
            grippers[side].get_gripper_status.side_effect = lambda: msg(NS(
                value=.04, mode='width', foc_status=NS(voltage_too_low=False, motor_overheating=False,
                driver_overcurrent=False, driver_overheating=False, sensor_status=False, driver_error_status=False)))
        obs.controller = NS(arms=arms, grippers=grippers)
        return obs

    def test_actual_is_live_sdk_feedback_not_ros_bag(self):
        obs = self.sdk_feedback()
        np.testing.assert_allclose(obs.actual(), self.pose)
        obs.deployment_hardware()
        obs.hardware.deployment_hardware.assert_called_once()
        obs.hardware.actual.assert_not_called()

    def test_stale_sdk_state_or_gripper_fault_rejected(self):
        obs = self.sdk_feedback()
        obs.controller.arms['left'].get_joint_angles.side_effect = lambda: NS(
            timestamp=time.time()-2, msg=self.pose[:6])
        with self.assertRaisesRegex(RuntimeError, 'stale SDK'):
            obs.actual()
        obs = self.sdk_feedback()
        getter = obs.controller.grippers['right'].get_gripper_status
        value = getter()
        value.msg.foc_status.driver_overcurrent = True
        getter.side_effect = None
        getter.return_value = value
        with self.assertRaisesRegex(RuntimeError, 'right.*driver_overcurrent'):
            obs.actual()

    def test_recheck_start_before_enable_or_motion(self):
        obs = Mock()
        obs.actual.return_value = self.pose.copy()
        controller = Controller(self.cfg, obs, require_images=False)
        controller.arms = {s: Mock() for s in ('left', 'right')}
        for arm in controller.arms.values():
            arm.has_comm_error.return_value = False
        wrong_start = self.pose.copy()
        wrong_start[0] = .5
        with self.assertRaisesRegex(ValueError, 'Replay start pose mismatch'):
            controller.start(on_initial_pose=lambda a: check_start(self.cfg, wrong_start, a))
        for arm in controller.arms.values():
            arm.enable.assert_not_called()
            arm.move_j.assert_not_called()

    def test_replay_routes_to_sdk_and_closes_at_measured_pose(self):
        actual = self.pose.copy()
        obs = Mock()
        obs.actual.side_effect = lambda **_: actual.copy()
        controller = Controller(self.cfg, obs, require_images=False)
        for index, side in enumerate(('left', 'right')):
            arm, grip = Mock(), Mock()
            arm.has_comm_error.return_value = False
            arm.move_j.side_effect = lambda q, i=index: actual.__setitem__(slice(i*7, i*7+6), q)
            grip.move_gripper_m.side_effect = lambda width, force, i=index: actual.__setitem__(i*7+6, width)
            controller.arms[side], controller.grippers[side] = arm, grip
        controller.active = True
        controller.guard.commit(actual)
        elapsed = [0.]
        def sleep(dt):
            elapsed[0] += dt
        points = np.tile(self.pose, (2, 1))
        points[1] += np.tile([.02, 0, 0, 0, 0, 0, .002], 2)
        try:
            result = play_trajectory(self.cfg, [0., .01], points, controller, obs,
                                     clock=lambda: elapsed[0], sleep=sleep)
            self.assertEqual(result['state'], 'complete')
        finally:
            controller.close()
        np.testing.assert_allclose(actual, points[-1])
        for index, side in enumerate(('left', 'right')):
            controller.arms[side].move_j.assert_called_with(points[-1, index*7:index*7+6].tolist())
            controller.grippers[side].move_gripper_m.assert_called_with(
                points[-1, index*7+6], self.cfg['control']['gripper_force'])
            controller.arms[side].disconnect.assert_called_once()
        self.assertFalse(controller.active)


if __name__ == '__main__':
    unittest.main()
