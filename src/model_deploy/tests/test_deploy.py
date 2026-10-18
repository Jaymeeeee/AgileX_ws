import importlib.util
import json
from pathlib import Path
import socket
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from sensor_msgs.msg import Image, JointState
from model_deploy.core import ActionGuard, load_config, joint_values, rgb_array, recv_packet, send_packet
from model_deploy.controller import Controller
from model_deploy.observations import Observations
from hardware_bringup.config import check_can

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT/'config/act.yaml'


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(CFG)
        # Explicit regression settings, independent of operator edits to live YAML.
        self.cfg['control'].update(check_initial_target_error=True, check_runtime_target_error=True,
                                   check_gripper_tracking_error=True,
                                   max_initial_error_rad=.20, max_initial_error_m=.02,
                                   max_target_error_rad=.50, max_target_error_m=.04,
                                   max_tracking_error_rad=.20, max_tracking_error_m=.02,
                                   max_joint_speed_rad_s=.60, max_gripper_speed_m_s=.03)
        self.pose = np.tile([0, .5, -.5, 0, 0, 0, .04], 2)

    def test_disabled_model_distance_gates_retain_bounded_commands(self):
        self.cfg['control'].update(check_initial_target_error=False, check_runtime_target_error=False)
        guard = ActionGuard(self.cfg['control'], 1/15)
        actual = self.pose.copy()
        actual[5] = .9179
        target = actual.copy()
        target[5] = 1.4288  # Reported stop: model is 0.5109 rad ahead.
        target[6] = .10
        for initial in (False, True):
            sent = guard.prepare(target, actual, initial=initial)
            self.assertAlmostEqual(sent[5], .9579)
            self.assertAlmostEqual(sent[6], .042)
        # Even extreme finite predictions cannot escape command bounds/rate limits.
        sent = guard.prepare(np.full(14, 100.), actual)
        self.assertTrue(np.all(sent <= guard.high))
        self.assertTrue(np.all(sent >= guard.low))
        self.assertTrue(np.all(np.abs(sent-actual) <= guard.speed*guard.period + 1e-10))
        with self.assertRaisesRegex(ValueError, 'finite 14D'):
            guard.prepare(np.full(14, np.nan), actual)
        actual[5] = 4
        with self.assertRaisesRegex(ValueError, 'exceeds configured limits'):
            guard.prepare(target, actual)

    def test_disabled_model_gates_still_report_tracking_joint_and_gripper(self):
        self.cfg['control'].update(check_initial_target_error=False, check_runtime_target_error=False)
        guard = ActionGuard(self.cfg['control'], 1/15)
        previous = self.pose.copy()
        previous[5] += .25
        previous[13] += .03
        guard.commit(previous)
        with self.assertRaises(ValueError) as caught:
            guard.prepare(self.pose, self.pose)
        message = str(caught.exception)
        self.assertIn('left_joint_6: commanded=0.2500, actual=0.0000, error=0.2500, limit=0.2000 rad', message)
        self.assertIn('right_gripper_width: commanded=0.0700, actual=0.0400, error=0.0300, limit=0.0200 m', message)

    def test_model_distance_switches_are_independent_and_validate_boolean(self):
        target = self.pose.copy()
        target[5] = 1
        for initial in (False, True):
            self.cfg['control'].update(check_initial_target_error=initial,
                                       check_runtime_target_error=not initial)
            guard = ActionGuard(self.cfg['control'], 1/15)
            with self.assertRaisesRegex(ValueError, 'too far'):
                guard.prepare(target, self.pose, initial=initial)
            guard.prepare(target, self.pose, initial=not initial)
        self.cfg['control']['check_runtime_target_error'] = 'false'
        with self.assertRaisesRegex(ValueError, 'YAML boolean'):
            ActionGuard(self.cfg['control'], 1/15)

    def test_legacy_configs_keep_model_checks_enabled(self):
        for key in ('check_initial_target_error', 'check_runtime_target_error', 'check_gripper_tracking_error'):
            del self.cfg['control'][key]
        guard = ActionGuard(self.cfg['control'], 1/15)
        self.assertTrue(guard.check_initial_target_error)
        self.assertTrue(guard.check_runtime_target_error)
        self.assertTrue(guard.check_gripper_tracking_error)

    def test_gripper_tracking_stop_disabled_both_sides_but_joint_stop_retained(self):
        self.cfg['control'].update(check_gripper_tracking_error=False, check_runtime_target_error=False)
        guard = ActionGuard(self.cfg['control'], 1/15)
        actual, previous, target = self.pose.copy(), self.pose.copy(), self.pose.copy()
        actual[[6, 13]] = .0546
        previous[[6, 13]] = .0344
        target[[6, 13]] = 0
        guard.commit(previous)
        sent = guard.prepare(target, actual)
        np.testing.assert_allclose(sent[[6, 13]], [.0346, .0346])
        # A stalled grasp must not accumulate an ever farther closing command.
        for _ in range(100):
            guard.commit(sent)
            sent = guard.prepare(target, actual)
        np.testing.assert_allclose(sent[[6, 13]], [.0346, .0346])
        previous[5] = .25
        guard.commit(previous)
        with self.assertRaisesRegex(ValueError, 'left_joint_6'):
            guard.prepare(target, actual)

    def test_gripper_feedback_jump_does_not_cause_command_jump(self):
        self.cfg['control'].update(check_gripper_tracking_error=False, check_runtime_target_error=False)
        guard = ActionGuard(self.cfg['control'], 1/15)
        previous, actual, target = self.pose.copy(), self.pose.copy(), self.pose.copy()
        previous[[6, 13]] = 0
        actual[[6, 13]] = .10
        target[[6, 13]] = 0
        for _ in range(50):
            guard.commit(previous)
            sent = guard.prepare(target, actual)
            self.assertTrue(np.all(np.abs(sent-previous) <= guard.speed*guard.period + 1e-10))
            self.assertTrue(np.all(sent >= guard.low))
            self.assertTrue(np.all(sent <= guard.high))
            previous = sent
        np.testing.assert_allclose(sent[[6, 13]], [.08, .08])
        self.cfg['control']['check_gripper_tracking_error'] = 'false'
        with self.assertRaisesRegex(ValueError, 'YAML boolean'):
            ActionGuard(self.cfg['control'], 1/15)

    def test_name_order_and_missing_joints(self):
        msg = JointState(name=['b', 'a'], position=[2, 1])
        np.testing.assert_equal(joint_values(msg, ['a', 'b']), [1, 2])
        with self.assertRaises(KeyError):
            joint_values(msg, ['c'])
        msg.name = ['a', 'a']
        with self.assertRaises(ValueError):
            joint_values(msg, ['a'])

    def test_rgb_bgr_padding(self):
        msg = Image(height=1, width=2, encoding='bgr8', step=8,
                    data=bytes([1, 2, 3, 4, 5, 6, 99, 99]))
        np.testing.assert_equal(rgb_array(msg, 1, 2), [[[3, 2, 1], [6, 5, 4]]])
        msg.encoding = '16UC1'
        with self.assertRaises(ValueError):
            rgb_array(msg, 1, 2)

    def test_guard_units_velocity_and_large_jump(self):
        self.cfg['control'].update(max_joint_speed_rad_s=.30, max_gripper_speed_m_s=.02)
        guard = ActionGuard(self.cfg['control'], 1/15)
        target = self.pose.copy()
        target[0] += .1
        target[6] += .01
        sent = guard.prepare(target, self.pose)
        self.assertAlmostEqual(sent[0], .02)
        self.assertAlmostEqual(sent[6], .04+.02/15)
        guard.commit(sent)
        self.assertAlmostEqual(guard.prepare(target, self.pose)[0], .04)
        target[0] = 1
        with self.assertRaises(ValueError):
            guard.prepare(target, self.pose)
        target[0] = float('nan')
        with self.assertRaises(ValueError):
            guard.prepare(target, self.pose)

    def test_limits_tolerance_and_not_silent_large_clipping(self):
        # Test a fixed tolerance, independent of operator edits to act.yaml.
        self.cfg['control']['boundary_tolerance_m'] = .002
        guard = ActionGuard(self.cfg['control'], 1/15)
        pose = self.pose.copy()
        pose[6] = -.001
        self.assertEqual(guard.bounded(pose)[6], 0)
        pose[6] = -.01
        with self.assertRaises(ValueError):
            guard.bounded(pose)

    def test_runtime_allowance_does_not_relax_startup_or_tracking(self):
        guard = ActionGuard(self.cfg['control'], 1/15)
        target = self.pose.copy()
        target[5] += .3
        sent = guard.prepare(target, self.pose)
        self.assertAlmostEqual(sent[5], .04)
        with self.assertRaisesRegex(ValueError, 'Initial model target'):
            guard.prepare(target, self.pose, initial=True)
        previous = self.pose.copy()
        previous[5] += .25
        guard.commit(previous)
        with self.assertRaisesRegex(ValueError, 'not tracking'):
            guard.prepare(target, self.pose)

    def test_reported_j6_stop_passes_with_bounded_next_command(self):
        guard = ActionGuard(self.cfg['control'], 1/15)
        actual = self.pose.copy()
        actual[5] = .1128
        target = actual.copy()
        target[5] = .3495
        previous = actual.copy()
        previous[5] = .1438950306876574
        guard.commit(previous)
        result = guard.prepare(target, actual)
        self.assertAlmostEqual(result[5], .1838950306876574)

    def test_sent_target_stays_within_tracking_envelope(self):
        guard = ActionGuard(self.cfg['control'], 1/15)
        previous = self.pose.copy()
        previous[5] += .19
        guard.commit(previous)
        target = self.pose.copy()
        target[5] += .4
        sent = guard.prepare(target, self.pose)
        self.assertAlmostEqual(sent[5], .2)

    def test_regression_endpoint_projection_after_raw_error_check(self):
        guard = ActionGuard(self.cfg['control'], 1/15)
        actual = self.pose.copy()
        actual[1] = .005
        target = actual.copy()
        target[1] = -.08
        self.assertEqual(guard.prepare(target, actual)[1], 0)
        target[1] = -10
        with self.assertRaises(ValueError):
            guard.prepare(target, actual)

    def test_packet_roundtrip_and_disconnect(self):
        a, b = socket.socketpair()
        try:
            send_packet(a, state=self.pose, seq=np.array([7]))
            packet = recv_packet(b)
            np.testing.assert_equal(packet['state'], self.pose)
            self.assertEqual(packet['seq'][0], 7)
            a.close()
            with self.assertRaises(ConnectionError):
                recv_packet(b)
        finally:
            a.close()
            b.close()

    def test_can_modes_do_not_weaken_record_mode(self):
        raw = '1: can0: <NOARP,UP,LOWER_UP> mtu 16\n can state ERROR-ACTIVE bitrate 1000000'
        with patch('hardware_bringup.config.subprocess.check_output', return_value=raw):
            check_can('can0', 1000000, 'deploy')
            with self.assertRaises(RuntimeError):
                check_can('can0', 1000000)
        with patch('hardware_bringup.config.subprocess.check_output', return_value=raw+' LISTEN-ONLY'):
            check_can('can0', 1000000)
            with self.assertRaises(RuntimeError):
                check_can('can0', 1000000, 'deploy')

    def test_observation_alignment_and_staleness(self):
        with patch('rospy.Subscriber'):
            obs = Observations(self.cfg)
        now = time.time()-.03
        for cam in ('cam_h', 'cam_l', 'cam_r'):
            obs.put(cam, now, np.zeros((480, 640, 3), np.uint8))
        for s in ('left', 'right'):
            obs.put(s+'_j', now-.005, self.pose[:6])
            obs.put(s+'_g', now-.005, self.pose[6:7])
            obs.put(s+'_status', now, {})
            # Future state must NOT enter the camera-aligned observation.
            obs.put(s+'_j', now+.01, self.pose[:6]+.01)
        packet, anchor, stamps = obs.snapshot()
        np.testing.assert_equal(packet['state'], self.pose.astype(np.float32))
        self.assertLessEqual(stamps['left_j'], anchor)
        # One new camera callback must not invalidate the previous complete set.
        obs.put('cam_h', now+.06, np.zeros((480, 640, 3), np.uint8))
        self.assertEqual(obs.snapshot()[1], anchor)
        with patch('model_deploy.observations.time.time', return_value=now+1):
            with self.assertRaises(RuntimeError):
                obs.snapshot()

    def controller(self):
        obs = Mock()
        obs.actual.return_value = self.pose.copy()
        controller = Controller(self.cfg, obs)
        controller.arms = {s: Mock() for s in ('left', 'right')}
        controller.grippers = {s: Mock() for s in ('left', 'right')}
        for arm in controller.arms.values():
            arm.has_comm_error.return_value = False
        controller.active = True
        return controller

    def test_stop_holds_measured_pose_without_disable_reset(self):
        c = self.controller()
        c.stop('test')
        for i, side in enumerate(('left', 'right')):
            c.arms[side].move_j.assert_called_once_with(self.pose[i*7:i*7+6].tolist())
            c.grippers[side].move_gripper_m.assert_called_once_with(.04, 1.0)
            c.arms[side].disable.assert_not_called()
            c.arms[side].reset.assert_not_called()
        with self.assertRaises(RuntimeError):
            c.send(self.pose)

    def test_stale_feedback_uses_stop_not_stale_target(self):
        c = self.controller()
        c.obs.actual.side_effect = RuntimeError('stale')
        c.stop('test')
        for side in ('left', 'right'):
            c.arms[side].electronic_emergency_stop.assert_called_once()
            c.arms[side].move_j.assert_not_called()
            c.grippers[side].move_gripper_m.assert_not_called()

    def test_watchdog_works_without_inference_response(self):
        c = self.controller()
        c.last_send = time.monotonic()-1
        t = threading.Thread(target=c.watchdog)
        t.start()
        t.join(timeout=1)
        self.assertFalse(t.is_alive())
        self.assertFalse(c.active)
        self.assertIn('watchdog', c.fault)

    def test_external_master_prevents_commands(self):
        c = self.controller()
        c.external = 'another master'
        with self.assertRaises(RuntimeError):
            c.send(self.pose)
        for arm in c.arms.values():
            arm.move_j.assert_not_called()

    def test_firmware_waits_for_rx_and_retries_without_motion(self):
        c = self.controller()
        robot = c.arms['left']
        clock = [0.0]
        robot.get_fps.side_effect = lambda: 100 if clock[0] >= .12 else 0
        robot.get_joint_angles.side_effect = lambda: SimpleNamespace(timestamp=time.time())
        expected = {'software_version': 'S-V1.9-0'}
        robot.get_firmware.side_effect = [None, expected]
        def advance(dt):
            clock[0] += dt
        with patch('model_deploy.controller.time.monotonic', side_effect=lambda: clock[0]), \
             patch('model_deploy.controller.time.sleep', side_effect=advance):
            self.assertEqual(c.read_firmware(robot, 'left'), expected)
        self.assertGreaterEqual(clock[0], .52)
        self.assertEqual(robot.get_firmware.call_count, 2)
        robot.enable.assert_not_called()
        robot.move_j.assert_not_called()

    def test_firmware_no_feedback_exits_without_query_or_motion(self):
        c = self.controller()
        robot = c.arms['left']
        robot.get_fps.return_value = 0
        robot.get_joint_angles.return_value = None
        clock = [0.0]
        def advance(dt):
            clock[0] += dt
        with patch('model_deploy.controller.time.monotonic', side_effect=lambda: clock[0]), \
             patch('model_deploy.controller.time.sleep', side_effect=advance):
            with self.assertRaisesRegex(RuntimeError, 'SDK receive not ready'):
                c.read_firmware(robot, 'left')
        robot.get_firmware.assert_not_called()
        robot.enable.assert_not_called()

    def test_firmware_retries_are_bounded(self):
        c = self.controller()
        robot = c.arms['left']
        robot.get_fps.return_value = 100
        robot.get_joint_angles.side_effect = lambda: SimpleNamespace(timestamp=time.time())
        robot.get_firmware.return_value = None
        clock = [0.0]
        def advance(dt):
            clock[0] += dt
        with patch('model_deploy.controller.time.monotonic', side_effect=lambda: clock[0]), \
             patch('model_deploy.controller.time.sleep', side_effect=advance):
            with self.assertRaisesRegex(RuntimeError, 'after 3 attempts'):
                c.read_firmware(robot, 'left')
        self.assertEqual(robot.get_firmware.call_count, 3)
        robot.enable.assert_not_called()

    def test_sdk_uses_confirmed_joint_limits_without_connecting(self):
        from pyAgxArm import create_agx_arm_config, AgxArmFactory
        c = self.cfg['control']
        limits = {'joint'+str(i+1): [c['joint_min'][i], c['joint_max'][i]] for i in range(6)}
        arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version='default', channel='can0', joint_limits=limits))
        self.assertEqual(arm.get_config()['joint_limits']['joint6'], [-np.pi, np.pi])
        arm._send_msg = Mock()  # intercept message objects; no connect, no CAN
        arm.move_j([0, .5, -.5, 0, 0, 2.7])
        self.assertTrue(arm._send_msg.called)
        self.assertEqual(arm._send_msg.call_args_list[-1].args[0].joint_6, round(2.7*180/np.pi*1000))

    def test_partial_send_failure_stops_both_sides(self):
        c = self.controller()
        c.arms['right'].move_j.side_effect = RuntimeError('disconnected')
        with self.assertRaises(RuntimeError):
            c.send(self.pose)
        c.stop('send failed')
        self.assertEqual(c.arms['left'].move_j.call_count, 2)
        c.arms['right'].electronic_emergency_stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
