import copy
import importlib.util
import time
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch, mock_open

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from hardware_bringup.config import acquisition_can_mode, check_identity
from model_deploy.teleop_core import CommandInbox, GripperTakeover, command_values, load_teleop, names
from model_deploy.teleop_device import TeleopDevice


CONFIG = Path(__file__).resolve().parents[1]/'config/teleop.yaml'


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_teleop(CONFIG)
        self.pose = np.array([0., .3, -.3, 0., .3, 0., .02])

    def message(self, side='left', stamp=None):
        msg = JointState()
        msg.name = names(side, 'leader_target')
        msg.position = self.pose.tolist()
        msg.header.stamp = rospy.Time.from_sec(stamp or time.time())
        return msg

    def status(self):
        return dict(side='left', role='leader', phase='teaching', can_age_sec=.01)

    def test_confirmed_four_way_mapping_and_record_ack(self):
        h = self.cfg['hardware']
        self.assertEqual({s: a['channel'] for s, a in h['arms'].items()}, {'left': 'can3', 'right': 'can0'})
        self.assertEqual({s: a['channel'] for s, a in h['leaders'].items()}, {'left': 'can1', 'right': 'can2'})
        self.assertEqual(acquisition_can_mode(h, 'record'), 'deploy')
        self.assertEqual(acquisition_can_mode({}, 'record'), 'record')

    def test_wrong_usb_path_refuses_connection(self):
        with patch('hardware_bringup.config.Path.exists', return_value=True), \
             patch('hardware_bringup.config.Path.resolve', return_value=Path('/usb/wrong')):
            with self.assertRaisesRegex(RuntimeError, 'USB identity mismatch'):
                check_identity(self.cfg['hardware']['leaders']['left'])

    def test_wrong_side_order_and_nonfinite_input_rejected(self):
        with self.assertRaises(ValueError):
            command_values(self.message('right'), 'left')
        msg = self.message(); msg.name.reverse()
        with self.assertRaises(ValueError):
            command_values(msg, 'left')
        msg = self.message(); msg.position[0] = float('nan')
        with self.assertRaises(ValueError):
            command_values(msg, 'left')

    def test_static_target_requires_fresh_independent_hardware_health(self):
        inbox = CommandInbox('left', 1)
        msg = self.message(stamp=time.time()-30)
        inbox.put_command(msg, '/left'); inbox.put_source(self.status(), '/left')
        np.testing.assert_array_equal(inbox.latest(), self.pose)
        # Old source stamps remain old; only genuine source health allows hold.
        self.assertLess(inbox.command[1], time.time()-20)
        inbox.source = (inbox.source[0], time.monotonic()-2, '/left')
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            inbox.latest()

    def test_ros_heartbeat_does_not_hide_dead_can_or_wrong_mode(self):
        inbox = CommandInbox('left', 1)
        inbox.put_command(self.message(), '/left')
        status = self.status(); status['can_age_sec'] = 2
        inbox.put_source(status, '/left')
        with self.assertRaises(RuntimeError):
            inbox.latest()
        status = self.status(); status['phase'] = 'aligning'
        inbox.put_source(status, '/left')
        with self.assertRaises(RuntimeError):
            inbox.latest()

    def test_reordered_time_and_different_publishers_rejected(self):
        inbox = CommandInbox('left', 1)
        inbox.put_command(self.message(), '/one'); inbox.put_source(self.status(), '/two')
        with self.assertRaisesRegex(RuntimeError, 'different publishers'):
            inbox.latest()
        with self.assertRaisesRegex(ValueError, 'backwards'):
            inbox.put_command(self.message(stamp=time.time()-10), '/one')

    def device(self, role='follower'):
        device = TeleopDevice(self.cfg, 'left', role)
        device.robot, device.gripper = Mock(), Mock()
        device.robot.has_comm_error.return_value = False
        device.robot.get_joints_enable_status_list.return_value = [True]*6
        device.actual = Mock(return_value=self.pose.copy())
        return device

    def test_hold_target_precedes_motor_enable_and_follow_is_bounded(self):
        device = self.device()
        calls = []
        device.robot.move_j.side_effect = lambda x: calls.append(('move', x))
        device.robot.enable.side_effect = lambda: calls.append(('enable',))
        device.enable_hold()
        self.assertEqual(calls[0], ('move', self.pose[:6].tolist()))
        self.assertEqual(calls[1], ('enable',))
        target = self.pose.copy(); target[0] += 1; target[6] += .05
        sent = device.send(target)
        self.assertLessEqual(abs(sent[0]-self.pose[0]), self.cfg['joint_speed_rad_s']/self.cfg['fps']+1e-9)
        self.assertLessEqual(abs(sent[6]-self.pose[6]), self.cfg['gripper_speed_m_s']/self.cfg['fps']+1e-9)

    def test_drag_stop_does_not_command_old_pose(self):
        device = self.device('leader'); device.enabled = True; device.phase = 'teaching'
        device.stop('test')
        device.robot.electronic_emergency_stop.assert_called_once()
        device.robot.move_j.assert_not_called()
        device.robot.disable.assert_not_called()

    def test_fast_follow_removes_slew_gate_but_preserves_physical_bounds(self):
        device = self.device(); device.enable_hold()
        target = self.pose.copy(); target[0] += .7; target[6] = .08
        sent = device.send(target, following=True)
        np.testing.assert_array_equal(sent, target)
        self.assertGreater(abs(sent[0]-self.pose[0]), self.cfg['joint_speed_rad_s']/self.cfg['fps'])
        target[0] = 100; target[6] = -.03
        sent = device.send(target, following=True)
        self.assertEqual(sent[0], device.guard.high[0])
        self.assertEqual(sent[6], 0)
        device.actual.side_effect = RuntimeError('stale')
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            device.send(target, following=True)

    def test_fast_follow_can_be_disabled_without_changing_alignment(self):
        device = self.device(); device.enable_hold()
        device.cfg = copy.deepcopy(device.cfg); device.cfg['limit_follow_speed'] = True
        target = self.pose.copy(); target[0] += .7
        sent = device.send(target, following=True)
        self.assertLessEqual(sent[0]-self.pose[0], self.cfg['joint_speed_rad_s']/self.cfg['fps']+1e-9)

    def test_record_readiness_requires_following_and_live_can(self):
        import json
        from std_msgs.msg import String
        from hardware_bringup.monitor import summarize
        spec = dict(kind='teleop_status',side='left',role='follower',phase='following')
        data = dict(side='left',role='follower',phase='following',fault=None,can_age_sec=.01,gripper_ready=True)
        summarize(String(data=json.dumps(data)),spec)
        for changes in ({'phase':'holding'},{'can_age_sec':2},{'fault':'lost source'},{'side':'right'},{'gripper_ready':False}):
            with self.assertRaises(ValueError):
                summarize(String(data=json.dumps(dict(data,**changes))),spec)

    def test_aligned_gripper_seed_cannot_arm_a_later_zero_command(self):
        takeover = GripperTakeover(.1,.005)
        self.assertEqual(takeover.update(.1,native=False), .1)
        self.assertFalse(takeover.ready)
        self.assertEqual(takeover.update(0,native=True), .1)
        self.assertFalse(takeover.ready)
        self.assertEqual(takeover.update(.05,native=True), .1)
        self.assertEqual(takeover.update(.099,native=True), .099)
        self.assertTrue(takeover.ready)
        self.assertEqual(takeover.update(.02,native=True), .02)

    def test_gripper_pickup_accepts_crossing_and_closed_initial_pose(self):
        takeover = GripperTakeover(.04,.001)
        self.assertEqual(takeover.update(.02,True), .04)
        self.assertEqual(takeover.update(.06,True), .06)
        self.assertTrue(takeover.ready)
        closed = GripperTakeover(0,.005)
        self.assertEqual(closed.update(0,True),0)
        self.assertTrue(closed.ready)

    def test_seed_provenance_and_unchanged_compat_targets_are_not_republished(self):
        spec = importlib.util.spec_from_file_location('teleop_node_test',CONFIG.parents[1]/'scripts/piper_teleop_node.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        node = module.Node.__new__(module.Node)
        node.role, node.side, node.follow = 'leader','left',False
        node.lock, node.service_busy = threading.RLock(), False
        node.gripper_takeover = None
        node.device = self.device('leader')
        node.device.phase = 'teaching'; node.device.last_rx = time.monotonic()
        node.device.events = {k:dict(stamp=time.time()) for k in ('joint_states','gripper/actual_stamped')}
        node.seed, node.seed_stamp = self.pose.copy(), time.time()
        node.compat_stamps = [None,None]
        node.status_pub, node.state_pub, node.command_pub = Mock(),Mock(),Mock()
        node.compat = [Mock(),Mock()]
        node.publish(); node.publish()
        self.assertEqual(node.command_pub.publish.call_count,2)
        self.assertEqual(node.command_pub.publish.call_args[0][0].header.frame_id,'aligned_gripper_seed')
        for pub in node.compat:
            self.assertEqual(pub.publish.call_count,1)
        node.device.events['gripper/target_stamped'] = dict(position=[.04],stamp=node.seed_stamp+.01,set_zero=0)
        node.publish()
        self.assertEqual(node.command_pub.publish.call_args[0][0].header.frame_id,'native_gripper')
        self.assertEqual(node.compat[0].publish.call_count,1)
        self.assertEqual(node.compat[1].publish.call_count,2)

    def test_stale_feedback_stop_uses_damping_and_fault_is_latched(self):
        device = self.device(); device.enabled = True; device.phase = 'holding'
        device.actual.side_effect = RuntimeError('stale')
        device.stop('lost feedback')
        device.robot.electronic_emergency_stop.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, 'lost feedback'):
            device.send(self.pose)

    def test_fresh_gripper_fault_is_rejected_before_fast_follow(self):
        device = TeleopDevice(self.cfg,'left','follower')
        stamp = time.time()
        device.events = {
            'joint_states':dict(stamp=stamp,position=self.pose[:6].tolist()),
            'gripper/actual_stamped':dict(stamp=stamp,position=[.02],status_code=1),
            'status':dict(stamp=stamp,err_code=0,arm_status=0)}
        with self.assertRaisesRegex(RuntimeError,'Gripper reports'):
            device.actual()

    def test_previous_sdk_echo_is_not_replayed_as_human_input(self):
        device = self.device('leader'); seed_stamp = time.time()
        device.events['leader_targets'] = dict(position=[1]*6, stamp=seed_stamp-1)
        out, stamp, _ = device.leader_target(self.pose, seed_stamp)
        np.testing.assert_array_equal(out, self.pose)
        self.assertEqual(stamp, seed_stamp)
        device.events['leader_targets'] = dict(position=[.1]*6, stamp=seed_stamp+.01)
        out, stamp, _ = device.leader_target(self.pose, seed_stamp)
        np.testing.assert_allclose(out[:6], [.1]*6)

    def test_alignment_cancel_or_out_of_limits_sends_no_target(self):
        device = self.device('leader'); device.send = Mock()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            device.align(self.pose, cancelled=lambda: True)
        device.send.assert_not_called()
        bad = self.pose.copy(); bad[0] = 10
        with self.assertRaises(ValueError):
            device.align(bad)
        device.send.assert_not_called()

    def test_observation_mode_never_constructs_sdk_or_sends_frames(self):
        device = TeleopDevice(self.cfg, 'left', 'leader')
        with patch('model_deploy.teleop_device.check_can'), patch('model_deploy.teleop_device.check_identity'), \
             patch('model_deploy.teleop_device.can.Bus') as bus, patch('builtins.open', mock_open()), \
             patch('model_deploy.teleop_device.fcntl.flock'), patch('model_deploy.teleop_device.threading.Thread'), \
             patch('pyAgxArm.AgxArmFactory.create_arm') as factory:
            device.connect(configure=False)
            device.close()
            factory.assert_not_called()
            bus.return_value.send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
