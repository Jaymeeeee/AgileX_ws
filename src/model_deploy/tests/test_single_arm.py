"""Single-side operation without power, CAN traffic, cameras, or model weights."""
import importlib.util
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from model_deploy.core import ActionGuard, action_names, arm_sides, load_config, selected_sides
from model_deploy.controller import Controller
from model_deploy.observations import Observations
from model_deploy.reset_joint_lift import plan_joint_lift
from model_deploy.reset_pose import load_reset_pose, load_start_pose, save_start_pose
from model_deploy.reset_sequence import ResetSequence
from model_deploy.replay import replay_config
from model_deploy.teleop_core import load_teleop
from model_deploy.teleop_device import TeleopDevice
from model_deploy.teleop_reset import reset_collection

PACKAGE = Path(__file__).resolve().parents[1]


def script(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE/'scripts'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SingleArmTests(unittest.TestCase):
    def cfg(self, side):
        return load_config(PACKAGE/'config/act.yaml', arms=side)

    def pose(self):
        return np.array([0., .5, -.5, 0., .4, 0., .04])

    def test_default_both_and_explicit_selection(self):
        self.assertEqual(selected_sides(load_config(PACKAGE/'config/act.yaml')), ('left', 'right'))
        for side in ('left', 'right'):
            cfg = self.cfg(side)
            self.assertEqual(selected_sides(cfg), (side,))
            for mapping in (cfg['observations']['arms'], cfg['control']['channels'], cfg['control']['expected_firmware']):
                self.assertEqual(list(mapping), [side])
            with self.assertRaisesRegex(ValueError, 'replay currently requires both arms'):
                replay_config(cfg, PACKAGE/'config/replay.yaml')
        for value in ('none', '', ['left', 'left'], []):
            with self.assertRaises(ValueError):
                arm_sides(value)

    def test_single_guard_rejects_dual_outputs_and_preserves_limits(self):
        for side in ('left', 'right'):
            cfg = self.cfg(side)
            guard = ActionGuard(cfg['control'], 1/15, selected_sides(cfg))
            self.assertEqual(guard.names, action_names((side,)))
            sent = guard.prepare(np.full(7, 100.), self.pose())
            self.assertEqual(sent.shape, (7,))
            self.assertTrue(np.all(sent <= guard.high))
            self.assertTrue(np.all(np.abs(sent-self.pose()) <= guard.speed*guard.period+1e-10))
            with self.assertRaisesRegex(ValueError, 'finite 7D'):
                guard.prepare(np.zeros(14), self.pose())
            bad = self.pose(); bad[0] = 10
            with self.assertRaisesRegex(ValueError, side+'_joint_1'):
                guard.bounded(bad)

    def test_observation_subscriptions_and_snapshot_need_only_selected_arm(self):
        for side in ('left', 'right'):
            cfg = self.cfg(side)
            other = 'right' if side == 'left' else 'left'
            with patch('rospy.Subscriber') as subscribe:
                obs = Observations(cfg)
            topics = [call.args[0] for call in subscribe.call_args_list]
            self.assertFalse(any('/piper/'+other+'_' in topic for topic in topics))
            now = time.time()-.03
            for camera in ('cam_h', 'cam_l', 'cam_r'):
                obs.put(camera, now, np.zeros((480, 640, 3), np.uint8))
            obs.put(side+'_j', now-.005, self.pose()[:6])
            obs.put(side+'_g', now-.005, self.pose()[6:])
            obs.put(side+'_status', now, {})
            np.testing.assert_allclose(obs.actual(), self.pose())
            packet, _, stamps = obs.snapshot()
            np.testing.assert_allclose(packet['state'], self.pose())
            self.assertFalse(any(key.startswith(other+'_') for key in stamps))
            obs.hardware_mode = 'deploy'
            obs.hardware_received = time.monotonic()
            obs.hardware_config = {'arms': {side: {'channel': cfg['control']['channels'][side], 'model': 'piper'}}}
            obs.deployment_hardware()

    def test_controller_send_stop_and_fk_only_touch_selected_side(self):
        for side in ('left', 'right'):
            cfg = self.cfg(side)
            obs = Mock()
            obs.actual.return_value = self.pose()
            controller = Controller(cfg, obs)
            arm, gripper = Mock(), Mock()
            arm.has_comm_error.return_value = False
            arm.fk.return_value = [0]*6
            controller.arms, controller.grippers = {side: arm}, {side: gripper}
            controller.active = True
            controller.send(self.pose())
            arm.move_j.assert_called_once_with(self.pose()[:6].tolist())
            gripper.move_gripper_m.assert_called_once_with(.04, cfg['control']['gripper_force'])
            self.assertEqual(controller.lift_fk(np.zeros((1, 6))).shape, (1, 6))
            controller.stop()
            self.assertEqual(arm.move_j.call_count, 2)

    def test_checkpoint_contract_rejects_14d_and_accepts_selected_7d(self):
        worker = script('inference_worker.py')
        for side in ('left', 'right'):
            cfg = self.cfg(side)
            pc = SimpleNamespace(type='act', input_features={
                'observation.state': SimpleNamespace(shape=(7,)),
                **{'observation.images.'+k: SimpleNamespace(shape=(3, 480, 640)) for k in ('cam_h','cam_l','cam_r')}},
                output_features={'action': SimpleNamespace(shape=(7,))})
            self.assertEqual(worker.validate_checkpoint_contract(cfg, pc), 7)
            pc.input_features['observation.state'].shape = (14,)
            with self.assertRaisesRegex(ValueError, 'no automatic slicing'):
                worker.validate_checkpoint_contract(cfg, pc)
            pc.input_features['observation.state'].shape = (7,)
            pc.output_features['action'].shape = (14,)
            with self.assertRaisesRegex(ValueError, 'absolute 7D'):
                worker.validate_checkpoint_contract(cfg, pc)

    def test_selected_pose_roundtrip_and_cross_side_reset_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            for side in ('left', 'right'):
                cfg = self.cfg(side)
                cfg['log_dir'] = temp
                run = Path(temp)/side
                saved = save_start_pose(cfg, run, self.pose())
                self.assertEqual(saved['name'], action_names((side,)))
                actual, _ = load_start_pose(cfg, run)
                np.testing.assert_array_equal(actual, self.pose())
                other = self.cfg('right' if side == 'left' else 'left')
                with self.assertRaisesRegex(ValueError, 'joint order'):
                    load_start_pose(other, run)

    def test_reset_sequence_and_home_are_7d_and_keep_selected_pose(self):
        for side in ('left', 'right'):
            cfg = self.cfg(side)
            cfg['reset'].pop('vertical_lift')
            cfg['reset']['home_joints_rad'][side][4] = .25
            home, reference = load_reset_pose(cfg)
            self.assertEqual(home.shape, (7,))
            self.assertEqual(home[4], .25)
            self.assertEqual(reference['name'], action_names((side,)))
            state, clock, sent, phases = [self.pose()], [0.], [], []
            def send(target):
                self.assertEqual(target.shape, (7,))
                state[0] = target.copy(); sent.append(target.copy())
                return target
            def sleep(seconds):
                clock[0] += seconds
            obs = SimpleNamespace(actual=lambda: state[0])
            controller = SimpleNamespace(check_fault=lambda: None, send=send)
            runner = ResetSequence(cfg, controller, obs, clock=lambda: clock[0], sleep=sleep,
                                   report=lambda row: phases.append(row))
            runner.run(home)
            np.testing.assert_allclose(state[0], home, atol=.001)
            self.assertEqual([row['phase'] for row in phases if row['state'] == 'phase_complete'],
                             ['opening_grippers','returning_home','closing_grippers'])

    def test_single_joint_lift_planner_returns_one_arm_path(self):
        cfg = self.cfg('right')
        fk = lambda q: [q[0], q[2], .5+.1*q[1], 0., 0., 0.]
        joints = self.pose()[None, :6]
        plan = plan_joint_lift([fk], joints, np.array([fk(joints[0])]),
            np.array([cfg['control']['joint_min']]), np.array([cfg['control']['joint_max']]),
            cfg['reset']['vertical_lift'])
        self.assertEqual(plan['joints'].shape[1:], (1, 6))
        self.assertAlmostEqual(plan['poses'][-1, 0, 2]-plan['poses'][0, 0, 2], .1, places=3)

    def test_teleop_configuration_and_collection_reset_use_selected_pair(self):
        for side in ('left', 'right'):
            cfg = load_teleop(PACKAGE/'config/teleop.yaml', arms=side)
            self.assertEqual(selected_sides(cfg), (side,))
            with self.assertRaisesRegex(ValueError, 'outside the selected'):
                TeleopDevice(cfg, 'right' if side == 'left' else 'left', 'follower')
            events = []
            reset_collection(lambda *args: events.append(args), lambda: events.append('saved'), (side,))
            self.assertEqual(events, ['saved', (side,'follower','follow',False),
                (side,'leader','hold'), (side,'follower','reset'), (side,'leader','align_hold')])

    def test_teleop_configure_only_queries_selected_devices(self):
        console = script('teleop_console.py')
        for side in ('left', 'right'):
            with patch('sys.argv', ['teleop_console.py', '--configure-only', '--arms', side]), \
                 patch.object(console, 'check_can') as check_can, patch.object(console, 'check_identity'), \
                 patch('model_deploy.teleop_device.TeleopDevice') as device:
                console.main()
            self.assertEqual(check_can.call_count, 2)
            self.assertEqual([call.args[1:] for call in device.call_args_list], [(side, 'follower'), (side, 'leader')])


if __name__ == '__main__':
    unittest.main()
