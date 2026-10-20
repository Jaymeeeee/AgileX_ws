import copy
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np

from model_deploy.reset_sequence import ResetSequence
from model_deploy.controller import Controller
from model_deploy.teleop_core import load_teleop
from model_deploy.teleop_device import TeleopDevice
from model_deploy.teleop_reset import EndpointReset, reset_collection

PACKAGE = Path(__file__).resolve().parents[1]
WS = PACKAGE.parents[1]


def script(name):
    spec = importlib.util.spec_from_file_location(name, PACKAGE/'scripts'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CollectionResetTests(unittest.TestCase):
    def test_recording_finalized_before_pause_and_both_followers_before_leaders(self):
        events = []
        reset_collection(lambda *args: events.append(args), lambda: events.append('saved'))
        self.assertEqual(events, ['saved', ('left','follower','follow',False),
            ('right','follower','follow',False), ('left','leader','hold'), ('right','leader','hold'),
            ('left','follower','reset'), ('right','follower','reset'),
            ('left','leader','align_hold'), ('right','leader','align_hold')])

    def test_failed_save_prevents_all_arm_operations(self):
        call = Mock()
        with self.assertRaisesRegex(RuntimeError, 'disk'):
            reset_collection(call, Mock(side_effect=RuntimeError('disk')))
        call.assert_not_called()

    def test_failed_follower_reset_does_not_start_next_arm_or_resume(self):
        events = []
        def call(*args):
            events.append(args)
            if args == ('left','follower','reset'):
                raise RuntimeError('blocked')
        with self.assertRaisesRegex(RuntimeError,'blocked'):
            reset_collection(call, lambda: None)
        self.assertEqual(events[-1], ('left','follower','reset'))

    def device(self, side):
        cfg = load_teleop(PACKAGE/'config/teleop.yaml')
        cfg['deployment']['reset'].pop('vertical_lift')  # Full lift has separate planner tests.
        cfg['deployment']['reset']['home_joints_rad']['right'][0] = .12
        device = TeleopDevice(cfg, side, 'follower')
        device.enabled, device.phase = True, 'resetting'
        device.robot, device.gripper = Mock(), Mock()
        device.robot.has_comm_error.return_value = False
        actual = np.array([.1,.3,-.3,0,.3,0,.02])
        device.actual = lambda: actual.copy()
        device.robot.move_j.side_effect = lambda q: actual.__setitem__(slice(0,6), q)
        device.gripper.move_gripper_m.side_effect = lambda width, force: actual.__setitem__(6, width)
        return device

    def test_existing_sequence_uses_own_side_home_and_restores_teleop_guard(self):
        for side in ('left','right'):
            device = self.device(side)
            original_guard = device.guard
            cfg_before = copy.deepcopy(device.cfg)
            reset = EndpointReset(device)
            clock = [0.]
            def runner(*args, **kwargs):
                return ResetSequence(*args, **kwargs, clock=lambda: clock[0],
                    sleep=lambda dt: clock.__setitem__(0,clock[0]+dt))
            events = []
            with patch('model_deploy.teleop_reset.ResetSequence', side_effect=runner):
                reset.run(lambda: False, events.append)
            self.assertEqual([e['phase'] for e in events if e['state']=='phase_complete'],
                             ['opening_grippers','returning_home','closing_grippers'])
            np.testing.assert_allclose(device.actual()[:6], reset.cfg['reset']['home_joints_rad'][side], atol=.001)
            self.assertLess(device.actual()[6],.001)
            self.assertIs(device.guard, original_guard)
            self.assertEqual(device.cfg,cfg_before)
            device.robot.enable.assert_not_called()
            self.assertTrue(all(len(c.args[0])==6 for c in device.robot.move_j.call_args_list))

    def test_single_arm_adapter_rejects_different_pair_targets(self):
        device = self.device('left')
        reset = EndpointReset(device)
        goal = reset.actual();goal[7]+=.1
        with self.assertRaises(ValueError):
            reset.send(goal)
        device.robot.move_j.assert_not_called()

    def node(self, role='follower'):
        module = script('piper_teleop_node.py')
        node = module.Node.__new__(module.Node)
        node.device = self.device('left')
        node.cfg, node.side, node.role = node.device.cfg, 'left', role
        node.execute, node.follow, node.service_busy, node.service_holding = True, False, False, False
        node.lock = threading.RLock()
        return node

    def test_observe_and_following_refuse_reset_without_movement(self):
        node = self.node()
        for execute, follow in ((False,False),(True,True)):
            node.execute, node.follow = execute, follow
            self.assertFalse(node.reset(None).success)
        node.device.robot.move_j.assert_not_called()

    def test_reset_failure_latches_stop_and_does_not_return_to_following(self):
        node = self.node()
        node.device.stop = Mock()
        with patch('model_deploy.teleop_reset.EndpointReset') as reset:
            reset.return_value.cfg = node.cfg['deployment']
            reset.return_value.run.side_effect = RuntimeError('blocked')
            self.assertFalse(node.reset(None).success)
        node.device.stop.assert_called_once()
        self.assertFalse(node.follow)
        self.assertFalse(node.service_busy)

    def test_recorder_save_failure_propagates_to_console(self):
        console = script('teleop_console.py')
        master = (1,'', [[],[],[('/agilex/recording/stop',['recorder'])]])
        with patch.object(console.rospy,'get_master') as rpc, patch.object(console.rospy,'ServiceProxy') as proxy:
            rpc.return_value.getSystemState.return_value = master
            proxy.return_value.return_value = Mock(success=False,message='disk failure')
            with self.assertRaisesRegex(RuntimeError,'reset cancelled'):
                console.stop_recording()

    def test_deploy_can_preparation_does_not_access_leaders(self):
        # Fake ip always reports UP. Missing leader interfaces fail if accessed.
        with tempfile.TemporaryDirectory() as tmp:
            ip = Path(tmp)/'ip'
            ip.write_text('#!/bin/bash\ncase "$*" in\n*can3|*can0) echo "1: can: <UP>"; echo "bitrate 1000000";;\n*) exit 99;;\nesac\n')
            ip.chmod(0o755)
            env = dict(os.environ, PATH=tmp+':'+os.environ['PATH'])
            result = subprocess.run(['bash',str(WS/'prepare_can.sh'),'--mode=deploy'],
                                    env=env,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('can3',result.stdout)
            self.assertIn('can0',result.stdout)
            self.assertNotIn('can1',result.stdout)
            self.assertNotIn('can2',result.stdout)

    def test_deploy_controller_connects_only_followers_with_leaders_configured(self):
        cfg = load_teleop(PACKAGE/'config/teleop.yaml')
        obs = Mock();obs.hardware_config = cfg['hardware']
        controller = Controller(cfg['deployment'], obs)
        controller.read_firmware = Mock(return_value={'software_version':'S-V1.9-0'})
        robot = Mock();robot.has_comm_error.return_value = False
        with patch('model_deploy.controller.check_identity') as identity, \
             patch('model_deploy.controller.check_can') as check_can, \
             patch('model_deploy.controller.can.Bus') as bus, \
             patch('model_deploy.controller.threading.Thread'), \
             patch('model_deploy.controller.time.sleep'), \
             patch('model_deploy.controller.fcntl.flock'), patch('builtins.open'), \
             patch('pyAgxArm.create_agx_arm_config',side_effect=lambda **kw:kw) as sdk_config, \
             patch('pyAgxArm.resolve_firmware_profile',return_value='default'), \
             patch('pyAgxArm.AgxArmFactory.create_arm',return_value=robot):
            controller.connect()
            self.assertEqual([c.args[0]['channel'] for c in identity.call_args_list],['can3','can0'])
            self.assertEqual([c.args[0] for c in check_can.call_args_list],['can3','can0'])
            self.assertEqual([c.kwargs['channel'] for c in bus.call_args_list],['can3','can0'])
            self.assertEqual([c.kwargs['channel'] for c in sdk_config.call_args_list],['can3','can0'])
            controller.close()


if __name__ == '__main__':
    unittest.main()
