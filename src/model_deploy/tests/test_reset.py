import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, mock_open, patch

import numpy as np

from model_deploy.controller import Controller
from model_deploy.core import load_config
from model_deploy.observations import Observations
from model_deploy.reset_pose import load_reset_pose, load_start_pose, save_start_pose
from model_deploy.reset_sequence import (
    GRIPPER_INDICES, JOINT_INDICES, ResetSequence, validate_reset_config,
)


class ResetTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(Path(__file__).resolve().parents[1]/'config/act.yaml')
        # Legacy joint-only tests; Cartesian lift is covered separately.
        self.cfg['reset'].pop('vertical_lift', None)
        self.pose = np.tile([.1, .5, -.5, .1, .1, .1, .04], 2)
        self.home = self.pose.copy()
        self.home[JOINT_INDICES] += .12
        self.clock = 0.0
        self.events, self.commands = [], []
        self.actual = self.pose.copy()
        self.obs = Mock()
        self.obs.actual.side_effect = lambda: self.actual.copy()
        self.controller = Mock()
        self.controller.send.side_effect = self.send
        self.follow = True

    def send(self, command):
        self.commands.append(command.copy())
        if self.follow:
            # Feedback approaches the sent target instead of immediately equaling it.
            self.actual += .6 * (command-self.actual)
        return command.copy()

    def sleep(self, duration):
        self.clock += duration

    def runner(self, **kw):
        return ResetSequence(self.cfg, self.controller, self.obs,
                             clock=lambda: self.clock, sleep=self.sleep,
                             report=self.events.append, **kw)

    def test_return_with_open_grippers_then_close_after_measured_arrival(self):
        result = self.runner().run(self.home)
        completed = [e for e in self.events if e['state'] == 'phase_complete']
        self.assertEqual([e['phase'] for e in completed],
                         ['opening_grippers', 'returning_home', 'closing_grippers'])
        for event, width, joints in zip(completed, [.10, .10, 0], [self.pose, self.home, self.home]):
            np.testing.assert_allclose(np.array(event['actual'])[GRIPPER_INDICES], width, atol=.003)
            np.testing.assert_allclose(np.array(event['actual'])[JOINT_INDICES], joints[JOINT_INDICES], atol=.02)
        np.testing.assert_allclose(self.actual[JOINT_INDICES], self.home[JOINT_INDICES], atol=.02)
        np.testing.assert_equal(result[GRIPPER_INDICES], [0, 0])
        delta = np.abs(np.diff(np.vstack([self.pose, *self.commands]), axis=0))
        self.assertLessEqual(delta[:, JOINT_INDICES].max(), .30/15 + 1e-10)
        self.assertLessEqual(delta[:, GRIPPER_INDICES].max(), .03/15 + 1e-10)
        # Caller-owned reference must not be rewritten to a closed gripper pose.
        np.testing.assert_equal(self.home[GRIPPER_INDICES], [.04, .04])

    def test_return_stall_does_not_close_grippers(self):
        self.cfg['reset']['phase_timeout_sec'] = 5.0
        def report(event):
            self.events.append(event)
            if event.get('phase') == 'returning_home':
                self.follow = False
        runner = self.runner()
        runner.report = report
        with self.assertRaisesRegex(RuntimeError, 'timed out: returning_home'):
            runner.run(self.home)
        self.assertFalse(any(e.get('phase') == 'closing_grippers' for e in self.events))
        np.testing.assert_allclose(self.actual[GRIPPER_INDICES], [.10, .10], atol=.003)
        np.testing.assert_equal(self.commands[-1][GRIPPER_INDICES], [.10, .10])

    def test_stalled_grippers_do_not_advance_to_closing_or_return(self):
        self.follow = False
        self.cfg['reset']['phase_timeout_sec'] = 5.0
        with self.assertRaisesRegex(RuntimeError, 'timed out: opening_grippers'):
            self.runner().run(self.home)
        self.assertEqual([e['phase'] for e in self.events], ['opening_grippers'])
        np.testing.assert_allclose(self.commands[-1][GRIPPER_INDICES], [.10, .10])

    def test_cancel_and_stale_feedback_abort_before_next_command(self):
        with self.assertRaises(InterruptedError):
            self.runner(cancelled=lambda: self.clock > .2).run(self.home)
        self.assertFalse(any(e.get('phase') == 'closing_grippers' for e in self.events))
        self.controller.send.reset_mock()
        self.obs.actual.side_effect = RuntimeError('feedback stale')
        with self.assertRaisesRegex(RuntimeError, 'feedback stale'):
            self.runner().run(self.home)
        self.controller.send.assert_not_called()

    def test_invalid_pose_or_reset_config_never_sends(self):
        bad = self.home.copy()
        bad[5] = float('nan')
        with self.assertRaises(ValueError):
            self.runner().run(bad)
        self.controller.send.assert_not_called()
        self.cfg['reset']['gripper_open_m'] = .2
        with self.assertRaises(ValueError):
            validate_reset_config(self.cfg)

    def test_start_reference_roundtrip_and_mapping_validation(self):
        with tempfile.TemporaryDirectory() as d:
            self.cfg['log_dir'] = d
            run = Path(d)/'execute'
            saved = save_start_pose(self.cfg, run, self.pose)
            pose, reference = load_start_pose(self.cfg)
            np.testing.assert_equal(pose, self.pose)
            self.assertEqual(reference, saved)
            self.assertFalse(reference['estimated'])
            self.assertEqual(load_start_pose(self.cfg, run)[1], saved)
            self.cfg['control']['channels']['left'] = 'can2'
            with self.assertRaisesRegex(ValueError, 'mapping differs'):
                load_start_pose(self.cfg)

    def test_fixed_pose_ignores_latest_deploy_and_allows_explicit_override(self):
        with tempfile.TemporaryDirectory() as d:
            self.cfg['log_dir'] = d
            fixed, reference = load_reset_pose(self.cfg)
            self.assertEqual(reference['source'], 'configured_fixed_pose')
            np.testing.assert_equal(fixed[:6], self.cfg['reset']['home_joints_rad']['left'])
            np.testing.assert_equal(fixed[7:13], self.cfg['reset']['home_joints_rad']['right'])
            np.testing.assert_equal(fixed[GRIPPER_INDICES], [0, 0])
            run = Path(d)/'execute'
            save_start_pose(self.cfg, run, self.pose)
            np.testing.assert_equal(load_reset_pose(self.cfg)[0], fixed)
            np.testing.assert_equal(load_reset_pose(self.cfg, run)[0], self.pose)

    def test_invalid_fixed_pose_rejected_without_falling_back_to_logs(self):
        for bad in ([0]*5, [float('nan')]*6, [20]*6):
            with self.subTest(bad=bad):
                self.cfg['reset']['home_joints_rad']['left'] = bad
                with self.assertRaises(ValueError):
                    load_reset_pose(self.cfg)
        del self.cfg['reset']['home_joints_rad']
        with self.assertRaisesRegex(ValueError, 'home_joints_rad'):
            load_reset_pose(self.cfg)

    def test_legacy_requires_explicit_run_and_rejects_dry_run(self):
        with tempfile.TemporaryDirectory() as d:
            self.cfg['log_dir'] = d
            run = Path(d)/'legacy'
            run.mkdir()
            (run/'config.json').write_text(json.dumps(self.cfg))
            row = dict(seq=0, observation_stamp=123, state=self.pose.tolist(), commanded=self.pose.tolist())
            (run/'actions.jsonl').write_text(json.dumps(row)+'\n')
            with self.assertRaisesRegex(ValueError, 'No saved start pose'):
                load_start_pose(self.cfg)
            pose, reference = load_start_pose(self.cfg, run)
            np.testing.assert_equal(pose, self.pose)
            self.assertTrue(reference['estimated'])
            self.assertFalse((Path(d)/'latest_start_pose.json').exists())
            row['commanded'] = None
            (run/'actions.jsonl').write_text(json.dumps(row)+'\n')
            with self.assertRaisesRegex(ValueError, 'not dry-run'):
                load_start_pose(self.cfg, run)

    def sdk_controller(self):
        controller = Controller(self.cfg, self.obs, require_images=False)
        controller.arms = {s: Mock() for s in ('left', 'right')}
        controller.grippers = {s: Mock() for s in ('left', 'right')}
        for arm in controller.arms.values():
            arm.has_comm_error.return_value = False
            arm.get_joints_enable_status_list.return_value = [True]*6
        return controller

    def test_snapshot_saved_before_any_enable_or_motion(self):
        c = self.sdk_controller()
        def save(pose):
            np.testing.assert_equal(pose, self.pose)
            for arm in c.arms.values():
                arm.enable.assert_not_called()
                arm.move_j.assert_not_called()
            for gripper in c.grippers.values():
                gripper.move_gripper_m.assert_not_called()
        callback = Mock(side_effect=save)
        with patch('model_deploy.controller.threading.Thread'):
            c.start(on_initial_pose=callback)
        callback.assert_called_once()
        for arm in c.arms.values():
            arm.enable.assert_called_once()

    def test_snapshot_write_failure_prevents_commands(self):
        c = self.sdk_controller()
        with self.assertRaises(OSError):
            c.start(on_initial_pose=Mock(side_effect=OSError('disk full')))
        self.assertFalse(c.active)
        for arm in c.arms.values():
            arm.enable.assert_not_called()
            arm.move_j.assert_not_called()

    def test_running_deploy_blocks_reset_before_sdk_or_can_connection(self):
        c = Controller(self.cfg, self.obs, require_images=False)
        with patch('model_deploy.controller.check_can'), \
             patch('builtins.open', mock_open()), \
             patch('model_deploy.controller.fcntl.flock', side_effect=BlockingIOError), \
             patch('model_deploy.controller.can.Bus') as bus, \
             patch('pyAgxArm.AgxArmFactory.create_arm') as factory:
            with self.assertRaisesRegex(RuntimeError, 'another deploy/reset controller'):
                c.connect()
            bus.assert_not_called()
            factory.assert_not_called()
            c.close()

    def test_reset_needs_arm_feedback_but_no_camera_subscription_or_watchdog(self):
        with patch('rospy.Subscriber') as subscribe:
            Observations(self.cfg, include_images=False)
        topics = [call.args[0] for call in subscribe.call_args_list]
        self.assertEqual(len(topics), 7)
        self.assertFalse(any(t.startswith('/cam_') for t in topics))
        c = self.sdk_controller()
        c.active = True
        c.closed = Mock()
        c.closed.wait.side_effect = [False, True]
        with patch('model_deploy.controller.time.monotonic', return_value=c.last_send):
            c.watchdog()
        self.assertTrue(c.active)
        self.obs.actual.assert_called_once()
        self.obs.deployment_hardware.assert_called_once()
        self.obs.snapshot.assert_not_called()


if __name__ == '__main__':
    unittest.main()
