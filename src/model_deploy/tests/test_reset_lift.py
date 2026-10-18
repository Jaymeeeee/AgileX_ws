from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock
import numpy as np

from model_deploy.core import load_config
from model_deploy.controller import Controller
from model_deploy.reset_lift import validate_lift, orientation_error
from model_deploy.reset_sequence import ResetSequence


class LiftTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(Path(__file__).resolve().parents[1]/'config/act.yaml')
        self.cfg['reset']['vertical_lift'] = dict(base_z_is_up=True, distance_m=.10,
            speed_m_s=.02, position_tolerance_m=.003, orientation_tolerance_rad=.05,
            max_path_error_m=.01)
        self.pose = np.tile([.1, .5, -.5, .1, .1, .1, .04], 2)
        self.actual = self.pose.copy()
        self.home = self.pose.copy()
        self.home[0] += .1
        self.initial_flange = np.array([[.25, .05, .20, .1, .3, -.2], [.28, -.04, .18, .2, .3, .1]])
        self.flange = self.initial_flange.copy()
        self.clock = 0.
        self.events, self.sent = [], []
        self.follow = True
        self.obs = Mock()
        self.obs.actual.side_effect = lambda **_: self.actual.copy()
        self.controller = Mock()
        self.controller.flange_poses.side_effect = lambda: self.flange.copy()
        self.controller.send.side_effect = self.send_joints
        self.controller.send_lift.side_effect = self.send_lift
        self.controller.finish_lift.side_effect = lambda: self.actual.copy()
        self.runner = ResetSequence(self.cfg, self.controller, self.obs, clock=lambda: self.clock,
                                    sleep=self.sleep, report=self.events.append)

    def sleep(self, duration):
        self.clock += duration

    def send_joints(self, command):
        self.actual = command.copy()
        return command.copy()

    def send_lift(self, poses, width):
        self.sent.append(poses.copy())
        if self.follow:
            self.flange += .7*(poses-self.flange)
            # Cartesian lift changes measured joints; returning must start here.
            self.actual[1] = .7
            self.actual[8] = .7

    def test_open_lift_return_close_and_preserve_xy_orientation(self):
        self.runner.run(self.home)
        phases = [e['phase'] for e in self.events if e['state'] == 'phase_complete']
        self.assertEqual(phases, ['opening_grippers', 'lifting_arms', 'returning_home', 'closing_grippers'])
        for sent in self.sent:
            np.testing.assert_equal(sent[:, [0, 1, 3, 4, 5]], self.initial_flange[:, [0, 1, 3, 4, 5]])
        heights = np.array([self.initial_flange, *self.sent])[:, :, 2]
        self.assertTrue(np.all(np.diff(heights, axis=0) >= -1e-12))
        self.assertLessEqual(np.diff(heights, axis=0).max(), .02/15+1e-12)
        np.testing.assert_allclose(self.flange[:, 2], self.initial_flange[:, 2]+.1, atol=.003)
        self.controller.finish_lift.assert_called_once()
        # First return command is close to the post-lift .7 rad, not pre-lift .5.
        calls = self.controller.send.call_args_list
        self.assertTrue(any(abs(call.args[0][1]-.68) < 1e-8 for call in calls))
        np.testing.assert_equal(self.actual[[6, 13]], [0, 0])

    def test_unreachable_height_times_out_without_return_or_close(self):
        self.follow = False
        with self.assertRaisesRegex(RuntimeError, 'timed out: lifting_arms'):
            self.runner.run(self.home)
        self.assertFalse(any(e.get('phase') in ('returning_home', 'closing_grippers') for e in self.events))
        # Blocking does not allow the whole 10 cm target to run ahead.
        self.assertLessEqual(np.max(self.sent[-1][:, 2]-self.initial_flange[:, 2]), .01)
        self.controller.finish_lift.assert_not_called()

    def test_bad_path_orientation_and_downward_motion_abort(self):
        for column, delta in ((0, .02), (2, -.02), (3, .1)):
            with self.subTest(column=column):
                self.setUp()
                original = self.controller.send_lift.side_effect
                def deviating_send(poses, width):
                    original(poses, width)
                    self.flange[0, column] += delta
                self.controller.send_lift.side_effect = deviating_send
                with self.assertRaisesRegex(RuntimeError, 'deviated'):
                    self.runner.run(self.home)
                self.controller.finish_lift.assert_not_called()

    def test_stale_pose_and_cancellation_do_not_start_return(self):
        original = self.controller.send_lift.side_effect
        def stale_send(poses, width):
            original(poses, width)
            self.controller.flange_poses.side_effect = RuntimeError('stale flange')
        self.controller.send_lift.side_effect = stale_send
        with self.assertRaisesRegex(RuntimeError, 'stale flange'):
            self.runner.run(self.home)
        self.controller.finish_lift.assert_not_called()
        self.setUp()
        self.runner.cancelled = lambda: bool(self.sent)
        with self.assertRaisesRegex(InterruptedError, 'lifting_arms'):
            self.runner.run(self.home)
        self.controller.finish_lift.assert_not_called()

    def test_config_and_equivalent_wrapped_orientation(self):
        self.assertAlmostEqual(orientation_error([0, 0, np.pi], [0, 0, -np.pi]), 0.)
        for key, bad in (('distance_m', -.1), ('base_z_is_up', False),
                         ('speed_m_s', .0001), ('distance_m', float('nan'))):
            with self.subTest(key=key):
                self.setUp()
                self.cfg['reset']['vertical_lift'][key] = bad
                with self.assertRaises(ValueError):
                    validate_lift(self.cfg)

    def sdk_controller(self):
        c = Controller(self.cfg, self.obs, require_images=False)
        c.active = True
        c.arms = {s: Mock() for s in ('left', 'right')}
        c.grippers = {s: Mock() for s in ('left', 'right')}
        for i, side in enumerate(('left', 'right')):
            c.arms[side].has_comm_error.return_value = False
            c.arms[side].get_flange_pose.return_value = SimpleNamespace(
                msg=self.initial_flange[i].tolist(), timestamp=time.time())
        return c

    def test_sdk_sends_move_l_in_meters_and_preserves_rotation(self):
        c = self.sdk_controller()
        target = c.flange_poses()
        target[:, 2] += .001
        c.send_lift(target, .1)
        for i, side in enumerate(('left', 'right')):
            c.arms[side].move_l.assert_called_once_with(target[i].tolist())
            c.arms[side].move_j.assert_not_called()
            c.grippers[side].move_gripper_m.assert_called_once_with(.1, self.cfg['control']['gripper_force'])
        self.actual[1] = .7
        c.finish_lift()
        np.testing.assert_equal(c.guard.previous, self.actual)
        c.arms['left'].move_j.assert_called_once_with(self.actual[:6].tolist())

    def test_stale_or_invalid_sdk_feedback_prevents_both_sends(self):
        for field, value in (('timestamp', time.time()-1), ('msg', [0, 0, .2, 0, 2., 0]),
                             ('msg', [float('nan')]*6)):
            with self.subTest(field=field):
                c = self.sdk_controller()
                setattr(c.arms['right'].get_flange_pose.return_value, field, value)
                with self.assertRaises(RuntimeError):
                    c.send_lift(self.initial_flange, .1)
                for arm in c.arms.values():
                    arm.move_l.assert_not_called()

    def test_installed_sdk_linear_mode_and_units_without_can(self):
        from pyAgxArm import create_agx_arm_config, AgxArmFactory
        arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version='v189', channel='can0'))
        arm._send_msg = Mock()  # no connect(); intercept all outgoing SDK messages
        arm.move_l([.25, .05, .30, .1, .3, -.2])
        messages = [call.args[0] for call in arm._send_msg.call_args_list]
        self.assertEqual(messages[0].move_mode, 2)  # firmware L mode
        self.assertEqual(messages[1].X_axis, 250000)
        self.assertEqual(messages[1].Y_axis, 50000)
        self.assertEqual(messages[2].Z_axis, 300000)
        self.assertEqual(messages[2].RX_axis, round(.1*180/np.pi*1000))
        self.assertEqual(messages[3].RY_axis, round(.3*180/np.pi*1000))
        self.assertEqual(messages[3].RZ_axis, round(-.2*180/np.pi*1000))

    def test_partial_cartesian_send_failure_uses_existing_hold_on_close(self):
        c = self.sdk_controller()
        c.arms['right'].move_l.side_effect = RuntimeError('transport failed')
        with self.assertRaisesRegex(RuntimeError, 'transport failed'):
            c.send_lift(self.initial_flange, .1)
        c.close()
        for i, side in enumerate(('left', 'right')):
            c.arms[side].move_j.assert_called_once_with(self.actual[i*7:i*7+6].tolist())
            c.arms[side].disable.assert_not_called()


if __name__ == '__main__':
    unittest.main()
