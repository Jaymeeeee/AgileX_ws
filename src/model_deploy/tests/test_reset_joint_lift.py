"""Offline motion regression: real SDK FK, no connect/enable/CAN operations."""
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from pyAgxArm import AgxArmFactory, create_agx_arm_config

from model_deploy.controller import Controller
from model_deploy.core import load_config
from model_deploy.reset_joint_lift import plan_joint_lift
from model_deploy.reset_lift import orientation_error, validate_lift
from model_deploy.reset_sequence import ResetSequence


class JointLiftTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(Path(__file__).resolve().parents[1]/'config/act.yaml')
        self.arm = AgxArmFactory.create_arm(create_agx_arm_config(
            robot='piper', firmeware_version='v189', channel='can0'))
        # Actual start of reset_20261016_203250_965274, which failed in MOVE L.
        self.q = np.array([[-.3035476635, 1.8984120807, -.7987673855, -1.1992855422, .7452032307, 2.2283316692],
                           [.8155574529, 2.0071809997, -1.5333241078, .1718974780, 1.1639600782, -.7337538708]])
        self.low = np.tile(self.cfg['control']['joint_min'], (2, 1))
        self.high = np.tile(self.cfg['control']['joint_max'], (2, 1))
        self.initial = self.fk(self.q)

    def fk(self, q):
        return np.array([self.arm.fk(v.tolist()) for v in q])

    def plan(self, **kw):
        args = dict(fks=[self.arm.fk]*2, joints=self.q, measured=self.initial,
                    low=self.low, high=self.high, lift=self.cfg['reset']['vertical_lift'])
        args.update(kw)
        return plan_joint_lift(**args)

    def test_reported_failure_pose_has_bounded_upward_path(self):
        p = self.plan()
        np.testing.assert_allclose(p['poses'][-1, :, 2]-self.initial[:, 2], .1, atol=.0002)
        self.assertTrue(np.all(p['joints'] >= self.low+.03-1e-9))
        self.assertTrue(np.all(p['joints'] <= self.high-.03+1e-9))
        self.assertTrue(np.all(np.diff(p['poses'][:, :, 2], axis=0) >= -.0002))
        self.assertLess(np.linalg.norm(p['poses'][:, :, :2]-self.initial[:, :2], axis=2).max(), .05)
        # Orientation changes instead of demanding the old infeasible fixed orientation.
        self.assertGreater(orientation_error(p['poses'][-1, 1, 3:], self.initial[1, 3:]), .1)
        self.assertLess(np.max(np.abs(np.diff(p['joints'], axis=0))), .0051)

    def test_unreachable_and_bad_model_refuse_plan(self):
        lift = copy.deepcopy(self.cfg['reset']['vertical_lift'])
        lift['distance_m'] = 1.
        with self.assertRaisesRegex(RuntimeError, 'no bounded|envelope'):
            self.plan(lift=lift)
        measured = self.initial.copy()
        measured[1, 2] += .02
        with self.assertRaisesRegex(RuntimeError, 'FK disagrees'):
            self.plan(measured=measured)
        with self.assertRaises(ValueError):
            self.plan(high=self.q-.01)

    def make_runner(self):
        self.actual = np.column_stack([self.q, [.04, .04]]).reshape(-1)
        self.clock, self.events, self.sent = 0., [], []
        self.block = False
        self.obs = Mock()
        self.obs.actual.side_effect = lambda **_: self.actual.copy()
        self.c = Mock()
        self.c.joint_lift_plan = self.plan()
        self.c.lift_fk.side_effect = self.fk
        self.c.flange_poses.side_effect = lambda: self.fk(self.actual.reshape(2, 7)[:, :6])
        def send(target):
            self.sent.append(target.copy())
            if not self.block:
                self.actual += .7*(target-self.actual)
            return target.copy()
        self.c.send.side_effect = send
        self.c.finish_lift.side_effect = lambda: self.actual.copy()
        def sleep(dt):
            self.clock += dt
        self.runner = ResetSequence(self.cfg, self.c, self.obs, clock=lambda: self.clock,
                                    sleep=sleep, report=self.events.append)
        self.home = np.r_[self.cfg['reset']['home_joints_rad']['left'], 0,
                          self.cfg['reset']['home_joints_rad']['right'], 0]

    def test_feedback_confirmed_lift_before_return_and_close(self):
        self.make_runner()
        self.runner.run(self.home)
        phases = [e for e in self.events if e['state'] == 'phase_complete']
        self.assertEqual([e['phase'] for e in phases],
                         ['opening_grippers', 'lifting_arms', 'returning_home', 'closing_grippers'])
        np.testing.assert_allclose(phases[1]['actual_lift_m'], [.1, .1], atol=.003)
        self.c.send_lift.assert_not_called()
        self.c.finish_lift.assert_called_once()
        delta = np.diff(np.array(self.sent).reshape(-1, 2, 7)[:, :, :6], axis=0)
        # finish_lift reanchors the next phase at fresh measured joints.
        self.assertLessEqual(np.abs(delta).max(), .3/15+1e-6)

    def test_lift_from_home_at_joint_boundaries(self):
        q = np.array([self.cfg['reset']['home_joints_rad'][s] for s in ('left', 'right')])
        p = self.plan(joints=q, measured=self.fk(q))
        np.testing.assert_allclose(p['poses'][-1, :, 2]-p['poses'][0, :, 2], .1, atol=.0002)
        self.assertTrue(np.all(p['joints'] >= self.low))
        self.assertTrue(np.all(p['joints'] <= self.high))

    def test_blocked_lift_never_returns_or_closes(self):
        self.make_runner()
        def report(e):
            self.events.append(e)
            if e['state'] == 'running' and e['phase'] == 'lifting_arms':
                self.block = True
        self.runner.report = report
        with self.assertRaisesRegex(RuntimeError, 'timed out: lifting_arms'):
            self.runner.run(self.home)
        self.assertFalse(any(e.get('phase') in ('returning_home', 'closing_grippers') for e in self.events))
        self.c.finish_lift.assert_not_called()
        p = self.fk(self.sent[-1].reshape(2, 7)[:, :6])
        self.assertLess((p[:, 2]-self.initial[:, 2]).max(), .01)

    def test_deviation_stale_fault_cancellation_and_changed_start_abort(self):
        for kind in ('down', 'lateral', 'rotation', 'stale', 'fault', 'cancel', 'changed_start'):
            with self.subTest(kind=kind):
                self.make_runner()
                def report(e):
                    self.events.append(e)
                    if e['state'] == 'phase_complete' and e['phase'] == 'opening_grippers':
                        if kind == 'changed_start':
                            self.actual[1] += .02
                    if e['state'] == 'running' and e['phase'] == 'lifting_arms':
                        if kind in ('down', 'lateral', 'rotation'):
                            p = self.initial.copy()
                            col, delta = dict(down=(2, -.02), lateral=(0, .06), rotation=(3, 1.))[kind]
                            p[1, col] += delta
                            self.c.flange_poses.side_effect = lambda: p
                        elif kind == 'stale':
                            self.c.flange_poses.side_effect = RuntimeError('stale')
                        elif kind == 'fault':
                            self.c.check_fault.side_effect = RuntimeError('arm fault')
                        elif kind == 'cancel':
                            self.runner.cancelled = lambda: True
                self.runner.report = report
                with self.assertRaises((RuntimeError, InterruptedError)):
                    self.runner.run(self.home)
                self.assertFalse(any(e.get('phase') in ('returning_home', 'closing_grippers') for e in self.events))

    def test_device_limits_are_intersected_before_any_motion(self):
        self.make_runner()
        c = Controller(self.cfg, self.obs, require_images=False)
        c.flange_poses = Mock(return_value=self.initial)
        for i, side in enumerate(('left', 'right')):
            arm = Mock()
            arm.has_comm_error.return_value = False
            arm.fk.side_effect = self.arm.fk
            def get_limit(j, i=i):
                return SimpleNamespace(msg=SimpleNamespace(min_angle_limit=self.low[i, j-1]+.01,
                                                            max_angle_limit=self.high[i, j-1]-.01))
            arm.get_joint_angle_vel_limits.side_effect = get_limit
            c.arms[side] = arm
        result = c.prepare_joint_lift()
        np.testing.assert_allclose(result['low'], self.low+.01)
        for arm in c.arms.values():
            arm.enable.assert_not_called()
            arm.move_j.assert_not_called()
        c.arms['right'].get_joint_angle_vel_limits.side_effect = lambda _: None
        with self.assertRaisesRegex(RuntimeError, 'cannot read'):
            c.prepare_joint_lift()

    def test_new_config_validates(self):
        validate_lift(self.cfg)
        for key, value in (('mode', 'bad'), ('max_lateral_m', 0), ('max_rotation_rad', float('nan')),
                           ('joint_limit_margin_rad', -.01)):
            cfg = copy.deepcopy(self.cfg)
            cfg['reset']['vertical_lift'][key] = value
            with self.assertRaises(ValueError):
                validate_lift(cfg)


if __name__ == '__main__':
    unittest.main()
