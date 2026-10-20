"""Run the existing reset sequence through an endpoint's already-owned SDK.

The reset planner operates on pairs. Duplicate this one arm's observations for
its two logical slots; only send once to the real device. Never open another
SDK connection or acquire a second CAN writer while teleoperation owns it.
"""
import copy
import time

import numpy as np

from .controller import Controller
from .core import SIDES, ActionGuard
from .reset_pose import load_reset_pose
from .reset_sequence import ResetSequence, validate_reset_config


class EndpointReset:
    flange_poses = Controller.flange_poses
    lift_fk = Controller.lift_fk
    prepare_joint_lift = Controller.prepare_joint_lift

    def __init__(self, device):
        self.device = device
        self.cfg = copy.deepcopy(device.cfg['deployment'])
        self.cfg['policy']['fps'] = device.cfg['fps']
        self.cfg['control']['sdk_speed_percent'] = self.cfg['reset']['sdk_speed_percent']
        self.cfg['control']['max_joint_speed_rad_s'] = min(
            self.cfg['control']['max_joint_speed_rad_s'], self.cfg['reset']['joint_speed_rad_s'])
        self.cfg['control']['max_gripper_speed_m_s'] = min(
            self.cfg['control']['max_gripper_speed_m_s'], self.cfg['reset']['gripper_speed_m_s'])
        validate_reset_config(self.cfg)
        self.obs = self
        # This endpoint still presents two identical logical slots to the planner.
        self.sides = SIDES
        self.arms = dict(left=device.robot, right=device.robot)
        self.active = False

    def actual(self):
        return np.tile(self.device.actual(), 2)

    def check_fault(self):
        self.device.check()

    def send(self, target):
        target = np.asarray(target, dtype=float)
        if target.shape != (14,) or not np.allclose(target[:7], target[7:], atol=1e-9):
            raise ValueError('Single-arm reset requires matching logical targets')
        return np.tile(self.device.send(target[:7]), 2)

    def send_lift(self, poses, gripper_width):
        poses = np.asarray(poses, dtype=float)
        if (poses.shape != (2, 6) or not np.isfinite(poses).all()
                or not np.allclose(poses[0], poses[1], atol=1e-9)
                or np.any(np.abs(poses[:, 3:]) > np.array([np.pi, np.pi/2, np.pi])+1e-9)
                or not self.cfg['control']['gripper_min'] <= gripper_width <= self.cfg['control']['gripper_max']):
            raise ValueError('Invalid single-arm lift target')
        with self.device.mutex:
            self.check_fault()
            self.actual()
            self.flange_poses()
            self.device.robot.move_l(poses[0].tolist())
            self.device.gripper.move_gripper_m(float(gripper_width), self.cfg['control']['gripper_force'])
            self.device.last_send = time.monotonic()
            self.check_fault()

    def finish_lift(self):
        actual = self.actual()
        self.device.guard.commit(actual)
        return self.send(actual)

    def run(self, cancelled, report):
        home, _ = load_reset_pose(self.cfg)
        slot = 0 if self.device.side == 'left' else 7
        home = np.tile(home[slot:slot+7], 2)
        old_guard = self.device.guard
        try:
            self.device.guard = ActionGuard(self.cfg['control'], 1/self.cfg['policy']['fps'])
            self.device.guard.commit(self.actual())
            self.device.robot.set_speed_percent(self.cfg['reset']['sdk_speed_percent'])
            ResetSequence(self.cfg, self, self, cancelled=cancelled, report=report).run(home)
            self.device.hold_target = self.device.actual()
        finally:
            self.device.guard = old_guard
            # No automatic recovery or further movement after a failed phase.
            # The caller latches a fault and closes the session on failure.
        self.device.guard.commit(self.actual())


def reset_collection(call, stop_recording, sides=SIDES):
    """Stop/save the episode before any pause/motion; leave all four arms held.

    `call(side, role, operation, value=None)` is supplied by the ROS console.
    Each failed step aborts immediately; the console then closes the session.
    """
    stop_recording()
    for side in sides:
        call(side, 'follower', 'follow', False)
    for side in sides:
        call(side, 'leader', 'hold')
    for side in sides:
        call(side, 'follower', 'reset')
    for side in sides:
        call(side, 'leader', 'align_hold')
