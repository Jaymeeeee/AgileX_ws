"""Feedback-confirmed base +Z translation before the joint-space reset."""
import math
import numpy as np


def validate_lift(cfg):
    lift = cfg['reset'].get('vertical_lift')
    if lift is None:
        return None
    if lift.get('base_z_is_up') is not True:
        raise ValueError('Confirm reset.vertical_lift.base_z_is_up before Cartesian lifting')
    mode = lift.get('mode', 'linear')
    if mode not in ('linear', 'joint_path'):
        raise ValueError('Unknown reset.vertical_lift.mode')
    keys = ('distance_m', 'speed_m_s', 'position_tolerance_m', 'max_path_error_m')
    keys += (('max_lateral_m', 'max_rotation_rad', 'joint_limit_margin_rad') if mode == 'joint_path'
             else ('orientation_tolerance_rad',))
    for key in keys:
        value = lift[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid positive reset.vertical_lift.' + key)
    if lift['position_tolerance_m'] >= lift['distance_m']:
        raise ValueError('Lift arrival tolerance must be smaller than lift distance')
    if lift['max_path_error_m'] <= lift['position_tolerance_m']:
        raise ValueError('Lift path error limit must exceed arrival tolerance')
    if lift['distance_m']/lift['speed_m_s'] + cfg['reset']['settle_sec'] >= cfg['reset']['phase_timeout_sec']:
        raise ValueError('Lift duration exceeds reset phase timeout; adjust distance/speed/timeout')
    return lift


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def orientation_error(a, b):
    return math.acos(float(np.clip((np.trace(rotation(a).T @ rotation(b))-1)/2, -1, 1)))


def lift_before_return(runner):
    """Use firmware linear interpolation, keeping XY and rotation at measured start."""
    lift = validate_lift(runner.cfg)
    if lift.get('mode', 'linear') == 'joint_path':
        from .reset_joint_lift import execute_joint_lift
        return execute_joint_lift(runner)
    controller, r = runner.controller, runner.cfg['reset']
    start = controller.flange_poses()
    goal = start.copy()
    goal[:, 2] += lift['distance_m']
    command = start.copy()
    began = runner.clock()
    stable_since = None
    runner.report({'state': 'running', 'phase': 'lifting_arms', 'frame': 'each_arm_base',
                   'start_flange_poses': start.tolist(), 'goal_flange_poses': goal.tolist()})
    while True:
        tick = runner.clock()
        if runner.cancelled():
            raise InterruptedError('Reset cancelled during lifting_arms')
        controller.check_fault()
        joints = runner.guard.bounded(runner.obs.actual())
        actual = controller.flange_poses()
        if tick-began > r['phase_timeout_sec']:
            raise RuntimeError('Reset phase timed out: lifting_arms; no joint return/closing sent')
        for i, side in enumerate(runner.sides):
            xy_error = np.linalg.norm(actual[i, :2]-start[i, :2])
            z = actual[i, 2]
            angle = orientation_error(actual[i, 3:], start[i, 3:])
            if (xy_error > lift['max_path_error_m']
                    or z < start[i, 2]-lift['max_path_error_m']
                    or z > goal[i, 2]+lift['max_path_error_m']
                    or angle > lift['orientation_tolerance_rad']):
                raise RuntimeError(side + ': lift deviated from vertical path/orientation; no joint return sent')
        if np.any(np.abs(joints[runner.gripper_indices]-r['gripper_open_m']) > r['gripper_tolerance_m']):
            raise RuntimeError('Gripper no longer open during lift')
        # Advance only a short distance ahead of actual height; a blocked arm
        # cannot accumulate the entire lift target while the other keeps moving.
        lead = command[:, 2]-actual[:, 2]
        if np.any(np.abs(lead) > lift['max_path_error_m']):
            raise RuntimeError('Lift target tracking error exceeded; no joint return sent')
        step = lift['speed_m_s']*runner.period
        if np.all(lead <= lift['max_path_error_m']/2):
            command[:, 2] = np.minimum(goal[:, 2], command[:, 2]+step)
        controller.send_lift(command, r['gripper_open_m'])
        settled = (np.all(np.linalg.norm(actual[:, :3]-goal[:, :3], axis=1) <= lift['position_tolerance_m'])
                   and np.all(np.abs(command[:, 2]-goal[:, 2]) <= lift['position_tolerance_m']))
        if settled:
            if stable_since is None:
                stable_since = tick
            if tick-stable_since >= r['settle_sec']:
                runner.report({'state': 'phase_complete', 'phase': 'lifting_arms',
                               'actual_flange_poses': actual.tolist()})
                # Cartesian motion changed joints: start subsequent move_j at
                # fresh measured joints, never the pre-lift joint command.
                return controller.finish_lift()
        else:
            stable_since = None
        runner.sleep(max(0., runner.period-(runner.clock()-tick)))
