"""Position-only lift planning with SDK FK; no CAN or motion during planning."""
import numpy as np

from .reset_lift import orientation_error


def plan_joint_lift(fks, joints, measured, low, high, lift, poll=lambda: None):
    """Plan both arms completely, allowing rotation and bounded lateral drift.

    Damped position IK is seeded from each previous waypoint. Joint-space
    segments are sampled densely, including their interiors, before execution.
    This checks kinematics, not collisions with the environment or the other arm.
    """
    joints, measured = np.asarray(joints), np.asarray(measured)
    low, high = np.asarray(low), np.asarray(high)
    if (joints.shape != (2, 6) or measured.shape != (2, 6)
            or low.shape != (2, 6) or high.shape != (2, 6)
            or not np.isfinite([joints, measured, low, high]).all()
            or np.any(low >= high) or np.any(joints < low) or np.any(joints > high)):
        raise ValueError('Invalid joint lift start/limits; no lift sent')
    origin = np.array([fk(q.tolist()) for fk, q in zip(fks, joints)])
    if (not np.isfinite(origin).all()
            or np.any(np.linalg.norm(origin[:, :3]-measured[:, :3], axis=1) > lift['position_tolerance_m'])
            or any(orientation_error(a[3:], b[3:]) > .05 for a, b in zip(origin, measured))):
        raise RuntimeError('SDK FK disagrees with measured flange pose; no lift sent')
    # A start near a limit may move inward, but never farther toward that limit.
    lower = np.minimum(low+lift['joint_limit_margin_rad'], joints)
    upper = np.maximum(high-lift['joint_limit_margin_rad'], joints)
    if np.any(lower >= upper):
        raise ValueError('Joint lift margin leaves no usable joint range')
    paths = []
    for side, fk in enumerate(fks):
        q = joints[side].copy()
        path = [q.copy()]
        xyz0 = origin[side, :3]
        count = int(np.ceil(lift['distance_m']/.001))
        for height in np.linspace(0, lift['distance_m'], count+1)[1:]:
            target = xyz0 + [0, 0, height]
            for _ in range(100):
                poll()
                pose = np.asarray(fk(q.tolist()))
                error = target-pose[:3]
                if abs(error[2]) < .0001 and np.linalg.norm(error[:2]) < .001:
                    break
                jac = np.empty((3, 6))
                for j in range(6):
                    probe = q.copy()
                    probe[j] += 1e-5
                    jac[:, j] = (np.asarray(fk(probe.tolist()))[:3]-pose[:3])/1e-5
                # Free orientation, prefer unchanged XY, strongly prioritize Z.
                weight = np.array([.25, .25, 1.])
                jac = weight[:, None]*jac
                mobility = np.clip(np.minimum(q-lower[side], upper[side]-q)/.2, .05, 1.)
                weighted = jac*mobility
                dq = mobility*(weighted.T @ np.linalg.solve(
                    weighted @ weighted.T + .002**2*np.eye(3), weight*error))
                dq *= min(1., .01/max(np.max(np.abs(dq)), 1e-12))
                candidate = np.clip(q+dq, lower[side], upper[side])
                if np.max(np.abs(candidate-q)) < 1e-8:
                    break
                q = candidate
            pose = np.asarray(fk(q.tolist()))
            if abs(pose[2]-target[2]) > .0002:
                raise RuntimeError(('left', 'right')[side] + ': no bounded upward joint path; no lift sent')
            path.append(q.copy())
        paths.append(path)
    # Synchronize at equal height waypoints, then subdivide joint segments.
    points = [joints.copy()]
    poses = [origin.copy()]
    for pair in zip(paths[0][1:], paths[1][1:]):
        end = np.array(pair)
        start = points[-1]
        n = max(1, int(np.ceil(np.max(np.abs(end-start))/.005)))
        for fraction in np.linspace(0, 1, n+1)[1:]:
            poll()
            q = start+fraction*(end-start)
            p = np.array([fk(v.tolist()) for fk, v in zip(fks, q)])
            if (not np.isfinite(p).all()
                    or np.any(p[:, 2] < poses[-1][:, 2]-.0002)
                    or np.any(p[:, 2] < origin[:, 2]-.0002)
                    or np.any(p[:, 2] > origin[:, 2]+lift['distance_m']+.0002)
                    or np.any(np.linalg.norm(p[:, :2]-origin[:, :2], axis=1) > lift['max_lateral_m'])
                    or any(orientation_error(a[3:], b[3:]) > lift['max_rotation_rad']
                           for a, b in zip(p, origin))):
                raise RuntimeError('Joint lift path exceeds height/lateral/rotation envelope; no lift sent')
            points.append(q)
            poses.append(p)
    return dict(joints=np.array(points), poses=np.array(poses), measured=measured.copy(),
                low=low.copy(), high=high.copy())


def execute_joint_lift(runner):
    c, r = runner.controller, runner.cfg['reset']
    lift = r['vertical_lift']
    plan = c.joint_lift_plan
    start = c.flange_poses()
    command = runner.guard.bounded(runner.obs.actual())
    indices = np.array([[0, 1, 2, 3, 4, 5], [7, 8, 9, 10, 11, 12]])
    if (np.max(np.abs(command[indices]-plan['joints'][0])) > .01
            or np.any(np.linalg.norm(start[:, :3]-plan['measured'][:, :3], axis=1) > lift['position_tolerance_m'])):
        raise RuntimeError('Pose changed since lift planning; rerun reset to replan')
    command[[6, 13]] = r['gripper_open_m']
    goal_z = start[:, 2]+lift['distance_m']
    began, stable_since, index = runner.clock(), None, 1
    runner.report(dict(state='running', phase='lifting_arms', mode='joint_path',
                       frame='each_arm_base', start_flange_poses=start.tolist(),
                       goal_z_m=goal_z.tolist(), planned_end_flange_poses=plan['poses'][-1].tolist()))
    while True:
        tick = runner.clock()
        if runner.cancelled():
            raise InterruptedError('Reset cancelled during lifting_arms')
        c.check_fault()
        actual_joints = runner.guard.bounded(runner.obs.actual())
        actual = c.flange_poses()
        if tick-began > r['phase_timeout_sec']:
            raise RuntimeError('Reset phase timed out: lifting_arms; no joint return/closing sent')
        if (np.any(actual[:, 2] < start[:, 2]-lift['max_path_error_m'])
                or np.any(actual[:, 2] > goal_z+lift['max_path_error_m'])
                or np.any(np.linalg.norm(actual[:, :2]-start[:, :2], axis=1) > lift['max_lateral_m'])
                or any(orientation_error(a[3:], b[3:]) > lift['max_rotation_rad'] for a, b in zip(actual, start))):
            raise RuntimeError('Joint lift deviated from height/lateral/rotation envelope')
        if np.any(np.abs(actual_joints[[6, 13]]-r['gripper_open_m']) > r['gripper_tolerance_m']):
            raise RuntimeError('Gripper no longer open during lift')
        expected = c.lift_fk(command[indices])
        error = np.linalg.norm(actual[:, :3]-expected[:, :3], axis=1)
        if np.any(error > lift['max_path_error_m']):
            raise RuntimeError('Joint lift target tracking error exceeded; no joint return sent')
        following = (np.all(error <= lift['max_path_error_m']/2)
                     and np.max(np.abs(command[indices]-actual_joints[indices]))
                     <= min(.03, runner.cfg['control']['max_tracking_error_rad']/2))
        if index < len(plan['joints']) and following:
            delta = plan['joints'][index]-command[indices]
            length = np.max(np.linalg.norm(plan['poses'][index, :, :3]-expected[:, :3], axis=1))
            fraction = min(1., min(r['joint_speed_rad_s'], runner.cfg['control']['max_joint_speed_rad_s'])*runner.period
                           /max(np.max(np.abs(delta)), 1e-12),
                           lift['speed_m_s']*runner.period/max(length, 1e-12))
            command[indices] += fraction*delta
        command = np.asarray(c.send(command))
        if index < len(plan['joints']) and np.max(np.abs(command[indices]-plan['joints'][index])) < 1e-8:
            index += 1
        arrived = (index == len(plan['joints'])
                   and np.all(np.abs(actual[:, 2]-goal_z) <= lift['position_tolerance_m'])
                   and np.max(np.abs(actual_joints[indices]-plan['joints'][-1])) <= r['joint_tolerance_rad'])
        if arrived:
            stable_since = tick if stable_since is None else stable_since
            if tick-stable_since >= r['settle_sec']:
                runner.report(dict(state='phase_complete', phase='lifting_arms',
                                   actual_flange_poses=actual.tolist(), actual_lift_m=(actual[:, 2]-start[:, 2]).tolist()))
                return c.finish_lift()
        else:
            stable_since = None
        runner.sleep(max(0., runner.period-(runner.clock()-tick)))
