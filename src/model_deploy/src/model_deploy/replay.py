"""Read follower feedback from ROS1 bags and replay with bounded SDK commands."""
import copy
import json
import math
from pathlib import Path
import time
import numpy as np
import yaml
from .core import ActionGuard, NAMES, SIDES, joint_values


KEYS = ('left_joints', 'left_gripper', 'right_joints', 'right_gripper')
INPUT_NAMES = ([f'left_follower_joint_{i}' for i in range(1, 7)],
               ['left_follower_gripper_width'],
               [f'right_follower_joint_{i}' for i in range(1, 7)],
               ['right_follower_gripper_width'])


def replay_config(base, path, rate=None):
    with open(Path(path).expanduser()) as f:
        r = yaml.safe_load(f)
    if rate is not None:
        r['rate'] = rate
    for key, value in r.items():
        if key == 'topics':
            continue
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid positive replay setting: ' + key)
    if not 10 <= r['command_hz'] <= 100 or not 0 < r['rate'] <= 2:
        raise ValueError('Replay requires command_hz 10..100 and rate >0..2')
    if not isinstance(r['sdk_speed_percent'], int) or not 1 <= r['sdk_speed_percent'] <= 100:
        raise ValueError('Invalid replay SDK speed')
    if set(r['topics']) != set(KEYS) or len(set(r['topics'].values())) != 4:
        raise ValueError('Expected four distinct follower state topics')
    if r['waypoint_timeout_sec'] <= r['final_settle_sec']:
        raise ValueError('Waypoint timeout must exceed final settle time')
    cfg = copy.deepcopy(base)
    cfg['replay'] = r
    # Controller uses this period for rate limiting; no model is used here.
    cfg['policy']['fps'] = r['command_hz']
    cfg['control'].update(sdk_speed_percent=r['sdk_speed_percent'],
        max_joint_speed_rad_s=r['joint_speed_rad_s'], max_gripper_speed_m_s=r['gripper_speed_m_s'],
        check_initial_target_error=False, check_runtime_target_error=False)
    return cfg


def load_trajectory(path, cfg):
    import rosbag
    path = Path(path).expanduser().resolve()
    if path.is_dir():
        path = path/'data.bag'
    if not path.is_file() or path.suffix != '.bag' or Path(str(path)+'.active').exists():
        raise ValueError('Expected a completed .bag file or recording directory: ' + str(path))
    manifest = path.parent/'manifest.json'
    if manifest.exists() and json.loads(manifest.read_text()).get('state') != 'complete':
        raise ValueError('Recording manifest is not complete')
    identity = (path.stat().st_size, path.stat().st_mtime_ns)
    r = cfg['replay']
    lookup = {r['topics'][key]: (key, names) for key, names in zip(KEYS, INPUT_NAMES)}
    series = {key: ([], []) for key in KEYS}
    guard = ActionGuard(cfg['control'], 1/r['command_hz'])
    with rosbag.Bag(str(path), 'r') as bag:
        bag_start, bag_end = bag.get_start_time(), bag.get_end_time()
        for topic, msg, received in bag.read_messages(topics=list(lookup)):
            key, names = lookup[topic]
            if getattr(msg, '_type', '') != 'sensor_msgs/JointState':
                raise ValueError(topic + ': expected sensor_msgs/JointState')
            stamp = msg.header.stamp.to_nsec()
            if stamp <= 0 or abs(stamp-received.to_nsec()) > r['max_clock_skew_sec']*1e9:
                raise ValueError(topic + ': invalid header/bag timestamp')
            stamps, values = series[key]
            if stamps and stamp <= stamps[-1]:
                raise ValueError(topic + ': non-increasing source timestamps')
            stamps.append(stamp)
            values.append(joint_values(msg, names))
    if identity != (path.stat().st_size, path.stat().st_mtime_ns):
        raise ValueError('Bag changed while reading')
    for key, (stamps, _) in series.items():
        if len(stamps) < 2:
            raise ValueError(key + ': need at least two messages')
        if np.max(np.diff(np.asarray(stamps, dtype=np.int64))) > r['max_source_gap_sec']*1e9:
            raise ValueError(key + ': source gap exceeds replay.max_source_gap_sec')
    start = max(v[0][0] for v in series.values())
    end = min(v[0][-1] for v in series.values())
    if end <= start:
        raise ValueError('No common time interval for all four state topics')
    duration = (end-start)*1e-9
    grid = np.arange(0., duration, r['rate']/r['command_hz'])
    grid = np.r_[grid, duration]  # include exact final common endpoint
    parts, projected, max_projection = [], {}, {}
    offset = 0
    for key in KEYS:
        stamps, values = series[key]
        values = np.asarray(values)
        n = values.shape[1]
        low, high, tolerance = guard.low[offset:offset+n], guard.high[offset:offset+n], guard.tolerance[offset:offset+n]
        if np.any((values < low-tolerance) | (values > high+tolerance)):
            raise ValueError(key + ': recorded feedback exceeds physical range plus configured tolerance')
        clipped = np.clip(values, low, high)
        projected[key] = int(np.sum(np.any(clipped != values, axis=1)))
        max_projection[key] = float(np.max(np.abs(clipped-values)))
        relative = (np.asarray(stamps, dtype=np.int64)-start)*1e-9
        parts.append(np.column_stack([np.interp(grid, relative, clipped[:, i]) for i in range(n)]))
        offset += n
    points = np.concatenate(parts, axis=1)
    summary = dict(bag=str(path), size_bytes=identity[0], mtime_ns=identity[1],
        source='follower_actual', topics=r['topics'], name=NAMES,
        timestamp_source='JointState.header.stamp', interpolation='linear_without_angle_wrapping',
        common_start_ns=int(start), common_end_ns=int(end), source_duration_sec=duration,
        nominal_playback_sec=duration/r['rate'], rate=r['rate'], command_hz=r['command_hz'],
        points=len(points), source_counts={k:len(v[0]) for k,v in series.items()},
        trimmed_bag_start_sec=start*1e-9-bag_start, trimmed_bag_end_sec=bag_end-end*1e-9,
        projected_source_samples=projected, max_projection_per_source=max_projection,
        first=points[0].tolist(), last=points[-1].tolist())
    return grid, points, summary


def check_start(cfg, target, actual):
    r = cfg['replay']
    guard = ActionGuard(cfg['control'], 1/r['command_hz'])
    actual = guard.bounded(actual)
    limits = np.tile([r['start_tolerance_rad']]*6+[r['start_tolerance_m']], 2)
    bad = np.flatnonzero(np.abs(target-actual) > limits)
    if len(bad):
        detail = ', '.join('{}: bag={:.4f}, actual={:.4f}, limit={:.4f}'.format(
            NAMES[i], target[i], actual[i], limits[i]) for i in bad)
        raise ValueError('Replay start pose mismatch; manually prepare the recorded start pose: ' + detail)


def move_to_start(cfg, target, controller, observations, *, clock=time.monotonic,
                  sleep=time.sleep, cancelled=lambda: False, report=lambda _: None):
    """Joint-space approach using live feedback; no Cartesian collision planning."""
    r = cfg['replay']
    guard = controller.guard
    start = guard.bounded(observations.actual())
    target = guard.bounded(target)
    old_speed = guard.speed.copy()
    speed = np.minimum(old_speed, np.tile([r.get('approach_joint_speed_rad_s', .20)]*6+
                                         [r.get('approach_gripper_speed_m_s', .01)], 2))
    duration = float(np.max(np.abs(target-start)/speed))
    count = max(1, int(math.ceil(duration*r['command_hz'])))
    points = start+(target-start)*np.linspace(0., 1., count+1)[:, None]
    grid = np.linspace(0., duration, count+1)
    began = clock()
    def stop_check():
        if cancelled():
            return True
        if clock()-began > r.get('approach_timeout_sec', 30.):
            raise RuntimeError('Move to bag start timed out; replay not started')
        return False
    try:
        guard.speed = speed
        result = play_trajectory(cfg, grid, points, controller, observations,
                                 clock=clock, sleep=sleep, cancelled=stop_check, report=report)
        check_start(cfg, target, observations.actual())
        return result
    finally:
        guard.speed = old_speed


class SDKReplayFeedback:
    """Use live SDK state, never bag-republished ROS JointState as actual pose."""
    def __init__(self, cfg, hardware):
        self.cfg, self.hardware, self.controller = cfg, hardware, None

    def deployment_hardware(self):
        self.hardware.deployment_hardware()

    def fresh(self, msg, label, max_age):
        if msg is None or not np.isfinite(msg.timestamp) or not -.05 <= time.time()-msg.timestamp <= max_age:
            raise RuntimeError(label + ': missing/stale SDK feedback')
        return msg.msg

    def actual(self, status=True):
        c = self.controller
        if c is None or set(c.arms) != set(SIDES) or set(c.grippers) != set(SIDES):
            raise RuntimeError('SDK feedback not connected')
        values = []
        for side in SIDES:
            arm, grip = c.arms[side], c.grippers[side]
            if status:
                state = self.fresh(arm.get_arm_status(), side+' status', self.cfg['observations']['status_age_sec'])
                if state.arm_status != 0 or state.err_code != 0:
                    raise RuntimeError(side + ': SDK arm fault')
            joints = np.asarray(self.fresh(arm.get_joint_angles(), side+' joints', self.cfg['observations']['state_age_sec']), dtype=float).copy()
            gripper = self.fresh(grip.get_gripper_status(), side+' gripper', self.cfg['observations']['state_age_sec'])
            if gripper.mode != 'width':
                raise RuntimeError(side + ': SDK gripper feedback is not width in meters')
            if status:
                for key in ('voltage_too_low', 'motor_overheating', 'driver_overcurrent',
                            'driver_overheating', 'sensor_status', 'driver_error_status'):
                    if getattr(gripper.foc_status, key):
                        raise RuntimeError(side + ': gripper fault: ' + key)
            if joints.shape != (6,):
                raise RuntimeError(side + ': invalid SDK joint dimension')
            values.extend(joints.tolist()+[float(gripper.value)])
        if not np.isfinite(values).all():
            raise RuntimeError('Nonfinite SDK feedback')
        return np.asarray(values)


def play_trajectory(cfg, grid, points, controller, observations, *, clock=time.monotonic,
                    sleep=time.sleep, cancelled=lambda: False, report=lambda _: None):
    r = cfg['replay']
    period = 1/r['command_hz']
    tolerance = np.tile([r['tracking_tolerance_rad']]*6+[r['tracking_tolerance_m']], 2)
    started = clock()
    for index, target in enumerate(points):
        began, stable_since = clock(), None
        while True:
            tick = clock()
            if cancelled():
                raise InterruptedError('Replay cancelled')
            controller.check_fault()
            actual = observations.actual()
            if tick-began > r['waypoint_timeout_sec']:
                raise RuntimeError('Replay waypoint timed out at bag time {:.3f}s; motion blocked or not tracking'.format(grid[index]))
            sent = controller.send(target)
            reached = np.all(np.abs(sent-target) < 1e-6) and np.all(np.abs(actual-target) <= tolerance)
            report(dict(index=index, source_time_sec=float(grid[index]), elapsed_sec=tick-started,
                        target=target.tolist(), commanded=np.asarray(sent).tolist(), actual=actual.tolist()))
            done = False
            if reached:
                if index < len(points)-1:
                    done = True
                else:
                    stable_since = tick if stable_since is None else stable_since
                    done = tick-stable_since >= r['final_settle_sec']
            else:
                stable_since = None
            # No catch-up bursts after a delay. Each trajectory point is visited.
            sleep(max(0., period-(clock()-tick)))
            if done:
                break
    controller.check_fault()
    return dict(state='complete', elapsed_sec=clock()-started, source_duration_sec=float(grid[-1]),
                points=len(points), final_actual=observations.actual().tolist())
