"""Pure Python/numpy validation and bounded wire protocol (Python 3.8/3.12)."""
import io
import math
import socket
import struct
from pathlib import Path
import numpy as np
import yaml

CAMERAS = ('cam_h', 'cam_l', 'cam_r')
SIDES = ('left', 'right')
NAMES = [n for s in SIDES for n in
         [s + '_joint_' + str(i) for i in range(1, 7)] + [s + '_gripper_width']]


def arm_sides(value=None) -> tuple:
    """Canonical physical arm selection; never infer a side from vector length."""
    if value is None or value in ('both', 'all'):
        return SIDES
    if isinstance(value, str):
        if value not in SIDES:
            raise ValueError('arms must be both, left or right')
        return (value,)
    if not isinstance(value, (list, tuple)) or not value or len(set(value)) != len(value) or any(s not in SIDES for s in value):
        raise ValueError('active_arms must contain left and/or right without duplicates')
    return tuple(s for s in SIDES if s in value)


def selected_sides(cfg) -> tuple:
    return arm_sides(cfg.get('active_arms'))


def action_names(sides=SIDES) -> list:
    return [n for s in sides for n in
            [s + '_joint_' + str(i) for i in range(1, 7)] + [s + '_gripper_width']]


def load_config(path, arms=None, checkpoint=None):
    with open(Path(path).expanduser()) as f:
        c = yaml.safe_load(f)
    if c['schema_version'] != 1:
        raise ValueError('Unsupported deployment schema')
    sides = arm_sides(c.get('active_arms') if arms is None else arms)
    c['active_arms'] = list(sides)
    for mapping in (c['observations']['arms'], c['control']['channels'], c['control']['expected_firmware']):
        if not set(sides) <= set(mapping):
            raise ValueError('Selected arm missing from deployment configuration')
    c['observations']['arms'] = {s: c['observations']['arms'][s] for s in sides}
    for key in ('channels', 'expected_firmware'):
        c['control'][key] = {s: c['control'][key][s] for s in sides}
    if checkpoint is not None:
        c['checkpoint'] = checkpoint
    for section in ('policy', 'observations', 'control'):
        for k, v in c[section].items():
            if k.endswith('_sec') or k in ('fps', 'width', 'height', 'bitrate') or k.startswith('max_'):
                if not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0:
                    raise ValueError('Invalid positive setting: ' + k)
    if c['policy']['fps'] != 15 or (c['policy']['height'], c['policy']['width']) != (480, 640):
        raise ValueError('This trained policy requires 15 Hz, 640x480 RGB')
    if c['policy']['device'] not in ('cpu', 'cuda'):
        raise ValueError('device must be cpu or cuda')
    if set(c['observations']['cameras']) != set(CAMERAS):
        raise ValueError('Expected cam_h/l/r cameras')
    if len(set(c['control']['channels'].values())) != len(sides):
        raise ValueError('Expected distinct CAN channels for selected arms')
    if not 1 <= c['control']['sdk_speed_percent'] <= 100:
        raise ValueError('Invalid SDK speed percent')
    if not math.isfinite(c['policy']['temporal_ensemble_coeff']):
        raise ValueError('Invalid temporal ensemble coefficient')
    for k in ('checkpoint', 'python', 'log_dir'):
        c[k] = str(Path(c[k]).expanduser().resolve())
    ActionGuard(c['control'], 1.0 / c['policy']['fps'], sides)
    return c


def joint_values(msg, names):
    if len(msg.name) != len(set(msg.name)) or len(msg.position) != len(msg.name):
        raise ValueError('Malformed JointState names/position')
    mapping = dict(zip(msg.name, msg.position))
    v = np.asarray([mapping[n] for n in names], dtype=np.float64)
    if not np.isfinite(v).all():
        raise ValueError('Non-finite joint feedback')
    return v


def rgb_array(msg, height, width):
    if (msg.height, msg.width) != (height, width) or msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('Image must be 640x480 rgb8/bgr8')
    if msg.step < width * 3 or len(msg.data) != height * msg.step:
        raise ValueError('Invalid image stride/data')
    a = np.frombuffer(msg.data, np.uint8).reshape(height, msg.step)[:, :width*3].reshape(height, width, 3)
    if msg.encoding == 'bgr8':
        a = a[:, :, ::-1]
    return a.copy()


class ActionGuard:
    def __init__(self, cfg, period, sides=SIDES):
        self.cfg, self.period = cfg, period
        self.sides = arm_sides(sides)
        count = len(self.sides)
        self.names = action_names(self.sides)
        self.dimension = 7*count
        self.gripper_indices = np.arange(6, self.dimension, 7)
        low = np.asarray(cfg['joint_min'], dtype=float)
        high = np.asarray(cfg['joint_max'], dtype=float)
        if low.shape != (6,) or high.shape != (6,) or not np.isfinite([low, high]).all() or np.any(low >= high):
            raise ValueError('Invalid joint limits')
        if not 0 <= cfg['gripper_min'] < cfg['gripper_max'] <= 0.1:
            raise ValueError('Invalid gripper range')
        for k in ('boundary_tolerance_rad', 'boundary_tolerance_m', 'gripper_force'):
            if not math.isfinite(cfg[k]) or cfg[k] < 0:
                raise ValueError('Invalid ' + k)
        self.low = np.tile(np.r_[low, cfg['gripper_min']], count)
        self.high = np.tile(np.r_[high, cfg['gripper_max']], count)
        self.tolerance = np.tile([cfg['boundary_tolerance_rad']]*6 + [cfg['boundary_tolerance_m']], count)
        self.max_error = np.tile([cfg['max_target_error_rad']]*6 + [cfg['max_target_error_m']], count)
        self.initial_error = np.tile([cfg.get('max_initial_error_rad', cfg['max_target_error_rad'])]*6
                                     + [cfg.get('max_initial_error_m', cfg['max_target_error_m'])], count)
        self.tracking_error = np.tile([cfg.get('max_tracking_error_rad', cfg['max_target_error_rad'])]*6
                                      + [cfg.get('max_tracking_error_m', cfg['max_target_error_m'])], count)
        self.speed = np.tile([cfg['max_joint_speed_rad_s']]*6 + [cfg['max_gripper_speed_m_s']], count)
        for key in ('check_initial_target_error', 'check_runtime_target_error', 'check_gripper_tracking_error'):
            if not isinstance(cfg.get(key, True), bool):
                raise ValueError(key + ' must be a YAML boolean (true/false)')
        self.check_initial_target_error = cfg.get('check_initial_target_error', True)
        self.check_runtime_target_error = cfg.get('check_runtime_target_error', True)
        self.check_gripper_tracking_error = cfg.get('check_gripper_tracking_error', True)
        self.previous = None

    def bounded(self, value):
        value = np.asarray(value, dtype=float)
        if value.shape != (self.dimension,) or not np.isfinite(value).all():
            raise ValueError('Expected finite {}D state/action'.format(self.dimension))
        invalid = (value < self.low-self.tolerance) | (value > self.high+self.tolerance)
        if np.any(invalid):
            detail = ', '.join('{}={:.5f} range=[{:.5f},{:.5f}]'.format(
                self.names[i], value[i], self.low[i], self.high[i]) for i in np.flatnonzero(invalid))
            raise ValueError('State/action exceeds configured limits: ' + detail)
        return np.clip(value, self.low, self.high)

    def prepare(self, target, actual, initial=False):
        target = np.asarray(target, dtype=float)
        if target.shape != (self.dimension,) or not np.isfinite(target).all():
            raise ValueError('Expected finite {}D action'.format(self.dimension))
        actual = self.bounded(actual)
        # Regression can predict just beyond an endpoint (e.g. negative width).
        # Optional model-distance gate; wire commands always retain rate/range/tracking limits.
        error_limit = self.initial_error if initial else self.max_error
        check_error = self.check_initial_target_error if initial else self.check_runtime_target_error
        if check_error and np.any(np.abs(target-actual) > error_limit):
            bad = np.flatnonzero(np.abs(target-actual) > error_limit)
            detail = ', '.join('{}: target={:.4f}, actual={:.4f}, limit={:.4f}'.format(
                self.names[i], target[i], actual[i], error_limit[i]) for i in bad)
            raise ValueError(('Initial model target' if initial else 'Model target') + ' too far from actual pose: ' + detail)
        target = np.clip(target, self.low, self.high)
        previous = actual if self.previous is None else self.previous
        tracking_violation = np.abs(previous-actual) > self.tracking_error
        if not self.check_gripper_tracking_error:
            tracking_violation[self.gripper_indices] = False
        if np.any(tracking_violation):
            bad = np.flatnonzero(tracking_violation)
            detail = ', '.join('{}: commanded={:.4f}, actual={:.4f}, error={:.4f}, limit={:.4f} {}'.format(
                self.names[i], previous[i], actual[i], abs(previous[i]-actual[i]), self.tracking_error[i],
                'm' if i % 7 == 6 else 'rad') for i in bad)
            raise ValueError('Follower is not tracking commanded pose: ' + detail)
        # Never allow a larger jump after a delayed cycle.
        sent = previous + np.clip(target-previous, -self.speed*self.period, self.speed*self.period)
        # A relaxed model horizon must not let issued commands run away from feedback.
        sent = np.clip(sent, np.maximum(self.low, actual-self.tracking_error),
                       np.minimum(self.high, actual+self.tracking_error))
        if not self.check_gripper_tracking_error:
            # If feedback suddenly moves far from the prior gripper command,
            # approach the feedback envelope at the configured rate, never jump to it.
            indices = self.gripper_indices
            step = self.speed[indices]*self.period
            sent[indices] = np.clip(sent[indices], previous[indices]-step, previous[indices]+step)
        return sent

    def commit(self, sent):
        self.previous = np.asarray(sent).copy()


def send_packet(sock, **arrays):
    buf = io.BytesIO()
    np.savez(buf, **arrays)
    data = buf.getvalue()
    if len(data) > 16*1024*1024:
        raise ValueError('IPC message too large')
    sock.sendall(struct.pack('!I', len(data)) + data)


def recv_packet(sock):
    def exact(n):
        data = bytearray()
        while len(data) < n:
            part = sock.recv(n-len(data))
            if not part:
                raise ConnectionError('Inference process disconnected')
            data.extend(part)
        return bytes(data)
    size = struct.unpack('!I', exact(4))[0]
    if not 0 < size <= 16*1024*1024:
        raise ValueError('Invalid IPC size')
    with np.load(io.BytesIO(exact(size)), allow_pickle=False) as z:
        return {k: z[k] for k in z.files}
