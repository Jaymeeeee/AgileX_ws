"""Four independent CAN arms: configuration and ROS command validation."""
import copy
import math
import threading
import time
from pathlib import Path

import numpy as np
import yaml

from hardware_bringup.config import load_config as load_hardware
from .core import ActionGuard, arm_sides, load_config as load_deployment


def load_teleop(path, arms=None):
    path = Path(path).expanduser().resolve()
    with path.open() as stream:
        cfg = yaml.safe_load(stream)
    if cfg.get('schema_version') != 1:
        raise ValueError('Expected teleop schema_version: 1')
    sides = arm_sides(cfg.get('active_arms') if arms is None else arms)
    cfg['active_arms'] = list(sides)
    for key in ('hardware_config', 'deployment_config'):
        cfg[key] = str((path.parent / Path(cfg[key]).expanduser()).resolve())
    cfg['hardware'] = load_hardware(cfg['hardware_config'])
    cfg['deployment'] = load_deployment(cfg['deployment_config'], arms='both')
    if cfg['hardware'].get('topology') != 'independent_can':
        raise ValueError('Teleoperation requires independent_can topology')
    if not set(sides) <= set(cfg['hardware'].get('leaders', {})):
        raise ValueError('Selected independent leader channels are required')
    for key in ('fps', 'feedback_timeout_sec', 'command_timeout_sec', 'source_timeout_sec',
                'alignment_timeout_sec', 'alignment_joint_tolerance_rad',
                'alignment_gripper_tolerance_m', 'alignment_settle_sec',
                'joint_speed_rad_s', 'gripper_speed_m_s'):
        if isinstance(cfg[key], bool) or not isinstance(cfg[key], (int, float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('Invalid positive teleop setting: ' + key)
    if not isinstance(cfg['sdk_speed_percent'], int) or not 1 <= cfg['sdk_speed_percent'] <= 100:
        raise ValueError('Invalid SDK speed percent')
    if not isinstance(cfg['alignment_sdk_speed_percent'], int) or not 1 <= cfg['alignment_sdk_speed_percent'] <= 100:
        raise ValueError('Invalid alignment SDK speed percent')
    if not isinstance(cfg['limit_follow_speed'], bool):
        raise ValueError('limit_follow_speed must be boolean')
    if cfg['alignment_waypoint_rad'] is not None:
        waypoint = np.asarray(cfg['alignment_waypoint_rad'], dtype=float)
        if waypoint.shape != (6,) or not np.isfinite(waypoint).all():
            raise ValueError('Alignment waypoint must contain six finite joint angles')
    for side in sides:
        if cfg['deployment']['control']['channels'][side] != cfg['hardware']['arms'][side]['channel']:
            raise ValueError('Deployment and follower channel mappings differ')
    return cfg


def arm_guard(cfg):
    control = copy.deepcopy(cfg['deployment']['control'])
    control.update(max_joint_speed_rad_s=cfg['joint_speed_rad_s'],
                   max_gripper_speed_m_s=cfg['gripper_speed_m_s'],
                   check_initial_target_error=False, check_runtime_target_error=False)
    # Reuse the existing 14D guard by repeating a single arm's 7D vector.
    return ActionGuard(control, 1.0 / cfg['fps'])


def names(side, role):
    return [side + '_' + role + '_joint_' + str(i) for i in range(1, 7)] + [side + '_' + role + '_gripper_width']


def command_values(msg, side):
    expected = names(side, 'leader_target')
    if list(msg.name) != expected or len(msg.position) != 7:
        raise ValueError('Wrong leader side/order or command dimension')
    result = np.asarray(msg.position, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('Non-finite leader command')
    stamp = msg.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0 or stamp - time.time() > 0.1:
        raise ValueError('Invalid leader source timestamp')
    return result


class CommandInbox:
    """Separate command value lifetime from independently checked source health.

    Static native leader targets can be event-like. Repeated publications retain
    their original timestamp; a genuine CAN reply is required for source health.
    """
    def __init__(self, side, timeout):
        self.side, self.timeout = side, timeout
        self.lock = threading.Lock()
        self.command = None
        self.source = None
        self.error = None

    def put_command(self, msg, caller=None):
        value = command_values(msg, self.side)
        with self.lock:
            if self.command is not None and msg.header.stamp.to_sec() < self.command[1]:
                raise ValueError('Leader command timestamp went backwards')
            self.command = (value, msg.header.stamp.to_sec(), time.monotonic(), caller,
                            msg.header.frame_id == 'native_gripper')

    def put_source(self, data, caller=None):
        age = data.get('can_age_sec')
        if data.get('side') != self.side or data.get('role') != 'leader':
            raise ValueError('Leader status identity mismatch')
        if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
            raise ValueError('Invalid leader CAN health')
        with self.lock:
            self.source = (data, time.monotonic(), caller)

    def latest(self, with_provenance=False):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.command is None or self.source is None:
                raise RuntimeError('Waiting for aligned leader input')
            data, received, caller = self.source
            now = time.monotonic()
            if now-received > self.timeout or now-self.command[2] > self.timeout:
                raise RuntimeError('Leader ROS input stale')
            if data.get('phase') != 'teaching' or data['can_age_sec'] + now-received > self.timeout:
                raise RuntimeError('Leader is not healthy and teaching')
            if caller and self.command[3] and caller != self.command[3]:
                raise RuntimeError('Leader command and health have different publishers')
            value = self.command[0].copy()
            return (value, self.command[4]) if with_provenance else value


class GripperTakeover:
    """Hold an open follower until the native hand input picks up its value.

    Entering teaching can reset the leader's gripper command to zero even when
    its physical gripper was aligned open. An aligned synthetic seed must not
    count as a native hand input, or the subsequent zero would snap it closed.
    """
    def __init__(self, held_width, tolerance):
        self.held_width, self.tolerance = float(held_width), float(tolerance)
        self.previous = None
        self.ready = False

    def update(self, width, native):
        width = float(width)
        if not math.isfinite(width):
            raise ValueError('Non-finite gripper input')
        if self.ready:
            return width
        if native:
            crossed = self.previous is not None and min(self.previous, width) <= self.held_width <= max(self.previous, width)
            if abs(width-self.held_width) <= self.tolerance or crossed:
                self.ready = True
            self.previous = width
        return width if self.ready else self.held_width
