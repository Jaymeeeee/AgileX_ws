import copy
import math
import re
import subprocess
from pathlib import Path
import yaml


def load_config(path):
    with open(Path(path).expanduser()) as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict) or cfg.get('schema_version') != 1:
        raise ValueError('Expected hardware schema_version: 1')
    seen_serials, seen_channels = set(), set()
    for name, item in cfg['cameras'].items():
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('Invalid camera namespace: ' + name)
        serial = item['serial']
        if not isinstance(serial, str) or not re.fullmatch(r'[0-9]+', serial) or serial in seen_serials:
            raise ValueError('Camera serials must be unique quoted digit strings')
        seen_serials.add(serial)
    for side, item in cfg['arms'].items():
        if side not in ('left', 'right') or not re.fullmatch(r'can[0-9]+', item['channel']):
            raise ValueError('Invalid arm side or CAN channel')
        if item['channel'] in seen_channels or item['parser'] not in ('default', 'v188'):
            raise ValueError('Duplicate CAN channel or unsupported parser')
        if item.get('model','unknown') not in ('unknown','piper','piper_h','piper_l','piper_x'):
            raise ValueError('Unsupported arm model for torque conversion')
        seen_channels.add(item['channel'])
        for key in ('command_id_offset', 'feedback_id_offset'):
            if not isinstance(item[key], int) or not 0 <= item[key] <= 0x100:
                raise ValueError('Invalid CAN ID offset')
        motor_offset=item.get('motor_id_offset',0)
        if not isinstance(motor_offset,int) or not 0 <= motor_offset <= 0x100:
            raise ValueError('Invalid motor CAN ID offset')
    for section, keys in {'camera_settings': ['width', 'height', 'fps', 'startup_timeout_sec'],
                          'arm_settings': ['bitrate', 'assembly_span_sec', 'max_sample_age_sec', 'can_recheck_sec', 'startup_timeout_sec'],
                          'monitor': ['stale_sec', 'minimum_samples']}.items():
        for key in keys:
            value = float(cfg[section][key])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(section + '.' + key + ' must be positive')
    if not cfg['camera_settings'].get('align_depth'):
        raise ValueError('This topic contract requires align_depth: true')
    if cfg['camera_settings']['stagger_sec'] < 0 or cfg['monitor']['leader_target_timeout_sec'] < 0:
        raise ValueError('Timeouts/delays cannot be negative')
    return cfg


def select(value, available):
    names = list(available) if value == 'all' else ([] if value in ('none', '') else value.split(','))
    if len(set(names)) != len(names) or any(n not in available for n in names):
        raise ValueError('Invalid selection {!r}; choices {}'.format(value, list(available)))
    return names


def topic_specs(cfg, arms, cameras):
    specs = []
    def add(topic, kind, meaning, group, timeout, count=3, **extra):
        specs.append(dict(topic=topic, kind=kind, meaning=meaning, group=group,
                          timeout=timeout, minimum_samples=count, **extra))
    stale = cfg['monitor']['stale_sec']
    count = int(cfg['monitor']['minimum_samples'])
    for side in arms:
        prefix = '/piper/{}_bus'.format(side)
        group = 'arm:' + side
        for suffix, size, meaning, timeout in [
            ('joint_states', 6, '从臂六关节实际角度 [rad]', stale),
            ('gripper/actual_stamped', 1, '从臂夹爪实际开口 [m]', stale),
            ('leader_targets', 6, '主臂目标角度回显 [rad]，非独立实测反馈', cfg['monitor']['leader_target_timeout_sec']),
            ('gripper/target_stamped', 1, '主臂夹爪目标开口回显 [m]', cfg['monitor']['leader_target_timeout_sec'])]:
            add(prefix+'/'+suffix, 'joint', meaning, group, timeout,
                count if timeout else 1, size=size)
        add(prefix+'/status', 'status', '从臂控制模式、故障码与状态', group, stale, count)
        add(prefix+'/gripper/status', 'gripper_status', '从臂夹爪状态位与原始协议力字段', group, stale, count)
        add(prefix+'/motor_feedback', 'motor_feedback', '六电机高速反馈：电流、轴状态及按型号换算的力矩估计',
            group, stale, count, model=cfg['arms'][side].get('model','unknown'))
    for name in cameras:
        group = 'camera:' + name
        for stream, image, encoding, meaning in [
            ('color', 'image_raw', 'rgb8', 'RGB 彩色图'),
            ('depth', 'image_rect_raw', '16UC1', '原始深度图 [mm], 0 无效'),
            ('aligned_depth_to_color', 'image_raw', '16UC1', '对齐彩色的深度 [mm], 0 无效')]:
            add('/'+name+'/'+stream+'/'+image, 'image', meaning, group, stale, count,
                encoding=encoding, width=cfg['camera_settings']['width'], height=cfg['camera_settings']['height'])
    return specs


def camera_command(cfg, name):
    c = cfg['camera_settings']
    args = dict(camera=name, serial_no=cfg['cameras'][name]['serial'],
                width=c['width'], height=c['height'], fps=c['fps'])
    def value(x):
        return str(x).lower() if isinstance(x, bool) else str(x)
    return ['roslaunch', 'hardware_bringup', 'camera.launch'] + [k+':='+value(v) for k,v in args.items()]


def check_can(channel, bitrate):
    output = subprocess.check_output(['ip', '-details', 'link', 'show', channel],
                                     stderr=subprocess.STDOUT, text=True, timeout=3)
    flags = re.search(r'<([^>]+)>', output.splitlines()[0])
    if not flags or 'UP' not in flags.group(1).split(','):
        raise RuntimeError(channel + ' is DOWN; run prepare_can.sh in a local terminal')
    if 'LISTEN-ONLY' not in output.upper():
        raise RuntimeError(channel + ' must be LISTEN-ONLY for receive-only acquisition')
    if not re.search(r'\bbitrate\s+{}\b'.format(int(bitrate)), output):
        raise RuntimeError(channel + ' bitrate mismatch')
    if 'BUS-OFF' in output or 'ERROR-PASSIVE' in output:
        raise RuntimeError(channel + ' reports a CAN transport error')
    return output
