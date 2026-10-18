"""Fixed reset reference, with explicit optional deployment-log overrides."""
import json
import os
from pathlib import Path
import tempfile
import time
import numpy as np
from .core import ActionGuard, NAMES


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.'+path.name, dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def save_start_pose(cfg, run_dir, actual):
    pose = ActionGuard(cfg['control'], 1/cfg['policy']['fps']).bounded(actual)
    value = dict(schema_version=1, source='measured_before_enable_and_model_commands',
                 estimated=False, stamp=time.time(), channels=cfg['control']['channels'],
                 run_dir=str(Path(run_dir).resolve()), name=NAMES, position=pose.tolist())
    atomic_json(Path(run_dir)/'start_pose.json', value)
    atomic_json(Path(cfg['log_dir'])/'latest_start_pose.json', value)
    return value


def load_start_pose(cfg, from_run=None):
    root = Path(from_run).expanduser().resolve() if from_run else None
    path = root/'start_pose.json' if root else Path(cfg['log_dir'])/'latest_start_pose.json'
    if path.exists():
        value = json.loads(path.read_text())
    elif root is not None:
        # Explicit --from-run only. Never silently substitute a dry-run or some
        # other episode when the current session has no precise start snapshot.
        run_cfg = json.loads((root/'config.json').read_text())
        with (root/'actions.jsonl').open() as f:
            row = json.loads(next(f))
        if row.get('seq') != 0 or row.get('commanded') is None:
            raise ValueError('Legacy reset reference must be the first EXECUTE action, not dry-run')
        value = dict(schema_version=1, source='legacy_first_observation_estimate', estimated=True,
                     stamp=row['observation_stamp'], channels=run_cfg['control']['channels'],
                     run_dir=str(root), name=NAMES, position=row['state'])
    else:
        raise ValueError('No saved start pose. New --execute runs save it automatically. '
                         'For an older run, explicitly use --from-run RUN_DIR; '
                         'its first observation is only an approximate starting pose.')
    if value.get('schema_version') != 1 or value.get('name') != NAMES:
        raise ValueError('Invalid reset pose schema/joint order')
    if value['channels'] != cfg['control']['channels']:
        raise ValueError('Saved reset CAN mapping differs from current configuration')
    pose = ActionGuard(cfg['control'], 1/cfg['policy']['fps']).bounded(value['position'])
    # Preserve original reference and provenance; final gripper width is defined
    # by the reset sequence, not the saved open width.
    return pose, value


def load_reset_pose(cfg, from_run=None):
    if from_run is not None:
        return load_start_pose(cfg, from_run)
    joints = cfg['reset'].get('home_joints_rad')
    if not isinstance(joints, dict):
        raise ValueError('Set reset.home_joints_rad for the fixed reset pose, or use --from-run')
    position = []
    for side in ('left', 'right'):
        values = np.asarray(joints.get(side), dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('reset.home_joints_rad.' + side + ' must contain six finite joint angles')
        position.extend(values.tolist())
        position.append(cfg['reset']['gripper_closed_m'])
    guard = ActionGuard(cfg['control'], 1/cfg['policy']['fps'])
    # A configured command must be inside limits, not merely within feedback tolerance.
    if np.any(np.asarray(position) < guard.low) or np.any(np.asarray(position) > guard.high):
        raise ValueError('Fixed reset pose exceeds configured joint/gripper limits')
    pose = guard.bounded(position)
    return pose, dict(schema_version=1, source='configured_fixed_pose', estimated=False,
                      channels=cfg['control']['channels'], name=NAMES, position=pose.tolist())
