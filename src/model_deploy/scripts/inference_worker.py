#!/usr/bin/env python3
"""Run in the isolated LeRobot environment. This process never imports ROS/SDK."""
import argparse
import json
from pathlib import Path
import socket
import sys
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from model_deploy.core import CAMERAS, selected_sides, ActionGuard, load_config, recv_packet, send_packet


def validate_checkpoint_contract(cfg, policy_config) -> int:
    dimension = 7*len(selected_sides(cfg))
    expected = {'observation.state': [dimension], **{
        'observation.images.' + k: [3, 480, 640] for k in CAMERAS}}
    if policy_config.type != 'act' or {k: list(v.shape) for k, v in policy_config.input_features.items()} != expected:
        raise ValueError('Checkpoint input contract does not match selected arms {} ({}D state, three RGB cameras); '
                         'use matching single/dual-arm weights, no automatic slicing'.format(selected_sides(cfg), dimension))
    if list(policy_config.output_features['action'].shape) != [dimension]:
        raise ValueError('Checkpoint action must be absolute {}D Piper targets for {}'.format(dimension, selected_sides(cfg)))
    return dimension


class Inference:
    def __init__(self, cfg):
        import torch
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies.act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        self.torch = torch
        self.cfg = cfg
        torch.set_num_threads(4)
        if cfg['policy']['device'] == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable; no silent CPU fallback')
        path = cfg['checkpoint']
        pc = PreTrainedConfig.from_pretrained(path, local_files_only=True)
        self.dimension = validate_checkpoint_contract(cfg, pc)
        pc.device = cfg['policy']['device']
        pc.pretrained_backbone_weights = None  # full trained weights below; no downloads
        pc.n_action_steps = 1
        pc.temporal_ensemble_coeff = cfg['policy']['temporal_ensemble_coeff']
        self.policy = ACTPolicy.from_pretrained(path, config=pc, local_files_only=True, strict=True)
        self.policy.to(pc.device).eval()
        self.pre, self.post = make_pre_post_processors(
            pc, pretrained_path=path,
            preprocessor_overrides={'device_processor': {'device': pc.device}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'}})
        dummy = {'state': np.zeros(self.dimension, np.float32), **{
            k: np.zeros((480, 640, 3), np.uint8) for k in CAMERAS}}
        for _ in range(3):
            self.run(dummy)
        self.policy.reset()

    def run(self, packet):
        torch = self.torch
        state = np.asarray(packet['state'])
        if state.shape != (self.dimension,) or not np.isfinite(state).all():
            raise ValueError('Invalid observation.state')
        obs = {'observation.state': torch.from_numpy(state.astype(np.float32))}
        for k in CAMERAS:
            a = packet[k]
            if a.shape != (480, 640, 3) or a.dtype != np.uint8:
                raise ValueError('Invalid RGB array: ' + k)
            obs['observation.images.'+k] = torch.from_numpy(a.copy()).permute(2, 0, 1).float()/255.0
        start = time.monotonic()
        with torch.inference_mode():
            action = self.post(self.policy.select_action(self.pre(obs)))
        result = action.detach().cpu().numpy().reshape(-1)
        if result.shape != (self.dimension,) or not np.isfinite(result).all():
            raise ValueError('Invalid model output')
        return result, time.monotonic()-start


def offline(engine, root, count, fixture):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = Path(root).expanduser().resolve()
    info = json.loads((root/'meta/info.json').read_text())
    if info['fps'] != 15:
        raise ValueError('Offline dataset FPS mismatch')
    ds = LeRobotDataset('local/piper_insert_plug', root=root, video_backend='torchcodec')
    times, errors, ranges = [], [], []
    guard = ActionGuard(engine.cfg['control'], 1/15, selected_sides(engine.cfg))
    engine.policy.reset()
    for i in range(min(count, len(ds))):
        sample = ds[i]
        packet = {'state': sample['observation.state'].numpy(), **{
            k: (sample['observation.images.'+k].permute(1, 2, 0).numpy()*255).round().clip(0, 255).astype(np.uint8)
            for k in CAMERAS}}
        if i == 0 and fixture:
            np.savez(Path(fixture).expanduser(), **packet)
        out, elapsed = engine.run(packet)
        times.append(elapsed)
        errors.append(float(np.max(np.abs(out-packet['state']))))
        try:
            guard.prepare(out, packet['state'])
        except ValueError as exc:
            ranges.append({'frame': i, 'reason': str(exc), 'predicted': out.tolist()})
    report = {'frames': len(times), 'checkpoint': engine.cfg['checkpoint'],
              'device': engine.cfg['policy']['device'], 'inference_median_ms': float(np.median(times)*1000),
              'inference_p95_ms': float(np.percentile(times, 95)*1000),
              'max_action_state_abs_error_mixed_units': max(errors),
              'guard_rejections': ranges, 'can_commands_sent': 0}
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', required=True)
    p.add_argument('--socket')
    p.add_argument('--offline')
    p.add_argument('--frames', type=int, default=30)
    p.add_argument('--fixture', help='Save first offline observation for ROS transport testing')
    p.add_argument('--arms', choices=('both', 'left', 'right'))
    p.add_argument('--checkpoint')
    args = p.parse_args()
    cfg = load_config(args.config, arms=args.arms, checkpoint=args.checkpoint)
    engine = Inference(cfg)
    if args.offline:
        if args.frames < 1:
            p.error('frames must be positive')
        return offline(engine, args.offline, args.frames, args.fixture)
    if not args.socket:
        p.error('--socket or --offline is required')
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.connect(args.socket)
        send_packet(sock, ready=np.array([1]))
        while True:
            try:
                packet = recv_packet(sock)
            except ConnectionError:
                break
            if 'reset' in packet:
                engine.policy.reset()
                send_packet(sock, ready=np.array([1]))
                continue
            result, elapsed = engine.run(packet)
            send_packet(sock, action=result, inference_sec=np.array([elapsed]), seq=packet['seq'])


if __name__ == '__main__':
    main()
