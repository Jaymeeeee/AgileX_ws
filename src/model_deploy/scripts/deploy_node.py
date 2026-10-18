#!/usr/bin/python3
"""ROS1 parent process: live observations, inference IPC and SDK command gate."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from model_deploy.core import NAMES, ActionGuard, load_config, recv_packet, send_packet
from model_deploy.observations import Observations

PACKAGE = Path(__file__).resolve().parents[1]


def inference_env(python):
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    prefix = Path(python).parent.parent
    env.update(PYTHONNOUSERSITE='1', LD_LIBRARY_PATH=str(prefix/'lib'),
               PATH=str(prefix/'bin')+':/usr/bin:/bin', HF_HUB_OFFLINE='1',
               HF_DATASETS_OFFLINE='1', TOKENIZERS_PARALLELISM='false', OMP_NUM_THREADS='4')
    return env


class Worker:
    def __init__(self, cfg, config_path, log_dir):
        self.cfg = cfg
        self.proc = self.sock = self.server = self.log = self.temp = None
        try:
            self.temp = tempfile.TemporaryDirectory(prefix='agilex-inference-')
            path = str(Path(self.temp.name)/'worker.sock')
            self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.server.bind(path)
            self.server.listen(1)
            self.server.settimeout(0.5)
            self.log = open(log_dir/'inference.log', 'w')
            self.proc = subprocess.Popen([cfg['python'], str(PACKAGE/'scripts/inference_worker.py'),
                '--config', str(config_path), '--socket', path], env=inference_env(cfg['python']),
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+cfg['policy']['startup_timeout_sec']
            while self.sock is None:
                if self.proc.poll() is not None:
                    raise RuntimeError('Inference worker failed; see ' + str(log_dir/'inference.log'))
                if time.monotonic() > deadline or rospy.is_shutdown():
                    raise RuntimeError('Inference worker startup timeout/shutdown')
                try:
                    self.sock, _ = self.server.accept()
                except socket.timeout:
                    continue
            self.sock.settimeout(cfg['policy']['inference_timeout_sec'])
            recv_packet(self.sock)
        except BaseException:
            self.close()
            raise

    def predict(self, packet, seq):
        send_packet(self.sock, **packet, seq=np.array([seq], np.int64))
        result = recv_packet(self.sock)
        if int(result['seq'][0]) != seq:
            raise RuntimeError('Inference sequence mismatch')
        return result['action'], float(result['inference_sec'][0])

    def reset(self):
        send_packet(self.sock, reset=np.array([1]))
        recv_packet(self.sock)

    def close(self):
        if self.sock:
            self.sock.close()
        if self.server:
            self.server.close()
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if self.log:
            self.log.close()
        if self.temp:
            self.temp.cleanup()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default=str(PACKAGE/'config/act.yaml'))
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    mode.add_argument('--probe-firmware', action='store_true', help='CAN query only; no enable or movement')
    p.add_argument('--duration', type=float, help='Stop after this many seconds; default from YAML')
    args = p.parse_args(rospy.myargv()[1:])
    cfg = load_config(args.config)
    duration = cfg['policy']['max_duration_sec'] if args.duration is None else args.duration
    if not np.isfinite(duration) or duration <= 0:
        p.error('duration must be positive')
    rospy.init_node('agilex_model_deploy', disable_signals=True)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Live deployment requires /use_sim_time=false')
    log_dir = Path(cfg['log_dir'])/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    log_dir.mkdir(parents=True)
    (log_dir/'config.json').write_text(json.dumps(cfg, ensure_ascii=False, indent=2))
    print('Mode: {} | log: {}'.format('EXECUTE' if args.execute else ('PROBE' if args.probe_firmware else 'DRY-RUN'), log_dir), flush=True)
    obs = Observations(cfg)
    raw_pub = rospy.Publisher('/agilex/deploy/predicted_action', JointState, queue_size=1)
    sent_pub = rospy.Publisher('/agilex/deploy/commanded_action', JointState, queue_size=1)
    status_pub = rospy.Publisher('/agilex/deploy/status', String, queue_size=1, latch=True)
    worker = controller = None
    mode_name = 'execute' if args.execute else 'dry_run'
    status_pub.publish(String(data=json.dumps({'state': 'starting', 'mode': mode_name})))
    failure = None
    try:
        if args.probe_firmware:
            from model_deploy.controller import Controller
            controller = Controller(cfg, obs)
            controller.connect()
            print(json.dumps(controller.firmware, ensure_ascii=False, indent=2), flush=True)
            (log_dir/'firmware.json').write_text(json.dumps(controller.firmware, indent=2))
            return
        worker = Worker(cfg, Path(args.config).expanduser().resolve(), log_dir)
        guard = ActionGuard(cfg['control'], 1/cfg['policy']['fps'])
        deadline = time.monotonic()+30
        while True:
            try:
                packet, anchor, stamps = obs.snapshot()
                if args.execute:
                    obs.deployment_hardware()
                break
            except RuntimeError as exc:
                if time.monotonic() > deadline or rospy.is_shutdown():
                    raise
                rospy.logwarn_throttle(3, str(exc))
                time.sleep(0.05)
        preview, _ = worker.predict(packet, -1)
        try:
            guard.prepare(preview, obs.actual(), initial=True)
            print('Action/feedback validation passed. Model-distance checks: initial={}, runtime={}; '
                  'command rate/range/tracking limits remain active.'.format(
                      guard.check_initial_target_error, guard.check_runtime_target_error), flush=True)
            print('Gripper tracking-error stop: {}; joint tracking-error stop remains active.'.format(
                guard.check_gripper_tracking_error), flush=True)
        except ValueError as exc:
            if args.execute:
                raise
            print('DRY-RUN initial guard: ' + str(exc), flush=True)
        if args.execute:
            if not sys.stdin.isatty():
                raise RuntimeError('--execute requires an interactive terminal; no unattended start')
            print('主臂必须已退出控制，双从臂已手动放好。按回车开始；Ctrl+C 取消。', flush=True)
            input()
            obs.deployment_hardware()
            from model_deploy.controller import Controller
            controller = Controller(cfg, obs)
            controller.connect()
            (log_dir/'firmware.json').write_text(json.dumps(controller.firmware, indent=2))
            packet, anchor, stamps = obs.snapshot()
            worker.reset()
            preview, _ = worker.predict(packet, -2)
            guard.prepare(preview, obs.actual(), initial=True)
            from model_deploy.reset_pose import save_start_pose
            controller.start(on_initial_pose=lambda actual: save_start_pose(cfg, log_dir, actual))
        worker.reset()
        status_pub.publish(String(data=json.dumps({'state': 'running', 'mode': mode_name})))
        start = next_tick = time.monotonic()
        previous_anchor = None
        seq = 0
        with open(log_dir/'actions.jsonl', 'w', buffering=1) as log:
            while not rospy.is_shutdown() and time.monotonic()-start < duration:
                if controller:
                    controller.check_fault()
                packet, anchor, stamps = obs.snapshot()
                if anchor == previous_anchor:
                    time.sleep(0.002)
                    continue
                previous_anchor = anchor
                began = time.monotonic()
                action, infer = worker.predict(packet, seq)
                elapsed = time.monotonic()-began
                if elapsed > cfg['policy']['inference_timeout_sec'] or time.time()-anchor > cfg['observations']['image_age_sec']:
                    raise RuntimeError('Inference/observation too old to execute')
                actual = obs.actual()
                error, sent = None, None
                try:
                    if controller:
                        sent = controller.send(action)
                    else:
                        guard.prepare(action, actual)
                except ValueError as exc:
                    if controller:
                        raise
                    error = str(exc)
                msg = JointState()
                msg.header.stamp = rospy.Time.from_sec(anchor)
                msg.name, msg.position = NAMES, action.tolist()
                raw_pub.publish(msg)
                if sent is not None:
                    msg.position = sent.tolist()
                    msg.header.stamp = rospy.Time.now()
                    sent_pub.publish(msg)
                log.write(json.dumps(dict(seq=seq, stamp=time.time(), observation_stamp=anchor,
                    source_stamps=stamps, state=packet['state'].tolist(), actual=actual.tolist(),
                    predicted=action.tolist(), commanded=None if sent is None else sent.tolist(),
                    clipped_indices=np.flatnonzero((action < guard.low) | (action > guard.high)).tolist(),
                    inference_sec=infer, roundtrip_sec=elapsed, guard_error=error), allow_nan=False)+'\n')
                if seq % 15 == 0:
                    status_pub.publish(String(data=json.dumps({'state': 'running', 'mode': mode_name,
                        'command_ready': error is None, 'guard_error': error,
                        'inference_ms': infer*1000, 'stamp': time.time()})))
                    print('{} step={} infer={:.1f}ms {}'.format(mode_name, seq, infer*1000,
                          error or 'observation/action OK'), flush=True)
                seq += 1
                next_tick += 1/cfg['policy']['fps']
                wait = next_tick-time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                else:
                    next_tick = time.monotonic()
    except KeyboardInterrupt:
        print('Stopping deployment...', flush=True)
    except Exception as exc:
        failure = str(exc)
        rospy.logerr('Deployment stopped: %s', failure)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if controller:
            controller.close()
        if worker:
            worker.close()
        (log_dir/'result.json').write_text(json.dumps({'mode': mode_name,
            'state': 'fault' if failure else 'stopped', 'reason': failure or 'operator/duration stop',
            'stamp': time.time()}, ensure_ascii=False, indent=2))
        status_pub.publish(String(data=json.dumps({'state': 'fault' if failure else 'stopped',
            'mode': mode_name, 'reason': failure or 'operator/duration stop'})))
        rospy.signal_shutdown(failure or 'deployment finished')
    if failure:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
