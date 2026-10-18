#!/usr/bin/python3
"""Preview or explicitly execute recorded dual-Piper follower poses via SDK."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from model_deploy.core import load_config
from model_deploy.reset_pose import atomic_json
from model_deploy.replay import (
    SDKReplayFeedback, check_start, load_trajectory, move_to_start, play_trajectory, replay_config,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='ROS1 .bag file or recording directory containing data.bag')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='Control real arms after start check and Enter')
    mode.add_argument('--dry-run', action='store_true', help='Default: validate/preview entire bag, no SDK/CAN')
    parser.add_argument('--move-to-start', action='store_true',
                        help='With --execute: slowly move arms/grippers to bag start before playback')
    parser.add_argument('--rate', type=float, help='Playback speed, default from replay.yaml (0.5); 1=recorded speed')
    parser.add_argument('--config', default=str(PACKAGE/'config/act.yaml'), help='Base CAN/firmware/limits config')
    parser.add_argument('--replay-config', default=str(PACKAGE/'config/replay.yaml'))
    args = parser.parse_args()
    cfg = replay_config(load_config(args.config), args.replay_config, args.rate)
    print('Reading follower state trajectory; no motion sent.', flush=True)
    grid, points, plan = load_trajectory(args.bag, cfg)
    plan['move_to_start'] = args.move_to_start
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    for i, side in enumerate(('left', 'right')):
        print('{} 起点：J1~J6 度={}，夹爪 {:.1f} mm'.format(side,
            np.round(np.degrees(points[0, i*7:i*7+6]), 2).tolist(), points[0, i*7+6]*1000), flush=True)
    if not args.execute:
        print('DRY-RUN complete: no SDK/CAN connection; no motion. Use --execute for physical playback.', flush=True)
        return
    if not sys.stdin.isatty():
        raise RuntimeError('--execute requires an interactive terminal')

    import rospy
    from std_msgs.msg import String
    from model_deploy.controller import Controller
    from model_deploy.observations import Observations
    rospy.init_node('agilex_trajectory_replay', disable_signals=True)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Physical replay requires live hardware and use_sim_time=false')
    log_dir = Path(cfg['log_dir']).parent/'replay'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    log_dir.mkdir(parents=True)
    atomic_json(log_dir/'config.json', cfg)
    atomic_json(log_dir/'plan.json', plan)
    np.savez_compressed(log_dir/'trajectory.npz', source_time_sec=grid, target=points)
    print('Replay log: ' + str(log_dir), flush=True)
    publisher = rospy.Publisher('/agilex/replay/status', String, queue_size=10, latch=True)
    hardware = Observations(cfg, include_images=False)
    obs = SDKReplayFeedback(cfg, hardware)
    controller = Controller(cfg, obs, require_images=False)
    obs.controller = controller
    result = dict(state='failed', reason='startup incomplete')
    def status(event):
        publisher.publish(String(data=json.dumps(dict(event, stamp=time.time()), ensure_ascii=False)))
    def wait_ready(check):
        deadline = time.monotonic()+cfg['replay']['startup_timeout_sec']
        while True:
            if rospy.is_shutdown():
                raise InterruptedError('ROS shutdown')
            try:
                check()
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.05)
    with (log_dir/'actions.jsonl').open('w', buffering=1) as log:
        last_report = {}
        def report(event):
            event = dict(event, phase=event.get('phase', 'replay'))
            log.write(json.dumps(dict(event, stamp=time.time()), allow_nan=False)+'\n')
            phase = event['phase']
            if event['elapsed_sec']-last_report.get(phase, -1.) >= 1:
                last_report[phase] = event['elapsed_sec']
                if phase == 'moving_to_start':
                    status(dict(state='running', phase=phase, elapsed_sec=event['elapsed_sec']))
                    print('moving_to_start elapsed={:.2f}s'.format(event['elapsed_sec']), flush=True)
                    return
                progress = dict(state='running', index=event['index'], source_time_sec=event['source_time_sec'],
                                source_duration_sec=plan['source_duration_sec'], elapsed_sec=event['elapsed_sec'], phase=phase)
                status(progress)
                print('replay bag={:.2f}/{:.2f}s elapsed={:.2f}s'.format(
                    event['source_time_sec'], plan['source_duration_sec'], event['elapsed_sec']), flush=True)
        try:
            status(dict(state='waiting_for_hardware'))
            wait_ready(obs.deployment_hardware)
            controller.connect()  # locks and firmware queries only
            atomic_json(log_dir/'firmware.json', controller.firmware)
            wait_ready(obs.actual)
            if args.move_to_start:
                print('将先缓慢移动双臂和夹爪到 bag 起点，到位后自动开始回放。'
                      '这是关节插值，不会自动抬臂或避障；请确认路径畅通、主臂已退出控制。'
                      '回车开始；Ctrl+C 取消。', flush=True)
            else:
                check_start(cfg, points[0], obs.actual())
                print('起点检查通过。主臂须退出控制，确认物体及回放空间已准备好。回车开始；Ctrl+C 取消。', flush=True)
            input()
            if rospy.is_shutdown():
                raise InterruptedError('ROS shutdown before enabling')
            def initial_pose(actual):
                if not args.move_to_start:
                    check_start(cfg, points[0], actual)
                atomic_json(log_dir/'start_pose.json', dict(actual=actual.tolist(), target=points[0].tolist(),
                                                           move_to_start=args.move_to_start, stamp=time.time()))
            controller.start(on_initial_pose=initial_pose)
            if args.move_to_start:
                approach = move_to_start(cfg, points[0], controller, obs, cancelled=rospy.is_shutdown,
                                        report=lambda e: report(dict(e, phase='moving_to_start')))
                atomic_json(log_dir/'approach_result.json', approach)
                print('已到达 bag 起点，开始轨迹回放。', flush=True)
            result = play_trajectory(cfg, grid, points, controller, obs,
                                     cancelled=rospy.is_shutdown, report=report)
        except (KeyboardInterrupt, InterruptedError):
            result = dict(state='cancelled', reason='operator/ROS cancellation')
        except Exception as exc:
            result = dict(state='failed', reason=str(exc))
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            controller.close()
            result['log_dir'] = str(log_dir)
            atomic_json(log_dir/'result.json', result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
            status(result)
            rospy.signal_shutdown(result['state'])
    if result['state'] != 'complete':
        raise SystemExit(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Replay cancelled before motion.', file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print('Replay failed: ' + str(exc), file=sys.stderr)
        raise SystemExit(1)
