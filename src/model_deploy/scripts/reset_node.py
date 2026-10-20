#!/usr/bin/python3
"""Open grippers, lift vertically, return to fixed joint pose, then close."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from model_deploy.core import load_config, selected_sides
from model_deploy.reset_pose import atomic_json, load_reset_pose
from model_deploy.reset_sequence import ResetSequence, validate_reset_config

PACKAGE = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default=str(PACKAGE/'config/act.yaml'))
    p.add_argument('--from-run', help='Override fixed pose with a deployment log; old logs use an approximate first observation')
    p.add_argument('--dry-run', action='store_true', help='Print reset reference/sequence only; no ROS, SDK, CAN or movement')
    p.add_argument('--arms', choices=('both', 'left', 'right'), help='Reset selected follower(s); default both')
    args = p.parse_args()
    cfg = load_config(args.config, arms=args.arms)
    sides = selected_sides(cfg)
    validate_reset_config(cfg)
    home, reference = load_reset_pose(cfg, args.from_run)
    gripper_key = 'both_grippers_m' if len(sides) == 2 else 'gripper_m'
    plan = dict(active_arms=list(sides), reference=reference, sequence=[
        {'phase': 'opening_grippers', gripper_key: cfg['reset']['gripper_open_m']},
        {'phase': 'returning_home', **{side+'_joints_rad': home[i*7:i*7+6].tolist() for i, side in enumerate(sides)},
         gripper_key: cfg['reset']['gripper_open_m']},
        {'phase': 'closing_grippers', gripper_key: cfg['reset']['gripper_closed_m']}])
    lift = cfg['reset'].get('vertical_lift')
    if lift is not None:
        plan['sequence'].insert(1, dict(phase='lifting_arms', frame='each_arm_base',
                                       mode=lift.get('mode', 'linear'),
                                       delta_z_m=lift['distance_m'],
                                       orientation=('allow_bounded_rotation' if lift.get('mode') == 'joint_path'
                                                    else 'hold_measured_at_lift_start'),
                                       **{gripper_key: cfg['reset']['gripper_open_m']}))
        if lift.get('mode') == 'joint_path':
            plan['sequence'][1].update(max_lateral_m=lift['max_lateral_m'],
                                       max_rotation_rad=lift['max_rotation_rad'])
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    if reference['estimated']:
        print('注意：旧日志没有精确起始快照，使用首帧观测作为近似回位姿态。', flush=True)
    if args.dry_run:
        print('DRY-RUN: only printed the plan; no SDK/CAN connection.', flush=True)
        return

    import rospy
    from std_msgs.msg import String
    from model_deploy.observations import Observations
    from model_deploy.controller import Controller
    rospy.init_node('agilex_model_reset', disable_signals=True)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Reset requires live hardware, /use_sim_time=false')
    run_dir = Path(cfg['log_dir'])/('reset_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run_dir.mkdir(parents=True)
    atomic_json(run_dir/'plan.json', plan)
    atomic_json(run_dir/'config.json', cfg)
    publisher = rospy.Publisher('/agilex/reset/status', String, queue_size=10, latch=True)
    # Reset needs fresh arm feedback and hardware status, not cameras or GPU.
    obs = Observations(cfg, include_images=False)
    controller = None
    failure = None
    outcome = 'failed'
    with (run_dir/'events.jsonl').open('w', buffering=1) as log:
        def report(event):
            event = dict(event, stamp=time.time())
            line = json.dumps(event, ensure_ascii=False, allow_nan=False)
            log.write(line+'\n')
            print(line, flush=True)
            publisher.publish(String(data=line))
        try:
            report({'state': 'waiting_for_hardware'})
            deadline = time.monotonic()+10
            while True:
                if rospy.is_shutdown():
                    raise InterruptedError('ROS shutdown before reset')
                try:
                    obs.deployment_hardware()
                    obs.actual()
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.05)
            reset_cfg = copy.deepcopy(cfg)
            reset_cfg['control']['sdk_speed_percent'] = cfg['reset']['sdk_speed_percent']
            controller = Controller(reset_cfg, obs, require_images=False)
            # Same per-CAN locks as deployment. If a model controller is running,
            # connect fails BEFORE any gripper or movement command is sent.
            controller.connect()
            atomic_json(run_dir/'firmware.json', controller.firmware)
            if lift is not None and lift.get('mode') == 'joint_path':
                report({'state': 'planning', 'phase': 'lifting_arms'})
                path = controller.prepare_joint_lift(cancelled=rospy.is_shutdown)
                atomic_json(run_dir/'lift_path.json', path)
                report({'state': 'planned', 'phase': 'lifting_arms',
                        'waypoints': len(path['joints']), 'path_file': str(run_dir/'lift_path.json')})
            controller.start()  # deliberately does not overwrite the saved start pose
            sequence = ResetSequence(reset_cfg, controller, obs,
                                     cancelled=rospy.is_shutdown, report=report)
            sequence.run(home)
            outcome = 'complete'
        except (KeyboardInterrupt, InterruptedError):
            outcome, failure = 'cancelled', 'operator/ROS cancellation'
        except Exception as exc:
            failure = str(exc)
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            if controller is not None:
                controller.close()
            result = dict(state=outcome, reason=failure, run_dir=str(run_dir))
            report(result)
            atomic_json(run_dir/'result.json', result)
            rospy.signal_shutdown(outcome)
    if outcome != 'complete':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
