#!/usr/bin/python3
"""Launch four ROS endpoints; align leaders before enabling software follow."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from model_deploy.teleop_core import load_teleop
from model_deploy.core import selected_sides
from hardware_bringup.config import check_can, check_identity


def call_endpoint(side, role, operation, value=None):
    name = '/teleop_{}_{}'.format(side, role)+'/'+operation
    response = (rospy.ServiceProxy(name, SetBool)(value) if operation == 'follow'
                else rospy.ServiceProxy(name, Trigger)())
    if not response.success:
        raise RuntimeError(name+': '+response.message)


def stop_recording():
    """Finalize a live episode before resetting; a failed save prevents motion."""
    code, message, state = rospy.get_master().getSystemState()
    if code != 1:
        raise RuntimeError('Cannot check recorder: '+message)
    services = dict(state[2])
    if '/agilex/recording/stop' in services:
        response = rospy.ServiceProxy('/agilex/recording/stop', Trigger)()
        if not response.success and response.message != 'Not recording':
            raise RuntimeError('Recording not finalized; reset cancelled: '+response.message)
        print('录制：'+response.message, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    mode.add_argument('--configure-only', action='store_true', help='Set independent CAN feedback mode and query firmware; no enable or motion')
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]/'config/teleop.yaml'))
    parser.add_argument('--cameras', default='none', help='none, all or camera names')
    parser.add_argument('--duration', type=float, default=0, help='Bounded observation-mode run, seconds')
    parser.add_argument('--arms', choices=('both', 'left', 'right'), help='Default: both arms; left/right selects one leader/follower pair')
    args = parser.parse_args()
    cfg = load_teleop(args.config, arms=args.arms)
    sides = selected_sides(cfg)
    label = "双侧" if len(sides) == 2 else ("左侧" if sides[0] == "left" else "右侧")
    if not args.duration >= 0 or args.duration == float('inf'):
        parser.error('duration must be finite and nonnegative')
    if args.execute and args.duration:
        parser.error('duration is only supported for dry-run')
    for role, section in (('leader', 'leaders'), ('follower', 'arms')):
        for side in sides:
            arm = cfg['hardware'][section][side]
            check_identity(arm)
            check_can(arm['channel'], cfg['hardware']['arm_settings']['bitrate'], 'deploy')
            print('{} {} -> {} ({})'.format(side, role, arm['channel'], arm['usb_path']), flush=True)
    if args.configure_only:
        from model_deploy.teleop_device import TeleopDevice
        for role in ('follower', 'leader'):
            for side in sides:
                device = TeleopDevice(cfg, side, role)
                try:
                    device.connect(configure=True)
                    print('{} {}: firmware={} actual={} (no enable/motion)'.format(
                        side, role, device.firmware, device.actual().tolist()), flush=True)
                finally:
                    device.close()
        return
    if args.execute:
        print(label+'执行模式：所选主从臂将先保持当前位置。输入 d 后，主臂移动对齐从臂，再进入拖动示教。', flush=True)
        if input('请确认所选主从臂独立连接、运动空间已清空；按回车使能，Ctrl+C 取消：') != '':
            return
    log_dir = Path('/home/robot/AgileX_ws/logs/teleop')/time.strftime('%Y%m%d_%H%M%S')
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(log_dir)
    proc = subprocess.Popen(['roslaunch', 'model_deploy', 'teleop.launch',
                             'config:='+str(Path(args.config).resolve()),
                             'hardware_config:='+cfg['hardware_config'],
                             'execute:='+str(args.execute).lower(), 'cameras:='+args.cameras,
                             'arms:='+('both' if len(sides) == 2 else sides[0])], start_new_session=True)
    try:
        # roslaunch starts ROS master when needed.
        os.environ.setdefault('ROS_MASTER_URI', 'http://localhost:11311')
        import xmlrpc.client
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError('ROS launch exited')
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/teleop_console')
                break
            except OSError:
                time.sleep(.2)
        else:
            raise RuntimeError('ROS master startup timed out')
        rospy.init_node('teleop_console', disable_signals=True)
        if not args.execute:
            end = time.monotonic()+args.duration if args.duration else float('inf')
            print('只接收模式：不配置机械臂、不使能、不移动。Ctrl+C 退出。', flush=True)
            while proc.poll() is None and time.monotonic() < end:
                time.sleep(.2)
            if proc.poll() not in (None, 0):
                raise RuntimeError('ROS endpoint failed')
            return
        for side in sides:
            for role in ('leader', 'follower'):
                topic = '/agilex/teleop/{}_{}'.format(side, role)+'/status'
                deadline = time.monotonic()+30
                while time.monotonic() < deadline:
                    if proc.poll() is not None:
                        raise RuntimeError('ROS endpoint failed during startup')
                    try:
                        state = json.loads(rospy.wait_for_message(topic, String, timeout=1).data)
                    except rospy.ROSException:
                        continue
                    if state['phase'] == 'holding':
                        break
                else:
                    raise RuntimeError('Endpoint not holding: '+topic)
        print('所选主从臂已保持。d：主臂对齐并跟随；p：从臂暂停保持；r：保存录制并复位所选主从臂；q：退出。', flush=True)
        print('r 会开爪至 100 mm、抬升从臂并返回固定姿态，再移动主臂对齐；请先解除接触并留出路径。', flush=True)
        while proc.poll() is None:
            command = input('teleop> ').strip().lower()
            if command == 'q':
                break
            if command == 'r':
                from model_deploy.teleop_reset import reset_collection
                print('正在结束录制并复位；请放开主臂，保持所选主从臂运动空间畅通。', flush=True)
                reset_collection(call_endpoint, stop_recording, sides)
                print('所选主从臂复位完成并保持；输入 d 再次开启主从跟随，之后可录制下一段。', flush=True)
                continue
            if command not in ('d', 'p'):
                continue
            for side in sides:
                response = rospy.ServiceProxy('/teleop_'+side+'_follower/follow', SetBool)(False)
                if not response.success:
                    raise RuntimeError(response.message)
            time.sleep(.3)
            if command == 'd':
                print('主臂即将对齐从臂；请放开主臂并保持运动空间畅通。', flush=True)
                for side in sides:
                    response = rospy.ServiceProxy('/teleop_'+side+'_leader/align', Trigger)()
                    if not response.success:
                        raise RuntimeError(side+': '+response.message)
                time.sleep(.3)
                for side in sides:
                    response = rospy.ServiceProxy('/teleop_'+side+'_follower/follow', SetBool)(True)
                    if not response.success:
                        raise RuntimeError(side+': '+response.message)
                print('所选侧关节跟随已开启。若夹爪尚未跟随，请把主臂夹爪开合到当前从臂开口，匹配后自动接入快速跟随。', flush=True)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        pass
