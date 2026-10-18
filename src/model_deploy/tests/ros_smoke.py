#!/usr/bin/python3
"""Private ROS master + recorded fixture. Never opens CAN or a camera device."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--fixture', required=True)
    args = p.parse_args()
    ws = Path(__file__).resolve().parents[3]
    with np.load(args.fixture, allow_pickle=False) as data:
        fixture = {k: data[k] for k in data.files}
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port), ROS_HOSTNAME='127.0.0.1')
    import rospy
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String
    result = []
    with tempfile.TemporaryDirectory(prefix='agilex-ros-smoke-') as tmp:
        tmp = Path(tmp)
        master_log = open(tmp/'master.log', 'w')
        master = subprocess.Popen(['/opt/ros/noetic/bin/roscore', '-p', str(port)], stdout=master_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        stopped, pause = threading.Event(), threading.Event()
        try:
            for _ in range(100):
                try:
                    xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/smoke')
                    break
                except OSError:
                    time.sleep(.1)
            else:
                raise RuntimeError('Private ROS master failed')
            rospy.init_node('deploy_fixture_publisher', disable_signals=True)
            pubs = {}
            for side in ('left', 'right'):
                for suffix in ('joint_states', 'gripper/actual_stamped'):
                    pubs[side+'/'+suffix] = rospy.Publisher('/piper/'+side+'_bus/'+suffix, JointState, queue_size=2)
                pubs[side+'/status'] = rospy.Publisher('/piper/'+side+'_bus/status', String, queue_size=2)
            for cam in ('cam_h', 'cam_l', 'cam_r'):
                pubs[cam] = rospy.Publisher('/'+cam+'/color/image_raw', Image, queue_size=1)
            def publish():
                next_image = 0
                while not stopped.wait(.005):
                    if pause.is_set():
                        continue
                    stamp = rospy.Time.now()
                    for i, side in enumerate(('left', 'right')):
                        msg = JointState()
                        msg.header.stamp = stamp
                        msg.name = [side+'_follower_joint_'+str(j) for j in range(1, 7)]
                        msg.position = fixture['state'][i*7:i*7+6].tolist()
                        pubs[side+'/joint_states'].publish(msg)
                        msg.name = [side+'_follower_gripper_width']
                        msg.position = [float(fixture['state'][i*7+6])]
                        pubs[side+'/gripper/actual_stamped'].publish(msg)
                        pubs[side+'/status'].publish(String(data=json.dumps(dict(stamp=stamp.to_sec(), err_code=0, arm_status=0, ctrl_mode=1))))
                    if time.monotonic() >= next_image:
                        next_image = time.monotonic()+1/15
                        for cam in ('cam_h', 'cam_l', 'cam_r'):
                            msg = Image(height=480, width=640, encoding='rgb8', step=640*3, data=fixture[cam].tobytes())
                            msg.header.stamp = stamp
                            pubs[cam].publish(msg)
            thread = threading.Thread(target=publish, daemon=True)
            thread.start()
            for case in ('normal', 'stale_feedback', 'worker_exit', 'ctrl_c'):
                pause.clear()
                config = tmp/(case+'.yaml')
                import yaml
                cfg = yaml.safe_load((ws/'src/model_deploy/config/act.yaml').read_text())
                cfg['log_dir'] = str(tmp/case)
                config.write_text(yaml.safe_dump(cfg))
                logpath = ws/'logs'/('deploy_ros_'+case+'.log')
                with open(logpath, 'w') as log:
                    proc = subprocess.Popen([str(ws/'deploy.sh'), '--config', str(config), '--dry-run',
                        '--duration', '3' if case == 'normal' else '30'], stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
                    try:
                        deadline = time.monotonic()+120
                        while True:
                            paths = list((tmp/case).glob('*/actions.jsonl'))
                            if paths and len(paths[0].read_text().splitlines()) >= 5:
                                break
                            if proc.poll() is not None or time.monotonic() > deadline:
                                raise RuntimeError(case+': failed to produce actions; see '+str(logpath))
                            time.sleep(.1)
                        if case == 'stale_feedback':
                            pause.set()
                        elif case == 'worker_exit':
                            children = Path('/proc/{}/task/{}/children'.format(proc.pid, proc.pid)).read_text().split()
                            assert len(children) == 1, children
                            os.kill(int(children[0]), signal.SIGTERM)
                        elif case == 'ctrl_c':
                            proc.send_signal(signal.SIGINT)
                        rc = proc.wait(timeout=15)
                        expected = 1 if case in ('stale_feedback', 'worker_exit') else 0
                        if rc != expected:
                            raise RuntimeError('{} exit={} expected={}; {}'.format(case, rc, expected, logpath))
                        rows = [json.loads(line) for line in paths[0].read_text().splitlines()]
                        assert rows and all(row['commanded'] is None for row in rows)
                        result.append(dict(case=case, exit_code=rc, frames=len(rows),
                                           roundtrip_p95_ms=float(np.percentile([r['roundtrip_sec'] for r in rows],95)*1000)))
                    finally:
                        if proc.poll() is None:
                            proc.terminate()
                            proc.wait(timeout=10)
            print(json.dumps(result, indent=2))
        finally:
            stopped.set()
            rospy.signal_shutdown('test complete')
            os.killpg(master.pid, signal.SIGINT)
            master.wait(timeout=15)
            master_log.close()


if __name__ == '__main__':
    main()
