#!/usr/bin/python3
"""Synthetic full-topic test on a private ROS master; never accesses CAN/USB."""
import json
import os
import signal
import subprocess
import threading
import time
import tempfile
import rosbag
from pathlib import Path
import xmlrpc.client

os.environ['ROS_MASTER_URI']='http://127.0.0.1:11451'
os.environ['ROS_IP']='127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
import rospy
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from hardware_bringup.config import load_config,topic_specs
from hardware_bringup.monitor import TopicMonitor
from hardware_bringup.msg import MotorFeedback

ROOT=Path('/home/robot/AgileX_ws')
OUT=ROOT/'logs/synthetic_validation'


def main():
    OUT.mkdir(exist_ok=True)
    processes=[];files=[];stop=threading.Event();fault=threading.Event()
    monitor=None
    try:
        log=(OUT/'roscore.log').open('w');files.append(log)
        core=subprocess.Popen(['roscore','-p','11451'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(core)
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(100):
            try:
                if master.getPid('/test')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('Private ROS master unavailable')
        rospy.init_node('agilex_synthetic_validation')
        cfg=load_config(ROOT/'src/hardware_bringup/config/hardware.yaml')
        cfg['camera_settings'].update(width=32,height=24)
        arms=list(cfg['arms']);cameras=list(cfg['cameras'])
        specs=topic_specs(cfg,arms,cameras)
        types={'joint':JointState,'image':Image,'info':CameraInfo,'status':String,'gripper_status':String,'motor_feedback':MotorFeedback}
        pubs={s['topic']:rospy.Publisher(s['topic'],types[s['kind']],queue_size=5,latch=s['timeout']==0) for s in specs}
        state_pub=rospy.Publisher('/agilex/hardware/status',String,queue_size=1,latch=True)
        monitor=TopicMonitor(specs)

        def feed():
            sent_events=set()
            while not stop.is_set():
                stamp=rospy.Time.now()
                for spec in specs:
                    if spec['timeout']==0 and spec['topic'] in sent_events:continue
                    kind=spec['kind']
                    if kind=='joint':
                        msg=JointState();msg.name=['j'+str(i) for i in range(spec['size'])];msg.position=[.1]*spec['size']
                    elif kind=='image':
                        bpp=3 if spec['encoding']=='rgb8' else 2
                        msg=Image(width=32,height=24,encoding=spec['encoding'],step=32*bpp,data=bytes(32*24*bpp))
                    elif kind=='info':msg=CameraInfo(width=32,height=24,K=[100,0,16,0,100,12,0,0,1])
                    elif kind=='status':msg=String(data=json.dumps(dict(stamp=stamp.to_sec(),err_code=0,arm_status=0,ctrl_mode=1)))
                    elif kind=='motor_feedback':
                        known=spec['model']!='unknown'
                        msg=MotorFeedback(current_a=[.1]*6,
                            motor_velocity_rad_s=[0.0]*6,motor_position_rad=[.1]*6,
                            joint_order=list(range(1,7)),torque_estimate_nm=[.2]*6 if known else [])
                    else:msg=String(data=json.dumps(dict(stamp=stamp.to_sec(),status_code=64,position=[.05],protocol_force=0)))
                    if hasattr(msg,'header'):msg.header.stamp=stamp;msg.header.frame_id='synthetic'
                    pubs[spec['topic']].publish(msg)
                    if spec['timeout']==0:sent_events.add(spec['topic'])
                rows=monitor.snapshot()
                payload=dict(state='error' if fault.is_set() else ('ready' if all(r['ok'] for r in rows) else 'starting'),
                    reason='synthetic injected fault' if fault.is_set() else '',stamp=time.time(),config=cfg,arms=arms,cameras=cameras,
                    topics=rows,record_topics=[s['topic'] for s in specs]+['/agilex/hardware/status'])
                state_pub.publish(String(data=json.dumps(payload)))
                time.sleep(.05)
        worker=threading.Thread(target=feed,daemon=True);worker.start()
        log=(OUT/'recorder.log').open('w');files.append(log)
        rec=subprocess.Popen(['roslaunch','data_collection','record.launch','auto_start:=true',
                              'output_dir:='+str(OUT/'bags')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(rec)
        rospy.wait_for_service('/agilex/recording/stop',timeout=20)
        def wait_state(expected,timeout=25):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                msg=rospy.wait_for_message('/agilex/recording/status',String,timeout=3)
                state=json.loads(msg.data)
                if state['state']==expected:return state
                time.sleep(.1)
            raise RuntimeError('Expected recorder state '+expected)
        first=wait_state('recording');time.sleep(3)
        response=rospy.ServiceProxy('/agilex/recording/stop',Trigger)()
        assert response.success,response.message
        manifest=json.loads((Path(first['session'])/'manifest.json').read_text())
        assert manifest['state']=='complete'
        assert all(manifest['topic_counts'].get(s['topic'],0)>0 for s in specs)
        # Latched stationary leader targets must still be in the bag.
        for side in arms:assert manifest['topic_counts']['/piper/'+side+'_bus/leader_targets']>=1
        response=rospy.ServiceProxy('/agilex/recording/start',Trigger)()
        assert response.success,response.message
        second=wait_state('recording');time.sleep(2);fault.set()
        failure=wait_state('incomplete')
        assert 'synthetic injected fault' in failure['reason']
        # The public CLI writes a recording directory, including paths with spaces,
        # and finalizes it when Ctrl+C is sent to its foreground process group.
        os.killpg(rec.pid,signal.SIGINT);rec.wait(timeout=30)
        fault.clear()
        explicit_dir=Path(tempfile.mkdtemp(prefix='explicit output ',dir=str(OUT)))
        target=explicit_dir/'episode 001'
        bag_path=target/'data.bag'
        def launch_explicit(target):
            handle=(explicit_dir/(target.stem+'.console.log')).open('w');files.append(handle)
            proc=subprocess.Popen([str(ROOT/'record.sh'),'--output='+str(target)],
                stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(proc)
            metadata=target/'manifest.json'
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                if proc.poll() is not None:raise RuntimeError('Explicit recorder exited early')
                if metadata.exists() and (target/'data.bag.active').exists():return proc,metadata
                time.sleep(.1)
            raise RuntimeError('Explicit recorder did not start')
        direct,meta=launch_explicit(target)
        # A second process must fail rather than replacing the existing ROS node.
        duplicate=subprocess.run([str(ROOT/'record.sh'),'--output='+str(explicit_dir/'concurrent')],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=10)
        assert duplicate.returncode!=0,duplicate.stdout
        assert 'Another AgileX recorder' in duplicate.stdout,duplicate.stdout
        time.sleep(3)
        os.killpg(direct.pid,signal.SIGINT)
        assert direct.wait(timeout=30)==0
        saved=json.loads(meta.read_text())
        assert saved['state']=='complete',saved
        assert bag_path.exists() and not (target/'data.bag.active').exists()
        assert {p.name for p in target.iterdir()}=={'data.bag','manifest.json','rosbag.log'}
        assert not list(target.glob('data_*.bag'))
        with rosbag.Bag(str(bag_path)) as bag:
            counts={}
            for topic,msg,stamp in bag.read_messages():counts[topic]=counts.get(topic,0)+1
        assert all(counts.get(s['topic'],0)>0 for s in specs)
        original=(bag_path.stat().st_size,bag_path.stat().st_mtime_ns)
        note=target/'notes.txt';note.write_text('Keep this unrelated file')
        # Waiting for READY must preserve the old bag and metadata.
        fault.set();time.sleep(.3)
        waiting_log=(explicit_dir/'waiting.log').open('w');files.append(waiting_log)
        waiting=subprocess.Popen([str(ROOT/'record.sh'),'--output='+str(target)],
            stdout=waiting_log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(waiting);time.sleep(2)
        assert waiting.poll() is None
        assert original==(bag_path.stat().st_size,bag_path.stat().st_mtime_ns)
        assert json.loads(meta.read_text())==saved
        os.killpg(waiting.pid,signal.SIGINT);waiting.wait(timeout=15)
        fault.clear();time.sleep(.3)
        overwrite,meta=launch_explicit(target)
        time.sleep(2);os.killpg(overwrite.pid,signal.SIGINT)
        assert overwrite.wait(timeout=30)==0
        replaced=json.loads(meta.read_text())
        assert replaced['state']=='complete' and replaced['started_at']>saved['stopped_at']
        assert bag_path.stat().st_mtime_ns!=original[1]
        assert note.read_text()=='Keep this unrelated file'
        with rosbag.Bag(str(bag_path)) as bag:
            assert bag.get_start_time()>saved['stopped_at']
            assert sum(1 for _ in bag.read_messages())>0
        # New directory supports the next episode; runtime faults still stop recording.
        second_target=explicit_dir/'episode 002'
        direct2,meta2=launch_explicit(second_target)
        time.sleep(2);fault.set()
        assert direct2.wait(timeout=30)!=0
        failed=json.loads(meta2.read_text())
        assert failed['state']=='incomplete' and 'synthetic injected fault' in failed['reason'],failed
        result=dict(passed=True,synthetic=True,full_topic_recording=first['session'],
                    topic_count=len(specs),fault_interruption=second['session'],
                    explicit_output_directory=str(target),ctrl_c_finalized=True,unrelated_file_preserved=True,overwrite_verified=True,
                    previous_file_preserved_while_not_ready=True,
                    concurrent_recorder_refused=True,explicit_fault_stopped=True)
        (OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        for proc in reversed(processes[1:]):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=25)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        stop.set()
        if monitor:monitor.close()
        if processes:
            core=processes[0]
            if core.poll() is None:os.killpg(core.pid,signal.SIGINT)
            try:core.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(core.pid,signal.SIGKILL);core.wait()
        for f in files:f.close()


if __name__=='__main__':main()
