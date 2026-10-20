#!/usr/bin/python3
"""Record ROS messages verbatim with rosbag; require a live hardware readiness report."""
import datetime
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path
import rosbag
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse


def ready_error(status, last_received, timeout, required_arms, required_cameras, allow_partial):
    if not status or time.monotonic()-last_received>timeout:
        return 'Hardware status missing/stale; start hardware.launch first'
    if status.get('state')!='ready':return 'Hardware not ready: '+status.get('reason',status.get('state','unknown'))
    if abs(time.time()-status.get('stamp',0))>timeout:return 'Hardware heartbeat timestamp stale'
    if not allow_partial and (set(status.get('arms',[]))!=set(required_arms) or set(status.get('cameras',[]))!=set(required_cameras)):
        return 'Recording requires arms={} and cameras={}; allow_partial must be explicit for a subset'.format(required_arms, required_cameras)
    if not status.get('topics') or not all(t.get('ok') for t in status['topics']):return 'Hardware report has invalid topic data'
    if not status.get('record_topics'):return 'No record topics'
    return ''


def selected_hardware(status: dict, sides: list) -> dict:
    """Select recorded arm topics while retaining global/camera diagnostics."""
    excluded = {'left', 'right'} - set(sides)
    def keep(topic):
        return not any(topic.startswith(('/piper/'+s+'_', '/agilex/teleop/'+s+'_', '/teleop_'+s+'_')) for s in excluded)
    result = dict(status)
    result['arms'] = [s for s in ('left', 'right') if s in sides and s in status.get('arms', [])]
    result['topics'] = [row for row in status.get('topics', []) if keep(row['topic'])]
    result['record_topics'] = [topic for topic in status.get('record_topics', []) if keep(topic)]
    return result


def output_error(path):
    """Validate a recording directory without modifying its contents."""
    if path.suffix=='.bag':
        return '--output must name a directory, not a .bag file: '+str(path)
    if path.is_symlink() or (path.exists() and not path.is_dir()):
        return 'Output must be a directory, not a file or symlink: '+str(path)
    if os.path.lexists(path/'data.bag.active'):
        return 'Recording .active file exists; stop/recover that recording first: '+str(path)
    for name in ('data.bag','manifest.json','manifest.json.tmp','rosbag.log'):
        item=path/name
        if item.is_symlink() or (item.exists() and not item.is_file()):
            return 'Invalid recording file: '+str(item)
    return ''


class Recorder:
    def __init__(self, output_directory=None):
        self.guard=threading.RLock()
        self.hardware=None;self.last_received=0
        self.proc=None;self.session=None;self.manifest=None;self.log=None
        self.output_directory=Path(output_directory).expanduser().absolute() if output_directory else None
        if self.output_directory:
            error=output_error(self.output_directory)
            if error:raise ValueError(error)
        self.output_file=self.output_directory/'data.bag' if self.output_directory else None
        self.root=self.output_directory if self.output_directory else Path(rospy.get_param('~output_dir')).expanduser()
        self.minimum=float(rospy.get_param('~min_free_gb',5))*1024**3
        self.timeout=float(rospy.get_param('~status_timeout_sec',3))
        self.allow_partial=bool(rospy.get_param('~allow_partial',False))
        self.required_arms=rospy.get_param('~required_arms',['left','right'])
        selection = rospy.get_param('~arms', None)
        if selection is not None:
            if selection not in ('both', 'left', 'right'):
                raise ValueError('arms must be both, left or right')
            self.required_arms = ['left', 'right'] if selection == 'both' else [selection]
        self.required_cameras=rospy.get_param('~required_cameras',['cam_h','cam_l','cam_r'])
        self.compression=rospy.get_param('~compression','lz4')
        self.split_mb=int(rospy.get_param('~split_size_mb',2048))
        self.buffer_mb=int(rospy.get_param('~buffer_size_mb',512))
        if self.minimum<0 or self.timeout<=0 or self.split_mb<=0 or self.buffer_mb<=0 or self.compression not in ('lz4','none'):
            raise ValueError('Invalid recording configuration')
        self.pub=rospy.Publisher('/agilex/recording/status',String,queue_size=1,latch=True)
        self.sub=rospy.Subscriber('/agilex/hardware/status',String,self.on_status,queue_size=1)
        self.services=[rospy.Service('/agilex/recording/start',Trigger,self.start),
                       rospy.Service('/agilex/recording/stop',Trigger,self.stop)]
        self.publish('idle')

    def on_status(self,msg):
        try:status=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.guard:
            self.hardware=selected_hardware(status, self.required_arms)
            self.last_received=time.monotonic()

    def reason(self):
        error=ready_error(self.hardware,self.last_received,self.timeout,
                          self.required_arms,self.required_cameras,self.allow_partial)
        if not error and self.proc is not None:
            selection=dict(arms=self.hardware['arms'],cameras=self.hardware['cameras'])
            if selection!=self.manifest['selection'] or self.hardware['config']!=self.manifest['hardware']['config']:
                return 'Hardware selection/config changed during recording'
        return error

    def publish(self,state,reason=''):
        self.pub.publish(String(data=json.dumps(dict(state=state,reason=reason,
            session=str(self.session) if self.session else '',stamp=time.time()),ensure_ascii=False)))

    def save_manifest(self):
        tmp=self.session/'manifest.json.tmp'
        tmp.write_text(json.dumps(self.manifest,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(self.session/'manifest.json')

    def start(self,_req):
        with self.guard:
            if self.proc is not None:return TriggerResponse(False,'Already recording')
            error=self.reason()
            if error:return TriggerResponse(False,error)
            if self.output_directory:
                error=output_error(self.output_directory)
                if error:return TriggerResponse(False,error)
            self.root.mkdir(parents=True,exist_ok=True)
            if shutil.disk_usage(self.root).free<self.minimum:return TriggerResponse(False,'Insufficient free disk space')
            if self.output_file:
                self.session=self.output_directory
                if self.output_file.exists():
                    rospy.logwarn('Overwriting previous recording: %s',self.output_file)
                    self.output_file.unlink()
            else:
                local=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
                self.session=self.root/(local.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
                self.session.mkdir()
            topics=list(dict.fromkeys(self.hardware['record_topics']))
            cmd=['rosbag','record','--buffsize',str(self.buffer_mb)]
            if not self.output_file:cmd+=['--split','--size',str(self.split_mb)]
            cmd+=['-O',str(self.output_file or self.session/'data.bag')]
            if self.compression=='lz4':cmd+=['--lz4']
            cmd+=topics+['__name:=agilex_bag_writer_'+uuid.uuid4().hex[:8]]
            self.manifest=dict(schema_version=1,state='recording',started_at=time.time(),
                timezone='Asia/Shanghai',selection=dict(arms=self.hardware['arms'],cameras=self.hardware['cameras']),
                hardware=self.hardware,topics=topics,command=cmd,output_file=str(self.output_file) if self.output_file else None,
                timing='Messages retain original headers; bag time is recorder receipt time. No resampling or cross-camera synchronization.',
                leader='Leader data is target/command echo, not independent measured feedback; latched last target preserves its original timestamp.')
            self.save_manifest()
            self.log=(self.session/'rosbag.log').open('w')
            try:self.proc=subprocess.Popen(cmd,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            except Exception as exc:
                self.log.close();self.log=None;self.manifest.update(state='error',reason=str(exc));self.save_manifest()
                return TriggerResponse(False,str(exc))
            self.publish('recording');rospy.loginfo('Recording %d topics -> %s; Ctrl+C to stop and save',len(topics),self.output_file or self.session)
            return TriggerResponse(True,str(self.session))

    def finish(self,state,reason):
        if self.proc is None:return TriggerResponse(False,'Not recording')
        proc=self.proc;self.proc=None
        if proc.poll() is None:
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
        try:code=proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            state='incomplete';reason+='; rosbag SIGINT timeout'
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:code=proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);code=proc.wait()
        if self.log:self.log.close();self.log=None
        counts={};files=[]
        try:
            writer_log=(self.session/'rosbag.log').read_text(errors='replace').lower()
            if 'buffer exceeded' in writer_log or 'dropping oldest' in writer_log:
                state='incomplete';reason+='; rosbag reported buffer overflow/message loss'
            paths=([self.output_file] if self.output_file.exists() else []) if self.output_file else sorted(self.session.glob('*.bag'))
            for path in paths:
                with rosbag.Bag(str(path),'r') as bag:
                    info=bag.get_type_and_topic_info()
                    for topic,meta in info.topics.items():counts[topic]=counts.get(topic,0)+meta.message_count
                files.append(dict(name=path.name,size_bytes=path.stat().st_size))
            required=[row['topic'] for row in self.manifest['hardware']['topics']]
            missing=[t for t in required if not counts.get(t)]
            active=Path(str(self.output_file)+'.active').exists() if self.output_file else bool(list(self.session.glob('*.active')))
            if code!=0 or active or not files or missing:
                state='incomplete';reason+='; exit={} missing={}'.format(code,missing)
        except Exception as exc:state='incomplete';reason+='; bag verification: '+str(exc)
        self.manifest.update(state=state,reason=reason,stopped_at=time.time(),exit_code=code,files=files,topic_counts=counts)
        self.save_manifest();self.publish(state,reason)
        rospy.loginfo('Recording %s: %s (%s)',state,self.session,reason)
        return TriggerResponse(state=='complete',str(self.session)+' '+state+': '+reason)

    def stop(self,_req):
        with self.guard:
            error=self.reason() if self.proc is not None else ''
            return self.finish('incomplete' if error else 'complete',error or 'user stop')

    def run(self):
        auto=self.output_file is not None or bool(rospy.get_param('~auto_start',False))
        deadline=time.monotonic()+float(rospy.get_param('~startup_timeout_sec',120))
        rospy.loginfo('Waiting for hardware READY (up to %.0f s)',max(0,deadline-time.monotonic()))
        try:
            while not rospy.is_shutdown():
                with self.guard:
                    if auto and self.proc is None:
                        error=self.reason()
                        if not error:
                            response=self.start(None)
                            if not response.success:raise RuntimeError(response.message)
                            auto=False
                        elif time.monotonic()>deadline:raise RuntimeError(error)
                    if self.proc is not None:
                        error=self.reason()
                        if self.proc.poll() is not None:error='rosbag process exited unexpectedly'
                        if shutil.disk_usage(self.root).free<self.minimum:error='Low disk space'
                        if error:
                            self.finish('incomplete',error)
                            rospy.logerr('Recording stopped: %s',error)
                            if self.output_file:raise RuntimeError(error)
                    # A console reset can finish this CLI episode through the
                    # stop service. Release its lock so the next record.sh can
                    # start, rather than leaving an idle recorder behind.
                    if self.output_file and not auto and self.proc is None and self.manifest:
                        if self.manifest['state'] != 'complete':
                            raise RuntimeError(self.manifest.get('reason', 'Recording incomplete'))
                        return
                time.sleep(.2)
        finally:
            with self.guard:
                if self.proc is not None:
                    error=self.reason()
                    result=self.finish('incomplete' if error else 'complete',error or 'Ctrl+C / recorder shutdown')
                    if self.output_file and not result.success:raise RuntimeError(result.message)


if __name__=='__main__':
    args=rospy.myargv()[1:]
    options=None
    if args:
        parser=argparse.ArgumentParser(description='Record data.bag, manifest.json and rosbag.log in one directory; Ctrl+C stops and saves.')
        parser.add_argument('--output',required=True,help='Recording directory; previous recording is overwritten after readiness checks')
        parser.add_argument('--allow-partial',action='store_true',help='Allow a ready subset of hardware')
        parser.add_argument('--arms',choices=('both','left','right'),default='both',help='Record both arms (default), or only left/right')
        options=parser.parse_args(args)
        if not options.output.strip():parser.error('--output must name a directory')
        output=Path(options.output).expanduser().absolute()
        error=output_error(output)
        if error:parser.error(error)
    # One recorder per ROS master. Hold the lock until bag finalization completes.
    lock_dir=Path.home()/'.ros';lock_dir.mkdir(exist_ok=True)
    master_key=hashlib.sha256(os.environ.get('ROS_MASTER_URI','http://localhost:11311').encode()).hexdigest()[:16]
    with (lock_dir/('agilex_recorder_'+master_key+'.lock')).open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Another AgileX recorder is running; stop it before starting a new recording.')
        rospy.init_node('agilex_recorder',anonymous=options is not None)
        if options:
            import rospkg
            import yaml
            config=Path(rospkg.RosPack().get_path('data_collection'))/'config/recording.yaml'
            settings=yaml.safe_load(config.read_text())
            settings['allow_partial']=options.allow_partial
            settings['arms']=options.arms
            rospy.set_param('~',settings)
        Recorder(options.output if options else None).run()
