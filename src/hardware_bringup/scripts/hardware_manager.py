#!/usr/bin/python3
"""Start selected devices in order; declare readiness only after real messages."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rospy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from hardware_bringup.config import load_config,select,topic_specs,camera_command,check_can
from hardware_bringup.monitor import TopicMonitor


def blocking_rows(rows, require_leader_targets):
    """Missing target echoes may degrade acquisition without stopping feedback."""
    return [r for r in rows if not r['ok'] and
            (require_leader_targets or not (
                r['topic'].endswith(('/leader_targets', '/gripper/target_stamped'))
                and r['count'] == 0 and r['error'] == 'waiting for valid messages'))]


class Manager:
    def __init__(self):
        self.cfg_path=rospy.get_param('~config')
        self.cfg=load_config(self.cfg_path)
        self.mode=rospy.get_param('~mode','record')
        if self.mode not in ('record','deploy'):raise ValueError('mode must be record or deploy')
        fps=int(rospy.get_param('~fps',0))
        if fps:
            if fps not in (6,15,30):raise ValueError('fps must be 6, 15 or 30')
            self.cfg['camera_settings']['fps']=fps
        self.arms=select(rospy.get_param('~arms','all'),self.cfg['arms'])
        self.cameras=select(rospy.get_param('~cameras','all'),self.cfg['cameras'])
        if not self.arms and not self.cameras:raise ValueError('Select at least one device')
        self.specs=topic_specs(self.cfg,self.arms,self.cameras)
        if self.mode=='deploy':
            self.specs=[s for s in self.specs if not s['topic'].endswith(('/leader_targets','/gripper/target_stamped'))]
        self.pub=rospy.Publisher('/agilex/hardware/status',String,queue_size=1,latch=True)
        self.diag=rospy.Publisher('/agilex/hardware/diagnostics',DiagnosticArray,queue_size=1)
        self.children=[]
        self.monitor=None
        self.active_groups=[]
        self.last_status_time=0
        self.calibration={}
        self.require_leader_targets=bool(rospy.get_param('~require_leader_targets',False))

    def publish(self,state,reason=''):
        rows=self.monitor.snapshot() if self.monitor else []
        record_topics=[s['topic'] for s in self.specs]
        record_topics += ['/piper/'+side+'_bus/events' for side in self.arms]
        record_topics += ['/agilex/hardware/status','/agilex/hardware/diagnostics']
        payload=dict(state=state,reason=reason,stamp=time.time(),mode=self.mode,arms=self.arms,cameras=self.cameras,
                     config=self.cfg,calibration=self.calibration,topics=rows,record_topics=record_topics)
        self.pub.publish(String(data=json.dumps(payload,ensure_ascii=False)))
        diag=DiagnosticArray();diag.header.stamp=rospy.Time.now()
        diag.status=[DiagnosticStatus(name='agilex/system',level=0 if state=='ready' else (1 if state=='degraded' else 2),
                                      message=state+': '+reason)]
        for row in rows:
            diag.status.append(DiagnosticStatus(name=row['topic'],level=0 if row['ok'] else 2,
                message=row['summary'] if row['ok'] else row['error'],
                values=[KeyValue('age_sec',str(row['age_sec'])),KeyValue('hz',str(row['hz']))]))
        self.diag.publish(diag)

    def children_ok(self):
        for name,proc in self.children:
            if proc.poll() is not None:
                raise RuntimeError('{} process exited: {}'.format(name,proc.returncode))
        for group in self.active_groups:
            bad=blocking_rows(self.monitor.snapshot(group),self.require_leader_targets)
            if bad:raise RuntimeError(bad[0]['topic']+': '+bad[0]['error'])

    def spawn(self,name,cmd):
        rospy.loginfo('Starting %s: %s',name,' '.join(cmd))
        self.children.append((name,subprocess.Popen(cmd,start_new_session=True)))

    def wait_group(self,group,timeout):
        deadline=time.monotonic()+timeout
        while not rospy.is_shutdown():
            self.children_ok()
            rows=self.monitor.snapshot(group)
            if rows and not blocking_rows(rows,self.require_leader_targets):
                self.active_groups.append(group)
                missing=[r['topic'] for r in rows if not r['ok']]
                if missing:rospy.logwarn('%s feedback ready; target echoes not ready: %s',group,', '.join(missing))
                else:rospy.loginfo('%s data ready',group)
                return
            if time.monotonic()>deadline:
                raise RuntimeError('Timeout '+group+': '+'; '.join(r['topic']+' '+r['error'] for r in rows if not r['ok']))
            if time.monotonic()-self.last_status_time>1:
                self.publish('starting','waiting for '+group);self.last_status_time=time.monotonic()
            time.sleep(0.1)
        raise rospy.ROSInterruptException('shutdown')

    def run(self):
        if rospy.get_param('/use_sim_time',False):raise RuntimeError('Live hardware requires /use_sim_time=false')
        existing=dict(rospy.get_published_topics('/'))
        conflicts=[s['topic'] for s in self.specs if s['topic'] in existing]
        if conflicts:raise RuntimeError('Topics already published, stop old drivers: '+str(conflicts))
        # Check every selected CAN before starting any camera or arm instance.
        for side in self.arms:check_can(self.cfg['arms'][side]['channel'],self.cfg['arm_settings']['bitrate'],self.mode)
        if rospy.get_param('~dry_run',False):
            for s in self.specs:rospy.loginfo('%s : %s',s['topic'],s['meaning'])
            self.publish('dry_run','configuration/preflight only; no devices started')
            return
        self.monitor=TopicMonitor(self.specs)
        for side in self.arms:
            self.spawn('arm:'+side,['rosrun','hardware_bringup','piper_state_bridge.py',
                '__name:=agilex_'+side+'_state','_config:='+self.cfg_path,'_side:='+side,'_mode:='+self.mode])
            self.wait_group('arm:'+side,self.cfg['arm_settings']['startup_timeout_sec'])
        for name in self.cameras:
            self.spawn('camera:'+name,camera_command(self.cfg,name))
            self.wait_group('camera:'+name,self.cfg['camera_settings']['startup_timeout_sec'])
            self.calibration[name]=rospy.get_param('/'+name+'/calibration')
            until=time.monotonic()+self.cfg['camera_settings']['stagger_sec']
            while time.monotonic()<until and not rospy.is_shutdown():
                self.children_ok();time.sleep(.1)
        self.publish_running()
        rospy.loginfo('Topic 与实际内容如下（缺失数据单独标记）：')
        for row in self.monitor.snapshot():
            if row['ok']:
                rospy.loginfo('%s | %s | %s | %.2f Hz',row['topic'],row['meaning'],row['summary'],row['hz'])
            else:rospy.logwarn('%s | 未就绪: %s',row['topic'],row['error'])
        while not rospy.is_shutdown():
            self.children_ok();self.publish_running();time.sleep(.5)

    def publish_running(self):
        missing=[r['topic']+': '+r['error'] for r in self.monitor.snapshot() if not r['ok']]
        state='degraded' if missing else 'ready'
        reason='; '.join(missing)
        if getattr(self,'last_running_state',None)!=state:
            if missing:rospy.logwarn('DEGRADED: 可用反馈和图像继续发布；未就绪: %s',reason)
            else:rospy.loginfo('READY: 已收到所选设备的全部有效数据。')
            self.last_running_state=state
        self.publish(state,reason)

    def close(self):
        for name,proc in reversed(self.children):
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
        for name,proc in reversed(self.children):
            try:proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        if self.monitor:self.monitor.close()


def main():
    rospy.init_node('agilex_hardware')
    manager=Manager()
    failed=False
    try:manager.run()
    except rospy.ROSInterruptException:pass
    except Exception as exc:
        failed=True
        manager.publish('error',str(exc));rospy.logerr('Hardware not ready: %s',exc)
        raise
    finally:
        manager.close()
        if not rospy.is_shutdown() and not failed:manager.publish('stopped','drivers stopped')


if __name__=='__main__':main()
