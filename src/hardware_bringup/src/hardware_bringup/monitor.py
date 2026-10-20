import json
import math
import threading
import time
from collections import deque
import rospy
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String
from hardware_bringup.msg import MotorFeedback


def summarize(msg, spec):
    kind = spec['kind']
    if kind == 'image':
        bpp = 3 if spec['encoding']=='rgb8' else 2
        if (msg.width!=spec['width'] or msg.height!=spec['height'] or msg.encoding!=spec['encoding']
                or msg.step<msg.width*bpp or len(msg.data)!=msg.step*msg.height):
            raise ValueError('Image encoding/dimensions/data length mismatch')
        return '{}x{} {} frame={}'.format(msg.width,msg.height,msg.encoding,msg.header.frame_id)
    if kind == 'info':
        if msg.width<=0 or msg.height<=0 or not all(math.isfinite(v) for v in msg.K) or msg.K[0]<=0 or msg.K[4]<=0:
            raise ValueError('Invalid camera intrinsics')
        return '{}x{} fx={:.2f} fy={:.2f}'.format(msg.width,msg.height,msg.K[0],msg.K[4])
    if kind == 'joint':
        if len(msg.position)!=spec['size'] or len(msg.name)!=spec['size'] or not all(math.isfinite(v) for v in msg.position):
            raise ValueError('Invalid joint/gripper vector')
        return 'position={} velocity={} effort={}'.format([round(x,5) for x in msg.position],list(msg.velocity),list(msg.effort))
    if kind=='motor_feedback':
        for field in ('current_a','motor_velocity_rad_s','motor_position_rad'):
            values=getattr(msg,field)
            if len(values)!=6 or not all(math.isfinite(v) for v in values):
                raise ValueError('Invalid six-motor feedback: '+field)
        if list(msg.joint_order)!=list(range(1,7)):
            raise ValueError('Motor order mismatch')
        torque=msg.torque_estimate_nm
        if spec['model']=='unknown':
            if len(torque):
                raise ValueError('Unconfirmed model must not report torque')
        elif len(torque)!=6 or not all(math.isfinite(v) for v in torque):
            raise ValueError('Invalid six-joint torque estimate')
        return 'current_A={} torque_estimate_Nm={} ({})'.format(
            [round(v,4) for v in msg.current_a], [round(v,4) for v in torque],
            'current_based_estimate' if len(torque) else 'unavailable_model_unconfirmed')
    data=json.loads(msg.data)
    if kind=='teleop_status':
        if data.get('side') != spec['side'] or data.get('role') != spec['role']:
            raise ValueError('Teleop status identity mismatch')
        if data.get('fault') or data.get('phase') != spec['phase']:
            raise ValueError('Teleoperation not active: '+str(data.get('phase')))
        if spec['role'] == 'follower' and not data.get('gripper_ready'):
            raise ValueError('Gripper hand input has not picked up follower opening')
        age=data.get('can_age_sec')
        if not isinstance(age,(int,float)) or not math.isfinite(age) or not 0 <= age < 1.5:
            raise ValueError('Teleop CAN source stale')
        return '{} {} {}'.format(data['side'],data['role'],data['phase'])
    if kind=='status':
        if data['err_code'] or data['arm_status'] not in (0,11,12,13):
            raise ValueError('Arm reports fault: '+msg.data)
        return 'ctrl_mode={} arm_status={} err_code={}'.format(data['ctrl_mode'],data['arm_status'],data['err_code'])
    if kind=='gripper_status':
        # Bits 0..5 indicate device faults. Bit 6 is enable state, not a fault.
        if int(data['status_code']) & 0x3f:
            raise ValueError('Gripper reports fault: '+msg.data)
        return 'width_m={} status_code={} protocol_force={}'.format(data['position'][0],data['status_code'],data['protocol_force'])
    raise ValueError('Unknown topic kind')


class TopicMonitor:
    def __init__(self, specs):
        self.specs=specs
        self.lock=threading.Lock()
        self.state={s['topic']:dict(count=0,last=None,stamp=None,summary='',error='',times=deque(maxlen=100)) for s in specs}
        types={'image':Image,'info':CameraInfo,'joint':JointState,'status':String,'gripper_status':String,'motor_feedback':MotorFeedback,'teleop_status':String}
        self.subs=[rospy.Subscriber(s['topic'],types[s['kind']],self.callback,s,
                                   queue_size=3,buff_size=4*1024*1024) for s in specs]

    def callback(self,msg,spec):
        now=time.monotonic()
        try:
            summary=summarize(msg,spec)
            stamp=msg.header.stamp.to_sec() if hasattr(msg,'header') else float(json.loads(msg.data)['stamp'])
            if not math.isfinite(stamp) or stamp<=0 or not -0.2 <= time.time()-stamp <= max(3,spec['timeout']):
                raise ValueError('Invalid/stale source timestamp')
            with self.lock:
                state=self.state[spec['topic']]
                if state['stamp'] is not None and stamp<=state['stamp']:
                    raise ValueError('Repeated/backwards source timestamp')
                state.update(count=state['count']+1,last=now,stamp=stamp,summary=summary,error='')
                state['times'].append(now)
        except (ValueError,KeyError,TypeError) as exc:
            with self.lock:
                self.state[spec['topic']]['error']=str(exc)

    def snapshot(self, group=None):
        now=time.monotonic()
        items=[]
        with self.lock:
            for spec in self.specs:
                if group and spec['group']!=group:
                    continue
                state=self.state[spec['topic']]
                age=None if state['last'] is None else now-state['last']
                error=state['error']
                if state['count']<spec['minimum_samples']:
                    error=error or 'waiting for valid messages'
                elif spec['timeout'] and age>spec['timeout']:
                    error='stream stale: {:.2f}s'.format(age)
                times=state['times']
                hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0
                items.append(dict(topic=spec['topic'],kind=spec['kind'],meaning=spec['meaning'],
                                  ok=not error,error=error,count=state['count'],age_sec=age,hz=round(hz,2),
                                  source_stamp=state['stamp'],summary=state['summary'],
                                  event_like=spec['timeout']==0))
        return items

    def close(self):
        for sub in self.subs:sub.unregister()
