"""ROS callbacks only collect data; hardware sends never run in callbacks."""
from collections import deque
import json
import threading
import time
import numpy as np
from .core import CAMERAS, selected_sides, joint_values, rgb_array


class Observations:
    def __init__(self, cfg, include_images=True):
        import rospy
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import String
        self.cfg = cfg
        self.sides = selected_sides(cfg)
        self.lock = threading.Lock()
        self.buffers, self.errors, self.subs = {}, {}, []
        self.hardware_mode = None
        self.hardware_config = None
        self.hardware_received = 0
        for cam, topic in (cfg['observations']['cameras'].items() if include_images else []):
            self.subs.append(rospy.Subscriber(topic, Image, self.image, cam, queue_size=2, buff_size=4*1024*1024))
        for side, prefix in cfg['observations']['arms'].items():
            for suffix, key, names in (
                ('joint_states', side+'_j', [side+'_follower_joint_'+str(i) for i in range(1, 7)]),
                ('gripper/actual_stamped', side+'_g', [side+'_follower_gripper_width'])):
                self.subs.append(rospy.Subscriber(prefix+'/'+suffix, JointState, self.joints,
                                                  (key, names), queue_size=30))
            self.subs.append(rospy.Subscriber(prefix+'/status', String, self.status, side+'_status', queue_size=5))
        self.subs.append(rospy.Subscriber('/agilex/hardware/status', String, self.hardware, queue_size=1))

    def put(self, key, stamp, value):
        now = time.time()
        if not np.isfinite(stamp) or stamp <= 0 or stamp > now+0.05:
            raise ValueError('Invalid/future header stamp')
        with self.lock:
            buf = self.buffers.setdefault(key, deque(maxlen=100 if key not in CAMERAS else 6))
            if buf and stamp <= buf[-1][0]:
                raise ValueError('Non-increasing source timestamp')
            buf.append((stamp, value, time.monotonic()))
            self.errors.pop(key, None)

    def failed(self, key, exc):
        with self.lock:
            self.errors[key] = str(exc)

    def image(self, msg, cam):
        try:
            self.put(cam, msg.header.stamp.to_sec(), rgb_array(msg, self.cfg['policy']['height'], self.cfg['policy']['width']))
        except Exception as exc:
            self.failed(cam, exc)

    def joints(self, msg, spec):
        key, names = spec
        try:
            self.put(key, msg.header.stamp.to_sec(), joint_values(msg, names))
        except Exception as exc:
            self.failed(key, exc)

    def status(self, msg, key):
        try:
            d = json.loads(msg.data)
            if d['err_code'] != 0 or d['arm_status'] != 0:
                raise ValueError('Arm fault: ' + msg.data)
            self.put(key, float(d['stamp']), d)
        except Exception as exc:
            self.failed(key, exc)

    def hardware(self, msg):
        try:
            d = json.loads(msg.data)
            with self.lock:
                self.hardware_mode = d.get('mode', 'record')
                self.hardware_config = d.get('config')
                self.hardware_received = time.monotonic()
        except Exception as exc:
            self.failed('hardware', exc)

    def _latest(self, key, age):
        if key in self.errors:
            raise RuntimeError(key + ': ' + self.errors[key])
        if not self.buffers.get(key):
            raise RuntimeError('Waiting for ' + key)
        value = self.buffers[key][-1]
        if time.time()-value[0] > age or time.monotonic()-value[2] > age:
            raise RuntimeError(key + ' data stale')
        return value

    def actual(self, status=True):
        with self.lock:
            parts = []
            for s in self.sides:
                if status:
                    self._latest(s+'_status', self.cfg['observations']['status_age_sec'])
                parts.extend(self._latest(s+k, self.cfg['observations']['state_age_sec'])[1] for k in ('_j', '_g'))
            return np.concatenate(parts)

    def snapshot(self):
        o = self.cfg['observations']
        with self.lock:
            for cam in CAMERAS:
                self._latest(cam, o['image_age_sec'])
            # Independent ROS callbacks need not arrive in exposure order.
            # Use the newest COMPLETE camera group, not an unmatched new cam_h.
            result, stamps = {}, {}
            for candidate in reversed(self.buffers['cam_h']):
                anchor = candidate[0]
                if time.time()-anchor > o['image_age_sec']:
                    break
                selected = {cam: min(self.buffers[cam], key=lambda x: abs(x[0]-anchor)) for cam in CAMERAS}
                if all(abs(v[0]-anchor) <= o['camera_skew_sec'] for v in selected.values()):
                    result = {cam: v[1] for cam, v in selected.items()}
                    stamps = {cam: v[0] for cam, v in selected.items()}
                    break
            if not result:
                raise RuntimeError('No complete fresh camera group within timestamp tolerance')
            parts = []
            for s in self.sides:
                self._latest(s+'_status', o['status_age_sec'])
                for suffix in ('_j', '_g'):
                    k = s+suffix
                    self._latest(k, o['state_age_sec'])
                    previous = [v for v in self.buffers[k] if v[0] <= anchor]
                    if not previous or anchor-previous[-1][0] > o['joint_alignment_sec']:
                        raise RuntimeError('No joint feedback aligned to RGB: ' + k)
                    parts.append(previous[-1][1])
                    stamps[k] = previous[-1][0]
            result['state'] = np.concatenate(parts).astype(np.float32)
        return result, anchor, stamps

    def deployment_hardware(self):
        with self.lock:
            if self.hardware_mode != 'deploy' or time.monotonic()-self.hardware_received > 2:
                raise RuntimeError('Start hardware with mode:=deploy and wait for status')
            cfg = self.hardware_config
            for s in self.sides:
                arm = cfg['arms'][s]
                if arm['channel'] != self.cfg['control']['channels'][s] or arm['model'] != 'piper':
                    raise RuntimeError('Hardware/control CAN mapping or model mismatch')
                if any(arm.get(k, 0) for k in ('command_id_offset','feedback_id_offset','motor_id_offset')):
                    raise RuntimeError('Deployment currently requires zero CAN ID offsets')
