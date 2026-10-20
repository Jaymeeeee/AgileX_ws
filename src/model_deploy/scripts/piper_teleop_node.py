#!/usr/bin/python3
"""One independent Piper leader or follower ROS endpoint; no policy loaded."""
import json
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse

from model_deploy.teleop_core import CommandInbox, GripperTakeover, load_teleop, names
from model_deploy.teleop_device import TeleopDevice


class Node:
    def __init__(self):
        self.cfg = load_teleop(rospy.get_param('~config'), arms=rospy.get_param('~arms', None))
        self.side, self.role = rospy.get_param('~side'), rospy.get_param('~role')
        if self.side not in ('left', 'right') or self.role not in ('leader', 'follower'):
            raise ValueError('Invalid teleop side/role')
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError('Live teleoperation requires /use_sim_time=false')
        self.execute = bool(rospy.get_param('~execute', False))
        self.device = TeleopDevice(self.cfg, self.side, self.role)
        self.prefix = '/agilex/teleop/'+self.side+'_'+self.role
        self.status_pub = rospy.Publisher(self.prefix+'/status', String, queue_size=1)
        self.state_pub = rospy.Publisher(self.prefix+'/state', JointState, queue_size=1)
        self.sent_pub = rospy.Publisher(self.prefix+'/commanded', JointState, queue_size=1)
        self.command_pub = rospy.Publisher('/piper/'+self.side+'_leader/command', JointState, queue_size=1)
        if self.role != 'leader':
            self.command_pub.unregister()
        self.compat = None
        if self.role == 'leader':
            self.compat = [rospy.Publisher('/piper/'+self.side+'_bus/'+suffix, JointState,
                                          queue_size=1, latch=True) for suffix in ('leader_targets', 'gripper/target_stamped')]
        self.inbox = CommandInbox(self.side, self.cfg['source_timeout_sec'])
        self.lock = threading.RLock()
        self.follow = False
        self.service_busy = False
        self.service_holding = False
        self.peer = None
        self.peer_status = None
        self.seed = self.seed_stamp = None
        self.compat_stamps = [None, None]
        self.error = None
        self.gripper_takeover = None
        if self.role == 'follower':
            rospy.Subscriber('/piper/'+self.side+'_leader/command', JointState, self.command, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber('/agilex/teleop/'+self.side+'_leader/status', String, self.source, queue_size=1, tcp_nodelay=True)
            self.service = rospy.Service('~follow', SetBool, self.set_follow)
            self.reset_service = rospy.Service('~reset', Trigger, self.reset)
        else:
            peer_prefix = '/agilex/teleop/'+self.side+'_follower'
            rospy.Subscriber(peer_prefix+'/state', JointState, self.peer_state, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber(peer_prefix+'/status', String, self.peer_health, queue_size=1, tcp_nodelay=True)
            self.service = rospy.Service('~align', Trigger, self.align)
            self.align_hold_service = rospy.Service('~align_hold', Trigger, lambda req: self.align(req, teach=False))
            self.hold_service = rospy.Service('~hold', Trigger, self.hold)

    def command(self, msg):
        try:
            self.inbox.put_command(msg, getattr(msg, '_connection_header', {}).get('callerid'))
        except Exception as exc:
            self.inbox.error = str(exc)

    def source(self, msg):
        try:
            self.inbox.put_source(json.loads(msg.data), getattr(msg, '_connection_header', {}).get('callerid'))
        except Exception as exc:
            self.inbox.error = str(exc)

    def peer_state(self, msg):
        if list(msg.name) != names(self.side, 'follower') or len(msg.position) != 7 or not np.isfinite(msg.position).all():
            self.error = 'Invalid paired follower feedback'
            return
        with self.lock:
            self.peer = (np.asarray(msg.position), msg.header.stamp.to_sec(), time.monotonic())

    def peer_health(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('side') != self.side or data.get('role') != 'follower':
                raise ValueError('Paired follower identity mismatch')
            with self.lock:
                self.peer_status = (data, time.monotonic())
        except Exception as exc:
            self.error = str(exc)

    def set_follow(self, request):
        with self.lock:
            return self._set_follow(request)

    def _set_follow(self, request):
        try:
            if self.service_busy:
                raise RuntimeError('Maintenance in progress')
            self.device.check()
            if request.data:
                if not self.execute or not self.device.enabled:
                    raise RuntimeError('Follower not enabled in execute mode')
                target = self.inbox.latest()
                actual = self.device.actual()
                limits = self.device.guard.initial_error[:7]
                if np.any(np.abs(target[:6]-actual[:6]) > limits[:6]):
                    raise RuntimeError('Leader/follower start mismatch: target={} actual={} diff={} limits={}'.format(
                        target.tolist(), actual.tolist(), np.abs(target-actual).tolist(), limits.tolist()))
            with self.lock:
                if request.data:
                    self.gripper_takeover = GripperTakeover(actual[6], self.cfg['alignment_gripper_tolerance_m'])
                if not request.data and self.execute and self.device.enabled:
                    self.device.hold_target = self.device.actual()
                    self.device.guard.commit(np.tile(self.device.hold_target, 2))
                if self.execute and self.device.enabled:
                    self.device.robot.set_speed_percent(self.cfg['sdk_speed_percent'] if request.data
                                                       else self.cfg['alignment_sdk_speed_percent'])
                self.follow = bool(request.data)
            return SetBoolResponse(True, 'following' if request.data else 'holding')
        except Exception as exc:
            return SetBoolResponse(False, str(exc))

    def controlled_hold(self):
        """Leave drag mode at the current measured pose, without an old target."""
        if self.device.phase == 'teaching':
            self.device.phase = 'switching'
            self.device.robot.set_follower_mode()
            self.device.teach_frame(False)
            self.device.wait_actual(8)
            self.device.enable_hold()
        with self.lock:
            self.seed = self.seed_stamp = None
        self.device.hold_target = self.device.actual()
        self.device.guard.commit(np.tile(self.device.hold_target, 2))
        self.device.send(self.device.hold_target)
        self.device.phase = 'holding'

    def hold(self, _request):
        if not self.execute or not self.device.enabled:
            return TriggerResponse(False, 'Hold requires enabled execute mode')
        with self.lock:
            if self.service_busy:
                return TriggerResponse(False, 'Maintenance already in progress')
            self.service_busy = True
        try:
            self.controlled_hold()
            return TriggerResponse(True, 'Leader holding measured pose')
        except Exception as exc:
            self.device.stop('Hold failed: '+str(exc))
            return TriggerResponse(False, str(exc))
        finally:
            with self.lock:
                self.service_busy = False

    def reset(self, _request):
        if not self.execute or not self.device.enabled:
            return TriggerResponse(False, 'Reset requires enabled execute mode')
        with self.lock:
            if self.service_busy or self.follow:
                return TriggerResponse(False, 'Pause following before reset; no concurrent maintenance')
            self.service_busy = self.service_holding = True
        try:
            from model_deploy.teleop_reset import EndpointReset
            reset = EndpointReset(self.device)
            lift = reset.cfg['reset'].get('vertical_lift')
            if lift is not None and lift.get('mode') == 'joint_path':
                # Main loop maintains the fixed hold while limit queries/IK run.
                reset.prepare_joint_lift(cancelled=rospy.is_shutdown)
            with self.lock:
                self.service_holding = False
                self.device.phase = 'resetting'
            reset.run(rospy.is_shutdown, lambda event: rospy.loginfo(
                '%s reset: %s', self.side, json.dumps(event, ensure_ascii=False)))
            self.device.robot.set_speed_percent(self.cfg['alignment_sdk_speed_percent'])
            self.device.phase = 'holding'
            return TriggerResponse(True, 'Follower reset complete; holding fixed home pose')
        except Exception as exc:
            self.device.stop('Reset failed: '+str(exc))
            return TriggerResponse(False, str(exc))
        finally:
            with self.lock:
                self.service_busy = self.service_holding = False

    def align(self, _request, teach=True):
        if not self.execute:
            return TriggerResponse(False, 'Dry-run cannot move/enable arms')
        with self.lock:
            if self.service_busy:
                return TriggerResponse(False, 'Alignment already in progress')
            peer, health = self.peer, self.peer_status
            self.service_busy = True
        try:
            if self.error:
                raise RuntimeError(self.error)
            if not peer or not health:
                raise RuntimeError('Waiting for paired follower holding feedback')
            if (time.time()-peer[1] > self.cfg['feedback_timeout_sec'] or
                    time.monotonic()-peer[2] > self.cfg['feedback_timeout_sec'] or
                    time.monotonic()-health[1] > self.cfg['source_timeout_sec'] or
                    health[0].get('phase') != 'holding'):
                raise RuntimeError('Paired follower must be fresh and holding')
            target = peer[0].copy()
            self.controlled_hold()
            waypoint = self.cfg['alignment_waypoint_rad']
            if waypoint is not None:
                self.device.align(np.r_[waypoint, self.device.actual()[6]], rospy.is_shutdown)
            self.device.align(target, rospy.is_shutdown)
            # Require the follower to have remained held throughout alignment.
            with self.lock:
                latest, health = self.peer, self.peer_status
            if (not latest or time.monotonic()-latest[2] > self.cfg['feedback_timeout_sec'] or
                    not health or time.monotonic()-health[1] > self.cfg['source_timeout_sec'] or
                    health[0].get('phase') != 'holding' or
                    np.max(np.abs(latest[0][:6]-target[:6])) > self.cfg['alignment_joint_tolerance_rad'] or
                    abs(latest[0][6]-target[6]) > self.cfg['alignment_gripper_tolerance_m']):
                raise RuntimeError('Follower moved/lost feedback during leader alignment')
            if teach:
                with self.lock:
                    self.seed = self.device.enter_teach()
                    self.seed_stamp = time.time()
            else:
                self.device.hold_target = self.device.actual()
            return TriggerResponse(True, 'Aligned; leader is '+('in drag teaching mode' if teach else 'holding'))
        except Exception as exc:
            self.device.stop('Alignment failed: '+str(exc))
            return TriggerResponse(False, str(exc))
        finally:
            with self.lock:
                self.service_busy = False

    def publish(self):
        age = time.monotonic()-self.device.last_rx if self.device.last_rx else 1e6
        phase = self.device.phase
        if self.role == 'follower' and self.follow:
            phase = 'following'
        self.status_pub.publish(String(data=json.dumps(dict(
            stamp=time.time(), side=self.side, role=self.role, phase=phase, can_age_sec=age,
            channel=self.device.spec['channel'], firmware=self.device.firmware,
            gripper_ready=self.gripper_takeover.ready if self.gripper_takeover else False,
            fault=self.device.fault or self.device.rx_error), allow_nan=False)))
        try:
            actual = self.device.actual()
        except RuntimeError:
            actual = None
        if actual is not None:
            msg = JointState()
            with self.device.event_lock:
                stamp = max(self.device.events[k]['stamp'] for k in ('joint_states', 'gripper/actual_stamped'))
            msg.header.stamp = rospy.Time.from_sec(stamp)
            msg.name, msg.position = names(self.side, self.role), actual.tolist()
            self.state_pub.publish(msg)
        with self.lock:
            seed = (self.seed.copy(), self.seed_stamp) if (
                self.role == 'leader' and self.device.phase == 'teaching' and self.seed is not None
                and not self.service_busy) else None
        if seed is not None:
            target, stamp, stamps = self.device.leader_target(*seed)
            msg = JointState()
            msg.header.stamp = rospy.Time.from_sec(stamp)
            msg.header.frame_id = 'native_gripper' if stamps[1] > seed[1] else 'aligned_gripper_seed'
            msg.name, msg.position = names(self.side, 'leader_target'), target.tolist()
            self.command_pub.publish(msg)
            # Compatibility with the existing rosbag -> LeRobot conversion.
            # Source stamps are retained, never replaced with publication time.
            for i, (pub, indices, original) in enumerate(zip(self.compat, (slice(0, 6), slice(6, 7)), stamps)):
                if self.compat_stamps[i] is not None and original <= self.compat_stamps[i]:
                    continue
                part = JointState()
                part.header.stamp = rospy.Time.from_sec(original)
                part.name, part.position = msg.name[indices], msg.position[indices]
                pub.publish(part)
                self.compat_stamps[i] = original

    def run(self):
        self.device.connect(configure=self.execute)
        if self.execute:
            self.device.enable_hold()
        rate, next_ping = rospy.Rate(self.cfg['fps']), 0.0
        while not rospy.is_shutdown():
            self.device.check()
            if self.error:
                raise RuntimeError(self.error)
            with self.lock:
                if self.execute and self.service_busy:
                    if self.service_holding:
                        self.device.send(self.device.hold_target)
                elif self.execute:
                    if self.role == 'follower':
                        if self.follow:
                            target, native = self.inbox.latest(with_provenance=True)
                            target[6] = self.gripper_takeover.update(target[6], native)
                        else:
                            target = self.device.hold_target
                        sent = self.device.send(target, following=self.follow)
                        msg = JointState()
                        msg.header.stamp = rospy.Time.now()
                        msg.name, msg.position = names(self.side, 'follower'), sent.tolist()
                        self.sent_pub.publish(msg)
                    elif self.device.phase == 'holding':
                        self.device.send(self.device.hold_target)
            if self.role == 'leader' and self.execute and time.monotonic() >= next_ping:
                self.device.ping()
                next_ping = time.monotonic()+.5
            self.publish()
            rate.sleep()


def main():
    rospy.init_node('piper_teleop')
    node = Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.device.close()


if __name__ == '__main__':
    main()
