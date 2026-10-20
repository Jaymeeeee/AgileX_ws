#!/usr/bin/python3
"""Receive-only SocketCAN -> timestamped ROS state/target topics."""
import json
import time
import can
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from hardware_bringup.config import load_config, check_can, check_identity, acquisition_can_mode
from hardware_bringup.decoder import Decoder
from hardware_bringup.msg import MotorFeedback
from hardware_bringup.messages import motor_message


def main():
    rospy.init_node('piper_state_bridge')
    cfg = load_config(rospy.get_param('~config'))
    side = rospy.get_param('~side')
    role = rospy.get_param('~role', 'follower')
    if role not in ('follower', 'leader'):
        raise ValueError('role must be follower or leader')
    arm = cfg['leaders' if role == 'leader' else 'arms'][side]
    settings = cfg['arm_settings']
    mode = rospy.get_param('~mode', 'record')
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Live CAN acquisition requires /use_sim_time=false')
    check_identity(arm)
    can_mode = acquisition_can_mode(cfg, mode)
    check_can(arm['channel'], settings['bitrate'], can_mode)
    decoder = Decoder(arm, settings['assembly_span_sec'])
    prefix = '/piper/'+side+('_leader' if role == 'leader' else '_bus')
    pubs = {kind:rospy.Publisher(prefix+'/'+kind, JointState, queue_size=100,
                               latch=kind in ('leader_targets','gripper/target_stamped')) for kind in
            ('joint_states', 'leader_targets', 'gripper/actual_stamped', 'gripper/target_stamped')}
    # In deployment these frames can be computer commands, not demonstrations.
    suppress_targets = role == 'follower' and (mode == 'deploy' or cfg.get('topology') == 'independent_can')
    if suppress_targets:
        for kind in ('leader_targets', 'gripper/target_stamped'):
            pubs[kind].unregister()
            del pubs[kind]
    status_pub = rospy.Publisher(prefix+'/status', String, queue_size=30)
    gripper_pub = rospy.Publisher(prefix+'/gripper/status', String, queue_size=30)
    motor_pub = rospy.Publisher(prefix+'/motor_feedback', MotorFeedback, queue_size=30)
    events_pub = rospy.Publisher(prefix+'/events', String, queue_size=200)
    # No send API is used. No SDK connect/enable/control calls exist in this bridge.
    bus = can.Bus(interface='socketcan', channel=arm['channel'], receive_own_messages=False)
    next_check = time.monotonic()
    rospy.loginfo('Receive-only %s: %s -> %s', side, arm['channel'], prefix)
    try:
        while not rospy.is_shutdown():
            if time.monotonic() >= next_check:
                check_identity(arm)
                check_can(arm['channel'], settings['bitrate'], can_mode)
                next_check = time.monotonic()+settings['can_recheck_sec']
            frame = bus.recv(timeout=0.1)
            if frame is None:
                continue
            age = time.time()-frame.timestamp
            if age < -0.1 or age > settings['max_sample_age_sec']:
                raise RuntimeError('CAN clock/backlog age outside limit: '+str(age))
            event = decoder.feed(frame)
            if event is None:
                continue
            if suppress_targets and event['kind'] in ('leader_targets','gripper/target_stamped'):
                event['command_source'] = 'bus_command_echo_not_leader_measurement'
                events_pub.publish(String(data=json.dumps(event, separators=(',', ':'), allow_nan=False)))
                continue
            payload = String(data=json.dumps(event, separators=(',', ':'), allow_nan=False))
            events_pub.publish(payload)
            if event['kind'] == 'status':
                status_pub.publish(payload)
            elif event['kind'] == 'motor_feedback':
                motor_pub.publish(motor_message(event))
            else:
                msg = JointState()
                msg.header.stamp = rospy.Time.from_sec(event['stamp'])
                label = 'leader_target' if event['kind'] in ('leader_targets','gripper/target_stamped') else role
                msg.name = ([side+'_'+label+'_joint_'+str(i) for i in range(1,7)]
                            if len(event['position'])==6 else [side+'_'+label+'_gripper_width'])
                msg.position = event['position']
                # Missing velocity/effort are empty, never fabricated zero torque.
                pubs[event['kind']].publish(msg)
                if event['kind'] == 'gripper/actual_stamped':
                    gripper_pub.publish(payload)
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
