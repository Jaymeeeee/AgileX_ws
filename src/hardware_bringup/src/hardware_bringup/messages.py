"""Convert decoded events to compact typed ROS messages without rounding samples."""
import rospy
from hardware_bringup.msg import MotorFeedback


def motor_message(event):
    msg = MotorFeedback()
    msg.header.stamp = rospy.Time.from_sec(event['stamp'])
    for field in ('joint_order', 'current_a', 'motor_position_rad',
                  'motor_velocity_rad_s', 'torque_estimate_nm'):
        setattr(msg, field, list(event[field]))
    return msg
