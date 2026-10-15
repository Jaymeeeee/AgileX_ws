# Latest receive timestamp of the six CAN frames; individual stamps are in events.
std_msgs/Header header
uint8[6] joint_order
# Each array follows joint_order. SDK motor-shaft states are kept separate from joint_states.
float64[6] current_a
float64[6] motor_position_rad
float64[6] motor_velocity_rad_s
# Current-based drive torque estimate, NOT external wrench or a torque sensor reading.
# Six values for a confirmed model; empty if model/coefficients are unknown.
float64[] torque_estimate_nm
