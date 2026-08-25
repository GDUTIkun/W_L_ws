uint8 CONTACT_UNKNOWN=0
uint8 CONTACT_NO_CONTACT=1
uint8 CONTACT_CONTACT=2

uint64 sample_time_ns
geometry_msgs/Point base_position_n_m
geometry_msgs/Quaternion q_n_from_b
geometry_msgs/Vector3 base_linear_velocity_n_m_s
geometry_msgs/Vector3 base_angular_velocity_n_rad_s
float64[6] joint_position_rad
float64[6] joint_velocity_rad_s
uint8[2] contact_state
