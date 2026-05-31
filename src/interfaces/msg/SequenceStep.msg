# One step of a blended sequence.
uint8 GOAL_POSE=0
uint8 GOAL_JOINTS=1
uint8 GOAL_NAMED=2

uint8 goal_type
string primitive_type           # PTP / LIN / CIRC / HOME
geometry_msgs/Pose target_pose
float64[] joint_target
string named_target            # "home", "start"
geometry_msgs/Pose auxiliary_pose  # CIRC interim waypoint (on-arc via-point)
float64 blend_radius_m
float64 velocity_scale
float64 acceleration_scale
string planner_id
