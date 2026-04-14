# Experimental joint jog command from HMI web pendant to jog_input_node.
#
# This message carries a single joint jog request.
# The jog_input_node translates this into MoveIt Servo JointJog input.
#
# Mode semantics:
#   "continuous" — hold-to-run: jog continues while heartbeat is active.
#                   watchdog > 200ms without heartbeat → halt.
#   "discrete"  — single-step: one JointJog message per command.
#                   suitable for incremental moves via JointJog.displacements.

std_msgs/Header header

# Which joint to jog (0-based index into gp4_arm joint list).
# Valid range: [0, 5]
#   0 = joint_1_s
#   1 = joint_2_l
#   2 = joint_3_u
#   3 = joint_4_r
#   4 = joint_5_b
#   5 = joint_6_t
int8 joint_index

# Direction: +1 = positive (increasing angle), -1 = negative.
# Rejected if not -1 or +1.
int8 direction

# Step size in degrees (used in discrete mode or as magnitude hint in continuous).
# Clamped to [0.01, 10.0] degrees by jog_input_node.
float64 step_degrees

# Velocity scale applied to the joint's configured max velocity.
# Clamped to (0.0, 0.3] by jog_input_node (conservative default for experimental).
float64 velocity_scale

# Operating mode: "continuous" or "discrete".
# Defaults to "continuous" if empty or invalid.
string mode
