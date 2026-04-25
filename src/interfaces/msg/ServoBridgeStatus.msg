# Servo bridge status — published by servo_bridge_node for HMI consumption.
#
# Consumed by: HMI backend jog_adapter, WebSocket stream to frontend JogPendent UI.

std_msgs/Header header

# Human-readable bridge state.
# One of: IDLE | STARTING | READY | ACTIVE | HALTING | HALTED | ERROR | TIMEOUT | BUSY_RETRY | REJECTED_NOT_READY | REJECTED_FJT_ACTIVE
string state

# Number of points currently queued in the MotoROS2 controller.
# Meaningful only when state == ACTIVE.
int32 points_queued

# Approximate forwarding rate from Servo output to queue_traj_point.
float64 effective_hz

# True when MotoROS2 robot is confirmed ready (drives powered, not in error/e-stop).
bool robot_ready

# True when MoveIt Servo node is confirmed healthy (status topic fresh).
bool servo_active

# True when bridge is actively forwarding Servo output to MotoROS2.
bool bridge_active

# Last error message. Empty string when state is not ERROR.
string last_error

# Human-readable detail about why activation was rejected (if applicable).
string rejection_reason
