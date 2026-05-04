# One step of a blended LIN sequence (W2.T0).
string primitive_type           # "LIN" or "PTP" or "CIRC"; "LIN" for drawing
geometry_msgs/Pose target_pose
float64 blend_radius_m          # 0.0 for first/last, > 0.0 for intermediates
string planner_id               # "PILZ_LIN" or "PILZ_PTP"
float64 velocity_scale          # 0.0–1.0; required (per W1 fail-closed)
float64 acceleration_scale      # 0.0–1.0; required
