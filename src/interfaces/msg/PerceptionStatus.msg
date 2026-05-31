builtin_interfaces/Time stamp
bool calibration_valid
string calibration_date_iso  # propagated from extrinsics.yaml
float64 calibration_age_days
bool depth_in_range          # current depth-noise sample within budget
float64 depth_noise_mm_p95   # rolling P95 over last 50 samples
string capability            # "READY" | "DEGRADED" | "DISABLED"
string detail
