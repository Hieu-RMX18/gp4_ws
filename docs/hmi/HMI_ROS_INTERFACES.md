# HMI ROS Interface Inventory — W0.T9

**Date:** 2026-05-04
**Purpose:** Catalog all ROS surfaces consumed by `hmi/backend/` and `hmi/frontend/`
so that later waves (W2–W7) can assess HMI breaking-change risk before modifying
any ROS topic, service, or action.

## Discovery evidence

```
$ rg -n -e 'rosbridge|websocket|rclpy' hmi/backend/ hmi/shared/ hmi/frontend/bridgeClient.ts
hmi/backend/ros/adapter.py:35:    import rclpy
hmi/backend/ros/adapter.py:43:    from rclpy.action import ActionClient
hmi/backend/ros/adapter.py:44:    from rclpy.executors import SingleThreadedExecutor
hmi/backend/ros/adapter.py:45:    from rclpy.qos import qos_profile_sensor_data
hmi/backend/services/jog_service.py:25:    import rclpy
hmi/backend/api/app.py:297:    @app.websocket('/api/hmi/stream')

$ rg -n -e 'topic_name|service_name|action_name' hmi/backend/
hmi/backend/ros/command_dispatch.py:486:            action_name=self._execute_motion_action,
```

## Transport layer

HMI backend creates its own `rclpy` context and node (`hmi/backend/ros/adapter.py:160-168`).
It connects to ROS via native rclpy, NOT via rosbridge/websocket on the ROS side.
HMI frontend connects to the HMI backend via WebSocket (`/api/hmi/stream`), NOT directly to ROS.

## ROS surface inventory

| ROS surface | Type | Used by (file:line) | Direction | Message type | Change sensitivity |
|-------------|------|---------------------|-----------|--------------|-------------------|
| `/gateway_status` | Topic | `adapter.py:86,115` | Subscribe | `std_msgs/String` [INFERRED] | HIGH |
| `/llm_debug` | Topic | `adapter.py:87,116` | Subscribe | `std_msgs/String` [INFERRED] | MEDIUM |
| `/llm_command` | Topic | `adapter.py:88,117` | Subscribe | `std_msgs/String` [INFERRED] | HIGH |
| `/llm_text_input` | Topic | Deprecated; not listed in `KNOWN_WORKSPACE_ENDPOINTS.write_capable_interfaces` | Disabled HMI ingress | `std_msgs/String` [INFERRED] | HIGH |
| `/llm_gateway/review_intent` | Service | `adapter.py:121,157,662`; `telemetry_snapshot.py:307` | Client | `interfaces/ReviewIntent` (`raw_text`, `runtime_mode`, HMI metadata, `review_token`) | HIGH |
| `/validate_command` | Service | `adapter.py:117,152`; `command_dispatch.py:451` | Client | `interfaces/ValidateCommand` [INFERRED] | HIGH |
| `/execute_motion` | Action | `adapter.py:118,153`; `command_dispatch.py:500` | Client | `interfaces/ExecuteMotion` | HIGH |
| `/get_current_pose` | Service | `adapter.py:154`; `supervisor_execution.py:406` | Client | `interfaces/GetCurrentPose` [INFERRED] | MEDIUM |
| `/start_trajectory_recording` | Service | `adapter.py:378` [INFERRED] | Client | Unknown | LOW |
| `/reset_error` | Service | `adapter.py:382` [INFERRED] | Client | `std_srvs/Trigger` [INFERRED] | LOW |
| Jog command topic | Topic | `jog_service.py:303` | Publish | `JogCommand` [INFERRED] | MEDIUM |
| Jog activate/deactivate | Service | `jog_service.py:312,315` | Client | Unknown | MEDIUM |
| `/perception/status` | Topic | — | Subscribe | `interfaces/PerceptionStatus` | LOW |
| `/perception/calibrate_hand_eye` | Service | — | Client | `interfaces/CalibrateHandEye` | LOW |
| `/perception/get_object_positions` | Service | — | Client | `interfaces/GetObjectPositions` | LOW |
| `/perception/check_camera` | Service | — | Client | `interfaces/CheckCamera` | LOW |
| `/llm_gateway/hydrate_workplane` | Service | `adapter.py:155,422` | Client | `interfaces/HydrateWorkplane` | MEDIUM |
| `/llm_gateway/get_primitive_constants` | Service | `adapter.py:156,453` | Client | `interfaces/GetPrimitiveConstants` | LOW |
| `/supervisor/confirm_execution` | Service | `adapter.py:158,489` | Client | `interfaces/ConfirmExecution` | HIGH |

## Change sensitivity definitions

- **HIGH:** HMI breaks on any rename, field rename, or type change. Paired `hmi/` patch required.
- **MEDIUM:** HMI must update on schema change but tolerates field additions.
- **LOW:** HMI tolerates additions and minor changes; only total removal breaks it.

## Re-verification rule

Every wave from W2 onward that touches a ROS surface must:
1. Re-run the inventory grep matrix and diff against this file.
2. Update the `Change sensitivity` column if needed.
3. List breaking changes (if any) in that wave's MIGRATION-W<N>.md.

This rule is enforced by the "Hard never" bullet in AGENTS.md:
> Never change a HIGH-sensitivity ROS surface without paired hmi/ patch in the same PR.
