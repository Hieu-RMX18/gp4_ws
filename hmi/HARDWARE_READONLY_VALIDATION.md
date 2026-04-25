# GP4 read-only hardware validation checklist

This checklist validates the current GP4/YRC1000micro stack without commanding
robot motion. It is intended for the first real-hardware validation pass after
code or wiring changes.

## Safety boundary

- Do not send goals to `/execute_motion`, `/hw_adapter/dispatch_trajectory`, or
  `/yaskawa/follow_joint_trajectory`.
- Do not run `ros2 run llm_gateway gp4_cmd ...` while connected to hardware.
- Do not call `/hw_adapter/alarm_reset`, `/hw_adapter/io_set`, or jog-pendant
  commands during this read-only pass.
- Keep HMI hardware command gate locked unless a separate commissioning record
  explicitly authorizes execution.

## 1. Baseline environment

```bash
cd /home/hieu2/gp4_ws
git status --short --branch
source /opt/ros/humble/setup.bash

export GP4_LLM_ENV_FILE=/home/hieu2/gp4_ws/.env
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# GP4 workspace: isolate from other ROS2 stacks on same network.
export ROS_DOMAIN_ID=39

RUN_ID="$(date +%Y%m%d_%H%M%S)"
export GP4_LOG_DIR="/tmp/gp4_hw_validation_${RUN_ID}"
mkdir -p "$GP4_LOG_DIR"

colcon list | tee "$GP4_LOG_DIR/colcon_list.txt"
```

## 2. Full software build and test gate

```bash
cd /home/hieu2/gp4_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tee "$GP4_LOG_DIR/colcon_build.log"

source install/setup.bash

colcon test \
  --event-handlers console_direct+ \
  --return-code-on-test-failure \
  2>&1 | tee "$GP4_LOG_DIR/colcon_test.log"

colcon test-result --verbose \
  2>&1 | tee "$GP4_LOG_DIR/colcon_test_result.log"
```

## 3. Python, HMI, and contract checks

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash

python3 -m pytest -q src/llm_gateway/tests \
  2>&1 | tee "$GP4_LOG_DIR/llm_gateway_pytest.log"

python3 -m pytest -q src/safety/tests \
  2>&1 | tee "$GP4_LOG_DIR/safety_pytest.log"

python3 -m pytest -q hmi/backend/tests \
  2>&1 | tee "$GP4_LOG_DIR/hmi_backend_pytest.log"

cd /home/hieu2/gp4_ws/hmi/frontend
npm install
npm run build \
  2>&1 | tee "$GP4_LOG_DIR/hmi_frontend_build.log"

cd /home/hieu2/gp4_ws
ros2 interface show interfaces/srv/ValidateCommand
ros2 interface show interfaces/action/ExecuteMotion
ros2 interface show interfaces/action/DispatchTrajectory
```

## 4. Simulation dry-run

Terminal A:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=39

ros2 launch gp4_bringup sim.launch.py use_rviz:=false \
  2>&1 | tee "$GP4_LOG_DIR/sim_launch.log"
```

Terminal B:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
export ROS_DOMAIN_ID=39

ros2 node list | sort
ros2 topic list | sort
ros2 action list | sort

python3 -m pytest -q hmi/backend/tests/test_command_e2e_sim.py \
  2>&1 | tee "$GP4_LOG_DIR/hmi_command_e2e_sim.log"

hmi/tools/run_command_e2e_validation.sh \
  2>&1 | tee "$GP4_LOG_DIR/command_e2e_validation.log"
```

Stop Terminal A after the sim checks pass.

## 5. Real hardware read-only preflight

Operator checks before starting the hardware stack:

- Robot cell clear and guarded.
- E-stop reachable.
- YRC1000micro in the expected mode for MotoROS2 validation.
- No one in the robot envelope.
- MotoROS2 config uses namespace `yaskawa`.
- Joint order is `joint_1_s joint_2_l joint_3_u joint_4_r joint_5_b joint_6_t`.

Terminal A, start the micro-ROS Agent:

```bash
cd /home/hieu2/gp4_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

docker run --rm -it --net=host microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6 \
  2>&1 | tee "$GP4_LOG_DIR/micro_ros_agent.log"
```

Terminal B, verify network readiness:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=39

ip -br addr
ping -c 3 192.168.1.33
ss -lunp | grep ':8888'
```

## 6. Launch real stack, still read-only

Terminal C:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=39

ros2 launch gp4_bringup system.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.1.33 \
  agent_ip:=192.168.1.99 \
  2>&1 | tee "$GP4_LOG_DIR/hardware_system_launch.log"
```

Terminal D, capture read-only graph, params, and telemetry:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
export ROS_DOMAIN_ID=39

hmi/tools/run_readonly_hardware_validation.sh \
  --duration-sec 120 \
  --log-dir "$GP4_LOG_DIR"
```

## 7. HMI read-only gate check

Terminal E:

```bash
cd /home/hieu2/gp4_ws
source install/setup.bash
export ROS_DOMAIN_ID=39
unset HMI_ENABLE_HARDWARE_COMMANDS

python3 -m uvicorn hmi.backend.api.app:app \
  --host 127.0.0.1 \
  --port 8000 \
  2>&1 | tee "$GP4_LOG_DIR/hmi_backend_hw_readonly.log"
```

Terminal F:

```bash
SESSION_ID="hw-readonly-${RUN_ID}"
OPERATOR_ID="operator-validation"

curl -sS "http://127.0.0.1:8000/api/hmi/connection-state" | python3 -m json.tool
curl -sS "http://127.0.0.1:8000/api/hmi/snapshot?session_id=${SESSION_ID}&operator_id=${OPERATOR_ID}" | python3 -m json.tool
curl -sS "http://127.0.0.1:8000/api/hmi/runtime-state?session_id=${SESSION_ID}&operator_id=${OPERATOR_ID}" | python3 -m json.tool
curl -sS "http://127.0.0.1:8000/api/hmi/lease-state?session_id=${SESSION_ID}&operator_id=${OPERATOR_ID}" | python3 -m json.tool
```

Expected result: hardware capabilities remain read-only unless the separate
hardware gate evidence file and environment flag are both explicitly unlocked.

## Pass criteria

- Build, colcon tests, Python tests, and HMI frontend build pass.
- Sim command e2e tests pass.
- Hardware ROS graph exposes `/yaskawa/*`, `/execute_motion`,
  `/hw_adapter/dispatch_trajectory`, and `/validate_command`.
- `/move_group allow_trajectory_execution` is `False`.
- Motion limits are `<= 0.06`.
- `/hw_adapter_node sim_mode` is `False`.
- Hardware telemetry report shows live `/yaskawa/joint_states` and
  `/yaskawa/robot_status`.
- HMI hardware command path remains read-only/fail-closed.
