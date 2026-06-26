# Grasp Pipeline Commissioning Checklist (real hardware)

Run order. Hand on e-stop for every motion step. velocity_scale = 0.06.

## A. Bench the gripper IO alone (NO motion)
1. Launch hw stack: `ros2 launch gp4_bringup hw.launch.py robot_ip:=192.168.1.33 agent_ip:=192.168.1.99`
2. Close (grasp): `ros2 service call /yaskawa/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10017, value: 1}"`
   → CONFIRM the air/vacuum actuates and physically grips a yellow_box held by hand.
3. Read feedback: `ros2 service call /yaskawa/read_single_io motoros2_interfaces/srv/ReadSingleIO "{address: 30017}"`
   → CONFIRM value == 1 while grasped, 0 when released.
4. Open: `WriteSingleIO {address: 10017, value: 0}` → CONFIRM release.
   GATE: if address/value/feedback are wrong, fix `safety_rules.yaml gripper:` before any motion.

## B. Perception quality at the grasp pose
5. Place a real yellow_box on the conveyor pick area.
6. `python test_svc.py` (class_filter yellow_box) → CONFIRM one detection, score, and a sane base_link XYZ.
7. Tune `perception.yaml min_publish_confidence` UP from the debug value 0.20 until only the true box publishes
   (start 0.45). Confirm the published Z matches the real box top within a few mm.
   GATE: do not grasp on a DEGRADED_DEPTH / low-confidence detection. Z error here = TCP crash.

## C. Motion dry-run, gripper disconnected (air OFF at the valve)
8. Send "đi tới yellow box" via HMI/gp4_cmd. CONFIRM approach pose is ABOVE the box (z + 0.08), tool-down.
9. Tune `approach_clearance_m` / `descend_m` (skill args) so descend stops at the real grasp surface, not into the table.
   GATE: jog limits and descend depth verified before reconnecting air.

## D. Full closed loop
10. Reconnect air. Command "gắp yellow box thả ở gá phôi".
11. Watch the supervisor confirm gate; approve. CONFIRM sequence:
    approach → descend → close(10017=1) → verify_grasp(30017==1) → lift →
    approach fixture → descend → open(10017=0) → lift.
12. CONFIRM box ends on the fixture. If verify_grasp fails, runtime stops before moving — re-tune B/C.
