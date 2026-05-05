# GP4 HMI — Tổng quan kiến trúc (vi)

> Ngày cập nhật: 2026-05-04 (W5).
> Tài liệu này là pointer ngắn gọn cho operator nói tiếng Việt.
> **Source of truth là các tài liệu được liệt kê ở mục 1.**

---

## 1. Source of truth (đọc trước khi sửa code)

| Chủ đề | File chuẩn |
|---|---|
| Kiến trúc tổng (4 lớp + HMI bridge) | `README.md` |
| Telemetry contract + lifecycle command (v2) | `hmi/HMI_V2_COMMAND_INGRESS.md` |
| Hardware gate (dual gate) | `hmi/HARDWARE_TELEMETRY_VALIDATION.md`, `hmi/HARDWARE_READONLY_VALIDATION.md` |
| Safety hard limits | `src/safety/config/safety_rules.yaml` |
| Action / service / message hợp lệ | `src/interfaces/action/`, `src/interfaces/srv/`, `src/interfaces/msg/` |

Bất kỳ phát biểu nào trong file này mâu thuẫn với các file trên thì các file trên thắng.

---

## 2. Lifecycle command v2 (HMI ingress)

`POST /api/hmi/commands/intent` -> supervisor parse + validate ->
`POST /api/hmi/commands/{id}/confirm` -> supervisor dispatch ->
`POST /api/hmi/commands/{id}/cancel` (tuỳ chọn).

State chuẩn (xem chi tiết trong `hmi/HMI_V2_COMMAND_INGRESS.md`):

```
RECEIVED -> PARSING -> VALIDATING -> NEEDS_CONFIRMATION
       -> CONFIRMED -> EXECUTION_REQUESTED -> EXECUTING
       -> SUCCEEDED | FAILED | REJECTED | CANCELLED | EXPIRED
```

Browser **không** gọi trực tiếp `/llm_text_input`, `/validate_command`, hay
`/execute_motion`. Toàn bộ command đi qua HMI supervisor service.

**W5:** HMI backend là thin layer over ROS services. `_hydrate_draw_workplane`
gọi `/llm_gateway/hydrate_workplane` (fallback local khi ROS unavailable).
Primitive constants fetch được từ `/llm_gateway/get_primitive_constants`.
Confirm gate gọi `/supervisor/confirm_execution` để re-validate.

---

## 3. Approval & lease

- **Approval owner duy nhất** là HMI supervisor (lease + confirm flow).
  `motion_core` không có approval state machine riêng.
- Single-controller lease: TTL 15 s, renew period 5 s. Force takeover được audit.
- Replay endpoints (`GET /api/hmi/commands/replay`) là read-only, không có execute path.

---

## 4. Hardware gate

Hardware execution chỉ bật khi **cả hai** điều kiện đúng:

1. Biến môi trường `HMI_ENABLE_HARDWARE_COMMANDS=1`.
2. File `hmi/data/hardware_gate.json` xác nhận hardware mode.

Sim mode mặc định fail-closed: confirm và execute disabled cho đến khi sim
bridge sẵn sàng và lease được nắm giữ.

---

## 5. Telemetry & freshness

Frontend tiêu thụ một WebSocket stream từ `telemetry_bridge_service`. Sources
freshness-critical (block command UI khi stale):

- `gateway_status`
- `readiness`
- `supervisor_alerts`
- joint source đang active (`/yaskawa/joint_states` ưu tiên, fallback `/joint_states`)

`llm_debug` và `llm_command` là non-blocking telemetry chỉ.

**W5:** Adapter readiness tracks 5 command interfaces: `/validate_command`,
`/execute_motion`, `/llm_gateway/hydrate_workplane`,
`/llm_gateway/get_primitive_constants`, `/supervisor/confirm_execution`.

---

## 6. Lưu ý còn lại (residual risks)

Các điểm chưa-xử-lý đã được di dời sang `hmi/HMI_V2_COMMAND_INGRESS.md` Section 13.
Không duplicate ở đây để tránh drift.

---

## 7. File này KHÔNG nên chứa

- Lifecycle state cũ (`PARSED`, `PLANNED`, `QUALITY_CHECKED`, `READY_FOR_CONFIRM` v.v.)
- Endpoint cũ (`/commands/submit`, `/commands/{id}/abort`)
- Cờ deprecated (`auto_clear_unimplemented_approval`, `ExecuteMotion.require_approval`)
- Bất kỳ giá trị an toàn nào không khớp `safety_rules.yaml`

Nếu thấy nội dung như trên xuất hiện trở lại, hãy xoá hoặc trỏ về source of truth.
