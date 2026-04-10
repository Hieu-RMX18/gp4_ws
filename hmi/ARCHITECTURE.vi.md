# GP4 HMI refactor scaffold

> Cap nhat 2026-04-08: tai lieu scaffold ben duoi co mot so ten lifecycle cu cua pre-v2.
> Ghi chu kien truc hien hanh cho command ingress nam o `hmi/HMI_V2_COMMAND_INGRESS.md`.
> Telemetry v1 van la baseline; HMI v2 them supervisor-owned command ingress sim-only va giu hardware/MotoROS2 o trang thai chua xac minh.

## 1. Chiến lược refactor ngắn gọn

Workspace hiện có ROS-side control flow (`llm_gateway`, `safety`, `motion_core`, `hw_adapter`, `supervisor`) nhưng **chưa có frontend TSX/HMI bridge đã check-in** để làm ranh giới an toàn giữa browser và robot. Vì vậy bản scaffold này đi theo hướng **refactor an toàn, không redesign**:

- giữ nguyên visual identity của file tham chiếu `gp4_hmi_chatbot_interface.html`
- thêm **typed supervisor bridge boundary** để frontend chỉ nói chuyện với backend bridge
- thêm **single-controller lease model** để chỉ một operator session có quyền điều khiển
- thêm **audit + replay shape** để mọi lệnh có thể trace theo `command_id`, `session_id`, `operator_id`
- giữ ROS-specific logic ở backend adapter, **không đưa ROS publish/service/action vào browser**

> Lưu ý an toàn: repo hiện **chưa có supervisor-owned parse/confirm ingress** cho HMI. Các skeleton backend mới vì thế fail-closed và không tự nối browser vào `/llm_text_input`, `/validate_command`, hay `/execute_motion`.

---

## 2. Updated folder structure

```text
hmi/
├── ARCHITECTURE.vi.md
├── shared/
│   └── contracts.ts
├── frontend/
│   ├── App.tsx
│   ├── main.tsx
│   ├── bridgeClient.ts
│   ├── hooks/
│   │   └── useGP4Bridge.ts
│   ├── components/
│   │   ├── GP4HMI.tsx
│   │   └── RuntimeStateBanner.tsx
│   └── styles/
│       └── gp4-hmi.css
└── backend/
    ├── api/
    │   └── app.py
    ├── domain/
    │   ├── models.py
    │   └── state_machine.py
    ├── ros/
    │   └── adapter.py
    └── services/
        ├── audit_service.py
        ├── session_lock_service.py
        ├── telemetry_bridge_service.py
        └── supervisor_service.py
```

### Boundary intent

- `frontend/`: HMI render + operator interaction only
- `shared/contracts.ts`: single source of truth cho browser/backend typed contracts
- `backend/services/telemetry_bridge_service.py`: read-only telemetry aggregator cho REST/WebSocket v1
- `backend/services/supervisor_service.py`: application service cho submit/confirm/abort ở giai đoạn sau
- `backend/services/session_lock_service.py`: single-controller lease authority
- `backend/services/audit_service.py`: persistence + replay read model
- `backend/ros/adapter.py`: lớp duy nhất được phép biết ROS endpoints thật
- `backend/domain/*`: runtime state + command lifecycle + transition rules
- `backend/api/app.py`: HMI-facing API boundary

---

## 3. Typed contract design

Nguồn chuẩn nằm ở `hmi/shared/contracts.ts`.

> Ghi chú phase: danh sách dưới đây giữ nguyên contract shape cho roadmap đầy đủ; **build telemetry bridge v1 hiện tại chỉ bật `GET /snapshot`, `GET /runtime-state`, `GET /connection-state`, `GET /lease-state`, và `WS /stream`**. Mọi lease/command/replay mutation vẫn fail-closed.

### HMI -> supervisor requests

- `POST /api/hmi/lease/acquire`
  - body: `sessionId`, `operatorId`, `requestedRole`, `forceTakeover`, `takeoverReason`
- `POST /api/hmi/lease/renew`
  - body: `sessionId`, `operatorId`, `leaseToken`
- `POST /api/hmi/lease/release`
  - body: `sessionId`, `operatorId`, `leaseToken`
- `POST /api/hmi/commands/submit`
  - body: `sessionId`, `operatorId`, `leaseToken`, `rawText`, `mode`
- `POST /api/hmi/commands/{commandId}/confirm`
  - body: `sessionId`, `operatorId`, `leaseToken`
- `POST /api/hmi/commands/{commandId}/abort`
  - body: `sessionId`, `operatorId`, `leaseToken`, `reason`

### supervisor -> HMI stream/events

WebSocket stream: `GET ws /api/hmi/stream?session_id=...&operator_id=...`

Event families:

- `snapshot`
- `lease_state`
- `command_lifecycle`
- `runtime_state`
- `replay_updated`
- `connection_state`

### Shared read models

- `HmiStateSnapshot`
- `LeaseView`
- `CommandView`
- `RuntimeSnapshot`
- `JointPosition`
- `PlanMetrics`
- `ReplayListItem`
- `ReplayDetail`

---

## 4. State model design

### 4.1 Command lifecycle states

```text
IDLE
-> RECEIVED
-> PARSED
-> VALIDATED
-> PLANNED
-> QUALITY_CHECKED
-> READY_FOR_CONFIRM
-> EXECUTING
-> SUCCEEDED | FAILED | REJECTED | CANCELLED | ABORTED
```

### 4.2 System/runtime states

```text
NORMAL
FAULT
ESTOP
HOLD
TIMEOUT
LOST_CONN
SAFETY_BLOCKED
```

### 4.3 Blocking rules

Frontend phải chặn control-capable actions khi runtime state thuộc:

- `FAULT`
- `ESTOP`
- `LOST_CONN`
- `SAFETY_BLOCKED`

`HOLD` và `TIMEOUT` hiển thị banner cảnh báo nhưng không tự cho phép browser bỏ qua backend policy.

### 4.4 Transition ownership

- React chỉ **render** theo backend snapshot/event
- state machine authority nằm ở `backend/domain/state_machine.py`
- local component state chỉ được dùng cho UI draft như input text, tab, scroll state

---

## 5. Session lock model

### Lease entity

- `leaseId`
- `leaseToken`
- `sessionId`
- `operatorId`
- `role`: `controller | observer`
- `acquiredAt`
- `expiresAt`
- `forceTakeover`
- `takeoverReason`

### Ownership rules

- chỉ **1** active `controller` lease tại một thời điểm
- session khác mặc định là `observer`
- `submit`, `confirm`, `abort`, `execute-capable` actions đều cần lease controller còn hạn
- `observer` chỉ xem telemetry, replay, audit

### TTL / renewal

- TTL mặc định: **15 giây**
- frontend renew chu kỳ **5 giây** khi đang giữ controller lease
- lease hết hạn -> backend tự hạ session về observer mode
- force takeover cần `takeoverReason` rõ ràng và được audit-log

### Rejected actions

Nếu lease thiếu/expired/sai owner:

- trả `409` cho conflict lease
- trả `403` cho forbidden control action
- luôn ghi audit event với `reject_reason`

---

## 6. Audit schema

`backend/services/audit_service.py` dùng SQLite để có query ổn định cho replay.

### commands table

Các cột chính:

- `command_id`
- `session_id`
- `operator_id`
- `raw_text`
- `parsed_intent_json`
- `validation_result_json`
- `reject_reason`
- `plan_summary_json`
- `confirm_at`
- `execute_at`
- `final_state`
- `mode`
- `frame_used`
- `planner_used`
- `created_at`
- `updated_at`

### command_events table

- `event_id`
- `command_id`
- `session_id`
- `operator_id`
- `from_state`
- `to_state`
- `runtime_state`
- `reason`
- `payload_json`
- `created_at`

### runtime_events table

- `event_id`
- `system_state`
- `session_id`
- `operator_id`
- `command_id`
- `message`
- `payload_json`
- `created_at`

Replay chỉ dùng để **inspect/debug**. Không có API re-execute từ replay.

---

## 7. Replay endpoint design

> Phase note: telemetry bridge v1 mới chuẩn bị audit schema; replay read endpoints bên dưới là target kế tiếp và chưa bật trong API hiện tại.

### List endpoint

`GET /api/hmi/replay?limit=50&operator_id=...&session_id=...&final_state=...&from=...&to=...`

Trả về:

- command summary list
- lifecycle state cuối
- thời điểm tạo/xác nhận/thực thi/kết thúc
- mode, planner, frame

### Detail endpoint

`GET /api/hmi/replay/{commandId}`

Trả về:

- command header
- raw text
- parsed intent
- validation result
- plan summary
- timeline event list
- runtime/system fault events liên quan

### Safety constraints

- không có `POST /replay/{id}/execute`
- replay payload không được map ngược thành execution command
- mọi nút UI liên quan replay là read-only

---

## 8. TSX refactor plan

### Bỏ các production anti-pattern

- bỏ mọi direct ROS publish/service/action từ browser
- bỏ demo chat responses, demo joints, demo plan metrics khỏi production render path
- bỏ local fake sequencing của parse/validate/plan/execute trong React state

### Refactor target

- `useGP4Bridge()` là input duy nhất cho runtime state
- `bridgeClient.ts` là transport adapter duy nhất của frontend
- `GP4HMI.tsx` chỉ render typed state + invoke typed actions
- `RuntimeStateBanner.tsx` chịu trách nhiệm blocking overlay/banner

### UI still preserved

- top bar gọn
- chat/main bên trái
- status sidebar bên phải
- dark control-room aesthetic
- thin border / rounded bubble / compact technical typography

---

## 9. Minimal code skeleton locations

- typed contracts: `hmi/shared/contracts.ts`
- hook + transport: `hmi/frontend/hooks/useGP4Bridge.ts`, `hmi/frontend/bridgeClient.ts`
- TSX HMI: `hmi/frontend/components/GP4HMI.tsx`
- runtime overlay: `hmi/frontend/components/RuntimeStateBanner.tsx`
- session lease: `hmi/backend/services/session_lock_service.py`
- audit + replay: `hmi/backend/services/audit_service.py`
- supervisor orchestration: `hmi/backend/services/supervisor_service.py`
- runtime/state machine: `hmi/backend/domain/state_machine.py`
- ROS boundary adapter (telemetry + sim-only execution): `hmi/backend/ros/adapter.py`
- HMI bridge API: `hmi/backend/api/app.py`

---

## 10. Migration steps from current codebase

1. **Giữ nguyên ROS execution path hiện tại** (`llm_gateway`, `safety`, `motion_core`, `hw_adapter`, `supervisor`).
2. Thêm HMI bridge backend mới làm boundary cho browser.
3. Thêm supervisor-owned command intake an toàn ở backend; **không** nối browser trực tiếp vào `/llm_text_input`.
4. Map telemetry read-only từ các endpoint ROS hiện có:
   - `/gateway_status`
   - `/llm_debug`
   - `/llm_command`
   - `/hw_adapter/ready`
   - `/supervisor/alerts`
   - `/yaskawa/joint_states`
   - `/yaskawa/robot_status`
5. `submit/confirm/abort` hiện đã nối thật cho **sim mode only**; hardware mode vẫn fail-closed.
6. Khi backend bridge ổn định, mới đóng gói frontend vào Next.js/Vite project chính thức.

---

## 11. Risks / open questions

1. **Hiện chưa có supervisor-owned parse-only ingress trong ROS workspace.** Nếu dùng trực tiếp `/llm_text_input` thì browser sẽ vòng vào execution pipeline hiện có, trái với yêu cầu an toàn.
2. `ExecuteMotion.action` có `require_approval`, nhưng current stack chưa có complete external confirm workflow cho HMI bridge.
3. `llm_gateway_node.py` hiện có nhánh `auto_clear_unimplemented_approval` cho sim; flow này không phù hợp để làm production HMI backend.
4. Joint telemetry trong workspace dùng cả `/joint_states` và `/yaskawa/joint_states`; bridge nên chuẩn hóa thành một typed UI model.
5. Muốn productionize tiếp cần quyết định backend runtime chính thức:
   - FastAPI sidecar ngoài ROS process
   - hoặc package ROS2 Python mới có embedded API server

---

## 12. Telemetry bridge v1 runtime notes

### 12.1 WebSocket heartbeat and reconnect

- V1 phát explicit WebSocket event:
  - initial `snapshot` ngay sau `accept`
  - `snapshot` tiếp theo khi có **semantic telemetry change**
  - `heartbeat` mỗi `5 s` nếu không có semantic change mới
- Frontend reconnect policy nằm trong `frontend/bridgeClient.ts`:
  - khi socket `close`, client chuyển `transportState` sang `disconnected`
  - reconnect dùng exponential backoff + jitter:
    - base `500 ms`
    - cap `8000 ms`
  - client watchdog đóng socket nếu không nhận traffic trong `15 s`
  - reconnect luôn nhận lại initial `snapshot`, nên browser không giữ authority cục bộ

### 12.2 LOST_CONN timeout rules

`backend/ros/adapter.py` dùng freshness windows:

- ROS aggregate freshness: `3.0 s`
- robot status: `3.0 s`
- readiness: `3.0 s`
- joint states: `3.0 s`
- supervisor alerts: `5.0 s`
- LLM telemetry: `30.0 s`

Bridge chuyển sang `LOST_CONN` khi:

- ROS adapter không start được, hoặc
- không còn telemetry tươi từ các read-only topics trong cửa sổ tương ứng

Bridge cho phép startup grace `3.0 s` sau khi node ROS vừa start:

- trong grace window: `transportState = connecting`
- sau grace window mà vẫn không có telemetry tươi: `transportState = disconnected`, `runtime = LOST_CONN`

### 12.2a Freshness exposure

Bridge expose thêm:

- `telemetryState = fresh | stale | unavailable`
- `telemetrySources[]` gồm:
  - `topic`
  - `lastSeenAt`
  - `freshnessThresholdSec`
  - `freshnessState`
  - `preferred`
  - `active`

Diễn giải deterministic:

- `transportState = disconnected` => backend không còn trust transport path
- `telemetryState = stale` => backend còn sống nhưng active telemetry source bị stale
- `runtime.systemState = SAFETY_BLOCKED` => safety gate block dù transport có thể vẫn còn

### 12.3 Joint topic precedence

Nếu cả `/joint_states` và `/yaskawa/joint_states` cùng có dữ liệu:

- ưu tiên `/yaskawa/joint_states`
- chỉ fallback sang `/joint_states` khi nguồn ưu tiên chưa có hoặc đã stale

### 12.4 Snapshot schema and versioning

- Snapshot payload mang `schemaVersion = "telemetry.v1"`
- field này có trong:
  - `GET /api/hmi/snapshot`
  - `GET /api/hmi/runtime-state`
  - `GET /api/hmi/connection-state`
  - `GET /api/hmi/lease-state`
  - WebSocket `snapshot`
  - WebSocket `heartbeat`
- nếu payload đổi backward-incompatible, tăng version string thay vì silently mutate shape
- backend REST response được validate bằng FastAPI response models
- WebSocket payload được validate trước khi send
- frontend reject schema không tương thích một cách explicit

### 12.5 BridgeCapabilities enforcement in frontend

Frontend enforce `BridgeCapabilities` ở nhiều lớp:

- `frontend/bridgeClient.ts`
  - mọi lease/command/replay mutation method trả về read-only rejection
  - không mở network write path trong v1
- `frontend/hooks/useGP4Bridge.ts`
  - không renew lease khi `readOnly = true`
  - `blockingRuntime` coi `readOnly` là trạng thái chặn control
- `frontend/components/GP4HMI.tsx`
  - disable submit / confirm / abort / lease buttons theo capabilities
  - render caption rõ ràng rằng control-capable paths đang fail-closed

### 12.6 Audit retention and storage policy

Telemetry storage policy mặc định:

- chỉ persist **semantic telemetry changes**
- heartbeat **không** ghi vào audit
- giữ tối đa `50,000` telemetry snapshots
- prune telemetry snapshots cũ hơn `7` ngày

Các bảng replay-oriented vẫn giữ riêng:

- `commands`
- `command_events`
- `runtime_events`
- `state_transitions`

Giả định tăng trưởng:

- idle polling không làm DB tăng
- tăng trưởng chủ yếu đến từ state/freshness change mà operator quan tâm
- bursty identical updates không tạo flood audit mới

---

## 13. Residual risk

### 13.1 Acceptable for v1

- Frontend reconnect/backoff đã harden nhưng chưa có browser-level automated test harness riêng.
- Live verification hiện mới cover sourced ROS environment startup/staleness path; chưa cover live publishers thật trên tất cả topics.
- `telemetrySources[].lastSeenAt` chủ yếu hữu ích cho diagnostics/operator tooling, chưa có dedicated UI rendering riêng trong v1.

### 13.2 Must-fix before any command-capable v2

- Phải verify end-to-end với ROS publishers thật cho readiness, robot status, alerts, và joint telemetry dưới tải thực.
- Phải có authenticated operator identity trước khi bật bất kỳ lease mutation hoặc command path nào.
- Phải có supervisor-owned command ingress + validation/execution boundary trước khi mở control-capable endpoints.
- Phải có stronger persistence strategy nếu audit/replay chuyển từ local SQLite dev mode sang production multi-session use.
