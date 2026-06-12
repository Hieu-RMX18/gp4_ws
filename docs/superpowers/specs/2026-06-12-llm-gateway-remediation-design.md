# LLM Gateway Remediation — Design Spec

Date: 2026-06-12
Status: Approved by user (scope = All-in; execution owner = task_runtime)
Branch context: `upgrade-react-8626`
Supplements: `docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md`

## 0. Vì sao có spec này

Audit codebase (2026-06-12) đối chiếu spec gốc 2026-06-10 phát hiện migration đã
**dọn legacy tốt** nhưng **đi lệch phần kiến trúc đích**, và **closed-loop chưa
actuate end-to-end** — tức mục tiêu #1 của spec gốc chưa đạt.

### Hiện trạng đã verify

**Đúng:**
- Legacy xóa sạch: `intent_engine.py`, `react_planner.py`, `composite_tools.py`,
  `station_scene_graph.py`, HMI `intent_resolution.py`/`intent_normalization.py`.
- Phase 0–2 đạt: review cache bỏ; `parse_source`/`code_version` stamping;
  `direct_commands.py` (tier-1) wired; `task_planner.py` single-shot, **không import ROS**.
- `task_planner` (Tier-2 LLM→FactoryTask) wired vào review: `_generate_review_semantic_ir`
  route đúng direct_commands → task_planner → FactoryTask preview.
- Perception bridge có thật và fail-closed: service `GetObjectPositions`
  (`_query_perception_detections`) trả detections + `calibration_valid` /
  `depth_in_range` / `depth_noise_mm_p95`. Gripper IO qua `_gripper_adapter` +
  `_read_gripper_feedback`. `/validate_command`, `/execute_motion` clients đều có.
- HMI G5 đạt: `supervisor_validation.py` dùng `IntentRouter`/`Normalizer` từ
  factory_task + `_to_jsonable()` ép kiểu ROS msg trước khi log (fix 500 crash).

**Lệch nghiêm trọng:**
1. **`factory_task.py` = 4331 dòng (god-file).** Nuốt toàn bộ intent_engine
   (`Normalizer`, `SchemaValidator`, `SemanticValidator`, `SequenceValidator`,
   `GoalMapper`, `DrawRouterMixin`, `IntentRouter`) + composite tools. Đi **ngược**
   quyết định Phase 3 đã chốt (memory: "Do NOT merge into factory_task; split into
   modules"). Vi phạm ranh giới spec §3.
2. **`llm_gateway_node.py` = 2860 dòng (target <600).** Vẫn giữ nguyên đường dispatch
   legacy: `_process_llm_payload` → `_normalize_and_validate` → `_process_single_command`
   / `_process_sequence` → `_dispatch_sequence_step` → `_dispatch_normalized_command` →
   `send_goal_async` (dòng ~1902). Đây là chỗ DUY NHẤT thật sự phát motion.
3. **Closed-loop KHÔNG actuate.** Confirm FactoryTask → `_on_confirm_factory_task_runtime`
   → `TaskRuntime.run` với `_execute_skill` **chỉ validate** (`_validate_runtime_semantic_ir`),
   `dispatched_to_ros = False`, **không truyền `event_callback`**. Robot đứng yên.
4. **Pick/place tools emit delta cố định, không grounded.** `PickObjectTool` /
   `ApproachObjectTool` / `PlaceObjectTool` phát `move_relative` delta hằng số
   (−0.08, −0.05m) + `io_set`, KHÔNG dùng pose vật từ perception. Comment tự thú
   "Actual target pose comes from perception at execution time" — không chỗ nào implement.
5. **`/llm_gateway/task_events` không tồn tại.** System Log schema §6 chưa publish.
6. **Hai đường thực thi song song** (legacy node dispatch + task_runtime validate-only)
   — vi phạm rule project "replace the owner, don't layer".

### Mấu chốt

Mọi mảnh hạ tầng (perception service, safety gate, motion action, gripper IO) **đã có
sẵn ở tầng node**. Khoảng trống hẹp: `skill_executor` của task_runtime đang validate-only
thay vì "ground pose → validate → dispatch → verify", thiếu task_events publisher, và
god-file/node chưa tách. Remediation chủ yếu là **tái bố trí + wiring**, không phải xây lại.

## 1. Quyết định đã chốt (clarifying)

| Câu hỏi | Quyết định |
|---------|-----------|
| Chủ thể thực thi FactoryTask tree | **task_runtime trong ROS node** (đúng spec §4): ground pose → /validate_command → /execute_motion, loop/retry/replan, publish task_events |
| Scope | **All-in**: closed-loop actuate + split factory_task + thin node + System Log |
| R4 xóa legacy & route lệnh đơn | Collapse hẳn đường legacy; tier-1 (home/stop/get_pose/wait) tái biểu diễn thành single-skill FactoryTask chạy qua cùng task_runtime → task_runtime là chỗ DUY NHẤT phát motion |
| Thứ tự | R1 → R2 → R3 → R4 (an toàn trước, collapse legacy cuối khi đã có task_events để verify) |

## 2. Nguyên tắc xuyên suốt

- Mỗi phase là **checkpoint demo được** — robot luôn chạy được giữa các phase.
- Mỗi sub-phase: `cd src/llm_gateway && python -m pytest tests/ -q` (≥442 pass) +
  `colcon build --packages-select llm_gateway --symlink-install` green **trước khi commit**.
- Reindex GitNexus (`npx gitnexus analyze`) sau mỗi commit. Commit cần
  `PRE_COMMIT_ALLOW_NO_CONFIG=1`. Node đang chạy phải restart (egg-link) để load code mới.
- Chạy `gitnexus_impact({target, direction:"upstream"})` trên symbol **trước khi sửa**;
  cảnh báo nếu HIGH/CRITICAL. Chạy `gitnexus_detect_changes()` trước commit.
- **Safety-first khóa cứng:** mọi motion qua `/validate_command` → `/execute_motion`.
  Perception pose chỉ dùng khi đủ tươi (5s) + calibration OK, không thì fail-closed.
- Behavior-preserving khi relocate (R1); chỉ R2 thay đổi hành vi → sim test trước hardware.

## 3. Phase R1 — Tách `factory_task.py` (4331 → ~600 dòng)  [spec §3]

Pure relocation, guard bằng 442 test (import-only, không đổi hành vi).

| Module mới | Lớp chuyển ra |
|-----------|---------------|
| `normalization.py` | `Normalizer` |
| `validation.py` | `SchemaValidator`, `SemanticValidator`, `SequenceValidator` (+ `*Result`/`*Error`) |
| `goal_mapper.py` | `GoalMapper` |
| `drawing_router.py` | `DrawRouterMixin` (giữ quan hệ với `drawing_geometry`/`stroke_font`) |
| `intent_router.py` | `IntentRouter`, `RouteResult`, `_SchemaValidatorLike` |
| `composite_tools.py` | `_CompositeTool`, `Pick/Approach/Place/VerifyPostcondition/VerifyGrasp/EmitSequence/RefreshScene` tools, `ToolResult`, `PostconditionVerifier`, `mtc_select` |

**`factory_task.py` giữ lại đúng spec §3:** `FactoryTaskError`, `ResolveResult`,
`StationSceneGraph`, `SkillCall`, `TaskNode`, `FactoryTask`, `PolicyDecision`,
`CompiledTask`, `WorldModel`, `PolicyEngine`, `TaskCompiler`.

**Lưu ý vòng import:** factory_task hiện import `task_runtime` (TYPE_CHECKING). Sau khi
tách, giữ import nội bộ ở mức hàm để tránh circular. Mọi `__all__` re-export giữ nguyên
để callers ngoài không vỡ; cập nhật import nội bộ trong `llm_gateway_node.py`,
`supervisor_validation.py`, tests.

Verify: pytest xanh + build xanh, không đổi public API. `gitnexus_impact` trên mỗi lớp
trước khi move (nhiều caller → HIGH dự kiến, nhưng relocation an toàn).

## 4. Phase R2 — Closed-loop actuate thật  [spec §4 — phần lõi]

Đây là phase **thay đổi hành vi**. Thay `_execute_skill` (validate-only) bằng một
**real skill executor** inject vào `TaskRuntime.run(task, skill_executor)`.

### 4.1 Skill executor (chỗ DUY NHẤT phát motion)

Mỗi skill name → handler, đều fail-closed:

- **`observe`**: `_query_perception_detections(class_filter)` → cập nhật `WorldModel`
  (pose base_link + stamp). calibration_invalid / depth_quality_invalid → fail-closed.
- **`pick_object` / `move_to_object` / `approach_object`**: lấy pose tuyệt đối từ
  `WorldModel.object_pose(ref)` (thay delta hằng số). Check freshness ≤5s (stale →
  re-observe 1 lần; vẫn stale → `PERCEPTION_STALE`, pause). Dựng command:
  approach (+Z offset) → descend → build semantic_ir pose-target →
  `/validate_command` → `/execute_motion` send_goal + await result.
- **`place_object`**: ground destination region từ `StationSceneGraph` → descend →
  release → lift clear, mỗi bước qua validate→execute.
- **gripper close/open**: `_gripper_adapter` WriteSingleIO (config phải `verified()`).
- **`verify_grasp`**: `_read_gripper_feedback` (IO thật); trượt → `GRASP_FAILED`.
- **`verify_scene` / `verify_postcondition`**: re-observe; sai → trigger replan.

### 4.2 Runtime wiring

`TaskRuntime(world_model, replan_handler, max_replans, is_stopped_fn, event_callback)`:
- `event_callback` → publish task_events (R3).
- `is_stopped_fn` → đọc STOP flag; STOP giữa chừng → **cancel ExecuteMotion goal đang
  chạy ngay**, state STOPPED, fail-closed.
- `replan_handler` → re-plan qua task_planner (max_replans=1) hoặc fail rõ ràng.
- Tham số: freshness 5s, retry pick max 2, max_replans 1 — đọc từ
  `safety_rules.yaml` / task limits (configurable).

### 4.3 Bảng lỗi (spec §5, một enum chung)

`MISSING_SLOT`, `UNSUPPORTED_OR_AMBIGUOUS`, `WORLD_MODEL_UNGROUNDED`,
`PERCEPTION_STALE`, `SAFETY_REJECTED`, `MOTION_FAILED`, `GRASP_FAILED`,
`OPERATOR_STOP` — mỗi reject kèm `hint` human-readable (VN/EN theo input).

### 4.4 Tests
- Fake executor (unit): sequence/repeat/for_each/retry/fallback/replan; STOP giữa chừng;
  event đúng thứ tự; freshness/stale; grasp fail → retry → fallback.
- Integration sim với **fake `GetObjectPositions` server**: happy path pick-place;
  grasp fail → retry; object biến mất → replan → fail rõ. Mọi motion qua safety gate.

## 5. Phase R3 — System Log `/llm_gateway/task_events`  [spec §6]

- Node tạo publisher `/llm_gateway/task_events` (std_msgs/String JSON). `event_callback`
  của runtime → publish theo schema cố định:
  `{ts, level, source, category, event, detail, data}`.
- Map `robot_status` / `joint_states` / safety result / motion result vào cùng schema
  (gộp/bridge với `_emit_trace` hiện có thay vì hai hệ song song).
- Categories: `TASK`, `MOTION`, `PERCEPTION`, `HARDWARE`, `IO`, `SAFETY`, `SYSTEM`
  (nội dung từng category theo bảng spec §6).
- HMI: backend ros adapter subscribe → forward WS `/api/hmi/stream`; frontend System Log
  panel filter category/level/source, search, expand `data`; robot status strip riêng
  (TCP/joints/servo/alarm) cập nhật live; export JSONL.

## 6. Phase R4 — Thu mỏng node (2860 → <600) + xóa legacy  [spec §6]

Làm cuối, khi R2 đã chuyển execution vào task_runtime và R3 đã có task_events để verify.

- **Xóa đường dispatch song song** trong node: `_process_llm_payload`,
  `_normalize_and_validate`, `_process_single_command`, `_process_sequence`,
  `_dispatch_sequence_step`, `_dispatch_normalized_command`, legacy `send_goal_async`
  path, `_SequenceExecutionState`.
- **Tier-1 lệnh đơn** (home/stop/get_pose/alarm_reset/wait) tái biểu diễn thành
  single-skill FactoryTask chạy qua cùng task_runtime → đúng "task_runtime là chỗ DUY
  NHẤT phát motion".
- Tách skill_executor closure ra `runtime_skill_executor.py` (adapter dùng node clients).
- Node còn lại chỉ wiring: parameters, subscriptions, service handlers, action clients,
  publishers, runtime wiring. Target <600 dòng.
- Cập nhật `.claude/rules/llm-gateway.md` (nếu có) + README + CLAUDE.md theo
  `when-to-update-claude-docs.md` (đổi pipeline/topic mới).

## 7. Thứ tự & rủi ro

R1 (relocation an toàn, thu nhỏ god-file) → R2 (core, sim trước hardware) →
R3 (observability) → R4 (collapse legacy — rủi ro cao nhất, làm cuối).
Mỗi phase build green + pytest pass + checkpoint demo.

## 8. Out of scope (giữ spec §9)

- Không đổi ROS contracts: `interfaces`, `safety`, `motion_core`, `hw_adapter`.
- Không đổi MoveIt/planner config.
- Không tách node `task_runtime` riêng (Hướng B) — để sau nếu cần.
- Frontend redesign tổng thể — chỉ thêm panel/filter System Log + task progress.
