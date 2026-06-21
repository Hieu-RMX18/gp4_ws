# LLM Gateway Factory Pipeline — Design Spec

Date: 2026-06-10
Status: Approved by user (sections 1, 2, 3, 3b reviewed interactively)
Branch context: `upgrade-react-8626`

## 1. Goal

Chuyển `llm_gateway` từ kiểu "LLM interpreter" nhiều đường parse song song sang
kiến trúc **closed-loop smart factory**:

- Lệnh NL phức tạp đa bước (VN/EN): "đi tới A hạ xuống 5cm chờ 2s rồi về home",
  "đi tới băng tải, camera xác định vật, gắp từng cái qua gá phôi".
- Camera detect → pick → verify → retry/replan, fail-closed.
- Dọn legacy code: xóa các đường parse trùng lặp, file khổng lồ.

### Quyết định đã chốt (clarifying)

| Câu hỏi | Quyết định |
|---------|-----------|
| Mục tiêu demo | Closed-loop smart factory (camera → gắp từng vật → verify → retry) |
| Đường parse | 2 tầng: regex tối giản (~5 lệnh an toàn) + LLM→FactoryTask. HMI không parse |
| Perception hiện trạng | Detect được vật; chuỗi detect→gắp chưa chạy được — design phải bridge |
| Dọn code | Viết module pipeline mới, port dần kèm test, xóa file cũ sau khi thay thế |
| Confirm model | Confirm 1 lần cho cả task; STOP luôn hiện; mọi motion vẫn qua safety gate |
| File layout | Ít file, tên nói thẳng nhiệm vụ, chấp nhận file nhiều dòng |

## 2. Root-cause bug hiện tại (bối cảnh)

Hành vi "move to pose A" → GET_POSE / MISSING_SLOT trên HMI **không tái hiện được
với code nguồn hiện tại** (đã verify bằng cách chạy trực tiếp parser). Nguyên nhân:

1. **Stale process**: package cài egg-link; Python chỉ load code lúc node start.
   Node gateway đang chạy là code cũ (prompt cũ phân loại nhầm "pose" → get_pose).
2. **Review cache ghim kết quả sai**: `_generate_review_semantic_ir` check cache
   trước fast-path; một lần LLM trả sai → cache trả lại y nguyên suốt phiên.
3. **5 đường parse song song** cho cùng một câu lệnh: review cache → direct regex
   fast-path (~700 dòng trong node) → ReAct→FactoryTask → plain LLM → HMI local
   parser (~1.400 dòng). Hành vi phụ thuộc đường nào chộp được câu lệnh và process
   nào đang chạy bản code nào.

## 3. Kiến trúc module (Phần 1 — approved)

```
src/llm_gateway/llm_gateway/
├── direct_commands.py     # TẦNG 1: parse deterministic ~5 lệnh an toàn
│                          #   stop / home / get pose / alarm reset / wait N s
│                          #   Regex + alias VN/EN. KHÔNG gọi LLM. Trả Semantic IR đơn.
├── task_planner.py        # TẦNG 2: NL → FactoryTask JSON qua LLM
│                          #   System prompt (giữ bản hiện tại), LLM client,
│                          #   retry/backoff, parse + validate JSON output.
│                          #   KHÔNG import ROS, không biết world model/thực thi.
├── factory_task.py        # MÔ HÌNH + GROUNDING (giữ tên, mở rộng)
│                          #   - FactoryTask dataclasses (sequence/repeat/retry/if/
│                          #     for_each/until/fallback/observe/wait_until)
│                          #   - WorldModel: snapshot perception, freshness,
│                          #     object/region pose (hấp thụ station_scene_graph)
│                          #   - TaskCompiler: skill → primitive command grounded,
│                          #     fail-closed (hấp thụ phần routing của intent_engine)
├── task_runtime.py        # THỰC THI: walk tree, loops/retry/fallback/replan,
│                          #   verify_grasp, phát từng primitive qua safety gate →
│                          #   motion_core. Phát runtime events cho HMI.
└── llm_gateway_node.py    # ROS HOST MỎNG (viết lại, mục tiêu < 600 dòng)
                           #   Services/topics/actions, wiring. KHÔNG logic parse.
```

**Ranh giới:** `task_planner` không import ROS; `factory_task` không gọi LLM;
`task_runtime` là chỗ DUY NHẤT phát lệnh motion; node không chứa logic.
Một câu lệnh chỉ có đúng 2 đường: direct_commands bắt được → đi thẳng;
không bắt được → task_planner.

**Bỏ vòng lặp ReAct đa iteration** — thay bằng single-shot LLM → FactoryTask
(FactoryTask runtime đảm nhận loop/retry/replan thay cho ReAct loop).

**Xóa sau khi port xong:** `intent_engine.py` (3.417 dòng), `react_planner.py`
(1.894 dòng), `composite_tools.py`, `station_scene_graph.py`, fast-path ~700 dòng
trong node cũ, HMI `intent_resolution.py` + `intent_normalization.py` (~1.400 dòng).

## 4. Data flow (Phần 2 — approved)

### Luồng lệnh chuẩn

```
Operator gõ NL → HMI POST /api/commands (raw text, KHÔNG parse)
   ↓
llm_gateway_node: /llm_gateway/review_intent
   ↓
1. direct_commands.parse(text)
   ├─ khớp → Semantic IR đơn
   └─ không khớp → 2. task_planner.plan(text)
                        → FactoryTask JSON (hoặc MISSING_SLOT/AMBIGUOUS → reject + hint)
   ↓
factory_task.TaskCompiler.compile(task, world_model)
   ├─ grounded đủ → plan tree + step preview
   └─ thiếu fact → WORLD_MODEL_UNGROUNDED → reject kèm hint
   ↓
HMI hiển thị plan tree → operator CONFIRM 1 lần
   ↓
task_runtime.execute(task):
   motion step → /validate_command (safety) → /execute_motion (motion_core)
   observe step → query perception, cập nhật WorldModel
   runtime events → HMI stream; STOP → hủy action ngay, fail-closed
```

### Luồng closed-loop pick ("gắp từng vật trên băng tải qua gá phôi")

```
FactoryTask: sequence[ observe(băng tải),
             for_each(visible_objects):
               sequence[ pick_object($obj), place_object($obj, gá),
                         verify_scene($obj, placed) ] ]

1. observe → perception detect → WorldModel có N objects (pose base_link + timestamp)
2. pick_object: check freshness (mặc định 5s; stale → re-observe) →
   approach (+10cm) → descend → close gripper → lift →
   verify_grasp (IO kẹp / re-detect); trượt → retry (max 2)
3. place_object → tới gá → hạ → mở kẹp
4. verify_scene: re-observe; sai → replan (max_replans=1) hoặc fail rõ ràng
5. lặp vật tiếp theo; world thay đổi → replan_policy quyết định re-observe trước motion
```

**Khóa cứng safety-first:** runtime không bao giờ gửi thẳng trajectory — mọi motion
qua `/validate_command` → `/execute_motion`. Perception pose chỉ dùng khi đủ tươi
và calibration OK, không thì fail-closed.

Tham số chốt: freshness 5s, retry gắp max 2, max_replans 1 (đều configurable
trong safety_rules.yaml / task limits).

## 5. Error handling & HMI (Phần 3 — approved)

### Bảng lỗi thống nhất (một enum, mọi tầng dùng chung)

| Lỗi | Nguồn | Hành vi |
|------|-------|---------|
| `MISSING_SLOT` | task_planner | Reject + hint câu hỏi cho operator |
| `UNSUPPORTED_OR_AMBIGUOUS` | task_planner | Reject + lý do ngắn |
| `WORLD_MODEL_UNGROUNDED` | TaskCompiler | Reject + hint hành động ("bấm Observe / kiểm tra camera") |
| `PERCEPTION_STALE` | WorldModel | Re-observe 1 lần; vẫn stale → pause task, báo HMI |
| `SAFETY_REJECTED` | safety gate | Step fail, task dừng, hiện lý do safety nguyên văn |
| `MOTION_FAILED` | motion_core | Retry theo policy; hết retry → task fail, robot đứng yên |
| `GRASP_FAILED` | verify_grasp | Retry pick (max 2) → fallback branch → fail |
| `OPERATOR_STOP` | HMI | Hủy action ngay, state STOPPED, cần confirm mới |

Mỗi reject bắt buộc kèm `hint` human-readable (VN/EN theo ngôn ngữ input).

### Thay đổi HMI backend

- Xóa parser cục bộ (~1.400 dòng). HMI gửi raw text, nhận `planSummary` + lỗi có hint.
- Task states mới: `PLANNED → CONFIRMED → RUNNING → PAUSED/STOPPED → DONE/FAILED`
  + task progress (step N/M, vòng lặp thứ i). Lệnh đơn tầng 1 giữ lifecycle cũ.
- Quick commands giữ nguyên (đã structured, không qua parser).

### Chống tái phát bug cũ

1. **Không cache kết quả LLM** giữa các lần submit (xóa review cache).
2. Mọi review response đính kèm `parse_source: "direct"|"llm"` + `code_version`
   (git short hash lúc node start) — hiện trên HMI debug panel.

## 6. System Log toàn diện (Phần 3b — approved)

Thay log "Step X/6" bằng **một event stream thống nhất, schema cố định**:

```json
{
  "ts": "15:02:01.123",
  "level": "INFO | WARN | ERR",
  "source": "gateway | runtime | safety | motion | hw_adapter | perception | system",
  "category": "TASK | MOTION | PERCEPTION | HARDWARE | SAFETY | IO | SYSTEM",
  "event": "ten_su_kien_ngan",
  "detail": "câu human-readable cho operator",
  "data": { "payload máy đọc được" }
}
```

| Category | Sự kiện |
|----------|---------|
| `TASK` | nhận task, plan tree, confirm, step N/M start/done/fail, loop i, retry j, replan, DONE/FAILED + lý do gốc |
| `MOTION` | pose TCP trước/sau mỗi step (x,y,z + frame), planner, thời gian plan/exec, % trajectory |
| `PERCEPTION` | detect: N vật, class + pose, frame, độ trễ, freshness; stale/re-observe |
| `HARDWARE` | robot_status: alarm code + tên MotoROS2, e-stop, servo on/off, mode; mất agent; motoros2 result code |
| `IO` | gripper lệnh + kết quả ReadSingleIO thực tế |
| `SAFETY` | validate pass/fail + rule chặn (bounds, forbidden zone, velocity cap) |
| `SYSTEM` | node up/down, code_version, runtime_mode, joint_states alive, snapshot pose định kỳ khi idle |

HMI: filter category/level/source, search, expand xem `data`; robot status strip
riêng (TCP, joints, servo, alarm) cập nhật live, không trộn vào log; Export JSONL.

Nguồn: `task_runtime` publish `/llm_gateway/task_events` chuẩn hóa theo schema;
HMI backend ros adapter map `robot_status`/`joint_states` sang event schema.

## 7. Testing

| Tầng | Test |
|------|------|
| `direct_commands` | table-driven VN/EN: mỗi lệnh an toàn × biến thể alias; property: mọi text khác trả None |
| `task_planner` | mock LLM: JSON hợp lệ/lỗi/markdown bẩn; MISSING_SLOT passthrough; retry/backoff |
| `factory_task` | TaskCompiler grounding fail-closed (vật chưa detect, stale, calib thiếu); parse mọi node type; WorldModel freshness |
| `task_runtime` | fake executor: sequence/repeat/for_each/retry/fallback/replan; STOP giữa chừng; event đúng thứ tự |
| Contract | `/llm_gateway/review_intent` request/response; task states; event schema |
| Integration (sim) | closed-loop pick-place với fake perception: happy path, grasp fail → retry, object biến mất → replan → fail rõ |
| HMI backend | submit→review→confirm→events; xóa test của parser cũ |

Coverage mục tiêu 80%+ cho 4 module mới. Mọi phase build green + pytest pass
trước khi sang phase sau.

## 8. Migration phases

| Phase | Nội dung | Xóa được |
|-------|----------|----------|
| 0 | Quick wins: xóa review cache; thêm `parse_source` + `code_version` vào review response; restart discipline ghi vào docs | review cache |
| 1 | `direct_commands.py` + tests, wire vào node trước các đường legacy | fast-path ~700 dòng trong node |
| 2 | `task_planner.py` (port prompt + LLM client từ react_planner, single-shot) | ReAct loop, `react_planner.py` |
| 3 | `factory_task.py` mở rộng: WorldModel (hấp thụ station_scene_graph + scene cache), TaskCompiler (hấp thụ routing của intent_engine) | `intent_engine.py`, `station_scene_graph.py`, `composite_tools.py` |
| 4 | `task_runtime.py`: thực thi tree, confirm-once, STOP, task_events | logic thực thi rải rác trong node |
| 5 | HMI: xóa local parser, task states + progress, System Log mới (schema, filter, status strip, export) | `intent_resolution.py`, `intent_normalization.py` |
| 6 | Viết lại `llm_gateway_node.py` mỏng; dọn file/test mồ côi; cập nhật CLAUDE.md + README | node cũ 3.103 dòng |

Mỗi phase là một checkpoint demo được — robot luôn chạy được giữa các phase.

## 9. Out of scope

- Không đổi ROS contracts: `interfaces`, `safety`, `motion_core`, `hw_adapter` giữ nguyên.
- Không đổi MoveIt/planner config.
- Tách node `task_runtime` riêng (Hướng B) — để sau khi pipeline ổn định nếu cần.
- Frontend redesign tổng thể — chỉ thêm panel/filter cho System Log và task progress.
