# ĐỀ CƯƠNG LUẬN VĂN TỐT NGHIỆP — BẢN V3 (KHUNG VIẾT)

> **Đề tài:** Thiết kế hệ thống điều khiển robot công nghiệp Yaskawa GP4 bằng ngôn ngữ tự nhiên sử dụng ROS 2, mô hình ngôn ngữ lớn và camera RGB-D.
> **Tác giả:** Nguyễn Minh Hiếu – 20222531
> **GVHD:** TS. Nguyễn Thị Vân Anh; PGS.TS. Nguyễn Danh Huy
> **Bản:** v3 — viết lại khung cho khớp codebase `gp4_ws` (branch `upgrade-react-8626`, ngày 2026-06-12).

## Cách dùng tài liệu này

Mỗi chương gồm 4 khối:
- **(a) Khung dàn bài** — mục lục chi tiết, đánh số 1.1, 1.2 …
- **(b) Công thức + ký hiệu** — công thức cần đưa vào, ký hiệu, biến số
- **(c) Hình / bảng cần có** — danh sách, mô tả hình dạng, nguồn dữ liệu
- **(d) Đoạn viết mẫu** — 1–2 đoạn giọng luận văn, bạn đọc rồi sửa

Mọi claim phải map được sang file/commit trong repo. Mọi số liệu phải lấy từ `src/` hoặc log thật.

---

## 0. FRONT MATTER (theo mẫu ĐATN BKHN)

### 0.1. Bìa + trang đề tài + lời cảm ơn + tóm tắt VN + abstract EN

| Hạng mục | Nội dung chính | Nguồn |
|---|---|---|
| Bìa | Tên đề tài, GVHD, SV, khoa, Hà Nội 7/2026 | giữ nguyên |
| Lời cảm ơn | ~150 từ, không sáo rỗng | giữ nguyên |
| Tóm tắt VN | ~300 từ: pipeline 6 lớp, closed-loop, fail-closed, kết quả L1–L5 | viết lại theo Ch 5 |
| Abstract EN | ~300 từ, dùng "closed-loop", "fail-closed", "L1–L5" | dịch từ tóm tắt VN |

### 0.2. Mục lục — danh mục hình — danh mục bảng — danh mục viết tắt

Bảng viết tắt **phải** có các mục tối thiểu: ROS 2, DDS, RTPS, QoS, URDF, SRDF, IK, FK, TCP, PTP, LIN, CIRC, TRAC-IK, OMPL, TOTG, Ruckig, IR, LLM, D435i, RGB-D, ChArUco, DLS, SQP, YRC1000micro, MotoROS2, micro-ROS, PLd, IP67.

---

## CHƯƠNG 1 — GIỚI THIỆU ĐỀ TÀI

### (a) Khung dàn bài

```
1.1. Bối cảnh và động lực
    1.1.1. Robot công nghiệp & lập trình truyền thống (teach pendant)
    1.1.2. Xu hướng dùng LLM trong robotics
    1.1.3. Rủi ro khi để LLM điều khiển robot trực tiếp
1.2. Vấn đề nghiên cứu
    1.2.1. Câu hỏi trung tâm
    1.2.2. 4 câu hỏi nghiên cứu cụ thể
1.3. Mục tiêu đề tài
1.4. Đối tượng & phương pháp nghiên cứu
1.5. Phạm vi (có / không)
1.6. Đóng góp chính (6 điểm)
1.7. Kết cấu báo cáo
```

### (b) Công thức + ký hiệu

Không có công thức trong Ch 1. Ký hiệu xuất hiện: **ROS 2**, **MoveIt 2**, **MotoROS2**, **MCP** (MotoPlus), **HMI**, **LLM**, **IR**, **FactoryTask**, **WorldModel**, **fail-closed**, **closed-loop**.

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả | Nguồn dữ liệu |
|---|---|---|---|
| H 1.1 | Ảnh | Trạm robot GP4 thực tế (đã có sẵn trong Ch 2 đề cương cũ) | `references/DS_GP4.pdf` |
| H 1.2 | Sơ đồ khối | Pipeline 6 lớp tổng quan (HMI → llm_gateway → safety → motion_core → hw_adapter → MotoROS2 → robot) | vẽ lại từ `docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md` mục 3 |
| B 1.1 | Bảng | So sánh 3 cách tiếp cận LLM-robot: end-to-end, ReAct nhiều vòng, **đường ống kiểm chứng (chọn)** | tổng hợp |

### (d) Đoạn viết mẫu

> *Trong môi trường sản xuất, robot công nghiệp thường được lập trình thông qua teach pendant, phần mềm chuyên dụng của hãng hoặc các giao diện kỹ thuật đòi hỏi người vận hành có kiến thức chuyên sâu về hệ tọa độ, quỹ đạo, tốc độ và quy tắc an toàn. Cách lập trình này ổn định và phù hợp với dây chuyền lặp lại, nhưng trở nên cứng nhắc khi yêu cầu thay đổi nhanh, khi đối tượng thao tác thay đổi vị trí theo thời gian thực, hoặc khi cần tích hợp thêm cảm biến ngoài.*
>
> *Sự phát triển gần đây của các mô hình ngôn ngữ lớn mở ra khả năng cho phép người vận hành diễn đạt yêu cầu bằng ngôn ngữ tự nhiên, ví dụ: "đưa robot về vị trí home", "hạ xuống 5 cm", "gắp vật màu đỏ rồi đặt sang khu vực kia". Tuy nhiên, trong bối cảnh robot công nghiệp thật, cách tiếp cận end-to-end từ LLM xuống phần cứng là không chấp nhận được. Mô hình ngôn ngữ có thể sinh sai schema, sai đơn vị, bịa service name, hoặc bỏ qua giới hạn an toàn — những lỗi mà bộ điều khiển công nghiệp tuyệt đối không được phép dung thứ.*

---

## CHƯƠNG 2 — NỀN TẢNG KỸ THUẬT VÀ NGHIÊN CỨU LIÊN QUAN

### (a) Khung dàn bài

```
2.1. Robot Yaskawa GP4 và bộ điều khiển YRC1000micro
    2.1.1. Thông số cơ khí
    2.1.2. Cấu trúc khớp S–L–U–R–B–T, vùng làm việc
    2.1.3. YRC1000micro: an toàn PLd, I/O, micro-ROS
    2.1.4. Trạng thái cần giám sát
2.2. ROS 2 Humble, MoveIt 2, MotoROS2
    2.2.1. ROS 2: node / topic / service / action / QoS
    2.2.2. MoveIt 2: planning scene, collision checking
    2.2.3. Pilz / OMPL / TRAC-IK / TOTG / Ruckig
    2.2.4. MotoROS2: vai trò, kiến trúc micro-ROS Agent
2.3. Hệ tọa độ và động học robot
    2.3.1. Biến đổi thuần nhất, quy ước DH
    2.3.2. FK cho GP4
    2.3.3. IK và TRAC-IK (Newton + SQP song song)
    2.3.4. DLS, kỳ dị cổ tay J4–J5–J6
2.4. Mô hình ngôn ngữ lớn trong robotics
    2.4.1. Tổng quan LLM, prompt engineering
    2.4.2. ReAct và giới hạn của ReAct multi-turn
    2.4.3. Lý do chọn prompt + 2-tier parser
2.5. Biểu diễn trung gian (IR) — Semantic Contract
    2.5.1. IR là gì, vì sao không dùng function calling
    2.5.2. Yêu cầu của IR
2.6. Phần mềm safety-critical và nguyên tắc fail-closed
2.7. Camera RGB-D Intel RealSense D435i
    2.7.1. Cấu tạo, thông số
    2.7.2. Mô hình lỗ kim, intrinsic
    2.7.3. Hiệu chuẩn Eye-to-Hand (Park–Martin, Daniilidis)
2.8. Closed-loop robotics: observe → plan → act → verify
```

### (b) Công thức + ký hiệu

| Ký hiệu | Mô tả | Nơi dùng |
|---|---|---|
| ^{a}T_{b} | Ma trận biến đổi thuần nhất từ frame {a} sang {b} | § 2.3 |
| ^{i-1}T_{i} | Phép biến đổi khâu i theo DH | § 2.3.1 |
| a_{i-1}, α_{i-1}, d_i, θ_i | 4 tham số DH | § 2.3.1 |
| J(θ) | Ma trận Jacobian 6×6 | § 2.3.4 |
| Δθ | Bước cập nhật khớp | § 2.3.4 |
| λ | Hệ số cản DLS | § 2.3.4 |
| K | Ma trận intrinsic 3×3 của camera | § 2.7.2 |
| s | Bộ tham số scale-khẩu-cách-tiêu | § 2.7.2 |
| ^{cam}T_{base} | Ma trận ngoại tham số | § 2.7.3 |
| ^{b}T_{grip}_{ij} | Dịch chuyển gripper giữa pose i, j | § 2.7.3 |
| ^{cam}T_{grip}_{ij} | Dịch chuyển marker giữa pose i, j | § 2.7.3 |
| ω_{max}, α_{max} | Giới hạn vận tốc / gia tốc | § 2.6 |
| q_min, q_max | Giới hạn khớp operational | § 2.6 |

Công thức bắt buộc đưa vào:

```
(2.1)  ^{a}T_{b} = [[R, p], [0 0 0 1]]
(2.2)  ^{i-1}T_{i}(θ_i) = Rz(θ_i) · Tz(d_i) · Tx(a_{i-1}) · Rx(α_{i-1})
(2.3)  ^{0}T_{6}(θ) = Π_{i=1..6} ^{i-1}T_{i}(θ_i)
(2.4)  J(θ) = [J_v; J_ω]   với cột thứ i = [z_{i-1} × (p_{ee} − p_{i-1});  z_{i-1}]
(2.5)  Δθ = J^T (J J^T + λ^2 I)^{-1} Δx   (DLS)
(2.6)  K = [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]]
(2.7)  P_cam = (X/Z, Y/Z, 1)  sau khi áp K
(2.8)  AX = XB   với A = ^{b}T_{grip}_{ij},  B = ^{cam}T_{grip}_{ij}    (Hand-eye)
```

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả | Nguồn dữ liệu |
|---|---|---|---|
| H 2.1 | Ảnh | Robot GP4 thực tế | `references/DS_GP4.pdf` |
| H 2.2 | Ảnh | YRC1000micro | `references/DS_GP4.pdf` |
| H 2.3 | Ảnh | Camera D435i | `references/DS_GP4.pdf` |
| H 2.4 | Sơ đồ | Khung động học GP4 với 6 frame khớp + frame TCP | vẽ từ `motoman_gp4.urdf.xacro` |
| H 2.5 | Sơ đồ | Kiến trúc ROS 2: Publisher/Subscriber, Service, Action | vẽ tay |
| H 2.6 | Sơ đồ | Kiến trúc MoveIt 2 (MoveGroup / Planning Scene / Pipeline) | tham khảo `gp4_moveit_config` |
| H 2.7 | Sơ đồ | Kiến trúc MotoROS2 + micro-ROS Agent | tham khảo `motoros2_support` |
| H 2.8 | Sơ đồ | Mô hình lỗ kim, phép chiếu thuận/ngược | vẽ tay |
| H 2.9 | Sơ đồ | Eye-to-Hand setup, vị trí bảng ChArUco | từ `scratch_handeye.py` |
| B 2.1 | Bảng | Thông số GP4 (số trục, tải, lặp lại, tầm với) | datasheet Yaskawa |
| B 2.2 | Bảng | Thông số YRC1000micro | datasheet Yaskawa |
| B 2.3 | Bảng | Bộ tham số DH của GP4 | `motoman_gp4.urdf.xacro` |
| B 2.4 | Bảng | So sánh planner: Pilz / OMPL / TRAC-IK | tổng hợp từ config |
| B 2.5 | Bảng | Bảng 16 primitive hợp lệ trong `llm_schema.yaml` | `src/llm_gateway/config/llm_schema.yaml` |
| B 2.6 | Bảng | Bảng 7 trạng thái lỗi hệ thống | tổng hợp từ `specs/.../llm-gateway-factory-pipeline-design.md` mục 5 |

### (d) Đoạn viết mẫu

> *Robot Yaskawa Motoman GP4 là robot 6 bậc tự do với tải trọng 4 kg, độ lặp lại ±0,01 mm, cấp bảo vệ IP67 và tốc độ trục tối đa 1000°/s. Sáu khớp quay được hãng gắn nhãn theo chuẩn S, L, U, R, B, T; trong ROS 2 chúng tương ứng với `joint_1_s` đến `joint_6_t`. Ba khớp đầu (S–L–U) phục vụ định vị hình học cánh tay, ba khớp sau (R–B–T) tạo thành cổ tay cầu (spherical wrist) phục vụ định hướng TCP. Dải chuyển động cực đại là S ±170°, L +130°/−110°, U +200°/−65°, R ±200°, B ±123°, T ±455°; đây là cơ sở trực tiếp cho giới hạn khớp trong MoveIt 2 và cho ràng buộc workspace.*
>
> *Trong hệ thống này, bài toán động học không được giải trực tiếp bằng bảng tham số Denavit–Hartenberg. Toàn bộ hình học được khai báo trong tệp `motoman_gp4.urdf.xacro` với mỗi thẻ `<joint>` mô tả quan hệ giữa hai khâu liên tiếp. MoveIt 2 dựng cây động học nội bộ từ chính mô hình này, nhờ đó việc kiểm tra va chạm, lập kế hoạch quỹ đạo và xử lý tín hiệu vào/ra cùng nằm trong một quy trình thống nhất.*

---

## CHƯƠNG 3 — THIẾT KẾ KIẾN TRÚC HỆ THỐNG

### (a) Khung dàn bài

```
3.1. Yêu cầu hệ thống
    3.1.1. Yêu cầu chức năng (F-REQ)
    3.1.2. Yêu cầu phi chức năng (NF-REQ)
    3.1.3. Ràng buộc an toàn (an toàn phần cứng + phần mềm)
3.2. Tổng quan kiến trúc 6 lớp
    3.2.1. Nguyên tắc phân lớp, ranh giới trách nhiệm
    3.2.2. Sơ đồ pipeline 6 lớp
    3.2.3. Luồng dữ liệu tổng thể
3.3. Lớp giao tiếp người – máy (HMI)
    3.3.1. Frontend React 18 + Vite
    3.3.2. Backend FastAPI + WebSocket
    3.3.3. Quick command, raw NL, System Log
3.4. Lớp LLM Gateway
    3.4.1. Nguyên tắc: LLM không điều khiển robot
    3.4.2. 2-tier parser: direct_commands + task_planner
    3.4.3. Cấu trúc module: direct_commands / task_planner / factory_task / task_runtime / llm_gateway_node
3.5. Lớp Safety
    3.5.1. Safety contract fail-closed
    3.5.2. Các kiểm tra: workspace, forbidden zone, joint limits, MOVE_REL delta, perception freshness
    3.5.3. Bảng lỗi thống nhất (B 3.x)
3.6. Lớp Motion Core
    3.6.1. Action server /execute_motion
    3.6.2. 16 primitive, ánh xạ primitive → MoveIt
    3.6.3. Chuỗi thực thi: validate → goal → plan → execute → feedback
3.7. Lớp Hardware Adapter
    3.7.1. Action server /hw_adapter/dispatch_trajectory
    3.7.2. Kiểm tra trạng thái đầu, session, point budget
    3.7.3. Cầu nối MotoROS2
3.8. Lớp Perception (Camera D435i)
    3.8.1. ROS 2 driver, luồng ảnh RGB + depth
    3.8.2. Nhận diện vật, sinh pose trong base_link
    3.8.3. Eye-to-Hand calibration
3.9. WorldModel và TaskCompiler
    3.9.1. WorldModel: snapshot, freshness, region/object pose
    3.9.2. TaskCompiler: grounding, fail-closed
3.10. Cơ chế System Log thống nhất
    3.10.1. Schema 7 trường (ts, level, source, category, event, detail, data)
    3.10.2. 7 category: TASK / MOTION / PERCEPTION / HARDWARE / SAFETY / IO / SYSTEM
    3.10.3. Topic /llm_gateway/task_events
3.11. Đồng bộ trạng thái và các action phụ
    3.11.1. /validate_command, /get_current_pose
    3.11.2. Confirm một lần, STOP tức thời
```

### (b) Công thức + ký hiệu

| Ký hiệu | Mô tả |
|---|---|
| p_ee ∈ R^3 | Tọa độ TCP trong base_link |
| R_ee ∈ SO(3) | Ma trận quay TCP |
| ξ = (p_ee, R_ee) | Pose TCP |
| q ∈ R^6 | Vectơ biến khớp |
| v_max, a_max, j_max | Giới hạn vận tốc / gia tốc / giật |
| s_v, s_a | Hệ số scale vận tốc / gia tốc (mặc định 0.06) |
| t_fresh | Tuổi freshness của một fact (giây) |
| B_p | Hộp giới hạn workspace |
| Z = {Z_1, Z_2, …} | Tập forbidden zone (AABB) |
| Δ_max | Ngưỡng MOVE_REL dịch chuyển tối đa (0.21 m) |
| Γ = (V, E) | Cây FactoryTask; V = node, E = cha–con |
| S = (N, F) | Trạng thái WorldModel: N object, F fact freshness |
| FAILED = {MISSING_SLOT, UNSUPPORTED_OR_AMBIGUOUS, WORLD_MODEL_UNGROUNDED, PERCEPTION_STALE, SAFETY_REJECTED, MOTION_FAILED, GRASP_FAILED, OPERATOR_STOP} | Bảng lỗi thống nhất |
| OK = (semantic_ok ∧ safety_ok ∧ planner_ok ∧ hw_ok) | Điều kiện cho phép thực thi |

Công thức bắt buộc:

```
(3.1)  q̇_safe = s_v · q̇_max
(3.2)  q̈_safe = s_a · q̈_max
(3.3)  ∀p_ee ∈ traj : p_ee ∈ B_p ∧ ¬∃Z ∈ Z : p_ee ∈ Z
(3.4)  ‖Δp_ee‖ ≤ Δ_max     (MOVE_REL)
(3.5)  fresh(f) ⇔ (t_now − t_stamp(f)) ≤ t_fresh    (5s mặc định)
(3.6)  OK = 𝟙(semantic_ok) ∧ 𝟙(safety_ok) ∧ 𝟙(planner_ok) ∧ 𝟙(hw_ok)
(3.7)  ∀f ∈ fact :  ¬fresh(f) → re-observe hoặc fail-closed
(3.8)  Γ_root  →^{task_runtime}  sequence[step_1, step_2, …, step_n]  → primitive
```

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả | Nguồn |
|---|---|---|---|
| H 3.1 | Sơ đồ khối | Pipeline 6 lớp dọc | tự vẽ |
| H 3.2 | Sơ đồ tuần tự | Submit NL → review → confirm → run | từ `docs/superpowers/specs/2026-06-10-llm-gateway-factory-pipeline-design.md` §4 |
| H 3.3 | Sơ đồ khối | Cấu trúc llm_gateway (5 module) | từ spec trên §3 |
| H 3.4 | Sơ đồ | Cây FactoryTask ví dụ (sequence + for_each + retry) | tự vẽ từ §3.4 |
| H 3.5 | Sơ đồ trạng thái | Task state machine: PLANNED → CONFIRMED → RUNNING → PAUSED/STOPPED → DONE/FAILED | từ spec §5 |
| H 3.6 | Sơ đồ | Luồng System Log: 7 category, schema cố định | từ spec §6 |
| H 3.7 | Sơ đồ | ROS 2 interface map (service/action/topic) | từ `CLAUDE.md` root |
| B 3.1 | Bảng | Yêu cầu hệ thống (F-REQ, NF-REQ) | tự tổng hợp |
| B 3.2 | Bảng | 16 primitive hợp lệ | `src/llm_gateway/config/llm_schema.yaml` |
| B 3.3 | Bảng | Bảng 8 lỗi thống nhất + hành vi | từ spec §5 |
| B 3.4 | Bảng | Tham số safety: workspace_bounds, forbidden_zones, joint_limits_override, motion_limits | `src/safety/config/safety_rules.yaml` |
| B 3.5 | Bảng | Trường schema System Log | từ spec §6 |

### (d) Đoạn viết mẫu

> *Hệ thống được tổ chức thành sáu lớp có ranh giới rõ ràng, theo thứ tự từ lớp giao tiếp đến lớp phần cứng: HMI, llm_gateway, safety, motion_core, hw_adapter và MotoROS2. Nguyên tắc thiết kế là LLM chỉ đóng vai trò hiểu ý định và sinh biểu diễn trung gian có cấu trúc (FactoryTask); mọi quyết định liên quan đến an toàn, lập kế hoạch và thực thi phải đi qua các lớp deterministic có kiểm chứng. Lệnh sai schema, sai đơn vị, ngoài workspace, hoặc vượt giới hạn khớp đều bị chặn trước khi một joint nào của robot được phép dịch chuyển.*
>
> *Lớp llm_gateway thực hiện phân tích câu lệnh theo hai tầng. Tầng một (`direct_commands`) là bộ phân tích định lượng dựa trên biểu thức chính quy, xử lý khoảng năm lệnh an toàn thường gặp (stop, home, get pose, alarm reset, wait N giây) mà không cần gọi mô hình ngôn ngữ. Tầng hai (`task_planner`) sinh cây FactoryTask từ mô hình ngôn ngữ thông qua một lần gọi duy nhất, dùng prompt kỹ thuật có cấu trúc. Một câu lệnh chỉ đi qua đúng một trong hai tầng này; không tồn tại đường đi song song để tránh hiện tượng cùng một câu sinh ra hai kết quả khác nhau như đã từng xảy ra ở phiên bản trước.*

---

## CHƯƠNG 4 — TRIỂN KHAI HỆ THỐNG

### (a) Khung dàn bài

```
4.1. Cấu trúc workspace và danh sách package
    4.1.1. Sơ đồ thư mục gp4_ws
    4.1.2. 9 package chính: interfaces, llm_gateway, safety, motion_core, primitives, hw_adapter, supervisor, jog_pendant, gp4_bringup
    4.1.3. Package phụ trợ: gp4_moveit_config, gp4_station, gp4_perception
4.2. Cấu hình MoveIt 2 cho GP4
    4.2.1. URDF / SRDF
    4.2.2. Kinematics (TRAC-IK)
    4.2.3. Joint limits, Pilz cartesian limits
    4.2.4. OMPL planning, controllers
4.3. Triển khai safety package
    4.3.1. Service /validate_command
    4.3.2. Các validator: workspace, forbidden zone, joint, MOVE_REL, freshness
    4.3.3. Đọc safety_rules.yaml
4.4. Triển khai llm_gateway
    4.4.1. Schema lệnh: llm_schema.yaml
    4.4.2. direct_commands.py
    4.4.3. task_planner.py (single-shot, retry/backoff)
    4.4.4. factory_task.py: FactoryTask, WorldModel, TaskCompiler
    4.4.5. task_runtime.py: walk tree, retry/fallback/replan
    4.4.6. llm_gateway_node.py: ROS host mỏng
4.5. Triển khai motion_core
    4.5.1. Action server /execute_motion
    4.5.2. Ánh xạ primitive → MoveIt goal
    4.5.3. TOTG + Ruckig
4.6. Triển khai hw_adapter
    4.6.1. Action server /hw_adapter/dispatch_trajectory
    4.6.2. Bọc MotoROS2
    4.6.3. Kiểm tra trạng thái, point budget
4.7. Triển khai supervisor
    4.7.1. Audit log (rosbag2 + JSONL)
    4.7.2. /llm_gateway/task_events publisher
4.8. HMI Frontend
    4.8.1. Cấu trúc React: App, GP4HMI, CommandComposer, SystemLog
    4.8.2. useGP4Bridge hook, bridgeClient
    4.8.3. CSS tokens, system-log.css, chat.css
4.9. HMI Backend
    4.9.1. FastAPI app
    4.9.2. /api/commands, /api/hmi/stream
    4.9.3. ROS adapter: telemetry, command, jog
4.10. Triển khai camera D435i
    4.10.1. Driver RealSense ROS 2
    4.10.2. Hand-eye calibration script
    4.10.3. Nhận diện vật, xuất pose
4.11. Launch và quy trình chạy
    4.11.1. sim.launch.py
    4.11.2. hw.launch.py
    4.11.3. llm_stack.launch.py
```

### (b) Công thức + ký hiệu

| Ký hiệu | Mô tả |
|---|---|
| t_plan, t_exec, t_total | Thời gian lập kế hoạch / thực thi / tổng |
| n_point | Số điểm trajectory |
| δ_q | Sai số khớp |
| Σ_event | Tập event log phát ra trong một task |
| R̂_T | Tin cậy detection vật |
| δ_calib | Sai số calibration eye-to-hand (mm) |

Công thức:

```
(4.1)  t_total = t_plan + t_exec
(4.2)  n_point ≤ n_budget     (giới hạn point budget của hw_adapter)
(4.3)  R̂_T ≥ R_T_min  ⇒  pickable
(4.4)  δ_calib < δ_max         (tiêu chí chấp nhận calibration)
(4.5)  q̇ = (q[k+1] − q[k]) / dt    (ước lượng vận tốc khớp rời rạc)
```

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả | Nguồn |
|---|---|---|---|
| H 4.1 | Sơ đồ cây | Cấu trúc thư mục `gp4_ws` (rút gọn 1–2 level) | `tree -L 2 src/ hmi/` |
| H 4.2 | Sơ đồ | Luồng khởi động `gp4_bringup/sim.launch.py` | đọc launch file |
| H 4.3 | Sơ đồ | Schema IR (16 primitive, 1 ví dụ HOME, 1 ví dụ LIN) | từ `llm_schema.yaml` |
| H 4.4 | Sơ đồ | Cấu trúc `factory_task.py` (FactoryTask node types) | từ code |
| H 4.5 | Sơ đồ | Cấu trúc HMI React (component tree) | từ `hmi/frontend/components` |
| H 4.6 | Sơ đồ tuần tự | Submit → review → confirm → run → STOP | tự vẽ |
| B 4.1 | Bảng | 9 package chính + vai trò | tổng hợp |
| B 4.2 | Bảng | 16 primitive và ý nghĩa | từ `llm_schema.yaml` |
| B 4.3 | Bảng | Tham số TRAC-IK (timeout, attempts) | `src/gp4_moveit_config/config/kinematics.yaml` |
| B 4.4 | Bảng | Tham số Pilz cartesian limits | `pilz_cartesian_limits.yaml` |
| B 4.5 | Bảng | Bảng định danh node ROS 2 (service/action/topic) | từ CLAUDE.md root |

### (d) Đoạn viết mẫu

> *Package `llm_gateway` được viết lại theo hướng phân lớp: mỗi module đảm nhận đúng một trách nhiệm. Module `direct_commands` chứa các biểu thức chính quy và bảng ánh xạ lệnh an toàn, không gọi mô hình ngôn ngữ, không phụ thuộc ROS 2. Module `task_planner` chứa prompt hệ thống, client gọi mô hình, cơ chế thử lại với backoff và bộ phân tích JSON; module này cũng không import ROS 2. Module `factory_task` định nghĩa cây tác vụ, WorldModel và TaskCompiler với cơ chế grounding fail-closed. Module `task_runtime` là nơi duy nhất phát lệnh chuyển động, có nhiệm vụ đi qua cây, thực hiện các node lặp, retry, fallback, replan và phát sự kiện System Log. Cuối cùng, `llm_gateway_node.py` đóng vai trò ROS host mỏng, chỉ thực hiện khởi tạo client, khai báo service/action và chuyển tiếp sự kiện — toàn bộ logic nghiệp vụ nằm ở bốn module trên.*

---

## CHƯƠNG 5 — THỰC NGHIỆM VÀ ĐÁNH GIÁ

### (a) Khung dàn bài

```
5.1. Phương pháp đánh giá tổng thể
    5.1.1. Tháp đánh giá 5 tầng L1–L5
    5.1.2. Nguyên tắc tăng dần độ cô lập test
5.2. L1 — IR Generation
    5.2.1. Mục tiêu: schema hợp lệ, intent accuracy, không hallucination
    5.2.2. Bộ test ~120 câu VN/EN (xuất từ test fixture)
    5.2.3. Kết quả: % schema_valid, % intent_accuracy
5.3. L2 — Safety Validation
    5.3.1. Mục tiêu: reject không an toàn, pass khi an toàn
    5.3.2. Test: workspace, forbidden zone, joint limits, MOVE_REL delta, freshness
    5.3.3. Kết quả: false-reject rate, false-accept rate
5.4. L3 — Motion Planning
    5.4.1. Mục tiêu: planning success, IK failure rate, trajectory quality
    5.4.2. Test: 16 primitive, motion_core trong sim
    5.4.3. Kết quả: plan_success_rate, IK_failure_rate, trajectory_smoothness
5.5. L4 — Full Pipeline E2E
    5.5.1. Mục tiêu: NL → robot thành công, latency
    5.5.2. Test trong sim với 10–20 kịch bản tổng hợp
    5.5.3. Kết quả: success_rate, p50/p95 latency
5.6. L5 — Vision Grounding
    5.6.1. Mục tiêu: độ chính xác detect vật bằng D435i
    5.6.2. Test: bộ mẫu vật (màu, hình dạng, vị trí khác nhau)
    5.6.3. Kết quả: detection accuracy (mAP hoặc IoU), δ_calib
5.7. Kết quả đánh giá tổng hợp
    5.7.1. Bảng 5 tầng × 4 chỉ số = 20 ô
    5.7.2. Nhận xét từng tầng
    5.7.3. So sánh với baseline (ReAct cũ, end-to-end)
5.8. Thực nghiệm bổ sung
    5.8.1. Kịch bản STOP giữa chừng
    5.8.2. Kịch bản perception stale → re-observe
    5.8.3. Kịch bản grasp fail → retry → fallback
```

### (b) Công thức + ký hiệu

| Ký hiệu | Mô tả |
|---|---|
| A_IR | Độ chính xác intent (L1) |
| A_schema | Tỷ lệ schema hợp lệ |
| H_rate | Tỷ lệ hallucination |
| FPR, FNR | False positive / false negative rate (L2) |
| P_success | Tỷ lệ plan thành công (L3) |
| IK_fail | Tỷ lệ IK fail |
| T_smooth | Chỉ số mượt trajectory (jerk, snap) |
| S_e2e | Success rate end-to-end (L4) |
| L_p50, L_p95 | Latency percentile |
| mAP | Mean Average Precision (L5) |
| IoU | Intersection over Union |
| δ_calib | Sai số hand-eye calibration |

Công thức:

```
(5.1)  A_IR = (số câu intent đúng) / (tổng câu test)
(5.2)  A_schema = (số IR schema hợp lệ) / (tổng)
(5.3)  FPR = FP / (FP + TN)        (false accept = cho qua lệnh xấu)
(5.4)  FNR = FN / (FN + TP)        (false reject = chặn lệnh tốt)
(5.5)  P_success = (số plan OK) / (tổng plan)
(5.6)  T_smooth = ‖J‖_∞ của trajectory
(5.7)  S_e2e = (số task thành công) / (tổng task)
(5.8)  L_p95 = quantile(latency, 0.95)
(5.9)  IoU = area(B_pred ∩ B_gt) / area(B_pred ∪ B_gt)
```

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả | Nguồn |
|---|---|---|---|
| H 5.1 | Hình | Tháp đánh giá L1–L5 (bản clean, tiếng Việt) | vẽ lại từ pyramid bạn gửi |
| H 5.2 | Đồ thị | Bar chart so sánh A_IR giữa prompt gốc / prompt đã tinh chỉnh | log thật |
| H 5.3 | Đồ thị | ROC safety: FPR vs FNR theo ngưỡng | log thật |
| H 5.4 | Đồ thị | Boxplot T_smooth theo 16 primitive | log thật |
| H 5.5 | Đồ thị | Histogram latency E2E | log thật |
| H 5.6 | Ảnh | Confusion matrix detection vật (L5) | log thật |
| H 5.7 | Ảnh | Ảnh chụp Detect vật thành công + overlay bounding box | ảnh thật từ sim/hw |
| H 5.8 | Ảnh | Ảnh chụp Detect thất bại (sai màu, sai vị trí) | ảnh thật |
| B 5.1 | Bảng | Tổng hợp 20 chỉ số (5 tầng × 4 chỉ số) | tổng hợp |
| B 5.2 | Bảng | Bộ test L1 (số câu, phân bố) | từ test fixture |
| B 5.3 | Bảng | Bộ test L2 (số kịch bản) | từ test fixture |
| B 5.4 | Bảng | Bộ test L3 (16 primitive × số pose) | từ test fixture |
| B 5.5 | Bảng | Bộ test L4 (kịch bản E2E) | từ test fixture |
| B 5.6 | Bảng | Bộ test L5 (số vật, số pose) | từ test fixture |
| B 5.7 | Bảng | Kết quả calibration eye-to-hand | từ `scratch_handeye.py` |

### (d) Đoạn viết mẫu

> *Đánh giá hệ thống được tổ chức theo năm tầng với độ cô lập test tăng dần và độ tích hợp giảm dần. Tầng một (L1) đo khả năng của llm_gateway sinh ra biểu diễn trung gian đúng schema và đúng ý định; tầng hai (L2) đo khả năng lớp safety phân biệt lệnh an toàn và không an toàn; tầng ba (L3) đo khả năng lập kế hoạch chuyển động của motion_core; tầng bốn (L4) đo tỷ lệ thành công và độ trễ khi chạy toàn pipeline; tầng năm (L5) đo độ chính xác của perception trong việc nhận diện và định vị vật. Cách đánh giá phân tầng này giúp xác định rõ bottleneck khi tỷ lệ thành công giảm: nếu L4 thấp mà L1, L2, L3 đều cao, nguyên nhân nằm ở sự ghép nối giữa các tầng chứ không phải ở từng tầng riêng lẻ.*

---

## CHƯƠNG 6 — PHÂN TÍCH KẾT QUẢ, LỖI THƯỜNG GẶP, GIỚI HẠN

### (a) Khung dàn bài

```
6.1. Phân tích kết quả theo tầng
    6.1.1. Điểm mạnh
    6.1.2. Điểm yếu
6.2. Các lỗi thường gặp
    6.2.1. Lỗi LLM (hallucination, sai schema)
    6.2.2. Lỗi perception (stale, occlusion, lighting)
    6.2.3. Lỗi planning (IK fail, planner timeout)
    6.2.4. Lỗi phần cứng (alarm, e-stop, mất kết nối agent)
6.3. Giới hạn hệ thống
    6.3.1. Giới hạn về ngôn ngữ (chỉ VN/EN, ~5 lệnh tầng 1)
    6.3.2. Giới hạn về perception (ánh sáng, che khuất, marker)
    6.3.3. Giới hạn về chuyển động (tốc độ, tải, vùng làm việc)
    6.3.4. Giới hạn về tính an toàn (không thay thế safety controller Yaskawa)
6.4. Yếu tố ảnh hưởng độ ổn định khi vận hành robot thật
    6.4.1. Nhiễu mạng, micro-ROS Agent
    6.4.2. Độ trễ LLM, prompt caching
    6.4.3. Calibration drift
    6.4.4. Tải CPU/GPU host
```

### (b) Công thức + ký hiệu

```
(6.1)  S_e2e = S_L1 · S_L2 · S_L3 · S_L5   (xấp xỉ tích, khi các tầng độc lập)
(6.2)  δ_e2e = δ_LLM + δ_safety + δ_planner + δ_hw
```

Trong đó δ là độ trễ từng tầng, δ_e2e là độ trễ end-to-end.

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả |
|---|---|---|
| B 6.1 | Bảng | Tổng hợp lỗi theo tầng (mã lỗi, tần suất, cách xử lý) |
| B 6.2 | Bảng | Giới hạn hệ thống (tham số, giá trị) |
| H 6.1 | Đồ thị | Phân bố lỗi theo thời gian (System Log category) |
| H 6.2 | Đồ thị | Latency theo từng tầng (stacked bar) |

### (d) Đoạn viết mẫu

> *Trong quá trình thực nghiệm, bốn nhóm lỗi xuất hiện thường xuyên nhất là: (i) lỗi LLM sinh schema không hợp lệ hoặc hallucination tên service; (ii) lỗi perception khi vật bị che khuất một phần hoặc điều kiện ánh sáng thay đổi; (iii) lỗi planning khi IK thất bại ở vị trí gần kỳ dị cổ tay; (iv) lỗi phần cứng khi alarm kích hoạt do quá tải hoặc mất kết nối tới micro-ROS Agent. Bốn nhóm lỗi này tương ứng với bốn lớp LLM, perception, motion và hardware, mỗi lớp có cơ chế xử lý riêng. Đáng lưu ý, lỗi LLM gần như được loại bỏ hoàn toàn ở tầng một nhờ tầng `direct_commands` xử lý các lệnh thường gặp mà không cần gọi mô hình, trong khi ở tầng hai bộ lọc schema nghiêm ngặt kết hợp với retry/backoff đã giảm đáng kể tỷ lệ hallucination so với phiên bản trước.*

---

## CHƯƠNG 7 — KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN

### (a) Khung dàn bài

```
7.1. Tổng kết kết quả đạt được
7.2. Đóng góp của đồ án
7.3. Hạn chế
7.4. Hướng phát triển tiếp theo
    7.4.1. Mở rộng tập primitive
    7.4.2. Tăng cường perception (nhiều camera, point cloud)
    7.4.3. Tối ưu độ trễ LLM (cache, batching)
    7.4.4. Tích hợp giám sát an toàn từ xa
    7.4.5. Tích hợp với teach pendant để chuyển tiếp liền mạch
```

### (b) Công thức + ký hiệu

Không có công thức mới; có thể tóm tắt (5.1)–(6.2).

### (c) Hình / bảng cần có

| Mã | Loại | Mô tả |
|---|---|---|
| B 7.1 | Bảng | Tổng hợp đóng góp đối chiếu mục tiêu §1.3 |
| B 7.2 | Bảng | Lộ trình hướng phát triển (ngắn hạn / trung hạn / dài hạn) |

### (d) Đoạn viết mẫu

> *Đồ án đã xây dựng được một hệ thống điều khiển robot công nghiệp Yaskawa GP4 bằng ngôn ngữ tự nhiên, trong đó mô hình ngôn ngữ lớn chỉ đóng vai trò sinh biểu diễn trung gian có cấu trúc, còn toàn bộ quyết định an toàn, lập kế hoạch và thực thi thuộc về các lớp deterministic. Hệ thống đã được đánh giá theo năm tầng độc lập từ khả năng sinh lệnh của LLM đến độ chính xác của perception, kết quả cho thấy tỷ lệ thành công end-to-end ổn định trong mô phỏng và có thể chuyển sang vận hành trên robot thật sau khi hoàn tất hiệu chuẩn và kiểm định an toàn độc lập.*

---

## PHỤ LỤC A — RUBRIC ĐÁNH GIÁ L1–L5

### A.1. Nguyên tắc chung

- **Tăng dần độ tích hợp:** L1 đo một module, L5 đo toàn hệ thống trong điều kiện vận hành.
- **Tăng dần độ cô lập test:** tầng dưới dùng mock, tầng trên dùng thật.
- **Mỗi tầng có ít nhất 4 chỉ số định lượng.**

### A.2. Bảng chỉ số

| Tầng | Mục tiêu | Chỉ số chính | Công thức | Ngưỡng chấp nhận (đề xuất) |
|---|---|---|---|---|
| L1 IR Generation | Schema hợp lệ, intent accuracy, không hallucination | A_IR, A_schema, H_rate | (5.1), (5.2) | A_IR ≥ 0.90, A_schema ≥ 0.95, H_rate ≤ 0.02 |
| L2 Safety Validation | Reject không an toàn, pass khi an toàn | FPR, FNR, τ_reject, coverage | (5.3), (5.4) | FPR ≤ 0.01, FNR ≤ 0.05 |
| L3 Motion Planning | Planning success, IK fail, trajectory quality | P_success, IK_fail, T_smooth | (5.5), (5.6) | P_success ≥ 0.95, IK_fail ≤ 0.05, T_smooth ≤ ngưỡng |
| L4 Full Pipeline E2E | NL → robot, latency | S_e2e, L_p50, L_p95, abort_rate | (5.7), (5.8) | S_e2e ≥ 0.85, L_p95 ≤ 8 s, abort_rate ≤ 0.05 |
| L5 Vision Grounding | Detect vật chính xác bằng D435i | mAP, IoU, δ_calib, R̂_T | (5.9), (4.3), (4.4) | mAP ≥ 0.80, IoU ≥ 0.70, δ_calib ≤ 5 mm |

### A.3. Cách chạy từng tầng

- **L1:** `pytest src/llm_gateway/tests/test_task_planner.py` với fixture câu VN/EN.
- **L2:** `pytest src/safety/tests/test_command_validator.py` với bộ test workspace / forbidden zone / joint.
- **L3:** `ros2 launch gp4_bringup sim.launch.py` + script batch 16 primitive.
- **L4:** Kịch bản NL → HMI → gateway → motion → hw, đo log.
- **L5:** Đặt vật mẫu trước camera, chạy detection, so sánh với ground truth pose.

---

## PHỤ LỤC B — CHECKLIST KHI VIẾT TỪNG CHƯƠNG

- [ ] Có đoạn mở đầu chương (3–5 câu) giới thiệu chương này làm gì.
- [ ] Có đoạn kết thúc chương (3–5 câu) tóm tắt đã chứng minh gì, dẫn sang chương sau.
- [ ] Mỗi bảng / hình đều có lời dẫn 1–2 câu trước, và 1–2 đoạn phân tích sau.
- [ ] Mọi con số (góc khớp, sai số mm, tỷ lệ %) đều trích từ `src/`, log, hoặc test fixture.
- [ ] Mọi khẳng định "đã làm" phải có dẫn chứng (log, commit, ảnh).
- [ ] Không viết "theo code", "qua phân tích codebase"; viết theo giọng luận văn: "hệ thống được thiết kế…", "kết quả thực nghiệm cho thấy…".
- [ ] Không claim quá khả năng đã làm (đặc biệt: ISO 10218, fine-tune Qwen, ReAct multi-turn).

---

## PHỤ LỤC C — BẢNG ĐỐI CHIẾU OUTLINE CŨ → OUTLINE V3

| Outline cũ (v2) | Outline v3 | Ghi chú |
|---|---|---|
| ReAct vòng lặp nhiều vòng | Single-shot LLM + task_runtime loop | Bỏ ReAct, dùng cây FactoryTask |
| Fine-tune Qwen 2.5-7B + QLoRA | Prompt engineering + 2-tier parser | Không có fine-tune trong codebase |
| Đánh giá L1–L4 | Đánh giá L1–L5 (thêm L5 Vision) | Khớp pyramid bạn gửi |
| 6 đóng góp | 6 đóng góp (sửa chi tiết) | Khớp codebase |
| Bảng 8 primitive | Bảng 16 primitive | Lấy từ `llm_schema.yaml` |
| Bảng 6 trạng thái lỗi | Bảng 8 trạng thái lỗi (mở rộng OPERATOR_STOP, GRASP_FAILED) | Lấy từ `specs/.../llm-gateway-factory-pipeline-design.md` |

---

## PHỤ LỤC D — DANH SÁCH CÔNG THỨC TỔNG HỢP

```
Ch 2: (2.1)–(2.8)        — biến đổi thuần nhất, DH, FK, Jacobian, DLS, hand-eye
Ch 3: (3.1)–(3.8)        — safety constraints, freshness, OK predicate
Ch 4: (4.1)–(4.5)        — thời gian, point budget, calibration, vận tốc rời rạc
Ch 5: (5.1)–(5.9)        — đánh giá L1–L5
Ch 6: (6.1)–(6.2)        — phân tích tích lũy và độ trễ
```

---

## PHỤ LỤC E — TÀI LIỆU THAM KHẢO GỢI Ý (CẦN BỔ SUNG)

- Tài liệu Yaskawa: `references/DS_GP4.pdf`, `Flyer_Robot_GP4_E_05.2022.pdf`.
- Tài liệu ROS 2 Humble chính thức.
- MoveIt 2 tutorials.
- Intel RealSense D435i datasheet.
- TRAC-IK paper (Beeson, Ames).
- Damped Least Squares (Wampler, Nakamura).
- Park & Martin hand-eye calibration.
- Bài báo ReAct (Yao et al., 2022) — để bàn về hạn chế.
- LLM evaluation in robotics (gợi ý: survey gần đây).
- ISO 10218-1, ISO/TS 15066 (bàn về giới hạn an toàn — chỉ để tham chiếu).
