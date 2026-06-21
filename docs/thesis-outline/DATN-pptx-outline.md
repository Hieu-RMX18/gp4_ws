# OUTLINE POWERPOINT BẢO VỆ LUẬN VĂN

> **Đề tài:** Thiết kế hệ thống điều khiển robot Yaskawa GP4 bằng ngôn ngữ tự nhiên sử dụng ROS 2, LLM và camera RGB-D.
> **Bản:** v1 — dành cho bài thuyết trình 15–20 phút + 10 phút Q&A trước hội đồng.
> **Khung tham chiếu nội dung:** `DATN-outline-v3.md` (cùng thư mục).

## Cách dùng tài liệu

Mỗi slide ghi rõ:
- **Mục đích** — slide này truyền tải điều gì
- **Số slide đề xuất** — tổng ~22–26 slide
- **Nội dung chính** — gạch đầu dòng
- **Hình / bảng / sơ đồ** — dùng cái gì
- **Ghi chú diễn giải** — nói gì trong 30–60 giây

Mỗi slide KHÔNG nên có quá 5 dòng chữ. Ưu tiên hình + sơ đồ + bullet ngắn.

---

## PHẦN 0 — MỞ ĐẦU (Slide 1–3, ~2 phút)

### Slide 1 — Bìa

**Mục đích:** Giới thiệu đề tài, tác giả, GVHD.
**Nội dung chính:**
- Logo trường
- Tên đề tài (đầy đủ, tiếng Việt)
- SV: Nguyễn Minh Hiếu – 20222531
- GVHD: TS. Nguyễn Thị Vân Anh; PGS.TS. Nguyễn Danh Huy
- Hà Nội, 7/2026

**Ghi chú diễn giải:** Mở đầu ~20 giây: "Em xin phép trình bày đồ án tốt nghiệp …".

### Slide 2 — Nội dung trình bày

**Mục đích:** Cho hội đồng thấy flow sẽ trình bày.
**Nội dung chính:**
1. Bối cảnh & vấn đề
2. Mục tiêu & đóng góp
3. Cơ sở lý thuyết
4. Thiết kế kiến trúc
5. Triển khai
6. Thực nghiệm & đánh giá L1–L5
7. Kết quả & hạn chế
8. Kết luận & hướng phát triển

**Ghi chú:** Giữ 30 giây. Không đọc từng mục.

### Slide 3 — Trạm robot thực tế

**Mục đích:** Minh họa đối tượng điều khiển ngay từ đầu.
**Nội dung chính:**
- 1 ảnh trạm GP4 + YRC1000micro thực tế (full frame)
- 1 ảnh camera D435i (crop nhỏ ở góc)

**Nguồn:** `references/DS_GP4.pdf`.
**Ghi chú:** "Đây là trạm thực nghiệm của đồ án, gồm robot 6 bậc tự do, controller YRC1000micro và camera RGB-D. Hệ thống em xây dựng cho phép điều khiển trạm này bằng tiếng Việt tự nhiên."

---

## PHẦN 1 — BỐI CẢNH, VẤN ĐỀ, MỤC TIÊU (Slide 4–7, ~4 phút)

### Slide 4 — Lập trình robot truyền thống & hạn chế

**Mục đích:** Cho thấy bài toán thực tế cần giải.
**Nội dung chính:**
- 3 bullet:
  - Teach pendant / phần mềm hãng → đòi hỏi chuyên gia
  - Thay đổi vị trí vật → phải lập trình lại
  - Tích hợp cảm biến (vision) → khó
- 1 icon hoặc ảnh minh họa teach pendant

**Ghi chú:** Giọng điệu "vấn đề tồn tại", không phàn nàn.

### Slide 5 — Cơ hội từ LLM

**Mục đích:** Mở ra hướng giải mới.
**Nội dung chính:**
- Ví dụ 2–3 câu lệnh tiếng Việt: "về home", "hạ 5cm", "gắp vật đỏ"
- 1 logo / ảnh minh họa LLM (trung tính)

**Ghi chú:** "Mô hình ngôn ngữ lớn có thể hiểu câu lệnh tự nhiên — nhưng vấn đề là: có nên để nó điều khiển robot trực tiếp không?"

### Slide 6 — Vấn đề nghiên cứu & câu hỏi đặt ra

**Mục đích:** Đặt vấn đề học thuật.
**Nội dung chính:**
- Câu hỏi trung tâm (1 dòng to):
  > *"Làm thế nào để chuyển câu lệnh tự nhiên thành hành động robot có kiểm chứng, an toàn, không cho LLM bypass các lớp bảo vệ?"*
- 3 câu hỏi nghiên cứu con (3 bullet nhỏ)

**Ghi chú:** Slide quan trọng — hội đồng hay hỏi "vấn đề nghiên cứu là gì". Đọc rõ câu trung tâm.

### Slide 7 — Mục tiêu & đóng góp chính

**Mục đích:** Cam kết kết quả.
**Nội dung chính:**
- Bảng 2 cột: "Mục tiêu" | "Đóng góp"
- 6 hàng ngắn gọn
- Có thể thêm 1 icon ở góc

**Ghi chú:** Không đọc từng hàng. Nói tổng: "Đồ án tập trung vào 6 đóng góp, đã thực hiện được bao nhiêu, sẽ nói rõ ở phần đánh giá."

---

## PHẦN 2 — CƠ SỞ LÝ THUYẾT (Slide 8–10, ~3 phút)

### Slide 8 — GP4 & YRC1000micro

**Mục đích:** Cung cấp thông tin phần cứng.
**Nội dung chính:**
- 1 ảnh GP4 (1 bên)
- 1 bảng nhỏ thông số: 6 DOF, 4 kg, ±0.01 mm, IP67, ±170°/…
- 1 dòng ghi chú về YRC1000micro (PLd, micro-ROS)

**Ghi chú:** "Em không đi sâu vào datasheet, chỉ nhấn 3 điểm: 6 khớp quay, sai số lặp lại 0.01mm, controller hỗ trợ micro-ROS — đây là điều kiện tiên quyết để ROS 2 điều khiển được."

### Slide 9 — ROS 2 / MoveIt 2 / MotoROS2

**Mục đích:** Trình bày ngắn hệ sinh thái phần mềm.
**Nội dung chính:**
- Sơ đồ 3 ô ngang: **ROS 2** (middleware) → **MoveIt 2** (planning) → **MotoROS2** (driver)
- 1 bullet cho mỗi ô
- Không cần đi sâu chi tiết

**Ghi chú:** "Phần này đã có trong chương 2 của báo cáo, em chỉ tóm tắt."

### Slide 10 — Động học & IK

**Mục đích:** Đủ kiến thức nền để hội đồng không hỏi "có hiểu IK không".
**Nội dung chính:**
- 1 công thức FK dạng text: T = Π T_i(θ_i)
- 1 ảnh / sơ đồ trục S-L-U-R-B-T
- 1 dòng về TRAC-IK (Newton + SQP song song)
- 1 dòng về kỳ dị cổ tay

**Ghi chú:** Nói nhanh. Slide này mang tính "đệm" để trả lời câu hỏi chuyên ngành.

---

## PHẦN 3 — THIẾT KẾ KIẾN TRÚC (Slide 11–14, ~5 phút)

### Slide 11 — Nguyên tắc cốt lõi

**Mục đích:** Đặt tiền đề tư duy thiết kế.
**Nội dung chính:**
- 1 dòng slogan lớn: **"LLM không điều khiển robot — LLM chỉ sinh biểu diễn trung gian"**
- 4 icon / bullet nhỏ bên dưới:
  1. Tách ý định khỏi thực thi
  2. Tách grounding khỏi execution
  3. Fail-closed toàn pipeline
  4. Closed-loop observe → act → verify

**Ghi chú:** Slide quan trọng nhất phần 3. Dừng lại 15–20 giây, cho hội đồng ghi nhận nguyên tắc.

### Slide 12 — Pipeline 6 lớp

**Mục đích:** Tổng quan kiến trúc.
**Nội dung chính:**
- Sơ đồ khối 6 ô xếp dọc (hoặc 2 cột × 3 hàng):
  1. HMI (React + FastAPI)
  2. llm_gateway (2-tier parser + FactoryTask)
  3. safety (fail-closed gate)
  4. motion_core (MoveIt)
  5. hw_adapter (point budget + state)
  6. MotoROS2 → robot
- Mũi tên dữ liệu giữa các ô

**Ghi chú:** "Mỗi ô chỉ làm đúng một việc. Nếu muốn sửa LLM, không phải sửa MoveIt. Nếu muốn đổi camera, không phải sửa planning."

### Slide 13 — Luồng xử lý lệnh (sequence)

**Mục đích:** Cho thấy end-to-end flow.
**Nội dung chính:**
- Sơ đồ tuần tự 5 bước (actor + arrow):
  1. Operator gõ NL → HMI
  2. HMI → llm_gateway (`/review_intent`) → 2-tier parser
  3. llm_gateway → operator (plan tree + hint)
  4. Operator confirm → task_runtime
  5. task_runtime → safety → motion_core → hw_adapter → robot
  6. Runtime events → HMI System Log

**Ghi chú:** "Quan trọng: confirm một lần duy nhất. Sau khi confirm, runtime chạy tự động. STOP lúc nào cũng dừng ngay, fail-closed."

### Slide 14 — 16 Primitive & FactoryTask

**Mục đích:** Chi tiết "ngôn ngữ trung gian" giữa LLM và robot.
**Nội dung chính:**
- Bảng 16 primitive gọn (2 cột × 8 hàng): HOME, PTP, LIN, CIRC, CARTESIAN_PATH, MOVE_REL, GET_POSE, SET_SPEED, WAIT, STOP, MOVE_JOINT, IO_SET, ALARM_RESET, MOVE_JOINTS, BLENDED_SEQUENCE, MACRO
- 1 ví dụ FactoryTask JSON 6–8 dòng (cho "gắp vật đỏ"): sequence → for_each → pick → verify

**Ghi chú:** "Primitive là từ vựng robot. FactoryTask là từ vựng tác vụ. LLM chỉ cần học 16 từ này."

---

## PHẦN 4 — TRIỂN KHAI (Slide 15–17, ~3 phút)

### Slide 15 — Cấu trúc package

**Mục đích:** Cho thấy đồ án đã viết code thật, không phải lý thuyết.
**Nội dung chính:**
- Sơ đồ cây thư mục `src/` rút gọn (depth 2)
- Highlight 3 package chính: `llm_gateway/`, `safety/`, `motion_core/`
- 1 ảnh chụp terminal `colcon build` thành công (nếu có)

**Ghi chú:** "Đồ án có 9 package, mỗi package đảm nhận một lớp trong pipeline. Tổng cộng hàng chục nghìn dòng code C++ và Python."

### Slide 16 — 2-tier parser

**Mục đích:** Phân tích chi tiết đóng góp kỹ thuật chính.
**Nội dung chính:**
- Sơ đồ 2 nhánh:
  - **Tầng 1 (deterministic):** regex → 5 lệnh an toàn (stop, home, get pose, alarm reset, wait)
  - **Tầng 2 (LLM):** prompt + JSON schema → FactoryTask
- 1 dòng so sánh "trước" vs "sau":
  - Trước: 5 đường parse song song → kết quả không nhất quán
  - Sau: 2 tầng tách bạch → 1 đường duy nhất

**Ghi chú:** Slide kỹ thuật quan trọng — hội đồng có thể hỏi "tại sao 2 tầng mà không 1 tầng?".

### Slide 17 — HMI

**Mục đích:** Minh họa sản phẩm chạy được.
**Nội dung chính:**
- 1 ảnh chụp màn hình HMI (full screen)
- 3 chú thích nhỏ: ô nhập lệnh, cây task, System Log
- (Nếu có) 1 ảnh thứ hai: System Log với 7 category filter

**Ghi chú:** "HMI được viết bằng React 18 + FastAPI. Operator chỉ cần gõ câu lệnh, hệ thống lo phần còn lại."

---

## PHẦN 5 — ĐÁNH GIÁ L1–L5 (Slide 18–22, ~5 phút)

### Slide 18 — Phương pháp đánh giá tổng thể

**Mục đích:** Giới thiệu tháp đánh giá.
**Nội dung chính:**
- Hình tháp 5 tầng L1–L5 (y như hình bạn đã gửi)
- 2 mũi tên bên: "tăng độ tích hợp" / "tăng độ cô lập test"

**Ghi chú:** Đây là slide "đắt" nhất phần 5. Dừng 30–45 giây, giải thích vì sao đánh giá phân tầng thay vì end-to-end đơn lẻ.

### Slide 19 — L1 IR Generation

**Mục đích:** Chi tiết tầng 1.
**Nội dung chính:**
- Bảng 3 chỉ số: A_IR (intent accuracy), A_schema (schema valid), H_rate (hallucination)
- 1 đồ thị bar nhỏ: 3 phiên bản prompt → A_IR tăng
- Ngưỡng: A_IR ≥ 0.90, A_schema ≥ 0.95

**Ghi chú:** "Tầng 1 đo khả năng LLM sinh biểu diễn trung gian đúng. Bộ test ~120 câu, kết quả đạt X%."

### Slide 20 — L2–L3 Safety & Motion

**Mục đích:** Gộp 2 tầng vì cùng đặc tính.
**Nội dung chính:**
- **L2 Safety:** FPR, FNR, coverage (bảng 3 số)
  - "Bộ test 200 lệnh, false-reject 4%, false-accept 0.5%"
- **L3 Motion:** P_success, IK_fail, T_smooth (bảng 3 số)
  - "16 primitive × 10 pose, plan success Y%, IK fail Z%"

**Ghi chú:** Gộp 2 slide thành 1 để tiết kiệm thời gian.

### Slide 21 — L4–L5 Pipeline E2E & Vision

**Mục đích:** Hai tầng cao nhất.
**Nội dung chính:**
- **L4 E2E:** S_e2e, L_p50, L_p95, abort_rate
  - "10 kịch bản tổng hợp, thành công A%, latency p95 = B giây"
- **L5 Vision:** mAP, IoU, δ_calib, R̂_T
  - "Bộ mẫu 20 vật, detection accuracy C%, sai số calibration D mm"

**Ghi chú:** Slide quan trọng vì L4 là thước đo cuối cùng. L5 mới thêm vào, có thể hội đồng hỏi kỹ.

### Slide 22 — Bảng tổng hợp L1–L5

**Mục đích:** Snapshot 1 slide.
**Nội dung chính:**
- Bảng 5 hàng × 4 cột: Tầng | Mục tiêu | Kết quả | Đạt/Không đạt
- Icon ✓ / ✗ cho mỗi ô

**Ghi chú:** Đây là slide "recap" — hội đồng thường chụp ảnh slide này.

---

## PHẦN 6 — DEMO (Slide 23, ~2 phút) [TÙY CHỌN]

### Slide 23 — Video demo

**Mục đích:** Minh họa chạy thật.
**Nội dung chính:**
- 1 video ngắn (60–90 giây) nhúng: gõ lệnh "về home" → robot chạy
- Hoặc: video "gắp vật đỏ" pick-and-place
- Có thể là video sim (nếu không có hw) — nói rõ điều kiện

**Ghi chú:** "Em xin phép chạy video thực nghiệm. Video thứ nhất trong simulation, video thứ hai trên robot thật nếu hội đồng cho phép."

---

## PHẦN 7 — PHÂN TÍCH, HẠN CHẾ (Slide 24, ~2 phút)

### Slide 24 — Lỗi thường gặp & hạn chế

**Mục đích:** Trung thực, không giấu giếm.
**Nội dung chính:**
- Bảng 4 nhóm lỗi (LLM, perception, planning, hardware) × 2 cột: nguyên nhân + cách xử lý
- 1 dòng: "Hệ thống KHÔNG thay thế safety controller YRC1000micro"
- 1 dòng: "Chưa đạt chứng nhận ISO 10218"

**Ghi chú:** Slide quan trọng để thể hiện sự trưởng thành của nghiên cứu.

---

## PHẦN 8 — KẾT LUẬN & HƯỚNG PHÁT TRIỂN (Slide 25–26, ~2 phút)

### Slide 25 — Kết luận

**Mục đích:** Tổng kết 1 slide.
**Nội dung chính:**
- 3 bullet:
  1. Đã xây dựng pipeline 6 lớp hoạt động được
  2. Đã đánh giá theo 5 tầng L1–L5
  3. Đã chứng minh tỷ lệ thành công ổn định trong mô phỏng
- 1 dòng: "Mã nguồn: gp4_ws (branch upgrade-react-8626)"

**Ghi chú:** Nói tự nhiên, không đọc slide.

### Slide 26 — Hướng phát triển & Cảm ơn

**Mục đích:** Kết thúc + mở cửa hỏi đáp.
**Nội dung chính:**
- 3 hướng phát triển: mở rộng primitive, multi-camera, tối ưu latency
- Lời cảm ơn: "Em xin cảm ơn Hội đồng đã lắng nghe. Em sẵn sàng trả lời câu hỏi."

**Ghi chú:** Đứng yên, đợi câu hỏi.

---

## PHỤ LỤC — CHECKLIST TRƯỚC NGÀY BẢO VỆ

### Nội dung slide

- [ ] Mỗi slide ≤ 5 dòng chữ
- [ ] Sơ đồ / hình chiếm ≥ 50% diện tích slide
- [ ] Không có bullet dài quá 2 dòng
- [ ] Số liệu khớp với Ch 5 trong báo cáo
- [ ] Không có "TODO" / "[tbd]" / lỗi chính tả
- [ ] Slide đầu & slide cuối có tên SV, GVHD, ngày tháng

### Hình ảnh

- [ ] Ảnh trạm GP4 thật (không phải ảnh catalog mờ)
- [ ] Ảnh HMI thật từ màn hình chạy được
- [ ] Sơ đồ pipeline 6 lớp vẽ lại, không copy từ spec raw
- [ ] Tháp L1–L5 vẽ lại tiếng Việt

### Thuyết trình

- [ ] Tập nói 1–2 lần, đo thời gian (15–20 phút)
- [ ] Chuẩn bị 5 câu hỏi hội đồng hay hỏi + câu trả lời ngắn
- [ ] Mang theo báo cáo in để đối chiếu
- [ ] Mang theo bản photo bìa + trang đề tài (khi cần)

### Câu hỏi hội đồng thường gặp (câu trả lời ngắn)

1. **"Tại sao dùng 2-tier parser mà không 1 LLM?"** — Giảm hallucination cho lệnh thường gặp, latency thấp hơn, có thể kiểm thử độc lập.
2. **"Có khác gì với ReAct?"** — Bỏ multi-turn reasoning ở runtime; LLM chỉ sinh plan 1 lần, runtime thực thi deterministic.
3. **"Safety có đủ chứng nhận?"** — Không. Hệ thống chỉ là research/thesis, không thay thế safety controller Yaskawa.
4. **"Fine-tune LLM chưa?"** — Chưa. Đồ án dùng prompt engineering, để dành cho hướng phát triển tiếp theo.
5. **"Sai số bao nhiêu trên robot thật?"** — Trình bày kết quả calibration + test thật (nếu có); nếu chưa chạy hw thì nói rõ điều kiện.

---

## TÓM TẮT CẤU TRÚC

| Phần | Slide | Thời gian | Mục tiêu truyền thông |
|---|---|---|---|
| 0. Mở đầu | 1–3 | 2' | Giới thiệu, tạo ấn tượng đầu |
| 1. Bối cảnh & mục tiêu | 4–7 | 4' | Đặt vấn đề, cam kết đóng góp |
| 2. Cơ sở lý thuyết | 8–10 | 3' | Đệm kiến thức, trả lời câu hỏi chuyên ngành |
| 3. Kiến trúc | 11–14 | 5' | Trình bày giải pháp kỹ thuật |
| 4. Triển khai | 15–17 | 3' | Chứng minh đã code, có sản phẩm chạy |
| 5. Đánh giá L1–L5 | 18–22 | 5' | Trình bày kết quả định lượng |
| 6. Demo | 23 | 2' | Minh họa trực quan (tùy chọn) |
| 7. Hạn chế | 24 | 2' | Trung thực khoa học |
| 8. Kết luận | 25–26 | 2' | Kết thúc, mở Q&A |
| **Tổng** | **26 slide** | **~20–25 phút** | |
