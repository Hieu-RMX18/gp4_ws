# Hướng dẫn Hoàn thiện Cấu hình Phần Cứng (Hardware Configuration Guide)

Tài liệu này hướng dẫn cách đo đạc, kiểm tra và cấu hình các giá trị thực tế trên robot thật (Yaskawa GP4) để thay thế các cờ `VERIFY_CONFIG`. Do hệ thống được thiết kế theo nguyên tắc **Fail-Closed**, nếu các bước dưới đây chưa được thực hiện, robot sẽ từ chối chạy nghiệm thu và báo lỗi `verify_config_required`.

---

## 1. Đo đạc Tọa độ Trạm (Station Semantic Map)

**File cần sửa:** `src/llm_gateway/config/station_semantic_map.yaml`

Hiện tại, tọa độ tâm (`center`) và kích thước (`size`) của `conveyor` (băng tải) và `fixture` (gá phôi) đã được điền theo số đo hiện trường. Dùng quy trình dưới đây để kiểm tra lại hoặc cập nhật khi bố trí trạm thay đổi.

### `size` và `center` là gì?
- `size`: Là kích thước chiều dài (X), chiều rộng (Y) và chiều cao/chiều sâu (Z) của khối hộp (bounding box) bao quanh trạm làm việc.
- `center`: Là **tọa độ tâm hình học (điểm chính giữa) của khối hộp đó**, được so với gốc tọa độ `base_link` của robot.

### Cách lấy tọa độ `center` chính xác nhất (Tránh sai lệch hệ tọa độ):
> [!IMPORTANT]
> **LUÔN ĐỌC TỌA ĐỘ TỪ ROS (HMI HOẶC CLI), KHÔNG DÙNG TEACHING PENDANT!**
> Do gốc tọa độ Base của tủ Yaskawa có thể khác với `base_link` mà bạn đã config trong URDF (ROS), việc lấy số từ Pendant đập vào ROS có thể khiến robot đâm sai vị trí.

**Quy trình lấy số:**
1. **Đo `size`:** Dùng thước đo đạc chiều dài (X), chiều rộng (Y), và chiều sâu (Z) của khu vực băng tải / gá phôi (tính bằng mét).
2. **Tìm tọa độ mặt trên:**
   - Dùng tay gạt (Teach Pendant) di chuyển robot sao cho **mũi gắp (TCP) chạm nhẹ vào mặt băng tải** (ngay điểm bạn muốn làm tâm mặt phẳng).
   - Bỏ Pendant xuống, mở terminal của PC (đã chạy mạng ROS) và gọi service đọc pose hiện tại:
     ```bash
     ros2 service call /get_current_pose interfaces/srv/GetCurrentPose "{reference_frame: base_link}"
     ```
     *(Hoặc đơn giản là nhìn thông số Pose đang hiển thị trên giao diện màn hình HMI của ROS).*
   - Ghi lại tọa độ `[x, y, z]` mà ROS vừa in ra.
3. **Tính `center`:**
   - Vì mũi gắp đang chạm vào *mặt trên cùng* của khối băng tải, trong khi `center` đòi hỏi *tâm của khối hộp*.
   - Bạn cần trừ tọa độ Z lấy được đi một nửa chiều cao (`size.z / 2`).
   - *Ví dụ:* Nếu ROS báo mặt băng tải là `Z = 0.250`, và đo tay thấy băng tải dày `0.100` (`size.z = 0.100`), thì tọa độ tâm của `center.z` sẽ là: `0.250 - (0.100 / 2) = 0.200`.
4. **Cập nhật file YAML:**
   - Điền các giá trị vừa tính vào dòng `center: {x: ..., y: ..., z: ...}`.
   - Khi hoàn tất, chỉ giữ `geometry_verified: true` nếu toàn bộ tọa độ và kích thước đã được kiểm tra trong hệ tọa độ ROS `base_link`.

---

## 2. Cấu hình I/O cho Tay Gắp (Gripper)

**File cần sửa:** `src/safety/config/safety_rules.yaml` (phần `gripper`)

Hệ thống MotoROS2 giao tiếp với tay gắp qua các chân tín hiệu (I/O) trên tủ điện YRC1000micro. Các địa chỉ I/O này phải khớp chính xác với đấu nối thực tế.

### Cách tìm và cấu hình I/O:
1. **Xác định chân ra (Output - Để điều khiển van khí/cơ):**
   - Tra cứu sơ đồ đấu nối tủ điện hoặc mở màn hình I/O trên Pendant.
   - Tìm số địa chỉ I/O (ví dụ: `10010` hoặc `10011`) điều khiển trạng thái MỞ (Open) và ĐÓNG (Close) của kẹp.
2. **Xác định chân vào (Input - Feedback):**
   - Tìm cảm biến báo trạng thái đã gắp được vật (nếu có, ví dụ cảm biến từ trên xilanh). Lấy địa chỉ I/O của nó.
3. **Cập nhật file YAML:**
   Thay thế các `VERIFY_CONFIG` thành số thực:
   ```yaml
   gripper:
     open_output_address: 10010       # Địa chỉ kích mở kẹp
     open_output_value: 1             # 1 = Bật, 0 = Tắt
     close_output_address: 10010      # Hoặc địa chỉ khác nếu van kép
     close_output_value: 0            # (Tùy cấu hình van)
     closed_input_address: 20010      # Địa chỉ đọc cảm biến feedback
     closed_input_active_value: 1     # Giá trị mong đợi khi gắp chặt
   ```

---

## 3. Khai báo Offset Công Cụ (TCP Offset)

**ĐÃ HOÀN THÀNH:** `src/gp4_moveit_config/config/motoman_gp4.urdf.xacro` và `motoman_gp4.srdf`

Khoảng cách từ mặt bích của robot (`joint_6`/`tool0`) đến mũi kẹp (TCP tool) đã được chuẩn hóa là **11cm (0.11m)**. 

Hệ thống đã loại bỏ việc tính toán bù trừ trên code (`tcp_offset_m`) và cấu hình thẳng một Frame cho đầu gắp trong hệ thống ROS:
1. Đã thêm `tcp_link` nối cứng vào `tool0` với khoảng dịch `z = 0.11m`.
2. Planning Group (MoveIt) đã cấu hình dùng `tcp_link` thay vì `tool0` làm end-effector chuẩn.

> [!TIP]
> Việc cấu hình `tcp_link` dài 11cm trong URDF giúp Rviz hiển thị chính xác và thuật toán nội bộ của MoveIt tự động tránh va chạm (collision) giữa tay gắp và các vật cản một cách mượt mà nhất.

---

## 4. Các Bước Kiểm Tra (Testing Flow)

Sau khi nhập thông số thực tế, hãy tiến hành bài test trên thiết bị thật (Hardware Test) bằng CLI công cụ như sau:

**Bước 1: Khởi động ROS với chế độ Hardware**
```bash
ros2 launch gp4_bringup hw.launch.py robot_ip:=<IP_ROBOT> agent_ip:=<IP_PC>
```

**Bước 2: Test I/O độc lập bằng CLI (Không chuyển động)**
```bash
# Thử mở kẹp
ros2 service call /yaskawa/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10010, value: 1}"

# Đọc feedback
ros2 service call /yaskawa/read_single_io motoros2_interfaces/srv/ReadSingleIO "{address: 20010}"
```

**Bước 3: Kiểm tra ReAct review path trước khi xác nhận chạy thật**
- Hãy bắt đầu bằng lệnh review-only. Service này chỉ sinh Semantic IR để kiểm tra, không dispatch motion:
```bash
ros2 service call /llm_gateway/review_intent interfaces/srv/ReviewIntent "{raw_text: 'pick the white workpiece and place it on the conveyor', runtime_mode: 'hardware', session_id: 'hardware-config-check', operator_id: 'operator', command_id: 'dry-run-review-001', review_token: 'manual-review'}"
```
- Theo dõi log trên Terminal và nội dung `semantic_ir_json`. Nếu có sai sót về Safety bounds hoặc tọa độ, hệ thống phải từ chối ở bước review. Chỉ dùng HMI supervisor confirm flow để xác nhận chạy motion thật.

> [!CAUTION]
> Luôn cầm sẵn nút Dừng Khẩn Cấp (E-STOP) trên tay khi chạy thử nghiệm lệnh Pick/Place đầu tiên có kết nối phần cứng. Hệ thống phần mềm đã được thiết kế fail-closed, nhưng trong lúc tinh chỉnh tọa độ vật lý lần đầu luôn có rủi ro sai số.
