#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import csv
import numpy as np
import time
import os
from datetime import datetime

class AutoRecorder(Node):
    def __init__(self):
        super().__init__('auto_joint_recorder')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            100)
        
        # Trạng thái máy
        self.is_recording = False
        self.idle_start_time = None
        self.vel_threshold = 0.005 # Ngưỡng vận tốc để nhận diện chuyển động (rad/s)
        self.stop_debounce_time = 0.5 # Thời gian chờ sau khi dừng để đóng file (giây)
        self.joint_idx = 0 # Mặc định là khớp 0 (sẽ tự tìm khớp J1)
        
        # Thư mục lưu dữ liệu
        self.output_dir = 'joint_records'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.reset_data()
        self.get_logger().info('🚀 Tool Auto Recorder Đã Khởi Động!')
        self.get_logger().info('👉 Chỉ việc cho robot chạy, tool sẽ tự động ghi dữ liệu và vẽ đồ thị khi robot dừng.')

    def reset_data(self):
        self.data = {
            'time': [],
            'pos': [],
            'vel': []
        }
        self.start_time = None

    def listener_callback(self, msg):
        try:
            vels = msg.velocity
            if len(vels) == 0:
                return # Bỏ qua nếu topic không chứa vận tốc
                
            max_vel = max([abs(v) for v in vels])
            is_moving = max_vel > self.vel_threshold
            
            # Tự động map khớp J1 (khớp đầu tiên)
            if 'joint_1_s' in msg.name:
                self.joint_idx = msg.name.index('joint_1_s')
            elif 'group_1/joint_1' in msg.name:
                self.joint_idx = msg.name.index('group_1/joint_1')
            
            # Bắt đầu di chuyển -> Ghi dữ liệu
            if not self.is_recording and is_moving:
                self.is_recording = True
                self.reset_data()
                self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.get_logger().info('🟢 Robot bắt đầu chạy! Đang ghi dữ liệu...')
                
            # Đang trong quá trình di chuyển
            if self.is_recording:
                current_t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.start_time
                self.data['time'].append(current_t)
                self.data['pos'].append(msg.position[self.joint_idx])
                self.data['vel'].append(msg.velocity[self.joint_idx])
                
                # Cảm nhận robot đã dừng
                if not is_moving:
                    if self.idle_start_time is None:
                        self.idle_start_time = time.time()
                    # Nếu dừng đủ lâu (debounce) -> Lưu file và vẽ hình
                    elif time.time() - self.idle_start_time > self.stop_debounce_time:
                        self.get_logger().info('🛑 Robot đã dừng. Đang xuất file và vẽ đồ thị...')
                        self.is_recording = False
                        self.idle_start_time = None
                        self.process_and_save()
                else:
                    self.idle_start_time = None # Reset debounce nếu nhích tiếp
                    
        except Exception as e:
            self.get_logger().error(f'Lỗi: {e}')
            
    def process_and_save(self):
        if len(self.data['time']) < 15:
            self.get_logger().info('⚠️ Chuyển động quá ngắn, bỏ qua.')
            return
            
        timestamp = datetime.now().strftime('%H%M%S')
        
        # Tính gia tốc bằng đạo hàm (do joint_states thường không có trường acceleration)
        acc = np.gradient(self.data['vel'], self.data['time'])
        
        # Lọc nhiễu nhẹ cho gia tốc (tuỳ chọn) bằng numpy
        window_acc = 5
        acc_smooth = np.convolve(acc, np.ones(window_acc)/window_acc, mode='same')
        
        # Tính Jerk (đạo hàm của gia tốc)
        jerk = np.gradient(acc_smooth, self.data['time'])
        
        # Lọc nhiễu cho Jerk để đồ thị mượt mà dễ so sánh hơn
        window_jerk = 7
        jerk_smooth = np.convolve(jerk, np.ones(window_jerk)/window_jerk, mode='same')
        
        # Tạo dictionary chứa dữ liệu để truyền vào plot và ghi csv
        df = {
            'time': self.data['time'],
            'pos': self.data['pos'],
            'vel': self.data['vel'],
            'acc': acc,
            'acc_smooth': acc_smooth,
            'jerk': jerk,
            'jerk_smooth': jerk_smooth
        }
        
        # 1. Lưu ra CSV
        csv_path = os.path.join(self.output_dir, f'record_{timestamp}.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'pos', 'vel', 'acc', 'acc_smooth', 'jerk', 'jerk_smooth'])
            for i in range(len(df['time'])):
                writer.writerow([df['time'][i], df['pos'][i], df['vel'][i], df['acc'][i], df['acc_smooth'][i], df['jerk'][i], df['jerk_smooth'][i]])
                
        self.get_logger().info(f'✅ Đã lưu file CSV: {csv_path}')
        
        # 2. Vẽ đồ thị tự động
        self.plot_data(df, timestamp)
        
    def plot_data(self, df, timestamp):
        plt.rcParams.update({'font.size': 12, 'font.family': 'serif'})
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # Vẽ đường gia tốc thô (nét mờ) và đã lọc nhiễu (nét đậm)
        ax1.plot(df['time'], df['acc'], color='gray', linewidth=1, alpha=0.5, label='Gia tốc thô (Đạo hàm)')
        ax1.plot(df['time'], df['acc_smooth'], color='#e74c3c', linewidth=2, label='Gia tốc (Smooth)')
        ax1.set_title(f'So sánh Gia tốc & Jerk Khớp J1 (Mã test: {timestamp})', fontsize=13, pad=15, fontweight='bold')
        ax1.set_ylabel('Gia tốc (rad/s²)')
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc='upper right')
        
        # Vẽ đường Jerk thô và đã lọc nhiễu
        ax2.plot(df['time'], df['jerk'], color='gray', linewidth=1, alpha=0.5, label='Jerk thô (Đạo hàm)')
        ax2.plot(df['time'], df['jerk_smooth'], color='#3498db', linewidth=2, label='Jerk (Smooth)')
        ax2.set_xlabel('Thời gian (s)')
        ax2.set_ylabel('Jerk (rad/s³)')
        ax2.grid(True, linestyle=':', alpha=0.7)
        ax2.legend(loc='upper right')
        
        plt.tight_layout()
        img_path = os.path.join(self.output_dir, f'plot_{timestamp}.png')
        plt.savefig(img_path, dpi=300)
        plt.close()
        
        self.get_logger().info(f'✅ Đã lưu Đồ thị:  {img_path}')
        self.get_logger().info('--------------------------------------------------')
        self.get_logger().info('⏳ Đang đợi lệnh chạy tiếp theo...\n')

def main(args=None):
    rclpy.init(args=args)
    node = AutoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Tắt Tool...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
