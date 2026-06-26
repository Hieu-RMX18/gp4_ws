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
        self.joint_indices = [] # Sẽ tự động tìm cả 6 khớp
        
        # Thư mục lưu dữ liệu
        self.output_dir = 'joint_records'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.reset_data()
        self.get_logger().info('🚀 Tool Auto Recorder Đã Khởi Động!')
        self.get_logger().info('👉 Chỉ việc cho robot chạy, tool sẽ tự động ghi dữ liệu và vẽ đồ thị khi robot dừng.')

    def reset_data(self):
        self.data = {
            'time': [],
            'pos': [[] for _ in range(6)],
            'vel': [[] for _ in range(6)]
        }
        self.start_time = None

    def listener_callback(self, msg):
        try:
            vels = msg.velocity
            if len(vels) == 0:
                return # Bỏ qua nếu topic không chứa vận tốc
                
            max_vel = max([abs(v) for v in vels])
            is_moving = max_vel > self.vel_threshold
            
            # Tự động map 6 khớp
            if len(self.joint_indices) == 0:
                if 'joint_1_s' in msg.name:
                    names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
                    self.joint_indices = [msg.name.index(n) for n in names if n in msg.name]
                elif 'group_1/joint_1' in msg.name:
                    names = [f'group_1/joint_{i}' for i in range(1, 7)]
                    self.joint_indices = [msg.name.index(n) for n in names if n in msg.name]
                else:
                    self.joint_indices = list(range(min(6, len(msg.name))))
            
            # Bắt đầu di chuyển -> Ghi dữ liệu
            if not self.is_recording and is_moving:
                self.is_recording = True
                self.reset_data()
                self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.get_logger().info('🟢 Robot bắt đầu chạy! Đang ghi dữ liệu 6 khớp...')
                
            # Đang trong quá trình di chuyển
            if self.is_recording:
                current_t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.start_time
                self.data['time'].append(current_t)
                for i, idx in enumerate(self.joint_indices):
                    if i < 6:
                        self.data['pos'][i].append(msg.position[idx])
                        self.data['vel'][i].append(msg.velocity[idx])
                
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
        num_joints = min(6, len(self.joint_indices))
        
        df = {
            'time': self.data['time'],
            'pos': [], 'vel': [], 'acc': [], 'acc_smooth': [], 'jerk': [], 'jerk_smooth': []
        }
        
        window_acc = 5
        window_jerk = 7

        for i in range(num_joints):
            vel = self.data['vel'][i]
            pos = self.data['pos'][i]
            
            # Tính gia tốc bằng đạo hàm
            acc = np.gradient(vel, self.data['time'])
            # Lọc nhiễu nhẹ cho gia tốc bằng numpy
            acc_smooth = np.convolve(acc, np.ones(window_acc)/window_acc, mode='same')
            
            # Tính Jerk (đạo hàm của gia tốc)
            jerk = np.gradient(acc_smooth, self.data['time'])
            # Lọc nhiễu cho Jerk
            jerk_smooth = np.convolve(jerk, np.ones(window_jerk)/window_jerk, mode='same')
            
            df['pos'].append(pos)
            df['vel'].append(vel)
            df['acc'].append(acc)
            df['acc_smooth'].append(acc_smooth)
            df['jerk'].append(jerk)
            df['jerk_smooth'].append(jerk_smooth)
        
        # 1. Lưu ra CSV
        csv_path = os.path.join(self.output_dir, f'record_{timestamp}.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            headers = ['time']
            for i in range(num_joints):
                headers.extend([f'pos_J{i+1}', f'vel_J{i+1}', f'acc_J{i+1}', f'acc_smooth_J{i+1}', f'jerk_J{i+1}', f'jerk_smooth_J{i+1}'])
            writer.writerow(headers)
            
            for k in range(len(df['time'])):
                row = [df['time'][k]]
                for i in range(num_joints):
                    row.extend([
                        df['pos'][i][k], df['vel'][i][k], df['acc'][i][k], 
                        df['acc_smooth'][i][k], df['jerk'][i][k], df['jerk_smooth'][i][k]
                    ])
                writer.writerow(row)
                
        self.get_logger().info(f'✅ Đã lưu file CSV: {csv_path}')
        
        # 2. Vẽ đồ thị tự động
        self.plot_data(df, timestamp, num_joints)
        
    def plot_data(self, df, timestamp, num_joints):
        plt.rcParams.update({'font.size': 12, 'font.family': 'serif'})
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        for i in range(num_joints):
            c = colors[i % len(colors)]
            ax1.plot(df['time'], df['acc_smooth'][i], color=c, linewidth=2, label=f'J{i+1}')
            ax2.plot(df['time'], df['jerk_smooth'][i], color=c, linewidth=2, label=f'J{i+1}')
            
        ax1.set_title(f'So sánh Gia tốc 6 Khớp (Mã test: {timestamp})', fontsize=13, pad=15, fontweight='bold')
        ax1.set_ylabel('Gia tốc (rad/s²)')
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc='upper right', ncol=3)
        
        ax2.set_title(f'So sánh Jerk 6 Khớp', fontsize=13, pad=15, fontweight='bold')
        ax2.set_xlabel('Thời gian (s)')
        ax2.set_ylabel('Jerk (rad/s³)')
        ax2.grid(True, linestyle=':', alpha=0.7)
        ax2.legend(loc='upper right', ncol=3)
        
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
