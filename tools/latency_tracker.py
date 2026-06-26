#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import time
import csv
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class LatencyTrackerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("System Latency Tracker - GP4 Thesis")
        self.root.geometry("1100x600")
        
        self.prompt_counter = 1
        self.csv_file = "latency_results.csv"
        self.image_dir = "latency_image"
        
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
            
        self.phases = ["LLM", "Xác thực an toàn", "Lập kế hoạch", "Thực thi"]
        self.current_phase_idx = -1
        self.start_time = 0
        self.latencies = []
        
        self.setup_ui()
        self.init_csv()

    def init_csv(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp", "Prompt", "LLM Latency (s)", "Safety Latency (s)", "Planning Latency (s)", "Execution Latency (s)", "Total Latency (s)"])

    def setup_ui(self):
        # PanedWindow to split left (controls+table) and right (plot)
        self.paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # LEFT FRAME
        left_frame = ttk.Frame(self.paned)
        self.paned.add(left_frame, weight=1)
        
        # Controls in Left Frame
        controls_frame = ttk.Frame(left_frame, padding="5")
        controls_frame.pack(fill=tk.X)
        
        self.lbl_status = ttk.Label(controls_frame, text=f"Sẵn sàng cho Lệnh {self.prompt_counter}", font=("Arial", 14, "bold"))
        self.lbl_status.pack(pady=5)
        
        style = ttk.Style()
        style.configure('Large.TButton', font=('Arial', 12, 'bold'))
        
        self.btn_main = ttk.Button(controls_frame, text="Bắt đầu: Hiểu LLM", command=self.handle_main_button, style='Large.TButton', width=40)
        self.btn_main.pack(pady=10, ipady=5)
        
        self.btn_reset = ttk.Button(controls_frame, text="Hủy đo lệnh ĐANG CHẠY (Cancel)", command=self.cancel_current)
        self.btn_reset.pack(pady=2)
        
        self.btn_undo = ttk.Button(controls_frame, text="Xóa kết quả lệnh VỪA LƯU (Undo)", command=self.undo_last)
        self.btn_undo.pack(pady=2)

        self.btn_reset_all = ttk.Button(controls_frame, text="Xóa toàn bộ dữ liệu (Reset All)", command=self.reset_all)
        self.btn_reset_all.pack(pady=2)
        
        # Table in Left Frame
        table_frame = ttk.Frame(left_frame, padding="5")
        table_frame.pack(fill=tk.BOTH, expand=True)
        
        columns = ("prompt", "llm", "safety", "plan", "exec", "total")
        self.tree = ttk.Treeview(table_frame, columns=columns, show='headings', height=10)
        self.tree.heading("prompt", text="Lệnh")
        self.tree.heading("llm", text="LLM")
        self.tree.heading("safety", text="An toàn")
        self.tree.heading("plan", text="Kế hoạch")
        self.tree.heading("exec", text="Thực thi")
        self.tree.heading("total", text="Tổng")
        
        self.tree.column("prompt", width=50, anchor=tk.CENTER)
        self.tree.column("llm", width=70, anchor=tk.CENTER)
        self.tree.column("safety", width=70, anchor=tk.CENTER)
        self.tree.column("plan", width=70, anchor=tk.CENTER)
        self.tree.column("exec", width=70, anchor=tk.CENTER)
        self.tree.column("total", width=70, anchor=tk.CENTER)
        
        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # RIGHT FRAME for Plot
        self.right_frame = ttk.Frame(self.paned)
        self.paned.add(self.right_frame, weight=2)
        
        self.fig, self.ax = plt.subplots(figsize=(6, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def handle_main_button(self):
        now = time.time()
        
        if self.current_phase_idx == -1:
            self.start_time = now
            self.current_phase_idx = 0
            self.btn_main.config(text="Dừng LLM -> Bắt đầu XÁC THỰC AN TOÀN")
            self.lbl_status.config(text=f"Lệnh {self.prompt_counter} - Đang đo: {self.phases[0]}...")
            
        elif self.current_phase_idx == 0:
            self.latencies.append(now - self.start_time)
            self.start_time = now
            self.current_phase_idx = 1
            self.btn_main.config(text="Dừng AN TOÀN -> Bắt đầu LẬP KẾ HOẠCH")
            self.lbl_status.config(text=f"Lệnh {self.prompt_counter} - Đang đo: {self.phases[1]}...")
            
        elif self.current_phase_idx == 1:
            self.latencies.append(now - self.start_time)
            self.start_time = now
            self.current_phase_idx = 2
            self.btn_main.config(text="Dừng KẾ HOẠCH -> Bắt đầu THỰC THI")
            self.lbl_status.config(text=f"Lệnh {self.prompt_counter} - Đang đo: {self.phases[2]}...")
            
        elif self.current_phase_idx == 2:
            self.latencies.append(now - self.start_time)
            self.start_time = now
            self.current_phase_idx = 3
            self.btn_main.config(text="Dừng THỰC THI & Lưu Lệnh")
            self.lbl_status.config(text=f"Lệnh {self.prompt_counter} - Đang đo: {self.phases[3]}...")
            
        elif self.current_phase_idx == 3:
            self.latencies.append(now - self.start_time)
            self.save_data()
            self.reset_for_next()

    def save_data(self):
        total_time = sum(self.latencies)
        l = [f"{x:.2f}" for x in self.latencies]
        t = f"{total_time:.2f}"
        
        self.tree.insert('', tk.END, values=(self.prompt_counter, l[0], l[1], l[2], l[3], t))
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.csv_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, self.prompt_counter, l[0], l[1], l[2], l[3], t])
            
        self.prompt_counter += 1
        
        # Tự động vẽ lại biểu đồ
        self.plot_data()

    def reset_for_next(self):
        self.current_phase_idx = -1
        self.latencies = []
        self.btn_main.config(text=f"Bắt đầu: Hiểu LLM (Lệnh {self.prompt_counter})")
        self.lbl_status.config(text=f"Sẵn sàng cho Lệnh {self.prompt_counter}")

    def cancel_current(self):
        if self.current_phase_idx != -1:
            if messagebox.askyesno("Hủy", "Bạn có chắc chắn muốn hủy bỏ dữ liệu đang đo của lệnh hiện tại không?"):
                self.current_phase_idx = -1
                self.latencies = []
                self.btn_main.config(text=f"Bắt đầu: Hiểu LLM (Lệnh {self.prompt_counter})")
                self.lbl_status.config(text=f"Sẵn sàng cho Lệnh {self.prompt_counter}")
        else:
            messagebox.showinfo("Thông báo", "Không có tiến trình nào đang chạy để hủy.")

    def undo_last(self):
        if self.prompt_counter <= 1:
            messagebox.showinfo("Thông báo", "Chưa có lệnh nào được lưu để xóa.")
            return
            
        if messagebox.askyesno("Xóa lệnh vừa lưu", f"Bạn có chắc chắn muốn XÓA kết quả của Lệnh {self.prompt_counter - 1} không?"):
            try:
                with open(self.csv_file, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                with open(self.csv_file, 'w', encoding='utf-8') as f:
                    f.writelines(lines[:-1])
            except Exception as e:
                messagebox.showerror("Lỗi", f"Không thể cập nhật CSV: {e}")
                return
                
            children = self.tree.get_children()
            if children:
                self.tree.delete(children[-1])
                
            self.prompt_counter -= 1
            self.cancel_current()
            self.current_phase_idx = -1
            self.btn_main.config(text=f"Bắt đầu: Hiểu LLM (Lệnh {self.prompt_counter})")
            self.lbl_status.config(text=f"Sẵn sàng cho Lệnh {self.prompt_counter}")
            
            # Tự động vẽ lại biểu đồ
            self.plot_data()

    def reset_all(self):
        if messagebox.askyesno("Xóa TOÀN BỘ", "CẢNH BÁO: Xóa toàn bộ dữ liệu CSV và làm lại từ đầu (Lệnh 1)?"):
            if os.path.exists(self.csv_file):
                os.remove(self.csv_file)
            self.init_csv()
            
            for item in self.tree.get_children():
                self.tree.delete(item)
                
            self.prompt_counter = 1
            self.current_phase_idx = -1
            self.latencies = []
            self.btn_main.config(text=f"Bắt đầu: Hiểu LLM (Lệnh {self.prompt_counter})")
            self.lbl_status.config(text=f"Sẵn sàng cho Lệnh {self.prompt_counter}")
            
            # Tự động vẽ lại biểu đồ trống
            self.plot_data()

    def load_existing_data(self):
        if not os.path.exists(self.csv_file):
            return
        
        # Delete existing tree rows
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        try:
            with open(self.csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                max_prompt = 0
                for row in reader:
                    p = int(row['Prompt'])
                    if p > max_prompt:
                        max_prompt = p
                    self.tree.insert('', tk.END, values=(p, row['LLM Latency (s)'], row['Safety Latency (s)'], row['Planning Latency (s)'], row['Execution Latency (s)'], row['Total Latency (s)']))
                self.prompt_counter = max_prompt + 1
        except Exception as e:
            print(f"Error loading CSV: {e}")
            
    def plot_data(self):
        self.ax.clear()
        
        if not os.path.exists(self.csv_file):
            self.canvas.draw()
            return
            
        prompts = []
        llm_data = []
        safety_data = []
        plan_data = []
        exec_data = []
        
        try:
            with open(self.csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    prompts.append(f"Lệnh {row['Prompt']}")
                    llm_data.append(float(row['LLM Latency (s)']))
                    safety_data.append(float(row['Safety Latency (s)']))
                    plan_data.append(float(row['Planning Latency (s)']))
                    exec_data.append(float(row['Execution Latency (s)']))
        except Exception as e:
            print(f"Error parsing CSV for plot: {e}")
            return
            
        if not prompts:
            self.canvas.draw()
            return

        x = np.arange(len(prompts))
        width = 0.5
        
        self.ax.bar(x, llm_data, width, label='Hiểu LLM')
        self.ax.bar(x, safety_data, width, bottom=llm_data, label='An toàn')
        self.ax.bar(x, plan_data, width, bottom=np.array(llm_data)+np.array(safety_data), label='Lập kế hoạch')
        self.ax.bar(x, exec_data, width, bottom=np.array(llm_data)+np.array(safety_data)+np.array(plan_data), label='Thực thi')
        
        self.ax.set_ylabel('Thời gian trễ (giây)')
        self.ax.set_title('Biểu đồ độ trễ qua các giai đoạn (System Latency)')
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(prompts, rotation=45, ha='right')
        self.ax.legend()
        
        self.fig.tight_layout()
        self.canvas.draw()
        
        # Tự động lưu hình ảnh đồ thị ra file PNG
        try:
            # Lưu file mới nhất (luôn ghi đè)
            latest_path = os.path.join(self.image_dir, "latency_chart_latest.png")
            self.fig.savefig(latest_path, dpi=300, bbox_inches='tight')
            
            # Lưu file theo số lượng prompt để có tính lịch sử
            if prompts:
                history_path = os.path.join(self.image_dir, f"latency_chart_{len(prompts)}_prompts.png")
                self.fig.savefig(history_path, dpi=300, bbox_inches='tight')
        except Exception as e:
            print(f"Error saving plot image: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = LatencyTrackerApp(root)
    app.load_existing_data()
    app.reset_for_next()
    app.plot_data()
    root.mainloop()
