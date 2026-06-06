import math
import asyncio
import sys
import subprocess
import json
import threading
import queue
import csv
import os
import time

# --- Détection et installation automatique des dépendances (pour BLE direct) ---
try:
    from bleak import BleakClient, BleakScanner
    import serial.tools.list_ports
except ImportError:
    print("[Python] Modules 'bleak' ou 'pyserial' manquants. Installation en cours...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "bleak", "pyserial"])
    from bleak import BleakClient, BleakScanner
    import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk, messagebox

class RobotRemoteGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("DrawRobot Remote Control (Python)")
        self.root.geometry("1100x750")
        
        # Configuration BLE (doit correspondre à BluetoothManager.h/cpp)
        self.device_name = "GiRobot_BLE"
        self.service_uuid = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
        self.char_uuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
        
        self.client = None
        self.is_connected = False
        self.cmd_queue = queue.Queue()
        self.telemetry = {}
        self.path_history = [(0.0, 0.0)]

        # Logging state
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None

        self.setup_ui()
        self.start_ble_thread()

    def setup_ui(self):
        style = ttk.Style()
        style.configure("TButton", padding=5)
        style.configure("Warning.TButton", foreground="red")

        left_panel = ttk.Frame(self.root)
        left_panel.pack(side="left", fill="both", expand=True, padx=5)

        right_panel = ttk.Frame(self.root)
        right_panel.pack(side="right", fill="both", expand=True, padx=5)

        status_frame = ttk.LabelFrame(right_panel, text="System Status", padding=10)
        status_frame.pack(fill="x", padx=10, pady=5)
        self.lbl_status = ttk.Label(status_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack()

        manual_frame = ttk.LabelFrame(left_panel, text="Manual Control (Hold to move)", padding=10)
        manual_frame.pack(fill="x", padx=10, pady=5)

        btn_fwd = ttk.Button(manual_frame, text="FORWARD")
        btn_fwd.grid(row=0, column=1, pady=2)
        btn_fwd.bind('<ButtonPress-1>', lambda e: self.send_cmd("FORWARD"))
        btn_fwd.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        btn_lft = ttk.Button(manual_frame, text="LEFT")
        btn_lft.grid(row=1, column=0, pady=2)
        btn_lft.bind('<ButtonPress-1>', lambda e: self.send_cmd("TURN_LEFT"))
        btn_lft.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        btn_stop = ttk.Button(manual_frame, text="STOP", style="Warning.TButton")
        btn_stop.grid(row=1, column=1, pady=2)
        btn_stop.configure(command=lambda: self.send_cmd("STOP"))

        btn_rgt = ttk.Button(manual_frame, text="RIGHT")
        btn_rgt.grid(row=1, column=2, pady=2)
        btn_rgt.bind('<ButtonPress-1>', lambda e: self.send_cmd("TURN_RIGHT"))
        btn_rgt.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        btn_bwd = ttk.Button(manual_frame, text="BACKWARD")
        btn_bwd.grid(row=2, column=1, pady=2)
        btn_bwd.bind('<ButtonPress-1>', lambda e: self.send_cmd("BACKWARD"))
        btn_bwd.bind('<ButtonRelease-1>', lambda e: self.send_cmd("STOP"))

        seq_frame = ttk.LabelFrame(left_panel, text="Drawing Sequences", padding=10)
        seq_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(seq_frame, text="Sequence 1: Escalier", command=self.gui_draw_stairs).pack(fill="x", pady=2)

        line_subframe = ttk.Frame(seq_frame)
        line_subframe.pack(fill="x", pady=2)
        self.ent_line_length = ttk.Entry(line_subframe, width=8)
        self.ent_line_length.insert(0, "1000")
        self.ent_line_length.pack(side="left", padx=2)
        ttk.Label(line_subframe, text="mm").pack(side="left")
        ttk.Button(line_subframe, text="Draw Line", command=self.gui_draw_line).pack(side="left", padx=2)
        
        co_subframe = ttk.Frame(seq_frame)
        co_subframe.pack(fill="x", pady=2)
        self.ent_corner_size = ttk.Entry(co_subframe, width=8)
        self.ent_corner_size.insert(0, "100")
        self.ent_corner_size.pack(side="left", padx=2)
        ttk.Label(co_subframe, text="mm").pack(side="left")
        ttk.Button(co_subframe, text="Draw Corner", command=self.gui_draw_corner).pack(side="left", padx=2)

        sq_subframe = ttk.Frame(seq_frame)
        sq_subframe.pack(fill="x", pady=2)
        self.ent_sq_size = ttk.Entry(sq_subframe, width=8)
        self.ent_sq_size.insert(0, "300")
        self.ent_sq_size.pack(side="left", padx=2)
        ttk.Label(sq_subframe, text="mm").pack(side="left")
        ttk.Button(sq_subframe, text="Draw Squares", command=self.gui_draw_squares).pack(side="left", padx=2)

        spiral_subframe = ttk.Frame(seq_frame)
        spiral_subframe.pack(fill="x", pady=2)
        self.ent_spiral_first = ttk.Entry(spiral_subframe, width=8)
        self.ent_spiral_first.insert(0, "220")
        self.ent_spiral_first.pack(side="left", padx=2)
        ttk.Label(spiral_subframe, text="first mm").pack(side="left")
        self.ent_spiral_count = ttk.Entry(spiral_subframe, width=5)
        self.ent_spiral_count.insert(0, "2")
        self.ent_spiral_count.pack(side="left", padx=2)
        ttk.Label(spiral_subframe, text="squares").pack(side="left")
        ttk.Button(spiral_subframe, text="Draw Square Spiral", command=self.gui_draw_square_spiral).pack(side="left", padx=2)

        ci_subframe = ttk.Frame(seq_frame)
        ci_subframe.pack(fill="x", pady=2)
        self.ent_radius = ttk.Entry(ci_subframe, width=8)
        self.ent_radius.insert(0, "130")
        self.ent_radius.pack(side="left", padx=2)
        ttk.Label(ci_subframe, text="mm").pack(side="left")
        ttk.Button(ci_subframe, text="Draw Circle", command=self.gui_draw_circle).pack(side="left", padx=2)
        ttk.Button(ci_subframe, text="Draw Rosace", command=self.gui_draw_rosace).pack(side="left", padx=2)

        ttk.Button(seq_frame, text="Sequence 3: North", command=lambda: self.send_cmd("north")).pack(fill="x", pady=2)
        
        special_frame = ttk.LabelFrame(left_panel, text="Special Operations", padding=10)
        special_frame.pack(fill="x", padx=10, pady=5)

        ttk.Button(special_frame, text="Calibrate Magnetometer", command=lambda: self.send_cmd("calibrateMag")).pack(fill="x", pady=2)
        ttk.Button(special_frame, text="📍 Reset Position (Set 0,0)", command=lambda: self.send_cmd("RESET_POSE")).pack(fill="x", pady=2)

        # Compass offset control (read/write persisted value)
        comp_frame = ttk.Frame(special_frame)
        comp_frame.pack(fill="x", pady=2)
        ttk.Label(comp_frame, text="Compass Offset (°):").pack(side="left")
        self.ent_compass_offset = ttk.Entry(comp_frame, width=8)
        self.ent_compass_offset.insert(0, "0.0")
        self.ent_compass_offset.pack(side="left", padx=2)
        ttk.Button(comp_frame, text="Set Offset", command=self.gui_set_compass_offset).pack(side="left", padx=2)
        ttk.Button(comp_frame, text="Get Offset", command=self.gui_get_compass_offset).pack(side="left", padx=2)
        self.lbl_compass_offset_current = ttk.Label(comp_frame, text="Current: --°")
        self.lbl_compass_offset_current.pack(side="left", padx=8)

        wifi_frame = ttk.Frame(special_frame)
        wifi_frame.pack(fill="x", pady=2)
        ttk.Button(wifi_frame, text="🌐 WiFi ON", command=lambda: self.send_cmd("wifi", {"state": True})).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(wifi_frame, text="📡 WiFi OFF", command=lambda: self.send_cmd("wifi", {"state": False})).pack(side="left", expand=True, fill="x", padx=2)

        ttk.Button(special_frame, text="🛹 Segway Mode (Secret)", command=self.gui_balance).pack(fill="x", pady=2)

        pid_frame = ttk.LabelFrame(left_panel, text="PID Tuning", padding=10)
        pid_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(pid_frame, text="LinearKp:").grid(row=0, column=0)
        self.ent_lkp = ttk.Entry(pid_frame, width=8)
        self.ent_lkp.insert(0, "0.5")
        self.ent_lkp.grid(row=0, column=1)

        ttk.Label(pid_frame, text="LinearKi:").grid(row=0, column=2)
        self.ent_lki = ttk.Entry(pid_frame, width=8)
        self.ent_lki.insert(0, "0.05")
        self.ent_lki.grid(row=0, column=3)

        ttk.Label(pid_frame, text="LinearKd:").grid(row=0, column=4)
        self.ent_lkd = ttk.Entry(pid_frame, width=8)
        self.ent_lkd.insert(0, "0.1")
        self.ent_lkd.grid(row=0, column=5)

        ttk.Label(pid_frame, text="AngularKp:").grid(row=1, column=0)
        self.ent_akp = ttk.Entry(pid_frame, width=8)
        self.ent_akp.insert(0, "0.3")
        self.ent_akp.grid(row=1, column=1)

        ttk.Label(pid_frame, text="AngularKi:").grid(row=1, column=2)
        self.ent_aki = ttk.Entry(pid_frame, width=8)
        self.ent_aki.insert(0, "0.02")
        self.ent_aki.grid(row=1, column=3)

        ttk.Label(pid_frame, text="AngularKd:").grid(row=1, column=4)
        self.ent_akd = ttk.Entry(pid_frame, width=8)
        self.ent_akd.insert(0, "0.05")
        self.ent_akd.grid(row=1, column=5)

        ttk.Label(pid_frame, text="Circle P:").grid(row=2, column=0)
        self.ent_ckp = ttk.Entry(pid_frame, width=8)
        self.ent_ckp.insert(0, "0.15")
        self.ent_ckp.grid(row=2, column=1)

        ttk.Label(pid_frame, text="Circle Sens:").grid(row=2, column=2)
        self.ent_csen = ttk.Entry(pid_frame, width=8)
        self.ent_csen.insert(0, "2.0")
        self.ent_csen.grid(row=2, column=3)

        ttk.Button(pid_frame, text="Apply PID", command=self.gui_send_config).grid(row=3, column=0, columnspan=6, pady=5)

        telem_frame = ttk.LabelFrame(right_panel, text="Live Telemetry", padding=10)
        telem_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_heading = ttk.Label(telem_frame, text="Heading: --°")
        self.lbl_heading.pack(anchor="w")
        self.lbl_battery = ttk.Label(telem_frame, text="Battery: -- V")
        self.lbl_battery.pack(anchor="w")
        self.lbl_targets = ttk.Label(telem_frame, text="Targets: L=--, θ=--")
        self.lbl_targets.pack(anchor="w")
        
        self.lbl_speeds = ttk.Label(telem_frame, text="Speed: V --/-- mm/s, W --/-- rad/s")
        self.lbl_speeds.pack(anchor="w")
        self.lbl_enc_l = ttk.Label(telem_frame, text="Enc L: --")
        self.lbl_enc_l.pack(anchor="w")
        self.lbl_enc_r = ttk.Label(telem_frame, text="Enc R: --")
        self.lbl_enc_r.pack(anchor="w")

        self.compass_canvas = tk.Canvas(telem_frame, width=100, height=100, bg="#f0f0f0", highlightthickness=1)
        self.compass_canvas.pack(pady=5)

        map_frame = ttk.LabelFrame(right_panel, text="Pen Path (XY Map)", padding=10)
        map_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.map_canvas = tk.Canvas(map_frame, width=400, height=300, bg="white", highlightthickness=1)
        self.map_canvas.pack(fill="both", expand=True)
        ttk.Button(map_frame, text="Clear Map", command=self.clear_map).pack(pady=2)

        log_frame = ttk.LabelFrame(right_panel, text="Data Logging", padding=10)
        log_frame.pack(fill="x", padx=10, pady=5)

        self.btn_record = ttk.Button(log_frame, text="🔴 Start Recording Telemetry", command=self.toggle_recording)
        self.btn_record.pack(fill="x")
        self.lbl_file = ttk.Label(log_frame, text="Not recording", font=("Arial", 8))
        self.lbl_file.pack(pady=2)

        self.update_ui_loop()

    def toggle_recording(self):
        if not self.is_recording:
            if not os.path.exists("Output"):
                os.makedirs("Output")
            
            filename = f"robot_data_{int(time.time())}.csv"
            filepath = os.path.join("Output", filename)
            
            try:
                csv_headers = ["timestamp_ms", "x_mm", "y_mm", "heading_deg", "battery_v",
                               "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
                               "mag_x", "mag_y", "mag_z", "target_l", "target_theta", "target_r",
                               "target_v", "actual_v", "target_w", "actual_w",
                               "ghost_x", "ghost_y", "ghost_heading", "enc_l", "enc_r"]

                self.csv_file = open(filepath, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(csv_headers)
                self.is_recording = True
                self.btn_record.config(text="⏹️ Stop Recording")
                self.lbl_file.config(text=f"Recording to: {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Could not create file: {e}")
        else:
            self.is_recording = False
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            self.btn_record.config(text="🔴 Start Recording Telemetry")
            self.lbl_file.config(text="File saved.")

    def clear_map(self):
        self.path_history = [(0.0, 0.0)]
        self.telemetry['x'] = 0.0
        self.telemetry['y'] = 0.0
        self.map_canvas.delete("all")
        self.draw_map()

    def update_ui_loop(self):
        if self.telemetry:
            h = self.telemetry.get('h', 0.0)
            b = self.telemetry.get('b', 0.0)
            x = self.telemetry.get('x', 0.0)
            y = self.telemetry.get('y', 0.0)
            lth = self.telemetry.get('lth', 0.0)
            ath = self.telemetry.get('ath', 0.0)
            tv = self.telemetry.get('tv', 0.0)
            av = self.telemetry.get('av', 0.0)
            tw = self.telemetry.get('tw', 0.0)
            aw = self.telemetry.get('aw', 0.0)
            enc_l = self.telemetry.get('enc_l', 0)
            enc_r = self.telemetry.get('enc_r', 0)
            self.lbl_heading.config(text=f"Heading: {h:.1f}°")
            self.lbl_battery.config(text=f"Battery: {b:.2f} V")
            self.lbl_targets.config(text=f"Targets: L={lth:.0f}mm, θ={ath:.1f}°")
            self.lbl_speeds.config(text=f"Speed: V {av:.1f}/{tv:.1f} mm/s, W {aw:.3f}/{tw:.3f} rad/s")
            self.lbl_enc_l.config(text=f"Enc L: {enc_l}")
            self.lbl_enc_r.config(text=f"Enc R: {enc_r}")
            self.draw_compass(h)
            self.draw_map()
        if 'co' in self.telemetry:
            try:
                self.lbl_compass_offset_current.config(text=f"Current: {self.telemetry['co']:.2f}°")
            except Exception:
                pass

        self.root.after(200, self.update_ui_loop)

    def draw_compass(self, heading):
        self.compass_canvas.delete("all")
        cx, cy, r = 50, 50, 40
        
        self.compass_canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline="black", width=2)
        self.compass_canvas.create_text(cx, cy-r+8, text="N", font=("Arial", 8, "bold"), fill="red")
        self.compass_canvas.create_text(cx, cy+r-8, text="S", font=("Arial", 7))
        self.compass_canvas.create_text(cx+r-8, cy, text="E", font=("Arial", 7))
        self.compass_canvas.create_text(cx-r+8, cy, text="W", font=("Arial", 7))
        
        rad = math.radians(heading)
        nx = cx + (r-5) * math.sin(rad)
        ny = cy - (r-5) * math.cos(rad)
        
        self.compass_canvas.create_line(cx, cy, nx, ny, fill="red", arrow=tk.LAST, width=2)
        self.compass_canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="black")

    def draw_map(self):
        if not self.path_history:
            return
            
        self.map_canvas.delete("all")
        w = self.map_canvas.winfo_width()
        h = self.map_canvas.winfo_height()

        if w < 10 or h < 10:
            w, h = 400, 300

        cx, cy = w // 2, h // 2

        all_x = [p[0] for p in self.path_history]
        all_y = [p[1] for p in self.path_history]
        min_x, max_x = min(min(all_x), 0), max(max(all_x), 0)
        min_y, max_y = min(min(all_y), 0), max(max(all_y), 0)

        range_x = max(max_x - min_x, 50.0)
        range_y = max(max_y - min_y, 50.0)
        scale = min((w * 0.85) / range_x, (h * 0.85) / range_y)

        offset_x = cx - ((min_x + max_x) / 2.0) * scale
        offset_y = cy - ((min_y + max_y) / 2.0) * scale

        ax_x, ax_y = offset_x + 0 * scale, offset_y + 0 * scale
        self.map_canvas.create_line(0, ax_y, w, ax_y, fill="#eeeeee")
        self.map_canvas.create_line(ax_x, 0, ax_x, h, fill="#eeeeee")

        max_range = max(range_x, range_y)
        if max_range <= 120: tick_step = 20
        elif max_range <= 300: tick_step = 50
        elif max_range <= 750: tick_step = 100
        elif max_range <= 1500: tick_step = 200
        elif max_range <= 4000: tick_step = 500
        else: tick_step = 1000

        start_tick_x = int(min_x // tick_step) * tick_step
        for val in range(start_tick_x, int(max_x) + tick_step, tick_step):
            tx = offset_x + val * scale
            if 0 <= tx <= w:
                self.map_canvas.create_line(tx, ax_y - 3, tx, ax_y + 3, fill="#cccccc")
                if val != 0:
                    self.map_canvas.create_text(tx, ax_y + 12, text=str(val), fill="#bbbbbb", font=("Arial", 7))

        start_tick_y = int(min_y // tick_step) * tick_step
        for val in range(start_tick_y, int(max_y) + tick_step, tick_step):
            ty = offset_y + val * scale
            if 0 <= ty <= h:
                self.map_canvas.create_line(ax_x - 3, ty, ax_x + 3, ty, fill="#cccccc")
                if val != 0:
                    self.map_canvas.create_text(ax_x - 15, ty, text=str(val), fill="#bbbbbb", font=("Arial", 7))

        points = []
        for p in self.path_history:
            px = offset_x + p[0] * scale
            py = offset_y + p[1] * scale
            points.extend([px, py])
            
        if len(points) >= 4:
            self.map_canvas.create_line(points, fill="#3498db", width=2)
            
        curr_x = offset_x + self.path_history[-1][0] * scale
        curr_y = offset_y + self.path_history[-1][1] * scale
        self.map_canvas.create_oval(curr_x-4, curr_y-4, curr_x+4, curr_y+4, fill="#e74c3c", outline="white")

    def start_ble_thread(self):
        def run_async_loop():
            asyncio.run(self.ble_manager_task())

        self.ble_thread = threading.Thread(target=run_async_loop, daemon=True)
        self.ble_thread.start()

    async def ble_manager_task(self):
        while True:
            try:
                if not self.is_connected:
                    self.lbl_status.config(text=f"Searching for {self.device_name}...", foreground="orange")
                    device = await BleakScanner.find_device_by_name(self.device_name, timeout=5.0)
                    
                    if device:
                        self.lbl_status.config(text="Connecting...", foreground="blue")
                        try:
                            async with BleakClient(device) as client:
                                self.client = client
                                self.is_connected = True
                                self.lbl_status.config(text=f"Connected to {self.device_name}", foreground="green")
                                
                                def notification_handler(sender, data):
                                    try:
                                        decoded = data.decode('utf-8')
                                        parts = decoded.split('|')
                                        
                                        kv = {}
                                        for p in parts:
                                            if ':' in p:
                                                k, v = p.split(':', 1)
                                                kv[k] = v

                                        if 'H' in kv: self.telemetry['h'] = float(kv['H'])
                                        if 'B' in kv: self.telemetry['b'] = float(kv['B'])
                                        if 'X' in kv: self.telemetry['x'] = float(kv['X'])
                                        if 'Y' in kv: self.telemetry['y'] = float(kv['Y'])
                                        if 'Lth' in kv: self.telemetry['lth'] = float(kv['Lth'])
                                        if 'Ath' in kv: self.telemetry['ath'] = float(kv['Ath'])
                                        if 'TV' in kv: self.telemetry['tv'] = float(kv['TV'])
                                        if 'AV' in kv: self.telemetry['av'] = float(kv['AV'])
                                        if 'TW' in kv: self.telemetry['tw'] = float(kv['TW'])
                                        if 'AW' in kv: self.telemetry['aw'] = float(kv['AW'])
                                        if 'EL' in kv: self.telemetry['enc_l'] = int(kv['EL'])
                                        if 'ER' in kv: self.telemetry['enc_r'] = int(kv['ER'])
                                        if 'CO' in kv: self.telemetry['co'] = float(kv['CO'])
                                        
                                        if 'X' in kv and 'Y' in kv:
                                            self.path_history.append((self.telemetry['x'], self.telemetry['y']))
                                            if len(self.path_history) > 1500:
                                                self.path_history.pop(0)

                                        if self.is_recording and self.csv_writer:
                                            row_data = [
                                                int(kv.get('T', 0)), float(kv.get('X', 0)), float(kv.get('Y', 0)),
                                                float(kv.get('H', 0)), float(kv.get('B', 0)),
                                                float(kv.get('A', '0,0,0').split(',')[0]), float(kv.get('A', '0,0,0').split(',')[1]), float(kv.get('A', '0,0,0').split(',')[2]),
                                                float(kv.get('G', '0,0,0').split(',')[0]), float(kv.get('G', '0,0,0').split(',')[1]), float(kv.get('G', '0,0,0').split(',')[2]),
                                                float(kv.get('M', '0,0,0').split(',')[0]), float(kv.get('M', '0,0,0').split(',')[1]), float(kv.get('M', '0,0,0').split(',')[2]),
                                                float(kv.get('Lth', 0)), float(kv.get('Ath', 0)), float(kv.get('Rth', 0)),
                                                float(kv.get('TV', 0)), float(kv.get('AV', 0)), float(kv.get('TW', 0)), float(kv.get('AW', 0)),
                                                float(kv.get('GX', 0)), float(kv.get('GY', 0)), float(kv.get('GH', 0)),
                                                int(kv.get('EL', 0)), int(kv.get('ER', 0))
                                            ]
                                            self.csv_writer.writerow(row_data)
                                    except Exception as e:
                                        print(f"[Telemetry Error] {e} - Raw: {data.hex()}")

                                await client.start_notify(self.char_uuid, notification_handler)

                                while client.is_connected:
                                    try:
                                        cmd_json = self.cmd_queue.get_nowait()
                                        await client.write_gatt_char(self.char_uuid, cmd_json.encode('utf-8'))
                                        print(f"[BLE] Sent: {cmd_json}")
                                    except queue.Empty:
                                        await asyncio.sleep(0.1)
                                        
                                self.is_connected = False
                        except Exception as e:
                            print(f"[BLE] Connection error: {e}")
                            self.is_connected = False
                    else:
                        self.lbl_status.config(text="Robot not found", foreground="red")
                
                await asyncio.sleep(2.0)
            except Exception as e:
                print(f"[BLE Manager] Unexpected error: {e}")
                self.is_connected = False
                await asyncio.sleep(2.0)

    def send_cmd(self, cmd, params=None):
        if not self.is_connected:
            print(f"[Remote] Cannot send '{cmd}': BLE not connected")
            return

        payload = {"cmd": cmd.upper()}
        if params:
            payload.update(params)
        self.cmd_queue.put(json.dumps(payload))

    def gui_draw_squares(self):
        size = float(self.ent_sq_size.get())
        self.send_cmd("squares", {"size": size, "count": 1})

    def gui_draw_square_spiral(self):
        try:
            first = float(self.ent_spiral_first.get())
            count = int(self.ent_spiral_count.get())
        except ValueError:
            messagebox.showerror("Invalid spiral value", "First line length and square count must be numbers.")
            return

        if first <= 200:
            messagebox.showerror("Invalid spiral value", "First line length must be greater than 200 mm.")
            return
        if count < 2:
            messagebox.showerror("Invalid spiral value", "Number of squares must be at least 2.")
            return

        self.send_cmd("squareSpiral", {"first": first, "count": count})

    def gui_draw_stairs(self):
        size = float(self.ent_sq_size.get())
        self.send_cmd("stairs", {"size": size})

    def gui_draw_line(self):
        length = float(self.ent_line_length.get())
        self.send_cmd("line", {"length": length})

    def gui_draw_corner(self):
        try:
            size = float(self.ent_corner_size.get())
            self.send_cmd("corner", {"size": size})
        except ValueError:
            messagebox.showerror("Invalid corner value", "Corner size must be a number.")

    def gui_draw_circle(self):
        radius = float(self.ent_radius.get())
        self.send_cmd("circle", {"radius": radius})

    def gui_draw_rosace(self):
        radius = float(self.ent_radius.get())
        self.send_cmd("rosace", {"radius": radius})

    def gui_balance(self):
        if messagebox.askokcancel("Segway Mode", "Ensure robot is standing vertical before starting!"):
            self.send_cmd("balance")

    def gui_send_config(self):
        try:
            self.send_cmd("config", {
                "lkp": float(self.ent_lkp.get()),
                "lki": float(self.ent_lki.get()),
                "lkd": float(self.ent_lkd.get()),
                "akp": float(self.ent_akp.get()),
                "aki": float(self.ent_aki.get()),
                "akd": float(self.ent_akd.get()),
                "ckp": float(self.ent_ckp.get()),
                "csen": float(self.ent_csen.get())
            })
        except ValueError:
            messagebox.showerror("Invalid PID value", "All PID fields must contain numbers.")

    def gui_set_compass_offset(self):
        try:
            val = float(self.ent_compass_offset.get())
        except ValueError:
            messagebox.showerror("Invalid value", "Compass offset must be a number (degrees).")
            return
        # Send to robot to save in NVS, then request confirmation via next telemetry update
        self.send_cmd("compass", {"value": val, "get": True})
        print(f"[Remote] Sent compass offset set request: {val}")

    def gui_get_compass_offset(self):
        # Request the robot to include the compass offset in the next telemetry packet
        self.send_cmd("compass", {"get": True})

if __name__ == "__main__":
    root = tk.Tk()
    gui = RobotRemoteGUI(root)
    root.mainloop()
