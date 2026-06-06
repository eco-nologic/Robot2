import tkinter as tk
from tkinter import ttk
import math

# --- Constantes du Robot ---
DIAMETRE_ROUE = 9.0
TICKS_PAR_ROTATION = 1000
ENTRAXE = 8.75
ROUE_PERIMETRE = math.pi * DIAMETRE_ROUE
STYLO_OFFSET = 13.0

class SimulateurRobot(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Simulateur de Mouvement Robot")
        self.geometry("1300x750")

        self.sequences = []
        self.sequence_paths = []
        self.is_simulating = False
        self.echelle = 15
        self.selected_seq_idx = -1

        # IK mode animation
        self.is_animating_ik = False
        self.ik_pen_trace = []
        self.ik_trace_idx = 0
        
        self.setup_ui()
        self.reset_simulation()

        # Generate IK staircase waypoints and compute path
        step = 4.0  # cm
        waypoints = []
        for i in range(5):
            x = (i + 1) * step
            y_h = i * step
            y_v = (i + 1) * step
            waypoints.append((x, y_h))
            waypoints.append((x, y_v))
        ik_pen_trace, _ = self.run_gcode_ik(waypoints)

        # Create a special sequence entry for IK path
        ik_sequence = ("IK", "Staircase", 0, 0, 0, 0)

        # Clear and rebuild sequences with IK data included
        self.sequences = []
        self.seq_listbox.delete(0, tk.END)

        # Load default sequences
        self.load_default_sequences()

        # Add IK sequence
        self.sequences.append(ik_sequence)
        self.seq_listbox.insert(tk.END, "[IK] Staircase")

        # Compute trajectories
        self.compute_full_trajectory()

        # Replace the last (IK) sequence path with actual IK pen trace
        self.sequence_paths[-1] = ik_pen_trace

    def setup_ui(self):
        # Panneau de gauche (Contrôles) élargi
        control_frame = ttk.Frame(self, width=450, padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)
        control_frame.pack_propagate(False) # Empêche le frame de se réduire
        
        # Ajout / Modification de séquence
        add_frame = ttk.LabelFrame(control_frame, text="Paramètres de Séquence", padding=10)
        add_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(add_frame, text="Ticks Gauche:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.entry_ticks_g = ttk.Entry(add_frame, width=12)
        self.entry_ticks_g.grid(row=0, column=1, pady=2, padx=5)
        
        ttk.Label(add_frame, text="Ticks Droit:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.entry_ticks_d = ttk.Entry(add_frame, width=12)
        self.entry_ticks_d.grid(row=1, column=1, pady=2, padx=5)
        
        ttk.Label(add_frame, text="Vitesse Gauche:").grid(row=0, column=2, sticky=tk.W, pady=2, padx=10)
        self.entry_vitesse_g = ttk.Entry(add_frame, width=12)
        self.entry_vitesse_g.grid(row=0, column=3, pady=2)
        
        ttk.Label(add_frame, text="Vitesse Droite:").grid(row=1, column=2, sticky=tk.W, pady=2, padx=10)
        self.entry_vitesse_d = ttk.Entry(add_frame, width=12)
        self.entry_vitesse_d.grid(row=1, column=3, pady=2)

        ttk.Label(add_frame, text="Délai Gauche (s):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.entry_delai_g = ttk.Entry(add_frame, width=12)
        self.entry_delai_g.grid(row=2, column=1, pady=2, padx=5)
        
        ttk.Label(add_frame, text="Délai Droit (s):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.entry_delai_d = ttk.Entry(add_frame, width=12)
        self.entry_delai_d.grid(row=3, column=1, pady=2, padx=5)
        
        # Valeurs par défaut
        self.entry_delai_g.insert(0, "0.0")
        self.entry_delai_d.insert(0, "0.0")
        self.entry_vitesse_g.insert(0, "80")
        self.entry_vitesse_d.insert(0, "80")
        
        btn_frame = ttk.Frame(add_frame)
        btn_frame.grid(row=4, column=0, columnspan=4, pady=10)
        
        ttk.Button(btn_frame, text="Ajouter Séquence", command=self.add_sequence).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Modifier Séquence Sélectionnée", command=self.modify_sequence).pack(side=tk.LEFT, padx=5)
        
        # Liste des séquences
        list_frame = ttk.LabelFrame(control_frame, text="Séquences actuelles", padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.seq_listbox = tk.Listbox(list_frame, height=15)
        self.seq_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.seq_listbox.bind('<<ListboxSelect>>', self.on_sequence_select)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.seq_listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.seq_listbox.config(yscrollcommand=scrollbar.set)
        
        ttk.Button(control_frame, text="Supprimer la sélection", command=self.remove_sequence).pack(fill=tk.X, pady=2)
        ttk.Button(control_frame, text="Vider la liste", command=self.clear_sequences).pack(fill=tk.X, pady=2)
        
        # Contrôles de simulation
        sim_frame = ttk.Frame(control_frame, padding=10)
        sim_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(sim_frame, text="Animer la Simulation", command=self.start_simulation).pack(fill=tk.X, pady=5)
        ttk.Button(sim_frame, text="Simuler G-code IK", command=self.simulate_gcode_ik).pack(fill=tk.X, pady=5)
        ttk.Button(sim_frame, text="Réinitialiser Vue", command=self.reset_simulation).pack(fill=tk.X, pady=5)

        # Panneau de droite (Canvas)
        self.canvas = tk.Canvas(self, bg="white")
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Bindings pour le zoom
        self.canvas.bind("<MouseWheel>", self.zoom)
        self.canvas.bind("<Button-4>", self.zoom)
        self.canvas.bind("<Button-5>", self.zoom)

    def zoom(self, event):
        if event.num == 4 or getattr(event, 'delta', 0) > 0:
            self.echelle += 1
        elif event.num == 5 or getattr(event, 'delta', 0) < 0:
            self.echelle -= 1
            
        if self.echelle < 2:
            self.echelle = 2
        elif self.echelle > 100:
            self.echelle = 100
            
        self.draw_environment()

    def format_seq_str(self, tg, td, vg, vd, dg, dd):
        return f"G:(t:{tg} v:{vg} d:{dg}s) | D:(t:{td} v:{vd} d:{dd}s)"

    def load_default_sequences(self):
        # 1- Avancer de 10 cm (environ 354 ticks)
        ticks_10cm = int((10.0 / ROUE_PERIMETRE) * TICKS_PAR_ROTATION)
        default_seqs = [
            (ticks_10cm, ticks_10cm, 80, 80, 0.0, 0.0), # 1
            (30, -10, 100, 100, 0.0, 0.0),              # 2
            (50, -10, 100, 100, 0.0, 0.0),              # 3
            (50, 1, 100, 70, 0.0, 0.5),                 # 4
            (50, 1, 100, 70, 0.0, 0.4),                 # 5
            (5, 30, 100, 100, 0.1, 0.0)                 # 6
        ]
        for seq in default_seqs:
            self.sequences.append(seq)
            self.seq_listbox.insert(tk.END, self.format_seq_str(*seq))

    def parse_inputs(self):
        try:
            tg = int(self.entry_ticks_g.get())
            td = int(self.entry_ticks_d.get())
            vg = int(self.entry_vitesse_g.get())
            vd = int(self.entry_vitesse_d.get())
            dg = float(self.entry_delai_g.get())
            dd = float(self.entry_delai_d.get())
            return (tg, td, vg, vd, dg, dd)
        except ValueError:
            return None

    def add_sequence(self):
        seq = self.parse_inputs()
        if seq:
            self.sequences.append(seq)
            self.seq_listbox.insert(tk.END, self.format_seq_str(*seq))
            self.compute_full_trajectory()

    def modify_sequence(self):
        if self.selected_seq_idx >= 0 and self.selected_seq_idx < len(self.sequences):
            seq = self.parse_inputs()
            if seq:
                self.sequences[self.selected_seq_idx] = seq
                # Update listbox text
                self.seq_listbox.delete(self.selected_seq_idx)
                self.seq_listbox.insert(self.selected_seq_idx, self.format_seq_str(*seq))
                self.seq_listbox.selection_set(self.selected_seq_idx)
                self.compute_full_trajectory()

    def on_sequence_select(self, event):
        selection = self.seq_listbox.curselection()
        if selection:
            self.selected_seq_idx = selection[0]
            seq = self.sequences[self.selected_seq_idx]
            
            # Populate fields
            self.entry_ticks_g.delete(0, tk.END)
            self.entry_ticks_g.insert(0, str(seq[0]))
            
            self.entry_ticks_d.delete(0, tk.END)
            self.entry_ticks_d.insert(0, str(seq[1]))
            
            self.entry_vitesse_g.delete(0, tk.END)
            self.entry_vitesse_g.insert(0, str(seq[2]))
            
            self.entry_vitesse_d.delete(0, tk.END)
            self.entry_vitesse_d.insert(0, str(seq[3]))
            
            self.entry_delai_g.delete(0, tk.END)
            self.entry_delai_g.insert(0, str(seq[4]))
            
            self.entry_delai_d.delete(0, tk.END)
            self.entry_delai_d.insert(0, str(seq[5]))
            
            # Redraw to highlight sequence
            self.draw_environment()

    def remove_sequence(self):
        selection = self.seq_listbox.curselection()
        if selection:
            index = selection[0]
            self.seq_listbox.delete(index)
            self.sequences.pop(index)
            self.selected_seq_idx = -1
            self.compute_full_trajectory()

    def clear_sequences(self):
        self.seq_listbox.delete(0, tk.END)
        self.sequences.clear()
        self.selected_seq_idx = -1
        self.compute_full_trajectory()

    def compute_full_trajectory(self):
        # Pré-calcule tout le tracé de manière instantanée
        rx = 10.0
        ry = 20.0
        rtheta = 0.0
        
        self.sequence_paths = []
        
        for seq in self.sequences:
            # Skip IK sequences (they have string entries)
            if isinstance(seq[0], str):
                self.sequence_paths.append([])
                continue

            tg, td, vg, vd, dg, dd = seq
            path = []

            # Position du stylo init
            px = rx + STYLO_OFFSET * math.cos(rtheta)
            py = ry + STYLO_OFFSET * math.sin(rtheta)
            path.append((px, py))
            
            delai_G_ms = dg * 1000.0
            delai_D_ms = dd * 1000.0
            fin_G = delai_G_ms + 1000.0
            fin_D = delai_D_ms + 1000.0
            duree_totale = max(fin_G, fin_D)
            
            t = 0.0
            last_ticks_G = 0.0
            last_ticks_D = 0.0
            
            while t <= duree_totale:
                current_ticks_G = 0.0
                if t > delai_G_ms and t <= fin_G:
                    current_ticks_G = (abs(tg) * (t - delai_G_ms)) / 1000.0
                elif t > fin_G:
                    current_ticks_G = abs(tg)
                    
                current_ticks_D = 0.0
                if t > delai_D_ms and t <= fin_D:
                    current_ticks_D = (abs(td) * (t - delai_D_ms)) / 1000.0
                elif t > fin_D:
                    current_ticks_D = abs(td)

                sign_G = 1 if tg >= 0 else -1
                sign_D = 1 if td >= 0 else -1

                delta_ticks_G = (current_ticks_G - last_ticks_G) * sign_G
                delta_ticks_D = (current_ticks_D - last_ticks_D) * sign_D

                delta_dist_G = (delta_ticks_G / TICKS_PAR_ROTATION) * ROUE_PERIMETRE
                delta_dist_D = (delta_ticks_D / TICKS_PAR_ROTATION) * ROUE_PERIMETRE

                distance_center = (delta_dist_G + delta_dist_D) / 2.0
                delta_theta = (delta_dist_D - delta_dist_G) / ENTRAXE
                
                rtheta += delta_theta
                rx += distance_center * math.cos(rtheta)
                ry += distance_center * math.sin(rtheta)
                
                px = rx + STYLO_OFFSET * math.cos(rtheta)
                py = ry + STYLO_OFFSET * math.sin(rtheta)
                path.append((px, py))
                
                last_ticks_G = current_ticks_G
                last_ticks_D = current_ticks_D
                t += 20.0
                
            self.sequence_paths.append(path)
            
        self.draw_environment()

    def reset_simulation(self):
        self.is_simulating = False
        # Position initiale du robot pour l'animation
        self.robot_x = 10.0 # cm
        self.robot_y = 20.0 # cm
        self.robot_theta = 0.0 # radians
        self.compute_full_trajectory() # Force le redessin propre

    def world_to_screen(self, x, y):
        return x * self.echelle, self.canvas.winfo_height() - (y * self.echelle)

    def draw_environment(self):
        self.canvas.delete("all")

        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        if width <= 1 or height <= 1:
            self.after(100, self.draw_environment)
            return

        for x in range(0, width, self.echelle * 10):
            self.canvas.create_line(x, 0, x, height, fill="#e0e0e0")
        for y in range(0, height, self.echelle * 10):
            self.canvas.create_line(0, y, width, y, fill="#e0e0e0")

        # Draw IK animation (if active)
        if self.is_animating_ik and self.ik_pen_trace:
            # Full desired path (red dashes)
            if len(self.ik_pen_trace) > 1:
                points = []
                for px, py in self.ik_pen_trace:
                    sx, sy = self.world_to_screen(px, py)
                    points.extend([sx, sy])
                self.canvas.create_line(points, fill="red", width=2, dash=(4, 4))

            # Traced path so far (green solid)
            if self.ik_trace_idx > 1:
                points = []
                for px, py in self.ik_pen_trace[:self.ik_trace_idx]:
                    sx, sy = self.world_to_screen(px, py)
                    points.extend([sx, sy])
                self.canvas.create_line(points, fill="green", width=2)
            return

        # Dessiner les tracés précalculés (normal mode)
        for idx, path in enumerate(self.sequence_paths):
            if len(path) > 1:
                points = []
                for px, py in path:
                    sx, sy = self.world_to_screen(px, py)
                    points.extend([sx, sy])

                # Surligner la séquence sélectionnée
                if idx == self.selected_seq_idx:
                    self.canvas.create_line(points, fill="green", width=4)
                else:
                    self.canvas.create_line(points, fill="red", width=2)

        # Dessiner le robot
        cx, cy = self.robot_x, self.robot_y
        rg_x = cx - (ENTRAXE / 2) * math.sin(self.robot_theta)
        rg_y = cy + (ENTRAXE / 2) * math.cos(self.robot_theta)
        rd_x = cx + (ENTRAXE / 2) * math.sin(self.robot_theta)
        rd_y = cy - (ENTRAXE / 2) * math.cos(self.robot_theta)
        
        ps_x = cx + STYLO_OFFSET * math.cos(self.robot_theta)
        ps_y = cy + STYLO_OFFSET * math.sin(self.robot_theta)
        
        s_rg_x, s_rg_y = self.world_to_screen(rg_x, rg_y)
        s_rd_x, s_rd_y = self.world_to_screen(rd_x, rd_y)
        s_ps_x, s_ps_y = self.world_to_screen(ps_x, ps_y)
        
        self.canvas.create_polygon(s_rg_x, s_rg_y, s_rd_x, s_rd_y, s_ps_x, s_ps_y, 
                                   fill="", outline="blue", width=2)
        
        r = 3
        self.canvas.create_oval(s_rg_x-r, s_rg_y-r, s_rg_x+r, s_rg_y+r, fill="black")
        self.canvas.create_oval(s_rd_x-r, s_rd_y-r, s_rd_x+r, s_rd_y+r, fill="black")
        self.canvas.create_oval(s_ps_x-r, s_ps_y-r, s_ps_x+r, s_ps_y+r, fill="red")

    def run_gcode_ik(self, waypoints):
        """Compute G-code path using Inverse Kinematics (Resolved Rate Control).
        Returns (pen_trace, sequences) where:
        - pen_trace is a list of (x, y) pen positions
        - sequences is a list of (tg, td, vg, vd, dg, dd) commands"""
        DT = 0.02  # 20 ms
        DRAW_SPEED = 3.0  # cm/s
        APPROACH_GAIN = 3.0  # proportional ramp-down
        WAYPOINT_TOL = 0.5  # cm

        rx, ry, theta = 10.0, 20.0, 0.0
        idx = 0
        pen_trace = [(rx + STYLO_OFFSET * math.cos(theta),
                     ry + STYLO_OFFSET * math.sin(theta))]
        sequences = []

        ticks_accum_left = 0.0
        ticks_accum_right = 0.0
        speed_accum_left = 0.0
        speed_accum_right = 0.0
        speed_steps = 0
        time_since_seq = 0.0

        def flush_sequence():
            nonlocal ticks_accum_left, ticks_accum_right, speed_accum_left, speed_accum_right, speed_steps, time_since_seq
            if speed_steps == 0:
                return
            tg = int(round(ticks_accum_left))
            td = int(round(ticks_accum_right))
            vg = int(round(speed_accum_left / speed_steps))
            vd = int(round(speed_accum_right / speed_steps))
            sequences.append((tg, td, vg, vd, 0.0, 0.0))
            ticks_accum_left = 0.0
            ticks_accum_right = 0.0
            speed_accum_left = 0.0
            speed_accum_right = 0.0
            speed_steps = 0
            time_since_seq = 0.0

        while idx < len(waypoints):
            wx, wy = waypoints[idx]
            pen_x = rx + STYLO_OFFSET * math.cos(theta)
            pen_y = ry + STYLO_OFFSET * math.sin(theta)

            ex, ey = wx - pen_x, wy - pen_y
            dist = math.hypot(ex, ey)

            if dist < WAYPOINT_TOL:
                idx += 1
                continue

            # Desired pen velocity
            speed = min(DRAW_SPEED, dist * APPROACH_GAIN)
            vpx = (ex / dist) * speed
            vpy = (ey / dist) * speed

            # Inverse Jacobian
            v = vpx * math.cos(theta) + vpy * math.sin(theta)
            omega = (-vpx * math.sin(theta) + vpy * math.cos(theta)) / STYLO_OFFSET

            # Wheel velocities
            v_left = v - omega * ENTRAXE / 2
            v_right = v + omega * ENTRAXE / 2

            # Odometry integration
            dl = v_left * DT
            dr = v_right * DT
            ds = (dl + dr) / 2
            dtheta = (dr - dl) / ENTRAXE
            theta += dtheta
            rx += ds * math.cos(theta)
            ry += ds * math.sin(theta)

            pen_trace.append((rx + STYLO_OFFSET * math.cos(theta),
                            ry + STYLO_OFFSET * math.sin(theta)))

            # Accumulate one-second commands from 20ms IK steps.
            ticks_accum_left += (v_left * DT / ROUE_PERIMETRE) * TICKS_PAR_ROTATION
            ticks_accum_right += (v_right * DT / ROUE_PERIMETRE) * TICKS_PAR_ROTATION
            speed_accum_left += v_left
            speed_accum_right += v_right
            speed_steps += 1
            time_since_seq += DT

            if time_since_seq >= 1.0 - 1e-6:
                flush_sequence()

        if speed_steps > 0:
            flush_sequence()

        return pen_trace, sequences

    def start_simulation(self):
        if self.is_simulating or not self.sequences:
            return

        # On réinitialise juste la position du robot pour l'animation
        self.robot_x = 10.0
        self.robot_y = 20.0
        self.robot_theta = 0.0

        self.is_simulating = True
        self.current_seq_idx = 0
        self.sim_time = 0.0
        self.last_ticks_G = 0.0
        self.last_ticks_D = 0.0

        self.simulate_next_step()

    def simulate_gcode_ik(self):
        """Test the IK algorithm with a hardcoded staircase."""
        if self.is_animating_ik or self.is_simulating:
            return

        # Generate 5-step staircase (20 cm total)
        step = 4.0  # cm
        waypoints = []
        for i in range(5):
            x = (i + 1) * step
            y_h = i * step
            y_v = (i + 1) * step
            waypoints.append((x, y_h))
            waypoints.append((x, y_v))

        # Compute pen trace and command sequence via IK
        pen_trace, ik_sequences = self.run_gcode_ik(waypoints)

        # Store for animation
        self.ik_pen_trace = pen_trace
        self.ik_trace_idx = 0
        self.is_animating_ik = True

        # Replace current sequences with the generated IK command sequence
        self.sequences.clear()
        self.seq_listbox.delete(0, tk.END)
        self.sequences.extend(ik_sequences)
        for seq in ik_sequences:
            self.seq_listbox.insert(tk.END, self.format_seq_str(*seq))

        # Recompute the trajectory from the generated sequence commands
        self.compute_full_trajectory()

        # Reset robot and update UI
        self.robot_x = 10.0
        self.robot_y = 20.0
        self.robot_theta = 0.0

        # Start animation
        self.animate_ik_trace()

    def animate_ik_trace(self):
        """Animate through the IK pen trace one point at a time."""
        if not self.is_animating_ik or self.ik_trace_idx >= len(self.ik_pen_trace):
            self.is_animating_ik = False
            return

        # Update robot position to current pen trace point
        if self.ik_trace_idx < len(self.ik_pen_trace):
            pen_x, pen_y = self.ik_pen_trace[self.ik_trace_idx]
            # For display: we animate the pen position
            # (In real robot, we'd reconstruct axle from pen via inverse of pen offset)
            self.robot_x = pen_x - 13.0 * math.cos(0)  # approximate for display
            self.robot_y = pen_y

        self.ik_trace_idx += 1
        self.draw_environment()
        self.after(20, self.animate_ik_trace)

    def simulate_next_step(self):
        if not self.is_simulating:
            return

        if self.current_seq_idx >= len(self.sequences):
            self.is_simulating = False
            return

        tg, td, vg, vd, dg, dd = self.sequences[self.current_seq_idx]
        
        delai_G_ms = dg * 1000.0
        delai_D_ms = dd * 1000.0
        fin_G = delai_G_ms + 1000.0
        fin_D = delai_D_ms + 1000.0
        duree_totale = max(fin_G, fin_D)

        if self.sim_time > duree_totale:
            self.current_seq_idx += 1
            self.sim_time = 0.0
            self.last_ticks_G = 0.0
            self.last_ticks_D = 0.0
            self.after(200, self.simulate_next_step)
            return

        current_ticks_G = 0.0
        if self.sim_time > delai_G_ms and self.sim_time <= fin_G:
            current_ticks_G = (abs(tg) * (self.sim_time - delai_G_ms)) / 1000.0
        elif self.sim_time > fin_G:
            current_ticks_G = abs(tg)
            
        current_ticks_D = 0.0
        if self.sim_time > delai_D_ms and self.sim_time <= fin_D:
            current_ticks_D = (abs(td) * (self.sim_time - delai_D_ms)) / 1000.0
        elif self.sim_time > fin_D:
            current_ticks_D = abs(td)

        sign_G = 1 if tg >= 0 else -1
        sign_D = 1 if td >= 0 else -1

        delta_ticks_G = (current_ticks_G - self.last_ticks_G) * sign_G
        delta_ticks_D = (current_ticks_D - self.last_ticks_D) * sign_D

        delta_dist_G = (delta_ticks_G / TICKS_PAR_ROTATION) * ROUE_PERIMETRE
        delta_dist_D = (delta_ticks_D / TICKS_PAR_ROTATION) * ROUE_PERIMETRE

        distance_center = (delta_dist_G + delta_dist_D) / 2.0
        delta_theta = (delta_dist_D - delta_dist_G) / ENTRAXE
        
        self.robot_theta += delta_theta
        self.robot_x += distance_center * math.cos(self.robot_theta)
        self.robot_y += distance_center * math.sin(self.robot_theta)

        self.draw_environment()

        self.last_ticks_G = current_ticks_G
        self.last_ticks_D = current_ticks_D
        self.sim_time += 20.0

        self.after(20, self.simulate_next_step)

if __name__ == "__main__":
    app = SimulateurRobot()
    app.mainloop()
