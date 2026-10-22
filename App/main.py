import os
import gc
import time
import math
import json
import threading
import queue
import numpy as np
import pybullet as p
import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog, messagebox

from core.simulation_engine import SimulationEngine
from core.kinematics_engine import KinematicsEngine
from core.telemetry_engine import TelemetryEngine
from core.robot_loader import RobotLoader

from ui.pages.manual_page import ManualPage
from ui.pages.fk_page import FKPage
from ui.pages.ik_page import IKPage
from ui.widgets.analysis_panel import AnalysisPanel
from render_engine import RobotGPURenderer

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# Fizik, gerçek zamanın bu katı hızında ilerler. Robot slider'ı hızlı takip etsin diye 1'den büyük;
# önceden bu oran makinenin döngü hızına bağlıydı (Linux'ta ~3.7x).
SIM_SPEED = 4.0
PHYSICS_DT = 1.0 / 240.0

class RobotKontrolApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Otomatik GC kapalı, toplama update_ui_loop'ta (Tk thread'i) yapılır. Aksi halde GC, render ya da fizik
        # thread'inde tetiklenip silinen Tk nesnelerinin (ör. IK'da yeniden kurulan butonların fontları)
        # __del__'ini orada çalıştırıyor ve Tk thread-safe olmadığı için uygulama çöküyordu (segfault).
        gc.disable()
        self.gc_ticks = 0

        self.title("FANUC LR-Mate 200iC | NexusControl Studio (Modüler Mimari)")
        self.geometry("1450x920")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.is_running = True

        self.data_lock = threading.RLock()
        self.position_history = [[0.0] * 6]
        self.history_idx = 0
        self.shared_targets = [0.0] * 6

        self.q_img = queue.Queue(maxsize=1)
        self.q_telemetry = queue.Queue(maxsize=1)
        self.tk_img = None

        self.status_message = "SİSTEM AKTİF - BEKLEMEDE"
        self.current_page = "manual"
        
        self.gpu_state_lock = threading.RLock()
        self.gpu_shared_state = {
            'running': True, 'transforms': [None] * 7, 'obstacle': None,
            'preset': 'standard', 'cam_dist': 1.8, 'cam_target': np.array([0., 0., 0.45]),
            'cam_yaw': 45.0, 'cam_pitch': -25.0, 'grid_size': 1.5,
        }

        # Bağımlılık Enjeksiyonu (Dependency Injection)
        self.sim_engine = SimulationEngine()
        self.sim_engine.set_gpu_shared_state(self.gpu_shared_state, self.gpu_state_lock)
        
        self.telemetry_engine = TelemetryEngine(self.sim_engine)
        self.kinematics_engine = KinematicsEngine(self.sim_engine, self.telemetry_engine)
        self.robot_loader = RobotLoader(self.sim_engine, self.kinematics_engine)

        self.build_ui()
        self.sim_engine.init_pybullet()
        
        self.builtin_urdf_path = os.path.normpath(os.path.join(
            os.path.dirname(__file__), "..", "fanuc_lrmate200ic_support", "urdf", "fanuc_lrmate200ic.urdf"
        ))
        cfg = self.load_config()
        saved_default = cfg.get("default_model_path")
        loaded = False
        if saved_default and os.path.isfile(saved_default):
            loaded, _, _ = self.robot_loader.load_source_model(saved_default)
        if not loaded:
            self.robot_loader.load_robot_model(self.builtin_urdf_path)
            
        self.update_default_model_label()
        self.on_model_loaded()
        self.show_page("manual")

        mesh_dir = os.path.join(os.path.dirname(__file__), "..", "fanuc_lrmate200ic_support", "meshes", "lrmate200ic", "visual")
        self.gpu_renderer = RobotGPURenderer(os.path.normpath(mesh_dir), self.q_img, self.gpu_shared_state, self.gpu_state_lock)
        self.gpu_renderer.start()

        self.worker_thread = threading.Thread(target=self.background_engine, daemon=True)
        self.worker_thread.start()
        self.update_ui_loop()

    def build_ui(self):
        self.grid_rowconfigure(0, weight=4)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=0, minsize=180)
        self.grid_columnconfigure(1, weight=3, minsize=310)
        self.grid_columnconfigure(2, weight=2, minsize=240)
        self.grid_columnconfigure(3, weight=4, minsize=320)

        # 1. NAVBAR
        self.sidebar = ctk.CTkFrame(self, corner_radius=0, fg_color="#141414")
        self.sidebar.grid(row=0, column=0, rowspan=2, sticky="nsew")

        logo_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        logo_frame.pack(pady=(25, 20))
        ctk.CTkLabel(logo_frame, text="NEXUS", text_color="#005A9E", font=ctk.CTkFont(family="Impact", size=34, weight="bold")).grid(row=0, column=0, padx=(0, 2))
        ctk.CTkLabel(logo_frame, text="CONTROL", text_color="#FFFFFF", font=ctk.CTkFont(family="Segoe UI Light", size=24)).grid(row=0, column=1, sticky="s", pady=(0, 5))

        ctk.CTkButton(self.sidebar, text="Manuel Kontrol", height=40, command=lambda: self.show_page("manual")).pack(pady=8, padx=10, fill="x")
        ctk.CTkButton(self.sidebar, text="İleri Kinematik", height=40, command=lambda: self.show_page("fk")).pack(pady=8, padx=10, fill="x")
        ctk.CTkButton(self.sidebar, text="Ters Kinematik", height=40, command=lambda: self.show_page("ik")).pack(pady=8, padx=10, fill="x")
        
        self.btn_nav_workspace = ctk.CTkButton(self.sidebar, text="Saydam İzi Göster", height=40, fg_color="#005A9E", command=self.toggle_workspace)
        self.btn_nav_workspace.pack(pady=(20, 8), padx=10, fill="x")

        obs_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        obs_frame.pack(pady=8, padx=10, fill="x")
        obs_frame.grid_columnconfigure(0, weight=3)
        obs_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkButton(obs_frame, text="Engel Aç/Kapat", height=40, fg_color="#D2691E", command=self.toggle_obstacle).grid(row=0, column=0, padx=(0, 5), sticky="ew")
        ctk.CTkButton(obs_frame, text="Rastgele", height=40, fg_color="#555555", command=self.randomize_obstacle).grid(row=0, column=1, sticky="ew")

        ctk.CTkButton(self.sidebar, text="E-STOP SIFIRLA", height=40, fg_color="#8B0000", command=self.reset_estop).pack(pady=(20, 8), padx=10, fill="x")

        # Kalite Seçici
        self.quality_menu = ctk.CTkOptionMenu(self.sidebar, values=["Standard (600p)", "HD (1.5× SS)", "FHD (2× SS)"], command=self.set_render_quality)
        self.quality_menu.pack(pady=(18, 6), padx=10, fill="x")
        self.lbl_qual_info = ctk.CTkLabel(self.sidebar, text="600p native · GPU · 60 FPS", font=ctk.CTkFont(size=10))
        self.lbl_qual_info.pack(pady=(0, 6))

        # Model Yükleyici
        ctk.CTkButton(self.sidebar, text="📁 Model Yükle", height=34, fg_color="#1E4D2B", command=self.open_urdf_file_dialog).pack(pady=3, padx=10, fill="x")
        
        btn_model_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        btn_model_frame.pack(pady=3, padx=10, fill="x")
        btn_model_frame.grid_columnconfigure(0, weight=1)
        btn_model_frame.grid_columnconfigure(1, weight=1)
        ctk.CTkButton(btn_model_frame, text="⭐ Ana Model", height=30, fg_color="#332B10", text_color="#FFD700", command=self.set_current_as_default).grid(row=0, column=0, padx=(0, 3), sticky="ew")
        ctk.CTkButton(btn_model_frame, text="↺ Orijinal", height=30, fg_color="#2A2A2A", command=self.reset_default_robot).grid(row=0, column=1, padx=(3, 0), sticky="ew")
        
        self.lbl_default_info = ctk.CTkLabel(self.sidebar, text="Açılış: Fanuc", font=ctk.CTkFont(size=10))
        self.lbl_default_info.pack(pady=(1, 6))

        # 2. PAGES
        self.pages_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.pages_frame.grid(row=0, column=1, padx=15, pady=(15, 5), sticky="nsew")
        self.pages_frame.grid_rowconfigure(0, weight=1)
        self.pages_frame.grid_columnconfigure(0, weight=1)

        self.page_manual = ManualPage(self.pages_frame, self, self.sim_engine)
        self.page_fk = FKPage(self.pages_frame, self)
        self.page_ik = IKPage(self.pages_frame, self, self.kinematics_engine, self.telemetry_engine)

        # 3. ANALYSIS PANEL
        self.analysis_panel = AnalysisPanel(self)
        self.analysis_panel.grid(row=0, column=2, padx=10, pady=(15, 5), sticky="nsew")

        # 4. RENDER FRAME
        self.render_frame = ctk.CTkFrame(self, fg_color="#1A1A1A", corner_radius=12, border_width=1, border_color="#2A2A2A")
        self.render_frame.grid(row=0, column=3, padx=(0, 15), pady=(15, 5), sticky="nsew")
        self.render_frame.grid_rowconfigure(0, weight=1)
        self.render_frame.grid_columnconfigure(0, weight=1)

        self.camera_label = ctk.CTkLabel(self.render_frame, text="Motor Yükleniyor...", text_color="gray", fg_color="transparent")
        self.camera_label.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        self.camera_label.bind("<ButtonPress-1>", self._on_cam_mouse_down)
        self.camera_label.bind("<B1-Motion>", self._on_cam_mouse_drag)
        self.camera_label.bind("<ButtonPress-3>", self._on_cam_right_down)
        self.camera_label.bind("<B3-Motion>", self._on_cam_right_drag)
        self.camera_label.bind("<MouseWheel>", self._on_cam_mouse_wheel)

        # 5. GRAPH FRAME
        self.graph_frame = ctk.CTkFrame(self, fg_color="#1E1E1E", corner_radius=12, border_width=1, border_color="#2A2A2A")
        self.graph_frame.grid(row=1, column=1, columnspan=3, padx=15, pady=(5, 15), sticky="nsew")

        self.graph_header = ctk.CTkFrame(self.graph_frame, fg_color="transparent")
        self.graph_header.pack(fill="x", padx=15, pady=(5, 2))
        self.graph_header.grid_columnconfigure(0, weight=1)
        self.graph_header.grid_columnconfigure(1, weight=2)
        self.graph_header.grid_columnconfigure(2, weight=0)

        ctk.CTkLabel(self.graph_header, text="⚡ CANLI MOTOR GÜÇ TÜKETİMİ (WATT)", font=ctk.CTkFont(size=12, weight="bold")).grid(row=0, column=0, sticky="w")
        self.lbl_energy_summary = ctk.CTkLabel(self.graph_header, text="Anlık: 0.0 W", font=ctk.CTkFont(size=11, weight="bold"), text_color="#00FFCC")
        self.lbl_energy_summary.grid(row=0, column=1, sticky="e", padx=15)
        
        ctk.CTkButton(self.graph_header, text="↺ Sıfırla", width=65, height=24, fg_color="#2A2A2A", command=self.reset_energy_counter).grid(row=0, column=2, sticky="e")
        
        self.canvas = tk.Canvas(self.graph_frame, bg="#0A0A0A", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

    # --- ACTIONS ---
    def show_page(self, page_name):
        self.page_manual.pack_forget()
        self.page_fk.pack_forget()
        self.page_ik.pack_forget()
        self.current_page = page_name

        if page_name == "manual": self.page_manual.pack(fill="both", expand=True)
        elif page_name == "fk": self.page_fk.pack(fill="both", expand=True)
        elif page_name == "ik": self.page_ik.pack(fill="both", expand=True)

    def toggle_obstacle(self):
        state, msg = self.sim_engine.toggle_obstacle()
        self.status_message = msg

    def randomize_obstacle(self):
        self.status_message = self.sim_engine.randomize_obstacle()

    def reset_estop(self):
        self.estop_reason = None
        with self.data_lock:
            self.status_message = self.sim_engine.reset_estop(self.shared_targets)
            for var in self.page_manual.slider_vars: var.set(0.0)

    def toggle_workspace(self):
        state = self.kinematics_engine.toggle_workspace()
        self.btn_nav_workspace.configure(text="Saydam İzi Temizle" if state else "Saydam İzi Göster")

    def reset_energy_counter(self):
        self.status_message = self.telemetry_engine.reset_energy_counter()

    def trigger_fk_calculation(self):
        log = self.kinematics_engine.calculate_fk()
        self.analysis_panel.update_report(log)
        self.save_position()

    def trigger_ik_calculation(self, x_val, y_val, z_val):
        with self.data_lock:
            current_angles = [float(v.get()) for v in self.page_manual.slider_vars]
            
        log, has_risk, err, desc = self.kinematics_engine.calculate_ik(x_val, y_val, z_val, current_angles)
        self.analysis_panel.update_report(log)
        self.page_ik.rebuild_waypoints(self.kinematics_engine.pending_waypoints, err, has_risk, desc)
        
        if has_risk or err > 0.03:
            fail_reason = desc if has_risk else f"Hedef Ulaşılamaz ({err * 100:.1f} cm sapma)"
            self.status_message = f"UYARI: {fail_reason}! Hareket kilitlendi."
        else:
            self.status_message = "IK Hesaplandı! Rota güvenli (Çarpışma yok). Yörünge seçin."

    def execute_trajectory(self, strategy):
        duration = self.kinematics_engine.time_fast if strategy == "fast" else self.kinematics_engine.time_eco
        strat_name = "Hızlı" if strategy == "fast" else "Ekonomik"
        self.start_trajectory(strategy, self.kinematics_engine.pending_waypoints, duration,
                              f"{strat_name} Rota başlatıldı. Süzülüyor...")

    def start_trajectory(self, strategy, waypoints, duration, status_text):
        if self.sim_engine.e_stop_active:
            self.status_message = "E-STOP AKTİF! Önce Sıfırlayın."
            return
        if getattr(self, "is_trajectory_playing", False):
            self.status_message = "Robot hareket ediyor, hareket bitince tekrar deneyin."
            return

        # Bitince butonlar önceki hâline döner (ör. riskli rotada kilitli kalmaya devam etmeli)
        self.traj_btn_states = (self.page_ik.btn_fast_traj.cget("state"), self.page_ik.btn_eco_traj.cget("state"))
        self.page_ik.btn_fast_traj.configure(state="disabled")
        self.page_ik.btn_eco_traj.configure(state="disabled")
        self.status_message = status_text

        with self.data_lock:
            current_angles = [self.shared_targets[i] for i in range(len(self.sim_engine.revolute_joints))]

        self.is_trajectory_playing = True
        threading.Thread(target=self.run_trajectory_thread,
                         args=(strategy, waypoints, current_angles, duration), daemon=True).start()

    def run_trajectory_thread(self, strategy, waypoints, start_angles, total_duration):
        num_waypoints = len(waypoints)
        # Tek süre verilirse segmentlere eşit bölünür; liste verilirse her segmentin kendi süresi
        durations = total_duration if isinstance(total_duration, list) else [total_duration / num_waypoints] * num_waypoints
        current = start_angles

        for wp_idx, target_angles in enumerate(waypoints):
            duration_per_wp = durations[wp_idx]
            steps = int(max(duration_per_wp * 30, 5))
            self.kinematics_engine.update_link_positions()

            for step in range(1, steps + 1):
                if self.sim_engine.e_stop_active: break
                t = step / steps
                factor = t if strategy == "fast" else (1 - math.cos(math.pi * t)) / 2.0
                angles = [c + (tr - c) * factor for c, tr in zip(current, target_angles)]

                with self.sim_engine.bullet_lock:
                    if self.sim_engine.e_stop_active: break
                    for i, joint_idx in enumerate(self.sim_engine.revolute_joints):
                        if i < len(angles): p.resetJointState(self.sim_engine.robotId, joint_idx, angles[i])
                    if self.kinematics_engine.show_workspace: self.kinematics_engine.draw_swept_volume()

                with self.data_lock:
                    if self.sim_engine.e_stop_active: break
                    for i in range(min(len(angles), len(self.shared_targets))):
                        self.shared_targets[i] = angles[i]

                time.sleep(duration_per_wp / steps)

            if self.sim_engine.e_stop_active: break
            current = target_angles

        if not self.sim_engine.e_stop_active: self.status_message = f"Robot hedefe başarıyla yerleşti!"
        # Tk thread-safe değil: kayıt ve buton güncellemesi update_ui_loop'ta (ana thread) yapılır
        self.trajectory_finished = True

    def route_position(self, angles, waypoints):
        """Robot rotadaki bir waypoint'te duruyorsa onun indeksi, değilse None. Sadece TCP zincirindeki
        eklemlere bakılır; gripper parmakları IK'dan sonra değişmiş olabilir."""
        chain = self.kinematics_engine.ee_chain_joints()
        cols = [k for k, j in enumerate(self.sim_engine.revolute_joints) if j in chain and k < len(angles)]
        for idx, wp in enumerate(waypoints):
            if all(abs(angles[k] - wp[k]) < 1e-3 for k in cols if k < len(wp)):
                return idx
        return None

    def go_to_ik_step(self, target_idx):
        waypoints = self.kinematics_engine.pending_waypoints
        if not 0 <= target_idx < len(waypoints): return
        with self.data_lock:
            current = list(self.shared_targets)

        # Aradaki manevralar engel kaçışının parçası olabilir, atlanmaz: robot rotada bir noktadaysa oradan,
        # değilse baştan başlayarak hedefe kadar sırayla oynatılır (hedef gerideyse ters sırayla).
        at = self.route_position(current, waypoints)
        if at == target_idx:
            self.status_message = f"Robot zaten Manevra {target_idx + 1} konumunda."
            return
        if at is None:
            sequence = list(range(0, target_idx + 1))
        elif at < target_idx:
            sequence = list(range(at + 1, target_idx + 1))
        else:
            sequence = list(range(at - 1, target_idx - 1, -1))

        # Duruşlar ve aradaki yollar tıklama anında tekrar kontrol edilir (engel IK'dan sonra değişmiş olabilir)
        prismatic = self.kinematics_engine.prismatic_joint_flags()
        steps, durations, prev = [], [], current
        for idx in sequence:
            wp = waypoints[idx]
            is_safe, desc = self.kinematics_engine.check_angles_safety(wp)
            if is_safe:
                is_safe, desc = self.kinematics_engine.check_path_safety(prev, wp)
            if not is_safe:
                messagebox.showwarning("Çarpışma Engeli", f"Manevra {idx + 1} konumuna gidilemez:\n{desc}")
                self.status_message = f"UYARI: Manevra {idx + 1}: {desc}! Hareket iptal edildi."
                return
            # Her segment Eko rotanın S-eğrisi ve süre ölçeğiyle
            max_deg = max([abs(math.degrees(a - c)) for a, c, lin in zip(wp, prev, prismatic) if not lin] or [0.0])
            steps.append(wp); durations.append(max(0.5, max_deg / 100.0) * 1.8); prev = wp

        label = f"Manevra {sequence[0] + 1}" if len(sequence) == 1 else f"Manevra {sequence[0] + 1} → {sequence[-1] + 1}"
        self.start_trajectory("eco", steps, durations, f"{label} oynatılıyor...")

    def make_scroll_smooth(self, scrollable_frame, step_px=50):
        def _on_wheel(event):
            try:
                canvas = getattr(scrollable_frame, '_parent_canvas', None)
                if canvas is not None and event.delta != 0:
                    canvas.yview_scroll(-int((event.delta / 120.0) * step_px), "units")
            except: pass
            return "break"
        def _bind_all_children(widget):
            widget.bind("<MouseWheel>", _on_wheel, add="+")
            for child in widget.winfo_children(): _bind_all_children(child)
        _bind_all_children(scrollable_frame)
        canvas = getattr(scrollable_frame, '_parent_canvas', None)
        if canvas: canvas.bind("<MouseWheel>", _on_wheel, add="+")

    def on_model_loaded(self):
        # Hedef listesi ve konum geçmişi modelin eksen sayısına göre yeniden boyutlanır;
        # sabit 6 elemanlı liste 7. ve sonraki eksenlere (ör. gripper) komut gitmesini engelliyordu.
        n_joints = len(self.sim_engine.revolute_joints)
        with self.data_lock:
            self.shared_targets = [0.0] * n_joints
            self.position_history = [[0.0] * n_joints]
            self.history_idx = 0
        self.page_manual.rebuild_joint_sliders(self.sim_engine.revolute_joints, self.sim_engine.robotId, self.make_scroll_smooth)
        self.page_ik.update_limits()
        self.sliders_locked = None  # yeni slider'lara kilit durumu bir sonraki UI tick'inde uygulanır

    def save_position(self):
        current_pos = [var.get() for var in self.page_manual.slider_vars]
        if current_pos != self.position_history[self.history_idx]:
            self.position_history = self.position_history[:self.history_idx + 1]
            self.position_history.append(current_pos)
            self.history_idx += 1

    def go_to_base(self):
        for var in self.page_manual.slider_vars: var.set(0.0)
        self.save_position()

    def undo_position(self):
        if self.history_idx > 0:
            self.history_idx -= 1
            hist = self.position_history[self.history_idx]
            for i, var in enumerate(self.page_manual.slider_vars):
                if i < len(hist): var.set(hist[i])

    def redo_position(self):
        if self.history_idx < len(self.position_history) - 1:
            self.history_idx += 1
            hist = self.position_history[self.history_idx]
            for i, var in enumerate(self.page_manual.slider_vars):
                if i < len(hist): var.set(hist[i])

    # --- CAM CONTROLS ---
    def _on_cam_mouse_down(self, event):
        self._cam_last_x, self._cam_last_y = event.x, event.y

    def _on_cam_mouse_drag(self, event):
        dx = event.x - getattr(self, '_cam_last_x', event.x)
        dy = event.y - getattr(self, '_cam_last_y', event.y)
        self._cam_last_x, self._cam_last_y = event.x, event.y
        with self.gpu_state_lock:
            self.gpu_shared_state['cam_yaw'] = (float(self.gpu_shared_state.get('cam_yaw', 45.0)) - dx * 0.45) % 360
            self.gpu_shared_state['cam_pitch'] = max(-89.0, min(89.0, float(self.gpu_shared_state.get('cam_pitch', -25.0)) - dy * 0.45))

    def _on_cam_right_down(self, event):
        self._cam_last_x, self._cam_last_y = event.x, event.y

    def _on_cam_right_drag(self, event):
        dy = event.y - getattr(self, '_cam_last_y', event.y)
        self._cam_last_y = event.y
        with self.gpu_state_lock:
            tgt = np.array(self.gpu_shared_state.get('cam_target', [0., 0., 0.45]), dtype=float)
            tgt[2] += dy * 0.002 * float(self.gpu_shared_state.get('cam_dist', 1.8))
            self.gpu_shared_state['cam_target'] = tgt

    def _on_cam_mouse_wheel(self, event):
        with self.gpu_state_lock:
            dist = float(self.gpu_shared_state.get('cam_dist', 1.8))
            self.gpu_shared_state['cam_dist'] = max(0.3, min(40.0, dist * (1.12 ** (-event.delta / 120.0))))

    def set_render_quality(self, value):
        preset_map = {"Standard (600p)": "standard", "HD (1.5× SS)": "hd", "FHD (2× SS)": "fhd"}
        preset = preset_map.get(value, "standard")
        with self.gpu_state_lock: self.gpu_shared_state['preset'] = preset
        self.lbl_qual_info.configure(text={"standard": "600p native · 60 FPS", "hd": "1.5× SS · 60 FPS", "fhd": "2× SS · 60 FPS"}[preset])

    # --- URDF LOADING ---
    def load_config(self):
        cfg_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json"))
        if os.path.isfile(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f: return json.load(f)
            except: pass
        return {"default_model_path": None, "recent_models": []}

    def save_config(self, cfg):
        try:
            with open(os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")), "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=4, ensure_ascii=False)
            return True
        except: return False

    def update_default_model_label(self):
        saved = self.load_config().get("default_model_path")
        if saved and os.path.isfile(saved):
            name = os.path.splitext(os.path.basename(saved))[0][:14] + ".."
            self.lbl_default_info.configure(text=f"Açılış: {name} ⭐", text_color="#FFD700")
        else:
            self.lbl_default_info.configure(text="Açılış: Fanuc (Dahili)", text_color="#777777")

    def open_urdf_file_dialog(self):
        fp = filedialog.askopenfilename(filetypes=[("Modeller", "*.urdf;*.xacro")])
        if fp:
            success, msg, data = self.robot_loader.load_source_model(fp)
            self.status_message = msg
            if success:
                self.current_source_path = fp
                self.on_model_loaded()

    def set_current_as_default(self):
        if not hasattr(self, 'current_source_path'): return
        cfg = self.load_config()
        cfg["default_model_path"] = os.path.abspath(self.current_source_path)
        self.save_config(cfg)
        self.update_default_model_label()

    def reset_default_robot(self):
        cfg = self.load_config()
        cfg["default_model_path"] = None
        self.save_config(cfg)
        self.robot_loader.load_robot_model(self.builtin_urdf_path)
        self.update_default_model_label()
        self.on_model_loaded()

    # --- MAIN LOOPS ---
    def background_engine(self):
        accumulator = 0.0
        last_time = time.perf_counter()
        last_swept_time = last_time
        last_transform_time = last_time
        last_safety_time = last_time

        while self.is_running:
            current_time = time.perf_counter()
            dt = min(current_time - last_time, 0.05)
            last_time = current_time

            with self.data_lock: targets = list(self.shared_targets)

            # E-Stop and Safety check natively inside step simulation or handled externally?
            # We decoupled check_angles_safety, but here we just step
            if not self.sim_engine.e_stop_active:
                self.telemetry_engine.calculate_power_tick(dt)

            # Sabit adımlı fizik: duvar saatinde geçen süre x SIM_SPEED kadar simülasyon
            accumulator += dt * SIM_SPEED
            while accumulator >= PHYSICS_DT:
                self.sim_engine.step_simulation(targets)
                accumulator -= PHYSICS_DT

            # Çarpışma -> E-stop (20 Hz). 08e7e66 refactor'ünde bu kontrol kaybolmuştu.
            if not self.sim_engine.e_stop_active and (current_time - last_safety_time >= 0.05):
                last_safety_time = current_time
                reason = self.kinematics_engine.detect_collision(check_floor=getattr(self, "is_trajectory_playing", False))
                if reason: self.trigger_estop(reason)

            if self.kinematics_engine.show_workspace and (current_time - last_swept_time >= (1.0 / 15.0)):
                last_swept_time = current_time
                self.kinematics_engine.draw_swept_volume()

            if current_time - last_transform_time >= (1.0 / 60.0):
                last_transform_time = current_time
                shapes_meta = getattr(self.sim_engine, 'visual_shapes_meta', [])
                transforms = [None] * len(shapes_meta)
                with self.sim_engine.bullet_lock:
                    if self.sim_engine.robotId is not None:
                        base_pos, base_quat = p.getBasePositionAndOrientation(self.sim_engine.robotId)
                        for idx, meta in enumerate(shapes_meta):
                            try:
                                if meta['link_idx'] == -1:
                                    lp_pos, lp_ori = p.multiplyTransforms(base_pos, base_quat, meta['local_pos'], meta['local_ori'])
                                else:
                                    ls = p.getLinkState(self.sim_engine.robotId, meta['link_idx'])
                                    lp_pos, lp_ori = p.multiplyTransforms(ls[4], ls[5], meta['local_pos'], meta['local_ori'])
                                mat = p.getMatrixFromQuaternion(lp_ori)
                                tf_mat = np.eye(4, dtype=np.float32)
                                tf_mat[:3, :3] = np.array(mat).reshape(3, 3)
                                tf_mat[:3, 3] = lp_pos
                                transforms[idx] = tf_mat
                            except: pass
                # Robot durduysa yayınlama: renderer sadece sahne değişince kare üretir.
                # Model yüklenince loader listeyi sıfırlar, o yüzden paylaşılan değerle karşılaştır.
                with self.gpu_state_lock:
                    if not self._transforms_equal(transforms, self.gpu_shared_state.get('transforms')):
                        self.gpu_shared_state['transforms'] = transforms

            time.sleep(max(0.0, PHYSICS_DT - (time.perf_counter() - current_time)))

    def trigger_estop(self, reason):
        # Arka plan thread'inden çağrılır, Tk'ye dokunmaz. Hedefler robotun durduğu yere çekilir;
        # update_ui_loop slider'ları bu değerlere kilitler, böylece slider çarpışma noktasından ileri gitmez.
        with self.sim_engine.bullet_lock:
            self.sim_engine.e_stop_active = True
            held = [s[0] for s in p.getJointStates(self.sim_engine.robotId, self.sim_engine.revolute_joints)]
        with self.data_lock:
            for i in range(min(len(held), len(self.shared_targets))):
                self.shared_targets[i] = held[i]
        self.estop_reason = reason

    @staticmethod
    def _transforms_equal(a, b):
        if a is None or b is None or len(a) != len(b): return False
        for ta, tb in zip(a, b):
            if (ta is None) != (tb is None): return False
            if ta is not None and not np.allclose(ta, tb, atol=1e-6): return False
        return True

    def update_ui_loop(self):
        if not self.is_running: return

        # Nesil 0 her tick (~0.01 ms), nesil 1 her 10 tick, tam toplama ~30 sn'de bir (~14 ms)
        self.gc_ticks += 1
        gc.collect(2 if self.gc_ticks % 600 == 0 else 1 if self.gc_ticks % 10 == 0 else 0)

        with self.data_lock:
            n_vars = len(self.page_manual.slider_vars)
            if getattr(self, "is_trajectory_playing", False) or self.sim_engine.e_stop_active:
                for i in range(min(n_vars, len(self.shared_targets))):
                    self.page_manual.slider_vars[i].set(self.shared_targets[i])
            else:
                for i in range(min(n_vars, len(self.shared_targets))):
                    self.shared_targets[i] = self.page_manual.slider_vars[i].get()

        # Slider'lar yukarıda son hedeflere eşitlendi; kayıt ondan sonra, oynatma bayrağı en son kapanır
        if getattr(self, "trajectory_finished", False):
            self.trajectory_finished = False
            self.save_position()
            self.is_trajectory_playing = False
            fast_state, eco_state = getattr(self, "traj_btn_states", ("normal", "normal"))
            self.page_ik.btn_fast_traj.configure(state=fast_state)
            self.page_ik.btn_eco_traj.configure(state=eco_state)

        img = None
        while not self.q_img.empty():
            try: img = self.q_img.get_nowait()
            except queue.Empty: break
        if img is not None:
            self.tk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(600, 800))
            self.camera_label.configure(image=self.tk_img, text="")

        self.analysis_panel.set_estop_state(self.sim_engine.e_stop_active)
        if self.sim_engine.e_stop_active:
            reason = getattr(self, "estop_reason", None) or "ÇARPIŞMA TESPİT EDİLDİ"
            self.status_message = f"!!! E-STOP: {reason} - MOTORLAR KİLİTLENDİ !!!"
        if self.sim_engine.e_stop_active != getattr(self, "sliders_locked", None):
            self.sliders_locked = self.sim_engine.e_stop_active
            for slider in self.page_manual.sliders:
                slider.configure(state="disabled" if self.sliders_locked else "normal")

        # Telemetry updates (Mockup for original system)
        telemetry_text = f"Motor Verimi: %{self.telemetry_engine.motor_efficiency*100:.0f}\nYük: {self.telemetry_engine.payload_kg:.1f} kg\nDurum: {self.status_message}"
        self.analysis_panel.update_telemetry(telemetry_text)

        # Graph updates
        kwh_val = self.telemetry_engine.cumulative_energy_kwh
        cost_val = self.telemetry_engine.cumulative_cost_tl
        pow_val = self.telemetry_engine.current_power
        self.lbl_energy_summary.configure(text=f"Anlık: {pow_val:5.1f} W  |  Sayaç: {kwh_val:.4f} kWh  |  Maliyet: {cost_val:.3f} TL")
        
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        self.canvas.delete("all")
        self.canvas.create_line(0, h / 2, w, h / 2, fill="#333333", dash=(4, 4))
        history = self.telemetry_engine.power_history
        max_power = max(200.0, max(history) * 1.2) if history else 200.0
        self.canvas.create_text(10, 10, text=f"Max: {max_power:.0f} W", fill="#666666", anchor="nw")
        
        points = []
        for i, val in enumerate(history):
            points.extend([(i / 100.0) * w, h - ((val / max_power) * h)])
        if len(points) >= 4:
            self.canvas.create_line(*points, fill="#FF3333" if self.sim_engine.e_stop_active else "#00FF66", width=2, smooth=True)

        self.after(30, self.update_ui_loop)

    def on_closing(self):
        self.is_running = False
        with self.gpu_state_lock: self.gpu_shared_state['running'] = False
        self.quit()
        self.destroy()

if __name__ == "__main__":
    app = RobotKontrolApp()
    app.mainloop()