import pybullet as p
import pybullet_data
import os
import time
import math
import json
import customtkinter as ctk
from PIL import Image
import numpy as np
import threading
import queue
import random
import tkinter as tk
from tkinter import filedialog, messagebox
from render_engine import RobotGPURenderer, quat_to_mat4

# Koyu Tema Ayarları
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class RobotKontrolApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("FANUC LR-Mate 200iC | NexusControl Studio (Final Otonom & Sınırlar)")
        self.geometry("1450x920")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.is_running = True

        self.position_history = [[0.0] * 6]
        self.history_idx = 0
        self.shared_targets = [0.0] * 6

        self.data_lock = threading.RLock()
        self.bullet_lock = threading.RLock()

        self.q_img = queue.Queue(maxsize=1)
        self.q_telemetry = queue.Queue(maxsize=1)
        self.tk_img = None

        self.status_message = "SİSTEM AKTİF - BEKLEMEDE"
        self.ik_math_log = ""
        self.fk_math_log = ""
        self.current_page = "manual"

        self.show_workspace = False
        self.workspace_ids = []
        self.last_link_positions = {}

        self.is_trajectory_playing = False
        self.pending_waypoints = []
        self.time_fast = 0.0
        self.time_eco = 0.0

        self.power_history = [0.0] * 100
        self.current_power = 0.0
        self.e_stop_active = False

        # Motor, Yük ve Enerji Maliyet Sistemi
        self.payload_kg = 0.0
        self.base_ee_mass = 1.0
        self.motor_efficiency = 0.85
        self.motor_standby_w = 15.0
        self.electricity_rate = 4.50
        self.cumulative_energy_joules = 0.0
        self.cumulative_energy_kwh = 0.0
        self.cumulative_cost_tl = 0.0

        # GPU Renderer paylaşılan durum
        self.gpu_state_lock = threading.RLock()
        self.gpu_shared_state = {
            'running':    True,
            'transforms': [None] * 7,
            'obstacle':   None,
            'preset':     'standard',   # 'standard' | 'hd' | 'fhd'
            'cam_dist':   1.8,
            'cam_target': np.array([0., 0., 0.45]),
            'cam_yaw':    45.0,
            'cam_pitch':  -25.0,
            'grid_size':  1.5,
        }
        self.shoulder_r_cm = 7.5
        self.shoulder_z_cm = 33.0
        self.arm_length_cm = 72.5
        self.robot_reach_cm = 80.0
        self.base_z_cm = 33.0
        self.stand_top_cm = 0.0
        self.stand_half_extent_cm = 0.0
        self.min_safe_z_cm = 5.0

        self.obstacle_id = None
        self.obstacle_pos = [0.0, 0.0, 0.0]

        # ==========================================
        # SÜTUN DÜZENİ
        # ==========================================
        self.grid_rowconfigure(0, weight=4)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=0, minsize=180)
        self.grid_columnconfigure(1, weight=3, minsize=310)
        self.grid_columnconfigure(2, weight=2, minsize=240)
        self.grid_columnconfigure(3, weight=4, minsize=320)

        # --- 1. NAVBAR ---
        self.sidebar = ctk.CTkFrame(self, corner_radius=0, fg_color="#141414")
        self.sidebar.grid(row=0, column=0, rowspan=2, sticky="nsew")

        # --- HAVALI NEXUS CONTROL LOGOSU ---
        self.logo_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        self.logo_frame.pack(pady=(25, 20))

        self.logo_nexus = ctk.CTkLabel(self.logo_frame, text="NEXUS", text_color="#005A9E",
                                       font=ctk.CTkFont(family="Impact", size=34, weight="bold"))
        self.logo_nexus.grid(row=0, column=0, padx=(0, 2))

        self.logo_control = ctk.CTkLabel(self.logo_frame, text="CONTROL", text_color="#FFFFFF",
                                         font=ctk.CTkFont(family="Segoe UI Light", size=24))
        self.logo_control.grid(row=0, column=1, sticky="s", pady=(0, 5))

        self.logo_sub = ctk.CTkLabel(self.sidebar, text="222812006\nMuhammet Mert Görgülü",
                                     font=ctk.CTkFont(family="Segoe UI", size=13), justify="center",
                                     text_color="#AAAAAA")
        self.logo_sub.pack(pady=(0, 20))

        # --- NAVBAR BUTONLARI ---
        self.btn_nav_manual = ctk.CTkButton(self.sidebar, text="Manuel Kontrol", height=40, corner_radius=8,
                                            command=lambda: self.show_page("manual"))
        self.btn_nav_manual.pack(pady=8, padx=10, fill="x")

        self.btn_nav_fk = ctk.CTkButton(self.sidebar, text="İleri Kinematik", height=40, corner_radius=8,
                                        command=lambda: self.show_page("fk"))
        self.btn_nav_fk.pack(pady=8, padx=10, fill="x")

        self.btn_nav_ik = ctk.CTkButton(self.sidebar, text="Ters Kinematik", height=40, corner_radius=8,
                                        command=lambda: self.show_page("ik"))
        self.btn_nav_ik.pack(pady=8, padx=10, fill="x")

        self.btn_nav_workspace = ctk.CTkButton(self.sidebar, text="Saydam İzi Göster", height=40, corner_radius=8,
                                               fg_color="#005A9E", hover_color="#003A68", command=self.toggle_workspace)
        self.btn_nav_workspace.pack(pady=(20, 8), padx=10, fill="x")

        # --- DİNAMİK ENGEL BUTONLARI ---
        obs_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        obs_frame.pack(pady=8, padx=10, fill="x")
        obs_frame.grid_columnconfigure(0, weight=3)
        obs_frame.grid_columnconfigure(1, weight=1)

        self.btn_obs_toggle = ctk.CTkButton(obs_frame, text="Engel Aç/Kapat", height=40, corner_radius=8,
                                            fg_color="#D2691E", hover_color="#A0522D", command=self.toggle_obstacle)
        self.btn_obs_toggle.grid(row=0, column=0, padx=(0, 5), sticky="ew")

        self.btn_obs_rand = ctk.CTkButton(obs_frame, text="Rastgele", height=40, corner_radius=8,
                                          fg_color="#555555", hover_color="#444444", command=self.randomize_obstacle)
        self.btn_obs_rand.grid(row=0, column=1, sticky="ew")

        # E-STOP SIFIRLAMA
        self.btn_estop_reset = ctk.CTkButton(self.sidebar, text="E-STOP SIFIRLA", height=40, corner_radius=8,
                                             fg_color="#8B0000", hover_color="#550000", command=self.reset_estop)
        self.btn_estop_reset.pack(pady=(20, 8), padx=10, fill="x")

        # --- RENDER KALİTE SEÇİCİ ---
        qual_lbl = ctk.CTkLabel(self.sidebar, text="RENDER KALİTESİ",
                                font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
                                text_color="#555555")
        qual_lbl.pack(pady=(18, 3))

        self.quality_menu = ctk.CTkOptionMenu(
            self.sidebar,
            values=["Standard (640p)", "HD (1.5× SS)", "FHD (2× SS)"],
            command=self.set_render_quality,
            fg_color="#1A3A5C",
            button_color="#005A9E",
            button_hover_color="#003A68",
            text_color="#FFFFFF",
            font=ctk.CTkFont(family="Segoe UI", size=12)
        )
        self.quality_menu.set("Standard (640p)")
        self.quality_menu.pack(pady=(0, 6), padx=10, fill="x")

        self.lbl_qual_info = ctk.CTkLabel(self.sidebar, text="640p native · GPU · 60 FPS",
                                          font=ctk.CTkFont(family="Consolas", size=10),
                                          text_color="#444444")
        self.lbl_qual_info.pack(pady=(0, 6))

        # --- MODEL / URDF SEÇİMİ ---
        model_lbl = ctk.CTkLabel(self.sidebar, text="ROBOT MODELİ",
                                 font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
                                 text_color="#555555")
        model_lbl.pack(pady=(8, 3))

        self.btn_load_urdf = ctk.CTkButton(self.sidebar, text="📁 Model Yükle (URDF / Xacro)", height=34, corner_radius=8,
                                           fg_color="#1E4D2B", hover_color="#163820",
                                           command=self.open_urdf_file_dialog)
        self.btn_load_urdf.pack(pady=3, padx=10, fill="x")

        # Varsayılan Yap & Fabrikaya Dön Butonları
        btn_model_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        btn_model_frame.pack(pady=3, padx=10, fill="x")
        btn_model_frame.grid_columnconfigure(0, weight=1)
        btn_model_frame.grid_columnconfigure(1, weight=1)

        self.btn_set_default = ctk.CTkButton(btn_model_frame, text="⭐ Ana Model Yap", height=30, corner_radius=8,
                                             fg_color="#332B10", hover_color="#4A3E14", text_color="#FFD700",
                                             font=ctk.CTkFont(size=11),
                                             command=self.set_current_as_default)
        self.btn_set_default.grid(row=0, column=0, padx=(0, 3), sticky="ew")

        self.btn_reset_model = ctk.CTkButton(btn_model_frame, text="↺ Orijinal Fanuc", height=30, corner_radius=8,
                                             fg_color="#2A2A2A", hover_color="#3A3A3A",
                                             font=ctk.CTkFont(size=11),
                                             command=self.reset_default_robot)
        self.btn_reset_model.grid(row=0, column=1, padx=(3, 0), sticky="ew")

        self.lbl_default_info = ctk.CTkLabel(self.sidebar, text="Açılış: Fanuc (Dahili)",
                                             font=ctk.CTkFont(family="Segoe UI", size=10),
                                             text_color="#777777")
        self.lbl_default_info.pack(pady=(1, 6))

        # --- 2. KONTROL PANELİ ---
        self.pages_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.pages_frame.grid(row=0, column=1, padx=15, pady=(15, 5), sticky="nsew")
        self.pages_frame.grid_rowconfigure(0, weight=1)
        self.pages_frame.grid_columnconfigure(0, weight=1)

        self.page_manual = ctk.CTkFrame(self.pages_frame, fg_color="transparent")
        self.page_fk = ctk.CTkFrame(self.pages_frame, fg_color="transparent")
        self.page_ik = ctk.CTkScrollableFrame(self.pages_frame, fg_color="transparent")

        self.setup_manual_page()
        self.setup_fk_page()
        self.setup_ik_page()

        # --- 3. ANALİZ PANELİ ---
        self.analysis_frame = ctk.CTkFrame(self, fg_color="#1E1E1E", corner_radius=12, border_width=1,
                                           border_color="#2A2A2A")
        self.analysis_frame.grid(row=0, column=2, padx=10, pady=(15, 5), sticky="nsew")

        self.analysis_title = ctk.CTkLabel(self.analysis_frame, text="SİSTEM & ANALİZ",
                                           font=ctk.CTkFont(family="Segoe UI", size=12, weight="bold"))
        self.analysis_title.pack(pady=(10, 4))

        # Canlı Telemetri Kartı (Sabit boy, kaydırma çubuğu yok, anlık ve hafif güncelleme)
        self.telemetry_box = ctk.CTkTextbox(self.analysis_frame, height=135,
                                            font=ctk.CTkFont(family="Consolas", size=11),
                                            fg_color="#0A0A0A", text_color="#00FFCC", corner_radius=8,
                                            border_width=1, border_color="#1F1F1F", activate_scrollbars=False)
        self.telemetry_box.pack(padx=12, pady=(0, 6), fill="x")

        # Statik Matematik ve Güvenlik Raporu (Sadece IK/FK hesaplandığında güncellenir, akıcı kaydırma)
        self.analysis_box = ctk.CTkTextbox(self.analysis_frame, font=ctk.CTkFont(family="Consolas", size=11),
                                           fg_color="#0A0A0A", text_color="#CCCCCC", corner_radius=8,
                                           border_width=1, border_color="#1F1F1F")
        self.analysis_box.pack(padx=12, pady=(0, 12), fill="both", expand=True)

        # --- 4. 3D ROBOT GÖRÜNÜMÜ ---
        self.render_frame = ctk.CTkFrame(self, fg_color="#1A1A1A", corner_radius=12, border_width=1,
                                         border_color="#2A2A2A")
        self.render_frame.grid(row=0, column=3, padx=(0, 15), pady=(15, 5), sticky="nsew")
        self.render_frame.grid_rowconfigure(0, weight=1)
        self.render_frame.grid_columnconfigure(0, weight=1)

        self.camera_label = ctk.CTkLabel(self.render_frame, text="Motor Yükleniyor...", text_color="gray",
                                         fg_color="transparent")
        self.camera_label.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Fare ile 3D Görünüm Kontrolleri (Sol tık: Döndür, Sağ tık: Yükseklik, Tekerlek: Zoom)
        self.camera_label.bind("<ButtonPress-1>", self._on_cam_mouse_down)
        self.camera_label.bind("<B1-Motion>", self._on_cam_mouse_drag)
        self.camera_label.bind("<ButtonPress-3>", self._on_cam_right_down)
        self.camera_label.bind("<B3-Motion>", self._on_cam_right_drag)
        self.camera_label.bind("<MouseWheel>", self._on_cam_mouse_wheel)

        # --- 5. CANLI OSİLOSKOP GRAFİĞİ ---
        self.graph_frame = ctk.CTkFrame(self, fg_color="#1E1E1E", corner_radius=12, border_width=1,
                                        border_color="#2A2A2A")
        self.graph_frame.grid(row=1, column=1, columnspan=3, padx=15, pady=(5, 15), sticky="nsew")

        # Osiloskop Üst Çubuğu (Başlık + Canlı Sayaç + Sıfırla Butonu)
        self.graph_header = ctk.CTkFrame(self.graph_frame, fg_color="transparent")
        self.graph_header.pack(fill="x", padx=15, pady=(5, 2))
        self.graph_header.grid_columnconfigure(0, weight=1)
        self.graph_header.grid_columnconfigure(1, weight=2)
        self.graph_header.grid_columnconfigure(2, weight=0)

        self.graph_label = ctk.CTkLabel(self.graph_header, text="⚡ CANLI MOTOR GÜÇ TÜKETİMİ (WATT)",
                                        font=ctk.CTkFont(family="Consolas", size=12, weight="bold"),
                                        text_color="#AAAAAA")
        self.graph_label.grid(row=0, column=0, sticky="w")

        self.lbl_energy_summary = ctk.CTkLabel(
            self.graph_header,
            text="Anlık: 0.0 W  |  Sayaç: 0.000 kWh  |  Maliyet: 0.00 TL",
            font=ctk.CTkFont(family="Consolas", size=11, weight="bold"),
            text_color="#00FFCC"
        )
        self.lbl_energy_summary.grid(row=0, column=1, sticky="e", padx=15)

        self.btn_reset_energy = ctk.CTkButton(
            self.graph_header, text="↺ Sıfırla", width=65, height=24, corner_radius=6,
            font=ctk.CTkFont(size=10), fg_color="#2A2A2A", hover_color="#3A3A3A",
            command=self.reset_energy_counter
        )
        self.btn_reset_energy.grid(row=0, column=2, sticky="e")

        self.canvas = tk.Canvas(self.graph_frame, bg="#0A0A0A", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

        self.init_pybullet()
        self.show_page("manual")

        # GPU Renderer thread başlat
        mesh_dir = os.path.join(os.path.dirname(__file__),
                                "..", "fanuc_lrmate200ic_support",
                                "meshes", "lrmate200ic", "visual")
        mesh_dir = os.path.normpath(mesh_dir)
        self.gpu_renderer = RobotGPURenderer(
            mesh_dir, self.q_img, self.gpu_shared_state, self.gpu_state_lock
        )
        self.gpu_renderer.start()

        self.worker_thread = threading.Thread(target=self.background_engine, daemon=True)
        self.worker_thread.start()
        self.update_ui_loop()

    # ==========================================
    # 3D KAMERA FARE ETKİLEŞİMİ (ORBIT & ZOOM)
    # ==========================================
    def _on_cam_mouse_down(self, event):
        self._cam_last_x = event.x
        self._cam_last_y = event.y

    def _on_cam_mouse_drag(self, event):
        dx = event.x - getattr(self, '_cam_last_x', event.x)
        dy = event.y - getattr(self, '_cam_last_y', event.y)
        self._cam_last_x = event.x
        self._cam_last_y = event.y
        with self.gpu_state_lock:
            cur_yaw = float(self.gpu_shared_state.get('cam_yaw', 45.0))
            cur_pitch = float(self.gpu_shared_state.get('cam_pitch', -25.0))
            self.gpu_shared_state['cam_yaw'] = (cur_yaw - dx * 0.45) % 360
            self.gpu_shared_state['cam_pitch'] = max(-89.0, min(89.0, cur_pitch - dy * 0.45))

    def _on_cam_right_down(self, event):
        self._cam_last_x = event.x
        self._cam_last_y = event.y

    def _on_cam_right_drag(self, event):
        dy = event.y - getattr(self, '_cam_last_y', event.y)
        self._cam_last_y = event.y
        with self.gpu_state_lock:
            tgt = np.array(self.gpu_shared_state.get('cam_target', [0., 0., 0.45]), dtype=float)
            dist = float(self.gpu_shared_state.get('cam_dist', 1.8))
            tgt[2] += dy * 0.002 * dist
            self.gpu_shared_state['cam_target'] = tgt

    def _on_cam_mouse_wheel(self, event):
        delta = -event.delta / 120.0
        with self.gpu_state_lock:
            dist = float(self.gpu_shared_state.get('cam_dist', 1.8))
            factor = 1.12 ** delta
            self.gpu_shared_state['cam_dist'] = max(0.3, min(40.0, dist * factor))

    # ==========================================
    # RENDER KALİTE KONTROLÜ
    # ==========================================
    def set_render_quality(self, value):
        preset_map = {
            "Standard (640p)":  "standard",
            "HD (1.5× SS)":     "hd",
            "FHD (2× SS)":      "fhd",
        }
        info_map = {
            "standard": "640p native · GPU · 60 FPS",
            "hd":       "960p→640p · 1.5× SS · 60 FPS",
            "fhd":      "1280p→640p · 2× SS · 60 FPS",
        }
        preset = preset_map.get(value, "standard")
        with self.gpu_state_lock:
            self.gpu_shared_state['preset'] = preset
        self.lbl_qual_info.configure(text=info_map.get(preset, ""))
        self.status_message = f"Render kalitesi: {value}"

    # ==========================================
    # ENGEL (OBSTACLE) FONKSİYONLARI (RASTGELE BOYUT VE KONUM)
    # ==========================================
    def toggle_obstacle(self):
        with self.bullet_lock:
            if self.obstacle_id is None:
                hx, hy, hz = 0.08, 0.08, 0.2
                self.obstacle_pos = [0.4, 0.0, hz]
                obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])
                self.obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obs_col,
                                                     basePosition=self.obstacle_pos)
                self.status_message = "Engel aktif edildi."
                with self.gpu_state_lock:
                    self.gpu_shared_state['obstacle'] = (list(self.obstacle_pos), [hx, hy, hz])
            else:
                p.removeBody(self.obstacle_id)
                self.obstacle_id = None
                self.status_message = "Engel kaldirildi."
                with self.gpu_state_lock:
                    self.gpu_shared_state['obstacle'] = None

    def randomize_obstacle(self):
        with self.bullet_lock:
            if self.obstacle_id is not None:
                p.removeBody(self.obstacle_id)

            hx = random.uniform(0.04, 0.08)
            hy = random.uniform(0.04, 0.08)
            hz = random.uniform(0.15, 0.40)

            angle = random.uniform(-math.pi / 1.5, math.pi / 1.5)
            radius = random.uniform(0.35, 0.60)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = hz

            self.obstacle_pos = [x, y, z]
            obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])
            self.obstacle_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obs_col,
                                                 basePosition=self.obstacle_pos)
            with self.gpu_state_lock:
                self.gpu_shared_state['obstacle'] = ([x, y, z], [hx, hy, hz])

            self.status_message = f"Rastgele Engel: Uzunluk {hz * 200:.0f}cm, X:{x * 100:.0f} Y:{y * 100:.0f}"

    def reset_estop(self):
        self.e_stop_active = False
        with self.bullet_lock:
            if self.obstacle_id is not None:
                p.removeBody(self.obstacle_id)
                self.obstacle_id = None
        with self.gpu_state_lock:
            self.gpu_shared_state['obstacle'] = None
        with self.data_lock:
            for i in range(len(self.slider_vars)):
                self.slider_vars[i].set(0.0)
                if i < len(self.shared_targets):
                    self.shared_targets[i] = 0.0
        with self.bullet_lock:
            for i, joint_idx in enumerate(self.revolute_joints):
                p.resetJointState(self.robotId, joint_idx, 0.0)

        self.status_message = "SİSTEM SIFIRLANDI. Güvenlik için engel silindi, Base'e dönüldü."

    # ==========================================
    # ŞEFFAF İZ (SWEPT VOLUME) ÇİZİMİ
    # ==========================================
    def toggle_workspace(self):
        with self.bullet_lock:
            self.show_workspace = not self.show_workspace
            if not self.show_workspace:
                for wid in self.workspace_ids:
                    p.removeBody(wid)
                self.workspace_ids = []
                self.btn_nav_workspace.configure(text="Saydam İzi Göster")
            else:
                self.update_link_positions()
                self.btn_nav_workspace.configure(text="Saydam İzi Temizle")

    def update_link_positions(self):
        for joint_idx in self.revolute_joints:
            state = p.getLinkState(self.robotId, joint_idx)
            self.last_link_positions[joint_idx] = state[4]

    def draw_swept_volume(self):
        if not self.show_workspace: return
        ghost_color = [0.0, 0.7, 1.0, 0.25]

        for joint_idx in [self.revolute_joints[-2], self.revolute_joints[-1]]:
            state = p.getLinkState(self.robotId, joint_idx)
            current_pos = state[4]
            last_pos = self.last_link_positions.get(joint_idx, current_pos)

            dist = math.sqrt((current_pos[0] - last_pos[0]) ** 2 +
                             (current_pos[1] - last_pos[1]) ** 2 +
                             (current_pos[2] - last_pos[2]) ** 2)

            if dist > 0.015:
                mid_x = (current_pos[0] + last_pos[0]) / 2.0
                mid_y = (current_pos[1] + last_pos[1]) / 2.0
                mid_z = (current_pos[2] + last_pos[2]) / 2.0

                diff = [current_pos[0] - last_pos[0], current_pos[1] - last_pos[1], current_pos[2] - last_pos[2]]
                pitch = -math.asin(diff[2] / dist) if dist != 0 else 0
                yaw = math.atan2(diff[1], diff[0])
                ori = p.getQuaternionFromEuler([0, pitch, yaw])

                v_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[dist / 2, 0.02, 0.01], rgbaColor=ghost_color)
                wid = p.createMultiBody(baseVisualShapeIndex=v_shape, basePosition=[mid_x, mid_y, mid_z],
                                        baseOrientation=ori)
                self.workspace_ids.append(wid)
                self.last_link_positions[joint_idx] = current_pos

    # ==========================================
    # SAYFA TASARIMLARI
    # ==========================================
    def make_scroll_smooth(self, scrollable_frame, step_px=50):
        """
        CTkScrollableFrame ve içindeki tüm alt bileşenlere (slider, label vb.)
        akıcı, kesintisiz ve takılmayan MouseWheel kaydırma desteği bağlar.
        """
        def _on_wheel(event):
            try:
                canvas = getattr(scrollable_frame, '_parent_canvas', None)
                if canvas is not None:
                    delta = getattr(event, 'delta', 0)
                    if delta != 0:
                        units = -int((delta / 120.0) * step_px)
                        canvas.yview_scroll(units, "units")
            except Exception:
                pass
            return "break"  # Global CTk çakışmasını ve takılmaları önler

        def _bind_all_children(widget):
            widget.bind("<MouseWheel>", _on_wheel, add="+")
            for child in widget.winfo_children():
                _bind_all_children(child)

        _bind_all_children(scrollable_frame)
        canvas = getattr(scrollable_frame, '_parent_canvas', None)
        if canvas is not None:
            canvas.bind("<MouseWheel>", _on_wheel, add="+")

    def setup_manual_page(self):
        title = ctk.CTkLabel(self.page_manual, text="Eksen Manipülasyonu", font=ctk.CTkFont(size=18, weight="bold"))
        title.pack(pady=(10, 15))

        btn_card = ctk.CTkFrame(self.page_manual, fg_color="#242424", corner_radius=10)
        btn_card.pack(fill="x", pady=(0, 15))

        self.btn_base = ctk.CTkButton(btn_card, text="Base Location", height=35, corner_radius=6,
                                      command=self.go_to_base)
        self.btn_base.pack(pady=(15, 10), padx=15, fill="x")

        nav_frame = ctk.CTkFrame(btn_card, fg_color="transparent")
        nav_frame.pack(pady=(0, 15), padx=15, fill="x")
        nav_frame.grid_columnconfigure(0, weight=1, uniform="group1")
        nav_frame.grid_columnconfigure(1, weight=1, uniform="group1")

        self.btn_undo = ctk.CTkButton(nav_frame, text="Önceki Konum", height=35, corner_radius=6,
                                      command=self.undo_position, fg_color="#4A4A4A", hover_color="#3A3A3A")
        self.btn_undo.grid(row=0, column=0, padx=(0, 5), sticky="ew")

        self.btn_redo = ctk.CTkButton(nav_frame, text="Sonraki Konum", height=35, corner_radius=6,
                                      command=self.redo_position)
        self.btn_redo.grid(row=0, column=1, padx=(5, 0), sticky="ew")

        self.slider_card = ctk.CTkScrollableFrame(self.page_manual, fg_color="#242424", corner_radius=10)
        self.slider_card.pack(fill="both", expand=True)

        self.sliders = []
        self.slider_vars = []
        self.rebuild_joint_sliders()

    def rebuild_joint_sliders(self):
        if not hasattr(self, 'slider_card'):
            return

        for widget in self.slider_card.winfo_children():
            widget.destroy()

        self.sliders = []
        self.slider_vars = []

        if not hasattr(self, 'revolute_joints') or not self.revolute_joints:
            empty_lbl = ctk.CTkLabel(self.slider_card, text="Aktif eklem bulunamadı.", font=ctk.CTkFont(size=12))
            empty_lbl.pack(pady=20)
            return

        for i, joint_idx in enumerate(self.revolute_joints):
            info = p.getJointInfo(self.robotId, joint_idx)
            raw_name = info[1].decode('utf-8')
            low = float(info[8])
            high = float(info[9])

            if low >= high or (abs(low) < 1e-5 and abs(high) < 1e-5):
                low, high = -math.pi, math.pi

            deg_low = math.degrees(low)
            deg_high = math.degrees(high)

            lbl_text = f"Eksen J{i+1}: {raw_name} ({deg_low:.0f}° .. {deg_high:.0f}°)"
            label = ctk.CTkLabel(self.slider_card, text=lbl_text, font=ctk.CTkFont(size=12, weight="bold"))
            label.pack(pady=(10, 0), padx=20, anchor="w")

            var = ctk.DoubleVar(value=0.0)
            self.slider_vars.append(var)

            slider = ctk.CTkSlider(
                self.slider_card, from_=low, to=high, variable=var,
                button_color="#00FFCC", button_hover_color="#00CCAA", border_width=2,
                border_color="#333"
            )
            slider.pack(pady=(2, 8), padx=20, fill="x")
            slider.bind("<ButtonRelease-1>", lambda event: self.save_position())
            self.sliders.append(slider)

        self.make_scroll_smooth(self.slider_card, step_px=50)

    def setup_fk_page(self):
        title = ctk.CTkLabel(self.page_fk, text="İleri Kinematik Modülü", font=ctk.CTkFont(size=18, weight="bold"))
        title.pack(pady=(10, 20))
        info_label = ctk.CTkLabel(self.page_fk,
                                  text="Mevcut eklem açılarını kullanarak Uç İşlevci\n(End-Effector) konumunu ve Dönüşüm Matrisini\n(T_0^6) hesaplar.",
                                  text_color="#AAAAAA", justify="left")
        info_label.pack(pady=(0, 20))
        self.btn_calc_fk = ctk.CTkButton(self.page_fk, text="D-H Tablosunu ve Matrisi Hesapla", height=42,
                                         corner_radius=8, command=self.calculate_fk)
        self.btn_calc_fk.pack(pady=10, fill="x")

    def setup_ik_page(self):
        title = ctk.CTkLabel(self.page_ik, text="Ters Kinematik (IK)",
                             font=ctk.CTkFont(size=16, weight="bold"))
        title.pack(pady=(10, 12))

        ik_card = ctk.CTkFrame(self.page_ik, fg_color="#242424", corner_radius=10)
        ik_card.pack(fill="x", pady=(0, 10))

        # --- SINIR GÖSTERGELERİ (TEXTBOX YANINDA) ---
        self.var_x = ctk.StringVar()
        self.var_y = ctk.StringVar()
        self.var_z = ctk.StringVar()
        self.var_x.trace_add("write", self.update_limits)
        self.var_y.trace_add("write", self.update_limits)
        self.var_z.trace_add("write", self.update_limits)

        # X Kutusu
        frame_x = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_x.pack(fill="x", padx=15, pady=(15, 5))
        self.entry_x = ctk.CTkEntry(frame_x, placeholder_text="Hedef X (cm)", textvariable=self.var_x,
                                    height=38, border_color="#444")
        self.entry_x.pack(side="left", fill="x", expand=True)
        self.lbl_limit_x = ctk.CTkLabel(frame_x, text="±80 cm", text_color="gray", width=100, font=ctk.CTkFont(size=11), anchor="e")
        self.lbl_limit_x.pack(side="right", padx=(8, 0))

        # Y Kutusu
        frame_y = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_y.pack(fill="x", padx=15, pady=5)
        self.entry_y = ctk.CTkEntry(frame_y, placeholder_text="Hedef Y (cm)", textvariable=self.var_y,
                                    height=38, border_color="#444")
        self.entry_y.pack(side="left", fill="x", expand=True)
        self.lbl_limit_y = ctk.CTkLabel(frame_y, text="±80 cm", text_color="gray", width=100, font=ctk.CTkFont(size=11), anchor="e")
        self.lbl_limit_y.pack(side="right", padx=(8, 0))

        # Z Kutusu
        frame_z = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_z.pack(fill="x", padx=15, pady=5)
        self.entry_z = ctk.CTkEntry(frame_z, placeholder_text="Hedef Z (cm)", textvariable=self.var_z,
                                    height=38, border_color="#444")
        self.entry_z.pack(side="left", fill="x", expand=True)
        self.lbl_limit_z = ctk.CTkLabel(frame_z, text="0 - 106 cm", text_color="gray", width=100, font=ctk.CTkFont(size=11), anchor="e")
        self.lbl_limit_z.pack(side="right", padx=(8, 0))

        # 3B Anlık Erişim Bilgilendirme Çubuğu
        self.lbl_reach_status = ctk.CTkLabel(
            ik_card, text="● Hedef koordinatları girin\n3B uzanma analizi bekleniyor",
            font=ctk.CTkFont(size=11), text_color="gray",
            anchor="w", justify="left"
        )
        self.lbl_reach_status.pack(fill="x", pady=(4, 10), padx=15)

        # --- MOTOR, YÜK & MALİYET AYARLARI KARTI ---
        motor_card = ctk.CTkFrame(self.page_ik, fg_color="#242424", corner_radius=10)
        motor_card.pack(fill="x", pady=(0, 10))

        lbl_motor_title = ctk.CTkLabel(
            motor_card, text="⚙️ YÜK VE ENERJİ AYARLARI",
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color="#FFB300"
        )
        lbl_motor_title.pack(pady=(8, 4), padx=15, anchor="w")

        row1 = ctk.CTkFrame(motor_card, fg_color="transparent")
        row1.pack(fill="x", padx=15, pady=(2, 4))
        row1.grid_columnconfigure(0, weight=1)
        row1.grid_columnconfigure(1, weight=1)

        # Yük (Payload)
        p_frame = ctk.CTkFrame(row1, fg_color="transparent")
        p_frame.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        ctk.CTkLabel(p_frame, text="Yük (Payload - kg):", font=ctk.CTkFont(size=11), text_color="#AAAAAA").pack(anchor="w")
        self.entry_payload = ctk.CTkEntry(p_frame, placeholder_text="0.0", height=32)
        self.entry_payload.insert(0, "0.0")
        self.entry_payload.pack(fill="x", pady=(2, 0))

        # Elektrik Birim Fiyatı (TL/kWh)
        t_frame = ctk.CTkFrame(row1, fg_color="transparent")
        t_frame.grid(row=0, column=1, sticky="ew", padx=(5, 0))
        ctk.CTkLabel(t_frame, text="Tarife (TL / kWh):", font=ctk.CTkFont(size=11), text_color="#AAAAAA").pack(anchor="w")
        self.entry_tariff = ctk.CTkEntry(t_frame, placeholder_text="4.50", height=32)
        self.entry_tariff.insert(0, "4.50")
        self.entry_tariff.pack(fill="x", pady=(2, 0))

        row2 = ctk.CTkFrame(motor_card, fg_color="transparent")
        row2.pack(fill="x", padx=15, pady=(2, 8))
        row2.grid_columnconfigure(0, weight=1)
        row2.grid_columnconfigure(1, weight=1)

        # Motor Verimi (%)
        eff_frame = ctk.CTkFrame(row2, fg_color="transparent")
        eff_frame.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        ctk.CTkLabel(eff_frame, text="Motor Verimi (%):", font=ctk.CTkFont(size=11), text_color="#AAAAAA").pack(anchor="w")
        self.entry_efficiency = ctk.CTkEntry(eff_frame, placeholder_text="85", height=32)
        self.entry_efficiency.insert(0, "85")
        self.entry_efficiency.pack(fill="x", pady=(2, 0))

        # Yükü Uygula Butonu
        self.btn_apply_payload = ctk.CTkButton(
            row2, text="⚡ Güncelle", height=32, corner_radius=6,
            fg_color="#332B10", hover_color="#4A3E14", text_color="#FFD700",
            font=ctk.CTkFont(size=11, weight="bold"),
            command=self.apply_payload_settings
        )
        self.btn_apply_payload.grid(row=0, column=1, sticky="se", padx=(5, 0))

        self.btn_calc_ik = ctk.CTkButton(self.page_ik, text="Yörünge ve Rotayı Hesapla", height=42, corner_radius=8,
                                         command=self.calculate_ik)
        self.btn_calc_ik.pack(pady=(5, 10), fill="x")

        self.traj_frame = ctk.CTkFrame(self.page_ik, fg_color="transparent")
        self.traj_frame.pack(fill="x", padx=10, pady=(5, 15))
        self.traj_frame.grid_columnconfigure(0, weight=1)
        self.traj_frame.grid_columnconfigure(1, weight=1)

        self.btn_fast_traj = ctk.CTkButton(self.traj_frame, text="⚡ Hızlı Rota", height=38, corner_radius=6,
                                           font=ctk.CTkFont(size=12, weight="bold"),
                                           fg_color="#8B0000", hover_color="#660000", state="disabled",
                                           command=lambda: self.execute_trajectory("fast"))
        self.btn_fast_traj.grid(row=0, column=0, padx=(0, 4), sticky="ew")

        self.btn_eco_traj = ctk.CTkButton(self.traj_frame, text="🌱 Eko Rota", height=38, corner_radius=6,
                                          font=ctk.CTkFont(size=12, weight="bold"),
                                          fg_color="#006400", hover_color="#004d00", state="disabled",
                                          command=lambda: self.execute_trajectory("eco"))
        self.btn_eco_traj.grid(row=0, column=1, padx=(4, 0), sticky="ew")

        step_title = ctk.CTkLabel(self.page_ik, text="Manevra Hamleleri", font=ctk.CTkFont(size=14, weight="bold"))
        step_title.pack(pady=(10, 5))
        self.ik_steps_frame = ctk.CTkFrame(self.page_ik, fg_color="#242424", corner_radius=10)
        self.ik_steps_frame.pack(fill="x", padx=10, pady=(0, 15))
        self.make_scroll_smooth(self.page_ik, step_px=50)

    # --- ÇALIŞMA ALANI LİMİT VE CANLI ERİŞİM GÜNCELLEYİCİ ---
    def update_limits(self, *args):
        try:
            x_str = self.var_x.get().strip().replace(',', '.')
            x_val = float(x_str) if x_str else None
        except Exception:
            x_val = None

        try:
            y_str = self.var_y.get().strip().replace(',', '.')
            y_val = float(y_str) if y_str else None
        except Exception:
            y_val = None

        try:
            z_str = self.var_z.get().strip().replace(',', '.')
            z_val = float(z_str) if z_str else None
        except Exception:
            z_val = None

        s_r = float(self.__dict__.get('shoulder_r_cm', 7.5))
        s_z = float(self.__dict__.get('shoulder_z_cm', 33.0))
        arm_len = float(self.__dict__.get('arm_length_cm', 72.8))
        stand_top = float(self.__dict__.get('stand_top_cm', 0.0))
        stand_half = float(self.__dict__.get('stand_half_extent_cm', 0.0))
        max_r = float(self.__dict__.get('robot_reach_cm', s_r + arm_len))

        min_floor = max(0.0, stand_top + (5.0 if stand_half > 0 else 0.0))
        global_max_z = s_z + arm_len

        # 1. X Sınırı: Sabit maksimum menzil
        if hasattr(self, 'lbl_limit_x'):
            self.lbl_limit_x.configure(text=f"±{max_r:.0f} cm", text_color="gray")

        # 2. Y Sınırı: X'e göre dinamik (X arttıkça izin verilen Y daralır)
        if hasattr(self, 'lbl_limit_y'):
            if x_val is not None:
                if abs(x_val) > max_r:
                    self.lbl_limit_y.configure(text="Menzil Dışı", text_color="#FF4444")
                else:
                    max_y = math.sqrt(max(0.0, max_r ** 2 - x_val ** 2))
                    self.lbl_limit_y.configure(text=f"±{max_y:.0f} cm", text_color="gray")
            else:
                self.lbl_limit_y.configure(text=f"±{max_r:.0f} cm", text_color="gray")

        # 3. Z Sınırı: X ve Y koordinatlarına göre dinamik (Yarıçap arttıkça dikey erişim küresi daralır)
        if hasattr(self, 'lbl_limit_z'):
            if x_val is not None or y_val is not None:
                x_eval = x_val if x_val is not None else 0.0
                y_eval = y_val if y_val is not None else 0.0
                r_xy = math.sqrt(x_eval ** 2 + y_eval ** 2)

                if r_xy > (max_r * 1.01):
                    self.lbl_limit_z.configure(text="Menzil Dışı", text_color="#FF4444")
                else:
                    d_horiz = max(0.0, r_xy - s_r)
                    if d_horiz > arm_len:
                        self.lbl_limit_z.configure(text="Menzil Dışı", text_color="#FF4444")
                    else:
                        dz = math.sqrt(max(0.0, arm_len ** 2 - d_horiz ** 2))
                        z_max = s_z + dz
                        z_min = s_z - dz
                        z_min_disp = max(min_floor, z_min)

                        if z_min_disp > z_max:
                            self.lbl_limit_z.configure(text="Menzil Dışı", text_color="#FF4444")
                        else:
                            self.lbl_limit_z.configure(text=f"{z_min_disp:.0f} - {z_max:.0f} cm", text_color="gray")
            else:
                self.lbl_limit_z.configure(text=f"{min_floor:.0f} - {global_max_z:.0f} cm", text_color="gray")

        # 4. Canlı 3B Erişim Analizi (Kullanıcı koordinat yazdıkça rehberlik)
        if hasattr(self, 'lbl_reach_status'):
            if x_val is not None and y_val is not None and z_val is not None:
                r_xy = math.sqrt(x_val ** 2 + y_val ** 2)
                d_horiz = max(0.0, r_xy - s_r)
                dist_3d = math.sqrt(d_horiz ** 2 + (z_val - s_z) ** 2)

                is_below_floor = (stand_half > 0 and abs(x_val) <= stand_half + 5.0 and abs(y_val) <= stand_half + 5.0 and z_val < min_floor) or (z_val < 0.0)

                if is_below_floor:
                    self.lbl_reach_status.configure(
                        text=f"⚠️ Hedef taban seviyesinin altında\nZ: {z_val:.0f} cm  (Min Güvenli: {min_floor:.0f} cm)",
                        text_color="#FF5555"
                    )
                elif dist_3d <= arm_len:
                    self.lbl_reach_status.configure(
                        text=f"● Hedef erişilebilir\nMesafe: {dist_3d:.1f} cm  |  Sınır: {arm_len:.1f} cm",
                        text_color="#00FFCC"
                    )
                else:
                    diff = dist_3d - arm_len
                    self.lbl_reach_status.configure(
                        text=f"⚠️ Hedef erişim sınırı dışında\nMesafe: {dist_3d:.1f} cm  |  Aşım: +{diff:.1f} cm",
                        text_color="#FFB300"
                    )
            elif x_val is not None or y_val is not None or z_val is not None:
                self.lbl_reach_status.configure(
                    text="● Hedef koordinatlarını tamamlayın\nX, Y ve Z değerlerini girin",
                    text_color="#AAAAAA"
                )
            else:
                self.lbl_reach_status.configure(
                    text="● Hedef koordinatları girin\n3B uzanma analizi bekleniyor",
                    text_color="gray"
                )

    def apply_payload_settings(self):
        try:
            val_p = self.entry_payload.get().strip().replace(',', '.')
            self.payload_kg = max(0.0, float(val_p)) if val_p else 0.0
        except Exception:
            self.payload_kg = 0.0

        try:
            val_t = self.entry_tariff.get().strip().replace(',', '.')
            self.electricity_rate = max(0.01, float(val_t)) if val_t else 4.50
        except Exception:
            self.electricity_rate = 4.50

        try:
            val_e = self.entry_efficiency.get().strip().replace(',', '.')
            eff_pct = max(10.0, min(100.0, float(val_e))) if val_e else 85.0
            self.motor_efficiency = eff_pct / 100.0
        except Exception:
            self.motor_efficiency = 0.85

        with self.bullet_lock:
            if hasattr(self, 'robotId') and self.robotId is not None and hasattr(self, 'end_effector_index'):
                base_m = getattr(self, 'base_ee_mass', 1.0)
                p.changeDynamics(self.robotId, self.end_effector_index, mass=base_m + self.payload_kg)

        self.status_message = f"Yük Güncellendi: {self.payload_kg:.2f} kg | Verim: %{self.motor_efficiency*100:.0f} | Tarife: {self.electricity_rate:.2f} TL/kWh"

    def reset_energy_counter(self):
        self.cumulative_energy_joules = 0.0
        self.cumulative_energy_kwh = 0.0
        self.cumulative_cost_tl = 0.0
        if hasattr(self, 'lbl_energy_summary'):
            self.lbl_energy_summary.configure(text="Anlık: 0.0 W  |  Sayaç: 0.000 kWh  |  Maliyet: 0.00 TL")
        self.status_message = "Enerji ve maliyet sayacı sıfırlandı."

    def show_page(self, page_name):
        self.page_manual.pack_forget()
        self.page_fk.pack_forget()
        self.page_ik.pack_forget()
        self.current_page = page_name

        if page_name == "manual":
            self.page_manual.pack(fill="both", expand=True)
        elif page_name == "fk":
            self.page_fk.pack(fill="both", expand=True)
        elif page_name == "ik":
            self.page_ik.pack(fill="both", expand=True)

    # ==========================================
    # KONFİGÜRASYON VE HAFIZA SİSTEMİ
    # ==========================================
    def get_config_path(self):
        return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json"))

    def load_config(self):
        cfg_path = self.get_config_path()
        if os.path.isfile(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                pass
        return {"default_model_path": None, "recent_models": []}

    def save_config(self, cfg):
        cfg_path = self.get_config_path()
        try:
            with open(cfg_path, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=4, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"[Config] Kaydetme hatası: {e}")
            return False

    def get_model_name_from_path(self, path):
        if not path:
            return "Bilinmeyen Model"
        base = os.path.splitext(os.path.basename(path))[0]
        for pfx in [".", "sanitized_", "compiled_"]:
            if base.startswith(pfx):
                base = base[len(pfx):]
        if base.endswith("_generated"):
            base = base[:-10]
        return base

    def update_default_model_label(self):
        if not hasattr(self, 'lbl_default_info'):
            return
        cfg = self.load_config()
        saved_default = cfg.get("default_model_path")
        if saved_default and os.path.isfile(saved_default):
            name = self.get_model_name_from_path(saved_default)
            if len(name) > 16:
                name = name[:14] + ".."
            self.lbl_default_info.configure(text=f"Açılış: {name} ⭐", text_color="#FFD700")
        else:
            self.lbl_default_info.configure(text="Açılış: Fanuc (Dahili)", text_color="#777777")

    # ==========================================
    # KİNEMATİK VE PYBULLET SİSTEMİ
    # ==========================================
    def init_pybullet(self):
        self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.builtin_urdf_path = os.path.normpath(os.path.join(
            os.path.dirname(__file__), "..", "fanuc_lrmate200ic_support", "urdf", "fanuc_lrmate200ic.urdf"
        ))
        self.default_urdf_path = self.builtin_urdf_path
        self.current_source_path = self.builtin_urdf_path
        self.current_urdf_path = self.builtin_urdf_path
        self.visual_shapes_meta = []
        self.render_link_names = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']

        # Kayıtlı ana model kontrolü
        loaded = False
        cfg = self.load_config()
        saved_default = cfg.get("default_model_path")
        if saved_default and os.path.isfile(saved_default):
            try:
                loaded = self.load_source_model(saved_default, initial=True)
            except Exception as e:
                print(f"[Init] Kayıtlı ana model yüklenirken hata: {e}")
                loaded = False

        if not loaded:
            self.load_source_model(self.builtin_urdf_path, initial=True)

        self.update_default_model_label()

    def find_package_dir(self, pkg_name, context_path=None):
        if not pkg_name:
            return None
        if not hasattr(self, 'known_packages'):
            self.known_packages = {}

        cache_key = (pkg_name.lower(), str(context_path))
        if cache_key in self.known_packages:
            return self.known_packages[cache_key]

        app_dir = os.path.dirname(os.path.abspath(__file__))
        sim_root = os.path.normpath(os.path.join(app_dir, ".."))
        downloads_dir = os.path.expanduser("~/Downloads")

        # 1. context_path hiyerarşisinde yukarı doğru ara (6 seviye)
        if context_path and os.path.exists(context_path):
            curr = os.path.abspath(context_path) if os.path.isdir(context_path) else os.path.dirname(os.path.abspath(context_path))
            for _ in range(6):
                if os.path.basename(curr).lower() == pkg_name.lower():
                    self.known_packages[cache_key] = curr
                    return curr
                cand = os.path.join(curr, pkg_name)
                if os.path.isdir(cand):
                    self.known_packages[cache_key] = cand
                    return cand
                pkg_xml = os.path.join(curr, "package.xml")
                if os.path.isfile(pkg_xml):
                    try:
                        with open(pkg_xml, 'r', encoding='utf-8', errors='ignore') as f:
                            if f"<name>{pkg_name}</name>" in f.read():
                                self.known_packages[cache_key] = curr
                                return curr
                    except Exception:
                        pass
                # Üst klasördeki kardeş klasörleri tara
                parent = os.path.dirname(curr)
                if os.path.isdir(parent):
                    for entry in os.listdir(parent):
                        cand_sib = os.path.join(parent, entry)
                        if os.path.isdir(cand_sib) and entry.lower() == pkg_name.lower():
                            self.known_packages[cache_key] = cand_sib
                            return cand_sib
                if parent == curr:
                    break
                curr = parent

        # 2. Downloads, sim_root/resources ve sim_root kontrolü
        for root_dir in [downloads_dir, os.path.join(sim_root, "resources"), sim_root]:
            if not os.path.isdir(root_dir):
                continue
            cand = os.path.join(root_dir, pkg_name)
            if os.path.isdir(cand):
                self.known_packages[cache_key] = cand
                return cand
            try:
                for entry in os.listdir(root_dir):
                    sub = os.path.join(root_dir, entry)
                    if os.path.isdir(sub):
                        if entry.lower() == pkg_name.lower():
                            self.known_packages[cache_key] = sub
                            return sub
                        cand_sub = os.path.join(sub, pkg_name)
                        if os.path.isdir(cand_sub):
                            self.known_packages[cache_key] = cand_sub
                            return cand_sub
            except Exception:
                pass

        self.known_packages[cache_key] = None
        return None

    def resolve_mesh_path(self, urdf_dir, mesh_filename, context_dir=None):
        if not mesh_filename:
            return None
        if isinstance(mesh_filename, bytes):
            mesh_filename = mesh_filename.decode('utf-8')
        if os.path.isabs(mesh_filename) and os.path.exists(mesh_filename):
            return os.path.abspath(mesh_filename).replace('\\', '/')

        clean_path = mesh_filename
        pkg_name = ''
        for prefix in ["package://", "model://", "file://"]:
            if clean_path.startswith(prefix):
                clean_path = clean_path[len(prefix):]
                parts = clean_path.replace("\\", "/").split("/")
                if len(parts) > 1:
                    pkg_name = parts[0]
                    clean_path = "/".join(parts[1:])
                break

        # 1. Eğer paket adı tanımlıysa, ilgili paketin kökünü çöz ve SADECE bu paket içinde ara
        if pkg_name:
            pkg_dir = self.find_package_dir(pkg_name, context_path=context_dir or urdf_dir)
            if pkg_dir:
                p_cand = os.path.normpath(os.path.join(pkg_dir, clean_path))
                if os.path.isfile(p_cand):
                    return os.path.abspath(p_cand).replace('\\', '/')
                # Paket kökü altında dosya adına göre ara (Asla başka paketlerin parçalarını karıştırmaz!)
                base_name = os.path.basename(clean_path)
                for root, dirs, files in os.walk(pkg_dir):
                    if base_name in files:
                        return os.path.abspath(os.path.join(root, base_name)).replace('\\', '/')

        # 2. Yerel dosya yolları (context_dir ve urdf_dir hiyerarşisi)
        search_dirs = []
        if context_dir and os.path.isdir(context_dir):
            search_dirs += [
                context_dir,
                os.path.normpath(os.path.join(context_dir, "..")),
                os.path.normpath(os.path.join(context_dir, "meshes")),
                os.path.normpath(os.path.join(context_dir, "..", "meshes")),
                os.path.normpath(os.path.join(context_dir, "..", "..")),
            ]
        if urdf_dir and os.path.isdir(urdf_dir):
            search_dirs += [
                urdf_dir,
                os.path.normpath(os.path.join(urdf_dir, "..")),
                os.path.normpath(os.path.join(urdf_dir, "meshes")),
                os.path.normpath(os.path.join(urdf_dir, "..", "meshes")),
                os.path.normpath(os.path.join(urdf_dir, "..", "..")),
            ]

        for cdir in search_dirs:
            p_check = os.path.normpath(os.path.join(cdir, clean_path))
            if os.path.isfile(p_check):
                return os.path.abspath(p_check).replace('\\', '/')

        # 3. Kendi kaynak klasöründe ara (Eğer paket adı belirtilmemişse)
        if not pkg_name and context_dir and os.path.isdir(context_dir):
            base_name = os.path.basename(mesh_filename)
            for root, dirs, files in os.walk(context_dir):
                if base_name in files:
                    return os.path.abspath(os.path.join(root, base_name)).replace('\\', '/')

        return None

    def sanitize_and_resolve_urdf(self, urdf_path, context_dir=None):
        """
        PyBullet C++ URDF ayrıştırıcısının 'cannot find mesh' veya 'Error=XML_ERROR_PARSING_ATTRIBUTE'
        hatalarıyla çökmesini önler. Tüm mesh yollarını mutlak yola dönüştürür; bulunamayan mesh'ler
        için güvenli geometrik ilkel yerleştirir ve XML'i düzgün satır aralıklarıyla kaydeder.
        """
        try:
            import xml.etree.ElementTree as ET
            app_dir = os.path.dirname(os.path.abspath(__file__))
            sim_root = os.path.normpath(os.path.join(app_dir, ".."))
            cache_dir = os.path.join(sim_root, ".cache")
            os.makedirs(cache_dir, exist_ok=True)

            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            for geom in root.iter("geometry"):
                mesh = geom.find("mesh")
                if mesh is not None:
                    fn = mesh.attrib.get("filename", "")
                    resolved = self.resolve_mesh_path(urdf_dir, fn, context_dir=context_dir)
                    if resolved and os.path.isfile(resolved):
                        mesh.attrib["filename"] = os.path.abspath(resolved).replace("\\", "/")
                    else:
                        # Eksik mesh için PyBullet çökmesini önleyen yedek silindir
                        geom.remove(mesh)
                        cyl = ET.SubElement(geom, "cylinder")
                        cyl.attrib["radius"] = "0.04"
                        cyl.attrib["length"] = "0.18"

            base_name = os.path.splitext(os.path.basename(urdf_path))[0]
            if base_name.startswith("."):
                base_name = base_name[1:]
            sanitized_path = os.path.join(cache_dir, f"sanitized_{base_name}.urdf")
            ET.indent(tree, space="  ")
            tree.write(sanitized_path, encoding="utf-8")
            return sanitized_path
        except Exception as e:
            print(f"[URDF Sanitize] Hata: {e}")
            return urdf_path

    def load_robot_model(self, urdf_path, initial=False, context_dir=None):
        if not os.path.isfile(urdf_path):
            if not initial:
                messagebox.showerror("Hata", f"Seçilen dosya bulunamadı:\n{urdf_path}")
            return False

        try:
            self.status_message = "Robot modeli işleniyor ve yükleniyor..."
            app_dir = os.path.dirname(os.path.abspath(__file__))
            sim_root = os.path.normpath(os.path.join(app_dir, ".."))
            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            if not context_dir:
                context_dir = getattr(self, 'current_source_dir', urdf_dir)

            # URDF'i PyBullet için sterilize et ve mesh yollarını bağla
            sanitized_urdf = self.sanitize_and_resolve_urdf(urdf_path, context_dir=context_dir)

            with self.bullet_lock:
                if hasattr(self, 'robotId') and self.robotId is not None:
                    try:
                        p.removeBody(self.robotId)
                    except Exception:
                        pass

                p.setAdditionalSearchPath(urdf_dir)
                if context_dir and os.path.isdir(context_dir):
                    p.setAdditionalSearchPath(context_dir)
                p.setAdditionalSearchPath(os.path.normpath(os.path.join(urdf_dir, "..")))
                p.setAdditionalSearchPath(os.path.join(sim_root, "resources"))
                p.setAdditionalSearchPath(sim_root)

                self.robotId = p.loadURDF(
                    sanitized_urdf,
                    [0, 0, 0],
                    p.getQuaternionFromEuler([0, 0, 0]),
                    useFixedBase=True
                )
                self.current_urdf_path = urdf_path

                num_joints = p.getNumJoints(self.robotId)
                self.revolute_joints = [
                    i for i in range(num_joints)
                    if p.getJointInfo(self.robotId, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, getattr(p, 'JOINT_CONTINUOUS', 0))
                ]
                if not self.revolute_joints:
                    self.revolute_joints = [i for i in range(num_joints) if p.getJointInfo(self.robotId, i)[2] != p.JOINT_FIXED]
                if not self.revolute_joints:
                    self.revolute_joints = list(range(min(6, num_joints)))

                self.link_joint_map = {}
                self.link_names = {-1: "base_link"}
                for ji in range(num_joints):
                    info = p.getJointInfo(self.robotId, ji)
                    child_link_name = info[12].decode('utf-8')
                    self.link_joint_map[child_link_name] = ji
                    self.link_names[ji] = child_link_name

                # Kaide (Pedestal/Stand) ve Zemin geometrisi tespiti
                stand_link_idx = None
                for i in range(num_joints):
                    name = p.getJointInfo(self.robotId, i)[12].decode('utf-8').lower()
                    if any(s in name for s in ['stand', 'table', 'pedestal', 'sehp', 'kaide']):
                        stand_link_idx = i
                        break

                self.stand_top_cm = 0.0
                self.stand_half_extent_cm = 0.0
                if stand_link_idx is not None:
                    aabb_min, aabb_max = p.getAABB(self.robotId, stand_link_idx)
                    self.stand_top_cm = max(0.0, aabb_max[2] * 100.0)
                    self.stand_half_extent_cm = max(abs(aabb_max[0]), abs(aabb_max[1])) * 100.0
                    self.min_safe_z_cm = self.stand_top_cm + 5.0
                else:
                    self.min_safe_z_cm = 5.0

                first_revolute = self.revolute_joints[0] if self.revolute_joints else 0
                shoulder_st = p.getLinkState(self.robotId, first_revolute)
                self.base_z_cm = shoulder_st[4][2] * 100.0

                # Akıllı Uç İşlevci (TCP / End-Effector) tespiti:
                # Gripper parmak kızakları yerine asıl TCP/tool0 veya son revolute mafsalı seç
                ee_candidates = []
                for ji in range(num_joints):
                    info = p.getJointInfo(self.robotId, ji)
                    j_name = info[1].decode('utf-8').lower()
                    c_name = info[12].decode('utf-8').lower()
                    if any(k in c_name or k in j_name for k in ['tcp', 'tool0', 'ee_link', 'flange', 'end_effector']):
                        ee_candidates.append(ji)

                if ee_candidates:
                    self.end_effector_index = ee_candidates[-1]
                else:
                    rev_only = [
                        i for i in self.revolute_joints
                        if p.getJointInfo(self.robotId, i)[2] in (p.JOINT_REVOLUTE, getattr(p, 'JOINT_CONTINUOUS', 0))
                    ]
                    self.end_effector_index = rev_only[-1] if rev_only else (self.revolute_joints[-1] if self.revolute_joints else 0)

                # Eklem sınırları ve kinematik parametreleri (IK için)
                self.joint_lower_limits = []
                self.joint_upper_limits = []
                self.joint_ranges = []
                self.joint_rest_poses = []

                for j_idx in self.revolute_joints:
                    info = p.getJointInfo(self.robotId, j_idx)
                    low = float(info[8])
                    high = float(info[9])
                    if low >= high:
                        low = -math.pi
                        high = math.pi
                    self.joint_lower_limits.append(low)
                    self.joint_upper_limits.append(high)
                    self.joint_ranges.append(high - low)
                    # Doğal dirsek yukarı (elbow-up) duruşu tercih etmek için rest pose
                    self.joint_rest_poses.append(0.0 if (low <= 0.0 <= high) else ((low + high) / 2.0))

                    # Eklemlere dinamik sönümleme (damping) uygula - titreşimi ve boşta jiggle'ı önler
                    try:
                        p.changeDynamics(self.robotId, j_idx, linearDamping=0.2, angularDamping=0.2, jointDamping=0.5)
                    except Exception:
                        pass

                # Başlangıç duruşundaki doğal temas ve bitişik eklem matrisi (Allowed Collision Matrix)
                self.allowed_collision_pairs = set()
                # 1. Bitişik kinematik ebeveyn-çocuk bağları daima izinlidir
                for ji in range(num_joints):
                    parent_idx = p.getJointInfo(self.robotId, ji)[16]
                    if parent_idx >= -1:
                        self.allowed_collision_pairs.add((min(parent_idx, ji), max(parent_idx, ji)))

                # 2. Başlangıç duruşundaki temas eden diğer doğal parçalar
                initial_contacts = p.getClosestPoints(self.robotId, self.robotId, distance=0.01)
                for c in initial_contacts:
                    lA, lB = c[3], c[4]
                    if lA != lB:
                        self.allowed_collision_pairs.add((min(lA, lB), max(lA, lB)))

                # Uç işlevci orijinal kütlesini al ve tanımlı yükü uygula
                try:
                    dyn = p.getDynamicsInfo(self.robotId, self.end_effector_index)
                    self.base_ee_mass = float(dyn[0]) if dyn else 1.0
                except Exception:
                    self.base_ee_mass = 1.0

                current_payload = getattr(self, 'payload_kg', 0.0)
                if current_payload > 0:
                    try:
                        p.changeDynamics(self.robotId, self.end_effector_index, mass=self.base_ee_mass + current_payload)
                    except Exception:
                        pass

                # Görsel şekilleri ve geometrileri çıkar
                try:
                    shapes = p.getVisualShapeData(self.robotId)
                except Exception:
                    shapes = []

                default_palette = [
                    (0.25, 0.25, 0.25),
                    (0.96, 0.76, 0.13),
                    (0.96, 0.76, 0.13),
                    (0.96, 0.76, 0.13),
                    (0.96, 0.76, 0.13),
                    (0.20, 0.20, 0.20),
                    (0.25, 0.25, 0.25),
                    (0.00, 0.65, 0.85),
                    (0.95, 0.35, 0.10)
                ]

                self.visual_shapes_meta = []
                mesh_data = []

                for idx, s in enumerate(shapes):
                    link_idx = s[1]
                    geom_type = s[2]
                    dims = s[3]
                    mesh_fn = s[4].decode('utf-8') if isinstance(s[4], bytes) else s[4]
                    resolved_mesh = self.resolve_mesh_path(urdf_dir, mesh_fn) if mesh_fn else None
                    local_pos = s[5]
                    local_ori = s[6]
                    rgba = s[7]

                    col = (float(rgba[0]), float(rgba[1]), float(rgba[2])) if (rgba and any(rgba[:3])) else None
                    if not col:
                        # Fanuc CR serisi için karakteristik yeşil/siyah teması
                        if 'cr35' in urdf_path.lower() or 'cr_' in urdf_path.lower():
                            col = (0.28, 0.61, 0.43) if link_idx >= 0 else (0.22, 0.22, 0.22)
                        else:
                            col = default_palette[idx % len(default_palette)]

                    self.visual_shapes_meta.append({
                        'link_idx': link_idx,
                        'geom_type': geom_type,
                        'dims': dims,
                        'local_pos': local_pos,
                        'local_ori': local_ori,
                    })

                    mesh_data.append({
                        'name': f'vis_{idx}_link_{link_idx}',
                        'mesh_path': resolved_mesh,
                        'color': col,
                        'geom_type': geom_type,
                        'dims': dims,
                    })

                self.render_link_names = [item['name'] for item in mesh_data]

                # ── OTOMATİK KAMERA VE MODEL ÖLÇEKLENDİRME (Auto-Framing) ──
                min_coords = [float('inf')] * 3
                max_coords = [float('-inf')] * 3
                for link_idx in [-1] + list(range(num_joints)):
                    try:
                        aabb_min, aabb_max = p.getAABB(self.robotId, link_idx)
                        for k in range(3):
                            min_coords[k] = min(min_coords[k], aabb_min[k])
                            max_coords[k] = max(max_coords[k], aabb_max[k])
                    except Exception:
                        pass

                span_x = max_coords[0] - min_coords[0]
                span_y = max_coords[1] - min_coords[1]
                span_z = max_coords[2] - min_coords[2]
                max_span = max(span_x, span_y, span_z)
                if max_span < 0.1 or max_span > 60.0 or math.isinf(max_span):
                    max_span = 0.8

                center_z = (max_coords[2] + min_coords[2]) / 2.0
                if math.isinf(center_z) or math.isnan(center_z):
                    center_z = 0.45

                auto_cam_dist = max(1.6, float(max_span * 2.2))
                auto_cam_target = np.array([0.0, 0.0, float(center_z)])
                auto_grid_size = max(1.5, float(max_span * 1.5))
                self.robot_reach_cm = max(35.0, float(max(span_x, span_y, span_z * 0.8)) * 100.0)

                # Kinematik omuz ve kol geometrisi
                rev = self.revolute_joints
                shoulder_idx = rev[1] if len(rev) > 1 else (rev[0] if rev else 0)
                ls_shoulder = p.getLinkState(self.robotId, shoulder_idx)
                s_pos = ls_shoulder[4]
                self.shoulder_r_cm = math.sqrt(s_pos[0]**2 + s_pos[1]**2) * 100.0
                self.shoulder_z_cm = s_pos[2] * 100.0
                self.base_z_cm = self.shoulder_z_cm

                # Omuzdan uç işlevciye kinematik zincir mesafesi
                arm_len_m = 0.0
                curr = getattr(self, 'end_effector_index', (rev[-1] if rev else 0))
                visited = set()
                while curr != shoulder_idx and curr >= 0 and curr not in visited:
                    visited.add(curr)
                    info = p.getJointInfo(self.robotId, curr)
                    arm_len_m += math.sqrt(sum(x**2 for x in info[14]))
                    curr = info[16]

                if arm_len_m > 0.1:
                    self.arm_length_cm = arm_len_m * 100.0 * 0.94
                    self.robot_reach_cm = self.shoulder_r_cm + self.arm_length_cm
                else:
                    self.arm_length_cm = self.robot_reach_cm * 0.9

            n_joints = len(self.revolute_joints)
            with self.data_lock:
                self.shared_targets = [0.0] * n_joints
                self.position_history = [[0.0] * n_joints]
                self.history_idx = 0

            with self.gpu_state_lock:
                self.gpu_shared_state['mesh_data'] = mesh_data
                self.gpu_shared_state['reload_meshes'] = True
                self.gpu_shared_state['transforms'] = [None] * len(self.visual_shapes_meta)
                self.gpu_shared_state['cam_dist'] = auto_cam_dist
                self.gpu_shared_state['cam_target'] = auto_cam_target
                self.gpu_shared_state['cam_yaw'] = 45.0
                self.gpu_shared_state['cam_pitch'] = -25.0
                self.gpu_shared_state['grid_size'] = auto_grid_size

            if hasattr(self, 'slider_card'):
                self.rebuild_joint_sliders()

            # IK Sınır etiketlerini güncelle
            self.update_limits()

            robot_name = os.path.splitext(os.path.basename(urdf_path))[0]
            for clean_pfx in [".", "sanitized_", "compiled_"]:
                if robot_name.startswith(clean_pfx):
                    robot_name = robot_name[len(clean_pfx):]
            if robot_name.endswith("_generated"):
                robot_name = robot_name[:-10]

            self.title(f"NexusControl Studio | Model: {robot_name} ({n_joints} Eksen)")
            self.status_message = f"Model Başarıyla Yüklendi: {robot_name} ({n_joints} Eksen, Boyut: {max_span:.2f}m)"
            return True

        except Exception as e:
            if not initial:
                messagebox.showerror("URDF Yükleme Hatası", f"Model yüklenirken bir hata oluştu:\n{str(e)}")
            self.status_message = f"HATA: Model yüklenemedi ({str(e)})"
            return False

    def convert_xacro_to_urdf(self, xacro_path):
        try:
            import xacro
        except ImportError:
            messagebox.showerror(
                "Eksik Modül",
                "Xacro dosyalarını dönüştürmek için 'xacro' kütüphanesi gereklidir.\n"
                "Terminalden 'pip install xacro' komutunu çalıştırabilirsiniz."
            )
            self.status_message = "HATA: 'xacro' modülü yüklü değil!"
            return None

        try:
            import sys
            import types
            import re

            self.status_message = "Xacro modeli derleniyor..."

            # ROS ortamı olmayan Windows sistemlerinde $(find ...) ve $(find-pkg-share ...)
            # paket çözümleme hatalarını önlemek için mock ament_index_python ve paket çözücü yapılandırılır
            def find_package_dir(pkg_name):
                found = self.find_package_dir(pkg_name, context_path=xacro_path)
                if found:
                    return found
                base_dir = os.path.dirname(os.path.abspath(xacro_path))
                return os.path.dirname(base_dir) if os.path.isdir(os.path.dirname(base_dir)) else base_dir

            # Mock ament_index_python
            if "ament_index_python" not in sys.modules:
                ament_mod = types.ModuleType("ament_index_python")
                pkg_mod = types.ModuleType("ament_index_python.packages")
                ament_mod.packages = pkg_mod
                sys.modules["ament_index_python"] = ament_mod
                sys.modules["ament_index_python.packages"] = pkg_mod

            sys.modules["ament_index_python.packages"].get_package_share_directory = find_package_dir

            # Xacro substitution_args monkey-patch
            try:
                import xacro.substitution_args as sa
                sa._eval_find = find_package_dir
                if hasattr(sa, "_resolve_args"):
                    orig_resolve = sa._resolve_args
                    def patched_resolve_args(arg_str, context, commands):
                        if "$(" in arg_str:
                            arg_str = re.sub(r"\$\(find-pkg-share\s+([^)]+)\)", r"$(find \1)", arg_str)
                            arg_str = re.sub(r"\$\(find-pkg-prefix\s+([^)]+)\)", r"$(find \1)", arg_str)
                        return orig_resolve(arg_str, context, commands)
                    sa._resolve_args = patched_resolve_args
            except Exception:
                pass

            # Akıllı include çözücü: Eğer include edilen dosya bulunamazsa yerel dizinde ara
            try:
                orig_abs = xacro.abs_filename_spec
                def smart_abs_filename_spec(filename_spec):
                    resolved = orig_abs(filename_spec)
                    if not os.path.exists(resolved):
                        target_name = os.path.basename(filename_spec)
                        base_dir = os.path.dirname(os.path.abspath(xacro_path))
                        app_dir = os.path.dirname(os.path.abspath(__file__))
                        sim_root = os.path.normpath(os.path.join(app_dir, ".."))

                        # Standart materyal/renk dosyaları için dahili kaynak kontrolü
                        if target_name in ["common_materials.xacro", "common_colours.xacro"]:
                            for r_dir in [
                                os.path.join(sim_root, "resources", "fanuc_resources", "urdf"),
                                os.path.join(sim_root, "resources", "abb_resources", "urdf")
                            ]:
                                cand = os.path.join(r_dir, target_name)
                                if os.path.isfile(cand):
                                    return cand

                        for search_root in [base_dir, os.path.dirname(base_dir), os.path.dirname(os.path.dirname(base_dir))]:
                            if not os.path.isdir(search_root):
                                continue
                            for root, dirs, files in os.walk(search_root):
                                if target_name in files:
                                    return os.path.join(root, target_name)
                                if root[len(search_root):].count(os.sep) >= 3:
                                    dirs.clear()
                    return resolved
                xacro.abs_filename_spec = smart_abs_filename_spec
            except Exception:
                pass

            doc = xacro.process_file(xacro_path)
            urdf_content = doc.toxml()

            app_dir = os.path.dirname(os.path.abspath(__file__))
            sim_root = os.path.normpath(os.path.join(app_dir, ".."))
            cache_dir = os.path.join(sim_root, ".cache")
            os.makedirs(cache_dir, exist_ok=True)

            file_name = os.path.splitext(os.path.basename(xacro_path))[0]
            out_path = os.path.join(cache_dir, f"compiled_{file_name}.urdf")

            with open(out_path, "w", encoding="utf-8") as f:
                f.write(urdf_content)

            return out_path
        except Exception as e:
            err_msg = str(e)
            missing_detail = ""
            if "No such file or directory" in err_msg or "FileNotFoundError" in str(type(e)):
                match = re.search(r"['\"]([^'\"]+\.xacro)['\"]", err_msg, re.IGNORECASE)
                if not match:
                    match = re.search(r"No such file or directory:\s*([^\r\n]+)", err_msg)
                missing_target = match.group(1).strip().strip("'\"") if match else "Bağımlı xacro dosyası"
                missing_filename = os.path.basename(missing_target)
                
                missing_detail = (
                    f"\n\n🔍 AÇIKLAMA:\n"
                    f"Seçtiğiniz Xacro dosyası başka bir robot paketine bağımlıdır (<xacro:include>):\n"
                    f"👉 Eksik Dosya / Paket: {missing_filename}\n\n"
                    f"Bu dosya bilgisayarınızda bulunamadı. Seçtiğiniz '.xacro' dosyası robotun gövdesini tek başına içermeyen, "
                    f"harici bir robot paketini (örneğin 'ros2srrc_robots' veya 'abb_irb4400_support') çağıran bir üst konfigürasyon dosyasıdır.\n\n"
                    f"💡 Çözüm:\n"
                    f"1. Robotun ana gövde makrolarını ve 3D mesh'lerini içeren tam robot paketini indirin, veya\n"
                    f"2. Robotun doğrudan derlenmiş saf '.urdf' dosyasını yükleyin."
                )

            messagebox.showerror(
                "Xacro Derleme Hatası (Eksik Bağımlılık)",
                f"Xacro dosyası işlenirken hata oluştu:\n{err_msg}{missing_detail}"
            )
            self.status_message = f"HATA: Xacro bağımlılığı eksik"
            return None

    def load_source_model(self, file_path, initial=False):
        if not file_path or not os.path.isfile(file_path):
            if not initial:
                messagebox.showerror("Hata", f"Seçilen model dosyası bulunamadı:\n{file_path}")
            return False

        self.current_source_dir = os.path.dirname(os.path.abspath(file_path))

        if file_path.lower().endswith(".xacro"):
            converted_urdf = self.convert_xacro_to_urdf(file_path)
            if not converted_urdf:
                return False
            urdf_path = converted_urdf
        else:
            urdf_path = file_path

        success = self.load_robot_model(urdf_path, initial=initial, context_dir=self.current_source_dir)
        if success:
            self.current_source_path = os.path.abspath(file_path)
            # Son kullanılan modeller listesine ekle
            try:
                cfg = self.load_config()
                recents = cfg.get("recent_models", [])
                norm_path = os.path.normpath(self.current_source_path)
                if norm_path in recents:
                    recents.remove(norm_path)
                recents.insert(0, norm_path)
                cfg["recent_models"] = recents[:5]
                self.save_config(cfg)
            except Exception:
                pass
        return success

    def open_urdf_file_dialog(self):
        file_path = filedialog.askopenfilename(
            title="Robot Modeli Seçin (URDF / Xacro)",
            filetypes=[
                ("Desteklenen Modeller (*.urdf, *.xacro)", "*.urdf;*.xacro"),
                ("URDF Modelleri (*.urdf)", "*.urdf"),
                ("Xacro Modelleri (*.xacro)", "*.xacro"),
                ("Tüm Dosyalar (*.*)", "*.*")
            ]
        )
        if not file_path:
            return

        success = self.load_source_model(file_path)
        if success:
            model_name = self.get_model_name_from_path(file_path)
            self.status_message = f"Model Yüklendi: {model_name} (Açılış modeli yapmak için '⭐ Ana Model Yap'a basın)"

    def set_current_as_default(self):
        if not hasattr(self, 'current_source_path') or not self.current_source_path or not os.path.isfile(self.current_source_path):
            messagebox.showwarning("Uyarı", "Şu anda yüklü geçerli bir robot modeli bulunamadı.")
            return

        cfg = self.load_config()
        current_abs = os.path.abspath(self.current_source_path)
        saved_abs = os.path.abspath(cfg.get("default_model_path")) if cfg.get("default_model_path") else None

        # Zaten fabrika Fanuc modeli ise
        is_builtin = hasattr(self, 'builtin_urdf_path') and os.path.normpath(current_abs) == os.path.normpath(os.path.abspath(self.builtin_urdf_path))
        if is_builtin:
            cfg["default_model_path"] = None
            self.save_config(cfg)
            self.update_default_model_label()
            self.status_message = "Açılış modeli: Fabrika Fanuc LR-Mate"
            messagebox.showinfo("Bilgi", "Fabrika modeli (Fanuc) varsayılan açılış modeli olarak ayarlandı.")
            return

        if saved_abs and os.path.normpath(current_abs) == os.path.normpath(saved_abs):
            messagebox.showinfo("Bilgi", "Bu model zaten varsayılan açılış modeliniz olarak kayıtlı.")
            return

        cfg["default_model_path"] = current_abs
        if self.save_config(cfg):
            model_name = self.get_model_name_from_path(self.current_source_path)
            self.update_default_model_label()
            self.status_message = f"Ana Model Belirlendi: {model_name}"
            messagebox.showinfo(
                "Ana Model Kaydedildi",
                f"'{model_name}' başarıyla ana model olarak kaydedildi!\n\n"
                f"Uygulamayı her açtığınızda otomatik olarak bu model yüklenecektir."
            )
        else:
            messagebox.showerror("Hata", "Ayar dosyası kaydedilemedi.")

    def reset_default_robot(self):
        target_path = getattr(self, 'builtin_urdf_path', getattr(self, 'default_urdf_path', None))
        if not target_path or not os.path.exists(target_path):
            messagebox.showwarning("Uyarı", "Varsayılan Fanuc URDF dosyası bulunamadı.")
            return

        cfg = self.load_config()
        has_custom_default = bool(cfg.get("default_model_path"))

        success = self.load_source_model(target_path)
        if success:
            if has_custom_default:
                ans = messagebox.askyesno(
                    "Açılış Modelini Sıfırla",
                    "Fabrika standardı Fanuc LR-Mate modeli yüklendi.\n\n"
                    "Uygulamanın açılış varsayılanını da orijinal Fanuc olarak sıfırlamak ister misiniz?\n\n"
                    "• EVET: Her açılışta Fanuc yüklenir (kayıtlı ana model temizlenir).\n"
                    "• HAYIR: Sadece bu oturum için Fanuc kullanılır, kayıtlı ana modeliniz korunur."
                )
                if ans:
                    cfg["default_model_path"] = None
                    self.save_config(cfg)
                    self.update_default_model_label()
                    self.status_message = "Açılış modeli Fanuc olarak sıfırlandı."
            else:
                self.status_message = "Varsayılan Fanuc LR-Mate modeli yüklendi."

    def save_position(self):
        current_pos = [var.get() for var in self.slider_vars]
        if current_pos != self.position_history[self.history_idx]:
            self.position_history = self.position_history[:self.history_idx + 1]
            self.position_history.append(current_pos)
            self.history_idx += 1

    def go_to_base(self):
        for var in self.slider_vars: var.set(0.0)
        self.save_position()

    def undo_position(self):
        if self.history_idx > 0:
            self.history_idx -= 1
            hist = self.position_history[self.history_idx]
            for i, var in enumerate(self.slider_vars):
                if i < len(hist):
                    var.set(hist[i])

    def redo_position(self):
        if self.history_idx < len(self.position_history) - 1:
            self.history_idx += 1
            hist = self.position_history[self.history_idx]
            for i, var in enumerate(self.slider_vars):
                if i < len(hist):
                    var.set(hist[i])

    def calculate_fk(self):
        with self.bullet_lock:
            state = p.getLinkState(self.robotId, self.end_effector_index)
            joint_states = p.getJointStates(self.robotId, self.revolute_joints)

        pos = state[4]
        quat = state[5]
        current_joints = [js[0] for js in joint_states]
        rot_matrix = p.getMatrixFromQuaternion(quat)

        T = [
            [rot_matrix[0], rot_matrix[1], rot_matrix[2], pos[0] * 100],
            [rot_matrix[3], rot_matrix[4], rot_matrix[5], pos[1] * 100],
            [rot_matrix[6], rot_matrix[7], rot_matrix[8], pos[2] * 100],
            [0.0, 0.0, 0.0, 1.0]
        ]

        n_joints = len(self.revolute_joints)
        fk_log = "\n\n" + "=" * 52 + "\n"
        fk_log += "=== İLERİ KİNEMATİK (FK) MATEMATİKSEL ANALİZİ ===\n"
        fk_log += f">> 1. EKLEM DURUMLARI (Aktif Eksen: {n_joints})\n"
        fk_log += "  i |  θ_i (Açı)  |  θ_i (Radyan) \n"
        fk_log += "-" * 52 + "\n"
        for i, val in enumerate(current_joints):
            fk_log += f"  {i+1} | {math.degrees(val):8.2f}° | {val:8.4f} rad\n"

        fk_log += "\n>> 2. DÖNÜŞÜM MATRİSİ (A_i) FORMÜLÜ\n"
        fk_log += "  [ cos(θ)  -sin(θ)*cos(α)   sin(θ)*sin(α)  a*cos(θ) ]\n"
        fk_log += "  [ sin(θ)   cos(θ)*cos(α)  -cos(θ)*sin(α)  a*sin(θ) ]\n"
        fk_log += "  [   0          sin(α)           cos(α)       d     ]\n"
        fk_log += "  [   0            0                0          1     ]\n"

        fk_log += f"\n>> 3. NİHAİ HOMOJEN DÖNÜŞÜM MATRİSİ (T_0^{n_joints})\n"
        fk_log += f"  T = A_1 * ... * A_{n_joints}\n"
        fk_log += "        [ R11    R12    R13  |    Px    ]\n"
        fk_log += f"        [{T[0][0]:6.3f} {T[0][1]:6.3f} {T[0][2]:6.3f}  | {T[0][3]:8.2f} cm]\n"
        fk_log += f"        [{T[1][0]:6.3f} {T[1][1]:6.3f} {T[1][2]:6.3f}  | {T[1][3]:8.2f} cm]\n"
        fk_log += f"        [{T[2][0]:6.3f} {T[2][1]:6.3f} {T[2][2]:6.3f}  | {T[2][3]:8.2f} cm]\n"
        fk_log += "        [ 0.000  0.000  0.000  |   1.000  ]\n"
        fk_log += "=" * 52 + "\n"

        self.fk_math_log = fk_log
        self.status_message = f"FK BAŞARILI: Matris Hesaplandı ve Loglandı."
        self.save_position()

    def check_angles_safety(self, angles):
        """
        Sanal olarak verilen açıları robota geçici uygulayıp çarpışma olup olmadığını test eder.
        Mevcut motor ve fizik durumunu bozmaz.
        Dönüş: (is_safe: bool, collision_desc: str)
        """
        with self.bullet_lock:
            original_states = p.getJointStates(self.robotId, self.revolute_joints)
            try:
                for i, j_idx in enumerate(self.revolute_joints):
                    if i < len(angles):
                        p.resetJointState(self.robotId, j_idx, angles[i])

                # 1. Gövde & Kaide Çarpışma Kontrolü (Self / Base / Stand Collision)
                if hasattr(self, 'allowed_collision_pairs'):
                    contacts = p.getClosestPoints(self.robotId, self.robotId, distance=0.0)
                    for c in contacts:
                        lA, lB, dist = c[3], c[4], c[8]
                        if lA == lB:
                            continue
                        pair = (min(lA, lB), max(lA, lB))
                        if pair not in self.allowed_collision_pairs and dist < -0.012:
                            nameA = getattr(self, 'link_names', {}).get(lA, f"Link_{lA}")
                            nameB = getattr(self, 'link_names', {}).get(lB, f"Link_{lB}")
                            return False, f"Gövde Çarpışması ({nameA} <-> {nameB})"

                # 2. Kaide (Stand) ve Zemin Penetrasyon Kontrolü
                stand_half = getattr(self, 'stand_half_extent_cm', 0.0)
                stand_top = getattr(self, 'stand_top_cm', 0.0)
                for j_idx in self.revolute_joints:
                    ls = p.getLinkState(self.robotId, j_idx)
                    lx = ls[4][0] * 100.0
                    ly = ls[4][1] * 100.0
                    lz = ls[4][2] * 100.0
                    if stand_half > 0 and abs(lx) <= stand_half and abs(ly) <= stand_half:
                        if lz < stand_top:
                            name = getattr(self, 'link_names', {}).get(j_idx, f"Link_{j_idx}")
                            return False, f"Kaideye Çarpma ({name})"
                    elif lz < 2.0:
                        name = getattr(self, 'link_names', {}).get(j_idx, f"Link_{j_idx}")
                        return False, f"Zemine Çarpma ({name})"

                # 3. Harici Engelle çarpışma kontrolü
                if self.obstacle_id is not None:
                    obs_contacts = p.getContactPoints(self.robotId, self.obstacle_id)
                    real_obs = [c for c in obs_contacts if c[8] < 0.0]
                    if real_obs:
                        c = real_obs[0]
                        link_hit = getattr(self, 'link_names', {}).get(c[3], f"Link_{c[3]}")
                        return False, f"Engelle Çarpışma ({link_hit})"

                return True, "Güvenli (Çarpışmasız)"
            finally:
                for i, j_idx in enumerate(self.revolute_joints):
                    p.resetJointState(self.robotId, j_idx, original_states[i][0], original_states[i][1])

    # --- TAM SORUNSUZ ÇALIŞAN ÇOKLU-TOHUM VE GÜVENLİK KORUMALI IK ---
    def calculate_ik(self):
        if hasattr(self, 'apply_payload_settings'):
            self.apply_payload_settings()

        try:
            x_val = self.entry_x.get().strip().replace(',', '.')
            y_val = self.entry_y.get().strip().replace(',', '.')
            z_val = self.entry_z.get().strip().replace(',', '.')

            max_r = float(getattr(self, 'robot_reach_cm', 70.0))
            stand_half = getattr(self, 'stand_half_extent_cm', 0.0)
            stand_top = getattr(self, 'stand_top_cm', 0.0)
            base_z = getattr(self, 'base_z_cm', 50.0)

            x_cm = float(x_val) if x_val else (max_r * 0.45)
            y_cm = float(y_val) if y_val else 0.0
            z_cm = float(z_val) if z_val else (base_z + 15.0)

            math_log = "\n\n" + "=" * 52 + "\n"
            math_log += "=== TERS KİNEMATİK (IK) MATEMATİKSEL ÇÖZÜMÜ ===\n"

            # Kaide ve Zemin Sınır Kontrolü
            min_z_req = (stand_top + 5.0) if (stand_half > 0 and abs(x_cm) <= stand_half + 5.0 and abs(y_cm) <= stand_half + 5.0) else 0.0
            if z_cm < min_z_req:
                math_log += f">> [!] GÜVENLİK KORUMASI: Hedef Z ({z_cm:.1f} cm) zemin/kaide seviyesinin altında!\n"
                math_log += f"    Z={min_z_req:.1f} cm güvenli yüksekliğe otomatik çekildi.\n\n"
                z_cm = min_z_req
                self.entry_z.delete(0, 'end')
                self.entry_z.insert(0, f"{z_cm:.1f}")

            x_m, y_m, z_m = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0
            self.pending_waypoints = []
            n_joints = len(self.revolute_joints)

            def solve_ik_pose(target_pos):
                ik_kwargs = {
                    'maxNumIterations': 400,
                    'residualThreshold': 1e-4
                }
                if hasattr(self, 'joint_lower_limits') and len(self.joint_lower_limits) == n_joints:
                    ik_kwargs['lowerLimits'] = self.joint_lower_limits
                    ik_kwargs['upperLimits'] = self.joint_upper_limits
                    ik_kwargs['jointRanges'] = self.joint_ranges
                    ik_kwargs['restPoses'] = self.joint_rest_poses

                yaw_base = math.atan2(target_pos[1], target_pos[0]) if (target_pos[0] != 0 or target_pos[1] != 0) else 0.0

                with self.data_lock:
                    curr_targets = [self.shared_targets[i] for i in range(min(n_joints, len(self.shared_targets)))]

                candidate_seeds = [
                    curr_targets,
                    [yaw_base, -0.4, 0.4] + [0.0] * max(0, n_joints - 3),
                    [yaw_base, -0.7, 0.8] + [0.0] * max(0, n_joints - 3),
                    [yaw_base, 0.0, 0.0] + [0.0] * max(0, n_joints - 3),
                    [yaw_base, 0.3, -0.3] + [0.0] * max(0, n_joints - 3),
                    [0.0] * n_joints
                ]

                best_angles = None
                best_err = float('inf')
                best_safe = False
                best_desc = ""

                orig = p.getJointStates(self.robotId, self.revolute_joints)
                for seed in candidate_seeds:
                    for i, j_idx in enumerate(self.revolute_joints):
                        if i < len(seed):
                            p.resetJointState(self.robotId, j_idx, seed[i])

                    sol = p.calculateInverseKinematics(
                        self.robotId, self.end_effector_index, target_pos, **ik_kwargs
                    )
                    angles = list(sol[:n_joints])
                    if hasattr(self, 'joint_lower_limits') and len(self.joint_lower_limits) == n_joints:
                        for k in range(n_joints):
                            low = self.joint_lower_limits[k]
                            high = self.joint_upper_limits[k]
                            if low < high:
                                angles[k] = max(low, min(high, angles[k]))

                    for i, j_idx in enumerate(self.revolute_joints):
                        p.resetJointState(self.robotId, j_idx, angles[i])
                    ee_pos = p.getLinkState(self.robotId, self.end_effector_index)[4]
                    err = math.sqrt(sum((a - b) ** 2 for a, b in zip(ee_pos, target_pos)))
                    is_safe, desc = self.check_angles_safety(angles)

                    if is_safe and err < 0.03:
                        best_angles = angles
                        best_err = err
                        best_safe = True
                        best_desc = desc
                        break

                    if not best_angles or (is_safe and not best_safe) or (is_safe == best_safe and err < best_err):
                        best_angles = angles
                        best_err = err
                        best_safe = is_safe
                        best_desc = desc

                for i, j_idx in enumerate(self.revolute_joints):
                    p.resetJointState(self.robotId, j_idx, orig[i][0], orig[i][1])

                return best_angles, best_err, best_safe, best_desc

            with self.bullet_lock:
                if self.obstacle_id is not None:
                    state = p.getLinkState(self.robotId, self.end_effector_index)
                    cx, cy, cz = state[4]
                    safe_z = max(cz, z_m, self.obstacle_pos[2] + 0.35)

                    wp1, err1, safe1, desc1 = solve_ik_pose([cx, cy, safe_z])
                    self.pending_waypoints.append(wp1)

                    wp2, err2, safe2, desc2 = solve_ik_pose([x_m, y_m, safe_z])
                    self.pending_waypoints.append(wp2)

                    math_log += f">> OTONOM KAÇIŞ AKTİF: Engel Algılandı.\n"
                    math_log += f">> {safe_z * 100:.0f}cm irtifadan U-Dönüş rotası çizildi.\n\n"

                final_angles, final_err, final_safe, final_desc = solve_ik_pose([x_m, y_m, z_m])
                self.pending_waypoints.append(final_angles)

            current_angles = []
            for var in self.slider_vars:
                try:
                    val = float(str(var.get()).replace(',', '.'))
                except Exception:
                    val = 0.0
                current_angles.append(val)
            deg_diffs = [abs(math.degrees(self.pending_waypoints[-1][i] - current_angles[i])) for i in range(min(n_joints, len(current_angles)))]
            max_deg = max(deg_diffs) if deg_diffs else 0

            # Yük ve motor dinamik süre & maliyet faktörleri
            reach_cm = getattr(self, 'robot_reach_cm', 70.0)
            rated_capacity = max(3.0, (reach_cm / 70.0) * 5.0)
            payload = getattr(self, 'payload_kg', 0.0)
            load_ratio = (payload / rated_capacity) if rated_capacity > 0 else 0.0
            duration_factor = math.sqrt(1.0 + 1.2 * load_ratio + 0.5 * (load_ratio ** 2))

            time_base = max(0.5, max_deg / 100.0)
            self.time_fast = time_base * duration_factor
            if self.obstacle_id is not None: self.time_fast *= 1.5
            self.time_eco = self.time_fast * 1.8

            eff = getattr(self, 'motor_efficiency', 0.85)
            tariff = getattr(self, 'electricity_rate', 4.50)
            standby_total_w = getattr(self, 'motor_standby_w', 15.0) * n_joints

            load_power_mult = 1.0 + (0.75 * load_ratio)
            self.peak_power_fast = (max_deg * 12.0 + 50.0) * load_power_mult
            self.peak_power_eco = self.peak_power_fast * 0.38

            # Hızlı Rota Enerji & Maliyet
            p_avg_fast_elec = (self.peak_power_fast * 0.55 / eff) + standby_total_w
            energy_fast_joules = p_avg_fast_elec * self.time_fast
            cost_fast_kwh = energy_fast_joules / 3.6e6
            cost_fast_tl = cost_fast_kwh * tariff

            # Ekonomik Rota Enerji & Maliyet
            p_avg_eco_elec = (self.peak_power_eco * 0.42 / eff) + standby_total_w
            energy_eco_joules = p_avg_eco_elec * self.time_eco
            cost_eco_kwh = energy_eco_joules / 3.6e6
            cost_eco_tl = cost_eco_kwh * tariff

            saving_pct = max(0.0, (1.0 - (cost_eco_tl / max(1e-6, cost_fast_tl))) * 100.0)

            math_log += f">> Adım 1: Hedef Uzay Vektörü (P_hedef)\n   P = [ {x_cm:.1f} cm, {y_cm:.1f} cm, {z_cm:.1f} cm ]^T\n\n"
            math_log += f">> Adım 2: Jacobian Ters Matris & Açı Sınırları (Constrained IK)\n   Δθ = J^-1(θ) * ΔX (Eklem Sınır Koruması: Aktif)\n"
            if final_err <= 0.03:
                math_log += f"   Hedef Doğrulama Sapması: {final_err * 100.0:.2f} cm (Mükemmel Hassasiyet)\n\n"
            else:
                math_log += f"   [!] Hedef Doğrulama Sapması: {final_err * 100.0:.2f} cm (Erişim sınırının dışında)\n\n"
            math_log += ">> Adım 3: DİNAMİK YÖRÜNGE, MOTOR YÜKÜ VE ELEKTRİK MALİYETİ\n"
            if payload <= 0.0:
                math_log += f"  - Uç Yükü (Payload) : 0.0 kg (Yüksüz / Nominal motor hızı ve torku)\n"
            else:
                math_log += f"  - Uç Yükü (Payload) : {payload:.2f} kg (Kapasite: {rated_capacity:.1f} kg | Yük Oranı: %{load_ratio*100:.0f})\n"
                if load_ratio > 1.0:
                    math_log += f"  - [!] AŞIRI YÜK: Taşıma kapasitesi aşıldı! Güvenlik için hız/tork sınırlandırıldı.\n"
                elif load_ratio > 0.05:
                    math_log += f"  - Yük Süre Çarpanı  : {duration_factor:.2f}x (Tork/eylemsizlik koruması devrede)\n"
            math_log += f"  - Motor Verimi      : %{eff*100:.0f} | Birim Elektrik: {tariff:.2f} TL/kWh | Bekleme: {standby_total_w:.0f}W\n\n"
            math_log += " [ Strateji 1: Hızlı Rota (Trapezoidal) ]\n"
            math_log += f"  • Tahmini Süre : {self.time_fast:.2f} sn\n"
            math_log += f"  • Zirve Güç    : {self.peak_power_fast:.1f} W (Mekanik)\n"
            math_log += f"  • Enerji Tüketimi : {cost_fast_kwh*1000:.3f} Wh ({energy_fast_joules:.1f} J)\n"
            math_log += f"  • Tahmini Maliyet : {cost_fast_tl:.4f} TL\n\n"
            math_log += " [ Strateji 2: Ekonomik Rota (S-Eğrisi) ]\n"
            math_log += f"  • Tahmini Süre : {self.time_eco:.2f} sn\n"
            math_log += f"  • Zirve Güç    : {self.peak_power_eco:.1f} W (Mekanik)\n"
            math_log += f"  • Enerji Tüketimi : {cost_eco_kwh*1000:.3f} Wh ({energy_eco_joules:.1f} J)\n"
            math_log += f"  • Tahmini Maliyet : {cost_eco_tl:.4f} TL\n"
            math_log += f"  >> TASARRUF FARKI : Ekonomik rota ile %{saving_pct:.1f} daha az elektrik faturası!\n\n"

            # 4. Adım: Sanal Ön Çarpışma ve Güvenlik Testi (Pre-Flight Safety Check)
            math_log += ">> Adım 4: SANAL GÜVENLİK & ÇARPIŞMA ANALİZİ\n"
            has_collision_risk = False
            collision_warning_text = ""
            for idx, wp in enumerate(self.pending_waypoints, 1):
                is_safe, desc = self.check_angles_safety(wp)
                if is_safe:
                    math_log += f"  [✓] Manevra {idx}: Güvenli (Çarpışmasız rota)\n"
                else:
                    has_collision_risk = True
                    collision_warning_text = desc
                    math_log += f"  [!] UYARI Manevra {idx}: TEHLİKE -> {desc}!\n"

            math_log += "=" * 52 + "\n"
            self.ik_math_log = math_log

            # Hamle Butonlarını Oluştur
            for widget in self.ik_steps_frame.winfo_children(): widget.destroy()

            step_num = 1
            for wp in self.pending_waypoints:
                wp_safe, wp_desc = self.check_angles_safety(wp)
                btn_state = "normal" if (wp_safe and final_err <= 0.03) else "disabled"
                btn_color = "#005A9E" if btn_state == "normal" else "#555555"
                btn_text = f"Manevra {step_num} Konumuna Git" if wp_safe else f"Manevra {step_num} (Kilitli: {wp_desc})"
                btn = ctk.CTkButton(self.ik_steps_frame, text=btn_text, corner_radius=6,
                                    command=lambda w=wp: self.go_to_ik_step(w),
                                    state=btn_state,
                                    fg_color=btn_color,
                                    hover_color="#003A68")
                btn.pack(pady=5, padx=10, fill="x")
                step_num += 1

            if has_collision_risk or final_err > 0.03:
                self.btn_fast_traj.configure(state="disabled")
                self.btn_eco_traj.configure(state="disabled")
                fail_reason = collision_warning_text if has_collision_risk else f"Hedef Ulaşılamaz ({final_err * 100:.1f} cm sapma)"
                self.status_message = f"UYARI: {fail_reason}! Hareket kilitlendi."
            else:
                self.btn_fast_traj.configure(state="normal")
                self.btn_eco_traj.configure(state="normal")
                self.status_message = "IK Hesaplandı! Rota güvenli (Çarpışma yok). Yörünge seçin."

            if hasattr(self, 'analysis_box'):
                self._last_report_text = math_log
                self.analysis_box.configure(state="normal")
                self.analysis_box.delete("0.0", "end")
                self.analysis_box.insert("end", math_log)
                self.analysis_box.configure(state="disabled")

        except Exception as e:
            import traceback
            traceback.print_exc()
            self.status_message = f"Hata: {str(e)}"

    def execute_trajectory(self, strategy):
        if self.e_stop_active:
            self.status_message = "E-STOP AKTİF! Önce Sıfırlayın."
            return

        self.btn_fast_traj.configure(state="disabled")
        self.btn_eco_traj.configure(state="disabled")

        duration = self.time_fast if strategy == "fast" else self.time_eco
        strat_name = "Hızlı" if strategy == "fast" else "Ekonomik"
        self.status_message = f"{strat_name} Rota başlatıldı. Süzülüyor..."

        n_joints = len(self.revolute_joints)
        with self.data_lock:
            current_angles = [self.shared_targets[i] for i in range(min(n_joints, len(self.shared_targets)))]

        threading.Thread(target=self.run_trajectory_thread,
                         args=(strategy, self.pending_waypoints, current_angles, duration), daemon=True).start()

    def run_trajectory_thread(self, strategy, waypoints, start_angles, total_duration):
        self.is_trajectory_playing = True
        num_waypoints = len(waypoints)
        duration_per_wp = total_duration / num_waypoints

        current = start_angles

        for wp_idx, target_angles in enumerate(waypoints):
            steps = int(max(duration_per_wp * 30, 5))

            with self.bullet_lock:
                self.update_link_positions()

            for step in range(1, steps + 1):
                if self.e_stop_active: break

                t = step / steps
                if strategy == "fast":
                    factor = t
                else:
                    factor = (1 - math.cos(math.pi * t)) / 2.0

                angles = [c + (tr - c) * factor for c, tr in zip(current, target_angles)]

                with self.bullet_lock:
                    for i, joint_idx in enumerate(self.revolute_joints):
                        if i < len(angles):
                            p.resetJointState(self.robotId, joint_idx, angles[i])
                    if self.show_workspace:
                        self.draw_swept_volume()

                with self.data_lock:
                    for i in range(min(len(angles), len(self.shared_targets))):
                        self.shared_targets[i] = angles[i]

                time.sleep(duration_per_wp / steps)

            if self.e_stop_active: break
            current = target_angles

        self.save_position()
        self.is_trajectory_playing = False
        if not self.e_stop_active:
            self.status_message = f"Robot hedefe başarıyla yerleşti!"
        if hasattr(self, 'btn_fast_traj'):
            self.btn_fast_traj.configure(state="normal")
        if hasattr(self, 'btn_eco_traj'):
            self.btn_eco_traj.configure(state="normal")

    def go_to_ik_step(self, angles):
        is_safe, desc = self.check_angles_safety(angles)
        if not is_safe:
            messagebox.showwarning("Çarpışma Engeli", f"Bu konuma gidilemez:\n{desc}")
            self.status_message = f"UYARI: {desc}! Hareket iptal edildi."
            return

        with self.data_lock:
            for i in range(min(len(angles), len(self.slider_vars), len(self.shared_targets))):
                self.slider_vars[i].set(angles[i])
                self.shared_targets[i] = angles[i]

        with self.bullet_lock:
            for i, joint_idx in enumerate(self.revolute_joints):
                if i < len(angles):
                    p.resetJointState(self.robotId, joint_idx, angles[i])
            if self.show_workspace:
                self.draw_swept_volume()

    # ==========================================
    # ARAYÜZ GÜNCELLEYİCİ VE GRAFİK ÇİZİCİ
    # ==========================================
    def update_ui_loop(self):
        if not self.is_running: return

        with self.data_lock:
            n_vars = len(self.slider_vars)
            if getattr(self, "is_trajectory_playing", False) or getattr(self, "e_stop_active", False):
                for i in range(min(n_vars, len(self.shared_targets))):
                    self.slider_vars[i].set(self.shared_targets[i])
            else:
                for i in range(min(n_vars, len(self.shared_targets))):
                    self.shared_targets[i] = self.slider_vars[i].get()

        img = None
        while not self.q_img.empty():
            try:
                img = self.q_img.get_nowait()
            except queue.Empty:
                break

        if img is not None:
            ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(600, 800))
            self.tk_img = ctk_img
            self.camera_label.configure(image=self.tk_img, text="")

        if self.e_stop_active:
            self.analysis_frame.configure(fg_color="#3A0000", border_color="#FF0000")
            self.status_message = "!!! ÇARPIŞMA TESPİT EDİLDİ - MOTORLAR KİLİTLENDİ !!!"
        else:
            self.analysis_frame.configure(fg_color="#1E1E1E", border_color="#2A2A2A")

        tel = None
        while not self.q_telemetry.empty():
            try:
                tel = self.q_telemetry.get_nowait()
            except queue.Empty:
                break

        if tel and hasattr(self, 'telemetry_box'):
            if not hasattr(self, '_last_tel_text') or self._last_tel_text != tel:
                self._last_tel_text = tel
                self.telemetry_box.configure(state="normal")
                self.telemetry_box.delete("0.0", "end")
                self.telemetry_box.insert("end", tel)
                self.telemetry_box.configure(state="disabled")

        # Statik Matematik ve Güvenlik Raporu (Sadece yeni hesaplama yapıldığında güncellenir)
        report_text = ""
        if self.current_page == "ik" and getattr(self, 'ik_math_log', None):
            report_text = self.ik_math_log
        elif self.current_page == "fk" and getattr(self, 'fk_math_log', None):
            report_text = self.fk_math_log

        if hasattr(self, 'analysis_box') and report_text:
            if not hasattr(self, '_last_report_text') or self._last_report_text != report_text:
                self._last_report_text = report_text
                self.analysis_box.configure(state="normal")
                self.analysis_box.delete("0.0", "end")
                self.analysis_box.insert("end", report_text)
                self.analysis_box.configure(state="disabled")

        self.update_power_graph()
        self.after(30, self.update_ui_loop)

    def update_power_graph(self):
        self.power_history.append(self.current_power)
        if len(self.power_history) > 100:
            self.power_history.pop(0)

        # Canlı sayaç ve maliyet etiketini güncelle
        if hasattr(self, 'lbl_energy_summary'):
            kwh_val = getattr(self, 'cumulative_energy_kwh', 0.0)
            cost_val = getattr(self, 'cumulative_cost_tl', 0.0)
            kwh_str = f"{kwh_val:.4f} kWh" if kwh_val < 1.0 else f"{kwh_val:.2f} kWh"
            cost_str = f"{cost_val:.3f} TL" if cost_val < 10.0 else f"{cost_val:.2f} TL"
            self.lbl_energy_summary.configure(
                text=f"Anlık: {self.current_power:5.1f} W  |  Sayaç: {kwh_str}  |  Maliyet: {cost_str}"
            )

        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        self.canvas.delete("all")

        self.canvas.create_line(0, h / 2, w, h / 2, fill="#333333", dash=(4, 4))
        max_power_scale = max(200.0, max(self.power_history) * 1.2)
        self.canvas.create_text(10, 10, text=f"Max: {max_power_scale:.0f} W", fill="#666666", anchor="nw")
        points = []
        for i, val in enumerate(self.power_history):
            x = (i / 100.0) * w
            y = h - ((val / max_power_scale) * h)
            points.append(x)
            points.append(y)

        if len(points) >= 4:
            line_color = "#FF3333" if self.e_stop_active else "#00FF66"
            self.canvas.create_line(*points, fill=line_color, width=2, smooth=True)

    # ==========================================
    # OYUN MOTORU & RENDER (ESKİ KASMAYAN ÇÖZÜNÜRLÜK: 200x266)
    # ==========================================
    def background_engine(self):
        accumulator = 0.0
        time_step = 1.0 / 240.0

        last_time           = time.perf_counter()
        last_tel_time       = last_time
        last_swept_time     = last_time
        last_transform_time = last_time
        last_safety_time    = last_time

        current_joints = [0.0] * len(self.revolute_joints)   # keeps last known joint positions for telemetry

        while self.is_running:
            current_time = time.perf_counter()
            dt = current_time - last_time
            last_time = current_time

            if dt > 0.05: dt = 0.05
            accumulator += dt

            with self.data_lock:
                targets = list(self.shared_targets)

            # ── HAFİF VE HIZLI ÇARPIŞMA / GÜVENLİK KONTROLÜ (20 Hz) ────────
            if not self.e_stop_active and (current_time - last_safety_time >= 0.05):
                last_safety_time = current_time
                with self.bullet_lock:
                    # 1. Engele çarpma kontrolü (0.03 ms)
                    if self.obstacle_id is not None:
                        contacts = p.getContactPoints(self.robotId, self.obstacle_id)
                        real_obs = [c for c in contacts if c[8] < 0.0]
                        if real_obs:
                            self.e_stop_active = True
                            hit_idx = real_obs[0][3]
                            hit_name = getattr(self, 'link_names', {}).get(hit_idx, f"Link {hit_idx}")
                            self.status_message = f"E-STOP: ENGELE ÇARPTI! ({hit_name})"

                    # 2. Gövde içi çarpışma (Doğrudan C++ temas önbelleği - 0.03 ms)
                    if not self.e_stop_active and hasattr(self, 'allowed_collision_pairs'):
                        contacts = p.getContactPoints(self.robotId, self.robotId)
                        for c in contacts:
                            lA, lB, dist = c[3], c[4], c[8]
                            if lA == lB or dist >= 0.0:
                                continue
                            pair = (min(lA, lB), max(lA, lB))
                            if pair not in self.allowed_collision_pairs:
                                self.e_stop_active = True
                                nameA = getattr(self, 'link_names', {}).get(lA, f"Link {lA}")
                                nameB = getattr(self, 'link_names', {}).get(lB, f"Link {lB}")
                                self.status_message = f"E-STOP: GÖVDE ÇARPIŞMASI! ({nameA} <-> {nameB})"
                                break

                    # 3. Kaide ve Zemin koruması (Sadece hareket esnasında)
                    if not self.e_stop_active and getattr(self, 'is_trajectory_playing', False):
                        stand_half = getattr(self, 'stand_half_extent_cm', 0.0)
                        stand_top = getattr(self, 'stand_top_cm', 0.0)
                        for j_idx in self.revolute_joints:
                            ls = p.getLinkState(self.robotId, j_idx)
                            lx, ly, lz = ls[4][0] * 100.0, ls[4][1] * 100.0, ls[4][2] * 100.0
                            if stand_half > 0 and abs(lx) <= stand_half and abs(ly) <= stand_half:
                                if lz < stand_top - 0.5:
                                    self.e_stop_active = True
                                    name = getattr(self, 'link_names', {}).get(j_idx, f"Link {j_idx}")
                                    self.status_message = f"E-STOP: KAİDEYE ÇARPTI! ({name})"
                                    break
                            elif lz < -1.0:
                                self.e_stop_active = True
                                name = getattr(self, 'link_names', {}).get(j_idx, f"Link {j_idx}")
                                self.status_message = f"E-STOP: ZEMİNE ÇARPTI! ({name})"
                                break

            # ── FİZİK VE MOTOR KONTROLÜ ───────────────────────────────────────
            with self.bullet_lock:
                if self.e_stop_active:
                    states = p.getJointStates(self.robotId, self.revolute_joints)
                    for i in range(min(len(states), len(targets))):
                        targets[i] = states[i][0]

                # Motor kontrol (Titreşim ve jiggle önleyici ayarlanmış P/V kazançları)
                for i, joint_idx in enumerate(self.revolute_joints):
                    if i < len(targets):
                        p.setJointMotorControl2(
                            self.robotId, joint_idx, p.POSITION_CONTROL,
                            targetPosition=targets[i], force=800, maxVelocity=4.0,
                            positionGain=0.15, velocityGain=1.0
                        )

                # Fizik adımı (Akümülatör spiral kilidini önleyici tavan: max 0.05s)
                accumulator = min(accumulator, 0.05)
                while accumulator >= time_step:
                    p.stepSimulation()
                    accumulator -= time_step

                # Güç & Enerji tahmini
                total_mech = 0.0
                joint_states = p.getJointStates(self.robotId, self.revolute_joints)
                for js in joint_states:
                    total_mech += abs(js[1] * js[3])   # velocity × torque

                n_axes = len(self.revolute_joints)
                eff = max(0.2, getattr(self, 'motor_efficiency', 0.85))
                standby_p = getattr(self, 'motor_standby_w', 15.0) * n_axes

                p_elec = (total_mech / eff) + standby_p
                self.current_power = (self.current_power * 0.8) + (p_elec * 0.2)

                self.cumulative_energy_joules += p_elec * dt
                self.cumulative_energy_kwh = self.cumulative_energy_joules / 3.6e6
                rate = getattr(self, 'electricity_rate', 4.50)
                self.cumulative_cost_tl = self.cumulative_energy_kwh * rate

            # ── Swept volume (workspace trace) ────────────────────────────
            if self.show_workspace and (current_time - last_swept_time >= (1.0 / 15.0)):
                last_swept_time = current_time
                with self.bullet_lock:
                    self.draw_swept_volume()

            # ── GPU transform update at 60 Hz ─────────────────────────────
            if current_time - last_transform_time >= (1.0 / 60.0):
                last_transform_time = current_time

                shapes_meta = getattr(self, 'visual_shapes_meta', [])
                transforms = [None] * len(shapes_meta)
                with self.bullet_lock:
                    base_pos, base_quat = p.getBasePositionAndOrientation(self.robotId)
                    for idx, meta in enumerate(shapes_meta):
                        link_idx = meta['link_idx']
                        local_pos = meta.get('local_pos', (0., 0., 0.))
                        local_ori = meta.get('local_ori', (0., 0., 0., 1.))
                        if link_idx == -1:
                            w_pos, w_quat = p.multiplyTransforms(base_pos, base_quat, local_pos, local_ori)
                        else:
                            ls = p.getLinkState(self.robotId, link_idx)
                            w_pos, w_quat = p.multiplyTransforms(ls[4], ls[5], local_pos, local_ori)
                        transforms[idx] = quat_to_mat4(w_pos, w_quat)

                    js_all = p.getJointStates(self.robotId, self.revolute_joints)
                    current_joints = [js[0] for js in js_all]

                with self.gpu_state_lock:
                    self.gpu_shared_state['transforms'] = transforms

            # ── Telemetry at 5 Hz (Sabit, hafif telemetri akışı) ──────────
            if current_time - last_tel_time >= 0.2:
                last_tel_time = current_time

                with self.bullet_lock:
                    state = p.getLinkState(self.robotId, self.end_effector_index)
                pos, rpy = state[4], p.getEulerFromQuaternion(state[5])

                model_name = os.path.splitext(os.path.basename(getattr(self, 'current_urdf_path', 'Robot')))[0]
                for pfx in [".", "sanitized_", "compiled_"]:
                    if model_name.startswith(pfx): model_name = model_name[len(pfx):]
                if model_name.endswith("_generated"): model_name = model_name[:-10]

                status_tag = "[KİLİTLENDİ]" if self.e_stop_active else "[NORMAL]"
                j_deg = [f"{math.degrees(c):.1f}°" for c in current_joints[:6]]

                log_text  = f"MODEL : {model_name}  {status_tag}\n"
                log_text += f"TCP   : X={pos[0]*100:6.1f}cm  Y={pos[1]*100:6.1f}cm  Z={pos[2]*100:6.1f}cm\n"
                log_text += f"DURUŞ : R={math.degrees(rpy[0]):5.1f}° P={math.degrees(rpy[1]):5.1f}° Y={math.degrees(rpy[2]):5.1f}°\n"
                log_text += f"EKSEN : {' '.join(f'J{i+1}:{v}' for i, v in enumerate(j_deg))}\n"
                if self.status_message:
                    log_text += f">> {self.status_message}"

                if not self.q_telemetry.full():
                    self.q_telemetry.put(log_text)

            time.sleep(0.003)   # CPU dostu 200 Hz stabil fizik çevrimi

    def on_closing(self):
        self.is_running = False
        with self.gpu_state_lock:
            self.gpu_shared_state['running'] = False
        try:
            self.worker_thread.join(timeout=1.0)
            p.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = RobotKontrolApp()
    app.mainloop()