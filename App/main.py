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

        self.data_lock = threading.Lock()
        self.bullet_lock = threading.Lock()

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

        # GPU Renderer paylaşılan durum
        self.gpu_state_lock = threading.Lock()
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
        self.robot_reach_cm = 70.0

        self.obstacle_id = None
        self.obstacle_pos = [0.0, 0.0, 0.0]

        # ==========================================
        # SÜTUN DÜZENİ
        # ==========================================
        self.grid_rowconfigure(0, weight=4)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=2)
        self.grid_columnconfigure(2, weight=3)
        self.grid_columnconfigure(3, weight=3)

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
        self.btn_nav_manual.pack(pady=8, padx=20, fill="x")

        self.btn_nav_fk = ctk.CTkButton(self.sidebar, text="İleri Kinematik", height=40, corner_radius=8,
                                        command=lambda: self.show_page("fk"))
        self.btn_nav_fk.pack(pady=8, padx=20, fill="x")

        self.btn_nav_ik = ctk.CTkButton(self.sidebar, text="Ters Kinematik", height=40, corner_radius=8,
                                        command=lambda: self.show_page("ik"))
        self.btn_nav_ik.pack(pady=8, padx=20, fill="x")

        self.btn_nav_workspace = ctk.CTkButton(self.sidebar, text="Saydam İzi Göster", height=40, corner_radius=8,
                                               fg_color="#005A9E", hover_color="#003A68", command=self.toggle_workspace)
        self.btn_nav_workspace.pack(pady=(20, 8), padx=20, fill="x")

        # --- DİNAMİK ENGEL BUTONLARI ---
        obs_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        obs_frame.pack(pady=8, padx=20, fill="x")
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
        self.btn_estop_reset.pack(pady=(20, 8), padx=20, fill="x")

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
        self.quality_menu.pack(pady=(0, 6), padx=20, fill="x")

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
        self.btn_load_urdf.pack(pady=3, padx=20, fill="x")

        # Varsayılan Yap & Fabrikaya Dön Butonları
        btn_model_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        btn_model_frame.pack(pady=3, padx=20, fill="x")
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
        self.page_ik = ctk.CTkFrame(self.pages_frame, fg_color="transparent")

        self.setup_manual_page()
        self.setup_fk_page()
        self.setup_ik_page()

        # --- 3. ANALİZ PANELİ ---
        self.analysis_frame = ctk.CTkFrame(self, fg_color="#1E1E1E", corner_radius=12, border_width=1,
                                           border_color="#2A2A2A")
        self.analysis_frame.grid(row=0, column=2, padx=10, pady=(15, 5), sticky="nsew")

        self.analysis_title = ctk.CTkLabel(self.analysis_frame, text="SİSTEM TELEMETRİSİ & ANALİZİ",
                                           font=ctk.CTkFont(family="Segoe UI", size=14, weight="bold"))
        self.analysis_title.pack(pady=(15, 10))

        self.analysis_box = ctk.CTkTextbox(self.analysis_frame, font=ctk.CTkFont(family="Consolas", size=13),
                                           fg_color="#0A0A0A", text_color="#00FFCC", corner_radius=8, border_width=1,
                                           border_color="#1F1F1F")
        self.analysis_box.pack(padx=15, pady=(0, 15), fill="both", expand=True)

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

        self.graph_label = ctk.CTkLabel(self.graph_frame, text="CANLI MOTOR GÜÇ TÜKETİMİ (WATT)",
                                        font=ctk.CTkFont(family="Consolas", size=12, weight="bold"),
                                        text_color="#AAAAAA")
        self.graph_label.pack(pady=(5, 0))

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
        title = ctk.CTkLabel(self.page_ik, text="Ters Kinematik Otonom Motoru",
                             font=ctk.CTkFont(size=18, weight="bold"))
        title.pack(pady=(10, 15))

        ik_card = ctk.CTkFrame(self.page_ik, fg_color="#242424", corner_radius=10)
        ik_card.pack(fill="x", pady=(0, 10))

        # --- YENİ: DİNAMİK SINIR GÖSTERGELERİ (TEXTBOX YANINDA) ---
        self.var_x = ctk.StringVar()
        self.var_y = ctk.StringVar()
        self.var_x.trace_add("write", self.update_limits)
        self.var_y.trace_add("write", self.update_limits)

        # X Kutusu
        frame_x = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_x.pack(fill="x", padx=15, pady=(15, 5))
        self.entry_x = ctk.CTkEntry(frame_x, placeholder_text="Hedef X (Örn: 30.0 cm)", textvariable=self.var_x,
                                    height=38, border_color="#444")
        self.entry_x.pack(side="left", fill="x", expand=True)
        self.lbl_limit_x = ctk.CTkLabel(frame_x, text="Sınır: ±70.0", text_color="gray", width=90)
        self.lbl_limit_x.pack(side="right", padx=(10, 0))

        # Y Kutusu
        frame_y = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_y.pack(fill="x", padx=15, pady=5)
        self.entry_y = ctk.CTkEntry(frame_y, placeholder_text="Hedef Y (Örn: 0.0 cm)", textvariable=self.var_y,
                                    height=38, border_color="#444")
        self.entry_y.pack(side="left", fill="x", expand=True)
        self.lbl_limit_y = ctk.CTkLabel(frame_y, text="Maks Y: ±70.0", text_color="gray", width=90)
        self.lbl_limit_y.pack(side="right", padx=(10, 0))

        # Z Kutusu
        frame_z = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_z.pack(fill="x", padx=15, pady=(5, 15))
        self.entry_z = ctk.CTkEntry(frame_z, placeholder_text="Hedef Z (Örn: 40.0 cm)", height=38, border_color="#444")
        self.entry_z.pack(side="left", fill="x", expand=True)
        self.lbl_limit_z = ctk.CTkLabel(frame_z, text="Maks Z: ±70.0", text_color="gray", width=90)
        self.lbl_limit_z.pack(side="right", padx=(10, 0))

        self.btn_calc_ik = ctk.CTkButton(self.page_ik, text="Yörünge ve Rotayı Hesapla", height=42, corner_radius=8,
                                         command=self.calculate_ik)
        self.btn_calc_ik.pack(pady=(10, 10), fill="x")

        self.traj_frame = ctk.CTkFrame(self.page_ik, fg_color="transparent")
        self.traj_frame.pack(fill="x", padx=10, pady=(5, 15))
        self.traj_frame.grid_columnconfigure(0, weight=1)
        self.traj_frame.grid_columnconfigure(1, weight=1)

        self.btn_fast_traj = ctk.CTkButton(self.traj_frame, text="Hızlı Rotayı Başlat", height=38, corner_radius=6,
                                           fg_color="#8B0000", hover_color="#660000", state="disabled",
                                           command=lambda: self.execute_trajectory("fast"))
        self.btn_fast_traj.grid(row=0, column=0, padx=(0, 5), sticky="ew")

        self.btn_eco_traj = ctk.CTkButton(self.traj_frame, text="Ekonomik Rotayı Başlat", height=38, corner_radius=6,
                                          fg_color="#006400", hover_color="#004d00", state="disabled",
                                          command=lambda: self.execute_trajectory("eco"))
        self.btn_eco_traj.grid(row=0, column=1, padx=(5, 0), sticky="ew")

        step_title = ctk.CTkLabel(self.page_ik, text="Manevra Hamleleri", font=ctk.CTkFont(size=14, weight="bold"))
        step_title.pack(pady=(10, 5))
        self.ik_steps_frame = ctk.CTkScrollableFrame(self.page_ik, fg_color="#242424", corner_radius=10, height=180)
        self.ik_steps_frame.pack(fill="x", padx=10)

    # --- DİNAMİK LİMİT GÜNCELLEYİCİ ---
    def update_limits(self, *args):
        try:
            x_val = float(self.var_x.get().strip().replace(',', '.'))
        except:
            x_val = 0.0

        try:
            y_val = float(self.var_y.get().strip().replace(',', '.'))
        except:
            y_val = 0.0

        max_r = float(getattr(self, 'robot_reach_cm', 70.0))  # Robotun gerçek ölçeğine göre dinamik uzanma (cm)
        if hasattr(self, 'lbl_limit_x'):
            self.lbl_limit_x.configure(text=f"Sınır: ±{max_r:.1f}")

        if abs(x_val) >= max_r:
            max_y = 0.0
        else:
            max_y = math.sqrt(max_r ** 2 - x_val ** 2)
        if hasattr(self, 'lbl_limit_y'):
            self.lbl_limit_y.configure(text=f"Maks Y: ±{max_y:.1f}")

        term_z = max_r ** 2 - x_val ** 2 - y_val ** 2
        if term_z <= 0:
            max_z = 0.0
        else:
            max_z = math.sqrt(term_z)
        if hasattr(self, 'lbl_limit_z'):
            self.lbl_limit_z.configure(text=f"Maks Z: ±{max_z:.1f}")

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

    def resolve_mesh_path(self, urdf_dir, mesh_filename):
        if not mesh_filename:
            return None
        if isinstance(mesh_filename, bytes):
            mesh_filename = mesh_filename.decode('utf-8')
        if os.path.isabs(mesh_filename) and os.path.exists(mesh_filename):
            return os.path.abspath(mesh_filename).replace('\\', '/')

        app_dir = os.path.dirname(os.path.abspath(__file__))
        sim_root = os.path.normpath(os.path.join(app_dir, ".."))

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

        # 1. Simülatör dahili kaynak paketleri (resources/<pkg>/... veya resources/...)
        if pkg_name:
            c1 = os.path.join(sim_root, "resources", pkg_name, clean_path)
            if os.path.isfile(c1):
                return os.path.abspath(c1).replace("\\", "/")
            c2 = os.path.join(sim_root, "resources", clean_path)
            if os.path.isfile(c2):
                return os.path.abspath(c2).replace("\\", "/")

        # 2. Yerel dosya yolları (urdf_dir ve üst klasörleri)
        candidate_dirs = [
            urdf_dir,
            os.path.normpath(os.path.join(urdf_dir, "..")),
            os.path.normpath(os.path.join(urdf_dir, "meshes")),
            os.path.normpath(os.path.join(urdf_dir, "..", "meshes")),
            os.path.normpath(os.path.join(urdf_dir, "..", "..")),
        ]
        for cdir in candidate_dirs:
            p_check = os.path.normpath(os.path.join(cdir, clean_path))
            if os.path.isfile(p_check):
                return os.path.abspath(p_check).replace("\\", "/")

        # 3. Dosya adına göre tarama (resources, urdf_dir ve Downloads)
        base_name = os.path.basename(mesh_filename)
        if pkg_name:
            pkg_res = os.path.join(sim_root, "resources", pkg_name)
            if os.path.isdir(pkg_res):
                for root, dirs, files in os.walk(pkg_res):
                    if base_name in files:
                        return os.path.abspath(os.path.join(root, base_name)).replace("\\", "/")

        for root, dirs, files in os.walk(os.path.join(sim_root, "resources")):
            if base_name in files:
                return os.path.abspath(os.path.join(root, base_name)).replace("\\", "/")

        for search_root in [os.path.dirname(urdf_dir), os.path.expanduser("~/Downloads")]:
            if os.path.isdir(search_root):
                try:
                    for root, dirs, files in os.walk(search_root):
                        if base_name in files:
                            return os.path.abspath(os.path.join(root, base_name)).replace("\\", "/")
                        if root[len(search_root):].count(os.sep) >= 4:
                            dirs.clear()
                except Exception:
                    pass

        return None

    def sanitize_and_resolve_urdf(self, urdf_path):
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
                    resolved = self.resolve_mesh_path(urdf_dir, fn)
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

    def load_robot_model(self, urdf_path, initial=False):
        if not os.path.isfile(urdf_path):
            if not initial:
                messagebox.showerror("Hata", f"Seçilen dosya bulunamadı:\n{urdf_path}")
            return False

        try:
            self.status_message = "Robot modeli işleniyor ve yükleniyor..."
            app_dir = os.path.dirname(os.path.abspath(__file__))
            sim_root = os.path.normpath(os.path.join(app_dir, ".."))
            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

            # URDF'i PyBullet için sterilize et ve mesh yollarını bağla
            sanitized_urdf = self.sanitize_and_resolve_urdf(urdf_path)

            with self.bullet_lock:
                if hasattr(self, 'robotId') and self.robotId is not None:
                    try:
                        p.removeBody(self.robotId)
                    except Exception:
                        pass

                p.setAdditionalSearchPath(urdf_dir)
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

                self.end_effector_index = self.revolute_joints[-1] if self.revolute_joints else 0

                self.link_joint_map = {}
                for ji in range(num_joints):
                    info = p.getJointInfo(self.robotId, ji)
                    child_link_name = info[12].decode('utf-8')
                    self.link_joint_map[child_link_name] = ji

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
                base_dir = os.path.dirname(os.path.abspath(xacro_path))
                app_dir = os.path.dirname(os.path.abspath(__file__))
                sim_root = os.path.normpath(os.path.join(app_dir, ".."))

                # 1. Dahili simülatör kaynak paketleri ve İndirilenler klasörünü kontrol et
                downloads_dir = os.path.expanduser("~/Downloads")
                builtin_candidates = [
                    os.path.join(sim_root, "resources", pkg_name),
                    os.path.join(sim_root, pkg_name),
                    os.path.join(downloads_dir, pkg_name),
                ]
                for bc in builtin_candidates:
                    if os.path.isdir(bc):
                        return bc

                # İndirilenler klasörü altındaki açılmış paketleri tara
                if os.path.isdir(downloads_dir):
                    for entry in os.listdir(downloads_dir):
                        full_p = os.path.join(downloads_dir, entry)
                        if os.path.isdir(full_p):
                            if entry.lower() == pkg_name.lower():
                                return full_p
                            sub_p = os.path.join(full_p, pkg_name)
                            if os.path.isdir(sub_p):
                                return sub_p

                # 2. Klasör hiyerarşisinde yukarı doğru tara
                search_dir = base_dir
                for _ in range(6):
                    if os.path.basename(search_dir).lower() == pkg_name.lower():
                        return search_dir
                    direct_child = os.path.join(search_dir, pkg_name)
                    if os.path.isdir(direct_child):
                        return direct_child
                    parent = os.path.dirname(search_dir)
                    if parent == search_dir:
                        break
                    search_dir = parent

                # 3. Üst klasördeki kardeş paketleri tara
                parent_dir = os.path.dirname(base_dir)
                if os.path.isdir(parent_dir):
                    for entry in os.listdir(parent_dir):
                        p_full = os.path.join(parent_dir, entry)
                        if os.path.isdir(p_full) and entry.lower() == pkg_name.lower():
                            return p_full

                # 4. Bulunamazsa xacro klasörünün bir üstünü (paket kökünü) döndür
                return parent_dir if os.path.isdir(parent_dir) else base_dir

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

        if file_path.lower().endswith(".xacro"):
            converted_urdf = self.convert_xacro_to_urdf(file_path)
            if not converted_urdf:
                return False
            urdf_path = converted_urdf
        else:
            urdf_path = file_path

        success = self.load_robot_model(urdf_path, initial=initial)
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

    # --- TAM SORUNSUZ ÇALIŞAN (AŞIRTMA) ALGORİTMASI ---
    def calculate_ik(self):
        try:
            x_val = self.entry_x.get().strip().replace(',', '.')
            y_val = self.entry_y.get().strip().replace(',', '.')
            z_val = self.entry_z.get().strip().replace(',', '.')

            x_cm = float(x_val) if x_val else 30.0
            y_cm = float(y_val) if y_val else 0.0
            z_cm = float(z_val) if z_val else 40.0
            x_m, y_m, z_m = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0

            self.pending_waypoints = []
            math_log = "\n\n" + "=" * 50 + "\n"
            math_log += "=== TERS KİNEMATİK (IK) MATEMATİKSEL ÇÖZÜMÜ ===\n"

            n_joints = len(self.revolute_joints)
            with self.bullet_lock:
                if self.obstacle_id is not None:
                    state = p.getLinkState(self.robotId, self.end_effector_index)
                    cx, cy, cz = state[4]

                    # U-Rotası: Önce engelin veya mevcut konumun üzerine güvenli bir aşırtma noktası belirle
                    safe_z = max(cz, z_m, self.obstacle_pos[2] + 0.35)

                    # Ara Nokta 1 (Sadece yukarı kalk)
                    wp1 = p.calculateInverseKinematics(self.robotId, self.end_effector_index, [cx, cy, safe_z],
                                                       maxNumIterations=500)
                    self.pending_waypoints.append(list(wp1[:n_joints]))

                    # Ara Nokta 2 (Hedefin üzerine süzül)
                    wp2 = p.calculateInverseKinematics(self.robotId, self.end_effector_index, [x_m, y_m, safe_z],
                                                       maxNumIterations=500)
                    self.pending_waypoints.append(list(wp2[:n_joints]))

                    math_log += f">> OTONOM KAÇIŞ AKTİF: Engel Algılandı.\n"
                    math_log += f">> {safe_z * 100:.0f}cm irtifadan U-Dönüş rotası çizildi.\n\n"

                # Nihai Hedefe İniş
                final_angles = p.calculateInverseKinematics(self.robotId, self.end_effector_index, [x_m, y_m, z_m],
                                                            maxNumIterations=500)
                self.pending_waypoints.append(list(final_angles[:n_joints]))

            current_angles = [var.get() for var in self.slider_vars]

            deg_diffs = [abs(math.degrees(self.pending_waypoints[-1][i] - current_angles[i])) for i in range(min(n_joints, len(current_angles)))]
            max_deg = max(deg_diffs) if deg_diffs else 0

            self.time_fast = max(0.5, max_deg / 100.0)
            if self.obstacle_id is not None: self.time_fast *= 1.5

            self.peak_power_fast = max_deg * 12.0 + 50.0
            self.energy_fast = self.peak_power_fast * 0.6 * self.time_fast
            self.time_eco = self.time_fast * 1.8
            self.peak_power_eco = self.peak_power_fast * 0.35
            self.energy_eco = self.energy_fast * 0.45

            math_log += f">> Adım 1: Hedef Uzay Vektörü (P_hedef)\n   P = [ {x_cm:.1f} cm, {y_cm:.1f} cm, {z_cm:.1f} cm ]^T\n\n"
            math_log += f">> Adım 2: Jacobian Ters Matris Çözümü (J^-1)\n   Δθ = J^-1(θ) * ΔX\n\n"
            math_log += ">> Adım 3: DİNAMİK YÖRÜNGE VE ENERJİ ANALİZİ\n"
            math_log += " [ Strateji 1: Hızlı (Trapezoidal) ]\n"
            math_log += f"  - Tahmini Süre : {self.time_fast:.2f} sn\n"
            math_log += f"  - Zirve Güç    : {self.peak_power_fast:.1f} W\n"
            math_log += f"  - Enerji Yükü  : {self.energy_fast:.1f} J\n"
            math_log += " [ Strateji 2: Ekonomik (S-Eğrisi) ]\n"
            math_log += f"  - Tahmini Süre : {self.time_eco:.2f} sn\n"
            math_log += f"  - Zirve Güç    : {self.peak_power_eco:.1f} W\n"
            math_log += f"  - Enerji Yükü  : {self.energy_eco:.1f} J\n"
            math_log += "=" * 50 + "\n"

            self.ik_math_log = math_log

            self.btn_fast_traj.configure(state="normal")
            self.btn_eco_traj.configure(state="normal")

            # Hamle Butonlarını Oluştur
            for widget in self.ik_steps_frame.winfo_children(): widget.destroy()

            step_num = 1
            for wp in self.pending_waypoints:
                btn = ctk.CTkButton(self.ik_steps_frame, text=f"Manevra {step_num} Konumuna Git", corner_radius=6,
                                    command=lambda w=wp: self.go_to_ik_step(w), fg_color="#005A9E",
                                    hover_color="#003A68")
                btn.pack(pady=5, padx=10, fill="x")
                step_num += 1

            self.status_message = "IK Hesaplandı! Lütfen hareket yörüngesi seçin."

        except Exception as e:
            self.status_message = f"HATA: {str(e)}"

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

    def go_to_ik_step(self, angles):
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
            if getattr(self, "is_trajectory_playing", False):
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

        if tel:
            final_text = tel
            if self.current_page == "ik" and self.ik_math_log:
                final_text += self.ik_math_log
            elif self.current_page == "fk" and self.fk_math_log:
                final_text += self.fk_math_log

            self.analysis_box.configure(state="normal")
            self.analysis_box.delete("0.0", "end")
            self.analysis_box.insert("end", final_text)
            self.analysis_box.configure(state="disabled")

        self.update_power_graph()
        self.after(30, self.update_ui_loop)

    def update_power_graph(self):
        self.power_history.append(self.current_power)
        if len(self.power_history) > 100:
            self.power_history.pop(0)

        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        self.canvas.delete("all")

        self.canvas.create_line(0, h / 2, w, h / 2, fill="#333333", dash=(4, 4))
        self.canvas.create_text(10, 10, text="Max W", fill="#555", anchor="nw")

        max_power_scale = max(200.0, max(self.power_history) * 1.2)
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

        last_time          = time.perf_counter()
        last_tel_time      = last_time
        last_swept_time    = last_time
        last_transform_time = last_time

        current_joints = [0.0] * len(self.revolute_joints)   # keeps last known joint positions for telemetry

        while self.is_running:
            current_time = time.perf_counter()
            dt = current_time - last_time
            last_time = current_time

            if dt > 0.1: dt = 0.1
            accumulator += dt

            with self.data_lock:
                targets = list(self.shared_targets)

            with self.bullet_lock:
                # ── Collision / E-STOP check ──────────────────────────────
                if self.obstacle_id is not None:
                    contacts = p.getContactPoints(self.robotId, self.obstacle_id)
                    if len(contacts) > 0 and not self.e_stop_active:
                        self.e_stop_active = True
                        states = p.getJointStates(self.robotId, self.revolute_joints)
                        for i in range(min(len(states), len(self.shared_targets))):
                            self.shared_targets[i] = states[i][0]

                if self.e_stop_active:
                    states = p.getJointStates(self.robotId, self.revolute_joints)
                    for i in range(min(len(states), len(targets))):
                        targets[i] = states[i][0]

                # ── Motor control ─────────────────────────────────────────
                for i, joint_idx in enumerate(self.revolute_joints):
                    if i < len(targets):
                        p.setJointMotorControl2(
                            self.robotId, joint_idx, p.POSITION_CONTROL,
                            targetPosition=targets[i], force=1500, maxVelocity=15.0
                        )

                # ── Physics step at 240 Hz ────────────────────────────────
                while accumulator >= time_step:
                    p.stepSimulation()
                    accumulator -= time_step

                # ── Power estimation ──────────────────────────────────────
                total_power = 0.0
                joint_states = p.getJointStates(self.robotId, self.revolute_joints)
                for js in joint_states:
                    total_power += abs(js[1] * js[3])   # velocity × torque
                self.current_power = (self.current_power * 0.8) + (total_power * 0.2 * 100)

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

                    # also refresh current_joints for telemetry
                    js_all = p.getJointStates(self.robotId, self.revolute_joints)
                    current_joints = [js[0] for js in js_all]

                with self.gpu_state_lock:
                    self.gpu_shared_state['transforms'] = transforms

            # ── Telemetry at 10 Hz ────────────────────────────────────────
            if current_time - last_tel_time >= 0.1:
                last_tel_time = current_time

                with self.bullet_lock:
                    state = p.getLinkState(self.robotId, self.end_effector_index)
                pos, rpy = state[4], p.getEulerFromQuaternion(state[5])

                model_name = os.path.splitext(os.path.basename(getattr(self, 'current_urdf_path', 'Fanuc LR-Mate')))[0]
                log_text  = f"=== KONTROLCÜ DURUMU ({model_name}) ===\n"
                log_text += f"STATUS       : {self.status_message}\n"
                log_text += f"ACTIVE TOOL  : 1\n"
                log_text += f"USER FRAME   : 0 (WORLD)\n\n"
                log_text += "--- TCP KOORDİNATLARI (Cartesian) ---\n"
                log_text += f"X : {pos[0] * 100:7.2f} cm    W (Roll) : {math.degrees(rpy[0]):7.2f}°\n"
                log_text += f"Y : {pos[1] * 100:7.2f} cm    P (Pitch): {math.degrees(rpy[1]):7.2f}°\n"
                log_text += f"Z : {pos[2] * 100:7.2f} cm    R (Yaw)  : {math.degrees(rpy[2]):7.2f}°\n\n"
                log_text += f"--- EKLEM AÇILARI (Joints - {len(current_joints)} Eksen) ---\n"
                for i, c_angle in enumerate(current_joints):
                    log_text += f"J{i + 1} : {math.degrees(c_angle):7.2f}°\n"

                if not self.q_telemetry.full():
                    self.q_telemetry.put(log_text)

            time.sleep(0.002)   # physics thread — rendering is done by GPU renderer

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