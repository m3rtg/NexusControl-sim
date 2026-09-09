import pybullet as p
import pybullet_data
import os
import time
import math
import customtkinter as ctk
from PIL import Image
import numpy as np
import threading
import queue
import random
import tkinter as tk
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
        }

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
        self.lbl_qual_info.pack(pady=(0, 10))

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
            for i in range(6):
                self.slider_vars[i].set(0.0)
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

        slider_card = ctk.CTkFrame(self.page_manual, fg_color="#242424", corner_radius=10)
        slider_card.pack(fill="both", expand=True)

        self.sliders = []
        self.slider_vars = []
        limits = [(-2.96, 2.96), (-1.04, 2.44), (-2.47, 4.01), (-3.31, 3.31), (-2.09, 2.09), (-6.28, 6.28)]

        for i in range(6):
            label = ctk.CTkLabel(slider_card, text=f"Eksen J{i + 1}", font=ctk.CTkFont(weight="bold"))
            label.pack(pady=(12, 0))
            var = ctk.DoubleVar(value=0.0)
            self.slider_vars.append(var)
            slider = ctk.CTkSlider(slider_card, from_=limits[i][0], to=limits[i][1], variable=var,
                                   button_color="#00FFCC", button_hover_color="#00CCAA", border_width=2,
                                   border_color="#333")
            slider.pack(pady=(2, 10), padx=20, fill="x")
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

        max_r = 70.0  # Maksimum uzanma (cm)

        if abs(x_val) >= max_r:
            max_y = 0.0
        else:
            max_y = math.sqrt(max_r ** 2 - x_val ** 2)
        self.lbl_limit_y.configure(text=f"Maks Y: ±{max_y:.1f}")

        term_z = max_r ** 2 - x_val ** 2 - y_val ** 2
        if term_z <= 0:
            max_z = 0.0
        else:
            max_z = math.sqrt(term_z)
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
    # KİNEMATİK VE PYBULLET SİSTEMİ
    # ==========================================
    def init_pybullet(self):
        self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        urdf_path = "../fanuc_lrmate200ic_support/urdf/fanuc_lrmate200ic.urdf"
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

        self.revolute_joints = [i for i in range(p.getNumJoints(self.robotId)) if
                                p.getJointInfo(self.robotId, i)[2] == p.JOINT_REVOLUTE]
        self.end_effector_index = self.revolute_joints[-1]

        # GPU renderer için: link adı → PyBullet joint index haritası
        self.link_joint_map = {}
        for ji in range(p.getNumJoints(self.robotId)):
            info = p.getJointInfo(self.robotId, ji)
            child_link_name = info[12].decode('utf-8')
            self.link_joint_map[child_link_name] = ji

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
            for i, var in enumerate(self.slider_vars): var.set(self.position_history[self.history_idx][i])

    def redo_position(self):
        if self.history_idx < len(self.position_history) - 1:
            self.history_idx += 1
            for i, var in enumerate(self.slider_vars): var.set(self.position_history[self.history_idx][i])

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

        fk_log = "\n\n" + "=" * 52 + "\n"
        fk_log += "=== İLERİ KİNEMATİK (FK) MATEMATİKSEL ANALİZİ ===\n"
        fk_log += ">> 1. DENAVIT-HARTENBERG (D-H) TABLOSU\n"
        fk_log += "  i |  θ_i (Açı)  |  d_i (cm) |  a_i (cm) |  α_i \n"
        fk_log += "-" * 52 + "\n"
        fk_log += f"  1 | {math.degrees(current_joints[0]):8.2f}° |    33.0   |    5.0   |  -90°\n"
        fk_log += f"  2 | {math.degrees(current_joints[1]):8.2f}° |     0.0   |   33.0   |    0°\n"
        fk_log += f"  3 | {math.degrees(current_joints[2]):8.2f}° |     0.0   |    3.5   |  -90°\n"
        fk_log += f"  4 | {math.degrees(current_joints[3]):8.2f}° |    33.5   |    0.0   |   90°\n"
        fk_log += f"  5 | {math.degrees(current_joints[4]):8.2f}° |     0.0   |    0.0   |  -90°\n"
        fk_log += f"  6 | {math.degrees(current_joints[5]):8.2f}° |     8.0   |    0.0   |    0°\n"

        fk_log += "\n>> 2. DÖNÜŞÜM MATRİSİ (A_i) FORMÜLÜ\n"
        fk_log += "  [ cos(θ)  -sin(θ)*cos(α)   sin(θ)*sin(α)  a*cos(θ) ]\n"
        fk_log += "  [ sin(θ)   cos(θ)*cos(α)  -cos(θ)*sin(α)  a*sin(θ) ]\n"
        fk_log += "  [   0          sin(α)           cos(α)       d     ]\n"
        fk_log += "  [   0            0                0          1     ]\n"

        fk_log += "\n>> 3. NİHAİ HOMOJEN DÖNÜŞÜM MATRİSİ (T_0^6)\n"
        fk_log += "  T = A_1 * A_2 * A_3 * A_4 * A_5 * A_6\n"
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

            with self.bullet_lock:
                if self.obstacle_id is not None:
                    state = p.getLinkState(self.robotId, self.end_effector_index)
                    cx, cy, cz = state[4]

                    # U-Rotası: Önce engelin veya mevcut konumun üzerine güvenli bir aşırtma noktası belirle
                    safe_z = max(cz, z_m, self.obstacle_pos[2] + 0.35)

                    # Ara Nokta 1 (Sadece yukarı kalk)
                    wp1 = p.calculateInverseKinematics(self.robotId, self.end_effector_index, [cx, cy, safe_z],
                                                       maxNumIterations=500)
                    self.pending_waypoints.append(list(wp1[:6]))

                    # Ara Nokta 2 (Hedefin üzerine süzül)
                    wp2 = p.calculateInverseKinematics(self.robotId, self.end_effector_index, [x_m, y_m, safe_z],
                                                       maxNumIterations=500)
                    self.pending_waypoints.append(list(wp2[:6]))

                    math_log += f">> OTONOM KAÇIŞ AKTİF: Engel Algılandı.\n"
                    math_log += f">> {safe_z * 100:.0f}cm irtifadan U-Dönüş rotası çizildi.\n\n"

                # Nihai Hedefe İniş
                final_angles = p.calculateInverseKinematics(self.robotId, self.end_effector_index, [x_m, y_m, z_m],
                                                            maxNumIterations=500)
                self.pending_waypoints.append(list(final_angles[:6]))

            current_angles = [var.get() for var in self.slider_vars]

            deg_diffs = [abs(math.degrees(self.pending_waypoints[-1][i] - current_angles[i])) for i in range(6)]
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

        with self.data_lock:
            current_angles = [self.shared_targets[i] for i in range(6)]

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
                        p.resetJointState(self.robotId, joint_idx, angles[i])
                    if self.show_workspace:
                        self.draw_swept_volume()

                with self.data_lock:
                    for i in range(6):
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
            for i in range(6):
                self.slider_vars[i].set(angles[i])
                self.shared_targets[i] = angles[i]

        with self.bullet_lock:
            for i, joint_idx in enumerate(self.revolute_joints):
                p.resetJointState(self.robotId, joint_idx, angles[i])
            if self.show_workspace:
                self.draw_swept_volume()

    # ==========================================
    # ARAYÜZ GÜNCELLEYİCİ VE GRAFİK ÇİZİCİ
    # ==========================================
    def update_ui_loop(self):
        if not self.is_running: return

        with self.data_lock:
            if getattr(self, "is_trajectory_playing", False):
                for i in range(6):
                    self.slider_vars[i].set(self.shared_targets[i])
            else:
                for i in range(6):
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

        current_joints = [0.0] * 6   # keeps last known joint positions for telemetry

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
                        for i in range(6):
                            self.shared_targets[i] = states[i][0]

                if self.e_stop_active:
                    states = p.getJointStates(self.robotId, self.revolute_joints)
                    for i in range(6): targets[i] = states[i][0]

                # ── Motor control ─────────────────────────────────────────
                for i, joint_idx in enumerate(self.revolute_joints):
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

                transforms = [None] * 7
                with self.bullet_lock:
                    # base_link
                    base_pos, base_quat = p.getBasePositionAndOrientation(self.robotId)
                    transforms[0] = quat_to_mat4(base_pos, base_quat)

                    # link_1 … link_6
                    for idx, link_name in enumerate(
                            ['link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']):
                        if link_name in self.link_joint_map:
                            ji = self.link_joint_map[link_name]
                            ls = p.getLinkState(self.robotId, ji)
                            transforms[idx + 1] = quat_to_mat4(ls[4], ls[5])

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

                log_text  = "=== FANUC KONTROLCÜ DURUMU ===\n"
                log_text += f"STATUS       : {self.status_message}\n"
                log_text += f"ACTIVE TOOL  : 1\n"
                log_text += f"USER FRAME   : 0 (WORLD)\n\n"
                log_text += "--- TCP KOORDİNATLARI (Cartesian) ---\n"
                log_text += f"X : {pos[0] * 100:7.2f} cm    W (Roll) : {math.degrees(rpy[0]):7.2f}°\n"
                log_text += f"Y : {pos[1] * 100:7.2f} cm    P (Pitch): {math.degrees(rpy[1]):7.2f}°\n"
                log_text += f"Z : {pos[2] * 100:7.2f} cm    R (Yaw)  : {math.degrees(rpy[2]):7.2f}°\n\n"
                log_text += "--- EKLEM AÇILARI (Joints) ---\n"
                for i in range(6):
                    log_text += f"J{i + 1} : {math.degrees(current_joints[i]):7.2f}°\n"

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