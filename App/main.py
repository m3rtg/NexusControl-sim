import os
import sys
import gc
import subprocess
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

from ui import theme
import i18n
from i18n import t
from ui.pages.jog_page import JogPage
from ui.pages.target_page import TargetPage
from ui.pages.energy_page import EnergyPage
from ui.pages.model_page import ModelPage
from ui.pages.settings_page import SettingsPage
from ui.widgets.route_timeline import RouteTimeline
from ui.widgets.side_panel import SidePanel
from render_engine import RobotGPURenderer

theme.apply_global()

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

        i18n.set_language(self.load_config().get("language", i18n.DEFAULT_LANGUAGE))
        self.title(t("app.title"))
        self.geometry("1400x930")
        self.configure(fg_color=theme.BG)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.is_running = True

        self.data_lock = threading.RLock()
        self.position_history = [[0.0] * 6]
        self.history_idx = 0
        self.shared_targets = [0.0] * 6

        self.q_img = queue.Queue(maxsize=1)
        self.q_telemetry = queue.Queue(maxsize=1)
        self.tk_img = None

        self.status_message = t("msg.idle")
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
            if loaded: self.current_source_path = saved_default
        if not loaded:
            self.robot_loader.load_robot_model(self.builtin_urdf_path)
            self.current_source_path = self.builtin_urdf_path
            
        self.update_default_model_label()
        self.on_model_loaded()
        self.show_page("jog")

        mesh_dir = os.path.join(os.path.dirname(__file__), "..", "fanuc_lrmate200ic_support", "meshes", "lrmate200ic", "visual")
        self.gpu_renderer = RobotGPURenderer(os.path.normpath(mesh_dir), self.q_img, self.gpu_shared_state, self.gpu_state_lock)
        self.gpu_renderer.start()

        self.worker_thread = threading.Thread(target=self.background_engine, daemon=True)
        self.worker_thread.start()
        self.update_ui_loop()

    def build_ui(self):
        """Teach pendant düzeni: üstte marka/model/E-stop, solda mod anahtarı ve panel, altında rota şeridi,
        ortada sahne (renderer'ın 600x800 karesi), sağda güç/sahne/durum/rapor."""
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # --- ÜST ÇUBUK ---
        top = ctk.CTkFrame(self, fg_color="transparent", height=52)
        top.grid(row=0, column=0, sticky="ew", padx=16, pady=(10, 8))
        mark = ctk.CTkFrame(top, fg_color="transparent")
        mark.pack(side="left")
        ctk.CTkLabel(mark, text="NexusControl", font=theme.font(17, "bold"), text_color=theme.FG).pack(side="left")
        ctk.CTkLabel(mark, text="Sim", font=theme.font(17, "bold"), text_color=theme.GOLD).pack(side="left")
        self.lbl_model_line = ctk.CTkLabel(top, text="", font=theme.font(12), text_color=theme.FG2)
        self.lbl_model_line.pack(side="left", padx=(18, 6))
        change = ctk.CTkLabel(top, text=t("topbar.change"), font=theme.font(12), text_color=theme.GOLD, cursor="hand2")
        change.pack(side="left")
        change.bind("<Button-1>", lambda e: self.show_page("model"))

        self.lbl_pill = ctk.CTkLabel(top, text=t("pill.ready"), font=theme.font(11, "bold"), text_color=theme.OK,
                                     fg_color=theme.BG2, corner_radius=999, height=28)
        self.lbl_pill.pack(side="right", ipadx=10)
        # TR | EN: aktif dil beyaz, diğeri tıklanabilir
        lang_box = ctk.CTkFrame(top, fg_color="transparent")
        lang_box.pack(side="right", padx=(0, 18))
        for k, code in enumerate(i18n.available_languages()):
            if k: ctk.CTkLabel(lang_box, text="|", font=theme.font(11), text_color=theme.BORDER2).pack(side="left", padx=4)
            active = code == i18n.language()
            lbl = ctk.CTkLabel(lang_box, text=code.upper(), font=theme.font(11, "bold"),
                               text_color=theme.FG if active else theme.MUTED, cursor="" if active else "hand2")
            lbl.pack(side="left")
            if not active: lbl.bind("<Button-1>", lambda e, c=code: self.set_language(c))

        # --- GÖVDE ---
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.grid(row=1, column=0, sticky="nsew", padx=16, pady=(0, 12))
        body.grid_rowconfigure(0, weight=1)
        body.grid_columnconfigure(3, weight=1, minsize=260)

        # mod anahtarı
        dial = theme.card(body, width=72)
        dial.grid(row=0, column=0, sticky="ns", padx=(0, theme.PAD))
        self.mode_buttons = {}
        for mode in ("jog", "target", "energy", "model", "settings"):
            b = ctk.CTkButton(dial, text=t(f"mode.{mode}"), width=58, height=62, corner_radius=10, fg_color="transparent",
                              hover_color=theme.BG3, text_color=theme.MUTED, font=theme.font(11),
                              command=lambda m=mode: self.show_page(m))
            b.pack(padx=6, pady=(6 if mode == "jog" else 2, 2))
            self.mode_buttons[mode] = b

        # panel + rota şeridi
        left = ctk.CTkFrame(body, fg_color="transparent", width=344)
        left.grid(row=0, column=1, sticky="ns", padx=(0, theme.PAD))
        left.grid_propagate(False)
        left.grid_rowconfigure(0, weight=1)
        left.grid_columnconfigure(0, weight=1)
        self.pendant = theme.card(left)
        self.pendant.grid(row=0, column=0, sticky="nsew")
        self.pages_frame = ctk.CTkFrame(self.pendant, fg_color="transparent")
        self.pages_frame.pack(fill="both", expand=True, padx=14, pady=12)
        self.page_jog = JogPage(self.pages_frame, self, self.sim_engine)
        self.page_target = TargetPage(self.pages_frame, self, self.kinematics_engine)
        self.page_energy = EnergyPage(self.pages_frame, self, self.telemetry_engine, self.sim_engine)
        self.page_model = ModelPage(self.pages_frame, self)
        self.page_settings = SettingsPage(self.pages_frame, self)
        self.pages = {"jog": self.page_jog, "target": self.page_target, "energy": self.page_energy,
                      "model": self.page_model, "settings": self.page_settings}
        # eski adlarla erişen kodlar için
        self.page_manual = self.page_jog
        self.timeline = RouteTimeline(left, self.go_to_ik_step, self.execute_trajectory, self.on_blocked_stop)
        self.timeline.grid(row=1, column=0, sticky="ew", pady=(theme.PAD, 0))

        # sahne: renderer'ın 600x800 karesi, üstüne hiçbir şey çizilmez
        self.stage = theme.card(body)
        self.stage.grid(row=0, column=2, sticky="n", padx=(0, theme.PAD))
        head = ctk.CTkFrame(self.stage, fg_color="transparent")
        head.pack(fill="x", padx=12, pady=(8, 2))
        ctk.CTkLabel(head, text=f"— {t('stage.section')}", font=theme.font(11), text_color=theme.MUTED, height=16).pack(side="left")
        ctk.CTkLabel(head, text=t("stage.hint"), font=theme.font(10), text_color=theme.MUTED, height=16).pack(side="right")
        self.camera_label = ctk.CTkLabel(self.stage, text="", width=600, height=800, fg_color=theme.BG, corner_radius=0)
        self.camera_label.pack(padx=8, pady=(0, 8))
        self.camera_label.bind("<ButtonPress-1>", self._on_cam_mouse_down)
        self.camera_label.bind("<B1-Motion>", self._on_cam_mouse_drag)
        self.camera_label.bind("<ButtonPress-3>", self._on_cam_right_down)
        self.camera_label.bind("<B3-Motion>", self._on_cam_right_drag)
        self.camera_label.bind("<MouseWheel>", self._on_cam_mouse_wheel)
        self.camera_label.bind("<Button-4>", lambda e: self._on_cam_mouse_wheel(type("E", (), {"delta": 120})()))
        self.camera_label.bind("<Button-5>", lambda e: self._on_cam_mouse_wheel(type("E", (), {"delta": -120})()))

        self.side = SidePanel(body, self)
        self.side.grid(row=0, column=3, sticky="nsew")

    # --- ACTIONS ---
    def show_page(self, page_name):
        for page in self.pages.values(): page.pack_forget()
        self.current_page = page_name
        self.pages[page_name].pack(fill="both", expand=True)
        for mode, btn in self.mode_buttons.items():
            on = mode == page_name
            btn.configure(fg_color=theme.BG3 if on else "transparent", text_color=theme.GOLD if on else theme.MUTED,
                          border_width=1 if on else 0, border_color=theme.BORDER2)

    def toggle_obstacle(self):
        state, msg = self.sim_engine.toggle_obstacle()
        self.status_message = msg
        self.side.set_obstacle(self.sim_engine.obstacle_id is not None)

    def randomize_obstacle(self):
        self.status_message = self.sim_engine.randomize_obstacle()
        self.side.set_obstacle(True)

    def press_estop(self):
        # Üst çubuktaki mantar buton; zaten aktifse bir şey yapmaz, sıfırlama 'Sıfırla' ile
        if not self.sim_engine.e_stop_active and self.sim_engine.robotId is not None:
            self.trigger_estop(t("estop.manual"))

    def reset_camera(self):
        home = getattr(self, "camera_home", None)
        if not home: return
        with self.gpu_state_lock: self.gpu_shared_state.update(home)

    def set_language(self, lang):
        if lang == i18n.language(): return
        cfg = self.load_config()
        cfg["language"] = lang
        self.save_config(cfg)
        # Metinler pencere kurulurken atanıyor; yeni dil yeniden başlatınca tamamen geçerli olur
        if messagebox.askyesno(t("settings.restart_title"), t("settings.restart_prompt")):
            self.restart_requested = True
            self.on_closing()
        else:
            self.status_message = t("settings.restart_later")

    def clear_trail(self):
        self.kinematics_engine.clear_trail()

    def reset_estop(self):
        self.estop_reason = None
        with self.data_lock:
            self.status_message = self.sim_engine.reset_estop(self.shared_targets)
            for var in self.page_manual.slider_vars: var.set(0.0)
        self.side.set_obstacle(False)

    def toggle_workspace(self):
        state = self.kinematics_engine.toggle_workspace()
        self.side.set_trail(state)

    def apply_energy_settings(self):
        page = self.page_energy
        self.telemetry_engine.apply_motor_settings(page.motor_settings())
        self.status_message = self.telemetry_engine.apply_payload_settings(
            page.entry_payload.get(), page.entry_tariff.get(), page.entry_efficiency.get(), page.entry_robot_mass.get())
        page.entry_robot_mass.delete(0, "end")
        page.entry_robot_mass.insert(0, f"{self.telemetry_engine.robot_mass_kg:.1f}")
        page.show_motor_summary()

    def reset_energy_counter(self):
        self.status_message = self.telemetry_engine.reset_energy_counter()

    def trigger_fk_calculation(self):
        log = self.kinematics_engine.calculate_fk()
        self.side.set_report(log)
        self.save_position()

    def trigger_ik_calculation(self, x_val, y_val, z_val):
        with self.data_lock:
            current_angles = [float(v.get()) for v in self.page_manual.slider_vars]
            
        log, has_risk, err, desc = self.kinematics_engine.calculate_ik(x_val, y_val, z_val, current_angles)
        self.side.set_report(log)
        k = self.kinematics_engine
        safety = k.pending_waypoint_safety if len(k.pending_waypoint_safety) == len(k.pending_waypoints) else \
            [k.check_angles_safety(wp) for wp in k.pending_waypoints]
        if err > 0.03:
            safety = [(False, f"{err * 100:.1f} cm")] * len(safety)
        playable = not has_risk and err <= 0.03
        coords = dict(x=x_val or "?", y=y_val or "?", z=z_val or "?")
        if not playable: summary = t("route.summary_blocked", **coords)
        elif len(k.pending_waypoints) > 1: summary = t("route.summary_detour", n=len(k.pending_waypoints), **coords)
        else: summary = t("route.summary_direct", **coords)
        self.timeline.set_route(k.pending_waypoint_labels, safety, getattr(k, "route_segment_energy", []), summary,
                                playable, k.time_fast, k.time_eco)

        if has_risk or err > 0.03:
            fail_reason = desc if has_risk else t("msg.unreachable", cm=f"{err * 100:.1f}")
            self.status_message = t("msg.ik_blocked", reason=fail_reason)
        else:
            self.status_message = t("msg.ik_ok")

    def execute_trajectory(self, strategy):
        duration = self.kinematics_engine.time_fast if strategy == "fast" else self.kinematics_engine.time_eco
        self.start_trajectory(strategy, self.kinematics_engine.pending_waypoints, duration,
                              t("msg.route_started", name=t("msg.fast" if strategy == "fast" else "msg.eco")))

    def start_trajectory(self, strategy, waypoints, duration, status_text):
        if self.sim_engine.e_stop_active:
            self.status_message = t("msg.estop_active")
            return
        if getattr(self, "is_trajectory_playing", False):
            self.status_message = t("msg.busy")
            return

        # Bitince butonlar önceki hâline döner (ör. riskli rotada kilitli kalmaya devam etmeli)
        self.traj_btn_states = self.timeline.play_states()
        self.timeline.set_play_enabled(False)
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
        try:
            self._play_waypoints(strategy, waypoints, current, durations)
        finally:
            self.telemetry_engine.clear_playback_state()

        if not self.sim_engine.e_stop_active: self.status_message = t("msg.arrived")
        # Tk thread-safe değil: kayıt ve buton güncellemesi update_ui_loop'ta (ana thread) yapılır
        self.trajectory_finished = True

    def _play_waypoints(self, strategy, waypoints, current, durations):
        for wp_idx, target_angles in enumerate(waypoints):
            duration_per_wp = durations[wp_idx]
            steps = int(max(duration_per_wp * 30, 5))
            self.kinematics_engine.update_link_positions()

            for step in range(1, steps + 1):
                if self.sim_engine.e_stop_active: break
                angles, qd, qdd = self.kinematics_engine.profile_state(current, target_angles, duration_per_wp, strategy, step / steps)

                with self.sim_engine.bullet_lock:
                    if self.sim_engine.e_stop_active: break
                    for i, joint_idx in enumerate(self.sim_engine.revolute_joints):
                        if i < len(angles): p.resetJointState(self.sim_engine.robotId, joint_idx, angles[i])
                    if self.kinematics_engine.show_workspace: self.kinematics_engine.draw_swept_volume()
                    # Eklemler doğrudan konumlandığı için fizik tork görmez; güç grafiği ters dinamikten beslenir
                    if len(angles) == len(self.sim_engine.revolute_joints) and self.telemetry_engine.dynamics_ok:
                        tau = self.telemetry_engine.joint_torques(angles, qd, qdd)
                        self.telemetry_engine.set_playback_state(qd, tau)

                with self.data_lock:
                    if self.sim_engine.e_stop_active: break
                    for i in range(min(len(angles), len(self.shared_targets))):
                        self.shared_targets[i] = angles[i]

                time.sleep(duration_per_wp / steps)

            if self.sim_engine.e_stop_active: break
            current = target_angles

    def route_position(self, angles, waypoints):
        """Robot rotadaki bir waypoint'te duruyorsa onun indeksi, değilse None. Sadece TCP zincirindeki
        eklemlere bakılır; gripper parmakları IK'dan sonra değişmiş olabilir."""
        chain = self.kinematics_engine.ee_chain_joints()
        cols = [k for k, j in enumerate(self.sim_engine.revolute_joints) if j in chain and k < len(angles)]
        for idx, wp in enumerate(waypoints):
            if all(abs(angles[k] - wp[k]) < 1e-3 for k in cols if k < len(wp)):
                return idx
        return None

    def on_blocked_stop(self, idx, reason):
        self.status_message = t("route.blocked", n=idx + 1, reason=reason)

    def go_to_ik_step(self, target_idx):
        waypoints = self.kinematics_engine.pending_waypoints
        if not 0 <= target_idx < len(waypoints): return
        with self.data_lock:
            current = list(self.shared_targets)

        # Aradaki manevralar engel kaçışının parçası olabilir, atlanmaz: robot rotada bir noktadaysa oradan,
        # değilse baştan başlayarak hedefe kadar sırayla oynatılır (hedef gerideyse ters sırayla).
        at = self.route_position(current, waypoints)
        if at == target_idx:
            self.status_message = t("msg.already_at", n=target_idx + 1)
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
                messagebox.showwarning(t("msg.collision_title"), t("msg.cannot_go", n=idx + 1, desc=desc))
                self.status_message = t("msg.move_cancelled", n=idx + 1, desc=desc)
                return
            # Her segment Eko rotanın S-eğrisi ve süre ölçeğiyle
            max_deg = max([abs(math.degrees(a - c)) for a, c, lin in zip(wp, prev, prismatic) if not lin] or [0.0])
            steps.append(wp); durations.append(max(0.5, max_deg / 100.0) * 1.8); prev = wp

        label = t("msg.waypoint", n=sequence[0] + 1) if len(sequence) == 1 else t("msg.waypoints", a=sequence[0] + 1, b=sequence[-1] + 1)
        self.start_trajectory("eco", steps, durations, t("msg.playing", label=label))

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
        # Yeni modelin kendi ağırlığı alana yazılır; yük, tarife ve motor alanları korunur ve yeni gövdeye uygulanır
        page = self.page_energy
        page.rebuild_axis_rows()
        page.entry_robot_mass.delete(0, "end")
        page.entry_robot_mass.insert(0, f"{self.telemetry_engine.robot_mass_default:.1f}")
        source = self.telemetry_engine.mass_source
        page.lbl_mass_source.configure(text=t("energy.mass_source", source=t(source)) + (t("energy.density_note") if source != "mass.urdf" else ""))
        self.apply_energy_settings()

        # Model bilgisi (üst çubuk ve Model paneli), kamera başlangıcı, eski rota temizlenir
        path = getattr(self, "current_source_path", None) or self.builtin_urdf_path
        is_builtin = os.path.normpath(path) == os.path.normpath(self.builtin_urdf_path)
        name = t("model.builtin_name") if is_builtin else os.path.splitext(os.path.basename(path))[0].replace(".urdf", "")
        tel, kin = self.telemetry_engine, self.kinematics_engine
        n_axes = len(self.sim_engine.revolute_joints)
        self.lbl_model_line.configure(text=f"— {name} · {t('topbar.axes', n=n_axes)} · {tel.robot_mass_kg:.1f} kg")
        self.page_model.update_info(name, os.path.basename(path), n_axes, tel.robot_mass_kg, t(tel.mass_source), kin.robot_reach_cm)
        with self.gpu_state_lock:
            self.camera_home = {k: self.gpu_shared_state.get(k) for k in ("cam_dist", "cam_target", "cam_yaw", "cam_pitch")}
        kin.pending_waypoints, kin.pending_waypoint_labels, kin.pending_waypoint_safety = [], [], []
        self.timeline.clear()
        self.side.set_obstacle(self.sim_engine.obstacle_id is not None)
        self.status_message = t("status.model_loaded", name=name, n=n_axes)

        # Hedef listesi ve konum geçmişi modelin eksen sayısına göre yeniden boyutlanır;
        # sabit 6 elemanlı liste 7. ve sonraki eksenlere (ör. gripper) komut gitmesini engelliyordu.
        n_joints = len(self.sim_engine.revolute_joints)
        with self.data_lock:
            self.shared_targets = [0.0] * n_joints
            self.position_history = [[0.0] * n_joints]
            self.history_idx = 0
        self.page_manual.rebuild_joint_sliders(self.sim_engine.revolute_joints, self.sim_engine.robotId, self.make_scroll_smooth)
        self.page_target.update_limits()
        self._estop_ui = None  # yeni eksen satırlarına E-stop kilidi bir sonraki UI tick'inde uygulanır

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

    def set_render_quality(self, preset):
        with self.gpu_state_lock: self.gpu_shared_state['preset'] = preset

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
            self.page_model.set_default_info(t("model.default_custom", name=os.path.basename(saved)), True)
        else:
            self.page_model.set_default_info(t("model.default_builtin"), False)

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
        self.current_source_path = self.builtin_urdf_path
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
                if not reason: reason = self.telemetry_engine.detect_overload(targets)
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

        self.page_jog.redraw_gauges()

        # Slider'lar yukarıda son hedeflere eşitlendi; kayıt ondan sonra, oynatma bayrağı en son kapanır
        if getattr(self, "trajectory_finished", False):
            self.trajectory_finished = False
            self.save_position()
            self.is_trajectory_playing = False
            self.timeline.restore_play_states(getattr(self, "traj_btn_states", ("normal", "normal")))

        img = None
        while not self.q_img.empty():
            try: img = self.q_img.get_nowait()
            except queue.Empty: break
        if img is not None:
            self.tk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(600, 800))
            self.camera_label.configure(image=self.tk_img, text="")

        estop = self.sim_engine.e_stop_active
        if estop:
            reason = getattr(self, "estop_reason", None) or t("estop.collision")
            self.status_message = f"E-STOP: {reason}"
        self._apply_estop_ui(estop)

        playing = getattr(self, "is_trajectory_playing", False)
        level = "err" if estop else ("busy" if playing else ("warn" if self.status_message.upper().startswith(t("msg.warn_prefix")) else "ok"))
        self.side.set_status(self.status_message, level)
        pill = {"err": ("pill.estop", theme.ERR), "busy": ("pill.moving", theme.STEEL), "warn": ("pill.warning", theme.WARN),
                "ok": ("pill.ready", theme.OK)}[level]
        if getattr(self, "_pill", None) != pill:
            self._pill = pill
            self.lbl_pill.configure(text=t(pill[0]), text_color=pill[1])

        tel = self.telemetry_engine
        self.side.update_power(tel.current_power, tel.cumulative_energy_kwh, tel.cumulative_cost_tl, tel.power_history, estop)

        # ~5 Hz: uç konumu, rota şeridindeki konum, hedef haritası
        if self.gc_ticks % 6 == 0 and self.sim_engine.robotId is not None:
            with self.sim_engine.bullet_lock:
                tcp = [v * 100 for v in p.getLinkState(self.sim_engine.robotId, self.sim_engine.end_effector_index)[4]]
            self.side.set_tcp(tcp)
            self.side.set_load(tel.payload_kg, tel.robot_mass_kg)
            waypoints = self.kinematics_engine.pending_waypoints
            if waypoints:
                with self.data_lock: current = list(self.shared_targets)
                self.timeline.set_position(self.route_position(current, waypoints))
            if self.current_page == "target":
                obs = None
                if self.sim_engine.obstacle_id is not None:
                    o, h = self.sim_engine.obstacle_pos, self.sim_engine.obstacle_half
                    obs = (round(o[0] * 100, 1), round(o[1] * 100, 1), round(h[0] * 100, 1), round(h[1] * 100, 1))
                self.page_target.refresh_map(tcp, obs)

        self.after(30, self.update_ui_loop)

    def _apply_estop_ui(self, estop):
        """E-stop görünümü sadece durum değişince güncellenir (kart yeniden çizimi pahalı)."""
        if estop == getattr(self, "_estop_ui", None): return
        self._estop_ui = estop
        self.side.set_estop(estop)
        self.stage.configure(border_color=theme.ERR if estop else theme.BORDER, border_width=2 if estop else 1)
        self.page_jog.set_locked(estop)

    def on_closing(self):
        self.is_running = False
        with self.gpu_state_lock: self.gpu_shared_state['running'] = False
        self.quit()
        self.destroy()

if __name__ == "__main__":
    app = RobotKontrolApp()
    app.mainloop()
    if getattr(app, "restart_requested", False):
        # Dil değişikliği: aynı komutla yeni bir süreç başlat
        subprocess.Popen([sys.executable] + sys.argv, cwd=os.getcwd())