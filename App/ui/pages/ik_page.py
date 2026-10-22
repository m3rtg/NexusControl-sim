import customtkinter as ctk
import math

class IKPage(ctk.CTkScrollableFrame):
    def __init__(self, parent, app, kinematics_engine, telemetry_engine):
        super().__init__(parent, fg_color="transparent")
        
        self.app = app
        self.kinematics = kinematics_engine
        self.telemetry = telemetry_engine
        
        title = ctk.CTkLabel(self, text="Ters Kinematik (IK)", font=ctk.CTkFont(size=16, weight="bold"))
        title.pack(pady=(10, 12))

        ik_card = ctk.CTkFrame(self, fg_color="#242424", corner_radius=10)
        ik_card.pack(fill="x", pady=(0, 10))

        # --- SINIR GÖSTERGELERİ ---
        self.var_x = ctk.StringVar()
        self.var_y = ctk.StringVar()
        self.var_z = ctk.StringVar()
        
        self.var_x.trace_add("write", self.update_limits)
        self.var_y.trace_add("write", self.update_limits)
        self.var_z.trace_add("write", self.update_limits)

        # X Kutusu
        frame_x = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_x.pack(fill="x", padx=15, pady=(15, 5))
        self.entry_x = ctk.CTkEntry(frame_x, placeholder_text="Hedef X (cm)", textvariable=self.var_x, height=38, border_color="#444")
        self.entry_x.pack(side="left", fill="x", expand=True)
        self.lbl_limit_x = ctk.CTkLabel(frame_x, text="±80 cm", text_color="gray", width=100, font=ctk.CTkFont(size=11), anchor="e")
        self.lbl_limit_x.pack(side="right", padx=(8, 0))

        # Y Kutusu
        frame_y = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_y.pack(fill="x", padx=15, pady=5)
        self.entry_y = ctk.CTkEntry(frame_y, placeholder_text="Hedef Y (cm)", textvariable=self.var_y, height=38, border_color="#444")
        self.entry_y.pack(side="left", fill="x", expand=True)
        self.lbl_limit_y = ctk.CTkLabel(frame_y, text="±80 cm", text_color="gray", width=100, font=ctk.CTkFont(size=11), anchor="e")
        self.lbl_limit_y.pack(side="right", padx=(8, 0))

        # Z Kutusu
        frame_z = ctk.CTkFrame(ik_card, fg_color="transparent")
        frame_z.pack(fill="x", padx=15, pady=5)
        self.entry_z = ctk.CTkEntry(frame_z, placeholder_text="Hedef Z (cm)", textvariable=self.var_z, height=38, border_color="#444")
        self.entry_z.pack(side="left", fill="x", expand=True)
        self.lbl_limit_z = ctk.CTkLabel(frame_z, text="0 - 106 cm", text_color="gray", width=100, font=ctk.CTkFont(size=11), anchor="e")
        self.lbl_limit_z.pack(side="right", padx=(8, 0))

        # 3B Anlık Erişim Bilgilendirme Çubuğu (KORUNAN KISIT)
        self.lbl_reach_status = ctk.CTkLabel(
            ik_card, text="● Hedef koordinatları girin\n3B uzanma analizi bekleniyor",
            font=ctk.CTkFont(size=11), text_color="gray",
            anchor="w", justify="left"
        )
        self.lbl_reach_status.pack(fill="x", pady=(4, 10), padx=15)

        self.btn_calc_ik = ctk.CTkButton(self, text="Yörünge ve Rotayı Hesapla", height=42, corner_radius=8,
                                         command=self.on_calculate_ik)
        self.btn_calc_ik.pack(pady=(5, 10), fill="x")

        self.traj_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.traj_frame.pack(fill="x", padx=10, pady=(5, 15))
        self.traj_frame.grid_columnconfigure(0, weight=1)
        self.traj_frame.grid_columnconfigure(1, weight=1)

        self.btn_fast_traj = ctk.CTkButton(self.traj_frame, text="⚡ Hızlı Rota", height=38, corner_radius=6,
                                           font=ctk.CTkFont(size=12, weight="bold"),
                                           fg_color="#8B0000", hover_color="#660000", state="disabled",
                                           command=lambda: self.app.execute_trajectory("fast"))
        self.btn_fast_traj.grid(row=0, column=0, padx=(0, 4), sticky="ew")

        self.btn_eco_traj = ctk.CTkButton(self.traj_frame, text="🌱 Eko Rota", height=38, corner_radius=6,
                                          font=ctk.CTkFont(size=12, weight="bold"),
                                          fg_color="#006400", hover_color="#004d00", state="disabled",
                                          command=lambda: self.app.execute_trajectory("eco"))
        self.btn_eco_traj.grid(row=0, column=1, padx=(4, 0), sticky="ew")

        step_title = ctk.CTkLabel(self, text="Manevra Hamleleri", font=ctk.CTkFont(size=14, weight="bold"))
        step_title.pack(pady=(10, 5))
        self.ik_steps_frame = ctk.CTkFrame(self, fg_color="#242424", corner_radius=10)
        self.ik_steps_frame.pack(fill="x", padx=10, pady=(0, 15))

    def on_calculate_ik(self):
        # Yük ve enerji ayarları soldaki menüde; hesaplamadan önce güncel değerler uygulanır
        self.app.apply_energy_settings()
        self.app.trigger_ik_calculation(
            self.entry_x.get(), 
            self.entry_y.get(), 
            self.entry_z.get()
        )

    def rebuild_waypoints(self, pending_waypoints, err, collision_risk, warning_text):
        for widget in self.ik_steps_frame.winfo_children(): widget.destroy()

        safety = self.kinematics.pending_waypoint_safety
        if len(safety) != len(pending_waypoints):
            safety = [self.kinematics.check_angles_safety(wp) for wp in pending_waypoints]

        step_num = 1
        for wp, (wp_safe, wp_desc) in zip(pending_waypoints, safety):
            btn_state = "normal" if (wp_safe and err <= 0.03) else "disabled"
            btn_color = "#005A9E" if btn_state == "normal" else "#555555"
            btn_text = f"Manevra {step_num} Konumuna Git" if wp_safe else f"Manevra {step_num} (Kilitli: {wp_desc})"
            btn = ctk.CTkButton(self.ik_steps_frame, text=btn_text, corner_radius=6,
                                command=lambda idx=step_num - 1: self.app.go_to_ik_step(idx),
                                state=btn_state,
                                fg_color=btn_color,
                                hover_color="#003A68")
            btn.pack(pady=5, padx=10, fill="x")
            step_num += 1

        if collision_risk or err > 0.03:
            self.btn_fast_traj.configure(state="disabled")
            self.btn_eco_traj.configure(state="disabled")
        else:
            self.btn_fast_traj.configure(state="normal")
            self.btn_eco_traj.configure(state="normal")

    def update_limits(self, *args):
        try: x_val = float(self.var_x.get().strip().replace(',', '.')) if self.var_x.get() else None
        except: x_val = None
        try: y_val = float(self.var_y.get().strip().replace(',', '.')) if self.var_y.get() else None
        except: y_val = None
        try: z_val = float(self.var_z.get().strip().replace(',', '.')) if self.var_z.get() else None
        except: z_val = None

        s_r = self.kinematics.shoulder_r_cm
        s_z = self.kinematics.shoulder_z_cm
        arm_len = self.kinematics.arm_length_cm
        max_r = self.kinematics.robot_reach_cm
        stand_top = self.kinematics.stand_top_cm
        stand_half = self.kinematics.stand_half_extent_cm
        min_floor = max(0.0, stand_top + (5.0 if stand_half > 0 else 0.0))
        global_max_z = s_z + arm_len

        self.lbl_limit_x.configure(text=f"±{max_r:.0f} cm", text_color="gray")

        if x_val is not None:
            if abs(x_val) > max_r:
                self.lbl_limit_y.configure(text="Menzil Dışı", text_color="#FF4444")
            else:
                max_y = math.sqrt(max(0.0, max_r ** 2 - x_val ** 2))
                self.lbl_limit_y.configure(text=f"±{max_y:.0f} cm", text_color="gray")
        else:
            self.lbl_limit_y.configure(text=f"±{max_r:.0f} cm", text_color="gray")

        if x_val is not None or y_val is not None:
            r_xy = math.sqrt((x_val or 0.0)**2 + (y_val or 0.0)**2)
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
            self.lbl_reach_status.configure(text="● Hedef koordinatlarını tamamlayın\nX, Y ve Z değerlerini girin", text_color="#AAAAAA")
        else:
            self.lbl_reach_status.configure(text="● Hedef koordinatları girin\n3B uzanma analizi bekleniyor", text_color="gray")
