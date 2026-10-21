import customtkinter as ctk
import math
import pybullet as p

class ManualPage(ctk.CTkFrame):
    def __init__(self, parent, app, sim_engine):
        super().__init__(parent, fg_color="transparent")
        
        self.app = app
        self.sim_engine = sim_engine
        
        title = ctk.CTkLabel(self, text="Eksen Manipülasyonu", font=ctk.CTkFont(size=18, weight="bold"))
        title.pack(pady=(10, 15))

        btn_card = ctk.CTkFrame(self, fg_color="#242424", corner_radius=10)
        btn_card.pack(fill="x", pady=(0, 15))

        self.btn_base = ctk.CTkButton(btn_card, text="Base Location", height=35, corner_radius=6,
                                      command=self.app.go_to_base)
        self.btn_base.pack(pady=(15, 10), padx=15, fill="x")

        nav_frame = ctk.CTkFrame(btn_card, fg_color="transparent")
        nav_frame.pack(pady=(0, 15), padx=15, fill="x")
        nav_frame.grid_columnconfigure(0, weight=1, uniform="group1")
        nav_frame.grid_columnconfigure(1, weight=1, uniform="group1")

        self.btn_undo = ctk.CTkButton(nav_frame, text="Önceki Konum", height=35, corner_radius=6,
                                      command=self.app.undo_position, fg_color="#4A4A4A", hover_color="#3A3A3A")
        self.btn_undo.grid(row=0, column=0, padx=(0, 5), sticky="ew")

        self.btn_redo = ctk.CTkButton(nav_frame, text="Sonraki Konum", height=35, corner_radius=6,
                                      command=self.app.redo_position)
        self.btn_redo.grid(row=0, column=1, padx=(5, 0), sticky="ew")

        self.slider_card = ctk.CTkScrollableFrame(self, fg_color="#242424", corner_radius=10)
        self.slider_card.pack(fill="both", expand=True)

        self.sliders = []
        self.slider_vars = []

    def rebuild_joint_sliders(self, revolute_joints, robotId, make_scroll_smooth):
        for widget in self.slider_card.winfo_children():
            widget.destroy()

        self.sliders = []
        self.slider_vars = []

        if not revolute_joints or robotId is None:
            empty_lbl = ctk.CTkLabel(self.slider_card, text="Aktif eklem bulunamadı.", font=ctk.CTkFont(size=12))
            empty_lbl.pack(pady=20)
            return

        with self.sim_engine.bullet_lock:
            for i, joint_idx in enumerate(revolute_joints):
                info = p.getJointInfo(robotId, joint_idx)
                raw_name = info[1].decode('utf-8')
                low = float(info[8])
                high = float(info[9])

                if low >= high or (abs(low) < 1e-5 and abs(high) < 1e-5):
                    low, high = -math.pi, math.pi

                if info[2] == p.JOINT_PRISMATIC:
                    range_text = f"{low * 1000:.0f} mm .. {high * 1000:.0f} mm"
                else:
                    range_text = f"{math.degrees(low):.0f}° .. {math.degrees(high):.0f}°"

                lbl_text = f"Eksen J{i+1}: {raw_name} ({range_text})"
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
                slider.bind("<ButtonRelease-1>", lambda event: self.app.save_position())
                self.sliders.append(slider)

        make_scroll_smooth(self.slider_card, step_px=50)
