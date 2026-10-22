import tkinter as tk
import customtkinter as ctk
from ui import theme
from i18n import t
from ui.widgets.estop_button import EStopButton

class SidePanel(ctk.CTkFrame):
    """Sağ sütun: canlı güç, sahne araçları, durum ve hesap raporu."""
    def __init__(self, parent, app):
        super().__init__(parent, fg_color="transparent")
        self.app = app
        self._spark_key = None
        self._report_text = ""
        self._status_key = None

        # --- Güç ---
        power = theme.card(self)
        power.pack(fill="x")
        top = ctk.CTkFrame(power, fg_color="transparent")
        top.pack(fill="x", padx=12, pady=(10, 0))
        theme.section(top, t("power.section")).pack(side="left", anchor="n")
        theme.button(top, "↺", app.reset_energy_counter, kind="ghost", width=26, height=22).pack(side="right")
        self.lbl_power = ctk.CTkLabel(power, text="0 W", font=theme.font(22, "bold", mono=True), text_color=theme.FG,
                                      anchor="w", height=30)
        self.lbl_power.pack(fill="x", padx=12)
        self.spark = tk.Canvas(power, height=38, bg=theme.BG2, highlightthickness=0)
        self.spark.pack(fill="x", padx=12, pady=(2, 4))
        self.lbl_counter = self._row(power, t("power.counter"))
        self.lbl_cost = self._row(power, t("power.cost"), pady=(0, 10))

        # --- Sahne ---
        scene = theme.card(self)
        scene.pack(fill="x", pady=(theme.PAD, 0))
        theme.section(scene, t("scene.section")).pack(fill="x", padx=12, pady=(10, 6))
        grid = ctk.CTkFrame(scene, fg_color="transparent")
        grid.pack(fill="x", padx=12, pady=(0, 12))
        grid.grid_columnconfigure((0, 1), weight=1, uniform="scene")
        self.btn_obstacle = theme.button(grid, t("scene.obstacle"), app.toggle_obstacle, width=60, height=30, font=theme.font(11))
        self.btn_obstacle.grid(row=0, column=0, sticky="ew", padx=(0, 3), pady=(0, 6))
        theme.button(grid, t("scene.random"), app.randomize_obstacle, width=60, height=30, font=theme.font(11)).grid(
            row=0, column=1, sticky="ew", padx=(3, 0), pady=(0, 6))
        self.btn_trail = theme.button(grid, t("scene.trail"), app.toggle_workspace, width=60, height=30, font=theme.font(11))
        self.btn_trail.grid(row=1, column=0, sticky="ew", padx=(0, 3), pady=(0, 6))
        theme.button(grid, t("scene.clear_trail"), app.clear_trail, width=60, height=30, font=theme.font(11)).grid(
            row=1, column=1, sticky="ew", padx=(3, 0), pady=(0, 6))
        theme.button(grid, t("scene.camera"), app.reset_camera, width=60, height=30, font=theme.font(11)).grid(
            row=2, column=0, columnspan=2, sticky="ew")

        # --- Acil stop + durum ---
        status = theme.card(self)
        status.pack(fill="x", pady=(theme.PAD, 0))
        theme.section(status, t("status.section")).pack(fill="x", padx=12, pady=(10, 6))
        body = ctk.CTkFrame(status, fg_color="transparent")
        body.pack(fill="x", padx=12, pady=(0, 10))
        body.grid_columnconfigure(1, weight=1)
        stop_col = ctk.CTkFrame(body, fg_color="transparent")
        stop_col.grid(row=0, column=0, sticky="n", padx=(0, 12))
        self.estop_btn = EStopButton(stop_col, app.press_estop, size=60, bg=theme.BG2)
        self.estop_btn.pack()
        self.btn_reset = theme.button(stop_col, t("topbar.reset"), app.reset_estop, kind="danger", width=60, height=24,
                                      font=theme.font(10, "bold"))
        info = ctk.CTkFrame(body, fg_color="transparent")
        info.grid(row=0, column=1, sticky="nsew")
        self.lbl_status = ctk.CTkLabel(info, text="", font=theme.font(11), text_color=theme.OK, anchor="w",
                                       justify="left", wraplength=168)
        self.lbl_status.pack(fill="x")
        self.lbl_tcp = self._row(info, t("status.tcp"), padx=0)
        self.lbl_load = self._row(info, t("status.load"), padx=0)

        # --- Rapor ---
        report = theme.card(self)
        report.pack(fill="both", expand=True, pady=(theme.PAD, 0))
        theme.section(report, t("report.section")).pack(fill="x", padx=12, pady=(10, 4))
        self.report = ctk.CTkTextbox(report, font=theme.font(10, mono=True), fg_color=theme.BG, text_color=theme.FG2,
                                     border_width=1, border_color=theme.BORDER, corner_radius=theme.RADIUS_SM, wrap="none")
        self.report.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.set_report(t("report.empty"))

    def _row(self, parent, label, pady=(0, 2), padx=12):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", padx=padx, pady=pady)
        ctk.CTkLabel(row, text=label, font=theme.font(10), text_color=theme.MUTED, height=16).pack(side="left")
        value = ctk.CTkLabel(row, text="", font=theme.font(10, mono=True), text_color=theme.FG, height=16)
        value.pack(side="right")
        return value

    def update_power(self, watts, kwh, cost, history, estop):
        self.lbl_power.configure(text=f"{watts:.0f} W")
        self.lbl_counter.configure(text=f"{kwh:.4f} kWh")
        self.lbl_cost.configure(text=f"{cost:.3f} TL")
        key = (len(history), round(history[-1], 1) if history else 0, estop, self.spark.winfo_width())
        if key == self._spark_key: return
        self._spark_key = key
        c = self.spark
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10 or len(history) < 2: return
        peak = max(1.0, max(history) * 1.15)
        pts = []
        for i, v in enumerate(history):
            pts += [i / (len(history) - 1) * w, h - 2 - v / peak * (h - 4)]
        c.create_polygon([0, h] + pts + [w, h], fill=theme.ERR_SUBTLE if estop else theme.GOLD_SUBTLE, outline="")
        c.create_line(*pts, fill=theme.ERR if estop else theme.GOLD, width=1.6, smooth=True)

    def set_status(self, text, level="ok"):
        if (text, level) == self._status_key: return
        self._status_key = (text, level)
        color = {"ok": theme.OK, "warn": theme.WARN, "err": theme.ERR, "busy": theme.STEEL}[level]
        self.lbl_status.configure(text=f"● {text}", text_color=color)

    def set_tcp(self, xyz_cm):
        self.lbl_tcp.configure(text=" · ".join(f"{v:.0f}" for v in xyz_cm) + " cm")

    def set_load(self, payload, robot):
        self.lbl_load.configure(text=t("status.load_value", payload=f"{payload:.1f}", robot=f"{robot:.0f}"))

    def set_report(self, text):
        if text == self._report_text: return
        self._report_text = text
        self.report.configure(state="normal")
        self.report.delete("0.0", "end")
        self.report.insert("end", text.strip("\n"))
        self.report.configure(state="disabled")

    def set_estop(self, active):
        self.estop_btn.set_active(active)
        if active: self.btn_reset.pack(pady=(6, 0))
        else: self.btn_reset.pack_forget()

    def set_obstacle(self, on):
        theme.set_button_kind(self.btn_obstacle, "hot" if on else "secondary")

    def set_trail(self, on):
        theme.set_button_kind(self.btn_trail, "hot" if on else "secondary")
