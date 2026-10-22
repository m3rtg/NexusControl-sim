import customtkinter as ctk
import pybullet as p
from ui import theme
from ui.i18n import t

SOURCE_KEYS = ("auto", "total", "labels")

class EnergyPage(ctk.CTkScrollableFrame):
    """Yük, robot ağırlığı, tarife, verim, bekleme gücü ve motor verisi kaynağı (otomatik / toplam güç / eksen etiketleri)."""
    def __init__(self, parent, app, telemetry_engine, sim_engine):
        super().__init__(parent, fg_color="transparent", scrollbar_button_color=theme.BORDER2,
                         scrollbar_button_hover_color=theme.MUTED)
        self.app, self.telemetry, self.sim_engine = app, telemetry_engine, sim_engine
        self.axis_rows = []
        self._source_labels = {k: t(f"energy.src_{k}") for k in SOURCE_KEYS}
        self._source_by_label = {v: k for k, v in self._source_labels.items()}

        theme.section(self, t("energy.section"), t("energy.title")).pack(fill="x", pady=(0, 10))

        # --- yük ve tarife ---
        box = self._box(t("energy.load_section"))
        grid = ctk.CTkFrame(box, fg_color="transparent")
        grid.pack(fill="x", padx=12, pady=(0, 4))
        grid.grid_columnconfigure((0, 1), weight=1, uniform="energy")
        self.entry_payload = self._field(grid, 0, 0, t("energy.payload"), "0.0")
        self.entry_robot_mass = self._field(grid, 0, 1, t("energy.robot_mass"), "")
        self.entry_tariff = self._field(grid, 1, 0, t("energy.tariff"), "4.50")
        self.entry_efficiency = self._field(grid, 1, 1, t("energy.efficiency"), "85")
        self.entry_standby = self._field(grid, 2, 0, t("energy.standby"), "15")
        self.lbl_mass_source = ctk.CTkLabel(box, text="", font=theme.font(10), text_color=theme.MUTED, anchor="w")
        self.lbl_mass_source.pack(fill="x", padx=12, pady=(0, 10))

        # --- motor verisi ---
        box = self._box(t("energy.motor_section"))
        self.source_selector = theme.segmented(box, list(self._source_labels.values()), command=self._on_source_changed)
        self.source_selector.set(self._source_labels["auto"])
        self.source_selector.pack(fill="x", padx=12, pady=(0, 6))
        self.lbl_source_info = ctk.CTkLabel(box, text="", font=theme.font(10), text_color=theme.FG2, justify="left",
                                            anchor="w", wraplength=280)
        self.lbl_source_info.pack(fill="x", padx=12, pady=(0, 8))

        self.total_frame = ctk.CTkFrame(box, fg_color="transparent")
        self.total_frame.grid_columnconfigure(0, weight=1)
        self.entry_total_power = self._field(self.total_frame, 0, 0, t("energy.total_power"), "")

        self.labels_frame = ctk.CTkFrame(box, fg_color="transparent")
        header = ctk.CTkFrame(self.labels_frame, fg_color="transparent")
        header.pack(fill="x")
        for col, (key, w) in enumerate((("energy.col_axis", 30), ("energy.col_type", 70), ("energy.col_power", 52),
                                        ("energy.col_torque", 60), ("energy.col_gear", 52))):
            ctk.CTkLabel(header, text=t(key), width=w, font=theme.font(9), text_color=theme.MUTED, height=14).grid(row=0, column=col, padx=1)
        self.rows_frame = ctk.CTkFrame(self.labels_frame, fg_color="transparent")
        self.rows_frame.pack(fill="x")
        theme.button(self.labels_frame, t("energy.copy_first"), self._copy_first_row, kind="ghost", height=26,
                     font=theme.font(10)).pack(fill="x", pady=(6, 0))

        theme.button(self, t("energy.apply"), self.app.apply_energy_settings, kind="primary", height=36).pack(fill="x", pady=(2, 8))
        self.lbl_motor_summary = ctk.CTkLabel(self, text="", font=theme.font(10, mono=True), text_color=theme.FG2,
                                              justify="left", anchor="w")
        self.lbl_motor_summary.pack(fill="x", pady=(0, 6))
        self._on_source_changed(self._source_labels["auto"])

    def _box(self, title):
        box = ctk.CTkFrame(self, fg_color=theme.BG, corner_radius=theme.RADIUS_SM + 2, border_width=1, border_color=theme.BORDER)
        box.pack(fill="x", pady=(0, 10))
        ctk.CTkLabel(box, text=title, font=theme.font(11, "bold"), text_color=theme.GOLD, anchor="w").pack(fill="x", padx=12, pady=(8, 4))
        return box

    @staticmethod
    def _field(parent, row, col, label, default):
        cell = ctk.CTkFrame(parent, fg_color="transparent")
        cell.grid(row=row, column=col, sticky="ew", padx=(0, 4) if col == 0 else (4, 0), pady=2)
        ctk.CTkLabel(cell, text=label, font=theme.font(10), text_color=theme.MUTED, anchor="w", height=14).pack(fill="x")
        e = theme.entry(cell, height=28)
        e.insert(0, default)
        e.pack(fill="x")
        return e

    def _on_source_changed(self, label):
        mode = self._source_by_label[label]
        self.total_frame.pack_forget()
        self.labels_frame.pack_forget()
        self.lbl_source_info.configure(text=t(f"energy.src_{mode}_info"))
        if mode == "total": self.total_frame.pack(fill="x", padx=12, pady=(0, 10))
        elif mode == "labels": self.labels_frame.pack(fill="x", padx=12, pady=(0, 10))

    def rebuild_axis_rows(self):
        """Model yüklenince eksen sayısı kadar satır; önceki değerler aynı sıradaki eksenlerde korunur."""
        old = [self._read_row(r) for r in self.axis_rows]
        for w in self.rows_frame.winfo_children(): w.destroy()
        self.axis_rows = []
        robot, joints = self.sim_engine.robotId, self.sim_engine.revolute_joints
        for i, j_idx in enumerate(joints):
            linear = robot is not None and p.getJointInfo(robot, j_idx)[2] == p.JOINT_PRISMATIC
            row = ctk.CTkFrame(self.rows_frame, fg_color="transparent")
            row.pack(fill="x", pady=1)
            ctk.CTkLabel(row, text=f"J{i + 1}", width=30, font=theme.font(10, "bold"), text_color=theme.MUTED).grid(row=0, column=0, padx=1)
            kind = theme.option_menu(row, ["Servo", "Step"], width=70, height=24)
            kind.grid(row=0, column=1, padx=1)
            entries = []
            for col, w in ((2, 52), (3, 60), (4, 52)):
                e = theme.entry(row, width=w, height=24, font=theme.font(10), placeholder_text="kg" if (linear and col == 3) else "")
                e.grid(row=0, column=col, padx=1)
                entries.append(e)
            entries[2].insert(0, "1")
            r = {"kind": kind, "power": entries[0], "torque": entries[1], "gear": entries[2], "linear": linear}
            if i < len(old): self._write_row(r, old[i])
            self.axis_rows.append(r)

    @staticmethod
    def _read_row(r):
        return {"kind": r["kind"].get(), "power": r["power"].get(), "torque": r["torque"].get(), "gear": r["gear"].get()}

    @staticmethod
    def _write_row(r, values):
        r["kind"].set(values["kind"])
        for key in ("power", "torque", "gear"):
            r[key].delete(0, "end")
            r[key].insert(0, values[key])

    def _copy_first_row(self):
        if not self.axis_rows: return
        first = self._read_row(self.axis_rows[0])
        for r in self.axis_rows[1:]: self._write_row(r, first)

    def select_source(self, mode):
        label = self._source_labels[mode]
        self.source_selector.set(label)
        self._on_source_changed(label)

    def motor_settings(self):
        """Ham alan değerleri; TelemetryEngine.apply_motor_settings ayrıştırır."""
        return {
            "mode": self._source_by_label[self.source_selector.get()],
            "total_power": self.entry_total_power.get(),
            "standby": self.entry_standby.get(),
            "axes": [dict(self._read_row(r), linear=r["linear"]) for r in self.axis_rows],
        }

    def show_motor_summary(self):
        lines = [t("energy.summary_header")]
        for i, m in enumerate(self.telemetry.motor_specs):
            unit = "N" if m.get("linear") else "N·m"
            loss = f"{m['constant_w']:.1f} W" if m["kind"] == "stepper" else f"k={m['copper_k']:.4f}"
            lines.append(f"J{i + 1:<5}{'Step' if m['kind'] == 'stepper' else 'Servo':<7}{m['limit']:>9.1f} {unit:<4}{loss:>12}")
        self.lbl_motor_summary.configure(text="\n".join(lines))
