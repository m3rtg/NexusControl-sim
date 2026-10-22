import customtkinter as ctk
import pybullet as p

MOTOR_SOURCES = {"Otomatik": "auto", "Toplam Güç": "total", "Eksen Etiketleri": "labels"}

class EnergyPage(ctk.CTkScrollableFrame):
    def __init__(self, parent, app, telemetry_engine, sim_engine):
        super().__init__(parent, fg_color="transparent")

        self.app = app
        self.telemetry = telemetry_engine
        self.sim_engine = sim_engine
        self.axis_rows = []

        ctk.CTkLabel(self, text="Güç ve Enerji", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(10, 12))

        # --- YÜK, ROBOT VE TARİFE ---
        card = self._card("⚙️ YÜK VE TARİFE")
        grid = ctk.CTkFrame(card, fg_color="transparent")
        grid.pack(fill="x", padx=15, pady=(2, 4))
        grid.grid_columnconfigure((0, 1), weight=1)
        self.entry_payload = self._field(grid, 0, 0, "Yük (Payload - kg):", "0.0")
        self.entry_robot_mass = self._field(grid, 0, 1, "Robot Ağırlığı (kg):", "")
        self.entry_tariff = self._field(grid, 1, 0, "Tarife (TL / kWh):", "4.50")
        self.entry_efficiency = self._field(grid, 1, 1, "Sürücü/Dişli Verimi (%):", "85")
        self.entry_standby = self._field(grid, 2, 0, "Bekleme Gücü (W / eksen):", "15")
        self.lbl_mass_source = ctk.CTkLabel(card, text="", font=ctk.CTkFont(size=10), text_color="#777777")
        self.lbl_mass_source.pack(padx=15, pady=(0, 8), anchor="w")

        # --- MOTOR VERİSİ ---
        card = self._card("🔌 MOTOR VERİSİ")
        self.source_selector = ctk.CTkSegmentedButton(card, values=list(MOTOR_SOURCES), command=self._on_source_changed)
        self.source_selector.set("Otomatik")
        self.source_selector.pack(fill="x", padx=15, pady=(2, 6))
        self.lbl_source_info = ctk.CTkLabel(card, text="", font=ctk.CTkFont(size=10), text_color="#AAAAAA", justify="left", anchor="w", wraplength=300)
        self.lbl_source_info.pack(fill="x", padx=15, pady=(0, 6))

        # Toplam güç
        self.total_frame = ctk.CTkFrame(card, fg_color="transparent")
        self.entry_total_power = self._field(self.total_frame, 0, 0, "Robotun Nominal Gücü (W):", "")
        self.total_frame.grid_columnconfigure(0, weight=1)

        # Eksen etiketleri: Tip | Güç (W) | Tork (kg·cm) | Dişli (1:x)
        self.labels_frame = ctk.CTkFrame(card, fg_color="transparent")
        header = ctk.CTkFrame(self.labels_frame, fg_color="transparent")
        header.pack(fill="x")
        for col, (text, w) in enumerate([("Eksen", 34), ("Tip", 76), ("Güç W", 56), ("Tork kg·cm", 64), ("Dişli 1:x", 56)]):
            ctk.CTkLabel(header, text=text, width=w, font=ctk.CTkFont(size=10), text_color="#AAAAAA").grid(row=0, column=col, padx=1)
        self.rows_frame = ctk.CTkFrame(self.labels_frame, fg_color="transparent")
        self.rows_frame.pack(fill="x")
        ctk.CTkButton(self.labels_frame, text="1. satırı tüm eksenlere kopyala", height=26, fg_color="#2A2A2A",
                      command=self._copy_first_row).pack(fill="x", pady=(6, 0))

        ctk.CTkButton(self, text="⚡ Uygula", height=38, corner_radius=8, fg_color="#332B10", hover_color="#4A3E14",
                      text_color="#FFD700", font=ctk.CTkFont(size=12, weight="bold"),
                      command=self.app.apply_energy_settings).pack(fill="x", pady=(4, 10))

        self.lbl_motor_summary = ctk.CTkLabel(self, text="", font=ctk.CTkFont(family="Consolas", size=10), text_color="#AAAAAA", justify="left", anchor="w")
        self.lbl_motor_summary.pack(fill="x", padx=5, pady=(0, 10))

        self._on_source_changed("Otomatik")

    def _card(self, title):
        card = ctk.CTkFrame(self, fg_color="#242424", corner_radius=10)
        card.pack(fill="x", pady=(0, 10))
        ctk.CTkLabel(card, text=title, font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"), text_color="#FFB300").pack(pady=(8, 4), padx=15, anchor="w")
        return card

    @staticmethod
    def _field(parent, row, col, label, default):
        cell = ctk.CTkFrame(parent, fg_color="transparent")
        cell.grid(row=row, column=col, sticky="ew", padx=(0, 5) if col == 0 else (5, 0), pady=2)
        ctk.CTkLabel(cell, text=label, font=ctk.CTkFont(size=11), text_color="#AAAAAA").pack(anchor="w")
        entry = ctk.CTkEntry(cell, height=30)
        entry.insert(0, default)
        entry.pack(fill="x", pady=(2, 0))
        return entry

    def _on_source_changed(self, value):
        mode = MOTOR_SOURCES[value]
        self.total_frame.pack_forget()
        self.labels_frame.pack_forget()
        if mode == "auto":
            self.lbl_source_info.configure(text="Motorlar robotun ağırlığından boyutlandırılır: her eksen kendi yerçekimi "
                                                "yükünün 2 katına göre, servo kabul edilir.")
        elif mode == "total":
            self.lbl_source_info.configure(text="Robotun etiket/katalog gücü eksenlere yük oranında dağıtılır (servo). "
                                                "Tork sınırı bu güçten çıkar.")
            self.total_frame.pack(fill="x", padx=15, pady=(0, 10))
        else:
            self.lbl_source_info.configure(text="Her eksenin motor etiketi. Step motor sürücüsü yükten bağımsız sabit akım "
                                                "çeker (etiketteki W); tork sınırı = tutma torku x dişli oranı.")
            self.labels_frame.pack(fill="x", padx=15, pady=(0, 10))

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
            ctk.CTkLabel(row, text=f"J{i + 1}", width=34, font=ctk.CTkFont(size=11)).grid(row=0, column=0, padx=1)
            kind = ctk.CTkOptionMenu(row, values=["Servo", "Step"], width=76, height=26)
            kind.grid(row=0, column=1, padx=1)
            entries = []
            for col, w in ((2, 56), (3, 64), (4, 56)):
                e = ctk.CTkEntry(row, width=w, height=26, placeholder_text="kg" if (linear and col == 3) else "")
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

    def motor_settings(self):
        """Ham alan değerleri; TelemetryEngine.apply_motor_settings ayrıştırır."""
        return {
            "mode": MOTOR_SOURCES[self.source_selector.get()],
            "total_power": self.entry_total_power.get(),
            "standby": self.entry_standby.get(),
            "axes": [dict(self._read_row(r), linear=r["linear"]) for r in self.axis_rows],
        }

    def show_motor_summary(self):
        tel = self.telemetry
        lines = [f"{'Eksen':<6}{'Tip':<7}{'Tork sınırı':>13}{'Sabit/Bakır':>14}"]
        for i, m in enumerate(tel.motor_specs):
            unit = "N" if m.get("linear") else "N·m"
            loss = f"{m['constant_w']:.1f} W" if m["kind"] == "stepper" else f"k={m['copper_k']:.4f}"
            lines.append(f"J{i + 1:<5}{'Step' if m['kind'] == 'stepper' else 'Servo':<7}{m['limit']:>9.1f} {unit:<3}{loss:>14}")
        self.lbl_motor_summary.configure(text="\n".join(lines))
