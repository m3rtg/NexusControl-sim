import math
import time
import customtkinter as ctk
import pybullet as p
from ui import theme
from i18n import t
from ui.widgets.joint_row import JointRow

# Jog hızı: döner eksen derece/sn, lineer eksen eksen aralığının saniyedeki oranı
JOG_SPEEDS = {"%1": (0.9, 0.01), "%10": (9.0, 0.1), "%100": (90.0, 1.0)}

class JogPage(ctk.CTkFrame):
    """Manuel kontrol: eksen başına −/+ jog tuşları, sürüklenebilir gösterge, hız seçici; FK raporu."""
    def __init__(self, parent, app, sim_engine):
        super().__init__(parent, fg_color="transparent")
        self.app, self.sim_engine = app, sim_engine
        self.slider_vars, self.rows, self.limits = [], [], []
        self._jog = None
        self._jog_after = None
        self.locked = False

        theme.section(self, t("jog.section"), t("jog.title")).pack(fill="x", pady=(0, 6))
        ctk.CTkLabel(self, text=t("jog.hint"), font=theme.font(10), text_color=theme.MUTED, anchor="w",
                     justify="left", wraplength=290).pack(fill="x", pady=(0, 8))
        self.speed = theme.segmented(self, list(JOG_SPEEDS))
        self.speed.set("%10")
        self.speed.pack(fill="x", pady=(0, 10))

        self.rows_frame = ctk.CTkScrollableFrame(self, fg_color="transparent", scrollbar_button_color=theme.BORDER2,
                                                 scrollbar_button_hover_color=theme.MUTED)
        self.rows_frame.pack(fill="x")

        nav = ctk.CTkFrame(self, fg_color="transparent")
        nav.pack(fill="x", pady=(10, 0))
        nav.grid_columnconfigure(2, weight=1)
        theme.button(nav, "↶", app.undo_position, width=36).grid(row=0, column=0, padx=(0, 6))
        theme.button(nav, "↷", app.redo_position, width=36).grid(row=0, column=1, padx=(0, 6))
        theme.button(nav, t("jog.home"), app.go_to_base, kind="ghost").grid(row=0, column=2, sticky="ew")
        theme.button(self, t("jog.fk_report"), app.trigger_fk_calculation, kind="ghost").pack(fill="x", pady=(8, 0))

    def rebuild_joint_sliders(self, revolute_joints, robotId, make_scroll_smooth):
        for w in self.rows_frame.winfo_children(): w.destroy()
        self.slider_vars, self.rows, self.limits = [], [], []
        if not revolute_joints or robotId is None:
            ctk.CTkLabel(self.rows_frame, text=t("jog.no_joints"), font=theme.font(12), text_color=theme.MUTED).pack(pady=20)
            return
        with self.sim_engine.bullet_lock:
            for i, joint_idx in enumerate(revolute_joints):
                info = p.getJointInfo(robotId, joint_idx)
                low, high = float(info[8]), float(info[9])
                if low >= high or (abs(low) < 1e-5 and abs(high) < 1e-5):
                    low, high = -math.pi, math.pi
                linear = info[2] == p.JOINT_PRISMATIC
                var = ctk.DoubleVar(value=0.0)
                row = JointRow(self.rows_frame, i, info[1].decode("utf-8"), var, low, high, linear,
                               self._jog_event, self.app.save_position, lambda idx=i: self._exact_value(idx))
                row.pack(fill="x", pady=3)
                self.slider_vars.append(var)
                self.rows.append(row)
                self.limits.append((low, high, linear))
        # Satırlar kadar yükseklik; çok eksenli modellerde kaydırma
        self.rows_frame.configure(height=min(len(self.rows) * 34 + 4, 420))
        make_scroll_smooth(self.rows_frame, step_px=40)
        self.locked = None

    # --- jog: basılı tutuldukça seçili hızda ---
    def _jog_event(self, index, direction):
        if direction == 0:
            if self._jog_after: self.after_cancel(self._jog_after)
            if self._jog: self.app.save_position()
            self._jog = self._jog_after = None
            return
        if self.locked: return
        self._jog = (index, direction, None)
        self._jog_tick()

    def _jog_tick(self):
        if not self._jog or self.locked: return
        index, direction, last = self._jog
        now = time.perf_counter()
        dt = 0.05 if last is None else min(now - last, 0.1)
        low, high, linear = self.limits[index]
        deg_rate, range_rate = JOG_SPEEDS[self.speed.get()]
        rate = range_rate * (high - low) if linear else math.radians(deg_rate)
        var = self.slider_vars[index]
        var.set(min(high, max(low, var.get() + direction * rate * dt)))
        self._jog = (index, direction, now)
        self._jog_after = self.after(30, self._jog_tick)

    def _exact_value(self, index):
        low, high, linear = self.limits[index]
        unit = "mm" if linear else "°"
        lo, hi = (low * 1000, high * 1000) if linear else (math.degrees(low), math.degrees(high))
        dialog = ctk.CTkInputDialog(title=t("jog.exact_title"), text=t("jog.exact_prompt", j=index + 1, lo=f"{lo:.0f}", hi=f"{hi:.0f}", unit=unit),
                                    fg_color=theme.BG2, button_fg_color=theme.GOLD, button_hover_color=theme.GOLD_HOVER,
                                    button_text_color="#1a1405", entry_fg_color=theme.BG, entry_border_color=theme.BORDER2)
        raw = dialog.get_input()
        try:
            value = float(str(raw).strip().replace(",", "."))
        except (TypeError, ValueError):
            return
        value = value / 1000 if linear else math.radians(value)
        self.slider_vars[index].set(min(high, max(low, value)))
        self.app.save_position()

    def redraw_gauges(self):
        for row in self.rows: row.gauge.redraw()

    def set_locked(self, locked):
        if locked == self.locked: return
        self.locked = locked
        for row in self.rows: row.set_locked(locked)
        if locked: self._jog_event(0, 0)
