import math
import customtkinter as ctk
from ui import theme
from ui.i18n import t
from ui.widgets.reach_map import ReachMap

def _num(text):
    try:
        return float(text.strip().replace(",", ".")) if text.strip() else None
    except ValueError:
        return None

class TargetPage(ctk.CTkFrame):
    """Hedef (ters kinematik): haritaya tıkla ya da X/Y/Z yaz, rotayı hesapla. Rota alttaki şeritte çıkar."""
    def __init__(self, parent, app, kinematics_engine):
        super().__init__(parent, fg_color="transparent")
        self.app, self.kinematics = app, kinematics_engine

        theme.section(self, t("target.section"), t("target.title")).pack(fill="x", pady=(0, 8))
        self.map = ReachMap(self, self._pick)
        self.map.pack(pady=(0, 10))

        self.var_x, self.var_y, self.var_z = ctk.StringVar(), ctk.StringVar(), ctk.StringVar()
        grid = ctk.CTkFrame(self, fg_color="transparent")
        grid.pack(fill="x")
        grid.grid_columnconfigure((0, 1, 2), weight=1, uniform="xyz")
        self.hints = []
        for col, (axis, var) in enumerate((("X", self.var_x), ("Y", self.var_y), ("Z", self.var_z))):
            ctk.CTkLabel(grid, text=f"{axis} (cm)", font=theme.font(10), text_color=theme.MUTED, height=14).grid(
                row=0, column=col, sticky="w", padx=(0 if col == 0 else 4, 0))
            e = theme.entry(grid, textvariable=var, width=60)
            e.grid(row=1, column=col, sticky="ew", padx=(0 if col == 0 else 4, 0))
            hint = ctk.CTkLabel(grid, text="", font=theme.font(9), text_color=theme.MUTED, height=14)
            hint.grid(row=2, column=col, sticky="w", padx=(0 if col == 0 else 4, 0))
            self.hints.append(hint)
            var.trace_add("write", self.update_limits)
        self.entry_x, self.entry_y, self.entry_z = [w for w in grid.winfo_children() if isinstance(w, ctk.CTkEntry)]

        self.lbl_reach = ctk.CTkLabel(self, text=t("target.enter"), font=theme.font(11), text_color=theme.MUTED,
                                      anchor="w", justify="left", wraplength=290)
        self.lbl_reach.pack(fill="x", pady=(6, 10))
        theme.button(self, t("target.calculate"), self.on_calculate_ik, kind="primary", height=36).pack(fill="x")

    def _pick(self, x_cm, y_cm):
        self.var_x.set(f"{x_cm:.0f}")
        self.var_y.set(f"{y_cm:.0f}")
        if not self.var_z.get().strip():
            self.var_z.set(f"{self.kinematics.shoulder_z_cm:.0f}")

    def on_calculate_ik(self):
        self.app.apply_energy_settings()
        self.app.trigger_ik_calculation(self.entry_x.get(), self.entry_y.get(), self.entry_z.get())

    def target(self):
        x, y = _num(self.var_x.get()), _num(self.var_y.get())
        return (x, y) if x is not None and y is not None else None

    def update_limits(self, *args):
        """Erişim ipuçları: X için ±erişim, Y için X'e göre kalan, Z için yatay mesafeye göre aralık."""
        k = self.kinematics
        x, y, z = _num(self.var_x.get()), _num(self.var_y.get()), _num(self.var_z.get())
        s_r, s_z, arm, reach = k.shoulder_r_cm, k.shoulder_z_cm, k.arm_length_cm, k.robot_reach_cm
        stand_top, stand_half = k.stand_top_cm, k.stand_half_extent_cm
        floor = max(0.0, stand_top + (5.0 if stand_half > 0 else 0.0))
        out = t("target.out_of_reach")

        def hint(i, text, bad=False):
            self.hints[i].configure(text=text, text_color=theme.ERR if bad else theme.MUTED)

        hint(0, f"±{reach:.0f}")
        if x is None: hint(1, f"±{reach:.0f}")
        elif abs(x) > reach: hint(1, out, True)
        else: hint(1, f"±{math.sqrt(max(0.0, reach ** 2 - x ** 2)):.0f}")
        if x is None and y is None:
            hint(2, f"{floor:.0f}…{s_z + arm:.0f}")
        else:
            d = max(0.0, math.hypot(x or 0.0, y or 0.0) - s_r)
            if d > arm: hint(2, out, True)
            else:
                dz = math.sqrt(max(0.0, arm ** 2 - d ** 2))
                lo, hi = max(floor, s_z - dz), s_z + dz
                hint(2, f"{lo:.0f}…{hi:.0f}" if lo <= hi else out, lo > hi)

        if x is not None and y is not None and z is not None:
            d = max(0.0, math.hypot(x, y) - s_r)
            dist = math.hypot(d, z - s_z)
            below = (stand_half > 0 and abs(x) <= stand_half + 5 and abs(y) <= stand_half + 5 and z < floor) or z < 0
            if below:
                self.lbl_reach.configure(text=t("target.below_floor", z=f"{z:.0f}", floor=f"{floor:.0f}"), text_color=theme.ERR)
            elif dist <= arm:
                self.lbl_reach.configure(text=t("target.reachable", d=f"{dist:.1f}", limit=f"{arm:.1f}"), text_color=theme.OK)
            else:
                self.lbl_reach.configure(text=t("target.too_far", d=f"{dist:.1f}", over=f"{dist - arm:.1f}"), text_color=theme.WARN)
        elif x is not None or y is not None or z is not None:
            self.lbl_reach.configure(text=t("target.complete"), text_color=theme.MUTED)
        else:
            self.lbl_reach.configure(text=t("target.enter"), text_color=theme.MUTED)

    def refresh_map(self, tcp_cm, obstacle_cm):
        k = self.kinematics
        tgt = self.target()
        reachable = True
        if tgt is not None:
            reachable = max(0.0, math.hypot(*tgt) - k.shoulder_r_cm) <= k.arm_length_cm
        self.map.update_data({
            "reach": round(k.robot_reach_cm, 1), "stand": round(k.stand_half_extent_cm, 1) if k.stand_half_extent_cm > 0 else 0,
            "obstacle": obstacle_cm, "tcp": tuple(round(v, 1) for v in tcp_cm[:2]) if tcp_cm else None,
            "target": tgt, "reachable": reachable,
        })
