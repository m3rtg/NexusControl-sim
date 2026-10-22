import math
import tkinter as tk
import customtkinter as ctk
from ui import theme

class JointGauge(tk.Canvas):
    """Eksen göstergesi: aralık içinde konum, sıfır çizgisi, değer. Tıkla/sürükle ile değer verilir,
    çift tıkla tam değer yazılır."""
    def __init__(self, parent, var, low, high, linear, on_drag_end, on_exact, height=26):
        super().__init__(parent, height=height, bg=theme.BG2, highlightthickness=0, cursor="sb_h_double_arrow")
        self.var, self.low, self.high, self.linear = var, low, high, linear
        self.on_drag_end, self.on_exact = on_drag_end, on_exact
        self.locked = False
        self._drawn = None
        self.bind("<Configure>", lambda e: self.redraw(force=True))
        self.bind("<Button-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)
        self.bind("<ButtonRelease-1>", lambda e: (not self.locked) and self.on_drag_end())
        self.bind("<Double-Button-1>", lambda e: (not self.locked) and self.on_exact())

    def text(self, value):
        return f"{value * 1000:.1f} mm" if self.linear else f"{math.degrees(value):.1f}°"

    def _x(self, value, w):
        return 1 + (value - self.low) / (self.high - self.low) * (w - 2)

    def _drag(self, event):
        if self.locked: return
        w = max(2, self.winfo_width())
        frac = min(1.0, max(0.0, (event.x - 1) / (w - 2)))
        self.var.set(self.low + frac * (self.high - self.low))

    def set_locked(self, locked):
        self.locked = locked
        self.redraw(force=True)

    def redraw(self, force=False):
        value = self.var.get()
        w, h = self.winfo_width(), self.winfo_height()
        state = (round(value, 5), w, h, self.locked)
        if not force and state == self._drawn: return
        self._drawn = state
        self.delete("all")
        if w < 10: return
        edge = theme.BORDER2 if not self.locked else theme.BORDER
        self.create_rectangle(0, 0, w - 1, h - 1, fill=theme.BG, outline=edge)
        x = self._x(min(self.high, max(self.low, value)), w)
        if self.low < 0 < self.high:
            x0 = self._x(0.0, w)
            self.create_line(x0, 2, x0, h - 2, fill=theme.BORDER2, dash=(2, 2))
        accent = theme.MUTED if self.locked else theme.GOLD
        self.create_rectangle(1, 1, x, h - 2, fill=theme.GOLD_SUBTLE if not self.locked else theme.BG3, outline="")
        self.create_line(x, 1, x, h - 1, fill=accent, width=2)
        label = self.text(value)
        tx = 8
        tid = self.create_text(tx, h / 2, text=label, anchor="w", fill=theme.FG if not self.locked else theme.MUTED,
                               font=(theme.mono_family(), 10))
        x1, y1, x2, y2 = self.bbox(tid)
        bg = self.create_rectangle(x1 - 3, y1, x2 + 3, y2, fill=theme.BG, outline="")
        self.tag_lower(bg, tid)

class JointRow(ctk.CTkFrame):
    """'J1  [−] [====|   32.5°] [+]'. −/+ basılı tutulunca seçili hızda sürer."""
    def __init__(self, parent, index, name, var, low, high, linear, on_jog, on_change_done, on_exact):
        super().__init__(parent, fg_color="transparent")
        self.grid_columnconfigure(2, weight=1)
        lbl = ctk.CTkLabel(self, text=f"J{index + 1}", width=24, font=theme.font(11, "bold"), text_color=theme.MUTED)
        lbl.grid(row=0, column=0, padx=(0, 4))
        self.minus = theme.button(self, "−", width=26, height=26, font=theme.font(14))
        self.minus.grid(row=0, column=1)
        self.gauge = JointGauge(self, var, low, high, linear, on_change_done, on_exact)
        self.gauge.grid(row=0, column=2, sticky="ew", padx=4)
        self.plus = theme.button(self, "+", width=26, height=26, font=theme.font(14))
        self.plus.grid(row=0, column=3)
        for btn, direction in ((self.minus, -1), (self.plus, 1)):
            btn.bind("<ButtonPress-1>", lambda e, d=direction: on_jog(index, d), add="+")
            btn.bind("<ButtonRelease-1>", lambda e: on_jog(index, 0), add="+")
        self.name = name

    def set_locked(self, locked):
        state = "disabled" if locked else "normal"
        self.minus.configure(state=state)
        self.plus.configure(state=state)
        self.gauge.set_locked(locked)
