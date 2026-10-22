import tkinter as tk
from ui import theme
from i18n import t

class ReachMap(tk.Canvas):
    """Üstten görünüş: erişim halkası, kaide, engel, uç işlevcinin şu anki yeri ve hedef. Tıklanan nokta X/Y olur.
    Eksenler: +X sağa, +Y yukarı (robot tabanı merkezde)."""
    def __init__(self, parent, on_pick, size=290):
        super().__init__(parent, width=size, height=size, bg=theme.BG2, highlightthickness=0, cursor="crosshair")
        self.size, self.on_pick = size, on_pick
        self.data = None
        self._drawn = None
        self.bind("<Button-1>", self._click)

    def _scale(self):
        reach = max(1.0, self.data["reach"])
        return (self.size / 2 - 14) / (reach * 1.08)

    def _xy(self, x_cm, y_cm):
        s = self._scale()
        return self.size / 2 + x_cm * s, self.size / 2 - y_cm * s

    def update_data(self, data):
        key = tuple((k, tuple(v) if isinstance(v, (list, tuple)) else v) for k, v in sorted(data.items()))
        if key == self._drawn: return
        self._drawn = key
        self.data = data
        self.redraw()

    def redraw(self):
        self.delete("all")
        if not self.data: return
        d, s, c = self.data, self._scale(), self.size / 2
        self.create_rectangle(0, 0, self.size - 1, self.size - 1, fill=theme.BG, outline=theme.BORDER)
        r = d["reach"] * s
        # erişim alanı ve halkaları
        self.create_oval(c - r, c - r, c + r, c + r, fill="#15140f", outline=theme.STEEL_DARK, dash=(4, 3))
        for frac in (0.5,):
            rr = r * frac
            self.create_oval(c - rr, c - rr, c + rr, c + rr, outline=theme.BORDER2, dash=(2, 4))
        self.create_line(c, 8, c, self.size - 8, fill=theme.BORDER)
        self.create_line(8, c, self.size - 8, c, fill=theme.BORDER)
        self.create_text(self.size - 10, c - 8, text="+X", anchor="e", fill=theme.MUTED, font=theme.font(9))
        self.create_text(c + 6, 12, text="+Y", anchor="w", fill=theme.MUTED, font=theme.font(9))
        self.create_text(c + r - 4, c + 10, text=f"{d['reach']:.0f} cm", anchor="e", fill=theme.STEEL_DARK, font=theme.font(9))
        if d.get("stand"):
            h = d["stand"] * s
            self.create_rectangle(c - h, c - h, c + h, c + h, outline=theme.BORDER2, fill="#1b1a16")
        self.create_oval(c - 7, c - 7, c + 7, c + 7, fill=theme.BG3, outline=theme.BORDER2)
        if d.get("obstacle"):
            ox, oy, hx, hy = d["obstacle"]
            x0, y0 = self._xy(ox - hx, oy + hy)
            x1, y1 = self._xy(ox + hx, oy - hy)
            self.create_rectangle(x0, y0, x1, y1, fill=theme.OBSTACLE, outline="")
        if d.get("tcp"):
            x, y = self._xy(*d["tcp"])
            self.create_oval(x - 5, y - 5, x + 5, y + 5, fill=theme.STEEL, outline="")
        if d.get("target"):
            x, y = self._xy(*d["target"])
            col = theme.GOLD if d.get("reachable", True) else theme.ERR
            self.create_oval(x - 9, y - 9, x + 9, y + 9, outline=col, width=2)
            self.create_line(x - 13, y, x + 13, y, fill=col)
            self.create_line(x, y - 13, x, y + 13, fill=col)
        self.create_text(10, self.size - 10, anchor="sw", fill=theme.MUTED, font=theme.font(9), text=t("target.map_legend"))

    def _click(self, event):
        if not self.data: return
        s = self._scale()
        self.on_pick((event.x - self.size / 2) / s, (self.size / 2 - event.y) / s)
