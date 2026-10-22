import tkinter as tk
import customtkinter as ctk
from ui import theme
from ui.i18n import t

def _blend(c1, c2, f):
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * f):02x}" for x, y in zip(a, b))

class RouteTimeline(ctk.CTkFrame):
    """Rota şeridi: Şimdi -> ara noktalar -> Hedef. Bir durağa tıklamak oraya kadar olan adımları oynatır;
    altındaki çubuklar segmentlerin enerjisini gösterir (koyu = az, açık = çok)."""
    def __init__(self, parent, on_stop, on_play, on_blocked):
        super().__init__(parent, fg_color=theme.BG2, corner_radius=theme.RADIUS, border_width=1, border_color=theme.BORDER)
        self.on_stop, self.on_play, self.on_blocked = on_stop, on_play, on_blocked
        self.labels, self.safety, self.energy = [], [], []
        self.position = None
        self._hit = []

        head = ctk.CTkFrame(self, fg_color="transparent")
        head.pack(fill="x", padx=14, pady=(10, 4))
        head.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(head, text=f"— {t('route.section')}", font=theme.font(11), text_color=theme.MUTED, height=16,
                     anchor="w").grid(row=0, column=0, sticky="w")
        self.summary = ctk.CTkLabel(head, text=t("route.empty"), font=theme.font(12, "bold"), text_color=theme.FG,
                                    anchor="w", height=18, wraplength=300, justify="left")
        self.summary.grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 0))
        btns = ctk.CTkFrame(head, fg_color="transparent")
        btns.grid(row=0, column=1, sticky="e")
        self.btn_fast = theme.button(btns, t("route.fast"), lambda: self.on_play("fast"), kind="primary", width=96, height=28,
                                     corner_radius=999, state="disabled")
        self.btn_fast.pack(side="left", padx=(0, 6))
        self.btn_eco = theme.button(btns, t("route.eco"), lambda: self.on_play("eco"), kind="ghost", width=90, height=28,
                                    corner_radius=999, state="disabled")
        self.btn_eco.pack(side="left")

        self.canvas = tk.Canvas(self, height=64, bg=theme.BG2, highlightthickness=0)
        self.canvas.pack(fill="x", padx=14, pady=(0, 10))
        self.canvas.bind("<Configure>", lambda e: self.redraw())
        self.canvas.bind("<Button-1>", self._click)
        self.canvas.bind("<Motion>", self._hover)

    def clear(self):
        self.labels, self.safety, self.energy, self.position = [], [], [], None
        self.summary.configure(text=t("route.empty"))
        self.set_play_enabled(False)
        self.redraw()

    def set_route(self, labels, safety, energy, summary, playable, time_fast, time_eco):
        self.labels, self.safety, self.energy = labels, safety, energy
        self.summary.configure(text=summary)
        self.btn_fast.configure(text=t("route.fast_time", s=f"{time_fast:.2f}"))
        self.btn_eco.configure(text=t("route.eco_time", s=f"{time_eco:.2f}"))
        self.set_play_enabled(playable)
        self.position = None
        self.redraw()

    def set_play_enabled(self, enabled):
        self.restore_play_states(("normal", "normal") if enabled else ("disabled", "disabled"))

    def play_states(self):
        return self.btn_fast.cget("state"), self.btn_eco.cget("state")

    def restore_play_states(self, states):
        # Kapalı Hızlı butonu altın kalırsa açık sanılıyor; kapalıyken nötr görünür
        fast_on = states[0] == "normal"
        self.btn_fast.configure(state=states[0], fg_color=theme.GOLD if fast_on else theme.BG3,
                                border_width=0 if fast_on else 1, border_color=theme.BORDER2)
        self.btn_eco.configure(state=states[1])

    def set_position(self, index):
        """index: robotun durduğu ara nokta (0 tabanlı) ya da None (rotanın başında/dışında)."""
        if index != self.position:
            self.position = index
            self.redraw()

    def _nodes(self):
        w = self.canvas.winfo_width()
        n = len(self.labels) + 1
        if n < 2: return []
        left, right = 18, max(19, w - 18)
        return [left + (right - left) * k / (n - 1) for k in range(n)]

    def redraw(self):
        c = self.canvas
        c.delete("all")
        self._hit = []
        xs = self._nodes()
        w = c.winfo_width()
        if not xs:
            c.create_line(18, 14, max(19, w - 18), 14, fill=theme.BORDER2, width=2, dash=(3, 4))
            c.create_text(w / 2, 40, text=t("route.hint"), fill=theme.MUTED, font=theme.font(10))
            return
        y = 14
        current = 0 if self.position is None else self.position + 1
        c.create_line(xs[0], y, xs[-1], y, fill=theme.BORDER2, width=2)
        if current > 0:
            c.create_line(xs[0], y, xs[current], y, fill=theme.GOLD, width=2)
        peak = max(self.energy) if self.energy else 0
        for k, e in enumerate(self.energy):
            if k + 1 >= len(xs): break
            f = 0.25 + 0.75 * (e / peak) if peak > 0 else 0.3
            c.create_rectangle(xs[k] + 6, 58, xs[k + 1] - 6, 62, fill=_blend(theme.BG2, theme.STEEL, f), outline="")
        # "route.arc:2" gibi anahtarlar sayı taşır
        names = [t("route.now")] + [t(l.split(":")[0], n=l.split(":")[1]) if ":" in l else t(l) for l in self.labels]
        for k, x in enumerate(xs):
            safe = True if k == 0 else self.safety[k - 1][0]
            last = k == len(xs) - 1
            r = 6
            if not safe:
                c.create_oval(x - r, y - r, x + r, y + r, fill=theme.ERR_SUBTLE, outline=theme.ERR, width=2)
                c.create_text(x, y, text="×", fill=theme.ERR, font=theme.font(10, "bold"))
            elif k < current:
                c.create_oval(x - r, y - r, x + r, y + r, fill=theme.GOLD, outline=theme.GOLD)
            elif k == current:
                c.create_oval(x - r - 4, y - r - 4, x + r + 4, y + r + 4, fill=theme.GOLD_SUBTLE, outline="")
                c.create_oval(x - r, y - r, x + r, y + r, fill=theme.BG, outline=theme.GOLD, width=2)
            elif last:
                c.create_rectangle(x - r, y - r, x + r, y + r, fill=theme.BG, outline=theme.STEEL, width=2)
            else:
                c.create_oval(x - r, y - r, x + r, y + r, fill=theme.BG, outline=theme.BORDER2, width=2)
            color = theme.FG if k == current else (theme.ERR if not safe else theme.MUTED)
            c.create_text(x, y + 22, text=names[k], fill=color, font=theme.font(9, "bold" if k == current else "normal"))
            if k > 0: self._hit.append((x, k - 1, safe))

    def _stop_at(self, x):
        for nx, idx, safe in self._hit:
            if abs(nx - x) <= 14: return idx, safe
        return None, None

    def _hover(self, event):
        idx, _ = self._stop_at(event.x)
        self.canvas.configure(cursor="hand2" if idx is not None else "")

    def _click(self, event):
        idx, safe = self._stop_at(event.x)
        if idx is None: return
        if safe: self.on_stop(idx)
        else: self.on_blocked(idx, self.safety[idx][1])
