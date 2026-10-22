import tkinter as tk
from ui import theme
from ui.i18n import t

class EStopButton(tk.Canvas):
    """Mantar acil stop: sarı yaka, kırmızı kafa. Basınca E-stop tetikler; aktifken koyu ve 'AKTİF' yazar."""
    def __init__(self, parent, command, size=46, bg=theme.BG):
        super().__init__(parent, width=size, height=size, bg=bg, highlightthickness=0, cursor="hand2")
        self.size, self.command = size, command
        self.active, self.hover = False, False
        self.bind("<Button-1>", lambda e: self.command())
        self.bind("<Enter>", lambda e: self._set_hover(True))
        self.bind("<Leave>", lambda e: self._set_hover(False))
        self._draw()

    def _set_hover(self, value):
        self.hover = value
        self._draw()

    def set_active(self, active):
        if active != self.active:
            self.active = active
            self._draw()

    def _draw(self):
        self.delete("all")
        s = self.size
        self.create_oval(1, 1, s - 1, s - 1, fill=theme.ESTOP_RING_DARK, outline="")
        self.create_oval(3, 3, s - 3, s - 3, fill=theme.ESTOP_RING, outline="")
        head = "#7a1010" if self.active else (theme.ESTOP_RED_LIGHT if self.hover else theme.ESTOP_RED)
        self.create_oval(8, 8, s - 8, s - 8, fill=head, outline="#5c0b0b")
        if not self.active:
            # parlama
            self.create_oval(13, 11, s / 2, s / 2 - 2, fill="#ff7a7a", outline="")
        self.create_text(s / 2, s / 2, text=t("estop.active") if self.active else t("estop.stop"),
                         fill="#ffffff", font=(theme.mono_family(), 6 if self.active else 7, "bold"))
