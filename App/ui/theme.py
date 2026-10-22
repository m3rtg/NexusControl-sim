"""Renk paleti, yazı tipleri ve ölçüler. Görünümün tamamı buradan değişir.
Palet mertgorgulu.com'un koyu temasından: sıcak siyah zemin, kırık beyaz metin, altın vurgu, çelik mavisi ikincil."""
import tkinter.font as tkfont
import customtkinter as ctk

BG = "#0e0e0d"          # pencere zemini
BG2 = "#171714"         # kartlar
BG3 = "#1e1e1a"         # kart içi kontroller, seçili öğe
BORDER = "#252521"
BORDER2 = "#323230"
FG = "#e8e8e1"
FG2 = "#9a9a90"
MUTED = "#8a8a82"
GOLD = "#d4a843"
GOLD_DARK = "#a87d22"
GOLD_HOVER = "#e0b955"
GOLD_SUBTLE = "#2a2619"  # %10 altın, BG2 üstünde
GOLD_EDGE = "#6b5a2c"    # %40 altın kenar
STEEL = "#9eb4cc"
STEEL_DARK = "#7a96b0"
OK = "#4ade80"
WARN = "#fbbf24"
ERR = "#f87171"
ERR_SUBTLE = "#2a1414"
ESTOP_RED = "#c81e1e"
ESTOP_RED_LIGHT = "#ff5c5c"
ESTOP_RING = "#e7b416"
ESTOP_RING_DARK = "#8a6a0c"
OBSTACLE = "#c2410c"

RADIUS = 14         # kart köşesi
RADIUS_SM = 8       # buton, giriş kutusu
PAD = 12            # kartlar arası boşluk

_SANS = ["Inter", "Segoe UI", "Helvetica Neue", "Helvetica", "Arial"]
_MONO = ["JetBrains Mono", "Cascadia Mono", "Consolas", "DejaVu Sans Mono", "Menlo", "Courier New"]
_families = {}
_fonts = {}

def _family(candidates):
    key = candidates[0]
    if key not in _families:
        available = set(tkfont.families())
        _families[key] = next((f for f in candidates if f in available), candidates[-1])
    return _families[key]

def font(size=12, weight="normal", mono=False):
    """CTkFont önbellekli; Tk kökü oluşturulduktan sonra çağrılmalı."""
    key = (size, weight, mono)
    if key not in _fonts:
        _fonts[key] = ctk.CTkFont(family=_family(_MONO if mono else _SANS), size=size, weight=weight)
    return _fonts[key]

def mono_family():
    return _family(_MONO)

def card(parent, **kw):
    return ctk.CTkFrame(parent, fg_color=BG2, corner_radius=RADIUS, border_width=1, border_color=BORDER, **kw)

def section(parent, label, title=None):
    """'— Başlık' bölüm etiketi; isteğe bağlı büyük alt başlıkla."""
    frame = ctk.CTkFrame(parent, fg_color="transparent")
    ctk.CTkLabel(frame, text=f"— {label}", font=font(11), text_color=MUTED, anchor="w", height=16).pack(anchor="w")
    if title:
        ctk.CTkLabel(frame, text=title, font=font(14, "bold"), text_color=FG, anchor="w", height=20).pack(anchor="w")
    return frame

def button(parent, text, command=None, kind="secondary", **kw):
    """kind: primary (altın), secondary (koyu), ghost (kenarlıklı), hot (aktif durum), danger."""
    styles = {
        "primary": dict(fg_color=GOLD, hover_color=GOLD_HOVER, text_color="#1a1405", border_width=0),
        "secondary": dict(fg_color=BG3, hover_color=BORDER2, text_color=FG, border_width=1, border_color=BORDER2),
        "ghost": dict(fg_color="transparent", hover_color=BG3, text_color=FG2, border_width=1, border_color=BORDER2),
        "hot": dict(fg_color=GOLD_SUBTLE, hover_color="#3a3320", text_color=GOLD, border_width=1, border_color=GOLD_EDGE),
        "danger": dict(fg_color=ESTOP_RED, hover_color="#e02424", text_color="#ffffff", border_width=0),
    }
    opts = dict(corner_radius=RADIUS_SM, height=32, font=font(12, "bold" if kind == "primary" else "normal"),
                text_color_disabled=MUTED)
    opts.update(styles[kind])
    opts.update(kw)
    return ctk.CTkButton(parent, text=text, command=command, **opts)

def set_button_kind(btn, kind):
    styles = {
        "secondary": dict(fg_color=BG3, hover_color=BORDER2, text_color=FG, border_color=BORDER2),
        "hot": dict(fg_color=GOLD_SUBTLE, hover_color="#3a3320", text_color=GOLD, border_color=GOLD_EDGE),
    }
    btn.configure(**styles[kind])

def entry(parent, **kw):
    opts = dict(fg_color=BG, border_color=BORDER2, border_width=1, text_color=FG, corner_radius=RADIUS_SM,
                height=32, font=font(12), placeholder_text_color=MUTED)
    opts.update(kw)
    return ctk.CTkEntry(parent, **opts)

def segmented(parent, values, command=None, **kw):
    opts = dict(fg_color=BG, selected_color=BG3, selected_hover_color=BORDER2, unselected_color=BG,
                unselected_hover_color=BG3, text_color=FG2, font=font(11), corner_radius=999, height=26)
    opts.update(kw)
    seg = ctk.CTkSegmentedButton(parent, values=values, command=command, **opts)
    return seg

def option_menu(parent, values, command=None, **kw):
    opts = dict(fg_color=BG3, button_color=BG3, button_hover_color=BORDER2, text_color=FG, font=font(11),
                dropdown_fg_color=BG2, dropdown_hover_color=BG3, dropdown_text_color=FG, dropdown_font=font(11),
                corner_radius=RADIUS_SM, height=28)
    opts.update(kw)
    return ctk.CTkOptionMenu(parent, values=values, command=command, **opts)

def apply_global():
    ctk.set_appearance_mode("dark")
