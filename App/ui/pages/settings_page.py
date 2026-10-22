import customtkinter as ctk
from ui import theme
from ui.i18n import t, available_languages, language

QUALITY = {"standard": "settings.quality_standard", "hd": "settings.quality_hd", "fhd": "settings.quality_fhd"}

class SettingsPage(ctk.CTkFrame):
    """Dil, görüntü kalitesi, kamera kullanımı."""
    def __init__(self, parent, app):
        super().__init__(parent, fg_color="transparent")
        self.app = app
        theme.section(self, t("settings.section"), t("settings.title")).pack(fill="x", pady=(0, 12))

        ctk.CTkLabel(self, text=t("settings.language"), font=theme.font(11), text_color=theme.FG2, anchor="w").pack(fill="x")
        langs = [l.upper() for l in available_languages()]
        self.lang = theme.segmented(self, langs, command=lambda v: app.set_language(v.lower()))
        self.lang.set(language().upper())
        self.lang.pack(fill="x", pady=(4, 2))
        ctk.CTkLabel(self, text=t("settings.language_hint"), font=theme.font(10), text_color=theme.MUTED, anchor="w",
                     justify="left", wraplength=290).pack(fill="x", pady=(0, 14))

        ctk.CTkLabel(self, text=t("settings.quality"), font=theme.font(11), text_color=theme.FG2, anchor="w").pack(fill="x")
        self._quality_by_label = {t(v): k for k, v in QUALITY.items()}
        self.quality = theme.segmented(self, list(self._quality_by_label), command=lambda v: app.set_render_quality(self._quality_by_label[v]))
        self.quality.set(t(QUALITY["standard"]))
        self.quality.pack(fill="x", pady=(4, 2))
        ctk.CTkLabel(self, text=t("settings.quality_hint"), font=theme.font(10), text_color=theme.MUTED, anchor="w",
                     justify="left", wraplength=290).pack(fill="x", pady=(0, 14))

        ctk.CTkLabel(self, text=t("settings.camera"), font=theme.font(11), text_color=theme.FG2, anchor="w").pack(fill="x")
        ctk.CTkLabel(self, text=t("settings.camera_help"), font=theme.font(10), text_color=theme.MUTED, anchor="w",
                     justify="left", wraplength=290).pack(fill="x", pady=(4, 6))
        theme.button(self, t("scene.camera"), app.reset_camera, kind="ghost").pack(fill="x")

    def set_quality(self, preset):
        self.quality.set(t(QUALITY[preset]))
