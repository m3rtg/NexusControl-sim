import customtkinter as ctk
from ui import theme
from ui.i18n import t

class ModelPage(ctk.CTkFrame):
    """Yüklü model bilgisi; yeni model yükleme, açılış modeli yapma, dahili modele dönme."""
    def __init__(self, parent, app):
        super().__init__(parent, fg_color="transparent")
        self.app = app
        head = theme.section(self, t("model.section"))
        head.pack(fill="x")
        self.lbl_name = ctk.CTkLabel(self, text="", font=theme.font(16, "bold"), text_color=theme.FG, anchor="w",
                                     justify="left", wraplength=290)
        self.lbl_name.pack(fill="x", pady=(2, 10))

        info = theme.card(self)
        info.configure(fg_color=theme.BG)
        info.pack(fill="x")
        self.rows = {}
        for key in ("file", "axes", "mass", "reach"):
            row = ctk.CTkFrame(info, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=(8 if key == "file" else 2, 8 if key == "reach" else 2))
            ctk.CTkLabel(row, text=t(f"model.{key}"), font=theme.font(11), text_color=theme.MUTED, height=18).pack(side="left")
            value = ctk.CTkLabel(row, text="", font=theme.font(11), text_color=theme.FG, height=18, anchor="e",
                                 wraplength=190, justify="right")
            value.pack(side="right")
            self.rows[key] = value

        theme.button(self, t("model.load"), app.open_urdf_file_dialog, kind="primary", height=36).pack(fill="x", pady=(14, 6))
        theme.button(self, t("model.set_default"), app.set_current_as_default).pack(fill="x", pady=(0, 6))
        theme.button(self, t("model.builtin"), app.reset_default_robot, kind="ghost").pack(fill="x")
        self.lbl_default = ctk.CTkLabel(self, text="", font=theme.font(10), text_color=theme.MUTED, anchor="w",
                                        justify="left", wraplength=290)
        self.lbl_default.pack(fill="x", pady=(10, 0))
        ctk.CTkLabel(self, text=t("model.hint"), font=theme.font(10), text_color=theme.MUTED, anchor="w",
                     justify="left", wraplength=290).pack(fill="x", pady=(14, 0))

    def update_info(self, name, file_name, axes, mass, mass_source, reach_cm):
        self.lbl_name.configure(text=name)
        self.rows["file"].configure(text=file_name)
        self.rows["axes"].configure(text=str(axes))
        self.rows["mass"].configure(text=f"{mass:.1f} kg · {mass_source}")
        self.rows["reach"].configure(text=f"{reach_cm:.0f} cm")

    def set_default_info(self, text, is_custom):
        self.lbl_default.configure(text=text, text_color=theme.GOLD if is_custom else theme.MUTED)
