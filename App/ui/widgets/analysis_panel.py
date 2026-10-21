import customtkinter as ctk

class AnalysisPanel(ctk.CTkFrame):
    def __init__(self, parent):
        super().__init__(parent, fg_color="#1E1E1E", corner_radius=12, border_width=1, border_color="#2A2A2A")
        
        self.analysis_title = ctk.CTkLabel(self, text="SİSTEM & ANALİZ",
                                           font=ctk.CTkFont(family="Segoe UI", size=12, weight="bold"))
        self.analysis_title.pack(pady=(10, 4))

        # Canlı Telemetri Kartı (Sabit boy, kaydırma çubuğu yok, anlık ve hafif güncelleme)
        self.telemetry_box = ctk.CTkTextbox(self, height=135,
                                            font=ctk.CTkFont(family="Consolas", size=11),
                                            fg_color="#0A0A0A", text_color="#00FFCC", corner_radius=8,
                                            border_width=1, border_color="#1F1F1F", activate_scrollbars=False)
        self.telemetry_box.pack(padx=12, pady=(0, 6), fill="x")

        # Statik Matematik ve Güvenlik Raporu (Sadece IK/FK hesaplandığında güncellenir, akıcı kaydırma)
        self.analysis_box = ctk.CTkTextbox(self, font=ctk.CTkFont(family="Consolas", size=11),
                                           fg_color="#0A0A0A", text_color="#CCCCCC", corner_radius=8,
                                           border_width=1, border_color="#1F1F1F")
        self.analysis_box.pack(padx=12, pady=(0, 12), fill="both", expand=True)
        
        self._last_tel_text = ""
        self._estop_state = None
        self._last_report_text = ""
        
    def update_telemetry(self, text):
        if text and text != self._last_tel_text:
            self._last_tel_text = text
            self.telemetry_box.configure(state="normal")
            self.telemetry_box.delete("0.0", "end")
            self.telemetry_box.insert("end", text)
            self.telemetry_box.configure(state="disabled")

    def update_report(self, text):
        if text and text != self._last_report_text:
            self._last_report_text = text
            self.analysis_box.configure(state="normal")
            self.analysis_box.delete("0.0", "end")
            self.analysis_box.insert("end", text)
            self.analysis_box.configure(state="disabled")

    def set_estop_state(self, is_active):
        # Her UI tick'inde çağrılır; renk değişimi tüm paneli yeniden çizdiği için sadece durum değişince uygula
        if is_active == self._estop_state: return
        self._estop_state = is_active
        if is_active:
            self.configure(fg_color="#3A0000", border_color="#FF0000")
        else:
            self.configure(fg_color="#1E1E1E", border_color="#2A2A2A")
