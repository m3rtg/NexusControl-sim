import customtkinter as ctk

class FKPage(ctk.CTkFrame):
    def __init__(self, parent, app):
        super().__init__(parent, fg_color="transparent")
        
        self.app = app
        
        title = ctk.CTkLabel(self, text="İleri Kinematik Modülü", font=ctk.CTkFont(size=18, weight="bold"))
        title.pack(pady=(10, 20))
        
        info_label = ctk.CTkLabel(self,
                                  text="Mevcut eklem açılarını kullanarak Uç İşlevci\n(End-Effector) konumunu ve Dönüşüm Matrisini\n(T_0^6) hesaplar.",
                                  text_color="#AAAAAA", justify="left")
        info_label.pack(pady=(0, 20))
        
        self.btn_calc_fk = ctk.CTkButton(self, text="D-H Tablosunu ve Matrisi Hesapla", height=42,
                                         corner_radius=8, command=self.app.trigger_fk_calculation)
        self.btn_calc_fk.pack(pady=10, fill="x")
