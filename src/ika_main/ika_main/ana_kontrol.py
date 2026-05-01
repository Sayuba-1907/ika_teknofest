import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class AnaKontrol(Node):
    def __init__(self):
        super().__init__('ana_kontrol_node')
        
        # 1. Katmandan (Sensörden) gelen istihbaratı dinle
        self.rapor_abone = self.create_subscription(String, '/cevre_durumu', self.rapor_callback, 10)
        
        # 3. Katmana (Şoföre) verilecek emirleri yayınla
        self.mod_yayinci = self.create_publisher(String, '/arac_modu', 10)
        
        self.aktif_mod = "MOD_DUZLUK"
        self.kilit_sayaci = 0  # Kararsızlığı (titremeyi) önleyen Histerezis kilidi
        
        self.get_logger().info('KATMAN 2: Karargah (Ana Kontrol) Aktif! Raporlar bekleniyor...')

    def rapor_callback(self, msg):
        # JSON string'ini tekrar sözlüğe (dictionary) çevir
        try:
            istihbarat = json.loads(msg.data)
        except:
            return # Bozuk veri gelirse es geç
            
        on_mesafe = istihbarat["on_mesafe"]
        duba_sayisi = istihbarat["duba_sayisi"]
        kanal_genisligi = istihbarat["kanal_genisligi"]
        pitch = istihbarat["pitch_egimi"]

        yeni_mod = self.aktif_mod

        # ---------------------------------------------------------
        # BEYNİN KARAR MEKANİZMASI (DURUM MAKİNESİ)
        # ---------------------------------------------------------
        
        # KURAL 1: Acil durum her şeyi ezer (Kilit dinlemez)
        if on_mesafe < 0.6:
            yeni_mod = "MOD_ACIL_DURUM"
            self.kilit_sayaci = 10 # 10 frame boyunca acil durumda kal
            
        # KURAL 2: Eğer bir moda yeni girdiysek ve kilit sürüyorsa, modu DEĞİŞTİRME! (Titremeyi önler)
        elif self.kilit_sayaci > 0:
            self.kilit_sayaci -= 1
            # Kilitliyken karar mekanizmasını atla
            
        # KURAL 3: Normal Karar Ağacı (Kilit sıfırsa yeni karar verilebilir)
        else:
            if duba_sayisi > 0:
                yeni_mod = "MOD_SLALOM"
                # Dubaların arasından geçerken anlık duba göremeyebiliriz.
                # Sistem hemen "düzlüğe çıktım" sanıp duvara girmesin diye kilitliyoruz.
                self.kilit_sayaci = 20 # Yaklaşık 1-2 saniye bu moddan ÇIKMA!
                
            elif pitch > 2.5 or abs(istihbarat["roll_egimi"]) > 2.5:
                yeni_mod = "MOD_DIK_ENGEL"
                self.kilit_sayaci = 30 # Rampa bitene kadar kolay kolay bu moddan çıkma
                
            elif on_mesafe < 2.0:
                yeni_mod = "MOD_U_DONUSU"
                self.kilit_sayaci = 80
                
            else:
                yeni_mod = "MOD_DUZLUK"

        # ---------------------------------------------------------
        # EMRİ YAYINLA
        # ---------------------------------------------------------
        if yeni_mod != self.aktif_mod:
            self.get_logger().info(f'MOD DEĞİŞTİ: {self.aktif_mod} ---> {yeni_mod}')
            self.aktif_mod = yeni_mod

        emir_msg = String()
        emir_msg.data = self.aktif_mod
        self.mod_yayinci.publish(emir_msg)

def main():
    rclpy.init()
    node = AnaKontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()