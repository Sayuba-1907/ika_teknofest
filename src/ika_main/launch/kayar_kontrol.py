import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class EngelKontrol(Node):
    def __init__(self):
        super().__init__('engel_kontrol_merkezi')
        
        # SDF dosyasındaki namespace ile aynı olmak zorunda
        self.pub = self.create_publisher(Twist, '/engel/cmd_vel', 10)
        
        # --- AYARLAR ---
        self.hiz = 0.2            # Hızımız: 0.2 m/s (20 cm/s)
        
        # DİKKAT: Kutunun karşı bariyere ulaşması için gereken süre.
        # Mesafe / Hız (0.2) formülüyle bulunur. 
        # Örneğin iki bariyer arası 3 metre ise: 3.0 / 0.2 = 15 saniye sürer.
        self.git_gel_suresi = 18.0 # <--- BURAYI BARİYER MESAFENE GÖRE DEĞİŞTİR
        
        self.bekleme_suresi = 1.0 # Bariyere değince 1 saniye bekle
        
        self.durum = "GIDIŞ"
        self.timer = self.create_timer(0.1, self.dongu)
        self.zaman_sayaci = 0.0

        self.get_logger().info('Hareketli Engel Başladı! Hedef süre: {} saniye'.format(self.git_gel_suresi))

    def dongu(self):
        msg = Twist()
        self.zaman_sayaci += 0.1

        if self.durum == "GIDIŞ":
            msg.linear.y = self.hiz # Y ekseninde kay
            if self.zaman_sayaci >= self.git_gel_suresi:
                self.durum = "BEKLE_1"
                self.zaman_sayaci = 0.0

        elif self.durum == "BEKLE_1":
            msg.linear.y = 0.0
            if self.zaman_sayaci >= self.bekleme_suresi:
                self.durum = "DÖNÜŞ"
                self.zaman_sayaci = 0.0

        elif self.durum == "DÖNÜŞ":
            msg.linear.y = -self.hiz # Ters yöne kay
            if self.zaman_sayaci >= self.git_gel_suresi:
                self.durum = "BEKLE_2"
                self.zaman_sayaci = 0.0

        elif self.durum == "BEKLE_2":
            msg.linear.y = 0.0
            if self.zaman_sayaci >= self.bekleme_suresi:
                self.durum = "GIDIŞ"
                self.zaman_sayaci = 0.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(EngelKontrol())
    rclpy.shutdown()

if __name__ == '__main__':
    main()