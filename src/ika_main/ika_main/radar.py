import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class RadarTest(Node):
    def __init__(self):
        super().__init__('radar_test_node')
        self.subscription = self.create_subscription(LaserScan, '/scan_temiz', self.scan_callback, 10)
        self.sayac = 0
        self.get_logger().info('RADAR TESTİ BAŞLADI - ARAÇ HAREKET ETMEYECEK!')

    def scan_callback(self, msg):
        self.sayac += 1
        # Ekrana çok hızlı akmasın diye her 20 mesajda bir yazdırıyoruz
        if self.sayac % 20 == 0: 
            n = len(msg.ranges)
            a_min = math.degrees(msg.angle_min)
            a_max = math.degrees(msg.angle_max)
            
            # Formül: (Max Açı - Min Açı) / Açı Artış Miktarı
            beklenen_n = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
            
            print("================= SENSÖR RÖNTGENİ =================")
            print(f"1. Lidar Açı Aralığı: {a_min:.1f} derece ile {a_max:.1f} derece arası")
            print(f"2. Gelen Toplam Veri: {n} adet")
            print(f"3. Beklenen Veri: {beklenen_n} adet (Eğer bu ikisi farklıysa veri üst üste binmiştir)")
            print("---")
            print("KRİTİK İNDEKSLERİN MESAFESİ:")
            print(f"İndeks [0]       -> Mesafe: {msg.ranges[0]:.2f} metre")
            print(f"İndeks [{n//4}]     -> Mesafe: {msg.ranges[n//4]:.2f} metre")
            print(f"İndeks [{n//2}]     -> Mesafe: {msg.ranges[n//2]:.2f} metre")
            print(f"İndeks [{(n*3)//4}]    -> Mesafe: {msg.ranges[(n*3)//4]:.2f} metre")
            print("===================================================\n")

def main():
    rclpy.init()
    node = RadarTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()