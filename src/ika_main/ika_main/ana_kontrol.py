#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class IkaAnaKontrol(Node):
    def __init__(self):
        super().__init__("ika_ana_kontrol_node")
        
        # 1. GERÇEK MOTORLAR: Sadece bu sınıf doğrudan motorlara emir verebilir
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # 2. OTONOM DİNLEYİCİ: Otonom sürüş kodunun ürettiği komutları dinler
        self.otonom_sub = self.create_subscription(Twist, "/cmd_vel_otonom", self.otonom_callback, 10)
        
        # 3. DURUM MAKİNESİ (State Machine): Otonom sürüşe şu an izin var mı?
        self.otonom_izin_verildi = True 
        
        self.get_logger().info("Beyin (Ana Kontrol) Devrede. Tüm yetki bende!")

    def otonom_callback(self, msg):
        """
        Otonom sürüş (PID) düğümünden gelen tavsiye Twist komutunu alır.
        Eğer beyin (bu sınıf) izin veriyorsa, gerçek motorlara iletir.
        """
        if self.otonom_izin_verildi:
            # İzin varsa otonomdan gelen hızı doğrudan motorlara gönder
            self.cmd_publisher.publish(msg)
        else:
            # İzin yoksa robotu olduğu yere çivile (Güvenlik Modu)
            dur_msg = Twist()
            dur_msg.linear.x = 0.0
            dur_msg.angular.z = 0.0
            self.cmd_publisher.publish(dur_msg)
            self.get_logger().warning("Otonom komut reddedildi, araç durduruluyor!")

def main(args=None):
    rclpy.init(args=args)
    node = IkaAnaKontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()