#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class IkaAnaKontrol(Node):
    def __init__(self):
        super().__init__("ika_hareket_node")
        
        # HAREKET MERKEZİ: Tüm hız komutları buradan çıkar
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.get_logger().info("Hareket Modülü Hazır. Komut bekleniyor...")

    def hareket_et(self, lineer_hiz, acisal_hiz):
        """
        Robotun hareketini sağlayan ana fonksiyon.
        İleride otonom sürüş de burayı çağıracak.
        """
        msg = Twist()
        msg.linear.x = float(lineer_hiz)
        msg.angular.z = float(acisal_hiz)
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IkaAnaKontrol()
    
    # Şimdilik test amaçlı robotu 2 saniye boyunca yavaşça döndürelim
    node.hareket_et(0.1, 0.5)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()