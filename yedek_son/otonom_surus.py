import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class OtonomSurucu(Node):
    def __init__(self):
        super().__init__('otonom_surucu_node')
        
        # Lidar verilerini dinlemek için abone (subscriber) oluyoruz
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        # Araca hareket komutu göndermek için yayıncı (publisher) oluşturuyoruz
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Hareket komutu değişkeni
        self.cmd = Twist()
        
        self.get_logger().info('Otonom Sürüş Düğümü Başlatıldı. Yolun açık olsun kral!')

    def lidar_callback(self, msg):
        # Lidar verisini bölgelere ayırıyoruz (Örn: 360 derecelik sensör için)
        # Ön bölge: Sensörün tam ortası (0-30 ve 330-360 dereceler arası)
        on_bolge = min(min(msg.ranges[0:30] + msg.ranges[330:360]), 10.0)
        sol_bolge = min(min(msg.ranges[30:90]), 10.0)
        sag_bolge = min(min(msg.ranges[270:330]), 10.0)

        self.get_logger().info(f'Ön Mesafe: {on_bolge:.2f}m')

        # KARAR MEKANİZMASI
        if on_bolge > 1.0: # Önümde 1 metre boyunca engel yoksa
            self.cmd.linear.x = 0.3  # Düz git
            self.cmd.angular.z = 0.0 # Dönme
        else: # Engel varsa (bariyer gördük!)
            self.get_logger().warn('Bariyer Tespit Edildi! Dönüyorum...')
            self.cmd.linear.x = 0.0 # Dur
            
            if sol_bolge > sag_bolge:
                self.cmd.angular.z = 0.5 # Sola dön
            else:
                self.cmd.angular.z = -0.5 # Sağa dön
        
        # Komutu gönder
        self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OtonomSurucu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Düğüm kapatılıyor...')
    finally:
        # Durdurma komutu gönder
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()