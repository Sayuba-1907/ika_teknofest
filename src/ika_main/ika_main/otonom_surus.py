import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class OtonomSurus(Node):
    def __init__(self):
        super().__init__('otonom_surus_node')
        self.subscription = self.create_subscription(LaserScan, '/scan_temiz', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('OTONOM SÜRÜŞ: PID KONTROLÜ VE VIRAJ İYİLEŞTİRMESİ AKTİF!')

        # PID Katsayıları
        self.kp = 0.6   # P: Hızlı tepki
        self.ki = 0.05  # I: Uzun süreli hata düzeltme
        self.kd = 0.8   # D: Oscillation önleme
        
        self.onceki_hata = 0.0
        self.integral_hata = 0.0
        self.dead_band = 0.05  # Küçük hatalar için eşik

    def scan_callback(self, msg):
        on_mesafeler = []
        sol_mesafeler = []
        sag_mesafeler = []
        max_mesafe = 5.0 

        for i, mesafe in enumerate(msg.ranges):
            if math.isinf(mesafe) or math.isnan(mesafe) or mesafe < 0.1:
                mesafe = max_mesafe
                
            aci_radyan = msg.angle_min + (i * msg.angle_increment)
            aci_derece = math.degrees(aci_radyan)
            
            # Açı normalizasyonu: -180 ile +180 arasında tutma
            aci_derece = ((aci_derece + 180) % 360) - 180

            # --- 1. HATA ÇÖZÜMÜ: SIFIR KÖR NOKTA ---
            # Dubalar ince olduğu için ön açıyı genişlettik (-30 ile 30)
            if -30 <= aci_derece <= 30:       
                on_mesafeler.append(mesafe)
            elif 30 < aci_derece <= 180:       
                sol_mesafeler.append(mesafe)
            elif -180 <= aci_derece < -30:     
                sag_mesafeler.append(mesafe)

        on_min = min(on_mesafeler) if on_mesafeler else max_mesafe
        sol_min = min(sol_mesafeler) if sol_mesafeler else max_mesafe
        sag_min = min(sag_mesafeler) if sag_mesafeler else max_mesafe

        cmd = Twist()
        
        # --- 2. HATA ÇÖZÜMÜ: YANLARI SÜRTME KORUMASI ---
        # Araç merkezden yanlara 0.55m taşıyor. 0.65'ten yakınına bariyer/duba girerse kurtar!
        
        if on_min < 1.3: # Önde duba veya duvar var
            cmd.linear.x = 0.15 
            if sol_min > sag_min:
                cmd.angular.z = 1.8   
            else:
                cmd.angular.z = -1.8  
            self.onceki_hata = 0.0 
            
        elif sol_min < 0.65: # SOL YAN bariyere sürtüyor! Hemen sağa kaç!
            cmd.linear.x = 0.2
            cmd.angular.z = -1.5
            self.onceki_hata = 0.0 
            self.get_logger().warning('Sol sürtme tehlikesi! Sağa kaçılıyor.')
            
        elif sag_min < 0.65: # SAĞ YAN bariyere sürtüyor! Hemen sola kaç!
            cmd.linear.x = 0.2
            cmd.angular.z = 1.5
            self.onceki_hata = 0.0 
            self.get_logger().warning('Sağ sürtme tehlikesi! Sola kaçılıyor.')

        else:
            # PID Kontrolü
            baz_hiz = 0.5 
            hata = sol_min - sag_min
            
            # Küçük hatalar için dead-band
            if abs(hata) < self.dead_band:
                hata = 0.0
                self.integral_hata = 0.0
            else:
                # Integrator terimi - uzun dönem hatası düzelt
                self.integral_hata += hata
                self.integral_hata = max(min(self.integral_hata, 2.0), -2.0)  # Anti-windup
            
            # P terimi: Mevcut hataya tepki
            p_terimi = self.kp * hata
            
            # I terimi: Uzun dönem hatası düzeltme
            i_terimi = self.ki * self.integral_hata
            
            # D terimi: Hata değişim hızına tepki (oscillation önleme)
            turev = hata - self.onceki_hata
            d_terimi = self.kd * turev
            
            # PID çıkışı
            angular_z_raw = p_terimi + i_terimi + d_terimi
            cmd.angular.z = max(min(angular_z_raw, 1.5), -1.5)
            
            # Viraj dönüşünde hızı yumuşak şekilde azalt (adaptif hız)
            # Büyük açılarda (viraj) yavaşla, düz gidişte hızlan
            donme_orani = abs(cmd.angular.z) / 1.5  # 0.0 - 1.0
            hiz_faktoru = 1.0 - (donme_orani * 0.4)  # Max %40 azalma
            cmd.linear.x = baz_hiz * hiz_faktoru
            cmd.linear.x = max(cmd.linear.x, 0.2)  # Minimum hız

            self.onceki_hata = hata

        self.publisher.publish(cmd)

def main():
    rclpy.init()
    node = OtonomSurus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()