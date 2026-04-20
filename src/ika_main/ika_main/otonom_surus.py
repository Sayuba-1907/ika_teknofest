import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

DURUM_DUZLUK       = 'DUZLUK'
DURUM_DAR_KORIDOR  = 'DAR_KORIDOR'
DURUM_VIRAJ_YAKLAS = 'VIRAJ_YAKLAS'
DURUM_PID_IPTAL    = 'PID_IPTAL_KOR_DONUS' # Senin istediğin özel durum!
DURUM_ENGEL        = 'ENGEL'
DURUM_ACIL_DUR     = 'ACIL_DUR'

class OtonomSurus(Node):
    def __init__(self):
        super().__init__('otonom_surus_node')
        self.subscription = self.create_subscription(LaserScan, '/scan_temiz', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_otonom', 10)
        
        self.get_logger().info('OTONOM SURUS AKTIF (Bariyerlerde PID Iptal Mantigi)')

        # PID Değişkenleri
        self.onceki_hata    = 0.0
        self.integral       = 0.0
        self.INTEGRAL_LIMIT = 1.0 

        # --- SENİN İSTEDİĞİN "DEVRE DIŞI BIRAKMA" KİLİTLERİ ---
        self.pid_iptal_kilidi = False
        self.sabit_donus_yonu = 0.0

        # Engel
        self.engel_donus_yonu = 0.0
        self.engel_sayaci     = 0
        self.ENGEL_ESIGI      = 10

        self.son_durum = ''

    def pid_sifirla(self):
        self.integral    = 0.0
        self.onceki_hata = 0.0

    def adaptif_katsayilar(self, durum, hata_abs):
        if durum == DURUM_ENGEL:
            return 1.2, 0.0, 0.5
        elif durum == DURUM_DAR_KORIDOR:
            return (1.2, 0.0, 1.2) if hata_abs > 0.8 else (0.8, 0.01, 1.5)
        elif durum == DURUM_VIRAJ_YAKLAS:
            return 1.1, 0.01, 1.3
        else:  # DUZLUK
            if hata_abs > 1.2:   return 1.0, 0.0,   1.0
            elif hata_abs > 0.4: return 0.7, 0.01,  1.4
            else:                return 0.5, 0.02,  1.8

    def pid_hesapla(self, hata, kp, ki, kd):
        # Yalpalamayı önleyen anti-windup
        if hata * self.onceki_hata <= 0:
            self.integral = 0.0
            
        if ki > 0.0:
            self.integral += hata
            self.integral  = max(min(self.integral, self.INTEGRAL_LIMIT), -self.INTEGRAL_LIMIT)
        else:
            self.integral = 0.0
            
        p = kp * hata
        i = ki * self.integral
        d = kd * (hata - self.onceki_hata)
        self.onceki_hata = hata
        return p + i + d

    def durum_belirle(self, on_min, on_genis_min, sol_min, sag_min):
        kanal = sol_min + sag_min

        # 1. Önce Çarpışma Önleyici
        if on_min < 0.45:
            self.pid_iptal_kilidi = False
            return DURUM_ACIL_DUR
        
        # 2. SENİN FİKRİN: PİD İPTAL KİLİDİ (Bariyerden Çıkış Beklentisi)
        # Eğer robot bariyerlere girip PID'yi devre dışı bıraktıysa...
        if self.pid_iptal_kilidi:
            # Önünde 2.5 metrelik DÜMDÜZ bir yol görene kadar bu moddan ASLA çıkma!
            if on_min > 2.5: 
                self.pid_iptal_kilidi = False
                self.pid_sifirla() # Eski hataları çöpe at, tertemiz başla
                self.get_logger().info('>>> YOL ACILDI! PID TEKRAR DEVREDE <<<')
                return DURUM_DUZLUK
            else:
                return DURUM_PID_IPTAL
                
        # 3. Bariyere Girme Tespiti (PID'yi Kapatma Anı)
        if on_min < 1.0:                         
            self.pid_iptal_kilidi = True
            # Hangi taraf kolaysa (genişse) oraya dönmeye karar ver ve kilitlen
            self.sabit_donus_yonu = 1.0 if sol_min > sag_min else -1.0
            self.get_logger().info(f'>>> BARIYERE GIRILDI! PID KAPANISI YON: {self.sabit_donus_yonu:+.0f} <<<')
            return DURUM_PID_IPTAL

        # 4. Diğer Standart PID Durumları
        if on_min < 1.3:                           return DURUM_ENGEL
        if kanal < 2.2:                          return DURUM_DAR_KORIDOR
        if on_genis_min < 2.0 or on_min < 2.5:   return DURUM_VIRAJ_YAKLAS
        return DURUM_DUZLUK

    def scan_callback(self, msg):
        on_m, on_g, sol_m, sag_m = [], [], [], []
        MAX = 6.0

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.1:
                r = MAX
            deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            
            # Kör noktaları kapattığımız mükemmel sektör ayarı
            if  -20 <= deg <=  20: on_m.append(r)
            if  -40 <= deg <=  40: on_g.append(r)
            if   25 <= deg <=  85: sol_m.append(r) 
            elif -85 <= deg <= -25: sag_m.append(r)

        on_min       = min(on_m)  if on_m  else MAX
        on_genis_min = min(on_g)  if on_g  else MAX
        sol_min      = min(sol_m) if sol_m else MAX
        sag_min      = min(sag_m) if sag_m else MAX

        hata  = sol_min - sag_min
        durum = self.durum_belirle(on_min, on_genis_min, sol_min, sag_min)
        cmd   = Twist()

        # ════════════════════════════════════════════════════
        if durum == DURUM_ACIL_DUR:
            cmd.linear.x  = 0.0
            cmd.angular.z = 1.8 if sol_min > sag_min else -1.8
            self.pid_sifirla()

        # SENİN KURALIN: PID TAMAMEN DEVRE DIŞI
        elif durum == DURUM_PID_IPTAL:
            # Sadece yavaşça ileri git ve direksiyonu kilitlediğimiz yöne tam tur çevir
            cmd.linear.x  = 0.20
            cmd.angular.z = self.sabit_donus_yonu * 2.5

        elif durum == DURUM_ENGEL:
            if self.engel_sayaci == 0:
                self.engel_donus_yonu = 1.0 if sol_min > sag_min else -1.0
            self.engel_sayaci += 1
            cmd.linear.x  = 0.25
            cmd.angular.z = self.engel_donus_yonu * 1.8
            if self.engel_sayaci >= self.ENGEL_ESIGI:
                self.engel_sayaci     = 0
                self.engel_donus_yonu = 0.0

        elif durum == DURUM_DAR_KORIDOR:
            kp, ki, kd = self.adaptif_katsayilar(durum, abs(hata))
            pid = self.pid_hesapla(hata, kp, ki, kd)
            cmd.angular.z = max(min(pid, 1.8), -1.8)
            cmd.linear.x  = max(0.35 - abs(cmd.angular.z) * 0.15, 0.15)

        elif durum == DURUM_VIRAJ_YAKLAS:
            kp, ki, kd = self.adaptif_katsayilar(durum, abs(hata))
            pid = self.pid_hesapla(hata, kp, ki, kd)
            cmd.angular.z = max(min(pid, 2.0), -2.0)
            cmd.linear.x  = max(0.40 - abs(cmd.angular.z) * 0.15, 0.20)

        else:  # DUZLUK
            self.engel_sayaci     = 0
            self.engel_donus_yonu = 0.0
            kp, ki, kd = self.adaptif_katsayilar(durum, abs(hata))
            pid = self.pid_hesapla(hata, kp, ki, kd)
            cmd.angular.z = max(min(pid, 1.2), -1.2)
            cmd.linear.x  = max(0.60 - abs(cmd.angular.z) * 0.25, 0.25)

        self.publisher.publish(cmd)

        if durum != self.son_durum:
            self.get_logger().info(
                f'[{durum}] On:{on_min:.2f} Sol:{sol_min:.2f} Sag:{sag_min:.2f} '
                f'H:{hata:+.2f} -> lin:{cmd.linear.x:.2f} ang:{cmd.angular.z:+.2f}'
            )
            self.son_durum = durum

def main():
    rclpy.init()
    node = OtonomSurus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()