import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from abc import ABC, abstractmethod

# --- 1. STRATEGY PATTERN: Sürüş Modları ---

class SurusStratejisi(ABC):
    @abstractmethod
    def hesapla(self, hata, mesafe_verileri):
        pass

class DuzlukStratejisi(SurusStratejisi):
    def __init__(self, pid_kontrolcu):
        self.pid = pid_kontrolcu

    def hesapla(self, hata, veriler):
        kp, ki, kd = (0.8, 0.0, 0.8) if abs(hata) > 1.2 else (0.5, 0.005, 1.0)
        pid_val = self.pid.hesapla(hata, kp, ki, kd)
        ang_z = max(min(pid_val, 1.0), -1.0)
        lin_x = max(0.45 - abs(ang_z) * 0.20, 0.20)
        return lin_x, ang_z

class VirajStratejisi(SurusStratejisi):
    def __init__(self, pid_kontrolcu):
        self.pid = pid_kontrolcu

    def hesapla(self, hata, veriler):
        # Viraj / Yan eğim için yumuşatılmış katsayılar
        pid_val = self.pid.hesapla(hata, 0.7, 0.002, 1.3)
        ang_z = max(min(pid_val, 1.6), -1.6)
        lin_x = max(0.32 - abs(ang_z) * 0.15, 0.20)
        return lin_x, ang_z

class DarKoridorStratejisi(SurusStratejisi):
    def __init__(self, pid_kontrolcu):
        self.pid = pid_kontrolcu

    def hesapla(self, hata, veriler):
        hata_abs = abs(hata)
        kp, ki, kd = (1.0, 0.0, 1.0) if hata_abs > 0.8 else (0.6, 0.01, 1.2)
        pid_val = self.pid.hesapla(hata, kp, ki, kd)
        ang_z = max(min(pid_val, 1.4), -1.4)
        lin_x = max(0.28 - abs(ang_z) * 0.15, 0.15)
        return lin_x, ang_z

class EngelStratejisi(SurusStratejisi):
    def __init__(self):
        self.engel_sayaci = 0
        self.engel_donus_yonu = 0.0
        self.ENGEL_ESIGI = 20

    def hesapla(self, hata, veriler):
        sol_min, sag_min = veriler['sol_min'], veriler['sag_min']
        
        if self.engel_sayaci == 0:
            self.engel_donus_yonu = 1.0 if sol_min > sag_min else -1.0
            
        self.engel_sayaci += 1
        lin_x = 0.20
        ang_z = self.engel_donus_yonu * 1.5
        
        if self.engel_sayaci >= self.ENGEL_ESIGI:
            self.engel_sayaci = 0
            self.engel_donus_yonu = 0.0
            
        return lin_x, ang_z

# --- 2. PID KONTROL SINIFI ---

class PIDKontrolcu:
    def __init__(self, limit=0.5):
        self.onceki_hata = 0.0
        self.integral = 0.0
        self.INTEGRAL_LIMIT = limit

    def sifirla(self):
        self.onceki_hata = 0.0
        self.integral = 0.0

    def hesapla(self, hata, kp, ki, kd):
        if hata * self.onceki_hata <= 0:
            self.integral = 0.0
            
        if ki > 0.0:
            self.integral += hata
            self.integral = max(min(self.integral, self.INTEGRAL_LIMIT), -self.INTEGRAL_LIMIT)
        else:
            self.integral = 0.0
            
        p = kp * hata
        i = ki * self.integral
        d = kd * (hata - self.onceki_hata)
        self.onceki_hata = hata
        return p + i + d

# --- 3. DÜĞÜM (NODE) SINIFI ---

class OtonomSurus(Node):
    def __init__(self):
        super().__init__('otonom_surus_node')
        self.subscription = self.create_subscription(LaserScan, '/scan_temiz', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('OTONOM SURUS AKTIF (Dual-Mode Eğim Ayarlı)')
        
        self.pid = PIDKontrolcu(limit=0.4)
        self.stratejiler = {
            'DUZLUK': DuzlukStratejisi(self.pid),
            'VIRAJ_YAKLAS': VirajStratejisi(self.pid),
            'DAR_KORIDOR': DarKoridorStratejisi(self.pid),
            'ENGEL': EngelStratejisi()
        }
        
        self.pid_iptal_kilidi = False
        self.sabit_donus_yonu = 0.0
        self.son_durum = ''
        
        self.son_on_min = 6.0

    def durum_belirle(self, on_min, on_genis_min, sol_min, sag_min, aci_verileri):
        kanal = sol_min + sag_min

        if on_min < 0.45:
            self.pid_iptal_kilidi = False
            return 'ACIL_DUR'

        if self.pid_iptal_kilidi:
            if on_min > 2.0:
                self.pid_iptal_kilidi = False
                self.pid.sifirla()
                self.get_logger().info('>>> YOL ACILDI! <<<')
                return 'DUZLUK'
            else:
                return 'PID_IPTAL'

        # DUBA TESPİTİ: Yanların simetrik olarak darlık oluşturması durumu
        # Eğer yandaki nesneler çok dar bir açıdaysa duba algıla.
        duba_durumu = (aci_verileri['sol_duba_yakin'] and aci_verileri['sag_duba_yakin'])
        
        mesafe_degisimi = self.son_on_min - on_min
        if (on_min < 0.6 and duba_durumu) or (mesafe_degisimi > 0.4 and on_min < 0.9):
            self.pid_iptal_kilidi = True
            self.sabit_donus_yonu = 1.0 if sol_min > sag_min else -1.0
            self.get_logger().info(f'>>> DUBA ALGILANDI! KİLİT YÖN: {self.sabit_donus_yonu:+.0f} <<<')
            return 'PID_IPTAL'

        self.son_on_min = on_min

        # VİRAJ / EĞİM TESPİTİ
        # Mesafe farkı (hata) çoksa ancak yanlar açık/normal ise virajdır.
        if on_min < 1.1:                                   return 'ENGEL'
        if kanal < 2.0:                                    return 'DAR_KORIDOR'
        if on_genis_min < 1.6 or on_min < 2.0:             return 'VIRAJ_YAKLAS'
        
        return 'DUZLUK'

    def scan_callback(self, msg):
        on_m, on_g, sol_m, sag_m = [], [], [], []
        sol_duba_yakin, sag_duba_yakin = False, False
        MAX = 6.0

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.1:
                r = MAX
            deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            
            if   -20 <= deg <=  20: on_m.append(r)
            if   -40 <= deg <=  40: on_g.append(r)
            
            if 25 <= deg <= 85: 
                sol_m.append(r)
                if r < 0.8: sol_duba_yakin = True # Dubaya yakınlık kontrolü
                
            elif -85 <= deg <= -25: 
                sag_m.append(r)
                if r < 0.8: sag_duba_yakin = True # Dubaya yakınlık kontrolü

        on_min       = min(on_m)  if on_m  else MAX
        on_genis_min = min(on_g)  if on_g  else MAX
        sol_min      = min(sol_m) if sol_m else MAX
        sag_min      = min(sag_m) if sag_m else MAX

        hata = max(min(sol_min - sag_min, 2.0), -2.0)
        
        aci_verileri = {
            'sol_duba_yakin': sol_duba_yakin,
            'sag_duba_yakin': sag_duba_yakin
        }
        
        durum = self.durum_belirle(on_min, on_genis_min, sol_min, sag_min, aci_verileri)
        cmd = Twist()

        if durum == 'ACIL_DUR':
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 if sol_min > sag_min else -1.5
            self.pid.sifirla()

        elif durum == 'PID_IPTAL':
            cmd.linear.x = 0.18
            cmd.angular.z = self.sabit_donus_yonu * 2.2

        else:
            strateji = self.stratejiler.get(durum, self.stratejiler['DUZLUK'])
            veri_paketi = {'sol_min': sol_min, 'sag_min': sag_min}
            cmd.linear.x, cmd.angular.z = strateji.hesapla(hata, veri_paketi)

        self.publisher.publish(cmd)

        if durum != self.son_durum:
            self.get_logger().info(f'[{durum}] Hat:{hata:+.2f} | Linear:{cmd.linear.x:.2f} | Angular:{cmd.angular.z:.2f}')
            self.son_durum = durum

def main():
    rclpy.init()
    node = OtonomSurus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()