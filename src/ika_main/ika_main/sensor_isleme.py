import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import json

class EasySensor(Node):
    def __init__(self):
        super().__init__('sensor_manager_node')
        
        # Abonelikler (Gözler ve Kulaklar)
        self.subscription = self.create_subscription(PointCloud2, '/points', self.pointcloud_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Yayıncılar (Raporlar)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan_temiz', 10)
        self.rapor_yayinci = self.create_publisher(String, '/cevre_durumu', 10)
        
        self.get_logger().info('KATMAN 1: Dinamik Rampa Filtresi ve İstihbarat Analizi Aktif!')

        # Eğim verileri için değişkenler
        self.anlik_pitch = 0.0
        self.anlik_roll = 0.0

    # ------------------------------------------------------------------ IMU
    def imu_callback(self, msg):
        """ IMU verisinden rampa (pitch) ve yan eğim (roll) açılarını çıkarır. """
        q = msg.orientation
        t0 = 2.0 * (q.w * q.x + q.y * q.z)
        t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.anlik_roll = math.degrees(math.atan2(t0, t1))
        
        t2 = max(min(2.0 * (q.w * q.y - q.z * q.x), 1.0), -1.0)
        self.anlik_pitch = math.degrees(math.asin(t2))

    # ------------------------------------------------------------------ DUBA TESPİTİ
    def duba_sayisi_bul(self, ranges, angle_min, angle_increment):
        """ YENİ VE KUSURSUZ DUBA DEDEKTÖRÜ (Çıkıntı Mantığı) """
        duba_sayisi = 0
        
        def guvenli_mesafe(index):
            if 0 <= index < len(ranges):
                r = ranges[index]
                if math.isinf(r) or math.isnan(r) or r > 11.0:
                    return 6.0
                return r
            return 6.0

        for i in range(15, len(ranges) - 15):
            aci = math.degrees(angle_min + (i * angle_increment))
            if -60 <= aci <= 60:
                merkez_mesafe = guvenli_mesafe(i)
                if merkez_mesafe < 3.5:
                    sol_mesafe = guvenli_mesafe(i + 15)
                    sag_mesafe = guvenli_mesafe(i - 15)
                    if (sol_mesafe - merkez_mesafe > 0.5) and (sag_mesafe - merkez_mesafe > 0.5):
                        duba_sayisi += 1
        return duba_sayisi

    # ------------------------------------------------------------------ 3D -> 2D ve ANALİZ
    def pointcloud_callback(self, cloud_msg):
        # 1. HATA VERMEYEN GÜVENLİ OKUMA
        points_list = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
            
        points = np.array(points_list)
        if len(points) == 0: return

        # ---------------------------------------------------------
        # 2. DİNAMİK EĞİM FİLTRESİ (RAMPA CANAVARI GÜNCELLEMESİ)
        # ---------------------------------------------------------
        # Araba eğime (3 dereceden fazla) girdiğinde LiDAR ışınları asfalta çarpar.
        # Bu asfalt çarpmalarını silmek için taban (Z) filtresini yukarı çekiyoruz.
        if abs(self.anlik_pitch) > 3.0 or abs(self.anlik_roll) > 3.0:
            z_alt_sinir = -0.10 # Rampa çıkarken yeri görme
        else:
            z_alt_sinir = -0.45 # Düz yolda normal duba tespiti
            
        dist_xy = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask = (points[:, 2] > z_alt_sinir) & (points[:, 2] < 0.75) & (dist_xy > 0.25)
        engel_noktalari = points[mask]

        # 3. 2D'ye çevir
        scan_msg = LaserScan()
        scan_msg.header = cloud_msg.header
        scan_msg.header.frame_id = "lidar_link"
        scan_msg.angle_min, scan_msg.angle_max = -np.pi, np.pi
        scan_msg.angle_increment = (2 * np.pi) / 360
        scan_msg.range_min, scan_msg.range_max = 0.3, 12.0
        
        ranges = np.full(360, 12.0)
        for p in engel_noktalari:
            angle = np.arctan2(p[1], p[0])
            index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)
            if 0 <= index < 360:
                dist = np.sqrt(p[0]**2 + p[1]**2)
                if dist < ranges[index]: ranges[index] = dist

        scan_msg.ranges = ranges.tolist()
        self.scan_publisher.publish(scan_msg)
        
        # ---------------------------------------------------------
        # 4. ÇEVREYİ ANALİZ ET VE KARARGAHA RAPORLA
        # ---------------------------------------------------------
        on_mesafeler = []
        sol_mesafeler = []
        sag_mesafeler = []
        max_gorus = 6.0

        for i, r in enumerate(scan_msg.ranges):
            mesafe = r if r < 11.0 else max_gorus
            aci_derece = math.degrees(scan_msg.angle_min + (i * scan_msg.angle_increment))

            if -15 <= aci_derece <= 15:     on_mesafeler.append(mesafe)
            elif 45 <= aci_derece <= 85:    sol_mesafeler.append(mesafe)
            elif -85 <= aci_derece <= -45:  sag_mesafeler.append(mesafe)

        on_min = min(on_mesafeler) if on_mesafeler else max_gorus
        sol_ort = sum(sol_mesafeler) / len(sol_mesafeler) if sol_mesafeler else max_gorus
        sag_ort = sum(sag_mesafeler) / len(sag_mesafeler) if sag_mesafeler else max_gorus
        kanal_genisligi = sol_ort + sag_ort

        duba_sayisi = self.duba_sayisi_bul(scan_msg.ranges, scan_msg.angle_min, scan_msg.angle_increment)

        # Raporu JSON Formatına Çevir
        rapor_dict = {
            "on_mesafe": round(on_min, 2),
            "sol_duvar": round(sol_ort, 2),
            "sag_duvar": round(sag_ort, 2),
            "kanal_genisligi": round(kanal_genisligi, 2),
            "duba_sayisi": duba_sayisi,
            "pitch_egimi": round(abs(self.anlik_pitch), 1),
            "roll_egimi": round(abs(self.anlik_roll), 1)
        }

        msg_out = String()
        msg_out.data = json.dumps(rapor_dict)
        self.rapor_yayinci.publish(msg_out)

def main():
    rclpy.init()
    node = EasySensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()