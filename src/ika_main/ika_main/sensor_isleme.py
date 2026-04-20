import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class EasySensor(Node):
    def __init__(self):
        super().__init__('sensor_manager_node')
        self.subscription = self.create_subscription(PointCloud2, '/points', self.pointcloud_callback, 10)
        self.publisher = self.create_publisher(LaserScan, '/scan_temiz', 10)
        self.get_logger().info('Basit ve ZIRHLI 3D Filtre Aktif!')

    def pointcloud_callback(self, cloud_msg):
        # 1. HATA VERMEYEN GÜVENLİ OKUMA YÖNTEMİ
        points_list = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])
            
        points = np.array(points_list)
        if len(points) == 0: return

        # 2. EĞİM DOSTU FİLTRE
        # Yatay mesafe < 0.6m olan noktalar araç gövdesi veya eğimli zeminin kendisi olabilir
        dist_xy = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        mask = (points[:, 2] > -0.45) & (points[:, 2] < 0.40) & (dist_xy > 1.0)
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
        self.publisher.publish(scan_msg)

def main():
    rclpy.init()
    rclpy.spin(EasySensor())
    rclpy.shutdown()

# BU BLOK EKSİKTİ, EKLENDİ!
if __name__ == '__main__':
    main()