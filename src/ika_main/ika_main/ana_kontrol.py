#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class IkaAnaKontrol(Node):
    def __init__(self):
        super().__init__('ika_ana_kontrol_node')

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.otonom_sub = self.create_subscription(
            Twist, '/cmd_vel_otonom', self.otonom_callback, 10)

        # Duba bölümü için manuel sürüş komutu
        # teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard
        #         --ros-args -r cmd_vel:=/cmd_vel_manuel
        self.manuel_sub = self.create_subscription(
            Twist, '/cmd_vel_manuel', self.manuel_callback, 10)

        # Mod değiştirme:
        #   ros2 topic pub /mod std_msgs/msg/String "data: 'manuel'" --once
        #   ros2 topic pub /mod std_msgs/msg/String "data: 'otonom'" --once
        self.mod_sub = self.create_subscription(
            String, '/mod', self.mod_callback, 10)

        self.mod = 'otonom'
        self.get_logger().info('Ana Kontrol Devrede — Mod: OTONOM')

    def mod_callback(self, msg):
        yeni_mod = msg.data.strip().lower()
        if yeni_mod in ('otonom', 'manuel'):
            self.mod = yeni_mod
            self.get_logger().info(f'MOD DEĞİŞTİ → {self.mod.upper()}')
        else:
            self.get_logger().warning(f'Bilinmeyen mod: {msg.data}  (otonom/manuel yazın)')

    def otonom_callback(self, msg):
        if self.mod == 'otonom':
            self.cmd_publisher.publish(msg)

    def manuel_callback(self, msg):
        if self.mod == 'manuel':
            self.cmd_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IkaAnaKontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
