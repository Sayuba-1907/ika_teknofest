from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. GÖZLER
        Node(
            package='ika_main',
            executable='sensor_isleme',
            name='sensor_manager_node',
            output='screen'
        ),
        # 2. REFLEKS (PID)
        Node(
            package='ika_main',
            executable='otonom_surucu', 
            name='drive_controller_node',
            output='screen'
        ),
        # 3. BEYİN
        Node(
            package='ika_main',
            executable='ana_kontrol',
            name='ika_ana_kontrol_node',
            output='screen'
        )
    ])