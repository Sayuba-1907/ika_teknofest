import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ika_main'
    pkg_path = get_package_share_directory(pkg_name)

    # 1. Gazebo ve Robot (Simülasyon)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'sim.launch.py'))
    )

    # 2. Ana Kontrol (Beyin/Patron)
    ana_kontrol = Node(
        package=pkg_name,
        executable='ana_kontrol',
        output='screen'
    )

    # 3. Sensör İşleme (Gözler)
    sensor = Node(
        package=pkg_name,
        executable='sensor_isleme',
        output='screen'
    )

    # 4. Otonom Sürüş (Şoför - 3 saniye rötarla başlar)
    otonom = TimerAction(
        period=3.0,
        actions=[
            Node(
                package=pkg_name,
                executable='otonom_surus',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        ana_kontrol,
        sensor,
        otonom
    ])