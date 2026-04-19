import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'ika_main'
    # Nav2 parametre dosyanın olduğu paket adı 'ika' ise burası 'ika' kalsın
    config_pkg_name = 'ika_main'
    
    # 1. Yolları Tanımla
    pkg_path = get_package_share_directory(pkg_name)
    config_path = get_package_share_directory(config_pkg_name)
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'ika.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Senin hazırladığın özel Nav2 parametre dosyasının yolu
    nav2_params_path = os.path.join(config_path, 'config', 'nav2_params.yaml')

    # 2. Robot State Publisher (Robotun eklemlerini yayınlar)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 3. Gazebo'yu başlat (Parkur dünyası ile)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(pkg_path, 'worlds', 'parkur.world')}.items()
    )

    # 4. Robotu Gazebo'ya indir (Spawn)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ika_robot',
                                   '-x', '2.721940',
                                   '-y', '-0.540341',
                                   '-z', '0.1',
                                   '-Y', '3.141305'],
                        output='screen')

    # 5. NAV2 Navigasyon Sistemini Başlat (Senin özel ayarlarınla)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )

    # 6. Tüm düğümleri (nodes) ROS2'ye teslim et
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        nav2_launch, # Navigasyon sistemi burada devreye giriyor
    ])