from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paket yolları
    rplidar_ros_pkg = os.path.join(get_package_share_directory('rplidar_ros'))
    diffdrive_arduino_pkg = os.path.join(get_package_share_directory('diffdrive_arduino'))

    # Serial port argümanı
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )

    # Harita dosyası argümanı (AMCL için zorunlu)
    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file (e.g. /home/user/maps/my_map.yaml)'
    )

    # 1. diffbot.launch.py çalışacak
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diffdrive_arduino_pkg, 'launch', 'diffbot.launch.py')
        )
    )

    # 2. 8 saniye sonra lidar başlasın (diffbot hazır olsun)
    rplidar_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rplidar_ros_pkg, 'launch', 'rplidar_a2m12_launch.py')
                ),
                launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()
            )
        ]
    )

    # 3. 14 saniye sonra localization başlasın (map_server + AMCL)
    localization_launch = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(diffdrive_arduino_pkg, 'launch', 'localization.launch.py')
                ),
                launch_arguments={'map': LaunchConfiguration('map')}.items()
            )
        ]
    )

    # 4. 20 saniye sonra navigation başlasın (AMCL hazır olsun)
    nav2_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(diffdrive_arduino_pkg, 'launch', 'navigation.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        map_arg,
        diffbot_launch,
        rplidar_launch,
        localization_launch,
        nav2_launch
    ])
