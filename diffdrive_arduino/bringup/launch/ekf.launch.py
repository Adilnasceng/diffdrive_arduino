import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share = get_package_share_directory('diffdrive_arduino')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(package_share, 'config', 'ekf.yaml'),
                {"use_sim_time": False}  # gerçek robot için False olmalı
            ]
        )
    ])
