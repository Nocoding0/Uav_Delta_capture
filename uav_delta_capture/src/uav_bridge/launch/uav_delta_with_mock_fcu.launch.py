from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    system_launch = os.path.join(
        get_package_share_directory('uav_bridge'),
        'launch',
        'uav_delta_system.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'use_mock_fcu': 'true',
                'start_mavros': 'false',
                'start_fcu_guard': 'true',
                'delta_target_topic': 'target_point_safe',
            }.items(),
        ),
    ])
