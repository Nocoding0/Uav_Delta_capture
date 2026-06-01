from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('uav_bridge') + '/launch/uav_delta_system.launch.py'
        ),
        launch_arguments={
            'start_uwb_nav': 'true',
            'use_mock_fcu': LaunchConfiguration('use_mock_fcu', default='false'),
            'start_mavros': LaunchConfiguration('start_mavros', default='false'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_mock_fcu', default_value='false'),
        DeclareLaunchArgument('start_mavros', default_value='false'),
        system_launch,
    ])
