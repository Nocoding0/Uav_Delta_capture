from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('uwb_navigation')
    config_file = os.path.join(pkg_share, 'test_mission.yaml')
    script_path = os.path.join(pkg_share, '..', '..', 'lib', 'uwb_navigation',
                               'test_mission_node.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),
        ExecuteProcess(
            cmd=['python3', script_path,
                 '--ros-args', '--params-file', config_file],
            output='screen',
            name='test_mission_node',
        ),
    ])
