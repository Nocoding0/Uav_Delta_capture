from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # 启动 UWB AOA 驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uwb_driver'),
                    'launch',
                    'uwb_aoa_driver.launch.py'
                ])
            )
        ),

        # 启动姿态发布
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('fcu_bridge'),
                    'launch',
                    'attitude_publisher.launch.py'
                ])
            )
        ),

        # 启动视觉变换
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('vision_bridge'),
                    'launch',
                    'vision_transform.launch.py'
                ])
            )
        ),

        # 启动 UWB 任务规划
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uwb_navigation'),
                    'launch',
                    'uwb_mission_planner.launch.py'
                ])
            )
        ),
    ])
