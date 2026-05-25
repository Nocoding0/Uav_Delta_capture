from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    delta_default = os.path.join(
        get_package_share_directory('delta_kinematics'),
        'config',
        'delta_kinematics.yaml'
    )
    perception_default = os.path.join(
        get_package_share_directory('perception_logic'),
        'config',
        'perception_logic.yaml'
    )
    bridge_default = os.path.join(
        get_package_share_directory('uav_bridge'),
        'config',
        'uav_bridge.yaml'
    )
    mock_default = os.path.join(
        get_package_share_directory('uav_bridge'),
        'config',
        'mock_mavros_pose.yaml'
    )
    health_default = os.path.join(
        get_package_share_directory('uav_bridge'),
        'config',
        'fcu_health.yaml'
    )
    failsafe_default = os.path.join(
        get_package_share_directory('uav_bridge'),
        'config',
        'failsafe.yaml'
    )
    mavros_default = os.path.join(
        get_package_share_directory('uav_bridge'),
        'config',
        'mavros_bridge.yaml'
    )

    delta_params = LaunchConfiguration('delta_params')
    perception_params = LaunchConfiguration('perception_params')
    bridge_params = LaunchConfiguration('bridge_params')
    mock_params = LaunchConfiguration('mock_params')
    health_params = LaunchConfiguration('health_params')
    failsafe_params = LaunchConfiguration('failsafe_params')
    mavros_params = LaunchConfiguration('mavros_params')
    use_mock_fcu = LaunchConfiguration('use_mock_fcu')
    start_mavros = LaunchConfiguration('start_mavros')
    start_fcu_guard = LaunchConfiguration('start_fcu_guard')
    delta_target_topic = LaunchConfiguration('delta_target_topic')

    return LaunchDescription([
        DeclareLaunchArgument('delta_params', default_value=delta_default),
        DeclareLaunchArgument('perception_params', default_value=perception_default),
        DeclareLaunchArgument('bridge_params', default_value=bridge_default),
        DeclareLaunchArgument('mock_params', default_value=mock_default),
        DeclareLaunchArgument('health_params', default_value=health_default),
        DeclareLaunchArgument('failsafe_params', default_value=failsafe_default),
        DeclareLaunchArgument('mavros_params', default_value=mavros_default),
        DeclareLaunchArgument('use_mock_fcu', default_value='false'),
        DeclareLaunchArgument('start_mavros', default_value='false'),
        DeclareLaunchArgument('start_fcu_guard', default_value='true'),
        DeclareLaunchArgument('delta_target_topic', default_value='target_point_safe'),

        Node(
            package='delta_kinematics',
            executable='delta_kinematics_node',
            name='delta_kinematics_node',
            output='screen',
            parameters=[delta_params, {'target_topic': delta_target_topic}]
        ),
        Node(
            package='perception_logic',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[perception_params]
        ),
        Node(
            package='uav_bridge',
            executable='uav_bridge_node',
            name='uav_bridge_node',
            output='screen',
            parameters=[bridge_params]
        ),
        Node(
            package='uav_bridge',
            executable='mock_mavros_pose_node',
            name='mock_mavros_pose_node',
            output='screen',
            parameters=[mock_params],
            condition=IfCondition(use_mock_fcu)
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_node',
            output='screen',
            parameters=[mavros_params],
            condition=IfCondition(start_mavros)
        ),
        Node(
            package='uav_bridge',
            executable='fcu_link_monitor_node',
            name='fcu_link_monitor_node',
            output='screen',
            parameters=[health_params],
            condition=IfCondition(start_fcu_guard)
        ),
        Node(
            package='uav_bridge',
            executable='flight_state_machine_node',
            name='flight_state_machine_node',
            output='screen',
            parameters=[health_params],
            condition=IfCondition(start_fcu_guard)
        ),
        Node(
            package='uav_bridge',
            executable='failsafe_manager_node',
            name='failsafe_manager_node',
            output='screen',
            parameters=[failsafe_params],
            condition=IfCondition(start_fcu_guard)
        ),
    ])
