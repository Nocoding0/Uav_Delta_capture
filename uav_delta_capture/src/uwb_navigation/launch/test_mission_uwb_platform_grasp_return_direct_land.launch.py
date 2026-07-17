#!/usr/bin/env python3
"""Isolated UWB fixed-platform grasp, return-home, and direct LAND test."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('uwb_navigation')
    allow_descend = LaunchConfiguration('platform_allow_descend')

    test_mission_node = Node(
        package='uwb_navigation',
        executable='test_mission_uwb_platform_node.py',
        name='test_mission_uwb_platform_node',
        parameters=[
            os.path.join(
                share_dir,
                'test_mission_uwb_platform_grasp_return_direct_land.yaml',
            ),
            {
                'enable_drop_stage': False,
                'platform_allow_descend': ParameterValue(
                    allow_descend,
                    value_type=bool,
                ),
            },
        ],
        output='screen',
    )

    set_body_frame = ExecuteProcess(
        cmd=[
            'ros2',
            'param',
            'set',
            '/mavros/setpoint_velocity',
            'mav_frame',
            'BODY_NED',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'platform_allow_descend',
            default_value='false',
            description=(
                'Allow descent to the 0.15m platform grasp clearance. '
                'Keep false for the first prop-on platform verification run.'
            ),
        ),
        Node(
            package='fcu_bridge',
            executable='fcu_state_node',
            name='fcu_state_node',
        ),
        Node(
            package='fcu_bridge',
            executable='fcu_link_monitor_node',
            name='fcu_link_monitor_node',
        ),
        Node(
            package='fcu_bridge',
            executable='flight_commander_node',
            name='flight_commander_node',
            parameters=[{
                'skip_ekf_check': True,
                'vel_timeout_sec': 0.15,
                'auto_vel_modes': 'GUIDED',
                'takeoff_target_clearance_m': 0.2,
            }],
        ),
        Node(
            package='fcu_bridge',
            executable='flight_state_machine_node',
            name='flight_state_machine_node',
        ),
        Node(
            package='uwb_driver',
            executable='uwb_aoa_driver_node',
            name='uwb_aoa_driver_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baud': 115200}],
        ),
        set_body_frame,
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_body_frame,
                on_exit=[test_mission_node],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test_mission_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='uwb_platform_grasp_return_direct_land mission completed'
                        )
                    )
                ],
            )
        ),
    ])
