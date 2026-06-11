#!/usr/bin/env python3
"""真实 FCU + UWB 测试 (MAVROS 需先单独启动)"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('uwb_navigation')

    return LaunchDescription([
        # ── FCU 桥接 ──
        Node(
            package='fcu_bridge', executable='fcu_state_node', name='fcu_state_node',
        ),
        Node(
            package='fcu_bridge', executable='flight_commander_node', name='flight_commander_node',
            parameters=[{'skip_ekf_check': True}],
        ),
        Node(
            package='fcu_bridge', executable='flight_state_machine_node', name='flight_state_machine_node',
        ),

        # ── UWB 驱动 ──
        Node(
            package='uwb_driver', executable='uwb_aoa_driver_node', name='uwb_aoa_driver_node',
            parameters=[{'serial_port': '/dev/ttySTM1', 'baud_rate': 115200}],
        ),

        # ── 测试任务 ──
        Node(
            package='uwb_navigation',
            executable='test_mission_node.py',
            name='test_mission_node',
            parameters=[os.path.join(share_dir, 'test_mission.yaml')],
            output='screen',
        ),
    ])
