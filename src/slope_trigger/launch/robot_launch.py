#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Slope Trigger (압축 Depth 사용하도록 수정됨)
    slope_trigger = Node(
        package='slope_trigger',
        executable='slope_trigger',
        name='slope_trigger_node',
        output='screen'
    )

    # 2. Mode Manager
    mode_manager = Node(
        package='slope_trigger',
        executable='mode_manager',
        name='mode_manager_node',
        output='screen'
    )

    # Visualizer는 로봇에서 실행하지 않음! (PC에서 실행)

    return LaunchDescription([
        slope_trigger,
        mode_manager
    ])