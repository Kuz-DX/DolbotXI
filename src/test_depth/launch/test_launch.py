#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Slope Trigger Node (횡경사 감지)
        Node(
            package='test_depth',
            executable='slope_trigger',  # setup.py의 entry_points 이름과 일치해야 함
            name='slope_trigger_node',
            output='screen'
        ),

        # 2. Mode Manager Node (모드 전환 관리)
        Node(
            package='test_depth',
            executable='mode_manager',
            name='mode_manager_node',
            output='screen'
        ),

        # # 3. Slope Terrain Planner Node (경로 생성)
        # Node(
        #     package='test_depth',
        #     executable='slope_terrain_planner',
        #     name='slope_terrain_planner_node',
        #     output='screen'
        # ),

        # # 4. Visualizer (시각화)
        # Node(
        #     package='test_depth',
        #     executable='visualizer',
        #     name='visualizer_node',
        #     output='screen'
        # )
    ])