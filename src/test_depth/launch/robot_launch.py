#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. RealSense 카메라 (압축 설정 강제 주입)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            # 기본 활성화
            'enable_color': True,
            'enable_depth': True,
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'align_depth.enable': True,

            # [핵심] 압축 포맷 강제 설정
            # Color -> JPEG (PC 보기용, 빠름)
            'rgb_camera.image_raw.compressed.format': 'jpeg',
            'rgb_camera.image_raw.compressed.jpeg_quality': 80,
        }]
    )

    # 2. Slope Trigger (압축 Depth 사용하도록 수정됨)
    slope_trigger = Node(
        package='test_depth',
        executable='slope_trigger',
        name='slope_trigger_node',
        output='screen'
    )

    # 3. Mode Manager
    mode_manager = Node(
        package='test_depth',
        executable='mode_manager',
        name='mode_manager_node',
        output='screen'
    )

    # Visualizer는 로봇에서 실행하지 않음! (PC에서 실행)

    return LaunchDescription([
        realsense_node,
        slope_trigger,
        mode_manager
    ])