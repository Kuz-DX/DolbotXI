#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RealSense 카메라 노드 실행 설정
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            # 1. 기본 카메라 설정
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            
            # 해상도 및 FPS (너무 높으면 CPU 부하가 걸리므로 적절히 타협)
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'align_depth.enable': True,       # 컬러-뎁스 정렬

            # 2. [핵심] 압축 전송 설정 (대역폭 절약)
            
            # (A) RGB 컬러 영상 -> JPEG 압축 (빠르고 가벼움, 보기 용도)
            # image_raw/compressed 토픽의 포맷을 jpeg로 강제
            'rgb_camera.image_raw.compressed.format': 'jpeg',
            'rgb_camera.image_raw.compressed.jpeg_quality': 80, # 화질 80% (낮출수록 빠름)

            # (B) Depth 깊이 영상 -> PNG 압축 (데이터 깨짐 방지)
            # image_rect_raw/compressed 토픽의 포맷을 png로 강제
            'depth_module.depth_image_transport_format': 'png', 
            # 혹은 아래 파라미터 이름이 먹힐 수도 있음 (버전별 상이)
            'depth.image_rect_raw.compressed.format': 'png',
            'depth.image_rect_raw.compressed.png_level': 3, # 압축 레벨 (1:빠름 ~ 9:고압축)
        }]
    )

    return LaunchDescription([
        realsense_node
    ])