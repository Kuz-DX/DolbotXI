#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import cv2
import numpy as np

class PCVisualizer(Node):
    def __init__(self):
        super().__init__('pc_visualizer_node')
        self.bridge = CvBridge()

        self.current_mode = "WAITING"
        self.current_slope = 0.0
        
        # [수정 완료] 실제 확인된 토픽 이름으로 변경 (/camera/camera -> /camera)
        self.sub_rgb = self.create_subscription(
            CompressedImage, 
            '/camera/color/image_raw/compressed',  # <--- 여기를 고쳤습니다!
            self.rgb_cb, 
            qos_profile_sensor_data
        )
        
        self.sub_mode = self.create_subscription(String, '/system/mode', self.mode_cb, 10)
        self.sub_slope = self.create_subscription(Float32, '/terrain/side_slope_angle_deg', self.slope_cb, 10)
        
        self.get_logger().info("PC Visualizer Started (Topic Fixed)")

    def mode_cb(self, msg): self.current_mode = msg.data
    def slope_cb(self, msg): self.current_slope = msg.data

    def rgb_cb(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Decoding Failed: {e}")
            return

        if cv_img is None: return

        h, w = cv_img.shape[:2]

        # --- HUD 그리기 ---
        y0, y1 = int(h * 0.6), int(h * 0.9)
        mid = w // 2
        margin = int(w * 0.05)
        edge = int(w * 0.30)

        cv2.rectangle(cv_img, (edge, y0), (mid - margin, y1), (0, 255, 0), 2)
        cv2.rectangle(cv_img, (mid + margin, y0), (w - edge, y1), (255, 0, 0), 2)

        mode_color = (0, 0, 255) if self.current_mode == "SLOPE" else (0, 255, 0)
        cv2.putText(cv_img, f"MODE: {self.current_mode}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2)
        cv2.putText(cv_img, f"SLOPE: {self.current_slope:.2f} deg", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        cv2.imshow("Robot View (Compressed)", cv_img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = PCVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()