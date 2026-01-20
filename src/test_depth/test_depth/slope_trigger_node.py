import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# [변경] CompressedImage 메시지 타입 추가
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class slope_trigger_node(Node):
    def __init__(self):
        super().__init__('side_slope_trigger_node')
        self.get_logger().info("Slope Trigger Node Initialized. Waiting for topics...")

        # ROI 설정
        self.declare_parameter('roi_y_start_ratio', 0.60)
        self.declare_parameter('roi_y_end_ratio',   0.90)
        self.declare_parameter('sample_step', 8)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('track_width_m', 0.45) 

        self.roi_y0 = float(self.get_parameter('roi_y_start_ratio').value)
        self.roi_y1 = float(self.get_parameter('roi_y_end_ratio').value)
        self.sample_step = int(self.get_parameter('sample_step').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.track_w = float(self.get_parameter('track_width_m').value)

        self.last_log_time = 0.0
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.on_info, qos_profile_sensor_data
        )
        
        # [복구] Raw Depth 토픽 구독 (로컬 실행 시 성능 및 호환성 유리)
        self.sub_depth = self.create_subscription(
            Image, 
            '/camera/depth/image_rect_raw', 
            self.on_depth, 
            qos_profile_sensor_data
        )
        
        self.pub_slope = self.create_publisher(Float32, '/terrain/side_slope_angle_deg', 10)
        
        # [추가] 연결 상태 확인용 타이머 (2초마다 실행)
        self.create_timer(2.0, self.check_connection_status)

    def check_connection_status(self):
        if self.fx is None:
            self.get_logger().warn(f"Waiting for CameraInfo... (Topic: {self.sub_info.topic_name})")
        elif time.time() - self.last_log_time > 5.0:
            self.get_logger().warn(f"Waiting for Depth images... (Topic: {self.sub_depth.topic_name})")

    def on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        # [디버깅] 카메라 정보 수신 확인 (최초 1회만 출력)
        if not hasattr(self, '_info_logged'):
            self.get_logger().info(f"CameraInfo Received: fx={self.fx:.1f}, fy={self.fy:.1f}")
            self._info_logged = True

    def _depth_to_meters(self, cv_img: np.ndarray) -> np.ndarray:
        if cv_img.dtype == np.uint16:
            return cv_img.astype(np.float32) * 0.001
        return cv_img.astype(np.float32)

    # [복구] 콜백 함수 입력 타입 변경 (CompressedImage -> Image)
    def on_depth(self, msg: Image):
        if self.fx is None:
            return

        try:
            # [복구] Raw 이미지 변환
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth Conversion Error: {e}")
            return
        
        depth = self._depth_to_meters(cv_img)
        h, w = depth.shape[:2]
        
        # --- 이하 로직은 기존과 동일 ---
        y_start = int(h * self.roi_y0)
        y_end   = int(h * self.roi_y1)
        y_start = max(0, min(h-1, y_start))
        y_end   = max(0, min(h, y_end))

        edge_margin = int(w * 0.30)
        mid_margin = int(w * 0.05)
        left_x0, left_x1   = edge_margin, w//2 - mid_margin
        right_x0, right_x1 = w//2 + mid_margin, w-edge_margin

        left_roi  = depth[y_start:y_end:self.sample_step, left_x0:left_x1:self.sample_step]
        right_roi = depth[y_start:y_end:self.sample_step, right_x0:right_x1:self.sample_step]

        def median_height(roi: np.ndarray, u0: int, v0: int):
            mask = np.isfinite(roi) & (roi > 0.1) & (roi < self.max_depth)
            if np.count_nonzero(mask) < 80:
                return None
            z = roi[mask]
            v_center = (y_start + y_end) * 0.5
            Y = (v_center - self.cy) * z / self.fy
            height = -Y 
            return float(np.nanmedian(height))

        hL = median_height(left_roi, left_x0, y_start)
        hR = median_height(right_roi, right_x0, y_start)
        
        if hL is None or hR is None:
            # [디버깅] 유효한 Depth 픽셀 부족 (1초마다 출력)
            if time.time() - self.last_log_time > 1.0:
                self.get_logger().warn(f"Invalid Depth ROI: hL={hL}, hR={hR} (Too close/far?)")
                self.last_log_time = time.time()
            return

        dh = (hR - hL)
        roll_rad = math.atan2(dh, self.track_w)
        roll_deg = math.degrees(roll_rad)

        out = Float32()
        out.data = float(abs(roll_deg))
        self.pub_slope.publish(out)
        
        # [디버깅] 정상 계산된 각도 출력 (1초마다 출력)
        if time.time() - self.last_log_time > 1.0:
            self.get_logger().info(f"Slope Calculated: {roll_deg:.2f} deg")
            self.last_log_time = time.time()

def main():
    rclpy.init()
    node = slope_trigger_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()