import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# [변경] CompressedImage 메시지 타입 추가
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class DepthTrigger:
    """Depth 이미지 기반 슬로프 판정 클래스"""
    def __init__(self, node: Node, roi_y0, roi_y1, sample_step, max_depth, track_w):
        self.node = node
        self.roi_y0 = roi_y0
        self.roi_y1 = roi_y1
        self.sample_step = sample_step
        self.max_depth = max_depth
        self.track_w = track_w
        
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.current_angle = 0.0

    def update_camera_info(self, fx, fy, cx, cy):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy

    def process(self, cv_img: np.ndarray) -> float:
        if self.fx is None:
            return 0.0

        # Depth 변환 (mm -> m)
        if cv_img.dtype == np.uint16:
            depth = cv_img.astype(np.float32) * 0.001
        else:
            depth = cv_img.astype(np.float32)

        h, w = depth.shape[:2]
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
            return None

        dh = (hR - hL)
        roll_rad = math.atan2(dh, self.track_w)
        self.current_angle = abs(math.degrees(roll_rad))
        return self.current_angle


class ImuTrigger:
    """IMU 기반 슬로프 판정 클래스"""
    def __init__(self, node: Node):
        self.node = node
        self.current_angle = 0.0
        self.accel_x = 0.0

    def process(self, msg: Imu) -> float:
        # [변경] Orientation 대신 Accel X 사용 (/camera/accel/sample)
        # Accel X값을 각도로 변환 (g=9.8 가정)
        raw_x = msg.linear_acceleration.x
        
        # asin domain check (-1 ~ 1)
        ratio = raw_x / 9.8
        ratio = max(-1.0, min(1.0, ratio))
        
        self.current_angle = abs(math.degrees(math.asin(ratio)))
        self.accel_x = raw_x
        
        return self.current_angle


class slope_trigger_node(Node):
    def __init__(self):
        super().__init__('side_slope_trigger_node')
        self.get_logger().info("Slope Trigger Node Initialized (Dual Trigger Mode).")

        # ROI 설정
        self.declare_parameter('roi_y_start_ratio', 0.60)
        self.declare_parameter('roi_y_end_ratio',   0.90)
        self.declare_parameter('sample_step', 8)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('track_width_m', 0.45) 
        
        # Trigger 설정
        self.declare_parameter('slope_threshold_deg', 5.0) # Slope 판정 기준
        self.declare_parameter('flat_threshold_deg', 2.0)  # Flat 판정 기준
        self.declare_parameter('imu_wait_timeout_sec', 10.0)

        self.roi_y0 = float(self.get_parameter('roi_y_start_ratio').value)
        self.roi_y1 = float(self.get_parameter('roi_y_end_ratio').value)
        self.sample_step = int(self.get_parameter('sample_step').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.track_w = float(self.get_parameter('track_width_m').value)
        
        self.slope_thresh = float(self.get_parameter('slope_threshold_deg').value)
        self.flat_thresh = float(self.get_parameter('flat_threshold_deg').value)
        self.imu_timeout = float(self.get_parameter('imu_wait_timeout_sec').value)

        self.last_log_time = 0.0
        self.bridge = CvBridge()

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
        
        # [추가] IMU 구독
        self.sub_imu = self.create_subscription(
            Imu, '/camera/accel/sample', self.on_imu, qos_profile_sensor_data
        )
        
        self.pub_slope = self.create_publisher(Float32, '/terrain/side_slope_angle_deg', 10)
        
        # [추가] 연결 상태 확인용 타이머 (2초마다 실행)
        self.create_timer(2.0, self.check_connection_status)

        # --- 클래스 인스턴스화 ---
        self.depth_trigger = DepthTrigger(self, self.roi_y0, self.roi_y1, self.sample_step, self.max_depth, self.track_w)
        self.imu_trigger = ImuTrigger(self)

        # --- 상태 관리 변수 ---
        self.state = "DEPTH_MODE" # DEPTH_MODE, WAITING_IMU, IMU_LOCKED
        self.wait_start_time = 0.0

    def check_connection_status(self):
        if self.depth_trigger.fx is None:
            self.get_logger().warn(f"Waiting for CameraInfo... (Topic: {self.sub_info.topic_name})")
        elif time.time() - self.last_log_time > 5.0:
            self.get_logger().warn(f"Waiting for Depth images... (Topic: {self.sub_depth.topic_name})")

    def on_info(self, msg: CameraInfo):
        self.depth_trigger.update_camera_info(msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        # [디버깅] 카메라 정보 수신 확인 (최초 1회만 출력)
        if not hasattr(self, '_info_logged'):
            self.get_logger().info(f"CameraInfo Received: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}")
            self._info_logged = True

    def on_imu(self, msg: Imu):
        # IMU 데이터 업데이트 (항상 최신값 유지)
        self.imu_trigger.process(msg)

    # [복구] 콜백 함수 입력 타입 변경 (CompressedImage -> Image)
    def on_depth(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth Conversion Error: {e}")
            return

        # 1. Depth 기반 각도 계산
        depth_angle = self.depth_trigger.process(cv_img)
        if depth_angle is None:
            # Depth 데이터가 유효하지 않으면 이전 상태 유지하거나 리턴
            return
        
        imu_angle = self.imu_trigger.current_angle
        now = time.time()
        final_angle = 0.0

        # --- 상태 머신 로직 ---
        if self.state == "DEPTH_MODE":
            final_angle = depth_angle
            # Depth가 Slope라고 판단하면 IMU 대기 모드로 진입
            if depth_angle > self.slope_thresh:
                self.state = "WAITING_IMU"
                self.wait_start_time = now
                self.get_logger().warn(f"[Trigger] Depth Slope Detected ({depth_angle:.1f} deg). Waiting for IMU...")
        
        elif self.state == "WAITING_IMU":
            final_angle = depth_angle # 대기 중에는 아직 Depth 값 신뢰
            
            # 10초 타임아웃 체크
            if now - self.wait_start_time > self.imu_timeout:
                self.state = "DEPTH_MODE"
                self.get_logger().info("[Trigger] IMU Timeout. Reverting to DEPTH_MODE.")
            
            # IMU도 Slope라고 판단하면 (Accel X 변화 등) -> IMU Locked 모드 진입
            # 요청사항: "imu acel x값이 10초안에 바뀌면" -> 여기서는 Roll 각도 변화로 통합하여 처리 (Slope = Roll)
            elif imu_angle > self.slope_thresh:
                self.state = "IMU_LOCKED"
                self.get_logger().warn(f"[Trigger] IMU Confirmed Slope ({imu_angle:.1f} deg). Locked to IMU.")

        elif self.state == "IMU_LOCKED":
            final_angle = imu_angle # 이제부터는 IMU 값만 사용
            
            # IMU가 Flat이라고 판단하면 다시 Depth 모드로 복귀
            if imu_angle < self.flat_thresh:
                self.state = "DEPTH_MODE"
                self.get_logger().warn(f"[Trigger] IMU Flat Detected ({imu_angle:.1f} deg). Reverting to DEPTH_MODE.")

        # --- 결과 발행 ---
        out = Float32()
        out.data = float(final_angle)
        self.pub_slope.publish(out)
        
        # [디버깅] 정상 계산된 각도 출력 (1초마다 출력)
        if time.time() - self.last_log_time > 1.0:
            self.get_logger().info(f"State: {self.state} | Pub Angle: {final_angle:.2f} (Depth: {depth_angle:.1f}, IMU: {imu_angle:.1f})")
            self.last_log_time = time.time()

def main():
    rclpy.init()
    node = slope_trigger_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()