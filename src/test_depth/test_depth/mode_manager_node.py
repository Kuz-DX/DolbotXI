#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class mode_manager_node(Node):
    def __init__(self):
        super().__init__('mode_manager_node')

        self.declare_parameter('enter_slope_deg', 8.0)
        self.declare_parameter('exit_slope_deg', 6.0)
        self.declare_parameter('enter_frames', 5)
        self.declare_parameter('exit_frames', 10)
        self.declare_parameter('min_hold_ms', 2000)

        self.enter_deg = float(self.get_parameter('enter_slope_deg').value)
        self.exit_deg = float(self.get_parameter('exit_slope_deg').value)
        self.enter_frames = int(self.get_parameter('enter_frames').value)
        self.exit_frames = int(self.get_parameter('exit_frames').value)
        self.min_hold_ms = int(self.get_parameter('min_hold_ms').value)

        self.mode = 'FLAT'
        self.enter_cnt = 0
        self.exit_cnt = 0
        self.last_switch = time.time()
        self.last_log_time = 0.0

        self.sub = self.create_subscription(Float32, '/terrain/side_slope_angle_deg', self.on_slope, 10)
        self.pub = self.create_publisher(String, '/system/mode', 10)

        self.publish_mode()
        self.get_logger().info(f'ModeManager started. enter={self.enter_deg}, exit={self.exit_deg}')

    def publish_mode(self):
        m = String()
        m.data = self.mode
        self.pub.publish(m)

    def on_slope(self, msg: Float32):
        theta = float(msg.data)
        now = time.time()
        hold_ok = (now - self.last_switch) * 1000.0 >= self.min_hold_ms

        # [디버깅] 수신된 각도 로그 출력 (1초마다)
        if now - self.last_log_time > 1.0:
            self.get_logger().info(f"Received Slope: {theta:.2f} (Mode: {self.mode})")
            self.last_log_time = now

        if self.mode == 'FLAT':
            self.enter_cnt = self.enter_cnt + 1 if theta > self.enter_deg else 0
            if hold_ok and self.enter_cnt >= self.enter_frames:
                self.mode = 'SLOPE'
                self.last_switch = now
                self.enter_cnt = 0
                self.exit_cnt = 0
                self.publish_mode()
                self.get_logger().warn(f'Switch -> SLOPE (theta={theta:.2f})')
        else:  # SLOPE
            self.exit_cnt = self.exit_cnt + 1 if theta < self.exit_deg else 0
            if hold_ok and self.exit_cnt >= self.exit_frames:
                self.mode = 'FLAT'
                self.last_switch = now
                self.enter_cnt = 0
                self.exit_cnt = 0
                self.publish_mode()
                self.get_logger().warn(f'Switch -> FLAT (theta={theta:.2f})')


def main():
    rclpy.init()
    node = mode_manager_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()