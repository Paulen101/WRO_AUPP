

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
import struct
import pyrealsense2 as rs
import numpy as np
import cv2
from std_msgs.msg import Float32

class MotorServoController(Node):
    def __init__(self):
        super().__init__('motor_servo_controller')

        # CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)

        # Motion and control parameters
        self.desired_speed = 4000
        self.servo_angle = 90
        self.target_distance = 0.5  # Right wall distance target (meters)
        self.Kp = 100

        # Front obstacle detection
        self.front_threshold = 0.9  # Stop turning if object is this close
        self.front_max_range = 1.0  # Ignore noise beyond this range

        # ROI configuration
        self.roi_height_px = 10
        self.roi_width_px = 90
        self.front_roi_height_px = 30

        # RealSense config
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        profile = self.pipeline.get_active_profile()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # CAN send loop
        self.timer = self.create_timer(0.1, self.send_command)

        # IMU yaw tracking for clockwise rotation
        self.subscription = self.create_subscription(Float32, '/bno055/yaw_deg', self.yaw_callback, 10)
        self.passed_270 = False
        self.passed_180 = False
        self.passed_90 = False
        self.rotation_count = 0
        self.robot_stopped = False

        self.get_logger().info("✅ Right-wall follower initialized with clockwise rotation tracking.")

    def send_command(self):
        if self.robot_stopped:
            return

        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        width, height = depth_frame.get_width(), depth_frame.get_height()
        right_dist, front_dist, _, _ = self.process_frames(depth_frame, color_frame)

        if front_dist is not None and front_dist <= self.front_threshold:
            self.servo_angle = 165  # Turn right sharply
            self.get_logger().info(f"🚧 Obstacle ahead at {front_dist:.2f}m → Turning right (servo=165°).")
        elif right_dist is not None:
            if 0.2 <= right_dist <= 0.9:
                error = right_dist - self.target_distance
                offset = self.Kp * error
                self.servo_angle = 90 + offset
                self.servo_angle = max(35, min(135, self.servo_angle))
                self.get_logger().info(f"📐 RightDist={right_dist:.2f}m | Adjusted Servo={self.servo_angle:.1f}")
            else:
                self.servo_angle = 90
                self.get_logger().info(f"↔️ RightDist={right_dist:.2f}m out of range → Going straight.")
        else:
            self.servo_angle = 90
            self.get_logger().info("➡️ No right wall detected → Going straight.")

        # Allow manual stop
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.stop()
            rclpy.shutdown()
            return

        # Send to CAN
        servo_raw = int(self.servo_angle * 100)
        data = struct.pack(">ii", self.desired_speed, servo_raw)
        msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
        self.bus.send(msg)

    def process_frames(self, depth_frame, color_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        height, width = depth_image.shape

        # Right side ROI
        roi_x = width - self.roi_width_px
        roi_y = max(0, int(height * 0.55) - self.roi_height_px // 2)
        right_roi = depth_image[roi_y:roi_y + self.roi_height_px, roi_x:roi_x + self.roi_width_px]
        right_valid = right_roi[right_roi > 0]
        right_dist = np.median(right_valid) * self.depth_scale if right_valid.size > 0 else None

        # Front ROI
        front_x = (width - self.roi_width_px) // 2
        front_y = int(height * 0.5)
        front_roi = depth_image[front_y:front_y + self.front_roi_height_px,
                                front_x:front_x + self.roi_width_px]
        front_valid = front_roi[(front_roi > 0) &
                                (front_roi < (self.front_max_range / self.depth_scale))]
        front_dist = np.median(front_valid) * self.depth_scale if front_valid.size > 0 else None

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        return right_dist, front_dist, color_image, depth_colormap

    def yaw_callback(self, msg):
        yaw = msg.data
        self.get_logger().info(f"🧭 Yaw: {yaw:.2f}°")

        # Track clockwise (right-turn) sequence: 270° → 180° → 90° → (wraps around to ~0°)
        if 260 <= yaw <= 280:
            self.passed_270 = True
        elif 170 <= yaw <= 190 and self.passed_270:
            self.passed_180 = True
        elif 80 <= yaw <= 100 and self.passed_180:
            self.passed_90 = True

        if self.passed_270 and self.passed_180 and self.passed_90:
            if yaw <= 10 or yaw >= 350:
                self.rotation_count += 1
                self.get_logger().info(f"🔁 Completed {self.rotation_count} full clockwise rotation(s).")
                self.passed_270 = self.passed_180 = self.passed_90 = False

                if self.rotation_count >= 3 and not self.robot_stopped:
                    self.get_logger().info("✅ Completed 3 full rotations — move forward before stopping.")
                    self.move_forward_for_duration()
                    self.robot_stopped = True

    def move_forward_for_duration(self, speed=10, duration_sec=0.05):
        self.get_logger().info(f"⏩ Moving forward for {duration_sec}s before stopping...")
        self.desired_speed = speed
        self.servo_angle = 90
        servo_raw = int(self.servo_angle * 100)
        data = struct.pack(">ii", self.desired_speed, servo_raw)
        msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
        self.bus.send(msg)

        rclpy.spin_once(self, timeout_sec=0.1)
        self.sleep_for(duration_sec)
        self.stop()

    def sleep_for(self, seconds):
        end_time = self.get_clock().now().to_msg().sec + int(seconds)
        while self.get_clock().now().to_msg().sec < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def stop(self):
        self.desired_speed = 0
        self.servo_angle = 90
        servo_raw = int(self.servo_angle * 100)
        data = struct.pack(">ii", self.desired_speed, servo_raw)
        msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
        self.bus.send(msg)
        self.get_logger().info("🛑 Motors stopped, servo reset to 90°.")
        cv2.destroyAllWindows()

    def destroy_node(self):
        self.pipeline.stop()
        self.bus.shutdown()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down — sending stop command...")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
