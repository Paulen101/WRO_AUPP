#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
import struct
import threading
import pyrealsense2 as rs
import numpy as np
import cv2
from std_msgs.msg import Float32

class MotorServoController(Node):
    def __init__(self):
        super().__init__('motor_servo_controller')

        # CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
       
        self.first90_active = False            # currently in the 0–90° boost window
        self.first90_done = False              # boost already used once
        # Desired values
        self.slowdown_done = False    # <<< only slow down once on the 3rd lap
        self.desired_speed = 700
        self.servo_angle = 90
        self.base_speed = self.desired_speed   # remember -1400 (or whatever you start with)
        # Wall following parameters
        self.target_distance = 0.65   # left wall (m)
        self.Kp = 100

        # Front obstacle parameters
        self.front_threshold = 0.82   # (m)
        self.front_max_range = 1.0   # ignore floor if too far

        # ROI & camera parameters
        self.roi_height_px = 10
        self.roi_width_px = 90
        self.front_roi_height_px = 20

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        profile = self.pipeline.get_active_profile()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # Start periodic CAN sender
        self.timer = self.create_timer(0.1, self.send_command)

        # Yaw tracking
        self.subscription = self.create_subscription(Float32, '/bno055/yaw_deg', self.yaw_callback, 10)
        self.passed_90 = False
        self.passed_180 = False
        self.passed_270 = False
        self.rotation_count = 0
        self.robot_stopped = False

        self.get_logger().info("✅ Motor & Servo controller following LEFT wall with full rotation stop.")

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
        left_dist, front_dist, color_image, depth_colormap = self.process_frames(depth_frame, color_frame)

        if front_dist is not None and front_dist <= self.front_threshold:
            self.servo_angle = 45
            self.get_logger().info(f"🚧 Front wall at {front_dist:.2f}m → Turning left (servo=10°).")
        elif left_dist is not None:
            if 0.2<= left_dist <= 0.9:
                error = self.target_distance - left_dist
                offset = self.Kp * error
                self.servo_angle = 90 + offset
                self.servo_angle = max(45, min(135, self.servo_angle))
                self.get_logger().info(f"📐 LeftDist={left_dist:.2f}m | Following wall | Servo={self.servo_angle:.1f}")
            else:
                self.servo_angle = 90
                self.get_logger().info(f"↔️ LeftDist={left_dist:.2f}m out of [0.4, 1.0] → Going straight.")
        else:
            self.servo_angle = 90
            self.get_logger().info("➡️ Left wall not detected → Going straight.")

        # cv2.imshow("Color", color_image)
        # cv2.imshow("Depth", depth_colormap)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.stop()
            rclpy.shutdown()
            return

        servo_raw = int(self.servo_angle * 100)
        data = struct.pack(">ii", self.desired_speed, servo_raw)
        msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
        self.bus.send(msg)

    def process_frames(self, depth_frame, color_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        height, width = depth_image.shape

        target_row = int(height * 0.55)
        roi_x = 0
        roi_y = max(0, target_row - self.roi_height_px//2)
        left_roi = depth_image[roi_y:roi_y+self.roi_height_px, roi_x:roi_x+self.roi_width_px]
        left_valid = left_roi[left_roi > 0]
        left_dist = np.median(left_valid) * self.depth_scale if left_valid.size > 0 else None

        front_x = (width - self.roi_width_px) // 2
        front_y = int(height * 0.46)
        front_roi = depth_image[front_y:front_y+self.front_roi_height_px,
                                front_x:front_x+self.roi_width_px]
        front_valid = front_roi[(front_roi > 0) &
                                (front_roi < (self.front_max_range / self.depth_scale))]
        front_dist = np.median(front_valid) * self.depth_scale if front_valid.size > 0 else None

        cv2.rectangle(color_image, (roi_x, roi_y),
                      (roi_x+self.roi_width_px, roi_y+self.roi_height_px), (0, 0, 255), 2)
        dist_text = f"Left: {left_dist:.2f}m" if left_dist else "Left: N/A"
        cv2.putText(color_image, dist_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

        cv2.rectangle(color_image, (front_x, front_y),
                      (front_x+self.roi_width_px, front_y+self.front_roi_height_px), (255, 0, 0), 2)
        front_text = f"Front: {front_dist:.2f}m" if front_dist else "Front: N/A"
        cv2.putText(color_image, front_text, (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.rectangle(depth_colormap, (roi_x, roi_y),
                      (roi_x+self.roi_width_px, roi_y+self.roi_height_px), (0,255,0), 2)
        cv2.rectangle(depth_colormap, (front_x, front_y),
                      (front_x+self.roi_width_px, front_y+self.front_roi_height_px), (255,0,0), 2)

        return left_dist, front_dist, color_image, depth_colormap

    def yaw_callback(self, msg):
        yaw = msg.data
        self.get_logger().info(f"🧭 Yaw: {yaw:.2f}°")

        # --- First-lap 0°→90° boost to 1700 ---
        if self.rotation_count == 0 and not self.robot_stopped:
            # Enter boost window (only once on first lap)
            if (0.0 <= yaw <= 90.0) and not self.first90_done and not self.first90_active:
                sign = 1 if self.base_speed < 0 else 1
                self.desired_speed = sign * 1300
                self.first90_active = True
                self.get_logger().info("🚀 First lap 0–90° → speed set to 1700.")

        # Sector markers (your original thresholds)
        if 80 <= yaw <= 100:
            self.passed_90 = True
            # Leave boost window once we've passed 90°, restore baseline
            if self.first90_active and not self.first90_done:
                self.desired_speed = self.base_speed
                self.first90_active = False
                self.first90_done = True
                self.get_logger().info("↩️ Passed 90° on first lap → restoring baseline speed.")
        elif 170 <= yaw <= 190:
            self.passed_180 = True
        elif 220 <= yaw <= 300:
            self.passed_270 = True

            # Third-lap slowdown at 270° (keeps your existing logic)
            if self.rotation_count == 2 and not self.slowdown_done and not self.robot_stopped:
                sign = 1 if self.desired_speed < 0 else 1
                self.desired_speed = sign *300  # use 1000 if you prefer: sign * 1000
                self.slowdown_done = True
                self.get_logger().info("🕘 Third lap @270° → slowing to 700.")

        # Full-rotation check (unchanged)
        if self.passed_90 and self.passed_180 and self.passed_270:
            if yaw <= 10 or yaw >= 350:
                self.rotation_count += 1
                self.get_logger().info(f"🔁 Completed {self.rotation_count} full rotation(s).")
                self.passed_90 = self.passed_180 = self.passed_270 = False

                if self.rotation_count >= 3 and not self.robot_stopped:
                    self.get_logger().info("✅ Completed 3 full rotations — move forward before stopping.")
                    self.move_forward_for_duration()
                    self.robot_stopped = True

    def move_forward_for_duration(self, speed=0, duration_sec=0.02):
        self.get_logger().info(f"⏩ Moving forward for {duration_sec} sec before final stop...")
        self.desired_speed = speed
        self.servo_angle = 90
        servo_raw = int(self.servo_angle * 100)
        data = struct.pack(">ii", self.desired_speed, servo_raw)
        msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
        self.bus.send(msg)

        rclpy.spin_once(self, timeout_sec=0.1)  # Allow node to process events
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
