#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

class StudyWheelsNode(Node):
    def __init__(self):
        super().__init__('study_wheels')
        
        # Updated to use TwistStamped for the velocity smoother / mecanum controller
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/int_brain/odom', self.odom_callback, 10)
        
        self.vx_data = []
        self.vy_data = []
        self.wz_data = []
        self.is_recording = False

        self.get_logger().info("Starting Wheel Covariance Calibration in 3 seconds...")
        self.timer = self.create_timer(1.0, self.state_machine)
        self.start_time = time.time()
        self.state = 0

    def odom_callback(self, msg):
        if self.is_recording:
            self.vx_data.append(msg.twist.twist.linear.x)
            self.vy_data.append(msg.twist.twist.linear.y)
            self.wz_data.append(msg.twist.twist.angular.z)

    def state_machine(self):
        elapsed = time.time() - self.start_time
        
        # Construct the Stamped message with current ROS time
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # State 0: Wait before starting
        if elapsed < 3.0:
            pass

        # State 1: Drive Forward (0.2 m/s)
        elif 3.0 <= elapsed < 8.0:
            if self.state == 0:
                self.get_logger().info("Driving Forward at 0.2 m/s...")
                self.state = 1
            msg.twist.linear.x = 0.2
            if elapsed > 4.5: # Give it 1.5s to reach steady state before recording
                self.is_recording = True

        # State 2: Strafe Sideways (0.2 m/s) for Mecanum testing
        elif 8.0 <= elapsed < 13.0:
            if self.state == 1:
                self.get_logger().info("Strafing Left at 0.2 m/s...")
                self.state = 2
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.2
            # Keep recording, treating the noise profile as similar

        # State 3: Stop and Calculate
        elif elapsed >= 13.0:
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            self.cmd_pub.publish(msg)
            self.is_recording = False
            self.calculate_and_exit()
            return

        self.cmd_pub.publish(msg)

    def calculate_and_exit(self):
        self.timer.cancel()
        
        if not self.vx_data:
            self.get_logger().error("No odometry data received on /int_brain/odom!")
            rclpy.shutdown()
            return

        # Calculate pure variance (sigma squared)
        vx_var = np.var(self.vx_data)
        vy_var = np.var(self.vy_data)
        
        self.get_logger().info("=== CALIBRATION RESULTS ===")
        self.get_logger().info(f"Raw X Velocity Variance: {vx_var:.6f}")
        self.get_logger().info(f"Raw Y Velocity Variance: {vy_var:.6f}")
        
        # Apply the 5x "Mecanum slip penalty" for realistic floor tracking
        recommended_x = vx_var * 5.0
        recommended_y = vy_var * 5.0
        
        self.get_logger().info("=== RECOMMENDED VALUES FOR controllers.yaml ===")
        self.get_logger().info("Populate twist_covariance_diagonal (assuming standard 36-element array):")
        self.get_logger().info(f"X (Index 0): {recommended_x:.5f}")
        self.get_logger().info(f"Y (Index 7): {recommended_y:.5f}")
        self.get_logger().info(f"Yaw (Index 35): 0.2 (Hardcoded high because mecanum turns slip heavily)")
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StudyWheelsNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()