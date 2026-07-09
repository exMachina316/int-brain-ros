#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math

class StudyImuNode(Node):
    def __init__(self):
        super().__init__('study_imu')
        
        # Assuming default imu_sensor_broadcaster topic. Change if needed.
        self.imu_sub = self.create_subscription(Imu, '/imu_sensor/imu', self.imu_callback, 10)
        
        self.yaw_data = []
        self.wz_data = []
        self.samples_collected = 0
        self.target_samples = 500 # Approx 10 seconds at 50Hz

        self.get_logger().info("Please leave the robot completely still.")
        self.get_logger().info(f"Collecting {self.target_samples} samples from IMU...")

    def imu_callback(self, msg):
        if self.samples_collected >= self.target_samples:
            return

        # Extract quaternion
        q = msg.orientation
        
        # Convert Quaternion to Yaw (Euler Z)
        # yaw = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.yaw_data.append(yaw)
        self.wz_data.append(msg.angular_velocity.z)
        
        self.samples_collected += 1
        
        if self.samples_collected % 100 == 0:
            self.get_logger().info(f"Collected {self.samples_collected}/{self.target_samples} samples...")
            
        if self.samples_collected == self.target_samples:
            self.calculate_and_exit()

    def calculate_and_exit(self):
        yaw_var = np.var(self.yaw_data)
        wz_var = np.var(self.wz_data)
        
        self.get_logger().info("=== IMU CALIBRATION RESULTS ===")
        self.get_logger().info(f"Absolute Yaw Variance: {yaw_var:.8f} rad^2")
        self.get_logger().info(f"Angular Velocity Z Variance: {wz_var:.8f} (rad/s)^2")
        
        self.get_logger().info("=== RECOMMENDED VALUES FOR controllers.yaml ===")
        self.get_logger().info("Uncomment and update 'static_covariance_angular_velocity' in your config:")
        self.get_logger().info(f"Set the 9th element (Index 8) of angular_velocity covariance to: {wz_var:.6f}")
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StudyImuNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()