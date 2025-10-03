import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations

from .policy_inference import PolicyExecutor

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        # Declare parameters for model path and goal position
        self.declare_parameter('model_path', 'default/path/to/model.zip')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('lidar_beams', 36)

        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.lidar_beams = self.get_parameter('lidar_beams').get_parameter_value().integer_value
        
        self.goal_pos = np.array([goal_x, goal_y])
        self.get_logger().info(f"Loading model from: {model_path}")
        self.get_logger().info(f"Navigating to goal: {self.goal_pos}")

        # Instantiate the policy executor (from the other file)
        self.policy_executor = PolicyExecutor(model_path)

        # Class variables to store the latest state
        self.current_pose = None
        self.current_velocity = np.zeros(3) # vx, vy, wz

        # Create publisher for /cmd_vel
        self.cmd_vel_publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Create subscriber to odometry
        self.odom_subscription_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # The main callback is triggered by the LiDAR scan
        self.lidar_subscription_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        # Store the robot's current pose
        self.current_pose = msg.pose.pose
        
        # Store the robot's current velocity [vx_body, vy_body, wz]
        # Odometry gives vx in the robot's frame, but vy in the world frame's orientation.
        # We need to rotate the world vy into the robot's frame.
        q = self.current_pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        vx_world = msg.twist.twist.linear.x
        vy_world = msg.twist.twist.linear.y

        self.current_velocity[0] = vx_world * np.cos(-yaw) - vy_world * np.sin(-yaw) # vx_body
        self.current_velocity[1] = vx_world * np.sin(-yaw) + vy_world * np.cos(-yaw) # vy_body
        self.current_velocity[2] = msg.twist.twist.angular.z # wz

    def lidar_callback(self, msg: LaserScan):
        # Wait until we have odometry data
        if self.current_pose is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        # 1. Process real-world data into the observation format
        observation = self.process_observation(msg)

        # 2. Get the action from the policy
        action = self.policy_executor.predict_action(observation)

        if action is not None:
            # 3. Convert the action into a TwistStamped message
            twist_msg = self.action_to_twist(action)
            
            # 4. Publish the command
            self.cmd_vel_publisher_.publish(twist_msg)

    def process_observation(self, lidar_msg: LaserScan) -> np.ndarray:
        # --- This logic is copied and adapted from your MecanumRobotEnv ---

        # Part 1: Process LiDAR
        # Your RPLidar A1 provides ~360 samples. Your policy expects 36.
        # We must downsample the LiDAR data.
        full_scan = np.array(lidar_msg.ranges)
        full_scan[np.isinf(full_scan)] = lidar_msg.range_max
        full_scan[np.isnan(full_scan)] = lidar_msg.range_min
        
        # Simple downsampling by taking every Nth element
        step = len(full_scan) // self.lidar_beams
        lidar_readings = full_scan[::step][:self.lidar_beams]

        # Part 2: Calculate goal distance and angle
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        q = self.current_pose.orientation
        # The z-axis rotation is the yaw
        _, _, robot_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = self.goal_pos[0] - robot_x
        dy = self.goal_pos[1] - robot_y
        dist_to_goal = np.sqrt(dx**2 + dy**2)
        angle_to_goal = np.arctan2(dy, dx) - robot_yaw
        # Normalize angle to [-pi, pi]
        angle_to_goal = np.arctan2(np.sin(angle_to_goal), np.cos(angle_to_goal))
        
        # Part 3: Concatenate into the final observation vector
        # The order MUST BE IDENTICAL to your training environment
        obs = np.concatenate([
            self.current_velocity,
            lidar_readings,
            np.array([dist_to_goal, angle_to_goal])
        ]).astype(np.float32)
        
        return obs

    def action_to_twist(self, action: np.ndarray) -> TwistStamped:
        # This logic is copied from your MecanumRobotEnv action space
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = float(action[0])  # vx
        twist_stamped.twist.linear.y = float(action[1])  # vy (for holonomic movement)
        twist_stamped.twist.angular.z = float(action[2]) # wz
        return twist_stamped


def main(args=None):
    rclpy.init(args=args)
    policy_node = PolicyNode()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()