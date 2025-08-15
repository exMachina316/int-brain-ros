import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64
import numpy as np
import csv

class SystemIdNode(Node):
    def __init__(self):
        super().__init__('system_id_node')
        self.command_publisher_ = self.create_publisher(Float64, '/system_identifier/command', 10)

        # Define a QoS profile with TRANSIENT_LOCAL durability
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.response_subscription = self.create_subscription(
            Float64,
            '/system_identifier/response',
            self.response_callback,
            qos_profile)

        self.max_value = 34.5
        self.min_value = -34.5
        self.step = 0.1
        self.current_value = 0.0
        self.state = 1  # 1: 0->max, 2: max->min, 3: min->0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.data_pairs = []
        self.last_command = None

    def response_callback(self, msg):
        if self.last_command is not None:
            self.data_pairs.append((self.last_command, msg.data))
            self.last_command = None  # To avoid reusing the same command

    def timer_callback(self):
        if self.state == 1:  # Ramping up from 0 to max_value
            if self.current_value >= self.max_value:
                self.state = 2
                self.current_value = self.max_value
            else:
                self.current_value += self.step
        elif self.state == 2:  # Ramping down from max_value to min_value
            if self.current_value <= self.min_value:
                self.state = 3
                self.current_value = self.min_value
            else:
                self.current_value -= self.step
        elif self.state == 3:  # Ramping up from min_value to 0
            if self.current_value >= 0:
                self.state = 4  # Finished
                self.current_value = 0.0
            else:
                self.current_value += self.step

        if self.state == 4:
            self.get_logger().info('Finished publishing')
            self.export_to_csv()
            self.timer.cancel()
            # Allow some time for final responses to arrive before shutting down
            self.create_timer(1.0, self.shutdown_node)
            return

        msg = Float64()
        msg.data = self.current_value
        self.last_command = msg.data
        self.command_publisher_.publish(msg)

    def export_to_csv(self):
        self.get_logger().info('Exporting data to system_id_data.csv')
        with open('system_id_data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['command', 'response'])
            writer.writerows(self.data_pairs)
        self.get_logger().info('Finished exporting data')

    def shutdown_node(self):
        self.get_logger().info('Shutting down node.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    system_id_node = SystemIdNode()

    rclpy.spin(system_id_node)

    # The node is now shut down inside shutdown_node, so these lines are not strictly necessary
    # if spin() exits cleanly.
    if rclpy.ok():
        system_id_node.destroy_node()
        rclpy.shutdown()
