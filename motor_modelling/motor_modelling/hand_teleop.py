import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import json
import asyncio
import websockets
import math
import threading

class HandTeleop(Node):
    """
    A ROS2 node that connects to a WebSocket server to receive IMU data (roll, pitch, yaw),
    converts it to a TwistStamped message, and publishes it.
    """
    def __init__(self):
        super().__init__('hand_teleop_node')

        # Declare parameters for WebSocket server IP, port, and scaling factors
        self.declare_parameter('websocket_ip', '192.168.7.59') # CHANGE to your ESP8266's IP
        self.declare_parameter('websocket_port', 81)
        self.declare_parameter('pitch_to_linear_x_scale', 0.01) # Scale factor for forward/backward motion
        self.declare_parameter('roll_to_linear_y_scale', 0.01)  # Scale factor for sideway motion
        self.declare_parameter('yaw_to_angular_z_scale', 0.02)  # Scale factor for rotation
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('websocket_timeout', 1.0)  # Timeout in seconds for receiving data

        # Get parameters
        self.ip = self.get_parameter('websocket_ip').get_parameter_value().string_value
        self.port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.linear_x_scale = self.get_parameter('pitch_to_linear_x_scale').get_parameter_value().double_value
        self.linear_y_scale = self.get_parameter('roll_to_linear_y_scale').get_parameter_value().double_value
        self.angular_z_scale = self.get_parameter('yaw_to_angular_z_scale').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.websocket_timeout = self.get_parameter('websocket_timeout').get_parameter_value().double_value

        # Create a publisher for the TwistStamped message
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel_raw', 10)

        self.get_logger().info(f"Attempting to connect to WebSocket server at ws://{self.ip}:{self.port}")

        # Run the WebSocket client in a separate thread to avoid blocking rclpy.spin()
        self.ws_thread = threading.Thread(target=self.websocket_thread_func)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def websocket_thread_func(self):
        """
        Wrapper function to run the async WebSocket client.
        """
        try:
            asyncio.run(self.websocket_client())
        except Exception as e:
            self.get_logger().error(f"WebSocket thread error: {e}")

    async def websocket_client(self):
        """
        The main async function that connects to the server and processes messages.
        Includes a reconnection logic.
        """
        uri = f"ws://{self.ip}:{self.port}"
        while rclpy.ok():
            try:
                async with websockets.connect(uri) as websocket:
                    self.get_logger().info(f"Successfully connected to {uri}")
                    while rclpy.ok():
                        try:
                            message = await asyncio.wait_for(websocket.recv(), timeout=self.websocket_timeout)
                            self.process_imu_data(message)
                        except asyncio.TimeoutError:
                            self.get_logger().warn("WebSocket recv timed out. Reconnecting...")
                            break  # Break inner loop to reconnect
            except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError) as e:
                self.get_logger().warn(f"Connection lost or refused: {e}. Retrying in 1 seconds...")
                await asyncio.sleep(1)
            except Exception as e:
                self.get_logger().error(f"An unexpected WebSocket error occurred: {e}")
                await asyncio.sleep(1)

    def process_imu_data(self, message):
        """
        Parses the incoming JSON message and publishes a TwistStamped message.
        """
        try:
            # Ignore the initial "Connected" message from the server
            if message == "Connected":
                return

            data = json.loads(message)

            enable = data.get('enable', 0)
            if enable == 1:
                # Extract roll, pitch, yaw
                roll = data.get('roll', 0.0)
                pitch = data.get('pitch', 0.0)
            else:
                roll = 0.0
                pitch = 0.0

            # Create a TwistStamped message
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id

            # --- Conversion Logic ---
            # Pitch (rotation around Y-axis) controls forward/backward speed (linear.x)
            # Positive pitch (nose down) -> positive linear.x (forward)
            twist_msg.twist.linear.x = pitch * self.linear_x_scale

            # Roll (rotation around X-axis) controls sideways speed (linear.y)
            # In ROS, +Y is to the left. A positive roll (right side down) should move left.
            twist_msg.twist.linear.y = - roll * self.linear_y_scale

            # Publish the message
            self.publisher_.publish(twist_msg)
            self.get_logger().debug(f"Publishing: linear_x={twist_msg.twist.linear.x:.2f}, angular_z={twist_msg.twist.angular.z:.2f}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON from message: {message}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HandTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
