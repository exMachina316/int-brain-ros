#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import speech_recognition as sr
import threading

class VoiceTeleopNode(Node):
    def __init__(self):
        super().__init__('voice_teleop_node')
        
        # Publishing TwistStamped to accommodate the velocity smoother
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Configurable speed parameters (m/s and rad/s)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        
        # State variables for the keep-alive publisher
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Publish loop at 10Hz to prevent ros2_control timeout
        self.create_timer(0.1, self.publish_cmd_vel)
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise once on startup
        with self.microphone as source:
            self.get_logger().info("Adjusting for ambient noise... Please wait.")
            self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
            self.get_logger().info("Ready! Start giving commands: 'forward', 'backward', 'left', 'right', 'stop'.")

        # Start listening in a background thread to avoid blocking ROS 2 execution
        self.listen_thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.listen_thread.start()

    def audio_loop(self):
        with self.microphone as source:
            while rclpy.ok():
                try:
                    # phrase_time_limit cuts off long rambling, timeout allows loop to breathe
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=3.0)
                    self.process_audio(audio)
                except sr.WaitTimeoutError:
                    # Nobody spoke in the last second. Loop silently.
                    continue
                except Exception as e:
                    self.get_logger().error(f"Audio loop error: {e}")

    def process_audio(self, audio):
        try:
            text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f"Heard: '{text}'")
            
            lin_speed = self.get_parameter('linear_speed').value
            ang_speed = self.get_parameter('angular_speed').value

            # Keyword matching (allows for phrases like "go forward please")
            if "forward" in text:
                self.target_linear = lin_speed
                self.target_angular = 0.0
            elif "backward" in text:
                self.target_linear = -lin_speed
                self.target_angular = 0.0
            elif "left" in text:
                self.target_linear = 0.0
                self.target_angular = ang_speed
            elif "right" in text:
                self.target_linear = 0.0
                self.target_angular = -ang_speed
            elif "stop" in text:
                self.target_linear = 0.0
                self.target_angular = 0.0
            else:
                self.get_logger().warn("Unrecognized command.")
                
        except sr.UnknownValueError:
            # Speech was unintelligible, ignore silently
            pass
        except sr.RequestError as e:
            self.get_logger().error(f"Google Speech API error: {e}")

    def publish_cmd_vel(self):
        msg = TwistStamped()
        
        # Inject current timestamp for velocity_smoother
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = self.target_linear
        msg.twist.angular.z = self.target_angular
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down voice teleop...")
    finally:
        # Failsafe: Command a hard stop on exit
        node.target_linear = 0.0
        node.target_angular = 0.0
        node.publish_cmd_vel()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()