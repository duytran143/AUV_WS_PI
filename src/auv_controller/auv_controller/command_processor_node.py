#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandProcessorNode(Node):
    def __init__(self):
        super().__init__('command_processor_node')
        
        # Subscribe đến topic 'joystick_data'
        self.joystick_subscription = self.create_subscription(
            String,
            'joystick_data',
            self.joystick_callback,
            10
        )
        # Subscribe đến topic 'sensor_data'
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )
        
        # Lưu trữ dữ liệu nhận được, có thể xử lý sau này
        self.latest_joystick_data = None
        self.latest_sensor_data = None

    def joystick_callback(self, msg: String):
        self.latest_joystick_data = msg.data
        # Log dữ liệu joystick ra terminal
        self.get_logger().info(f"Joystick: {msg.data}")

    def sensor_callback(self, msg: String):
        self.latest_sensor_data = msg.data
        # Log dữ liệu sensor ra terminal
        self.get_logger().info(f"Sensor {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
