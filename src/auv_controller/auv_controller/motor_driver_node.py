#!/usr/bin/env python3
"""
ROS2 node that subscribes to /PWM (Float32MultiArray) and sends
8 PWM values over serial (USB) to /dev/ttyACM0 as a space-separated string.
Includes emergency stop: on shutdown (Ctrl+C), sends eight '1500' commands.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import sys
import atexit

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        # Serial port setup
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Opened serial port /dev/ttyACM0 at 115200 baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            sys.exit(1)

        # Subscribe to PWM topic
        self.create_subscription(
            Float32MultiArray,
            '/PWM',
            self.pwm_callback,
            10
        )

        # Ensure emergency stop on exit
        atexit.register(self.emergency_stop)

    def pwm_callback(self, msg: Float32MultiArray):
        pwms = msg.data
        if len(pwms) != 8:
            self.get_logger().warn(f'Received PWM array of length {len(pwms)}, expected 8')
        pwm_str = ' '.join(str(int(p)) for p in pwms)
        try:
            self.ser.write((pwm_str + '\n').encode('utf-8'))
            self.get_logger().debug(f'Sent to serial: {pwm_str}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error writing to serial: {e}')

    def emergency_stop(self):
        # Send eight '1500' PWM to stop all motors
        stop_str = ' '.join(['1500'] * 8)
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write((stop_str + '\n').encode('utf-8'))
                self.get_logger().warn(f'Emergency stop sent: {stop_str}')
        except Exception:
            pass

    def destroy_node(self):
        # Emergency stop before closing
        self.emergency_stop()
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                self.get_logger().info('Closed serial port')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
