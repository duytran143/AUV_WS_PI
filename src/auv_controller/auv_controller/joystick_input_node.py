#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import threading
import sys
import time

RECONNECT_DELAY = 1.0  # seconds

class JoystickInputNode(Node):
    def __init__(self):
        super().__init__('joystick_input_node')

        # Publisher Joy message on /joy
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Setup TCP server for joystick input
        self.host = '0.0.0.0'
        self.port = 5000
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            # Suppress info logs to avoid line breaks during console updates
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
            self.get_logger().info(f"Listening for joystick on {self.host}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind joystick socket: {e}")
            rclpy.shutdown()
            return

        # Start accept loop in background
        threading.Thread(target=self.accept_loop, daemon=True).start()

    def accept_loop(self):
        """
        Continuously accept connections. If a connection drops or fails, retry after delay.
        """
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                self.get_logger().info(f"Joystick connected from {addr}")
                self.receive_loop(conn)
            except Exception as e:
                self.get_logger().error(f"Error accepting connection: {e}. Retrying in {RECONNECT_DELAY}s...")
                time.sleep(RECONNECT_DELAY)

    def receive_loop(self, conn):
        """
        Receive data until connection is lost.
        On exception or disconnect, return to accept_loop.
        """
        with conn:
            while rclpy.ok():
                try:
                    data = conn.recv(1024).decode().strip()
                    if not data:
                        continue
                except Exception as e:
                    self.get_logger().warn(f"Receive failed: {e}. Waiting for new connection.")
                    return

                parts = data.split()
                if len(parts) < 8:
                    continue

                # Parse axes (first 8 values)
                try:
                    axes = [float(v) for v in parts[:8]]
                except ValueError:
                    continue

                # Parse buttons: support concatenated or space-separated
                buttons = []
                if len(parts) == 9:
                    raw = parts[8]
                    for c in raw:
                        if c in ('0', '1'):
                            buttons.append(int(c))
                else:
                    try:
                        buttons = [int(b) for b in parts[8:]]
                    except ValueError:
                        continue

                # Publish Joy message
                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = 'joystick'
                joy_msg.axes = axes
                joy_msg.buttons = buttons
                self.joy_pub.publish(joy_msg)

                # Single-line console log with fixed-width fields
                line = (
                    f"Axes: {axes[0]:6.2f} {axes[1]:6.2f} | "
                    f"{axes[2]:6.2f} {axes[3]:6.2f} | "
                    f"Tri: {axes[4]:6.2f} {axes[5]:6.2f} | "
                    f"Hat: {int(axes[6]):2d},{int(axes[7]):2d} | "
                    f"Btns: {' '.join(str(b) for b in buttons)}"
                )
                sys.stdout.write('\r' + line)
                sys.stdout.flush()

    def destroy_node(self):
        try:
            self.server_socket.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
