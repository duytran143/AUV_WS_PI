#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import socket
import threading
import sys
import time

RECONNECT_DELAY = 1.0  # seconds
alphaX  = -15.0
alphaY  = -15.0
alphaVz = -10
alphaD  = 0.1

class JoystickInputNode(Node):
    def __init__(self):
        super().__init__('joystick_input_V2_node')

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.sp_pub  = self.create_publisher(Vector3Stamped, 'setpoints', 10)
        self.depth_pub = self.create_publisher(Float32, 'depth_sp', 10)
        self.depth_sp = 0.0

        self.host = '0.0.0.0'
        self.port = 5000
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        except Exception as e:
            self.get_logger().error(f"Failed to bind joystick socket: {e}")
            rclpy.shutdown()
            return

        threading.Thread(target=self.accept_loop, daemon=True).start()

    def accept_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                self.get_logger().info(f"Joystick connected from {addr}")
                self.receive_loop(conn)
            except Exception as e:
                self.get_logger().error(f"Error accepting connection: {e}. Retrying...")
                time.sleep(RECONNECT_DELAY)

    def receive_loop(self, conn):
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
                try:
                    axes = [float(v) for v in parts[:8]]
                except ValueError:
                    continue

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

                # === Compute and publish setpoints ===
                Fy_sp = alphaY  * axes[0]
                Fx_sp = alphaX  * axes[1]
                Vz_sp = alphaVz * axes[2]

                self.depth_sp += alphaD * axes[3]
                self.depth_sp = min(max(self.depth_sp, 0.0), 10.0)

                sp_msg = Vector3Stamped()
                sp_msg.header.stamp = self.get_clock().now().to_msg()
                sp_msg.header.frame_id = 'setpoint'
                sp_msg.vector.x = Fx_sp
                sp_msg.vector.y = Fy_sp
                sp_msg.vector.z = Vz_sp
                self.sp_pub.publish(sp_msg)

                self.depth_pub.publish(Float32(data=self.depth_sp))

                # === Single-line log ===
                log_line = (
                    f"Fx: {Fx_sp:6.2f}  Fy: {Fy_sp:6.2f}  "
                    f"Vz: {Vz_sp:6.2f}  Depth_sp: {self.depth_sp:5.2f}"
                )
                sys.stdout.write('\r' + log_line)
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
