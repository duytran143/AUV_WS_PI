#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class GUICommandNode(Node):
    def __init__(self):
        super().__init__('gui_command_node')
        # Publisher lên topic 'gui_command'
        self.pub_ = self.create_publisher(String, 'gui_command', 10)

        # UDP socket để nhận lệnh từ GUI
        self.port = 5002
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('0.0.0.0', self.port))
            self.get_logger().info(f"Listening for GUI commands on UDP port {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP port {self.port}: {e}")
            rclpy.shutdown()
            return

        # Thread để loop nhận lệnh
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                cmd = data.decode().strip()
                if not cmd:
                    continue
                # Publish lên ROS topic
                msg = String(data=cmd)
                self.pub_.publish(msg)
                self.get_logger().info(f"Received & published GUI command: {cmd}")
            except Exception as e:
                self.get_logger().error(f"Error in recv_loop: {e}")

    def destroy_node(self):
        try:
            self.sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GUICommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
