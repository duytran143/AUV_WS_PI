#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import String  # Bạn có thể thay thế bằng message tùy chỉnh sau

class JoystickInputNode(Node):
    def __init__(self):
        super().__init__('joystick_input_node')
        # Tạo publisher cho dữ liệu joystick
        self.publisher_ = self.create_publisher(String, 'joystick_data', 10)
        
        # Cấu hình server để nhận dữ liệu điều khiển từ Windows (hoặc từ tay cầm)
        self.host = "192.168.2.2"  # Địa chỉ tĩnh của Raspberry Pi (cho server)
        self.port = 5000         # Cổng lắng nghe
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Cho phép tái sử dụng cổng
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.get_logger().info(f"Listening on {self.host}:{self.port} ...")
        except Exception as e:
            self.get_logger().error(f"Failed to bind server socket: {e}")
            rclpy.shutdown()
            return
        
        self.conn = None  # Kết nối client
        # Khởi động thread để chấp nhận kết nối và nhận dữ liệu
        self.listen_thread = threading.Thread(target=self.listen_for_connection, daemon=True)
        self.listen_thread.start()
    
    def listen_for_connection(self):
        try:
            self.conn, addr = self.server_socket.accept()
            self.get_logger().info(f"Accepted connection from {addr}")
            self.receive_loop()
        except Exception as e:
            self.get_logger().error(f"Error accepting connection: {e}")
    
    def receive_loop(self):
        while rclpy.ok():
            try:
                data = self.conn.recv(1024).decode().strip()
                if not data:
                    continue
                # Publish dữ liệu nhận được
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
                self.get_logger().info(f"Received joystick data: {data}")
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")
                break

    def destroy_node(self):
        if self.conn:
            self.conn.close()
        self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
