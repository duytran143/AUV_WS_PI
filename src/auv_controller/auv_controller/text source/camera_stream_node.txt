#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int32, Float32
import cv2
import socket
import threading
import time

# ==== CẤU HÌNH ====
HOST_IP      = '192.168.2.1'   # IP Windows client
PORT         = 5001
CHUNK_SIZE   = 65507           # bytes mỗi gói UDP
JPEG_QUALITY = 40              # chất lượng JPEG
WIDTH,HEIGHT = 960, 540        # độ phân giải
FPS          = 40

# Kích thước buffer UDP (ví dụ 4 MB)
UDP_BUFFER_SIZE = 4 * 1024 * 1024

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        # những giá trị cảm biến để overlay
        self.euler = [0.0, 0.0, 0.0]
        self.depth = 0.0
        self.p1    = 0
        self.p2    = 0

        # ROS2 subscriptions
        self.create_subscription(
            Vector3Stamped, 'imu/euler', self._euler_cb, 10)
        self.create_subscription(
            Float32, 'depth',      lambda m: setattr(self, 'depth', m.data), 10)
        self.create_subscription(
            Int32, 'proximity1',   lambda m: setattr(self, 'p1',    m.data), 10)
        self.create_subscription(
            Int32, 'proximity2',   lambda m: setattr(self, 'p2',    m.data), 10)

        # tạo socket UDP và mở rộng buffer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, UDP_BUFFER_SIZE)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_BUFFER_SIZE)
        self.addr = (HOST_IP, PORT)

        # mở camera (MJPG qua V4L2 cho tốc độ cao)
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC,      cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,         FPS)
        self.cap = cap

        # chờ camera mở
        start = time.time()
        while not self.cap.isOpened() and time.time() - start < 3.0:
            time.sleep(0.01)
        if self.cap.isOpened():
            self.get_logger().info(">> Camera opened successfully.")
        else:
            self.get_logger().error(">> Failed to open camera!")

        # buffer cho thread capture
        self.raw_frame = None
        self.raw_lock  = threading.Lock()

        # params encode JPEG
        self.enc_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

        # 1) Thread ROS executor để nhận callback
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        threading.Thread(target=executor.spin, daemon=True).start()

        # 2) Thread liên tục đọc frame
        threading.Thread(target=self._capture_loop, daemon=True).start()

        # 3) Thread stream (overlay → encode → gửi)
        threading.Thread(target=self._stream_loop, daemon=True).start()

    def _euler_cb(self, msg: Vector3Stamped):
        self.euler = [msg.vector.x, msg.vector.y, msg.vector.z]

    def _capture_loop(self):
        """Liên tục đọc frame mới nhất vào self.raw_frame."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            with self.raw_lock:
                self.raw_frame = frame

    def _stream_loop(self):
        """Overlay, encode JPEG và gửi UDP cố định FPS."""
        interval = 1.0 / FPS
        while rclpy.ok():
            t0 = time.time()

            # lấy frame mới nhất
            with self.raw_lock:
                frame = None if self.raw_frame is None else self.raw_frame.copy()

            if frame is not None:
                # vẽ overlay text
                h, w = frame.shape[:2]
                text = (
                    f"R:{self.euler[0]:6.2f}  "
                    f"P:{self.euler[1]:6.2f}  "
                    f"Y:{self.euler[2]:6.2f}  "
                    f"P1:{self.p1:4d}  "
                    f"P2:{self.p2:4d}  "
                    f"D:{self.depth:6.2f}"
                )
                (tw, th), _ = cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (5, h-th-15), (5+tw+10, h-5),
                              (0,0,0), cv2.FILLED)
                cv2.putText(frame, text, (10, h-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0,255,0), 2, cv2.LINE_AA)

                # encode JPEG
                success, buf = cv2.imencode('.jpg', frame, self.enc_params)
                if not success:
                    continue
                data = buf.tobytes()
                size = len(data)

                # gửi header + chunks
                self.sock.sendto(size.to_bytes(4, 'big'), self.addr)
                for i in range(0, size, CHUNK_SIZE):
                    self.sock.sendto(data[i:i+CHUNK_SIZE], self.addr)

            # đảm bảo loop đạt đúng FPS
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    def destroy_node(self):
        self.cap.release()
        self.sock.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
