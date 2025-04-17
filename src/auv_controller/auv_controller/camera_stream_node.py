#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import socket
import numpy as np
import threading
import re
import time

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        # Subscribe sensor_data như trước
        self.create_subscription(String, 'sensor_data', self.sensor_callback, 10)
        self.roll = self.pitch = self.yaw = 0.0

        # UDP socket
        self.host_ip = '192.168.2.1'
        self.port    = 5001
        self.sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr   = (self.host_ip, self.port)

        # Flags và capture object
        self.cap_ready = False
        self.cap = None

        # Thread mở camera
        threading.Thread(target=self._open_camera, daemon=True).start()
        # Thread stream video
        threading.Thread(target=self._stream_loop, daemon=True).start()

    def _open_camera(self):
        self.get_logger().info(">> Starting camera open in background...")
        # Sử dụng backend V4L2 trực tiếp để tránh auto‑detect
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # Giữ full resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        # Chờ cho camera sẵn sàng
        start = time.time()
        while not cap.isOpened() and time.time() - start < 3.0:
            time.sleep(0.05)
        if cap.isOpened():
            self.cap = cap
            self.cap_ready = True
            self.get_logger().info(">> Camera opened successfully.")
        else:
            self.get_logger().error(">> Failed to open camera after 3s!")

    def sensor_callback(self, msg: String):
        # parse Euler:
        try:
            euler = msg.data.split("Euler:(")[1].split(")")[0]
            r, p, y = [float(s.strip()) for s in euler.split(",")]
            self.roll, self.pitch, self.yaw = r, p, y
        except Exception:
            pass

    def _stream_loop(self):
        # Chỉ stream khi camera sẵn
        while rclpy.ok():
            if not self.cap_ready:
                time.sleep(0.01)
                continue

            ret, frame = self.cap.read()
            if not ret:
                continue

            # Overlay full‑resolution
            for i, txt in enumerate((
                f"Roll : {self.roll:.2f}",
                f"Pitch: {self.pitch:.2f}",
                f"Yaw  : {self.yaw:.2f}"
            )):
                cv2.putText(frame, txt, (10, 30 + i*30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            # Encode & split
            result, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            data = enc.tobytes()
            for i in range(0, len(data), 65507):
                self.sock.sendto(data[i:i+65507], self.addr)

        # cleanup
        if self.cap:
            self.cap.release()
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
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
