#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int32, Float32
import cv2
import socket
import threading
import time

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        # Latest sensor values
        self.euler = [0.0, 0.0, 0.0]
        self.depth = 0.0
        self.p1 = 0
        self.p2 = 0

        # Subscriptions
        self.create_subscription(
            Vector3Stamped, 'imu/euler', self._euler_cb, 10)
        self.create_subscription(
            Float32, 'depth',      lambda msg: setattr(self, 'depth', msg.data), 10)
        self.create_subscription(
            Int32, 'proximity1',   lambda msg: setattr(self, 'p1', msg.data), 10)
        self.create_subscription(
            Int32, 'proximity2',   lambda msg: setattr(self, 'p2', msg.data), 10)

        # UDP socket to Windows
        self.host_ip = '192.168.2.1'
        self.port    = 5001
        self.sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr    = (self.host_ip, self.port)

        # Camera flags
        self.cap_ready = False
        self.cap = None

        # Start camera opening in background
        threading.Thread(target=self._open_camera, daemon=True).start()
        # Start streaming loop
        threading.Thread(target=self._stream_loop, daemon=True).start()

    def _euler_cb(self, msg: Vector3Stamped):
        # update Euler angles [roll, pitch, yaw]
        self.euler = [
            msg.vector.x,
            msg.vector.y,
            msg.vector.z
        ]

    def _open_camera(self):
        self.get_logger().info(">> Opening camera in background...")
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # reduce resolution for faster startup
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        start = time.time()
        while not cap.isOpened() and time.time() - start < 3.0:
            time.sleep(0.05)
        if cap.isOpened():
            self.cap = cap
            self.cap_ready = True
            self.get_logger().info(">> Camera opened successfully.")
        else:
            self.get_logger().error(">> Failed to open camera after 3s!")

    def _stream_loop(self):
        while rclpy.ok():
            if not self.cap_ready:
                time.sleep(0.01)
                continue

            ret, frame = self.cap.read()
            if not ret:
                continue

            h, w = frame.shape[:2]

            # build overlay string
            overlay = (
                f"R:{self.euler[0]:6.2f}  "
                f"P:{self.euler[1]:6.2f}  "
                f"Y:{self.euler[2]:6.2f}  "
                f"P1:{self.p1:4d}  "
                f"P2:{self.p2:4d}  "
                f"D:{self.depth:6.2f}"
            )

            # draw semi‑transparent background
            (tw, th), _ = cv2.getTextSize(
                overlay, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(
                frame,
                (5, h - th - 15),
                (5 + tw + 10, h - 5),
                (0, 0, 0), cv2.FILLED
            )

            # draw overlay text
            cv2.putText(
                frame, overlay,
                (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2, cv2.LINE_AA
            )

            # attempt to show locally (if GUI available)
            try:
                cv2.imshow('Camera Stream', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception:
                pass

            # encode & send via UDP
            result, buf = cv2.imencode(
                '.jpg', frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            )
            data = buf.tobytes()
            for i in range(0, len(data), 65507):
                self.sock.sendto(data[i:i+65507], self.addr)

        # cleanup
        if self.cap:
            self.cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
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
