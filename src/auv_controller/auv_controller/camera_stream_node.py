#!/usr/bin/env python3
"""
ROS 2 node – **CameraStreamNode**
Stream raw JPEG video over UDP at fixed FPS.
This variant is *video‑only*: it no longer subscribes to any sensor topics
or draws overlays.  The CPU budget on the Raspberry Pi is therefore lower
(~20‑30 % gain in most tests) and latency is improved.
"""

import rclpy
from rclpy.node import Node
import cv2
import socket
import threading
import time

# ─── CONFIGURATION ─────────────────────────────────────────────────────────────
HOST_IP      = "192.168.2.1"   # Windows client IP
PORT         = 5001            # UDP port for video
CHUNK_SIZE   = 65_507          # max UDP payload (bytes)
JPEG_QUALITY = 40              # 0‑100 – higher = better quality / larger size
WIDTH, HEIGHT = 640, 480       # capture resolution
FPS          = 30              # target frames per second

# Optional: enlarge OS UDP buffers (bytes)
UDP_BUFFER_SIZE = 4 * 1024 * 1024  # 4 MiB

# ─── NODE DEFINITION ───────────────────────────────────────────────────────────
class CameraStreamNode(Node):
    def __init__(self):
        super().__init__("camera_stream_node")

        # 1) UDP socket initialisation
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, UDP_BUFFER_SIZE)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_BUFFER_SIZE)
        self.addr = (HOST_IP, PORT)

        # 2) Open camera (MJPG over V4L2 for low CPU usage)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC,      cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,         FPS)

        # Wait max 3 s for camera to open
        start = time.time()
        while not self.cap.isOpened() and time.time() - start < 3.0:
            time.sleep(0.01)
        if self.cap.isOpened():
            self.get_logger().info("Camera opened successfully.")
        else:
            self.get_logger().error("Failed to open camera!")

        # 3) Shared buffer between capture / stream threads
        self.raw_frame = None
        self.raw_lock  = threading.Lock()

        # 4) JPEG encode parameters
        self.enc_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

        # 5) Background threads
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._stream_loop,  daemon=True).start()

    # ── CAPTURE LOOP ────────────────────────────────────────────────────────────
    def _capture_loop(self):
        """Continuously read the newest frame into *self.raw_frame*."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            with self.raw_lock:
                self.raw_frame = frame

    # ── STREAM LOOP ─────────────────────────────────────────────────────────────
    def _stream_loop(self):
        """Encode JPEG & send UDP at fixed FPS (no overlay)."""
        interval = 1.0 / FPS
        while rclpy.ok():
            t0 = time.time()

            # Copy the latest frame (avoid holding lock during encode)
            with self.raw_lock:
                frame = None if self.raw_frame is None else self.raw_frame.copy()

            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame, self.enc_params)
                if not ok:
                    continue
                data = memoryview(buf)  # zero‑copy sliceable buffer
                size = len(data)

                # Send 4‑byte header (big endian size) then data in CHUNK_SIZE blocks
                try:
                    self.sock.sendto(size.to_bytes(4, "big"), self.addr)
                    for i in range(0, size, CHUNK_SIZE):
                        self.sock.sendto(data[i:i + CHUNK_SIZE], self.addr)
                except Exception as e:
                    self.get_logger().warn(f"UDP send error: {e}")

            # Maintain target FPS
            dt = time.time() - t0
            if dt < interval:
                time.sleep(interval - dt)

    # ── CLEAN‑UP ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            if self.cap.isOpened():
                self.cap.release()
            self.sock.close()
        finally:
            super().destroy_node()

# ─── MAIN ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)  # keep ROS alive for rclpy.ok() in loops
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
