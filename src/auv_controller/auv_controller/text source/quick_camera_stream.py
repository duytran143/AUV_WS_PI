#!/usr/bin/env python3
"""
quick_camera_stream.py – Test stream from Raspberry Pi camera on Ubuntu (libcamera stack)
──────────────────────────────────────────────────────────────────────────
* Pure Python + OpenCV, **no ROS 2**.
* Reads YUYV frames from ISP node (default `/dev/video14`), encodes JPEG, sends over UDP.
* Compatible with your existing `camera_receiveV2.py` on Windows.

Usage
-----
    python3 quick_camera_stream.py <DEST_IP> [DEVICE] [WIDTH] [HEIGHT]

    # ví dụ 640×480, node video14
    python3 quick_camera_stream.py 192.168.2.1 /dev/video14 640 480

Prerequisites
-------------
    sudo apt install python3-opencv v4l-utils
    # Đảm bảo sensor hỗ trợ độ phân giải đã chọn.
"""
import cv2
import socket
import struct
import sys
import time

# ---------- CLI parameters ----------
DEST_IP = sys.argv[1] if len(sys.argv) > 1 else '192.168.2.1'
DEVICE  = sys.argv[2] if len(sys.argv) > 2 else '/dev/video14'
WIDTH   = int(sys.argv[3]) if len(sys.argv) > 3 else 640
HEIGHT  = int(sys.argv[4]) if len(sys.argv) > 4 else 480
FPS     = 30
PORT    = 5001
JPEG_QUALITY = 40
MAX_DGRAM    = 65507
# ------------------------------------

# Open camera
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    sys.exit(f"[!] Cannot open {DEVICE}")
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = (DEST_IP, PORT)
print(f"[+] Streaming {WIDTH}x{HEIGHT}@{FPS} from {DEVICE} to udp://{DEST_IP}:{PORT}")
print("[+] Ctrl‑C to stop")

start, frames = time.time(), 0
try:
    while True:
        ret, yuv = cap.read()
        if not ret:
            print('[!] Frame grab failed'); continue
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
        ok, jpg = cv2.imencode('.jpg', bgr, encode_param)
        if not ok:
            continue
        data = jpg.tobytes()
        size = len(data)
        # header + first chunk
        sock.sendto(struct.pack('>I', size) + data[:MAX_DGRAM-4], addr)
        # remaining chunks
        for off in range(MAX_DGRAM-4, size, MAX_DGRAM):
            sock.sendto(data[off:off+MAX_DGRAM], addr)
        frames += 1
        time.sleep(max(0, 1.0/FPS - 0.002))
except KeyboardInterrupt:
    pass
finally:
    cap.release(); sock.close()
    dur = time.time() - start
    if dur:
        print(f"[i] Sent {frames} frames – avg {frames/dur:.2f} fps")
