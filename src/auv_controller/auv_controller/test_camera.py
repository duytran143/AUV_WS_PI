#!/usr/bin/env python3
"""
camera_streamV2.py – Stream Raspberry Pi camera over UDP (compatible with camera_receiveV2.py)
────────────────────────────────────────────────────────
► How it works
  •  Sets the RAW sensor node (/dev/video0) to the requested size (BA81)
  •  Configures the ISP Capture node (default /dev/video14) to YUYV
  •  Captures from the ISP node with OpenCV, JPEG‑encodes each frame
  •  Sends one or more UDP datagrams per frame:
      – Packet 0: 4‑byte big‑endian total_length + first chunk
      – Next packets: remaining JPEG bytes (≤65 507 B each)

► Usage
    python3 camera_streamV2.py <DEST_IP> [DEVICE] [WIDTH] [HEIGHT]
    # Example:  python3 camera_streamV2.py 192.168.2.1 /dev/video14 640 480

► Prerequisites
    sudo apt install v4l-utils python3-opencv
    Add cma=128M (or higher) to /boot/firmware/cmdline.txt & reboot.
"""
import cv2
import sys
import socket
import struct
import time
import subprocess
from typing import Tuple

# ---------- Default parameters ----------
DEST_IP      = sys.argv[1] if len(sys.argv) > 1 else '192.168.2.3'
DEVICE       = sys.argv[2] if len(sys.argv) > 2 else '/dev/video14'  # ISP Capture0
WIDTH        = int(sys.argv[3]) if len(sys.argv) > 3 else 640
HEIGHT       = int(sys.argv[4]) if len(sys.argv) > 4 else 480
FPS          = 30
VIDEO_PORT   = 5001
MAX_DGRAM    = 65507
JPEG_QUALITY = 80
# ----------------------------------------

def pre_configure_formats(width: int, height: int, isp_dev: str) -> None:
    """Set RAW (video0) and ISP node to matched formats via v4l2-ctl."""
    try:
        subprocess.run([
            'v4l2-ctl', '-d', '/dev/video0',
            '--set-fmt-video', f'width={width},height={height},pixelformat=BA81'
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        subprocess.run([
            'v4l2-ctl', '-d', isp_dev,
            '--set-fmt-video', f'width={width},height={height},pixelformat=YUYV'
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"[!] Could not set formats with v4l2-ctl ({e}); continuing anyway…")

def open_camera(dev: str, size: Tuple[int, int]) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"[!] Cannot open camera device {dev}")
    w, h = size
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # fewer DMA buffers → lower CMA need
    return cap

def main():
    pre_configure_formats(WIDTH, HEIGHT, DEVICE)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (DEST_IP, VIDEO_PORT)
    print(f"[+] Streaming from {DEVICE} {WIDTH}x{HEIGHT}@{FPS} to udp://{DEST_IP}:{VIDEO_PORT}")
    print("[+] Ctrl‑C to quit")

    cap = open_camera(DEVICE, (WIDTH, HEIGHT))
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    t0 = time.time(); frame_cnt = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[!] Frame grab failed – retrying")
                time.sleep(0.05)
                continue

            ok, enc = cv2.imencode('.jpg', frame, encode_param)
            if not ok:
                print("[!] JPEG encode failed – skipping frame")
                continue
            buf = enc.tobytes(); frame_len = len(buf)
            header = struct.pack('>I', frame_len)

            # Send first packet with header
            chunk = buf[:MAX_DGRAM - 4]
            sock.sendto(header + chunk, dest)
            # Remaining packets
            offset = len(chunk)
            while offset < frame_len:
                sock.sendto(buf[offset:offset + MAX_DGRAM], dest)
                offset += MAX_DGRAM

            frame_cnt += 1
            time.sleep(max(0, (1.0 / FPS) - 0.002))
    except KeyboardInterrupt:
        pass
    finally:
        cap.release(); sock.close()
        dur = time.time() - t0
        if dur:
            print(f"[i] Sent {frame_cnt} frames – avg {frame_cnt/dur:.2f} fps")

if __name__ == '__main__':
    main()
