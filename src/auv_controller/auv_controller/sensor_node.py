#!/usr/bin/env python3
"""
ROS 2 node – **SensorNode**
Reads IMU/proximity/depth from serial, publishes on ROS topics, logs to console,
and additionally streams Euler angles, P1, P2, and depth via UDP.

Original functionality (serial → ROS topics/log) is preserved; UDP sending
is appended in each timer callback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int32, Float32
import serial
import time
import sys
from collections import deque
import socket

# UDP configuration
CLIENT_IP   = "192.168.2.1"   # Windows client IP
SENSOR_PORT = 5003            # UDP port for sensor data

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # Publishers
        self.gyro_pub   = self.create_publisher(Vector3Stamped, 'gyro',      10)
        self.accel_pub  = self.create_publisher(Vector3Stamped, 'accel',     10)
        self.euler_pub  = self.create_publisher(Vector3Stamped, 'imu/euler', 10)
        self.p1_pub     = self.create_publisher(Int32,           'proximity1',10)
        self.p2_pub     = self.create_publisher(Int32,           'proximity2',10)
        self.depth_pub  = self.create_publisher(Float32,         'depth',     10)

        # Depth moving-average buffer
        self.declare_parameter('depth_filter_size', 3)
        size = self.get_parameter('depth_filter_size').value
        if size < 1:
            self.get_logger().warn('depth_filter_size should be >=1; defaulting to 3')
            size = 3
        self.depth_buffer = deque(maxlen=size)

        # Serial port setup
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
            time.sleep(0.5)
            self.get_logger().info("Serial opened at 115200 baud.")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            rclpy.shutdown()
            return

        # UDP socket setup
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = (CLIENT_IP, SENSOR_PORT)

        # Timer (~100 Hz)
        self.create_timer(0.01, self.timer_callback)
        # Suppress excessive log level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

    def timer_callback(self):
        # Read one line from serial
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        parts = line.split()
        if len(parts) != 12:
            self.get_logger().warn(f"Expected 12 fields, got {len(parts)}: '{line}'")
            return

        # Parse raw values
        gx, gy, gz       = map(float, parts[0:3])
        ax, ay, az       = map(float, parts[3:6])
        roll, pitch, yaw = map(float, parts[6:9])
        p1, p2           = map(int,   parts[9:11])
        depth     = float(parts[11])

        now = self.get_clock().now().to_msg()

        # --- Publish on ROS topics ---
        gyro_msg = Vector3Stamped();  gyro_msg.header.stamp = now; gyro_msg.header.frame_id = 'imu_link'
        gyro_msg.vector.x, gyro_msg.vector.y, gyro_msg.vector.z = gx, gy, gz
        self.gyro_pub.publish(gyro_msg)

        accel_msg = Vector3Stamped(); accel_msg.header.stamp = now; accel_msg.header.frame_id = 'imu_link'
        accel_msg.vector.x, accel_msg.vector.y, accel_msg.vector.z = ax, ay, az
        self.accel_pub.publish(accel_msg)

        euler_msg = Vector3Stamped(); euler_msg.header.stamp = now; euler_msg.header.frame_id = 'imu_link'
        euler_msg.vector.x, euler_msg.vector.y, euler_msg.vector.z = roll, pitch, yaw
        self.euler_pub.publish(euler_msg)

        self.p1_pub.publish(Int32(data=p1))
        self.p2_pub.publish(Int32(data=p2))
        self.depth_pub.publish(Float32(data=depth))

        # --- Console log ---
        log_line = (
            f"Gyro: {gx:7.2f} {gy:7.2f} {gz:7.2f} | "
            f"Acc : {ax:7.2f} {ay:7.2f} {az:7.2f} | "
            f"Ang : {roll:7.2f} {pitch:7.2f} {yaw:7.2f} | "
            f"P1:{p1:3d} P2:{p2:3d} | "
            f"D :{depth:4.2f}"
        )
        sys.stdout.write('\r' + log_line)
        sys.stdout.flush()

        # --- UDP send ---
        payload = f"{roll:.2f} {pitch:.2f} {yaw:.2f} {p1:d} {p2:d} {depth:.2f}"
        try:
            self.udp_sock.sendto(payload.encode(), self.udp_addr)
        except Exception as e:
            self.get_logger().warn(f"UDP send error: {e}")

    def destroy_node(self):
        # Close serial and UDP socket
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        if hasattr(self, 'udp_sock'):
            self.udp_sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
