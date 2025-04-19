#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int32, Float32
import serial
import time
import sys

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # Publishers
        self.gyro_pub   = self.create_publisher(Vector3Stamped, 'gyro', 10)
        self.accel_pub  = self.create_publisher(Vector3Stamped, 'accel', 10)
        self.euler_pub  = self.create_publisher(Vector3Stamped, 'imu/euler', 10)
        self.p1_pub     = self.create_publisher(Int32, 'proximity1', 10)
        self.p2_pub     = self.create_publisher(Int32, 'proximity2', 10)
        self.depth_pub  = self.create_publisher(Float32, 'depth', 10)

        # Serial port setup
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
            time.sleep(0.5)
            self.get_logger().info("Serial opened at 115200 baud.")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            rclpy.shutdown()
            return

        # Timer (~20 Hz)
        self.create_timer(0.01, self.timer_callback)
        # Set logger level to WARN to suppress info logs during loop
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        parts = line.split()
        if len(parts) != 12:
            self.get_logger().warn(f"Expected 12 fields, got {len(parts)}: '{line}'")
            return

        # Parse values
        gx, gy, gz       = map(float, parts[0:3])
        ax, ay, az       = map(float, parts[3:6])
        roll, pitch, yaw = map(float, parts[6:9])
        p1, p2           = map(int,   parts[9:11])
        depth            = float(parts[11])

        now = self.get_clock().now().to_msg()

        # Publish gyro
        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp = now
        gyro_msg.header.frame_id = 'imu_link'
        gyro_msg.vector.x = gx
        gyro_msg.vector.y = gy
        gyro_msg.vector.z = gz
        self.gyro_pub.publish(gyro_msg)

        # Publish accel
        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = now
        accel_msg.header.frame_id = 'imu_link'
        accel_msg.vector.x = ax
        accel_msg.vector.y = ay
        accel_msg.vector.z = az
        self.accel_pub.publish(accel_msg)

        # Publish euler
        euler_msg = Vector3Stamped()
        euler_msg.header.stamp = now
        euler_msg.header.frame_id = 'imu_link'
        euler_msg.vector.x = roll
        euler_msg.vector.y = pitch
        euler_msg.vector.z = yaw
        self.euler_pub.publish(euler_msg)

        # Publish proximity
        self.p1_pub.publish(Int32(data=p1))
        self.p2_pub.publish(Int32(data=p2))

        # Publish depth
        self.depth_pub.publish(Float32(data=depth))

        # Single-line console output with fixed-width fields
        log_line = (
            f"Gyro: {gx:7.2f} {gy:7.2f} {gz:7.2f} | "
            f"Acc : {ax:7.2f} {ay:7.2f} {az:7.2f} | "
            f"Ang : {roll:7.2f} {pitch:7.2f} {yaw:7.2f} | "
            f"P1:{p1:3d} P2:{p2:3d} | "
            f"D :{depth:4.2f}"
        )
        sys.stdout.write('\r' + log_line)
        sys.stdout.flush()

    def destroy_node(self):
        if hasattr(self, 'ser'):
            self.ser.close()
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
