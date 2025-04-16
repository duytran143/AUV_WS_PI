#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # Publisher để gửi dữ liệu cảm biến lên topic 'sensor_data'
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        
        # Mở cổng serial (ví dụ: /dev/ttyUSB0) với tốc độ 115200 baud, timeout 1 giây.
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(0.5)  # Cho Serial ổn định
            self.get_logger().info("Serial port /dev/ttyUSB0 opened at 115200 baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return
        
        # Tạo timer gọi hàm timer_callback mỗi 0.1 giây
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Đọc một dòng từ serial (kết thúc bằng newline)
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return  # Nếu không có dữ liệu, bỏ qua

            # Giả sử dữ liệu có dạng:
            # "gyroX gyroY gyroZ roll pitch yaw prox1 prox2 depth"
            fields = line.split()
            if len(fields) != 9:
                self.get_logger().warn(f"Invalid format: {line}")
                return

            try:
                # Ép kiểu các trường ra float
                gyroX = float(fields[0])
                gyroY = float(fields[1])
                gyroZ = float(fields[2])
                roll  = float(fields[3])
                pitch = float(fields[4])
                yaw   = float(fields[5])
                prox1 = float(fields[6])
                prox2 = float(fields[7])
                depth = float(fields[8])
            except ValueError as e:
                self.get_logger().warn(f"Parse error: {e} for line: {line}")
                return

            # Định dạng dữ liệu thành chuỗi để publish (có thể sau này chuyển thành custom message)
            sensor_str = (f"Gyro:({gyroX:.2f}, {gyroY:.2f}, {gyroZ:.2f}) "
                          f"Euler:({roll:.2f}, {pitch:.2f}, {yaw:.2f}) "
                          f"Proxi:({prox1:.0f} mm, {prox2:.0f} mm) "
                          f"Depth:{depth:.2f}")
            msg = String()
            msg.data = sensor_str

            # Publish lên topic 'sensor_data'
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published sensor data: {sensor_str}")
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    def destroy_node(self):
        # Đóng cổng serial khi node bị dừng
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
