#!/usr/bin/env python3
import serial
import time

def main():
    # Thay đổi cổng thiết bị nếu cần, ví dụ ttyUSB1, v.v.
    port = "/dev/ttyUSB0"
    baud_rate = 115200
    
    # Mở cổng serial
    ser = serial.Serial(port, baud_rate, timeout=1)
    time.sleep(0.5)  # Chờ một chút cho cổng ổn định

    print(f"Listening on {port} at {baud_rate} baud...")

    try:
        while True:
            # Đọc một dòng (kết thúc bằng \n) từ vi điều khiển
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue  # Nếu chuỗi trống, bỏ qua

            # line có dạng: -0.75 -29.31 5.94 0 0 1
            # Ta tách thành mảng fields
            fields = line.split()

            # Kiểm tra số trường dữ liệu, sau đó parse
            if len(fields) == 9:
                try:
                    gyroX     = float(fields[0])
                    gyroY     = float(fields[1])
                    gyroZ     = float(fields[2])
                    roll     = float(fields[3])
                    pitch    = float(fields[4])
                    yaw      = float(fields[5])
                    dist1    = float(fields[6])  # or int, tùy bạn
                    dist2    = float(fields[7])  # or int
                    depth    = float(fields[8])  # or int

                    # In ra màn hình (hoặc xử lý tùy ý)
                    print(f"{gyroX:.2f} {gyroY:.2f} {gyroZ:.2f}"
                          f"{roll:.2f} {pitch:.2f}, {yaw:.2f}, "
                          f"{dist1}, {dist2}, {depth}")
                except ValueError:
                    # Trong trường hợp parse float/int lỗi
                    print(f"Cannot parse: {line}")
            else:
                print(f"Invalid format: {line}")

    except KeyboardInterrupt:
        print("User interrupted. Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()