import socket
import signal
import sys
import time

# 🔹 Cấu hình server nhận dữ liệu từ Windows
HOST = "192.168.2.2"
PORT = 5000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Cho phép tái sử dụng cổng nếu cổng đã được sử dụng trước đó
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # Thêm dòng này

conn = None  # Đảm bảo conn được khai báo trước khi sử dụng

def graceful_exit(signal, frame):
    print("\n❌ Tiến trình bị dừng bởi tín hiệu. Đóng kết nối...")
    if conn:
        conn.close()
    server_socket.close()
    print("✅ Đã đóng kết nối và socket.")
    sys.exit(0)  # Dừng tiến trình một cách sạch sẽ

# Đăng ký tín hiệu Ctrl + C (SIGINT) hoặc tín hiệu kết thúc (SIGTERM)
signal.signal(signal.SIGINT, graceful_exit)
signal.signal(signal.SIGTERM, graceful_exit)

try:
    # Gắn kết socket với địa chỉ và cổng
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    print(f"🔹 Đang lắng nghe trên cổng {PORT}...")
    conn, addr = server_socket.accept()
    print(f"✅ Kết nối từ {addr}")

    # Nhận dữ liệu từ máy tính Windows và xử lý
    while True:
        data = conn.recv(1024).decode().strip()
        if not data:
            continue

        # Tách dữ liệu từ joystick và các nút nhấn
        try:
            left_x, left_y, right_x, right_y, left_z, right_z, hat_x, hat_y, buttons = data.split()
            left_x, left_y, right_x, right_y, left_z, right_z = map(float, [left_x, left_y, right_x, right_y, left_z, right_z])
            hat_x, hat_y = map(int, [hat_x, hat_y])
            button_4_pressed = int(buttons[4])  # Nút 4 (bắt đầu từ 0)
            button_5_pressed = int(buttons[5])  # Nút 5 (bắt đầu từ 0)
        except ValueError:
            continue

        # Debug: In ra giá trị nhận được từ joystick
        print(f"{left_x}, {left_y}, {right_x}, {right_y}, {left_z}, {right_z}, {hat_x}, {hat_y}, {buttons}")

except Exception as e:
    print(f"❌ Lỗi xảy ra: {e}")

finally:
    time.sleep(1)  # Đợi 1 giây để hệ điều hành giải phóng cổng
    if conn:
        conn.close()
    server_socket.close()
    print("✅ Đã đóng kết nối và socket.")