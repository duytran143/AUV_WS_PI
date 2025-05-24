import os
import math
import serial
import socket
import glob

# 🔹 Tự động tìm cổng STM32 USB
def find_stm32_port():
    ports = glob.glob('/dev/serial/by-id/usb-STMicroelectronics*')
    return ports[0] if ports else None

USB_PORT = find_stm32_port()
BAUD_RATE = 115200

if USB_PORT:
    try:
        ser = serial.Serial(USB_PORT, BAUD_RATE, timeout=1)
        print(f"✅ Đã kết nối với {USB_PORT} ở baud rate {BAUD_RATE}")
    except serial.SerialException:
        print(f"❌ Không thể mở {USB_PORT}. Kiểm tra kết nối USB!")
        exit(1)
else:
    print("❌ Không tìm thấy thiết bị STM32! Kiểm tra kết nối USB.")
    exit(1)

# 🔹 Cấu hình server nhận dữ liệu từ Windows
HOST = "0.0.0.0"
PORT = 5000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"🔹 Đang lắng nghe trên cổng {PORT}...")
conn, addr = server_socket.accept()
print(f"✅ Kết nối từ {addr}")

# Hàm tính PWM với tham số hướng, hệ số làm chậm, và tùy chỉnh ngưỡng min/max
def calculate_pwm(dir, speed_scale, slowdown_factor, min_pwm, max_pwm):
    PWM_CENTER = 1500
    PWM_RANGE = 500
    pwm_value = int(PWM_CENTER + (-1 if dir else 1) * PWM_RANGE * speed_scale)

    # Ràng buộc giá trị PWM trong khoảng từ min_pwm đến max_pwm
    pwm_value = max(min_pwm, min(max_pwm, pwm_value))

    # Thêm yếu tố làm chậm vào giá trị PWM
    return pwm_value

# Hàm điều chỉnh PWM từ từ đến giá trị mục tiêu
def adjust_pwm_to_target(current_pwm, target_pwm, slowdown_factor):
    if current_pwm < target_pwm:
        return current_pwm + slowdown_factor
    elif current_pwm > target_pwm:
        return current_pwm - slowdown_factor
    return current_pwm

# Hàm để thiết lập min/max PWM cho động cơ
def set_pwm_limits(target_pwm, min_pwm, max_pwm):
    return max(min_pwm, min(max_pwm, target_pwm))

# Giá trị min và max PWM có thể tùy chỉnh
min_pwm = 1100
max_pwm = 1900
min_pwm_Vfast = 1600
max_pwm_Vfast = 2000
min_pwm_Vslow = 1550
max_pwm_Vslow = 1900

# Giá trị min và max PWM cho điều khiển yaw
min_pwm_yaw = 1200
max_pwm_yaw = 1800

# Tùy chỉnh sự chênh lệch giữa 2 động cơ dọc
motor_diff = -100  # Mức độ chênh lệch PWM giữa động cơ 5,6 và 7,8
motor_diff_yaw = 100

# Hệ số làm chậm cho động cơ ngang, dọc và yaw
slowdown_factor = 50  # Hệ số làm chậm cho các động cơ ngang
slowdown_factor_vertical = 20  # Hệ số làm chậm riêng cho động cơ dọc
yaw_slowdown_factor = 20  # Hệ số làm chậm riêng cho yaw

# Biến kiểm tra trạng thái nút nhấn 4 và 5 (yaw control)
button_4_pressed = False
button_5_pressed = False

# Biến lưu trữ trạng thái PWM hiện tại của các động cơ
current_pwm_motor_1 = 1500
current_pwm_motor_2 = 1500
current_pwm_motor_3 = 1500
current_pwm_motor_4 = 1500
current_pwm_motor_5 = 1530
current_pwm_motor_6 = 1540
current_pwm_motor_7 = 1550
current_pwm_motor_8 = 1550

# Nhận dữ liệu từ máy tính Windows và xử lý
while True:
    data = conn.recv(1024).decode().strip()
    if not data:
        continue

    # Tách dữ liệu từ joystick và nút nhấn yaw
    try:
        left_x, left_y, right_x, right_y, left_z, right_z, hat_x, hat_y, buttons = data.split()
        left_x, left_y, right_x, right_y, left_z, right_z = map(float, [left_x, left_y, right_x, right_y, left_z, right_z])
        hat_x, hat_y = map(int, [hat_x, hat_y])
        button_4_pressed = int(buttons[4])  # Nút 4 (bắt đầu từ 0)
        button_5_pressed = int(buttons[5])  # Nút 5 (bắt đầu từ 0)
    except ValueError:
        continue

    # Tính toán PWM cho 4 động cơ ngang theo trục X của joystick trái
    if left_x < 0:  # Khi x > 0
        # target_pwm_1 = calculate_pwm(0, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 1
        target_pwm_2 = calculate_pwm(0, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 2
        # target_pwm_3 = calculate_pwm(1, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 3
        target_pwm_4 = calculate_pwm(1, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 4
    elif left_x > 0:  # Khi x < 0
        # target_pwm_1 = calculate_pwm(1, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 1
        target_pwm_2 = calculate_pwm(1, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 2
        # target_pwm_3 = calculate_pwm(0, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 3
        target_pwm_4 = calculate_pwm(0, abs(left_x), slowdown_factor, min_pwm, max_pwm)  # Động cơ 4
    elif left_y < 0:  # Khi y > 0
        target_pwm_1 = calculate_pwm(1, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 1
        # target_pwm_2 = calculate_pwm(1, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 2
        # target_pwm_3 = calculate_pwm(1, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 3
        target_pwm_4 = calculate_pwm(1, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 4
    elif left_y > 0:  # Khi y < 0
        # target_pwm_1 = calculate_pwm(0, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 1
        target_pwm_2 = calculate_pwm(0, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 2
        target_pwm_3 = calculate_pwm(0, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 3
        # target_pwm_4 = calculate_pwm(0, abs(left_y), slowdown_factor, min_pwm, max_pwm)  # Động cơ 4
    else:  # Khi joystick trái ở vị trí trung tâm
        target_pwm_1 = 1500
        target_pwm_2 = 1500
        target_pwm_3 = 1500
        target_pwm_4 = 1500

    # Tính toán PWM cho 4 động cơ dọc theo trục Y của joystick phải
    if right_y < 0:  # Khi y > 0
        target_pwm_5 = calculate_pwm(1, abs(right_y), slowdown_factor_vertical, min_pwm_Vslow, max_pwm_Vslow)  # Động cơ 5
        target_pwm_6 = calculate_pwm(1, abs(right_y), slowdown_factor_vertical, min_pwm_Vslow, max_pwm_Vslow)  # Động cơ 6
        target_pwm_7 = calculate_pwm(1, abs(right_y), slowdown_factor_vertical, min_pwm_Vfast, max_pwm_Vfast)  # Động cơ 7
        target_pwm_8 = calculate_pwm(1, abs(right_y), slowdown_factor_vertical, min_pwm_Vfast, max_pwm_Vfast)  # Động cơ 8
    elif right_y > 0:  # Khi y < 0
        target_pwm_5 = calculate_pwm(0, abs(right_y), slowdown_factor_vertical, min_pwm_Vslow, max_pwm_Vslow)  # Động cơ 5
        target_pwm_6 = calculate_pwm(0, abs(right_y), slowdown_factor_vertical, min_pwm_Vslow, max_pwm_Vslow)  # Động cơ 6
        target_pwm_7 = calculate_pwm(0, abs(right_y), slowdown_factor_vertical, min_pwm_Vfast, max_pwm_Vfast)  # Động cơ 7
        target_pwm_8 = calculate_pwm(0, abs(right_y), slowdown_factor_vertical, min_pwm_Vfast, max_pwm_Vfast)  # Động cơ 8
    else:
        # Nếu joystick phải không thay đổi, giữ nguyên giá trị PWM
        target_pwm_5 = current_pwm_motor_5
        target_pwm_6 = current_pwm_motor_6
        target_pwm_7 = current_pwm_motor_7
        target_pwm_8 = current_pwm_motor_8

    # Điều khiển yaw khi nút nhấn 4 và 5 được nhấn
    if button_4_pressed:  # Khi nút 4 được nhấn
        target_pwm_1 = set_pwm_limits(target_pwm_1 + motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
        target_pwm_2 = set_pwm_limits(target_pwm_2 - motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
        target_pwm_3 = set_pwm_limits(target_pwm_3 + motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
        target_pwm_4 = set_pwm_limits(target_pwm_4 - motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
    if button_5_pressed:  # Khi nút 5 được nhấn
        target_pwm_1 = set_pwm_limits(target_pwm_1 - motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
        target_pwm_2 = set_pwm_limits(target_pwm_2 + motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
        target_pwm_3 = set_pwm_limits(target_pwm_3 - motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)
        target_pwm_4 = set_pwm_limits(target_pwm_4 + motor_diff_yaw, min_pwm_yaw, max_pwm_yaw)

    # Điều chỉnh PWM từ từ đến giá trị mục tiêu
    current_pwm_motor_1 = adjust_pwm_to_target(current_pwm_motor_1, target_pwm_1, slowdown_factor)
    current_pwm_motor_2 = adjust_pwm_to_target(current_pwm_motor_2, target_pwm_2, slowdown_factor)
    current_pwm_motor_3 = adjust_pwm_to_target(current_pwm_motor_3, target_pwm_3, slowdown_factor)
    current_pwm_motor_4 = adjust_pwm_to_target(current_pwm_motor_4, target_pwm_4, slowdown_factor)

    current_pwm_motor_5 = adjust_pwm_to_target(current_pwm_motor_5, target_pwm_5, slowdown_factor_vertical)
    current_pwm_motor_6 = adjust_pwm_to_target(current_pwm_motor_6, target_pwm_6, slowdown_factor_vertical)
    current_pwm_motor_7 = adjust_pwm_to_target(current_pwm_motor_7, target_pwm_7, slowdown_factor_vertical)
    current_pwm_motor_8 = adjust_pwm_to_target(current_pwm_motor_8, target_pwm_8, slowdown_factor_vertical)

    # Gửi dữ liệu PWM đến STM32 qua USB CDC
    pwm_data = f"{current_pwm_motor_1}{current_pwm_motor_2}{current_pwm_motor_3}{current_pwm_motor_4}{current_pwm_motor_5}{current_pwm_motor_6}{current_pwm_motor_7}{current_pwm_motor_8}\n"
    ser.write(pwm_data.encode())

    # 🟢 Debug: In ra giá trị PWM đã gửi
    print(f"✅ Đã gửi đến STM32: {pwm_data.strip()}")

# 🔹 Đóng kết nối khi kết thúc
conn.close()
server_socket.close()