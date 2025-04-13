import cv2
import socket
import numpy as np
import threading

# Cấu hình camera
cap = cv2.VideoCapture(0)

# Thiết lập FPS tối đa và giảm độ phân giải video để tăng FPS
cap.set(cv2.CAP_PROP_FPS, 30)  # Tối đa 30 FPS
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Độ phân giải thấp hơn
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Cấu hình socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host_ip = '192.168.2.1'  # Địa chỉ IP máy nhận
port = 5001
socket_address = (host_ip, port)

# Hàm để tăng độ sáng
def increase_brightness(frame, value=10):  # Giá trị độ sáng thấp hơn
    # Thêm giá trị vào mỗi pixel của khung hình
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.add(v, value)  # Tăng độ sáng (value có thể là một số dương)
    final_hsv = cv2.merge((h, s, v))
    frame_bright = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return frame_bright

# Hàm gửi dữ liệu qua socket (chạy trong một thread riêng)
def send_frame_data(frame_data):
    server_socket.sendto(frame_data, socket_address)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Tăng độ sáng của khung hình
    frame = increase_brightness(frame, value=10)

    # Nén khung hình
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # Chất lượng nén giảm xuống (70)
    result, frame = cv2.imencode('.jpg', frame, encode_param)

    # Chia nhỏ dữ liệu nếu quá lớn
    frame_data = np.array(frame)
    max_packet_size = 65507  # Kích thước tối đa của một gói UDP (với header là 8 bytes)
    data_len = len(frame_data)

    # Gửi dữ liệu theo từng phần (bằng cách sử dụng thread để gửi dữ liệu trong khi tiếp tục đọc frame mới)
    for i in range(0, data_len, max_packet_size):
        packet = frame_data[i:i + max_packet_size]

        # Khởi động thread để gửi mỗi gói dữ liệu
        thread = threading.Thread(target=send_frame_data, args=(packet,))
        thread.start()

cap.release()
server_socket.close()