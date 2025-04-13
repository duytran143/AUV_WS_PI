#!/bin/bash

echo "🛠️ Đang khởi động hệ thống điều khiển joystick và camera..."

# Dừng tất cả các tiến trình cũ (nếu có)
pkill -f joystick_processor.py
pkill -f camera_send.py
sleep 1

# Kiểm tra kết nối với Windows
ping -c 1 192.168.2.1 > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "❌ Không thể kết nối với Windows (192.168.2.1). Kiểm tra kết nối mạng!"
    exit 1
else
    echo "✅ Kết nối mạng ổn định!"
fi
# Khởi động camera send
echo "🔹 Khởi động camera_send.py..."
python3 camera_send.py &  # Thêm dấu "&" để chạy dưới background

# Khởi động bộ xử lý dữ liệu joystick
echo "🔹 Khởi động joystick_processor.py..."
python3 joystick_input.py &  # Thêm dấu "&" để chạy dưới background

# Đợi cho đến khi người dùng dừng tiến trình
wait