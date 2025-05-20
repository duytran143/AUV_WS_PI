#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Int32, Bool
import socket
import threading
import sys
import time

RECONNECT_DELAY    = 1.0     # seconds
DEPTH_UDP_PORT     = 5004    # port for GUI depth
alphaX             = -10.0
alphaY             = -10.0
alphaVz            = -40.0
alphaD             = 0.1
ADJUST_INTERVAL    = 0.1     # seconds between setpoint adjustments
ADJUST_STEP        = 1       # mm per adjustment

class JoystickInputNode(Node):
    def __init__(self):
        super().__init__('joystick_input_V2_node')

        # Publishers
        self.joy_pub       = self.create_publisher(Joy,             'joy',           10)
        self.sp_pub        = self.create_publisher(Vector3Stamped, 'setpoints',     10)
        self.depth_pub     = self.create_publisher(Float32,         'depth_sp',      10)
        self.dist_lock_pub = self.create_publisher(Bool,            'distance_lock', 10)

        # Subscribers for proximity sensors
        self.p1 = float('inf')
        self.p2 = float('inf')
        self.create_subscription(Int32, 'proximity1', self._p1_cb, 10)
        self.create_subscription(Int32, 'proximity2', self._p2_cb, 10)

        # Internal state
        self.depth_sp      = 0.0
        self.dist_lock_on  = False
        self.default_lock_setpoint = 500  # mm
        self.lock_setpoint = self.default_lock_setpoint
        self._last_button1 = 0
        self._last_adjust  = time.monotonic()

        # UDP socket
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_ip = None

        # TCP server for joystick
        self.host = '0.0.0.0'; self.port = 5000
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        except Exception as e:
            self.get_logger().error(f"Failed to bind joystick socket: {e}")
            rclpy.shutdown()
            return

        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _p1_cb(self, msg: Int32):
        self.p1 = msg.data

    def _p2_cb(self, msg: Int32):
        self.p2 = msg.data

    def _accept_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                self.client_ip = addr[0]
                self.get_logger().info(f"Joystick connected from {addr}")
                self._receive_loop(conn)
            except Exception as e:
                self.get_logger().error(f"Accept error: {e}. Retrying...")
                time.sleep(RECONNECT_DELAY)

    def _receive_loop(self, conn):
        with conn:
            while rclpy.ok():
                try:
                    raw = conn.recv(1024).decode().strip()
                    if not raw:
                        continue
                except Exception as e:
                    self.get_logger().warn(f"Receive failed: {e}")
                    return

                parts = raw.split()
                if len(parts) < 8:
                    continue

                # parse axes & buttons
                try:
                    axes = [float(v) for v in parts[:8]]
                except ValueError:
                    continue
                buttons = []
                for b in parts[8:]:
                    try:
                        buttons.append(int(b))
                    except ValueError:
                        pass

                # publish Joy
                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = 'joystick'
                joy_msg.axes    = axes
                joy_msg.buttons = buttons
                self.joy_pub.publish(joy_msg)

                # toggle distance lock on button1 press, P1,P2<60
                btn1 = buttons[1] if len(buttons) > 1 else 0
                if btn1 and not self._last_button1:
                    if self.p1 < 60 and self.p2 < 60:
                        self.dist_lock_on = not self.dist_lock_on
                        if not self.dist_lock_on:
                            # reset setpoint when turned off
                            self.lock_setpoint = self.default_lock_setpoint
                        # publish lock state
                        lock_msg = Bool(data=self.dist_lock_on)
                        self.dist_lock_pub.publish(lock_msg)
                self._last_button1 = btn1

                # adjust lock setpoint via axes[4] (decr), axes[5] (incr)
                now = time.monotonic()
                if self.dist_lock_on and now - self._last_adjust >= ADJUST_INTERVAL:
                    if len(axes) > 4 and axes[4] == 1.0:
                        self.lock_setpoint = max(0, self.lock_setpoint - ADJUST_STEP)
                        self._last_adjust = now
                    if len(axes) > 5 and axes[5] == 1.0:
                        self.lock_setpoint += ADJUST_STEP
                        self._last_adjust = now

                # compute setpoints
                Fy_sp = alphaY  * axes[0]
                Fx_sp = alphaX  * axes[1]
                Mz_sp = alphaVz * axes[2]

                # update and clamp depth_sp
                self.depth_sp = min(max(self.depth_sp + alphaD * axes[3], 0.0), 98.0)

                # publish Vector3Stamped setpoints
                sp_msg = Vector3Stamped()
                sp_msg.header.stamp    = self.get_clock().now().to_msg()
                sp_msg.header.frame_id = 'setpoint'
                sp_msg.vector.x = Fx_sp
                sp_msg.vector.y = Fy_sp
                sp_msg.vector.z = Mz_sp
                self.sp_pub.publish(sp_msg)

                # publish depth_sp
                depth_msg = Float32(data=float(self.depth_sp))
                self.depth_pub.publish(depth_msg)

                # send depth via UDP
                if self.client_ip:
                    try:
                        self.udp_sock.sendto(f"{self.depth_sp:.2f}".encode(),
                                             (self.client_ip, DEPTH_UDP_PORT))
                    except Exception:
                        pass

                # log
                log_line = (
                    f"Fx:{Fx_sp:6.2f} Fy:{Fy_sp:6.2f} Mz:{Mz_sp:6.2f} "
                    f"Depth:{self.depth_sp:5.2f} "
                    f"Lock:{int(self.dist_lock_on)} "
                    f"SP:{self.lock_setpoint:4d}"
                )
                sys.stdout.write('\r' + log_line)
                sys.stdout.flush()

    def destroy_node(self):
        try:
            self.server_socket.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
