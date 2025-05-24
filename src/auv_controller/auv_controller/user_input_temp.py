#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int32, Bool
import socket
import threading
import sys
import time

RECONNECT_DELAY      = 1.0     # seconds
CONTROL_STATES_PORT  = 5004

class JoystickInputNode(Node):
    def __init__(self):
        super().__init__('user_input')

        # declare ROS2 parameters with defaults
        self.declare_parameter('alphaX',          -10.0)
        self.declare_parameter('alphaY',          -10.0)
        self.declare_parameter('alphaVz',         -5.0)
        self.declare_parameter('alphaD',          100.0)   # scale for depth_pwm_joy
        self.declare_parameter('adjust_interval',  0.01)
        self.declare_parameter('adjust_step',      1)
        self.declare_parameter('lock_min',         0)
        self.declare_parameter('lock_max',       600)

        # record max alphaY for lock mode
        self.alphaY_max = self.get_parameter('alphaY').value

        # Publishers
        self.joy_pub        = self.create_publisher(Joy,             'joy',            10)
        self.sp_pub         = self.create_publisher(Vector3Stamped, 'setpoints',      10)
        self.dist_lock_pub  = self.create_publisher(Bool,            'distance_lock',  10)
        self.lock_sp_pub    = self.create_publisher(Int32,           'lock_setpoint',  10)
        self.pwm_pub        = self.create_publisher(Int32,           'PWM_depth_total',10)

        # Subscribers for proximity sensors
        self.p1 = float('inf')
        self.p2 = float('inf')
        self.create_subscription(Int32, 'proximity1', self._p1_cb, 10)
        self.create_subscription(Int32, 'proximity2', self._p2_cb, 10)

        # Internal state
        self.dist_lock_on           = False
        self.default_lock_setpoint  = 300   # mm
        self.lock_setpoint          = self.default_lock_setpoint
        self._last_button1          = 0
        self._last_lock_adjust      = time.monotonic()

        # PWM state
        self.baseline_pwm           = 1600  # µs
        self._last_baseline_adjust  = time.monotonic()

        # UDP socket for control‐states payloads
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_ip = None

        # TCP server for joystick input
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
                conn.settimeout(0.001)
                self.client_ip = addr[0]
                self.get_logger().info(f"Joystick connected from {addr}")
                self._receive_loop(conn)
            except Exception as e:
                self.get_logger().error(f"Accept error: {e}. Retrying...")
                time.sleep(RECONNECT_DELAY)

    def _receive_loop(self, conn):
        with conn:
            while rclpy.ok():
                # receive raw joystick data
                try:
                    raw = conn.recv(1024).decode().strip()
                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().warn(f"Receive failed: {e}")
                    return

                if not raw:
                    continue
                parts = raw.split()
                if len(parts) < 8:
                    continue

                # parse axes & buttons
                try:
                    axes = [float(v) for v in parts[:8]]
                except ValueError:
                    continue
                buttons = [int(b) for b in parts[8:] if b.isdigit()]

                # publish Joy
                joy_msg = Joy()
                joy_msg.header.stamp    = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = 'joystick'
                joy_msg.axes            = axes
                joy_msg.buttons         = buttons
                self.joy_pub.publish(joy_msg)

                # fetch parameters
                alphaX          = self.get_parameter('alphaX').value
                raw_alphaY      = self.get_parameter('alphaY').value
                alphaVz         = self.get_parameter('alphaVz').value
                alphaD          = self.get_parameter('alphaD').value
                adjust_interval = self.get_parameter('adjust_interval').value
                adjust_step     = self.get_parameter('adjust_step').value
                lock_min        = self.get_parameter('lock_min').value
                lock_max        = self.get_parameter('lock_max').value

                # distance lock condition
                dist_lock_condition = (20 < self.p1 < 500 and 20 < self.p2 < 500)

                # toggle distance lock on button1 press
                btn1 = buttons[1] if len(buttons) > 1 else 0
                if btn1 and not self._last_button1:
                    if not self.dist_lock_on and dist_lock_condition:
                        self.dist_lock_on = True
                    else:
                        self.dist_lock_on = False
                        self.lock_setpoint = self.default_lock_setpoint
                    self.dist_lock_pub.publish(Bool(data=self.dist_lock_on))
                self._last_button1 = btn1

                # adjust lock_setpoint via axes[4]/axes[5]
                now = time.monotonic()
                if self.dist_lock_on and now - self._last_lock_adjust >= adjust_interval:
                    if len(axes) > 4 and axes[4] == 1.0:
                        self.lock_setpoint = max(lock_min, self.lock_setpoint - adjust_step)
                        self._last_lock_adjust = now
                    if len(axes) > 5 and axes[5] == 1.0:
                        self.lock_setpoint = min(lock_max, self.lock_setpoint + adjust_step)
                        self._last_lock_adjust = now

                # apply 60% alphaY when distance lock is active
                alphaY = raw_alphaY * 0.6 if self.dist_lock_on else raw_alphaY

                # compute manual setpoints
                Fy_sp = alphaY  * axes[0]
                Fx_sp = alphaX  * axes[1]
                Mz_sp = alphaVz * axes[2]

                # compute depth_pwm_joy
                depth_pwm_joy = axes[3] * alphaD

                # baseline_pwm via hat up/down (axes[7])
                if now - self._last_baseline_adjust >= adjust_interval:
                    if len(axes) > 7 and axes[7] == 1.0:
                        self.baseline_pwm = min(1700, self.baseline_pwm + 10)
                        self._last_baseline_adjust = now
                    elif len(axes) > 7 and axes[7] == -1.0:
                        self.baseline_pwm = max(1550, self.baseline_pwm - 10)
                        self._last_baseline_adjust = now

                # PWM_depth_total
                pwm_total = int(self.baseline_pwm + depth_pwm_joy)
                pwm_msg   = Int32(data=pwm_total)
                self.pwm_pub.publish(pwm_msg)

                # publish setpoints
                sp_msg = Vector3Stamped()
                sp_msg.header.stamp    = self.get_clock().now().to_msg()
                sp_msg.header.frame_id = 'setpoint'
                sp_msg.vector.x = Fx_sp
                sp_msg.vector.y = Fy_sp
                sp_msg.vector.z = Mz_sp
                self.sp_pub.publish(sp_msg)

                # publish lock_setpoint
                lock_sp_msg = Int32(data=self.lock_setpoint)
                self.lock_sp_pub.publish(lock_sp_msg)

                # send UDP control‐states unchanged…

                cond = 1 if dist_lock_condition else 0
                on   = 1 if self.dist_lock_on    else 0
                payload = f"{pwm_total} {cond:d} {on:d} {self.lock_setpoint:d}"
                if self.client_ip:
                    try:
                        self.udp_sock.sendto(
                            payload.encode(),
                            (self.client_ip, CONTROL_STATES_PORT)
                        )
                    except:
                        pass

                # log status
                cond_str = 'yes' if dist_lock_condition else 'no'
                log_line = (
                    f"Fx:{Fx_sp:6.2f} Fy:{Fy_sp:6.2f} Mz:{Mz_sp:6.2f} "
                    f"PWM:{pwm_total:4d} Cond:{cond_str} SP:{self.lock_setpoint:3d}"
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
