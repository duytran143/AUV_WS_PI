#!/usr/bin/env python3
"""
XY_stabilizer_node.py  —  ROS 2 node for velocity‑based control of a 4‑vectored‑thruster ROV

* Map joystick axes → desired surge / sway velocity (m/s) & yaw‑rate (rad/s)
* Estimate body‑frame velocity from accelerometer only (gravity compensation, bias tracking,
  higher‑cutoff LPF, leaky integrator + adaptive leak + ZUPT)
* Run PID control on velocity errors (surge, sway) and yaw‑rate error → forces Fx, Fy, Mz
* Publish Fx, Fy, Mz on `/force_cmds`
* Log filtered accel (x,y), gyro-z, estimated vel, and force cmds on one aligned line
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit

    def __call__(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return out

class XYStabilizerNode(Node):
    CONTROL_HZ = 50.0
    MAX_VEL = 0.6
    MAX_YAW_RATE = 0.5
    ZUPT_DELAY = 0.1               # shorten ZUPT wait
    STATIONARY_ACC_THRESH = 0.15
    LPF_ALPHA = 0.8                # higher cutoff for faster response
    LEAK_NORMAL = 0.1              # normal slow leak
    LEAK_IDLE = 10.0               # fast leak when stationary

    def __init__(self):
        super().__init__('XY_stabilizer_node')
        self._vx_des = 0.0
        self._vy_des = 0.0
        self._wz_des = 0.0
        self.v_est = np.zeros(3)
        self.a_f_prev = np.zeros(3)
        self.a_f = np.zeros(3)
        self.bias = np.zeros(3)
        self.yaw_rate_meas = 0.0
        self.stationary_start = None

        self.pid_x   = PID(1.2, 0.1,   0.01, integral_limit=1.0)
        self.pid_y   = PID(1.2, 0.1,   0.01, integral_limit=1.0)
        self.pid_yaw = PID(1.0, 0.05, 0.005, integral_limit=0.5)

        self.joy_sub  = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.acc_sub  = self.create_subscription(Vector3Stamped, '/accel', self.accel_cb, 10)
        self.gyro_sub = self.create_subscription(Vector3Stamped, '/gyro', self.gyro_cb, 10)
        self.force_pub = self.create_publisher(Float32MultiArray, '/force_cmds', 10)

        self.last_time = self.get_clock().now()
        self.control_timer = self.create_timer(1.0 / self.CONTROL_HZ, self.control_loop)
        self.get_logger().info('XY stabilizer node ready.')

    def joy_cb(self, msg: Joy):
        # swap axes[1]→vx, axes[0]→vy
        self._vx_des = self.MAX_VEL * float(msg.axes[1])
        self._vy_des = self.MAX_VEL * float(msg.axes[0])
        self._wz_des = self.MAX_YAW_RATE * float(msg.axes[2])

    def accel_cb(self, msg: Vector3Stamped):
        # gravity remove
        a_raw = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        a_lin = a_raw - np.array([0, 0, 9.81])
        norm = np.linalg.norm(a_lin)
        # bias tracking when truly stationary
        if norm < self.STATIONARY_ACC_THRESH:
            if self.stationary_start is None:
                self.stationary_start = self.get_clock().now()
            self.bias = 0.995 * self.bias + 0.005 * a_lin
        else:
            self.stationary_start = None
        # LPF
        self.a_f = self.LPF_ALPHA * (a_lin - self.bias) + (1 - self.LPF_ALPHA) * self.a_f_prev
        self.a_f_prev = self.a_f
        # velocity estimation
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        lam = self.LEAK_IDLE if norm < self.STATIONARY_ACC_THRESH else self.LEAK_NORMAL
        # always leak
        self.v_est = (1 - lam * dt) * self.v_est
        # integrate only on active accel
        if norm >= self.STATIONARY_ACC_THRESH:
            self.v_est += self.a_f * dt
        self.last_time = now

    def gyro_cb(self, msg: Vector3Stamped):
        self.yaw_rate_meas = msg.vector.z

    def control_loop(self):
        # ZUPT and reset
        if self.stationary_start is not None:
            elapsed = (self.get_clock().now() - self.stationary_start).nanoseconds * 1e-9
            if elapsed >= self.ZUPT_DELAY:
                self.v_est[0:2] = 0.0
                self.pid_x.integral = 0.0
                self.pid_y.integral = 0.0
        dt = 1.0 / self.CONTROL_HZ
        Fx = self.pid_x(self._vx_des - self.v_est[0], dt)
        Fy = self.pid_y(self._vy_des - self.v_est[1], dt)
        Mz = self.pid_yaw(self._wz_des - self.yaw_rate_meas, dt)
        # publish
        msg = Float32MultiArray(data=[Fx, Fy, Mz])
        self.force_pub.publish(msg)
        # log
        fx, fy = self.a_f[0], self.a_f[1]
        vx, vy = self.v_est[0], self.v_est[1]
        log = (
            f"filt=[{fx:+7.3f},{fy:+7.3f}] "
            f"gyro={self.yaw_rate_meas:+7.3f} "
            f"vel=[{vx:+7.3f},{vy:+7.3f}] "
            f"Fx={Fx:+7.3f} Fy={Fy:+7.3f} Mz={Mz:+7.3f}"
        )
        print(log, end='\r', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = XYStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
