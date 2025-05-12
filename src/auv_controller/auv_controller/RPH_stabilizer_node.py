#!/usr/bin/env python3
"""
ROS 2 **RPH_stabilizer_node** – Euler‑angle stabilisation with depth jog.
Logs depth sensor value, desired depth, depth error, and thruster forces on a
single updating line, using tabs for clear alignment.
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Int32MultiArray
from sensor_msgs.msg import Joy

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.i = 0.0
        self.prev = 0.0
    def update(self, e):
        self.i += e * self.dt
        d = (e - self.prev) / self.dt
        self.prev = e
        return self.kp * e + self.ki * self.i + self.kd * d

class VelInt:
    def __init__(self):
        self.v = 0.0
    def update(self, a, dt):
        self.v += a * dt
        return self.v

class RPHStabilizerNode(Node):
    def __init__(self):
        super().__init__('RPH_stabilizer_node')
        gp = self.declare_parameter
        self.desired_depth = gp('desired_depth', 0.0).value
        self.depth_jog_rate = gp('depth_jog_rate', 0.3).value
        self.deadzone = gp('deadzone', 0.05).value
        self.depth_min = gp('depth_min', 0.0).value
        self.depth_max = gp('depth_max', 10.0).value
        self.dt = gp('dt_control', 0.02).value

        self.desired_roll = 0.0
        self.desired_pitch = 0.0

        self.roll = self.pitch = self.yaw = 0.0
        self.az = 0.0
        self.depth = self.depth_prev = 0.0
        self.axis2 = 0.0

        self.pid_roll = PID(0.4, 0, 0.05, self.dt)
        self.pid_pitch = PID(0.3, 0, 0.04, self.dt)
        self.pid_depth = PID(0.1, 0, 0, self.dt)
        self.vel_int = VelInt()

        qos = 10
        self.create_subscription(Vector3Stamped, 'imu/euler', self.cb_euler, qos)
        self.create_subscription(Vector3Stamped, 'accel', self.cb_accel, qos)
        self.create_subscription(Float32,        'depth', self.cb_depth, qos)
        self.create_subscription(Float32,        'desired_depth', self.cb_depth_set, qos)
        self.create_subscription(Joy,            'joy', self.cb_joy, qos)
        self.pwm_pub = self.create_publisher(Int32MultiArray, 'thruster_pwm', qos)

        self.create_timer(self.dt, self.loop)

    # callbacks
    def cb_euler(self, m):
        self.roll, self.pitch, self.yaw = m.vector.x, m.vector.y, m.vector.z
    def cb_accel(self, m):
        self.az = m.vector.z
    def cb_depth(self, m):
        self.depth = m.data
    def cb_depth_set(self, m):
        self.desired_depth = max(self.depth_min, min(self.depth_max, float(m.data)))
    def cb_joy(self, m):
        self.axis2 = m.axes[2] if len(m.axes) > 2 else 0.0

    # control loop
    def loop(self):
        if abs(self.axis2) > self.deadzone:
            self.desired_depth += self.axis2 * self.depth_jog_rate * self.dt
            self.desired_depth = max(self.depth_min, min(self.depth_max, self.desired_depth))

        ctrl_r = self.pid_roll.update(-self.roll)
        ctrl_p = self.pid_pitch.update(-self.pitch)

        vz_int = self.vel_int.update(self.az, self.dt)
        vz_d = (self.depth - self.depth_prev) / self.dt
        self.depth_prev = self.depth
        vz = 0.8 * vz_d + 0.2 * vz_int
        err_depth = self.desired_depth - self.depth
        ctrl_d = self.pid_depth.update(err_depth - vz)

        pwm, forces = self.mix(ctrl_r, ctrl_p, ctrl_d)
        self.pwm_pub.publish(Int32MultiArray(data=pwm))

        f5, f6, f7, f8 = forces
        sys.stdout.write(
            f"\rDepth:\t{self.depth:5.2f}\tDsp:\t{self.desired_depth:5.2f}\tErr:\t{err_depth:+5.2f}\t"
            f"f5:\t{f5:6.2f}\tf6:\t{f6:6.2f}\tf7:\t{f7:6.2f}\tf8:\t{f8:6.2f}")
        sys.stdout.flush()

    def mix(self, r, p, az):
        dr, dp, m = 0.14685, 0.13401, 6.0
        a, b, c = r / dr, p / dp, m * az
        f5 = (c - a - 2*b) / 6.0
        f6 = (a - b + 2*c) / 6.0
        f7 = (c + 2*a + b) / 6.0
        f8 = (-a + b + c) / 3.0
        pwm = [self.f2p(f) for f in (f5, f6, f7, f8)]
        return pwm, (f5, f6, f7, f8)

    @staticmethod
    def f2p(f):
        return int(round(max(1000, min(2000, (f * 2000) / 11 + 1500))))


def main():
    rclpy.init()
    node = RPHStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print()

if __name__ == '__main__':
    main()
