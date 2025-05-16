#!/usr/bin/env python3
"""
ROS2 node to stabilize ROV: maintains roll, pitch, yaw rate, and depth,
while open-loop controlling surge and sway from setpoints.
Logs PWM outputs on a single line for debugging, clamped between 1200–1800 µs.
"""
import rclpy
from rclpy.node import Node
import numpy as np
import sys

from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from std_msgs.msg import Float32, Float32MultiArray


class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float) -> float:
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


class ROVStabilizerV1Node(Node):
    def __init__(self):
        super().__init__('ROV_stabilizer_V1_node')

        # --- Parameters (tune these) ---
        params = {
            'kp_roll': 1.0, 'ki_roll': 0.0, 'kd_roll': 0.1,
            'kp_pitch': 1.0, 'ki_pitch': 0.0, 'kd_pitch': 0.1,
            'kp_yaw': 1.0, 'ki_yaw': 0.0, 'kd_yaw': 0.1,
            'kp_depth': 2.0, 'ki_depth': 0.0, 'kd_depth': 0.2,
            'control_rate': 20.0,
            'deadband_threshold': 0.1, 'slope_pos': 50.0, 'slope_neg': 50.0, 'pwm_zero': 1500.0
        }
        for name, default in params.items():
            self.declare_parameter(name, default)

        rate = self.get_parameter('control_rate').value
        dt = 1.0 / rate

        # --- PID controllers ---
        self.pid_roll  = PID(
            self.get_parameter('kp_roll').value,
            self.get_parameter('ki_roll').value,
            self.get_parameter('kd_roll').value,
            dt
        )
        self.pid_pitch = PID(
            self.get_parameter('kp_pitch').value,
            self.get_parameter('ki_pitch').value,
            self.get_parameter('kd_pitch').value,
            dt
        )
        self.pid_yaw   = PID(
            self.get_parameter('kp_yaw').value,
            self.get_parameter('ki_yaw').value,
            self.get_parameter('kd_yaw').value,
            dt
        )
        self.pid_depth = PID(
            self.get_parameter('kp_depth').value,
            self.get_parameter('ki_depth').value,
            self.get_parameter('kd_depth').value,
            dt
        )

        # --- Thruster allocation matrix A and pseudoinverse ---
        A = np.array([
            [-0.707,    0.0,     0.0,   -0.707,  -0.707,    0.0,     0.0,   -0.707],
            [ 0.707,    0.0,     0.0,   -0.707,  -0.707,    0.0,     0.0,    0.707],
            [   0.0,    1.0,     1.0,     0.0,      0.0,     1.0,    1.0,      0.0],
            [   0.0,  -0.14685,-0.14685,   0.0,      0.0,     0.14685,0.14685,   0.0],
            [   0.0,   0.13400,-0.13400,   0.0,      0.0,     0.13400,-0.13400,  0.0],
            [ 0.27975,  0.0,     0.0,     0.27975, -0.27975,   0.0,     0.0,    -0.27975]
        ])
        self.A_pinv = np.linalg.pinv(A)

        # Initial state and setpoints
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0
        self.depth = 0.0
        self.Fx_sp = 0.0
        self.Fy_sp = 0.0
        self.Vz_sp = 0.0
        self.depth_sp = 0.0

        # --- Subscribers ---
        self.create_subscription(Vector3Stamped, '/setpoints', self.setpoints_cb, 10)
        self.create_subscription(Float32,        '/depth_sp', self.depth_sp_cb, 10)
        self.create_subscription(Vector3Stamped, '/imu/euler', self.euler_cb, 10)
        self.create_subscription(Vector3Stamped, '/gyro',      self.gyro_cb, 10)
        self.create_subscription(Float32,        '/depth',     self.depth_cb, 10)

        # --- Publishers ---
        self.wrench_pub = self.create_publisher(WrenchStamped,     '/wrench', 10)
        self.pwm_pub    = self.create_publisher(Float32MultiArray, '/PWM',    10)

        # Control loop timer
        self.timer = self.create_timer(dt, self.control_loop)

    # ----- Callbacks -----
    def setpoints_cb(self, msg: Vector3Stamped):
        self.Fx_sp = msg.vector.x
        self.Fy_sp = msg.vector.y
        self.Vz_sp = msg.vector.z

    def depth_sp_cb(self, msg: Float32):
        self.depth_sp = msg.data

    def euler_cb(self, msg: Vector3Stamped):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y

    def gyro_cb(self, msg: Vector3Stamped):
        self.yaw_rate = msg.vector.z

    def depth_cb(self, msg: Float32):
        self.depth = msg.data

    # ----- Control loop -----
    def control_loop(self):
        e_roll = -self.roll
        e_pitch = -self.pitch
        e_yaw = self.Vz_sp - self.yaw_rate
        e_depth = self.depth_sp - self.depth

        # PID outputs
        Mx = self.pid_roll.compute(e_roll)
        My = self.pid_pitch.compute(e_pitch)
        Mz = self.pid_yaw.compute(e_yaw)
        Fz = self.pid_depth.compute(e_depth)

        # Open-loop Fx, Fy
        Fx = self.Fx_sp
        Fy = self.Fy_sp

        # Assemble wrench
        wrench = np.array([[Fx], [Fy], [Fz], [Mx], [My], [Mz]])

        # Thruster forces
        forces = self.A_pinv @ wrench

        # Map & clamp
        deadband = self.get_parameter('deadband_threshold').value
        slope_pos = self.get_parameter('slope_pos').value
        slope_neg = self.get_parameter('slope_neg').value
        pwm_zero = self.get_parameter('pwm_zero').value
        pwm_values = []
        for f in forces.flatten():
            if abs(f) <= deadband:
                pwm = pwm_zero
            elif f > 0:
                pwm = pwm_zero + slope_pos * f
            else:
                pwm = pwm_zero - slope_neg * abs(f)
            pwm = max(1200.0, min(1800.0, pwm))
            pwm_values.append(int(pwm))

        # Publish wrench
        ws = WrenchStamped()
        ws.header.stamp = self.get_clock().now().to_msg()
        ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z = Fx, Fy, Fz
        ws.wrench.torque.x, ws.wrench.torque.y, ws.wrench.torque.z = Mx, My, Mz
        self.wrench_pub.publish(ws)
        arr = Float32MultiArray(data=[float(p) for p in pwm_values])
        self.pwm_pub.publish(arr)

        # Console log PWM values
        log_line = ' '.join(str(p) for p in pwm_values)
        sys.stdout.write('\r' + log_line)
        sys.stdout.flush()


def main(args=None):
    rclpy.init()
    node = ROVStabilizerV1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
