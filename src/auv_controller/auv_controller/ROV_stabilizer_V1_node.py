#!/usr/bin/env python3
"""
ROS2 node to stabilize ROV: maintains roll, pitch, yaw rate, and depth,
while open-loop controlling surge and sway from setpoints.
Supports live tuning of all PID coefficients via ROS2 parameters.
Logs selected yaw and PWM parameters for debugging.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
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

    def reset_integral(self):
        self.integral = 0.0


class ROVStabilizerV1Node(Node):
    def __init__(self):
        super().__init__('ROV_stabilizer_V1_node')  

        defaults = {
            'kp_roll': 0.062, 'ki_roll': 0.0, 'kd_roll': 0.04,
            'kp_pitch': 0.09, 'ki_pitch': 0.0, 'kd_pitch': 0.07,
            'kp_yaw': 0.007, 'ki_yaw': 0.0, 'kd_yaw': 0.004,
            'kp_depth': 0.0, 'ki_depth': 0.0, 'kd_depth': 0.0,
            'control_rate': 100.0,
            'deadband_threshold': 0.1,
            'slope_pos': 50.0,
            'slope_neg': 50.0,
            'pwm_zero': 1500.0
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self.control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.control_rate

        self.pid_roll = PID(self.get_parameter('kp_roll').value,
                            self.get_parameter('ki_roll').value,
                            self.get_parameter('kd_roll').value,
                            self.dt)
        self.pid_pitch = PID(self.get_parameter('kp_pitch').value,
                             self.get_parameter('ki_pitch').value,
                             self.get_parameter('kd_pitch').value,
                             self.dt)
        self.pid_yaw = PID(self.get_parameter('kp_yaw').value,
                           self.get_parameter('ki_yaw').value,
                           self.get_parameter('kd_yaw').value,
                           self.dt)
        self.pid_depth = PID(self.get_parameter('kp_depth').value,
                             self.get_parameter('ki_depth').value,
                             self.get_parameter('kd_depth').value,
                             self.dt)

        self.add_on_set_parameters_callback(self._on_parameter_update)

        A = np.array([
            [-0.707, 0.0, 0.0, -0.707, -0.707, 0.0, 0.0, -0.707],
            [0.707, 0.0, 0.0, -0.707, -0.707, 0.0, 0.0, 0.707],
            [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
            [0.0, -0.14685, -0.14685, 0.0, 0.0, 0.14685, 0.14685, 0.0],
            [0.0, 0.134, -0.134, 0.0, 0.0, 0.134, -0.134, 0.0],
            [0.27975, 0.0, 0.0, 0.27975, -0.27975, 0.0, 0.0, -0.27975]
        ])
        self.A_pinv = np.linalg.pinv(A)

        self.roll = self.pitch = self.yaw_rate = self.depth = 0.0
        self.Fx_sp = self.Fy_sp = self.Vz_sp = self.depth_sp = 0.0

        self.create_subscription(Vector3Stamped, '/imu/euler', self.euler_cb, 10)
        self.create_subscription(Vector3Stamped, 'gyro', self.gyro_cb, 10)
        self.create_subscription(Float32, '/depth', self.depth_cb, 10)
        self.create_subscription(Vector3Stamped, '/setpoints', self.setpoints_cb, 10)
        self.create_subscription(Float32, '/depth_sp', self.depth_sp_cb, 10)

        self.wrench_pub = self.create_publisher(WrenchStamped, '/wrench', 10)
        self.pwm_pub = self.create_publisher(Float32MultiArray, '/PWM', 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def _on_parameter_update(self, params):
        for p in params:
            if p.name == 'kp_roll': self.pid_roll.kp = p.value
            elif p.name == 'ki_roll': self.pid_roll.ki = p.value
            elif p.name == 'kd_roll': self.pid_roll.kd = p.value
            elif p.name == 'kp_pitch': self.pid_pitch.kp = p.value
            elif p.name == 'ki_pitch': self.pid_pitch.ki = p.value
            elif p.name == 'kd_pitch': self.pid_pitch.kd = p.value
            elif p.name == 'kp_yaw': self.pid_yaw.kp = p.value
            elif p.name == 'ki_yaw': self.pid_yaw.ki = p.value
            elif p.name == 'kd_yaw': self.pid_yaw.kd = p.value
            elif p.name == 'kp_depth': self.pid_depth.kp = p.value
            elif p.name == 'ki_depth': self.pid_depth.ki = p.value
            elif p.name == 'kd_depth': self.pid_depth.kd = p.value
        return SetParametersResult(successful=True)

    def setpoints_cb(self, msg: Vector3Stamped):
        self.Fx_sp, self.Fy_sp, self.Vz_sp = msg.vector.x, msg.vector.y, msg.vector.z

    def depth_sp_cb(self, msg: Float32):
        self.depth_sp = msg.data

    def euler_cb(self, msg: Vector3Stamped):
        self.roll, self.pitch = msg.vector.x, msg.vector.y

    def gyro_cb(self, msg: Vector3Stamped):
        self.yaw_rate = msg.vector.z

    def depth_cb(self, msg: Float32):
        self.depth = msg.data

    def control_loop(self):
        e_roll = -self.roll
        e_pitch = -self.pitch
        e_depth = self.depth_sp - self.depth

        Mx = self.pid_roll.compute(e_roll)
        My = self.pid_pitch.compute(e_pitch)

        e_yaw = self.Vz_sp - self.yaw_rate
        Mz = self.pid_yaw.compute(e_yaw)

        Fz = self.pid_depth.compute(e_depth)

        Fx, Fy = self.Fx_sp, self.Fy_sp
        wrench = np.array([[Fx], [Fy], [Fz], [Mx], [My], [Mz]])
        forces = self.A_pinv @ wrench

        deadband = self.get_parameter('deadband_threshold').value
        slope_pos = self.get_parameter('slope_pos').value
        slope_neg = self.get_parameter('slope_neg').value
        pwm_zero = self.get_parameter('pwm_zero').value

        pwm_vals = []
        for f in forces.flatten():
            if abs(f) <= deadband:
                pwm = pwm_zero
            elif f > 0:
                pwm = pwm_zero + slope_pos * f
            else:
                pwm = pwm_zero - slope_neg * abs(f)
            pwm = max(1200.0, min(1800.0, pwm))
            pwm_vals.append(int(pwm))

        ws = WrenchStamped()
        ws.header.stamp = self.get_clock().now().to_msg()
        ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z = Fx, Fy, Fz
        ws.wrench.torque.x, ws.wrench.torque.y, ws.wrench.torque.z = Mx, My, Mz
        self.wrench_pub.publish(ws)

        arr_msg = Float32MultiArray(data=pwm_vals)
        self.pwm_pub.publish(arr_msg)

        # Debug yaw + selected PWM thrusters
        pwm1 = pwm_vals[0]
        pwm4 = pwm_vals[3]
        pwm5 = pwm_vals[4]
        pwm8 = pwm_vals[7]
        log_line = (f"Vz_sp: {self.Vz_sp:.2f}  yaw_rate: {self.yaw_rate:.2f}  "
                    f"e_yaw: {e_yaw:.2f}  Mz: {Mz:.2f}  "
                    f"PWM1:{pwm1} PWM4:{pwm4} PWM5:{pwm5} PWM8:{pwm8}")
        sys.stdout.write('\r' + log_line)
        sys.stdout.flush()

    def destroy_node(self):
        super().destroy_node()


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
