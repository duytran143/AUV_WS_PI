#!/usr/bin/env python3
"""
ROS2 node to stabilize ROV: maintains roll, pitch, yaw rate, and depth,
while open-loop controlling sway when not distance-lock.
Optional distance-lock feature uses two PIDs:
  - yaw equalization PID (kp_yaw_eq, ki_yaw_eq, kd_yaw_eq) on error P1–P2
  - range-hold PID (kp_range, ki_range, kd_range) on error (avg(P1,P2)–D_sp)
Supports live tuning of all PID coefficients via ROS2 parameters.
Accepts an external baseline PWM for vertical thrusters via the 'PWM_depth_total' topic.
Depth control can be bypassed (Fz set to 0) to let external PWM dictate vertical thrusters.
Logs only PWM values for debugging.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import numpy as np
import sys

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Float32MultiArray, Int32, Bool


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

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class ROVStabilizerV1Node(Node):
    def __init__(self):
        super().__init__('ROV_stabilizer_V1_node')

        # default PID and control parameters
        defaults = {
            'kp_roll': 0.062, 'ki_roll': 0.0, 'kd_roll': 0.05,
            'kp_pitch': 0.09, 'ki_pitch': 0.0, 'kd_pitch': 0.07,
            'kp_yaw': 0.0,   'ki_yaw': 0.0,  'kd_yaw': 0.0,
            'kp_depth': 1.0,'ki_depth': 0.0, 'kd_depth': 0.1,
            # distance-lock PIDs
            'kp_yaw_eq': 0.01, 'ki_yaw_eq': 0.0,  'kd_yaw_eq': 0.01,
            'kp_range':  0.05, 'ki_range':  0.0,  'kd_range':  0.01,
            # output limits
            'max_fx': 10.0,
            'control_rate': 100.0,
            'deadband_threshold': 0.1,
            'slope_pos': 50.0,
            'slope_neg': 50.0,
            # baseline PWM defaults
            'pwm_zero_horiz': 1500.0,
            'pwm_zero_vert':  1600.0
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        self.dt = 1.0 / self.get_parameter('control_rate').value

        # PID controllers
        self.pid_roll   = PID(self.get_parameter('kp_roll').value,
                              self.get_parameter('ki_roll').value,
                              self.get_parameter('kd_roll').value,
                              self.dt)
        self.pid_pitch  = PID(self.get_parameter('kp_pitch').value,
                              self.get_parameter('ki_pitch').value,
                              self.get_parameter('kd_pitch').value,
                              self.dt)
        self.pid_yaw    = PID(self.get_parameter('kp_yaw').value,
                              self.get_parameter('ki_yaw').value,
                              self.get_parameter('kd_yaw').value,
                              self.dt)
        # depth PID available but bypassed
        self.pid_depth  = PID(self.get_parameter('kp_depth').value,
                              self.get_parameter('ki_depth').value,
                              self.get_parameter('kd_depth').value,
                              self.dt)
        self.pid_yaw_eq = PID(self.get_parameter('kp_yaw_eq').value,
                              self.get_parameter('ki_yaw_eq').value,
                              self.get_parameter('kd_yaw_eq').value,
                              self.dt)
        self.pid_range  = PID(self.get_parameter('kp_range').value,
                              self.get_parameter('ki_range').value,
                              self.get_parameter('kd_range').value,
                              self.dt)

        self.add_on_set_parameters_callback(self._on_parameters)

        # thrust-mapping matrix (6×8)
        A = np.array([
            [-0.707,   0.0,     0.0,    -0.707, -0.707,  0.0,     0.0,    -0.707],
            [ 0.707,   0.0,     0.0,    -0.707, -0.707,  0.0,     0.0,     0.707],
            [ 0.0,     1.0,     1.0,     0.0,    0.0,     1.0,     1.0,     0.0  ],
            [ 0.0,    -0.14685,-0.14685, 0.0,    0.0,     0.14685, 0.14685, 0.0  ],
            [ 0.0,     0.134,  -0.134,   0.0,    0.0,     0.134,  -0.134,  0.0  ],
            [ 0.27975, 0.0,     0.0,     0.27975,-0.27975,0.0,     0.0,    -0.27975]
        ])
        self.A_pinv = np.linalg.pinv(A)

        # internal state
        self.roll = self.pitch = self.yaw_rate = self.depth = 0.0
        self.Fx_sp = self.Fy_sp = self.Vz_sp = self.depth_sp = 0.0
        self.p1 = self.p2 = 0
        self.distance_lock_on = False
        self.lock_sp = 0.0
        self.pwm_zero_horiz = self.get_parameter('pwm_zero_horiz').value
        self.pwm_zero_vert  = self.get_parameter('pwm_zero_vert').value

        # subscriptions
        self.create_subscription(Vector3Stamped, '/imu/euler', self.euler_cb, 10)
        self.create_subscription(Vector3Stamped, 'gyro',          self.gyro_cb,   10)
        self.create_subscription(Float32,         '/depth',        self.depth_cb, 10)
        self.create_subscription(Vector3Stamped, '/setpoints',    self.setpoints_cb, 10)
        self.create_subscription(Float32,         'depth_sp',      self.depth_sp_cb, 10)
        self.create_subscription(Int32,           'proximity1',    self.prox1_cb,  10)
        self.create_subscription(Int32,           'proximity2',    self.prox2_cb,  10)
        self.create_subscription(Bool,            'distance_lock', self.lock_cb,   10)
        self.create_subscription(Int32,           'lock_setpoint', self.lock_sp_cb,10)
        self.create_subscription(Int32,           'PWM_depth_total', self.pwm_depth_cb, 10)

        # publisher for PWM
        self.pwm_pub = self.create_publisher(Float32MultiArray, '/PWM', 10)

        # start control loop
        self.timer = self.create_timer(self.dt, self.loop)

    def _on_parameters(self, params):
        return SetParametersResult(successful=True)

    # callbacks
    def setpoints_cb(self,     msg: Vector3Stamped):
        self.Fx_sp, self.Fy_sp, self.Vz_sp = msg.vector.x, msg.vector.y, msg.vector.z
    def depth_sp_cb(self,      msg: Float32):
        self.depth_sp = msg.data
    def euler_cb(self,         msg: Vector3Stamped):
        self.roll, self.pitch = msg.vector.x, msg.vector.y
    def gyro_cb(self,          msg: Vector3Stamped):
        self.yaw_rate = msg.vector.z
    def depth_cb(self,         msg: Float32):
        self.depth = msg.data
    def prox1_cb(self,         msg: Int32):
        self.p1 = msg.data
    def prox2_cb(self,         msg: Int32):
        self.p2 = msg.data
    def lock_cb(self,          msg: Bool):
        self.distance_lock_on = msg.data
    def lock_sp_cb(self,       msg: Int32):
        self.lock_sp = float(msg.data)
        self.pid_range.reset()
    def pwm_depth_cb(self,     msg: Int32):
        self.pwm_zero_vert = float(msg.data)
        self.get_logger().info(f"V-thruster PWM baseline{self.pwm_zero_vert}")

    def loop(self):
        # attitude stabilization
        Mx = self.pid_roll.compute(-self.roll)
        My = self.pid_pitch.compute(-self.pitch)
        # depth bypass: do not use PID depth
        Fz = 0.0

        if self.distance_lock_on:
            # yaw equalization PID
            Mz = self.pid_yaw_eq.compute(self.p1 - self.p2)
            # range-hold override Fx, clamp output
            avg = 0.5 * (self.p1 + self.p2)
            Fx = self.pid_range.compute(avg - self.lock_sp)
            fx_limit = self.get_parameter('max_fx').value
            Fx = max(-fx_limit, min(fx_limit, Fx))
        else:
            # normal control
            Mz = self.pid_yaw.compute(self.Vz_sp - self.yaw_rate)
            Fx = self.Fx_sp
        Fy = self.Fy_sp

        # map to thruster forces
        wrench = np.array([[Fx], [Fy], [Fz], [Mx], [My], [Mz]])
        forces = self.A_pinv @ wrench

        # convert forces to PWM signals
        deadband = self.get_parameter('deadband_threshold').value
        slope_pos = self.get_parameter('slope_pos').value
        slope_neg = self.get_parameter('slope_neg').value

        pwm_vals = []
        for idx, f in enumerate(forces.flatten()):
            # choose baseline
            if idx in [1, 2, 5, 6]:  # vertical thrusters
                pwm_zero = self.pwm_zero_vert
            else:
                pwm_zero = self.pwm_zero_horiz
            if abs(f) <= deadband:
                pwm = pwm_zero
            elif f > 0:
                pwm = pwm_zero + slope_pos * f
            else:
                pwm = pwm_zero - slope_neg * abs(f)
            pwm_vals.append(int(max(1200, min(1800, pwm))))

        # publish PWM array
        self.pwm_pub.publish(Float32MultiArray(data=pwm_vals))

        # log PWM only
        sys.stdout.write('\r' + ' '.join(str(p) for p in pwm_vals))
        sys.stdout.flush()

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = ROVStabilizerV1Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
