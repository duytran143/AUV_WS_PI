#!/usr/bin/env python3
"""
XY_stabilizer_node.py — ROS 2 node for surge/heave user control + yaw stabilization + thruster allocation

* Subscribe to /joy (sensor_msgs/Joy) for joystick inputs:
    - axes[0] → surge force command (Fx)
    - axes[1] → heave force command (Fy)
    - axes[2] → yaw-axis for stabilization
* Subscribe to /gyro (geometry_msgs/Vector3Stamped) for measured yaw-rate
* Compute yaw torque Mz via PID on yaw-rate error
* Allocate wrench [Fx, Fy, Mz] to N thrusters based on geometry
* Publish individual thruster forces (std_msgs/Float32MultiArray) on /thruster_cmds
* Log Fx, Fy, wz_user, wz_meas, Mz and thruster forces on one line
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray
import numpy as np

def clamp(x, min_val, max_val):
    return max(min(x, max_val), min_val)

class PID:
    def __init__(self, Kp, Ki, Kd, integrator_max=1.0, integrator_min=-1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integrator = 0.0
        self.prev_error = 0.0
        self.int_max = integrator_max
        self.int_min = integrator_min

    def compute(self, error, dt):
        self.integrator += error * dt
        self.integrator = clamp(self.integrator, self.int_min, self.int_max)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integrator + self.Kd * derivative
        self.prev_error = error
        return output

class XYStabilizerNode(Node):
    CONTROL_HZ = 50.0
    MAX_SURGE = 10.0    # max Fx
    MAX_HEAVE = 10.0    # max Fy
    MAX_YAW_RATE = 0.5  # rad/s
    DEADZONE = 0.05
    # Thruster geometry: list of (x, y, angle) in vehicle frame
    THRUSTERS = [
        {'pos': ( 0.3,  0.2), 'angle': 0},   # front-right, forward
        {'pos': (-0.3,  0.2), 'angle': 0},   # front-left, forward
        {'pos': (-0.3, -0.2), 'angle': 0},   # rear-left, forward
        {'pos': ( 0.3, -0.2), 'angle': 0},   # rear-right, forward
    ]

    def __init__(self):
        super().__init__('XY_stabilizer_node')
        # user commands
        self.Fx = 0.0
        self.Fy = 0.0
        self.wz_user = 0.0
        # measured yaw-rate
        self.wz_meas = 0.0
        # PID for yaw
        self.pid = PID(Kp=1.5, Ki=0.1, Kd=0.0, integrator_max=2.0, integrator_min=-2.0)
        # build allocation matrix A (3×N)
        self.A = self._build_allocation_matrix()
        self.A_pinv = np.linalg.pinv(self.A)

        # subscriptions
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(Vector3Stamped, '/gyro', self.gyro_cb, 10)
        # publisher
        self.pub = self.create_publisher(Float32MultiArray, '/thruster_cmds', 10)
        # control timer
        self.create_timer(1.0/self.CONTROL_HZ, self.control_loop)
        self.get_logger().info('XY stabilizer node ready')

    def _build_allocation_matrix(self):
        cols = len(self.THRUSTERS)
        A = np.zeros((3, cols))
        for i, th in enumerate(self.THRUSTERS):
            theta = np.deg2rad(th['angle'])
            ux, uy = np.cos(theta), np.sin(theta)
            x, y = th['pos']
            A[0, i] = ux
            A[1, i] = uy
            A[2, i] = x*uy - y*ux
        return A

    def joy_cb(self, msg: Joy):
        self.Fx = self.MAX_SURGE * float(msg.axes[0])
        self.Fy = self.MAX_HEAVE * float(msg.axes[1])
        yaw_axis = msg.axes[2]
        self.wz_user = self.MAX_YAW_RATE * yaw_axis if abs(yaw_axis) > self.DEADZONE else 0.0

    def gyro_cb(self, msg: Vector3Stamped):
        self.wz_meas = msg.vector.z

    def control_loop(self):
        error = self.wz_user - self.wz_meas
        Mz = self.pid.compute(error, 1.0/self.CONTROL_HZ)
        wrench = np.array([self.Fx, self.Fy, Mz])
        thr_forces = self.A_pinv.dot(wrench)
        msg = Float32MultiArray(data=thr_forces.tolist())
        self.pub.publish(msg)
        forces_str = ' '.join(f'{f:+6.2f}' for f in thr_forces)
        print(
            f"Fx={self.Fx:+6.2f} Fy={self.Fy:+6.2f} wz_user={self.wz_user:+.3f} "
            f"wz_meas={self.wz_meas:+.3f} Mz={Mz:+6.2f} thr=[{forces_str}]",
            end='\r', flush=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = XYStabilizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
