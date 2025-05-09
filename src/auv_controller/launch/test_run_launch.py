from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. sensor_node
        Node(
            package='auv_controller',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
        ),

        # 2. joystick_input_node
        Node(
            package='auv_controller',
            executable='joystick_input_node',
            name='joystick_input_node',
            output='screen',
        ),

        # 3. XY_stabilizer_node
        Node(
            package='auv_controller',
            executable='XY_stabilizer_node',
            name='XY_stabilizer_node',
            output='screen',
        ),
    ])
