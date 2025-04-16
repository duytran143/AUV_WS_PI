from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auv_controller',
            executable='joystick_input_node',
            name='joystick_input_node'
        ),
        Node(
            package='auv_controller',
            executable='sensor_node',
            name='sensor_node'
        ),
        Node(
            package='auv_controller',
            executable='command_processor_node',
            name='command_processor_node'
        )
    ])
