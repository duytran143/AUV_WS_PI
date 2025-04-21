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

        # 2. camera_stream_node
        Node(
            package='auv_controller',
            executable='camera_stream_node',
            name='camera_stream_node',
            output='screen',
        ),

        # 3. gui_command_node
        Node(
            package='auv_controller',
            executable='gui_command_node',
            name='gui_command_node',
            output='screen',
        ),
    ])
