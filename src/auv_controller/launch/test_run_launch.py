from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # 1. sensor_node: run but only log to file, not screen
    sensor_node = Node(
        package='auv_controller',
        executable='sensor_node',
        name='sensor_node',
        output='log',
        emulate_tty=False,
    )

    # 2. joystick_input_node: run but only log to file
    joystick_node = Node(
        package='auv_controller',
        executable='joystick_input_V2_node',
        name='joystick_input_V2_node',
        output='log',
        emulate_tty=False,
    )

    # 3. ROV_stabilizer: only this node prints to screen
    stabilizer_node = Node(
        package='auv_controller',
        executable='ROV_stabilizer_V1_node',
        name='ROV_stabilizer_V1_node',
        output='screen',
        emulate_tty=True,
    )

    # 4. motor_driver_node: run but only log to file
    driver_node = Node(
        package='auv_controller',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='log',
        emulate_tty=False,
    )


    return LaunchDescription([
        # Launch sensor first
        sensor_node,
        # Then joystick
        RegisterEventHandler(
            OnProcessStart(
                target_action=sensor_node,
                on_start=[joystick_node],
            ),
        ),
        # Then stabilizer
        RegisterEventHandler(
            OnProcessStart(
                target_action=joystick_node,
                on_start=[stabilizer_node],
            ),
        ),
        # Finally motor driver
        RegisterEventHandler(
            OnProcessStart(
                target_action=stabilizer_node,
                on_start=[driver_node],
            ),
        ),
    ])
