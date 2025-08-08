from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This launch file is responsible for one thing:
    # launching the motor driver node.

    motor_driver_node = Node(
        package='four_wd_control',
        executable='motor_driver',
        output='screen'
    )

    return LaunchDescription([
        motor_driver_node
    ])