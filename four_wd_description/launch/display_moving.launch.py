#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    # Get path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory('four_wd_description'),
        'urdf',
        'robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Dummy Controller Node
    dummy_controller_node = Node(
        package='four_wd_description',
        executable='dummy_controller.py',
        output='screen'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('four_wd_description'), 'rviz', 'config.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        dummy_controller_node,
        rviz_node
    ])