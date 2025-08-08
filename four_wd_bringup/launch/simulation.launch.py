import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get paths to packages
    pkg_four_wd_description = get_package_share_directory('four_wd_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')
    pkg_four_wd_bringup = get_package_share_directory('four_wd_bringup')
    pkg_four_wd_control = get_package_share_directory('four_wd_control')

    # Process the URDF file
    robot_description_config = xacro.process_file(
        os.path.join(pkg_four_wd_description, 'urdf', '4wd_properties.urdf.xacro'),
    )
    robot_description = robot_description_config.toxml()
    

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description, '-name', '4wd_robot'],
        output='screen'
    )

    # for ROS2 command
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                get_package_share_directory('four_wd_bringup'), 'config', 'gz_bridge.yaml'
            )
        }],
        output='screen'
    )

    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_four_wd_control, 'launch', 'control.launch.py')
        )
    )

    # The new launch description with event handlers for robust startup
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        gz_ros_bridge,
        robot_controller_launch
    ])