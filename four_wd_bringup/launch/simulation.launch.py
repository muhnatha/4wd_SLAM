import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get paths to packages
    pkg_four_wd_description = get_package_share_directory('four_wd_description')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_gz_sim')
    pkg_four_wd_bringup = get_package_share_directory('four_wd_bringup')
    
    # Controller config path
    controller_config_path = os.path.join(pkg_four_wd_bringup, 'config', 'controllers.yaml')
    
    # rviz config path
    rviz_config_file = os.path.join(
        pkg_four_wd_bringup, 
        'config', 'lidar_test.rviz'
    )
    # process URDF file
    robot_description_config = xacro.process_file(
        os.path.join(pkg_four_wd_description, 'urdf', '4wd_properties.urdf.xacro'),
        mappings={'controller_config_path': controller_config_path}
    )
    robot_description = robot_description_config.toxml()

    # world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description="Specify world file name. Worlds are searched for in '<prefix>/share/four_wd_bringup/worlds'."
    )
    
    # argument world path
    world_path = PathJoinSubstitution([
        FindPackageShare('four_wd_bringup'),
        'worlds',
        LaunchConfiguration('world')
    ])
    
    # Launch Gazebo conditionally
    # This will launch only if the world argument is 'empty.sdf' (the default)
    gazebo_empty = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('world'), "' == 'empty.sdf'"])
        )
    )

    # This will launch only if a different world is provided
    gazebo_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [TextSubstitution(text='-r -v 4 '), world_path]}.items(),
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration('world'), "' == 'empty.sdf'"])
        )
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
    
    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description, 
                     'use_sim_time': True},
                    controller_config_path],
        output="screen",
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Differential Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
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
    
    # Keyboard Teleop Node
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )


    # The new launch description with event handlers for robust startup
    return LaunchDescription([
        world_arg,          
        gazebo_empty,       
        gazebo_custom, 
        gz_ros_bridge,
        robot_state_publisher,
        spawn_entity,
        controller_manager, 
        joint_state_broadcaster_spawner, 
        diff_drive_controller_spawner,
        teleop_twist_keyboard_node, 
        rviz_node,
    ])