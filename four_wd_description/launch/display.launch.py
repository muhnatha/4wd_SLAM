import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # define pkg_name and path
    pkg_name = 'four_wd_description'
    file_subpath = 'urdf/4wd_properties.urdf.xacro'

    # define launch path
    urdf_launch_path = os.path.join(
        get_package_share_directory('urdf_launch'),
        'launch',
        'display.launch.py'
    )

    # define urdf path
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        file_subpath
    )

    # create IncludeLaunchDescription action
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urdf_launch_path),
        launch_arguments={
            'urdf_package' : pkg_name,
            'urdf_package_path' : urdf_path
        }.items()
    )

    # return launch description
    return LaunchDescription([
        display_launch
    ])