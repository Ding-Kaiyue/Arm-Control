import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robot_fk_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_kinematics')), 'launch', 'robot_fk.launch.py'))
    )

    robot_ik_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_kinematics')), 'launch', 'robot_ik.launch.py'))
    )

    robot_functions_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_kinematics')), 'launch', 'robot_func.launch.py'))
    )

    return LaunchDescription ([
        robot_fk_up,
        robot_ik_up,
        robot_functions_up
    ])
