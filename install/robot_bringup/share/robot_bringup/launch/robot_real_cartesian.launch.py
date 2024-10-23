import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_driver'), 'launch', 'robot_driver.launch.py'))
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_description'), 'launch', 'robot_display.launch.py'))
    )

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_control'), 'launch', 'robot_control.launch.py'))
    )

    robot_moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_config'), 'launch', 'real_moveit_demo.launch.py'))
    )

    robot_cartesian_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_cartesian'), 'launch', 'robot_cartesian.launch.py'))
    )
    
    return LaunchDescription([
        robot_driver,
        robot_description,
        robot_control,
        robot_moveit_config,
        robot_cartesian_planning,
    ])