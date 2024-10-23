import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    robot_gazebo_up =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_gazebo')), 'launch', 'robot_gazebo.launch.py'))
    )

    robot_moveit_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_config')), 'launch', 'gazebo_moveit_demo.launch.py'))
    )

    robot_state_get_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_kinematics')), 'launch', 'robot_state_get.launch.py'))
    )

    robot_qtrecv_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_qtrecv')), 'launch', 'robot_qtrecv.launch.py'))
    )

    robot_functions_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_kinematics')), 'launch', 'robot_func.launch.py'))
    )

    return LaunchDescription ([
        robot_gazebo_up,
        robot_moveit_up,
        robot_state_get_up,
        robot_qtrecv_up,
        robot_functions_up
    ])

   