from launch import LaunchDescription 
from launch.actions import LogInfo, TimerAction, RegisterEventHandler
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from  ament_index_python.packages import get_package_share_directory


def generate_launch_description():
        robot_gazebo_up =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_gazebo')), 'launch', 'robot_gazebo.launch.py'))
        )

        robot_moveit_up = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_config')), 'launch', 'gazebo_moveit_demo.launch.py'))
        )

        robot_cartesian_up = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_cartesian')), 'launch', 'robot_cartesian.launch.py'))
        )

        return LaunchDescription ([
            robot_gazebo_up,
            robot_moveit_up,
            robot_cartesian_up
        ])