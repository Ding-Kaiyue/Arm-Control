import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    # robot_qtrecv_up = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_qtrecv')), 'launch', 'robot_qtrecv.launch.py'))
    # )

    robot_tcp_server_get_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_qtrecv')), 'launch', 'robot_tcp_server.launch.py'))
    )

    robot_functions_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_kinematics')), 'launch', 'robot_func.launch.py'))
    )

    return LaunchDescription ([
        robot_gazebo_up,
        robot_moveit_up,
        robot_state_get_up,
        robot_tcp_server_get_up,
        robot_functions_up
    ])

     