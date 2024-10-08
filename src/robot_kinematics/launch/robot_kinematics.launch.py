from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_ik = Node (
        package = 'robot_kinematics',
        executable ='robot_kinematics',
    )
    ld.add_action(robot_ik)
    return ld