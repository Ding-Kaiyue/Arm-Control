from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_fk = Node (
        package = 'robot_kinematics',
        executable ='robot_fk',
    )
    ld.add_action(robot_fk)
    return ld
