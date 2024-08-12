from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_cartesian = Node (
        package='robot_cartesian',
        executable='robot_cartesian',
    )
    ld.add_action(robot_cartesian)
    return ld