from launch import LaunchDescription 
from launch.actions import LogInfo, TimerAction, RegisterEventHandler
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():

    start_robot_driver = ExecuteProcess (
        cmd=['ros2', 'launch', 'robot_driver', 'robot_driver.launch.py'],
        name='robot_driver',
        output='screen'
    )

    start_robot_description = ExecuteProcess (
        cmd=['ros2', 'launch', 'robot_description', 'robot_display.launch.py'],
        name='robot_description',
        output='screen'
    )

    start_robot_control = ExecuteProcess (  
        cmd=['ros2', 'launch', 'robot_control', 'robot_control.launch.py'],
        name='robot_control',
        output='screen'
    )

    start_robot_moveit_config = ExecuteProcess (
        cmd=['ros2', 'launch', 'robot_config', 'real_moveit_demo.launch.py'],
        name='robot_moveit_config',
        output='screen'
    )

    start_robot_kinematics = ExecuteProcess (
        cmd=['ros2', 'launch', 'robot_kinematics', 'robot_kinematics.launch.py'],
        name='robot_kinematics',
        output='screen'
    )

    log_robot_driver_started = LogInfo(msg='Robot_Driver has started.')
    log_robot_description_started = LogInfo(msg='Robot_Description has started.')
    log_robot_control_started = LogInfo(msg='Robot_Control has started.')
    log_robot_moveit_config_started = LogInfo(msg='MoveIt has started.')


    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_robot_description,
            on_start=[start_robot_moveit_config, log_robot_description_started],
        )
    )

    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_robot_moveit_config,
            on_start=[start_robot_control, log_robot_moveit_config_started],
        )
    )

    close_evt3 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_robot_control,
            on_start=[start_robot_driver, log_robot_control_started],
        )
    )

    close_evt4 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_robot_driver,
            on_start=[TimerAction(period=1.0, actions=[start_robot_kinematics, log_robot_driver_started])],
        )
    )

    ld = LaunchDescription([
        start_robot_description,
        close_evt1,
        close_evt2,
        close_evt3,
        close_evt4,
    ])

    return ld