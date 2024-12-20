from launch import LaunchDescription 
from launch.actions import LogInfo, TimerAction, RegisterEventHandler
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
        # 使用 ExecuteProcess 执行启动 robot_gazebo 的命令
        start_gazebo = ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_gazebo', 'robot_gazebo.launch.py'],
            name='robot_gazebo_launch',
            output='screen'
        )
        
        # 使用 ExecuteProcess 执行启动 gazebo_moveit_demo 的命令
        start_moveit = ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_config', 'gazebo_moveit_demo.launch.py'],
            name='gazebo_moveit_demo_launch',
            output='screen'
        )

        start_ik = ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_kinematics', 'robot_ik.launch.py'],
            name='robot_ik_launch',
            output='screen',
        )

        # start_fk = ExecuteProcess(
        #     cmd=['ros2', 'launch', 'robot_kinematics', 'robot_fk.launch.py'],
        #     name='robot_fk_launch',
        #     output='screen',
        # )

        log_gazebo_started = LogInfo(msg='Gazebo has started.')
        log_moveit_started = LogInfo(msg='MoveIt has started.')

        close_evt1 = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=start_gazebo,
                on_start=[start_moveit, log_gazebo_started],
            )
        )

        close_evt2 = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=start_moveit,
                on_start=[TimerAction(period=1.0, actions=[start_ik, log_moveit_started])],
            )
        )
        ld = LaunchDescription([
            start_gazebo,
            close_evt1,
            close_evt2,
        ])

        return ld
        