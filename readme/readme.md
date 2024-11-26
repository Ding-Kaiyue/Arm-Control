# <span style="color:#FCDB72;">功能包说明</span>
## <span style="color:#B885B0;">robot_bringup</span>
该功能包是launch文件的集合，其中带gazebo的是仿真文件
### <span style="color:#8DB732;">robot_gazebo.launch.py</span>
该launch文件的功能:
* 启动gazebo
* gazebo中moveit的配置
* 机械臂末端执行器状态发布<span style="color:#707070">(正式版本将这个launch文件删去)</span>
* 启动TCP服务器<span style="color:#707070">(正式版本将这个launch文件删去)</span>
* 解析上位机发布的命令<span style="color:#707070">(正式版本将这个launch文件删去)</span>

<span style="color:#6687FF;">**TODO**</span>

当前在gazebo中测试上位机的使用，所以把后三个launch放进来,正式版本无需在gazebo中接入上位机

## <span style="color:#B885B0;">robot_cartesian</span>
* 使用rqt向<span style="color:#F38B83;">robot_cartesian_planning_trajectory</span>话题发布robot_interfaces/msg/LineMsg类型的数据，判断执行的Cartesian Space的轨迹是直线或圆，设定运动轨迹进行规划
* 使用rqt向<span style="color:#F38B83;">"if_initial"</span>话题发布std_msgs/msg/Bool类型的数据，为`true`时机械臂姿态恢复为初始姿态，为`false`时机械臂才能执行规划
  
## <span style="color:#B885B0;">robot_config</span>
该功能包是使用moveit_setup_assistant生成的功能包，新添加了gazebo仿真环境(gazebo_moveit_demo.launch.py)和实体机械臂(real_moveit_demo.launch.py)的moveit配置

## <span style="color:#B885B0;">robot_control</span>
定义了两个类：
* cubicSpline类三次样条插值数学方法实现
* RobotControl类应用样条插值法对moveit运动规划中的轨迹进行插值
  * 创建动作服务器`action_server_`, 在动作服务器的`handle_accepted`中创建线程并进行插值，将结果存入vector
  * 创建joint_pos_publisher和20ms定时器State_Timer, 在定时器回调`timer_callback`中依次向<span style="color:#F38B83;">"motor_cmd"</span>话题发布`sensor_msgs/msg/JointState`类型的消息

## <span style="color:#B885B0;">robot_description</span>
该功能包存放机械臂urdf的描述文件及gazebo的urdf描述文件

## <span style="color:#B885B0;">robot_driver</span>

* 订阅<span style="color:#F38B83;">"motor_cmd"</span>话题，发送目标joint位置
* 订阅<span style="color:#F38B83;">“motor_states_req"</span>话题，根据上位机Slider的值设置关节位置和夹爪信息
* 发布<span style="color:#F38b83;">"joint_states"</span>话题，发布真实电机的运动状态信息

<span style="color:#6687FF;">**TODO**</span>

1. 此功能包当前按有stm32转发的功能设计，后续需着重与关节模组对接协议
2. 当前为串口通信，需修改为CAN通信

## <span style="color:#B885B0;">robot_gazebo</span>
<span style="color:#8DB732;">gazebo_robot_description.urdf.xacro</span>
* 该文件在urdf文件的基础上增加了机械臂在gazebo的描述

<span style="color:#8DB732;">robot_gazebo.launch.py</span>
* 该文件是启动gazebo的文件, 同时启动了一些状态反馈节点和action, 该文件写法是固定的

## <span style="color:#B885B0;">robot_interfaces</span>
该功能包是机械臂自定义接口部分
### <span style="color:#8DB732;">ArmState.msg</span>
```
# geometry_msgs/Quaternion end_effector_quat
geometry_msgs/Pose end_effector_pose
```
### <span style="color:#8DB732;">LineMsg.msg</span>
```
uint8 type  # 0: line 1: circle

geometry_msgs/Pose initial_pose     # 0: start line pose  1: start center pose

float32 delta_x                     # delta position with respect to the initial_pose      
float32 delta_y
float32 delta_z
float32 radius                      # 0: without any meaning  1: radius of the circle
```
### <span style="color:#8DB732;">QtPub.msg</span>
```
uint8 working_mode                      # 机械臂工作模式
bool qt_flag                            # 是否使用上位机控制
# -------------- Cartesian Space Planning --------------
trajectory_msgs/JointTrajectory trajectory         # 关节位置轨迹点个数
trajectory_msgs/JointTrajectoryPoint point   # 关节位置轨迹
uint16 delta_time                       # 插值周期

# ---------------- Joint Space Planning ----------------
float64[] joint_group_positions     # 机械臂目标关节位置
```
### <span style="color:#8DB732;">QtRecv.msg</span>
```
uint8 working_mode                      # 机械臂工作模式
bool qt_flag                            # 是否使用上位机控制
std_msgs/Int32MultiArray joint_angles_goal          # 关节目标位置
geometry_msgs/Pose arm_pose_goal                    # 机械臂目标姿态
```

<span style="color:#6687FF;">**TODO**</span>
* 删去ArmState类型消息, 直接使用<span style="color:#F38B83;">geometry_msgs/Pose</span>类型
* QtPub消息类型中可以删去<span style="color:#F38B83;">Cartesian Space Planning</span>部分，因为上位机总是直线运动，不存在三个以上的点供插值；可以删去<span style="color:#F38B83;">qt_flag</span>，用到了QtPub的消息类型则必然为上位机控制
* 同理, QtRecv消息类型中的<span style="color:#F38B83;">qt_flag</span>也可以删去
* 
<span style="color:#6687FF;">**Notice**</span>

* 新增或删除消息类型时，要修改<span style="color:#F38B83;">CMakeLists.txt</span>文件中的<span style="color:#F38B83;">rosidl_generate_interfaces</span>部分

## <span style="color:#B885B0;">robot_kinematics</span>
### <span style="color:#8DB732;">robot_func.cpp</span>
### <span style="color:#8DB732;">robot_state_get.cpp</span>
目前使用的是<span style="color:#F38B83;">ArmState</span>类型的消息, 后续可直接换为<span style="color:#F38B83;">geometry_msgs/Pose</span>类型


