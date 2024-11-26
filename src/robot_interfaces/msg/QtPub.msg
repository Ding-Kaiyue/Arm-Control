uint8 working_mode                      # 机械臂工作模式
bool qt_flag                            # 是否使用上位机控制

# -------------- Cartesian Space Planning --------------
trajectory_msgs/JointTrajectory trajectory         # 关节位置轨迹点个数
trajectory_msgs/JointTrajectoryPoint point   # 关节位置轨迹
uint16 delta_time                       # 插值周期

# ---------------- Joint Space Planning ----------------
float64[] joint_group_positions     # 机械臂目标关节位置
uint8[] gripper_msgs                # 夹爪信息