#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/qt_recv.hpp>
#include <robot_interfaces/msg/qt_pub.hpp>
#include <robot_interfaces/msg/arm_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <chrono>

/* 
 * @brief 这是一个没用的注释
 * @details 这是一个没用的类
*/

class RobotFunctions : public rclcpp :: Node
{
    public:
        RobotFunctions(const std::string& node_name) : Node(node_name)
        {
            subscriber_qt_cmd_ = this->create_subscription<robot_interfaces::msg::QtRecv>("qt_cmd", 
                                                            10, std::bind(&RobotFunctions::working_mode_callback,
                                                            this, std::placeholders::_1));
            // 对真实机械臂使用这段实时获取当前关节位置
            // subscriber_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 
            //                                                 10, std::bind(&RobotFunctions::joint_state_callback, 
            //                                                 this, std::placeholders::_1));
            publisher_ = this->create_publisher<robot_interfaces::msg::QtPub>("motor_states_req", 10);
            
            arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("arm"));

            recorded_joint_values.clear();

            // 设置机械臂运动允许的误差
            arm->setGoalJointTolerance(0.0007);
            // 设置机械臂运动参数
            arm->setMaxAccelerationScalingFactor(0.1);
            arm->setMaxVelocityScalingFactor(0.1);
            arm->setPoseReferenceFrame("base_link");
            arm->allowReplanning(true);
            arm->setGoalPositionTolerance(0.0005);
            arm->setGoalOrientationTolerance(0.0007);
            sleep(1);
        }

    private:
        rclcpp::Subscription<robot_interfaces::msg::QtRecv>::SharedPtr subscriber_qt_cmd_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_state_;
        rclcpp::Publisher<robot_interfaces::msg::QtPub>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Subscription<robot_interfaces::msg::ArmState>::SharedPtr subscriber_states_;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
        std::vector<double> joint_group_positions, current_joint_positions;
        std::vector<std::vector<double>> recorded_joint_values;
        bool is_recording = false;

        void working_mode_callback(const robot_interfaces::msg::QtRecv::SharedPtr msg)
        {
            robot_interfaces::msg::QtPub pub;
            switch (msg->working_mode) {
                case 0x01: {
                    if (!is_recording) {
                        RCLCPP_INFO(this->get_logger(), "Start Recording the arm position...");
                        is_recording = true;
                        recorded_joint_values.clear();
                        // 创建订阅器订阅joint_state的消息 使用gazebo才需要这样进行，使用实体机器人时无需创建timer_,订阅joint_states也在构造函数中
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(800), std::bind(&RobotFunctions::timer_callback, this));
                        subscriber_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 
                                                                            10, std::bind(&RobotFunctions::joint_state_callback, 
                                                                            this, std::placeholders::_1));
                        pub.working_mode = 1;
                        publisher_->publish(pub);
                    }
                    break;
                }
                case 0x02: {
                    if (is_recording) {
                        RCLCPP_INFO(this->get_logger(), "Stop Recording the arm position!");                         
                        subscriber_joint_state_.reset();
                        timer_.reset();
                        is_recording = false;
                        pub.working_mode = 2;
                        publisher_->publish(pub);
                    }
                    break;
                }
                case 0x03: {
                    RCLCPP_INFO(this->get_logger(), "Replaying recorded arm positions...");
                    if (recorded_joint_values.empty()) {
                        RCLCPP_WARN(this->get_logger(), "No recorded positions to reproduce!");
                    } else {
                        for (const auto& joint_positions : recorded_joint_values) {                            
                            arm->setJointValueTarget(joint_positions);
                            arm->move();
                            // std::this_thread::sleep_for(std::chrono::milliseconds(800));
                        }                        
                        RCLCPP_INFO(this->get_logger(), "Replaying recorded arm positions completed!");
                    }
                    // 重现轨迹结束后需要将状态转移，不再重复执行
                    // msg->working_mode = 0x00;
                    pub.working_mode = 3;
                    publisher_->publish(pub);
                    break;
                }
                case 0x04: 
                case 0x05:
                case 0x06:
                // case 0x07:
                {
                    pub.working_mode = msg->working_mode;
                    pub.joint_group_positions = {0.0f};
                    publisher_->publish(pub);
                    break;
                }
                case 0x07: {
                    // 暂时只控制六个关节的值，不控制夹爪的值
                    joint_group_positions.resize(6);
                    for (size_t i = 0; i < 6; i++) {
                        joint_group_positions[i] = 0.0f;
                    }
                    // 写入关节值(for gazebo)正式版本删除在gazebo中控制的内容
                    arm->setJointValueTarget(joint_group_positions);
                    arm->move();
                    break;
                }
                case 0x08: {
                    RCLCPP_INFO(this->get_logger(), "Start robot_ik mode");
                    // 由于上位机控制均为直线运动，moveit规划不出来三个以上有效路点，不能进行插值，应直接按照计算得出的结果赋值
                    arm->setStartStateToCurrentState();
                    arm->setPoseTarget(msg->arm_pose_goal, arm->getEndEffectorLink());

                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    moveit::core::MoveItErrorCode success = arm->plan(plan);

                    RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success ? "SUCCEED" : "FAILED");
                    
                    pub.working_mode = msg->working_mode;
                    if (success) { 
                        const trajectory_msgs::msg::JointTrajectoryPoint& last_point = plan.trajectory_.joint_trajectory.points.back();
                        std_msgs::msg::Float64MultiArray inv_kin_msg;
                        inv_kin_msg.data.resize(last_point.positions.size());
                        for (size_t i = 0; i < last_point.positions.size(); i++) {
                            inv_kin_msg.data[i] = last_point.positions[i];
                        }
                        pub.joint_group_positions = inv_kin_msg.data;
                        publisher_->publish(pub);
                        // 此时应该给上位机反馈规划成功标志
                        // ************** For Test **************
                        arm->execute(plan); 
                        // ************** For Test **************
                    } else {
                        RCLCPP_INFO(this->get_logger(), "No valid plan found! ");
                        // 此时应该给上位机反馈规划失败标志
                    }
                    break;
                }
                case 0x09: {
                    RCLCPP_INFO(this->get_logger(), "Start robot_fk mode");
                    if (msg->joint_angles_goal.data.size() < 6) {
                        RCLCPP_WARN(this->get_logger(), "Received joint goal with incorrect size: %zu", msg->joint_angles_goal.data.size());
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "received joint goal: %d, %d, %d, %d, %d, %d", 
                                                                        msg->joint_angles_goal.data[0], msg->joint_angles_goal.data[1], 
                                                                        msg->joint_angles_goal.data[2], msg->joint_angles_goal.data[3], 
                                                                        msg->joint_angles_goal.data[4], msg->joint_angles_goal.data[5]);
                    RCLCPP_INFO(this->get_logger(), "received gripper goal: %d, %d, %d", 
                                                                        msg->gripper_goal.data[0], msg->gripper_goal.data[1], msg->gripper_goal.data[2]);
                    pub.working_mode = msg->working_mode;
                    pub.qt_flag = true;
                    // 暂时只控制六个关节的值，不控制夹爪的值
                    joint_group_positions.resize(6);
                    for (size_t i = 0; i < 6; i++) {
                        joint_group_positions[i] = static_cast<double>(msg->joint_angles_goal.data[i] * M_PI / 180.0);
                    }
                    pub.joint_group_positions = joint_group_positions;
                    publisher_->publish(pub);
                    // ************** For Test **************
                    arm->setJointValueTarget(joint_group_positions);
                    arm->move();
                    // ************** For Test **************
                    break;
                }
                default: {
                    break;
                }
            }
        }

        void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
            current_joint_positions.resize(joint_names.size(), 0.0);
            for (size_t i = 0; i < msg->name.size(); ++i) {
                for (size_t j = 0; j < joint_names.size(); ++j) {
                    if (msg->name[i] == joint_names[j]) {
                        current_joint_positions[j] = msg->position[i];
                        break;
                    }
                }
            }
            // 实体机械臂采集关节电机速度，当速度停止时，将此刻的位置存入其中，此时的current_joint_velocities和current_joint_positions应该是电机反馈的数据
            
        }

        void timer_callback()
        {
            // 卡顿的原因是我给机械臂发布了很多个目标姿态，这些目标姿态是订阅joint_states话题得到的机械臂关节位置
            recorded_joint_values.push_back(current_joint_positions);
        }
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotFunctions>("robot_func");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
