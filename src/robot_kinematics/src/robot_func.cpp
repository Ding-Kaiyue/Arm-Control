#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/qt_recv.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <chrono>

using std::placeholders::_1;

/* 
 * @brief 这是一个没用的注释
 * @details 这是一个没用的类
*/

class RobotFunctions : public rclcpp :: Node
{
    public:
        RobotFunctions(const std::string& node_name) : Node(node_name)
        {
            subscriber_ = this->create_subscription<robot_interfaces::msg::QtRecv>("qt_cmd", 10, std::bind(&RobotFunctions::working_mode_callback, this, _1));
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
        rclcpp::Subscription<robot_interfaces::msg::QtRecv>::SharedPtr subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_state_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
        std::vector<double> joint_group_positions, recorded_joint_values;
        // rclcpp::TimerBase::SharedPtr recording_timer_;
        bool is_recording = false;

        void working_mode_callback(const robot_interfaces::msg::QtRecv::SharedPtr msg)
        {
            switch (msg->working_mode) {
                case 0x01: {
                    if (!is_recording) {
                        RCLCPP_INFO(this->get_logger(), "Start Recording the arm position...");
                        is_recording = true;
                        recorded_joint_values.clear();

                        // 创建订阅器订阅joint_state的消息
                        subscriber_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&RobotFunctions::joint_state_callback, this, _1));
                    }
                    break;
                }
                case 0x02: {
                    if (is_recording) {
                        RCLCPP_INFO(this->get_logger(), "Stop Recording the arm position!"); 
                        subscriber_joint_state_.reset();
                        is_recording = false;
                    }
                    break;
                }
                case 0x03: {
                    RCLCPP_INFO(this->get_logger(), "Reproducing the arm positions...");
                    if (recorded_joint_values.empty()) {
                        RCLCPP_WARN(this->get_logger(), "No recorded positions to reproduce!");
                    } else {
                        joint_group_positions.resize(6);

                        // 首先回到初始姿态
                        for (int i = 0; i < 6; i++) {
                            joint_group_positions[i] = recorded_joint_values[i];
                        }
                        arm->setJointValueTarget(joint_group_positions);
                        arm->move();
                        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                        // 再复现之前轨迹
                        for (size_t i = 6; i < recorded_joint_values.size() - 12; i = i + 6) {
                            joint_group_positions[0] = recorded_joint_values[i];
                            joint_group_positions[1] = recorded_joint_values[i + 1];
                            joint_group_positions[2] = recorded_joint_values[i + 2];
                            joint_group_positions[3] = recorded_joint_values[i + 3];
                            joint_group_positions[4] = recorded_joint_values[i + 4];
                            joint_group_positions[5] = recorded_joint_values[i + 5];
                            arm->setJointValueTarget(joint_group_positions);
                            arm->move();
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        }                        
                    }
                    msg->working_mode = 0x00;
                    break;
                }
                case 0x04: {
                    RCLCPP_INFO(this->get_logger(), "Enable the arm robot.");
                    msg->working_mode = 0x00;
                    break;
                }
                case 0x05: {
                    RCLCPP_INFO(this->get_logger(), "Disable the arm robot.");
                    msg->working_mode = 0x00;
                    break;
                }
                case 0x06: {
                    RCLCPP_INFO(this->get_logger(), "Emergency Stop the arm robot.");
                    msg->working_mode = 0x00;
                    break;
                }
                case 0x07: {
                    geometry_msgs::msg::Pose target_pose = msg->arm_pose_goal;

                    // 获取当前状态
                    moveit::core::RobotStatePtr current_state = arm->getCurrentState();
                    std::vector<double> joint_values;
                    current_state->copyJointGroupPositions(arm->getCurrentState()->getRobotModel()->getJointModelGroup("arm"), joint_values);

                    // 打印当前关节值
                    RCLCPP_INFO(this->get_logger(), "Current Joint Values: [%f, %f, %f, %f, %f, %f]", 
                        joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4], joint_values[5]);

                    arm->setStartStateToCurrentState();
                    arm->setPoseTarget(target_pose, arm->getEndEffectorLink());

                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    moveit::core::MoveItErrorCode success = arm->plan(plan);

                    RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success ? "SUCCEED" : "FAILED");

                    if (success) {
                        arm->execute(plan);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "No valid plan found! ");
                    }
                    break;
                }
                case 0x08:
                {
                    RCLCPP_INFO(this->get_logger(), "Start robot_fk mode");
                    if (msg->joint_angles_goal.data.size() < 6) {
                        RCLCPP_WARN(this->get_logger(), "Received joint goal with incorrect size: %zu", msg->joint_angles_goal.data.size());
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "received joint goal: %d, %d, %d, %d, %d, %d", 
                                                                        msg->joint_angles_goal.data[0], msg->joint_angles_goal.data[1], 
                                                                        msg->joint_angles_goal.data[2], msg->joint_angles_goal.data[3], 
                                                                        msg->joint_angles_goal.data[4], msg->joint_angles_goal.data[5]);
                    // 暂时只控制六个关节的值，不控制夹爪的值
                    joint_group_positions.resize(6);
                    for (size_t i = 0; i < 6; i++) {
                        joint_group_positions[i] = static_cast<double>(msg->joint_angles_goal.data[i] * M_PI / 180.0);
                    }

                    // 写入关节值
                    arm->setJointValueTarget(joint_group_positions);
                    // 运动规划
                    arm->move();
                    rclcpp::sleep_for(std::chrono::seconds(1));
                    break;
                }
                default: {
                    break;
                }
            }
        }

        void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // moveit::core::RobotStatePtr current_state = arm->getCurrentState();
            std::vector<double> joint_values;
            // current_state->copyJointGroupPositions(arm->getCurrentState()->getRobotModel()->getJointModelGroup("arm"), joint_values);
            for (size_t i = 0; i < 6; i++) {
                joint_values[i] = msg->position[i];
                recorded_joint_values.push_back(joint_values[i]);
            }
            // recorded_joint_values.push_back(joint_values[0]);
            // recorded_joint_values.push_back(joint_values[1]);
            // recorded_joint_values.push_back(joint_values[2]);
            // recorded_joint_values.push_back(joint_values[3]);
            // recorded_joint_values.push_back(joint_values[4]);
            // recorded_joint_values.push_back(joint_values[5]);
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
