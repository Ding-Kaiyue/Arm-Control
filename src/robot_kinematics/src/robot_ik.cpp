#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "message_filters/subscriber.h"

using std::placeholders::_1;


class Robot_IK : public rclcpp :: Node
{
    public:
        Robot_IK(const std::string &node_name) : Node(node_name) {
            arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("arm"));
            std::string end_effector_link = arm->getEndEffectorLink();

            std::string reference_frame = "base_link";
            arm->setPoseReferenceFrame(reference_frame);

            arm->allowReplanning(true);
            arm->setGoalPositionTolerance(0.0005);
            arm->setGoalOrientationTolerance(0.0007);

            arm->setMaxVelocityScalingFactor(0.2);
            arm->setMaxAccelerationScalingFactor(0.2);

            arm->setNamedTarget("home");
            arm->move();
            sleep(1);

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = 0.2;
            target_pose.position.y = 0.2;
            target_pose.position.z = 0.45;
            target_pose.orientation.x = 0.70692;
            target_pose.orientation.y = 0;
            target_pose.orientation.z = 0;
            target_pose.orientation.w = 0.70729;

            // 设置机械臂当前运动状态为初始状态
            arm->setStartStateToCurrentState();

            // 写入目标位姿
            arm->setPoseTarget(target_pose, end_effector_link);

            // 运动规划, 此时机械臂不会运动
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::core::MoveItErrorCode success = arm->plan(plan);

            // 是否规划成功
            RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success? "SUCCEED" : "FAILED");

            // 运动执行
            if(success) {
                arm->execute(plan);
            }
            
            // subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>("arm_goal", 10, std::bind(&Robot_IK::arm_goal_callback, this, _1));
        }
    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
        // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;

        // void arm_goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        //     geometry_msgs::msg::Pose target_pose = *msg;

        //     arm->setStartStateToCurrentState();
        //     arm->setPoseTarget(target_pose, arm->getEndEffectorLink());

        //     moveit::planning_interface::MoveGroupInterface::Plan plan;
        //     moveit::core::MoveItErrorCode success = arm->plan(plan);

        //     RCLCPP_INFO(this->get_logger(), "Plan (pose goal) %s", success ? "SUCCEED" : "FAILED");

        //     if (success) {
        //         arm->execute(plan);
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "No valid plan found");
        //     }
        // }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_IK>("robot_ik");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
