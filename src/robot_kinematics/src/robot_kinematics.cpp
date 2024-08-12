#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
bool initial_flag = false;

class Robot_Kinematics : public rclcpp :: Node
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_goal_pose;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_initial_pose;

    void arm_goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (initial_flag == false) {
            geometry_msgs::msg::Pose target_pose = *msg;

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
        } else {
            RCLCPP_INFO(this->get_logger(), "The arm must be initial pose.");
        }
    }

    void arm_initial_call_back(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            arm->setNamedTarget("home");
            arm->move();
            initial_flag = true;
        } else {
            RCLCPP_INFO(this->get_logger(), "Initial pose not received");
            initial_flag = false;
        }
    }
public:
    Robot_Kinematics(const std::string& node_name) : Node(node_name) {
        subscriber_goal_pose = this->create_subscription<geometry_msgs::msg::Pose>("arm_goal", 10, std::bind(&Robot_Kinematics::arm_goal_callback, this, _1));
        subscriber_initial_pose = this->create_subscription<std_msgs::msg::Bool>("if_initial", 10, std::bind(&Robot_Kinematics::arm_initial_call_back, this, _1));

        arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("arm"));

        arm->setPoseReferenceFrame("base_link");
        arm->allowReplanning(true);
        arm->setGoalPositionTolerance(0.0005);
        arm->setGoalOrientationTolerance(0.0007);

        arm->setMaxVelocityScalingFactor(0.2);
        arm->setMaxAccelerationScalingFactor(0.2);

        // arm->setNamedTarget("home");
        // arm->move();
        sleep(1);
    }
};



int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_Kinematics>("robot_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
