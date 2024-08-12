#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
#include <iostream>

using std::placeholders::_1;
bool initial_flag = false;

class Robot_Cartesian : public rclcpp :: Node
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_trajectory;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_initial_pose;


    void arm_trajectiry_goal_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string traj = msg->data;
        
        if (traj == "line") {
            geometry_msgs::msg::Pose pose;
            pose.position.x = 0.0;
            pose.position.y = 0.2;
            pose.position.z = 0.25;
            pose.orientation.x = 0.0;
            pose.orientation.y = 1.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 0.0;
            arm->setPoseTarget(pose);
            arm->move();

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose);

            // -------------- line ---------------
            geometry_msgs::msg::Pose wpose = pose;
            wpose.position.y += 0.15;
            waypoints.push_back(wpose);

            wpose.position.x -= 0.15;
            waypoints.push_back(wpose);

            wpose.position.z -= 0.1;
            waypoints.push_back(wpose);

            // -------------- line ---------------


            arm->setStartStateToCurrentState();

            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = 0.0f;     // 路径规划覆盖率
            int maxtries = 100;         // 最大尝试次数
            int attempts = 0;           // 已尝试规划次数
            while (fraction < 1.0 && attempts < maxtries) {
                fraction = arm->computeCartesianPath(waypoints, 0.01, 0.00, trajectory, true);
                attempts++;

                if (attempts % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Still trying after %d attempts...", attempts);
                }
            }
            if (fraction == 1.0 && attempts < maxtries) {
                RCLCPP_INFO(this->get_logger(), "Cartesian path successfully planned and executed after %d attempts.", attempts);
                arm->execute(trajectory);
                RCLCPP_INFO(this->get_logger(), "Cartesian path successfully execution."); 
            }

            else if (fraction < 1.0) {
                RCLCPP_INFO(this->get_logger(), "Cartesian path planning failed after %d attempts. The fraction is %f", attempts, fraction);
            }
        }
        else if (traj == "circle") {
            geometry_msgs::msg::Pose pose;
            pose.position.x = 0.0;
            pose.position.y = 0.2;
            pose.position.z = 0.25;
            pose.orientation.x = 0.0;
            pose.orientation.y = 1.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 0.0;
            arm->setPoseTarget(pose);
            arm->move();

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose);

            // --------- circle ---------
            double centerA = pose.position.x;
            double centerB = pose.position.y;
            double radius = 0.08;

            for (double th = 0; th < 6.28; th += 0.02) {
                geometry_msgs::msg::Pose wpose = pose;
                wpose.position.x = centerA + radius * cos(th);
                wpose.position.y = centerB + radius * sin(th);
                waypoints.push_back(wpose);
            }
            // --------- circle ---------

            arm->setStartStateToCurrentState();

            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = 0.0f;     // 路径规划覆盖率
            int maxtries = 100;         // 最大尝试次数
            int attempts = 0;           // 已尝试规划次数
            while (fraction < 1.0 && attempts < maxtries) {
                fraction = arm->computeCartesianPath(waypoints, 0.01, 0.00, trajectory, true);
                attempts++;

                if (attempts % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Still trying after %d attempts...", attempts);
                }
            }
            if (fraction == 1.0 && attempts < maxtries) {
                RCLCPP_INFO(this->get_logger(), "Cartesian path successfully planned and executed after %d attempts.", attempts);
                arm->execute(trajectory);
                RCLCPP_INFO(this->get_logger(), "Cartesian path successfully execution."); 
            }

            else if (fraction < 1.0) {
                RCLCPP_INFO(this->get_logger(), "Cartesian path planning failed after %d attempts. The fraction is %f", attempts, fraction);
            }
        } 

        else {
            RCLCPP_INFO(this->get_logger(), "Received unknown trajectory goal: %s", msg->data.c_str());
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
    Robot_Cartesian(const std::string& node_name) : Node(node_name) {
        subscriber_trajectory = this->create_subscription<std_msgs::msg::String>("robot_cartesian_planning_trajectory", 10, std::bind(&Robot_Cartesian::arm_trajectiry_goal_callback, this, _1));
        subscriber_initial_pose = this->create_subscription<std_msgs::msg::Bool>("if_initial", 10, std::bind(&Robot_Cartesian::arm_initial_call_back, this, _1));
        arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("arm"));

        arm->setPoseReferenceFrame("base_link"); 
        arm->allowReplanning(true);
        arm->setGoalPositionTolerance(0.0005);
        arm->setGoalOrientationTolerance(0.0007);
        arm->setMaxVelocityScalingFactor(0.2);
        arm->setMaxAccelerationScalingFactor(0.2);

        sleep(1);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_Cartesian>("robot_cartesian");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
