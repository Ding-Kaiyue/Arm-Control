#include "robot_interfaces/msg/qt_recv.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <cstring>


class QtCmdPub : public rclcpp :: Node {
    public:
        QtCmdPub(const std::string &node_name) : Node(node_name) {
            publisher_ = this->create_publisher<robot_interfaces::msg::QtRecv>("qt_cmd", 10);

            robot_interfaces::msg::QtRecv qt_cmd_test_msg;

            qt_cmd_test_msg.working_mode = 0x08;
            qt_cmd_test_msg.joint_angles_goal.data = {0, 40, 73, 0, 67, 0};
            qt_cmd_test_msg.arm_pose_goal.position.x = -0.3f;
            qt_cmd_test_msg.arm_pose_goal.position.y = 0.0f;
            qt_cmd_test_msg.arm_pose_goal.position.z = 0.1f;
            qt_cmd_test_msg.arm_pose_goal.orientation.x = 0.0f;
            qt_cmd_test_msg.arm_pose_goal.orientation.y = -1.0f;
            qt_cmd_test_msg.arm_pose_goal.orientation.z = 0.0f;
            qt_cmd_test_msg.arm_pose_goal.orientation.w = 0.0f;
            // qt_cmd_test_msg.end_effector_quat.x = 0.0f;
            // qt_cmd_test_msg.end_effector_quat.y = 0.0f;
            // qt_cmd_test_msg.end_effector_quat.z = 0.0f;
            // qt_cmd_test_msg.end_effector_quat.w = 1.0f;
            publisher_->publish(qt_cmd_test_msg);
        }

    private:
        rclcpp::Publisher<robot_interfaces::msg::QtRecv>::SharedPtr publisher_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QtCmdPub>("qt_cmd_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
