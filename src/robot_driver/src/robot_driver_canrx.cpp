#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define CAN_FDB_ID 0x200

using namespace drivers::socketcan;

class SocketCanReceiverNode : public rclcpp :: Node
{
    public:
        SocketCanReceiverNode(const std::string& node_name) : Node(node_name)
        {
            SocketCanReceiver receiver("can0", false);
            // publish the joint attitude from the real motors
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            // Set CAN Filters
            try {
                receiver.SetCanFilters(filters);
            } catch (const std::runtime_error & e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set filters: %s", e.what());
            }

            // CAN Message Receive
            std::array<uint8_t, 8> data;
            while (rclcpp::ok()) {
                try {
                    CanId can_id = receiver.receive(data, std::chrono::seconds(10));
                    RCLCPP_INFO(this->get_logger(), "Received CAN Data: %d, %d, %d, %d, %d, %d, %d, %d", 
                                                                        data[0], data[1], data[2], data[3], 
                                                                        data[4], data[5], data[6], data[7]);
                    
                } catch (const SocketCanTimeout & e) {
                    RCLCPP_ERROR(this->get_logger(), "Timeout: %s", e.what());
                } catch (const std::runtime_error & e) {
                    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                }
            }
        }
    private:
        std::string filter_str = "0x00000000~0x000000FF";
        SocketCanReceiver::CanFilterList filters = SocketCanReceiver::CanFilterList::ParseFilters(filter_str);
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        // 电机反馈信息处理
        void motor_message_feedback(std::array<uint8_t, 8> data)
        {
            // 之前的同时反馈六个joint消息的代码，需确定反馈协议后更改
            sensor_msgs::msg::JointState joint_state_fdb;
            joint_state_fdb.header.stamp = this->get_clock()->now();
            joint_state_fdb.name.resize(6);
            joint_state_fdb.position.resize(6);

            joint_state_fdb.name[0] = "joint1";
            joint_state_fdb.name[1] = "joint2";
            joint_state_fdb.name[2] = "joint3";
            joint_state_fdb.name[3] = "joint4";
            joint_state_fdb.name[4] = "joint5";
            joint_state_fdb.name[5] = "joint6";

            for (int i = 0; i < 6; i++) {
                float position;
                memcpy(&position, &data[i * 4 + 1], sizeof(float));
                joint_state_fdb.position[i] = (double)position / 57.296;
            }

            RCLCPP_INFO(this->get_logger(), "Received motor_msg: %f, %f, %f, %f, %f, %f", joint_state_fdb.position[0], joint_state_fdb.position[1],
                                                                                        joint_state_fdb.position[2], joint_state_fdb.position[3], 
                                                                                        joint_state_fdb.position[4], joint_state_fdb.position[5]);
            publisher_->publish(joint_state_fdb);
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketCanReceiverNode>("robot_driver_canrx");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
