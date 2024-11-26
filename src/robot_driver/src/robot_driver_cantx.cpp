#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_interfaces/msg/qt_pub.hpp"

#define POSITION_MODE 0x03
#define SPEED_MODE 0x04

using namespace drivers::socketcan;
using std::placeholders::_1;

// 06 01 04 00 00 00 00 enable
class SocketCanSenderNode : public rclcpp :: Node 
{
    public:
        SocketCanSenderNode(const std::string& node_name) : Node(node_name)
        {
            // receive the cubic polynomials from the robot_control node
            subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("motor_cmd", 10, std::bind(&SocketCanSenderNode::joint_pos_callback, this, _1));
            // receive the qt cmd from the DRobot app
            subscriber_motor_states_ = this->create_subscription<robot_interfaces::msg::QtPub>("motor_states_req", 10, std::bind(&SocketCanSenderNode::motor_states_request_callback, this, _1));
            // publish the joint attitude from the real motors
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SocketCanSenderNode::timer_callback, this));
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
        rclcpp::Subscription<robot_interfaces::msg::QtPub>::SharedPtr subscriber_motor_states_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void joint_pos_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            for (size_t i = 0; i < msg->position.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "joint %lu: pos: %lf", i, msg->position[i]);
                motor_req(i, POSITION_MODE, msg->position[i]);
            }
        }

        void motor_states_request_callback(const robot_interfaces::msg::QtPub::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "working mode: %d", msg->working_mode);
            for (size_t i = 0; i < msg->joint_group_positions.size(); i++) {
                motor_req(i, POSITION_MODE, msg->joint_group_positions[i]);
            }
            gripper_req(msg->gripper_msgs[0], msg->gripper_msgs[1], msg->gripper_msgs[2]);
        }

        void motor_req(uint8_t id, uint8_t motor_mode, const double data) {
            uint8_t tx_data[8] = {0};
            SocketCanSender sender("can0", false);

            uint8_t* data_bytes = reinterpret_cast<uint8_t*>(const_cast<double*>(&data));

            CanId canid(id, 0, FrameType::DATA, ExtendedFrame);
            tx_data[0] = 0x06;
            tx_data[1] = 0x00;
            tx_data[2] = motor_mode;
            tx_data[3] = data_bytes[0];
            tx_data[4] = data_bytes[1];
            tx_data[5] = data_bytes[2];
            tx_data[6] = data_bytes[3];
            sender.send(tx_data, sizeof(tx_data), canid, std::chrono::seconds(1));
        }

        void gripper_req(uint8_t gripper_position, uint8_t gripper_velocity, uint8_t gripper_force) {
            uint8_t tx_data[3] = {0};
            SocketCanSender sender("can0", false);
            CanId canid(0x3F, 0, FrameType::DATA, ExtendedFrame);       // 夹爪ID定为0x3F
            tx_data[0] = gripper_position;
            tx_data[1] = gripper_velocity;
            tx_data[2] = gripper_force;
            sender.send(tx_data, sizeof(tx_data), canid, std::chrono::seconds(1));
        }

        void motor_fdb() {
            // uint8_t tx_data[8] = {0};
            // SocketCanSender sender("can0", false);
            // CanId canid(0x1F, 0, FrameType::DATA, ExtendedFrame);
            // sender.send(tx_data, sizeof(tx_data), canid, std::chrono::seconds(1));
        }

        void timer_callback() {
            motor_fdb();
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketCanSenderNode>("socket_cantx_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
