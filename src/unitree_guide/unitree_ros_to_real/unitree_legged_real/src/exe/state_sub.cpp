#include <unitree_legged_msgs/msg/low_cmd.hpp>
#include <unitree_legged_msgs/msg/low_state.hpp>
#include <unitree_legged_msgs/msg/high_cmd.hpp>
#include <unitree_legged_msgs/msg/high_state.hpp>
#include <unitree_legged_msgs/msg/motor_cmd.hpp>
#include <unitree_legged_msgs/msg/motor_state.hpp>
#include <unitree_legged_msgs/msg/bms_cmd.hpp>
#include <unitree_legged_msgs/msg/bms_state.hpp>
#include <unitree_legged_msgs/msg/imu.hpp>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <rclcpp/rclcpp.hpp>

using namespace UNITREE_LEGGED_SDK;

void highStateCallback(const unitree_legged_msgs::msg::HighState::SharedPtr msg)
{
    printf("yaw = %f\n", msg->imu.rpy[2]);
}

void lowStateCallback(const unitree_legged_msgs::msg::LowState::SharedPtr msg)
{
    printf("FR_2_pos = %f\n", msg->motor_state[FR_2].q);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_high_state_sub");

    unitree_legged_msgs::msg::HighState high_state_ros;

    auto high_sub = node->create_subscription<unitree_legged_msgs::msg::HighState>(
        "high_state", 1, highStateCallback);
    auto low_sub = node->create_subscription<unitree_legged_msgs::msg::LowState>(
        "low_state", 1, lowStateCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}