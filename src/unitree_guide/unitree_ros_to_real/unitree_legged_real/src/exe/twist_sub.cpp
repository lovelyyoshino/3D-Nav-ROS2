#include <rclcpp/rclcpp.hpp>
#include <unitree_legged_msgs/msg/high_cmd.hpp>
#include <unitree_legged_msgs/msg/high_state.hpp>
#include <unitree_legged_msgs/msg/low_cmd.hpp>
#include <unitree_legged_msgs/msg/low_state.hpp>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/msg/twist.hpp>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
rclcpp::Publisher<unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;

long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::msg::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high->publish(high_state_ros);

    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("twist_sub");

    pub_high = node->create_publisher<unitree_legged_msgs::msg::HighState>("high_state", 1);

    sub_cmd_vel = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, std::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, std::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
