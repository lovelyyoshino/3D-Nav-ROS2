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

rclcpp::Subscription<unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

long high_count = 0;
long low_count = 0;

void highCmdCallback(const unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::msg::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high->publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::msg::LowState low_state_ros;

    low_state_ros = state2rosMsg(custom.low_state);

    pub_low->publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ros_udp");

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        sub_low = node->create_subscription<unitree_legged_msgs::msg::LowCmd>(
            "low_cmd", 1, lowCmdCallback);
        pub_low = node->create_publisher<unitree_legged_msgs::msg::LowState>("low_state", 1);

        LoopFunc loop_udpSend("low_udp_send", 0.002, 3, std::bind(&Custom::lowUdpSend, &custom));
        LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, std::bind(&Custom::lowUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        rclcpp::spin(node);

        // printf("low level runing!\n");
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        sub_high = node->create_subscription<unitree_legged_msgs::msg::HighCmd>(
            "high_cmd", 1, highCmdCallback);
        pub_high = node->create_publisher<unitree_legged_msgs::msg::HighState>("high_state", 1);

        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, std::bind(&Custom::highUdpSend, &custom));
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, std::bind(&Custom::highUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        rclcpp::spin(node);

        // printf("high level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();
    return 0;
}
