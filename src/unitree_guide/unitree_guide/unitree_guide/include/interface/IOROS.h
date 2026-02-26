/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "rclcpp/rclcpp.hpp"
#include "interface/IOInterface.h"
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <thread>

class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
rclcpp::Node::SharedPtr _node;
rclcpp::Subscription<unitree_legged_msgs::msg::MotorState>::SharedPtr _servo_sub[12];
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _foot_states_sub[4], _base_w_sub, _base_t_sub;
rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr _time_sub;
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
rclcpp::Publisher<unitree_legged_msgs::msg::MotorCmd>::SharedPtr _servo_pub[12];
unitree_legged_msgs::msg::LowCmd _lowCmd;
unitree_legged_msgs::msg::LowState _lowState;
std::string _robot_name;
rclcpp::executors::SingleThreadedExecutor _executor;
std::thread _spin_thread;

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::msg::Imu & msg);

void FRhipCallback(const unitree_legged_msgs::msg::MotorState& msg);
void FRthighCallback(const unitree_legged_msgs::msg::MotorState& msg);
void FRcalfCallback(const unitree_legged_msgs::msg::MotorState& msg);

void FLhipCallback(const unitree_legged_msgs::msg::MotorState& msg);
void FLthighCallback(const unitree_legged_msgs::msg::MotorState& msg);
void FLcalfCallback(const unitree_legged_msgs::msg::MotorState& msg);

void RRhipCallback(const unitree_legged_msgs::msg::MotorState& msg);
void RRthighCallback(const unitree_legged_msgs::msg::MotorState& msg);
void RRcalfCallback(const unitree_legged_msgs::msg::MotorState& msg);

void RLhipCallback(const unitree_legged_msgs::msg::MotorState& msg);
void RLthighCallback(const unitree_legged_msgs::msg::MotorState& msg);
void RLcalfCallback(const unitree_legged_msgs::msg::MotorState& msg);


void timeCallback(const rosgraph_msgs::msg::Clock& msg);
void baseWorldCallback(const nav_msgs::msg::Odometry& msg);
void baseTrunkCallback(const nav_msgs::msg::Odometry& msg);
void FL_footCallback(const nav_msgs::msg::Odometry& msg);
void FR_footCallback(const nav_msgs::msg::Odometry& msg);
void RL_footCallback(const nav_msgs::msg::Odometry& msg);
void RR_footCallback(const nav_msgs::msg::Odometry& msg);
void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS