/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include "body.h"

using namespace std;
using namespace unitree_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(rclcpp::Node::SharedPtr node, string rname){
        robot_name = rname;
        imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
            "/trunk_imu", 1, std::bind(&multiThread::imuCallback, this, std::placeholders::_1));
        footForce_sub[0] = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FR_foot_contact/the_force", 1, std::bind(&multiThread::FRfootCallback, this, std::placeholders::_1));
        footForce_sub[1] = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FL_foot_contact/the_force", 1, std::bind(&multiThread::FLfootCallback, this, std::placeholders::_1));
        footForce_sub[2] = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RR_foot_contact/the_force", 1, std::bind(&multiThread::RRfootCallback, this, std::placeholders::_1));
        footForce_sub[3] = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RL_foot_contact/the_force", 1, std::bind(&multiThread::RLfootCallback, this, std::placeholders::_1));
        servo_sub[0] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_hip_controller/state", 1, std::bind(&multiThread::FRhipCallback, this, std::placeholders::_1));
        servo_sub[1] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, std::bind(&multiThread::FRthighCallback, this, std::placeholders::_1));
        servo_sub[2] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FR_calf_controller/state", 1, std::bind(&multiThread::FRcalfCallback, this, std::placeholders::_1));
        servo_sub[3] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_hip_controller/state", 1, std::bind(&multiThread::FLhipCallback, this, std::placeholders::_1));
        servo_sub[4] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, std::bind(&multiThread::FLthighCallback, this, std::placeholders::_1));
        servo_sub[5] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/FL_calf_controller/state", 1, std::bind(&multiThread::FLcalfCallback, this, std::placeholders::_1));
        servo_sub[6] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_hip_controller/state", 1, std::bind(&multiThread::RRhipCallback, this, std::placeholders::_1));
        servo_sub[7] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, std::bind(&multiThread::RRthighCallback, this, std::placeholders::_1));
        servo_sub[8] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RR_calf_controller/state", 1, std::bind(&multiThread::RRcalfCallback, this, std::placeholders::_1));
        servo_sub[9] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_hip_controller/state", 1, std::bind(&multiThread::RLhipCallback, this, std::placeholders::_1));
        servo_sub[10] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, std::bind(&multiThread::RLthighCallback, this, std::placeholders::_1));
        servo_sub[11] = node->create_subscription<unitree_legged_msgs::msg::MotorState>(
            "/" + robot_name + "_gazebo/RL_calf_controller/state", 1, std::bind(&multiThread::RLcalfCallback, this, std::placeholders::_1));
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        lowState.imu.quaternion[0] = msg->orientation.w;
        lowState.imu.quaternion[1] = msg->orientation.x;
        lowState.imu.quaternion[2] = msg->orientation.y;
        lowState.imu.quaternion[3] = msg->orientation.z;

        lowState.imu.gyroscope[0] = msg->angular_velocity.x;
        lowState.imu.gyroscope[1] = msg->angular_velocity.y;
        lowState.imu.gyroscope[2] = msg->angular_velocity.z;

        lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg->linear_acceleration.z;

    }

    void FRhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[0].mode = msg->mode;
        lowState.motor_state[0].q = msg->q;
        lowState.motor_state[0].dq = msg->dq;
        lowState.motor_state[0].tau_est = msg->tau_est;
    }

    void FRthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[1].mode = msg->mode;
        lowState.motor_state[1].q = msg->q;
        lowState.motor_state[1].dq = msg->dq;
        lowState.motor_state[1].tau_est = msg->tau_est;
    }

    void FRcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[2].mode = msg->mode;
        lowState.motor_state[2].q = msg->q;
        lowState.motor_state[2].dq = msg->dq;
        lowState.motor_state[2].tau_est = msg->tau_est;
    }

    void FLhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[3].mode = msg->mode;
        lowState.motor_state[3].q = msg->q;
        lowState.motor_state[3].dq = msg->dq;
        lowState.motor_state[3].tau_est = msg->tau_est;
    }

    void FLthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[4].mode = msg->mode;
        lowState.motor_state[4].q = msg->q;
        lowState.motor_state[4].dq = msg->dq;
        lowState.motor_state[4].tau_est = msg->tau_est;
    }

    void FLcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[5].mode = msg->mode;
        lowState.motor_state[5].q = msg->q;
        lowState.motor_state[5].dq = msg->dq;
        lowState.motor_state[5].tau_est = msg->tau_est;
    }

    void RRhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[6].mode = msg->mode;
        lowState.motor_state[6].q = msg->q;
        lowState.motor_state[6].dq = msg->dq;
        lowState.motor_state[6].tau_est = msg->tau_est;
    }

    void RRthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[7].mode = msg->mode;
        lowState.motor_state[7].q = msg->q;
        lowState.motor_state[7].dq = msg->dq;
        lowState.motor_state[7].tau_est = msg->tau_est;
    }

    void RRcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[8].mode = msg->mode;
        lowState.motor_state[8].q = msg->q;
        lowState.motor_state[8].dq = msg->dq;
        lowState.motor_state[8].tau_est = msg->tau_est;
    }

    void RLhipCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        start_up = false;
        lowState.motor_state[9].mode = msg->mode;
        lowState.motor_state[9].q = msg->q;
        lowState.motor_state[9].dq = msg->dq;
        lowState.motor_state[9].tau_est = msg->tau_est;
    }

    void RLthighCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[10].mode = msg->mode;
        lowState.motor_state[10].q = msg->q;
        lowState.motor_state[10].dq = msg->dq;
        lowState.motor_state[10].tau_est = msg->tau_est;
    }

    void RLcalfCallback(const unitree_legged_msgs::msg::MotorState::SharedPtr msg)
    {
        lowState.motor_state[11].mode = msg->mode;
        lowState.motor_state[11].q = msg->q;
        lowState.motor_state[11].dq = msg->dq;
        lowState.motor_state[11].tau_est = msg->tau_est;
    }

    void FRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.ee_force[0].x = msg->wrench.force.x;
        lowState.ee_force[0].y = msg->wrench.force.y;
        lowState.ee_force[0].z = msg->wrench.force.z;
        lowState.foot_force[0] = msg->wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.ee_force[1].x = msg->wrench.force.x;
        lowState.ee_force[1].y = msg->wrench.force.y;
        lowState.ee_force[1].z = msg->wrench.force.z;
        lowState.foot_force[1] = msg->wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.ee_force[2].x = msg->wrench.force.x;
        lowState.ee_force[2].y = msg->wrench.force.y;
        lowState.ee_force[2].z = msg->wrench.force.z;
        lowState.foot_force[2] = msg->wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        lowState.ee_force[3].x = msg->wrench.force.x;
        lowState.ee_force[3].y = msg->wrench.force.y;
        lowState.ee_force[3].z = msg->wrench.force.z;
        lowState.foot_force[3] = msg->wrench.force.z;
    }

private:
    rclcpp::Subscription<unitree_legged_msgs::msg::MotorState>::SharedPtr servo_sub[12];
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr footForce_sub[4];
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    string robot_name;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("unitree_gazebo_servo");
    g_node = node;

    string robot_name;
    node->declare_parameter<std::string>("robot_name", "");
    node->get_parameter("robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    multiThread listen_publish_obj(node, robot_name);

    usleep(300000); // must wait 300ms, to get first state

    rclcpp::Publisher<unitree_legged_msgs::msg::LowState>::SharedPtr lowState_pub;
    lowState_pub = node->create_publisher<unitree_legged_msgs::msg::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    motion_init();

    while (rclcpp::ok()){
        /*
        control logic
        */
        lowState_pub->publish(lowState);
        sendServoCmd();

    }
    rclcpp::shutdown();
    return 0;
}
