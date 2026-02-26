/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
	RCLCPP_INFO(rclcpp::get_logger("IOROS"), "ROS interface shutting down!");
	rclcpp::shutdown();
}

IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    _node = rclcpp::Node::make_shared("ioros_node");
    _node->declare_parameter<std::string>("robot_name", "a1");
    _node->get_parameter("robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // start subscriber
    initRecv();
    _executor.add_node(_node);
    _spin_thread = std::thread([this]() { _executor.spin(); });
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}

IOROS::~IOROS(){
    delete cmdPanel;
    _executor.cancel();
    if (_spin_thread.joinable()) {
        _spin_thread.join();
    }
    rclcpp::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < 12; ++i){
        _lowCmd.motor_cmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motor_cmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motor_cmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motor_cmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motor_cmd[i].kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motor_cmd[i].kp = lowCmd->motorCmd[i].Kp;
    }
    for(int m(0); m < 12; ++m){
        _servo_pub[m]->publish(_lowCmd.motor_cmd[m]);
    }
}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motor_state[i].q;
        state->motorState[i].dq = _lowState.motor_state[i].dq;
        state->motorState[i].ddq = _lowState.motor_state[i].ddq;
        state->motorState[i].tauEst = _lowState.motor_state[i].tau_est;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend(){
    _servo_pub[0] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 1);
    _servo_pub[1] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 1);
    _servo_pub[2] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 1);
    _servo_pub[3] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 1);
    _servo_pub[4] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 1);
    _servo_pub[5] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 1);
    _servo_pub[6] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 1);
    _servo_pub[7] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 1);
    _servo_pub[8] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 1);
    _servo_pub[9] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 1);
    _servo_pub[10] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 1);
    _servo_pub[11] = _node->create_publisher<unitree_legged_msgs::msg::MotorCmd>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 1);
}

void IOROS::initRecv(){
    _imu_sub = _node->create_subscription<sensor_msgs::msg::Imu>("/trunk_imu", 1, std::bind(&IOROS::imuCallback, this, std::placeholders::_1));
    _servo_sub[0] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FR_hip_controller/state", 1, std::bind(&IOROS::FRhipCallback, this, std::placeholders::_1));
    _servo_sub[1] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 1, std::bind(&IOROS::FRthighCallback, this, std::placeholders::_1));
    _servo_sub[2] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FR_calf_controller/state", 1, std::bind(&IOROS::FRcalfCallback, this, std::placeholders::_1));
    _servo_sub[3] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FL_hip_controller/state", 1, std::bind(&IOROS::FLhipCallback, this, std::placeholders::_1));
    _servo_sub[4] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 1, std::bind(&IOROS::FLthighCallback, this, std::placeholders::_1));
    _servo_sub[5] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/FL_calf_controller/state", 1, std::bind(&IOROS::FLcalfCallback, this, std::placeholders::_1));
    _servo_sub[6] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RR_hip_controller/state", 1, std::bind(&IOROS::RRhipCallback, this, std::placeholders::_1));
    _servo_sub[7] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 1, std::bind(&IOROS::RRthighCallback, this, std::placeholders::_1));
    _servo_sub[8] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RR_calf_controller/state", 1, std::bind(&IOROS::RRcalfCallback, this, std::placeholders::_1));
    _servo_sub[9] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RL_hip_controller/state", 1, std::bind(&IOROS::RLhipCallback, this, std::placeholders::_1));
    _servo_sub[10] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 1, std::bind(&IOROS::RLthighCallback, this, std::placeholders::_1));
    _servo_sub[11] = _node->create_subscription<unitree_legged_msgs::msg::MotorState>("/" + _robot_name + "_gazebo/RL_calf_controller/state", 1, std::bind(&IOROS::RLcalfCallback, this, std::placeholders::_1));
    _foot_states_sub[0] = _node->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/FL_foot", 1, std::bind(&IOROS::FL_footCallback, this, std::placeholders::_1));
    _foot_states_sub[1] = _node->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/FR_foot", 1, std::bind(&IOROS::FR_footCallback, this, std::placeholders::_1));
    _foot_states_sub[2] = _node->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/RL_foot", 1, std::bind(&IOROS::RL_footCallback, this, std::placeholders::_1));
    _foot_states_sub[3] = _node->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/RR_foot", 1, std::bind(&IOROS::RR_footCallback, this, std::placeholders::_1));
    _base_w_sub = _node->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/base_w", 1, std::bind(&IOROS::baseWorldCallback, this, std::placeholders::_1));
    _base_t_sub = _node->create_subscription<nav_msgs::msg::Odometry>("/ground_truth/base_trunk", 1, std::bind(&IOROS::baseTrunkCallback, this, std::placeholders::_1));
    auto clock_qos = rclcpp::QoS(1).best_effort();
    _time_sub = _node->create_subscription<rosgraph_msgs::msg::Clock>("/clock", clock_qos, std::bind(&IOROS::timeCallback, this, std::placeholders::_1));
    joy_sub = _node->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&IOROS::joyCallback, this, std::placeholders::_1));
}

void IOROS::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    axes = msg->axes;
    buttons = msg->buttons;
}

void IOROS::timeCallback(const rosgraph_msgs::msg::Clock& msg) {
    current_time = (msg.clock.sec)*1e6 + (msg.clock.nanosec)/1000;
    // std::cout << "current_time: " << current_time << std::endl;
}

void IOROS::baseWorldCallback(const nav_msgs::msg::Odometry& msg) {
    _base_w_pos[0] = msg.pose.pose.position.x;
    _base_w_pos[1] = msg.pose.pose.position.y;
    _base_w_pos[2] = msg.pose.pose.position.z;
    _base_w_ori[0] = msg.pose.pose.orientation.x;
    _base_w_ori[1] = msg.pose.pose.orientation.y;
    _base_w_ori[2] = msg.pose.pose.orientation.z;
    _base_w_ori[3] = msg.pose.pose.orientation.w;
    _base_w_linear_vel[0] = msg.twist.twist.linear.x;
    _base_w_linear_vel[1] = msg.twist.twist.linear.y;
    _base_w_linear_vel[2] = msg.twist.twist.linear.z;
    _base_w_angular_vel[0] = msg.twist.twist.angular.x;
    _base_w_angular_vel[1] = msg.twist.twist.angular.y;
    _base_w_angular_vel[2] = msg.twist.twist.angular.z;
    // std::cout << "_base_w_angular_vel" << _base_w_angular_vel[0] << " " << _base_w_angular_vel[1] << " " << _base_w_angular_vel[2] << std::endl;
}

void IOROS::baseTrunkCallback(const nav_msgs::msg::Odometry& msg) {
    _base_t_pos[0] = msg.pose.pose.position.x;
    _base_t_pos[1] = msg.pose.pose.position.y;
    _base_t_pos[2] = msg.pose.pose.position.z;
    _base_t_ori[0] = msg.pose.pose.orientation.x;
    _base_t_ori[1] = msg.pose.pose.orientation.y;
    _base_t_ori[2] = msg.pose.pose.orientation.z;
    _base_t_ori[3] = msg.pose.pose.orientation.w;
    _base_t_linear_vel[0] = msg.twist.twist.linear.x;
    _base_t_linear_vel[1] = msg.twist.twist.linear.y;
    _base_t_linear_vel[2] = msg.twist.twist.linear.z;
    _base_t_angular_vel[0] = msg.twist.twist.angular.x;
    _base_t_angular_vel[1] = msg.twist.twist.angular.y;
    _base_t_angular_vel[2] = msg.twist.twist.angular.z;
    // std::cout << "_base_t_angular_vel" << _base_t_angular_vel[0] << " " << _base_t_angular_vel[1] << " " << _base_t_angular_vel[2] << std::endl;
}

void IOROS::FL_footCallback(const nav_msgs::msg::Odometry& msg) {
    // std::cout << "Received FL foot position:" << std::endl;
    _FL_foot_pos[0] = msg.pose.pose.position.x;
    _FL_foot_pos[1] = msg.pose.pose.position.y;
    _FL_foot_pos[2] = msg.pose.pose.position.z;
    _FL_foot_vel[0] = msg.twist.twist.linear.x;
    _FL_foot_vel[1] = msg.twist.twist.linear.y;
    _FL_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::FR_footCallback(const nav_msgs::msg::Odometry& msg) {
    _FR_foot_pos[0] = msg.pose.pose.position.x;
    _FR_foot_pos[1] = msg.pose.pose.position.y;
    _FR_foot_pos[2] = msg.pose.pose.position.z;
    _FR_foot_vel[0] = msg.twist.twist.linear.x;
    _FR_foot_vel[1] = msg.twist.twist.linear.y;
    _FR_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::RL_footCallback(const nav_msgs::msg::Odometry& msg) {
    // std::cout << "Received RL foot position:" << std::endl;
    _RL_foot_pos[0] = msg.pose.pose.position.x;
    _RL_foot_pos[1] = msg.pose.pose.position.y;
    _RL_foot_pos[2] = msg.pose.pose.position.z;
    _RL_foot_vel[0] = msg.twist.twist.linear.x;
    _RL_foot_vel[1] = msg.twist.twist.linear.y;
    _RL_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::RR_footCallback(const nav_msgs::msg::Odometry& msg) {
    // std::cout << "Received RR foot position:" << std::endl;
    _RR_foot_pos[0] = msg.pose.pose.position.x;
    _RR_foot_pos[1] = msg.pose.pose.position.y;
    _RR_foot_pos[2] = msg.pose.pose.position.z;
    _RR_foot_vel[0] = msg.twist.twist.linear.x;
    _RR_foot_vel[1] = msg.twist.twist.linear.y;
    _RR_foot_vel[2] = msg.twist.twist.linear.z;
}

void IOROS::imuCallback(const sensor_msgs::msg::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void IOROS::FRhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[0].mode = msg.mode;
    _lowState.motor_state[0].q = msg.q;
    _lowState.motor_state[0].dq = msg.dq;
    _lowState.motor_state[0].tau_est = msg.tau_est;
}

void IOROS::FRthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[1].mode = msg.mode;
    _lowState.motor_state[1].q = msg.q;
    _lowState.motor_state[1].dq = msg.dq;
    _lowState.motor_state[1].tau_est = msg.tau_est;
}

void IOROS::FRcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[2].mode = msg.mode;
    _lowState.motor_state[2].q = msg.q;
    _lowState.motor_state[2].dq = msg.dq;
    _lowState.motor_state[2].tau_est = msg.tau_est;
}

void IOROS::FLhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[3].mode = msg.mode;
    _lowState.motor_state[3].q = msg.q;
    _lowState.motor_state[3].dq = msg.dq;
    _lowState.motor_state[3].tau_est = msg.tau_est;
}

void IOROS::FLthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[4].mode = msg.mode;
    _lowState.motor_state[4].q = msg.q;
    _lowState.motor_state[4].dq = msg.dq;
    _lowState.motor_state[4].tau_est = msg.tau_est;
}

void IOROS::FLcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[5].mode = msg.mode;
    _lowState.motor_state[5].q = msg.q;
    _lowState.motor_state[5].dq = msg.dq;
    _lowState.motor_state[5].tau_est = msg.tau_est;
}

void IOROS::RRhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[6].mode = msg.mode;
    _lowState.motor_state[6].q = msg.q;
    _lowState.motor_state[6].dq = msg.dq;
    _lowState.motor_state[6].tau_est = msg.tau_est;
}

void IOROS::RRthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[7].mode = msg.mode;
    _lowState.motor_state[7].q = msg.q;
    _lowState.motor_state[7].dq = msg.dq;
    _lowState.motor_state[7].tau_est = msg.tau_est;
}

void IOROS::RRcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[8].mode = msg.mode;
    _lowState.motor_state[8].q = msg.q;
    _lowState.motor_state[8].dq = msg.dq;
    _lowState.motor_state[8].tau_est = msg.tau_est;
}

void IOROS::RLhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[9].mode = msg.mode;
    _lowState.motor_state[9].q = msg.q;
    _lowState.motor_state[9].dq = msg.dq;
    _lowState.motor_state[9].tau_est = msg.tau_est;
}

void IOROS::RLthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[10].mode = msg.mode;
    _lowState.motor_state[10].q = msg.q;
    _lowState.motor_state[10].dq = msg.dq;
    _lowState.motor_state[10].tau_est = msg.tau_est;
}

void IOROS::RLcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
{
    _lowState.motor_state[11].mode = msg.mode;
    _lowState.motor_state[11].q = msg.q;
    _lowState.motor_state[11].dq = msg.dq;
    _lowState.motor_state[11].tau_est = msg.tau_est;
}


#endif  // COMPILE_WITH_ROS