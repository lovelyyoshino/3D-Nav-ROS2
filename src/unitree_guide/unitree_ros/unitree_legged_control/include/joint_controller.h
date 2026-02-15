/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _UNITREE_ROS_JOINT_CONTROLLER_H_
#define _UNITREE_ROS_JOINT_CONTROLLER_H_

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <control_toolbox/pid.hpp>
#include <memory>
#include <mutex>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <std_msgs/msg/float64.hpp>
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "unitree_joint_control_tool.h"

#define PMSM      (0x0A)
#define BRAKE     (0x00)
#define PosStopF  (2.146E+9f)
#define VelStopF  (16000.0f)

namespace unitree_legged_control
{
    class UnitreeJointController: public controller_interface::ControllerInterface
    {
private:
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft;
        rclcpp::Subscription<unitree_legged_msgs::msg::MotorCmd>::SharedPtr sub_cmd;
        control_toolbox::Pid pid_controller_;
        std::unique_ptr<realtime_tools::RealtimePublisher<unitree_legged_msgs::msg::MotorState>> controller_state_publisher_;

public:
        // bool start_up;
        std::string name_space;
        std::string joint_name;
        float sensor_torque;
        bool isHip, isThigh, isCalf, rqtTune;
        urdf::JointConstSharedPtr joint_urdf;
        realtime_tools::RealtimeBuffer<unitree_legged_msgs::msg::MotorCmd> command;
        unitree_legged_msgs::msg::MotorCmd lastCmd;
        unitree_legged_msgs::msg::MotorState lastState;
        ServoCmd servoCmd;

        UnitreeJointController();
        ~UnitreeJointController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        void setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg);
        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

    };
}

#endif
