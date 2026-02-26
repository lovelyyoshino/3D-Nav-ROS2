/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "joint_controller.h"
#include <pluginlib/class_list_macros.hpp>

// #define rqtTune // use rqt or not

namespace unitree_legged_control
{

    UnitreeJointController::UnitreeJointController(){
        memset(&lastCmd, 0, sizeof(unitree_legged_msgs::msg::MotorCmd));
        memset(&lastState, 0, sizeof(unitree_legged_msgs::msg::MotorState));
        memset(&servoCmd, 0, sizeof(ServoCmd));
    }

    UnitreeJointController::~UnitreeJointController(){}

    void UnitreeJointController::setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if(isHip) sensor_torque = msg->wrench.torque.x;
        else sensor_torque = msg->wrench.torque.y;
    }

    void UnitreeJointController::setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg)
    {
        lastCmd.mode = msg->mode;
        lastCmd.q = msg->q;
        lastCmd.kp = msg->kp;
        lastCmd.dq = msg->dq;
        lastCmd.kd = msg->kd;
        lastCmd.tau = msg->tau;
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        command.writeFromNonRT(lastCmd);
    }

    controller_interface::InterfaceConfiguration UnitreeJointController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(joint_name + "/effort");
        return config;
    }

    controller_interface::InterfaceConfiguration UnitreeJointController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.push_back(joint_name + "/position");
        config.names.push_back(joint_name + "/velocity");
        config.names.push_back(joint_name + "/effort");
        return config;
    }

    // Controller initialization in non-realtime
    controller_interface::CallbackReturn UnitreeJointController::on_init()
    {
        isHip = false;
        isThigh = false;
        isCalf = false;
        sensor_torque = 0;

        try {
            auto_declare<std::string>("joint", std::string(""));
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        joint_name = get_node()->get_parameter("joint").as_string();
        if (joint_name.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "No joint given (namespace: '%s')", get_node()->get_namespace());
            return controller_interface::CallbackReturn::ERROR;
        }
        name_space = get_node()->get_namespace();

        // load pid param from yaml only if rqt need
#ifdef rqtTune
        pid_controller_.initPid(0, 0, 0, 0, 0);
#endif

        urdf::Model urdf; // Get URDF info about joint
        auto robot_description = get_node()->get_parameter("robot_description").as_string();
        if (!urdf.initString(robot_description)){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
            return controller_interface::CallbackReturn::ERROR;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf){
            RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        if(joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint"){
            isHip = true;
        }
        if(joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint"){
            isCalf = true;
        }

        // Start command subscriber
        // Use node name as topic prefix so each controller gets its own
        // command/state topics: /a1_gazebo/FR_thigh_controller/command, etc.
        std::string node_name = get_node()->get_name();
        sub_ft = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
            node_name + "/joint_wrench", 1,
            std::bind(&UnitreeJointController::setTorqueCB, this, std::placeholders::_1));
        sub_cmd = get_node()->create_subscription<unitree_legged_msgs::msg::MotorCmd>(
            node_name + "/command", 20,
            std::bind(&UnitreeJointController::setCommandCB, this, std::placeholders::_1));

        // Start realtime state publisher
        auto state_pub = get_node()->create_publisher<unitree_legged_msgs::msg::MotorState>(
            node_name + "/state", 1);
        controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<unitree_legged_msgs::msg::MotorState>>(state_pub);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    void UnitreeJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    // Controller startup in realtime
    controller_interface::CallbackReturn UnitreeJointController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Zero gains so robot is limp on spawn (like original ROS1 starting()).
        // Actual joint position is read on the first update() call via firstRun flag,
        // because state_interfaces may not have valid data yet here.
        lastCmd.mode = PMSM;
        lastCmd.q = 0;
        lastCmd.kp = 0;
        lastCmd.kd = 0;
        lastCmd.dq = 0;
        lastCmd.tau = 0;
        lastState.q = 0;
        lastState.dq = 0;
        lastState.tau_est = 0;
        firstRun = true;
        command.initRT(lastCmd);

        pid_controller_.reset();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // Controller update loop in realtime
    controller_interface::return_type UnitreeJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        double currentPos, currentVel, calcTorque;

        // On first update, seed lastState with actual joint position
        // (state_interfaces may not be valid during on_activate).
        if(firstRun){
            double initPos = state_interfaces_[0].get_value();
            lastState.q = initPos;
            lastState.dq = 0;
            // Also update the RT command so the robot holds current position with zero gains
            lastCmd.q = initPos;
            command.writeFromNonRT(lastCmd);
            firstRun = false;
        }

        lastCmd = *(command.readFromRT());

        // set command data
        if(lastCmd.mode == PMSM) {
            servoCmd.pos = lastCmd.q;
            positionLimits(servoCmd.pos);
            servoCmd.posStiffness = lastCmd.kp;
            if(fabs(lastCmd.q - PosStopF) < 0.00001){
                servoCmd.posStiffness = 0;
            }
            servoCmd.vel = lastCmd.dq;
            velocityLimits(servoCmd.vel);
            servoCmd.velStiffness = lastCmd.kd;
            if(fabs(lastCmd.dq - VelStopF) < 0.00001){
                servoCmd.velStiffness = 0;
            }
            servoCmd.torque = lastCmd.tau;
            effortLimits(servoCmd.torque);
        }
        if(lastCmd.mode == BRAKE) {
            servoCmd.posStiffness = 0;
            servoCmd.vel = 0;
            servoCmd.velStiffness = 20;
            servoCmd.torque = 0;
            effortLimits(servoCmd.torque);
        }

        // rqt set P D gains
#ifdef rqtTune
            double i, i_max, i_min;
            getGains(servoCmd.posStiffness,i,servoCmd.velStiffness,i_max,i_min);
#endif

        currentPos = state_interfaces_[0].get_value(); // position
        currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.seconds());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);
        effortLimits(calcTorque);

        command_interfaces_[0].set_value(calcTorque); // effort

        lastState.q = currentPos;
        lastState.dq = currentVel;
        lastState.tau_est = state_interfaces_[2].get_value(); // effort

        // publish state
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tau_est = lastState.tau_est;
            controller_state_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }

    // Controller stopping in realtime
    controller_interface::CallbackReturn UnitreeJointController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void UnitreeJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void UnitreeJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void UnitreeJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerInterface);
