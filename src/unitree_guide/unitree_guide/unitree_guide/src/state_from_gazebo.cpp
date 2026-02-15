#include "gazebo_msgs/msg/link_states.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <nav_msgs/msg/odometry.hpp>


using namespace std;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robotVelocity_BASE_frame_pub;
rclcpp::Node::SharedPtr g_node;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;
string robot_name = "a1";
nav_msgs::msg::Odometry Odom;
double x=0, y=0, z=0, roll=0, pitch=0, yaw=0;


void callback_BASE(const gazebo_msgs::msg::LinkStates::SharedPtr msg) {
    int index = 0;
    for (auto &linkName : msg->name) {
        if (linkName == robot_name+"_gazebo::base")
            break;
        ++index;
    }
    rclcpp::WallRate rate(500);//延迟至100hz发布，避免重复发布

    //map到odom的tf变换
    tf2::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);
    tf2::Transform transform_odom2map;
    transform_odom2map.setRotation(tf2::Quaternion(qtn.x(),
                                         qtn.y(),
                                         qtn.z(),
                                         qtn.w()));
    transform_odom2map.setOrigin(tf2::Vector3(x,
                                    y,
                                    z));
    // 发布odom到map的tf关系
    geometry_msgs::msg::TransformStamped odom2map_tf;
    odom2map_tf.header.stamp = g_node->now();
    odom2map_tf.header.frame_id = "map";
    odom2map_tf.child_frame_id = "odom";
    odom2map_tf.transform.translation.x = transform_odom2map.getOrigin().x();
    odom2map_tf.transform.translation.y = transform_odom2map.getOrigin().y();
    odom2map_tf.transform.translation.z = transform_odom2map.getOrigin().z();
    odom2map_tf.transform.rotation.x = transform_odom2map.getRotation().x();
    odom2map_tf.transform.rotation.y = transform_odom2map.getRotation().y();
    odom2map_tf.transform.rotation.z = transform_odom2map.getRotation().z();
    odom2map_tf.transform.rotation.w = transform_odom2map.getRotation().w();
    g_tf_broadcaster->sendTransform(odom2map_tf);

    //求变化矩阵的逆解，用于推算map到odom的关系，以便能得到base到map的关系，及
    tf2::Transform transform_map2odom = transform_odom2map.inverse();

    tf2::Vector3 pt_map(msg->pose[index].position.x,msg->pose[index].position.y,msg->pose[index].position.z);
    tf2::Vector3 pt_odom = transform_map2odom * pt_map;

    tf2::Quaternion q_map(msg->pose[index].orientation.x,
                        msg->pose[index].orientation.y,
                        msg->pose[index].orientation.z,
                        msg->pose[index].orientation.w);
    tf2::Quaternion q_odom = transform_map2odom.getRotation() * q_map;

    // 转换为odom的速度关系
    tf2::Vector3 linear_vel(
        msg->twist[index].linear.x,
        msg->twist[index].linear.y,
        msg->twist[index].linear.z);
    tf2::Vector3 transformed_linear_vel = transform_map2odom * linear_vel;

    tf2::Vector3 angular_vel(
        msg->twist[index].angular.x,
        msg->twist[index].angular.y,
        msg->twist[index].angular.z);
    tf2::Vector3 transformed_angular_vel = transform_map2odom * angular_vel;

    //发布base到odom的tf变换
    geometry_msgs::msg::TransformStamped odom2base_tf;
    odom2base_tf.header.stamp = g_node->now();
    odom2base_tf.header.frame_id = "odom";
    odom2base_tf.child_frame_id = "base";
    odom2base_tf.transform.translation.x = pt_odom.x();
    odom2base_tf.transform.translation.y = pt_odom.y();
    odom2base_tf.transform.translation.z = pt_odom.z();
    odom2base_tf.transform.rotation.x = q_odom.x();
    odom2base_tf.transform.rotation.y = q_odom.y();
    odom2base_tf.transform.rotation.z = q_odom.z();
    odom2base_tf.transform.rotation.w = q_odom.w();
    g_tf_broadcaster->sendTransform(odom2base_tf);

    Odom.header.stamp = g_node->now();
    Odom.header.frame_id = "odom";
    Odom.child_frame_id = "base";

    // set the position
    Odom.pose.pose.position.x = pt_odom.x();
    Odom.pose.pose.position.y = pt_odom.y();
    Odom.pose.pose.position.z = pt_odom.z();

    Odom.pose.pose.orientation.w = q_odom.w();
    Odom.pose.pose.orientation.x = q_odom.x();
    Odom.pose.pose.orientation.y = q_odom.y();
    Odom.pose.pose.orientation.z = q_odom.z();


    // set the velocity
    Odom.twist.twist.linear.x = transformed_linear_vel.x();
    Odom.twist.twist.linear.y = transformed_linear_vel.y();
    Odom.twist.twist.linear.z = transformed_linear_vel.z();

    Odom.twist.twist.angular.x = transformed_angular_vel.x();
    Odom.twist.twist.angular.y = transformed_angular_vel.y();
    Odom.twist.twist.angular.z = transformed_angular_vel.z();


    robotVelocity_BASE_frame_pub->publish(Odom);
    rate.sleep();
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("state_from_gazebo");

    if (argc != 7)   // x y z qx qy qz qw
    {
        RCLCPP_ERROR(g_node->get_logger(), "Usage: static_transform_publisher x y z yaw pitch roll");
        return -1;
    }

    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);

    double yaw   = atof(argv[4]);
    double pitch = atof(argv[5]);
    double roll  = atof(argv[6]);

    g_node->declare_parameter<std::string>("robot_name", "a1");
    g_node->get_parameter("robot_name", robot_name);

    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

    auto tfState_BASE_sub = g_node->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/gazebo/link_states", 10, callback_BASE);
    robotVelocity_BASE_frame_pub = g_node->create_publisher<nav_msgs::msg::Odometry>("/Odometry_gazebo", 1);

    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}



