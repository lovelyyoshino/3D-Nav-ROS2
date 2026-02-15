/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_LEFT  0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_SPACE 0x20

int mode = 1; // pulsed mode or continuous mode

class teleForceCmd
{
public:
    teleForceCmd(rclcpp::Node::SharedPtr node) : node_(node)
    {
        Fx = 0;
        Fy = 0;
        Fz = 0;
        force_pub = node_->create_publisher<geometry_msgs::msg::Wrench>("/apply_force/trunk", 20);
        sleep(1);
        pubForce(Fx, Fy, Fz);
    }
    void keyLoop();
    void pubForce(double x, double y, double z);
private:
    double Fx, Fy, Fz;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub;
    geometry_msgs::msg::Wrench Force;
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("external_force");
    teleForceCmd remote(node);
    signal(SIGINT,quit);
    remote.keyLoop();
    return(0);
}

void teleForceCmd::pubForce(double x, double y, double z)
{
    Force.force.x = Fx;
    Force.force.y = Fy;
    Force.force.z = Fz;
    force_pub->publish(Force);
    rclcpp::spin_some(node_);
}

void teleForceCmd::keyLoop()
{
    char c;
    bool dirty=false;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'Space' to change mode, default is Pulsed mode:");
    puts("Use 'Up/Down/Left/Right' to change direction");
    for(;;){
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }
        RCLCPP_DEBUG(node_->get_logger(), "value: 0x%02X\n", c);
        switch(c){
        case KEYCODE_UP:
            if(mode > 0) {
                Fx = 60;
            } else {
                Fx += 16;
                if(Fx > 220) Fx = 220;
                if(Fx < -220) Fx = -220;
            }
            RCLCPP_INFO(node_->get_logger(), "Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_DOWN:
            if(mode > 0) {
                Fx = -60;
            } else {
                Fx -= 16;
                if(Fx > 220) Fx = 220;
                if(Fx < -220) Fx = -220;
            }
            RCLCPP_INFO(node_->get_logger(), "Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_LEFT:
            if(mode > 0) {
                Fy = 30;
            } else {
                Fy += 8;
                if(Fy > 220) Fy = 220;
                if(Fy < -220) Fy = -220;
            }
            RCLCPP_INFO(node_->get_logger(), "Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_RIGHT:
            if(mode > 0) {
                Fy = -30;
            } else {
                Fy -= 8;
                if(Fy > 220) Fy = 220;
                if(Fy < -220) Fy = -220;
            }
            RCLCPP_INFO(node_->get_logger(), "Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        case KEYCODE_SPACE:
            mode = mode*(-1);
            if(mode > 0){
                RCLCPP_INFO(node_->get_logger(), "Change to Pulsed mode.");
            } else {
                RCLCPP_INFO(node_->get_logger(), "Change to Continuous mode.");
            }
            Fx = 0;
            Fy = 0;
            Fz = 0;
            RCLCPP_INFO(node_->get_logger(), "Fx:%3d   Fy:%3d   Fz:%3d", (int)Fx, (int)Fy, (int)Fz);
            dirty = true;
            break;
        }
        if(dirty == true){
            pubForce(Fx, Fy, Fz);
            if(mode > 0){
                usleep(100000); // 100 ms
                Fx = 0;
                Fy = 0;
                Fz = 0;
                pubForce(Fx, Fy, Fz);
            }
            dirty=false;
        }
    }
    return;
}
