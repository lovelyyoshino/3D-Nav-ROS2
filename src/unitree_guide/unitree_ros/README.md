# Introduction
Here are the ROS2 simulation packages for Unitree robots, You can load robots and joint controllers in Gazebo, so you can perform low-level control (control the torque, position and angular velocity) of the robot joints. Please be aware that the Gazebo simulation cannot do high-level control, namely walking. Aside from these simulation functions, you can also control your real robots in ROS2 with the [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real) packages. For real robots, you can do high-level and low-level control using our ROS2 packages.

## Packages:
Robot description:
* `a1_description`
* `aliengo_description`
* `aliengoZ1_description`
* `b1_description`
* `b2_description`
* `b2w_description`
* `g1_description`
* `go1_description`
* `go2_description`
* `go2w_description`
* `h1_2_description`
* `h1_description`
* `laikago_description`
* `z1_description`

Robot and joints controller:
* `unitree_controller`
* `z1_controller`

Simulation related:
* `unitree_gazebo`
* `unitree_legged_control`

# Dependencies
* [ROS2](https://docs.ros.org/en/humble/) Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
* [Gazebo](http://gazebosim.org/) (ros-humble-gazebo-ros-pkgs)
* [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real): `unitree_legged_msgs` is a package under [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).
# Build

For ROS2 Humble:
```
sudo apt-get install ros-humble-controller-interface ros-humble-gazebo-ros2-control ros-humble-joint-state-broadcaster ros-humble-effort-controllers ros-humble-joint-trajectory-controller
```

And open the file `unitree_gazebo/worlds/stairs.world`. At the end of the file:
```
<include>
    <uri>model:///home/unitree/ros2_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```
Please change the path of `building_editor_models/stairs` to the real path on your PC.

Then you can use colcon to build:
```
cd ~/ros2_ws
colcon build
```

If you face a dependency problem, you can just run `colcon build` again.

# Detail of Packages
## unitree_legged_control:
It contains the joints controllers for Gazebo simulation, which allows users to control joints with position, velocity and torque. Refer to "[unitree_ros/unitree_controller/src/servo.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/servo.cpp)" for joint control examples in different modes.

## The description of robots:
Namely the description of Go1, A1, Aliengo and Laikago. Each package includes mesh, urdf and xacro files of robot. Take Laikago for example, you can check the model in Rviz by:
```
ros2 launch laikago_description laikago_rviz.launch.py
```

## unitree_gazebo & unitree_controller:
You can launch the Gazebo simulation with the following command:
```
ros2 launch unitree_gazebo normal.launch.py rname:=a1 wname:=stairs
```
Where the `rname` means robot name, which can be `laikago`, `aliengo`, `a1` or `go1`. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `laikago`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

### 1. Stand controller
After launching the gazebo simulation, you can start to control the robot:
```
ros2 run unitree_controller unitree_servo
```

And you can add external disturbances, like a push or a kick:
```
ros2 run unitree_controller unitree_external_force
```
### 2. Position and pose publisher
Here we demonstrated how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

Then run the position and pose publisher in another terminal:
```
ros2 run unitree_controller unitree_move_kinetic
```
The robot will turn around the origin, which is the movement under the world coordinate frame. And inside of the source file [move_publisher.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/move_publisher.cpp), we also provide the method to move using the robot coordinate frame. You can change the value of `def_frame` to `coord::ROBOT` and run `colcon build` again, then the `unitree_move_publisher` will move robot under its own coordinate frame.

## z1_controller

You can launch the z1 Gazebo simulation with the following command:

```
ros2 launch unitree_gazebo z1.launch.py
```

After launching the gazebo simulation, you can start to control the z1 robot by z1_sdk.
see [z1_documentation](https://dev-z1.unitree.com)
