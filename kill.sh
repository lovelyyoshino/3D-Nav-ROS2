#!/bin/bash
echo "Killing all ROS/Gazebo processes..."
pkill -f "gazebo|junior_ctrl|gzserver|gzclient|spawn_entity|controller_manager/spawner|state_from_gazebo|unitree_servo" 2>/dev/null
sleep 1
pkill -9 -f "gazebo|junior_ctrl|gzserver|gzclient|spawn_entity|controller_manager/spawner|state_from_gazebo|unitree_servo" 2>/dev/null
echo "Done."
