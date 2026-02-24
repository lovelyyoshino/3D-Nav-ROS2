#!/bin/bash
echo "Terminating all related processes..."
# 先 SIGTERM，等1秒，再 SIGKILL 确保杀干净
pkill -f "gazebo|junior_ctrl|gzserver|gzclient|spawn_entity|controller_manager/spawner|state_from_gazebo" 2>/dev/null
sleep 1
pkill -9 -f "gazebo|junior_ctrl|gzserver|gzclient|spawn_entity|controller_manager/spawner|state_from_gazebo" 2>/dev/null
sleep 1
echo "Cleanup done."

# Step 2: 构建工作空间
echo "Building workspace with colcon..."
# cd ../..
# source 到环境中
source ./install/setup.bash

if [ $? -ne 0 ]; then
    echo "Failed to source workspace. Exiting."
    exit 1
fi

colcon build --packages-select unitree_guide
if [ $? -ne 0 ]; then
    echo "colcon build failed. Please check for errors."
    exit 1
fi
echo "colcon build completed successfully."
# Step3: 启动 gazeboSim.launch 和 junior_ctrl
echo "Launching ROS nodes..."

# 启动 gazeboSim.launch
sleep 2s
#将地图模型添加到环境变量中
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix unitree_gazebo --share)/models
ros2 launch unitree_guide gazeboSim.launch.py user_debug:=False rname:=a1 &
LAUNCH_PID=$!
if [ $? -eq 0 ]; then
    echo "Successfully launched gazeboSim.launch (PID: $LAUNCH_PID)."
else
    echo "Failed to launch gazeboSim.launch. Exiting."
    exit 1
fi

echo "Gazebo launched successfully."
