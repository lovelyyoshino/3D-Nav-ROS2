#!/bin/bash
set -e

# 默认参数（可通过命令行覆盖，如: bash auto.sh empty false）
WORLD_NAME="${1:-Building}"
GUI="${2:-true}"

echo "=== Step 1: 清理残留进程 ==="
# 必须包含所有 launch 产生的进程，否则僵尸进程会吃光 CPU/内存
KILL_PATTERN="gzclient|gzserver|spawn_entity|controller_manager/spawner|state_from_gazebo|unitree_servo|pointcloud2livox|virtual_joy|joy_node|robot_state_publisher|run_mapping_online|junior_ctrl"
pkill -f "$KILL_PATTERN" 2>/dev/null || true
sleep 0.5
pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
sleep 0.5
# 清理 FastRTPS 共享内存残留（防止 DDS 死锁）
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
echo "Cleanup done."

# Step 2: 构建工作空间
echo "=== Step 2: 构建 ==="
source ./install/setup.bash
colcon build --packages-select a1_description unitree_legged_control unitree_controller unitree_guide
# 重新 source（确保使用刚编译的版本）
source ./install/setup.bash
echo "Build done."

# Step 3: 启动仿真
echo "=== Step 3: 启动仿真 (world=$WORLD_NAME, gui=$GUI) ==="
sleep 1
ros2 launch unitree_guide gazeboSim.launch.py \
    user_debug:=False rname:=a1 \
    wname:="$WORLD_NAME" gui:="$GUI" &
LAUNCH_PID=$!
echo "Launch started (PID: $LAUNCH_PID)."

# 等待仿真稳定（控制器激活需要~41s + servo standup ~5s）
echo "等待仿真启动完成（约50秒）..."
sleep 50

# Step 4: 启动 X-FAST_LIWO 建图节点
echo "=== Step 4: 启动 X-FAST_LIWO 建图 ==="
ros2 launch x_fast_liwo mapping_mid360_launch.py &
LIWO_PID=$!
if [ $? -eq 0 ]; then
    echo "Successfully launched X-FAST_LIWO mapping (PID: $LIWO_PID)."
else
    echo "Failed to launch X-FAST_LIWO mapping. Exiting."
    exit 1
fi

# Step 5: 启动 junior_ctrl 控制器（前台运行，接收键盘输入）
echo "=== Step 5: 启动 junior_ctrl（前台，按 2 站立，按 4 行走）==="
sleep 5
# ros2 run unitree_guide junior_ctrl
