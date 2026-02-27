# unitree_guide

#### 介绍
gazebo 强化学习部署（ROS2 Humble）

#### 软件架构

```
unitree_guide/
├── models/                  # RL 策略模型（.pt）
├── unitree_guide/           # 主控程序（FSM + RL 推理）
├── unitree_ros/             # Gazebo 仿真（URDF + 控制器插件）
└── unitree_ros_to_real/     # 实机通信接口
```

#### 安装教程

1. 将代码 clone 到 ROS2 工作目录 `/src` 下，目录结构为 `ros2_ws/src/unitree_guide`
2. `cd ros2_ws && colcon build`
3. `source ./install/setup.bash`
4. `cd src/unitree_guide`
5. `chmod 777 ./auto.sh`
6. 修改 `unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp` 中的 `model_path` 为 `models/` 中模型的绝对路径
7. 修改 `unitree_guide/unitree_guide/CMakeLists.txt` 中的 libtorch 路径以及 `CMAKE_CUDA_COMPILER` 路径
8. `./auto.sh`（可在 auto.sh 中修改 robot 名称）
9. `./install/unitree_guide/lib/unitree_guide/junior_ctrl`

#### 键盘控制说明

##### 状态切换按键

| 按键 | 功能 | 说明 |
|------|------|------|
| `1` | PASSIVE（被动） | 关闭所有电机，机器人瘫软倒地。安全模式，用于紧急停止 |
| `2` | FIXEDSTAND（固定站立） | 从被动状态站起来，插值到默认站立姿态。**启动后第一步必须先按2** |
| `3` | FREESTAND（自由站立） | 站立状态下可通过 IJKL 控制身体姿态（俯仰/横滚） |
| `4` | TROTTING（小跑） | 进入小跑步态，通过 WASD 控制移动方向 |
| `5` | MOVE_BASE（导航模式） | 接收 `/cmd_vel` 话题控制（需编译时启用 `COMPILE_WITH_MOVE_BASE`） |
| `6` | RL（强化学习） | 进入 RL 策略控制模式，通过 `/cmd_vel` 话题或手柄控制移动 |
| `8` | STEPTEST（踏步测试） | 原地踏步测试 |
| `9` | SWINGTEST（摆腿测试） | 单腿摆动测试 |
| `0` | BALANCETEST（平衡测试） | 平衡控制测试 |
| `r` / `R` | 重置仿真 | **Gazebo 专用**：重置世界并回到 PASSIVE 状态，机器人翻倒后无需重启 |
| `空格` | 归零 | 清除所有方向输入值 |

##### 运动控制按键

| 按键 | 功能 | 适用模式 |
|------|------|----------|
| `W` / `S` | 前进 / 后退 | TROTTING、FREESTAND |
| `A` / `D` | 左移 / 右移 | TROTTING、FREESTAND |
| `I` / `K` | 俯仰（抬头/低头） | FREESTAND |
| `J` / `L` | 横滚（左倾/右倾） | FREESTAND |

##### 典型操作流程

```
启动 → 按2（站立） → 按4（小跑用WASD移动）或 按6（RL模式）
翻倒 → 按r（重置） → 按2（重新站立）
紧急停止 → 按1（被动模式）
```

##### RL 模式补充说明

- 进入 RL 模式后，机器人由神经网络策略控制
- 通过 `/cmd_vel` 话题发送速度指令（linear.x/y + angular.z）
- 按 `0` 可在预设姿态之间切换
- 按 `2` 退回固定站立，按 `1` 退回被动模式
- 模型文件位于 `models/` 目录：`policy_act_inference_plane.pt`（平地）、`policy_act_inference_stair.pt`（楼梯）

#### 模型说明

有两个版本的模型，平地以及楼梯，楼梯自旋转不行，平地原地旋转可以