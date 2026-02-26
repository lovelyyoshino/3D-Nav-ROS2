# 3D 导航仿真项目

## 项目介绍
本项目通过优化融合 [PCT-planner](https://github.com/byangw/PCT_planner.git) 和 [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git)（ROS2 版本）两种路径规划算法实现3D导航，并在实际环境与Gazebo仿真环境中得到验证。此项目中使用了 Unitree A1 机器人模型以及强化学习控制器 [chy2948331536/unitree_guide](https://gitee.com/chy2948331536/unitree_guide)，其中控制器和 PCT-planner 需要 CUDA 支持。

## 下载与依赖

### 依赖项安装
1. **libtorch**：LibTorch 版本必须与 CUDA 版本完全匹配，即下载带有 +cu118 后缀的版本。下载 C++ 版本的 [libtorch](https://download.pytorch.org/libtorch/cu118/)。我们选择`libtorch-cxx11-abi-shared-with-deps-2.3.0+cu118.zip`作为下载项目，如果不是cxx11 ABI 版本，则会出现ROS2以及libtorch冲突问题。
2. **PCT-planner**：将其放置在 `src` 文件夹外单独编译。
3. **ego-planner-swarm**（ROS2 版本）：已集成在 `src/planner/` 和 `src/uav_simulator/` 中，无需单独下载。原始仓库参考：[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git)（ros2_version 分支）
4. **Fast-lio**：`.auto.sh` 仿真脚本自动发布 `livox/lidar` 和 `livox/imu` 话题，无需额外配置，但需手动设置 `tf` 坐标关系。
5. **More PCD files**: [https://pan.baidu.com/s/1DnUMtvqcTSCsWxjJAQFnhQ?pwd=cjjj](https://pan.baidu.com/s/1DnUMtvqcTSCsWxjJAQFnhQ?pwd=cjjj)

---

## 安装步骤

### 1. 配置 libtorch 和 CUDA 路径

修改 `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` 中的 `libtorch` 路径和 `CMAKE_CUDA_COMPILER` 路径。

### 2. 安装 ego-planner-swarm

#### 前置条件：切换 DDS 为 cyclonedds（必须，否则运行严重卡顿）
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

#### 安装依赖并编译
ego-planner-swarm 已集成在 `src/` 目录中，直接在工作空间根目录编译即可：
```bash
sudo apt-get install libarmadillo-dev libpcl-dev
colcon build
```
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/b4691d6aa11545709920c5d968d335ab.png)


### 3. 安装 PCT-planner

#### Environment

- Ubuntu >= 20.04
- ROS2 >= Humble with ros-desktop-full installation
- CUDA >= 11.7

#### Python（建议使用虚拟环境）

- Python >= 3.8
- [CuPy](https://docs.cupy.dev/en/stable/install.html) with CUDA >= 11.7
- Open3d

####  Build & Install

```bash
cd PCT_planner/planner/
./build_thirdparty.sh
./build.sh
```
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/b5eafa747b2047c7ac84b74c88eee8a2.png)

---

## 使用说明

### 1. 启动 RL 控制器
由于 RL 控制器需要手柄，因此需先启动虚拟手柄（注意这个不是控制器！！！）：
```bash
# 设置权限信息
# sudo -s 不推荐这个方法，最好使用下面方案
pip install python-uinput  
echo 'KERNEL=="uinput", MODE="0666"' | sudo tee /etc/udev/rules.d/99-uinput.rules
sudo udevadm control --reload-rules
sudo udevadm trigger


source ./install/setup.bash
ros2 run unitree_guide virtual_joy.py
```
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/0f1096904eb9480bb055a1f6d9ecc716.png)


然后启动 Gazebo 仿真环境并运行控制器：
```bash
pip install transforms3d tf_transformations 
sudo apt-get install -y ros-humble-tf-transformations
. auto.sh  # 等待 Unitree A1 机器人展开（第一次需要大概4分钟左右如果不行需要调整timeout，src/unitree_guide/unitree_guide/unitree_guide/launch/gazeboSim.launch.py，或者我建议先运行timeout 300 ros2 launch unitree_guide gazeboSim.launch.py gui:=false 来完成工作）
```
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/4f16b7ec3415416c8ffdfdca3327af4d.png)

```bash
ros2 run unitree_guide junior_ctrl
```
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/8bb9ed248a8d457bb1e25400c69dcaf1.png)


在控制器中：
- 按键 **2**：站立
- 按键 **6**：切换为 RL 模式（此时接收 `cmd_vel` 消息）
- 再次按键 **2**：会闪退，需重新启动控制器

### 2. 启动 ego-planner-swarm
```bash
source ./install/setup.bash
ros2 launch ego_planner rviz.launch.py  # RViz2 可视化
```

新开一个终端，运行规划程序：
```bash
source ./install/setup.bash
# 单机模式
ros2 launch ego_planner single_run_in_sim.launch.py
# 或集群模式
ros2 launch ego_planner swarm.launch.py
# 或大规模集群模式
ros2 launch ego_planner swarm_large.launch.py
```

可选参数：
- `use_mockamap`：地图生成方式，默认 `False`（Random Forest），`True` 使用 mockamap
- `use_dynamic`：是否考虑动力学，默认 `False`
```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```

修改 `single_run_in_sim.launch.py` 文件中的 `flight_type` 参数可切换导航模式：
```python
# 1: 使用 2D Nav Goal 设置目标
DeclareLaunchArgument('flight_type', default_value='1')
# 3: 使用 move_base 的路径
DeclareLaunchArgument('flight_type', default_value='3')
```

### 3. 启动 PCT-planner
进入 `PCT-planner` 文件夹并运行以下命令：
```bash
# 将分层地图可视化在RVIZ中
cd tomography/scripts/ 
python3 tomography.py --scene Building

# 发布plan任务，启用interactive_marker_server
cd planner/scripts/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/media/bigdisk/3D-Nav-ROS2/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
python3 plan.py --scene Building
```
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/71b2629cb08e448ebcb7a2d2e879d875.png)

> ⚠️ **注意**：如果地图配置更改，需重新生成地图，路径规划器才会重新规划路径。

---

## 注意事项
1. 项目代码较为仓促，可能存在不规范或混乱的情况，敬请谅解。
2. 如发现问题或有任何建议，请及时提交 issue，便于改进。
3. 本项目基于 [PCT-planner](https://github.com/byangw/PCT_planner.git)、[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git)（ROS2 版本）和 [unitree_guide](https://gitee.com/chy2948331536/unitree_guide.git) 构建，仅限学习使用，禁止用于商业用途。

---

## 联系方式
- **Bilibili**：[https://space.bilibili.com/29152879](https://space.bilibili.com/29152879)
- **邮箱**：1906570332@qq.com

如果您觉得本项目对您有帮助，欢迎在 Gitee 上给我点个 **star**！谢谢支持！
