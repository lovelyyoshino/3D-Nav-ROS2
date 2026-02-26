from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rname_arg = DeclareLaunchArgument(
        'rname',
        default_value='a1',
        description='Robot name'
    )

    robot_name = LaunchConfiguration('rname')

    # unitree_servo 桥接节点：转发 IOROS 的 MotorCmd/MotorState 到控制器
    unitree_servo_node = Node(
        package='unitree_controller',
        executable='unitree_servo',
        name='unitree_servo',
        respawn=True,
        parameters=[{'robot_name': robot_name}]
    )

    # ============================================================
    # 三阶段控制器启动策略（解决暂停仿真下控制器激活失败问题）
    #
    # Phase 1: 用 --inactive 加载所有控制器（仿真暂停时可正常 load+configure）
    # Phase 2: unpause 仿真（controller_manager update loop 开始运行）
    # Phase 3: 一次性激活所有控制器（避免部分激活导致机器狗倒塌）
    # ============================================================

    actions = []

    # 所有控制器名称
    all_controller_names = ['joint_state_broadcaster']
    for prefix in ['FL', 'FR', 'RL', 'RR']:
        for joint in ['hip', 'thigh', 'calf']:
            all_controller_names.append(f'{prefix}_{joint}_controller')

    # --- Phase 1: 加载所有控制器（--inactive，不激活）---
    # joint_state_broadcaster 先加载（延迟10s等待 controller_manager 就绪）
    actions.append(
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster',
                               '--controller-manager', '/a1_gazebo/controller_manager',
                               '--controller-manager-timeout', '120',
                               '--inactive'],
                    output='screen',
                )
            ]
        )
    )

    # 12个关节控制器逐个加载，每个间隔1.5秒（仅 load+configure，不激活）
    load_delay = 13.0
    for i, prefix in enumerate(['FL', 'FR', 'RL', 'RR']):
        for j, joint in enumerate(['hip', 'thigh', 'calf']):
            name = f'{prefix}_{joint}_controller'
            actions.append(
                TimerAction(
                    period=load_delay + (i * 3 + j) * 1.5,
                    actions=[
                        Node(
                            package='controller_manager',
                            executable='spawner',
                            arguments=[name,
                                       '--controller-manager', '/a1_gazebo/controller_manager',
                                       '--controller-manager-timeout', '120',
                                       '--inactive'],
                            output='screen',
                        )
                    ]
                )
            )

    # 最后一个控制器在 load_delay + 11*1.5 = 29.5s 开始加载
    # 加载本身需要几秒，预留到 t=38s 确保全部 load+configure 完成

    # --- Phase 2: 先发起激活请求（会阻塞等待 update loop）---
    # switch_controllers 在仿真暂停时会阻塞（因为 update() 不运行）
    # 但激活请求已入队，unpause 后第一个 update 周期就会完成激活
    # 这样实现"零延迟"激活——unpause 和控制器激活在同一个 update 周期完成
    activate_time = 38.0
    actions.append(
        TimerAction(
            period=activate_time,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'switch_controllers',
                         '--controller-manager', '/a1_gazebo/controller_manager',
                         '--activate'] + all_controller_names,
                    output='screen',
                )
            ]
        )
    )

    # --- Phase 3: unpause 仿真（1秒后，确保激活请求已入队）---
    actions.append(
        TimerAction(
            period=activate_time + 1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call',
                         '/unpause_physics', 'std_srvs/srv/Empty'],
                    output='screen',
                )
            ]
        )
    )

    # --- Phase 4: 启动 servo 节点 ---
    # on_activate now holds position, so we only need a short delay
    # for controllers to fully activate after unpause
    actions.append(
        TimerAction(
            period=activate_time + 3.0,
            actions=[unitree_servo_node]
        )
    )

    return LaunchDescription([rname_arg] + actions)
