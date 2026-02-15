from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    # joint_state_broadcaster — 发布 /joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/a1_gazebo/controller_manager'],
        output='screen',
    )

    # 12 个 UnitreeJointController spawner
    controller_names = []
    for prefix in ['FL', 'FR', 'RL', 'RR']:
        for joint in ['hip', 'thigh', 'calf']:
            controller_names.append(f'{prefix}_{joint}_controller')

    controller_spawners = []
    for name in controller_names:
        controller_spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name,
                           '--controller-manager', '/a1_gazebo/controller_manager'],
                output='screen',
            )
        )

    # 延迟启动控制器 spawner，等待 controller_manager 就绪
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner] + controller_spawners
    )

    # 延迟启动 servo 节点，等待控制器就绪
    delayed_servo = TimerAction(
        period=6.0,
        actions=[unitree_servo_node]
    )

    return LaunchDescription([
        rname_arg,
        delayed_spawners,
        delayed_servo,
    ])
