from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Twist subscriber node
    twist_sub_node = Node(
        package='unitree_legged_real',
        executable='twist_sub',
        name='node_twist_sub',
        output='screen'
    )

    # Keyboard control node
    keyboard_control_node = Node(
        package='unitree_legged_real',
        executable='control_via_keyboard',
        name='node_control_via_keyboard',
        output='screen'
    )

    return LaunchDescription([
        twist_sub_node,
        keyboard_control_node
    ])
