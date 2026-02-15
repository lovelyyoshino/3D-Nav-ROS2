from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    unitree_move_base_share = FindPackageShare('unitree_move_base')

    # Move base node with parameter files
    move_base_node = Node(
        package='nav2_bt_navigator',  # ROS2 uses Nav2 instead of move_base
        executable='bt_navigator',
        name='move_base',
        output='screen',
        parameters=[
            PathJoinSubstitution([unitree_move_base_share, 'config', 'costmap_common_params.yaml']),
            PathJoinSubstitution([unitree_move_base_share, 'config', 'local_costmap_params.yaml']),
            PathJoinSubstitution([unitree_move_base_share, 'config', 'global_costmap_params.yaml']),
            PathJoinSubstitution([unitree_move_base_share, 'config', 'base_local_planner_params.yaml'])
        ]
    )

    return LaunchDescription([
        move_base_node
    ])
