from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directories
    unitree_move_base_share = FindPackageShare('unitree_move_base')

    # Include pointCloud2LaserScan launch
    pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                unitree_move_base_share,
                'launch',
                'pointCloud2LaserScan.launch.py'
            ])
        ])
    )

    # Include move_base launch
    move_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                unitree_move_base_share,
                'launch',
                'move_base.launch.py'
            ])
        ])
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            unitree_move_base_share,
            'config',
            'move_base.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        pointcloud_launch,
        move_base_launch,
        rviz_node
    ])
