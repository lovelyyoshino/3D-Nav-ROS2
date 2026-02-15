from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    ctrl_level_arg = DeclareLaunchArgument(
        'ctrl_level',
        default_value='highlevel',
        description='Control level (highlevel or lowlevel)'
    )

    # ROS UDP node
    ros_udp_node = Node(
        package='unitree_legged_real',
        executable='ros_udp',
        name='node_ros_udp',
        output='screen',
        arguments=[LaunchConfiguration('ctrl_level')],
        parameters=[{'control_level': LaunchConfiguration('ctrl_level')}]
    )

    return LaunchDescription([
        ctrl_level_arg,
        ros_udp_node
    ])
