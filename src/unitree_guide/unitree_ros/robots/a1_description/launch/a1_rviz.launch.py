from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    user_debug_arg = DeclareLaunchArgument(
        'user_debug',
        default_value='false',
        description='Debug mode flag'
    )

    # Get package share directory
    a1_description_share = FindPackageShare('a1_description')

    # Robot description parameter
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([a1_description_share, 'xacro', 'robot.xacro']),
            ' DEBUG:=',
            LaunchConfiguration('user_debug')
        ]),
        value_type=str
    )

    # Joint state publisher GUI node
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[{'use_gui': True}]
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 1000.0}
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            a1_description_share,
            'launch',
            'check_joint.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        user_debug_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
