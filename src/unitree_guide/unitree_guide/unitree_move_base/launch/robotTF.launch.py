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
    go1_description_share = FindPackageShare('go1_description')

    # Robot description parameter
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([go1_description_share, 'xacro', 'robot.xacro']),
            ' DEBUG:=',
            LaunchConfiguration('user_debug')
        ]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[
            ('/joint_states', '/realRobot/joint_states')
        ]
    )

    return LaunchDescription([
        user_debug_arg,
        robot_state_publisher_node
    ])
