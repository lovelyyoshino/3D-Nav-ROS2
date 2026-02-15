import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_laser_simulation')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'standardrobots_factory.world'),
        description='Path to world file'
    )

    livox_sensor_arg = DeclareLaunchArgument(
        'livox_sensor',
        default_value=os.path.join(pkg_share, 'urdf', 'livox_mid360.xacro'),
        description='Path to livox sensor xacro'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        arguments=[
            '-entity', 'livox_lidar',
            '-topic', 'sensor_description',
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', LaunchConfiguration('livox_sensor')
            ])
        }],
        remappings=[('robot_description', 'sensor_description')],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'livox_simulation.rviz')],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        livox_sensor_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz,
    ])
