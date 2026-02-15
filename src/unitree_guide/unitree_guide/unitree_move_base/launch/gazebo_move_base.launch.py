from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    wname_arg = DeclareLaunchArgument('wname', default_value='smallRoom')
    rname_arg = DeclareLaunchArgument('rname', default_value='go1')
    paused_arg = DeclareLaunchArgument('paused', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    user_debug_arg = DeclareLaunchArgument('user_debug', default_value='false')

    # Get package share directories
    unitree_move_base_share = FindPackageShare('unitree_move_base')
    unitree_controller_share = FindPackageShare('unitree_controller')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    # Robot description path substitution
    rname = LaunchConfiguration('rname')
    robot_description_share = FindPackageShare(
        [rname, TextSubstitution(text='_description')]
    )

    # Robot description parameter
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([robot_description_share, 'xacro', 'robot.xacro']),
            ' DEBUG:=',
            LaunchConfiguration('user_debug')
        ]),
        value_type=str
    )

    # Include Gazebo empty world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                unitree_move_base_share,
                'worlds',
                [LaunchConfiguration('wname'), TextSubstitution(text='.world')]
            ]),
            'verbose': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('paused'),
            'server_required': 'true'
        }.items()
    )

    # Spawn robot entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-entity', [LaunchConfiguration('rname'), TextSubstitution(text='_gazebo')],
            '-topic', 'robot_description',
            '-z', '0.6',
            '-unpause'
        ]
    )

    # Load joint controller configurations
    # Note: ROS2 uses ros2_control instead of controller_manager
    # This would need to be adapted based on your ros2_control setup

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/joint_states', [
                TextSubstitution(text='/'),
                LaunchConfiguration('rname'),
                TextSubstitution(text='_gazebo/joint_states')
            ])
        ]
    )

    # Include unitree_controller set_ctrl launch
    set_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                unitree_controller_share,
                'launch',
                'set_ctrl.launch.py'
            ])
        ]),
        launch_arguments={'rname': LaunchConfiguration('rname')}.items()
    )

    return LaunchDescription([
        wname_arg,
        rname_arg,
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        user_debug_arg,
        gazebo_launch,
        spawn_entity_node,
        robot_state_publisher_node,
        set_ctrl_launch
    ])
