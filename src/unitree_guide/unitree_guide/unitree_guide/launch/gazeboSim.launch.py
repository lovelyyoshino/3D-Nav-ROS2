import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Set GAZEBO_PLUGIN_PATH directly in os.environ BEFORE any launch actions,
    # so gzserver.launch.py sees it when it reads os.environ in generate_launch_description()
    ugz_share = get_package_share_directory('unitree_gazebo')
    livox_share = get_package_share_directory('livox_laser_simulation')
    extra_plugin_paths = os.pathsep.join([
        os.path.join(ugz_share, '..', '..', 'lib'),
        os.path.join(livox_share, '..', '..', 'lib'),
        '/opt/ros/humble/lib',
    ])
    existing = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    os.environ['GAZEBO_PLUGIN_PATH'] = extra_plugin_paths + (os.pathsep + existing if existing else '')

    # Also set GAZEBO_MODEL_PATH directly
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(ugz_share, 'models')

    # Declare launch arguments
    wname_arg = DeclareLaunchArgument('wname', default_value='Building')
    rname_arg = DeclareLaunchArgument('rname', default_value='a1')
    paused_arg = DeclareLaunchArgument('paused', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    user_debug_arg = DeclareLaunchArgument('user_debug', default_value='false')

    # Get package share directories
    unitree_gazebo_share = FindPackageShare('unitree_gazebo')
    unitree_controller_share = FindPackageShare('unitree_controller')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    # Robot description path
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

    # Include Gazebo launch (server only, we launch gzclient separately without EOL plugin)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                unitree_gazebo_share,
                'worlds',
                [LaunchConfiguration('wname'), TextSubstitution(text='.world')]
            ]),
            'verbose': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('paused'),
            'server_required': 'true'
        }.items()
    )

    # Launch gzclient without the EOL plugin that causes Camera assertion crash
    gzclient_process = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Spawn robot entity (use -file instead of -topic to avoid DDS discovery issues)
    urdf_file = PathJoinSubstitution([robot_description_share, 'urdf', 'a1.urdf'])
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-entity', [LaunchConfiguration('rname'), TextSubstitution(text='_gazebo')],
            '-file', urdf_file,
            '-x', '-5',
            '-y', '7',
            '-z', '0.6',
            '-unpause',
            '-timeout', '120'
        ]
    )

    # State from gazebo node
    state_from_gazebo_node = Node(
        package='unitree_guide',
        executable='state_from_gazebo',
        name='state_from_gazebo',
        output='screen',
        arguments=['0', '9', '0', '0', '0', '0'],
        parameters=[{
            'robot_name': LaunchConfiguration('rname'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

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

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
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

    # Pointcloud to livox node
    pointcloud2livox_node = Node(
        package='unitree_guide',
        executable='pointcloud2livox.py',
        name='pointcloud2livox',
        output='screen',
        parameters=[{
            'laser_blind': 0.5,
            'min_angle': -7.0,
            'max_angle': 55.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
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
        gzclient_process,
        spawn_entity_node,
        state_from_gazebo_node,
        robot_state_publisher_node,
        joy_node,
        set_ctrl_launch,
        pointcloud2livox_node
    ])
