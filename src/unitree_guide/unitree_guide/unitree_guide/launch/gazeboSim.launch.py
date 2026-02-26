import os
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _kill_leftover_gazebo():
    """Kill all leftover Gazebo/ROS2 processes from previous runs."""
    import time, glob
    # First graceful kill, then force kill
    patterns = [
        'gzclient', 'gzserver', 'gzmaster',
        'spawn_entity', 'unitree_servo', 'state_from_gazebo',
        'controller_manager/spawner',
    ]
    for sig in ['-15', '-9']:
        for pat in patterns:
            try:
                subprocess.run(['pkill', sig, '-f', pat],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        time.sleep(0.3)
    # Clean stale FastRTPS shared memory
    for f in glob.glob('/dev/shm/fastrtps_*'):
        try:
            os.remove(f)
        except Exception:
            pass
    time.sleep(0.5)


def generate_launch_description():
    # Clean up leftover Gazebo processes to prevent port conflicts and hangs
    _kill_leftover_gazebo()
    # --- Fix DDS shared-memory deadlock ---
    # When gzserver loads many ROS2 plugins simultaneously, FastDDS SHM transport
    # can deadlock during discovery. Force UDP-only transport for all nodes.
    ugz_share = get_package_share_directory('unitree_gazebo')
    fastdds_profile = os.path.join(ugz_share, 'config', 'fastdds_no_shm.xml')
    if os.path.isfile(fastdds_profile):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = fastdds_profile

    # Set GAZEBO_PLUGIN_PATH directly in os.environ BEFORE any launch actions,
    # so gzserver.launch.py sees it when it reads os.environ in generate_launch_description()
    livox_share = get_package_share_directory('livox_laser_simulation')
    extra_plugin_paths = os.pathsep.join([
        os.path.join(ugz_share, '..', '..', 'lib'),
        os.path.join(livox_share, '..', '..', 'lib'),
        '/opt/ros/humble/lib',
    ])
    existing = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    os.environ['GAZEBO_PLUGIN_PATH'] = extra_plugin_paths + (os.pathsep + existing if existing else '')

    # Also set GAZEBO_MODEL_PATH directly
    # Include a1_description share parent so Gazebo resolves package://a1_description/...
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join([
        os.path.join(ugz_share, 'models'),
        os.path.dirname(get_package_share_directory('a1_description')),
    ])

    # Pre-generate URDF file for spawn_entity (avoids topic QoS issues)
    a1_share = get_package_share_directory('a1_description')
    xacro_file = os.path.join(a1_share, 'xacro', 'robot.xacro')
    urdf_content = subprocess.check_output(['xacro', xacro_file, 'DEBUG:=false'])
    urdf_tmpfile = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, mode='w')
    urdf_tmpfile.write(urdf_content.decode())
    urdf_tmpfile.close()

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
    unitree_controller_share = FindPackageShare('unitree_controller')

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

    # Launch gzserver directly (bypasses gazebo.launch.py which silently fails)
    # Use wname LaunchConfiguration so user can switch worlds:
    #   wname:=empty  → fast loading (no mesh)
    #   wname:=Building → full environment (slow cold start)
    worlds_dir = os.path.join(ugz_share, 'worlds') + '/'
    gzserver_process = ExecuteProcess(
        cmd=['gzserver',
             [TextSubstitution(text=worlds_dir),
              LaunchConfiguration('wname'),
              TextSubstitution(text='.world')],
             '--verbose', '--pause',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so'],
        output='screen',
        emulate_tty=True,
        additional_env={
            'GAZEBO_PLUGIN_PATH': os.environ['GAZEBO_PLUGIN_PATH'],
            'GAZEBO_MODEL_PATH': os.environ['GAZEBO_MODEL_PATH'],
            'GAZEBO_MASTER_URI': 'http://localhost:11345',
            'GAZEBO_IP_WHITE_LIST': '127.0.0.1',
            'DISPLAY': os.environ.get('DISPLAY', ':1'),
            'FASTRTPS_DEFAULT_PROFILES_FILE': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', ''),
        },
    )

    # Launch gzclient with a long delay to let gzserver fully load the world first.
    # Building world with meshes needs ~15-20s to initialize; starting gzclient too
    # early causes SIGSEGV or resource contention that freezes gzserver.
    gzclient_process = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('gui')),
                additional_env={'GAZEBO_MASTER_URI': 'http://localhost:11345',
                                'GAZEBO_IP_WHITE_LIST': '127.0.0.1'},
            )
        ]
    )

    # Spawn robot entity via file (bypasses topic subscription issues)
    # spawn_entity.py internally polls for /spawn_entity service, so no fixed
    # TimerAction delay needed — just give it a generous timeout for heavy worlds
    # like Building (mesh loading can exceed 2 minutes on cold start).
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-entity', [LaunchConfiguration('rname'), TextSubstitution(text='_gazebo')],
            '-file', urdf_tmpfile.name,
            '-x', '-5',
            '-y', '7',
            '-z', '0.35',
            '-timeout', '300'
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
    # Must be in /a1_gazebo namespace so gazebo_ros2_control can discover it
    # (the plugin looks for robot_state_publisher within its own namespace)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=[LaunchConfiguration('rname'), TextSubstitution(text='_gazebo')],
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

    # Include unitree_controller set_ctrl launch AFTER spawn_entity completes
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

    # Event handler: start controllers only after spawn_entity exits
    spawn_done_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[set_ctrl_launch]
        )
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
        gzserver_process,
        gzclient_process,
        robot_state_publisher_node,
        spawn_entity_node,
        state_from_gazebo_node,
        joy_node,
        spawn_done_handler,
        pointcloud2livox_node
    ])
