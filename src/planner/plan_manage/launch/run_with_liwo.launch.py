import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- 参数定义 ---
    map_size_x = LaunchConfiguration('map_size_x', default=50.0)
    map_size_y = LaunchConfiguration('map_size_y', default=50.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)

    # Gazebo 仿真: odom_topic=/Odometry_gazebo cloud_topic=/livox/Pointcloud2 frame_id=odom
    # X-FAST_LIWO: odom_topic=x_fast_liwo/lio_odom cloud_topic=x_fast_liwo/world_cloud frame_id=map
    odom_topic = LaunchConfiguration('odom_topic', default='/Odometry_gazebo')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/livox/Pointcloud2')
    frame_id = LaunchConfiguration('frame_id', default='odom')

    max_vel = LaunchConfiguration('max_vel', default=1.0)
    max_acc = LaunchConfiguration('max_acc', default=2.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.0)

    # --- 声明参数 ---
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)
    cloud_topic_cmd = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic)
    frame_id_cmd = DeclareLaunchArgument('frame_id', default_value=frame_id)
    max_vel_cmd = DeclareLaunchArgument('max_vel', default_value=max_vel)
    max_acc_cmd = DeclareLaunchArgument('max_acc', default_value=max_acc)
    planning_horizon_cmd = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon)

    # --- ego_planner 节点（无 drone_0_ 前缀，直接对接 X-FAST_LIWO） ---
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', odom_topic),
            ('/move_base_simple/goal', '/goal_pose'),
            ('planning/bspline', 'planning/bspline'),
            ('planning/data_display', 'planning/data_display'),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            ('grid_map/odom', odom_topic),
            ('grid_map/cloud', cloud_topic),
            ('grid_map/occupancy_inflate', 'grid_map/occupancy_inflate'),
        ],
        parameters=[
            # FSM
            {'fsm/flight_type': 1},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': planning_horizon},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            # 航点（flight_type=1 时由 RViz2 手动指定，这里仅占位）
            {'fsm/waypoint_num': 1},
            {'fsm/waypoint0_x': 0.0},
            {'fsm/waypoint0_y': 0.0},
            {'fsm/waypoint0_z': 1.0},
            # Grid map
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': map_size_x},
            {'grid_map/map_size_y': map_size_y},
            {'grid_map/map_size_z': map_size_z},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.099},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            # 使用点云输入（非深度相机）
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            # 概率更新
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},
            {'grid_map/virtual_ceil_height': 2.9},
            {'grid_map/visualization_truncate_height': 1.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': frame_id},
            # Planner manager
            {'manager/max_vel': max_vel},
            {'manager/max_acc': max_acc},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.4},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': planning_horizon},
            {'manager/use_distinctive_trajs': True},
            {'manager/drone_id': 0},
            # 轨迹优化
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 0.5},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': 0.5},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': max_vel},
            {'optimization/max_acc': max_acc},
            # B-spline
            {'bspline/limit_vel': max_vel},
            {'bspline/limit_acc': max_acc},
            {'bspline/limit_ratio': 1.1},
            # 预测
            {'prediction/obj_num': 0},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0},
        ]
    )

    # --- traj_server 节点 ---
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        remappings=[
            ('position_cmd', 'planning/pos_cmd'),
            ('planning/bspline', 'planning/bspline'),
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    # --- waypoint_generator 节点 ---
    waypoint_generator_node = Node(
        package='waypoint_generator',
        executable='waypoint_generator',
        name='waypoint_generator',
        output='screen',
        remappings=[
            ('odom', odom_topic),
            ('goal', '/goal_pose'),
            ('traj_start_trigger', '/traj_start_trigger'),
        ],
        parameters=[
            {'waypoint_type': 'manual-lonely-waypoint'}
        ]
    )

    # --- RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'liwo.rviz')],
        output='screen',
    )

    # --- 组装 ---
    ld = LaunchDescription()
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(cloud_topic_cmd)
    ld.add_action(frame_id_cmd)
    ld.add_action(max_vel_cmd)
    ld.add_action(max_acc_cmd)
    ld.add_action(planning_horizon_cmd)

    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)
    ld.add_action(waypoint_generator_node)
    ld.add_action(rviz_node)

    return ld
