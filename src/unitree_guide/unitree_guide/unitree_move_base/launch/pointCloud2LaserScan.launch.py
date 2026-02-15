from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Face pointcloud to laserscan node
    face_pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='face_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cam1/point_cloud_face'),
            ('/scan', '/faceLaserScan')
        ],
        parameters=[{
            'target_frame': 'camera_face',
            'transform_tolerance': 0.01,
            'min_height': -0.05,
            'max_height': 0.1,
            'angle_min': -0.087,
            'angle_max': 0.087,
            'angle_increment': 0.0175,
            'scan_time': 0.3333,
            'range_min': 0.1,
            'range_max': 1.5,
            'use_inf': True,
            'concurrency_level': 1
        }],
        output='screen'
    )

    # Left pointcloud to laserscan node
    left_pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='left_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cam3/point_cloud_left'),
            ('/scan', '/leftLaserScan')
        ],
        parameters=[{
            'target_frame': 'camera_laserscan_link_left',
            'transform_tolerance': 0.01,
            'min_height': -0.05,
            'max_height': 0.1,
            'angle_min': -0.087,
            'angle_max': 0.087,
            'angle_increment': 0.0175,
            'scan_time': 0.3333,
            'range_min': 0.1,
            'range_max': 1.5,
            'use_inf': True,
            'concurrency_level': 1
        }],
        output='screen'
    )

    # Right pointcloud to laserscan node
    right_pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='right_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cam4/point_cloud_right'),
            ('/scan', '/rightLaserScan')
        ],
        parameters=[{
            'target_frame': 'camera_laserscan_link_right',
            'transform_tolerance': 0.01,
            'min_height': -0.05,
            'max_height': 0.1,
            'angle_min': -0.087,
            'angle_max': 0.087,
            'angle_increment': 0.0175,
            'scan_time': 0.3333,
            'range_min': 0.1,
            'range_max': 1.5,
            'use_inf': True,
            'concurrency_level': 1
        }],
        output='screen'
    )

    return LaunchDescription([
        face_pointcloud_node,
        left_pointcloud_node,
        right_pointcloud_node
    ])
