#!/usr/bin/env python3

# 将PointCloud -> PointCloud2 -> livox_ros_drive2.CustomMsg

import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header
import struct
from threading import Lock

# 全局变量
pub = None
lidar_frame = "lidar_frame"  # 替换为实际的 LiDAR 帧 ID
m_buf = Lock()


def pointcloud2_to_custommsg(node, pointcloud2):
    custom_msg = CustomMsg()
    custom_msg.header = pointcloud2.header
    custom_msg.timebase = node.get_clock().now().nanoseconds
    custom_msg.point_num = pointcloud2.width
    custom_msg.lidar_id = 1  # Assuming lidar_id is 1
    custom_msg.rsvd = [0, 0, 0]  # Reserved fields

    # Parse PointCloud2 data
    fmt = _get_struct_fmt(pointcloud2)
    for i in range(0, len(pointcloud2.data), pointcloud2.point_step):
        point_data = pointcloud2.data[i:i+pointcloud2.point_step]
        x, y, z = struct.unpack(fmt, point_data)

        custom_point = CustomPoint()
        custom_point.offset_time = node.get_clock().now().nanoseconds - custom_msg.timebase
        custom_point.x = x
        custom_point.y = y
        custom_point.z = z
        # custom_point.reflectivity = int(intensity * 255)  # Scale intensity to 0-255
        custom_point.tag = 0  # Assuming no tag
        custom_point.line = 0  # Assuming no line number

        custom_msg.points.append(custom_point)

    return custom_msg

def _get_struct_fmt(pointcloud2):
    fmt = ''
    for field in pointcloud2.fields:
        if field.datatype == PointField.FLOAT32:
            fmt += 'f'
        elif field.datatype == PointField.UINT8:
            fmt += 'B'
        elif field.datatype == PointField.INT8:
            fmt += 'b'
        elif field.datatype == PointField.UINT16:
            fmt += 'H'
        elif field.datatype == PointField.INT16:
            fmt += 'h'
        elif field.datatype == PointField.UINT32:
            fmt += 'I'
        elif field.datatype == PointField.INT32:
            fmt += 'i'
    return fmt


class PointCloudConverterNode(Node):
    def __init__(self):
        super().__init__('pre_mmw')

        self.declare_parameter('enable_pcd2livox', False)
        sim_param = self.get_parameter('enable_pcd2livox').value

        if not sim_param:
            self.get_logger().info("Pointcloud2livox is enabled!")

            # 订阅 PointCloud 话题
            self.sub_mmw_cloud = self.create_subscription(
                PointCloud, '/scan', self.mmw_handler, 10)

            # pub_laser_cloud = self.create_publisher(PointCloud2, "livox/lidar", 2000)
            self.pub = self.create_publisher(CustomMsg, '/livox/lidar2', 10)
        else:
            self.get_logger().info("Pointcloud2livox is disabled!")

    def mmw_handler(self, mmw_cloud_msg):
        global lidar_frame, m_buf

        # 加锁
        m_buf.acquire()

        # 将 PointCloud 转换为 PointCloud2
        laser_cloud_msg = PointCloud2()
        laser_cloud_msg.header.stamp = mmw_cloud_msg.header.stamp
        laser_cloud_msg.header.frame_id = lidar_frame
        laser_cloud_msg = pc2.create_cloud_xyz32(laser_cloud_msg.header, [(p.x, p.y, p.z) for p in mmw_cloud_msg.points])
        # laser_cloud_msg 是 PointCloud2格式数据
        custom_msg = pointcloud2_to_custommsg(self, laser_cloud_msg)
        self.pub.publish(custom_msg)
        # 发布 PointCloud2 消息
        # pub_laser_cloud.publish(laser_cloud_msg)

        # 解锁
        m_buf.release()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverterNode()

    if node.get_parameter('enable_pcd2livox').value:
        node.destroy_node()
        rclpy.shutdown()
    else:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
