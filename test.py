#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from builtin_interfaces.msg import Time
from sensor_msgs_py import point_cloud2


class TimeComparator(Node):
    def __init__(self):
        super().__init__("time_comparator")
        self.subscription = self.create_subscription(LaserScan, "/scan", self.call_back, 10)

    def call_back(self, msg):
        # 提取消息时间戳
        stamp = msg.header.stamp
        msg_time = self.stamp_to_seconds(stamp)

        # 获取当前ROS时间
        current_ros_time = self.get_clock().now().to_msg()
        current_time = self.stamp_to_seconds(current_ros_time)

        # 计算消息时间差（当前时间 - 消息时间）
        time_diff = current_time - msg_time

        # 格式化输出
        self.get_logger().info(
            f"消息时间戳: {stamp.sec}.{stamp.nanosec:09d}"
            f"当前ROS时间: {current_ros_time.sec}.{current_ros_time.nanosec:09d}"
            f"消息延迟: {time_diff:.9f} 秒"
            f"点云扫描耗时: {msg.scan_time:.9f} 秒)"
        )

    @staticmethod
    def stamp_to_seconds(stamp: Time) -> float:
        """将ROS2时间戳转换为浮点秒数"""
        return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = TimeComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
