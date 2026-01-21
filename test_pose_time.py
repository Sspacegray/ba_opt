#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import time


class TimeComparator(Node):
    def __init__(self):
        super().__init__("time_comparator")

        # 创建订阅者
        self.subscription = self.create_subscription(PoseStamped, "/agv_pose", self.pose_callback, 10)

        self.get_logger().info("节点已启动，正在监听/agv_pose话题...")

    def pose_callback(self, msg):
        # 提取消息时间戳
        stamp = msg.header.stamp
        msg_time = self.stamp_to_seconds(stamp)

        # 获取当前ROS时间（使用ROS时钟）
        current_ros_time = self.get_clock().now().to_msg()
        current_time = self.stamp_to_seconds(current_ros_time)

        # 计算时间差（秒）
        time_diff = current_time - msg_time

        # 格式化输出
        self.get_logger().info(
            f"消息时间戳: {stamp.sec}.{stamp.nanosec:09d} 当前ROS时间: {current_ros_time.sec}.{current_ros_time.nanosec:09d} 时间差: {time_diff:.9f} 秒"
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
