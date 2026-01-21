#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

class PoseTimeDiff(Node):
    def __init__(self):
        super().__init__("pose_time_diff")
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, "/pcl_pose", self.listener_callback, 10)
        self.get_logger().info("节点已启动，等待/pcl_pose消息...")

    def listener_callback(self, msg):
        # 提取消息中的时间戳
        msg_stamp = msg.header.stamp
        msg_time = self.time_to_float(msg_stamp)

        # 获取当前系统时间
        current_time = self.get_clock().now()

        # 计算时间差（秒）
        time_diff = current_time - msg_time

        # 打印结果
        self.get_logger().info(f"消息时间戳: {msg_time:.9f} (ROS时间)" f"系统接收时间: {current_time:.9f} (UNIX时间)" f"时间差: {time_diff:.9f} 秒")

    def time_to_float(self, t: Time) -> float:
        """将ROS Time对象转换为浮点秒数"""
        return float(t.sec) + float(t.nanosec) / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = PoseTimeDiff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
