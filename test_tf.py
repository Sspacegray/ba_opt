#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TfRelativePose(Node):
    def __init__(self):
        super().__init__("tf_relative_pose")

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 保存1秒前的位姿
        self.last_pose = None

        # 定时器
        self.timer_01 = self.create_timer(0.1, self.timer_callback)

        self.current_pose = None

    def get_current_pose(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return None

    def timer_callback(self):
        pose = self.get_current_pose()
        if pose:
            self.current_pose = pose
        if self.last_pose is None:
            self.last_pose = self.current_pose
            return
        dx = self.current_pose[0] - self.last_pose[0]
        dy = self.current_pose[1] - self.last_pose[1]
        dyaw = self.current_pose[2] - self.last_pose[2]
        # yaw差值归一化到 [-pi, pi]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))

        self.get_logger().info(f"Relative change: Δx={dx:5.3f}, Δy={dy:5.3f}, Δyaw={math.degrees(dyaw):5.2f}°")

        # 更新上一次的姿态
        self.last_pose = self.current_pose


def main(args=None):
    rclpy.init(args=args)
    node = TfRelativePose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
