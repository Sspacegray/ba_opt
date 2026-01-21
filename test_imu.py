#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Vector3Stamped
import tf2_geometry_msgs
import math


def quaternion_multiply(q1, q2):
    """手动实现四元数乘法"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


class ImuTransformNode(Node):
    def __init__(self):
        super().__init__("imu_transform_listener")

        self.declare_parameter("imu_topic", "/lidar_F/imu_data")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("imu_frame", "lidar_F_imu_link")
        self.declare_parameter('output_topic', '/imu/data_base')

        self.imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.imu_frame = self.get_parameter("imu_frame").get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, self.output_topic, 10)

        self.get_logger().info(f"Listening to {self.imu_topic}, transforming to {self.target_frame}")

    def imu_callback(self, msg: Imu):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.imu_frame, rclpy.time.Time())

            # --- 变换角速度 ---
            ang_vel_in = Vector3Stamped()
            ang_vel_in.header = msg.header
            ang_vel_in.vector = msg.angular_velocity
            ang_vel_out = tf2_geometry_msgs.do_transform_vector3(ang_vel_in, transform)

            # --- 变换线加速度 ---
            lin_acc_in = Vector3Stamped()
            lin_acc_in.header = msg.header
            lin_acc_in.vector = msg.linear_acceleration
            lin_acc_out = tf2_geometry_msgs.do_transform_vector3(lin_acc_in, transform)

            # --- 变换姿态 ---
            q_in = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            q_tf = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            q_out = quaternion_multiply(q_tf, q_in)

            imu_out = Imu()
            imu_out.header.stamp = self.get_clock().now().to_msg()
            imu_out.header.frame_id = self.target_frame
            imu_out.orientation.x = q_out[0]
            imu_out.orientation.y = q_out[1]
            imu_out.orientation.z = q_out[2]
            imu_out.orientation.w = q_out[3]
            imu_out.angular_velocity = ang_vel_out.vector
            imu_out.linear_acceleration = lin_acc_out.vector
            self.pub.publish(imu_out)

            self.get_logger().info(
                f"\n[IMU in {self.target_frame}]\n"
                f"  Orientation (quat): {q_out}\n"
                f"  Angular vel: {ang_vel_out.vector}\n"
                f"  Linear acc: {lin_acc_out.vector}\n"
            )

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
