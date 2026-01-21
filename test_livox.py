#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
import struct
import numpy as np
from builtin_interfaces.msg import Time
import time

class LivoxTimestampValidator(Node):
    def __init__(self):
        super().__init__("livox_timestamp_validator")

        self.imu_sub = self.create_subscription(Imu, "/lidar_F/imu_data", self.imu_callback, 10)
        # self.subscription = self.create_subscription(PointCloud2, "/lidar_F/pointcloud", self.pointcloud_callback, 10)

        self.get_logger().info("Livox Timestamp Validator started. Waiting for point cloud data...")

    def time_to_float(self, t: Time) -> float:
        return float(t.sec) + float(t.nanosec) / 1e9


    def imu_callback(self, msg: Imu):
        msg_stamp = msg.header.stamp
        msg_time = self.time_to_float(msg_stamp)
        current_time = self.time_to_float(self.get_clock().now().to_msg())
        time_diff = current_time - msg_time
        self.get_logger().info(f"消息时间戳: {msg_time:.9f} (ROS时间)" f"系统接收时间: {current_time:.9f} (UNIX时间)" f"时间差: {time_diff:.9f} 秒")


    def pointcloud_callback(self, msg):
        msg_stamp = msg.header.stamp
        msg_time = self.time_to_float(msg_stamp)
        current_time = self.time_to_float(self.get_clock().now().to_msg())
        time_diff = current_time - msg_time
        self.get_logger().info(f"消息时间戳: {msg_time:.9f} (ROS时间)" f"系统接收时间: {current_time:.9f} (UNIX时间)" f"时间差: {time_diff:.9f} 秒")

        try:
            # 1. 获取点云结构信息
            point_step = msg.point_step
            width = msg.width
            if width == 0:
                self.get_logger().warning("Received empty point cloud")
                return

            # 2. 找到时间戳字段的位置
            timestamp_offset = None
            for field in msg.fields:
                if field.name == 'timestamp':
                    timestamp_offset = field.offset
                    break

            if timestamp_offset is None:
                self.get_logger().error("Timestamp field not found in point cloud")
                return

            # 3. 遍历所有点，提取时间戳
            timestamps = []
            for i in range(width):
                point_start = i * point_step

                # 提取时间戳（double类型，8字节）
                timestamp_bytes = msg.data[point_start + timestamp_offset : point_start + timestamp_offset + 8]
                timestamp_ns = struct.unpack('<d', timestamp_bytes)[0] * 1e-9
                timestamps.append(timestamp_ns)

            # 4. 验证时间戳顺序
            is_sorted = all(timestamps[i] <= timestamps[i+1] for i in range(len(timestamps)-1))
            is_first_min = timestamps[0] == min(timestamps) if timestamps else True
            is_last_max = timestamps[-1] == max(timestamps) if timestamps else True

            # 5. 输出详细统计信息
            min_timestamp = min(timestamps) if timestamps else 0
            max_timestamp = max(timestamps) if timestamps else 0
            avg_timestamp = sum(timestamps) / len(timestamps) if timestamps else 0

            # self.get_logger().info("\n=== Point Cloud Timestamp Analysis ===")
            # self.get_logger().info(f"Total points: {width}")
            # self.get_logger().info(f"  Min timestamp: {min_timestamp:.9f} ns")
            # self.get_logger().info(f"  Avg timestamp: {avg_timestamp:.9f} ns")
            # self.get_logger().info(f"First timestamp: {timestamps[0]:.9f} ns")
            # self.get_logger().info(f" Last timestamp: {timestamps[-1]:.9f} ns")
            # self.get_logger().info(f"  Max timestamp: {max_timestamp:.9f} ns")
            # self.get_logger().info(f"Timestamp range: {max_timestamp - min_timestamp:.9f} ns")

            # 6. 检查结果
            # if is_sorted:
            #     self.get_logger().info("✓ Timestamps are strictly increasing (sorted)")
            # else:
            #     self.get_logger().warning("✗ Timestamps are NOT strictly increasing")

            if timestamps[0] == msg_time:
                self.get_logger().info("timestamps[0] == msg_time")
            elif timestamps[0] > msg_time:
                self.get_logger().warning(f"{timestamps[0]:.9f} > {msg_time:.9f}")
            else :
                self.get_logger().warning(f"{timestamps[0]:.9f} < {msg_time:.9f}")

            if is_first_min:
                self.get_logger().info("✓ First point has the minimum timestamp")
            else:
                self.get_logger().warning("✗ First point does NOT have the minimum timestamp")

            if is_last_max:
                self.get_logger().info("✓ Last point has the maximum timestamp")
            else:
                self.get_logger().warning("✗ Last point does NOT have the maximum timestamp")

            # # 7. 找出非递增的位置（如果有）
            # if not is_sorted:
            #     problematic_indices = []
            #     for i in range(len(timestamps)-1):
            #         if timestamps[i] > timestamps[i+1]:
            #             problematic_indices.append(i)
            #             # self.get_logger().warning(f"  Point {i}: {timestamps[i]:.9f} > Point {i+1}: {timestamps[i+1]:.9f}")

            #     self.get_logger().warning(f"Found {len(problematic_indices)} problematic timestamp transitions")

            # # 8. 检查最大时间戳位置
            # if not is_last_max:
            #     max_index = timestamps.index(max_timestamp)
            #     self.get_logger().warning(f"Maximum timestamp is at point {max_index}, not the last point")

            # self.get_logger().info("=====================================")

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LivoxTimestampValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
