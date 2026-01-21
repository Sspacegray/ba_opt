#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import csv
import os
from datetime import datetime
import numpy as np

# 定义Livox点云数据结构（与C++代码对应）
class LivoxPointXyzrtlt:
    """Livox点云数据结构，对应C++中的LivoxPointXyzrtlt"""
    STRUCT_FORMAT = 'fff f B B d'  # x, y, z, intensity, tag, line, timestamp (纳秒)
    SIZE = struct.calcsize(STRUCT_FORMAT)
    
    def __init__(self, x=0.0, y=0.0, z=0.0, intensity=0.0, tag=0, line=0, timestamp_ns=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.tag = tag
        self.line = line
        # 将纳秒转换为秒
        self.timestamp = timestamp_ns / 1e9
    
    @classmethod
    def from_bytes(cls, data):
        """从字节数据解析点云数据"""
        values = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(*values)
    
    def to_list(self):
        """转换为列表格式，便于CSV写入"""
        return [self.x, self.y, self.z, self.intensity, self.tag, self.line, self.timestamp]

class LivoxPointCloudParser(Node):
    """Livox点云解析节点"""
    
    def __init__(self):
        super().__init__('livox_pointcloud_parser')
        
        # 参数声明
        self.declare_parameter('pointcloud_topic', '/lidar_F/pointcloud')
        self.declare_parameter('output_dir', './livox_pointcloud_data')
        self.declare_parameter('save_to_csv', True)
        self.declare_parameter('csv_filename_prefix', 'livox_pointcloud')
        self.declare_parameter('publish_parsed_pointcloud', False)  # 是否发布解析后的点云
        
        # 获取参数
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.save_to_csv = self.get_parameter('save_to_csv').value
        self.csv_filename_prefix = self.get_parameter('csv_filename_prefix').value
        self.publish_parsed_pointcloud = self.get_parameter('publish_parsed_pointcloud').value
        
        # 创建输出目录
        if self.save_to_csv and not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            self.get_logger().info(f'创建输出目录: {self.output_dir}')
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            10
        )
        
        # 发布解析后的点云（可选）
        if self.publish_parsed_pointcloud:
            self.publisher = self.create_publisher(PointCloud2, '/livox/parsed_pointcloud', 10)
        
        # CSV文件相关
        self.current_csv_file = None
        self.csv_writer = None
        self.point_count = 0
        self.frame_count = 0
        
        # CSV文件头
        self.csv_header = ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp(s)']
        
        self.get_logger().info(f'Livox点云解析节点已启动，订阅话题: {pointcloud_topic}')
        self.get_logger().info(f'CSV文件保存路径: {self.output_dir}')
        self.get_logger().info(f'时间戳单位: 秒')
    
    def validate_pointcloud_structure(self, msg):
        """验证点云消息结构是否符合Livox标准格式"""
        try:
            # 检查字段数量
            if len(msg.fields) != 7:
                self.get_logger().warn(f'字段数量不匹配: 期望7个，实际{len(msg.fields)}个')
                return False
            
            # 检查字段名称和数据类型
            expected_fields = [
                ('x', PointField.FLOAT32, 0),
                ('y', PointField.FLOAT32, 4),
                ('z', PointField.FLOAT32, 8),
                ('intensity', PointField.FLOAT32, 12),
                ('tag', PointField.UINT8, 16),
                ('line', PointField.UINT8, 17),
                ('timestamp', PointField.FLOAT64, 18)
            ]
            
            for i, (expected_name, expected_type, expected_offset) in enumerate(expected_fields):
                if i >= len(msg.fields):
                    return False
                
                field = msg.fields[i]
                if field.name != expected_name or field.datatype != expected_type or field.offset != expected_offset:
                    self.get_logger().warn(f'字段{i}不匹配: 名称={field.name}, 类型={field.datatype}, 偏移={field.offset}')
                    return False
            
            # 检查点步长
            expected_point_step = LivoxPointXyzrtlt.SIZE
            if msg.point_step != expected_point_step:
                self.get_logger().warn(f'点步长不匹配: 期望{expected_point_step}，实际{msg.point_step}')
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'验证点云结构时出错: {str(e)}')
            return False
    
    def parse_pointcloud_data(self, msg):
        """解析点云数据"""
        points = []
        
        try:
            # 验证点云结构
            if not self.validate_pointcloud_structure(msg):
                self.get_logger().warn('点云结构验证失败，尝试直接解析数据')
            
            # 解析每个点
            for i in range(msg.width):
                # 计算当前点在数据中的偏移量
                start_idx = i * msg.point_step
                end_idx = start_idx + LivoxPointXyzrtlt.SIZE
                
                if end_idx > len(msg.data):
                    self.get_logger().warn(f'点{i}的数据超出范围，跳过')
                    break
                
                # 提取点数据并解析
                point_data = msg.data[start_idx:end_idx]
                point = LivoxPointXyzrtlt.from_bytes(point_data)
                points.append(point)
            
            self.get_logger().info(f'成功解析 {len(points)} 个点')
            return points
            
        except Exception as e:
            self.get_logger().error(f'解析点云数据时出错: {str(e)}')
            return []
    
    def create_parsed_pointcloud_msg(self, points, original_msg):
        """创建解析后的点云消息"""
        if not points or not self.publish_parsed_pointcloud:
            return None
        
        try:
            # 创建新的点云消息
            parsed_msg = PointCloud2()
            
            # 复制头部信息
            parsed_msg.header = original_msg.header
            
            # 设置点云属性
            parsed_msg.height = 1
            parsed_msg.width = len(points)
            parsed_msg.is_bigendian = False
            parsed_msg.is_dense = True
            parsed_msg.point_step = LivoxPointXyzrtlt.SIZE
            parsed_msg.row_step = parsed_msg.width * parsed_msg.point_step
            
            # 设置字段（与原始消息相同）
            parsed_msg.fields = original_msg.fields.copy()
            
            # 构建数据
            data = bytearray()
            for point in points:
                # 注意：这里的时间戳已经是秒单位
                point_data = struct.pack(
                    LivoxPointXyzrtlt.STRUCT_FORMAT,
                    point.x, point.y, point.z,
                    point.intensity,
                    point.tag, point.line,
                    point.timestamp  # 已经是秒单位
                )
                data.extend(point_data)
            
            parsed_msg.data = bytes(data)
            return parsed_msg
            
        except Exception as e:
            self.get_logger().error(f'创建解析点云消息时出错: {str(e)}')
            return None
    
    def create_new_csv_file(self):
        """创建新的CSV文件"""
        if not self.save_to_csv:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = f'{self.csv_filename_prefix}_{timestamp}.csv'
        filepath = os.path.join(self.output_dir, filename)
        
        try:
            self.current_csv_file = open(filepath, 'w', newline='')
            self.csv_writer = csv.writer(self.current_csv_file)
            self.csv_writer.writerow(self.csv_header)
            self.point_count = 0
            self.get_logger().info(f'创建新的CSV文件: {filename}')
        except Exception as e:
            self.get_logger().error(f'创建CSV文件时出错: {str(e)}')
            self.current_csv_file = None
            self.csv_writer = None
    
    def save_points_to_csv(self, points):
        """将点云数据保存到CSV文件"""
        if not self.save_to_csv or not self.csv_writer:
            return
        
        try:
            for point in points:
                self.csv_writer.writerow(point.to_list())
                self.point_count += 1
            
            self.get_logger().info(f'已保存 {len(points)} 个点到CSV，当前文件总计 {self.point_count} 个点')
            
        except Exception as e:
            self.get_logger().error(f'保存数据到CSV时出错: {str(e)}')
    
    def pointcloud_callback(self, msg):
        """点云消息回调函数"""
        self.frame_count += 1
        self.get_logger().info(f'收到第 {self.frame_count} 帧点云，点数: {msg.width}')
        
        # 创建新的CSV文件（每帧一个文件）
        self.create_new_csv_file()
        
        # 解析点云数据
        points = self.parse_pointcloud_data(msg)
        
        if points:
            # 保存到CSV
            self.save_points_to_csv(points)
            
            # 发布解析后的点云（可选）
            if self.publish_parsed_pointcloud:
                parsed_msg = self.create_parsed_pointcloud_msg(points, msg)
                if parsed_msg:
                    self.publisher.publish(parsed_msg)
                    self.get_logger().info('已发布解析后的点云')
            
            # 记录统计信息
            if len(points) > 0:
                # 计算点云统计信息
                x_coords = [p.x for p in points]
                y_coords = [p.y for p in points]
                z_coords = [p.z for p in points]
                timestamps = [p.timestamp for p in points]
                
                self.get_logger().info(
                    f'点云统计 - X: [{min(x_coords):.2f}, {max(x_coords):.2f}], '
                    f'Y: [{min(y_coords):.2f}, {max(y_coords):.2f}], '
                    f'Z: [{min(z_coords):.2f}, {max(z_coords):.2f}], '
                    f'时间戳: [{min(timestamps):.6f}s, {max(timestamps):.6f}s]'
                )
        
        # 关闭当前CSV文件
        if self.current_csv_file:
            self.current_csv_file.close()
            self.current_csv_file = None
            self.csv_writer = None
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        if self.current_csv_file:
            self.current_csv_file.close()
        self.get_logger().info('Livox点云解析节点已关闭')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LivoxPointCloudParser()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行出错: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()