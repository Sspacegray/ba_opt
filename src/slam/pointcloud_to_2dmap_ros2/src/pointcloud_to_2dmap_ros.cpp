#include <iostream>                     // 标准输入输出库，用于调试或打印信息
#include <boost/filesystem.hpp>         // Boost 文件系统库，用于路径检查和目录创建
#include <opencv2/opencv.hpp>           // OpenCV库，用于图像处理
#include <pcl/io/pcd_io.h>              // PCL库，用于读取PCD点云文件
#include <pcl/point_types.h>            // PCL点类型定义
#include <pcl/point_cloud.h>            // PCL点云数据结构
#include <rclcpp/rclcpp.hpp>            // ROS2 C++客户端库

// 定义一个地图生成器类
class MapGenerator
{
public:
    double resolution;        // 地图每个像素对应的实际米数
    double m2pix;             // 分辨率倒数，用于将米转像素
    int map_width;            // 地图宽度（像素）
    int map_height;           // 地图高度（像素）
    int min_points_in_pix;    // 每个像素最小点数
    int max_points_in_pix;    // 每个像素最大点数
    double min_height;        // 高度下限
    double max_height;        // 高度上限
    std::string dest_directory;  // 地图输出目录
    std::string input_pcd;       // 输入点云路径
    std::string map_name;        // 输出地图文件名

    // 根据点云生成二维栅格图
    cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ> &cloud) const
    {
        // 创建一个空的灰度图像，初始化为0
        cv::Mat map(map_height, map_width, CV_8UC1, cv::Scalar::all(0));

        // 遍历点云中的每个点
        for (const auto &point : cloud)
        {
            // 高度过滤：只保留在 min_height 到 max_height 范围内的点
            if (point.z < min_height || point.z > max_height)
            {
                continue;
            }

            // 将点云坐标转换为像素坐标
            int x = static_cast<int>(point.x * m2pix + map_width / 2);
            int y = static_cast<int>(-point.y * m2pix + map_height / 2);  // y取负，因为图像坐标y向下

            // 边界检查：如果超出图像范围则跳过
            if (x < 0 || x >= map_width || y < 0 || y >= map_height)
            {
                continue;
            }

            // 像素值累加，表示该位置点云密度
            map.at<uchar>(y, x)++;
        }

        // 减去最小点数阈值
        map -= min_points_in_pix;

        // 将像素值映射到0~255灰度
        map.convertTo(map, CV_8UC1, -255.0 / (max_points_in_pix - min_points_in_pix), 255);

        return map; // 返回生成的灰度图
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化ROS2节点
    auto node = rclcpp::Node::make_shared("pointcloud_to_2dmap_node"); // 创建节点对象

    MapGenerator map_generator; // 创建地图生成器对象

    // 声明并获取参数，ROS2参数服务器机制
    node->declare_parameter("resolution", 0.1);
    node->declare_parameter("map_width", 10240);
    node->declare_parameter("map_height", 10240);
    node->declare_parameter("min_points_in_pix", 2);
    node->declare_parameter("max_points_in_pix", 5);
    node->declare_parameter("min_height", 0.5);
    node->declare_parameter("max_height", 3.0);
    node->declare_parameter("dest_directory", std::string(""));
    node->declare_parameter("input_pcd", std::string(""));
    node->declare_parameter("map_name", std::string("map"));

    // 获取参数值并赋给MapGenerator成员变量
    node->get_parameter("resolution", map_generator.resolution);
    node->get_parameter("map_width", map_generator.map_width);
    node->get_parameter("map_height", map_generator.map_height);
    node->get_parameter("min_points_in_pix", map_generator.min_points_in_pix);
    node->get_parameter("max_points_in_pix", map_generator.max_points_in_pix);
    node->get_parameter("min_height", map_generator.min_height);
    node->get_parameter("max_height", map_generator.max_height);
    node->get_parameter("dest_directory", map_generator.dest_directory);
    node->get_parameter("input_pcd", map_generator.input_pcd);
    node->get_parameter("map_name", map_generator.map_name);

    map_generator.m2pix = 1.0 / map_generator.resolution; // 计算米转像素比例

    // 打印参数信息
    RCLCPP_INFO(node->get_logger(), "resolution    : %f", map_generator.resolution);
    RCLCPP_INFO(node->get_logger(), "m2pix         : %f", map_generator.m2pix);
    RCLCPP_INFO(node->get_logger(), "map_width     : %d", map_generator.map_width);
    RCLCPP_INFO(node->get_logger(), "map_height    : %d", map_generator.map_height);
    RCLCPP_INFO(node->get_logger(), "min_points_in_pix: %d", map_generator.min_points_in_pix);
    RCLCPP_INFO(node->get_logger(), "max_points_in_pix: %d", map_generator.max_points_in_pix);
    RCLCPP_INFO(node->get_logger(), "min_height    : %f", map_generator.min_height);
    RCLCPP_INFO(node->get_logger(), "max_height    : %f", map_generator.max_height);
    RCLCPP_INFO(node->get_logger(), "dest_directory: %s", map_generator.dest_directory.c_str());
    RCLCPP_INFO(node->get_logger(), "input_pcd     : %s", map_generator.input_pcd.c_str());
    RCLCPP_INFO(node->get_logger(), "map_name      : %s", map_generator.map_name.c_str());

    // 读取点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_generator.input_pcd, *cloud) == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open the input cloud: %s", map_generator.input_pcd.c_str());
        return 1; // 打开点云失败则退出
    }
    RCLCPP_DEBUG(node->get_logger(), "Read PointCloud OK!");

    // 生成二维栅格地图
    cv::Mat map = map_generator.generate(*cloud);

    // 如果输出目录不存在，则创建
    if (!boost::filesystem::exists(map_generator.dest_directory))
    {
        boost::filesystem::create_directories(map_generator.dest_directory);
    }

    // 保存生成的地图为PNG图像
    std::string map_path = map_generator.dest_directory + "/" + map_generator.map_name + ".png";
    cv::imwrite(map_path, map);
    RCLCPP_INFO(node->get_logger(), "Saved 2D map as PNG: %s", map_path.c_str());

    // 保存对应的YAML文件，供ROS导航使用
    std::string yaml_path = map_generator.dest_directory + "/map.yaml";
    std::ofstream ofs(yaml_path);
    ofs << "image: " << map_generator.map_name << ".png" << std::endl;
    ofs << "resolution: " << map_generator.resolution << std::endl;
    ofs << "origin: ["
        << -map_generator.resolution * map_generator.map_width / 2 << ", "
        << -map_generator.resolution * map_generator.map_height / 2 << ", 0.0]" << std::endl;
    ofs << "occupied_thresh: 0.5" << std::endl;
    ofs << "free_thresh: 0.2" << std::endl;
    ofs << "negate: 0" << std::endl;
    RCLCPP_INFO(node->get_logger(), "Saved YAML file: %s", yaml_path.c_str());

    // 提示用户如果修改地图名，需要修改YAML文件中的图像名
    RCLCPP_INFO(node->get_logger(), "If the 2D map name is changed, update the image name in the YAML file!");

    rclcpp::shutdown(); // 关闭ROS2节点
    return 0;           // 程序正常结束
}
