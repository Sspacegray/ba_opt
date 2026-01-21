cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_2dmap_ros2)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# Executable
add_executable(pointcloud_to_2dmap_ros2 src/pointcloud_to_2dmap_ros.cpp)

ament_target_dependencies(pointcloud_to_2dmap_ros2
        rclcpp
        sensor_msgs
        cv_bridge
        PCL
        OpenCV
        )

target_link_libraries(pointcloud_to_2dmap_ros2
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        )

install(TARGETS pointcloud_to_2dmap_ros2
        DESTINATION lib/${PROJECT_NAME})
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
ament_package()
