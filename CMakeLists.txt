cmake_minimum_required(VERSION 3.5)
project(point_counter)

# 依存関係の設定
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 実行可能ファイルの追加
add_executable(point_counter_node src/point_counter_node.cpp)

ament_target_dependencies(point_counter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS point_counter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
