cmake_minimum_required(VERSION 3.8)
project(auto_aim_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 查找必要的ROS 2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# 添加可执行文件
add_executable(send_debug_detector_node src/send_debug_detector_node.cpp)
add_executable(send_debug_processor_node src/send_debug_processor_node.cpp)
add_executable(send_image_node src/send_image_node.cpp)
add_executable(send_serial_read_data_node src/send_serial_read_data_node.cpp)
# 链接依赖项
ament_target_dependencies(send_debug_detector_node
  rclcpp
  OpenCV
  cv_bridge
  auto_aim_interfaces
)
ament_target_dependencies(send_debug_processor_node
  rclcpp
  auto_aim_interfaces
)
ament_target_dependencies(send_image_node
  rclcpp
  OpenCV
  cv_bridge
  auto_aim_interfaces
)
ament_target_dependencies(send_serial_read_data_node
  rclcpp
  auto_aim_interfaces
)

# 安装可执行文件
install(TARGETS
  send_debug_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  send_debug_processor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  send_image_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  send_serial_read_data_node
  DESTINATION lib/${PROJECT_NAME}
)

# 生成Ament CMake配置文件
ament_package()