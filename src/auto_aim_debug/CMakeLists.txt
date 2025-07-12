cmake_minimum_required(VERSION 3.8)
project(auto_aim_debug)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(auto_aim_interfaces REQUIRED)

add_executable(auto_aim_debug_node 
  node/auto_aim_debug_node.cpp  
  src/ImageShower/ImageShower.cpp
  src/Tools/Tools.cpp
  src/debugger/Debugger.cpp
  src/VideoDownloader/VideoDownloader.cpp
)

ament_target_dependencies(auto_aim_debug_node rclcpp
  sensor_msgs cv_bridge OpenCV auto_aim_interfaces)

install(TARGETS auto_aim_debug_node DESTINATION lib/${PROJECT_NAME})

ament_package()

