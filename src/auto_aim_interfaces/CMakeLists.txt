cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2f.msg"
  "msg/SendArmor.msg"
  "msg/SerialReadData.msg"
  "msg/SerialSendData.msg"
  "msg/TransDetector.msg"
  "msg/DebugArmor.msg"
  "msg/DebugCar.msg"
  "msg/DebugFaceAngle.msg"
  "msg/DebugDetector.msg"
  "msg/DebugProcessor.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    builtin_interfaces 
)

ament_package()
