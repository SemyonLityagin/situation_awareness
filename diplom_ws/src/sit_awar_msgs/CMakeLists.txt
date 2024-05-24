cmake_minimum_required(VERSION 3.8)
project(sit_awar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/SensorPack.msg"
  "msg/ObjectState.msg"
  "msg/SelfObjectState.msg"
)
rosidl_get_typesupport_target(${PROJECT_NAME}_typesupport_c
  ${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/SensorPack.msg"
  "msg/ObjectState.msg"
  "msg/SelfObjectState.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
