cmake_minimum_required(VERSION 3.8)
project(sit_awar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sit_awar_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include/sit_awar)

add_executable(sit_awar src/main.cpp 
  src/IdGenerator.cpp
  src/WorldModel.cpp
  src/SituationAwareness.cpp
  src/TopicListener.cpp
  src/DataHungarianAggregation.cpp
  src/Kalman.cpp
  src/DataKalmanFilter.cpp
)

target_link_libraries (sit_awar Eigen3::Eigen)
ament_target_dependencies(sit_awar rclcpp sit_awar_msgs Eigen3)
ament_export_dependencies(Eigen3)
target_include_directories(sit_awar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sit_awar>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sit_awar PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS sit_awar
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY agent_config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()