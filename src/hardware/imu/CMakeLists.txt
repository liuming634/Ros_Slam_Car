cmake_minimum_required(VERSION 3.5)
project(imu)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create imu node
add_executable(imu_node src/imu_node.cpp)

target_link_libraries(imu_node
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
)

# Install
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
