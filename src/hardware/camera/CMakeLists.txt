cmake_minimum_required(VERSION 3.5)
project(camera)

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
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(LIBSODIUM REQUIRED libsodium)
# FFmpeg 不提供 CMake 配置文件，用 pkg-config 查找（fpv 解码用到 libavcodec/libavformat/libswscale）
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libswscale libavutil)

include_directories(
  fpv/include
  fpv/include/hal
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${LIBSODIUM_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
)

# FPV WFB 接收端（fpv/）暂未启用：
#   当前相机走 USB UVC（camera_source: usb），fpv 分支尚未实现，此库不被 camera_node 调用。
#   该库依赖 C++20（std::span 等），启用时需把 CMAKE_CXX_STANDARD 提到 20 并修复可能的编译问题。
option(BUILD_FPV_RX "构建 FPV WFB 接收库" OFF)

if(BUILD_FPV_RX)
  # Create FPV receiver library
  add_library(fpvrx
    fpv/src/receiver.cpp
    fpv/src/FrameParser.cpp
    fpv/src/WFBProcessor.cpp
    fpv/src/RtlUsbAdapter.cpp
    fpv/src/Rtl8812aDevice.cpp
    fpv/src/WiFiDriver.cpp
    fpv/src/RadioManagementModule.cpp
    fpv/src/HalModule.cpp
    fpv/src/fec.c
    fpv/src/FirmwareManager.cpp
    fpv/src/EepromManager.cpp
    fpv/include/hal/Hal8812PwrSeq.c
    fpv/include/hal/hal8812a_fw.c
  )

  target_link_libraries(fpvrx
    ${OpenCV_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    ${LIBSODIUM_LIBRARIES}
    ${FFMPEG_LIBRARIES}
  )
endif()

# Create camera node
add_executable(camera_node src/camera_node.cpp)

target_link_libraries(camera_node
  ${ament_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_FPV_RX)
  install(TARGETS
    fpvrx
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()