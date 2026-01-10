cmake_minimum_required(VERSION 3.8)
project(qxlidar_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(LIDAR_SDK_ROOT  /home/ubuntu/workspace/SDK_V3.0-master)   # 改这里即可
set(LIDAR_LIB       ${LIDAR_SDK_ROOT}/bin/lib/liblidar.a)     # .a 文件
set(LIDAR_INC       ${LIDAR_SDK_ROOT}/include)                # 头文件根

# 把你原来的 lidar SDK 编好的静态库引进来
add_library(lidar STATIC IMPORTED)
set_target_properties(lidar PROPERTIES
    IMPORTED_LOCATION ${LIDAR_LIB})  # 路径按实际放
include_directories(
    ${LIDAR_INC}
    ${LIDAR_SDK_ROOT}/src/base
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(lidar_node
  src/qxlidar_test.cpp
  src/qxlidar_ros2_driver_node.cpp
)
target_link_libraries(lidar_node
  lidar
  pthread
)
ament_target_dependencies(lidar_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
