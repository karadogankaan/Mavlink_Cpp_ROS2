cmake_minimum_required(VERSION 3.10)
project(movement)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/mavlink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(mavlink REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/mavlink)

add_executable(mov_node src/movement_drone.cpp)

include_directories(${MAVLINK_DIR}/include/mavlink/v2.0)

ament_target_dependencies(
mov_node
rclcpp
std_msgs
)

target_link_libraries(mov_node ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS mov_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()



