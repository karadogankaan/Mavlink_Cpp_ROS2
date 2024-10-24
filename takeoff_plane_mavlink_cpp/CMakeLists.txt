cmake_minimum_required(VERSION 3.10)
project(takeoff)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/mavlink)

find_package(ament_cmake REQUIRED)
find_package(mavlink REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/src/arm/mavlink/include/mavlink/v2.0)

include_directories(${CMAKE_SOURCE_DIR}/src/arm/mavlink/include)
include_directories(${MAVLINK_DIR}/include/mavlink/v2.0)

add_executable(takeoff_node src/takeoff_drone.cpp)

#ament_target_dependencies(
#arm_node
#rclcpp
#std_msgs
#)
#buraya tam olarak ne kutuphane yazacaksan onu yaz.

target_link_libraries(takeoff_node ${CMAKE_THREAD_LIBS_INIT})

find_package(Threads REQUIRED)
target_link_libraries(takeoff_node PRIVATE Threads::Threads)

install(TARGETS takeoff_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()



