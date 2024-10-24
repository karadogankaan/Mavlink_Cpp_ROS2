cmake_minimum_required(VERSION 3.10)
project(arm)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(mavlink REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)



#add_library(arm_node src/arming_drone.cpp)

set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/mavlink)
add_executable(arm_node src/arming_drone.cpp)


ament_target_dependencies(
arm_node
rclcpp
std_msgs
)
#buraya tam olarak ne kutuphane bagladiysan onu yaz.


include_directories(${MAVLINK_DIR}/include/mavlink/v2.0)
#add_library

add_executable(arm src/arming_drone.cpp)
target_link_libraries(arm_node ${CMAKE_THREAD_LIBS_INIT})

find_package(Threads REQUIRED)
target_link_libraries(arm PRIVATE Threads::Threads)

install(TARGETS arm_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()

