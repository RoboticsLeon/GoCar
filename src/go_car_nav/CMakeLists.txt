cmake_minimum_required(VERSION 3.0.2)
project(go_car_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  go_car_manual_control
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    go_car_manual_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_vel_to_cmd_car src/cmd_vel_to_cmd_car.cpp)
add_dependencies(cmd_vel_to_cmd_car go_car_manual_control_generate_messages_cpp)
target_link_libraries(cmd_vel_to_cmd_car ${catkin_LIBRARIES})
