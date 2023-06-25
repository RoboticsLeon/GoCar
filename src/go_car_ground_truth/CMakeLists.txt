cmake_minimum_required(VERSION 3.0.2)
project(go_car_ground_truth)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp tf nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_truth_tf_publisher src/ground_truth_tf_publisher.cpp)
target_link_libraries(ground_truth_tf_publisher ${catkin_LIBRARIES})
