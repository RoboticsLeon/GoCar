cmake_minimum_required(VERSION 3.0.2)
project(go_car_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  go_car_manual_control
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(FILES car_joint_forces.msg)
generate_messages()
catkin_package(CATKIN_DEPENDS roscpp tf nav_msgs message_runtime)
catkin_package(CATKIN_DEPENDS roscpp
                DEPENDS gazebo)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_truth_tf_publisher src/ground_truth_tf_publisher.cpp)
target_link_libraries(ground_truth_tf_publisher ${catkin_LIBRARIES})

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(ackermann_control_plugin SHARED src/ackermann_control_plugin.cpp)
add_dependencies(ackermann_control_plugin go_car_gazebo_generate_messages_cpp)
target_link_libraries(ackermann_control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})