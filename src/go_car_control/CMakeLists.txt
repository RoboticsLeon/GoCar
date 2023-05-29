cmake_minimum_required(VERSION 2.8.3)
project(go_car_control)
find_package(catkin REQUIRED COMPONENTS
                roscpp
                go_car_manual_control
                message_generation)
find_package(gazebo REQUIRED)

add_message_files(FILES car_joint_forces.msg)
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(CATKIN_DEPENDS roscpp
                DEPENDS gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(ackermann_control_plugin SHARED src/ackermann_control_plugin.cpp)
add_dependencies(ackermann_control_plugin go_car_control_generate_messages_cpp)
target_link_libraries(ackermann_control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
