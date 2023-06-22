cmake_minimum_required(VERSION 3.0.2)
project(go_car_manual_control)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             message_generation)

add_message_files(FILES car_control_command.msg)
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include/gamepad include/keyboard ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(gamepad_driver src/gamepad/gamepad_driver.cpp src/gamepad/gamepad_driver_ros_wrapper.cpp)
add_dependencies(gamepad_driver go_car_manual_control_generate_messages_cpp)
target_link_libraries(gamepad_driver ${catkin_LIBRARIES})

add_executable(keyboard_driver src/keyboard/keyboard_driver_ros_wrapper.cpp src/keyboard/keyboard_driver.cpp)
add_dependencies(keyboard_driver go_car_manual_control_generate_messages_cpp)
target_link_libraries(keyboard_driver ${catkin_LIBRARIES})

