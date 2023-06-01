#include "keyboard_driver_ros_wrapper.hpp"
#include "keyboard_driver.hpp"

go_car_manual_control::car_control_command output_msg;

void KeyboardDriverRosWrapper::publishCmdVel() {
  _cmd_vel_pub.publish(output_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_driver");
  ros::NodeHandle nh;

  KeyboardDriverRosWrapper keyboardDriverRosWrapper(nh);
  output_msg.desired_steering_angle = 0.0;
  output_msg.desired_speed = 0.0;

  std::vector<std::string> keyboardEventFileOptions =
      keyboard_driver::getKeyboardEventFile();

  std::string keyboardEventFile = keyboardEventFileOptions.at(1);
  std::string globalPath_to_eventFile = "/dev/input/";
  std::string virtualKeyboardEventFile_path =
      globalPath_to_eventFile + keyboardEventFile;
  int virtualKeyboardEventFile_descriptor =
      open(virtualKeyboardEventFile_path.c_str(), O_RDONLY | O_NONBLOCK);

  ros::Rate rate(50);
  while (ros::ok()) {
    keyboard_driver::keys keysStatus =
        keyboard_driver::getKeyStatus(virtualKeyboardEventFile_descriptor);
    std::pair<double, double> cmdOutput =
        keyboard_driver::calculateCarCmd(keysStatus);
    output_msg.desired_speed = cmdOutput.first;
    output_msg.desired_steering_angle = cmdOutput.second;

    keyboardDriverRosWrapper.publishCmdVel();
    rate.sleep();
  }
  close(virtualKeyboardEventFile_descriptor);

  return 0;
}
