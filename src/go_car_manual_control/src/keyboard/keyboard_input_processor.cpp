#include "keyboard_input_processor.hpp"

go_car_manual_control::car_control_command output_msg;
std::vector<int> key_code;

void KeyboardInputProcessor::callbackPressedKeySub(
    const std_msgs::Int32ConstPtr msg) {
  key_code.push_back(msg->data);
}

void KeyboardInputProcessor::publishCmdVel() {
  _cmd_vel_pub.publish(output_msg);
}

int extract_first_value(std::vector<int> vector) {
  int key_value{-1};
  if (vector.size() != 0) {
    key_value = vector.at(0);
    key_code.erase(vector.begin());
  }
  return key_value;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_input_processor");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  KeyboardInputProcessor controllerAdapterRosWrapper(nh);
  output_msg.desired_steering_angle = 0.0;
  output_msg.desired_speed = 0.0;
  int key_value{0};

  ros::Rate rate(50);
  while (ros::ok()) {
    key_value = extract_first_value(key_code);
    // check for ANSI escape
    if (key_value == '\033') {
      key_value = key_code.at(0);
      key_code.erase(key_code.begin());
      // confirm ANSI escape sequence
      if (key_value == '[') {
        key_value = key_code.at(0);
        key_code.erase(key_code.begin());
        switch (key_value) {
        case 'A':
          output_msg.desired_speed = MAX_SPEED;
          std::cout << "SPEED: " << output_msg.desired_speed << std::endl;
          break;
        case 'B':
          output_msg.desired_speed = -MAX_SPEED;
          std::cout << "SPEED: " << output_msg.desired_speed << std::endl;
          break;
        case 'C':
          output_msg.desired_steering_angle = MAX_STEERING_ANGLE;
          std::cout << "STEERING: " << output_msg.desired_steering_angle
                    << std::endl;
          break;
        case 'D':
          output_msg.desired_steering_angle = -MAX_STEERING_ANGLE;
          std::cout << "STEERING: " << output_msg.desired_steering_angle
                    << std::endl;
          break;
        default:
          break;
        }
      }
    } else {
      output_msg.desired_speed = 0;
      output_msg.desired_steering_angle = 0;
    }
    controllerAdapterRosWrapper.publishCmdVel();
    ros::spinOnce();
    rate.sleep();
  }

  ros::waitForShutdown();
}