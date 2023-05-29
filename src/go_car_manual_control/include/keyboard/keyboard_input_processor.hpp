#pragma once

#include "go_car_manual_control/car_control_command.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <vector>

constexpr double MAX_STEERING_ANGLE = 0.8;
constexpr double MAX_SPEED = 2;

class KeyboardInputProcessor {
public:
  KeyboardInputProcessor(ros::NodeHandle &nh) {
    _pressed_key_code =
        nh.subscribe("pressed_key_code", 10,
                     &KeyboardInputProcessor::callbackPressedKeySub, this);
    _cmd_vel_pub = nh.advertise<go_car_manual_control::car_control_command>(
        "/cmd_car", 10);
  }
  void publishCmdVel();
  void callbackPressedKeySub(const std_msgs::Int32ConstPtr msg);

private:
  ros::Subscriber _pressed_key_code;
  ros::Publisher _cmd_vel_pub;
};
