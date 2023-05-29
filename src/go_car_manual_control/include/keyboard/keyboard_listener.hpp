#pragma once

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <termios.h>

class KeyboardListener {
public:
  KeyboardListener(ros::NodeHandle &nh) {
    _pressed_key_pub = nh.advertise<std_msgs::Int32>("/pressed_key_code", 10);
  }
  void publishPressedKeyCode(int key_code);

private:
  ros::Publisher _pressed_key_pub;
};
