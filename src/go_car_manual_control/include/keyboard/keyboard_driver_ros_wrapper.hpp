#pragma once

#include "go_car_manual_control/car_control_command.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>

class KeyboardDriverRosWrapper {
public:
  KeyboardDriverRosWrapper(ros::NodeHandle &nh) {
    _cmd_vel_pub = nh.advertise<go_car_manual_control::car_control_command>(
        "/cmd_car", 10);
  }
  void publishCmdVel();

private:
  ros::Publisher _cmd_vel_pub;
};
