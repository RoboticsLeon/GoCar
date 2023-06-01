#pragma once

#include "go_car_manual_control/car_control_command.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class ControllerAdapterRosWrapper {
public:
  ControllerAdapterRosWrapper(ros::NodeHandle &nh) {
    _joy_sub = nh.subscribe("joy", 10,
                            &ControllerAdapterRosWrapper::callbackJoySub, this);
    _cmd_vel_pub = nh.advertise<go_car_manual_control::car_control_command>(
        "/cmd_car", 10);
  }
  void callbackJoySub(const sensor_msgs::JoyConstPtr &input_msg);
  void publishCmdVel();

private:
  ros::Subscriber _joy_sub;
  ros::Publisher _cmd_vel_pub;
};
