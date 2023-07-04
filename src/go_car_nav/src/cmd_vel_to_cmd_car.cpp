#include <geometry_msgs/Twist.h>
#include <go_car_manual_control/car_control_command.h>
#include <ros/ros.h>

ros::Publisher cmd_car_pub;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg) {
  // Create a custom message of type cmd_car
  go_car_manual_control::car_control_command cmd_car_msg;

  // Convert the cmd_vel message to cmd_car message
  cmd_car_msg.desired_speed = cmd_vel_msg->linear.x * 3.6; // kmh
  cmd_car_msg.desired_steering_angle = cmd_vel_msg->angular.z;

  cmd_car_pub.publish(cmd_car_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cmd_vel_converter");
  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub =
      nh.subscribe("cmd_vel_quit", 10, cmdVelCallback);
  cmd_car_pub =
      nh.advertise<go_car_manual_control::car_control_command>("cmd_car", 10);

  // Spin ROS
  ros::spin();

  return 0;
}
