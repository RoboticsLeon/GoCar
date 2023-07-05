#include <ros/ros.h>

ros::Subscriber cmd_car_pub;
ros::

    float speed = 0;
float steering_angle = 0;

struct ground_truth_coordinate {
  float gt_x_coordinate = 0;
  float gt_y_coordinate = 0;
  float gt_z_coordinate = 0;
};

struct estimated_localization_coordinate {
  float estimated_localization_x_coordinate = 0;
  float estimated_localization_y_coordinate = 0;
  float estimated_localization_z_coordinate = 0;
};

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg) {
  // Create a custom message of type cmd_car
  go_car_manual_control::car_control_command cmd_car_msg;

  // Convert the cmd_vel message to cmd_car message
  cmd_car_msg.desired_speed = cmd_vel_msg->linear.x * 3.6; // kmh
  cmd_car_msg.desired_steering_angle = cmd_vel_msg->angular.z;

  cmd_car_pub.publish(cmd_car_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "telemetry_node");
  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
  cmd_car_pub =
      nh.advertise<go_car_manual_control::car_control_command>("cmd_car", 10);

  // Spin ROS
  ros::spin();

  return 0;
}
