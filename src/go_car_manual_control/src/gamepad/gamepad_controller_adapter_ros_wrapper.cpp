#include "gamepad_controller_adapter_ros_wrapper.hpp"
#include "gamepad_controller_adapter.hpp"

go_car_manual_control::car_control_command output_msg;

void ControllerAdapterRosWrapper::callbackJoySub(
    const sensor_msgs::JoyConstPtr &input_msg) {
  const std::vector<float> axes(input_msg->axes);
  const std::vector<int> buttons(input_msg->buttons);

  std::pair<double, double> calculated_command =
      controller_adapter::controller_input_processing(axes, buttons);
  output_msg.desired_steering_angle = calculated_command.first;
  output_msg.desired_speed = calculated_command.second;
}

void ControllerAdapterRosWrapper::publishCmdVel() {
  _cmd_vel_pub.publish(output_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_adapter");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ControllerAdapterRosWrapper controllerAdapterRosWrapper(nh);
  output_msg.desired_steering_angle = 0.0;
  output_msg.desired_speed = 0.0;
  ros::Rate rate(50);
  while (ros::ok()) {
    controllerAdapterRosWrapper.publishCmdVel();
    rate.sleep();
  }

  ros::waitForShutdown();
}