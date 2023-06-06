#pragma once

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include "go_car_low_level_control/car_joint_forces.h"
#include "go_car_manual_control/car_control_command.h"
#include <ros/ros.h>

namespace gazebo_ackermann_control {

constexpr double wheel_radius{0.356};

struct gains {
  double p{0};
  double i{0};
  double d{0};
  double i_max{0};
  double i_min{0};
  double output_max{0};
  double output_min{0};
};

enum class control_type { POSITION = 1, VELOCITY = 2 };

class AckermannControlPlugin : public gazebo::ModelPlugin {

public:
  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void OnUpdate();
  void cmdCarSubCallback(
      const go_car_manual_control::car_control_commandConstPtr command);
  void getControllerParams();
  void setPID(std::string joint_name,
              gazebo::physics::JointControllerPtr joint_controller,
              gains constants, control_type controller_type);

private:
  // Gazebo
  gazebo::physics::ModelPtr _model;
  gazebo::event::ConnectionPtr _updateConnection;
  gazebo::physics::JointControllerPtr _rightSteeringColumnJointController;
  gazebo::physics::JointControllerPtr _leftSteeringColumnJointController;
  gazebo::physics::JointControllerPtr _rightTractionWheelJointController;
  gazebo::physics::JointControllerPtr _leftTractionWheelJointController;

  // ROS
  ros::NodeHandle _nh;
  ros::Subscriber _cmdCarSub;
  ros::Publisher _carJointForces;

  // Module
  std::string _right_steering_joint_name;
  std::string _left_steering_joint_name;
  std::string _right_traction_joint_name;
  std::string _left_traction_joint_name;
  gains _steering_control;
  gains _speed_control;
};

} // namespace gazebo_ackermann_control