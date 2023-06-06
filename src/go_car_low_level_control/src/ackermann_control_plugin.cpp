#include "ackermann_control_plugin.hpp"

namespace gazebo_ackermann_control {

void AckermannControlPlugin::Load(gazebo::physics::ModelPtr model,
                                  sdf::ElementPtr sdf) {
  // Gazebo
  _model = model;
  _rightSteeringColumnJointController.reset(
      new gazebo::physics::JointController(_model));
  _leftSteeringColumnJointController.reset(
      new gazebo::physics::JointController(_model));
  _rightTractionWheelJointController.reset(
      new gazebo::physics::JointController(_model));
  _leftTractionWheelJointController.reset(
      new gazebo::physics::JointController(_model));

  // ROS
  getControllerParams();
  _cmdCarSub = _nh.subscribe("/cmd_car", 10,
                             &AckermannControlPlugin::cmdCarSubCallback, this);
  _carJointForces = _nh.advertise<go_car_low_level_control::car_joint_forces>(
      "/go_car/joint_forces", 10);

  // Module
  setPID(_right_steering_joint_name, _rightSteeringColumnJointController,
         _steering_control, control_type::POSITION);
  setPID(_left_steering_joint_name, _leftSteeringColumnJointController,
         _steering_control, control_type::POSITION);
  setPID(_right_traction_joint_name, _rightTractionWheelJointController,
         _speed_control, control_type::VELOCITY);
  setPID(_left_traction_joint_name, _leftTractionWheelJointController,
         _speed_control, control_type::VELOCITY);

  this->_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannControlPlugin::OnUpdate, this));
}

void AckermannControlPlugin::OnUpdate() {
  _rightSteeringColumnJointController->Update();
  _leftSteeringColumnJointController->Update();
  _rightTractionWheelJointController->Update();
  _leftTractionWheelJointController->Update();
  go_car_low_level_control::car_joint_forces msg;
  msg.right_steering_column =
      _model->GetJoint(_right_steering_joint_name)->GetForce(0);
  msg.left_steering_column =
      _model->GetJoint(_left_steering_joint_name)->GetForce(0);
  msg.right_rear_traction =
      _model->GetJoint(_right_traction_joint_name)->GetForce(0);
  msg.left_rear_traction =
      _model->GetJoint(_left_traction_joint_name)->GetForce(0);
  _carJointForces.publish(msg);
}

void AckermannControlPlugin::cmdCarSubCallback(
    const go_car_manual_control::car_control_commandConstPtr command) {

  _rightSteeringColumnJointController->SetPositionTarget(
      _model->GetJoint(_right_steering_joint_name)->GetScopedName(),
      command->desired_steering_angle);
  _leftSteeringColumnJointController->SetPositionTarget(
      _model->GetJoint(_left_steering_joint_name)->GetScopedName(),
      command->desired_steering_angle);
  _rightTractionWheelJointController->SetVelocityTarget(
      _model->GetJoint(_right_traction_joint_name)->GetScopedName(),
      (command->desired_speed) / (3.6 * wheel_radius));
  _leftTractionWheelJointController->SetVelocityTarget(
      _model->GetJoint(_left_traction_joint_name)->GetScopedName(),
      (command->desired_speed) / (3.6 * wheel_radius));
}

void AckermannControlPlugin::getControllerParams() {
  if (!_nh.getParam("/go_car/control/steering/joints_names/right",
                    _right_steering_joint_name)) {
    ROS_ERROR("No right steering joint name given");
  }
  if (!_nh.getParam("/go_car/control/steering/joints_names/left",
                    _left_steering_joint_name)) {
    ROS_ERROR("No left steering joint name given");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/p", _steering_control.p)) {
    ROS_ERROR("No p gain set for steering control");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/i", _steering_control.i)) {
    ROS_ERROR("No i gain set for steering control");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/d", _steering_control.d)) {
    ROS_ERROR("No d gain set for steering control");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/i_max",
                    _steering_control.i_max)) {
    ROS_ERROR("No i_max set for steering control");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/i_min",
                    _steering_control.i_min)) {
    ROS_ERROR("No i_min set for steering control");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/output_max",
                    _steering_control.output_max)) {
    ROS_ERROR("No output_max set for steering control");
  }
  if (!_nh.getParam("/go_car/control/steering/gains/output_min",
                    _steering_control.output_min)) {
    ROS_ERROR("No output_min set for steering control");
  }
  if (!_nh.getParam("/go_car/control/speed/joints_names/right",
                    _right_traction_joint_name)) {
    ROS_ERROR("No right traction joint name given");
  }
  if (!_nh.getParam("/go_car/control/speed/joints_names/left",
                    _left_traction_joint_name)) {
    ROS_ERROR("No left traction joint name given");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/p", _speed_control.p)) {
    ROS_ERROR("No p gain set for speed control");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/i", _speed_control.i)) {
    ROS_ERROR("No i gain set for speed control");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/d", _speed_control.d)) {
    ROS_ERROR("No d gain set for speed control");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/i_max",
                    _speed_control.i_max)) {
    ROS_ERROR("No i_max set for speed control");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/i_min",
                    _speed_control.i_min)) {
    ROS_ERROR("No i_min set for speed control");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/output_max",
                    _speed_control.output_max)) {
    ROS_ERROR("No output_max set for speed control");
  }
  if (!_nh.getParam("/go_car/control/speed/gains/output_min",
                    _speed_control.output_min)) {
    ROS_ERROR("No output_min set for speed control");
  }
}

void AckermannControlPlugin::setPID(
    std::string joint_name,
    gazebo::physics::JointControllerPtr joint_controller, gains constants,
    control_type controller_type) {
  if (_model->GetJoint(joint_name)) {
    joint_controller->AddJoint(_model->GetJoint(joint_name));
  } else {
    std::cout << joint_name << "not found in model" << std::endl;
  }
  if (controller_type == control_type::POSITION) {
    joint_controller->SetPositionPID(
        _model->GetJoint(joint_name)->GetScopedName(),
        gazebo::common::PID(constants.p, constants.i, constants.d,
                            constants.i_max, constants.i_min,
                            constants.output_max, constants.output_min));
  } else if (controller_type == control_type::VELOCITY) {
    joint_controller->SetVelocityPID(
        _model->GetJoint(joint_name)->GetScopedName(),
        gazebo::common::PID(constants.p, constants.i, constants.d,
                            constants.i_max, constants.i_min,
                            constants.output_max, constants.output_min));
  } else {
    ROS_ERROR("Wrong type for controller");
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AckermannControlPlugin)

} // namespace gazebo_ackermann_control