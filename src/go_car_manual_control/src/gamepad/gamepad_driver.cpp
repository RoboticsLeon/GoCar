#include "gamepad_driver.hpp"

namespace controller_adapter {

void operator++(Gears &gear, int) {
  int newGear;
  if (gear != Gears::Sixth) {
    newGear = static_cast<int>(gear) + 1;
  } else {
    throw std::runtime_error("[controller_adapter]: Invalid gear value");
  }
  gear = static_cast<Gears>(newGear);
}

void operator--(Gears &gear, int) {
  int newGear;
  if (gear != Gears::Reverse) {
    newGear = static_cast<int>(gear) - 1;
  } else {
    throw std::runtime_error("[controller_adapter]: Invalid gear value");
  }
  gear = static_cast<Gears>(newGear);
}

std::pair<double, double>
controller_input_processing(const std::vector<float> &axes,
                            const std::vector<int> &buttons) {
  const double steering_percentage = axes[0];
  const double throttle_percentage = axes[4];
  static Gears current_gear{Gears::Neutral};
  const double desired_steering_angle =
      steering_percentage * MAX_STEERING_ANGLE;
  const double desired_speed =
      (throttle_percentage + static_cast<double>(current_gear)) *
      (MAX_SPEED / 6);

  if ((buttons[5] == 1) && (current_gear != Gears::Sixth)) {
    current_gear++;
    std::cout << "Marcha actual:" << static_cast<double>(current_gear)
              << std::endl;
  } else if ((buttons[4] == 1) && (current_gear != Gears::Reverse)) {
    current_gear--;
    std::cout << "Marcha actual:" << static_cast<double>(current_gear)
              << std::endl;
  }

  return std::make_pair(desired_steering_angle, desired_speed);
}

} // namespace controller_adapter
