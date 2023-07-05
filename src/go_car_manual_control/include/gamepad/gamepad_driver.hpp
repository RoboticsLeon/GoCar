#pragma once

#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace controller_adapter {

constexpr double MAX_STEERING_ANGLE = 0.8;
constexpr double MAX_SPEED = 20.0;

enum class Gears {
  Reverse = -1,
  Neutral = 0,
  First = 1,
  Second = 2,
  Third = 3,
  Fourth = 4,
};

// Addition operator overload
void operator++(Gears &gear, int);

// Subtraction operator overload
void operator--(Gears &gear, int);

std::pair<double, double>
controller_input_processing(const std::vector<float> &axes,
                            const std::vector<int> &buttons);

} // namespace controller_adapter
