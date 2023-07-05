#pragma once

#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/input.h>
#include <regex>
#include <string>
#include <unistd.h>

namespace keyboard_driver {

constexpr double MAX_STEERING_ANGLE = 0.8;
constexpr double MAX_SPEED = 20;
constexpr int BUFFER_SIZE{64};
constexpr int INPUT_DEVICE_ID_SELECTOR{1};

enum class key_status { pressed, non_pressed };

enum class Gears {
  Reverse = -1,
  Neutral = 0,
  First = 1,
  Second = 2,
  Third = 3,
  Fourth = 4,
};

struct keys {
  key_status upArrowKey{key_status::non_pressed};
  key_status leftArrowKey{key_status::non_pressed};
  key_status rightArrowKey{key_status::non_pressed};
  key_status downArrowKey{key_status::non_pressed};
  key_status minusKey{key_status::non_pressed};
  key_status numberZeroKey{key_status::non_pressed};
  key_status numberOneKey{key_status::non_pressed};
  key_status numberTwoKey{key_status::non_pressed};
  key_status numberThreeKey{key_status::non_pressed};
  key_status numberFourKey{key_status::non_pressed};
};

std::vector<std::string> getKeyboardEventFile();
keys getKeyStatus(int virtualKeyboardEventFileDescriptor);
std::pair<double, double> calculateCarCmd(keys keysStatus);

} // end namespace keyboard_driver