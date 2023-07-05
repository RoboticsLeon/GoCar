#include "keyboard_driver.hpp"

namespace keyboard_driver {

void operator++(Gears &gear, int) {
  int newGear;
  if (gear != Gears::Fourth) {
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

std::vector<std::string> getKeyboardEventFile() {
  std::string virtualInputDevicesFile_path = "/proc/bus/input/devices";
  std::string readLine;
  std::string possibleEventFile;
  std::vector<std::string> keyboardEventFile;

  std::ifstream virtualInputDevicesFile(virtualInputDevicesFile_path);
  if (virtualInputDevicesFile.is_open()) {
    while (virtualInputDevicesFile) // iterates for every line in file
    {
      std::getline(virtualInputDevicesFile, readLine);
      if (readLine.find("Handlers") != std::string::npos) {
        std::regex pattern("event\\d+"); // matches "event"+<some number>
        std::smatch matches;
        if (std::regex_search(readLine, matches, pattern)) {
          possibleEventFile = matches.str(0);
        }
      }
      if (readLine.find("EV") !=
          std::string::npos) { // search for supported events characteristic
        // field of device
        size_t equalSignPos = readLine.find('=');
        size_t newLinePos = readLine.find('\n');
        std::string supportedEventsBitmask = // extract supported events bitmask
            readLine.substr(equalSignPos + 1, newLinePos - equalSignPos - 1);
        if ((std::stoi(supportedEventsBitmask, nullptr, 16) & 0x120013) ==
            0x120013) { // check for supporting minimum features for being
          // considered a keyboard
          std::cout << "Keyboard event file found: " << possibleEventFile
                    << std::endl;
          keyboardEventFile.push_back(possibleEventFile);
        }
      }
    }
  } else {
    std::cout << "Error opening input devices virtual file" << std::endl;
  }
  return keyboardEventFile;
}

keys getKeyStatus(int virtualKeyboardEventFileDescriptor) {
  struct input_event inputBuffer[BUFFER_SIZE];
  static keys KeysStatus;

  if (virtualKeyboardEventFileDescriptor != -1) { // check file could be opened
    ssize_t readedBytes = read(virtualKeyboardEventFileDescriptor, &inputBuffer,
                               sizeof(struct input_event) * BUFFER_SIZE);
    int numberReadedEvents = readedBytes / sizeof(struct input_event);
    for (int numberSavedEvents = 0; numberSavedEvents < numberReadedEvents;
         numberSavedEvents++) {
      struct input_event *event = &inputBuffer[numberSavedEvents];
      if (event->type == EV_KEY) {
        switch (event->code) {
        case KEY_UP:
          if (event->value == 1) {
            std::cout << "Tecla UP pulsada" << std::endl;
            KeysStatus.upArrowKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla UP soltada" << std::endl;
            KeysStatus.upArrowKey = key_status::non_pressed;
          }
          break;
        case KEY_LEFT:
          if (event->value == 1) {
            std::cout << "Tecla LEFT pulsada" << std::endl;
            KeysStatus.leftArrowKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla LEFT soltada" << std::endl;
            KeysStatus.leftArrowKey = key_status::non_pressed;
          }
          break;

        case KEY_RIGHT:
          if (event->value == 1) {
            std::cout << "Tecla RIGHT pulsada" << std::endl;
            KeysStatus.rightArrowKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla RIGHT soltada" << std::endl;
            KeysStatus.rightArrowKey = key_status::non_pressed;
          }
          break;

        case KEY_DOWN:
          if (event->value == 1) {
            std::cout << "Tecla DOWN pulsada" << std::endl;
            KeysStatus.downArrowKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla DOWN soltada" << std::endl;
            KeysStatus.downArrowKey = key_status::non_pressed;
          }
          break;

        case KEY_SLASH: // for spanish keyboard configuration this key is -
          if (event->value == 1) {
            std::cout << "Tecla - pulsada" << std::endl;
            KeysStatus.minusKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla - soltada" << std::endl;
            KeysStatus.minusKey = key_status::non_pressed;
          }
          break;

        case KEY_0:
          if (event->value == 1) {
            std::cout << "Tecla 0 pulsada" << std::endl;
            KeysStatus.numberZeroKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla 0 soltada" << std::endl;
            KeysStatus.numberZeroKey = key_status::non_pressed;
          }
          break;

        case KEY_1:
          if (event->value == 1) {
            std::cout << "Tecla 1 pulsada" << std::endl;
            KeysStatus.numberOneKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla 1 soltada" << std::endl;
            KeysStatus.numberOneKey = key_status::non_pressed;
          }
          break;

        case KEY_2:
          if (event->value == 1) {
            std::cout << "Tecla 2 pulsada" << std::endl;
            KeysStatus.numberTwoKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla 2 soltada" << std::endl;
            KeysStatus.numberTwoKey = key_status::non_pressed;
          }
          break;

        case KEY_3:
          if (event->value == 1) {
            std::cout << "Tecla 3 pulsada" << std::endl;
            KeysStatus.numberThreeKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla 3 soltada" << std::endl;
            KeysStatus.numberThreeKey = key_status::non_pressed;
          }
          break;

        case KEY_4:
          if (event->value == 1) {
            std::cout << "Tecla 4 pulsada" << std::endl;
            KeysStatus.numberFourKey = key_status::pressed;
          } else if (event->value == 0) {
            std::cout << "Tecla 4 soltada" << std::endl;
            KeysStatus.numberFourKey = key_status::non_pressed;
          }
          break;

        default:
          std::cout << "Tecla no valida pulsada" << std::endl;
          break;
        }
      }
    }
  } else {
    std::cout << "Error opening keyboard event virtual file" << std::endl;
  }

  return KeysStatus;
} // end getKeyStatus

std::pair<double, double> calculateCarCmd(keys keysStatus) {
  double cmdSpeed{0.0};
  double cmdSteering{0.0};
  static Gears current_gear{Gears::Neutral};

  if ((keysStatus.upArrowKey == key_status::pressed) &&
      (keysStatus.downArrowKey == key_status::non_pressed)) {
    if ((current_gear != Gears::Neutral) && (current_gear != Gears::Reverse)) {
      cmdSpeed = (1 + static_cast<double>(current_gear)) * (MAX_SPEED / 4);
    } else {
      cmdSpeed = 0.0;
    }
  } else if ((keysStatus.upArrowKey == key_status::non_pressed) &&
             (keysStatus.downArrowKey == key_status::pressed)) {
    if (current_gear == Gears::Reverse) {
      cmdSpeed = -(MAX_SPEED / 4);
    } else if (current_gear == Gears::Neutral) {
      cmdSpeed = 0.0;
    } else {
      cmdSpeed = (-1 + static_cast<double>(current_gear)) * (MAX_SPEED / 4);
    }
  } else {
    if (current_gear == Gears::Reverse) {
      cmdSpeed = 0.0;
    } else {
      cmdSpeed = static_cast<double>(current_gear) * (MAX_SPEED / 4);
    }
  }

  if ((keysStatus.leftArrowKey == key_status::pressed) &&
      (keysStatus.rightArrowKey == key_status::non_pressed)) {
    cmdSteering = MAX_STEERING_ANGLE;
  } else if ((keysStatus.leftArrowKey == key_status::non_pressed) &&
             (keysStatus.rightArrowKey == key_status::pressed)) {
    cmdSteering = -MAX_STEERING_ANGLE;
  } else {
    cmdSteering = 0.0;
  }

  if (keysStatus.minusKey == key_status::pressed) {
    current_gear = Gears::Reverse;
  } else if (keysStatus.numberZeroKey == key_status::pressed) {
    current_gear = Gears::Neutral;
  } else if (keysStatus.numberOneKey == key_status::pressed) {
    current_gear = Gears::First;
  } else if (keysStatus.numberTwoKey == key_status::pressed) {
    current_gear = Gears::Second;
  } else if (keysStatus.numberThreeKey == key_status::pressed) {
    current_gear = Gears::Third;
  } else if (keysStatus.numberFourKey == key_status::pressed) {
    current_gear = Gears::Fourth;
  }

  return std::pair<double, double>(cmdSpeed, cmdSteering);
} // namespace keyboard_driver

} // end namespace keyboard_driver
