#include "keyboard_driver.hpp"

namespace keyboard_driver {

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

  if ((keysStatus.upArrowKey == key_status::pressed) &&
      (keysStatus.downArrowKey == key_status::non_pressed)) {
    cmdSpeed = MAX_SPEED;
  } else if ((keysStatus.upArrowKey == key_status::non_pressed) &&
             (keysStatus.downArrowKey == key_status::pressed)) {
    cmdSpeed = -MAX_SPEED;
  } else {
    cmdSpeed = 0.0;
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

  return std::pair<double, double>(cmdSpeed, cmdSteering);
}

} // end namespace keyboard_driver
