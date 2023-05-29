#include "keyboard_listener.hpp"

void KeyboardListener::publishPressedKeyCode(int key_code) {
  std_msgs::Int32 msg;
  msg.data = key_code;
  _pressed_key_pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_listener");
  ros::NodeHandle nh;

  KeyboardListener keyboardListener(nh);
  while (ros::ok()) {
    int key_code;
    struct termios standard_terminal_interface;
    struct termios temporal_terminal_interface;

    // Preventing Linux from buffering keystrokes
    tcgetattr(STDIN_FILENO, &standard_terminal_interface);
    temporal_terminal_interface = standard_terminal_interface;
    temporal_terminal_interface.c_lflag &= ~(ICANON | ECHO);
    temporal_terminal_interface.c_iflag |= IGNBRK;
    temporal_terminal_interface.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    temporal_terminal_interface.c_lflag &=
        ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    temporal_terminal_interface.c_cc[VMIN] = 1;
    temporal_terminal_interface.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &temporal_terminal_interface);

    key_code = getchar();

    // Reapply old settings for terminal interface
    tcsetattr(STDIN_FILENO, TCSANOW, &standard_terminal_interface);

    // check for SIGINT to abort node
    if (key_code == '\x03') {
      break;
    } else {
      keyboardListener.publishPressedKeyCode(key_code);
    }
  }

  return 0;
}