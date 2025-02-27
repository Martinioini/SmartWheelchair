#include "JoystickHandler.hpp"
#include <string>

JoystickHandler::JoystickHandler() {
    // Initialize joystick
    joystick_fd = open("/dev/input/js0", O_RDONLY);

    if (joystick_fd < 0) {
        std::cerr << "Error opening joystick" << std::endl;
        return;
    }
    joystick_X = 0;
    joystick_Y = 0;

    //Retrieve joystick name
    std::vector<uint8_t> buf(64, 0);
    ioctl(joystick_fd, 0x80006a13 + (0x10000 * buf.size()), buf.data()); // JSIOCGNAME(len)
    joystick_name = std::string(buf.begin(), buf.end());
    std::cout << "Joystick name: " << joystick_name << std::endl;

    //Retrieve number of axes and buttons (the number is saved in a single byte)
    uint8_t num_axes = 0;
    uint8_t num_buttons = 0;
    ioctl(joystick_fd, 0x80016a11, &num_axes);     // JSIOCGAXES
    ioctl(joystick_fd, 0x80016a12, &num_buttons);  // JSIOCGBUTTONS
    numAxes_ = num_axes;
    numButtons_ = num_buttons;

    //Retrieve axis map
    std::vector<uint8_t> buf_axes(64, 0);
    ioctl(joystick_fd, 0x80406a32, buf_axes.data()); // JSIOCGAXMAP
}   

int JoystickHandler::getJoystickX() {
    return joystick_X;
}

int JoystickHandler::getJoystickY() {
    return joystick_Y;
}



    

