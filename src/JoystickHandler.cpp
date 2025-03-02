#include "JoystickHandler.hpp"
#include <string>
#include <map>

// Define the static members in the implementation file
const std::map<int, std::string> JoystickHandler::AXIS_NAMES = {
    {0x00, "x"},
    {0x01, "y"},
    {0x02, "z"},
    {0x03, "rx"},
    {0x04, "ry"},
    {0x05, "rz"},
    {0x06, "throttle"},
    {0x07, "rudder"},
    {0x08, "wheel"},
    {0x09, "gas"},
    {0x0a, "brake"},
    {0x10, "hat0x"},
    {0x11, "hat0y"},
    {0x12, "hat1x"},
    {0x13, "hat1y"},
    {0x14, "hat2x"},
    {0x15, "hat2y"},
    {0x16, "hat3x"},
    {0x17, "hat3y"},
    {0x18, "pressure"},
    {0x19, "distance"},
    {0x1a, "tilt_x"},
    {0x1b, "tilt_y"},
    {0x1c, "tool_width"},
    {0x20, "volume"},
    {0x28, "misc"}
};

const std::map<int, std::string> JoystickHandler::BUTTON_NAMES = {
    {0x120, "trigger"},
    {0x121, "thumb"},
    {0x122, "thumb2"},
    {0x123, "top"},
    {0x124, "top2"},
    {0x125, "pinkie"},
    {0x126, "base"},
    {0x127, "base2"},
    {0x128, "base3"},
    {0x129, "base4"},
    {0x12a, "base5"},
    {0x12b, "base6"},
    {0x12f, "dead"},
    {0x130, "a"},
    {0x131, "b"},
    {0x132, "c"},
    {0x133, "x"},
    {0x134, "y"},
    {0x135, "z"},
    {0x136, "tl"},
    {0x137, "tr"},
    {0x138, "tl2"},
    {0x139, "tr2"},
    {0x13a, "select"},
    {0x13b, "start"},
    {0x13c, "mode"},
    {0x13d, "thumbl"},
    {0x13e, "thumbr"},
    {0x220, "dpad_up"},
    {0x221, "dpad_down"},
    {0x222, "dpad_left"},
    {0x223, "dpad_right"},
    {0x2c0, "dpad_left"},   // XBox 360 specific
    {0x2c1, "dpad_right"},  // XBox 360 specific
    {0x2c2, "dpad_up"},     // XBox 360 specific
    {0x2c3, "dpad_down"}    // XBox 360 specific
};

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

    //todo: retrieve axis map
    std::vector<uint8_t> buf_axes(64, 0);
    ioctl(joystick_fd, 0x80406a32, buf_axes.data()); // JSIOCGAXMAP
}   

int JoystickHandler::getJoystickX() {
    return joystick_X;
}

int JoystickHandler::getJoystickY() {
    return joystick_Y;
}



    

