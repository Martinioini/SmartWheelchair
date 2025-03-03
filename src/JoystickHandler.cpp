#include "JoystickHandler.hpp"
#include <iostream>
#include <fcntl.h>      
#include <unistd.h>     
#include <sys/ioctl.h>  
#include <vector>
#include <cstring>      
#include <cmath>
#include <linux/joystick.h>  

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
    joystickFd_ = open("/dev/input/js0", O_RDONLY);

    if (joystickFd_ < 0) {
        std::cerr << "Error opening joystick" << std::endl;
        return;
    }
    joystickX_ = 0;
    joystickY_ = 0;

    //Retrieve joystick name
    std::vector<char> buf(64, 0);
    ioctl(joystickFd_, 0x80006a13 + (0x10000 * buf.size()), buf.data()); // JSIOCGNAME(len)
    joystickName_ = std::string(buf.data());
    std::cout << "Using joystick: " << joystickName_ << std::endl;

    //Retrieve number of axes and buttons (the number is saved in a single byte)
    uint8_t num_axes = 0;
    uint8_t num_buttons = 0;
    ioctl(joystickFd_, 0x80016a11, &num_axes);     // JSIOCGAXES
    ioctl(joystickFd_, 0x80016a12, &num_buttons);  // JSIOCGBUTTONS
    numAxes_ = num_axes;
    numButtons_ = num_buttons;

    //Retrieve axis map
    std::vector<uint8_t> buf_axes(64, 0);
    ioctl(joystickFd_, 0x80406a32, buf_axes.data()); // JSIOCGAXMAP

    // Populate the axis map
    for (int i = 0; i < numAxes_; i++) {
        uint8_t axis = buf_axes[i];
        std::string axis_name;
        
        // Get the axis name from AXIS_NAMES or use "unknown" if not found
        if (AXIS_NAMES.count(axis) > 0) {
            axis_name = AXIS_NAMES.at(axis);
        } else {
            axis_name = "uknown";
        }
        
        axisMap_.push_back(axis_name);
        axisStates_[axis_name] = 0.0;
    }

    //Retrieve buttons
    std::vector<uint16_t> buf_buttons(200, 0);
    ioctl(joystickFd_, 0x80406a34, buf_buttons.data()); // JSIOCGBTNMAP

    //Populate the button map
    for(int i = 0; i < numButtons_; i++){
        uint16_t button = buf_buttons[i];
        std::string button_name;

        //Get the button name from BUTTON_NAMES or use "unknown" if not found
        if (BUTTON_NAMES.count(button) > 0) {
            button_name = BUTTON_NAMES.at(button);
        } else {
            button_name = "unknown";
        }

        buttonMap_.push_back(button_name);
        buttonStates_[button_name] = 0;
    }   
}   

//deadzone: don't register small movements
void JoystickHandler::readJoystickThread() {
    struct js_event event;
    ssize_t bytes = read(joystickFd_, &event, sizeof(event));

    if(bytes != sizeof(event)){
        std::cerr << "Error reading joystick event" << std::endl;
        return;
    }

    if(event.type & JS_EVENT_AXIS){
        // Check if axis number is valid before accessing the map
        if(event.number < axisMap_.size() && axisMap_[event.number] != "") {
            std::string axisName = axisMap_[event.number];
            
            //x and y axis
            if(axisName == "x"){
                if(abs(event.value) > xthreshold ){
                    joystickX_ = 0x100 + int(event.value * 100 / 128) >> 8 & 0xFF;
                }
                else{
                    joystickX_ = 0;
                }
                // Print x-axis value
                std::cout << "X-axis: " << joystickX_ << " (raw: " << event.value << ")" << std::endl;
            }
            else if(axisName == "y") {
                if(abs(event.value) > ythreshold ){
                    joystickY_ = 0x100 - int(event.value * 100 / 128) >> 8 & 0xFF;
                }
                else{
                    joystickY_ = 0;
                }
                // Print y-axis value
                std::cout << "Y-axis: " << joystickY_ << " (raw: " << event.value << ")" << std::endl;
            }
            else{
                // Print other axis values
                std::cout << "Axis " << axisName << ": " << event.value << std::endl;
            }
        }
    }
}

JoystickHandler::~JoystickHandler() {
    if (joystickFd_ >= 0) {
        close(joystickFd_);
    }
}

int JoystickHandler::getJoystickX() const {
    return joystickX_;
}

int JoystickHandler::getJoystickY() const {
    return joystickY_;
}



    

