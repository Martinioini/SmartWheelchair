#pragma once
#include <string>
#include <map>

class JoystickHandler {
public:
    JoystickHandler();
    ~JoystickHandler();

    bool initJoystick();
    void readJoystickThread();
    
    // Getters for joystick position
    int getJoystickX() const;
    int getJoystickY() const;

    // Just declare the static members
    static const std::map<int, std::string> AXIS_NAMES;
    static const std::map<int, std::string> BUTTON_NAMES;

private:
    int joystickFd_;
    int joystickX_;
    int joystickY_;
    std::string joystickName_;
    uint8_t numAxes_;
    uint8_t numButtons_;
    
    // Instance maps for current states
    std::map<std::string, int> axisStates_;
    std::map<std::string, int> buttonStates_;
    
    void initAxisMaps();
    void initButtonMaps();
}; 
