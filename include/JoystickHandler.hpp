#pragma once
#include <string>
#include <map>
#include <vector>

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

    float xthreshold = 8 * 0x10000 / 128;
    float ythreshold = 8 * 0x10000 / 128;
    
    // Maps to store the axis and button indices
    std::vector<std::string> axisMap_;
    std::vector<std::string> buttonMap_;
    
    // Instance maps for current states
    std::map<std::string, float> axisStates_;
    std::map<std::string, int> buttonStates_;
    
    void initAxisMaps();
    void initButtonMaps();
}; 
