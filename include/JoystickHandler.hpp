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

private:
    int joystickFd_;
    int joystickX_;
    int joystickY_;
    
    // Maps for axis and button names
    std::map<int, std::string> axisNames_;
    std::map<int, std::string> buttonNames_;
    
    void initAxisMaps();
    void initButtonMaps();
}; 
