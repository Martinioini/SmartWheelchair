#pragma once
#include "CANHandler.hpp"
#include "JoystickHandler.hpp"

class WheelchairController {
public:
    WheelchairController();
    ~WheelchairController();

    bool initialize();
    void run();
    void stop();

    // RNET specific functions
    void setSpeedRange(uint8_t speedRange);
    void playShortBeep();
    void playSong();

private:
    CANHandler canHandler_;
    JoystickHandler joystickHandler_;
    bool running_;

    void sendJoystickFrame();
    void injectRNETJoystickFrame();
    std::string waitForJoystickFrame(double timeout);
}; 
