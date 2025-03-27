#ifndef CONTROLLERHANDLER_HPP
#define CONTROLLERHANDLER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <functional>
#include "CANHandler.hpp"
#include <string>

class ControllerHandler {
private:

    const std::string CAN_INTERFACE = "can0";
    const std::string RNET_JOYSTICK_ID = "02000000"; // static id for joystick frame
    const std::string RNET_SPEED_ID = "0A040100"; // id for speed frame
    const int numModes = 4;
    const int numProfiles = 5;

    CANHandler can_handler;
    ros::Subscriber joy_sub;

    uint16_t joystick_x;
    uint16_t joystick_y;

    const int X_THRESHOLD = 8 * 0x10000 / 128;
    const int Y_THRESHOLD = 8 * 0x10000 / 128;

    uint16_t speed;

    bool is_jailbreak_mode;

    uint8_t mode;

    uint8_t profile;

public:
    ControllerHandler();
    ~ControllerHandler();

    void injectRnetJoystickFrame();
    void injectRNETjailbreakFrame();
    void increaseSpeed();
    void decreaseSpeed();

    const std::string& getCANInterface() const;
    const std::string& getRnetJoystickId() const;
    const std::string& getRnetSpeedId() const;

    uint16_t getJoystickX() const;
    uint16_t getJoystickY() const;
    uint16_t getSpeed() const;

    bool getIsJailbreakMode() const;

    // Setters
    void setJoystick(float x, float y);
    void setSpeed(uint16_t s);
    
    void setLastButtonTime(const ros::Time& time);
    void setIsJailbreakMode(bool mode);
    void disableJailbreakMode();
    void setProfile(bool profile);
    void rnetSetMode(bool mode);
};

#endif // CONTROLLERHANDLER_HPP
