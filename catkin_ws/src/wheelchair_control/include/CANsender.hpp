#ifndef CANSENDER_HPP
#define CANSENDER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <functional>
#include "CANHandler.hpp"
#include <string>

class CANsender{
    // Define CAN interface (in my case always can0)
    private:

        const std::string CAN_INTERFACE = "can0";
        std::string RNET_JOYSTICK_ID = "02000000"; //static id for joystick frame
        CANHandler can_handler;
        ros::Subscriber joy_sub;

    public:

        uint16_t joystick_x;
        uint16_t joystick_y;

        const int X_THRESHOLD = 8 * 0x10000 / 128;
        const int Y_THRESHOLD = 8 * 0x10000 / 128;

        CANsender(ros::NodeHandle& nh);

        ~CANsender();

        void injectRnetJoystickFrame();

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

        void canFrameCallback(const can::Frame& frame);
};

#endif // CANSENDER_HPP

    

