#ifndef CANSENDER_HPP
#define CANSENDER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <functional>

class CANsender{
    // Define CAN interface (in my case always can0)
    private:

        const std::string CAN_INTERFACE = "vcan0";
        const uint32_t RNET_JOYSTICK_ID = 0x0CFF0000; //todo: reverse engineer the real one
        can::SocketCANInterface can_driver;
        ros::Subscriber joy_sub;

    public:

        uint8_t joystick_x;
        uint8_t joystick_y;

        CANsender(ros::NodeHandle& nh);

        ~CANsender();

        void injectRnetJoystickFrame();

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

        void canFrameCallback(const can::Frame& frame);
};

#endif // CANSENDER_HPP

    

