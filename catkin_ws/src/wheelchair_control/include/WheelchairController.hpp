#ifndef WHEELCHAIRCONTROLLER_HPP
#define WHEELCHAIRCONTROLLER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <functional>
#include "CANHandler.hpp"
#include <string>

class WheelchairController{
    // Define CAN interface (in my case always can0)
    private:

        const std::string CAN_INTERFACE = "can0";
        std::string RNET_JOYSTICK_ID = "02000000"; //static id for joystick frame
        std::string RNET_SPEED_ID = "0A040100"; //id for speed frame

        CANHandler can_handler;
        ros::Subscriber joy_sub;

        uint16_t joystick_x;
        uint16_t joystick_y;

        const int X_THRESHOLD = 8 * 0x10000 / 128;
        const int Y_THRESHOLD = 8 * 0x10000 / 128;

        uint16_t speed;

        ros::Time last_button_time_;
        ros::Duration button_delay_ = ros::Duration(0.2); // in seconds

        ros::Rate loop_rate;

        bool is_jailbreak_mode;

    public:

        

        WheelchairController(ros::NodeHandle& nh);

        ~WheelchairController();

        void injectRnetJoystickFrame();

        void injectRNETjailbreakFrame();

        void increaseSpeed();

        void spin();

        void decreaseSpeed();

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

        void canFrameCallback(const can::Frame& frame);

        void restartController();
};

#endif // WHEELCHAIRCONTROLLER_HPP

    

