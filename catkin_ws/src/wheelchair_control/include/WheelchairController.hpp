#ifndef WHEELCHAIRCONTROLLER_HPP
#define WHEELCHAIRCONTROLLER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/filter.h>
#include <socketcan_interface/interface.h>
#include <functional>
#include "ControllerHandler.hpp"
#include <string>

class WheelchairController{
    // Define CAN interface (in my case always can0)
    private:

        ControllerHandler controller_handler;
        ros::Time last_button_time_;
        ros::Duration button_delay_ = ros::Duration(0.2); // in seconds
        ros::Subscriber joy_sub;

    public:

        WheelchairController(ros::NodeHandle& nh);

        ~WheelchairController();

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

};

#endif // WHEELCHAIRCONTROLLER_HPP

    

