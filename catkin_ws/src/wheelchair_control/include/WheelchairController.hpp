#ifndef WHEELCHAIRCONTROLLER_HPP
#define WHEELCHAIRCONTROLLER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <functional>
#include "ControllerHandler.hpp"
#include <string>

/**
 * @class WheelchairController
 * @brief A class to handle wheelchair control via joystick input.
 *
 * This class provides an interface between ROS joystick messages and the
 * wheelchair controller, managing button presses and joystick movements.
 */
class WheelchairController{
    // Define CAN interface (in my case always can0)
    private:
        /**
         * @brief Handler for wheelchair controller operations.
         */
        ControllerHandler controller_handler;
        
        /**
         * @brief Timestamp of the last button press.
         */
        ros::Time last_button_time_;
        
        /**
         * @brief Minimum delay between button presses to prevent bouncing.
         */
        ros::Duration button_delay_ = ros::Duration(0.2); // in seconds
        
        /**
         * @brief ROS subscriber for joystick input.
         */
        ros::Subscriber joy_sub;

    public:
        /**
         * @brief Constructor for the wheelchair controller.
         * @param nh ROS node handle for creating subscribers.
         */
        WheelchairController(ros::NodeHandle& nh);

        /**
         * @brief Destructor for the wheelchair controller.
         */
        ~WheelchairController();

        /**
         * @brief Callback function for joystick messages.
         * @param msg Pointer to the received joystick message.
         */
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

};

#endif // WHEELCHAIRCONTROLLER_HPP

