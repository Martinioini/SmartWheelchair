#ifndef WHEELCHAIRCONTROLLER_HPP
#define WHEELCHAIRCONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <functional>
#include "ControllerHandler.hpp"
#include <string>

/**
 * @class WheelchairController
 * @brief A class to handle wheelchair control via velocity and modality commands.
 *
 * This class provides an interface between ROS velocity/modality messages and the
 * wheelchair controller, managing speed, profile, and joystick movements.
 */
class WheelchairController{
    // Define CAN interface (in my case always can0)
    private:
        /**
         * @brief Handler for wheelchair controller operations.
         */
        ControllerHandler controller_handler;
        
        /**
         * @brief ROS subscriber for velocity commands.
         */
        ros::Subscriber cmd_vel_sub;
        
        /**
         * @brief ROS subscriber for modality commands.
         */
        ros::Subscriber modality_sub;

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
         * @brief Callback function for velocity command messages.
         * @param msg Pointer to the received velocity message.
         */
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        
        /**
         * @brief Callback function for modality command messages.
         * @param msg Pointer to the received modality message.
         */
        void modalityCallback(const std_msgs::Int8::ConstPtr& msg);
};

#endif // WHEELCHAIRCONTROLLER_HPP

