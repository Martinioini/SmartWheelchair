#ifndef JOY_UTILITY_H
#define JOY_UTILITY_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <map>

class JoyUtility {
public:
    JoyUtility();
    ~JoyUtility() = default;

    // Initialize with parameters from ROS parameter server
    void initFromParams(ros::NodeHandle& nh);

    // Convert joystick values to velocity
    geometry_msgs::Twist joyToVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg);

    // Convert velocity to joystick values (for visualization/debugging)
    sensor_msgs::Joy velocityToJoy(const geometry_msgs::Twist& twist);

    // Set speed level (20-100)
    void setSpeedLevel(int level);

private:
    // Speed mappings (level -> speed in km/h)
    std::map<int, double> speed_mappings_;

    // Turning speed parameters
    double min_turning_speed_;    // rad/s
    double max_turning_speed_;    // rad/s

    // Current speed level (20-100)
    int current_speed_level_;

    // Convert speed level to actual speed in km/h
    double speedLevelToKmh(int level);

    // Convert km/h to m/s
    double kmhToMs(double kmh);

    // Convert m/s to km/h
    double msToKmh(double ms);
};

#endif // JOY_UTILITY_H 
