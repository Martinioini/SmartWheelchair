#include "JoyUtility.h"
#include <cmath>

JoyUtility::JoyUtility() : current_speed_level_(50) // Default to middle speed
{
    // Initialize speed mappings
    speed_mappings_ = {
        {5, 0.5},
        {10, 0.8},
        {15, 1.2},
        {20, 1.5},
        {25, 1.9},
        {30, 2.3},
        {35, 2.6},
        {40, 3.0},
        {45, 3.4},
        {50, 3.8},
        {55, 4.1},
        {60, 4.5},
        {65, 4.8},
        {70, 5.2},
        {75, 5.6},
        {80, 6.0},
        {85, 6.3},
        {90, 6.7},
        {95, 7.0},
        {100, 7.2}
    };
}

void JoyUtility::initFromParams(ros::NodeHandle& nh)
{
    // Get current speed level from parameter server
    nh.param("current_speed_level", current_speed_level_, 50);

    // Get turning speed parameters
    nh.param("min_turning_speed", min_turning_speed_, 0.1);    // rad/s
    nh.param("max_turning_speed", max_turning_speed_, 1.0);    // rad/s
}

geometry_msgs::Twist JoyUtility::joyToVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    geometry_msgs::Twist twist;
    
    // Get current speed in km/h based on speed level
    double current_speed_kmh = speedLevelToKmh(current_speed_level_);
    
    // Convert to m/s
    double current_speed_ms = kmhToMs(current_speed_kmh);
    
    // Forward/backward movement (assuming axis 1 controls linear x)
    double linear_x = joy_msg->axes[1]; // -1 to 1
    
    // Turning movement (assuming axis 0 controls angular z)
    double angular_z = joy_msg->axes[0]; // -1 to 1
    
    // Apply speed limits
    if (linear_x > 0) {
        // Forward movement
        twist.linear.x = linear_x * current_speed_ms;
    } else {
        // Backward movement - use half of current speed
        twist.linear.x = linear_x * (current_speed_ms / 2.0);
    }
    
    // Apply turning limits
    if (angular_z > 0) {
        twist.angular.z = angular_z * max_turning_speed_;
    } else {
        twist.angular.z = angular_z * max_turning_speed_;
    }
    
    return twist;
}

sensor_msgs::Joy JoyUtility::velocityToJoy(const geometry_msgs::Twist& twist)
{
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(2); // Assuming 2 axes for simplicity
    
    // Convert current speed to km/h
    double current_speed_kmh = msToKmh(std::abs(twist.linear.x));
    
    // Convert to joystick value (-1 to 1)
    if (twist.linear.x > 0) {
        joy_msg.axes[1] = twist.linear.x / kmhToMs(speedLevelToKmh(current_speed_level_));
    } else {
        joy_msg.axes[1] = twist.linear.x / (kmhToMs(speedLevelToKmh(current_speed_level_)) / 2.0);
    }
    
    // Convert angular velocity to joystick value
    joy_msg.axes[0] = twist.angular.z / max_turning_speed_;
    
    return joy_msg;
}

void JoyUtility::setSpeedLevel(int level)
{
    // Ensure level is within bounds
    current_speed_level_ = std::max(20, std::min(100, level));
}

double JoyUtility::speedLevelToKmh(int level)
{
    // Find the closest lower level in the map
    auto it = speed_mappings_.lower_bound(level);
    if (it == speed_mappings_.end()) {
        // If level is higher than any in the map, use the highest available
        it = std::prev(speed_mappings_.end());
    } else if (it != speed_mappings_.begin() && it->first != level) {
        // If exact level not found, use the previous (lower) level
        it = std::prev(it);
    }
    return it->second;
}

double JoyUtility::kmhToMs(double kmh)
{
    return kmh / 3.6;
}

double JoyUtility::msToKmh(double ms)
{
    return ms * 3.6;
} 
