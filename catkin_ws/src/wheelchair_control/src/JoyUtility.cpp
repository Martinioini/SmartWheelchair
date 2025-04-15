#include "JoyUtility.h"
#include <cmath>

JoyUtility::JoyUtility() : current_profile_(0), current_speed_percentage_(20.0f) // Default to 20%
{
    // Initialize speed mappings
    speed_mappings_ = {
        {5, 0.5f},
        {10, 2.222f},
        {15, 1.2f},
        {20, 1.5f},
        {25, 5.278f},
        {30, 2.3f},
        {35, 2.6f},
        {40, 3.0f},
        {45, 3.4f},
        {50, 3.8f},
        {55, 4.1f},
        {60, 4.5f},
        {65, 4.8f},
        {70, 5.2f},
        {75, 5.6f},
        {80, 6.0f},
        {85, 6.3f},
        {90, 6.7f},
        {95, 7.0f},
        {100, 7.2f}
    };
}

void JoyUtility::initFromParams(ros::NodeHandle& nh)
{
    // Get current profile from parameter server
    nh.param("current_profile", current_profile_, 1);
    
    // Load speed levels for current profile
    loadProfileLevels(nh, current_profile_);
}

void JoyUtility::loadProfileLevels(ros::NodeHandle& nh, int profile)
{
    std::string profile_name = "profiles/profile" + std::to_string(profile);
    
    // Load speed levels for the profile
    nh.param(profile_name + "/max_forward_level", current_levels_.max_forward, 100);
    nh.param(profile_name + "/min_forward_level", current_levels_.min_forward, 20);
    nh.param(profile_name + "/max_backward_level", current_levels_.max_backward, 40);
    nh.param(profile_name + "/min_backward_level", current_levels_.min_backward, 10);
    nh.param(profile_name + "/max_turning_level", current_levels_.max_turning, 50);
    nh.param(profile_name + "/min_turning_level", current_levels_.min_turning, 15);
}

void JoyUtility::setSpeedPercentage(float percentage)
{
    // Clamp between 20% and 100%
    if(percentage > 100.0f || percentage < 20.0f){
        return;
    }
    current_speed_percentage_ = percentage;
    ROS_INFO("Speed percentage set to: %f", current_speed_percentage_);
}

geometry_msgs::Twist JoyUtility::joyToVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    geometry_msgs::Twist twist;
    
    float linear_x = joy_msg->axes[1]; // -1 to 1
    float angular_z = joy_msg->axes[0]; // -1 to 1
    
    // Calculate speed factor based on current percentage (20% -> 0.2, 100% -> 1.0)
    float speed_factor = current_speed_percentage_ / 100.0f;
    
    // Apply speed limits based on direction
    if (linear_x > 0) {
        // Forward movement
        float min_speed = speed_mappings_[current_levels_.min_forward];
        float max_speed = speed_mappings_[current_levels_.max_forward];
        // Calculate the current maximum speed based on speed percentage
        float current_max_speed = min_speed + (max_speed - min_speed) * speed_factor;
        // Calculate final speed based on joystick position
        float speed = current_max_speed * linear_x;
        twist.linear.x = speed;
    } else {
        // Backward movement
        float min_speed = speed_mappings_[current_levels_.min_backward];
        float max_speed = speed_mappings_[current_levels_.max_backward];
        // Calculate the current maximum speed based on speed percentage
        float current_max_speed = min_speed + (max_speed - min_speed) * speed_factor;
        // Calculate final speed based on joystick position
        float speed = current_max_speed * linear_x;
        twist.linear.x = -speed;
    }
    
    // Apply turning limits
    float min_turning = speed_mappings_[current_levels_.min_turning];
    float max_turning = speed_mappings_[current_levels_.max_turning];
    // Calculate the current maximum turning speed based on speed percentage
    float current_max_turning = min_turning + (max_turning - min_turning) * speed_factor;
    // Calculate final turning speed based on joystick position
    float turning_speed = current_max_turning * angular_z;
    twist.angular.z = turning_speed;
    
    return twist;
}

sensor_msgs::Joy JoyUtility::velocityToJoy(const geometry_msgs::Twist& twist)
{
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(2);
    
    float speed_factor = current_speed_percentage_ / 100.0f;
    
    // Convert linear velocity to joystick value
    if (twist.linear.x > 0) {
        float min_speed = current_levels_.min_forward;
        float max_speed = current_levels_.max_forward;
        float current_max_speed = min_speed + (max_speed - min_speed) * speed_factor;
        joy_msg.axes[1] = twist.linear.x / current_max_speed;
    } else {
        float min_speed = current_levels_.min_backward;
        float max_speed = current_levels_.max_backward;
        float current_max_speed = min_speed + (max_speed - min_speed) * speed_factor;
        joy_msg.axes[1] = -twist.linear.x / current_max_speed;
    }
    
    // Convert angular velocity to joystick value
    float min_turning = current_levels_.min_turning;
    float max_turning = current_levels_.max_turning;
    float current_max_turning = min_turning + (max_turning - min_turning) * speed_factor;
    joy_msg.axes[0] = twist.angular.z / current_max_turning;
    
    return joy_msg;
}

void JoyUtility::setProfile(int profile)
{
    // Make profile circular (0-4)
    if (profile < 0) {
        current_profile_ = 4;
    } else if (profile > 4) {
        current_profile_ = 0;
    } else {
        current_profile_ = profile;
    }
    
    // Load new profile levels
    ros::NodeHandle nh;
    loadProfileLevels(nh, current_profile_);
}

void JoyUtility::increaseProfile()
{
    setProfile(current_profile_ + 1);
}

void JoyUtility::decreaseProfile()
{
    setProfile(current_profile_ - 1);
}

void JoyUtility::increaseSpeed()
{
    setSpeedPercentage(current_speed_percentage_ + 10.0f);
}

void JoyUtility::decreaseSpeed()
{
    setSpeedPercentage(current_speed_percentage_ - 10.0f);
}


