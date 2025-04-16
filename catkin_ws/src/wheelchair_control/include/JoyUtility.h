#ifndef JOY_UTILITY_H
#define JOY_UTILITY_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <map>
#include <string>

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

    // Set current profile (1-5)
    void setProfile(int profile);

    // Get current profile
    int getCurrentProfile() const { return current_profile_; }

    // Set speed percentage (0-100)
    void setSpeedPercentage(float percentage);

    // Profile control methods
    void increaseProfile();
    void decreaseProfile();

    // Speed control methods
    void increaseSpeed();
    void decreaseSpeed();
    float getCurrentSpeedPercentage() const { return current_speed_percentage_; }

private:
    // Speed mappings (level -> speed in m/s)
    std::map<int, float> speed_mappings_;

    // Current profile (1-5)
    int current_profile_;

    // Current speed percentage (0-100)
    float current_speed_percentage_;

    // Speed levels for current profile
    struct SpeedLevels {
        int max_forward;
        int min_forward;
        int max_backward;
        int min_backward;
        int max_turning;
        int min_turning;
    } current_levels_;

    // Load speed levels for a profile
    void loadProfileLevels(ros::NodeHandle& nh, int profile);
};

#endif // JOY_UTILITY_H 
