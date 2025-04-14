#include "../include/WheelchairController.hpp"

WheelchairController::WheelchairController(ros::NodeHandle& nh): controller_handler(){
    // Subscribe to command velocity and modality topics
    cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &WheelchairController::cmdVelCallback, this);
    modality_sub = nh.subscribe("/wheelchair_modality", 10, &WheelchairController::modalityCallback, this);
    
    ROS_INFO("WheelchairController initialized");
}

WheelchairController::~WheelchairController() {
    // Ensure the wheelchair is in a safe state when shutting down
    controller_handler.disableJailbreakMode();
}

// Callback for velocity command messages
void WheelchairController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float x_axis = msg->angular.z;  // Rotation
    float y_axis = msg->linear.x;   // Forward/backward
    
    controller_handler.setJoystick(x_axis, y_axis);
}

// Callback for modality command messages
void WheelchairController::modalityCallback(const std_msgs::Int8::ConstPtr& msg) {
    switch(msg->data) {
        case 0: // Disable
            ROS_INFO("Disabling jailbreak mode");
            controller_handler.disableJailbreakMode();
            break;
        case 1: // Increase profile
            ROS_INFO("Increasing profile");
            controller_handler.setProfile(true);
            break;
        case 2: // Decrease profile
            ROS_INFO("Decreasing profile");
            controller_handler.setProfile(false);
            break;
        case 3: // Increase speed
            ROS_INFO("Increasing speed");
            controller_handler.increaseSpeed();
            break;
        case 4: // Decrease speed
            ROS_INFO("Decreasing speed");
            controller_handler.decreaseSpeed();
            break;
        default:
            ROS_WARN("Unknown modality command: %d", msg->data);
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheelchair_controller");
    ros::NodeHandle nh;

    try {
        WheelchairController wheelchair_controller(nh);
        
        ROS_INFO("Wheelchair controller started. Listening for velocity and modality commands...");

        // Set the loop rate to 100Hz
        ros::Rate rate(100);
        
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in main");
    }

    return 0;
}
