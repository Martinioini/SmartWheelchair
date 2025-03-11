#include "../include/wheelchair_control/CANsender.hpp"

ros::Time last_callback_time_;
double min_callback_interval_; // in seconds

CANsender::CANsender(ros::NodeHandle& nh) {
    joystick_x = 0x00;
    joystick_y = 0x00;
    

    min_callback_interval_ = 0.3;
    last_callback_time_ = ros::Time::now();
    
    joy_sub = nh.subscribe("/joy", 10, &CANsender::joyCallback, this);

    // Enable loopback mode for testing with vcan
    while(!can_driver.init(CAN_INTERFACE, false)) {
        ROS_ERROR("Failed to initialize CAN interface! Trying again...");
        ros::Duration(5.0).sleep();
    }
}

CANsender::~CANsender() {}

// Function to inject a spoofed joystick frame in the wheelchair
void CANsender::injectRnetJoystickFrame() {
    // Create a CAN frame
    can::Frame can_frame;
    can_frame.id = RNET_JOYSTICK_ID;
    can_frame.dlc = 8;
    
    // Set joystick values
    can_frame.data[0] = joystick_x;
    can_frame.data[1] = joystick_y;

    can_frame.is_extended = (RNET_JOYSTICK_ID > 0x7FF);
    
    // Print the frame details
    ROS_INFO("Sending CAN frame: ID = 0x%X, Data = [0x%02X, 0x%02X]", 
             can_frame.id, can_frame.data[0], can_frame.data[1]);
    
    // Send the frame
    if (!can_driver.send(can_frame)) {
        ROS_WARN("Failed to send CAN frame");
    }
}

// Callback for joystick messages
void CANsender::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // Check if enough time has passed since the last callback
    ros::Time current_time = ros::Time::now();
    double elapsed = (current_time - last_callback_time_).toSec();
    
    if (elapsed < min_callback_interval_) {
        // Not enough time has passed, skip this callback
        return;
    }
    
    // Update the last callback time
    last_callback_time_ = current_time;
    
    // Process the joystick message
    joystick_x = static_cast<uint8_t>((msg->axes[0] + 1.0) * 127);
    joystick_y = static_cast<uint8_t>((msg->axes[1] + 1.0) * 127);
    
    // Inject the frame
    injectRnetJoystickFrame();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheelchair_controller");
    ros::NodeHandle nh;
    
    // Initialize CAN interface
    CANsender can_sender(nh);
    
    ROS_INFO("Wheelchair controller started. Listening for joystick input...");
    
    ros::spin();
    
    return 0;
}
