#include "../include/wheelchair_control/CANsender.hpp"

CANsender::CANsender(ros::NodeHandle& nh) {
    joystick_x = 0x00;
    joystick_y = 0x00;
    joy_sub = nh.subscribe("/joy", 10, &CANsender::joyCallback, this);

    if (!can_driver.init(CAN_INTERFACE, false)) {
        ROS_ERROR("Failed to initialize CAN interface!");
    }
}

CANsender::~CANsender() {}
// Function to inject a spoofed joystick frame in the wheelchair
void CANsender::injectRnetJoystickFrame() {
    // Create a CAN frame
    can::Frame can_frame;
    can_frame.id = RNET_JOYSTICK_ID;
    can_frame.dlc = 2;  // Only sending joystick X and Y
    
    // Set joystick values
    can_frame.data[0] = joystick_x;
    can_frame.data[1] = joystick_y;
    
    // Send the frame
    if (!can_driver.send(can_frame)) {
        ROS_WARN("Failed to send CAN frame");
    }
}

// Callback for joystick messages
void CANsender::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // Map joystick values to 0-255 range
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
