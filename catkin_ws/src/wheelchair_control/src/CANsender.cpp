#include "../include/CANsender.hpp"

ros::Time last_callback_time_;
double min_callback_interval_; // in seconds

CANsender::CANsender(ros::NodeHandle& nh) : can_handler(0) {
    // Initialize joystick values to 0 as in Python
    joystick_x = 0x00;
    joystick_y = 0x00;
    
    // Subscribe to joystick topic
    joy_sub = nh.subscribe("/joy", 10, &CANsender::joyCallback, this);
    
    // Send a neutral frame to initialize the wheelchair
    ROS_INFO("Sending initial neutral frame to initialize wheelchair...");
    injectRnetJoystickFrame();
}

CANsender::~CANsender() {
    // Send a neutral frame when shutting down
    joystick_x = 0x00;
    joystick_y = 0x00;
    injectRnetJoystickFrame();
}

// Function to inject a spoofed joystick frame in the wheelchair
void CANsender::injectRnetJoystickFrame() {
    // Format the CAN string in the same way as the Python version
    std::stringstream ss;
    ss << RNET_JOYSTICK_ID << "#";
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(joystick_x);
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(joystick_y);
    
    std::string canStr = ss.str();
    
    // Print the frame details
    ROS_INFO("Sending CAN frame: %s", canStr.c_str());
    
    // Send the frame using the CANHandler
    if (can_handler.sendFrame(canStr)) {
        ROS_DEBUG("Frame sent successfully");
    } else {
        ROS_WARN("Failed to send CAN frame");
    }
}

// Callback for joystick messages
void CANsender::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // Check if enough time has passed since the last callback
    ros::Time current_time = ros::Time::now();
    double elapsed = (current_time - last_callback_time_).toSec();
    
    // Get the raw joystick values
    float x_axis = msg->axes[0];  // Left stick horizontal (-1 to 1)
    float y_axis = msg->axes[1];  // Left stick vertical (-1 to 1)
    
    // Convert ROS joystick values (-1 to 1) to the same range as in Python
    int16_t x_value = static_cast<int16_t>(x_axis * 32767.0);
    int16_t y_value = static_cast<int16_t>(y_axis * 32767.0);
    
    // Process X axis - match Python's calculation exactly
    if(std::abs(x_value) > X_THRESHOLD){
        joystick_x = (0x100 + static_cast<int>(x_value * 100.0 / 128.0)) >> 8 & 0xFF;
    }
    else{
        joystick_x = 0;
    }

    // Process Y axis - INVERT the Y value to match Python's behavior
    if(std::abs(y_value) > Y_THRESHOLD){
        // Invert the y_value to match Python's behavior
        int16_t inverted_y = -y_value;
        joystick_y = (0x100 - static_cast<int>(inverted_y * 100.0 / 128.0)) >> 8 & 0xFF;
    }
    else{
        joystick_y = 0;
    }
    
    // Inject the frame
    injectRnetJoystickFrame();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheelchair_controller");
    ros::NodeHandle nh;
    
    // Initialize CAN interface
    CANsender can_sender(nh);
    
    ROS_INFO("Wheelchair controller started. Listening for joystick input...");
    
    // Set a high rate for the ROS loop
    ros::Rate loop_rate(5000); // 1000 Hz (1ms)
    
    while (ros::ok()) {
        // Process callbacks
        ros::spinOnce();
        // Sleep to maintain the desired rate
        loop_rate.sleep();
    }
    
    return 0;
}
