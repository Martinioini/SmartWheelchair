#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <socketcan_interface/SocketCAN.h>
#include <socketcan_interface/CanFrame.h>
#include <socketcan_interface/CanReceiver.h>
#include <socketcan_interface/CanSender.h>
#include <iomanip>
#include <sstream>

// Define CAN interface
const std::string CAN_INTERFACE = "can0";

// Define joystick CAN ID
const uint32_t RNET_JOYSTICK_ID = 0x0CFF0000; // Example ID, replace with correct one

// Global variables for joystick position
uint8_t joystick_x = 0x00;
uint8_t joystick_y = 0x00;

ros::Publisher can_pub;
socketcan_interface::SocketCAN can_driver;
socketcan_interface::CanSender::Ptr can_sender;

// Convert integer to 2-digit HEX string
std::string dec2hex(uint8_t value) {
    std::stringstream stream;
    stream << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(value);
    return stream.str();
}

// Function to inject a spoofed joystick frame
void injectRnetJoystickFrame() {
    socketcan_interface::CanFrame can_frame;
    can_frame.id = RNET_JOYSTICK_ID;
    can_frame.dlc = 2;  // Only sending joystick X and Y

    // Fill frame with joystick data
    can_frame.data[0] = joystick_x;
    can_frame.data[1] = joystick_y;

    // Send the frame ASAP
    can_sender->send(can_frame);
    
    ROS_INFO("Injected spoofed frame: ID=0x%X Data=[%02X %02X]", can_frame.id, can_frame.data[0], can_frame.data[1]);
}

// Callback for joystick messages
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // Convert joystick X (-1.0 to 1.0) to 0x00 - 0xFF
    joystick_x = static_cast<uint8_t>((msg->axes[0] + 1.0) * 127); // -1.0 -> 0x00, 1.0 -> 0xFF
    joystick_y = static_cast<uint8_t>((msg->axes[1] + 1.0) * 127);

    ROS_INFO("Joystick updated: X=%02X, Y=%02X", joystick_x, joystick_y);
}

// Callback for receiving CAN messages
void canReceiveCallback(const socketcan_interface::CanFrame &frame) {
    // Check if the received frame is the expected joystick frame
    if (frame.id == RNET_JOYSTICK_ID && frame.data[0] == 0x00 && frame.data[1] == 0x00) {
        ROS_INFO("Received expected joystick frame, injecting spoofed response...");
        injectRnetJoystickFrame();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_to_can_spoofer");
    ros::NodeHandle nh;

    // Initialize CAN driver
    if (!can_driver.init(CAN_INTERFACE, false)) {
        ROS_ERROR("Failed to initialize CAN interface %s", CAN_INTERFACE.c_str());
        return -1;
    }

    // Create CAN sender
    can_sender = can_driver.createSender();

    // Create CAN receiver and subscribe to joystick CAN ID
    socketcan_interface::CanReceiver can_receiver;
    can_receiver.setCallback(canReceiveCallback);
    can_driver.createReceiver(&can_receiver);

    // Subscribe to joystick topic
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallback);

    // Run ROS event loop
    ros::spin();
    return 0;
}
