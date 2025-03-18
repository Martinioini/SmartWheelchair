#include "../include/WheelchairController.hpp"

WheelchairController::WheelchairController(ros::NodeHandle& nh) : can_handler(0) {

    joystick_x = 0x00;
    joystick_y = 0x00;

    last_button_time_ = ros::Time::now();
    
    // Subscribe to joystick topic
    joy_sub = nh.subscribe("/joy", 10, &WheelchairController::joyCallback, this);
    
    // Send a neutral frame to initialize the wheelchair
    ROS_INFO("Sending initial neutral frame to initialize wheelchair...");
    injectRnetJoystickFrame();
    can_handler.sendFrame("0a040100#0A");
}

WheelchairController::~WheelchairController() {
    // Send a neutral frame when shutting down
    joystick_x = 0x00;
    joystick_y = 0x00;
    injectRnetJoystickFrame();
}

void WheelchairController::increaseSpeed( ){
    if(speed == 100){
        return;
    }
    speed += 10;
    std::stringstream ss;
    ss << RNET_SPEED_ID << "#" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(speed);
    can_handler.sendFrame(ss.str());
}

void WheelchairController::decreaseSpeed(){
    if(speed == 0){
        return;
    }
    speed -= 10;
    std::stringstream ss;
    ss << RNET_SPEED_ID << "#" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(speed);
    can_handler.sendFrame(ss.str());
}

// Inject a spoofed joystick frame in the wheelchair
void WheelchairController::injectRnetJoystickFrame() {

    std::stringstream ss;
    ss << RNET_JOYSTICK_ID << "#";
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(joystick_x);
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(joystick_y);
    
    std::string canStr = ss.str();
    
    ROS_INFO("Sending CAN frame: %s", canStr.c_str());
    
    // Send the frame
    if (can_handler.sendFrame(canStr)) {
        ROS_DEBUG("Frame sent successfully");
    } else {
        ROS_WARN("Failed to send CAN frame");
    }
}

// Callback for joystick messages
void WheelchairController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {

   if (ros::Time::now() - last_button_time_ > button_delay_) {
       if (msg->buttons[1] == 1) {
           increaseSpeed();
           ROS_INFO("Increasing speed");
       } 
       else if (msg->buttons[2] == 1) {
           decreaseSpeed();
           ROS_INFO("Decreasing speed");
       }
       last_button_time_ = ros::Time::now();
   }
    
    float x_axis = msg->axes[0];  // Left stick horizontal (-1 to 1)
    float y_axis = msg->axes[1];  // Left stick vertical (-1 to 1)
    
    int16_t x_value = static_cast<int16_t>(x_axis * 32767.0);
    int16_t y_value = static_cast<int16_t>(y_axis * 32767.0);

    uint16_t new_x;
    uint16_t new_y;

    // Process X axis
    if(std::abs(x_value) > X_THRESHOLD){
        new_x = (0x100 + static_cast<int>(x_value * 100.0 / 128.0)) >> 8 & 0xFF;
    }
    else{
        new_x = 0;
    }

    // Process Y axis - INVERT the Y value 
    if(std::abs(y_value) > Y_THRESHOLD){

        int16_t inverted_y = -y_value;
        new_y = (0x100 - static_cast<int>(inverted_y * 100.0 / 128.0)) >> 8 & 0xFF;
    }
    else{
        new_y = 0;
    }
    
    // If the joystick is not moving, don't send a frame
    if(new_x == 0 && new_y == 0 && joystick_x == 0 && joystick_y == 0){
        return;
    }

    joystick_x = new_x;
    joystick_y = new_y;

    // Inject the frame
    injectRnetJoystickFrame();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheelchair_controller");
    ros::NodeHandle nh;
    
    // Initialize CAN interface
    WheelchairController wheelchair_controller(nh);
    
    ROS_INFO("Wheelchair controller started. Listening for joystick input...");
    
    ros::Rate loop_rate(300); // 100 Hz 
    
    while (ros::ok()) {
        // Process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
