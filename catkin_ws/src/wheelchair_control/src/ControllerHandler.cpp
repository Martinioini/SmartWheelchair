#include "../include/ControllerHandler.hpp"

ControllerHandler::ControllerHandler() : can_handler(0) {
    joystick_x = 0x00;
    joystick_y = 0x00;

    speed = 0;

    is_jailbreak_mode = false;

    injectRnetJoystickFrame();
    injectRNETjailbreakFrame();
    
    can_handler.sendFrame("0a040100#0A");
}

//destructor
ControllerHandler::~ControllerHandler() {
    disableJailbreakMode();
}

void ControllerHandler::injectRnetJoystickFrame() {
    std::stringstream ss;
    ss << RNET_JOYSTICK_ID << "#";
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(joystick_x);
    ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(joystick_y);
    
    std::string canStr = ss.str();
    
    //ROS_INFO("Sending CAN frame: %s", canStr.c_str());
    
    // Send the frame
    if (can_handler.sendFrame(canStr)) {
        //ROS_DEBUG_STREAM("Frame sent successfully");
    } else {
        ROS_WARN("Failed to send CAN frame");
    }
}

void ControllerHandler::setJoystick(float x, float y){
    
    float x_axis = x;  // Left stick horizontal (-1 to 1)
    float y_axis = y;  // Left stick vertical (-1 to 1)
    
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

    joystick_x = new_x;
    joystick_y = new_y;

    // Inject the frame
    injectRnetJoystickFrame();
}

void ControllerHandler::rnetSetMode(bool mode){
    
    uint8_t prev_mode = this->mode;

    if((mode && prev_mode == numModes - 1) || (!mode && prev_mode == 0)){
        return;
    }

    std::stringstream ss2;
    if(mode){
        ss2 << "061#000" << std::setfill('0') << std::setw(2) << std::hex <<(prev_mode + 1) << "0000";
        this->mode++;
    }
    else{
        ss2 << "061#000" << std::setfill('0') << std::setw(2) << std::hex << (prev_mode - 1) << "0000";
        this->mode--;
    }
    ROS_INFO("Sending CAN frame: %s", ss2.str().c_str());
    can_handler.sendFrame(ss2.str());
}

void ControllerHandler::rnetRemoveMode(bool mode){
    
    uint8_t prev_mode = this->mode;

    if((mode && prev_mode == numModes - 1) || (!mode && prev_mode == 0)){
        return;
    }
    std::stringstream ss;
    ss << "061#400" << std::setfill('0') << std::setw(1) << std::hex << 0 << "0000";
    can_handler.sendFrame(ss.str());
}

void ControllerHandler::setProfile(bool profile){
     uint8_t prev_profile = this->profile;

     if(profile && prev_profile == numProfiles - 1 || !profile && prev_profile == 0){
        return;
     }

     std::stringstream ss;
     if(profile){
        ss << "051#000" << std::setfill('0') << std::setw(1) << std::hex << (this->profile + 1) << "0000";
        this->profile++;
     }
     else{
        ss << "051#000" << std::setfill('0') << std::setw(1) << std::hex << (this->profile - 1) << "0000";
        this->profile--;
     }
     ROS_INFO("Sending CAN frame: %s", ss.str().c_str());
     can_handler.sendFrame(ss.str());
}

void ControllerHandler::injectRNETjailbreakFrame() {

    for(int i = 0; i < 3; i++)
    {
        can_handler.sendFrame("0c000000#");
    }

    is_jailbreak_mode = true;
    can_handler.sendFrame("181c0100#0260000000000000");
}

void ControllerHandler::disableJailbreakMode(){
    is_jailbreak_mode = false;
    can_handler.sendFrame("181c0100#0260000000000000");
    can_handler.sendFrame("000#R");
}

void ControllerHandler::increaseSpeed() {

    if(speed == 100){
        return;
    }
    speed += 10;
    std::stringstream ss;
    ss << RNET_SPEED_ID << "#" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(speed);
    can_handler.sendFrame(ss.str());
}

void ControllerHandler::decreaseSpeed() {
    if(speed == 0){
        return;
    }
    speed -= 10;
    std::stringstream ss;
    ss << RNET_SPEED_ID << "#" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(speed);
    can_handler.sendFrame(ss.str());
}

const std::string& ControllerHandler::getCANInterface() const {
    return CAN_INTERFACE;
}

const std::string& ControllerHandler::getRnetJoystickId() const {
    return RNET_JOYSTICK_ID;
}

const std::string& ControllerHandler::getRnetSpeedId() const {
    return RNET_SPEED_ID;
}

uint16_t ControllerHandler::getJoystickX() const {
    return joystick_x;
}

uint16_t ControllerHandler::getJoystickY() const {
    return joystick_y;
}

uint16_t ControllerHandler::getSpeed() const {
    return speed;
}
