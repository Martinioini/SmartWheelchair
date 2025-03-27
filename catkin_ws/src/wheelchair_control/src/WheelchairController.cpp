#include "../include/WheelchairController.hpp"

WheelchairController::WheelchairController(ros::NodeHandle& nh): controller_handler(){

    last_button_time_ = ros::Time::now();
    
    // Subscribe to joystick topic
    joy_sub = nh.subscribe("/joy", 10, &WheelchairController::joyCallback, this);
}

WheelchairController::~WheelchairController() {
}

// Callback for joystick messages
void WheelchairController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {

   if (ros::Time::now() - last_button_time_ > button_delay_) {
       //button 
       if (msg->buttons[1] == 1) {
           controller_handler.increaseSpeed();
           ROS_INFO("Increasing speed");
       } 
       //button A
       else if (msg->buttons[2] == 1) {
           controller_handler.decreaseSpeed();
           ROS_INFO("Decreasing speed");
       }
       //button Lb
       else if (msg->buttons[6] == 1) {
           controller_handler.rnetRemoveMode(false);
           ros::Duration(0.1).sleep();
           controller_handler.rnetSetMode(false);
           ROS_INFO("Decreasing mode");
       }
       //button Rb
       else if (msg->buttons[7] == 1) {
           controller_handler.rnetRemoveMode(true);
           ros::Duration(0.1).sleep();
           controller_handler.rnetSetMode(true);
           ROS_INFO("Increasing mode");
       }
       else if (msg->buttons[4] == 1) {
           controller_handler.setProfile(false);
           ROS_INFO("Decreasing profile");
       }
       else if (msg->buttons[5] == 1) {
           controller_handler.setProfile(true);
           ROS_INFO("Increasing profile");
       }
       last_button_time_ = ros::Time::now();
   }
   
    float x_axis = msg->axes[0];  // Left stick horizontal (-1 to 1)
    float y_axis = msg->axes[1];  // Left stick vertical (-1 to 1)

    controller_handler.setJoystick(x_axis, y_axis);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheelchair_controller");
    ros::NodeHandle nh;

    try {
        WheelchairController wheelchair_controller(nh);
        
        ROS_INFO("Wheelchair controller started. Listening for joystick input...");

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
