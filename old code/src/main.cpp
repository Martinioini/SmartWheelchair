#include "CANHandler.hpp"
#include <iostream>
#include <iomanip>  // for std::setfill, std::setw
#include <unistd.h>  // Add this for sleep()
#include <vector>
#include "JoystickHandler.hpp"
int main() {
    // Try to initialize CAN handler with bus 0 (can0 or vcan0)
    CANHandler canHandler(0);
    
    // Test cases - array of CAN frame strings to test
    std::vector<std::string> test_frames = {
        "02000000#0040",  // Forward movement frame (same ID, modified data)
    };
    
    for (int i = 0; i < 100000; i++) {
        if(canHandler.sendFrame(test_frames[0])){
            std::cout << "Frame sent successfully" << std::endl;
            usleep(10000);

        } else {
            std::cout << "Frame sent failed" << std::endl;
        }
    }
}
