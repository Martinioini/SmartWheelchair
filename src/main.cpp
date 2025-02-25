#include "CANHandler.hpp"
#include <iostream>
#include <unistd.h>  // Add this for sleep()

int main() {
    // Try to initialize CAN handler with bus 0 (can0 or vcan0)
    CANHandler canHandler(0);
    
    // Simple verification that socket is open
    std::cout << "CAN Handler initialized. Testing connection..." << std::endl;
    
    // Keep the program running briefly to check for any error messages
    sleep(2);
    
    return 0;
}
