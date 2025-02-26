#include "CANHandler.hpp"
#include <iostream>
#include <iomanip>  // for std::setfill, std::setw
#include <unistd.h>  // Add this for sleep()
#include <vector>

int main() {
    // Try to initialize CAN handler with bus 0 (can0 or vcan0)
    CANHandler canHandler(0);
    
    // Test cases - array of CAN frame strings to test
    std::vector<std::string> test_frames = {
        "181c0100#0260000000000000",  // Regular frame
        "123#1A2B3C",                 // Short frame
        "18FF0123#R"                  // RTR frame
    };
    
    // Test each frame
    for (const auto& test_frame : test_frames) {
        std::cout << "\nTesting CAN string: " << test_frame << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        // Convert string to frame
        can_frame frame = canHandler.buildFrame(test_frame);
        
        // Print the frame details
        std::cout << "Frame ID: 0x" << std::hex << std::setfill('0') << std::setw(8) 
                  << frame.can_id << std::endl;
        std::cout << "DLC: " << std::dec << static_cast<int>(frame.can_dlc) << std::endl;
        
        // Print data bytes
        std::cout << "Data: ";
        for(int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) 
                      << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::endl;
        
        // Convert frame back to string
        std::string recovered_string = canHandler.dissectFrame(frame);
        std::cout << "Recovered string: " << recovered_string << std::endl;
        
        // Verify if the conversion was lossless
        if (test_frame == recovered_string) {
            std::cout << "✓ Conversion successful - strings match" << std::endl;
        } else {
            std::cout << "✗ Conversion mismatch!" << std::endl;
            std::cout << "Original:  " << test_frame << std::endl;
            std::cout << "Recovered: " << recovered_string << std::endl;
        }
    }
    
    return 0;
}
