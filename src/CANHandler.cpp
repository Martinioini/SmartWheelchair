#include "CANHandler.hpp"
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>

CANHandler::CANHandler(int busNum) : socketFd_(-1) {
    openSocket(busNum);
}

CANHandler::~CANHandler() {
    if (socketFd_ >= 0) {
        close(socketFd_);
    }
}

//Opens a socket to the CAN interface and uses a virtual C if regular CAN fails
bool CANHandler::openSocket(int canNum) {
    // Create CAN socket
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketFd_ < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    // Specify CAN interface
    struct ifreq ifr;
    std::string canInterface = "can" + std::to_string(canNum);
    strcpy(ifr.ifr_name, canInterface.c_str());
    
    if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0) {
        // Try virtual CAN if regular CAN fails, used for testing
        std::string vcanInterface = "vcan" + std::to_string(canNum);
        strcpy(ifr.ifr_name, vcanInterface.c_str());
        if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "Failed to open " << canInterface << " and " << vcanInterface << std::endl;
            close(socketFd_);
            return false;
        }
        std::cout << "Connected to " << vcanInterface << std::endl;
    } else {
        std::cout << "Connected to " << canInterface << std::endl;
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketFd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        close(socketFd_);
        return false;
    }

    return true;
} 

//Builds a CAN frame from a string representation (see linux/can.h for the format)
struct can_frame CANHandler::buildFrame(const std::string& canStr) {
    struct can_frame frame = {0};  // Initialize to zeros
    
    // Check for '#' delimiter
    size_t delimiter = canStr.find('#');
    if (delimiter == std::string::npos) {
        std::cerr << "buildFrame: missing #" << std::endl;
        return frame;
    }

    // Split into ID and data 
    std::string id_str = canStr.substr(0, delimiter);
    std::string data_str = canStr.substr(delimiter + 1);
    
    // Parse ID
    try {
        // Convert hex string ID to integer
        unsigned int id = std::stoul(id_str, nullptr, 16);
        
        // Set CAN ID based on length (3 for standard, 8 for extended)
        if (id_str.length() == 3) {
            frame.can_id = id;
        } else if (id_str.length() == 8) {
            frame.can_id = id | CAN_EFF_FLAG;  // Set extended frame flag
        } else {
            std::cerr << "Invalid ID length" << std::endl;
            return frame;
        }

        // Check for Remote Transmission Request
        if (data_str.find('R') != std::string::npos) {
            frame.can_id |= CAN_RTR_FLAG;
            frame.can_dlc = 0;
            return frame;
        }

        // Parse data
        frame.can_dlc = data_str.length() / 2;  // Two hex chars per byte
        for (size_t i = 0; i < frame.can_dlc && i < 8; i++) {
            frame.data[i] = std::stoul(data_str.substr(i*2, 2), nullptr, 16);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing CAN frame" << std::endl;
        frame = {0};
    }

    return frame;
}

//Dissects a CAN frame and returns a string representation
std::string CANHandler::dissectFrame(const can_frame& frame) {
    uint32_t can_id = frame.can_id & CAN_EFF_MASK;  // Extract actual ID
    bool is_extended = frame.can_id & CAN_EFF_FLAG;
    bool is_rtr = frame.can_id & CAN_RTR_FLAG;
    uint8_t dlc = frame.can_dlc;

    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    
    // Format ID with appropriate width (3 or 8 chars)
    if (is_extended) {
        ss << std::setw(8) << can_id;
    } else {
        ss << std::setw(3) << can_id;
    }
    
    ss << "#";
    
    // Always format data bytes
    for (int i = 0; i < dlc; i++) {
        ss << std::setw(2) << static_cast<int>(frame.data[i]);
    }
    
    // Append R at the end for RTR frames (to match Python behavior)
    if (is_rtr) {
        ss << "R";
    }

    return ss.str();
}

bool CANHandler::sendFrame(const std::string& frameStr) {
    
    can_frame frame = buildFrame(frameStr);
    
    // Send the frame
    ssize_t nbytes = write(socketFd_, &frame, sizeof(struct can_frame));
    
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "Error sending CAN frame " << frameStr << std::endl;
        return false;
    }
    
    return true;
}
