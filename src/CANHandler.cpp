#include "CANHandler.hpp"
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <iostream>

CANHandler::CANHandler(int busNum) : socketFd_(-1) {
    openSocket(busNum);
}

CANHandler::~CANHandler() {
    if (socketFd_ >= 0) {
        close(socketFd_);
    }
}

//Opens a socket to the CAN interface and uses VSocket if regular CAN fails
bool CANHandler::openSocket(int busNum) {
    // Create CAN socket
    socketFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketFd_ < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    // Specify CAN interface
    struct ifreq ifr;
    std::string canInterface = "can" + std::to_string(busNum);
    strcpy(ifr.ifr_name, canInterface.c_str());
    
    if (ioctl(socketFd_, SIOCGIFINDEX, &ifr) < 0) {
        // Try virtual CAN if regular CAN fails
        std::string vcanInterface = "vcan" + std::to_string(busNum);
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

std::string buildFrame(const std::string& canStr)
{
    if(std::find(canStr.begin(), canStr.end(), '#') == canStr.end())
    {
        std::cerr << "buildFrame: missing #" << std::endl;
        return "Err!";
    }
    
    std::vector<std::string> cansplit = split(canStr, '#');
    int lcanid = cansplit[0].length();
    bool RTR = cansplit[0].find("R") != std::string::npos;

    if(lcanid == 3)
    {
        canid = struct.pack('I', int(cansplit[0], 16) + 0x40000000 * RTR);
    }

    else if(lcanid == 8)
    {
        canid = struct.pack('I', int(cansplit[0], 16) + 0x80000000 + 0x40000000 * RTR);
    }

    else
    {
        std::cerr << "build_frame: cansend frame id format error: " << canStr << std::endl;
        return "Err!";
    }
    int can_dlc = 0;
    int len_datstr = cansplit[1].length();
    
}

