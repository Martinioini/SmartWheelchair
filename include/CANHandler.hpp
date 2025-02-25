#pragma once
#include <string>
#include <cstdint>
#include <iostream>
#include <linux/can.h>

class CANHandler {
public:
    CANHandler(int busNum);
    ~CANHandler();

    bool sendFrame(const std::string& canStr);
    std::string dissectFrame(const uint8_t* frame);
    bool waitForFrame(const std::string& filterStr);
    bool openSocket(int busNum);
    struct can_frame buildFrame(const std::string& canStr);

private:
    int socketFd_;
}; 
