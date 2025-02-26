#pragma once
#include <string>
#include <cstdint>
#include <iostream>
#include <linux/can.h>

class CANHandler {
public:
    CANHandler(int busNum);
    ~CANHandler();

    bool openSocket(int canNum);
    struct can_frame buildFrame(const std::string& canStr);
    bool sendFrame(const std::string& frameStr);
    std::string dissectFrame(const can_frame& frame);

private:
    int socketFd_;
}; 
