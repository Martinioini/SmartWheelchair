#ifndef CANHANDLER_HPP
#define CANHANDLER_HPP

#include <string>
#include <cstdint>
#include <iostream>
#include <linux/can.h>
#include <ros/ros.h>

class CANHandler {
public:
    // Default constructor - uses bus 0
    CANHandler();
    
    // Constructor with specified bus number
    CANHandler(int busNum);
    
    // Destructor
    ~CANHandler();

    void restartInterface();
    void flushCANBuffer();
    bool openSocket(int canNum);
    struct can_frame buildFrame(const std::string& canStr);
    bool sendFrame(const std::string& frameStr);
    std::string dissectFrame(const can_frame& frame);

private:
    int socketFd_;
};

#endif // CANHANDLER_HPP 
