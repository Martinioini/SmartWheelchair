#pragma once
#include <string>
#include <cstdint>

class CANHandler {
public:
    CANHandler(int busNum);
    ~CANHandler();

    bool sendFrame(const std::string& canStr);
    std::string dissectFrame(const uint8_t* frame);
    bool waitForFrame(const std::string& filterStr);
    bool openSocket(int busNum);

private:
    int socketFd_;
    std::string buildFrame(const std::string& canStr);
}; 
