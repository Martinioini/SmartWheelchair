# CAN Interface Project

## Overview
This project provides an interface for CAN bus communication and rnet electronics wheelchair, allowing to control a rnet wheelchair with an xbox like controller.

## Features
- Can interface 
- Real-time CAN frames processing
- Support for standard CAN protocols
- Easy-to-use interface for sending and receiving CAN messages in string format

## Hardware Requirements
- Raspberry Pi (3B+ or 4 recommended)
- USB-to-CAN adapter (such as CANable, PCAN-USB, or similar CAN interface)
- CAN devices for communication
- Xbox like controller
## Software Requirements
- Ubuntu 20.04
- ROS Noetic
- can-utils package
- roscpp

## Installation

### 1. ROS Noetic
Ensure ROS Noetic is installed on your system. If not, follow the [official ROS installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

### 2. CAN Utils
Install the can-utils package:

```bash
sudo apt-get update
sudo apt-get install can-utils
```

### 3. Clone this repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/Martinioini/SmartWheelchair.git
cd ~/catkin_ws
catkin_make
```

## CAN Interface Setup

### 1. Configure CAN interface
Set up the CAN interface with a bitrate of 125000:

```bash
sudo ip link set can0 type can bitrate 125000
sudo ip link set up can0
```

### 2. Test CAN communication
You can test if your CAN interface is working properly using can-utils:

```bash
# Listen for CAN messages
candump can0

# Send a test message
cansend can0 123#DEADBEEF
```

## Usage

1. Source the setup script:
```bash
source ~/catkin_ws/devel/setup.bash
```

2. Launch the CAN interface node:
```bash
roslaunch wheelchair_control real_wheelchair.launch 
```

## Troubleshooting
- If the CAN interface doesn't come up, check your hardware connections
- Verify that your USB-to-CAN adapter is properly recognized with `dmesg | grep can`
- Ensure the correct bitrate is set for your CAN network

## License
[Your license information here]

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request.
