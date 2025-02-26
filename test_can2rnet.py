#!/usr/bin/env python3

import can2RNET
import binascii
import struct

def main():
    # Test buildFrame with the same sample CAN frame string
    test_frame = "181c0100#0260000000000000"
    print(f"Testing CAN string: {test_frame}")
    
    # Build the frame
    frame = can2RNET.build_frame(test_frame)
    
    # Unpack the frame to display its components
    can_frame_fmt = "<IB3x8s"
    can_id, can_dlc, data = struct.unpack(can_frame_fmt, frame)
    
    # Print the results in similar format to the C++ version
    print(f"Frame ID: 0x{can_id:08x}")
    print(f"DLC: {can_dlc}")
    
    # Print data bytes
    print("Data:", end=" ")
    for byte in data[:can_dlc]:
        print(f"{byte:02x}", end=" ")
    print()

if __name__ == "__main__":
    main() 
