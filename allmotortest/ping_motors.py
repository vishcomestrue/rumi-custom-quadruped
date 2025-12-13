#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Ping script to detect connected Dynamixel motors.
Usage: python ping_motors.py [--device DEVICE] [--protocol PROTOCOL] [--baudrate BAUDRATE]
"""

import os
import sys
import argparse

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Add parent directory to path to import dynamixel_sdk
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'DynamixelSDK', 'python', 'src'))

from dynamixel_sdk import *

def ping_motors(device_name, protocol_version, baudrate):
    """
    Ping all motors on the specified port and print their IDs.
    
    Args:
        device_name: Serial port device (e.g., '/dev/ttyUSB0')
        protocol_version: Dynamixel protocol version (1.0 or 2.0)
        baudrate: Communication baudrate
    """
    # Initialize PortHandler and PacketHandler
    portHandler = PortHandler(device_name)
    packetHandler = PacketHandler(protocol_version)

    # Open port
    if portHandler.openPort():
        print(f"Successfully opened port: {device_name}")
    else:
        print(f"Failed to open port: {device_name}")
        print("Press any key to terminate...")
        getch()
        return

    # Set port baudrate
    if portHandler.setBaudRate(baudrate):
        print(f"Successfully set baudrate to: {baudrate}")
    else:
        print(f"Failed to set baudrate to: {baudrate}")
        print("Press any key to terminate...")
        getch()
        portHandler.closePort()
        return

    print(f"\nScanning for motors using Protocol {protocol_version}...")
    print("-" * 50)

    # Broadcast ping to find all motors
    dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)
    
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Communication error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    
    # Print detected motors
    if dxl_data_list:
        print(f"\nDetected {len(dxl_data_list)} motor(s):\n")
        for dxl_id in sorted(dxl_data_list.keys()):
            model_number = dxl_data_list[dxl_id][0]
            firmware_version = dxl_data_list[dxl_id][1]
            print(f"  ID: {dxl_id:3d} | Model: {model_number:5d} | Firmware: {firmware_version:3d}")
    else:
        print("\nNo motors detected.")
    
    print("-" * 50)

    # Close port
    portHandler.closePort()
    print("\nPort closed.")

def main():
    parser = argparse.ArgumentParser(
        description='Ping and detect connected Dynamixel motors',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--device',
        type=str,
        default='/dev/ttyUSB0',
        help='Serial port device name'
    )
    
    parser.add_argument(
        '--protocol',
        type=float,
        default=2.0,
        choices=[1.0, 2.0],
        help='Dynamixel protocol version'
    )
    
    parser.add_argument(
        '--baudrate',
        type=int,
        default=2000000,
        help='Communication baudrate'
    )
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("Dynamixel Motor Ping Script")
    print("=" * 50)
    print(f"Device:   {args.device}")
    print(f"Protocol: {args.protocol}")
    print(f"Baudrate: {args.baudrate}")
    print("=" * 50)
    
    ping_motors(args.device, args.protocol, args.baudrate)

if __name__ == "__main__":
    main()
