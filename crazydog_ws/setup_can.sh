#!/bin/bash

# Load CAN kernel modules
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-dev
sudo modprobe can-bcm

# Configure and bring up the CAN interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

echo "CAN interface can0 set up with bitrate 1000000."
