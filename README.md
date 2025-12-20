# Drone Pi

A robust Raspberry Pi based drone control system for the **SpeedyBee F405 Mini** (or any Betaflight FC) using a PS4 controller via MSP.

## Features
- **MSP over USB**: High-rate RC command transmission (50Hz).
- **Auto-Reconnect**: Automatically recovers if USB connection to FC is lost.
- **PS4 Controller Support**: Full mapping for DualShock 4.
- **Safety First**:
  - **Safe Start**: Prevents arming unless throttle is at zero.
  - **Auto-Disarm**: Automatically disarms on program exit or crash.
  - **Kill Switch**: Instant stop via the Options button.
- **Input Filtering**: Smooth control response via low-pass filtering.

## Setup
See [Setup Guide](docs/setup_rpi.md) for detailed instructions on setting up your Raspberry Pi and Betaflight.

## Quick Start
1. Connect RPi to FC via USB.
2. Pair PS4 Controller to RPi via Bluetooth.
3. Run the controller:
   ```bash
   python3 src/controller.py
   ```

## Controls
- **X**: ARM (if throttle is 0)
- **Triangle / Circle**: DISARM
- **Options**: STOP / KILL
- **Sticks**: Standard Mode 2 (Throttle/Yaw on Left, Pitch/Roll on Right)
