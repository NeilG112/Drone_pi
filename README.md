# Drone Pi

A robust Raspberry Pi based drone control system for the **SpeedyBee F405 Mini** (or any Betaflight FC) using a PS4 controller via **CRSF protocol**.

## Features
- **CRSF over USB**: High-speed, industry-standard RC transmission.
- **Auto-Reconnect**: Automatically recovers if USB connection is lost.
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

### Controls (Mode 2)
- **X Button**: ARM (Only if **Left Stick** is held all the way **DOWN**)
- **Triangle / Circle**: DISARM
- **Options**: STOP PROGRAM / EMERGENCY DISARM
- **Left Stick**: Throttle (Up/Down) & Yaw (Left/Right)
- **Right Stick**: Pitch (Up/Down) & Roll (Left/Right)

> [!NOTE]
> Because PS4 sticks spring back to the center, releasing the stick will result in **50% throttle (0.50)**. You must keep your thumb on the stick for flight control.
