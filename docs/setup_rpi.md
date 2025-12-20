# Raspberry Pi Setup Guide for Drone Pi

Follow these steps to set up your Raspberry Pi for drone control.

## 1. Raspberry Pi OS Setup
- Recommendation: **Raspberry Pi OS Lite (64-bit)**.
- Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to flash your SD card.
- In "Edit Settings", enable SSH and set your username/password.

## 2. Install Dependencies
Connect to your RPi via SSH and run:

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-evdev python3-serial
```

## 3. Permissions
You need permission to access the serial port (USB to FC) and the input device (Controller).

```bash
sudo usermod -a -G dialout,input $USER
```
**REBOOT after this step** for changes to take effect: `sudo reboot`

## 4. Pair PS4 Controller (Bluetooth)
1. Put the PS4 controller in pairing mode: Press and hold **PS Button** + **Share Button** until the light bar flashes.
2. Run `bluetoothctl` on the RPi:
   ```bash
   bluetoothctl
   [bluetooth]# agent on
   [bluetooth]# default-agent
   [bluetooth]# scan on
   # Look for "Wireless Controller" MAC address (e.g., A1:B2:C3:D4:E5:F6)
   [bluetooth]# pair YOUR_MAC_ADDRESS
   [bluetooth]# trust YOUR_MAC_ADDRESS
   [bluetooth]# connect YOUR_MAC_ADDRESS
   [bluetooth]# exit
   ```

## 5. Betaflight Configuration
Connect your SpeedyBee F405 Mini to your computer and open Betaflight Configurator.

1. **Ports Tab**:
   - Find **USB VCP**.
   - Ensure **MSP** is enabled (it should be by default).
2. **Receiver Tab**:
   - Receiver Mode: **Serial-based receiver**
   - Serial Receiver Provider: **MSP**
3. **Configuration Tab**:
   - Ensure you are in a flight mode that allows arming via MSP (usually default).

## 6. Run the Program
Upload the code to your RPi and run:

```bash
cd drone_pi
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
