import os
import sys
import time
import threading
from drone_controller import DroneController
from controller_handler import ControllerHandler, PS4Buttons

# Configuration
MAVLINK_PORT = "/dev/ttyACM0"  # Default for USB connection to FC
MAVLINK_BAUD = 115200

def clear_screen():
    # Only move cursor to top-left. Don't clear screen to avoid flicker.
    sys.stdout.write("\033[H")
    sys.stdout.flush()

def main():
    # 1. Initialize Drone Controller
    drone = DroneController()
    
    # 2. Initialize Controller Handler
    handler = ControllerHandler()
    if not handler.find_device():
        print("❌ Fatal: PS4 Controller not found. Please connect via Bluetooth or USB.")
        sys.exit(1)
    
    # 3. Connect to Drone
    print(f"📡 Connecting to drone on {MAVLINK_PORT}...")
    if not drone.connect(MAVLINK_PORT, MAVLINK_BAUD):
        print("❌ Fatal: Could not connect to MAVLink. Check cable and port.")
        sys.exit(1)
        
    if not drone.wait_for_heartbeat():
        print("❌ Fatal: Heartbeat timeout. Is ArduPilot running?")
        sys.exit(1)
    
    drone.request_data_streams()
    
    # Start background threads
    stop_event = threading.Event()
    drone_thread = threading.Thread(target=drone.process_messages, args=(stop_event,), daemon=True)
    drone_thread.start()
    
    if not handler.start():
        print("❌ Fatal: Could not start controller handler.")
        sys.exit(1)

    print("🚀 System Ready! Switching to CMD Interface...")
    time.sleep(1)

    try:
        while True:
            # 4. Read Controller Input
            rc = handler.get_rc_values()
            
            # 5. Handle Buttons (Safety Logic)
            
            # KILL BUTTON: PS Button
            # Immediate emergency disarm and exit
            if handler.is_button_pressed(PS4Buttons.PS):
                print("\n🛑 KILL SWITCH TRIGGERED! EMERGENCY DISARM! 🛑")
                drone.disarm()
                break
            
            # ARM: Options Button
            # Safety check: Only arm if throttle is low
            if handler.is_button_pressed(PS4Buttons.OPTIONS):
                if rc["throttle"] < 1100:
                    drone.arm()
                else:
                    print("⚠️ Safety: Cannot ARM with throttle above minimum!")
            
            # DISARM: Share Button
            if handler.is_button_pressed(PS4Buttons.SHARE):
                drone.disarm()

            # 6. Send RC Overrides to Drone
            # We send this even if not armed (ArduPilot handles safety)
            drone.send_rc_override(
                roll=rc["roll"],
                pitch=rc["pitch"],
                throttle=rc["throttle"],
                yaw=rc["yaw"]
            )

            # 7. Update CMD Display
            s = drone.status
            clear_screen()
            
            # Use a list of lines to print all at once to minimize network packets over SSH
            lines = [
                "====================================================".ljust(60),
                "🚁 RASPBERRY PI DRONE CONTROL (Headless)".ljust(60),
                "====================================================".ljust(60),
                f" Status:  {'🔴 ARMED' if s['armed'] else '🟢 DISARMED'} | Mode: {s['mode']}".ljust(60),
                f" Battery: {s['battery_v']:.2f}V ({s['battery_remaining']}%) | Current: {s['battery_a']:.1f}A".ljust(60),
                f" GPS:     {s['gps_fix']} Fix | Satellites: {s['num_sats']}".ljust(60),
                f" Attitude: R:{s['roll']:>5.1f}° | P:{s['pitch']:>5.1f}° | Y:{s['yaw']:>5.1f}°".ljust(60),
                f" Alt:      {s['alt']:.1f}m | Speed: {s['groundspeed']:.1f}m/s".ljust(60),
                "----------------------------------------------------".ljust(60),
                f" 🎮 RC IN: T:{rc['throttle']:<4} | Y:{rc['yaw']:<4} | R:{rc['roll']:<4} | P:{rc['pitch']:<4}".ljust(60),
                "----------------------------------------------------".ljust(60),
                f" [Options] ARM | [Share] DISARM | [PS] KILL (STOP)".ljust(60),
                f" DEBUG: Last Button Code Received: {handler.last_button}".ljust(60),
                "====================================================".ljust(60)
            ]
            
            sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.flush()
            
            time.sleep(0.1) # Frequency reduced to 10Hz for smoother SSH

    except KeyboardInterrupt:
        print("\n👋 Exiting safely...")
    finally:
        # Cleanup
        stop_event.set()
        drone.release_rc_override()
        drone.disconnect()
        handler.stop()
        print("✅ Shutdown complete.")

if __name__ == "__main__":
    main()
