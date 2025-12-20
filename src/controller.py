import signal
import sys
import logging
from evdev import InputDevice, ecodes, list_devices
from msp_sender import CRSFSender


# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class DroneController:
    # Controller Configuration
    CENTER = 128
    MAX_RANGE = 127
    DEADZONE = 5
    ALPHA = 0.25  # Low-pass filter strength

    def __init__(self, device_name="Wireless Controller"):
        self.device_name = device_name
        self.dev = self._find_device()
        
        # State
        self.armed = False
        
        # Raw values
        self.raw = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
        
        # Filtered values
        self.filtered = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}

        # CRSF Sender
        self.sender = CRSFSender()

        # Setup exit handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _find_device(self):
        """Discovers the controller device."""
        for path in list_devices():
            try:
                dev = InputDevice(path)
                if dev.name == self.device_name:
                    logger.info(f"Found controller: {dev.name} at {path}")
                    return dev
            except Exception as e:
                logger.debug(f"Error checking device at {path}: {e}")
        
        raise RuntimeError(f"Controller '{self.device_name}' not found. Ensure it is connected.")

    def _signal_handler(self, sig, frame):
        """Gracefully handle exit signals."""
        print("\n")
        logger.info("Exit signal received. Disarming and shutting down...")
        self.armed = False
        if hasattr(self, 'sender'):
            self.sender.update_values(self.filtered, self.armed)
            self.sender.close()
        sys.exit(0)

    def _normalize(self, value, invert=False):
        """Normalizes stick input to -1.0 to 1.0 range with deadzone."""
        delta = value - self.CENTER
        if abs(delta) < self.DEADZONE:
            return 0.0
        n = max(-1.0, min(1.0, delta / self.MAX_RANGE))
        return -n if invert else n

    def _apply_lpf(self, new, old):
        """First-order low-pass filter."""
        return self.ALPHA * new + (1 - self.ALPHA) * old

    def _reset_inputs(self):
        """Resets all input values to zero."""
        for key in self.raw:
            self.raw[key] = 0.0
            self.filtered[key] = 0.0

    def run(self):
        """Main loop to process controller events."""
        print("-" * 50)
        print("DRONE PI SYSTEM READY (CRSF MODE)")
        print("Controls (Standard Mode 2):")
        print("  Left Stick  - Throttle (Up/Down), Yaw (Left/Right)")
        print("  Right Stick - Pitch (Up/Down), Roll (Left/Right)")
        print("-" * 50)
        print("Safety:")
        print("  [X]        - ARM (Must HOLD Left Stick DOWN)")
        print("  [Triangle] - DISARM")
        print("  [Circle]   - DISARM")
        print("  [Options]  - KILL/EXIT")
        print("-" * 50)
        
        try:
            for event in self.dev.read_loop():
                self._process_event(event)
        except Exception as e:
            logger.error(f"Error in event loop: {e}")
        finally:
            self._reset_inputs()

    def _process_event(self, event):
        """Processes a single evdev event."""
        
        # Button Events (Press only)
        if event.type == ecodes.EV_KEY and event.value == 1:
            if event.code == ecodes.BTN_SOUTH:  # X Button
                if not self.armed:
                    # Safe Start Check: Throttle must be below 0.05
                    # On PS4 sticks, holding down gives 0.0, center gives 0.5
                    if self.filtered["throttle"] < 0.05:
                        self.armed = True
                        logger.info("SYSTEM ARMED")
                    else:
                        logger.warning(f"CANNOT ARM: Throttle is at {self.filtered['throttle']:.2f}. HOLD LEFT STICK DOWN!")
            
            elif event.code in [ecodes.BTN_NORTH, ecodes.BTN_EAST]:  # Triangle or Circle
                if self.armed:
                    self.armed = False
                    self._reset_inputs()
                    logger.info("SYSTEM DISARMED")
            
            elif event.code == ecodes.BTN_START:  # Options Button
                logger.warning("!!! KILL SIGNAL RECEIVED !!!")
                self.armed = False
                self.sender.update_values(self.filtered, self.armed)
                self.sender.close()
                sys.exit(0)

        # Stick Events
        elif event.type == ecodes.EV_ABS:
            # LEFT STICK (Throttle & Yaw)
            if event.code == ecodes.ABS_X:
                self.raw["yaw"] = self._normalize(event.value)
            elif event.code == ecodes.ABS_Y:
                # Up is low value, Down is high value. Inverting makes Up=1.0, Down=-1.0.
                self.raw["throttle"] = (self._normalize(event.value, invert=True) + 1) / 2
            
            # RIGHT STICK (Roll & Pitch)
            elif event.code == ecodes.ABS_RX:
                self.raw["roll"] = self._normalize(event.value)
            elif event.code == ecodes.ABS_RY:
                self.raw["pitch"] = self._normalize(event.value, invert=True)

            # Apply Filtering to all axes
            for axis in self.filtered:
                self.filtered[axis] = self._apply_lpf(self.raw[axis], self.filtered[axis])

            # Update CRSF Sender
            self.sender.update_values(self.filtered, self.armed)

            self._print_status()

    def _print_status(self):
        """Prints the current state formatted for status line."""
        status = "[\033[91mARMED\033[0m]" if self.armed else "[\033[92mDISARMED\033[0m]"
        f = self.filtered
        # Clear line and print status
        sys.stdout.write(f"\r{status} | "
                         f"R: {f['roll']:+.2f} | "
                         f"P: {f['pitch']:+.2f} | "
                         f"Y: {f['yaw']:+.2f} | "
                         f"T: {f['throttle']:.2f}    ")
        sys.stdout.flush()

if __name__ == "__main__":
    try:
        controller = DroneController()
        controller.run()
    except Exception as e:
        logger.critical(f"Fatal error: {e}")
        sys.exit(1)
