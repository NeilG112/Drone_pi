import signal
import sys
import logging
from evdev import InputDevice, ecodes, list_devices
from msp_sender import MSPSender


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

        # MSP Sender
        self.msp = MSPSender()

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
        if hasattr(self, 'msp'):
            self.msp.update_values(self.filtered, self.armed)
            self.msp.close()
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
        logger.info("System Ready. Status: DISARMED (Press X to ARM, O to DISARM, Options to KILL)")
        
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
                    self.armed = True
                    logger.info("SYSTEM ARMED")
            elif event.code == ecodes.BTN_EAST:   # O Button
                if self.armed:
                    self.armed = False
                    self._reset_inputs()
                    logger.info("SYSTEM DISARMED")
            elif event.code == ecodes.BTN_START:  # Options Button
                logger.warning("!!! KILL SIGNAL RECEIVED !!!")
                self.armed = False
                sys.exit(0)

        # Stick Events
        elif event.type == ecodes.EV_ABS:
            if not self.armed:
                self._reset_inputs()
            else:
                if event.code == ecodes.ABS_X:
                    self.raw["roll"] = self._normalize(event.value)
                elif event.code == ecodes.ABS_Y:
                    self.raw["pitch"] = self._normalize(event.value, invert=True)
                elif event.code == ecodes.ABS_RX:
                    self.raw["yaw"] = self._normalize(event.value)
                elif event.code == ecodes.ABS_RY:
                    # Map throttle to 0.0 to 1.0
                    self.raw["throttle"] = (self._normalize(event.value, invert=True) + 1) / 2

            # Apply Filtering to all axes
            for axis in self.filtered:
                self.filtered[axis] = self._apply_lpf(self.raw[axis], self.filtered[axis])

            # Update MSP Sender
            self.msp.update_values(self.filtered, self.armed)

            self._print_status()

    def _print_status(self):
        """Prints the current state formatted for status line."""
        status = "ARMED" if self.armed else "DISARMED"
        f = self.filtered
        print(
            f"\r{status} | "
            f"Roll: {f['roll']:+.2f} | "
            f"Pitch: {f['pitch']:+.2f} | "
            f"Yaw: {f['yaw']:+.2f} | "
            f"Thrott: {f['throttle']:.2f}",
            end="",
            flush=True
        )

if __name__ == "__main__":
    try:
        controller = DroneController()
        controller.run()
    except Exception as e:
        logger.critical(f"Fatal error: {e}")
        sys.exit(1)
