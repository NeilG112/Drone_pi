import evdev
from evdev import ecodes
import threading
import time

class ControllerHandler:
    """Handles PS4 Controller input via evdev for RPi"""
    
    def __init__(self, device_name="Wireless Controller"):
        self.device = None
        self.device_name = device_name
        self.rc_values = {"roll": 1500, "pitch": 1500, "throttle": 1000, "yaw": 1500}
        self.buttons = {}
        self.last_button = None
        self.running = False
        self.thread = None
        
        # PS4 stick mapping (evdev)
        # ABS_X: Left X (Yaw), ABS_Y: Left Y (Throttle)
        # ABS_RX: Right X (Roll), ABS_RY: Right Y (Pitch)
        self.axis_map = {
            ecodes.ABS_X: "yaw",
            ecodes.ABS_Y: "throttle",
            ecodes.ABS_RX: "roll",
            ecodes.ABS_RY: "pitch"
        }
        
    def find_device(self):
        """Find the controller device by name with improved robustness"""
        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
                if self.device_name in device.name:
                    # Filter out non-main nodes
                    if "Touchpad" in device.name or "Motion Sensors" in device.name:
                        continue
                        
                    caps = device.capabilities()
                    if ecodes.EV_KEY in caps and ecodes.EV_ABS in caps:
                        self.device = device
                        print(f"✅ Found {device.name} (Main Node) at {device.path}")
                        return True
            except Exception as e:
                # Some devices might not allow access, skip them
                continue
        
        # Fallback: any device with the name
        for path in evdev.list_devices():
            try:
                device = evdev.InputDevice(path)
                if self.device_name in device.name:
                    self.device = device
                    print(f"⚠️ Found {device.name} (Partial Node) at {device.path}")
                    return True
            except:
                continue
                
        return False

    def start(self):
        """Start the background reading thread"""
        if not self.device:
            if not self.find_device():
                print(f"❌ Could not find {self.device_name}")
                return False
        
        # Mode 2 Layout:
        # ABS_RZ: Throttle (Trigger R2, 0 to 255)
        # ABS_X: Yaw (Left Stick X)
        # ABS_RX: Roll (Right Stick X)
        # ABS_RY: Pitch (Right Stick Y, Inverted)
        # ABS_HAT0X/Y: D-pad
        self.axis_map = {
            ecodes.ABS_X: "yaw",
            ecodes.ABS_RX: "roll",
            ecodes.ABS_RY: "pitch",
            ecodes.ABS_RZ: "throttle",
            ecodes.ABS_HAT0X: "dpad_x",
            ecodes.ABS_HAT0Y: "dpad_y"
        }
        
        # D-pad state
        self.dpad = {"x": 0, "y": 0}
        
        # Load capabilities
        self.abs_info = {}
        caps = self.device.capabilities().get(ecodes.EV_ABS, [])
        for code, info in caps:
            if code in self.axis_map:
                self.abs_info[code] = info

        # Filter parameters
        self.ALPHA = 0.25
        
        print(f"🎮 Connected to {self.device.name} at {self.device.path}")
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return True

    def _normalize(self, value, info, key, invert=False):
        """Normalizes stick/trigger/dpad input"""
        if key == "throttle":
            # Trigger: 0 to 255 -> 0.0 to 1.0 (no center, no deadzone needed usually)
            n = (value - info.min) / (info.max - info.min)
            return n if not invert else (1.0 - n)
        elif "dpad" in key:
            # D-pad: usually -1, 0, 1. Just return directly.
            return value
        else:
            # Stick: centered mapping with 5% deadzone
            center = (info.max + info.min) // 2
            max_range = (info.max - info.min) // 2
            
            delta = value - center
            if abs(delta) < (max_range * 0.05): # 5% deadzone
                return 0.0
                
            n = max(-1.0, min(1.0, delta / max_range))
            return -n if invert else n

    def _apply_lpf(self, new, old):
        """First-order low-pass filter"""
        return self.ALPHA * new + (1 - self.ALPHA) * old

    def _run(self):
        """Background thread to read events with LPF"""
        # Internal state for LPF
        # Sticks are -1.0 to 1.0, Throttle is 0.0 to 1.0
        filtered_state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
        raw_state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}

        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                
                # Handle axes (Sticks, Triggers, D-pad)
                if event.type == ecodes.EV_ABS:
                    if event.code in self.axis_map:
                        key = self.axis_map[event.code]
                        info = self.abs_info.get(event.code)
                        
                        if info:
                            # Normalize
                            invert = False
                            if key == "pitch": # Push stick forward = pitch down
                                invert = True
                            
                            val = self._normalize(event.value, info, key, invert=invert)
                            
                            if "dpad" in key:
                                if key == "dpad_x":
                                    self.dpad["x"] = val
                                else:
                                    self.dpad["y"] = val
                            else:
                                raw_state[key] = val
                                
                                # Update filtered state
                                for axis in filtered_state:
                                    filtered_state[axis] = self._apply_lpf(raw_state[axis], filtered_state[axis])
                                
                                # Convert to 1000-2000 for RC
                                if key == "throttle":
                                    # LIMITED: 0.0 to 1.0 -> 1000 to 1250
                                    # This prevents voltage drops on current power system
                                    self.rc_values["throttle"] = int(1000 + (filtered_state["throttle"] * 250))
                                else:
                                    # -1.0 to 1.0 -> 1000 to 2000 (center 1500)
                                    self.rc_values[key] = int(1500 + (filtered_state[key] * 500))
                                
                # Handle buttons
                elif event.type == ecodes.EV_KEY:
                    self.buttons[event.code] = event.value
                    if event.value == 1:
                        self.last_button = event.code
                    
        except Exception as e:
            print(f"❌ Controller error: {e}")
            self.running = False

    def get_rc_values(self):
        """Return clamped RC values"""
        def clamp(n):
            return max(1000, min(2000, n))
            
        return {k: clamp(v) for k, v in self.rc_values.items()}

    def is_button_pressed(self, btn_code):
        """Check if a button is currently pressed (1)"""
        return self.buttons.get(btn_code, 0) == 1

    def stop(self):
        """Stop the background thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)

# PS4 Button Codes (may vary, these are typical for evdev on RPi)
# BTN_SOUTH: X, BTN_EAST: Circle, BTN_NORTH: Triangle, BTN_WEST: Square
# BTN_SELECT: Share, BTN_START: Options, BTN_MODE: PS Button
class PS4Buttons:
    X = ecodes.BTN_SOUTH
    CIRCLE = ecodes.BTN_EAST
    TRIANGLE = ecodes.BTN_NORTH
    SQUARE = ecodes.BTN_WEST
    SHARE = ecodes.BTN_SELECT
    OPTIONS = ecodes.BTN_START
    PS = ecodes.BTN_MODE
    L1 = ecodes.BTN_TL
    R1 = ecodes.BTN_TR
    L2 = ecodes.BTN_TL2
    R2 = ecodes.BTN_TR2
    L3 = ecodes.BTN_THUMBL
    R3 = ecodes.BTN_THUMBR
