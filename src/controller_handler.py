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
        """Find the controller device by name"""
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if self.device_name in device.name:
                self.device = device
                return True
        return False

    def start(self):
        """Start the background reading thread"""
        if not self.device:
            if not self.find_device():
                print(f"❌ Could not find {self.device_name}")
                return False
        
        print(f"🎮 Connected to {self.device.name} at {self.device.path}")
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return True

    def _run(self):
        """Background thread to read events"""
        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                
                # Handle axes (Sticks)
                if event.type == ecodes.EV_ABS:
                    if event.code in self.axis_map:
                        key = self.axis_map[event.code]
                        # evdev values are usually 0-255 or 0-65535. 
                        # We need to map them to 1000-2000.
                        # ArduPilot expects: Roll/Pitch/Yaw center at 1500, Throttle bottom at 1000.
                        
                        # Note: PS4 Controller on RPi usually gives 0-255 for axes.
                        # But it depends on the driver. We'll use the device's info.
                        abs_info = self.device.capabilities().get(ecodes.EV_ABS, [])
                        info = next((i[1] for i in abs_info if i[0] == event.code), None)
                        
                        if info:
                            val_min = info.min
                            val_max = info.max
                            
                            # Map to 1000-2000
                            # normalized = (val - min) / (max - min)
                            # rc = 1000 + normalized * 1000
                            normalized = (event.value - val_min) / (val_max - val_min)
                            
                            if key == "throttle":
                                # Throttle: Up is 1000, Down is 2000 (usually) -> We want Up 2000, Down 1000
                                self.rc_values[key] = int(2000 - (normalized * 1000))
                            elif key == "pitch":
                                # Pitch: Up is 1000, Down is 2000 -> We want Up 1000 (Forward), Down 2000 (Backward)
                                # Wait, standard ArduPilot is Pitch Up (forward) is 1000? 
                                # Actually, it's stick forward = low value (1000), stick back = high value (2000)
                                self.rc_values[key] = int(1000 + (normalized * 1000))
                            else:
                                self.rc_values[key] = int(1000 + (normalized * 1000))
                                
                # Handle buttons
                elif event.type == ecodes.EV_KEY:
                    self.buttons[event.code] = event.value
                    
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
