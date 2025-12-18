from evdev import InputDevice, ecodes, list_devices

# Controller finden
device_path = None
for path in list_devices():
    dev = InputDevice(path)
    if dev.name == "Wireless Controller":
        device_path = path
        break

if device_path is None:
    raise RuntimeError("Controller nicht gefunden")

dev = InputDevice(device_path)

CENTER = 128
MAX_RANGE = 127
DEADZONE = 5
ALPHA = 0.25  # Filterstärke

# Rohwerte
raw_roll = raw_pitch = raw_yaw = raw_throttle = 0.0

# Gefilterte Werte
roll = pitch = yaw = throttle = 0.0

armed = False

def norm(v, invert=False):
    delta = v - CENTER
    if abs(delta) < DEADZONE:
        return 0.0
    n = max(-1.0, min(1.0, delta / MAX_RANGE))
    return -n if invert else n

def lpf(new, old):
    return ALPHA * new + (1 - ALPHA) * old

print("System DISARMED")

for event in dev.read_loop():

    # Buttons
    if event.type == ecodes.EV_KEY and event.value == 1:
        if event.code == ecodes.BTN_SOUTH:  # X
            armed = True
            print("\nARMED")
        elif event.code == ecodes.BTN_EAST:  # O
            armed = False
            raw_roll = raw_pitch = raw_yaw = raw_throttle = 0.0
            roll = pitch = yaw = throttle = 0.0
            print("\nDISARMED")
        elif event.code == ecodes.BTN_START:  # OPTIONS
            print("\n!!! KILL !!!")
            break

    # Sticks
    if event.type == ecodes.EV_ABS:
        if not armed:
            raw_roll = raw_pitch = raw_yaw = raw_throttle = 0.0
        else:
            if event.code == ecodes.ABS_X:
                raw_roll = norm(event.value)
            elif event.code == ecodes.ABS_Y:
                raw_pitch = norm(event.value, invert=True)
            elif event.code == ecodes.ABS_RX:
                raw_yaw = norm(event.value)
            elif event.code == ecodes.ABS_RY:
                raw_throttle = (norm(event.value, invert=True) + 1) / 2

        # Filter anwenden
        roll = lpf(raw_roll, roll)
        pitch = lpf(raw_pitch, pitch)
        yaw = lpf(raw_yaw, yaw)
        throttle = lpf(raw_throttle, throttle)

        status = "ARMED" if armed else "DISARMED"
        print(
            f"{status} "
            f"R:{roll:+.2f} "
            f"P:{pitch:+.2f} "
            f"Y:{yaw:+.2f} "
            f"T:{throttle:.2f}",
            end="\r"
        )
