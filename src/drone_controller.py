from pymavlink import mavutil
import threading
import time
import math

class DroneController:
    """Complete MAVLink drone control for RPi"""
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.debug_callback = print
        
        # Status tracking
        self.status = {
            "armed": False,
            "mode": "UNKNOWN",
            "mode_num": 0,
            "battery_v": 0.0,
            "battery_a": 0.0,
            "battery_remaining": 100,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "alt": 0.0,
            "throttle": 0,
            "gps_fix": 0,
            "num_sats": 0,
            "groundspeed": 0.0,
            "airspeed": 0.0,
            "climb_rate": 0.0
        }
        
        # ArduCopter mode mapping
        self.mode_map = {
            0: "STABILIZE",
            1: "ACRO",
            2: "ALT_HOLD",
            3: "AUTO",
            4: "GUIDED",
            5: "LOITER",
            6: "RTL",
            7: "CIRCLE",
            9: "LAND",
            11: "DRIFT",
            13: "SPORT",
            14: "FLIP",
            15: "AUTOTUNE",
            16: "POSHOLD",
            17: "BRAKE",
            18: "THROW",
            19: "AVOID_ADSB",
            20: "GUIDED_NOGPS",
            21: "SMART_RTL",
            22: "FLOWHOLD",
            23: "FOLLOW",
            24: "ZIGZAG",
            25: "SYSTEMID",
            26: "AUTOROTATE",
            27: "AUTO_RTL"
        }
        
    def set_debug_callback(self, callback):
        self.debug_callback = callback
        
    def debug(self, msg):
        if self.debug_callback:
            self.debug_callback(msg)
    
    def connect(self, port, baudrate=115200):
        """Connect to drone"""
        try:
            self.debug(f"Connecting to {port} at {baudrate} baud...")
            self.master = mavutil.mavlink_connection(port, baud=baudrate)
            return True
        except Exception as e:
            self.debug(f"❌ Connection error: {e}")
            return False

    def wait_for_heartbeat(self, timeout=5):
        """Wait for heartbeat"""
        try:
            self.master.wait_heartbeat(timeout=timeout)
            self.connected = True
            self.debug(f"✓ Connected to system {self.master.target_system}")
            return True
        except Exception as e:
            self.debug(f"❌ Heartbeat timeout: {e}")
            return False

    def request_data_streams(self, rate=4):
        """Request all data streams"""
        if not self.master:
            return
        
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3
        ]
        
        for stream in streams:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream,
                rate,  # Hz
                1   # Start
            )

    def process_messages(self, stop_event):
        """Process incoming MAVLink messages in a thread"""
        while not stop_event.is_set() and self.master:
            try:
                msg = self.master.recv_match(blocking=False, timeout=0.01)
                if msg:
                    msg_type = msg.get_type()
                    
                    if msg_type == 'HEARTBEAT':
                        self.status["mode_num"] = msg.custom_mode
                        self.status["mode"] = self.mode_map.get(msg.custom_mode, f"UNKNOWN({msg.custom_mode})")
                        self.status["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    
                    elif msg_type == 'ATTITUDE':
                        self.status["roll"] = math.degrees(msg.roll)
                        self.status["pitch"] = math.degrees(msg.pitch)
                        self.status["yaw"] = math.degrees(msg.yaw)
                    
                    elif msg_type == 'SYS_STATUS':
                        self.status["battery_v"] = msg.voltage_battery / 1000.0
                        self.status["battery_a"] = msg.current_battery / 100.0
                        self.status["battery_remaining"] = msg.battery_remaining
                    
                    elif msg_type == 'VFR_HUD':
                        self.status["alt"] = msg.alt
                        self.status["throttle"] = msg.throttle
                        self.status["groundspeed"] = msg.groundspeed
                        self.status["airspeed"] = msg.airspeed
                        self.status["climb_rate"] = msg.climb
                        
                    elif msg_type == 'GPS_RAW_INT':
                        self.status["gps_fix"] = msg.fix_type
                        self.status["num_sats"] = msg.satellites_visible
                
                time.sleep(0.01)
            except Exception as e:
                if not stop_event.is_set():
                    self.debug(f"❌ Message error: {e}")
                break

    def set_mode(self, mode_name):
        """Set flight mode by name"""
        if not self.master or not self.connected:
            return False
        
        mode_id = next((num for num, name in self.mode_map.items() if name == mode_name), None)
        if mode_id is None:
            self.debug(f"❌ Unknown mode: {mode_name}")
            return False
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.debug(f"→ Requesting mode: {mode_name}")
        return True

    def arm(self):
        """Arm the drone"""
        if not self.master or not self.connected:
            return False
            
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.debug("→ ARM requested")
        return True

    def disarm(self):
        """Disarm the drone"""
        if not self.master or not self.connected:
            return False
            
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.debug("→ DISARM requested")
        return True
    
    def takeoff(self, altitude):
        """Takeoff to specified altitude (meters)"""
        if not self.master or not self.connected:
            return False
            
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        self.debug(f"→ TAKEOFF to {altitude}m requested")
        return True
    
    def send_rc_override(self, roll=1500, pitch=1500, throttle=1500, yaw=1500):
        """Send RC override (1000-2000, 1500 is center)"""
        if not self.master or not self.connected:
            return False
            
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            roll, pitch, throttle, yaw,
            0, 0, 0, 0  # channels 5-8
        )
        return True
    
    def release_rc_override(self):
        """Release RC control back to transmitter"""
        if not self.master or not self.connected:
            return False
            
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.debug("→ RC override released")
        return True

    def disconnect(self):
        """Disconnect from drone"""
        if self.master:
            try:
                self.release_rc_override()
                self.master.close()
            except:
                pass
        self.connected = False
        self.master = None
        self.debug("Disconnected")
