import serial
import struct
import threading
import time
import logging

logger = logging.getLogger(__name__)

class MSPSender:
    """
    Handles sending MSP_SET_RAW_RC frames to Betaflight at a fixed rate.
    MSP v1: $ M < <length> <id> <payload> <checksum>
    """
    MSP_SET_RAW_RC = 200
    
    def __init__(self, port='/dev/ttyACM0', baud=115200, rate_hz=50):
        self.port = port
        self.baud = baud
        self.interval = 1.0 / rate_hz
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # State: [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
        # Channels: CH1-CH4 (1000-2000), CH5 (AUX1)
        self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.armed = False
        
        self.running = False
        self.ser = None
        self._start_serial()
        
        # Start daemon thread
        self.thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.running = True
        self.thread.start()
        logger.info(f"MSP Sender started on {port} at {rate_hz}Hz")

    def _start_serial(self):
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            logger.info(f"Connected to serial port {self.port}")
            return True
        except Exception as e:
            # Only log error occasionally to avoid spamming
            if not hasattr(self, '_last_conn_error') or time.time() - self._last_conn_error > 5:
                logger.error(f"Failed to open serial port {self.port}: {e}")
                self._last_conn_error = time.time()
            self.ser = None
            return False

    def update_values(self, filtered_vals, armed):
        """
        Updates internal RC values from normalized controller values.
        - Roll/Pitch/Yaw: -1.0 to 1.0 -> 1000-2000
        - Throttle: 0.0 to 1.0 -> 1000-2000
        """
        with self.data_lock:
            self.armed = armed
            
            if not self.armed:
                # Safety: Force throttle low and AUX1 low
                self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
            else:
                # Map control values
                self.rc_channels[0] = int(1500 + (filtered_vals["roll"] * 500))
                self.rc_channels[1] = int(1500 + (filtered_vals["pitch"] * 500))
                self.rc_channels[2] = int(1000 + (filtered_vals["throttle"] * 1000))
                self.rc_channels[3] = int(1500 + (filtered_vals["yaw"] * 500))
                self.rc_channels[4] = 2000 # AUX1 (ARMED)

            # Clamp values to 1000-2000
            for i in range(len(self.rc_channels)):
                self.rc_channels[i] = max(1000, min(2000, self.rc_channels[i]))

    def _create_packet(self, data_id, payload):
        """Constructs a MSP v1 packet."""
        length = len(payload)
        checksum = length ^ data_id
        for byte in payload:
            checksum ^= byte
        
        header = b'$M<'
        return header + struct.pack('<BB', length, data_id) + payload + struct.pack('<B', checksum)

    def _sender_loop(self):
        """Fixed-rate transmission loop."""
        while self.running:
            start_time = time.time()
            
            if self._start_serial():
                with self.data_lock:
                    # MSP_SET_RAW_RC payload is 8 units of uint16
                    payload = struct.pack('<8H', *self.rc_channels)
                
                packet = self._create_packet(self.MSP_SET_RAW_RC, payload)
                try:
                    self.ser.write(packet)
                    self.ser.flush()
                except Exception as e:
                    logger.error(f"Serial write error: {e}")
                    self.ser.close()
                    self.ser = None
            
            # Sleep to maintain frequency
            elapsed = time.time() - start_time
            sleep_time = self.interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def close(self):
        """Clean shutdown: disarm first, then stop thread."""
        logger.info("Closing MSP Sender...")
        # 1. Force disarm state in RC values
        with self.data_lock:
            self.armed = False
            self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        
        # 2. Give the loop a moment to send the disarm packet
        time.sleep(self.interval * 2)
        
        # 3. Stop thread and serial
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
            
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info("MSP Sender closed.")
