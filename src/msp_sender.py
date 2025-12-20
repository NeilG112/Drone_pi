import serial
import struct
import threading
import time
import logging

logger = logging.getLogger(__name__)

class CRSFSender:
    """
    Handles sending CRSF RC_CHANNELS_PACKED frames to Betaflight at 50Hz.
    CRSF v2: [Sync] [Len] [Type] [Payload] [CRC8]
    """
    CRSF_SYNC = 0xC8
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
    
    def __init__(self, port='/dev/ttyACM0', baud=115200, rate_hz=50):
        self.port = port
        self.baud = baud
        self.interval = 1.0 / rate_hz
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # 16 channels, 11-bit values (range 191 to 1792)
        # 992 is center (1500us)
        self.rc_channels = [992] * 16
        self.rc_channels[3] = 191 # Throttle low
        self.armed = False
        
        self.running = False
        self.ser = None
        
        # Start daemon thread
        self.thread = threading.Thread(target=self._sender_loop, daemon=True)
        self.running = True
        self.thread.start()
        logger.info(f"CRSF Sender started on {port} at {rate_hz}Hz")

    def _start_serial(self):
        if self.ser and self.ser.is_open:
            return True
        ports_to_try = [self.port, '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for p in ports_to_try:
            try:
                self.ser = serial.Serial(p, self.baud, timeout=0.1)
                self.ser.dtr = True
                self.ser.rts = True
                logger.info(f"Connected to Flight Controller via CRSF on {p}")
                self.port = p
                return True
            except Exception: continue
        return False

    def _crc8(self, data):
        """CRSF CRC8 implementation."""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def update_values(self, filtered_vals, armed):
        """Maps -1.0..1.0 to 191..1792 (CRSF 11-bit range)."""
        with self.data_lock:
            self.armed = armed
            if not self.armed:
                # Safety: R,P,Y center (992), T low (191), AUX1 low (191)
                self.rc_channels = [992, 992, 992, 191] + [191] * 12
            else:
                self.rc_channels[0] = int(992 + (filtered_vals["roll"] * 800))
                self.rc_channels[1] = int(992 + (filtered_vals["pitch"] * 800))
                self.rc_channels[2] = int(992 + (filtered_vals["yaw"] * 800))
                self.rc_channels[3] = int(191 + (filtered_vals["throttle"] * 1600))
                self.rc_channels[4] = 1792 # AUX1 (ARMED)
                # Keep others at 191 (OFF)

            # Clamp for safety
            for i in range(16):
                self.rc_channels[i] = max(191, min(1792, self.rc_channels[i]))

    def _create_frame(self):
        """Packs 16 x 11-bit channels into 22 bytes."""
        payload = bytearray(22)
        bits = 0
        current_byte = 0
        for ch_val in self.rc_channels:
            for i in range(11):
                if ch_val & (1 << i):
                    payload[current_byte] |= (1 << bits)
                bits += 1
                if bits == 8:
                    bits = 0
                    current_byte += 1
        
        # Full frame: [Sync] [Len] [Type] [Payload] [CRC]
        frame_head = struct.pack('BBB', self.CRSF_SYNC, len(payload) + 2, self.CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        frame = frame_head + payload
        # CRC is calculated on Type + Payload
        crc = self._crc8(frame[2:])
        return frame + struct.pack('B', crc)

    def _sender_loop(self):
        last_heartbeat = time.time()
        packet_cnt = 0
        while self.running:
            start_time = time.time()
            if self._start_serial():
                with self.data_lock:
                    frame = self._create_frame()
                try:
                    self.ser.write(frame)
                    packet_cnt += 1
                except Exception as e:
                    logger.error(f"Serial error: {e}")
                    self.ser = None

            if time.time() - last_heartbeat > 10:
                if self.ser:
                    logger.info(f"CRSF Heartbeat: Sent {packet_cnt} frames. Status: {'ARMED' if self.armed else 'DIS'}")
                last_heartbeat = time.time()
                packet_cnt = 0
            
            elapsed = time.time() - start_time
            sleep_time = self.interval - elapsed
            if sleep_time > 0: time.sleep(sleep_time)

    def close(self):
        logger.info("Closing CRSF Sender...")
        self.running = False
        if hasattr(self, 'thread'): self.thread.join(timeout=1.0)
        if self.ser: self.ser.close()
