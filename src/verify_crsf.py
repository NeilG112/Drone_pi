import struct

def crc8_crsf(data):
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

def create_crsf_frame(rc_channels):
    """Packs 16 x 11-bit channels into 22 bytes + Header + CRC."""
    payload = bytearray(22)
    bits = 0
    current_byte = 0
    for ch_val in rc_channels:
        for i in range(11):
            if ch_val & (1 << i):
                payload[current_byte] |= (1 << bits)
            bits += 1
            if bits == 8:
                bits = 0
                current_byte += 1
    
    # Sync(C8), Len(24), Type(16)
    SYNC = 0xC8
    TYPE = 0x16
    frame_head = struct.pack('BBB', SYNC, len(payload) + 2, TYPE)
    frame = frame_head + payload
    crc = crc8_crsf(frame[2:]) # CRC on Type + Payload
    return frame + struct.pack('B', crc)

def test_crsf():
    # Test neutral sticks (992)
    rc_channels = [992] * 16
    frame = create_crsf_frame(rc_channels)
    
    print(f"CRSF Frame (Neutral): {frame.hex(' ')}")
    
    # Header check
    assert frame[0] == 0xC8 # Sync
    assert frame[1] == 24 # Len
    assert frame[2] == 0x16 # Type
    
    # CRC check
    calculated_crc = crc8_crsf(frame[2:-1])
    assert frame[-1] == calculated_crc
    
    # Test min throttle (191)
    rc_channels[3] = 191
    frame_min = create_crsf_frame(rc_channels)
    print(f"CRSF Frame (Min Throttle): {frame_min.hex(' ')}")
    
    print("CRSF Frame Verification PASSED!")

if __name__ == "__main__":
    test_crsf()
