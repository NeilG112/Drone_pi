import struct

def create_packet(data_id, payload):
    """Constructs a MSP v1 packet (Copy of logic in MSPSender)."""
    length = len(payload)
    checksum = length ^ data_id
    for byte in payload:
        checksum ^= byte
    
    header = b'$M<'
    return header + struct.pack('<BB', length, data_id) + payload + struct.pack('<B', checksum)

def test_msp_packet():
    # MSP_SET_RAW_RC = 200
    # Payload: 8 uint16
    # Order: ROLL, PITCH, YAW, THROTTLE, AUX1...
    # Values: 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000
    rc_channels = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]
    payload = struct.pack('<8H', *rc_channels)
    
    packet = create_packet(200, payload)
    
    print(f"Packet: {packet.hex(' ')}")
    
    # Checksum calculation:
    # length (16) ^ id (200/0xC8) = 0xD8
    # 0xD8 XORed with all payload bytes:
    # 1500 = 0x05DC -> bytes (0xDC, 0x05)
    # 1000 = 0x03E8 -> bytes (0xE8, 0x03)
    # 0xDC ^ 0x05 ^ 0xDC ^ 0x05 ^ 0xDC ^ 0x05 ^ 0xE8 ^ 0x03 ^ 0xE8 ^ 0x03 ^ 0xE8 ^ 0x03 ^ 0xE8 ^ 0x03 ^ 0xE8 ^ 0x03 
    # = (0xDC^0xDC^0xDC) ^ (0x05^0x05^0x05) ^ (0xE8^0xE8^0xE8^0xE8^0xE8) ^ (0x03^0x03^0x03^0x03^0x03)
    # = 0xDC ^ 0x05 ^ 0xE8 ^ 0x03 = 0x02
    # Final Checksum calculation should be 0xEA for this payload:
    # 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000
    
    assert packet.startswith(b'$M<')
    assert packet[3] == 16 # length
    assert packet[4] == 200 # id
    assert packet[-1] == 0xEA # Correct checksum
    
    print("MSP Packet Verification PASSED!")

if __name__ == "__main__":
    test_msp_packet()
