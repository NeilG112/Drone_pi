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
    # Payload: 16 uint16
    # Order: ROLL, PITCH, YAW, THROTTLE, AUX1...
    # Values: 1500, 1500, 1500, 1000, 1000...
    rc_channels = [1500, 1500, 1500, 1000] + [1000] * 12
    payload = struct.pack('<16H', *rc_channels)
    
    packet = create_packet(200, payload)
    
    print(f"Packet: {packet.hex(' ')}")
    
    # 32 byte payload (20 hex)
    assert packet.startswith(b'$M<')
    assert packet[3] == 32 # length
    assert packet[4] == 200 # id
    assert packet[-1] == 0xDA # Calculated checksum for this 16-ch payload
    
    print("MSP Packet Verification (16 Channels) PASSED!")

if __name__ == "__main__":
    test_msp_packet()
