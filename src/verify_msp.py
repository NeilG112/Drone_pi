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
    # Payload: 8 * 1500 (uint16)
    rc_channels = [1500] * 8
    payload = struct.pack('<8H', *rc_channels)
    
    packet = create_packet(200, payload)
    
    print(f"Packet: {packet.hex(' ')}")
    
    # Header: 24 4d 3c ($M<)
    # Length: 10 (16 bytes)
    # ID: c8 (200)
    # Payload: dc 05 ... (1500 in hex is 05DC, little endian is DC 05)
    # Checksum: 10 ^ c8 = d8. Then XOR with each payload byte.
    # 1500 (0x05DC) XORed with self is 0. 
    # Since we have 8 identical uint16s, the payload XORs to 0.
    # So checksum should be d8.
    
    assert packet.startswith(b'$M<')
    assert packet[3] == 16 # length
    assert packet[4] == 200 # id
    assert packet[-1] == 0xD8 # checksum
    
    print("MSP Packet Verification PASSED!")

if __name__ == "__main__":
    test_msp_packet()
