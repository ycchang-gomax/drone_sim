# io/protocol.py — framing/deframing + CRC16-CCITT (pure stdlib)
import struct

START1 = 0xAA
START2 = 0x55

MSG_TRACK_CMD = 0x01

def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000):
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def frame_track_cmd(dx: float, dy: float, dz_rate: float, conf: int, mode: int) -> bytes:
    payload = struct.pack('<fffBBH', dx, dy, dz_rate, conf & 0xFF, mode & 0xFF, 0)
    header = bytes([START1, START2, len(payload)+1, MSG_TRACK_CMD])
    crc = crc16_ccitt(header[2:] + payload)
    return header + payload + struct.pack('<H', crc)

def deframe(buf: bytes):
    """Generator yielding (msg_id, payload_bytes) from a buffer."""
    i = 0
    n = len(buf)
    while i + 6 <= n:
        if buf[i] != START1 or buf[i+1] != START2:
            i += 1
            continue
        length = buf[i+2]
        msg_id = buf[i+3]
        end = i + 4 + length + 2
        if end > n:
            break  # wait for more data
        payload = buf[i+4: i+4+length]
        crc_recv = int.from_bytes(buf[i+4+length:i+4+length+2], 'little')
        calc = crc16_ccitt(buf[i+2:i+4] + payload)
        if calc == crc_recv:
            yield (msg_id, payload)
        i = end
