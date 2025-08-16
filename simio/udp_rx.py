# io/udp_rx.py — nonblocking UDP listener (pure stdlib)
import socket, json, struct, time
from .protocol import deframe, MSG_TRACK_CMD

class UdpReceiver:
    def __init__(self, port: int):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.last_rx_ms = 0
        self.packets = 0
        self.drops = 0

    def poll(self):
        """Non-blocking poll; returns list of parsed commands.
        Supported payloads:
          - JSON line: {"dx":..,"dy":..,"dz_rate":..,"conf":..,"mode":..}
          - Binary frames using io.protocol
        Returns list of dicts with keys dx,dy,dz_rate,conf,mode.
        """
        out = []
        while True:
            try:
                data, _addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            self.packets += 1
            now_ms = int(time.time()*1000)
            self.last_rx_ms = now_ms
            # Try JSON first
            try:
                cmd = json.loads(data.decode('utf-8').strip())
                if isinstance(cmd, dict) and 'dx' in cmd:
                    out.append({
                        'dx': float(cmd.get('dx', 0.0)),
                        'dy': float(cmd.get('dy', 0.0)),
                        'dz_rate': float(cmd.get('dz_rate', 0.0)),
                        'conf': int(cmd.get('conf', 0)),
                        'mode': int(cmd.get('mode', 0)),
                    })
                    continue
            except Exception:
                pass
            # Binary frames
            for msg_id, payload in deframe(data):
                if msg_id == MSG_TRACK_CMD and len(payload) >= (4*3 + 1 + 1 + 2):
                    dx, dy, dz_rate, conf, mode, _rsv = struct.unpack('<fffBBH', payload[:4*3+1+1+2])
                    out.append({'dx':dx, 'dy':dy, 'dz_rate':dz_rate, 'conf':conf, 'mode':mode})
        return out
