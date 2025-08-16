# simio/udp_tx.py — simple JSON UDP sender for telemetry
import socket, json

class TelemetrySender:
    def __init__(self, host: str = "127.0.0.1", port: int = 47801):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, obj) -> None:
        try:
            payload = json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            self.sock.sendto(payload, self.addr)
        except Exception:
            # keep sim robust even if no listener is present
            pass

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass
