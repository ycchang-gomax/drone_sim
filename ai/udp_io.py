# ai/udp_io.py — tiny JSON UDP send/recv
import socket, json, time

class UdpSender:
    def __init__(self, host: str, port: int):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, obj):
        try:
            data = json.dumps(obj).encode("utf-8")
            self.sock.sendto(data, self.addr)
        except Exception:
            pass

class UdpReceiver:
    def __init__(self, port: int, bufsize: int = 65535):
        self.port = int(port)
        self.bufsize = bufsize
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.setblocking(False)

    def poll(self):
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(self.bufsize)
            except BlockingIOError:
                break
            except Exception:
                break
            try:
                out.append(json.loads(data.decode("utf-8")))
            except Exception:
                pass
        return out
