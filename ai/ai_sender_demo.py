# ai_sender_demo.py  -- prove UDP control loop works
import socket, json, time, math

SIM_HOST = "127.0.0.1"
SIM_PORT =  47800

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

def send(dx, dy, dz_rate):
    pkt = json.dumps({"dx": float(dx), "dy": float(dy), "dz_rate": float(dz_rate)}).encode("utf-8")
    sock.sendto(pkt, (SIM_HOST, SIM_PORT))

t0 = time.time()
print("Sending test setpoints to simulator… (Ctrl+C to stop)")
try:
    while True:
        t = time.time() - t0
        # simple test pattern: sweep left/right, slight climb/descent
        dx = 200 * math.sin(0.4 * t)     # pixels, +right
        dy =  80 * math.cos(0.3 * t)     # pixels, +down
        dz =  0.5 * math.sin(0.2 * t)    # m/s climb (+ up)
        send(dx, dy, dz)
        time.sleep(1/20)  # ~20 Hz
except KeyboardInterrupt:
    pass
