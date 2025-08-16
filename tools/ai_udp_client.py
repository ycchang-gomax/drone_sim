# tools/ai_udp_client.py — tiny demo: listen to telemetry, send commands
import socket, json, time, math

SIM_HOST = "127.0.0.1"
SIM_CMD_PORT = 47800   # -> sim (matches config.UDP_LISTEN_PORT)
SIM_TELEM_PORT = 47801 # <- from sim

rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind((SIM_HOST, SIM_TELEM_PORT))
rx.settimeout(0.25)

tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = (SIM_HOST, SIM_CMD_PORT)

phase = 0.0
while True:
    # read any telemetry (not strictly needed for this demo)
    try:
        data, _ = rx.recvfrom(65536)
        telem = json.loads(data.decode("utf-8"))
        # you can print(telem) to verify
    except socket.timeout:
        telem = None

    # send a small oscillation to prove end-to-end control
    phase += 0.04
    cmd = {
        "dx": 180.0 * math.sin(phase),   # pixels right/left
        "dy": 120.0 * math.cos(0.6*phase),
        "dz_rate": 0.0
    }
    tx.sendto(json.dumps(cmd).encode("utf-8"), addr)
    time.sleep(0.03)
