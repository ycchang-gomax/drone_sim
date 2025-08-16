# make_drone_sim_scaffold.py
# Creates the full pure-Python/Tkinter drone_sim scaffold locally.
import os, time

ROOT = "drone_sim"
DIRS = [
    ROOT,
    f"{ROOT}/core",
    f"{ROOT}/io",
    f"{ROOT}/ui",
    f"{ROOT}/assets",
]

FILES = {}

FILES[f"{ROOT}/app.py"] = r'''# app.py — Entry point
from ui.main_window import MainWindow

def main():
    app = MainWindow()
    app.mainloop()

if __name__ == "__main__":
    main()
'''

FILES[f"{ROOT}/config.py"] = r'''# config.py — runtime settings (pure stdlib)
from dataclasses import dataclass

ASSETS_DIR = "assets"
MAP_IMAGE  = f"{ASSETS_DIR}/map_bg.png"   # fallback to grid if not found/loadable

# GUI
WINDOW_W = 1200
WINDOW_H = 720
FPS      = 60              # GUI redraw
DT       = 1.0 / 100.0     # physics step (s)

# World scaling (meters-to-pixels on the map)
METERS_PER_PIXEL = 1.0     # tune to your background; 1 px = 1 m by default

# I/O
UDP_LISTEN_PORT = 47800
UART_DEV        = "/dev/ttyUSB0"
UART_BAUD       = 115200

@dataclass
class Limits:
    V_MAX: float   = 15.0  # m/s
    A_MAX: float   = 4.0   # m/s^2 (not used in scaffold, future)
    A_LAT_MAX: float = 5.0 # m/s^2 lateral (future)
    VZ_MAX: float  = 3.0   # m/s
    YAW_RATE_MAX: float = 120.0 # deg/s cap

@dataclass
class Gains:
    K_YAW: float  = 90.0   # deg/s at dx=1
    K_FWD: float  = 8.0    # m/s at |err|=1
    V_BASE: float = 4.0    # m/s base forward

@dataclass
class WindDefault:
    east: float = 2.0  # m/s +x (east)
    north: float = 0.0 # m/s +y (north)
    up: float = 0.0    # m/s +z (up)
'''

FILES[f"{ROOT}/core/state.py"] = r'''# core/state.py — simulation state (pure stdlib)
from dataclasses import dataclass, field

@dataclass
class DroneState:
    # Position in ENU frame [m]
    x: float = 0.0   # East
    y: float = 0.0   # North
    z: float = 10.0  # Up (altitude AGL for now)

    # Attitude [deg]
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0   # heading (0 = east, 90 = north)

    # Velocities
    v_forward: float = 0.0  # along body x (m/s)
    vz: float = 0.0         # climb rate (m/s)

    # Trail (recent positions for plotting)
    trail: list = field(default_factory=list)

@dataclass
class EnvState:
    wind_e: float = 0.0  # m/s
    wind_n: float = 0.0
    wind_u: float = 0.0

@dataclass
class CmdSetpoints:
    yaw_rate_dps: float = 0.0  # desired yaw rate [deg/s]
    v_forward: float = 0.0     # desired forward speed [m/s]
    vz: float = 0.0            # desired climb rate [m/s]

@dataclass
class LinkStatus:
    source: str = "DEMO"     # DEMO | UDP | UART
    connected: bool = False
    last_rx_ms: int = 0
    packets: int = 0
    drops: int = 0
    mode: int = 0
    conf: int = 0            # tracker confidence (0..100)
'''

FILES[f"{ROOT}/core/physics.py"] = r'''# core/physics.py — simple kinematics (pure stdlib)
import math
from .state import DroneState, EnvState, CmdSetpoints

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def step(dt: float, st: DroneState, env: EnvState, cmd: CmdSetpoints,
         v_max: float, vz_max: float, yaw_rate_max_dps: float):
    """Advance state by dt using very simple kinematics.
    - ENU frame, yaw in deg (0=east)
    - v_forward cmd capped by v_max
    - yaw rate cmd capped by yaw_rate_max_dps
    - vz cmd capped by vz_max
    - Adds constant wind (E,N) to groundspeed
    """
    # Limit commands
    yaw_rate = clamp(cmd.yaw_rate_dps, -yaw_rate_max_dps, yaw_rate_max_dps)
    v_fwd    = clamp(cmd.v_forward, 0.0, v_max)
    vz       = clamp(cmd.vz, -vz_max, vz_max)

    # Update attitude (scaffold: only yaw evolves from cmd)
    st.yaw = (st.yaw + yaw_rate * dt) % 360.0

    # Body forward in ENU
    yaw_rad = math.radians(st.yaw)
    be = math.cos(yaw_rad)
    bn = math.sin(yaw_rad)

    # Ground velocity components (body forward + wind)
    ve = be * v_fwd + env.wind_e
    vn = bn * v_fwd + env.wind_n
    vu = vz + env.wind_u  # keep zero by default

    # Integrate position
    st.x += ve * dt
    st.y += vn * dt
    st.z += vu * dt

    # Save velocities (for HUD)
    st.v_forward = v_fwd
    st.vz = vz

    # Trail (keep last N points)
    st.trail.append((st.x, st.y))
    if len(st.trail) > 2000:
        del st.trail[0:len(st.trail)-2000]
'''

FILES[f"{ROOT}/core/guidance.py"] = r'''# core/guidance.py — map tracker errors to setpoints (pure stdlib)
from .state import CmdSetpoints

def map_tracking_to_setpoints(dx_norm: float, dy_norm: float, dz_rate: float,
                              k_yaw_dps: float, k_fwd: float, v_base: float,
                              v_max: float, vz_max: float) -> CmdSetpoints:
    # Yaw rate command (deg/s)
    yaw_rate = k_yaw_dps * dx_norm
    # Forward speed command (m/s)
    err_mag = max(abs(dx_norm), abs(dy_norm))
    v_cmd = v_base + k_fwd * err_mag
    if v_cmd > v_max:
        v_cmd = v_max
    # Vertical rate (already m/s)
    vz = dz_rate
    return CmdSetpoints(yaw_rate_dps=yaw_rate, v_forward=v_cmd, vz=vz)
'''

FILES[f"{ROOT}/io/protocol.py"] = r'''# io/protocol.py — framing/deframing + CRC16-CCITT (pure stdlib)
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
'''

FILES[f"{ROOT}/io/udp_rx.py"] = r'''# io/udp_rx.py — nonblocking UDP listener (pure stdlib)
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
'''

FILES[f"{ROOT}/io/uart_rx.py"] = r'''# io/uart_rx.py — placeholder (pure stdlib approach)
# For a pure-stdlib build, implementing robust UART on Linux uses termios/fcntl.
# We'll add it in a later pack. For now this is a stub so imports succeed.
class UartReceiver:
    def __init__(self, dev: str, baud: int):
        self.dev = dev
        self.baud = baud
        self.last_rx_ms = 0
        self.packets = 0
        self.drops = 0

    def poll(self):
        # TODO: implement in later step
        return []
'''

FILES[f"{ROOT}/ui/map_view.py"] = r'''# ui/map_view.py — Tkinter Canvas map & overlays (pure stdlib)
import math, os
import tkinter as tk

class MapView(tk.Frame):
    def __init__(self, master, map_path: str, meters_per_px: float = 1.0):
        super().__init__(master, bg='#202020')
        self.canvas = tk.Canvas(self, bg='#1e1e1e', highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.bind('<Configure>', self._on_resize)
        self.map_img = None
        self.map_img_id = None
        self.map_path = map_path
        self.m_per_px = meters_per_px
        self.center_px = (0,0)  # pixel center
        self._drone_items = {'body': None, 'trail': None, 'fov': None}

    def _on_resize(self, _evt=None):
        self.canvas.delete('all')
        w = self.winfo_width()
        h = self.winfo_height()
        if w <= 2 or h <= 2:
            return
        self.center_px = (w//2, h//2)
        # Try to load background image
        self._draw_background(w, h)

    def _draw_background(self, w, h):
        try:
            self.map_img = tk.PhotoImage(file=self.map_path)
            self.map_img_id = self.canvas.create_image(w//2, h//2, image=self.map_img)
        except Exception:
            self._draw_grid(w, h)

    def _draw_grid(self, w, h, step=50):
        self.canvas.create_rectangle(0,0,w,h, fill='#2a2a2a', outline='')
        for x in range(0, w, step):
            self.canvas.create_line(x, 0, x, h, fill='#3c3c3c')
        for y in range(0, h, step):
            self.canvas.create_line(0, y, w, y, fill='#3c3c3c')
        self.canvas.create_text(w-8, h-8, anchor='se', fill='#808080',
                                text='No map_bg.png found — showing grid')

    # World ENU meters -> canvas pixels
    def world_to_px(self, x_m, y_m):
        cx, cy = self.center_px
        px = cx + int(x_m / self.m_per_px)
        py = cy - int(y_m / self.m_per_px)
        return px, py

    def draw_drone(self, x_m, y_m, yaw_deg, fov_deg=60, trail=None):
        # trail
        if trail:
            pts = []
            for (wx, wy) in trail[-800:]:
                px, py = self.world_to_px(wx, wy)
                pts.extend([px, py])
            self._ensure('trail', lambda: self.canvas.create_line(*pts, fill='#6cf', width=2, smooth=True))
            if pts:
                self.canvas.coords(self._drone_items['trail'], *pts)

        # FOV wedge
        x, y = self.world_to_px(x_m, y_m)
        r = 60
        a0 = -(yaw_deg + fov_deg/2.0)
        a1 = -(yaw_deg - fov_deg/2.0)
        self._ensure('fov', lambda: self.canvas.create_arc(x-r, y-r, x+r, y+r,
                                                           start=a0, extent=(a1-a0),
                                                           style='pieslice', outline='', fill='#44ffffff'))
        self.canvas.coords(self._drone_items['fov'], x-r, y-r, x+r, y+r)
        self.canvas.itemconfig(self._drone_items['fov'], start=a0, extent=(a1-a0))

        # Drone body (triangle)
        size = 14
        yaw = math.radians(yaw_deg)
        pts = []
        # tip
        pts.append((x + size*math.cos(yaw), y - size*math.sin(yaw)))
        # back-left
        pts.append((x + -size*0.6*math.cos(yaw) + -size*0.6*math.sin(yaw),
                    y - -size*0.6*math.sin(yaw) +  size*0.6*math.cos(yaw)))
        # back-right
        pts.append((x + -size*0.6*math.cos(yaw) +  size*0.6*math.sin(yaw),
                    y - -size*0.6*math.sin(yaw) -  size*0.6*math.cos(yaw)))
        flat = [c for p in pts for c in p]
        self._ensure('body', lambda: self.canvas.create_polygon(*flat, fill='#00c8ff', outline='#ffffff'))
        self.canvas.coords(self._drone_items['body'], *flat)

    def _ensure(self, key, creator):
        if self._drone_items[key] is None:
            self._drone_items[key] = creator()
'''

FILES[f"{ROOT}/ui/controls.py"] = r'''# ui/controls.py — Tk controls (pure stdlib)
import tkinter as tk
from tkinter import ttk

class ControlsPanel(tk.Frame):
    def __init__(self, master, on_start, on_pause, on_reset, on_params_change):
        super().__init__(master, bg='#232323', padx=8, pady=8)
        self.on_params_change = on_params_change

        # Buttons
        btn_row = tk.Frame(self, bg='#232323')
        btn_row.pack(fill=tk.X, pady=(0,8))
        tk.Button(btn_row, text='Start', command=on_start).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text='Pause', command=on_pause).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text='Reset', command=on_reset).pack(side=tk.LEFT, padx=4)

        # Sliders
        self.vmax = self._slider('V_MAX [m/s]', 0, 30, 15.0)
        self.vzmax = self._slider('VZ_MAX [m/s]', 0, 10, 3.0)
        self.yawmax = self._slider('Yaw rate max [deg/s]', 0, 240, 120.0)
        self.kyaw = self._slider('K_yaw [deg/s @1px]', 0, 180, 90.0)
        self.kf = self._slider('K_fwd [m/s @1err]', 0, 20, 8.0)
        self.vbase = self._slider('V_base [m/s]', 0, 10, 4.0)
        self.w_e = self._slider('Wind East [m/s]', -10, 10, 2.0)
        self.w_n = self._slider('Wind North [m/s]', -10, 10, 0.0)

        # Status
        self.status = tk.Label(self, text='Status: idle', anchor='w', fg='#cfcfcf', bg='#232323')
        self.status.pack(fill=tk.X, pady=(8,0))

        self._notify()

    def _slider(self, label, a, b, init):
        row = tk.Frame(self, bg='#232323')
        row.pack(fill=tk.X, pady=2)
        tk.Label(row, text=label, fg='#cfcfcf', bg='#232323').pack(side=tk.LEFT)
        var = tk.DoubleVar(value=init)
        s = ttk.Scale(row, from_=a, to=b, orient='horizontal', variable=var, command=lambda _e: self._notify())
        s.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=8)
        return var

    def _notify(self):
        params = {
            'V_MAX': self.vmax.get(),
            'VZ_MAX': self.vzmax.get(),
            'YAW_MAX': self.yawmax.get(),
            'K_YAW': self.kyaw.get(),
            'K_FWD': self.kf.get(),
            'V_BASE': self.vbase.get(),
            'W_E': self.w_e.get(),
            'W_N': self.w_n.get(),
        }
        self.on_params_change(params)
'''

FILES[f"{ROOT}/ui/main_window.py"] = r'''# ui/main_window.py — Tk root & main loop (pure stdlib)
import tkinter as tk
from tkinter import ttk
import time, math

from config import WINDOW_W, WINDOW_H, MAP_IMAGE, METERS_PER_PIXEL, DT, FPS
from config import Limits, Gains, WindDefault, UDP_LISTEN_PORT
from core.state import DroneState, EnvState, CmdSetpoints, LinkStatus
from core.physics import step as physics_step
from core.guidance import map_tracking_to_setpoints
from io.udp_rx import UdpReceiver
from ui.map_view import MapView
from ui.controls import ControlsPanel

class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Path Simulator (Scaffold)")
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.configure(bg='#1b1b1b')

        # State
        self.limits = Limits()
        self.gains  = Gains()
        self.state  = DroneState()
        self.env    = EnvState(WindDefault().east, WindDefault().north, WindDefault().up)
        self.link   = LinkStatus(source='DEMO', connected=False)

        # Layout
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.rowconfigure(0, weight=1)

        self.map = MapView(self, MAP_IMAGE, METERS_PER_PIXEL)
        self.map.grid(row=0, column=0, sticky='nsew')

        self.controls = ControlsPanel(self, self._on_start, self._on_pause, self._on_reset, self._on_params_change)
        self.controls.grid(row=0, column=1, sticky='nsew')

        # Timer
        self.running = False
        self.accum = 0.0
        self.prev_time = time.time()

        # I/O (UDP only in scaffold)
        self.udp = UdpReceiver(port=UDP_LISTEN_PORT)

        # Demo command (if no external RX)
        self.demo_phase = 0.0

        # Kick GUI refresh
        self.after(int(1000/FPS), self._on_gui_tick)

    def _on_params_change(self, p):
        self.limits.V_MAX = p['V_MAX']
        self.limits.VZ_MAX = p['VZ_MAX']
        self.limits.YAW_RATE_MAX = p['YAW_MAX']
        self.gains.K_YAW = p['K_YAW']
        self.gains.V_BASE = p['V_BASE']
        self.gains.K_FWD = p['K_FWD']
        self.env.wind_e = p['W_E']
        self.env.wind_n = p['W_N']

    def _on_start(self):
        self.running = True
        self.controls.status.config(text='Status: running')

    def _on_pause(self):
        self.running = False
        self.controls.status.config(text='Status: paused')

    def _on_reset(self):
        self.state = DroneState()
        self.controls.status.config(text='Status: reset')

    def _on_gui_tick(self):
        # Physics substeps
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        if self.running:
            self.accum += dt
            while self.accum >= DT:
                cmd = self._get_command()
                physics_step(DT, self.state, self.env, cmd,
                             v_max=self.limits.V_MAX,
                             vz_max=self.limits.VZ_MAX,
                             yaw_rate_max_dps=self.limits.YAW_RATE_MAX)
                self.accum -= DT

        # Draw
        self.map.draw_drone(self.state.x, self.state.y, self.state.yaw, trail=self.state.trail)

        # Next tick
        self.after(int(1000 / FPS), self._on_gui_tick)

    def _get_command(self) -> CmdSetpoints:
        # Poll UDP commands (JSON or binary)
        msgs = self.udp.poll()
        if msgs:
            self.link.connected = True
            self.link.packets += len(msgs)
            # Take the last message in this tick
            m = msgs[-1]
            dx = float(m.get('dx', 0.0))
            dy = float(m.get('dy', 0.0))
            dz = float(m.get('dz_rate', 0.0))
            self.link.conf = int(m.get('conf', 0))
            self.link.mode = int(m.get('mode', 0))
            return map_tracking_to_setpoints(dx, dy, dz,
                                             k_yaw_dps=self.gains.K_YAW,
                                             k_fwd=self.gains.K_FWD,
                                             v_base=self.gains.V_BASE,
                                             v_max=self.limits.V_MAX,
                                             vz_max=self.limits.VZ_MAX)
        # Otherwise DEMO: gentle turn + forward
        self.demo_phase += DT
        yaw_rate = 20.0 * math.sin(self.demo_phase * 0.4)
        v_fwd = 5.0
        return CmdSetpoints(yaw_rate_dps=yaw_rate, v_forward=v_fwd, vz=0.0)
'''

# Tiny 1x1 PNG (valid)
TINY_PNG = bytes.fromhex(
    "89504E470D0A1A0A0000000D49484452000000010000000108020000009077"
    "530000000A49444154789C6360000002000155AA0A2F0000000049454E44AE426082"
)

def main():
    for d in DIRS:
        os.makedirs(d, exist_ok=True)
    for path, content in FILES.items():
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    with open(f"{ROOT}/assets/map_bg.png", "wb") as f:
        f.write(TINY_PNG)
    print("OK. Project created at ./drone_sim")
    print("Run:")
    print("  cd drone_sim")
    print("  python3 app.py")
    print("\nSend a test JSON command (in another terminal):")
    print(r"""  echo '{"dx":0.5,"dy":-0.2,"dz_rate":0.0,"conf":90,"mode":1}' | nc -u -w0 127.0.0.1 47800""")

if __name__ == "__main__":
    main()
