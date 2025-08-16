# app_ai.py — Reactive Car GUI (#2) + UDP setpoints to Simulator (#1)
# - Start / Pause / Stop
# - Pick car (folder) and road tile
# - Curvature-based roads (gentle_left/right, s_curve) with dψ/dt = u·κ
# - Auto-zoom & camera follow
# - Evasive target (Off/Mild/Strong): WEAVE + BURST with lateral + fore/aft bob + apparent scale
# - Forward Bias (px) -> ensures nonzero dy so External mode keeps moving
# - UDP JSON (send): {"dx": px_right, "dy": px_down, "dz_rate": mps}
# - UDP JSON (recv): backward/forward compatible; accepts extra {locked,conf,tid,fps}
# - AI panel docked on the RIGHT column (never covers Start/Pause/Stop)
from __future__ import annotations
import tkinter as tk
from tkinter import filedialog, ttk, messagebox
import os, json, time, math, socket, glob, warnings, random




from ai.yolo_panel import YoloDeepSortPanel
warnings.filterwarnings("ignore", message=".*weights_only=False.*", category=FutureWarning)
warnings.filterwarnings("ignore", category=FutureWarning, module="ultralytics")
warnings.filterwarnings("ignore", category=FutureWarning, module="deep_sort_realtime")

try:
    from PIL import Image, ImageTk
except Exception:
    raise SystemExit("Please install Pillow: pip install pillow")

try:
    from config import UDP_LISTEN_PORT
except Exception:
    UDP_LISTEN_PORT = 47800

APP_TITLE = "GoMax Reactive Car (AI Feeder)"
WINDOW_W, WINDOW_H = 1200, 720
TARGET_FPS = 60
SEND_HZ    = 20.0

ASSETS_ROADS_DIR = os.path.join("assets", "roads")
ASSETS_VEH_DIRS  = [os.path.join("assets", "vehicles"),
                    os.path.join("assets", "cars")]

REQUIRED_VIEWS = [
    "front", "front_left", "left", "rear_left",
    "rear", "rear_right", "right", "front_right", "top"
]

# Camera & motion
PAN_DAMP      = 0.12
ZOOM_MIN, ZOOM_MAX = 0.35, 1.75
ZOOM_S0       = 25.0     # range where zoom≈1.0
CAR_U_CRUISE  = 14.0     # m/s
CAR_U_MAX     = 22.0     # m/s
CAR_ACC       = 4.0      # m/s^2
PIX_PER_M     = 3.0      # visual road scroll

# AI defaults
AI_SOURCE   = "0"
AI_BACKEND  = "dshow"
AI_WIDTH    = 1280
AI_HEIGHT   = 720
AI_FPS      = 30
AI_MODEL    = "yolov8n.pt"
AI_CLASSES  = (2,5,7)
AI_BIAS_Y   = 120.0
AI_SEND_HZ  = 20.0

# --------------------------------------------------------------------
# --- UDP consumer & control tunables (sim side) ---------------------
UDP_TTL_S          = 0.25
DX_DEADBAND_PX     = 10.0
DX_EMA_ALPHA       = 0.15
K_YAW_DEGPS_PER_PX = 0.45
YAW_RATE_MAX       = 120.0
K_V_MPS_PER_PX     = 0.04
V_CRUISE           = 15.0
V_MIN, V_MAX       = 6.0, 22.0
WEAK_LOCK_SCALE    = 0.4
# --------------------------------------------------------------------

# --------------------------------------------------------------------
# --- Evasive controller tunables (straight_vertical only) -----------
EV_LOCK_EMA_A      = 0.85          # conf_ema = a*prev + (1-a)*conf
EV_T1              = 0.45          # weave threshold
EV_T2              = 0.70          # burst arm threshold
EV_T_BURST_ARM     = 0.60          # conf_ema ≥ T2 for this long to trigger
EV_BURST_LEN_S     = (0.8, 1.2)
EV_COOLDOWN_S      = (1.5, 2.5)
EV_SPEED_UP        = (1.20, 1.30)  # +20–30%
EV_SPEED_DOWN      = (0.70, 0.80)  # −20–30%

# lateral weave
EV_A_MILD_PX       = (32.0, 48.0)
EV_A_STRONG_PX     = (64.0, 96.0)
EV_F_HZ            = (0.18, 0.28)  # weave frequency
EV_NOISE_F_HZ      = 0.02          # small freq jitter
EV_LANE_PX         = 280.0
EV_LAT_GAIN_PXS    = 140.0
EV_LAT_V_MAX_PXS   = 220.0

# fore/aft bob (vertical)
EV_LONG_A_MILD_PX  = (18.0, 28.0)
EV_LONG_A_STRONG_PX= (32.0, 52.0)
EV_LONG_GAIN_PXS   = 120.0
EV_LONG_V_MAX_PXS  = 180.0

# apparent scale (sprite only; not the background)
EV_SCALE_MIN       = 0.92
EV_SCALE_MAX       = 1.10
EV_SCALE_RESP      = 0.20  # lerp factor per frame to target scale
# --------------------------------------------------------------------

def clamp(x, a, b): return a if x < a else b if x > b else x
def lerp(a, b, t):  return a + (b - a) * t

# ---------------- Car sprite ----------------

class CarSprite:
    def __init__(self, folder: str):
        self.folder = folder
        self.meta = {
            "id": os.path.basename(folder),
            "anchor_px": [0.5, 0.85],
            "length_m": 4.3,
            "width_m": 1.7,
            "scale_base_px": 480
        }
        meta_path = os.path.join(folder, "meta.json")
        if os.path.exists(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as f:
                    info = json.load(f)
                if isinstance(info, dict):
                    self.meta.update(info)
            except Exception:
                pass

        self.raw = {}
        for v in REQUIRED_VIEWS:
            p = os.path.join(folder, f"{v}.jpg")
            if os.path.exists(p):
                try:
                    self.raw[v] = Image.open(p).convert("RGBA")
                except Exception:
                    self.raw[v] = None
            else:
                self.raw[v] = None

        def fb(name, opts):
            if self.raw[name] is None:
                for o in opts:
                    if self.raw.get(o) is not None:
                        self.raw[name] = self.raw[o]; break
        fb("front_left",  ["front", "left"])
        fb("front_right", ["front", "right"])
        fb("rear_left",   ["rear", "left"])
        fb("rear_right",  ["rear", "right"])

        self.cache = {}  # (view, long_px:int) -> PhotoImage

    def _view_for_heading(self, car_hdg_deg: float, cam_hdg_deg: float = 90.0) -> str:
        rel = (car_hdg_deg - cam_hdg_deg + 360.0) % 360.0
        sectors = [
            (337.5, 360.0, "rear"), (0.0, 22.5, "rear"),
            (22.5,  67.5,  "rear_right"),
            (67.5,  112.5, "right"),
            (112.5, 157.5, "front_right"),
            (157.5, 202.5, "front"),
            (202.5, 247.5, "front_left"),
            (247.5, 292.5, "left"),
            (292.5, 337.5, "rear_left"),
        ]
        for a,b,name in sectors:
            if a <= rel <= b:
                return name
        return "rear"

    def get_sprite(self, car_hdg_deg: float, zoom: float, cam_hdg_deg: float = 90.0):
        view = self._view_for_heading(car_hdg_deg, cam_hdg_deg)
        img = self.raw.get(view) or self.raw.get("rear")
        if img is None:
            raise RuntimeError("No car images found in selected folder.")
        base = float(self.meta.get("scale_base_px", 480))
        tgt_long = clamp(base * zoom, base * ZOOM_MIN, base * ZOOM_MAX)
        key = (view, int(round(tgt_long)))
        if key in self.cache:
            return self.cache[key], view
        w, h = img.size
        long_side = max(w, h)
        scale = tgt_long / max(1.0, long_side)
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        rs = img.resize((new_w, new_h), Image.LANCZOS)
        ph = ImageTk.PhotoImage(rs)
        self.cache[key] = ph
        return ph, view

    def anchor_px(self, sprite_photo):
        ax, ay = self.meta.get("anchor_px", [0.5, 0.85])
        return int(sprite_photo.width() * ax), int(sprite_photo.height() * ay)

# ---------------- Road tiler ----------------

class RoadScroller:
    def __init__(self, tile_path: str):
        self.set_tile(tile_path)
        name = os.path.basename(tile_path).lower()
        self.mode = "horizontal" if "horizontal" in name else "vertical"
        self.offset = 0.0

    def set_tile(self, path: str):
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        self.tile = Image.open(path).convert("RGB")
        self.tk = ImageTk.PhotoImage(self.tile)

    def draw(self, canvas: tk.Canvas, pan_x: float, pan_y: float):
        w = max(2, canvas.winfo_width())
        h = max(2, canvas.winfo_height())
        tw, th = self.tk.width(), self.tk.height()
        if self.mode == "vertical":
            x0 = int(-pan_x % tw) - tw
            y0 = int(self.offset % th) - th
            x = x0
            while x < w + tw:
                y = y0
                while y < h + th:
                    canvas.create_image(x, y, anchor="nw", image=self.tk)
                    y += th
                x += tw
        else:
            x0 = int(self.offset % tw) - tw
            y0 = int(-pan_y % th) - th
            x = x0
            while x < w + tw:
                y = y0
                while y < h + th:
                    canvas.create_image(x, y, anchor="nw", image=self.tk)
                    y += th
                x += tw

    def advance(self, v_ground_px: float, dt: float):
        self.offset += v_ground_px * dt

# ---------------- UDP (sender) ----------------

class UdpSender:
    def __init__(self, host="127.0.0.1", port=UDP_LISTEN_PORT):
        self.host = host
        self.port = int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._t_last = 0.0

    def reconfig(self, host, port):
        self.host = host
        self.port = int(port)

    def maybe_send(self, dx, dy, dz_rate, hz=SEND_HZ, extra=None):
        now = time.time()
        if now - self._t_last < (1.0 / hz):
            return
        pkt = {"dx": float(dx), "dy": float(dy), "dz_rate": float(dz_rate)}
        if isinstance(extra, dict):
            pkt.update(extra)  # additive fields (e.g., {"veh": {...}})
        try:
            self.sock.sendto(json.dumps(pkt).encode("utf-8"), (self.host, self.port))
            self._t_last = now
        except Exception:
            pass

# ---------------- UDP (receiver) ----------------

class UdpReceiver:
    """Non-blocking UDP JSON receiver for AI setpoints (back/forward compatible)."""
    def __init__(self, host="0.0.0.0", port=UDP_LISTEN_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((host, int(port)))
        except Exception:
            self.sock.bind((host, 0))
        self.sock.setblocking(False)
        self.last_pkt = {}
        self.last_t = 0.0
        self.rx_count = 0

    def poll(self):
        got = False
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except Exception:
                break
            try:
                obj = json.loads(data.decode("utf-8"))
                self.last_pkt = obj if isinstance(obj, dict) else {}
                self.last_t = time.time()
                self.rx_count += 1
                got = True
            except Exception:
                pass
        return (self.last_pkt if got else None), self.last_t

# --------------- Utils (sim-side filter) ----------------

def deadband(x: float, b: float) -> float:
    return 0.0 if -b <= x <= b else (x - math.copysign(b, x))

class _SimDxFilter:
    """Sim-side dx stabilizer: symmetric deadband + EMA."""
    def __init__(self, alpha=DX_EMA_ALPHA, db=DX_DEADBAND_PX):
        self.a = float(alpha)
        self.db = float(db)
        self.x = 0.0
    def reset(self):
        self.x = 0.0
    def update(self, dx_px: float) -> float:
        dbx = deadband(float(dx_px), self.db)
        self.x = self.a * dbx + (1.0 - self.a) * self.x
        return self.x

# --------------- TargetController (new) ----------------

class TargetController:
    """
    Tiny state machine that makes the target 'react' to being locked'.
    Returns desired lateral offset (px), desired fore/aft offset (px) and speed factor.
    """
    def __init__(self, mode="mild"):
        self.mode = mode  # 'off'|'mild'|'strong'
        self.reset()

    def reset(self):
        self.conf_ema = 0.0
        self.state = "IDLE"
        self.t_above = 0.0
        self.burst_left = 0.0
        self.cooldown_left = 0.0
        self.phase = 0.0
        self.freq = random.uniform(*EV_F_HZ)
        self.sign = 1
        self.des_lat = 0.0
        self.des_long = 0.0
        self.speed_factor = 1.0
        self.unlock_timer = 0.0
        self._burst_factor = 1.0

    def set_mode(self, m: str):
        self.mode = m.lower()
        if self.mode == "off":
            self.reset()

    def _amps_lat(self):
        return EV_A_MILD_PX if self.mode != "strong" else EV_A_STRONG_PX

    def _amps_long(self):
        return EV_LONG_A_MILD_PX if self.mode != "strong" else EV_LONG_A_STRONG_PX

    def update(self, locked: bool, conf: float, dx_px: float, dt: float,
               is_straight_vertical: bool, lane_px: float):
        # Always keep EMA updated so overlay shows real values
        c_in = conf if locked else 0.0
        self.conf_ema = EV_LOCK_EMA_A * self.conf_ema + (1.0 - EV_LOCK_EMA_A) * c_in

        if self.mode == "off" or not is_straight_vertical:
            self.state = "IDLE"
            self.des_lat = 0.0
            self.des_long = 0.0
            self.speed_factor = 1.0
            if not locked:
                self.unlock_timer += dt
            else:
                self.unlock_timer = 0.0
            return self.des_lat, self.des_long, self.speed_factor, self._debug(0.0, 0.0)

        # timers
        if locked:
            self.unlock_timer = 0.0
        else:
            self.unlock_timer += dt

        # choose outward sign to increase |dx|
        self.sign = 1 if dx_px >= 0.0 else -1

        # state transitions
        if self.burst_left > 0.0:
            self.state = "BURST"
            self.burst_left -= dt
            if self.burst_left <= 0.0:
                self.cooldown_left = random.uniform(*EV_COOLDOWN_S)
                self._burst_factor = 1.0
                self.state = "WEAVE"
        else:
            if self.conf_ema >= EV_T1:
                if self.state == "IDLE":
                    self.state = "WEAVE"
                    self.t_above = 0.0
                if self.cooldown_left > 0.0:
                    self.cooldown_left -= dt
                else:
                    if self.conf_ema >= EV_T2:
                        self.t_above += dt
                        if self.t_above >= EV_T_BURST_ARM:
                            # start a burst (speed up OR brake)
                            if random.random() < 0.5:
                                self._burst_factor = random.uniform(*EV_SPEED_UP)
                            else:
                                self._burst_factor = random.uniform(*EV_SPEED_DOWN)
                            self.burst_left = random.uniform(*EV_BURST_LEN_S)
                            self.state = "BURST"
                            self.t_above = 0.0
                    else:
                        self.t_above = 0.0
            else:
                self.state = "IDLE"
                self.t_above = 0.0

        # amplitudes & frequency
        a_lat_min, a_lat_max = self._amps_lat()
        a_long_min, a_long_max = self._amps_long()
        growth = clamp((self.conf_ema - EV_T1) / max(1e-6, 1.0 - EV_T1), 0.0, 1.0)
        A_lat  = a_lat_min  + (a_lat_max  - a_lat_min)  * growth
        A_long = a_long_min + (a_long_max - a_long_min) * growth

        self.freq = clamp(self.freq + random.uniform(-EV_NOISE_F_HZ, EV_NOISE_F_HZ), EV_F_HZ[0], EV_F_HZ[1])
        self.phase += 2.0 * math.pi * self.freq * dt

        # desired offsets
        if self.state in ("WEAVE", "BURST"):
            off_lat  = self.sign * A_lat  * math.sin(self.phase)
            off_long =           A_long * math.cos(self.phase + math.pi / 3.0)  # phase-shifted bob
        else:
            # decay while idle/unlocked
            fade = clamp(1.0 - (self.unlock_timer / 1.0), 0.0, 1.0)
            off_lat  = self.des_lat  * fade * 0.92
            off_long = self.des_long * fade * 0.92

        # clamp inside lane for lateral
        half_lane = max(20.0, lane_px * 0.5 - 8.0)
        self.des_lat = clamp(off_lat,  -half_lane, +half_lane)
        self.des_long = clamp(off_long, -0.60 * half_lane, +0.60 * half_lane)  # vertical less than lateral

        # speed factor
        self.speed_factor = self._burst_factor if self.state == "BURST" else 1.0

        return self.des_lat, self.des_long, self.speed_factor, self._debug(A_lat, A_long)

    def _debug(self, A_lat, A_long):
        return {
            "conf_ema": self.conf_ema,
            "state": self.state,
            "A_lat": A_lat,
            "A_long": A_long,
            "burst_left": max(0.0, self.burst_left),
            "cooldown": max(0.0, self.cooldown_left),
            "freq": self.freq
        }

# --------------- App ----------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.configure(bg="#121212")
        try:
            icon_path = os.path.join(os.path.dirname(__file__), "logo.png")
            if os.path.exists(icon_path):
                self.iconphoto(True, tk.PhotoImage(file=icon_path))
        except Exception:
            pass

        self.columnconfigure(0, weight=3, uniform="cols")
        self.columnconfigure(1, weight=1, uniform="cols")
        self.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(self, bg="#0d0f12", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.ctrl = tk.Frame(self, bg="#1c1c1c")
        self.ctrl.grid(row=0, column=1, sticky="nsew")
        for i in range(42):
            self.ctrl.rowconfigure(i, weight=0)
        self.ctrl.columnconfigure(0, weight=1)

        # Top buttons
        top = tk.Frame(self.ctrl, bg="#1c1c1c")
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=(10,6))
        ttk.Button(top, text="Start", command=self.start_run).pack(side="left")
        ttk.Button(top, text="Pause", command=self.pause_run).pack(side="left", padx=6)
        ttk.Button(top, text="Stop",  command=self.stop_run).pack(side="left")

        # UDP target
        tk.Label(self.ctrl, text="UDP Target", fg="#e8f1ff", bg="#1c1c1c").grid(row=1, column=0, sticky="w", padx=10, pady=(10,2))
        netf = tk.Frame(self.ctrl, bg="#1c1c1c"); netf.grid(row=2, column=0, sticky="ew", padx=10)
        tk.Label(netf, text="IP:", bg="#1c1c1c", fg="#cfd8dc").pack(side="left")
        self.ip_var = tk.StringVar(value="127.0.0.1")
        tk.Entry(netf, textvariable=self.ip_var, width=12).pack(side="left", padx=(4,10))
        tk.Label(netf, text="Port:", bg="#1c1c1c", fg="#cfd8dc").pack(side="left")
        self.port_var = tk.StringVar(value=str(UDP_LISTEN_PORT))
        tk.Entry(netf, textvariable=self.port_var, width=8).pack(side="left", padx=(4,0))
        self.send_chk = tk.BooleanVar(value=True)
        tk.Checkbutton(self.ctrl, text="Send UDP (only while running)", variable=self.send_chk,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c")\
            .grid(row=3, column=0, sticky="w", padx=10, pady=(2,8))

        # Assets: car & road
        tk.Label(self.ctrl, text="Assets", fg="#e8f1ff", bg="#1c1c1c").grid(row=4, column=0, sticky="w", padx=10, pady=(10,2))
        carsf = tk.Frame(self.ctrl, bg="#1c1c1c"); carsf.grid(row=5, column=0, sticky="ew", padx=10)
        tk.Label(carsf, text="Car:", bg="#1c1c1c", fg="#cfd8dc").pack(side="left")
        self.cars = scan_car_dirs()
        self.car_var = tk.StringVar(value=os.path.basename(self.cars[0]) if self.cars else "")
        self.car_map = {os.path.basename(p): p for p in self.cars}
        self.car_dd = ttk.Combobox(carsf, values=list(self.car_map.keys()), textvariable=self.car_var,
                                   width=28, state="readonly")
        self.car_dd.pack(side="left", padx=(6,8))
        self.car_dd.bind("<<ComboboxSelected>>", self._on_car_dd)
        ttk.Button(carsf, text="Choose Car Folder…", command=self.choose_car).pack(side="left")
        ttk.Button(carsf, text="Rescan", command=self._rescan_cars).pack(side="left", padx=(6,0))

        roadf = tk.Frame(self.ctrl, bg="#1c1c1c"); roadf.grid(row=6, column=0, sticky="ew", padx=10, pady=(6,0))
        tk.Label(roadf, text="Road:", bg="#1c1c1c", fg="#cfd8dc").pack(side="left")
        rlist = roads_list() or ["straight_vertical.png"]
        self.road_var = tk.StringVar(value=rlist[0])
        self.road_dd = ttk.Combobox(roadf, values=rlist, textvariable=self.road_var, width=28, state="readonly")
        self.road_dd.pack(side="left", padx=(6,8))
        self.road_dd.bind("<<ComboboxSelected>>", lambda e: self.load_road())

        # Behavior
        tk.Label(self.ctrl, text="Behavior", fg="#e8f1ff", bg="#1c1c1c").grid(row=7, column=0, sticky="w", padx=10, pady=(12,2))
        bf = tk.Frame(self.ctrl, bg="#1c1c1c"); bf.grid(row=8, column=0, sticky="ew", padx=10)
        self.locked_var = tk.BooleanVar(value=True)   # speed up when locked
        tk.Checkbutton(bf, text="Locked (faster)", variable=self.locked_var,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").pack(side="left")
        self.jitter_var = tk.BooleanVar(value=False)  # legacy wobble
        tk.Checkbutton(bf, text="Weave jitter", variable=self.jitter_var,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").pack(side="left", padx=(12,0))

        zf = tk.Frame(self.ctrl, bg="#1c1c1c"); zf.grid(row=9, column=0, sticky="ew", padx=10, pady=(6,0))
        tk.Label(zf, text="dz_rate:", bg="#1c1c1c", fg="#cfd8dc").pack(side="left")
        self.dz_var = tk.StringVar(value="0.0")
        tk.Entry(zf, textvariable=self.dz_var, width=8).pack(side="left", padx=(6,14))
        tk.Label(zf, text="Forward bias (px):", bg="#1c1c1c", fg="#cfd8dc").pack(side="left")
        self.bias_var = tk.StringVar(value="170")
        tk.Entry(zf, textvariable=self.bias_var, width=8).pack(side="left", padx=(6,0))

        # Mapping/Signs
        mapf = tk.Frame(self.ctrl, bg="#1c1c1c"); mapf.grid(row=10, column=0, sticky="ew", padx=10, pady=(6,0))
        self.forward_up = tk.BooleanVar(value=True)
        tk.Checkbutton(mapf, text="Forward = Up (invert dy)", variable=self.forward_up,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").pack(side="left")
        self.swap_curve = tk.BooleanVar(value=False)
        tk.Checkbutton(mapf, text="Swap Left/Right", variable=self.swap_curve,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").pack(side="left", padx=(16,0))
        self.zero_dx_straight = tk.BooleanVar(value=True)
        tk.Checkbutton(self.ctrl, text="Zero dx on straight roads", variable=self.zero_dx_straight,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").grid(row=11, column=0, sticky="w", padx=10, pady=(6,0))

        # Evasive target
        tk.Label(self.ctrl, text="Evasive target", fg="#e8f1ff", bg="#1c1c1c").grid(row=12, column=0, sticky="w", padx=10, pady=(12,2))
        ef = tk.Frame(self.ctrl, bg="#1c1c1c"); ef.grid(row=13, column=0, sticky="w", padx=10)
        self.evade_var = tk.StringVar(value="mild")
        ttk.Radiobutton(ef, text="Off",    value="off",    variable=self.evade_var,
                        command=lambda: self.tc.set_mode(self.evade_var.get())).pack(side="left")
        ttk.Radiobutton(ef, text="Mild",   value="mild",   variable=self.evade_var,
                        command=lambda: self.tc.set_mode(self.evade_var.get())).pack(side="left", padx=(8,0))
        ttk.Radiobutton(ef, text="Strong", value="strong", variable=self.evade_var,
                        command=lambda: self.tc.set_mode(self.evade_var.get())).pack(side="left", padx=(8,0))

        # Overlays
        tk.Label(self.ctrl, text="Overlays", fg="#e8f1ff", bg="#1c1c1c").grid(row=14, column=0, sticky="w", padx=10, pady=(12,2))
        of = tk.Frame(self.ctrl, bg="#1c1c1c"); of.grid(row=15, column=0, sticky="ew", padx=10)
        self.ov_reticle = tk.BooleanVar(value=True)
        tk.Checkbutton(of, text="Reticle", variable=self.ov_reticle, bg="#1c1c1c", fg="#cfd8dc",
                       activebackground="#1c1c1c").pack(side="left")
        self.ov_box = tk.BooleanVar(value=False)
        tk.Checkbutton(of, text="BBox", variable=self.ov_box, bg="#1c1c1c", fg="#cfd8dc",
                       activebackground="#1c1c1c").pack(side="left")

        # Status
        self.status = tk.Label(self.ctrl, text="Ready", fg="#b0bec5", bg="#1c1c1c", anchor="w", justify="left")
        self.status.grid(row=16, column=0, sticky="ew", padx=10, pady=(12,10))

        # ---- AI panel on the RIGHT column (never covers the map) --------------
        self.ai_panel = YoloDeepSortPanel(
            self.ctrl,
            source=AI_SOURCE, backend=AI_BACKEND, width=AI_WIDTH, height=AI_HEIGHT, fps=AI_FPS,
            model_path=AI_MODEL, classes_keep=AI_CLASSES,
            host=self.ip_var.get(), port=self._port_safe(),
            send_hz=AI_SEND_HZ,
            dz_rate=self._float_safe(self.dz_var, 0.0),
            bias_y=AI_BIAS_Y, ema=0.35
        )
        self.ctrl.rowconfigure(17, weight=1)
        self.ai_panel.grid(row=17, column=0, sticky="nsew", padx=10, pady=(6, 10))

        # UDP (sender)
        self.tx = UdpSender(self.ip_var.get(), self._port_safe())

        # UDP consumer
        self.rx = UdpReceiver(host="0.0.0.0", port=self._port_safe())
        self.dx_filter = _SimDxFilter(alpha=DX_EMA_ALPHA, db=DX_DEADBAND_PX)
        self._udp_hz_ema = 0.0
        self._udp_last_seen = 0.0
        self._udp_rx_count_prev = 0

        # Commanded kinematics (from AI setpoints)
        self.yaw_rate_cmd = 0.0
        self.v_cmd = V_CRUISE
        self.vz_cmd = 0.0

        # Telemetry from AI (or panel probe)
        self.ai_locked = False
        self.ai_conf = 0.0
        self.ai_tid = -1
        self.ai_fps = 0.0

        # Fallback lock timer when panel doesn't send locked/conf
        self._fallback_lock_timer = 0.0

        # Evasive controller
        self.tc = TargetController(self.evade_var.get())
        self._evade_dbg = {"conf_ema":0.0,"state":"IDLE","A_lat":0.0,"A_long":0.0,"burst_left":0.0,"cooldown":0.0,"freq":0.0}
        self._last_dx_px = 0.0  # for sign choice on first frame
        self.evade_sprite_scale = 1.0  # apparent size multiplier

        # Load assets
        if not self.cars:
            d = filedialog.askdirectory(title="Select vehicle folder (with the 9 JPG views)")
            if not d:
                messagebox.showerror("No car selected", "Please choose a car folder with the 9 JPG views.")
                self.destroy(); return
            self._load_car_dir(d)
        else:
            self._load_car_dir(self.car_map[self.car_var.get()])

        self.road = None
        self.load_road()

        # Scene state
        self.running  = False
        self.last_t   = time.time()
        self.cam_x, self.cam_y = 0.0, 0.0
        self.cam_zoom = 1.0
        self.car_x, self.car_y = 0.0, 0.0
        self.car_hdg = 90.0
        self.car_u   = CAR_U_CRUISE
        self.range_m = 25.0
        self.s_travel = 0.0
        self._sprite_ref = None

        # Keys
        self.bind("<space>", lambda e: self.start_run() if not self.running else self.pause_run())
        self.bind("<s>",     lambda e: self.stop_run())
        self.bind("<S>",     lambda e: self.stop_run())

        self.after(int(1000 / TARGET_FPS), self._tick)

    # -------- safe getters / helpers
    def _infer_veh_type(self, veh_id: str) -> str:
        n = (veh_id or "").lower()
        if "bus" in n: return "bus"
        if "pickup" in n or "truck" in n: return "pickup"
        return "sedan"


    def _float_safe(self, var_str: tk.StringVar, default=0.0) -> float:
        try: return float(str(var_str.get()).strip())
        except Exception: return float(default)

    def _int_safe(self, var_str: tk.StringVar, default=0) -> int:
        try: return int(float(str(var_str.get()).strip()))
        except Exception: return int(default)

    def _port_safe(self) -> int:
        p = self._int_safe(self.port_var, UDP_LISTEN_PORT)
        return p if 1 <= p <= 65535 else UDP_LISTEN_PORT

    def _panel_probe(self):
        """If UDP is stale, read lock/conf/fps and dx/dy straight from the embedded panel."""
        p = getattr(self, "ai_panel", None)
        if p is None:
            return None
        try:
            locked = (p.lock_id is not None)
            conf = float(p._conf_by_tid.get(p.lock_id, 0.0)) if locked else 0.0
            fps = float(p._fps_ema) if p._fps_ema else 0.0
            dx = float(p.dx_s)
            dy = float(p.dy_s)
            return {"locked": locked, "conf": conf, "fps": fps, "dx": dx, "dy": dy}
        except Exception:
            return None

    # -------- buttons/events

    def start_run(self):
        self.running = True
        if hasattr(self, "ai_panel"):
            self.ai_panel.set_udp_target(self.ip_var.get(), self._port_safe(), AI_SEND_HZ)
        self.rx = UdpReceiver(host="0.0.0.0", port=self._port_safe())
        self.tc.reset()
        self.status.config(text="Running (sending UDP if enabled)")

    def pause_run(self):
        self.running = False
        self.status.config(text="Paused (no UDP)")

    def stop_run(self):
        self.running = False
        self.cam_x, self.cam_y = 0.0, 0.0
        self.cam_zoom = 1.0
        self.car_x, self.car_y = 0.0, 0.0
        name = (self.road_var.get() or "").lower()
        self.car_hdg = 0.0 if "horizontal" in name else 90.0
        self.car_u   = CAR_U_CRUISE
        self.range_m = 25.0
        self.s_travel = 0.0
        self.dx_filter.reset()
        self.yaw_rate_cmd = 0.0
        self.v_cmd = V_CRUISE
        self.vz_cmd = 0.0
        self.tc.reset()
        self.evade_sprite_scale = 1.0
        self.status.config(text="Stopped (reset)")

    def _on_car_dd(self, _ev=None):
        name = self.car_var.get()
        d = self.car_map.get(name)
        if d: self._load_car_dir(d)

    def _rescan_cars(self):
        self.cars = scan_car_dirs()
        self.car_map = {os.path.basename(p): p for p in self.cars}
        names = list(self.car_map.keys())
        self.car_dd["values"] = names
        if names:
            self.car_var.set(names[0])
            self._load_car_dir(self.car_map[names[0]])
        else:
            messagebox.showwarning("No cars found", "No valid car folders under assets/cars or assets/vehicles.")

    def choose_car(self):
        d = filedialog.askdirectory(title="Select vehicle folder (with the 9 JPG views)")
        if d and os.path.isdir(d):
            ok = all(os.path.exists(os.path.join(d, f"{v}.jpg")) for v in REQUIRED_VIEWS)
            if not ok:
                messagebox.showerror("Missing files", "That folder doesn't contain all 9 required JPGs.")
                return
            self._load_car_dir(d)
            key = os.path.basename(d)
            self.car_map[key] = d
            vals = list(set(list(self.car_dd["values"]) + [key]))
            self.car_dd["values"] = sorted(vals)
            self.car_var.set(key)

    def _load_car_dir(self, d: str):
        try:
            self.car = CarSprite(d)
            self.status.config(text=f"Loaded car: {os.path.basename(d)}")
        except Exception as e:
            messagebox.showerror("Car load error", f"{e}")

    def load_road(self):
        name = self.road_var.get()
        path = os.path.join(ASSETS_ROADS_DIR, name)
        try:
            self.road = RoadScroller(path)
            self.status.config(text=f"Road tile: {name}")
            self.car_hdg = 0.0 if "horizontal" in name.lower() else 90.0
        except Exception as e:
            messagebox.showerror("Road error", f"Could not load road tile:\n{e}")

    # -------- curvature model

    def _road_curvature(self, s_m: float) -> float:
        name = (self.road_var.get() or "").lower()
        k = 0.0
        if "gentle_left" in name:
            k = +0.020
        elif "gentle_right" in name:
            k = -0.020
        elif "s_curve" in name:
            k = 0.030 * math.sin((2.0 * math.pi / 120.0) * s_m)
        elif "roundabout" in name:
            k = +0.030
        else:
            k = 0.0
        if self.swap_curve.get():
            k = -k
        return k

    # -------- update/draw

    def _physics_and_camera(self, dt):
        cw = max(2, self.canvas.winfo_width())
        ch = max(2, self.canvas.winfo_height())

        if self.running:
            name = (self.road_var.get() or "").lower()
            base_hdg = 0.0 if "horizontal" in name else 90.0

            # speed model (base)
            if self.locked_var.get():
                self.car_u = min(CAR_U_MAX, self.car_u + CAR_ACC * dt)
            else:
                self.car_u = max(CAR_U_CRUISE, self.car_u - (CAR_ACC * 0.75) * dt)

            # curvature -> heading rate
            kappa = self._road_curvature(self.s_travel)
            self.car_hdg += (self.car_u * kappa) * (180.0 / math.pi) * dt

            # ------------------ Evasive controller (gated) ------------------
            is_straight_vertical = ("straight" in name) and ("vertical" in name)
            lane_px = min(EV_LANE_PX, cw * 0.48)

            # current offsets in px relative to screen center
            dx_now_px = (cw * 0.5 + (self.car_x - self.cam_x)) - (cw * 0.5)
            dy_now_px = (ch * 0.5 + (self.car_y - self.cam_y)) - (ch * 0.5)

            lat_des_px, long_des_px, speed_fac, dbg = self.tc.update(
                locked=self.ai_locked, conf=self.ai_conf,
                dx_px=dx_now_px if dx_now_px == dx_now_px else self._last_dx_px,
                dt=dt, is_straight_vertical=is_straight_vertical, lane_px=lane_px
            )
            self._evade_dbg = dbg

            # Only apply when:
            apply_recent_fade = (dbg.get("state") == "IDLE" and self.tc.unlock_timer < 1.0)
            apply_evasion = (
                self.evade_var.get() != "off"
                and is_straight_vertical
                and (dbg.get("state") in ("WEAVE", "BURST") or apply_recent_fade)
            )

            if apply_evasion:
                # speed factor (BURST/WEAVE)
                if dbg.get("state") in ("WEAVE", "BURST"):
                    u_target = clamp(self.car_u * speed_fac, V_MIN, V_MAX * 1.3)
                    self.car_u = lerp(self.car_u, u_target, 0.35)

                # lateral move toward desired
                lat_err = lat_des_px - dx_now_px
                lat_v = clamp(EV_LAT_GAIN_PXS * lat_err, -EV_LAT_V_MAX_PXS, EV_LAT_V_MAX_PXS)
                self.car_x += lat_v * dt

                # fore/aft move toward desired
                long_err = long_des_px - dy_now_px
                long_v = clamp(EV_LONG_GAIN_PXS * long_err, -EV_LONG_V_MAX_PXS, EV_LONG_V_MAX_PXS)
                self.car_y += long_v * dt

                # apparent scale from fore/aft desired (closer => bigger)
                A_long = max(1e-6, max(EV_LONG_A_MILD_PX[1], EV_LONG_A_STRONG_PX[1]))
                scale_bias = clamp(long_des_px / A_long, -1.0, 1.0)
                s_target = clamp(1.0 + 0.08 * scale_bias, EV_SCALE_MIN, EV_SCALE_MAX)
                self.evade_sprite_scale = lerp(self.evade_sprite_scale, s_target, EV_SCALE_RESP)
            else:
                self.evade_sprite_scale = lerp(self.evade_sprite_scale, 1.0, EV_SCALE_RESP)
            # ---------------------------------------------------------------

            # legacy jitter (unchanged)
            if self.jitter_var.get():
                self.car_hdg += 10.0 * math.sin(3.2 * time.time()) * dt

            # stabilizer (strong on straight)
            err = ((base_hdg - self.car_hdg + 540) % 360) - 180
            if "straight" in name:
                stab_gain = 24.0
            else:
                stab_gain = 8.0 * max(0.0, 1.0 - min(1.0, abs(kappa) / 0.02))
            self.car_hdg += clamp(err, -stab_gain * dt, stab_gain * dt)

            # integrate position (y down) for road scroll
            vx = self.car_u * math.cos(math.radians(self.car_hdg))
            vy = -self.car_u * math.sin(math.radians(self.car_hdg))
            self.car_x += vx * dt
            self.car_y += vy * dt
            self.s_travel += max(0.0, self.car_u * dt)

            # camera follow with deadzone
            dead_x = cw * 0.10
            dead_y = ch * 0.12
            dx_cam = (self.car_x - self.cam_x)
            dy_cam = (self.car_y - self.cam_y)
            if abs(dx_cam) > dead_x:
                self.cam_x += (dx_cam - math.copysign(dead_x, dx_cam)) * PAN_DAMP
            if abs(dy_cam) > dead_y:
                self.cam_y += (dy_cam - math.copysign(dead_y, dy_cam)) * PAN_DAMP

            # range/zoom (camera)
            if self.locked_var.get():
                self.range_m = max(8.0, self.range_m - 6.0 * dt)
            else:
                self.range_m = min(80.0, self.range_m + 4.0 * dt)
            z_target = clamp(ZOOM_S0 / (ZOOM_S0 + max(1.0, self.range_m)), ZOOM_MIN, ZOOM_MAX)
            self.cam_zoom = lerp(self.cam_zoom, z_target, 0.10)

            # background scroll
            if self.road:
                self.road.advance(self.car_u * PIX_PER_M, dt)

        # screen coord of car anchor
        car_sx = cw * 0.5 + (self.car_x - self.cam_x)
        car_sy = ch * 0.5 + (self.car_y - self.cam_y)

        # raw error (px, right/down positive)
        dx_px = (car_sx - cw * 0.5)
        dy_px = (car_sy - ch * 0.5)
        self._last_dx_px = dx_px

        # forward bias and sign for outgoing UDP
        bias = self._float_safe(self.bias_var, 0.0)
        if self.forward_up.get():
            dy_cmd = -(dy_px - bias)  # forward = up
        else:
            dy_cmd = +(dy_px - bias)  # forward = down

        # kill dx on straight roads if requested
        if self.zero_dx_straight.get() and "straight" in (self.road_var.get() or "").lower():
            dx_cmd = 0.0
        else:
            dx_cmd = dx_px

        dz_rate = self._float_safe(self.dz_var, 0.0)
        return dx_cmd, dy_cmd, dz_rate, (car_sx, car_sy)

    def _draw(self, car_screen_xy):
        c = self.canvas
        c.delete("all")
        if self.road:
            self.road.draw(c, self.cam_x, self.cam_y)
        # NOTE: apply apparent scale on sprite (camera zoom * evade scale)
        sprite, view = self.car.get_sprite(self.car_hdg, self.cam_zoom * self.evade_sprite_scale, cam_hdg_deg=90.0)
        ax, ay = self.car.anchor_px(sprite)
        sx, sy = car_screen_xy
        x0 = int(sx - ax)
        y0 = int(sy - ay)
        c.create_image(x0, y0, anchor="nw", image=sprite)
        self._sprite_ref = sprite
        cw = c.winfo_width(); ch = c.winfo_height()
        if self.ov_reticle.get():
            c.create_line(cw//2 - 20, ch//2, cw//2 + 20, ch//2, fill="#66aaff")
            c.create_line(cw//2, ch//2 - 20, cw//2, ch//2 + 20, fill="#66aaff")
        if self.ov_box.get():
            c.create_rectangle(x0, y0, x0 + sprite.width(), y0 + sprite.height(), outline="#ff5252")
        txt = f"view={view}  hdg={self.car_hdg:5.1f}°  u={self.car_u:4.1f} m/s  zoom={self.cam_zoom:3.2f}  s={self.s_travel:5.1f} m"
        c.create_text(8, 8, anchor="nw", text=txt, fill="#e0e0e0")

        # Telemetry overlay
        dbg = self._evade_dbg
        c.create_text(8, 28, anchor="nw",
                      text=(f"lock_conf_ema={dbg.get('conf_ema',0.0):.2f}  "
                            f"evade={dbg.get('state','IDLE').lower()}  "
                            f"A_lat≈{dbg.get('A_lat',0.0):.0f}px  "
                            f"A_long≈{dbg.get('A_long',0.0):.0f}px  "
                            f"burst_t={dbg.get('burst_left',0.0):.1f}s  "
                            f"fps={self.ai_fps:4.1f}"),
                      fill="#a8e6ff")

    def _tick(self):
        now = time.time()
        dt = max(1.0 / 200.0, min(0.050, now - self.last_t))
        self.last_t = now

        # --- UDP consumer (AI -> sim) with robust fallback ---
        pkt, _ = self.rx.poll()
        got_udp = pkt is not None
        dx = dy = dz = 0.0

        if got_udp:
            try:
                dx = float(pkt.get("dx", 0.0))
                dy = float(pkt.get("dy", 0.0))
                dz = float(pkt.get("dz_rate", 0.0))
            except Exception:
                dx, dy, dz = 0.0, 0.0, 0.0

            has_locked = ("locked" in pkt)
            has_conf   = ("conf"   in pkt)
            has_tid    = ("tid"    in pkt)

            if not has_locked and not has_conf:
                # proximity-based fallback
                R = 320.0
                conf_proxy = max(0.0, min(1.0, 1.0 - (math.hypot(dx, dy) / R)))
                self._fallback_lock_timer = (self._fallback_lock_timer + dt) if conf_proxy >= EV_T1 else 0.0
                locked_eff = (self._fallback_lock_timer >= 0.30)
                conf_eff   = conf_proxy
            else:
                locked_eff = bool(pkt.get("locked", False)) if has_locked else True
                if has_conf:
                    conf_eff = float(pkt.get("conf", 0.0))
                else:
                    R = 320.0
                    conf_eff = max(0.0, min(1.0, 1.0 - (math.hypot(dx, dy) / R)))

            self.ai_tid = int(pkt.get("tid", -1)) if has_tid else -1
            self.ai_fps = float(pkt.get("fps", 0.0) or 0.0)

            self.ai_locked = locked_eff
            self.ai_conf   = conf_eff

            # frequency estimate
            rx_new  = max(0, self.rx.rx_count - self._udp_rx_count_prev)
            elapsed = now - self._udp_last_seen if self._udp_last_seen > 0 else 0.0
            inst_hz = (rx_new / elapsed) if elapsed > 1e-3 else 0.0
            self._udp_rx_count_prev = self.rx.rx_count
            self._udp_last_seen     = now
            self._udp_hz_ema = (0.9 * self._udp_hz_ema + 0.1 * inst_hz) if self._udp_hz_ema > 0 else inst_hz

        # If UDP is stale, probe the embedded panel directly to keep evasive reactive
        if (time.time() - max(self.rx.last_t, self._udp_last_seen)) > UDP_TTL_S:
            pp = self._panel_probe()
            if pp is not None:
                self.ai_locked = pp["locked"]
                self.ai_conf   = pp["conf"]
                self.ai_fps    = pp["fps"]
                dx, dy         = pp["dx"], pp["dy"]

        # Mapping: dx -> yaw_rate (deadband/EMA), dy -> v, dz -> vz
        dx_s = self.dx_filter.update(dx)
        yaw  = clamp(K_YAW_DEGPS_PER_PX * dx_s, -YAW_RATE_MAX, YAW_RATE_MAX)
        v    = clamp(V_CRUISE + K_V_MPS_PER_PX * dy, V_MIN, V_MAX)
        vz   = float(dz)

        if (not self.ai_locked) or (self.ai_conf < 0.50):
            yaw *= WEAK_LOCK_SCALE
            v = V_CRUISE + (v - V_CRUISE) * WEAK_LOCK_SCALE

        self.yaw_rate_cmd, self.v_cmd, self.vz_cmd = yaw, v, vz

        # If input truly stale and no panel probe, drop to safe commands
        if (time.time() - max(self.rx.last_t, self._udp_last_seen)) > UDP_TTL_S and self._panel_probe() is None:
            self.yaw_rate_cmd = 0.0
            self.v_cmd = V_CRUISE
            self.vz_cmd = 0.0
            self.dx_filter.reset()

        # Advance sim & camera
        dx_out, dy_out, dz_out, (sx, sy) = self._physics_and_camera(dt)

        # Optional outbound UDP (unchanged)
        if self.running and self.send_chk.get():
            self.tx.reconfig(self.ip_var.get().strip() or "127.0.0.1", self._port_safe())
            # center of the car on GUI#2 canvas (normalize to 0..1)
            cw = max(1, self.canvas.winfo_width())
            ch = max(1, self.canvas.winfo_height())
            cx01 = max(0.0, min(1.0, (sx / cw)))
            cy01 = max(0.0, min(1.0, (sy / ch)))

            # vehicle identity from the currently loaded car folder
            veh_id = os.path.basename(getattr(self.car, "folder", ""))
            veh_type = self._infer_veh_type(veh_id)

            extra = {"veh": {"id": veh_id, "type": veh_type, "cx01": cx01, "cy01": cy01}}
            self.tx.maybe_send(dx_out, dy_out, dz_out, SEND_HZ, extra=extra)
                        

        # Draw scene
        self._draw((sx, sy))

        # Status/HUD
        fresh = (time.time() - max(self.rx.last_t, self._udp_last_seen)) <= UDP_TTL_S
        fresh_txt = "fresh" if fresh else "STALE"
        dx_raw = float(self.rx.last_pkt.get("dx", 0.0)) if isinstance(self.rx.last_pkt, dict) else 0.0
        dy_raw = float(self.rx.last_pkt.get("dy", 0.0)) if isinstance(self.rx.last_pkt, dict) else 0.0
        dz_raw = float(self.rx.last_pkt.get("dz_rate", 0.0)) if isinstance(self.rx.last_pkt, dict) else 0.0

        self.status.config(
            text=f"{'Running' if self.running else 'Paused/Stopped'} | "
                 f"UDP {self.ip_var.get()}:{self._port_safe()} {self._udp_hz_ema:4.1f} Hz ({fresh_txt}) | "
                 f"dx={dx_raw:+.0f}px  dy={dy_raw:+.0f}px  dz={dz_raw:+.2f} m/s  "
                 f"→ yaw={self.yaw_rate_cmd:+.1f}°/s  v={self.v_cmd:4.2f} m/s  vz={self.vz_cmd:+.2f} m/s | "
                 f"evade: {self._evade_dbg.get('state','IDLE').lower()}"
        )

        # Next frame
        self.after(int(1000 / TARGET_FPS), self._tick)

# ------------- helper scans -------------

def scan_car_dirs():
    found = []
    for root in ASSETS_VEH_DIRS:
        if not os.path.isdir(root):
            continue
        for d in sorted(glob.glob(os.path.join(root, "*"))):
            if not os.path.isdir(d):
                continue
            ok = True
            for v in REQUIRED_VIEWS:
                if not os.path.exists(os.path.join(d, f"{v}.jpg")):
                    ok = False; break
            if ok:
                found.append(d)
    return found

def roads_list():
    if not os.path.isdir(ASSETS_ROADS_DIR):
        return []
    return sorted([os.path.basename(p) for p in glob.glob(os.path.join(ASSETS_ROADS_DIR, "*.png"))])

# ------------- main -------------

if __name__ == "__main__":
    app = App()
    app.mainloop()
