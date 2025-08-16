# app_ai.py — Reactive Car GUI (#2) + UDP setpoints to Simulator (#1)
# - Start / Pause / Stop
# - Pick car (folder) and road tile
# - Curvature-based roads (gentle_left/right, s_curve) with dψ/dt = u·κ
# - Auto-zoom & camera follow
# - Evasive behavior when "Locked"
# - Forward Bias (px) -> ensures nonzero dy so External mode keeps moving
# - UDP JSON: {"dx": px_right, "dy": px_down, "dz_rate": mps}
# - AI panel docked on the RIGHT column (never covers Start/Pause/Stop)
#
# Assets:
#   assets/roads/*.png
#   assets/vehicles/<car_id>/*.jpg  or assets/cars/<car_id>/*.jpg :
#       front.jpg, front_left.jpg, left.jpg, rear_left.jpg,
#       rear.jpg, rear_right.jpg, right.jpg, front_right.jpg, top.jpg
#   optional per-car meta.json (anchor_px, length_m, width_m, scale_base_px)
#
# Window icon: uses logo.png in the SAME folder as app_ai.py / app.py

import tkinter as tk
from tkinter import filedialog, ttk, messagebox
import os, json, time, math, socket, glob, warnings

from ai.yolo_panel import YoloDeepSortPanel
# silence noisy future warnings from torch/ultralytics/deepsort
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
AI_BACKEND  = "dshow"   # use "auto" or "msmf" if needed
AI_WIDTH    = 1280
AI_HEIGHT   = 720
AI_FPS      = 30
AI_MODEL    = "yolov8n.pt"
AI_CLASSES  = (2,5,7)   # car,bus,truck
AI_BIAS_Y   = 120.0
AI_SEND_HZ  = 20.0

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

# ---------------- UDP ----------------

class UdpSender:
    def __init__(self, host="127.0.0.1", port=UDP_LISTEN_PORT):
        self.host = host
        self.port = int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._t_last = 0.0

    def reconfig(self, host, port):
        self.host = host
        self.port = int(port)

    def maybe_send(self, dx, dy, dz_rate, hz=SEND_HZ):
        now = time.time()
        if now - self._t_last < (1.0 / hz):
            return
        pkt = json.dumps({"dx": float(dx), "dy": float(dy), "dz_rate": float(dz_rate)}).encode("utf-8")
        try:
            self.sock.sendto(pkt, (self.host, self.port))
            self._t_last = now
        except Exception:
            pass

# --------------- Utils ----------------

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
        for i in range(32):
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
        self.jitter_var = tk.BooleanVar(value=False)  # wobble OFF by default
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
        self.forward_up = tk.BooleanVar(value=True)     # invert dy when True
        tk.Checkbutton(mapf, text="Forward = Up (invert dy)", variable=self.forward_up,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").pack(side="left")
        self.swap_curve = tk.BooleanVar(value=False)    # swap L/R
        tk.Checkbutton(mapf, text="Swap Left/Right", variable=self.swap_curve,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").pack(side="left", padx=(16,0))
        self.zero_dx_straight = tk.BooleanVar(value=True)  # stop circling on straights
        tk.Checkbutton(self.ctrl, text="Zero dx on straight roads", variable=self.zero_dx_straight,
                       bg="#1c1c1c", fg="#cfd8dc", activebackground="#1c1c1c").grid(row=11, column=0, sticky="w", padx=10, pady=(6,0))

        # Overlays
        tk.Label(self.ctrl, text="Overlays", fg="#e8f1ff", bg="#1c1c1c").grid(row=12, column=0, sticky="w", padx=10, pady=(12,2))
        of = tk.Frame(self.ctrl, bg="#1c1c1c"); of.grid(row=13, column=0, sticky="ew", padx=10)
        self.ov_reticle = tk.BooleanVar(value=True)
        tk.Checkbutton(of, text="Reticle", variable=self.ov_reticle, bg="#1c1c1c", fg="#cfd8dc",
                       activebackground="#1c1c1c").pack(side="left")
        self.ov_box = tk.BooleanVar(value=False)
        tk.Checkbutton(of, text="BBox", variable=self.ov_box, bg="#1c1c1c", fg="#cfd8dc",
                       activebackground="#1c1c1c").pack(side="left")

        # Status
        self.status = tk.Label(self.ctrl, text="Ready", fg="#b0bec5", bg="#1c1c1c", anchor="w", justify="left")
        self.status.grid(row=14, column=0, sticky="ew", padx=10, pady=(12,10))

        # ---- AI panel on the RIGHT column (never covers the map) --------------
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
        # place with GRID (same parent as all other right-column widgets)
        self.ctrl.rowconfigure(15, weight=1)  # let the AI panel grow
        self.ai_panel.grid(row=15, column=0, sticky="nsew", padx=10, pady=(6, 10))


        # UDP (for this app's own simulated car)
        self.tx = UdpSender(self.ip_var.get(), self._port_safe())

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
        self.car_hdg = 90.0  # 90 = up (vertical)
        self.car_u   = CAR_U_CRUISE
        self.range_m = 25.0
        self.s_travel = 0.0
        self._sprite_ref = None

        # Keys
        self.bind("<space>", lambda e: self.start_run() if not self.running else self.pause_run())
        self.bind("<s>",     lambda e: self.stop_run())
        self.bind("<S>",     lambda e: self.stop_run())

        self.after(int(1000 / TARGET_FPS), self._tick)

    # -------- safe getters

    def _float_safe(self, var_str: tk.StringVar, default=0.0) -> float:
        try: return float(str(var_str.get()).strip())
        except Exception: return float(default)

    def _int_safe(self, var_str: tk.StringVar, default=0) -> int:
        try: return int(float(str(var_str.get()).strip()))
        except Exception: return int(default)

    def _port_safe(self) -> int:
        p = self._int_safe(self.port_var, UDP_LISTEN_PORT)
        return p if 1 <= p <= 65535 else UDP_LISTEN_PORT

    # -------- buttons/events

    def start_run(self):
        self.running = True
        if hasattr(self, "ai_panel"):
            self.ai_panel.set_udp_target(self.ip_var.get(), self._port_safe(), AI_SEND_HZ)
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
        """
        Curvature κ [1/m]. +κ = left turn (screen-left).
        """
        name = (self.road_var.get() or "").lower()
        k = 0.0
        if "gentle_left" in name:
            k = +0.020
        elif "gentle_right" in name:
            k = -0.020
        elif "s_curve" in name:
            k = 0.030 * math.sin((2.0 * math.pi / 120.0) * s_m)
        elif "roundabout" in name:
            k = +0.030   # steady circle
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

            # speed model
            if self.locked_var.get():
                self.car_u = min(CAR_U_MAX, self.car_u + CAR_ACC * dt)
            else:
                self.car_u = max(CAR_U_CRUISE, self.car_u - (CAR_ACC * 0.75) * dt)

            # curvature -> heading rate
            kappa = self._road_curvature(self.s_travel)
            self.car_hdg += (self.car_u * kappa) * (180.0 / math.pi) * dt

            # optional jitter
            if self.jitter_var.get():
                self.car_hdg += 10.0 * math.sin(3.2 * time.time()) * dt

            # stabilizer (strong on straight)
            err = ((base_hdg - self.car_hdg + 540) % 360) - 180
            if "straight" in name:
                stab_gain = 24.0
            else:
                stab_gain = 8.0 * max(0.0, 1.0 - min(1.0, abs(kappa) / 0.02))
            self.car_hdg += clamp(err, -stab_gain * dt, stab_gain * dt)

            # integrate position (y down)
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

            # range/zoom
            if self.locked_var.get():
                self.range_m = max(8.0, self.range_m - 6.0 * dt)
            else:
                self.range_m = min(80.0, self.range_m + 4.0 * dt)
            z_target = clamp(ZOOM_S0 / (ZOOM_S0 + max(1.0, self.range_m)), ZOOM_MIN, ZOOM_MAX)
            self.cam_zoom = lerp(self.cam_zoom, z_target, 0.10)

            # scroll background
            if self.road:
                self.road.advance(self.car_u * PIX_PER_M, dt)

        # screen coord of car anchor
        car_sx = cw * 0.5 + (self.car_x - self.cam_x)
        car_sy = ch * 0.5 + (self.car_y - self.cam_y)

        # raw error (px, right/down positive)
        dx_px = (car_sx - cw * 0.5)
        dy_px = (car_sy - ch * 0.5)

        # forward bias and sign
        bias = self._float_safe(self.bias_var, 0.0)
        if self.forward_up.get():
            dy_cmd = -(dy_px - bias)  # forward = up
        else:
            dy_cmd = +(dy_px - bias)  # forward = down

        # kill dx on straight roads to stop "circling"
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
        sprite, view = self.car.get_sprite(self.car_hdg, self.cam_zoom, cam_hdg_deg=90.0)
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

    def _tick(self):
        now = time.time()
        dt = max(1.0 / 200.0, min(0.050, now - self.last_t))
        self.last_t = now

        dx, dy, dz, (sx, sy) = self._physics_and_camera(dt)

        if self.running and self.send_chk.get():
            self.tx.reconfig(self.ip_var.get().strip() or "127.0.0.1", self._port_safe())
            self.tx.maybe_send(dx, dy, dz, SEND_HZ)

        self._draw((sx, sy))
        self.status.config(
            text=f"{'Running' if self.running else 'Paused/Stopped'} | UDP {self.ip_var.get()}:{self._port_safe()} | "
                 f"dx={dx:+.0f}px dy={dy:+.0f}px dz={dz:+.2f} m/s"
        )
        self.after(int(1000 / TARGET_FPS), self._tick)

# ------------- main -------------

if __name__ == "__main__":
    app = App()
    app.mainloop()
