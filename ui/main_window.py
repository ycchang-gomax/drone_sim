# ui/main_window.py — Mission sim + Video box (toolbar), Wind overlay, LED, logging
from __future__ import annotations
import tkinter as tk
import time, math, os, csv, datetime as dt
from typing import Optional, Tuple, Dict, Any, List

from config import WINDOW_W, WINDOW_H, MAP_IMAGE, METERS_PER_PIXEL, DT, FPS
from config import Limits, Gains, WindDefault, UDP_LISTEN_PORT, UDP_TELEM_PORT, GRAVITY
from core.state import DroneState, EnvState, CmdSetpoints, LinkStatus
from core.physics import step as physics_step
from core.guidance import map_tracking_to_setpoints
from core.nav import Waypoint, route_guidance_with_wind
from simio.udp_rx import UdpReceiver
from simio.udp_tx import TelemetrySender
from ui.map_view import MapView
from ui.controls import ControlsPanel
from ui.video_panel import VideoPanel

# Pillow (optional; overlay falls back to simple dot if missing)
try:
    from PIL import Image, ImageTk
except Exception:
    Image = ImageTk = None

VERSION_TAG = "MW-r21"
VEH_SEEN_HOLD_S = 5.0   # keep veh status this long after last veh packet

# ENU(m) -> GPS(lat/lon) around an origin (display only)
ORIGIN_LAT = 0.0
ORIGIN_LON = 0.0
def enu_to_latlon(x_m, y_m, lat0=ORIGIN_LAT, lon0=ORIGIN_LON):
    R = 6378137.0
    lat0r = math.radians(lat0)
    dlat = (y_m / R) * (180.0 / math.pi)
    dlon = (x_m / (R * math.cos(lat0r))) * (180.0 / math.pi)
    return lat0 + dlat, lon0 + dlon


class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"GoMax Drone Path Simulator [{VERSION_TAG}]")
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.configure(bg='#1b1b1b')
        self._last_rx_keys = []
        self._veh_forced_id = os.environ.get("SIM_VEH_ID", "").strip().lower()



        # Icon (optional)
        try:
            icon_path = os.path.join(os.path.dirname(__file__), "..", "logo.png")
            icon_path = os.path.abspath(icon_path)
            if os.path.exists(icon_path):
                self.iconphoto(True, tk.PhotoImage(file=icon_path))
        except Exception:
            pass

        # ---- State -------------------------------------------------------------
        self.limits = Limits()
        self.gains  = Gains()
        self.state  = DroneState()
        self.env    = EnvState(WindDefault().east, WindDefault().north, WindDefault().up)
        self.link   = LinkStatus(source='DEMO', connected=False)

        self.mode = 'Manual'          # Manual | External | Route
        self.vehicle_type = 'quad'    # quad | fixed
        self.v_cruise = 8.0
        self.route_start: Waypoint | None = None
        self.route_dest : Waypoint | None = None

        # Manual keys
        self.keys = set()
        self.man_v = 0.0
        self.man_yaw = 0.0
        self.man_vz = 0.0

        # Battery (very simple model)
        self.batt_cap_wh = 120.0
        self.batt_wh     = self.batt_cap_wh
        self.P0 = 60.0
        self.kv3 = 0.25
        self.kclimb = 80.0

        # Logging
        self.sim_time = 0.0
        self.log_file = None
        self.log_writer = None

        # Landing state (anchored to destination)
        self.landing = False
        self.landed  = False
        self.land_anchor = None

        # HUD/LED flags defaults
        self._loitering = False
        self._goaround = False
        self._wind_limited = False
        self._hold_headwind = False

        # Route timing + LED debounce
        self._route_t0 = None
        self._goaround_time = 0.0
        self._led_color = None
        self._led_last_change = time.time()
        self._led_min_dwell = 0.35  # seconds

        # External info cache
        self._ext_last_msg = None
        self._ext_last_cmd = None
        self._ext_last_ts  = 0.0
        self._ext_rx_hz    = 0.0

        # Touchdown debounce
        self._touch_t0 = None

        # Missed-approach helpers
        self._missed_active = False
        self._missed_t0 = 0.0
        self._route_start_alt = 20.0
        self._landing_t0 = None

        # ---- Layout ------------------------------------------------------------
        # 60:40 split (Map : Controls)
        self.columnconfigure(0, weight=3, uniform="cols")
        self.columnconfigure(1, weight=2, uniform="cols")
        self.rowconfigure(0, weight=1)

        # Map (left)
        self.map = MapView(self, MAP_IMAGE, METERS_PER_PIXEL)
        self.map.grid(row=0, column=0, sticky='nsew')

        # Controls (right)
        self.controls = ControlsPanel(self, self._on_start, self._on_pause, self._on_reset, self._on_params_change)
        self.controls.grid(row=0, column=1, sticky='nsew')

        # HUD (top-left over the map)
        self.hud = tk.Label(self.map.canvas, text="", fg='#ffffff', bg='#000000', padx=6, pady=3)
        self.hud.place(x=10, y=10)

        # Manual input banner (below HUD)
        self.input_banner = tk.Label(
            self.map.canvas, text="", fg='#ffe28a', bg='#000000',
            padx=6, pady=3, font=("Segoe UI", 10, "bold")
        )
        self.input_banner.place(x=10, y=86)
        self._last_input_ts = 0.0
        self._last_input_txt = ""

        # ------- External banner (right panel; only in External mode) ----------
        self.ext_frame = tk.Frame(self.controls, bg="#202020", highlightthickness=0)
        self.ext_lbl   = tk.Label(self.ext_frame, text="External: waiting…",
                                  fg="#e8f1ff", bg="#202020", justify="left", anchor="w")
        self.ext_lbl.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)
        self.ext_frame.place_forget()  # hidden by default

        # ------- Video box inside the right panel (under Status) ----------------
        self.video_frame = tk.Frame(self.controls, bg="#181818", highlightthickness=0)

        toolbar = tk.Frame(self.video_frame, bg="#222222")
        toolbar.pack(side=tk.TOP, fill=tk.X)

        btn_open = tk.Button(toolbar, text="Open…", command=lambda: self.video.open_file_dialog())
        btn_play = tk.Button(toolbar, text="Play/Pause", command=lambda: self.video.toggle_play())
        btn_stop = tk.Button(toolbar, text="Stop", command=lambda: self.video.stop())
        for b in (btn_open, btn_play, btn_stop):
            b.pack(side=tk.LEFT, padx=4, pady=3)

        self.video = VideoPanel(self.video_frame)
        self.video.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- VEHICLE OVERLAY: toggle + caches ----------------------------------
        self.veh_overlay = tk.BooleanVar(value=True)
        self.veh_frame = tk.Frame(self.controls, bg="#202020", highlightthickness=0)
        tk.Checkbutton(
            self.veh_frame, text="Show vehicle overlay",
            variable=self.veh_overlay, bg="#202020", fg="#cfd8dc",
            activebackground="#202020", selectcolor="#202020"
        ).pack(side=tk.LEFT, padx=8, pady=6)

        # cache from AI 'veh' packets
        self._veh_last = None        # latest dict
        self._veh_last_ts = 0.0      # when we received it
        self._veh_seq = 0            # increments each new packet with veh
        self._veh_seq_drawn = -1     # last seq rendered (for trail)
        from collections import deque
        self._veh_trail = deque(maxlen=150)  # world-coord trail (x,y)

        # icon caches / directory index
        self._veh_base_images = {}   # key -> PIL.Image
        self._veh_icon_cache  = {}   # (key, bucket, size_px) -> ImageTk.PhotoImage
        self._veh_size_px = {
            "sedan": 22, "suv": 24, "van": 24, "pickup": 24,
            "truck": 26, "bus": 28, "motorcycle": 18
        }
        self._veh_dir_index = None   # list of (name_lower, path_to_top)
        # -----------------------------------------------------------------------

        # --- Right-pane layout --------------------------------------------------
        def _layout_right_pane(_evt=None):
            try:
                self.controls.update_idletasks()
                pad = 8
                w = max(200, self.controls.winfo_width() - 2 * pad)

                # anchor under status
                if hasattr(self.controls, "status"):
                    y0 = self.controls.status.winfo_y() + self.controls.status.winfo_height() + pad
                else:
                    y0 = int(self.controls.winfo_height() * 0.62)

                # Vehicle overlay toggle (always visible)
                vh = 36
                self.veh_frame.place(x=pad, y=y0, width=w, height=vh)
                y0 += vh + pad

                # External banner (only in External mode)
                if self.mode == 'External':
                    ext_h = 46
                    self.ext_frame.place(x=pad, y=y0, width=w, height=ext_h)
                    y0 += ext_h + pad
                else:
                    self.ext_frame.place_forget()

                # Video fills the rest
                h = max(150, self.controls.winfo_height() - y0 - pad)
                self.video_frame.place(x=pad, y=y0, width=w, height=h)
            except Exception:
                self.after(60, _layout_right_pane)

        self._layout_right_pane = _layout_right_pane
        self.after(60, self._layout_right_pane)
        self.controls.bind("<Configure>", self._layout_right_pane)

        # Menu + shortcuts for video
        menubar = tk.Menu(self)
        filem = tk.Menu(menubar, tearoff=False)
        filem.add_command(label="Open Video…", accelerator="Ctrl+O", command=self.video.open_file_dialog)
        filem.add_separator()
        filem.add_command(label="Quit", accelerator="Ctrl+Q", command=self.destroy)
        menubar.add_cascade(label="File", menu=filem)
        self.config(menu=menubar)
        self.bind_all("<Control-o>", lambda e: self.video.open_file_dialog())
        self.bind_all("<Control-O>", lambda e: self.video.open_file_dialog())
        self.bind_all("<Command-o>", lambda e: self.video.open_file_dialog())  # macOS
        self.bind_all("<Control-q>", lambda e: self.destroy())
        self.bind_all("<Command-q>", lambda e: self.destroy())

        # Timers / IO
        self.running = False
        self.accum = 0.0
        self.prev_time = time.time()
        self.udp = UdpReceiver(port=UDP_LISTEN_PORT)

        # External command hold (so 20 Hz senders still drive a 100 Hz physics loop)
        self._ext_last_rx   = None
        self._ext_hold_cmd  = CmdSetpoints(0.0, 0.0, 0.0)
        self._ext_hold_secs = 0.35

        # Telemetry sender (sim -> AI)
        self.telem = TelemetrySender(port=UDP_TELEM_PORT)
        self._telem_accum = 0.0   # throttle to ~15 Hz

        # Mouse (left: start then dest; right: clear)
        self.map.canvas.bind("<Button-1>", self._on_left_click)
        self.map.canvas.bind("<Button-3>", self._on_right_click)

        # Keys
        self.bind_all("<KeyPress>", self._on_key_down)
        self.bind_all("<KeyRelease>", self._on_key_up)
        self.bind_all("<space>", lambda e: self._on_pause() if self.running else self._on_start())
        self.bind_all("<r>", lambda e: self._on_reset())
        self.bind_all("<R>", lambda e: self._on_reset())

        self.after(int(1000/FPS), self._on_gui_tick)

    # ---- small helpers --------------------------------------------------------
    def _manual_action_text(self):
        acts = []
        if 'Up' in self.keys or 'w' in self.keys:    acts.append("Speed+")
        if 'Down' in self.keys or 's' in self.keys:  acts.append("Speed−")
        if 'Left' in self.keys or 'a' in self.keys:  acts.append("Turn←")
        if 'Right'in self.keys or 'd' in self.keys:  acts.append("Turn→")
        if 'Prior'in self.keys:                      acts.append("Climb↑")   # PageUp
        if 'Next' in self.keys:                      acts.append("Climb↓")   # PageDown
        return "Manual: " + ("  ".join(acts) if acts else "…")

    def _show_manual_hint(self, txt):
        self._last_input_txt = txt
        self._last_input_ts  = time.time()
        self.input_banner.config(text=txt)

    # ---- Params from panel -----------------------------------------------------
    def _on_params_change(self, p):
        prev_mode = getattr(self, "mode", "Manual")
        prev_vehicle = getattr(self, "vehicle_type", "quad")

        self.mode         = p.get('MODE', self.mode)
        self.vehicle_type = p.get('VEHICLE', prev_vehicle)

        self.limits.V_MAX        = float(p['V_MAX'])
        self.limits.VZ_MAX       = float(p['VZ_MAX'])
        self.limits.YAW_RATE_MAX = float(p['YAW_MAX'])
        self.gains.K_YAW         = float(p['K_YAW'])
        self.gains.V_BASE        = float(p['V_BASE'])
        self.gains.K_FWD         = float(p['K_FWD'])

        self.v_cruise = float(p.get('V_CRUISE', self.v_cruise))

        cap  = float(p.get('BATTERY_WH', self.batt_cap_wh))
        frac = 1.0 if self.batt_cap_wh <= 0 else max(0.0, min(1.0, self.batt_wh / self.batt_cap_wh))
        self.batt_cap_wh = cap
        self.batt_wh     = frac * self.batt_cap_wh

        self.env.wind_e = float(p.get('W_E', self.env.wind_e))
        self.env.wind_n = float(p.get('W_N', self.env.wind_n))

        # Mode transitions — smooth handoff & ensure keys go to the map
        if self.mode == 'Manual' and prev_mode != 'Manual':
            self.man_v   = float(self.state.v_forward)
            self.man_yaw = 0.0
            self.man_vz  = 0.0
            self.map.canvas.focus_set()
            self._show_manual_hint("Manual: ↑/↓ speed, ←/→ turn, PgUp/PgDn climb")
        elif prev_mode == 'Manual' and self.mode != 'Manual':
            self._show_manual_hint("")

        # Request a layout pass (safe even early in init)
        if hasattr(self, "_layout_right_pane"):
            self.after_idle(self._layout_right_pane)

    # ---- Buttons ---------------------------------------------------------------
    def _on_start(self):
        self.landing = False
        self.landed = False
        self.land_anchor = None
        self._route_t0 = time.time() if (self.route_start and self.route_dest and self.mode == 'Route') else None
        self._goaround_time = 0.0

        self.running = True
        self.accum = 0.0
        self.prev_time = time.time()
        self.sim_time = 0.0
        self._open_log()
        self.controls.status.config(text=f"Status: running | Mode={self.mode}")

        if self.mode == 'Manual':
            self.map.canvas.focus_set()

    def _on_pause(self):
        self.running = False
        self._close_log()
        self.controls.status.config(text="Status: paused")

    def _on_reset(self):
        self.state = DroneState()
        self.landing = False
        self.landed  = False
        self.land_anchor = None
        self.accum = 0.0
        self.prev_time = time.time()
        self.batt_wh = self.batt_cap_wh
        self.state.trail.clear()
        self.sim_time = 0.0
        self._route_t0 = None
        self._goaround_time = 0.0
        self._missed_active = False
        self._landing_t0 = None
        self._touch_t0 = None
        self._close_log()
        self.controls.status.config(text="Status: reset")
        if hasattr(self, "_layout_right_pane"):
            self.after_idle(self._layout_right_pane)

    # ---- Mouse (route) ---------------------------------------------------------
    def _on_left_click(self, ev):
        xw, yw = self.map.px_to_world(ev.x, ev.y)
        if self.route_start is None:
            self.route_start = Waypoint(xw, yw)
            self.map.set_route_start(xw, yw)
        else:
            self.route_dest = Waypoint(xw, yw)
            self.map.set_route_dest(xw, yw)
            self.landing = False
            self.landed  = False
            self.land_anchor = None
            self._route_t0 = time.time()
            self._goaround_time = 0.0

    def _on_right_click(self, _ev):
        self.route_start = None
        self.route_dest  = None
        self.landing = False
        self.landed  = False
        self.land_anchor = None
        self._route_t0 = None
        self._goaround_time = 0.0
        self.map.clear_route()

    # ---- Manual keys -----------------------------------------------------------
    def _on_key_down(self, ev):
        self.keys.add(ev.keysym)
        if self.mode == 'Manual':
            self._show_manual_hint(self._manual_action_text())
            return "break"

    def _on_key_up(self, ev):
        self.keys.discard(ev.keysym)
        if self.mode == 'Manual':
            self._show_manual_hint(self._manual_action_text())
            return "break"

    def _manual_cmd(self) -> CmdSetpoints:
        dv, dyaw, dvz = 0.5, 20.0, 0.5
        if 'Up' in self.keys or 'w' in self.keys:    self.man_v = min(self.man_v + dv, self.limits.V_MAX)
        if 'Down' in self.keys or 's' in self.keys:  self.man_v = max(self.man_v - dv, 0.0)
        if 'Left' in self.keys or 'a' in self.keys:  self.man_yaw = -dyaw
        elif 'Right' in self.keys or 'd' in self.keys: self.man_yaw = +dyaw
        else: self.man_yaw = 0.0
        if 'Prior' in self.keys: self.man_vz = min(self.man_vz + dvz,  self.limits.VZ_MAX)
        elif 'Next' in self.keys: self.man_vz = max(self.man_vz - dvz, -self.limits.VZ_MAX)
        else: self.man_vz = 0.0
        return CmdSetpoints(yaw_rate_dps=self.man_yaw, v_forward=self.man_v, vz=self.man_vz)

    # ---- Logging ---------------------------------------------------------------
    def _open_log(self):
        os.makedirs("logs", exist_ok=True)
        ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join("logs", f"flight_{ts}.csv")
        self.log_file = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["t","x","y","z","yaw_deg","v_fwd","vz","wind_e","wind_n"])
        self.controls.status.config(text=f"Status: running | Log: {path}")

    def _close_log(self):
        if self.log_file:
            try:
                self.log_file.flush(); self.log_file.close()
            except Exception:
                pass
        self.log_file = None
        self.log_writer = None

    def _log_step(self):
        if self.log_writer:
            self.log_writer.writerow([
                f"{self.sim_time:.3f}", f"{self.state.x:.3f}", f"{self.state.y:.3f}", f"{self.state.z:.3f}",
                f"{self.state.yaw:.3f}", f"{self.state.v_forward:.3f}", f"{self.state.vz:.3f}",
                f"{self.env.wind_e:.3f}", f"{self.env.wind_n:.3f}",
            ])

    def _auto_stop_if_arrived(self):
        if self.mode != 'Route' or not self.route_dest:
            return
        if self.vehicle_type == 'fixed':
            return
        dx = self.route_dest.x - self.state.x
        dy = self.route_dest.y - self.state.y
        dist_xy = (dx*dx + dy*dy) ** 0.5
        tol_xy = 3.0
        tol_z  = 0.25
        if dist_xy <= tol_xy and self.state.z <= tol_z:
            self.state.v_forward = 0.0
            self.state.vz = 0.0
            self.landing = False
            self.landed  = True
            self.running = False

    # ---- Main loop -------------------------------------------------------------
    def _set_led(self, color: str):
        now = time.time()
        if self._led_color != color and (now - self._led_last_change) >= self._led_min_dwell:
            self._led_color = color
            self._led_last_change = now
            self.controls.set_led(color)

    # --- VEHICLE OVERLAY helpers -----------------------------------------------
    def _scan_vehicle_icon_dirs(self):
        """Map exact folder -> top image. Also cache the set of valid IDs."""
        if self._veh_dir_index is not None:
            return
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "vehicles"))
        idx = []
        if os.path.isdir(root):
            for name in os.listdir(root):
                p = os.path.join(root, name)
                if not os.path.isdir(p):
                    continue
                for fn in os.listdir(p):
                    low = fn.lower()
                    if low in ("top.jpg", "top.jpeg", "top.png"):
                        idx.append((name.lower(), os.path.join(p, fn)))
                        break
        self._veh_dir_index = sorted(idx, key=lambda t: t[0])       # e.g. [('citybus01', ...), ('pickup01', ...), ('redtaxi01', ...)]
        self._veh_valid_ids = {n for (n, _path) in self._veh_dir_index}



    def _veh_id_from_packet(self) -> str:
        v = self._veh_last if isinstance(self._veh_last, dict) else {}
        for k in ("id", "veh_id", "folder", "model", "name", "type", "class"):
            if k in v:
                norm = self._normalize_veh_id(v.get(k))
                if norm:
                    return norm
        # fallback if nothing present
        if self._veh_forced_id:
            return self._normalize_veh_id(self._veh_forced_id)
        return ""


    def _veh_type_from_packet(self) -> str:
        v = self._veh_last if isinstance(self._veh_last, dict) else {}
        vid = str(v.get("id", "")).strip().lower()
        typ = str(v.get("type", "")).strip().lower()
        if vid.startswith("citybus"): return "bus"
        if vid.startswith("pickup"):  return "pickup"
        if vid.startswith("redtaxi"): return "sedan"
        if typ in ("bus","pickup","sedan"): return typ
        return "sedan"

    def _find_vehicle_image_path(self) -> Optional[str]:
        """Return top.* path for the exact folder id (no fuzzy matching here)."""
        if Image is None:
            return None
        self._scan_vehicle_icon_dirs()
        idx = dict(self._veh_dir_index or [])
        vid = self._veh_id_from_packet()   # already normalized to exact folder or ''
        return idx.get(vid, None)


    def _get_cam_dims_fov(self):
        try:
            cam_w = getattr(self.video, "last_w", 0) or int(self.video.canvas.winfo_width())
            cam_h = getattr(self.video, "last_h", 0) or int(self.video.canvas.winfo_height())
        except Exception:
            cam_w, cam_h = 1280, 720
        hfov_deg = float(getattr(self.video, "fov_deg", 72.0))
        return cam_w, cam_h, hfov_deg

    def _extract_norm_offsets(self, veh: Optional[Dict[str, Any]]):
        try:
            v = veh or {}
            if "cx01" in v or "cy01" in v:
                cx01 = float(v.get("cx01", 0.5)); cy01 = float(v.get("cy01", 0.5))
            else:
                cx01 = float(v.get("cx_n", 0.5));  cy01 = float(v.get("cy_n", 0.5))
            cx01 = 0.0 if cx01 < 0.0 else 1.0 if cx01 > 1.0 else cx01
            cy01 = 0.0 if cy01 < 0.0 else 1.0 if cy01 > 1.0 else cy01
            return cx01, cy01
        except Exception:
            return None

    def _veh_world_from_packet(self, veh: Optional[dict], look_ahead_m: float = 30.0):
        offs = self._extract_norm_offsets(veh)
        if offs is None:
            cx01, cy01 = 0.5, 0.5
        else:
            cx01, cy01 = offs
        cam_w, cam_h, hfov_deg = self._get_cam_dims_fov()
        hfov = math.radians(hfov_deg)
        dxn = (cx01 - 0.5) * 2.0
        dyn = (cy01 - 0.5) * 2.0
        ang_x = dxn * (hfov * 0.5)
        D = look_ahead_m * (1.0 - 0.40 * dyn)
        D = max(8.0, min(60.0, D))
        lateral = math.tan(ang_x) * D
        forward = D
        yaw = math.radians(self.state.yaw)
        fx, fy = math.cos(yaw), math.sin(yaw)
        rx, ry = -math.sin(yaw), math.cos(yaw)
        vx = self.state.x + fx * forward + rx * lateral
        vy = self.state.y + fy * forward + ry * lateral
        return vx, vy

    def _get_vehicle_icon(self, yaw_deg: float, size_px: int) -> Optional["ImageTk.PhotoImage"]:
        if Image is None:
            return None
        path = self._find_vehicle_image_path()
        if not path:
            return None
        bucket = int((yaw_deg % 360.0) / 15.0)
        key = (path, bucket, size_px)
        icon = self._veh_icon_cache.get(key)
        if icon is not None:
            return icon
        base = self._veh_base_images.get(path)
        if base is None:
            try:
                base = Image.open(path).convert("RGBA")
            except Exception:
                base = None
            self._veh_base_images[path] = base
        if base is None:
            return None
        try:
            img = base.resize((size_px, size_px), Image.LANCZOS)
            angle = -bucket * 15.0
            img = img.rotate(angle, expand=True, resample=Image.BICUBIC)
            icon = ImageTk.PhotoImage(img)
            self._veh_icon_cache[key] = icon
            return icon
        except Exception:
            return None

    def _draw_vehicle_overlay(self):
        try:
            c = self.map.canvas
        except Exception:
            return
        c.delete("veh_overlay")
        if not bool(self.veh_overlay.get()):
            return

        t_last = max(self._veh_last_ts or 0.0, self._ext_last_rx or 0.0)
        if (time.time() - t_last) > 0.8:
            return

        veh = self._veh_last if isinstance(self._veh_last, dict) else None
        vx, vy = self._veh_world_from_packet(veh)

        if isinstance(self._veh_last, dict) and (self._veh_seq_drawn != self._veh_seq):
            self._veh_trail.append((vx, vy))
            self._veh_seq_drawn = self._veh_seq

        cw = max(2, c.winfo_width())
        ch = max(2, c.winfo_height())
        try:
            mpp = float(METERS_PER_PIXEL)
        except Exception:
            mpp = 1.0
        px = cw * 0.5 + (vx - self.state.x) / mpp
        py = ch * 0.5 + (vy - self.state.y) / mpp

        vtype = self._veh_type_from_packet() if isinstance(self._veh_last, dict) else "sedan"
        color = "#ff8a3d"
        if vtype == "sedan": color = "#ff5555"
        elif vtype == "bus": color = "#ffd34d"
        elif vtype in ("truck","pickup"): color = "#ffb74d"

        c.create_line(cw*0.5, ch*0.5, px, py, fill=color, width=2, dash=(4,3), tags="veh_overlay")

        size_px = self._veh_size_px.get(vtype, 22)
        icon = self._get_vehicle_icon(self.state.yaw, size_px)
        if icon is not None:
            c.create_image(px, py, image=icon, anchor="center", tags="veh_overlay")
        else:
            r = 6
            c.create_oval(px - r, py - r, px + r, py + r, fill=color, outline="", tags="veh_overlay")

        if len(self._veh_trail) >= 2:
            pts = []
            for (wx, wy) in self._veh_trail:
                tx = cw * 0.5 + (wx - self.state.x) / mpp
                ty = ch * 0.5 + (wy - self.state.y) / mpp
                pts.extend([tx, ty])
            try:
                c.create_line(*pts, fill=color, width=2, tags="veh_overlay")
            except Exception:
                pass
    # ---------------------------------------------------------------------------

    def _on_gui_tick(self):
        now = time.time()
        dt_wall = now - self.prev_time
        self.prev_time = now

        # physics integration
        if self.running:
            self.accum = min(self.accum + dt_wall, 0.25)
            steps = 0
            while self.accum >= DT and steps < 8:
                cmd = self._get_command()
                P = self.P0 + self.kv3 * (self.state.v_forward ** 3) + self.kclimb * abs(self.state.vz)
                self.batt_wh = max(0.0, self.batt_wh - (P * DT) / 3600.0)
                if self.batt_wh <= 0.0:
                    cmd = CmdSetpoints(0.0, 0.0, 0.0)
                physics_step(
                    DT, self.state, self.env, cmd,
                    v_max=self.limits.V_MAX,
                    vz_max=self.limits.VZ_MAX,
                    yaw_rate_max_dps=self.limits.YAW_RATE_MAX,
                    a_max=self.limits.A_MAX,
                    bank_max_deg=self.limits.BANK_MAX,
                    g=GRAVITY,
                )
                self.sim_time += DT
                self._log_step()
                self.accum -= DT
                steps += 1
                self._auto_stop_if_arrived()

        # --- Telemetry out (sim -> AI), ~15 Hz -----------------------------------
        self._telem_accum += dt_wall
        if self._telem_accum >= (1.0 / 15.0):
            self._telem_accum = 0.0
            try:
                cam_w = getattr(self.video, "last_w", 0) or int(self.video.canvas.winfo_width())
                cam_h = getattr(self.video, "last_h", 0) or int(self.video.canvas.winfo_height())
            except Exception:
                cam_w, cam_h = 1280, 720
            cam_fov = getattr(self.video, "fov_deg", 72.0)
            telem = {
                "t": self.sim_time,
                "mode": self.mode,
                "vehicle": self.vehicle_type,
                "wind": {"e": self.env.wind_e, "n": self.env.wind_n},
                "drone": {
                    "x": self.state.x, "y": self.state.y, "z": self.state.z,
                    "yaw": self.state.yaw, "v_air": self.state.v_forward, "vz": self.state.vz
                },
                "cam": {"w": cam_w, "h": cam_h, "fov_deg": cam_fov},
                "targets": []
            }
            self.telem.send(telem)

        # clamp slight negatives (AGL)
        if self.state.z < 0.0:
            self.state.z = 0.0
            if self.state.vz < 0.0: self.state.vz = 0.0

        # finish landing (touchdown logic)
        if self.landing and self.land_anchor:
            ax, ay = self.land_anchor
            d_xy = math.hypot(self.state.x - ax, self.state.y - ay)
            tol_xy = 3.0 if self.vehicle_type == 'quad' else 12.0
            on_ground = (self.state.z <= 0.05)
            if on_ground and d_xy <= tol_xy:
                if self._touch_t0 is None:
                    self._touch_t0 = time.time()
                if time.time() - self._touch_t0 >= 0.35:
                    self.state.z = 0.0
                    self.state.vz = 0.0
                    self.state.v_forward = 0.0
                    self.running = False
                    self.landed = True
                    self.landing = False
            else:
                self._touch_t0 = None
            if d_xy > 80.0 and self.state.vz >= -0.05:
                self.landing = False

        # LED (debounced)
        loitering = bool(getattr(self, "_loitering", False)) or bool(getattr(self, "_goaround", False))
        descending = self.landing and (self.state.vz < -0.05) and self.running
        moving = self.running and not self.landed and (abs(self.state.v_forward) > 0.2 or abs(self.state.vz) > 0.05)

        if self.landed:
            self._set_led('#3c3')
        elif descending:
            self._set_led('#5ad')
        elif loitering and not self.landing:
            self._set_led('#cc8')
        elif moving:
            self._set_led('#d33')
        else:
            self._set_led('#777')

        # draw
        self.map.set_camera(self.state.x, self.state.y)

        # ALWAYS draw wind meter (top-right, offset below HUD bar)
        if hasattr(self.map, "draw_wind_overlay"):
            try:
                self.map.draw_wind_overlay(self.env.wind_e, self.env.wind_n, anchor='tr', yoff=60)
            except Exception:
                pass

        self.map.draw_drone(
            self.state.x, self.state.y, self.state.yaw,
            trail=self.state.trail,
            vehicle_type=self.vehicle_type,
            show_fov=False,
            wind=(self.env.wind_e, self.env.wind_n),
            wind_anchor='tr',
            wind_yoff=60
        )

        # --- VEHICLE OVERLAY ----------------------------------------------------
        self._draw_vehicle_overlay()

        # Manual banner auto-update/fade
        if self.mode == 'Manual':
            txt = self._manual_action_text()
            if self.keys:
                self._show_manual_hint(txt)
            elif time.time() - self._last_input_ts > 1.2:
                self._show_manual_hint("Manual: …")
        else:
            if self._last_input_txt:
                self._show_manual_hint("")

        # update External info panel text
        if self.mode == 'External':
            now = time.time()
            if self._ext_last_rx is None:
                txt = "External: waiting…"
            else:
                age = now - self._ext_last_rx
                veh_ok = isinstance(self._veh_last, dict) and (now - self._veh_last_ts) < VEH_SEEN_HOLD_S
                if veh_ok or self._veh_forced_id:
                    vid = self._veh_id_from_packet() or "?"
                    img_ok = (Image is not None) and (self._find_vehicle_image_path() is not None)
                    forced = "" if veh_ok else " (forced)"
                    veh_str = f"veh={vid}{forced} img={'Y' if img_ok else 'N'}"
                else:
                    keys_preview = ",".join(self._last_rx_keys[:6])
                    veh_str = f"veh=— keys=[{keys_preview}]"



                if age < self._ext_hold_secs:
                    m = self._ext_last_msg or {}
                    c = self._ext_last_cmd or CmdSetpoints(0.0, 0.0, 0.0)
                    freshness = "fresh" if age < 0.3 else f"held {age:.1f}s"
                    txt = (
                        f"UDP {self._ext_rx_hz:4.1f} Hz ({freshness}; {veh_str}) | "
                        f"dx={m.get('dx',0):+.0f}px  dy={m.get('dy',0):+.0f}px  dz={m.get('dz',0):+.2f} m/s\n"
                        f"→ yaw={c.yaw_rate_dps:+.1f}°/s  v={c.v_forward:.2f} m/s  vz={c.vz:+.2f} m/s"
                    )
                else:
                    txt = f"External: no packets for {age:.1f} s ({veh_str})"
            self.ext_lbl.config(text=txt)
        else:
            self.ext_frame.place_forget()

        # HUD --------------------------------------------------------------------
        ve = math.cos(math.radians(self.state.yaw)) * self.state.v_forward + self.env.wind_e
        vn = math.sin(math.radians(self.state.yaw)) * self.state.v_forward + self.env.wind_n
        GS  = math.hypot(ve, vn)
        Air = self.state.v_forward

        dist_m = 0.0
        if self.route_dest:
            dist_m = math.hypot(self.route_dest.x - self.state.x, self.route_dest.y - self.state.y)
        eta_txt = ""
        if GS > 0.3 and dist_m > 0.1:
            eta_s = dist_m / GS
            eta_txt = f" | Dist={dist_m:5.0f} m | ETA {int(eta_s//60):02d}:{int(eta_s%60):02d}"

        lat, lon = enu_to_latlon(self.state.x, self.state.y)
        Pnow = self.P0 + self.kv3 * (self.state.v_forward ** 3) + self.kclimb * abs(self.state.vz)
        t_left_s = (self.batt_wh / Pnow) * 3600.0 if Pnow > 1.0 else 0.0
        batt_pct = 100.0 * (self.batt_wh / max(1e-6, self.batt_cap_wh))
        alt_agl = max(0.0, self.state.z)

        badges = []
        if self.landing: badges.append("LANDING")
        if loitering and self.vehicle_type == 'fixed': badges.append("LOITER")
        if bool(getattr(self, "_wind_limited", False)): badges.append("WIND-LIMITED")
        if bool(getattr(self, "_hold_headwind", False)): badges.append("HOLD: Headwind > V_MAX")
        if bool(getattr(self, "_goaround", False)): badges.append("GO-AROUND")
        if self.landed: badges.append("LANDED")

        hud1 = (f"Mode={self.mode} | Veh={self.vehicle_type} | GS={GS:4.1f} m/s "
                f"(Air={Air:4.1f}){eta_txt}")
        hud2 = (f"Alt={alt_agl:5.1f} m | Climb={self.state.vz:4.1f} m/s | "
                f"Yaw={self.state.yaw:6.1f}° | Wind=({self.env.wind_e:.1f},{self.env.wind_n:.1f}) m/s | "
                f"GPS=({lat:.6f},{lon:.6f}) | Batt={batt_pct:5.1f}% ({self.batt_wh:4.0f}Wh) "
                f"P≈{Pnow:4.0f}W T_left={int(t_left_s//60):02d}:{int(t_left_s%60):02d}")
        if badges:
            hud2 += " | " + " · ".join(badges)
        self.hud.config(text=hud1 + "\n" + hud2)

        if self.mode == 'Route':
            if not (self.route_start and self.route_dest):
                self.controls.status.config(text='Status: running | Route ON (set start & dest)')
            elif self.landing and not self.landed:
                self.controls.status.config(text='Status: Route ON | LANDING to destination')
            elif loitering and self.vehicle_type == 'fixed':
                self.controls.status.config(text='Status: Route ON | LOITERING at destination')
            elif not self.running and self.landed:
                self.controls.status.config(text='Status: Route ON | LANDED')
            elif not self.running:
                self.controls.status.config(text='Status: paused | Route ON')
            else:
                self.controls.status.config(text='Status: running | Route ON')

        self.after(int(1000 / FPS), self._on_gui_tick)

    # ---- Command source --------------------------------------------------------
    def _direct_to_final(self, dest_x, dest_y, v_g, k_heading=3.0):
        dx = dest_x - self.state.x
        dy = dest_y - self.state.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return CmdSetpoints(0.0, 0.0, 0.0), True
        tx, ty = dx / dist, dy / dist
        we, wn = self.env.wind_e, self.env.wind_n
        air_e = tx * v_g - we
        air_n = ty * v_g - wn
        v_req = math.hypot(air_e, air_n)
        v_cmd = max(12.0 if self.vehicle_type == 'fixed' else 0.0, min(v_req, self.limits.V_MAX))
        hdg = math.degrees(math.atan2(air_n, air_e))
        yaw_err = ((hdg - self.state.yaw + 540) % 360) - 180
        yaw_rate = k_heading * yaw_err
        align_ok = abs(yaw_err) <= 35.0
        capture = (dist <= 45.0) and align_ok
        return CmdSetpoints(yaw_rate_dps=yaw_rate, v_forward=v_cmd, vz=0.0), capture

    def _normalize_veh_id(self, raw) -> str:
        """Return an exact folder id in assets/vehicles or '' if not resolvable."""
        self._scan_vehicle_icon_dirs()
        valid = getattr(self, "_veh_valid_ids", set())
        if raw is None:
            return ""

        # numeric index → alphabetical order of existing folders
        if isinstance(raw, (int, float)) and valid:
            lst = sorted(valid)  # alphabetical
            i = int(raw)
            return lst[i] if 0 <= i < len(lst) else ""

        s = str(raw).strip().lower()
        if not s:
            return ""

        # strict aliases → your folders only
        aliases = {
            "bus": "citybus01",
            "citybus": "citybus01",
            "pickup": "pickup01",
            "truck": "pickup01",
            "taxi": "redtaxi01",
            "car": "redtaxi01",
            "sedan": "redtaxi01",
        }
        s = aliases.get(s, s)

        return s if s in valid else ""


    def _extract_veh_from_msg(self, m: dict) -> Optional[dict]:
        """Be permissive: accept veh in many shapes (root, targets[0], flat fields)."""
        # direct dict or string under common keys
        for k in ("veh", "vehicle", "car", "target"):
            if k in m:
                v = m[k]
                if isinstance(v, dict):
                    return v
                if isinstance(v, str) and v.strip():
                    return {"id": self._normalize_veh_id(v)}
        # list of targets
        t = m.get("targets")
        if isinstance(t, list) and t:
            first = t[0]
            if isinstance(first, dict):
                if "veh" in first and isinstance(first["veh"], dict):
                    return first["veh"]
                # id/type placed directly on the element
                if any(k in first for k in ("id", "veh_id", "folder", "model", "name", "type", "class")):
                    return first
        # flat fields on root
        cand = {}
        for k_src, k_dst in (
            ('veh_id','id'), ('vehicle_id','id'), ('car_id','id'),
            ('id','id'), ('model','id'), ('folder','id'), ('name','id'),
            ('veh_type','type'), ('class','type'), ('type','type'), ('label','type'),
            ('cx01','cx01'), ('cy01','cy01'), ('cx_n','cx01'), ('cy_n','cy01'),
            ('u','cx'), ('v','cy'), ('cx','cx'), ('cy','cy'), ('px','cx'), ('py','cy'),
        ):
            if k_src in m: cand[k_dst] = m[k_src]
        # normalize id if present
        if 'id' in cand:
            cand['id'] = self._normalize_veh_id(cand['id'])
        return cand or None


    def _get_command(self) -> CmdSetpoints:
        if self.landed:
            return CmdSetpoints(0.0, 0.0, 0.0)

        if not hasattr(self, "_route_start_alt"): self._route_start_alt = max(20.0, float(self.state.z))
        if not hasattr(self, "_landing_t0"):      self._landing_t0 = None

        # -------- Manual --------
        if self.mode == 'Manual':
            return self._manual_cmd()

        # -------- External (UDP AI) --------
        if self.mode == 'External':
            now = time.time()
            msgs = self.udp.poll()
            if msgs:
                m  = msgs[-1]
                dx = float(m.get('dx', 0.0))
                dy = float(m.get('dy', 0.0))
                dz = float(m.get('dz_rate', 0.0))

                # remember top-level keys for banner debugging
                try:
                    self._last_rx_keys = list(m.keys())
                except Exception:
                    self._last_rx_keys = []

                # vehicle overlay capture (very permissive)
                veh = self._extract_veh_from_msg(m)
                if isinstance(veh, dict):
                    # normalize names used by the overlay
                    if "cx01" not in veh and "cx_n" in veh: veh["cx01"] = veh.get("cx_n")
                    if "cy01" not in veh and "cy_n" in veh: veh["cy01"] = veh.get("cy_n")
                    if "type" not in veh and "class" in veh: veh["type"] = veh.get("class")
                    if "id" in veh: veh["id"] = self._normalize_veh_id(veh.get("id"))

                    self._veh_last = veh
                    self._veh_last_ts = now
                    self._veh_seq += 1


                # Accept many shapes for vehicle metadata
                veh = (m.get('veh') or m.get('car') or m.get('vehicle') or m.get('target'))
                if isinstance(veh, str) and veh.strip():
                    veh = {"id": veh.strip()}
                if isinstance(veh, dict):
                    if "cx01" not in veh and "cx_n" in veh: veh["cx01"] = veh.get("cx_n")
                    if "cy01" not in veh and "cy_n" in veh: veh["cy01"] = veh.get("cy_n")
                    if "type" not in veh and "class" in veh: veh["type"] = veh.get("class")
                    self._veh_last = veh
                    self._veh_last_ts = now
                    self._veh_seq += 1
                else:
                    # build from flat fields if present (also accept cx_n/cy_n here)
                    cand = {}
                    for k_src, k_dst in (
                        ('veh_id','id'), ('car_id','id'), ('id','id'), ('model','id'), ('folder','id'),
                        ('veh_type','type'), ('class','type'), ('type','type'), ('name','type'),
                        ('cx01','cx01'), ('cy01','cy01'),
                        ('cx_n','cx01'), ('cy_n','cy01'),
                        ('u','u'), ('v','v'),
                        ('cx','cx'), ('cy','cy'), ('px','cx'), ('py','cy'),
                        ('x','x'), ('y','y'),
                    ):
                        if k_src in m: cand[k_dst] = m[k_src]
                    if cand:
                        self._veh_last = cand
                        self._veh_last_ts = now
                        self._veh_seq += 1

                cmd = map_tracking_to_setpoints(
                    dx, dy, dz,
                    k_yaw_dps=self.gains.K_YAW, k_fwd=self.gains.K_FWD,
                    v_base=self.gains.V_BASE, v_max=self.limits.V_MAX, vz_max=self.limits.VZ_MAX
                )
                cmd = CmdSetpoints(
                    yaw_rate_dps=max(-self.limits.YAW_RATE_MAX, min(self.limits.YAW_RATE_MAX, cmd.yaw_rate_dps)),
                    v_forward   =max(0.0, min(self.limits.V_MAX, cmd.v_forward)),
                    vz          =max(-self.limits.VZ_MAX, min(self.limits.VZ_MAX, cmd.vz)),
                )

                if self._ext_last_rx is not None:
                    dt_ = max(1e-3, now - self._ext_last_rx)
                    self._ext_rx_hz = 0.8*self._ext_rx_hz + 0.2*(1.0/dt_)
                self._ext_last_rx  = now
                self._ext_last_msg = {"dx": dx, "dy": dy, "dz": dz}
                self._ext_last_cmd = cmd
                self._ext_hold_cmd = cmd
                return cmd

            if (self._ext_last_rx is not None) and ((now - self._ext_last_rx) < self._ext_hold_secs):
                return self._ext_hold_cmd
            return CmdSetpoints(0.0, 0.0, 0.0)

        # -------- Route --------
        if self.mode == 'Route':
            if not (self.route_start and self.route_dest):
                return CmdSetpoints(0.0, 0.0, 0.0)

            now = time.time()
            if self._route_t0 and (now - self._route_t0) < 0.25:
                self._route_start_alt = max(20.0, float(self.state.z))
                self._missed_active = False
                self._landing_t0 = None

            yaw_rate_cmd, v_cmd, capture_ok, wind_limited, loitering, hold_headwind, goaround = route_guidance_with_wind(
                self.vehicle_type,
                self.state.x, self.state.y, self.state.yaw, self.state.v_forward,
                self.env.wind_e, self.env.wind_n,
                self.route_start, self.route_dest,
                v_cruise=self.v_cruise, v_max=self.limits.V_MAX,
                bank_max_deg=self.limits.BANK_MAX,
                stop_radius=5.0,
                final_leg=250.0, faf_radius=80.0,
                lookahead=90.0, align_deg=25.0,
                descend_radius=40.0, v_min_fixed=12.0
            )
            self._wind_limited  = wind_limited
            self._loitering     = loitering
            self._hold_headwind = hold_headwind
            self._goaround      = goaround

            p2x, p2y = self.route_dest.x, self.route_dest.y
            fx = p2x - self.route_start.x
            fy = p2y - self.route_start.y
            fn = math.hypot(fx, fy) or 1.0
            fx, fy = fx / fn, fy / fn
            FINAL = 250.0
            p1x, p1y = p2x - fx * FINAL, p2y - fy * FINAL
            relx, rely = self.state.x - p1x, self.state.y - p1y
            s_along = relx * fx + rely * fy               # from FAF (0..FINAL)
            s_rem   = FINAL - s_along                      # >0 before DEST, <0 after
            cte     = (-fy) * relx + fx * rely            # cross-track

            yawr = math.radians(self.state.yaw)
            ve = math.cos(yawr) * self.state.v_forward + self.env.wind_e
            vn = math.sin(yawr) * self.state.v_forward + self.env.wind_n
            v_along = ve * fx + vn * fy

            if self.vehicle_type == 'fixed' and self.landing and self.land_anchor:
                ax, ay = self.land_anchor
                if (math.hypot(self.state.x - ax, self.state.y - ay) <= 12.0) and (self.state.z <= 0.12):
                    self.state.z = 0.0
                    self.state.vz = 0.0
                    self.state.v_forward = 0.0
                    self.landing = False
                    self.landed = True
                    self.running = False
                    return CmdSetpoints(0.0, 0.0, 0.0)

            if self.vehicle_type == 'fixed' and self._missed_active:
                ahead_x = p2x + fx * 220.0
                ahead_y = p2y + fy * 220.0
                cmd_dir, _ = self._direct_to_final(ahead_x, ahead_y, max(self.v_cruise, 14.0), k_heading=3.0)
                v_fw = max(12.0, min(self.limits.V_MAX, cmd_dir.v_forward))
                alt_target = max(20.0, float(self._route_start_alt))
                climb = 0.0 if self.state.z >= alt_target - 0.2 else min(self.limits.VZ_MAX, 1.8)
                far_ahead = s_along > (FINAL + 0.6 * 220.0)
                if self.state.z >= alt_target - 0.2 and far_ahead:
                    self._missed_active = False
                    self._landing_t0 = None
                return CmdSetpoints(yaw_rate_dps=cmd_dir.yaw_rate_dps, v_forward=v_fw, vz=climb)

            if not self.landing and capture_ok:
                self.landing = True
                self.landed = False
                self.land_anchor = (p2x, p2y)
                self._landing_t0 = now

            if self.vehicle_type == 'fixed' and not self.landing:
                GS_DEG, FLR_D, FLR_H = 5.0, 12.0, 0.7
                if s_rem > FLR_D:
                    z_ref = math.tan(math.radians(GS_DEG)) * (s_rem - FLR_D) + FLR_H
                else:
                    z_ref = (FLR_H / FLR_D) * max(0.0, s_rem)
                vff = -abs(v_along) * math.tan(math.radians(GS_DEG))
                err = self.state.z - z_ref
                vz_pre = vff - 0.9 * err
                vz_pre = max(-self.limits.VZ_MAX, min(+0.9, vz_pre))
                if s_rem <= -5.0 and self.state.z > 1.0:
                    self._missed_active = True
                    self.landing = False
                    return CmdSetpoints(0.0, max(12.0, v_cmd), min(self.limits.VZ_MAX, 1.8))
                if abs(cte) <= 30.0 and -20.0 <= s_along <= FINAL + 10.0:
                    return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd, v_forward=v_cmd, vz=vz_pre)
                else:
                    return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd, v_forward=v_cmd, vz=0.0)

            if self.landing and self.land_anchor:
                ax, ay = self.land_anchor
                dx = ax - self.state.x
                dy = ay - self.state.y
                we, wn = self.env.wind_e, self.env.wind_n

                if self.vehicle_type == 'quad':
                    dist = math.hypot(dx, dy)
                    vz_land = -min(self.limits.VZ_MAX, 2.0)
                    eps = min(2.0, dist / 5.0)
                    if dist > 0.05:
                        track = math.atan2(dy, dx)
                        air_e = eps * math.cos(track) - we
                        air_n = eps * math.sin(track) - wn
                    else:
                        air_e, air_n = -we, -wn
                    v_cmd2 = min(self.limits.V_MAX, (air_e**2 + air_n**2) ** 0.5)
                    hdg = math.degrees(math.atan2(air_n, air_e)) if v_cmd2 > 0.05 else self.state.yaw
                    yaw_err = ((hdg - self.state.yaw + 540) % 360) - 180
                    return CmdSetpoints(yaw_rate_dps=3.0 * yaw_err, v_forward=v_cmd2, vz=vz_land)

                relx, rely = self.state.x - p1x, self.state.y - p1y
                s_along = relx * fx + rely * fy
                s_rem   = FINAL - s_along

                if s_rem <= -5.0 and self.state.z > 1.0:
                    self._missed_active = True
                    self.landing = False
                    return CmdSetpoints(0.0, max(12.0, v_cmd), min(self.limits.VZ_MAX, 1.8))

                v_g = max(8.0, min(self.v_cruise, 0.30 * max(0.0, s_rem) + 7.0))
                air_e = fx * v_g - we
                air_n = fy * v_g - wn
                v_req = (air_e**2 + air_n**2) ** 0.5

                if   s_rem < 4.0:  v_floor = 4.0
                elif s_rem < 8.0:  v_floor = 7.0
                elif s_rem < 15.0: v_floor = 10.0
                else:              v_floor = 12.0
                v_cmd2 = max(v_floor, min(self.limits.V_MAX, v_req))

                hdg = math.degrees(math.atan2(air_n, air_e))
                yaw_err = ((hdg - self.state.yaw + 540) % 360) - 180
                yaw_rate_cmd2 = 2.5 * yaw_err

                GS_DEG, FLR_D, FLR_H = 5.0, 12.0, 0.7
                if s_rem > FLR_D:
                    z_ref = math.tan(math.radians(GS_DEG)) * (s_rem - FLR_D) + FLR_H
                else:
                    z_ref = (FLR_H / FLR_D) * max(0.0, s_rem)
                vff = -abs(v_along) * math.tan(math.radians(GS_DEG))
                err = self.state.z - z_ref

                COMMIT_S, COMMIT_Z = 8.0, 1.0
                allow_climb = (s_rem > COMMIT_S) or (self.state.z > COMMIT_Z)
                climb_cap = 0.8 if allow_climb else 0.0

                vz_cmd = vff - 0.9 * err
                vz_cmd = max(-self.limits.VZ_MAX, min(climb_cap, vz_cmd))
                if s_rem < 3.0 and vz_cmd > -0.3:
                    vz_cmd = -0.3

                return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd2, v_forward=v_cmd2, vz=vz_cmd)

            return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd, v_forward=v_cmd, vz=0.0)

        # default
        return CmdSetpoints(0.0, 0.0, 0.0)
