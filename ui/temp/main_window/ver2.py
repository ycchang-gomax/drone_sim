# ui/main_window.py — Mission sim with Manual banner, stable LED, Route AP, landing, logging + window icon
import tkinter as tk
import time, math, os, csv, datetime as dt

from config import WINDOW_W, WINDOW_H, MAP_IMAGE, METERS_PER_PIXEL, DT, FPS
from config import Limits, Gains, WindDefault, UDP_LISTEN_PORT, GRAVITY
from core.state import DroneState, EnvState, CmdSetpoints, LinkStatus
from core.physics import step as physics_step
from core.guidance import map_tracking_to_setpoints
from core.nav import Waypoint, route_guidance_with_wind
from simio.udp_rx import UdpReceiver
from ui.map_view import MapView
from ui.controls import ControlsPanel

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
        self.title("GoMax Drone Path Simulator")
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.configure(bg='#1b1b1b')

        # ----- Window icon (logo.png placed next to app.py) -----
        try:
            # For Windows taskbar/titlebar consistency
            try:
                import ctypes
                ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
                    u"gomax.drone_sim"
                )
            except Exception:
                pass

            # Resolve project root (folder that contains app.py)
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            logo_path = os.path.join(project_root, "logo.png")
            if os.path.exists(logo_path):
                self._icon_img = tk.PhotoImage(file=logo_path)  # keep a ref!
                self.iconphoto(True, self._icon_img)
        except Exception as e:
            print("Icon set error:", e)

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
        self.land_anchor = None       # (x,y) where we touch down

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

        # ---- Layout ------------------------------------------------------------
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.rowconfigure(0, weight=1)

        self.map = MapView(self, MAP_IMAGE, METERS_PER_PIXEL)
        self.map.grid(row=0, column=0, sticky='nsew')

        self.controls = ControlsPanel(self, self._on_start, self._on_pause, self._on_reset, self._on_params_change)
        self.controls.grid(row=0, column=1, sticky='nsew')

        # HUD
        self.hud = tk.Label(self.map.canvas, text="", fg='#ffffff', bg='#000000', padx=6, pady=3)
        self.hud.place(x=10, y=10)

        # Manual input banner (right under HUD)
        self.input_banner = tk.Label(
            self.map.canvas, text="", fg='#ffe28a', bg='#000000',
            padx=6, pady=3, font=("Segoe UI", 10, "bold")
        )
        self.input_banner.place(x=10, y=86)
        self._last_input_ts = 0.0
        self._last_input_txt = ""

        # Timers / IO
        self.running = False
        self.accum = 0.0
        self.prev_time = time.time()
        self.udp = UdpReceiver(port=UDP_LISTEN_PORT)

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
            self.map.canvas.focus_set()  # keys -> canvas, not sliders
            self._show_manual_hint("Manual: ↑/↓ speed, ←/→ turn, PgUp/PgDn climb")
        elif prev_mode == 'Manual' and self.mode != 'Manual':
            self._show_manual_hint("")

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
        self._close_log()
        self.controls.status.config(text="Status: reset")

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
            return "break"  # stop focus wiggle on sliders

    def _on_key_up(self, ev):
        self.keys.discard(ev.keysym)
        if self.mode == 'Manual':
            self._show_manual_hint(self._manual_action_text())
            return "break"

    def _manual_cmd(self) -> CmdSetpoints:
        # ↑/↓ speed, ←/→ yaw, PgUp/PgDn climb
        dv, dyaw, dvz = 0.5, 20.0, 0.5
        if 'Up' in self.keys or 'w' in self.keys:    self.man_v = min(self.man_v + dv, self.limits.V_MAX)
        if 'Down' in self.keys or 's' in self.keys:  self.man_v = max(self.man_v - dv, 0.0)
        if 'Left' in self.keys or 'a' in self.keys:  self.man_yaw = -dyaw
        elif 'Right' in self.keys or 'd' in self.keys: self.man_yaw = +dyaw
        else: self.man_yaw = 0.0
        if 'Prior' in self.keys: self.man_vz = min(self.man_vz + dvz,  self.limits.VZ_MAX)   # PageUp
        elif 'Next' in self.keys: self.man_vz = max(self.man_vz - dvz, -self.limits.VZ_MAX)  # PageDown
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
        """Freeze the sim when we’re essentially at the destination on the ground."""
        if self.mode != 'Route' or not self.route_dest:
            return
        dx = self.route_dest.x - self.state.x
        dy = self.route_dest.y - self.state.y
        dist_xy = (dx*dx + dy*dy) ** 0.5
        tol_xy = 3.0 if self.vehicle_type == 'quad' else 5.0
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

    def _on_gui_tick(self):
        now = time.time()
        dt_wall = now - self.prev_time
        self.prev_time = now

        # physics integration (keep stepping during landing)
        if self.running:
            self.accum = min(self.accum + dt_wall, 0.25)
            steps = 0
            while self.accum >= DT and steps < 8:
                cmd = self._get_command()

                # battery drain
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

        # clamp slight negatives (AGL)
        if self.state.z < 0.0:
            self.state.z = 0.0
            if self.state.vz < 0.0: self.state.vz = 0.0

        # finish landing (touchdown logic)
        if self.landing and self.land_anchor:
            ax, ay = self.land_anchor
            d_xy = math.hypot(self.state.x - ax, self.state.y - ay)
            if self.state.z <= 0.05 and d_xy <= 5.0:
                self.state.z = 0.0
                self.state.vz = 0.0
                self.state.v_forward = 0.0
                self.running = False
                self.landed = True
                self.landing = False
            elif d_xy > 80.0 and self.state.vz >= -0.05:
                # abort stale landing if we drifted away without descending
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

        # draw (and optional wind overlay if available)
        self.map.set_camera(self.state.x, self.state.y)
        if hasattr(self.map, "draw_wind_overlay"):
            try:
                self.map.draw_wind_overlay(self.env.wind_e, self.env.wind_n)
            except Exception:
                pass

        self.map.draw_drone(
            self.state.x, self.state.y, self.state.yaw,
            trail=self.state.trail,
            vehicle_type=self.vehicle_type,
            show_fov=False,
            wind=(self.env.wind_e, self.env.wind_n),
            wind_anchor='tr',      # top-right
            wind_yoff=60           # push below the black HUD bar
        )

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

        # contextual status on panel
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
        """Simple wind-compensated straight-in final to DEST (no FAF)."""
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

    def _get_command(self) -> CmdSetpoints:
        if self.landed:
            return CmdSetpoints(0.0, 0.0, 0.0)

        if self.mode == 'Manual':
            return self._manual_cmd()

        if self.mode == 'External':
            msgs = self.udp.poll()
            if msgs:
                m = msgs[-1]
                dx = float(m.get('dx', 0.0)); dy = float(m.get('dy', 0.0)); dz = float(m.get('dz_rate', 0.0))
                return map_tracking_to_setpoints(
                    dx, dy, dz,
                    k_yaw_dps=self.gains.K_YAW, k_fwd=self.gains.K_FWD,
                    v_base=self.gains.V_BASE, v_max=self.limits.V_MAX, vz_max=self.limits.VZ_MAX
                )
            return CmdSetpoints(0.0, 0.0, 0.0)

        # Route AP
        if self.mode == 'Route':
            if not (self.route_start and self.route_dest):
                return CmdSetpoints(0.0, 0.0, 0.0)

            now = time.time()
            route_elapsed = (now - self._route_t0) if self._route_t0 else 0.0

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

            # count time spent in go-around for fallback straight-in
            if goaround:
                self._goaround_time += DT
            else:
                self._goaround_time = max(0.0, self._goaround_time - 0.5*DT)

            dx = self.route_dest.x - self.state.x
            dy = self.route_dest.y - self.state.y
            dist_dest = math.hypot(dx, dy)
            need_fallback = (
                (self.vehicle_type == 'fixed') and
                (self._goaround_time > 6.0 or route_elapsed > 25.0 or dist_dest < 120.0)
            )
            if need_fallback and not self.landing:
                cmd, cap2 = self._direct_to_final(self.route_dest.x, self.route_dest.y, self.v_cruise, k_heading=3.0)
                yaw_rate_cmd, v_cmd = cmd.yaw_rate_dps, cmd.v_forward
                capture_ok = capture_ok or cap2
                self._loitering = False
                self._goaround = False

            # Arm landing only when capture_ok
            if not self.landing and capture_ok:
                self.landing = True
                self.landed = False
                self.land_anchor = (self.route_dest.x, self.route_dest.y)

            # Fixed-wing intercept/go-around: do not descend until landing armed
            if self.vehicle_type == 'fixed' and not self.landing:
                return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd, v_forward=v_cmd, vz=0.0)

            # LANDING (anchored)
            if self.landing and self.land_anchor:
                vz_land = -min(self.limits.VZ_MAX, 2.0)
                ax, ay = self.land_anchor
                dx = ax - self.state.x; dy = ay - self.state.y
                dist = (dx*dx + dy*dy) ** 0.5
                we, wn = self.env.wind_e, self.env.wind_n

                if self.vehicle_type == 'quad':
                    eps = min(2.0, dist / 5.0)
                    if dist > 0.05:
                        track = math.atan2(dy, dx)
                        air_e = eps*math.cos(track) - we
                        air_n = eps*math.sin(track) - wn
                    else:
                        air_e, air_n = -we, -wn
                    v_cmd2 = min(self.limits.V_MAX, (air_e*air_e + air_n*air_n) ** 0.5)
                    hdg    = math.degrees(math.atan2(air_n, air_e)) if v_cmd2 > 0.05 else self.state.yaw
                    yaw_err = ((hdg - self.state.yaw + 540) % 360) - 180
                    yaw_rate_cmd2 = 3.0 * yaw_err
                    return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd2, v_forward=v_cmd2, vz=vz_land)

                else:  # fixed: wind-compensated final + flare
                    v_land = max(12.0, min(self.v_cruise, 20.0))
                    alt = self.state.z
                    v_floor = 12.0
                    if alt < 8.0: v_floor = 8.0
                    if alt < 3.0: v_floor = 2.0

                    v_g = max(1.0, min(v_land, dist * 0.3))
                    if dist > 0.05:
                        tx, ty = dx / dist, dy / dist
                    else:
                        tx, ty = 1.0, 0.0

                    air_e = tx * v_g - we
                    air_n = ty * v_g - wn
                    v_cmd2 = min(self.limits.V_MAX, (air_e*air_e + air_n*air_n) ** 0.5)
                    v_cmd2 = max(v_floor, v_cmd2)

                    hdg_rad = math.atan2(air_n, air_e)
                    g_min = 0.3 if alt < 3.0 else 0.5
                    wind_along = we * math.cos(hdg_rad) + wn * math.sin(hdg_rad)
                    need = g_min - wind_along
                    if need > v_cmd2:
                        v_cmd2 = min(self.limits.V_MAX, need)

                    if alt <= 0.2 and dist <= 6.0:
                        v_cmd2 = 0.0

                    hdg    = math.degrees(hdg_rad)
                    yaw_err = ((hdg - self.state.yaw + 540) % 360) - 180
                    yaw_rate_cmd2 = 2.5 * yaw_err
                    return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd2, v_forward=v_cmd2, vz=vz_land)

            # En-route (no landing yet)
            return CmdSetpoints(yaw_rate_dps=yaw_rate_cmd, v_forward=v_cmd, vz=0.0)

        # fallback
        return CmdSetpoints(0.0, 0.0, 0.0)
