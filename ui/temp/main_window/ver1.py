# ui/main_window.py — Tk root & main loop (with status HUD text)
import tkinter as tk
from tkinter import ttk
import time, math

from config import WINDOW_W, WINDOW_H, MAP_IMAGE, METERS_PER_PIXEL, DT, FPS
from config import Limits, Gains, WindDefault, UDP_LISTEN_PORT, GRAVITY
from core.state import DroneState, EnvState, CmdSetpoints, LinkStatus
from core.physics import step as physics_step
from core.guidance import map_tracking_to_setpoints
from simio.udp_rx import UdpReceiver   # ← folder was renamed from 'io' to 'simio'
from ui.map_view import MapView
from ui.controls import ControlsPanel

class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Path Simulator (Physics Pack 1)")
        self.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.configure(bg='#1b1b1b')

        # State
        self.limits = Limits()
        self.gains  = Gains()
        self.state  = DroneState()
        self.env    = EnvState(WindDefault().east, WindDefault().north, WindDefault().up)
        self.link   = LinkStatus(source='DEMO', connected=False)
        self.vehicle_type = 'quad'   # <-- add this BEFORE creating ControlsPanel

        # Layout
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.rowconfigure(0, weight=1)

        self.map = MapView(self, MAP_IMAGE, METERS_PER_PIXEL)
        self.map.grid(row=0, column=0, sticky='nsew')

        self.controls = ControlsPanel(self, self._on_start, self._on_pause, self._on_reset, self._on_params_change)
        self.controls.grid(row=0, column=1, sticky='nsew')
        self.vehicle_type = 'quad'


        # HUD text on map
        self.hud = tk.Label(self.map.canvas, text="", fg='#ffffff', bg='#000000', padx=6, pady=3)
        self.hud.place(x=10, y=10)

        # Timer
        self.running = False
        self.accum = 0.0
        self.prev_time = time.time()

        # I/O (UDP only now)
        self.udp = UdpReceiver(port=UDP_LISTEN_PORT)

        # Demo command (if no external RX)
        self.demo_phase = 0.0

        # Kick GUI refresh
        self.after(int(1000/FPS), self._on_gui_tick)

        # Keybinds (works even when a slider/button has focus)
        self.bind_all("<space>", lambda e: self._on_pause() if self.running else self._on_start())
        self.bind_all("<r>", lambda e: self._on_reset())
        self.bind_all("<R>", lambda e: self._on_reset())  # optional: uppercase R too


    def _on_params_change(self, p):
        self.vehicle_type = p.get('VEHICLE', getattr(self, 'vehicle_type', 'quad'))
        self.limits.V_MAX = p['V_MAX']
        self.limits.VZ_MAX = p['VZ_MAX']
        self.limits.YAW_RATE_MAX = p['YAW_MAX']
        self.gains.K_YAW = p['K_YAW']
        self.gains.V_BASE = p['V_BASE']
        self.gains.K_FWD = p['K_FWD']
        self.env.wind_e = p['W_E']
        self.env.wind_n = p['W_N']        


    def _on_start(self):
        # Start and reset timers so there is no backlog to "catch up"
        self.running = True
        self.accum = 0.0
        self.prev_time = time.time()
        self.controls.status.config(text='Status: running')

    def _on_pause(self):
        self.running = False
        self.controls.status.config(text='Status: paused')

    def _on_reset(self):
        self.state = DroneState()
        self.accum = 0.0
        self.prev_time = time.time()
        self.controls.status.config(text='Status: reset')


    def _on_gui_tick(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        if self.running:
            # Avoid giant backlog (e.g., when the app was paused/minimized)
            self.accum = min(self.accum + dt, 0.25)  # cap at 0.25 s
            max_steps = 8
            steps = 0
            while self.accum >= DT and steps < max_steps:
                cmd = self._get_command()
                physics_step(
                    DT, self.state, self.env, cmd,
                    v_max=self.limits.V_MAX,
                    vz_max=self.limits.VZ_MAX,
                    yaw_rate_max_dps=self.limits.YAW_RATE_MAX,
                    a_max=self.limits.A_MAX,
                    bank_max_deg=self.limits.BANK_MAX,
                    g=GRAVITY,
                )
                self.accum -= DT
                steps += 1

        # Keep camera centered on the drone
        self.map.set_camera(self.state.x, self.state.y)

        # Draw + HUD
        self.map.draw_drone(self.state.x, self.state.y, self.state.yaw,
                    trail=self.state.trail, vehicle_type=self.vehicle_type)
        gs = math.hypot(self.state.v_forward + self.env.wind_e, self.env.wind_n)
        hud_txt = (f"GS≈{gs:4.1f} m/s  |  Vfwd={self.state.v_forward:4.1f}  "
                   f"|  Alt={self.state.z:5.1f} m  |  Climb={self.state.vz:4.1f} m/s\n"
                   f"Roll={self.state.roll:5.1f}°  Pitch={self.state.pitch:5.1f}°  Yaw={self.state.yaw:6.1f}°  "
                   f"|  Wind=({self.env.wind_e:.1f}, {self.env.wind_n:.1f}) m/s")
        self.hud.config(text=hud_txt)

        self.after(int(1000 / FPS), self._on_gui_tick)


    def _get_command(self) -> CmdSetpoints:
        # Poll UDP commands (JSON or binary)
        msgs = self.udp.poll()
        if msgs:
            self.link.connected = True
            self.link.packets += len(msgs)
            m = msgs[-1]
            dx = float(m.get('dx', 0.0))
            dy = float(m.get('dy', 0.0))
            dz = float(m.get('dz_rate', 0.0))
            self.link.conf = int(m.get('conf', 0))
            self.link.mode = int(m.get('mode', 0))
            return map_tracking_to_setpoints(
                dx, dy, dz,
                k_yaw_dps=self.gains.K_YAW,
                k_fwd=self.gains.K_FWD,
                v_base=self.gains.V_BASE,
                v_max=self.limits.V_MAX,
                vz_max=self.limits.VZ_MAX
            )
        # DEMO if no external commands
        self.demo_phase += DT
        yaw_rate = 20.0 * math.sin(self.demo_phase * 0.4)
        v_fwd = 5.0
        return CmdSetpoints(yaw_rate_dps=yaw_rate, v_forward=v_fwd, vz=0.0)
