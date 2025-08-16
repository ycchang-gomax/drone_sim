# ui/controls.py — controls with Mode/Vehicle, Route toggle, LED, sliders
# and per-control clickable help popups.
import tkinter as tk
from tkinter import ttk, messagebox

HELP = {
    "MODE": (
        "Mode\n"
        "• Manual – you fly: ↑/↓ speed, ←/→ yaw, PgUp/PgDn climb.\n"
        "• External – moves only when UDP dx,dy,dz_rate arrive from your AI.\n"
        "• Route – internal autopilot flies Start → Destination (pure-pursuit)."
    ),
    "VEHICLE": (
        "Vehicle\n"
        "• quad – multirotor graphic (yaw-limited but can turn in place).\n"
        "• fixed – compact swept-wing airplane graphic (turns limited by bank)."
    ),
    "ROUTE_BTN": (
        "Route AP quick toggle\n"
        "A convenience button that flips External ⇄ Route without changing Manual."
    ),
    "V_MAX": (
        "V_MAX [m/s]\n"
        "Maximum forward airspeed the controller can command.\n"
        "Higher speed ⇒ larger turn radius (harder to stay on track)."
    ),
    "VZ_MAX": (
        "VZ_MAX [m/s]\n"
        "Maximum climb/descent rate. All vertical commands are clipped to this."
    ),
    "YAW_MAX": (
        "Yaw rate max [deg/s]\n"
        "Upper limit on commanded yaw rotation. The sim also enforces a bank-angle\n"
        "limit; the effective yaw rate is the smaller of this limit and the\n"
        "bank-limited value (at high speed the bank limit dominates)."
    ),
    "K_YAW": (
        "K_yaw [deg/s @1px]\n"
        "External mode only. Horizontal tracking gain: if the target is 1 pixel\n"
        "off-center, command K_yaw deg/s (scaled by error, then clipped by Yaw max)."
    ),
    "K_FWD": (
        "K_fwd [m/s @1err]\n"
        "External mode only. Forward-speed boost per unit of vertical/size error.\n"
        "Large error → faster approach; small error → near V_base."
    ),
    "V_BASE": (
        "V_base [m/s] (External)\n"
        "Baseline forward speed in External mode when error is small."
    ),
    "V_CRUISE": (
        "Cruise speed [m/s] (Route)\n"
        "Target forward speed for the Route autopilot.\n"
        "Lower it for tighter turns; capped by V_MAX."
    ),
    "BATTERY_WH": (
        "Battery capacity [Wh]\n"
        "Simple energy model: Power ≈ baseline + k·(speed³) + climb term.\n"
        "When energy reaches 0 Wh the sim cuts thrust and stops."
    ),
    "W_E": (
        "Wind East [m/s]\n"
        "Positive = wind blowing toward East (from West). Negative = toward West.\n"
        "Adds to airspeed to form ground speed and drifts the path."
    ),
    "W_N": (
        "Wind North [m/s]\n"
        "Positive = wind toward North (from South). Negative = toward South."
    ),
}

class ControlsPanel(tk.Frame):
    def __init__(self, master, on_start, on_pause, on_reset, on_params_change):
        super().__init__(master, bg='#232323', padx=8, pady=8)
        self.on_params_change = on_params_change

        # ── Run buttons ───────────────────────────────────────────────────────────
        row = tk.Frame(self, bg='#232323'); row.pack(fill=tk.X, pady=(0,8))
        tk.Button(row, text='Start', command=on_start).pack(side=tk.LEFT, padx=4)
        tk.Button(row, text='Pause', command=on_pause).pack(side=tk.LEFT, padx=4)
        tk.Button(row, text='Reset', command=on_reset).pack(side=tk.LEFT, padx=4)

        # ── Mode ─────────────────────────────────────────────────────────────────
        mrow = tk.Frame(self, bg='#232323'); mrow.pack(fill=tk.X, pady=(0,6))
        self._link_label(mrow, "Mode", "MODE").pack(side=tk.LEFT)
        self.mode_var = tk.StringVar(value='Manual')  # Manual | External | Route
        mode_cb = ttk.Combobox(mrow, state='readonly',
                               values=['Manual','External','Route'],
                               textvariable=self.mode_var, width=11)
        mode_cb.pack(side=tk.RIGHT)
        mode_cb.bind('<<ComboboxSelected>>', lambda e: self._notify())

        # ── Vehicle selector ─────────────────────────────────────────────────────
        vrow = tk.Frame(self, bg='#232323'); vrow.pack(fill=tk.X, pady=(0,6))
        self._link_label(vrow, "Vehicle", "VEHICLE").pack(side=tk.LEFT)
        self.vehicle_var = tk.StringVar(value='quad')  # quad | fixed
        vcb = ttk.Combobox(vrow, state='readonly',
                           values=['quad','fixed'],
                           textvariable=self.vehicle_var, width=11)
        vcb.pack(side=tk.RIGHT)
        vcb.bind('<<ComboboxSelected>>', lambda e: self._notify())

        # ── Quick Route toggle ───────────────────────────────────────────────────
        trow = tk.Frame(self, bg='#232323'); trow.pack(fill=tk.X, pady=(0,6))
        # info icon
        self._info_button(trow, "ROUTE_BTN").pack(side=tk.LEFT)
        self.route_btn = tk.Button(trow, text='Route AP: OFF', command=self._toggle_route)
        self.route_btn.pack(side=tk.RIGHT)

        # ── Flight LED ───────────────────────────────────────────────────────────
        lrow = tk.Frame(self, bg='#232323'); lrow.pack(fill=tk.X, pady=(0,6))
        tk.Label(lrow, text='Flight', fg='#cfcfcf', bg='#232323').pack(side=tk.LEFT)
        self.led = tk.Canvas(lrow, width=18, height=18, bg='#232323', highlightthickness=0)
        self.led.pack(side=tk.RIGHT)
        self._led_id = self.led.create_oval(3,3,15,15, fill='#777777', outline='#333333')

        # ── Performance / Guidance ───────────────────────────────────────────────
        self.vmax   = self._slider("V_MAX",  "V_MAX [m/s]",              0, 30, 15.0)
        self.vzmax  = self._slider("VZ_MAX", "VZ_MAX [m/s]",             0, 10,  3.0)
        self.yawmax = self._slider("YAW_MAX","Yaw rate max [deg/s]",     0, 240, 120.0)
        self.kyaw   = self._slider("K_YAW",  "K_yaw [deg/s @1px]",       0, 180, 90.0)
        self.kf     = self._slider("K_FWD",  "K_fwd [m/s @1err]",        0,  20,  8.0)
        self.vbase  = self._slider("V_BASE", "V_base [m/s] (External)",  0,  15,  4.0)

        # ── Mission ──────────────────────────────────────────────────────────────
        self.vcruise = self._slider("V_CRUISE","Cruise speed [m/s] (Route)", 0, 20, 8.0)
        self.batt_wh = self._slider("BATTERY_WH","Battery capacity [Wh]",    20, 400, 120.0)

        # ── Wind (signed) ───────────────────────────────────────────────────────
        self.w_e = self._slider("W_E", "Wind East [m/s]",  -15, 15, 0.0)  # East>0, West<0
        self.w_n = self._slider("W_N", "Wind North [m/s]", -15, 15, 0.0)  # North>0, South<0

        # ── Status ───────────────────────────────────────────────────────────────
        self.status = tk.Label(self, text='Status: idle', anchor='w', fg='#cfcfcf', bg='#232323')
        self.status.pack(fill=tk.X, pady=(8,0))

        self._notify()

    # public helpers --------------------------------------------------------------
    def set_led(self, color='#777777'):
        self.led.itemconfig(self._led_id, fill=color)

    def set_vehicle(self, v: str):
        if v in ('quad','fixed'):
            self.vehicle_var.set(v)
            self._notify()

    # internals -------------------------------------------------------------------
    def _toggle_route(self):
        cur = self.mode_var.get()
        if cur == 'Route':
            self.mode_var.set('External')
        else:
            self.mode_var.set('Route')
        self._notify()

    def _link_label(self, parent, text, key):
        """Create a blue, underlined clickable label that opens help(key)."""
        lbl = tk.Label(parent, text=text, fg='#7faaff', bg='#232323', cursor='hand2')
        lbl.configure(font=(None, 10, 'underline'))
        lbl.bind("<Button-1>", lambda _e: self._show_help(key))
        return lbl

    def _info_button(self, parent, key):
        """Small ⓘ that opens help(key)."""
        btn = tk.Label(parent, text='ⓘ', fg='#7faaff', bg='#232323', cursor='hand2')
        btn.bind("<Button-1>", lambda _e: self._show_help(key))
        return btn

    def _slider(self, key, label, a, b, init):
        row = tk.Frame(self, bg='#232323'); row.pack(fill=tk.X, pady=2)
        # clickable label + info icon
        left = tk.Frame(row, bg='#232323'); left.pack(side=tk.LEFT)
        self._link_label(left, label, key).pack(side=tk.LEFT)
        self._info_button(left, key).pack(side=tk.LEFT, padx=(4,0))
        # slider
        var = tk.DoubleVar(value=init)
        ttk.Scale(row, from_=a, to=b, orient='horizontal', variable=var,
                  command=lambda _e: self._notify()).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=8)
        # store var by key for notify usage
        setattr(self, f"_var_{key}", var)
        return var

    def _show_help(self, key):
        text = HELP.get(key, "No help available for this control.")
        messagebox.showinfo("Help", text, parent=self)

    def _notify(self):
        params = {
            'MODE': self.mode_var.get(),
            'VEHICLE': self.vehicle_var.get(),
            'V_MAX': getattr(self, "_var_V_MAX").get(),
            'VZ_MAX': getattr(self, "_var_VZ_MAX").get(),
            'YAW_MAX': getattr(self, "_var_YAW_MAX").get(),
            'K_YAW': getattr(self, "_var_K_YAW").get(),
            'K_FWD': getattr(self, "_var_K_FWD").get(),
            'V_BASE': getattr(self, "_var_V_BASE").get(),
            'V_CRUISE': getattr(self, "_var_V_CRUISE").get(),
            'BATTERY_WH': getattr(self, "_var_BATTERY_WH").get(),
            'W_E': getattr(self, "_var_W_E").get(),
            'W_N': getattr(self, "_var_W_N").get(),
        }
        self.on_params_change(params)
        self.route_btn.config(text=f"Route AP: {'ON' if self.mode_var.get()=='Route' else 'OFF'}")
