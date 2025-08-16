# ui/map_view.py — Tkinter Canvas map, tiled background, route, trail, vehicles + wind overlay
import math
import tkinter as tk

class MapView(tk.Frame):
    def __init__(self, master, map_path: str, meters_per_px: float = 1.0):
        super().__init__(master, bg='#202020')
        self.canvas = tk.Canvas(self, bg='#1e1e1e', highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.bind('<Configure>', self._on_resize)

        self.map_path = map_path
        self.m_per_px = meters_per_px

        self.center_px = (0, 0)        # canvas center (px)
        self.camera_m  = (0.0, 0.0)    # world camera center (m, ENU)
        self.map_img   = None          # PhotoImage tile

        # draw item buckets
        self._drone_items = {'body': None, 'trail': None, 'rotors': [], 'fov': None}
        self._last_vehicle_type = None
        self._spin = 0.0

        # route overlays
        self.route_start = None
        self.route_dest  = None
        self._route_items = {'start': None, 'dest': None, 'line': None}

        # HUD overlays (wind rose)
        self._hud_items = {'wind_circle': None, 'wind_arrow': None, 'wind_text': None, 'wind_tick': None}


        self._load_map()

    # ---------- camera / coords ----------
    def set_camera(self, x_m, y_m):
        self.camera_m = (x_m, y_m)

    def world_to_px(self, x_m, y_m):
        cx, cy = self.center_px
        px = cx + int((x_m - self.camera_m[0]) / self.m_per_px)
        py = cy - int((y_m - self.camera_m[1]) / self.m_per_px)
        return px, py

    def px_to_world(self, px, py):
        cx, cy = self.center_px
        x_m = self.camera_m[0] + (px - cx) * self.m_per_px
        y_m = self.camera_m[1] - (py - cy) * self.m_per_px
        return x_m, y_m

    # ---------- route API ----------
    def set_route_start(self, x_m, y_m):
        self.route_start = (x_m, y_m)

    def set_route_dest(self, x_m, y_m):
        self.route_dest = (x_m, y_m)

    def clear_route(self):
        self.route_start = None
        self.route_dest = None
        for it in self._route_items.values():
            if it:
                self.canvas.delete(it)
        self._route_items = {'start': None, 'dest': None, 'line': None}

    # ---------- window / map ----------
    def _on_resize(self, _evt=None):
        w = self.winfo_width()
        h = self.winfo_height()
        if w <= 2 or h <= 2:
            return
        self.center_px = (w // 2, h // 2)
        if self.map_img is None:
            self._load_map()

    def _load_map(self):
        try:
            self.map_img = tk.PhotoImage(file=self.map_path)
        except Exception:
            self.map_img = None

    def _draw_grid(self, w, h, step=50):
        self.canvas.delete('bg')
        self.canvas.create_rectangle(0, 0, w, h, fill='#2a2a2a', outline='', tags='bg')
        for x in range(0, w, step):
            self.canvas.create_line(x, 0, x, h, fill='#3c3c3c', tags='bg')
        for y in range(0, h, step):
            self.canvas.create_line(0, y, w, y, fill='#3c3c3c', tags='bg')
        self.canvas.tag_lower('bg')

    def _draw_background_tiled(self):
        w = self.winfo_width()
        h = self.winfo_height()
        self.canvas.delete('bg')

        if self.map_img is None:
            self._draw_grid(w, h)
            return

        tw = self.map_img.width()
        th = self.map_img.height()
        cx, cy = self.center_px
        px0 = cx - (self.camera_m[0] / self.m_per_px)
        py0 = cy + (self.camera_m[1]) / self.m_per_px
        x0 = int(px0 % tw) - tw
        y0 = int(py0 % th) - th

        for X in range(x0, w + tw, tw):
            for Y in range(y0, h + th, th):
                self.canvas.create_image(X, Y, image=self.map_img, anchor='nw', tags='bg')

        self.canvas.tag_lower('bg')

    def _draw_route(self):
        # clear previous
        for k in ('start', 'dest', 'line'):
            if self._route_items[k]:
                self.canvas.delete(self._route_items[k])
                self._route_items[k] = None
        # markers
        if self.route_start:
            sx, sy = self.world_to_px(*self.route_start)
            r = 6
            self._route_items['start'] = self.canvas.create_rectangle(
                sx-r, sy-r, sx+r, sy+r, outline='#00ff55', width=2)
        if self.route_dest:
            dx, dy = self.world_to_px(*self.route_dest)
            r = 3
            self._route_items['dest'] = self.canvas.create_oval(
                dx-r, dy-r, dx+r, dy+r, outline='#ff66ff', width=2)
        if self.route_start and self.route_dest:
            sx, sy = self.world_to_px(*self.route_start)
            dx, dy = self.world_to_px(*self.route_dest)
            self._route_items['line'] = self.canvas.create_line(
                sx, sy, dx, dy, fill='#ff33cc', width=3, dash=(6,4))

    # ---------- wind overlay ----------
    def _draw_wind(self, wind, anchor='tl', yoff=0):
        """
        Draw a wind rose. `wind` is (east_mps, north_mps).
        `anchor` in {'tl','tr','bl','br'} chooses the corner.
        `yoff` (px) lifts the widget for top anchors to avoid HUD overlap.
        """
        # Clear if hidden/None
        if wind is None:
            for k in ('wind_circle', 'wind_arrow', 'wind_text', 'wind_tick'):
                it = self._hud_items.get(k)
                if it:
                    self.canvas.delete(it)
                self._hud_items[k] = None
            return

        we, wn = wind
        speed = math.hypot(we, wn)
        ang   = math.atan2(wn, we)  # direction arrow points (from center), CCW from +x

        w = self.winfo_width() or 0
        h = self.winfo_height() or 0

        R   = 28      # circle radius
        pad = 70      # margin from edges

        # Choose corner + offset
        if anchor == 'tr':
            cx, cy = max(pad, w - pad), pad + yoff
        elif anchor == 'bl':
            cx, cy = pad, max(pad, h - pad)
        elif anchor == 'br':
            cx, cy = max(pad, w - pad), max(pad, h - pad)
        else:  # 'tl'
            cx, cy = pad, pad + yoff

        # Arrow length roughly 4 px per m/s, clamped
        L  = min(48.0, max(0.0, 4.0 * speed))
        ex = cx + L * math.cos(ang)
        ey = cy - L * math.sin(ang)

        # --- circle ---
        if not self._hud_items.get('wind_circle'):
            self._hud_items['wind_circle'] = self.canvas.create_oval(0, 0, 0, 0, outline='#aaaaaa', width=2)
        self.canvas.coords(self._hud_items['wind_circle'], cx - R, cy - R, cx + R, cy + R)

        # --- north tick ---
        if not self._hud_items.get('wind_tick'):
            self._hud_items['wind_tick'] = self.canvas.create_line(0, 0, 0, 0, fill='#aaaaaa', width=2)
        self.canvas.coords(self._hud_items['wind_tick'], cx, cy - R, cx, cy - R - 8)

        # --- arrow ---
        if not self._hud_items.get('wind_arrow'):
            self._hud_items['wind_arrow'] = self.canvas.create_line(0, 0, 0, 0, fill='#22ccff', width=3, arrow='last')
        self.canvas.coords(self._hud_items['wind_arrow'], cx, cy, ex, ey)

        # --- label text (placed outside the circle, inside the screen) ---
        txt = f"Wind {speed:0.1f} m/s  ({we:+.1f}E,{wn:+.1f}N)"
        if anchor in ('tr', 'br'):
            t_anchor, tx = 'e', cx - (R + 10)
        else:
            t_anchor, tx = 'w', cx + (R + 10)
        ty = cy
        if not self._hud_items.get('wind_text'):
            self._hud_items['wind_text'] = self.canvas.create_text(
                tx, ty, anchor=t_anchor, fill='#dddddd', text=txt, font=('Segoe UI', 10, 'bold')
            )
        else:
            self.canvas.coords(self._hud_items['wind_text'], tx, ty)
            self.canvas.itemconfig(self._hud_items['wind_text'], text=txt, anchor=t_anchor)

        # Keep on top of the background
        for k in ('wind_circle', 'wind_tick', 'wind_arrow', 'wind_text'):
            self.canvas.tag_raise(self._hud_items[k])



    # ---------- main draw ----------
    def draw_drone(self, x_m, y_m, yaw_deg, fov_deg=60, trail=None, vehicle_type='quad',
                   *, show_fov=False, wind=None, wind_anchor='tl', wind_yoff=60):
        self._draw_background_tiled()
        self._draw_route()
        self._draw_wind(wind, anchor=wind_anchor, yoff=wind_yoff)

        # reset layers if vehicle type changed
        if self._last_vehicle_type != vehicle_type:
            for v in list(self._drone_items.values()):
                if isinstance(v, list):
                    for it in v: self.canvas.delete(it)
                elif v is not None:
                    self.canvas.delete(v)
            self._drone_items = {'body': None, 'trail': None, 'rotors': [], 'fov': None}
            self._last_vehicle_type = vehicle_type

        # trail (full history, decimate if huge so Canvas stays fast)
        if trail and len(trail) >= 2:
            step = max(1, len(trail) // 4000)  # ~4k points max
            pts = []
            for i in range(0, len(trail), step):
                wx, wy = trail[i]
                px, py = self.world_to_px(wx, wy)
                pts.extend([px, py])
            if self._drone_items.get('trail') is None:
                self._drone_items['trail'] = self.canvas.create_line(
                    *pts, fill='#66ccff', width=3, smooth=True)
            else:
                self.canvas.coords(self._drone_items['trail'], *pts)

        # pose helpers
        x, y = self.world_to_px(x_m, y_m)
        yaw = math.radians(yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        def rx(dx, dy):   # rotate then translate
            return (x + int(dx*c - dy*s), y - int(dx*s + dy*c))
        def rxpt(px, py): # for polygons
            return (x + int(px*c - py*s), y - int(px*s + py*c))

        # ---- vehicle drawing ----
        if vehicle_type == 'quad':
            arm, hub, rotor_r = 30, 6, 10
            p_fw = rx(+arm, 0)
            p_bw = rx(-arm, 0)
            p_rt = rx(0, +arm)
            p_lt = rx(0, -arm)

            if self._drone_items.get('body') is None:
                self._drone_items['body'] = [
                    self.canvas.create_line(p_bw[0], p_bw[1], p_fw[0], p_fw[1], fill='#00f0ff', width=4),
                    self.canvas.create_line(p_lt[0], p_lt[1], p_rt[0], p_rt[1], fill='#00f0ff', width=4),
                    self.canvas.create_oval(x-hub, y-hub, x+hub, y+hub, fill='#ffcc00', outline='#ffffff'),
                ]
            else:
                self.canvas.coords(self._drone_items['body'][0], p_bw[0], p_bw[1], p_fw[0], p_fw[1])
                self.canvas.coords(self._drone_items['body'][1], p_lt[0], p_lt[1], p_rt[0], p_rt[1])
                self.canvas.coords(self._drone_items['body'][2], x-hub, y-hub, x+hub, y+hub)

            # rotor blur “animation”
            self._spin = (self._spin + 0.25) % 20.0
            r0 = rotor_r + (2.0 * math.sin(self._spin))
            rot_centers = [p_fw, p_bw, p_rt, p_lt]
            if not self._drone_items.get('rotors'):
                self._drone_items['rotors'] = [
                    self.canvas.create_oval(0,0,0,0, fill='#ffffff', outline='#000000') for _ in range(4)
                ]
            for i,(cx,cy) in enumerate(rot_centers):
                self.canvas.coords(self._drone_items['rotors'][i], cx-r0, cy-r0, cx+r0, cy+r0)

        else:  # fixed-wing (swept-back, compact)
            SCALE = 1.0
            Lf   = int(100 * SCALE)   # fuselage length
            Wf   = int(12  * SCALE)   # fuselage width
            S    = int(45  * SCALE)   # half-span
            Cr   = int(24  * SCALE)   # root chord
            Ct   = int(14  * SCALE)   # tip chord
            SWEEP_BACK = int(12 * SCALE)
            St   = int(24  * SCALE)   # tail half-span
            Ctr  = int(16  * SCALE)   # tail chord
            nose = int(12  * SCALE)

            # Wing poly (swept BACK)
            x_le_root = +Cr/2
            x_te_root = -Cr/2
            x_le_tip  = x_le_root - SWEEP_BACK
            x_te_tip  = x_le_tip  - Ct
            wing_pts = [coord for p in [
                (x_le_root,0), (x_le_tip,+S), (x_te_tip,+S),
                (x_te_root,0), (x_te_tip,-S), (x_le_tip,-S)
            ] for coord in rxpt(*p)]

            # Fuselage + nose wedge
            fus_pts = [coord for p in [
                (+Lf/2,0), (+Lf/2 - nose, +Wf/2), (-Lf/2, +Wf/2),
                (-Lf/2, -Wf/2), (+Lf/2 - nose, -Wf/2)
            ] for coord in rxpt(*p)]

            # Tailplane + fin + canopy
            x_tail = -Lf/2 + 18
            tail_pts = [coord for p in [
                (x_tail+Ctr/2,+St), (x_tail-Ctr/2,+St),
                (x_tail-Ctr/2,-St), (x_tail+Ctr/2,-St)
            ] for coord in rxpt(*p)]
            fin_pts = [coord for p in [
                (x_tail+10,0), (x_tail-12,+Wf*0.45), (x_tail-12,-Wf*0.45)
            ] for coord in rxpt(*p)]
            canopy_pts = [coord for p in [
                (+Lf/2-nose-22,+Wf*0.26), (+Lf/2-nose-10,+Wf*0.18),
                (+Lf/2-nose-10,-Wf*0.18), (+Lf/2-nose-22,-Wf*0.26)
            ] for coord in rxpt(*p)]

            if self._drone_items.get('body') is None or len(self._drone_items['body']) < 5:
                wing_id   = self.canvas.create_polygon(*wing_pts, fill='#e6e6e6', outline='#5a5a5a', width=2)
                fus_id    = self.canvas.create_polygon(*fus_pts , fill='#d0d0d0', outline='#3a3a3a', width=2)
                tail_id   = self.canvas.create_polygon(*tail_pts, fill='#e6e6e6', outline='#5a5a5a', width=2)
                fin_id    = self.canvas.create_polygon(*fin_pts , fill='#cfcfcf', outline='#3a3a3a', width=2)
                canopy_id = self.canvas.create_polygon(*canopy_pts, fill='#2b6cb0', outline='#1c3f66', width=1)
                self._drone_items['body'] = [wing_id, fus_id, tail_id, fin_id, canopy_id]
            else:
                wing_id, fus_id, tail_id, fin_id, canopy_id = self._drone_items['body']
                self.canvas.coords(wing_id,   *wing_pts)
                self.canvas.coords(fus_id,    *fus_pts)
                self.canvas.coords(tail_id,   *tail_pts)
                self.canvas.coords(fin_id,    *fin_pts)
                self.canvas.coords(canopy_id, *canopy_pts)

            # hide old rotors if switching from quad
            for it in self._drone_items.get('rotors', []):
                self.canvas.itemconfigure(it, state='hidden')

        # FOV wedge (OFF by default)
        if show_fov:
            r = 80
            a0 = -(yaw_deg + fov_deg / 2.0)
            a1 = -(yaw_deg - fov_deg / 2.0)
            if self._drone_items.get('fov') is None:
                self._drone_items['fov'] = self.canvas.create_arc(
                    x - r, y - r, x + r, y + r, start=a0, extent=(a1 - a0),
                    style='arc', outline='#88aaff', width=2
                )
            else:
                self.canvas.coords(self._drone_items['fov'], x - r, y - r, x + r, y + r)
                self.canvas.itemconfig(self._drone_items['fov'], start=a0, extent=(a1 - a0))
        else:
            if self._drone_items.get('fov') is not None:
                self.canvas.delete(self._drone_items['fov'])
                self._drone_items['fov'] = None

        # layering
        self.canvas.tag_lower('bg')
        if self._drone_items.get('trail'): self.canvas.tag_raise(self._drone_items['trail'])
        body = self._drone_items.get('body')
        if isinstance(body, list):
            for it in body: self.canvas.tag_raise(it)
        elif body:
            self.canvas.tag_raise(body)
        for it in self._drone_items.get('rotors', []): self.canvas.tag_raise(it)
