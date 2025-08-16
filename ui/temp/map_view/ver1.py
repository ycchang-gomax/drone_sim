# ui/map_view.py — Tkinter Canvas map & overlays (pure stdlib)
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

        self.center_px = (0, 0)    # canvas pixel center
        self.camera_m  = (0.0, 0.0)  # world meters (ENU) camera center

        self.map_img = None        # loaded tile image (PhotoImage)
        self._drone_items = {'body': None, 'trail': None, 'rotors': [], 'fov': None}
        self._last_vehicle_type = None
        self._spin = 0.0

        self._load_map()           # try to load at startup

    # ---------- camera / coords ----------
    def set_camera(self, x_m, y_m):
        self.camera_m = (x_m, y_m)

    def world_to_px(self, x_m, y_m):
        cx, cy = self.center_px
        px = cx + int((x_m - self.camera_m[0]) / self.m_per_px)
        py = cy - int((y_m - self.camera_m[1]) / self.m_per_px)
        return px, py

    # ---------- window resize ----------
    def _on_resize(self, _evt=None):
        w = self.winfo_width()
        h = self.winfo_height()
        if w <= 2 or h <= 2:
            return
        self.center_px = (w // 2, h // 2)
        if self.map_img is None:
            self._load_map()

    def _load_map(self):
        # Try PNG/PPM etc. If it fails, we'll draw a grid instead.
        try:
            self.map_img = tk.PhotoImage(file=self.map_path)
        except Exception:
            self.map_img = None

    # ---------- background drawing ----------
    def _draw_grid(self, w, h, step=50):
        self.canvas.delete('bg')
        self.canvas.create_rectangle(0, 0, w, h, fill='#2a2a2a', outline='', tags='bg')
        for x in range(0, w, step):
            self.canvas.create_line(x, 0, x, h, fill='#3c3c3c', tags='bg')
        for y in range(0, h, step):
            self.canvas.create_line(0, y, w, y, fill='#3c3c3c', tags='bg')
        self.canvas.create_text(w-8, h-8, anchor='se', fill='#808080',
                                text='No map image — showing grid', tags='bg')
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

        # Pixel position of world origin with current camera
        cx, cy = self.center_px
        px0 = cx - (self.camera_m[0] / self.m_per_px)
        py0 = cy + (self.camera_m[1] / self.m_per_px)

        # Tile so we cover the viewport
        x0 = int(px0 % tw) - tw
        y0 = int(py0 % th) - th

        for X in range(x0, w + tw, tw):
            for Y in range(y0, h + th, th):
                self.canvas.create_image(X, Y, image=self.map_img, anchor='nw', tags='bg')

        # ensure background stays under everything
        self.canvas.tag_lower('bg')

    # ---------- main draw ----------
    def draw_drone(self, x_m, y_m, yaw_deg, fov_deg=60, trail=None, vehicle_type='quad'):
        # 1) background that scrolls with camera
        self._draw_background_tiled()

        # 2) reset layers if vehicle type changed
        if self._last_vehicle_type != vehicle_type:
            for k, v in list(self._drone_items.items()):
                if isinstance(v, list):
                    for it in v:
                        self.canvas.delete(it)
                elif v is not None:
                    self.canvas.delete(v)
            self._drone_items = {'body': None, 'trail': None, 'rotors': [], 'fov': None}
            self._last_vehicle_type = vehicle_type

        # 3) trail
        if trail:
            pts = []
            for (wx, wy) in trail[-800:]:
                px, py = self.world_to_px(wx, wy)
                pts.extend([px, py])
            if self._drone_items.get('trail') is None:
                if pts:
                    self._drone_items['trail'] = self.canvas.create_line(
                        *pts, fill='#66ccff', width=3, smooth=True
                    )
            else:
                if pts:
                    self.canvas.coords(self._drone_items['trail'], *pts)

        # 4) pose helpers
        x, y = self.world_to_px(x_m, y_m)
        yaw = math.radians(yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)

        def rx(dx, dy):  # rotate then translate (body -> canvas)
            return (x + int(dx*c - dy*s), y - int(dx*s + dy*c))

        # 5) vehicle drawing
        if vehicle_type == 'quad':
            arm = 30
            hub = 6
            rotor_r = 10

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

            # rotors (simple animation)
            self._spin = (self._spin + 0.25) % 20.0
            r0 = rotor_r + (2.0 * math.sin(self._spin))
            rot_centers = [p_fw, p_bw, p_rt, p_lt]
            if not self._drone_items.get('rotors'):
                self._drone_items['rotors'] = [
                    self.canvas.create_oval(0, 0, 0, 0, fill='#ffffff', outline='#000000')
                    for _ in range(4)
                ]
            for i, (cx, cy) in enumerate(rot_centers):
                self.canvas.coords(self._drone_items['rotors'][i], cx - r0, cy - r0, cx + r0, cy + r0)

        else:  # fixed-wing (clean airplane silhouette, smaller wings)
            # ---------- sizing (pixels) ----------
            SCALE = 1.0
            Lf   = int(100 * SCALE)
            Wf   = int(12  * SCALE)
            S    = int(45  * SCALE)   # half-span
            Cr   = int(24  * SCALE)   # root chord
            Ct   = int(14  * SCALE)   # tip chord
            SWEEP_BACK = int(12 * SCALE)   # <-- sweep BACK, not forward
            St   = int(24  * SCALE)
            Ctr  = int(16  * SCALE)
            nose = int(12  * SCALE)

            def rxpt(px, py):
                return (x + int(px*c - py*s), y - int(px*s + py*c))

            # --------- MAIN WING (swept-back trapezoid, both sides) ---------
            x_le_root = +Cr/2
            x_te_root = -Cr/2
            x_le_tip  = x_le_root - SWEEP_BACK   # <-- swept BACK
            x_te_tip  = x_le_tip  - Ct

            wing_pts_body = [
                (x_le_root, 0),        # root LE
                (x_le_tip, +S),        # right tip LE
                (x_te_tip, +S),        # right tip TE
                (x_te_root, 0),        # root TE
                (x_te_tip, -S),        # left  tip TE
                (x_le_tip, -S),        # left  tip LE
            ]
            wing_pts = [coord for p in wing_pts_body for coord in rxpt(*p)]

            # Fuselage with nose wedge
            fus_body = [
                (+Lf/2, 0),
                (+Lf/2 - nose, +Wf/2),
                (-Lf/2, +Wf/2),
                (-Lf/2, -Wf/2),
                (+Lf/2 - nose, -Wf/2),
            ]
            fus_pts = [coord for p in fus_body for coord in rxpt(*p)]

            # Tailplane
            x_tail = -Lf/2 + 18
            tail_body = [
                (x_tail + Ctr/2, +St),
                (x_tail - Ctr/2, +St),
                (x_tail - Ctr/2, -St),
                (x_tail + Ctr/2, -St),
            ]
            tail_pts = [coord for p in tail_body for coord in rxpt(*p)]

            # Fin (vertical)
            fin_body = [
                (x_tail + 10, 0),
                (x_tail - 12, +Wf*0.45),
                (x_tail - 12, -Wf*0.45),
            ]
            fin_pts = [coord for p in fin_body for coord in rxpt(*p)]

            # Canopy
            canopy_body = [
                (+Lf/2 - nose - 22, +Wf*0.26),
                (+Lf/2 - nose - 10, +Wf*0.18),
                (+Lf/2 - nose - 10, -Wf*0.18),
                (+Lf/2 - nose - 22, -Wf*0.26),
            ]
            canopy_pts = [coord for p in canopy_body for coord in rxpt(*p)]

            if self._drone_items.get('body') is None or len(self._drone_items['body']) < 5:
                wing_id   = self.canvas.create_polygon(*wing_pts, fill='#e6e6e6', outline='#5a5a5a', width=2)
                fus_id    = self.canvas.create_polygon(*fus_pts,  fill='#d0d0d0', outline='#3a3a3a', width=2)
                tail_id   = self.canvas.create_polygon(*tail_pts, fill='#e6e6e6', outline='#5a5a5a', width=2)
                fin_id    = self.canvas.create_polygon(*fin_pts,  fill='#cfcfcf', outline='#3a3a3a', width=2)
                canopy_id = self.canvas.create_polygon(*canopy_pts, fill='#2b6cb0', outline='#1c3f66', width=1)
                self._drone_items['body'] = [wing_id, fus_id, tail_id, fin_id, canopy_id]
            else:
                wing_id, fus_id, tail_id, fin_id, canopy_id = self._drone_items['body']
                self.canvas.coords(wing_id,   *wing_pts)
                self.canvas.coords(fus_id,    *fus_pts)
                self.canvas.coords(tail_id,   *tail_pts)
                self.canvas.coords(fin_id,    *fin_pts)
                self.canvas.coords(canopy_id, *canopy_pts)

            # hide rotors if switching from quad
            for it in self._drone_items.get('rotors', []):
                self.canvas.itemconfigure(it, state='hidden')

        # 6) FOV wedge (common)
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

        # 7) Layering: background at bottom, drone + trail + FOV on top
        self.canvas.tag_lower('bg')
        if self._drone_items.get('trail'):
            self.canvas.tag_raise(self._drone_items['trail'])
        if self._drone_items.get('fov'):
            self.canvas.tag_raise(self._drone_items['fov'])
        body = self._drone_items.get('body')
        if isinstance(body, list):
            for it in body:
                self.canvas.tag_raise(it)
        elif body:
            self.canvas.tag_raise(body)
        for it in self._drone_items.get('rotors', []):
            self.canvas.tag_raise(it)
