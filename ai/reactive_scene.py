# ai/reactive_scene.py — simple 2D road + reactive car for AI tracking experiments
from __future__ import annotations
import math, time, random
from dataclasses import dataclass, field
try:
    from PIL import Image
except Exception:
    Image = None  # run without textures if Pillow missing

LANE_W = 3.6     # meters
ROAD_W = 2 * LANE_W
CAM_FOV_M = 30.0 # approx view span in meters across (used for scale)
BOX_ASPECT = 1.1 # target bbox h/w

@dataclass
class Car:
    s: float = 0.0               # longitudinal position (m)
    y: float = 0.0               # lateral offset (center=0) in meters
    u: float = 12.0              # speed along s (m/s)
    a: float = 0.0               # accel (m/s^2)
    lane: int = 0                # -1 left, 0 center, +1 right (we use two lanes: -0.5,+0.5)
    tex_w: int = 140             # base pixels of texture, for bbox scale
    tex_h: int = 120
    # evasive behavior
    mode: str = "cruise"         # cruise | lane | swerve | brake | burst
    t_mode: float = 0.0          # time left in current mode

    def pick_evasive(self, aggressiveness: float):
        # Weighted random choice depending on aggressiveness
        r = random.random()
        self.t_mode = random.uniform(0.6, 1.7) * (0.5 + aggressiveness)
        if r < 0.25 + 0.35*aggressiveness:
            self.mode = "lane"
        elif r < 0.55 + 0.25*aggressiveness:
            self.mode = "swerve"
        elif r < 0.75 + 0.15*aggressiveness:
            self.mode = "brake"
        else:
            self.mode = "burst"

@dataclass
class ReactiveScene:
    view_w: int
    view_h: int
    road_speed: float = 10.0
    evade_aggr: float = 0.6
    cam_zoom: float  = 1.0
    auto_lock_enabled: bool = True
    # internal
    car: Car = field(default_factory=Car)
    locked: bool = False
    lock_score: float = 0.0
    last_det_ts: float = 0.0
    tex_img: Image.Image | None = None  # PIL image for sizing/bbox reference
    _t: float = 0.0

    def reset(self):
        self.car = Car(u=max(8.0, self.road_speed))
        self.locked = False
        self.lock_score = 0.0
        self.last_det_ts = 0.0
        self._t = 0.0

    # called by the app when a car texture is loaded/changed
    def set_car_texture(self, pil_img: Image.Image):
        self.tex_img = pil_img
        if pil_img:
            self.car.tex_w, self.car.tex_h = pil_img.size

    # YOLO results can plug here later
    def feed_detection(self, *, locked: bool, bbox=None, score: float = 0.0, ts: float = None):
        self.locked = bool(locked)
        self.lock_score = float(score)
        self.last_det_ts = float(ts) if ts is not None else time.time()

    def _maybe_autolock(self):
        # If AI not wired yet, just pretend lock when target is fairly centered.
        cx, cy, w, h = self.get_target_bbox()
        vx, vy = self.view_w/2, self.view_h/2
        dx = abs(cx - vx) / (self.view_w*0.5)
        dy = abs(cy - vy) / (self.view_h*0.5)
        good_center = dx < 0.25 and dy < 0.25
        self.locked = good_center
        self.lock_score = 0.9 if good_center else 0.2

    def update(self, dt: float):
        self._t += dt

        if self.auto_lock_enabled:
            self._maybe_autolock()

        # Evasive choices if locked
        if self.locked:
            if self.car.t_mode <= 0.0 or self.car.mode == "cruise":
                self.car.pick_evasive(self.evade_aggr)
        else:
            # decay to cruise
            if self.car.t_mode <= 0.0:
                self.car.mode = "cruise"
                self.car.t_mode = 0.5

        # Integrate behavior
        c = self.car
        c.t_mode -= dt

        # Base dynamics
        u_des = self.road_speed
        a_cmd = (u_des - c.u) * 0.8    # relax to road speed

        # Behavior overlays
        if c.mode == "lane":
            # move toward +/- half-lane
            y_goal = (random.choice([-0.5, 0.5]) * LANE_W)
            c.y += (y_goal - c.y) * min(1.0, dt * (1.5 + 2.0*self.evade_aggr))
        elif c.mode == "swerve":
            c.y += math.sin(self._t * (2.5 + 4.0*self.evade_aggr)) * 1.2 * dt * LANE_W
        elif c.mode == "brake":
            a_cmd -= (2.0 + 5.0*self.evade_aggr)
        elif c.mode == "burst":
            a_cmd += (2.0 + 4.0*self.evade_aggr)
        # cruise: nothing special

        c.a = max(-6.0, min(6.0, a_cmd))
        c.u = max(0.5, min(30.0, c.u + c.a * dt))
        c.s += c.u * dt

        # damp lateral
        c.y *= (1.0 - 0.6*dt)

    # ---------- Rendering + geometry -----------------------------------------
    def meters_to_pixels(self, m: float) -> float:
        # map “meters across” to view pixels with zoom
        px_per_m = (self.view_w / (CAM_FOV_M / max(0.2, self.cam_zoom)))
        return m * px_per_m

    def draw(self, cvs, tkimg=None):
        # background road stripes scrolling based on car.s
        cvs_w, cvs_h = self.view_w, self.view_h
        cvs.configure(width=cvs_w, height=cvs_h)

        cvs.create_rectangle(0, 0, cvs_w, cvs_h, fill="#101314", outline="")

        road_w_px = self.meters_to_pixels(ROAD_W)
        road_x0 = (cvs_w - road_w_px) / 2
        road_x1 = road_x0 + road_w_px
        cvs.create_rectangle(road_x0, 0, road_x1, cvs_h, fill="#22292b", outline="")

        # lane lines (dashed, scroll with s)
        dash_m = 3.5
        gap_m  = 4.0
        period_m = dash_m + gap_m
        period_px = self.meters_to_pixels(period_m)
        offset_px = (self.meters_to_pixels(self.car.s) % period_px)
        x_mid = (road_x0 + road_x1) / 2
        for y in range(-int(period_px), cvs_h + int(period_px), int(period_px)):
            y0 = y + offset_px
            y1 = y0 + self.meters_to_pixels(dash_m)
            cvs.create_rectangle(x_mid-2, y0, x_mid+2, y1, fill="#cfd9db", outline="")

        # car screen position (centered horizontally by lane, vertically ~60% down)
        cx = (road_x0 + road_x1) / 2 + self.meters_to_pixels(self.car.y)
        cy = cvs_h * 0.6

        # scale car by zoom (fake depth scaling)
        base_w = max(40, min(300, int(self.meters_to_pixels(3.0))))  # ~3m width visual
        if tkimg:
            w = base_w
            h = int(w * (self.car.tex_h / max(1, self.car.tex_w)))
            cvs.create_image(cx, cy, image=tkimg, anchor="s")  # bottom-center anchor
        else:
            w = base_w
            h = int(w * 0.75)
            cvs.create_rectangle(cx - w/2, cy - h, cx + w/2, cy, fill="#b23a3a", outline="#882626", width=2)

        # bbox for “AI”
        bx, by = cx - w/2, cy - h
        cvs.create_rectangle(bx, by, bx+w, by+h, outline="#f44", width=2)

        # lock display
        lock_txt = "LOCKED" if self.locked else "searching"
        cvs.create_text(12, 12, text=f"{lock_txt}  score={self.lock_score:0.2f}",
                        fill="#f6e58d" if self.locked else "#9aa7b1",
                        anchor="nw", font=("Consolas", 12, "bold"))

        # aim reticle (center)
        cvs.create_oval(self.view_w/2 - 5, self.view_h/2 - 5, self.view_w/2 + 5, self.view_h/2 + 5,
                        outline="#ffe082")

    def get_target_bbox(self):
        # return bbox center x,y and size (px)
        base_w = max(40, min(300, int(self.meters_to_pixels(3.0))))
        cx = self.view_w / 2 + self.meters_to_pixels(self.car.y)
        cy = self.view_h * 0.6
        w = base_w
        h = int(w * 0.75)
        return cx, cy - h/2, w, h

    def debug_text(self, dx=0.0, dy=0.0):
        c = self.car
        return (f"s={c.s:7.1f} m  u={c.u:4.1f} m/s  a={c.a:+4.1f}  "
                f"y={c.y:+4.2f} m  mode={c.mode:>6} ({c.t_mode:4.2f}s)\n"
                f"lock={self.locked} score={self.lock_score:.2f}   "
                f"dx={dx:+.0f}px dy={dy:+.0f}px  zoom={self.cam_zoom:.2f}")
