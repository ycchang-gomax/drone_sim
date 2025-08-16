# ai/yolo_panel.py
# Embedded YOLOv8 + DeepSORT tracker as a Tkinter panel.
# See: env knobs at bottom of file header.

import tkinter as tk
from tkinter import ttk
import time, json, socket, os
from collections import deque
import cv2
import numpy as np

# Pillow for reliable color to Tk PhotoImage
try:
    from PIL import Image, ImageTk
except Exception:
    Image, ImageTk = None, None

# Torch to select device (cpu/cuda)
try:
    import torch
except Exception:
    torch = None

# Ultralytics + DeepSORT
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
except Exception:
    DeepSort = None


# ----------------------- utils -----------------------
def _truthy(x):
    return str(x).strip().lower() in ("1","true","t","yes","y","on")

def _open_capture(src, backend='auto', w=0, h=0, fps=0):
    # src may be "0" or 0 or a filename/URL
    if isinstance(src, str) and src.isdigit():
        idx = int(src)
        if backend == 'dshow':
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        elif backend == 'msmf':
            cap = cv2.VideoCapture(idx, cv2.CAP_MSMF)
        else:
            cap = cv2.VideoCapture(idx)
    else:
        cap = cv2.VideoCapture(src)

    if w:   cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    if h:   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if fps: cap.set(cv2.CAP_PROP_FPS,          fps)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video source: {src}")
    return cap


def _ema(prev, x, a):
    return a * x + (1.0 - a) * prev


def _iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
    return inter / max(1e-6, area)


# -------- Robust jitter filter for dx/dy (AI side) --------
class _JitterFilter:
    """
    Median -> hysteresis deadband -> EMA -> per-frame rate limit.
    Designed to kill tiny dx noise on straight roads without adding lag.
    """
    def __init__(self, win=5, dead_enter_px=14, dead_exit_px=8,
                 alpha=0.35, max_step_px=40):
        self.dx_hist = deque(maxlen=int(max(3, win)))
        self.dy_hist = deque(maxlen=int(max(3, win)))
        self.alpha = float(alpha)
        self.dead_enter = float(dead_enter_px)
        self.dead_exit  = float(dead_exit_px)
        self.active = False
        self.max_step = float(max_step_px)  # px change per frame (each _process_frame call)
        self.dx_s = 0.0
        self.dy_s = 0.0

    def _median(self, dq):
        arr = np.fromiter(dq, dtype=float)
        return float(np.median(arr)) if arr.size else 0.0

    def _rate_limit(self, prev, nxt):
        step = self.max_step
        if nxt > prev + step: return prev + step
        if nxt < prev - step: return prev - step
        return nxt

    def update(self, dx, dy):
        # 1) median
        self.dx_hist.append(float(dx))
        self.dy_hist.append(float(dy))
        mdx = self._median(self.dx_hist)
        mdy = self._median(self.dy_hist)

        # 2) hysteresis deadband on dx only (yaw is the problem on straights)
        thr = self.dead_exit if self.active else self.dead_enter
        if abs(mdx) <= thr:
            mdx = 0.0
            self.active = False
        else:
            self.active = True

        # 3) EMA
        nx = self.alpha * mdx + (1.0 - self.alpha) * self.dx_s
        ny = self.alpha * mdy + (1.0 - self.alpha) * self.dy_s

        # 4) per-frame rate limit
        nx = self._rate_limit(self.dx_s, nx)
        ny = self._rate_limit(self.dy_s, ny)

        self.dx_s, self.dy_s = nx, ny
        return nx, ny


def _class_to_type(k: int) -> str:
    # Ultralytics COCO ids: car=2, bus=5, truck=7
    if k == 2:  return "sedan"
    if k == 5:  return "bus"
    if k == 7:  return "pickup"
    return "vehicle"


# ---------------- UDP TX (robust debug) ----------------
class _UdpTX:
    def __init__(self, host="127.0.0.1", port=47800, hz=20.0):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.min_dt = 1.0 / float(hz)
        self.t_last = 0.0

        # ---- DEBUG knobs via env ----
        self.debug       = _truthy(os.getenv("YOLO_PANEL_UDP_DEBUG", "0"))
        self.debug_keys  = _truthy(os.getenv("YOLO_PANEL_UDP_DEBUG_KEYS", "0"))
        self.debug_hz    = float(os.getenv("YOLO_PANEL_UDP_DEBUG_HZ", "5"))
        self._dbg_last   = 0.0
        if self.debug:
            print(f"[AI] UDP debug ON → {self.addr} @ {self.debug_hz} Hz (keys_only={self.debug_keys})")

    def config(self, host, port, hz=None):
        self.addr = (host, int(port))
        if hz and hz > 0:
            self.min_dt = 1.0 / float(hz)

    def _maybe_debug_print(self, obj: dict):
        if not self.debug:
            return
        now = time.time()
        if now - self._dbg_last < (1.0 / max(0.1, self.debug_hz)):
            return
        self._dbg_last = now

        if self.debug_keys:
            print("SENT KEYS:", list(obj.keys()))
            return

        veh = obj.get("veh", None)

        # veh may be dict OR a string (older senders) OR missing
        if isinstance(veh, dict):
            print(
                "UDP→", self.addr,
                f"dx={obj.get('dx',0):+,.0f}  dy={obj.get('dy',0):+,.0f}  dz={obj.get('dz_rate',0):+.2f}  "
                f"locked={obj.get('locked')}  tid={obj.get('tid')}",
                "| veh:",
                f"type={veh.get('type')}  tid={veh.get('tid')}  conf={float(veh.get('conf',0) or 0):.2f}  "
                f"cx_n={float(veh.get('cx_n',0) or 0):.3f}  cy_n={float(veh.get('cy_n',0) or 0):.3f}  "
                f"w_n={float(veh.get('w_n',0) or 0):.3f}  h_n={float(veh.get('h_n',0) or 0):.3f}"
            )
        elif isinstance(veh, str):
            print(
                "UDP→", self.addr,
                f"dx={obj.get('dx',0):+,.0f}  dy={obj.get('dy',0):+,.0f}  dz={obj.get('dz_rate',0):+.2f}  "
                f"locked={obj.get('locked')}  tid={obj.get('tid')}  veh='{veh}'"
            )
        else:
            print(
                "UDP→", self.addr,
                f"dx={obj.get('dx',0):+,.0f}  dy={obj.get('dy',0):+,.0f}  dz={obj.get('dz_rate',0):+.2f}  "
                f"locked={obj.get('locked')}  tid={obj.get('tid')}  veh=None"
            )

    def send(self, dx, dy, dz_rate):
        now = time.time()
        if (now - self.t_last) < self.min_dt:
            return False
        pkt = json.dumps({"dx": float(dx), "dy": float(dy), "dz_rate": float(dz_rate)}).encode("utf-8")
        self.sock.sendto(pkt, self.addr)
        self.t_last = now
        return True

    # send a pre-built JSON dict (additive fields)
    def send_dict(self, obj: dict):
        now = time.time()
        if (now - self.t_last) < self.min_dt:
            return False
        self._maybe_debug_print(obj)  # safe for dict/str/None
        try:
            pkt = json.dumps(obj).encode("utf-8")
            self.sock.sendto(pkt, self.addr)
            self.t_last = now
            return True
        except Exception:
            return False


# ----------------- main panel class ------------------
class YoloDeepSortPanel(tk.Frame):
    """
    Env knobs:
      YOLO_PANEL_UDP_DEBUG=1               # console prints
      YOLO_PANEL_UDP_DEBUG_KEYS=1          # print only keys
      YOLO_PANEL_UDP_DEBUG_HZ=10
      YOLO_PANEL_ALWAYS_VEH=1              # include minimal veh dict even when not locked
      YOLO_PANEL_DEFAULT_TYPE=vehicle      # type used when unlocked and unknown
    """
    def __init__(self, parent,
                 source="0", backend="auto", width=0, height=0, fps=0,
                 model_path="yolov8n.pt", classes_keep=(2,5,7),
                 host="127.0.0.1", port=47800, send_hz=20.0,
                 dz_rate=0.0, bias_y=120.0, ema=0.35):
        super().__init__(parent, bg="#0f0f0f", highlightthickness=1, highlightbackground="#333")

        # config
        self.src = str(source)
        self.backend = backend
        self.w_hint, self.h_hint, self.fps_hint = int(width), int(height), int(fps)
        self.model_path = model_path
        self.classes_keep = tuple(classes_keep) if classes_keep else tuple()
        self.allowed_set = set(self.classes_keep) if classes_keep else None

        # env behavior
        self.always_veh   = _truthy(os.getenv("YOLO_PANEL_ALWAYS_VEH", "0"))
        self.default_type = os.getenv("YOLO_PANEL_DEFAULT_TYPE", "vehicle").strip().lower() or "vehicle"

        # toggles
        self.show_boxes     = tk.BooleanVar(value=True)
        self.show_conf      = tk.BooleanVar(value=True)
        self.auto_lock      = tk.BooleanVar(value=True)
        self.vehicles_only  = tk.BooleanVar(value=bool(self.classes_keep))
        self.sending        = tk.BooleanVar(value=True)
        self.swap_rb        = tk.BooleanVar(value=True)   # BGR->RGB swap (default ON)
        self.tight_boxes    = tk.BooleanVar(value=True)   # draw using detections, not tracker

        # thresholds / dynamics
        self.conf_th = tk.DoubleVar(value=0.25)  # detection threshold (YOLO)
        self.dz_rate = float(dz_rate)
        self.bias_y  = float(bias_y)
        self.ema_a   = float(ema)

        # state
        self.dx_s = 0.0; self.dy_s = 0.0
        self.lock_id = None
        self._cap = None; self._model = None; self._names = None; self._tracker = None
        self._udp = _UdpTX(host, int(port), hz=float(send_hz))
        self._running = False
        self._frame_bgr = None
        self._imgtk = None
        self._boxes = []        # [(x1,y1,x2,y2, tid)]
        self._det_cache = []    # [(x1,y1,x2,y2, conf, cls)]
        self._conf_by_tid = {}  # tid -> last conf
        self._cls_by_tid  = {}  # tid -> last class id (int)
        self._fps_ema = 0.0
        self._last_map = (1.0, 1.0, 0, 0, 0, 0)  # (scx, scy, ox, oy, W, H)
        self._last_type = self.default_type

        # choose device and display it
        if torch is not None and torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"

        # jitter filter (tuned for straight-road stability)
        self._jitter = _JitterFilter(
            win=5,
            dead_enter_px=14,
            dead_exit_px=8,
            alpha=self.ema_a,
            max_step_px=40
        )

        # ------------- UI -------------
        head = tk.Frame(self, bg="#0f0f0f")
        head.pack(side="top", fill="x", padx=6, pady=(6, 4))
        ttk.Button(head, text="Start", command=self.start).pack(side="left")
        ttk.Button(head, text="Pause", command=self.pause).pack(side="left", padx=6)
        ttk.Button(head, text="Stop",  command=self.stop ).pack(side="left")
        ttk.Button(head, text="Cancel lock", command=self.cancel_lock).pack(side="left", padx=6)
        tk.Checkbutton(head, text="Send UDP", variable=self.sending, bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left", padx=(10,0))
        self.lbl_status = tk.Label(head, text="Idle", fg="#cfd8dc", bg="#0f0f0f")
        self.lbl_status.pack(side="right")

        row2 = tk.Frame(self, bg="#0f0f0f"); row2.pack(side="top", fill="x", padx=6, pady=2)
        tk.Checkbutton(row2, text="Boxes",        variable=self.show_boxes,    bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left")
        tk.Checkbutton(row2, text="Conf",         variable=self.show_conf,     bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left", padx=(8,0))
        tk.Checkbutton(row2, text="Auto-lock",    variable=self.auto_lock,     bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left", padx=(8,0))
        tk.Checkbutton(row2, text="Vehicles only",variable=self.vehicles_only, bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left", padx=(8,0))
        tk.Checkbutton(row2, text="Swap R-B",     variable=self.swap_rb,       bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left", padx=(8,0))
        tk.Checkbutton(row2, text="Tight boxes",  variable=self.tight_boxes,   bg="#0f0f0f", fg="#cfd8dc",
                       activebackground="#0f0f0f").pack(side="left", padx=(8,0))
        tk.Label(row2, text="Conf?", bg="#0f0f0f", fg="#cfd8dc").pack(side="left", padx=(10,2))
        ttk.Entry(row2, width=5, textvariable=self.conf_th).pack(side="left")
        self.lbl_lock = tk.Label(row2, text="lock: –", fg="#cfd8dc", bg="#0f0f0f")
        self.lbl_lock.pack(side="right")

        # --- Display filters (separate from detection conf) ---
        row3 = tk.Frame(self, bg="#0f0f0f"); row3.pack(side="top", fill="x", padx=6, pady=2)
        tk.Label(row3, text="Draw ≥", bg="#0f0f0f", fg="#cfd8dc").pack(side="left")
        self.draw_conf = tk.DoubleVar(value=float(self.conf_th.get()))
        ttk.Entry(row3, width=5, textvariable=self.draw_conf).pack(side="left", padx=(4,10))
        tk.Label(row3, text="Max boxes", bg="#0f0f0f", fg="#cfd8dc").pack(side="left")
        self.max_boxes = tk.IntVar(value=0)  # 0 = unlimited
        ttk.Entry(row3, width=5, textvariable=self.max_boxes).pack(side="left", padx=(4,4))
        tk.Label(row3, text="(0 = all)", bg="#0f0f0f", fg="#7f8b94").pack(side="left")

        # video area
        self.canvas = tk.Canvas(self, bg="#101314", highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand=True, padx=6, pady=(4,6))
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Configure>", lambda e: self._redraw())

        # loop
        self.after(60, self._loop)

    # ------------- public API -------------
    def set_udp_target(self, host, port, send_hz=None):
        try:
            self._udp.config(str(host), int(port), send_hz)
        except Exception:
            pass

    def start(self):
        if self._running:
            return
        if YOLO is None or DeepSort is None:
            self.lbl_status.config(text="Missing ultralytics or deep-sort-realtime")
            return
        try:
            self._cap = _open_capture(self.src, self.backend, self.w_hint, self.h_hint, self.fps_hint)
            self._model = YOLO(self.model_path)
            self._names = self._model.model.names if hasattr(self._model, "model") else self._model.names
            # Tracker (slightly more tolerant so lock doesn't drop too fast)
            self._tracker = DeepSort(
                max_age=40, n_init=3, nms_max_overlap=1.0,
                max_cosine_distance=0.25, embedder="mobilenet"
            )
            self._running = True
            print(f"[AI] device={self.device}  model={self.model_path}")
        except Exception as e:
            self.lbl_status.config(text=f"Init error: {e}")
            self._running = False

    def pause(self):
        self._running = False
        self.lbl_status.config(text="Paused")

    def stop(self):
        self._running = False
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        self._cap = None
        self._frame_bgr = None
        self._imgtk = None
        self._boxes.clear()
        self._det_cache.clear()
        self._conf_by_tid.clear()
        self._cls_by_tid.clear()
        self.lock_id = None
        self.dx_s = self.dy_s = 0.0
        # reset the jitter filter, too
        self._jitter = _JitterFilter(alpha=self.ema_a)
        self._redraw()
        self.lbl_status.config(text="Stopped")

    def cancel_lock(self):
        """Cancel current lock and prevent immediate re-lock."""
        self.lock_id = None
        try:
            self.auto_lock.set(False)  # avoid instant re-lock
        except Exception:
            pass
        # reset smoothing so dx/dy decay cleanly
        self._jitter = _JitterFilter(alpha=self.ema_a)
        self.dx_s = self.dy_s = 0.0
        if self.sending.get():
            pkt = {
                "dx": float(self.dx_s),
                "dy": float(self.dy_s),
                "dz_rate": float(self.dz_rate),
                "locked": False,
                "conf": 0.0,
                "tid": -1,
                "fps": float(self._fps_ema) if self._fps_ema else None,
            }
            # include minimal veh if requested
            if self.always_veh:
                pkt["veh"] = {
                    "type": self._last_type or self.default_type,
                    "cx_n": 0.5, "cy_n": 0.5, "w_n": 0.0, "h_n": 0.0,
                    "tid": -1, "conf": 0.0, "ts": time.time()
                }
            self._udp.send_dict(pkt)
        self.lbl_lock.config(text="lock: –")
        self._redraw()

    # ------------- loop & processing -------------
    def _loop(self):
        if self._running and self._cap is not None:
            ok, frame = self._cap.read()
            if ok:
                self._process_frame(frame)
        self.after(10, self._loop)

    def _process_frame(self, frame_bgr):
        t0 = time.time()
        H, W = frame_bgr.shape[:2]
        self._frame_bgr = frame_bgr

        # YOLO inference
        conf_th = float(self.conf_th.get())
        res = self._model.predict(frame_bgr, imgsz=640, conf=conf_th,
                                  device=self.device, verbose=False)[0]

        # collect detections
        dets_for_ds = []
        det_cache = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clss  = res.boxes.cls.cpu().numpy().astype(int)
            for (x1,y1,x2,y2), c, k in zip(xyxy, confs, clss):
                if self.vehicles_only.get() and self.allowed_set is not None and int(k) not in self.allowed_set:
                    continue
                dets_for_ds.append(([float(x1), float(y1), float(x2), float(y2)], float(c), str(int(k))))
                det_cache.append((float(x1), float(y1), float(x2), float(y2), float(c), int(k)))
        self._det_cache = det_cache

        # update tracker
        tracks = self._tracker.update_tracks(dets_for_ds, frame=frame_bgr)

        # --- Build display boxes (prefer YOLO dets), attach conf & class by IoU ---
        boxes = []                 # list of (x1,y1,x2,y2, tid)
        conf_by_tid = {}
        cls_by_tid  = {}
        cx0, cy0 = W/2.0, H/2.0
        candidates = []            # (tid, tlbr_box, cx, cy, d2, area)

        for tr in tracks:
            if not tr.is_confirmed() or tr.time_since_update > 0:
                continue

            # get tracker box in TLBR from TLWH
            tx, ty, tw, th = tr.to_tlwh()
            tb = (float(tx), float(ty), float(tx + tw), float(ty + th))
            tid = tr.track_id

            # find best matching YOLO detection for a tight box
            best_det = None
            best_iou = 0.0
            for (dx1, dy1, dx2, dy2, dc, dk) in det_cache:
                iou = _iou_xyxy(tb, (dx1, dy1, dx2, dy2))
                if iou > best_iou:
                    best_iou, best_det = iou, (dx1, dy1, dx2, dy2, dc, dk)

            # prefer detection for the LOCKED tid (prevents "double-wide" tracker box)
            use_det = False
            if best_det:
                if tid == self.lock_id:
                    use_det = True
                elif self.tight_boxes.get() and best_iou >= 0.05:
                    use_det = True

            if use_det:
                x1, y1, x2, y2, det_conf, det_cls = best_det
                boxes.append((x1, y1, x2, y2, tid))
                conf_by_tid[tid] = max(conf_by_tid.get(tid, 0.0), float(det_conf))
                cls_by_tid[tid]  = int(det_cls)
            else:
                x1, y1, x2, y2 = tb
                boxes.append((x1, y1, x2, y2, tid))

            # distance-to-center for auto-lock
            cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
            area = max(1.0, (x2 - x1) * (y2 - y1))
            d2 = (cx - cx0) ** 2 + (cy - cy0) ** 2
            candidates.append((tid, (x1, y1, x2, y2), cx, cy, d2, area))

        # keep conf & class for TIDs across frames
        for tid, c in conf_by_tid.items():
            self._conf_by_tid[tid] = c
        for tid, k in cls_by_tid.items():
            self._cls_by_tid[tid] = k
        for tid in list(self._conf_by_tid.keys()):
            if tid not in [b[4] for b in boxes]:
                self._conf_by_tid.pop(tid, None)
                self._cls_by_tid.pop(tid, None)

        # auto-lock: nearest to center, penalize tiny boxes
        if self.auto_lock.get() and candidates:
            candidates.sort(key=lambda t: (t[4] + 0.0002 * (H * W) / t[5]))
            self.lock_id = candidates[0][0]

        # compute dx/dy (raw), then pass through the robust jitter filter
        had_lock = False
        veh_payload = None
        if self.lock_id is not None:
            for tid, box, cx, cy, *_ in candidates:
                if tid == self.lock_id:
                    had_lock = True
                    dx_raw = (cx - cx0)
                    dy_raw = (cy - cy0) - self.bias_y  # forward bias subtract
                    dx_f, dy_f = self._jitter.update(dx_raw, dy_raw)
                    self.dx_s, self.dy_s = dx_f, dy_f

                    # Build vehicle payload (normalized to the original frame)
                    x1, y1, x2, y2 = box
                    w = max(1e-6, x2 - x1)
                    h = max(1e-6, y2 - y1)
                    clsid = int(self._cls_by_tid.get(self.lock_id, 2))
                    vtype = _class_to_type(clsid)
                    self._last_type = vtype or self._last_type
                    conf  = float(self._conf_by_tid.get(self.lock_id, 0.0))
                    veh_payload = {
                        "type": vtype,
                        "cx_n": float(cx / W),
                        "cy_n": float(cy / H),
                        "w_n":  float(w / W),
                        "h_n":  float(h / H),
                        "tid":  int(self.lock_id),
                        "conf": conf,
                        "ts":   time.time(),
                    }

                    if self.sending.get():
                        pkt = {
                            "dx": float(self.dx_s),
                            "dy": float(self.dy_s),
                            "dz_rate": float(self.dz_rate),
                            "locked": True,
                            "conf": conf,
                            "tid": int(self.lock_id),
                            "fps": float(self._fps_ema) if self._fps_ema else None,
                            "veh": veh_payload,
                        }
                        self._udp.send_dict(pkt)
                    break

        if not had_lock:
            # decay to zero smoothly when no lock
            dx_f, dy_f = self._jitter.update(0.0, 0.0)
            self.dx_s, self.dy_s = dx_f, dy_f
            if self.sending.get():
                pkt = {
                    "dx": float(self.dx_s),
                    "dy": float(self.dy_s),
                    "dz_rate": float(self.dz_rate),
                    "locked": False,
                    "conf": 0.0,
                    "tid": -1,
                    "fps": float(self._fps_ema) if self._fps_ema else None,
                }
                # include minimal veh when requested so the sim can still render an icon
                if self.always_veh:
                    pkt["veh"] = {
                        "type": self._last_type or self.default_type,
                        "cx_n": 0.5, "cy_n": 0.5, "w_n": 0.0, "h_n": 0.0,
                        "tid": -1, "conf": 0.0, "ts": time.time(),
                    }
                self._udp.send_dict(pkt)

        self._boxes = boxes

        # fps label
        dt = max(1e-3, time.time() - t0)
        inst = 1.0 / dt
        self._fps_ema = 0.9 * self._fps_ema + 0.1 * inst if self._fps_ema > 0 else inst
        self.lbl_status.config(text=f"{'Running' if self._running else 'Paused'} | {self.device} | {self._fps_ema:4.1f} FPS")

        # draw
        self._redraw()

    # ------------- drawing -------------
    def _redraw(self):
        c = self.canvas
        c.delete("all")
        if self._frame_bgr is None:
            return

        frame = self._frame_bgr
        H, W = frame.shape[:2]
        cw = max(1, c.winfo_width())
        ch = max(1, c.winfo_height())

        # letterbox mapping (store BOTH scales, not a single scale)
        scx = cw / float(W)
        scy = ch / float(H)
        scale = min(scx, scy)
        dw, dh = int(W * scale), int(H * scale)
        ox, oy = (cw - dw) // 2, (ch - dh) // 2
        # true per-axis scales for mapping xy
        scx, scy = dw / float(W), dh / float(H)
        self._last_map = (scx, scy, ox, oy, W, H)

        # resize for display
        disp = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_LINEAR)

        # convert to RGB for Tk — allow user to toggle swap if their source is already RGB
        if Image is not None and ImageTk is not None:
            if self.swap_rb.get():
                rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
            else:
                rgb = disp.copy()
            img = Image.fromarray(rgb)
            self._imgtk = ImageTk.PhotoImage(img)
        else:
            rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB) if self.swap_rb.get() else disp
            png = cv2.imencode(".png", rgb)[1].tobytes()
            self._imgtk = tk.PhotoImage(data=png)

        # draw image
        c.create_image(ox, oy, anchor="nw", image=self._imgtk)

        # draw boxes on Tk canvas using the SAME mapping
        if self.show_boxes.get():
            base_set = self._boxes
            locked_tid = self.lock_id

            # If locked box exists this frame, prefer drawing ONLY that box
            locked_only = [b for b in base_set if locked_tid is not None and b[4] == locked_tid]
            if locked_tid is not None and locked_only:
                draw_set = locked_only
            else:
                if locked_tid is not None and not locked_only:
                    # locked track missing this frame → show all, drop lock
                    self.lock_id = None
                    locked_tid = None
                # Apply display threshold + top-K
                thr = float(getattr(self, "draw_conf", tk.DoubleVar(value=0.0)).get() or 0.0)
                kmax = int(getattr(self, "max_boxes", tk.IntVar(value=0)).get() or 0)

                items = []
                for (x1, y1, x2, y2, tid) in base_set:
                    cval = float(self._conf_by_tid.get(tid, 0.0))
                    if cval >= thr:
                        items.append((cval, x1, y1, x2, y2, tid))

                items.sort(key=lambda it: it[0], reverse=True)
                if kmax > 0:
                    items = items[:kmax]
                draw_set = [(x1, y1, x2, y2, tid) for (_, x1, y1, x2, y2, tid) in items]

            # Now actually draw
            scx, scy, ox, oy, W, H = self._last_map
            dw = int(W * scx); dh = int(H * scy)
            for (x1,y1,x2,y2, tid) in draw_set:
                X1 = int(round(ox + x1 * scx)); Y1 = int(round(oy + y1 * scy))
                X2 = int(round(ox + x2 * scx)); Y2 = int(round(oy + y2 * scy))
                locked = (tid == self.lock_id)

                # slightly widen locked box (keep subtle)
                if locked:
                    w = X2 - X1; h = Y2 - Y1
                    X1 -= int(0.06 * w); X2 += int(0.06 * w)
                    Y1 -= int(0.03 * h); Y2 += int(0.03 * h)

                # clamp to the display rect
                X1 = max(ox, X1); Y1 = max(oy, Y1)
                X2 = min(ox + dw - 1, X2); Y2 = min(oy + dh - 1, Y2)

                color = "#ff3333" if locked else "#50de78"
                width = 3 if locked else 2
                c.create_rectangle(X1, Y1, X2, Y2, outline=color, width=width)

                if locked:
                    cx = (X1 + X2) // 2; cy = (Y1 + Y2) // 2
                    c.create_line(cx-8, cy, cx+8, cy, fill=color, width=2)
                    c.create_line(cx, cy-8, cx, cy+8, fill=color, width=2)

                # confidence INSIDE the box (bottom-left)
                if self.show_conf.get():
                    conf = self._conf_by_tid.get(tid, 0.0)
                    if conf > 0:
                        c.create_text(X1 + 4, Y2 - 6, anchor="sw",
                                      text=f"{conf:.2f}", fill="#e8e8e8", font=("Arial", 9))

        # overlay: device & FPS
        c.create_text(ox + 8, oy + 12, anchor="w",
                      text=f"{self.device} | {self._fps_ema:4.1f} FPS",
                      fill="#e0e0e0", font=("Arial", 10, "bold"))

        # lock label
        self.lbl_lock.config(text=f"lock: {self.lock_id if self.lock_id is not None else '–'}")

    # ------------- mouse pick -------------
    def _on_click(self, ev):
        if self._frame_bgr is None or not self._boxes:
            return
        scx, scy, ox, oy, W, H = self._last_map
        # map click to original frame coords (use scx/scy!)
        x = (ev.x - ox) / max(1e-6, scx)
        y = (ev.y - oy) / max(1e-6, scy)

        # choose by containment first, else best IoU with tiny 3x3 box
        best, best_score = None, -1.0
        for (x1,y1,x2,y2, tid) in self._boxes:
            if x1 <= x <= x2 and y1 <= y <= y2:
                self.lock_id = tid
                self._redraw()
                return
            px1, py1, px2, py2 = x-1.5, y-1.5, x+1.5, y+1.5
            inter = _iou_xyxy((x1,y1,x2,y2), (px1,py1,px2,py2))
            if inter > best_score:
                best_score, best = inter, tid
        self.lock_id = best
        self._redraw()
