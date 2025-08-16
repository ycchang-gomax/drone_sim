# ai/yolo_deepsort_udp.py
# - YOLOv8 + DeepSORT (Kalman + ReID)
# - UDP {dx, dy, dz_rate} -> your simulator
# - Choose a single car to track (click), or auto-lock nearest
# - Start / Pause / Stop buttons (and hotkeys)
# - Toggle confidence on/off, show only locked box (default)
# - Can run as a CLI window OR be embedded as a Tkinter panel

import argparse, time, json, socket, sys, re, threading
from collections import deque

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except Exception:
    print("Missing ultralytics. pip install ultralytics", file=sys.stderr); raise
try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
except Exception:
    print("Missing deep-sort-realtime. pip install deep-sort-realtime", file=sys.stderr); raise


# ---------------- UDP ----------------
class UdpTX:
    def __init__(self, host="127.0.0.1", port=47800, hz=20.0):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.min_dt = 1.0 / float(hz)
        self.t_last = 0.0

    def send(self, dx, dy, dz_rate):
        now = time.time()
        if (now - self.t_last) < self.min_dt:
            return False
        pkt = json.dumps({"dx": float(dx), "dy": float(dy), "dz_rate": float(dz_rate)}).encode("utf-8")
        self.sock.sendto(pkt, self.addr)
        self.t_last = now
        return True


# ---------------- Video ----------------
def open_capture(src, backend='auto', w=0, h=0, fps=0):
    """OpenCV VideoCapture with backend/size options."""
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

    ok = cap.isOpened()
    print(f"[cam] opened={ok} backend={backend} size=({w}x{h}) fps={fps} src={src}")
    if not ok:
        raise SystemExit(f"ERROR: cannot open video source: {src}")
    return cap


# ---------------- Core tracker ----------------
def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0: return 0.0
    area = (ax2-ax1)*(ay2-ay1) + (bx2-bx1)*(by2-by1) - inter
    return inter / max(1e-6, area)

def ema(prev, x, a):  # exponential smoothing
    return a * x + (1.0 - a) * prev


class TrackerCore:
    """Model + DeepSORT + lock + UDP. Can be driven from CLI or a Tk panel."""
    def __init__(self,
                 source="0", backend="auto", width=0, height=0, fps=0,
                 model_path="yolov8n.pt", imgsz=640, conf=0.25, device=None,
                 classes=(2,5,7),                 # COCO: car=2 bus=5 truck=7
                 host="127.0.0.1", port=47800, send_hz=20.0,
                 dz_rate=0.0, gainx=1.0, gainy=1.0, bias_y=0.0, ema_a=0.35):

        self.cap = open_capture(source, backend, width, height, fps)
        self.W = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)  or 1280)
        self.H = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 720)
        self.cx0, self.cy0 = self.W/2.0, self.H/2.0

        self.model = YOLO(model_path)
        # accept either list or comma string for class IDs
        if isinstance(classes, (list, tuple)):
            self.allowed_ids = set(int(k) for k in classes)
        else:
            self.allowed_ids = set(int(s) for s in re.split(r"[,\s]+", str(classes).strip()) if s)

        self.imgsz = int(imgsz)
        self.conf  = float(conf)
        self.device = device

        self.tracker = DeepSort(max_age=30, n_init=3, nms_max_overlap=1.0,
                                max_cosine_distance=0.2, embedder="mobilenet")

        self.udp = UdpTX(host, port, hz=send_hz)

        # state
        self.auto_lock = True
        self.lock_id   = None
        self.sending   = True
        self.show_conf = False
        self.show_all  = False  # draw only the locked car by default

        self.dz_rate = float(dz_rate)
        self.gainx   = float(gainx)
        self.gainy   = float(gainy)
        self.bias_y  = float(bias_y)
        self.ema_a   = float(ema_a)
        self.dx_s, self.dy_s = 0.0, 0.0

        self.fps_hist = deque(maxlen=30)
        self.udp_hist = deque(maxlen=30)
        self.last_tracks = []

        self._stop = False

    def stop(self):
        self._stop = True

    def toggle_send(self):
        self.sending = not self.sending

    def reset_lock(self):
        self.lock_id = None

    def click_lock(self, x, y):
        """Pick the track whose box contains the click (or nearest via IoU)."""
        if not self.last_tracks: return
        best, best_iou = None, 0.0
        pt_box = [x-3, y-3, x+3, y+3]
        for tr in self.last_tracks:
            iou = iou_xyxy(pt_box, tr.to_tlbr())
            if iou > best_iou:
                best_iou, best = iou, tr.track_id
        if best is not None:
            self.lock_id = best

    def _send_cmd(self, dx, dy):
        if self.sending and self.udp.send(dx, dy, self.dz_rate):
            self.udp_hist.append(1.0)

    def step(self):
        """Process one frame. Returns a drawn frame for UI."""
        t0 = time.time()
        ok, frame = self.cap.read()
        if not ok:
            return None

        # YOLO
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                 device=self.device, verbose=False)[0]

        dets = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            conf = res.boxes.conf.cpu().numpy()
            cls  = res.boxes.cls.cpu().numpy().astype(int)
            for (x1,y1,x2,y2), c, k in zip(xyxy, conf, cls):
                if int(k) in self.allowed_ids:
                    dets.append(([float(x1), float(y1), float(x2), float(y2)], float(c), int(k)))

        tracks = self.tracker.update_tracks(dets, frame=frame)
        self.last_tracks = [tr for tr in tracks if tr.is_confirmed() and tr.time_since_update == 0]

        # choose/maintain lock
        candidates = []
        for tr in self.last_tracks:
            x1, y1, x2, y2 = tr.to_tlbr()
            cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
            area = max(1.0, (x2-x1)*(y2-y1))
            d2 = (cx-self.cx0)**2 + (cy-self.cy0)**2
            candidates.append((tr.track_id, (x1,y1,x2,y2), cx, cy, d2, area))

        if self.auto_lock and candidates:
            candidates.sort(key=lambda t: (t[4] + 0.0002*(self.H*self.W)/t[5]))
            self.lock_id = candidates[0][0]

        # compute command
        had_lock = False
        for tid, box, cx, cy, *_ in candidates:
            if self.lock_id == tid:
                had_lock = True
                dx = (cx - self.cx0) * self.gainx
                dy = (cy - self.cy0) * self.gainy
                dy = dy - self.bias_y  # forward bias (push target "up")

                a = float(self.ema_a)
                self.dx_s = ema(self.dx_s, dx, a) if a>0 else dx
                self.dy_s = ema(self.dy_s, dy, a) if a>0 else dy
                self._send_cmd(self.dx_s, self.dy_s)
                break

        if not had_lock:
            a = float(self.ema_a)
            self.dx_s = ema(self.dx_s, 0.0, a) if a>0 else 0.0
            self.dy_s = ema(self.dy_s, 0.0, a) if a>0 else 0.0
            self._send_cmd(self.dx_s, self.dy_s)

        # draw
        if self.show_all:
            for tr in self.last_tracks:
                x1, y1, x2, y2 = map(int, tr.to_tlbr())
                is_lock = (tr.track_id == self.lock_id)
                c = (50, 220, 120) if is_lock else (80, 160, 255)
                cv2.rectangle(frame, (x1,y1), (x2,y2), c, 2)
                if self.show_conf:
                    cv2.putText(frame, f"ID {tr.track_id}", (x1, y1-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
        else:
            # draw only the locked one
            for tr in self.last_tracks:
                if tr.track_id != self.lock_id:
                    continue
                x1, y1, x2, y2 = map(int, tr.to_tlbr())
                c = (50, 220, 120)
                cv2.rectangle(frame, (x1,y1), (x2,y2), c, 2)
                if self.show_conf:
                    cv2.putText(frame, f"ID {tr.track_id}", (x1, y1-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)

        # center + vector
        cv2.drawMarker(frame, (int(self.cx0), int(self.cy0)), (255,255,0),
                       cv2.MARKER_CROSS, 22, 1)
        cv2.arrowedLine(frame, (int(self.cx0), int(self.cy0)),
                        (int(self.cx0+self.dx_s), int(self.cy0+self.dy_s)),
                        (255,180,60), 2, tipLength=0.18)

        # HUD
        fps = (1.0 / max(1e-3, time.time()-t0))
        self.fps_hist.append(fps)
        u = (sum(self.udp_hist)/max(1,len(self.udp_hist))) / (1/ self.udp.min_dt) if self.udp.min_dt>0 else 0.0
        txt = (f"FPS {sum(self.fps_hist)/max(1,len(self.fps_hist)):4.1f} | UDP~{u:4.1f} Hz | "
               f"lock={self.lock_id} | dx={self.dx_s:+5.0f}px dy={self.dy_s:+5.0f}px "
               f"biasY={self.bias_y:.0f}  "
               f"[Send={'ON' if self.sending else 'OFF'}  All={'ON' if self.show_all else 'OFF'}  Conf={'ON' if self.show_conf else 'OFF'}]")
        cv2.putText(frame, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240,240,240), 2, cv2.LINE_AA)

        return frame


# ---------------- CLI runner (OpenCV window) ----------------
def run_cli(args):
    core = TrackerCore(
        source=args.source, backend=args.backend, width=args.width, height=args.height, fps=args.fps,
        model_path=args.model, imgsz=args.imgsz, conf=args.conf, device=args.device,
        classes=args.classes, host=args.host, port=args.port, send_hz=args.send_hz,
        dz_rate=args.dz_rate, gainx=args.gainx, gainy=args.gainy, bias_y=args.bias_y, ema_a=args.ema
    )

    win = "AI Vehicles (click to lock)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            core.click_lock(x, y)
    cv2.setMouseCallback(win, on_mouse)

    print("Hotkeys:  [SPACE]=Pause/Resume send,  [S]=Stop(reset lock),  [A]=Auto-lock on/off,"
          "  [V]=Show all boxes on/off,  [C]=Conf text on/off,  [Q/ESC]=Quit")

    while True:
        frame = core.step()
        if frame is None:
            break
        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xff
        if key in (ord('q'), 27): break
        elif key == 32: core.toggle_send()                 # space
        elif key in (ord('s'), ord('S')): core.reset_lock()
        elif key in (ord('a'), ord('A')): core.auto_lock = not core.auto_lock
        elif key in (ord('v'), ord('V')): core.show_all  = not core.show_all
        elif key in (ord('c'), ord('C')): core.show_conf = not core.show_conf

    core.cap.release()
    cv2.destroyAllWindows()


# ---------------- Tkinter panel (embed into GUI #2) ----------------
# Usage inside app_ai.py:
#   from ai.yolo_deepsort_udp import YoloDeepSortPanel
#   self.ai_panel = YoloDeepSortPanel(parent_frame, source="0", backend="dshow", width=1280, height=720,
#                                     model="yolov8n.pt", host="127.0.0.1", port=47800)
#   self.ai_panel.place(x=8, y=..., width=..., height=...)  # bottom-left dock
try:
    import tkinter as tk
    from PIL import Image, ImageTk
    TK_OK = True
except Exception:
    TK_OK = False

class YoloDeepSortPanel(tk.Frame):
    def __init__(self, master, **kwargs):
        if not TK_OK:
            raise RuntimeError("Tkinter/Pillow not available to embed panel.")
        super().__init__(master, bg="#151515")

        # create core
        self.core = TrackerCore(
            source=kwargs.get("source", "0"),
            backend=kwargs.get("backend", "auto"),
            width=kwargs.get("width", 0),
            height=kwargs.get("height", 0),
            fps=kwargs.get("fps", 0),
            model_path=kwargs.get("model", "yolov8n.pt"),
            imgsz=kwargs.get("imgsz", 640),
            conf=kwargs.get("conf", 0.25),
            device=kwargs.get("device", None),
            classes=kwargs.get("classes", (2,5,7)),
            host=kwargs.get("host", "127.0.0.1"),
            port=kwargs.get("port", 47800),
            send_hz=kwargs.get("send_hz", 20.0),
            dz_rate=kwargs.get("dz_rate", 0.0),
            gainx=kwargs.get("gainx", 1.0),
            gainy=kwargs.get("gainy", 1.0),
            bias_y=kwargs.get("bias_y", 0.0),
            ema_a=kwargs.get("ema", 0.35),
        )

        # video area
        self.lbl = tk.Label(self, bg="#000000")
        self.lbl.pack(side="top", fill="both", expand=True)

        # controls row
        bar = tk.Frame(self, bg="#1d1d1d")
        bar.pack(side="top", fill="x")
        self.btn_start = tk.Button(bar, text="Start", command=self._on_start)
        self.btn_pause = tk.Button(bar, text="Pause", command=self._on_pause)
        self.btn_stop  = tk.Button(bar, text="Stop",  command=self._on_stop)
        for b in (self.btn_start, self.btn_pause, self.btn_stop):
            b.pack(side="left", padx=4, pady=3)

        self.var_show_all  = tk.BooleanVar(value=False)
        self.var_show_conf = tk.BooleanVar(value=False)
        tk.Checkbutton(bar, text="Show all",  var=self.var_show_all,
                       command=self._toggle_all, bg="#1d1d1d", fg="#ddd",
                       selectcolor="#1d1d1d").pack(side="left", padx=8)
        tk.Checkbutton(bar, text="Conf",      var=self.var_show_conf,
                       command=self._toggle_conf, bg="#1d1d1d", fg="#ddd",
                       selectcolor="#1d1d1d").pack(side="left")

        # click to lock
        self.lbl.bind("<Button-1>", self._on_click)

        self._running = False
        self._img = None
        self.after(50, self._loop)

    def _on_click(self, ev):
        # map to image coords (label is 1:1 since we feed actual frames)
        self.core.click_lock(ev.x, ev.y)

    def _on_start(self):
        self._running = True
        self.core.sending = True

    def _on_pause(self):
        # pause sending but keep display
        self.core.toggle_send()

    def _on_stop(self):
        # stop sending and reset lock
        self.core.sending = False
        self.core.reset_lock()

    def _toggle_all(self):
        self.core.show_all = bool(self.var_show_all.get())

    def _toggle_conf(self):
        self.core.show_conf = bool(self.var_show_conf.get())

    def _loop(self):
        if self._running:
            frame = self.core.step()
            if frame is not None:
                # convert BGR->RGB for Tk
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                im = Image.fromarray(img)
                self._img = ImageTk.PhotoImage(im)
                self.lbl.config(image=self._img)
        self.after(15, self._loop)


# ---------------- main ----------------
def parse_args():
    ap = argparse.ArgumentParser("YOLOv8 + DeepSORT vehicle tracker -> UDP")
    ap.add_argument("--source", default="0", help="0/webcam or path/rtsp")
    ap.add_argument("--backend", type=str, default="auto", choices=["auto","dshow","msmf"])
    ap.add_argument("--width", type=int, default=0)
    ap.add_argument("--height", type=int, default=0)
    ap.add_argument("--fps", type=int, default=0)

    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default=None)

    ap.add_argument("--classes", type=int, nargs="+", default=[2,5,7],
                    help="COCO IDs (car=2 bus=5 truck=7)")

    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=47800)
    ap.add_argument("--send_hz", type=float, default=20.0)
    ap.add_argument("--dz_rate", type=float, default=0.0)
    ap.add_argument("--gainx", type=float, default=1.0)
    ap.add_argument("--gainy", type=float, default=1.0)
    ap.add_argument("--bias_y", type=float, default=0.0)
    ap.add_argument("--ema", type=float, default=0.35)

    ap.add_argument("--show", action="store_true", help="OpenCV window mode")
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    # Default: CLI window
    run_cli(args)
