# ui/video_panel.py
import time
import tkinter as tk
from tkinter import filedialog
from typing import Callable, Optional, Tuple, List

from PIL import Image, ImageTk

# Optional deps: OpenCV + NumPy
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None


class VideoPanel(tk.Frame):
    """
    Video widget that:
      - Opens/plays/pauses/stops local video files (MP4, AVI, MOV, ...).
      - Loops automatically at EOF.
      - Renders frames using 'cover' scaling to fill all available space (no letter/pillar boxes).
      - Applies per-frame 'virtual camera' effects before display:
          * zoom (digital), pan/aim shift (px), roll (deg)
        Effects can be set directly (set_effects/set_zoom/set_aim/set_roll) or supplied
        by a callback via set_effects_provider(fn) that returns a dict every frame:
            {"zoom": float>=1, "shift": (dx_px, dy_px), "roll": deg}
      - Overlays a status line. (Boxes overlay kept simple; disabled during heavy transforms.)
      - Exposes get_frame_for_ai() to fetch the CURRENT displayed RGB frame (NumPy array)
        for your detection loop; that 'sensor' frame is what the AI should operate on.
    """

    # --------------------------- Lifecycle ---------------------------------

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg="#000", **kwargs)

        # Display surface
        self._img_label = tk.Label(self, bg="#000", bd=0, highlightthickness=0)
        self._img_label.pack(fill=tk.BOTH, expand=True)

        # Playback state
        self._cap = None
        self._src_path = None
        self._fps = 25.0
        self._play = False
        self._speed = 1.0
        self._loop = True
        self._job = None
        self._last_tick = 0.0

        # Effects state (targets + smoothed actuals)
        self._zoom_t = 1.0
        self._shift_t = (0.0, 0.0)   # px relative to frame center: (+x right, +y down)
        self._roll_t = 0.0           # degrees
        self._zoom = 1.0
        self._shift = (0.0, 0.0)
        self._roll = 0.0
        self._smooth = 0.18          # EMA alpha for effects

        # Optional provider called each paint: fn() -> dict(zoom, shift, roll)
        self._effects_provider: Optional[Callable[[], dict]] = None

        # Overlays / status
        self._aim_dx = 0.0
        self._aim_dy = 0.0
        self._boxes: List[Tuple[int, int, int, int]] = []
        self._draw_boxes = True
        self._rec_on = False
        self._rec_path = ""
        self._fit_mode = "cover"     # 'cover' fills & crops; 'contain' letter/pillar boxes
        self._tk_img = None
        self._last_panel_frame: Optional["np.ndarray"] = None  # RGB ndarray after effects & scaling

        # Repaint on resize
        self.bind("<Configure>", lambda e: self._request_repaint())

    # --------------------------- Public API --------------------------------

    # File controls
    def open_file_dialog(self):
        path = filedialog.askopenfilename(
            title="Open Video",
            filetypes=[("Video files", "*.mp4 *.avi *.mov *.mkv *.m4v;*.MP4;*.AVI;*.MOV;*.MKV;*.M4V"),
                       ("All files", "*.*")]
        )
        if path:
            self.open(path)

    def open(self, path: str):
        self.stop()
        self._src_path = path
        if cv2 is None:
            self._show_text("OpenCV not available")
            return
        self._cap = cv2.VideoCapture(path)
        if not self._cap or not self._cap.isOpened():
            self._cap = None
            self._show_text("Failed to open video")
            return
        fps = self._cap.get(cv2.CAP_PROP_FPS)
        if fps and fps > 0.1:
            self._fps = float(fps)
        self._play = True
        self._last_tick = 0.0
        self._request_repaint()

    def toggle_play(self):
        if self._cap is None:
            return
        self._play = not self._play
        self._request_repaint()

    def stop(self):
        self._play = False
        if self._job is not None:
            try:
                self.after_cancel(self._job)
            except Exception:
                pass
            self._job = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        self._cap = None
        self._src_path = None
        self._last_panel_frame = None
        self._show_text("No video")

    def set_loop(self, enabled: bool):
        self._loop = bool(enabled)

    def set_speed(self, speed: float):
        try:
            s = float(speed)
            if s > 0:
                self._speed = s
        except Exception:
            pass

    # Effects controls (direct)
    def set_zoom(self, zoom: float):
        try:
            z = max(1.0, float(zoom))
            self._zoom_t = z
        except Exception:
            pass

    def set_aim(self, dx_px: float, dy_px: float):
        # (1) for status line; (2) also sets pan/shift target
        try:
            dx = float(dx_px); dy = float(dy_px)
            self._aim_dx, self._aim_dy = dx, dy
            self._shift_t = (dx, dy)
        except Exception:
            self._aim_dx = self._aim_dy = 0.0

    def set_roll(self, deg: float):
        try:
            self._roll_t = float(deg)
        except Exception:
            pass

    def set_effects(self, zoom: Optional[float] = None,
                    shift: Optional[Tuple[float, float]] = None,
                    roll: Optional[float] = None):
        if zoom is not None:
            self.set_zoom(zoom)
        if shift is not None:
            self._shift_t = (float(shift[0]), float(shift[1]))
        if roll is not None:
            self.set_roll(roll)

    # Effects provider
    def set_effects_provider(self, fn: Optional[Callable[[], dict]]):
        """
        fn should return any of: {"zoom": float>=1, "shift": (dx,dy), "roll": deg}
        Called each paint. Missing keys leave current targets unchanged.
        """
        self._effects_provider = fn

    # Overlays & options
    def set_fit_mode(self, mode: str):
        self._fit_mode = "cover" if (mode or "").lower() != "contain" else "contain"

    def set_boxes(self, boxes: List[Tuple[int, int, int, int]], draw=True):
        self._boxes = list(boxes or [])
        self._draw_boxes = bool(draw)

    def enable_record_badge(self, enabled: bool, filebase: str = ""):
        self._rec_on = bool(enabled)
        self._rec_path = filebase or ""

    # “Sensor” output for AI
    def get_frame_for_ai(self) -> Optional["np.ndarray"]:
        """
        Returns the most recently displayed RGB frame (NumPy array) or None.
        Use this in your AI loop to analyze the exact image the human sees.
        """
        return None if self._last_panel_frame is None else self._last_panel_frame.copy()

    # --------------------------- Internals ---------------------------------

    def _request_repaint(self):
        if self._job is None:
            self._job = self.after(10, self._paint_loop)

    def _paint_loop(self):
        self._job = None  # allow re-schedule

        rgb_src = None
        if self._cap is not None and self._play and cv2 is not None:
            # Timing
            now = time.time()
            interval = 1.0 / max(1e-3, self._fps * self._speed)
            if self._last_tick == 0.0 or (now - self._last_tick) >= interval:
                ok, frame_bgr = self._cap.read()
                self._last_tick = now
                if not ok or frame_bgr is None:
                    # EOF: loop or pause
                    if self._loop:
                        self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ok, frame_bgr = self._cap.read()
                        if not ok or frame_bgr is None:
                            self._play = False
                    else:
                        self._play = False
                        frame_bgr = None
                if frame_bgr is not None:
                    rgb_src = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # No new frame? render black pane w/ status line
        if rgb_src is None:
            w, h = max(2, self.winfo_width()), max(2, self.winfo_height())
            if np is not None:
                rgb_src = np.zeros((h, w, 3), dtype=np.uint8)
            else:
                self._show_text("No video")
                self._schedule_next()
                return

        # Pull dynamic effects from provider (if any)
        if callable(self._effects_provider):
            try:
                eff = self._effects_provider() or {}
                if "zoom" in eff and eff["zoom"] is not None:
                    self._zoom_t = max(1.0, float(eff["zoom"]))
                if "shift" in eff and eff["shift"] is not None:
                    sx, sy = eff["shift"]
                    self._shift_t = (float(sx), float(sy))
                if "roll" in eff and eff["roll"] is not None:
                    self._roll_t = float(eff["roll"])
            except Exception:
                pass

        # Smooth effects
        a = float(self._smooth)
        self._zoom = (1 - a) * self._zoom + a * self._zoom_t
        sx = (1 - a) * self._shift[0] + a * self._shift_t[0]
        sy = (1 - a) * self._shift[1] + a * self._shift_t[1]
        self._shift = (sx, sy)
        self._roll = (1 - a) * self._roll + a * self._roll_t

        # Apply virtual-camera effects
        rgb_fx = self._apply_effects(rgb_src, self._zoom, self._shift, self._roll)

        # Draw overlays (status; boxes optional)
        self._draw_overlays(rgb_fx)

        # Convert to PIL and fit to panel (cover/contain)
        pil = Image.fromarray(rgb_fx)
        pw, ph = max(1, self.winfo_width()), max(1, self.winfo_height())
        pil = self._resize_to_panel(pil, pw, ph, self._fit_mode)

        # Cache the exact panel image for AI
        if np is not None:
            self._last_panel_frame = np.asarray(pil)  # RGB

        # TK show
        self._tk_img = ImageTk.PhotoImage(pil)
        self._img_label.configure(image=self._tk_img, text="")

        self._schedule_next()

    def _schedule_next(self):
        if self._play and self._cap is not None:
            delay_ms = max(10, int(1000.0 / max(1.0, self._fps * self._speed)))
        else:
            delay_ms = 80
        self._job = self.after(delay_ms, self._paint_loop)

    # --------------------------- Effects / Render helpers -------------------

    def _apply_effects(self, rgb: "np.ndarray", zoom: float,
                       shift: Tuple[float, float], roll_deg: float) -> "np.ndarray":
        """Apply roll, then pan+zoom crop. Returns RGB ndarray."""
        if np is None or cv2 is None:
            return rgb

        h, w = rgb.shape[:2]
        out = rgb

        # 1) Roll (rotate about center; keep frame size; black border)
        if abs(roll_deg) > 0.05:
            M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), roll_deg, 1.0)
            out = cv2.warpAffine(out, M, (w, h), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        # 2) Pan + Zoom (digital)
        z = max(1.0, float(zoom))
        crop_w = int(round(w / z))
        crop_h = int(round(h / z))
        crop_w = max(2, min(w, crop_w))
        crop_h = max(2, min(h, crop_h))

        dx, dy = float(shift[0]), float(shift[1])
        cx = int(round(w / 2.0 + dx))
        cy = int(round(h / 2.0 + dy))

        x0 = int(round(cx - crop_w / 2.0))
        y0 = int(round(cy - crop_h / 2.0))
        x0 = max(0, min(w - crop_w, x0))
        y0 = max(0, min(h - crop_h, y0))

        roi = out[y0:y0 + crop_h, x0:x0 + crop_w]
        return roi

    def _draw_overlays(self, rgb: "np.ndarray"):
        """Status text and (optionally) detection boxes, in-place on RGB array."""
        if cv2 is None:
            return
        h, w = rgb.shape[:2]

        # Status
        status = (f"VIDEO {self._speed:.1f}x | zoom {self._zoom:.2f} | "
                  f"roll {self._roll:+.1f}° | aim {self._aim_dx:+.2f},{self._aim_dy:+.2f}")
        cv2.putText(rgb, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 240, 140), 2, cv2.LINE_AA)

        # Boxes (disabled if we've zoomed/cropped heavily because source coords change)
        if self._draw_boxes and self._zoom <= 1.05 and abs(self._roll) < 0.5 and max(abs(self._shift[0]), abs(self._shift[1])) < 1.0:
            for (x1, y1, x2, y2) in self._boxes:
                x1 = int(max(0, min(w - 1, x1))); x2 = int(max(0, min(w - 1, x2)))
                y1 = int(max(0, min(h - 1, y1))); y2 = int(max(0, min(h - 1, y2)))
                cv2.rectangle(rgb, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # REC badge
        if self._rec_on:
            cv2.rectangle(rgb, (12, 8), (90, 32), (32, 32, 32), -1)
            cv2.putText(rgb, "REC", (20, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 64, 64), 2, cv2.LINE_AA)

    def _resize_to_panel(self, pil_img: Image.Image, target_w: int, target_h: int, mode: str) -> Image.Image:
        """Return a PIL image that exactly matches (target_w, target_h)."""
        target_w = max(1, int(target_w))
        target_h = max(1, int(target_h))
        iw, ih = pil_img.size
        if iw <= 0 or ih <= 0:
            return pil_img.resize((target_w, target_h), Image.BILINEAR)

        sx = target_w / iw
        sy = target_h / ih
        if (mode or "").lower() == "contain":
            s = min(sx, sy)
            new_w, new_h = int(round(iw * s)), int(round(ih * s))
            img = pil_img.resize((new_w, new_h), Image.BILINEAR)
            canvas = Image.new("RGB", (target_w, target_h), (0, 0, 0))
            ox = (target_w - new_w) // 2
            oy = (target_h - new_h) // 2
            canvas.paste(img, (ox, oy))
            return canvas

        # COVER: fill and center-crop
        s = max(sx, sy)
        new_w, new_h = int(round(iw * s)), int(round(ih * s))
        img = pil_img.resize((new_w, new_h), Image.BILINEAR)
        left = max(0, (new_w - target_w) // 2)
        top = max(0, (new_h - target_h) // 2)
        right = left + target_w
        bottom = top + target_h
        return img.crop((left, top, right, bottom))

    def _show_text(self, text: str):
        self._img_label.configure(text=text, fg="#aaa", font=("Segoe UI", 12), image="")
        self._tk_img = None
