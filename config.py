# config.py — runtime settings (pure stdlib)
from dataclasses import dataclass

ASSETS_DIR = "assets"
MAP_IMAGE  = f"{ASSETS_DIR}/map_bg.ppm"   # or .png if you made one

# GUI
WINDOW_W = 1200
WINDOW_H = 720
FPS      = 60              # GUI redraw
DT       = 1.0 / 100.0     # physics step (s)

# World scaling (meters-to-pixels on the map)
METERS_PER_PIXEL = 1.0     # 1 px = 1 m

# I/O
UDP_LISTEN_PORT = 47800
UDP_TELEM_PORT = 47801   # sim -> AI telemetry (JSON over UDP)
UART_DEV        = "/dev/ttyUSB0"
UART_BAUD       = 115200

GRAVITY = 9.80665  # m/s^2

@dataclass
class Limits:
    V_MAX: float      = 15.0   # m/s (forward speed cap)
    A_MAX: float      = 4.0    # m/s^2 (forward accel/decay cap)
    A_LAT_MAX: float  = 5.0    # (reserved)
    VZ_MAX: float     = 3.0    # m/s climb/descent
    YAW_RATE_MAX: float = 120.0 # deg/s cap (command)
    BANK_MAX: float   = 35.0   # deg (coordinated-turn bank cap)

@dataclass
class Gains:
    K_YAW: float  = 90.0   # deg/s @ dx=1
    K_FWD: float  = 8.0    # m/s @ |err|=1
    V_BASE: float = 4.0    # m/s

@dataclass
class WindDefault:
    east: float = 2.0
    north: float = 0.0
    up: float = 0.0
