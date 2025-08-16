# # core/guidance.py — map tracker errors to setpoints (pure stdlib)
# from .state import CmdSetpoints

# def map_tracking_to_setpoints(dx_norm: float, dy_norm: float, dz_rate: float,
#                               k_yaw_dps: float, k_fwd: float, v_base: float,
#                               v_max: float, vz_max: float) -> CmdSetpoints:
#     # Yaw rate command (deg/s)
#     yaw_rate = k_yaw_dps * dx_norm
#     # Forward speed command (m/s)
#     err_mag = max(abs(dx_norm), abs(dy_norm))
#     v_cmd = v_base + k_fwd * err_mag
#     if v_cmd > v_max:
#         v_cmd = v_max
#     # Vertical rate (already m/s)
#     vz = dz_rate
#     return CmdSetpoints(yaw_rate_dps=yaw_rate, v_forward=v_cmd, vz=vz)


# core/guidance.py
from core.state import CmdSetpoints

def _signed_deadband(e, db):
    if abs(e) <= db:
        return 0.0
    return e - db if e > 0.0 else e + db

# persistent filters (function attributes)
def map_tracking_to_setpoints(
    dx_px, dy_px, dz_rate,
    k_yaw_dps=90.0, k_fwd=8.0, v_base=4.0, v_max=15.0, vz_max=3.0,
    deadband_x=8.0, deadband_y=6.0, lp=0.18, px_norm_y=120.0
):
    """
    dx_px: +right (pixels), dy_px: +down (pixels)
    k_yaw_dps: deg/s per 1.0 unit of (filtered) dx
    k_fwd:     m/s per 1.0 unit of (normalized) dy
    v_base:    base forward speed even at zero error
    """

    # initialize static state on first call
    if not hasattr(map_tracking_to_setpoints, "_xf"):
        map_tracking_to_setpoints._xf = 0.0
        map_tracking_to_setpoints._yf = 0.0

    # deadband in pixels
    dx_db = _signed_deadband(float(dx_px), float(deadband_x))
    dy_db = _signed_deadband(float(dy_px), float(deadband_y))

    # low-pass to suppress jitter (prevents "constant turn" from tiny biases)
    xf = (1.0 - lp) * map_tracking_to_setpoints._xf + lp * dx_db
    yf = (1.0 - lp) * map_tracking_to_setpoints._yf + lp * dy_db
    map_tracking_to_setpoints._xf = xf
    map_tracking_to_setpoints._yf = yf

    # yaw: proportional on filtered dx, simulator will bank-limit later
    yaw_cmd = k_yaw_dps * xf

    # forward: base + proportional to (positive) down error in normalized units
    # px_norm_y ≈ pixels that should feel like "1.0" unit of error
    ey = max(0.0, yf) / max(1e-6, px_norm_y)
    v_cmd = v_base + k_fwd * ey
    v_cmd = 0.0 if v_cmd < 0.05 else min(v_cmd, v_max)

    # climb/descent direct (clamped)
    vz_cmd = max(-vz_max, min(vz_max, float(dz_rate)))

    return CmdSetpoints(yaw_rate_dps=float(yaw_cmd), v_forward=float(v_cmd), vz=float(vz_cmd))
