# core/nav.py — Wind-aware guidance for Quad & Fixed-Wing
# -------------------------------------------------------
# Exposes:
#   • Waypoint(x, y)
#   • route_guidance_with_wind(...): returns
#       (yaw_rate_cmd_deg_s, v_cmd_air_mps, capture_ok, wind_limited,
#        loitering, hold_headwind, goaround)
#
# This module converts a desired *ground* path into *air* commands by
# subtracting wind. Quads “point the ground vector” at the target and
# solve for the needed air vector. Fixed-wings fly a wind-compensated
# straight final (FAF→DEST) with look-ahead, alignment gating, and
# go-around hints.

from __future__ import annotations
from dataclasses import dataclass
import math

# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
G = 9.80665  # m/s²

def wrap180(a: float) -> float:
    """Wrap degrees to [-180, 180)."""
    a = (a + 180.0) % 360.0 - 180.0
    return a

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def norm2(x: float, y: float) -> float:
    return math.hypot(x, y)

def unit(x: float, y: float) -> tuple[float, float, float]:
    n = norm2(x, y)
    if n < 1e-12:
        return 1.0, 0.0, 0.0
    return x / n, y / n, n

def dot(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by

def air_from_ground(ge: float, gn: float, we: float, wn: float) -> tuple[float, float]:
    """Air velocity vector = desired ground vector minus wind."""
    return ge - we, gn - wn

def bank_limited_yaw_max_deg_s(v_air: float, bank_max_deg: float) -> float:
    """
    Coordinated-turn limit on yaw rate (deg/s):
        r_dot(rad/s) <= g * tan(phi) / max(V, 0.1)
    """
    v = max(v_air, 0.1)
    rdot = G * math.tan(bank_max_deg * DEG2RAD) / v
    return rdot * RAD2DEG

# ---------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    x: float
    y: float

# ---------------------------------------------------------------------------
# Guidance
# ---------------------------------------------------------------------------

def route_guidance_with_wind(
    vehicle: str,
    x: float, y: float, yaw_deg: float, v_air: float,
    wind_e: float, wind_n: float,
    start: Waypoint, dest: Waypoint,
    *,
    # Tuning (kept explicit; adjust from main_window if you like)
    v_cruise: float = 8.0,
    v_max: float = 15.0,
    bank_max_deg: float = 35.0,

    # Quad stopping radius
    stop_radius: float = 5.0,

    # Fixed-wing straight-in final geometry
    final_leg: float = 250.0,   # length of the final (FAF→DEST)
    faf_radius: float = 80.0,   # (kept for compatibility; not strictly needed)
    lookahead: float = 90.0,    # how far ahead on final to aim
    align_deg: float = 25.0,    # alignment gate to “capture final”
    descend_radius: float = 40.0,   # near dest: allow landing arming
    v_min_fixed: float = 12.0,      # FW minimum commanded airspeed
):
    """
    Returns:
        yaw_rate_cmd_deg_s, v_cmd_air_mps, capture_ok, wind_limited,
        loitering, hold_headwind, goaround
    """

    # Vector from current position to destination
    dx = dest.x - x
    dy = dest.y - y
    tx, ty, dist_to_dest = unit(dx, dy)

    # Bookkeeping flags
    wind_limited = False
    loitering    = False
    hold_head    = False
    goaround     = False

    # -----------------------------------------------------------------------
    # QUADROTOR: steer the *ground* vector to the dest, then solve for air.
    # -----------------------------------------------------------------------
    if vehicle == 'quad':
        # If we are inside the stop bubble, signal capture (UI enters “landing”).
        if dist_to_dest <= stop_radius:
            return 0.0, 0.0, True, False, False, False, False

        # Desired ground speed along the line to DEST. Slow down near the end.
        v_g = min(v_cruise, max(0.8, 0.6 * dist_to_dest))
        g_e, g_n = tx * v_g, ty * v_g

        # Wind compensation: what air vector gives that ground vector?
        a_e, a_n = air_from_ground(g_e, g_n, wind_e, wind_n)
        v_cmd = norm2(a_e, a_n)

        # If not enough airspeed authority (headwind/crosswind), mark limited.
        if v_cmd > v_max + 1e-6:
            wind_limited = True
            # If strong headwind component toward the target, try to hold position
            # by pointing into the wind (prevents drifting away visually).
            if dot(tx, ty, wind_e, wind_n) < -0.25 * norm2(wind_e, wind_n):
                hold_head = True
                a_e, a_n = -wind_e, -wind_n
                v_cmd = min(v_max, norm2(a_e, a_n))

        v_cmd = clamp(v_cmd, 0.0, v_max)

        # Heading command toward the air vector; physics will enforce bank/yaw limits.
        hdg = math.degrees(math.atan2(a_n, a_e)) if v_cmd > 0.05 else yaw_deg
        yaw_err = wrap180(hdg - yaw_deg)
        # Proportional heading law; pre-limit by bank capacity for smoothness.
        yaw_lim = bank_limited_yaw_max_deg_s(max(v_air, v_cmd), bank_max_deg)
        yaw_rate = clamp(3.0 * yaw_err, -yaw_lim, yaw_lim)

        return yaw_rate, v_cmd, False, wind_limited, loitering, hold_head, goaround

    # -----------------------------------------------------------------------
    # FIXED-WING: fly a wind-compensated straight final (FAF→DEST).
    #            Aim at a point ahead on the final, convert ground→air.
    # -----------------------------------------------------------------------

    # Final direction: from START to DEST. (If identical, use current line-of-sight.)
    fx, fy, _ = unit(dest.x - start.x, dest.y - start.y)
    if abs(fx) < 1e-9 and abs(fy) < 1e-9:
        fx, fy = tx, ty

    # Final line endpoints: p1 = FAF, p2 = DEST
    p2x, p2y = dest.x, dest.y
    p1x, p1y = dest.x - fx * final_leg, dest.y - fy * final_leg

    # Project current position onto final to get along-track “s” (from p1).
    relx, rely = x - p1x, y - p1y
    s = dot(relx, rely, fx, fy)  # meters along the final from p1
    # Look ahead to make the path smooth and reduce hunting.
    s_aim = clamp(s + lookahead, 0.0, final_leg)

    # Aim point on final
    aim_x = p1x + fx * s_aim
    aim_y = p1y + fy * s_aim

    ax, ay, adist = unit(aim_x - x, aim_y - y)

    # Desired ground speed along final: fast when far, ease off near aim point.
    v_g = max(v_min_fixed, min(v_cruise, 0.6 * adist + v_min_fixed))
    g_e, g_n = ax * v_g, ay * v_g

    # Wind compensation
    a_e, a_n = air_from_ground(g_e, g_n, wind_e, wind_n)
    v_cmd = norm2(a_e, a_n)
    if v_cmd > v_max + 1e-6:
        wind_limited = True
        v_cmd = v_max
    v_cmd = max(v_min_fixed, v_cmd)

    # Heading toward air vector, with bank-limited pre-clamp
    hdg = math.degrees(math.atan2(a_n, a_e))
    yaw_err = wrap180(hdg - yaw_deg)
    yaw_lim = bank_limited_yaw_max_deg_s(max(v_air, v_cmd), bank_max_deg)
    yaw_rate = clamp(2.6 * yaw_err, -yaw_lim, yaw_lim)

    # Alignment / capture logic
    final_brg = math.degrees(math.atan2(fy, fx))   # direction of final
    align_err = abs(wrap180(final_brg - hdg))
    near_dest = dist_to_dest <= descend_radius
    aligned   = align_err <= align_deg
    capture_ok = near_dest and aligned

    # Go-around hint: near the threshold but poorly aligned.
    goaround = near_dest and (align_err > align_deg * 1.4)

    # Loiter hint: shot past the end of final and still not aligned.
    if (s > final_leg + 10.0) and not aligned:
        loitering = True

    return yaw_rate, v_cmd, capture_ok, wind_limited, loitering, False, goaround
