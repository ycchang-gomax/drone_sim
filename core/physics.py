# core/physics.py — 2.5D point-mass dynamics with wind, bank limit, and ground lock
import math

# ---- tunables ---------------------------------------------------------------
EPS_Z = 0.05          # [m] treat |z| < EPS_Z as "on ground"
GROUND_BRAKE = True   # bleed off forward speed while on ground
BRAKE_DECEL = 3.0     # [m/s^2] simple friction decel when GROUND_BRAKE=True
MAX_TRAIL = 20000     # points kept in state.trail

# ---- helpers ----------------------------------------------------------------
def _clamp(x, a, b):
    return a if x < a else b if x > b else x

def _wrap180(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg

# ---- integrator --------------------------------------------------------------
def step(
    dt,
    state,                 # DroneState: x,y,z, v_forward, vz, yaw, roll, pitch, trail(list)
    env,                   # EnvState:   wind_e, wind_n
    cmd,                   # CmdSetpoints: yaw_rate_dps, v_forward, vz
    v_max=20.0,
    vz_max=3.0,
    yaw_rate_max_dps=120.0,
    a_max=4.0,
    bank_max_deg=35.0,
    g=9.80665,
):
    """
    Integrate the vehicle for dt seconds.

    Model summary
      • Forward airspeed first-order toward cmd.v_forward, |a| ≤ a_max, 0..v_max.
      • Yaw rate limited by yaw_rate_max_dps AND bank (coordinated turn) limit.
      • Ground vel = air vel (by yaw) + wind (east/north).
      • Vertical speed clamped to ±vz_max.
      • Ground plane at z=0 with *lock*: if at/under ground and not climbing, freeze x/y and yaw-level.
    """
    # --- inputs / commands ----------------------------------------------------
    wind_e = float(getattr(env, "wind_e", 0.0))
    wind_n = float(getattr(env, "wind_n", 0.0))

    v_cmd  = _clamp(float(getattr(cmd, "v_forward", 0.0)), 0.0, float(v_max))
    vz_cmd = _clamp(float(getattr(cmd, "vz", 0.0)), -float(vz_max), float(vz_max))
    r_cmd  = _clamp(float(getattr(cmd, "yaw_rate_dps", 0.0)),
                    -float(yaw_rate_max_dps), float(yaw_rate_max_dps))

    # --- forward speed toward command ----------------------------------------
    v_now  = float(getattr(state, "v_forward", 0.0))
    dv_max = float(a_max) * float(dt)
    v_next = v_now + _clamp(v_cmd - v_now, -dv_max, dv_max)
    v_next = _clamp(v_next, 0.0, float(v_max))

    # --- bank-limited yaw rate -----------------------------------------------
    if v_next > 0.1:
        r_bank = (g * math.tan(math.radians(bank_max_deg)) / v_next) * (180.0 / math.pi)
        r_cmd = _clamp(r_cmd, -abs(r_bank), abs(r_bank))

    # --- yaw ------------------------------------------------------------------
    yaw_deg = _wrap180(float(getattr(state, "yaw", 0.0)) + r_cmd * dt)
    yaw_rad = math.radians(yaw_deg)

    # --- vertical -------------------------------------------------------------
    vz = vz_cmd

    # --- velocities -----------------------------------------------------------
    air_e = math.cos(yaw_rad) * v_next
    air_n = math.sin(yaw_rad) * v_next
    ve    = air_e + wind_e
    vn    = air_n + wind_n

    # --- previous state -------------------------------------------------------
    x0 = float(getattr(state, "x", 0.0))
    y0 = float(getattr(state, "y", 0.0))
    z0 = float(getattr(state, "z", 0.0))

    # --- ground lock decision -------------------------------------------------
    z1 = z0 + vz * dt
    on_ground_lock = (z0 <= EPS_Z) and (z1 <= EPS_Z) and (vz <= 0.0)

    if on_ground_lock:
        # freeze on ground: no drift, no sink
        x1, y1, z1 = x0, y0, 0.0
        if GROUND_BRAKE:
            v_next = max(0.0, v_next - BRAKE_DECEL * dt)
        else:
            v_next = 0.0
        vz = 0.0
        roll_deg = 0.0
        pitch_deg = 0.0
    else:
        # normal flight integration
        x1 = x0 + ve * dt
        y1 = y0 + vn * dt
        if z1 < 0.0:
            z1 = 0.0
            if vz < 0.0:
                vz = 0.0

        # attitude (kinematic approximation)
        r_rad = math.radians(r_cmd)
        a_lat = v_next * r_rad
        roll_deg  = math.degrees(math.atan2(a_lat, g)) if v_next > 0.01 else 0.0
        pitch_deg = math.degrees(math.atan2(vz, max(v_next, 0.1)))
        if z1 <= EPS_Z:
            roll_deg = 0.0
            pitch_deg = 0.0

    # --- write back -----------------------------------------------------------
    state.x = x1
    state.y = y1
    state.z = z1
    state.v_forward = v_next
    state.vz = vz
    state.yaw = yaw_deg
    state.roll = roll_deg
    state.pitch = pitch_deg

    # --- trail ---------------------------------------------------------------
    tr = getattr(state, "trail", None)
    if isinstance(tr, list):
        tr.append((x1, y1))
        if len(tr) > MAX_TRAIL:
            del tr[: len(tr) - MAX_TRAIL]
