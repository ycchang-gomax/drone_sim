# core/state.py — simulation state (pure stdlib)
from dataclasses import dataclass, field

@dataclass
class DroneState:
    # Position in ENU frame [m]
    x: float = 0.0   # East
    y: float = 0.0   # North
    z: float = 10.0  # Up (altitude AGL for now)

    # Attitude [deg]
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0   # heading (0 = east, 90 = north)

    # Velocities
    v_forward: float = 0.0  # along body x (m/s)
    vz: float = 0.0         # climb rate (m/s)

    # Trail (recent positions for plotting)
    trail: list = field(default_factory=list)

@dataclass
class EnvState:
    wind_e: float = 0.0  # m/s
    wind_n: float = 0.0
    wind_u: float = 0.0

@dataclass
class CmdSetpoints:
    yaw_rate_dps: float = 0.0  # desired yaw rate [deg/s]
    v_forward: float = 0.0     # desired forward speed [m/s]
    vz: float = 0.0            # desired climb rate [m/s]

@dataclass
class LinkStatus:
    source: str = "DEMO"     # DEMO | UDP | UART
    connected: bool = False
    last_rx_ms: int = 0
    packets: int = 0
    drops: int = 0
    mode: int = 0
    conf: int = 0            # tracker confidence (0..100)
