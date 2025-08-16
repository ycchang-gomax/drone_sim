Drone Path Simulator (pure Python / Tkinter)

Purpose
-------
Simulate a quadcopter’s path, altitude, speed and attitude while an external
AI tracker (YOLO) sends image-center errors over UDP/UART to steer the drone.
The app draws the drone flying over a map background and exposes live controls
for speed limits, wind, and guidance gains.

Folder Structure
----------------
drone_sim/
  app.py                 - entry point
  config.py              - GUI sizes, ports, limits, paths
  core/
    state.py             - DroneState, EnvState, CmdSetpoints, LinkStatus
    physics.py           - coordinated-turn kinematics w/ limits & wind
    guidance.py          - (dx,dy,dz_rate) -> (yaw_rate, v_forward, vz)
  simio/
    udp_rx.py            - nonblocking UDP (JSON + framed binary)
    uart_rx.py           - (stub) UART receiver (to be implemented)
    protocol.py          - CRC16 framing, TRACK_CMD encoder/decoder
  ui/
    main_window.py       - Tk root, timers, HUD text
    map_view.py          - Canvas with map, quad icon, trail, FOV
    controls.py          - Start/Pause/Reset + sliders
  assets/
    map_bg.ppm           - offline map image (or .png)

Run
---
Windows (Anaconda Prompt):
  cd drone_sim
  python app.py

Linux:
  cd drone_sim
  python3 app.py   (ensure python3-tk is installed)

Controls
--------
- Start / Pause / Reset
- Sliders:
  V_MAX, VZ_MAX, Yaw rate max, K_yaw, K_fwd, V_base, Wind East, Wind North

Networking
----------
UDP listen: 0.0.0.0:47800

Send JSON test:
  Windows:
    python -c "import socket,json; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); \
s.sendto(b'{\"dx\":0.5,\"dy\":-0.2,\"dz_rate\":0.0,\"conf\":90,\"mode\":1}', ('127.0.0.1',47800))"

Binary frame (TRACK_CMD):
  0xAA 0x55 [LEN] [0x01] [dx f32, dy f32, dz f32, conf u8, mode u8, rsv u16] [CRC16-CCITT]

Conventions
-----------
- World: ENU (meters). Body: x forward, y right, z up.
- Image errors: dx,dy in [-1..+1] where 0,0 is image center.
- dz_rate: vertical rate command (m/s).

Physics Model (Pack 1)
----------------------
- Forward speed slews with A_MAX
- Bank-limited turning (coordinated turn) from BANK_MAX
- Climb/descend limited by VZ_MAX
- Wind added to ground velocity
- Attitude estimates for HUD: roll ~ atan2(a_lat,g), pitch ~ atan2(vz,v_forward)

Roadmap
-------
1) Scaffold + Map + Demo motion (done)
2) Physics Pack 1 (done)
3) Guidance Pack: deadband, integrator trim, saturation indicators
4) I/O Pack: UART (Windows COM & Linux), robust binary framing, link stats
5) HUD/Logging Pack: tapes, error charts, CSV/KML export
6) Fault injection: latency, packet drop, gusts; presets for demos

License
-------
MIT (or your choice). Third-party map tiles NOT used; background is a local image.
