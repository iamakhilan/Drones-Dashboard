<div align="center">

<!-- ============ HERO BANNER ============ -->
<img src="drone_prototype_render.png" width="720" alt="AeroVIO — Autonomous Drone VIO Platform" />

# 🛰️ AeroVIO Ground Station

### *Monocular Visual-Inertial Odometry · MSCKF State Estimation · 50 Hz Cascaded SO(3) Flight Control*

**Aerospace-grade real-time Ground Control Station (GCS) for autonomous drone VIO, 3D mapping & closed-loop flight.**

---

<!-- Badges -->
<p>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-0EA5E9?style=for-the-badge" alt="MIT License" /></a>
  <a href="bridge_server.py"><img src="https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.8+" /></a>
  <a href="index.html"><img src="https://img.shields.io/badge/Browser-Chrome%20%2F%20Firefox%20%2F%20Safari-1A1A1A?style=for-the-badge" alt="Browser" /></a>
  <img src="https://img.shields.io/badge/Controller-50_Hz_SO(3)-10B981?style=for-the-badge" alt="50Hz SO3" />
  <img src="https://img.shields.io/badge/VIO-MSCKF_/_VINS-8B5CF6?style=for-the-badge" alt="MSCKF" />
</p>

<p>
  <a href="https://github.com/iamakhilan/Drones-Dashboard"><img src="https://img.shields.io/github/stars/iamakhilan/Drones-Dashboard?style=social" alt="Stars" /></a>
  <img src="https://img.shields.io/badge/status-DEMO_ready-success?style=flat-square" alt="demo" />
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros&logoColor=white" alt="ROS2" />
  <img src="https://img.shields.io/badge/WebSocket-RFC6455-EF4444?style=flat-square" alt="ws" />
  <img src="https://img.shields.io/badge/Tailwind-CSS-06B6D4?style=flat-square&logo=tailwindcss&logoColor=white" alt="tailwind" />
</p>

[🚀 Live Demo](#-quickstart) · [📖 Docs](#-mathematical-formulations) · [🏗️ Architecture](#️-architecture) · [🐛 Issues](https://github.com/iamakhilan/Drones-Dashboard/issues)

</div>

---

<div align="center">

> Companion repo: **[Drones_S5_CD2 — Controller & VIO Stack](https://github.com/Ravish2807/Drones_S5_CD2_3D_Mapping_using_normal_monocular_camera_controlling_Visual-Inertial-Odometry-VIO/tree/drones-controller)** — the 50 Hz `drones-controller` ROS 2 node this dashboard visualizes.

</div>

---

## ✨ Why AeroVIO?

| | |
|---|---|
| 🎯 **Truthful by design** | Explicitly separates **WebSocket bridge** connectivity from **ROS 2 live-node** connectivity — no fake "connected" states. |
| ⚡ **50 Hz closed-loop** | Full cascaded pipeline at controller rate: Position → Attitude Generator → Geometric SO(3) → FSM → ArduPilot. |
| 👁️ **Frame-rigorous VIO** | Strict `T_WB` (body) vs `T_WC = T_WB · T_BC` (camera optical) separation. No shortcuts. |
| 🗺️ **Real 3D mapping** | 442-point `synthetic_orbit_scanned_map.pcd` reconstruction, multi-projection (Isometric / Top XY / Side XZ). |
| 🔌 **Zero-dependency bridge** | Pure stdlib Python — `http.server` + hand-rolled RFC 6455 WebSocket. No `pip install`. |
| 📡 **5 flight regimes** | Calibrated datasets: `orbit` · `hover` · `straight` · `multi_axis` · `stress` (500 pts each). |

---

## 🖼️ Screenshots

<div align="center">

| Ground Station Dashboard | Monocular Tracking Feed | 3D Spatial Map |
|---|---|---|
| <img src="drone_vio_screen_89764d85cb77486e852b5b488c2f5fff.png" width="360" alt="Dashboard" /> | <img src="monocular_tracking_feed.png" width="360" alt="Camera feed" /> | <img src="stitch_screen_5f02e90f849a4a9b90ee2d2da12b516a.png" width="360" alt="3D map" /> |
| *KPI cards · SO(3) telemetry · oscilloscope graphs* | *KLT velocity vectors · Brown-Conrady · reticle grid* | *442 landmarks · T_WB / T_WC trajectories* |

*Light theme · #F6F6F7 surfaces · stone neutrals · SF Mono / Inter*

</div>

<details>
<summary><b>📸 More renders</b></summary>

<br/>

<div align="center">

| Prototype Airframe | Light Render |
|---|---|
| <img src="drone_prototype_render.png" width="400" alt="prototype" /> | <img src="drone_render_light.png" width="400" alt="light render" /> |

</div>

</details>

---

## 🚀 Feature Highlights

<table>
<tr>
<td width="33%" valign="top">

#### 👁️ VIO & Perception
- Monocular feed @ 30 Hz (`/camera/image_raw`)
- KLT feature velocity vectors
- Brown-Conrady camera model overlay
- Optical reticle + epipolar toggle
- 442-landmark PCD map (`HEIGHT=1`)
- `T_WC = T_WB · T_BC` projection

</td>
<td width="33%" valign="top">

#### 🎛️ 50 Hz Cascaded Control
- **Outer:** `F_d = -K_p e_p - K_v e_v + m g e₃ + m a_d`
- **Generator:** `b₃d = F_d/‖F_d‖`, `T_d = ‖F_d‖`, `R_d / q_d`
- **Inner:** `e_R = ½(R_dᵀR − RᵀR_d)∨`, `M_d = −k_R e_R − k_Ω e_ω + ω×Jω`
- FSM: `IDLE → ARMING → TAKEOFF → NAVIGATING → HOVER / FAILSAFE`
- Watchdog heartbeat + failsafe

</td>
<td width="33%" valign="top">

#### 📈 Telemetry & Transport
- 3 × 20 s rolling SVG oscilloscopes (accel / vel / pos)
- Channel toggles + hover crosshair + pause/clear/undo
- DEMO replay engine — 5 datasets @ 50 Hz
- `bridge_server.py` — HTTP + `/ws` WebSocket
- Cross-machine LAN: `ws://[IP]:8000/ws`
- Stale-warning banner + truthful status

</td>
</tr>
</table>

---

## 🏗️ Architecture

<div align="center">

*End-to-end 50 Hz cascaded pipeline — from photons & IMU to actuation*

</div>

```mermaid
flowchart TB
    subgraph SENSORS["📡 Sensors & Inputs"]
        CAM["📷 Monocular Camera<br/>30 Hz · /camera/image_raw"]
        IMU["🧭 6-DOF IMU<br/>200 Hz · /imu/data_raw<br/>ADIS16470 / BMI088"]
        AP["✈️ ArduPilot Status<br/>10 Hz · /ap/v1/status"]
    end

    subgraph EST["🔬 State Estimation — VIO (MSCKF)"]
        VIO["VIO Node<br/>/drone/vio/odometry → T_WB<br/>/drone/vio/camera_pose → T_WC<br/>/drone/vio/pointcloud → 442 pts"]
        STATE["Vehicle State<br/>/ap/v1/pose/filtered · 50 Hz<br/>/ap/v1/twist/filtered · 50 Hz"]
    end

    subgraph SAFETY["🛡️ Safety & Adapter"]
        WD["Watchdog<br/>Heartbeat · Failsafe Monitor"]
        ADAPT["State Adapter<br/>Frame & Rate Transform"]
    end

    subgraph CTRL["⚡ 50 Hz Cascaded Controller — drones-controller"]
        POS["① Position Controller (Outer)<br/>e_p = p − p_d · e_v = v − v_d<br/>F_d = −K_p e_p − K_v e_v + m g e₃ + m a_d"]
        ATTG["② Attitude Generator<br/>b₃d = F_d / ‖F_d‖ · T_d = ‖F_d‖<br/>→ R_d ∈ SO(3) · q_d"]
        SO3["③ Geometric SO(3) (Inner)<br/>e_R = ½(R_dᵀR − RᵀR_d)∨<br/>e_ω = ω − RᵀR_d ω_d<br/>M_d = −k_R e_R − k_Ω e_ω + ω×Jω"]
        FSM["④ Flight State Manager<br/>IDLE → ARMING → TAKEOFF<br/>→ NAVIGATING → HOVER / FAILSAFE"]
    end

    subgraph BRIDGE["🌉 Transport & Ground Station"]
        CMD["/ap/v1/cmd_vel<br/>TwistStamped @ 50 Hz"]
        BR["bridge_server.py<br/>HTTP :8000 + WS /ws<br/>DEMO replay / LIVE forward"]
        GCS["🖥️ AeroVIO Dashboard<br/>KPI · 3D Map · Oscilloscope · Camera HUD"]
    end

    CAM --> VIO
    IMU --> VIO
    VIO --> STATE
    STATE --> WD
    AP --> WD
    WD --> ADAPT
    ADAPT --> POS --> ATTG --> SO3 --> FSM --> CMD
    STATE -.-> BR
    POS -.-> BR
    ATTG -.-> BR
    SO3 -.-> BR
    VIO -.-> BR
    BR <==>|"JSON @ ~50 Hz"| GCS

    style POS fill:#0EA5E9,stroke:#0284C7,color:#fff
    style SO3 fill:#10B981,stroke:#059669,color:#fff
    style VIO fill:#8B5CF6,stroke:#7C3AED,color:#fff
    style BR fill:#F59E0B,stroke:#D97706,color:#fff
```

<details>
<summary><b>🔍 Data-flow notes</b></summary>

- **DEMO mode** — `bridge_server.py` replays CSV trajectories, synthesizes `T_WB`/`T_WC`, computes controller outputs on-the-fly @ 50 Hz, broadcasts normalized JSON to all `/ws` clients.
- **LIVE mode** — bridge forwards frames from a companion `rclpy` adapter subscribing to real ROS 2 topics. Dashboard unchanged.
- **Truthful status** — header pills show `Bridge: ● Connected` vs `ROS 2: ○ DEMO` independently. No conflation.

</details>

---

## 📐 Mathematical Formulations

> All gains / constants match `bridge_server.py` + `drones-controller` ROS 2 params exactly.

### ① Outer Loop — Position Controller

| Symbol | Definition |
|---|---|
| $e_p = p - p_d$ | Position error $[e_{px},e_{py},e_{pz}]^T$ |
| $e_v = v - v_d$ | Velocity error $[e_{vx},e_{vy},e_{vz}]^T$ |
| $F_d = -K_p e_p - K_v e_v + m g e_3 + m a_d$ | Desired force |

| Param | Value |
|---|---|
| $m$ | $1.50\ \text{kg}$ |
| $K_p$ | $\mathrm{diag}(0.5, 0.5, 0.5)$ |
| $K_v$ | $\mathrm{diag}(0.2, 0.2, 0.2)$ |
| $g$ | $9.81\ \text{m/s}^2$ |

### ② Attitude Generator

$$b_{3d} = \frac{F_d}{\lVert F_d \rVert} \qquad T_d = \lVert F_d \rVert$$

Produces $R_d \in \mathrm{SO}(3)$ and $q_d$ (desired quaternion) from thrust direction + yaw reference.

### ③ Inner Loop — Geometric SO(3) Controller

| Symbol | Definition |
|---|---|
| $e_R = \tfrac{1}{2}\left(R_d^T R - R^T R_d\right)^{\vee}$ | Attitude error $[e_{Rx},e_{Ry},e_{Rz}]^T$ |
| $e_{\omega} = \omega - R^T R_d\, \omega_d$ | Angular velocity error |
| $M_d = -k_R\, e_R - k_{\Omega}\, e_{\omega} + \omega \times J \omega$ | Desired moment |

| Param | Value |
|---|---|
| $J$ | $\mathrm{diag}(0.02,\ 0.02,\ 0.04)\ \text{kg·m}^2$ |
| $k_R$ | $[4.85,\ 4.85,\ 2.50]$ |
| $k_{\Omega}$ | $[0.35,\ 0.35,\ 0.20]$ |

### ④ Frame Extrinsics

$$T_{WC} = T_{WB} \cdot T_{BC} \qquad T_{BC} = [+0.10,\ 0.00,\ -0.04]\ \text{m}$$

Camera mounted **10 cm forward, 4 cm downward** on airframe. Dashboard renders both trajectories independently — toggle `T_WB` / `T_WC` layers in 3D view.

---

## 📡 ROS 2 Topic Contracts

*Verified against `drones-controller` — rates, types, semantics.*

#### Inputs — Controller subscribes

| Topic | Type | Rate | Description |
|---|:---:|:---:|---|
| `/ap/v1/pose/filtered` | `geometry_msgs/PoseStamped` | 50 Hz | Estimated pose $p, q$ |
| `/ap/v1/twist/filtered` | `geometry_msgs/TwistStamped` | 50 Hz | Linear + angular velocity $v, \omega$ |
| `/ap/v1/status` | `ardupilot_msgs/Status` | 10 Hz | Armed / failsafe / flight state |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Monocular feed |
| `/imu/data_raw` | `sensor_msgs/Imu` | 200 Hz | 6-DOF accel + gyro |

#### Outputs — Controller publishes

| Topic | Type | Rate | Description |
|---|:---:|:---:|---|
| `/ap/v1/cmd_vel` | `geometry_msgs/TwistStamped` | 50 Hz | ArduPilot actuation |
| `/drones_controller/position_error` | `geometry_msgs/Vector3` | 50 Hz | $e_p$ |
| `/drones_controller/velocity_error` | `geometry_msgs/Vector3` | 50 Hz | $e_v$ |
| `/drones_controller/desired_force` | `geometry_msgs/Vector3` | 50 Hz | $F_d$ |
| `/drones_controller/desired_attitude` | `geometry_msgs/Quaternion` | 50 Hz | $q_d$ (+ $R_d$, $T_d$, $b_{3d}$) |
| `/drones_controller/attitude_error` | `geometry_msgs/Vector3` | 50 Hz | $e_R$ |
| `/drones_controller/angular_velocity_error` | `geometry_msgs/Vector3` | 50 Hz | $e_{\omega}$ |
| `/drones_controller/desired_moment` | `geometry_msgs/Vector3` | 50 Hz | $M_d$ |

#### VIO — Perception

| Topic | Type | Rate | Description |
|---|:---:|:---:|---|
| `/drone/vio/odometry` | `nav_msgs/Odometry` | 50 Hz | Body odometry $T_{WB}$ |
| `/drone/vio/camera_pose` | `geometry_msgs/PoseStamped` | 50 Hz | Camera optical $T_{WC}$ |
| `/drone/vio/pointcloud` | `sensor_msgs/PointCloud2` | ~5 Hz | Sparse map (442 pts) |

---

## 🧱 Tech Stack

| Layer | Technology | Notes |
|---|---|---|
| **Bridge** | Python 3.8+ · `http.server` · `hashlib`/`base64`/`struct` | Zero-dependency RFC 6455 WebSocket, `--port` / `--host` flags |
| **Frontend** | Vanilla JS (ES5 IIFE) · Tailwind CSS (CDN) · HTML5 Canvas · SVG | No build step — open `index.html` and go |
| **3D / Map** | Canvas 2D projection · PCD ASCII parser | Isometric / Top XY / Side XZ switcher |
| **Graphs** | Pure SVG · rolling 20 s window · 250-pt buffer | Spline + gradient fills · hover crosshair |
| **Data** | CSV trajectories · PCD v.7 ASCII · `real_data.js` | 5 datasets × 500 rows · 442 landmarks |
| **ROS 2** | Humble · `rclpy` adapter (planned live path) | `geometry_msgs` · `sensor_msgs` · `nav_msgs` |

<div align="center">

*No `npm install`. No `pip install`. Clone → run → fly.*

</div>

---

## ⚡ Quickstart

### Prerequisites

- **Python 3.8+** (only for the bridge — check with `python3 --version`)
- Modern browser (Chrome / Firefox / Safari)
- For LIVE mode: ROS 2 Humble + companion machine on same LAN

### 1 — Clone

```bash
git clone https://github.com/iamakhilan/Drones-Dashboard.git
cd Drones-Dashboard
```

### 2 — Start the bridge

```bash
# DEMO mode (default) — replays trajectories @ 50 Hz
python3 bridge_server.py

# Custom port / host
python3 bridge_server.py --port 8000 --host 0.0.0.0

# Help
python3 bridge_server.py --help
```

You should see:

```
[Bridge] Loaded 5 trajectory datasets: ['orbit', 'hover', 'straight', 'multi_axis', 'stress']
[Bridge] Serving HTTP on http://0.0.0.0:8000  —  WebSocket on ws://0.0.0.0:8000/ws
```

### 3 — Open the dashboard

Open **`http://localhost:8000`** in your browser.

Or just open `index.html` directly — set the WebSocket URL in the header input to `ws://localhost:8000/ws` and hit **Connect**.

### 4 — Use it

| Action | How |
|---|---|
| **Switch dataset** | Top bar pill selector — `orbit` / `hover` / `straight` / `multi_axis` / `stress` (also `?dataset=orbit` URL param) |
| **Toggle trajectories** | 3D view — `T_WB` (body) vs `T_WC` (camera optical) layers |
| **Change projection** | Isometric · Top (XY) · Side (XZ) buttons above 3D canvas |
| **Inspect signals** | Scroll oscilloscope graphs — hover for crosshair, toggle Ax/Ay/Az channels, Pause / Clear / Undo |
| **Search topics** | `⌘ K` or click header search — filters ROS 2 topics + params |
| **Go LIVE** | Point header WS URL to `ws://<companion-ip>:8000/ws` · flip mode toggle |

### LAN / Cross-machine

```bash
# On companion (ROS 2 machine)
python3 bridge_server.py --host 0.0.0.0 --port 8000

# On ground station — set header input to
ws://192.168.1.42:8000/ws
```

### LIVE ROS 2 (upcoming)

A thin `rclpy` adapter will subscribe to the topics above and push normalized JSON to `/ws` — dashboard stays identical. See [Roadmap](#-roadmap).

---

## ⚙️ Configuration

| Setting | Where | Default | Description |
|---|---|---|---|
| `websocketUrl` | Header input · `localStorage: aerovio.wsUrl` · `APP_CONFIG` | `ws://localhost:8000/ws` | Bridge endpoint |
| `rateHz` | `APP_CONFIG.rateHz` | `50.0` | Controller / broadcast rate |
| `massKg` | `APP_CONFIG.massKg` | `1.5` | Vehicle mass for $F_d$ |
| `gravity` | `APP_CONFIG.gravity` | `9.81` | $g$ |
| `T_BC_OFFSET` | `bridge_server.py: T_BC_OFFSET` | `{x:0.10, y:0.00, z:-0.04}` | Body→Camera extrinsic |
| `windowDurationSec` | `graphs.js` | `20.0` | Oscilloscope window |
| `--port` / `--host` | CLI flags | `8000` / `0.0.0.0` | Bridge bind |

---

## 📂 Repository Structure

```
Drones-Dashboard/
├── index.html                  # Ground station — single-page dashboard
├── bridge_server.py            # ★ Zero-dep HTTP + RFC6455 WebSocket bridge
├── css/
│   └── dashboard.css           # Light theme tokens (#F6F6F7, stone, brand)
├── js/
│   ├── websocket.js            # WS client, APP_CONFIG, reconnect, stale banner
│   ├── telemetry.js            # Telemetry engine — T_WB/T_WC, controller math
│   ├── controller.js           # SO(3) panel — e_p/e_v/F_d/b₃d/q_d/e_R/e_ω/M_d + FSM
│   ├── vio.js                  # 3D spatial map — PCD + trajectory projection
│   ├── camera.js               # Monocular HUD — KLT vectors, reticle, Brown-Conrady
│   ├── graphs.js               # 3× SVG oscilloscopes — accel/vel/pos (20 s window)
│   └── ui.js                   # Orchestrator — mode, datasets, search, modals, toasts
├── data/
│   ├── real_data.js                        # Preloaded dataset registry
│   ├── synthetic_orbit_estimated_trajectory.csv       # 500 pts — circular orbit
│   ├── synthetic_hover_estimated_trajectory.csv       # 500 pts — station-keeping
│   ├── synthetic_straight_estimated_trajectory.csv    # 500 pts — corridor
│   ├── synthetic_multi_axis_estimated_trajectory.csv  # 500 pts — coupled axes
│   ├── synthetic_stress_estimated_trajectory.csv      # 500 pts — aggressive
│   └── synthetic_orbit_scanned_map.pcd               # 442 landmarks — PCD v.7 ASCII
├── drone_prototype_render.png
├── drone_render_light.png
├── monocular_tracking_feed.png
└── README.md
```

---

## 🗺️ Roadmap

| Status | Item |
|:---:|---|
| ✅ | Light-themed GCS with KPI cards + SO(3) matrices + topic inspector |
| ✅ | Monocular HUD — KLT vectors, reticle, Brown-Conrady params |
| ✅ | 3D map — 442-pt PCD + `T_WB`/`T_WC` trajectories + 3 projections |
| ✅ | 50 Hz cascaded controller telemetry (outer / generator / inner / FSM) |
| ✅ | 3× 20 s SVG oscilloscopes with channel toggles + crosshair |
| ✅ | DEMO replay — 5 datasets @ 50 Hz, synthetic KLT + controller synthesis |
| ✅ | Zero-dep bridge — HTTP + `/ws` + LAN + truthful status |
| ⏳ | `rclpy` live adapter — subscribe to ROS 2 topics → normalize → `/ws` |
| ⏳ | Live IMU + camera stream from hardware (ADIS16470 / BMI088) |
| ⏳ | `/ap/v1/cmd_vel` forwarding to physical ArduPilot |
| 🔜 | Bag replay + trajectory export (CSV / PCD) |
| 🔜 | Dark mode + keyboard palette (`⌘ K`) |
| 🔜 | Unit + integration tests (bridge + math) |

---

## 🤝 Contributing

Contributions welcome — especially VIO tuning, controller gains, and ROS 2 adapter work.

```bash
# Fork & branch
git checkout -b feat/your-feature

# Keep style: vanilla JS (IIFE), no build step, stdlib Python
# Run bridge locally and verify DEMO + WS reconnect

# Commit (conventional)
git commit -m "feat: add dark-mode toggle"

# Push & open PR
git push origin feat/your-feature
```

- Open an [Issue](https://github.com/iamakhilan/Drones-Dashboard/issues) for bugs / ideas first if the change is large.
- Match existing code style — no framework introductions without discussion.
- Update topic contracts / math tables if you change gains or rates.

---

## 📄 License

[MIT](LICENSE) — © AeroVIO contributors.

---

<div align="center">

**Built for autonomous flight. Designed for clarity.**

*If AeroVIO helped your research or build, leave a ⭐ — it helps others find it.*

<br/>

<sub>Ground Station · VIO · SO(3) Control · 50 Hz · MSCKF · PCD Mapping</sub>

</div>
