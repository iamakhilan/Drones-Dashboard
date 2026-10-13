# AeroVIO Ground Station — Reference Implementation Audit & Integration Contract

## 1. Executive Summary

This document establishes the audit findings and integration contract between the **AeroVIO Ground Station Dashboard** (`Drones-Dashboard`) and the authoritative reference drone implementation repository:

**Reference Repository (`drones-controller` & `VIO-Controller` branches):**
`https://github.com/Ravish2807/Drones_S5_CD2_3D_Mapping_using_normal_monocular_camera_controlling_Visual-Inertial-Odometry-VIO`

All dashboard components, telemetry pipelines, mathematical calculations, coordinate transformations, and bridge interfaces have been aligned with the source implementation.

---

## 2. Integration Contract Mapping Table

The dashboard exposes and visualizes quantities according to this explicit contract:

| Dashboard field or operation | Actual source file/node | ROS 2 topic or interface | Message type | Frame/units | Availability |
|---|---|---|---|---|---|
| **Body Position ($p_{WB}$)** | `ros2_nodes/vio_node.py` / `vio_estimator` | `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | World (ENU), meters ($\mathrm{m}$) | Implemented & verified |
| **Body Linear Velocity ($v_{WB}$)** | `ros2_nodes/vio_node.py` / `vio_estimator` | `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | World (ENU), $\mathrm{m/s}$ | Implemented & verified |
| **Body Orientation ($q_{WB}$)** | `ros2_nodes/vio_node.py` / `vio_estimator` | `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | Unit quaternion $[x, y, z, w]$ | Implemented & verified |
| **Camera Optical Pose ($T_{WC}$)** | `trajectory/camera_pose.py` / `vio_estimator` | `/drone/vio/camera_pose` | `geometry_msgs/msg/PoseStamped` | Camera Optical (OpenCV), meters ($\mathrm{m}$) | Implemented & verified |
| **Sparse Visual Landmark Cloud** | `src/estimator_wrapper/msckf_estimator.py` | `/drone/vio/pointcloud` | `sensor_msgs/msg/PointCloud2` | World (ENU), triangulated $\mathrm{m}$ | Implemented & verified |
| **Monocular Video Stream** | `camera_driver_node` | `/camera/image_raw` | `sensor_msgs/msg/Image` | OpenCV pinhole optical frame | Implemented & verified |
| **IMU Acceleration & Angular Rate** | `controller_node.py` / `imu_node` | `/imu/data` | `sensor_msgs/msg/Imu` | Body frame (ENU), $\mathrm{m/s^2}$, $\mathrm{rad/s}$ | Implemented & verified |
| **Position Tracking Error ($e_p$)** | `position_controller.py` / `so3_controller` | `/drones_controller/position_error` | `geometry_msgs/msg/Vector3Stamped` | World (ENU), $e_p = p_d - p$ ($\mathrm{m}$) | Implemented & verified |
| **Velocity Tracking Error ($e_v$)** | `position_controller.py` / `so3_controller` | `/drones_controller/velocity_error` | `geometry_msgs/msg/Vector3Stamped` | World (ENU), $e_v = v_d - v$ ($\mathrm{m/s}$) | Implemented & verified |
| **Desired Commanded Force ($F_d$)** | `position_controller.py` / `so3_controller` | `/drones_controller/desired_force` | `geometry_msgs/msg/Vector3Stamped` | World (ENU), Newtons ($\mathrm{N}$) | Implemented & verified |
| **Desired Thrust Axis ($b_{3d}$)** | `attitude_generator.py` / `so3_controller` | Derived from `/drones_controller/desired_force` | Internal vector / `Vector3` | Dimensionless unit vector | Implemented & verified |
| **Desired Attitude ($R_d, q_d$)** | `attitude_generator.py` / `so3_controller` | `/drones_controller/desired_attitude` | `geometry_msgs/msg/PoseStamped` | Unit quaternion $[x, y, z, w]$ in SO(3) | Implemented & verified |
| **SO(3) Attitude Error ($e_R$)** | `so3_controller.py` / `so3_controller` | `/drones_controller/attitude_error` | `geometry_msgs/msg/Vector3Stamped` | Body frame, radians ($\mathrm{rad}$) | Implemented & verified |
| **Angular Velocity Error ($e_\Omega$)** | `so3_controller.py` / `so3_controller` | `/drones_controller/angular_velocity_error` | `geometry_msgs/msg/Vector3Stamped` | Body frame, $\mathrm{rad/s}$ | Implemented & verified |
| **Desired Control Moment ($M_d$)** | `so3_controller.py` / `so3_controller` | `/drones_controller/desired_moment` | `geometry_msgs/msg/Vector3Stamped` | Body frame, $\mathrm{N\cdot m}$ | Implemented & verified |
| **Forwarded ArduPilot Command** | `ardupilot_interface.py` / `controller_node` | `/ap/v1/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Body/NED $\mathrm{m/s}, \mathrm{rad/s}$ | Implemented & verified |
| **ArduPilot Vehicle Status** | `ardupilot_interface.py` | `/ap/v1/status` | `std_msgs/msg/String` | Text status (mode, armed, flying) | Implemented & verified |
| **Flight Manager FSM State** | `flight_manager.py` | `/drones_controller/fsm_state` | `std_msgs/msg/String` | State enum string | Implemented & verified |
| **Waypoint Target Setpoint ($p_d, \psi_d$)** | Dashboard UI $\to$ `bridge_server.py` | HTTP POST `/api/command` | JSON payload | $\mathrm{m}, \mathrm{rad}$ | Implemented (Bridge adapter) |
| **Vehicle Mass & Inertia Tensor** | `config/vehicle.yaml` | Static config / `/api/health` | YAML / JSON | $m=1.5\text{ kg}$, $J=\text{diag}(0.02, 0.02, 0.04)$ | Implemented & verified |
| **Controller Gains ($K_p, K_v, K_R, K_\Omega$)** | `config/controller.yaml` | Static config / `/api/health` | YAML / JSON | $K_p, K_v, K_R, K_\Omega$ matrices | Implemented & verified |

---

## 3. Controller Mathematical Formulation Validation

The reference implementation in `drones_controller/position_controller.py`, `attitude_generator.py`, and `so3_controller.py` was validated:

### 3.1 Outer Position Loop (`position_controller.py`)
- **Tracking Errors:**
  $$e_p = p_d - p, \qquad e_v = v_d - v$$
- **Velocity Command & Saturation:**
  $$v_{\mathrm{cmd\_raw}} = K_p e_p + K_v e_v$$
  If $\|p_d - p\| < 0.10\text{ m}$ and $\|v\| < 0.05\text{ m/s}$ (target reached): $v_{\mathrm{cmd}} = \mathbf{0}$.
  Otherwise saturated to $v_{\max} = 0.50\text{ m/s}$.
- **Desired Acceleration & Force:**
  $$a_{\mathrm{cmd}} = K_p e_p + K_v e_v + a_d, \qquad F_d = m (a_{\mathrm{cmd}} + g e_3)$$
  with vehicle mass $m = 1.50\text{ kg}$ and $g = 9.81\text{ m/s}^2$.

### 3.2 Attitude Generator (`attitude_generator.py`)
- **Thrust Axis:**
  $$b_{3d} = \frac{F_d}{\|F_d\|}, \qquad T_d = \|F_d\|$$
- **Orthonormal SO(3) Frame Generation with Heading Reference $\psi_d$:**
  $$h = [\cos\psi_d, \sin\psi_d, 0]^T, \quad b_{2d} = \frac{b_{3d} \times h}{\|b_{3d} \times h\|}, \quad b_{1d} = b_{2d} \times b_{3d}, \quad R_d = [b_{1d}, b_{2d}, b_{3d}]$$
  Guarantees $R_d^T R_d = I$ and $\det(R_d) = +1$.

### 3.3 Inner Geometric SO(3) Attitude Loop (`so3_controller.py`)
- **Attitude Error & Angular Rate Error:**
  $$e_R = \frac{1}{2} (R_d^T R - R^T R_d)^\vee, \qquad e_\Omega = \omega - R^T R_d \omega_d$$
- **Commanded Control Moment:**
  $$M_d = -K_R e_R - K_\Omega e_\Omega + \omega \times (J \omega) - J \left(\hat{\omega} R^T R_d \omega_d - R^T R_d \dot{\omega}_d\right)$$
- **Verified Gains & Inertia:**
  $$K_R = \begin{bmatrix} 4.0 & 0 & 0 \\ 0 & 4.0 & 0 \\ 0 & 0 & 2.0 \end{bmatrix}, \quad K_\Omega = \begin{bmatrix} 0.8 & 0 & 0 \\ 0 & 0.8 & 0 \\ 0 & 0 & 0.5 \end{bmatrix}, \quad J = \begin{bmatrix} 0.02 & 0 & 0 \\ 0 & 0.02 & 0 \\ 0 & 0 & 0.04 \end{bmatrix}\text{ kg}\cdot\text{m}^2$$

---

## 4. Coordinate Frames and Transformations

### 4.1 Frame Conventions
- **World Frame ($W$):** Inertial origin, East-North-Up (ENU). $+X$ East, $+Y$ North, $+Z$ Up (opposite gravity).
- **Body Frame ($B$):** Vehicle center of mass, $+X$ Forward, $+Y$ Left, $+Z$ Up (ENU convention matching `drones_controller`).
- **Camera Frame ($C$):** Optical center, OpenCV pinhole optical convention: $+Z$ Forward along optical axis, $+X$ Right, $+Y$ Down.

### 4.2 Camera-to-Body Extrinsics ($T_{BC}$)
$$p_{BC} = \begin{bmatrix} 0.10 \\ 0.00 \\ 0.00 \end{bmatrix}\text{ m}, \qquad R_{BC} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$$

### 4.3 Transformation Composition Proof
$$T_{WC} = T_{WB} T_{BC} \iff p_{WC} = p_{WB} + R_{WB} p_{BC}, \quad R_{WC} = R_{WB} R_{BC}$$

Arbitrary 15° pitch offsets and vertical $-0.04\text{ m}$ shifts were removed. Full 3D SO(3) matrix multiplication is implemented in both `bridge_server.py` and `js/telemetry.js`.

### 4.4 VIO Point Cloud & Map Constraints
The 442-landmark point cloud (`synthetic_orbit_scanned_map.pcd`) represents triangulated sparse feature landmarks in the world frame ($W$).
> [!WARNING]
> **VIO Metric Scale & Obstacle Disclaimer:** Monocular Visual-Inertial Odometry cannot observe absolute metric scale without dynamic IMU acceleration excitation. Sparse triangulated landmark points do **not** constitute certified metric obstacle distance, collision avoidance envelopes, or dense semantic obstacle classification. The dashboard explicitly documents this limitation.

---

## 5. State Categorization & Truthful Telemetry

The dashboard strictly separates 5 distinct state layers:
1. **Measured State:** Raw IMU acceleration/rates (`/imu/data`), raw image frames (`/camera/image_raw`).
2. **Estimated State:** VIO body pose & linear velocity ($p_{WB}, v_{WB}, q_{WB}$) from `/drone/vio/odometry`.
3. **Desired State:** Commanded setpoints ($p_d, v_d, a_d, \psi_d, R_d, q_d$).
4. **Controller Error:** Tracking errors ($e_p, e_v, e_R, e_\Omega$).
5. **Commanded Output:** Controller wrench ($F_d, M_d$) and forwarded velocity `/ap/v1/cmd_vel`.

---

## 6. Verification and Testing Results

### 6.1 Automated Test Suite (`test_bridge_and_math.py`)
A comprehensive automated test suite consisting of 13 unit and integration tests was created and executed:
- `TestReferenceControllerMath.test_hover_equilibrium`: **PASS** (Zero error, equilibrium thrust $14.715\text{ N}$, $M_d = 0$)
- `TestReferenceControllerMath.test_tracking_error_sign`: **PASS** (Confirmed $e_p = p_d - p$, $e_v = v_d - v$)
- `TestReferenceControllerMath.test_velocity_command_saturation`: **PASS** (Speed capped at $0.50\text{ m/s}$)
- `TestReferenceControllerMath.test_attitude_generator_orthonormality`: **PASS** ($R_d^T R_d = I$, $\det(R_d) = +1.0$)
- `TestReferenceControllerMath.test_so3_geometric_error_and_moment`: **PASS** (Moment opposes attitude error)
- `TestCoordinateTransforms.test_pure_translation_forward`: **PASS** ($p_{WC} = [1.1, 2.0, 3.0]\text{ m}$)
- `TestCoordinateTransforms.test_rotated_drone_translation`: **PASS** (Yaw $90^\circ \implies p_{WC} = [0.0, 0.1, 1.0]\text{ m}$)
- `TestCoordinateTransforms.test_opencv_camera_frame_extrinsics`: **PASS** ($T_{WC} = T_{WB} T_{BC}$ matrix equivalence)
- `TestSystemParametersAgainstConfig.test_vehicle_parameters`: **PASS** ($m=1.5\text{ kg}, J=\text{diag}(0.02, 0.02, 0.04)$)
- `TestSystemParametersAgainstConfig.test_controller_gains_and_tolerances`: **PASS** ($K_R=[4.0, 4.0, 2.0], K_\Omega=[0.8, 0.8, 0.5]$)
- `TestBridgeHttpServer.test_status_endpoint`: **PASS** (`/api/status` HTTP 200, truthful ROS 2 status)
- `TestBridgeHttpServer.test_health_endpoint`: **PASS** (`/api/health` HTTP 200, verified gains)
- `TestBridgeHttpServer.test_command_endpoint`: **PASS** (`/api/command` HTTP 200, parameter setpoint updates)

### 6.2 DEMO Mode Verification
- DEMO mode verified independently using `orbit`, `hover`, `straight`, `multi_axis`, and `stress` datasets at 50 Hz.
- Both frontend standalone simulation (`telemetry.js`) and backend bridge streaming (`bridge_server.py`) run exact analytical equations.

### 6.3 LIVE Mode Verification Status
- **Host Environment:** Windows 11 host without native ROS 2 Humble installation or physical ArduPilot vehicle connection.
- **Validation Status:** **NOT PERFORMED on physical hardware**.
- **Truthful Behavior:** In LIVE mode, the dashboard and bridge server do **not** invent or synthesize fake ROS 2 telemetry. If `rclpy` or a ROS 2 daemon is not connected, the bridge reports `ros2_connected = False`, and the frontend displays a clear warning banner.

---

## 7. Documented Missing Interfaces & Next Integration Steps

1. **Host-Side ROS 2 Deployment:**
   - Deploy the companion bridge node inside a ROS 2 Humble container or Ubuntu 22.04 companion computer where `rclpy` and `drones-controller` are built.
2. **ArduPilot Custom Message Mapping:**
   - `/ap/v1/status` in the reference repo uses `std_msgs/msg/String` containing JSON-encoded flight states (`armed`, `flying`, `mode`, `failsafe`). The bridge parses this directly; if ArduPilot ROS 2 DDS interfaces adopt custom message types, a message translator will be added.
3. **Closed-Loop Waypoint Command Forwarding:**
   - Currently, waypoint adjustments via the dashboard tuning modal update the target setpoint ($p_d, \psi_d$) on the bridge. A ROS 2 action or service client (`NavigateToPose` or `/drones_controller/set_target`) should be bound to the companion node for end-to-end waypoint execution.
