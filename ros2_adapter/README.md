# AeroVIO ROS 2 Live Adapter

Real `rclpy` node that subscribes to the AeroVIO / drones-controller ROS 2 topic
contracts (see the root [README](../README.md#-ros-2-topic-contracts)),
normalizes messages into the dashboard telemetry JSON schema, and forwards
frames to `bridge_server.py` via `POST /ingest`. The dashboard and bridge stay
identical in DEMO and LIVE modes — this process is the live data source.

## Requirements

- ROS 2 Humble (or newer) with `rclpy`, `geometry_msgs`, `nav_msgs`,
  `sensor_msgs`, `std_msgs`
- `bridge_server.py` running and reachable over HTTP

## Build & run (ament_python package)

```bash
# In a ROS 2 workspace
mkdir -p ~/ros2_ws/src && ln -s $(pwd) ~/ros2_ws/src/aerovio_adapter
cd ~/ros2_ws && colcon build --packages-select aerovio_adapter
source install/setup.bash

# Run against a local bridge
ros2 run aerovio_adapter adapter --endpoint http://127.0.0.1:8000/ingest

# Or run the script directly (sourced ROS 2 env required)
python3 adapter.py --endpoint http://<bridge-host>:8000/ingest --rate 25
```

## CLI flags

| Flag | Default | Description |
|---|---|---|
| `--endpoint` | `http://127.0.0.1:8000/ingest` | Bridge ingest URL |
| `--rate` | `25.0` | Forward rate cap (Hz) |
| `--topic-odom` | `/drone/vio/odometry` | Body odometry ($T_{WB}$), `nav_msgs/Odometry` |
| `--topic-camera-pose` | `/drone/vio/camera_pose` | Camera optical pose ($T_{WC}$), `geometry_msgs/PoseStamped` |
| `--topic-imu` | `/imu/data` | `sensor_msgs/Imu` |
| `--topic-pos-err` | `/drones_controller/position_error` | $e_p$, `Vector3Stamped` |
| `--topic-vel-err` | `/drones_controller/velocity_error` | $e_v$, `Vector3Stamped` |
| `--topic-force` | `/drones_controller/desired_force` | $F_d$, `Vector3Stamped` |
| `--topic-desired-attitude` | `/drones_controller/desired_attitude` | $R_d, q_d$, `PoseStamped` |
| `--topic-att-err` | `/drones_controller/attitude_error` | $e_R$, `Vector3Stamped` |
| `--topic-ang-vel-err` | `/drones_controller/angular_velocity_error` | $e_\Omega$, `Vector3Stamped` |
| `--topic-moment` | `/drones_controller/desired_moment` | $M_d$, `Vector3Stamped` |
| `--topic-fsm-state` | `/drones_controller/fsm_state` | FSM state, `std_msgs/String` |

## Frame schema

The adapter emits exactly the JSON the bridge broadcasts on `/ws` (see the
module docstring in `adapter.py`). If `pose_wc` is absent (no camera-pose
topic), the bridge computes $T_{WC} = T_{WB}T_{BC}$ from the static extrinsic
calibration.

## Full LIVE-mode session

```bash
# Terminal 1 — bridge in LIVE mode (truthful ros2_connected status)
python3 bridge_server.py --mode live --port 8000

# Terminal 2 — adapter (sourced ROS 2 environment)
python3 ros2_adapter/adapter.py --endpoint http://127.0.0.1:8000/ingest

# Browser — http://localhost:8000
```

Without `rclpy` the adapter exits honestly with code 2 — no synthetic data is
ever fabricated under a live label.
