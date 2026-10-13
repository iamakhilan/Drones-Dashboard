# ros2_adapter — LIVE mode adapter

`rclpy` node that subscribes to the AeroVIO controller / VIO topics (see the
README "ROS 2 Topic Contracts" table), normalizes messages into the exact JSON
telemetry schema `bridge_server.py` broadcasts in DEMO mode, and forwards
frames to the dashboard bridge.

## Transport

The adapter pushes frames to `bridge_server.py` two ways (both are additive
ingest paths; DEMO replay is unaffected):

1. **WebSocket client (default):** connects to `ws://localhost:8000/ws` and
   sends each frame as a `{"type": "telemetry", ...}` text message. The bridge
   forwards inbound telemetry frames to all other connected `/ws` clients.
2. **HTTP POST (`--no-ws` or automatic fallback):** POSTs each frame to
   `http://localhost:8000/ingest`, which broadcasts it to all `/ws` clients.

Run the bridge in either mode; `--mode live` just flips the reported status
flag. The browser dashboard requires no changes.

## Requirements

- ROS 2 Humble (or newer) with `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`
- Python 3.8+; stdlib only beyond ROS 2

## Build (as a ROS 2 package)

```bash
mkdir -p ~/aerovio_ws/src
ln -s $(pwd)/ros2_adapter ~/aerovio_ws/src/ros2_adapter
cd ~/aerovio_ws
colcon build --packages-select ros2_adapter
source install/setup.bash
```

## Run

```bash
# Terminal 1 — dashboard bridge
python3 bridge_server.py --mode live

# Terminal 2 — adapter (sourced ROS 2 env)
python3 ros2_adapter/adapter.py --endpoint http://localhost:8000 --rate 50
```

### Options

| Flag | Default | Description |
|---|---|---|
| `--endpoint` | `http://localhost:8000` | Bridge HTTP base URL |
| `--rate` | `50.0` | Forward rate (Hz) |
| `--no-ws` | off | Only use HTTP `/ingest`, no websocket client |
| `--topic-pose` | `/ap/v1/pose/filtered` | Filtered pose (`PoseStamped`) |
| `--topic-twist` | `/ap/v1/twist/filtered` | Filtered twist (`TwistStamped`) |
| `--topic-vio-odom` | `/drone/vio/odometry` | VIO odometry (`Odometry`) |
| `--topic-vio-camera-pose` | `/drone/vio/camera_pose` | Camera optical pose (`PoseStamped`) |
| `--topic-imu` | `/imu/data_raw` | Raw IMU (`Imu`) |
| `--topic-position-error` | `/drones_controller/position_error` | e_p (`Vector3`) |
| `--topic-velocity-error` | `/drones_controller/velocity_error` | e_v (`Vector3`) |
| `--topic-desired-force` | `/drones_controller/desired_force` | F_d (`Vector3`) |
| `--topic-desired-attitude` | `/drones_controller/desired_attitude` | q_d (`Quaternion`) |
| `--topic-attitude-error` | `/drones_controller/attitude_error` | e_R (`Vector3`) |
| `--topic-angular-velocity-error` | `/drones_controller/angular_velocity_error` | e_ω (`Vector3`) |
| `--topic-desired-moment` | `/drones_controller/desired_moment` | M_d (`Vector3`) |

If `rclpy` is not importable, the module still parses/imports cleanly (for CI
syntax checks) and exits with a clear error when run.
