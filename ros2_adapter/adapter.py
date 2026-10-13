#!/usr/bin/env python3
"""
===============================================================================
AeroVIO — ROS 2 Live Telemetry Adapter
===============================================================================
rclpy node that subscribes to the controller / VIO topics documented in the
README "ROS 2 Topic Contracts" section, normalizes the messages into the exact
JSON telemetry schema `bridge_server.py` broadcasts in DEMO mode, and pushes
frames to the dashboard bridge at `/ingest` (HTTP POST) and/or over a raw
WebSocket client connection to `ws://localhost:8000/ws`.

Run with a sourced ROS 2 environment:
    ros2 run ros2_adapter adapter --ros-args ...
or directly:
    python3 ros2_adapter/adapter.py --endpoint http://localhost:8000 --rate 50

Without rclpy installed the module still imports (see HAS_RCLPY) so CI can
syntax-check it; running it requires ROS 2 Humble+.
===============================================================================
"""

import argparse
import json
import sys
import time
import urllib.request

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    HAS_RCLPY = True
except ImportError:  # pragma: no cover - allows py_compile/import on non-ROS CI
    HAS_RCLPY = False
    Node = object  # type: ignore

# Mirror of bridge_server.py T_BC extrinsic (Body->Camera): used when
# /drone/vio/camera_pose is not available and T_WC must be derived.
T_BC_OFFSET = {"x": 0.10, "y": 0.00, "z": -0.04}

DEFAULT_TOPICS = {
    "pose": "/ap/v1/pose/filtered",
    "twist": "/ap/v1/twist/filtered",
    "vio_odom": "/drone/vio/odometry",
    "vio_camera_pose": "/drone/vio/camera_pose",
    "imu": "/imu/data_raw",
    "position_error": "/drones_controller/position_error",
    "velocity_error": "/drones_controller/velocity_error",
    "desired_force": "/drones_controller/desired_force",
    "desired_attitude": "/drones_controller/desired_attitude",
    "attitude_error": "/drones_controller/attitude_error",
    "angular_velocity_error": "/drones_controller/angular_velocity_error",
    "desired_moment": "/drones_controller/desired_moment",
}


def _v3(msg):
    return {"x": round(msg.x, 4), "y": round(msg.y, 4), "z": round(msg.z, 4)}


def _v3n(msg):
    v = _v3(msg)
    v["norm"] = round((msg.x ** 2 + msg.y ** 2 + msg.z ** 2) ** 0.5, 4)
    return v


class TelemetryState:
    """Holds the latest sample of every subscribed topic."""

    def __init__(self):
        self.position = None          # Vector3-like dict
        self.orientation = None       # {qw,qx,qy,qz}
        self.lin_vel = None           # {vx,vy,vz}
        self.ang_vel = None           # {p,q,r}
        self.cam_position = None
        self.cam_orientation = None
        self.accel = None             # {ax,ay,az}
        self.position_error = None
        self.velocity_error = None
        self.desired_force = None
        self.desired_attitude = None
        self.attitude_error = None
        self.angular_velocity_error = None
        self.desired_moment = None
        self.last_update = 0.0

    def build_frame(self):
        """Assemble the exact schema bridge_server.py broadcasts."""
        now = time.time()
        p = self.position or {"x": 0.0, "y": 0.0, "z": 0.0}
        q = self.orientation or {"qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}
        if self.cam_position is not None:
            p_wc = self.cam_position
            q_wc = self.cam_orientation or q
        else:
            p_wc = {"x": round(p["x"] + T_BC_OFFSET["x"], 4),
                    "y": round(p["y"] + T_BC_OFFSET["y"], 4),
                    "z": round(p["z"] + T_BC_OFFSET["z"], 4)}
            q_wc = q
        lv = self.lin_vel or {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        av = self.ang_vel or {"p": 0.0, "q": 0.0, "r": 0.0}
        accel = self.accel or {"ax": 0.0, "ay": 0.0, "az": 9.81}

        def _or_zero(v, with_norm=True):
            if v is None:
                d = {"x": 0.0, "y": 0.0, "z": 0.0}
                if with_norm:
                    d["norm"] = 0.0
                return d
            return v

        controller = {
            "synthetic": False,
            "position_error": _or_zero(self.position_error),
            "velocity_error": _or_zero(self.velocity_error),
            "desired_force": _or_zero(self.desired_force),
            "attitude_generator": {
                "b3d": _or_zero(None, with_norm=False),
                "collective_thrust": (self.desired_force or {}).get("norm", 0.0),
                "desired_attitude": self.desired_attitude or
                    {"qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0},
            },
            "so3_controller": {
                "attitude_error": _or_zero(self.attitude_error),
                "angular_velocity_error": _or_zero(self.angular_velocity_error),
                "desired_moment": _or_zero(self.desired_moment),
            },
            "flight_state": "NAVIGATING",
            "watchdog_status": "OK",
        }

        return {
            "type": "telemetry",
            "source": "ros2_live",
            "bridge_connected": True,
            "ros2_connected": True,
            "timestamp": now,
            "data": {
                "pose_wb": {"position": p, "orientation": q},
                "pose_wc": {"position": p_wc, "orientation": q_wc},
                "twist": {"linear": lv, "angular": av},
                "imu": {
                    "accel": accel,
                    "bias_accel": [0.0, 0.0, 0.0],
                    "bias_gyro": [0.0, 0.0, 0.0],
                },
                "vio": {"status": "ACTIVE"},
                "controller": controller,
            },
        }


class WsClient:
    """Minimal RFC 6455 client for pushing frames to ws://host:port/ws.

    Matches the zero-dependency philosophy of bridge_server.py — stdlib only.
    """

    def __init__(self, url):
        self.url = url
        self.sock = None

    @staticmethod
    def _parse(url):
        from urllib.parse import urlparse
        u = urlparse(url)
        return u.hostname or "localhost", u.port or 80, u.path or "/ws"

    def connect(self):
        import base64
        import hashlib
        import os
        import socket
        host, port, path = self._parse(self.url)
        self.sock = socket.create_connection((host, port), timeout=5.0)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        req = (
            f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(req.encode("ascii"))
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("WS handshake failed")
            resp += chunk
        if b"101" not in resp.split(b"\r\n", 1)[0]:
            raise ConnectionError("WS upgrade refused: " + resp.decode(errors="ignore").splitlines()[0])

    def send_text(self, message):
        import struct
        payload = message.encode("utf-8")
        header = bytearray([0x81])  # FIN + text opcode; client frames are masked below
        length = len(payload)
        mask_bit = 0x80
        if length <= 125:
            header.append(mask_bit | length)
        elif length <= 65535:
            header.append(mask_bit | 126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(mask_bit | 127)
            header.extend(struct.pack("!Q", length))
        import os
        mask = os.urandom(4)
        header.extend(mask)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None


class BridgeForwarder:
    """Forwards normalized frames to bridge_server.py via HTTP /ingest and/or WS."""

    def __init__(self, endpoint, use_ws=True):
        self.endpoint = endpoint.rstrip("/")
        self.use_ws = use_ws
        self.ws = None

    def _ensure_ws(self):
        if not self.use_ws:
            return
        if self.ws is None or self.ws.sock is None:
            self.ws = WsClient(f"{self.endpoint.replace('http', 'ws', 1)}/ws")
            self.ws.connect()

    def send(self, frame):
        body = json.dumps(frame)
        if self.use_ws:
            try:
                self._ensure_ws()
                self.ws.send_text(body)
                return
            except Exception:
                if self.ws:
                    self.ws.close()
                # fall through to HTTP ingest
        req = urllib.request.Request(
            f"{self.endpoint}/ingest",
            data=body.encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=2.0).read()


if HAS_RCLPY:
    class AeroVIOAdapter(Node):
        def __init__(self, topics, forwarder, rate_hz):
            super().__init__("aerovio_dashboard_adapter")
            self.state = TelemetryState()
            self.forwarder = forwarder
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(PoseStamped, topics["pose"], self._on_pose, qos)
            self.create_subscription(TwistStamped, topics["twist"], self._on_twist, qos)
            self.create_subscription(Odometry, topics["vio_odom"], self._on_vio_odom, qos)
            self.create_subscription(PoseStamped, topics["vio_camera_pose"], self._on_cam_pose, qos)
            self.create_subscription(Imu, topics["imu"], self._on_imu, qos)
            self.create_subscription(Vector3, topics["position_error"], lambda m: self._set("position_error", _v3n(m)), qos)
            self.create_subscription(Vector3, topics["velocity_error"], lambda m: self._set("velocity_error", _v3n(m)), qos)
            self.create_subscription(Vector3, topics["desired_force"], lambda m: self._set("desired_force", _v3n(m)), qos)
            self.create_subscription(Quaternion, topics["desired_attitude"],
                                     lambda m: self._set("desired_attitude", {"qw": m.w, "qx": m.x, "qy": m.y, "qz": m.z}), qos)
            self.create_subscription(Vector3, topics["attitude_error"], lambda m: self._set("attitude_error", _v3n(m)), qos)
            self.create_subscription(Vector3, topics["angular_velocity_error"], lambda m: self._set("angular_velocity_error", _v3n(m)), qos)
            self.create_subscription(Vector3, topics["desired_moment"], lambda m: self._set("desired_moment", _v3n(m)), qos)
            self.create_timer(1.0 / rate_hz, self._tick)
            self.get_logger().info(f"AeroVIO adapter forwarding @ {rate_hz} Hz -> {forwarder.endpoint}")

        def _set(self, attr, value):
            setattr(self.state, attr, value)
            self.state.last_update = time.time()

        @staticmethod
        def _pq(pose):
            return ({"x": round(pose.position.x, 4), "y": round(pose.position.y, 4), "z": round(pose.position.z, 4)},
                    {"qw": round(pose.orientation.w, 4), "qx": round(pose.orientation.x, 4),
                     "qy": round(pose.orientation.y, 4), "qz": round(pose.orientation.z, 4)})

        def _on_pose(self, msg):
            self.state.position, self.state.orientation = self._pq(msg.pose)
            self.state.last_update = time.time()

        def _on_twist(self, msg):
            self.state.lin_vel = {"vx": round(msg.twist.linear.x, 3), "vy": round(msg.twist.linear.y, 3), "vz": round(msg.twist.linear.z, 3)}
            self.state.ang_vel = {"p": round(msg.twist.angular.x, 4), "q": round(msg.twist.angular.y, 4), "r": round(msg.twist.angular.z, 4)}
            self.state.last_update = time.time()

        def _on_vio_odom(self, msg):
            self.state.position, self.state.orientation = self._pq(msg.pose.pose)
            self.state.lin_vel = {"vx": round(msg.twist.twist.linear.x, 3), "vy": round(msg.twist.twist.linear.y, 3), "vz": round(msg.twist.twist.linear.z, 3)}
            self.state.ang_vel = {"p": round(msg.twist.twist.angular.x, 4), "q": round(msg.twist.twist.angular.y, 4), "r": round(msg.twist.twist.angular.z, 4)}
            self.state.last_update = time.time()

        def _on_cam_pose(self, msg):
            self.state.cam_position, self.state.cam_orientation = self._pq(msg.pose)
            self.state.last_update = time.time()

        def _on_imu(self, msg):
            self.state.accel = {"ax": round(msg.linear_acceleration.x, 3),
                                "ay": round(msg.linear_acceleration.y, 3),
                                "az": round(msg.linear_acceleration.z, 3)}
            self.state.ang_vel = {"p": round(msg.angular_velocity.x, 4),
                                  "q": round(msg.angular_velocity.y, 4),
                                  "r": round(msg.angular_velocity.z, 4)}

        def _tick(self):
            frame = self.state.build_frame()
            try:
                self.forwarder.send(frame)
            except Exception as e:
                self.get_logger().warn(f"forward failed: {e}", throttle_duration_sec=5.0)


def main(argv=None):
    parser = argparse.ArgumentParser(description="AeroVIO ROS 2 -> dashboard bridge adapter")
    parser.add_argument("--endpoint", default="http://localhost:8000",
                        help="Bridge HTTP base URL (default: http://localhost:8000)")
    parser.add_argument("--rate", type=float, default=50.0, help="Forward rate Hz (default: 50)")
    parser.add_argument("--no-ws", action="store_true", help="Only use HTTP /ingest, no websocket client")
    for name, default in DEFAULT_TOPICS.items():
        parser.add_argument(f"--topic-{name.replace('_', '-')}", default=default, help=f"Topic for {name} (default: {default})")
    args = parser.parse_args(argv)

    if not HAS_RCLPY:
        print("[ros2_adapter] ERROR: rclpy not available. Source a ROS 2 Humble+ environment first.",
              file=sys.stderr)
        sys.exit(1)

    topics = {name: getattr(args, f"topic_{name}") for name in DEFAULT_TOPICS}
    forwarder = BridgeForwarder(args.endpoint, use_ws=not args.no_ws)
    rclpy.init()
    node = AeroVIOAdapter(topics, forwarder, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        forwarder.ws and forwarder.ws.close()


if __name__ == "__main__":
    main()
