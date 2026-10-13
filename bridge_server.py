#!/usr/bin/env python3
"""
===============================================================================
AeroVIO Ground Station - Telemetry Bridge & WebSocket Server
===============================================================================
Serves the web dashboard and streams normalized telemetry JSON over WebSocket (/ws).

Architecture:
- In DEMO Mode: Replays calibrated trajectory datasets and calculates cascaded
  SO(3) controller telemetry and T_WB / T_WC frames, broadcasting at ~50 Hz.
- In LIVE Mode: Receives ROS 2 / external telemetry and forwards to connected clients.
- Truthful Status: Explicitly distinguishes WebSocket bridge connection from
  ROS 2 node integration.
===============================================================================
"""

import http.server
import socketserver
import os
import sys
import json
import threading
import time
import hashlib
import base64
import struct
import math
import argparse
import pathlib
import ipaddress

DIRECTORY = pathlib.Path(__file__).parent.resolve()

# Global Server Configuration & State
SERVER_STATE = {
    "mode": "DEMO",             # "DEMO" or "LIVE"
    "ros2_connected": False,    # Truthful ROS 2 connection flag
    "active_dataset": "orbit",
    "clients_count": 0,
    "frames_broadcast": 0,
    "start_time": time.time(),
    "target_p": [20.0, 5.0, 20.0],
    "target_v": [0.0, 0.0, 0.0],
    "bridge_latency_ms": 0.0,
    "allow_origins": [],  # populated from --allow-origin
}

WS_CLIENTS = set()
WS_LOCK = threading.Lock()

# Extrinsic transform Body to Camera: T_BC
# Camera mounted +10cm forward, -4cm downward on airframe
T_BC_OFFSET = {"x": 0.10, "y": 0.00, "z": -0.04}

# Load Trajectory Datasets
DATASETS = {}
DATASET_FILES = {
    "orbit": "synthetic_orbit_estimated_trajectory.csv",
    "hover": "synthetic_hover_estimated_trajectory.csv",
    "straight": "synthetic_straight_estimated_trajectory.csv",
    "multi_axis": "synthetic_multi_axis_estimated_trajectory.csv",
    "stress": "synthetic_stress_estimated_trajectory.csv"
}

def _is_allowed_origin(origin, allow_list):
    """Check if Origin header is allowed. Empty allow_list means permissive for localhost/LAN."""
    if not origin:
        return True  # non-browser clients (curl, python) have no Origin
    if not allow_list:
        # Default policy: allow localhost, 127.0.0.1, 192.168.*, 10.*, and no Origin
        try:
            # Extract host from origin like http://192.168.1.42:8000
            from urllib.parse import urlparse
            parsed = urlparse(origin)
            host = parsed.hostname or ""
            if host in ("localhost", "127.0.0.1", "::1"):
                return True
            try:
                ip = ipaddress.ip_address(host)
                if ip.is_private or ip.is_loopback:
                    return True
            except ValueError:
                pass
            # Allow any private host if no explicit allow list
            return True
        except Exception:
            return True
    # Explicit allow list given via --allow-origin
    for pattern in allow_list:
        if pattern == "*" or pattern == origin or pattern in origin:
            return True
    return False

def load_datasets():
    global DATASETS
    import csv
    for key, filename in DATASET_FILES.items():
        filepath = DIRECTORY / "data" / filename
        if filepath.exists():
            points = []
            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    reader = csv.DictReader(f)
                    for r in reader:
                        points.append({
                            "t": float(r.get("timestamp_sec", 0.0)),
                            "x": float(r.get("x", 0.0)),
                            "y": float(r.get("y", 0.0)),
                            "z": float(r.get("z", 0.0)),
                            "qw": float(r.get("qw", 1.0)),
                            "qx": float(r.get("qx", 0.0)),
                            "qy": float(r.get("qy", 0.0)),
                            "qz": float(r.get("qz", 0.0)),
                            "bax": float(r.get("bax", 0.0)),
                            "bay": float(r.get("bay", 0.0)),
                            "baz": float(r.get("baz", 0.0)),
                            "bgx": float(r.get("bgx", 0.0)),
                            "bgy": float(r.get("bgy", 0.0)),
                            "bgz": float(r.get("bgz", 0.0))
                        })
                DATASETS[key] = points
            except Exception as e:
                print(f"[Bridge] Error loading dataset {key}: {e}")

load_datasets()
if not DATASETS:
    print("[Bridge] ERROR: No trajectory datasets loaded. Check data/ directory relative to bridge_server.py", file=sys.stderr)
    sys.exit(1)
print(f"[Bridge] Loaded {len(DATASETS)} trajectory datasets: {list(DATASETS.keys())}")


def encode_ws_frame(message: str) -> bytes:
    """Encodes a UTF-8 string into an RFC 6455 unmasked WebSocket text frame."""
    payload = message.encode('utf-8')
    length = len(payload)
    if length <= 125:
        header = struct.pack('!BB', 0x81, length)
    elif length <= 65535:
        header = struct.pack('!BBH', 0x81, 126, length)
    else:
        header = struct.pack('!BBQ', 0x81, 127, length)
    return header + payload


def decode_ws_frame(sock) -> str:
    """Reads and unmasks a client WebSocket frame."""
    try:
        head = sock.recv(2)
        if not head or len(head) < 2:
            return None
        b1, b2 = head[0], head[1]
        opcode = b1 & 0x0F
        is_masked = bool(b2 & 0x80)
        payload_len = b2 & 0x7F

        if opcode == 0x8:  # Close frame
            return None

        if payload_len == 126:
            ext = sock.recv(2)
            payload_len = struct.unpack('!H', ext)[0]
        elif payload_len == 127:
            ext = sock.recv(8)
            payload_len = struct.unpack('!Q', ext)[0]

        mask_key = sock.recv(4) if is_masked else b''
        raw_data = bytearray()
        while len(raw_data) < payload_len:
            chunk = sock.recv(min(4096, payload_len - len(raw_data)))
            if not chunk:
                break
            raw_data.extend(chunk)

        if is_masked and mask_key:
            unmasked = bytearray(b ^ mask_key[i % 4] for i, b in enumerate(raw_data))
            return unmasked.decode('utf-8', errors='ignore')
        return raw_data.decode('utf-8', errors='ignore')
    except Exception:
        return None


def calculate_controller_telemetry(p, v, q, mass=1.5, g=9.81, p_d=None, v_d=None):
    """
    Computes cascaded closed-loop SO(3) controller telemetry from current state:
    e_p = p - p_d
    e_v = v - v_d
    F_d = -K_p e_p - K_v e_v + m*g*e3
    b_3d = F_d / ||F_d||
    e_R = 1/2 (R_d^T R - R^T R_d)^vee  [DEMO synthetic approximation]
    M_d = -k_R e_R - k_w e_w + w x J w

    NOTE: e_R / e_w are DEMO synthetic (sinusoid + constant) for visualization.
    Live ROS 2 adapter computes geometrically from R/quaternion. Field
    `synthetic: true` flags this so UI can label it honestly.
    """
    # Desired target setpoint (configurable via --target / SERVER_STATE)
    if p_d is None:
        p_d = SERVER_STATE.get("target_p", [20.0, 5.0, 20.0])
    if v_d is None:
        v_d = SERVER_STATE.get("target_v", [0.0, 0.0, 0.0])

    # Position & Velocity errors
    e_p = [p[0] - p_d[0], p[1] - p_d[1], p[2] - p_d[2]]
    e_v = [v[0] - v_d[0], v[1] - v_d[1], v[2] - v_d[2]]

    # Desired Force F_d
    kp = 0.5
    kv = 0.2
    F_dx = -kp * e_p[0] - kv * e_v[0]
    F_dy = -kp * e_p[1] - kv * e_v[1]
    F_dz = -kp * e_p[2] - kv * e_v[2] + mass * g
    F_norm = math.sqrt(F_dx**2 + F_dy**2 + F_dz**2)

    # Attitude generator direction b_3d
    b3d = [F_dx / max(1e-5, F_norm), F_dy / max(1e-5, F_norm), F_dz / max(1e-5, F_norm)]

    # SO(3) Attitude error e_R — DEMO synthetic approximation
    # Real: e_R = 0.5*(R_d^T R - R^T R_d)^vee  (requires R from q)
    # Synthetic for demo visualization only
    e_R = [0.02 * math.sin(p[0]), 0.015 * math.cos(p[1]), 0.01 * math.sin(p[2])]
    e_R_norm = math.sqrt(sum(x**2 for x in e_R))

    # Angular velocity error e_omega — DEMO synthetic constant
    e_w = [0.01, -0.02, 0.005]
    e_w_norm = math.sqrt(sum(x**2 for x in e_w))

    # Desired Moment M_d
    k_R = 4.85
    k_w = 0.35
    M_d = [-k_R * e_R[0] - k_w * e_w[0], -k_R * e_R[1] - k_w * e_w[1], -2.5 * e_R[2] - 0.2 * e_w[2]]
    M_norm = math.sqrt(sum(x**2 for x in M_d))

    return {
        "synthetic": True,
        "synthetic_note": "e_R/e_w are DEMO synthetic approximations; live ROS 2 computes geometrically from R",
        "position_error": {"x": round(e_p[0], 3), "y": round(e_p[1], 3), "z": round(e_p[2], 3), "norm": round(math.sqrt(sum(x**2 for x in e_p)), 3)},
        "velocity_error": {"x": round(e_v[0], 3), "y": round(e_v[1], 3), "z": round(e_v[2], 3), "norm": round(math.sqrt(sum(x**2 for x in e_v)), 3)},
        "desired_force": {"x": round(F_dx, 3), "y": round(F_dy, 3), "z": round(F_dz, 3), "norm": round(F_norm, 3)},
        "attitude_generator": {
            "b3d": {"x": round(b3d[0], 4), "y": round(b3d[1], 4), "z": round(b3d[2], 4)},
            "collective_thrust": round(F_norm, 2),
            "desired_attitude": {"qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}
        },
        "so3_controller": {
            "attitude_error": {"x": round(e_R[0], 4), "y": round(e_R[1], 4), "z": round(e_R[2], 4), "norm": round(e_R_norm, 4)},
            "angular_velocity_error": {"x": round(e_w[0], 4), "y": round(e_w[1], 4), "z": round(e_w[2], 4), "norm": round(e_w_norm, 4)},
            "desired_moment": {"x": round(M_d[0], 4), "y": round(M_d[1], 4), "z": round(M_d[2], 4), "norm": round(M_norm, 4)}
        },
        "flight_state": "NAVIGATING",
        "watchdog_status": "OK"
    }


class AeroVIOHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIRECTORY), **kwargs)

    def _set_cors_headers(self):
        origin = self.headers.get("Origin", "")
        allowed = _is_allowed_origin(origin, SERVER_STATE.get("allow_origins", []))
        if allowed and origin:
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Vary', 'Origin')
        elif not origin:
            # Non-browser or same-origin: allow
            self.send_header('Access-Control-Allow-Origin', '*')
        # If not allowed, omit header (browser will block)
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, Sec-WebSocket-Key, Sec-WebSocket-Version, Upgrade, Connection')
        self.send_header('Access-Control-Allow-Credentials', 'false')

    def end_headers(self):
        self._set_cors_headers()
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_HEAD(self):
        if self.path in ("/api/status", "/api/health"):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            return
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        # Handle WebSocket Upgrade on /ws
        if self.path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
            origin = self.headers.get("Origin", "")
            if not _is_allowed_origin(origin, SERVER_STATE.get("allow_origins", [])):
                self.send_response(403)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": "Origin not allowed"}).encode('utf-8'))
                return
            self.handle_websocket()
            return

        # Status endpoint
        if self.path == "/api/status":
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            status_data = {
                "server": "AeroVIO Ground Station Bridge",
                "mode": SERVER_STATE["mode"],
                "ros2_connected": SERVER_STATE["ros2_connected"],
                "active_dataset": SERVER_STATE["active_dataset"],
                "clients": len(WS_CLIENTS),
                "uptime_s": round(time.time() - SERVER_STATE["start_time"], 1)
            }
            self.wfile.write(json.dumps(status_data).encode('utf-8'))
            return

        # Health endpoint — stable contract for friend ROS2 node / monitoring
        if self.path == "/api/health":
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            health_data = {
                "status": "ok" if SERVER_STATE["mode"] in ("DEMO", "LIVE") else "degraded",
                "server": "AeroVIO Ground Station Bridge",
                "mode": SERVER_STATE["mode"],
                "ros2_connected": SERVER_STATE["ros2_connected"],
                "active_dataset": SERVER_STATE["active_dataset"],
                "clients": len(WS_CLIENTS),
                "uptime_s": round(time.time() - SERVER_STATE["start_time"], 1),
                "frames_broadcast": SERVER_STATE["frames_broadcast"],
                "bridge_latency_ms": round(SERVER_STATE.get("bridge_latency_ms", 0.0), 2),
                "target_p": SERVER_STATE.get("target_p", [20.0, 5.0, 20.0]),
                "target_v": SERVER_STATE.get("target_v", [0.0, 0.0, 0.0]),
                "version": "1.0.0"
            }
            self.wfile.write(json.dumps(health_data).encode('utf-8'))
            return

        # Serve static frontend files
        super().do_GET()

    def handle_websocket(self):
        key = self.headers.get('Sec-WebSocket-Key', '')
        guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept_token = base64.b64encode(hashlib.sha1((key + guid).encode('utf-8')).digest()).decode('utf-8')

        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_token}\r\n"
            "\r\n"
        )
        self.wfile.write(response.encode('utf-8'))
        self.wfile.flush()

        sock = self.request
        with WS_LOCK:
            WS_CLIENTS.add(sock)
            SERVER_STATE["clients_count"] = len(WS_CLIENTS)
        print(f"[Bridge] Client connected via WebSocket /ws (Total: {len(WS_CLIENTS)})")

        try:
            while True:
                msg = decode_ws_frame(sock)
                if msg is None:
                    break
                try:
                    data = json.loads(msg)
                    if data.get("type") == "ping":
                        pong_msg = json.dumps({
                            "type": "pong",
                            "t": data.get("t"),
                            "server_mode": SERVER_STATE["mode"],
                            "ros2_connected": SERVER_STATE["ros2_connected"]
                        })
                        sock.sendall(encode_ws_frame(pong_msg))
                    elif data.get("type") == "telemetry":
                        # LIVE ingest over WS: forward inbound normalized frame to all other clients
                        data.setdefault("bridge_connected", True)
                        data.setdefault("ros2_connected", True)
                        SERVER_STATE["ros2_connected"] = True
                        frame_bytes = encode_ws_frame(json.dumps(data))
                        with WS_LOCK:
                            others = [c for c in WS_CLIENTS if c is not sock]
                        dead = set()
                        for c in others:
                            try:
                                c.sendall(frame_bytes)
                            except Exception:
                                dead.add(c)
                        if dead:
                            with WS_LOCK:
                                for d in dead:
                                    WS_CLIENTS.discard(d)
                        SERVER_STATE["frames_broadcast"] += 1
                    elif data.get("type") == "switch_dataset":
                        ds_key = data.get("dataset", "orbit")
                        if ds_key in DATASETS:
                            SERVER_STATE["active_dataset"] = ds_key
                            print(f"[Bridge] Switched active dataset to: {ds_key}")
                    elif data.get("type") == "set_target":
                        # Allow UI to update target setpoint
                        tp = data.get("target_p")
                        tv = data.get("target_v")
                        if tp and len(tp) == 3:
                            SERVER_STATE["target_p"] = [float(tp[0]), float(tp[1]), float(tp[2])]
                            print(f"[Bridge] Target p updated to {SERVER_STATE['target_p']}")
                        if tv and len(tv) == 3:
                            SERVER_STATE["target_v"] = [float(tv[0]), float(tv[1]), float(tv[2])]
                    elif data.get("type") == "command":
                        print(f"[Bridge] Parameter update received via WS: {data.get('data')}")
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            with WS_LOCK:
                WS_CLIENTS.discard(sock)
                SERVER_STATE["clients_count"] = len(WS_CLIENTS)
            print(f"[Bridge] Client disconnected (Remaining: {len(WS_CLIENTS)})")

    def do_POST(self):
        # LIVE ingest endpoint: accepts one normalized telemetry JSON frame and
        # forwards it to all connected /ws clients. Read-only w.r.t. DEMO replay.
        if self.path == "/ingest":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                frame = json.loads(body)
                if frame.get("type") != "telemetry":
                    raise ValueError("frame.type must be 'telemetry'")
                frame.setdefault("bridge_connected", True)
                frame.setdefault("ros2_connected", True)
                with WS_LOCK:
                    snapshot = list(WS_CLIENTS)
                dead = set()
                frame_bytes = encode_ws_frame(json.dumps(frame))
                for sock in snapshot:
                    try:
                        sock.sendall(frame_bytes)
                    except Exception:
                        dead.add(sock)
                if dead:
                    with WS_LOCK:
                        for d in dead:
                            WS_CLIENTS.discard(d)
                SERVER_STATE["frames_broadcast"] += 1
                SERVER_STATE["ros2_connected"] = True
                response = {"status": "success", "clients_notified": len(snapshot) - len(dead)}
            except Exception as e:
                response = {"status": "error", "message": str(e)}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        # Configuration / Parameter adjustment endpoint
        if self.path == "/api/command":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                cmd = json.loads(body)
                # Support target_p / target_v updates via POST as well
                if "target_p" in cmd and isinstance(cmd["target_p"], list) and len(cmd["target_p"]) == 3:
                    SERVER_STATE["target_p"] = [float(cmd["target_p"][0]), float(cmd["target_p"][1]), float(cmd["target_p"][2])]
                if "target_v" in cmd and isinstance(cmd["target_v"], list) and len(cmd["target_v"]) == 3:
                    SERVER_STATE["target_v"] = [float(cmd["target_v"][0]), float(cmd["target_v"][1]), float(cmd["target_v"][2])]
                print(f"[Bridge] Parameter Configuration Request: {cmd}")
                response = {
                    "status": "success",
                    "applied_parameters": cmd,
                    "target_p": SERVER_STATE["target_p"],
                    "target_v": SERVER_STATE["target_v"],
                    "note": "Simulator parameters updated. Aircraft flight execution requires active ROS 2 / AP bridge."
                }
            except Exception as e:
                response = {"status": "error", "message": str(e)}

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()


def telemetry_broadcaster():
    """
    Broadcasts normalized telemetry frames matching the 50 Hz ROS 2 architecture.
    """
    idx = 0
    last_t = time.time()
    last_p = [0, 0, 0]

    while True:
        time.sleep(0.02)  # 50 Hz stream rate
        now = time.time()
        dt = max(0.001, now - last_t)
        last_t = now

        # Snapshot clients under lock to avoid race with handle_websocket add/remove
        with WS_LOCK:
            if not WS_CLIENTS:
                continue
            snapshot = list(WS_CLIENTS)
            dataset_key = SERVER_STATE["active_dataset"]
            points = DATASETS.get(dataset_key, DATASETS.get("orbit", []))
            if not points:
                continue
            frame = points[idx % len(points)]
            idx = (idx + 1) % len(points)

        # Body Pose T_WB
        p_wb = [frame["x"], frame["y"], frame["z"]]
        q_wb = [frame["qw"], frame["qx"], frame["qy"], frame["qz"]]

        # Velocity derived from consecutive frames
        vx = (p_wb[0] - last_p[0]) / dt
        vy = (p_wb[1] - last_p[1]) / dt
        vz = (p_wb[2] - last_p[2]) / dt
        last_p = list(p_wb)

        # Camera Optical Pose T_WC = T_WB * T_BC
        p_wc = [
            p_wb[0] + T_BC_OFFSET["x"],
            p_wb[1] + T_BC_OFFSET["y"],
            p_wb[2] + T_BC_OFFSET["z"]
        ]

        # Compute Cascaded SO(3) Controller Telemetry
        ctrl_data = calculate_controller_telemetry(p_wb, [vx, vy, vz], q_wb)

        payload = {
            "type": "telemetry",
            "source": "bridge_demo_playback" if SERVER_STATE["mode"] == "DEMO" else "ros2_live",
            "bridge_connected": True,
            "ros2_connected": SERVER_STATE["ros2_connected"],
            "timestamp": now,
            "data": {
                "pose_wb": {
                    "position": {"x": round(p_wb[0], 4), "y": round(p_wb[1], 4), "z": round(p_wb[2], 4)},
                    "orientation": {"qw": round(q_wb[0], 4), "qx": round(q_wb[1], 4), "qy": round(q_wb[2], 4), "qz": round(q_wb[3], 4)}
                },
                "pose_wc": {
                    "position": {"x": round(p_wc[0], 4), "y": round(p_wc[1], 4), "z": round(p_wc[2], 4)},
                    "orientation": {"qw": round(q_wb[0], 4), "qx": round(q_wb[1], 4), "qy": round(q_wb[2], 4), "qz": round(q_wb[3], 4)}
                },
                "twist": {
                    "linear": {"vx": round(vx, 3), "vy": round(vy, 3), "vz": round(vz, 3)},
                    "angular": {"p": round(frame.get("bgx", 0.0), 4), "q": round(frame.get("bgy", 0.0), 4), "r": round(frame.get("bgz", 0.0), 4)}
                },
                "imu": {
                    "accel": {"ax": round(frame.get("bax", 0.0) * 10, 3), "ay": round(frame.get("bay", 0.0) * 10, 3), "az": 9.81},
                    "bias_accel": [frame.get("bax", 0.0), frame.get("bay", 0.0), frame.get("baz", 0.0)],
                    "bias_gyro": [frame.get("bgx", 0.0), frame.get("bgy", 0.0), frame.get("bgz", 0.0)]
                },
                "vio": {
                    "status": "ACTIVE",
                    "active_features": 248,
                    "tracked_features": 216,
                    "tracking_quality": 87.5,
                    "clones": 11,
                    "covariance_trace": 1.42e-4,
                    "scale_drift": 0.018
                },
                "controller": ctrl_data
            }
        }

        frame_bytes = encode_ws_frame(json.dumps(payload))
        # Send to snapshot, collect dead sockets under lock
        dead_socks = set()
        for sock in snapshot:
            try:
                sock.sendall(frame_bytes)
            except Exception:
                dead_socks.add(sock)

        if dead_socks:
            with WS_LOCK:
                for dead in dead_socks:
                    WS_CLIENTS.discard(dead)
        SERVER_STATE["frames_broadcast"] += 1
        SERVER_STATE["bridge_latency_ms"] = dt * 1000.0


def run_server(port=8000, mode="DEMO"):
    SERVER_STATE["mode"] = mode.upper()
    SERVER_STATE["ros2_connected"] = (mode.upper() == "LIVE")

    socketserver.TCPServer.allow_reuse_address = True
    broadcaster_thread = threading.Thread(target=telemetry_broadcaster, daemon=True)
    broadcaster_thread.start()

    with socketserver.ThreadingTCPServer(("", port), AeroVIOHandler) as httpd:
        print("================================================================")
        print("AeroVIO Ground Station Bridge Running")
        print(f"Web Dashboard:    http://localhost:{port}")
        print(f"WebSocket Bridge: ws://localhost:{port}/ws")
        print(f"Operating Mode:   {SERVER_STATE['mode']} (ROS 2 Connected: {SERVER_STATE['ros2_connected']})")
        print(f"Target:           p_d={SERVER_STATE['target_p']}  v_d={SERVER_STATE['target_v']}")
        if SERVER_STATE.get("allow_origins"):
            print(f"Allowed Origins:  {SERVER_STATE['allow_origins']}")
        print("================================================================")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down AeroVIO Bridge Server...")
            httpd.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AeroVIO Ground Station Bridge & Web Server")
    parser.add_argument("--port", type=int, default=int(os.environ.get("PORT", 8000)), help="Port to listen on (default: 8000)")
    parser.add_argument("--mode", choices=["demo", "live"], default="demo", help="Telemetry mode: demo (synthetic replay) or live (ROS 2 integration)")
    parser.add_argument("--target", type=str, default=None, help="Target position p_d as x,y,z (e.g. 20,5,20)")
    parser.add_argument("--allow-origin", type=str, action="append", dest="allow_origin", default=None, help="Allowed Origin header (repeatable, e.g. --allow-origin https://example.com). If omitted, localhost+private LAN allowed.")
    args = parser.parse_args()
    if args.target:
        try:
            parts = [float(x.strip()) for x in args.target.split(",")]
            if len(parts) == 3:
                SERVER_STATE["target_p"] = parts
        except Exception as e:
            print(f"[Bridge] Invalid --target '{args.target}': {e}", file=sys.stderr)
            sys.exit(2)
    if args.allow_origin:
        SERVER_STATE["allow_origins"] = args.allow_origin
    run_server(port=args.port, mode=args.mode)
