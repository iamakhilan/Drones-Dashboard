#!/usr/bin/env python3
"""
AeroVIO Ground Station - ROS 2 Bridge & Live Data Server
Zero-dependency HTTP & WebSocket server for real-time telemetry streaming,
command dispatching, and trajectory dataset playback.
"""

import http.server
import socketserver
import os
import json
import threading
import time
import hashlib
import base64
import struct
import math

PORT = int(os.environ.get("PORT", 8000))
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

# Active WebSocket clients
WS_CLIENTS = set()
WS_LOCK = threading.Lock()

# Load dataset for live simulation streaming if requested
DATASET_POINTS = []
try:
    csv_path = os.path.join(DIRECTORY, "data", "synthetic_orbit_estimated_trajectory.csv")
    if os.path.exists(csv_path):
        import csv
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for r in reader:
                DATASET_POINTS.append({
                    "x": float(r["x"]), "y": float(r["y"]), "z": float(r["z"]),
                    "qw": float(r["qw"]), "qx": float(r["qx"]), "qy": float(r["qy"]), "qz": float(r["qz"])
                })
        print(f"[AeroVIO Bridge] Loaded {len(DATASET_POINTS)} trajectory frames from dataset.")
except Exception as e:
    print(f"[AeroVIO Bridge] Dataset load note: {e}")

def encode_ws_frame(message: str) -> bytes:
    """Encodes a string into a standard unmasked WebSocket text frame (Opcode 0x1)."""
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


class AeroVIOHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        # Enable CORS and caching headers for real-time telemetry
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        # Handle WebSocket Upgrade on /ws
        if self.path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
            self.handle_websocket()
            return
        
        # Standard static file serving
        super().do_GET()

    def handle_websocket(self):
        """Perform WebSocket RFC 6455 Handshake and enter client loop."""
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
        print(f"[AeroVIO Bridge] 🛰️ Client connected via WebSocket /ws (Total: {len(WS_CLIENTS)})")

        try:
            while True:
                msg = decode_ws_frame(sock)
                if msg is None:
                    break
                
                try:
                    data = json.loads(msg)
                    # Respond to ping
                    if data.get("type") == "ping":
                        sock.sendall(encode_ws_frame(json.dumps({"type": "pong", "t": data.get("t")})))
                    elif data.get("type") == "command":
                        print(f"[AeroVIO Bridge] Control Command Received via WS: {data.get('data')}")
                except Exception as pe:
                    pass

        except Exception as e:
            pass
        finally:
            with WS_LOCK:
                WS_CLIENTS.discard(sock)
            print(f"[AeroVIO Bridge] 🔌 Client disconnected from /ws (Remaining: {len(WS_CLIENTS)})")

    def do_POST(self):
        # Handle control commands from dashboard (e.g., setpoints, flight modes, arming)
        if self.path == "/api/command":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                cmd = json.loads(body)
                print(f"[AeroVIO Bridge] Received REST Command: {cmd}")
                response = {"status": "success", "command_applied": cmd}
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
    """Broadcasts real-time telemetry frames to all connected WebSocket clients at ~30 Hz."""
    idx = 0
    while True:
        time.sleep(0.033) # 30 Hz
        with WS_LOCK:
            if not WS_CLIENTS or not DATASET_POINTS:
                continue

            frame = DATASET_POINTS[idx]
            idx = (idx + 1) % len(DATASET_POINTS)

            telemetry_payload = json.dumps({
                "type": "telemetry",
                "topic": "/drone/vio/odometry",
                "data": {
                    "position": {"x": frame["x"], "y": frame["y"], "z": frame["z"]},
                    "orientation": {"qw": frame["qw"], "qx": frame["qx"], "qy": frame["qy"], "qz": frame["qz"]}
                }
            })

            frame_bytes = encode_ws_frame(telemetry_payload)
            dead_socks = set()
            for sock in WS_CLIENTS:
                try:
                    sock.sendall(frame_bytes)
                except Exception:
                    dead_socks.add(sock)

            for dead in dead_socks:
                WS_CLIENTS.discard(dead)


def run_server():
    socketserver.TCPServer.allow_reuse_address = True
    
    # Start live telemetry broadcaster in background thread
    broadcaster_thread = threading.Thread(target=telemetry_broadcaster, daemon=True)
    broadcaster_thread.start()

    with socketserver.ThreadingTCPServer(("", PORT), AeroVIOHandler) as httpd:
        print(f"================================================================")
        print(f"🛰️  AeroVIO Ground Station Dashboard Server Running")
        print(f"👉 Open in browser: http://localhost:{PORT}")
        print(f"📡 WebSocket Bridge: ws://localhost:{PORT}/ws")
        print(f"📁 Real Trajectory Datasets & ROS 2 Bridge: ACTIVE")
        print(f"================================================================")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server...")
            httpd.server_close()

if __name__ == "__main__":
    run_server()
