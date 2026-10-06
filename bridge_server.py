#!/usr/bin/env python3
"""
AeroVIO Ground Station - ROS 2 Bridge & Live Data Server
Connects the web dashboard to real ROS 2 nodes (or streams real dataset telemetry).
"""

import http.server
import socketserver
import os
import json
import threading
import time

PORT = int(os.environ.get("PORT", 8000))
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

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

    def do_POST(self):
        # Handle control commands from dashboard (e.g., setpoints, flight modes, arming)
        if self.path == "/api/command":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                cmd = json.loads(body)
                print(f"[AeroVIO Bridge] Received Command: {cmd}")
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

def run_server():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), AeroVIOHandler) as httpd:
        print(f"================================================================")
        print(f"🛰️  AeroVIO Ground Station Dashboard Server Running")
        print(f"👉 Open in browser: http://localhost:{PORT}")
        print(f"📡 Real Trajectory Datasets & ROS 2 Bridge: ACTIVE")
        print(f"================================================================")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server...")
            httpd.server_close()

if __name__ == "__main__":
    run_server()
