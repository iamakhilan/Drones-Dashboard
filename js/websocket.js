/**
 * ============================================================================
 * AeroVIO Ground Control Station - WebSocket Bridge Client
 * Connects browser frontend to ROS 2 backend / Bridge server.
 * Handles telemetry subscriptions, ping-pong latency, and reconnection.
 * ============================================================================
 */

(function(window) {
  'use strict';

  class WebSocketBridge {
    constructor() {
      this.defaultUrl = 'ws://localhost:8000/ws';
      this.url = this.defaultUrl;
      this.socket = null;
      
      // Connection states: 'DISCONNECTED', 'CONNECTING', 'CONNECTED', 'DEGRADED'
      this.status = 'DISCONNECTED';
      this.reconnectAttempts = 0;
      this.maxReconnectAttempts = 10;
      this.reconnectTimer = null;
      this.pingTimer = null;
      this.lastPingSent = 0;
      this.latencyMs = null;
      this.connectStartTime = 0;

      // Event listeners
      this.listeners = {
        statusChange: [],
        rawMessage: [],
        cameraFrame: [],
        pointCloud: []
      };

      // Watch telemetry mode: if in LIVE mode, start connection
      if (window.telemetry) {
        window.telemetry.onStateChange((state) => {
          if (state.connection.mode === 'LIVE' && this.status === 'DISCONNECTED') {
            this.connect();
          } else if (state.connection.mode === 'DEMO' && (this.status === 'CONNECTED' || this.status === 'CONNECTING')) {
            this.disconnect();
          }
        });
      }
    }

    /**
     * Establish WebSocket connection
     */
    connect(customUrl) {
      if (customUrl) this.url = customUrl;
      
      if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) {
        return;
      }

      this._setStatus('CONNECTING');
      this.connectStartTime = Date.now();

      try {
        this.socket = new WebSocket(this.url);

        this.socket.onopen = (event) => {
          this._setStatus('CONNECTED');
          this.reconnectAttempts = 0;
          this._startPing();
          console.log(`[AeroVIO Bridge] Connected to ${this.url}`);
        };

        this.socket.onmessage = (event) => {
          this._handleMessage(event.data);
        };

        this.socket.onerror = (error) => {
          // Log without uncaught exception
          console.warn('[AeroVIO Bridge] WebSocket connection error / server not found');
          this._setStatus('DEGRADED');
        };

        this.socket.onclose = (event) => {
          this._stopPing();
          this._setStatus('DISCONNECTED');
          this.socket = null;

          // If in LIVE mode, schedule reconnect
          if (window.telemetry && window.telemetry.mode === 'LIVE') {
            this._scheduleReconnect();
          }
        };

      } catch (err) {
        console.warn('[AeroVIO Bridge] Connection exception:', err);
        this._setStatus('DISCONNECTED');
        if (window.telemetry && window.telemetry.mode === 'LIVE') {
          this._scheduleReconnect();
        }
      }
    }

    /**
     * Close connection explicitly
     */
    disconnect() {
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = null;
      }
      this._stopPing();
      if (this.socket) {
        this.socket.close();
        this.socket = null;
      }
      this._setStatus('DISCONNECTED');
    }

    /**
     * Send command payload to ROS 2 backend
     */
    sendCommand(command) {
      const payload = JSON.stringify({
        type: 'command',
        timestamp: Date.now(),
        data: command
      });

      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this.socket.send(payload);
        return true;
      } else {
        // Fallback to REST API if WebSocket is offline
        fetch('/api/command', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(command)
        }).then(res => res.json()).then(data => {
          console.log('[AeroVIO REST Command Response]:', data);
        }).catch(err => {
          console.warn('[AeroVIO Command Failed]: Server offline', err);
        });
        return false;
      }
    }

    /**
     * Subscribe to connection status changes
     */
    onStatusChange(callback) {
      this.listeners.statusChange.push(callback);
    }

    /**
     * Subscribe to raw camera frames from WebSocket
     */
    onCameraFrame(callback) {
      this.listeners.cameraFrame.push(callback);
    }

    /**
     * Subscribe to point cloud streams
     */
    onPointCloud(callback) {
      this.listeners.pointCloud.push(callback);
    }

    /**
     * Set connection status and notify subsystems
     */
    _setStatus(status) {
      this.status = status;
      
      if (window.telemetry) {
        const isConn = (status === 'CONNECTED');
        window.telemetry.state.connection.websocket = isConn;
        window.telemetry.state.connection.ros = isConn;
        window.telemetry.state.connection.state = isConn ? 'ROS 2 CONNECTED' : `ROS 2 ${status}`;
        window.telemetry.state.connection.latency_ms = this.latencyMs;
        window.telemetry.state.health.websocket = isConn ? 'ONLINE' : (status === 'CONNECTING' ? 'READY' : 'DISCONNECTED');
        
        if (!isConn && window.telemetry.mode === 'LIVE') {
          window.telemetry.setDisconnectedState();
        }
      }

      this.listeners.statusChange.forEach(cb => cb(status));
    }

    /**
     * Heartbeat ping-pong
     */
    _startPing() {
      this._stopPing();
      this.pingTimer = setInterval(() => {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
          this.lastPingSent = performance.now();
          this.socket.send(JSON.stringify({ type: 'ping', t: this.lastPingSent }));
        }
      }, 2000);
    }

    _stopPing() {
      if (this.pingTimer) {
        clearInterval(this.pingTimer);
        this.pingTimer = null;
      }
    }

    _scheduleReconnect() {
      if (this.reconnectTimer) return;
      this.reconnectAttempts++;
      const delay = Math.min(1000 * Math.pow(1.5, this.reconnectAttempts), 10000);
      
      console.log(`[AeroVIO Bridge] Reconnecting in ${(delay/1000).toFixed(1)}s (Attempt ${this.reconnectAttempts})...`);
      this.reconnectTimer = setTimeout(() => {
        this.reconnectTimer = null;
        if (window.telemetry && window.telemetry.mode === 'LIVE') {
          this.connect();
        }
      }, delay);
    }

    /**
     * Parse incoming messages from ROS 2 bridge
     */
    _handleMessage(raw) {
      try {
        const msg = JSON.parse(raw);

        // Pong response
        if (msg.type === 'pong' && this.lastPingSent) {
          this.latencyMs = +(performance.now() - this.lastPingSent).toFixed(1);
          if (window.telemetry) {
            window.telemetry.state.connection.latency_ms = this.latencyMs;
          }
          return;
        }

        // Camera frame stream (/camera/image or /camera/vio_overlay)
        if (msg.topic === '/camera/image' || msg.topic === '/camera/vio_overlay' || msg.type === 'camera_frame') {
          this.listeners.cameraFrame.forEach(cb => cb(msg.data));
          return;
        }

        // Point cloud stream (/drone/vio/pointcloud)
        if (msg.topic === '/drone/vio/pointcloud' || msg.type === 'pointcloud') {
          this.listeners.pointCloud.forEach(cb => cb(msg.data));
          if (window.telemetry) {
            window.telemetry.state.landmarks = msg.data;
          }
          return;
        }

        // Map ROS 2 topics to Normalized Telemetry Model
        if (window.telemetry && window.telemetry.mode === 'LIVE') {
          this._mapRosToTelemetry(msg);
        }

      } catch (err) {
        console.error('[AeroVIO Bridge] Error parsing message:', err);
      }
    }

    /**
     * Translate ROS 2 message formats into Normalized Telemetry
     */
    _mapRosToTelemetry(msg) {
      const update = { timestamp: Date.now() };

      // /drone/vio/odometry (nav_msgs/Odometry)
      if (msg.topic === '/drone/vio/odometry' || msg.odometry) {
        const odom = msg.data || msg.odometry;
        const p = odom.pose?.pose?.position || odom.position || {};
        const q = odom.pose?.pose?.orientation || odom.orientation || {};
        const v = odom.twist?.twist?.linear || odom.linear_velocity || {};

        if (p.x !== undefined) {
          update.position = { x: p.x, y: p.y, z: p.z };
        }
        if (v.x !== undefined) {
          update.velocity = { vx: v.x, vy: v.y, vz: v.z, norm: Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) };
        }
        if (q.w !== undefined) {
          const euler = window.telemetry.quaternionToEuler(q.w, q.x, q.y, q.z);
          update.orientation = {
            roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw,
            qw: q.w, qx: q.x, qy: q.y, qz: q.z
          };
          if (p.x !== undefined) {
            update.camera_pose = window.telemetry.computeCameraPose(p.x, p.y, p.z, euler.roll, euler.pitch, euler.yaw);
          }
        }
        update.health = Object.assign({}, window.telemetry.state.health, { vio: 'ACTIVE', imu: 'ONLINE' });
      }

      // /drone/vio/camera_pose (geometry_msgs/PoseStamped)
      if (msg.topic === '/drone/vio/camera_pose') {
        const cp = msg.data?.pose?.position || msg.data?.position;
        const cq = msg.data?.pose?.orientation || msg.data?.orientation;
        if (cp) {
          const ceuler = cq ? window.telemetry.quaternionToEuler(cq.w, cq.x, cq.y, cq.z) : { roll: 0, pitch: -15, yaw: 0 };
          update.camera_pose = { x: cp.x, y: cp.y, z: cp.z, roll: ceuler.roll, pitch: ceuler.pitch, yaw: ceuler.yaw };
        }
      }

      // /imu/data (sensor_msgs/Imu)
      if (msg.topic === '/imu/data' || msg.imu) {
        const imu = msg.data || msg.imu;
        const a = imu.linear_acceleration || imu.accel || {};
        if (a.x !== undefined) {
          update.acceleration = { ax: a.x, ay: a.y, az: a.z, norm: Math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z) };
        }
      }

      // Direct full telemetry payload format from bridge_server
      if (msg.type === 'telemetry') {
        Object.assign(update, msg.data);
      }

      window.telemetry.updateFromNormalizedData(update);
    }
  }

  // Expose singleton on window
  window.wsBridge = new WebSocketBridge();

})(window);
