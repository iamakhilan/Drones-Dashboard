/**
 * ============================================================================
 * AeroVIO Ground Control Station - WebSocket Bridge Client
 * Connects to ROS 2 backend / Bridge server with configurable endpoint.
 * Ingests topics from drones-controller & VIO ROS 2 nodes.
 * ============================================================================
 */

(function(window) {
  'use strict';

  class WebSocketBridge {
    constructor() {
      // Configurable endpoint (can point to friend's machine IP on LAN)
      this.defaultUrl = (window.APP_CONFIG && window.APP_CONFIG.websocketUrl) || 'ws://localhost:8000/ws';
      this.url = this.defaultUrl;
      this.socket = null;
      
      // Connection states: 'DISCONNECTED', 'CONNECTING', 'CONNECTED', 'DEGRADED'
      this.status = 'DISCONNECTED';
      this.reconnectAttempts = 0;
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

      // Watch telemetry mode
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
     * Establish WebSocket connection to specified or default endpoint
     */
    connect(customUrl) {
      if (customUrl) {
        this.url = customUrl;
        if (window.APP_CONFIG) window.APP_CONFIG.websocketUrl = customUrl;
      } else {
        this.url = (window.APP_CONFIG && window.APP_CONFIG.websocketUrl) || this.defaultUrl;
      }
      
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
          console.log(`[AeroVIO Bridge] Connected to ROS 2 endpoint: ${this.url}`);
        };

        this.socket.onmessage = (event) => {
          this._handleMessage(event.data);
        };

        this.socket.onerror = (error) => {
          console.warn(`[AeroVIO Bridge] Connection error on ${this.url}`);
          this._setStatus('DEGRADED');
        };

        this.socket.onclose = (event) => {
          this._stopPing();
          this._setStatus('DISCONNECTED');
          this.socket = null;

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
        // Fallback to REST API
        fetch('/api/command', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(command)
        }).then(res => res.json()).then(data => {
          console.log('[AeroVIO REST Command Response]:', data);
        }).catch(err => {
          console.warn('[AeroVIO Command Note]: Backend offline (Architectural mode active)', err);
        });
        return false;
      }
    }

    onStatusChange(callback) {
      this.listeners.statusChange.push(callback);
    }

    onCameraFrame(callback) {
      this.listeners.cameraFrame.push(callback);
    }

    onPointCloud(callback) {
      this.listeners.pointCloud.push(callback);
    }

    _setStatus(status) {
      this.status = status;
      
      if (window.telemetry) {
        const isConn = (status === 'CONNECTED');
        window.telemetry.state.connection.websocket = isConn;
        window.telemetry.state.connection.state = isConn 
          ? (window.telemetry.mode === 'LIVE' ? 'BRIDGE LIVE' : 'BRIDGE DEMO') 
          : `BRIDGE ${status}`;
        window.telemetry.state.connection.latency_ms = this.latencyMs;
        window.telemetry.state.health.websocket = isConn ? 'ONLINE' : (status === 'CONNECTING' ? 'READY' : 'DISCONNECTED');
        
        // A WebSocket connection means bridge is connected; ROS 2 status is updated truthfully from server payloads
        if (!isConn) {
          window.telemetry.state.connection.ros = false;
          if (window.telemetry.mode === 'LIVE') {
            window.telemetry.setDisconnectedState();
          }
        }
      }

      this.listeners.statusChange.forEach(cb => cb(status));
    }

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
      
      console.log(`[AeroVIO Bridge] Reconnecting to ${this.url} in ${(delay/1000).toFixed(1)}s...`);
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

        // Ping-pong latency
        if (msg.type === 'pong' && this.lastPingSent) {
          this.latencyMs = +(performance.now() - this.lastPingSent).toFixed(1);
          if (window.telemetry) {
            window.telemetry.state.connection.latency_ms = this.latencyMs;
            if (msg.ros2_connected !== undefined) {
              window.telemetry.state.connection.ros = !!msg.ros2_connected;
            }
          }
          return;
        }

        // Check for server broadcast telemetry with connection flags
        if (msg.type === 'telemetry' && msg.ros2_connected !== undefined && window.telemetry) {
          window.telemetry.state.connection.ros = !!msg.ros2_connected;
        }

        // Camera frame stream
        if (msg.topic === '/camera/image' || msg.topic === '/camera/image_raw' || msg.topic === '/camera/vio_overlay' || msg.type === 'camera_frame') {
          this.listeners.cameraFrame.forEach(cb => cb(msg.data));
          return;
        }

        // Point cloud stream
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
     * Translate drones-controller & VIO ROS 2 topic formats into Normalized Telemetry
     */
    _mapRosToTelemetry(msg) {
      const update = { timestamp: Date.now(), controller: {} };

      // 1. /ap/v1/pose/filtered (geometry_msgs/PoseStamped)
      if (msg.topic === '/ap/v1/pose/filtered') {
        const p = msg.data?.pose?.position || msg.data?.position || {};
        const q = msg.data?.pose?.orientation || msg.data?.orientation || {};
        if (p.x !== undefined) update.position = { x: p.x, y: p.y, z: p.z };
        if (q.w !== undefined) {
          const euler = window.telemetry.quaternionToEuler(q.w, q.x, q.y, q.z);
          update.orientation = { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw, qw: q.w, qx: q.x, qy: q.y, qz: q.z };
          if (p.x !== undefined) {
            update.camera_pose = window.telemetry.computeCameraPose(p.x, p.y, p.z, euler.roll, euler.pitch, euler.yaw);
          }
        }
      }

      // 2. /ap/v1/twist/filtered (geometry_msgs/TwistStamped)
      if (msg.topic === '/ap/v1/twist/filtered') {
        const v = msg.data?.twist?.linear || msg.data?.linear || {};
        const w = msg.data?.twist?.angular || msg.data?.angular || {};
        if (v.x !== undefined) update.velocity = { vx: v.x, vy: v.y, vz: v.z, norm: Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) };
        if (w.x !== undefined) update.orientation = Object.assign(update.orientation || {}, { roll_rate: w.x, pitch_rate: w.y, yaw_rate: w.z });
      }

      // 3. /ap/v1/status (ardupilot_msgs/Status)
      if (msg.topic === '/ap/v1/status') {
        const st = msg.data || {};
        update.controller.armed = !!st.armed;
        update.controller.failsafe = !!st.failsafe;
        update.controller.flight_state = st.flight_state || (st.armed ? 'NAVIGATING' : 'IDLE');
        update.controller.watchdog_status = st.failsafe ? 'FAILSAFE' : 'OK';
      }

      // 4. /drones_controller/position_error (geometry_msgs/Vector3)
      if (msg.topic === '/drones_controller/position_error') {
        const ep = msg.data || {};
        const norm = Math.sqrt((ep.x||0)**2 + (ep.y||0)**2 + (ep.z||0)**2);
        update.controller.position_error = { x: ep.x, y: ep.y, z: ep.z, norm };
      }

      // 5. /drones_controller/velocity_error (geometry_msgs/Vector3)
      if (msg.topic === '/drones_controller/velocity_error') {
        const ev = msg.data || {};
        const norm = Math.sqrt((ev.x||0)**2 + (ev.y||0)**2 + (ev.z||0)**2);
        update.controller.velocity_error = { x: ev.x, y: ev.y, z: ev.z, norm };
      }

      // 6. /drones_controller/desired_force (geometry_msgs/Vector3)
      if (msg.topic === '/drones_controller/desired_force') {
        const fd = msg.data || {};
        const norm = Math.sqrt((fd.x||0)**2 + (fd.y||0)**2 + (fd.z||0)**2);
        update.controller.desired_force = { x: fd.x, y: fd.y, z: fd.z, norm };
        update.controller.attitude_generator = {
          b3d: norm > 0 ? { x: fd.x/norm, y: fd.y/norm, z: fd.z/norm } : { x: 0, y: 0, z: 1 },
          collective_thrust: norm
        };
      }

      // 7. /drones_controller/desired_moment (geometry_msgs/Vector3)
      if (msg.topic === '/drones_controller/desired_moment') {
        const md = msg.data || {};
        const norm = Math.sqrt((md.x||0)**2 + (md.y||0)**2 + (md.z||0)**2);
        update.controller.so3_controller = update.controller.so3_controller || {};
        update.controller.so3_controller.desired_moment = { x: md.x, y: md.y, z: md.z, norm };
      }

      // 8. /drones_controller/attitude_error (geometry_msgs/Vector3)
      if (msg.topic === '/drones_controller/attitude_error') {
        const er = msg.data || {};
        const norm = Math.sqrt((er.x||0)**2 + (er.y||0)**2 + (er.z||0)**2);
        update.controller.so3_controller = update.controller.so3_controller || {};
        update.controller.so3_controller.attitude_error = { x: er.x, y: er.y, z: er.z, norm };
      }

      // 9. /drones_controller/angular_velocity_error (geometry_msgs/Vector3)
      if (msg.topic === '/drones_controller/angular_velocity_error') {
        const ew = msg.data || {};
        const norm = Math.sqrt((ew.x||0)**2 + (ew.y||0)**2 + (ew.z||0)**2);
        update.controller.so3_controller = update.controller.so3_controller || {};
        update.controller.so3_controller.angular_velocity_error = { x: ew.x, y: ew.y, z: ew.z, norm };
      }

      // 10. /drones_controller/desired_attitude (geometry_msgs/Quaternion)
      if (msg.topic === '/drones_controller/desired_attitude') {
        const qd = msg.data || {};
        update.controller.attitude_generator = update.controller.attitude_generator || {};
        update.controller.attitude_generator.desired_attitude = qd;
      }

      // 11. /drone/vio/odometry (nav_msgs/Odometry)
      if (msg.topic === '/drone/vio/odometry') {
        const odom = msg.data || {};
        const p = odom.pose?.pose?.position || odom.position;
        const q = odom.pose?.pose?.orientation || odom.orientation;
        const v = odom.twist?.twist?.linear || odom.linear_velocity;
        if (p) update.position = { x: p.x, y: p.y, z: p.z };
        if (v) update.velocity = { vx: v.x, vy: v.y, vz: v.z, norm: Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) };
        if (q) {
          const euler = window.telemetry.quaternionToEuler(q.w, q.x, q.y, q.z);
          update.orientation = { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw, qw: q.w, qx: q.x, qy: q.y, qz: q.z };
          if (p) update.camera_pose = window.telemetry.computeCameraPose(p.x, p.y, p.z, euler.roll, euler.pitch, euler.yaw);
        }
      }

      // 12. /imu/data (sensor_msgs/Imu)
      if (msg.topic === '/imu/data') {
        const imu = msg.data || {};
        const a = imu.linear_acceleration || {};
        if (a.x !== undefined) update.acceleration = { ax: a.x, ay: a.y, az: a.z, norm: Math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z) };
      }

      // Direct full telemetry payload format from bridge_server
      if (msg.type === 'telemetry' && msg.data) {
        Object.assign(update, msg.data);
      }

      window.telemetry.updateFromNormalizedData(update);
    }
  }

  // Expose singleton on window
  window.wsBridge = new WebSocketBridge();

})(window);
