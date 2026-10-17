/**
 * ============================================================================
 * AeroVIO Ground Control Station - WebSocket Bridge Client
 * Connects to ROS 2 backend / Bridge server with configurable endpoint.
 * Ingests topics from drones-controller & VIO ROS 2 nodes.
 * ============================================================================
 */
(function(window) {
  'use strict';

  // APP_CONFIG now defined in js/config.js (loaded first) — no duplicate here

  var WS_URL_PATTERN = /^wss?:\/\/.+/i;
  var STORAGE_KEY = 'aerovio.wsUrl';

  function isValidWsUrl(url) {
    return typeof url === 'string' && WS_URL_PATTERN.test(url.trim());
  }

  function persistWsUrl(url) {
    try { localStorage.setItem(STORAGE_KEY, url); } catch(e) {}
    try { if (window.APP_CONFIG) window.APP_CONFIG.websocketUrl = url; } catch(e) {}
    // sync header badge if present
    var badge = document.getElementById('top-endpoint-badge');
    if (badge) badge.textContent = url;
    var tuneInput = document.getElementById('tune-ws-url');
    if (tuneInput && tuneInput.value !== url) tuneInput.value = url;
  }

  function setConnectingUI(isConnecting) {
    var btn = document.getElementById('tune-ws-url') ? document.querySelector('[onclick*="updateRemoteEndpoint"],[onclick*="updateWsEndpoint"]') : null;
    // generic connect button
    var connectBtn = document.querySelector('#tune-ws-url + button, button[onclick="updateRemoteEndpoint()"]');
    if (connectBtn) {
      connectBtn.disabled = !!isConnecting;
      if (isConnecting) {
        if (!connectBtn.dataset.origText) connectBtn.dataset.origText = connectBtn.textContent;
        connectBtn.innerHTML = '<svg class="animate-spin h-3 w-3 inline mr-1" viewBox="0 0 24 24" fill="none"><circle class="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" stroke-width="4"></circle><path class="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4z"></path></svg> Connecting...';
        connectBtn.classList.add('opacity-60','cursor-not-allowed');
      } else {
        if (connectBtn.dataset.origText) connectBtn.textContent = connectBtn.dataset.origText;
        else connectBtn.textContent = 'Connect Bridge';
        connectBtn.classList.remove('opacity-60','cursor-not-allowed');
      }
    }
    var wsInput = document.getElementById('tune-ws-url');
    if (wsInput) wsInput.disabled = !!isConnecting;
    // header mode toggle also disabled while connecting to prevent race
    var modeTog = document.getElementById('top-mode-toggle-btn');
    // do not fully disable mode toggle, just visual hint via latency badge handled in _setStatus
  }

  class WebSocketBridge {
    constructor() {
      // Configurable endpoint (can point to friend's machine IP on LAN)
      this.defaultUrl = (window.APP_CONFIG && window.APP_CONFIG.websocketUrl) || 'ws://localhost:8000/ws';
      // hydrate from localStorage again in case class instantiated late
      try {
        var ls = localStorage.getItem(STORAGE_KEY);
        if (ls && isValidWsUrl(ls)) this.defaultUrl = ls;
      } catch(e) {}
      this.url = this.defaultUrl;
      this.socket = null;
      
      // Connection states: 'DISCONNECTED', 'CONNECTING', 'CONNECTED', 'DEGRADED'
      this.status = 'DISCONNECTED';
      this.reconnectAttempts = 0;
      this.reconnectTimer = null;
      this.reconnectCountdownTimer = null;
      this.reconnectDelayMs = 0;
      this.reconnectCountdownSec = 0;
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
      // hydrate wsUrl badge on load
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function(){ persistWsUrl(this.url); }.bind(this));
      } else {
        // defer to ensure DOM parsed
        setTimeout(function(){ persistWsUrl(this.url); }.bind(this), 0);
      }
      // Watch telemetry mode
      if (window.telemetry) {
        window.telemetry.onStateChange((state) => {
          if (state.connection.mode === 'LIVE' && this.status === 'DISCONNECTED') {
            this.connect();
          } else if (state.connection.mode === 'DEMO' && (this.status === 'CONNECTED' || this.status === 'CONNECTING' || this.status === 'DEGRADED')) {
            this.disconnect();
          }
        });
      }
    }

    /**
     * Establish WebSocket connection to specified or default endpoint
     */
    connect(customUrl) {
      if (customUrl !== undefined && customUrl !== null && String(customUrl).trim() !== '') {
        var candidate = String(customUrl).trim();
        if (!isValidWsUrl(candidate)) {
          console.warn('[AeroVIO Bridge] Invalid WebSocket URL (must match ^wss?://):', candidate);
          if (window.ui && window.ui.showToast) window.ui.showToast('Invalid WebSocket URL — must start with ws:// or wss://', 'error');
          else console.error('Invalid WebSocket URL');
          this._exposeValidationError(candidate);
          return false;
        }
        this.url = candidate;
        persistWsUrl(candidate);
      } else {
        // use current or APP_CONFIG
        var fallback = (window.APP_CONFIG && window.APP_CONFIG.websocketUrl) || this.defaultUrl;
        try {
          var ls2 = localStorage.getItem(STORAGE_KEY);
          if (ls2 && isValidWsUrl(ls2)) fallback = ls2;
        } catch(e) {}
        if (!isValidWsUrl(fallback)) {
          console.warn('[AeroVIO Bridge] Stored URL invalid, falling back to default');
          fallback = this.defaultUrl;
        }
        this.url = fallback;
      }
      
      if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) {
        return true;
      }

      // clear any pending reconnect countdown
      this._clearReconnectCountdown();
      if (this.reconnectTimer) { clearTimeout(this.reconnectTimer); this.reconnectTimer = null; }

      this._setStatus('CONNECTING');
      this.connectStartTime = Date.now();

      try {
        this.socket = new WebSocket(this.url);

        this.socket.onopen = (event) => {
          this._setStatus('CONNECTED');
          this.reconnectAttempts = 0;
          this._clearReconnectCountdown();
          persistWsUrl(this.url);
          this._startPing();
          console.log(`[AeroVIO Bridge] Connected to ROS 2 endpoint: ${this.url}`);
          if (window.ui && window.ui.showToast) window.ui.showToast('Bridge CONNECTED: ' + this.url, 'success');
        };

        this.socket.onmessage = (event) => {
          this._handleMessage(event.data);
        };

        this.socket.onerror = (error) => {
          console.warn(`[AeroVIO Bridge] Connection error on ${this.url}`);
          // Proper DEGRADED handling: distinguish from DISCONNECTED
          // Keep socket alive if possible, mark degraded but do not immediately disconnect
          this._setStatus('DEGRADED');
          // propagate degraded to health without wiping stale timers
          if (window.telemetry) {
            window.telemetry.state.health.websocket = 'DEGRADED';
            window.telemetry.state.connection.state = 'BRIDGE DEGRADED';
          }
        };

        this.socket.onclose = (event) => {
          this._stopPing();
          // if we were DEGRADED and close was clean, move to DISCONNECTED
          var wasDegraded = this.status === 'DEGRADED';
          this._setStatus('DISCONNECTED');
          this.socket = null;

          if (wasDegraded) {
            console.log('[AeroVIO Bridge] Degraded connection closed, scheduling reconnect');
          }

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
        return false;
      }
      return true;
    }

    /**
     * Close connection explicitly
     */
    disconnect() {
      this._clearReconnectCountdown();
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = null;
      }
      this._stopPing();
      if (this.socket) {
        try { this.socket.close(); } catch(e) {}
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

    getReconnectCountdown() {
      return this.reconnectCountdownSec;
    }

    _exposeValidationError(url) {
      var el = document.getElementById('tune-ws-url');
      if (el) {
        el.classList.add('border-red-500','ring-1','ring-red-300');
        setTimeout(function(){ el.classList.remove('border-red-500','ring-1','ring-red-300'); }, 1800);
      }
    }

    _setStatus(status) {
      this.status = status;
      
      // Spinner / disable handling during CONNECTING
      if (status === 'CONNECTING') setConnectingUI(true);
      else setConnectingUI(false);

      // Proper DEGRADED handling
      var isConn = (status === 'CONNECTED');
      var isDegraded = (status === 'DEGRADED');
      var isConnecting = (status === 'CONNECTING');

      if (window.telemetry) {
        window.telemetry.state.connection.websocket = isConn;
        // keep degraded as distinct: state = DEGRADED, health = DEGRADED
        if (isDegraded) {
          window.telemetry.state.connection.state = 'BRIDGE DEGRADED';
          window.telemetry.state.health.websocket = 'DEGRADED';
        } else {
          window.telemetry.state.connection.state = isConn 
            ? (window.telemetry.mode === 'LIVE' ? 'BRIDGE LIVE' : 'BRIDGE DEMO') 
            : (isConnecting ? 'BRIDGE CONNECTING' : `BRIDGE ${status}`);
          window.telemetry.state.health.websocket = isConn ? 'ONLINE' : (isConnecting ? 'READY' : (isDegraded ? 'DEGRADED' : 'DISCONNECTED'));
        }
        window.telemetry.state.connection.latency_ms = this.latencyMs;
        
        if (!isConn && !isDegraded && !isConnecting) {
          window.telemetry.state.connection.ros = false;
          if (window.telemetry.mode === 'LIVE') {
            window.telemetry.setDisconnectedState();
          }
        }
        // when degraded, keep ros flag as-is but mark latency null
        if (isDegraded) {
          // don't wipe ros flag, allow stale banner logic to handle
        }
      }

      // Update header badges for DEGRADED as amber
      this._updateHeaderForStatus(status);

      this.listeners.statusChange.forEach(cb => { try{cb(status);}catch(e){} });
    }

    _updateHeaderForStatus(status) {
      var topDot = document.getElementById('top-status-dot');
      var topPing = document.getElementById('top-status-dot-ping');
      var latBadge = document.getElementById('top-latency-badge');
      var modeBadge = document.getElementById('top-mode-badge');
      if (status === 'DEGRADED') {
        if (topDot) topDot.className = 'relative inline-flex rounded-full h-2 w-2 bg-amber-500';
        if (topPing) topPing.className = 'animate-ping absolute inline-flex h-full w-full rounded-full bg-amber-400 opacity-75';
        if (latBadge) latBadge.textContent = 'Degraded — retrying...';
        if (modeBadge) { modeBadge.textContent = 'LIVE DEGRADED'; modeBadge.className = 'font-semibold text-amber-400'; }
      } else if (status === 'CONNECTING') {
        if (topDot) topDot.className = 'relative inline-flex rounded-full h-2 w-2 bg-amber-500';
        if (topPing) topPing.className = 'animate-ping absolute inline-flex h-full w-full rounded-full bg-amber-400 opacity-75';
        if (latBadge) latBadge.textContent = 'Connecting...';
      } else if (status === 'CONNECTED') {
        if (topDot) topDot.className = 'relative inline-flex rounded-full h-2 w-2 bg-emerald-500';
        if (topPing) topPing.className = 'animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75';
        if (latBadge) latBadge.textContent = (this.latencyMs !== null ? this.latencyMs + ' ms' : '2.1 ms');
      } else if (status === 'DISCONNECTED') {
        if (topDot) topDot.className = 'relative inline-flex rounded-full h-2 w-2 bg-stone-500';
        if (topPing) topPing.className = 'hidden';
        // latency badge will be managed by reconnect countdown if scheduled
        if (!this.reconnectTimer && latBadge) latBadge.textContent = 'Offline';
      }
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

    _clearReconnectCountdown() {
      if (this.reconnectCountdownTimer) {
        clearInterval(this.reconnectCountdownTimer);
        this.reconnectCountdownTimer = null;
      }
      this.reconnectCountdownSec = 0;
    }

    _exposeReconnectCountdown() {
      var latBadge = document.getElementById('top-latency-badge');
      var countEl = document.getElementById('ws-reconnect-countdown');
      // create countdown element if not present but badge exists
      if (!countEl && latBadge && latBadge.parentElement) {
        countEl = document.createElement('span');
        countEl.id = 'ws-reconnect-countdown';
        countEl.className = 'ml-1 text-amber-400 font-mono text-xs';
        latBadge.insertAdjacentElement('afterend', countEl);
      }
      var update = () => {
        var txt = this.reconnectCountdownSec > 0 ? `Reconnect in ${this.reconnectCountdownSec}s` : 'Reconnecting...';
        if (latBadge) latBadge.textContent = txt;
        if (countEl) countEl.textContent = this.reconnectCountdownSec > 0 ? `(${this.reconnectCountdownSec}s)` : '';
        // also sidebar bridge status
        var sb = document.getElementById('sidebar-bridge-status');
        if (sb) sb.textContent = txt;
      };
      update();
      this._clearReconnectCountdown();
      this.reconnectCountdownTimer = setInterval(() => {
        this.reconnectCountdownSec = Math.max(0, this.reconnectCountdownSec - 1);
        update();
        if (this.reconnectCountdownSec <= 0) {
          this._clearReconnectCountdown();
        }
      }, 1000);
    }

    _scheduleReconnect() {
      if (this.reconnectTimer) return;
      this.reconnectAttempts++;
      const delay = Math.min(1000 * Math.pow(1.5, this.reconnectAttempts), 10000);
      this.reconnectDelayMs = delay;
      this.reconnectCountdownSec = Math.ceil(delay / 1000);
      
      console.log(`[AeroVIO Bridge] Reconnecting to ${this.url} in ${(delay/1000).toFixed(1)}s...`);
      this._exposeReconnectCountdown();
      this.reconnectTimer = setTimeout(() => {
        this.reconnectTimer = null;
        this._clearReconnectCountdown();
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
          // update badge live
          var lb = document.getElementById('top-latency-badge');
          if (lb && this.status === 'CONNECTED') lb.textContent = this.latencyMs + ' ms';
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

      // 1. /ap/v1/pose/filtered (geometry_msgs/msg/PoseStamped)
      if (msg.topic === '/ap/v1/pose/filtered') {
        const p = msg.data?.pose?.position || msg.data?.position || {};
        const q = msg.data?.pose?.orientation || msg.data?.orientation || {};
        const qx = q.x !== undefined ? q.x : (q.qx !== undefined ? q.qx : 0.0);
        const qy = q.y !== undefined ? q.y : (q.qy !== undefined ? q.qy : 0.0);
        const qz = q.z !== undefined ? q.z : (q.qz !== undefined ? q.qz : 0.0);
        const qw = q.w !== undefined ? q.w : (q.qw !== undefined ? q.qw : 1.0);
        if (p.x !== undefined) update.position = { x: p.x, y: p.y, z: p.z };
        if (qw !== undefined) {
          const euler = window.telemetry.quaternionToEuler(qw, qx, qy, qz);
          update.orientation = { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw, qw: qw, qx: qx, qy: qy, qz: qz };
          if (p.x !== undefined) {
            update.camera_pose = window.telemetry.computeCameraPose(p.x, p.y, p.z, qw, qx, qy, qz);
          }
        }
      }

      // 2. /ap/v1/twist/filtered (geometry_msgs/msg/TwistStamped)
      if (msg.topic === '/ap/v1/twist/filtered') {
        const v = msg.data?.twist?.linear || msg.data?.linear || {};
        const w = msg.data?.twist?.angular || msg.data?.angular || {};
        if (v.x !== undefined) update.velocity = { vx: v.x, vy: v.y, vz: v.z, norm: Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) };
        if (w.x !== undefined) update.orientation = Object.assign(update.orientation || {}, { roll_rate: w.x, pitch_rate: w.y, yaw_rate: w.z });
      }

      // 3. /ap/v1/status (ardupilot_msgs/msg/Status)
      if (msg.topic === '/ap/v1/status') {
        const st = msg.data || {};
        update.controller.armed = !!st.armed;
        update.controller.flying = !!st.flying;
        update.controller.failsafe = !!st.failsafe;
        update.controller.mode = st.mode;
        update.controller.flight_state = st.flight_state || (st.failsafe ? 'FAILSAFE' : (st.flying ? 'FLYING' : (st.armed ? 'ARMED' : 'GUIDED')));
        update.controller.watchdog_status = st.failsafe ? 'FAILSAFE' : 'OK';
      }

      // 4. /ap/v1/cmd_vel (geometry_msgs/msg/TwistStamped - commanded vehicle output)
      if (msg.topic === '/ap/v1/cmd_vel') {
        const v = msg.data?.twist?.linear || msg.data?.linear || {};
        const w = msg.data?.twist?.angular || msg.data?.angular || {};
        update.controller.command_velocity = {
          vx: v.x, vy: v.y, vz: v.z,
          yaw_rate: w.z,
          speed: Math.sqrt((v.x||0)**2 + (v.y||0)**2 + (v.z||0)**2)
        };
      }

      // 5. /drones_controller/position_error (geometry_msgs/msg/Vector3Stamped)
      if (msg.topic === '/drones_controller/position_error') {
        const vec = msg.data?.vector || msg.data || {};
        const norm = Math.sqrt((vec.x||0)**2 + (vec.y||0)**2 + (vec.z||0)**2);
        update.controller.position_error = { x: vec.x, y: vec.y, z: vec.z, norm };
      }

      // 6. /drones_controller/velocity_error (geometry_msgs/msg/Vector3Stamped)
      if (msg.topic === '/drones_controller/velocity_error') {
        const vec = msg.data?.vector || msg.data || {};
        const norm = Math.sqrt((vec.x||0)**2 + (vec.y||0)**2 + (vec.z||0)**2);
        update.controller.velocity_error = { x: vec.x, y: vec.y, z: vec.z, norm };
      }

      // 7. /drones_controller/desired_force (geometry_msgs/msg/Vector3Stamped)
      if (msg.topic === '/drones_controller/desired_force') {
        const vec = msg.data?.vector || msg.data || {};
        const norm = Math.sqrt((vec.x||0)**2 + (vec.y||0)**2 + (vec.z||0)**2);
        update.controller.desired_force = { x: vec.x, y: vec.y, z: vec.z, norm };
        update.controller.attitude_generator = update.controller.attitude_generator || {};
        update.controller.attitude_generator.b3d = norm > 0 ? { x: vec.x/norm, y: vec.y/norm, z: vec.z/norm } : { x: 0, y: 0, z: 1 };
        update.controller.attitude_generator.collective_thrust = norm;
      }

      // 8. /drones_controller/desired_moment (geometry_msgs/msg/Vector3Stamped)
      if (msg.topic === '/drones_controller/desired_moment') {
        const vec = msg.data?.vector || msg.data || {};
        const norm = Math.sqrt((vec.x||0)**2 + (vec.y||0)**2 + (vec.z||0)**2);
        update.controller.so3_controller = update.controller.so3_controller || {};
        update.controller.so3_controller.desired_moment = { x: vec.x, y: vec.y, z: vec.z, norm };
      }

      // 9. /drones_controller/attitude_error (geometry_msgs/msg/Vector3Stamped)
      if (msg.topic === '/drones_controller/attitude_error') {
        const vec = msg.data?.vector || msg.data || {};
        const norm = Math.sqrt((vec.x||0)**2 + (vec.y||0)**2 + (vec.z||0)**2);
        update.controller.so3_controller = update.controller.so3_controller || {};
        update.controller.so3_controller.attitude_error = { x: vec.x, y: vec.y, z: vec.z, norm };
      }

      // 10. /drones_controller/angular_velocity_error (geometry_msgs/msg/Vector3Stamped)
      if (msg.topic === '/drones_controller/angular_velocity_error') {
        const vec = msg.data?.vector || msg.data || {};
        const norm = Math.sqrt((vec.x||0)**2 + (vec.y||0)**2 + (vec.z||0)**2);
        update.controller.so3_controller = update.controller.so3_controller || {};
        update.controller.so3_controller.angular_velocity_error = { x: vec.x, y: vec.y, z: vec.z, norm };
      }

      // 11. /drones_controller/desired_attitude (geometry_msgs/msg/PoseStamped)
      if (msg.topic === '/drones_controller/desired_attitude') {
        const q = msg.data?.pose?.orientation || msg.data?.orientation || msg.data || {};
        update.controller.attitude_generator = update.controller.attitude_generator || {};
        update.controller.attitude_generator.desired_attitude = {
          qx: q.x !== undefined ? q.x : (q.qx !== undefined ? q.qx : 0.0),
          qy: q.y !== undefined ? q.y : (q.qy !== undefined ? q.qy : 0.0),
          qz: q.z !== undefined ? q.z : (q.qz !== undefined ? q.qz : 0.0),
          qw: q.w !== undefined ? q.w : (q.qw !== undefined ? q.qw : 1.0)
        };
      }

      // 12. /drone/vio/odometry (nav_msgs/msg/Odometry)
      if (msg.topic === '/drone/vio/odometry') {
        const odom = msg.data || {};
        const p = odom.pose?.pose?.position || odom.position;
        const q = odom.pose?.pose?.orientation || odom.orientation;
        const v = odom.twist?.twist?.linear || odom.linear_velocity;
        if (p) update.position = { x: p.x, y: p.y, z: p.z };
        if (v) update.velocity = { vx: v.x, vy: v.y, vz: v.z, norm: Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) };
        if (q) {
          const qx = q.x !== undefined ? q.x : (q.qx !== undefined ? q.qx : 0.0);
          const qy = q.y !== undefined ? q.y : (q.qy !== undefined ? q.qy : 0.0);
          const qz = q.z !== undefined ? q.z : (q.qz !== undefined ? q.qz : 0.0);
          const qw = q.w !== undefined ? q.w : (q.qw !== undefined ? q.qw : 1.0);
          const euler = window.telemetry.quaternionToEuler(qw, qx, qy, qz);
          update.orientation = { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw, qw, qx, qy, qz };
          if (p) update.camera_pose = window.telemetry.computeCameraPose(p.x, p.y, p.z, qw, qx, qy, qz);
        }
      }

      // 13. /drone/vio/camera_pose (geometry_msgs/msg/PoseStamped)
      if (msg.topic === '/drone/vio/camera_pose') {
        const p = msg.data?.pose?.position || msg.data?.position || {};
        const q = msg.data?.pose?.orientation || msg.data?.orientation || {};
        const qx = q.x !== undefined ? q.x : (q.qx !== undefined ? q.qx : 0.0);
        const qy = q.y !== undefined ? q.y : (q.qy !== undefined ? q.qy : 0.0);
        const qz = q.z !== undefined ? q.z : (q.qz !== undefined ? q.qz : 0.0);
        const qw = q.w !== undefined ? q.w : (q.qw !== undefined ? q.qw : 1.0);
        const euler = window.telemetry.quaternionToEuler(qw, qx, qy, qz);
        update.camera_pose = { x: p.x, y: p.y, z: p.z, roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw, qw, qx, qy, qz };
      }

      // 14. /imu/data or /imu/data_raw (sensor_msgs/msg/Imu)
      if (msg.topic === '/imu/data' || msg.topic === '/imu/data_raw') {
        const imu = msg.data || {};
        const a = imu.linear_acceleration || {};
        if (a.x !== undefined) update.acceleration = { ax: a.x, ay: a.y, az: a.z, norm: Math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z) };
      }

      // Direct full telemetry payload format from bridge_server
      if (msg.type === 'telemetry' && msg.data) {
        const d = msg.data;
        if (d.pose_wb && d.pose_wb.position) {
          const pos = d.pose_wb.position;
          const ori = d.pose_wb.orientation || {};
          update.position = { x: pos.x, y: pos.y, z: pos.z };
          const qx = ori.qx !== undefined ? ori.qx : 0.0;
          const qy = ori.qy !== undefined ? ori.qy : 0.0;
          const qz = ori.qz !== undefined ? ori.qz : 0.0;
          const qw = ori.qw !== undefined ? ori.qw : 1.0;
          const euler = window.telemetry.quaternionToEuler(qw, qx, qy, qz);
          update.orientation = { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw, qw: qw, qx: qx, qy: qy, qz: qz };
          update.camera_pose = window.telemetry.computeCameraPose(pos.x, pos.y, pos.z, qw, qx, qy, qz);
        } else if (d.pose_wb === null) {
          update.position = { x: null, y: null, z: null };
          update.velocity = { vx: null, vy: null, vz: null, norm: null };
          update.acceleration = { ax: null, ay: null, az: null, norm: null };
          update.orientation = { roll: null, pitch: null, yaw: null, qw: null, qx: null, qy: null, qz: null };
        }
        if (d.twist && d.twist.linear) {
          const l = d.twist.linear;
          const w = d.twist.angular || {};
          update.velocity = { vx: l.vx, vy: l.vy, vz: l.vz, norm: Math.sqrt(l.vx*l.vx + l.vy*l.vy + l.vz*l.vz) };
          if (update.orientation) {
            update.orientation.roll_rate = w.p !== undefined ? w.p : 0.0;
            update.orientation.pitch_rate = w.q !== undefined ? w.q : 0.0;
            update.orientation.yaw_rate = w.r !== undefined ? w.r : 0.0;
          }
        }
        if (d.imu && d.imu.accel) {
          const a = d.imu.accel;
          update.acceleration = { ax: a.ax, ay: a.ay, az: a.az, norm: Math.sqrt(a.ax*a.ax + a.ay*a.ay + a.az*a.az) };
        }
        if (d.controller) {
          update.controller = Object.assign(update.controller || {}, d.controller);
        }
        if (d.vio) {
          update.vio = Object.assign(update.vio || {}, d.vio);
        }
      }

      window.telemetry.updateFromNormalizedData(update);
    }
  }

  // Expose singleton on window
  window.wsBridge = new WebSocketBridge();
  // Expose validators for testing / external hooks
  window.wsBridge._isValidWsUrl = isValidWsUrl;
  window.wsBridge._persistWsUrl = persistWsUrl;

})(window);
