/**
 * ============================================================================
 * AeroVIO Ground Control Station - Telemetry Data Engine
 * Normalizes live ROS 2 / WebSocket frames and drives the Demo dataset player.
 * Explicitly separates Body Trajectory (T_WB) from Camera Optical (T_WC).
 * ============================================================================
 */

(function(window) {
  'use strict';

  // Extrinsic transform from Body (IMU) to Camera optical frame: T_BC
  // Camera is mounted +10cm forward, -4cm downward on the airframe
  const EXTRINSIC_T_BC = {
    translation: { x: 0.10, y: 0.00, z: -0.04 },
    // Rotation matrix R_BC (Camera Z is optical axis forward, X right, Y down)
    rotation: [
      [ 0,  0,  1],
      [-1,  0,  0],
      [ 0, -1,  0]
    ]
  };

  class TelemetryEngine {
    constructor() {
      // Current mode: 'LIVE' or 'DEMO'
      this.mode = 'DEMO'; // Default to DEMO mode for immediate inspection
      
      // Normalized telemetry state
      this.state = {
        timestamp: Date.now(),
        lastUpdateTimestamp: 0,
        isStale: false,
        staleDuration: 0,
        
        connection: {
          ros: false,
          websocket: false,
          latency_ms: 0,
          state: 'DISCONNECTED', // 'CONNECTED' | 'CONNECTING' | 'DISCONNECTED' | 'DEGRADED'
          mode: 'DEMO',
          uptime_s: 0,
          packet_rate_hz: 0
        },

        // Body Trajectory State in World Frame: T_WB
        position: {
          x: null,
          y: null,
          z: null,
          total_dist: 0
        },

        // Body Linear Velocity (m/s)
        velocity: {
          vx: null,
          vy: null,
          vz: null,
          norm: null
        },

        // Body Acceleration (IMU / m/s²)
        acceleration: {
          ax: null,
          ay: null,
          az: null,
          norm: null
        },

        // Orientation / Attitude (Euler in deg & Quaternion)
        orientation: {
          roll: null,
          pitch: null,
          yaw: null,
          roll_rate: null,
          pitch_rate: null,
          yaw_rate: null,
          qw: null,
          qx: null,
          qy: null,
          qz: null
        },

        // Camera Optical Pose in World Frame: T_WC = T_WB * T_BC
        camera_pose: {
          x: null,
          y: null,
          z: null,
          roll: null,
          pitch: null,
          yaw: null
        },

        // Visual-Inertial Odometry Subsystem State
        vio: {
          status: 'OFFLINE', // 'OFFLINE' | 'INITIALIZING' | 'ACTIVE' | 'TRACKING_LOST'
          active_features: null,
          tracked_features: null,
          tracking_quality: null, // % derived dynamically from tracked/active
          clones: 10,
          update_rate_hz: null,
          imu_rate_hz: null,
          covariance_trace: null,
          scale_drift: null,
          accel_bias: [0, 0, 0],
          gyro_bias: [0, 0, 0]
        },

        // Closed-Loop Flight Controller Subsystem State
        controller: {
          status: 'OFFLINE', // 'OFFLINE' | 'STANDBY' | 'READY' | 'ACTIVE'
          mode: 'AUTO_WAYPOINT',
          stability: 'STABLE (LQR DAMPED)',
          desired_pos: { x: 20.0, y: 5.0, z: 20.0, yaw: 2.0 },
          position_error: null,
          desired_att: { roll: 0.0, pitch: 0.0, yaw: 0.0 },
          attitude_error: null,
          m1_rpm: null,
          m2_rpm: null,
          m3_rpm: null,
          m4_rpm: null,
          currents: [null, null, null, null],
          gains: {
            kp_pos: [0.5, 0.5, 0.5],
            kv_pos: [0.2, 0.2, 0.2],
            kp_att: [4.85, 4.80],
            ki_rates: 0.12,
            kd_rates: 0.035
          }
        },

        // Dynamic System Health (Camera, IMU, VIO, Controller, WebSocket)
        health: {
          camera: 'DISCONNECTED',   // 'ONLINE' | 'ACTIVE' | 'READY' | 'DEGRADED' | 'DISCONNECTED' | 'UNKNOWN'
          imu: 'DISCONNECTED',
          vio: 'DISCONNECTED',
          controller: 'DISCONNECTED',
          websocket: 'DISCONNECTED'
        },

        // Trajectory History Buffers
        trajectories: {
          body: [],     // Array of {x, y, z, t} for T_WB
          camera: []    // Array of {x, y, z, t} for T_WC
        },

        // Active 3D Pointcloud Landmarks
        landmarks: []
      };

      // Watchdog settings
      this.staleThresholdMs = 2000;
      this.packetCounter = 0;
      this.lastPacketCheck = Date.now();
      
      // Listeners
      this.listeners = {
        telemetry: [],
        stateChange: [],
        stale: [],
        health: [],
        modeChange: []
      };

      // Demo mode playback state
      this.demoDatasetName = 'synthetic_orbit';
      this.demoIndex = 0;
      this.demoInterval = null;
      this.totalTraveledDist = 0;
      this.lastPos = null;

      // Start internal watchdog and demo runner if initial mode is DEMO
      this._initWatchdog();
      if (this.mode === 'DEMO') {
        this.startDemoMode();
      }
    }

    /**
     * Subscribe to telemetry updates
     */
    onTelemetry(callback) {
      this.listeners.telemetry.push(callback);
    }

    /**
     * Subscribe to state or mode changes
     */
    onStateChange(callback) {
      this.listeners.stateChange.push(callback);
    }

    /**
     * Subscribe to stale alerts
     */
    onStale(callback) {
      this.listeners.stale.push(callback);
    }

    /**
     * Subscribe to system health updates
     */
    onHealth(callback) {
      this.listeners.health.push(callback);
    }

    /**
     * Switch telemetry mode: 'LIVE' vs 'DEMO'
     */
    setMode(mode) {
      if (mode === this.mode) return;
      this.mode = mode;
      this.state.connection.mode = mode;

      if (mode === 'DEMO') {
        this.startDemoMode();
      } else {
        this.stopDemoMode();
        // In LIVE mode, if websocket is not connected, set state to disconnected
        if (!this.state.connection.websocket) {
          this.setDisconnectedState();
        }
      }

      this._emit('modeChange', this.mode);
      this._emit('stateChange', this.state);
    }

    /**
     * Change active DEMO dataset (e.g., 'synthetic_orbit', 'synthetic_hover', 'synthetic_straight', etc.)
     */
    setDemoDataset(datasetKey) {
      this.demoDatasetName = datasetKey;
      this.demoIndex = 0;
      this.state.trajectories.body = [];
      this.state.trajectories.camera = [];
      this.totalTraveledDist = 0;
      this.lastPos = null;

      // Load landmarks from dataset if available
      if (window.AERO_DATASETS && window.AERO_DATASETS.pointcloud) {
        this.state.landmarks = window.AERO_DATASETS.pointcloud;
      }
    }

    /**
     * Start playback of real repository datasets
     */
    startDemoMode() {
      this.stopDemoMode();
      this.demoIndex = 0;
      this.state.connection.mode = 'DEMO';
      this.state.connection.state = 'DEMO MODE';
      
      // Update health for demo
      this.state.health = {
        camera: 'ONLINE',
        imu: 'ONLINE',
        vio: 'ACTIVE',
        controller: 'READY',
        websocket: 'DEMO'
      };

      // Load 3D landmarks
      if (window.AERO_DATASETS && window.AERO_DATASETS.pointcloud) {
        this.state.landmarks = window.AERO_DATASETS.pointcloud;
      }

      const stepIntervalMs = 33; // ~30 Hz playback rate
      this.demoInterval = setInterval(() => {
        this._stepDemoPlayback();
      }, stepIntervalMs);
    }

    /**
     * Stop demo playback
     */
    stopDemoMode() {
      if (this.demoInterval) {
        clearInterval(this.demoInterval);
        this.demoInterval = null;
      }
    }

    /**
     * Step through the real CSV trajectory frame by frame in DEMO mode
     */
    _stepDemoPlayback() {
      const datasets = window.AERO_DATASETS || {};
      const dataset = datasets[this.demoDatasetName] || datasets['synthetic_orbit'];
      if (!dataset || !dataset.length) return;

      const row = dataset[this.demoIndex];
      this.demoIndex = (this.demoIndex + 1) % dataset.length;

      const now = Date.now();
      const x = row.x;
      const y = row.y;
      const z = row.z;
      const qw = row.qw;
      const qx = row.qx;
      const qy = row.qy;
      const qz = row.qz;

      // Calculate Euler angles from quaternion
      const euler = this.quaternionToEuler(qw, qx, qy, qz);

      // Estimate velocity from position differences
      let vx = 0, vy = 0, vz = 0;
      if (this.lastPos) {
        const dt = 0.033;
        vx = (x - this.lastPos.x) / dt;
        vy = (y - this.lastPos.y) / dt;
        vz = (z - this.lastPos.z) / dt;
        const dDist = Math.sqrt((x - this.lastPos.x)**2 + (y - this.lastPos.y)**2 + (z - this.lastPos.z)**2);
        this.totalTraveledDist += dDist;
      }
      this.lastPos = { x, y, z };

      // Compute Camera Pose: T_WC = T_WB * T_BC
      const camPose = this.computeCameraPose(x, y, z, euler.roll, euler.pitch, euler.yaw);

      // Sim features based on trajectory motion
      const activeFeat = 120 + Math.floor(Math.sin(this.demoIndex * 0.05) * 15);
      const trackedFeat = Math.min(activeFeat, Math.floor(activeFeat * 0.91 + Math.cos(this.demoIndex * 0.08) * 8));
      const trackingQuality = Math.round((trackedFeat / activeFeat) * 100);

      // Controller errors to target setpoint (setpoint: [20, 5, 20])
      const target = this.state.controller.desired_pos;
      const posErr = Math.sqrt((target.x - x)**2 + (target.y - y)**2 + (target.z - z)**2);
      const attErr = Math.sqrt((0 - euler.roll)**2 + (0 - euler.pitch)**2 + (target.yaw - euler.yaw * Math.PI/180)**2);

      // Normalized update payload
      this.updateFromNormalizedData({
        timestamp: now,
        connection: {
          ros: true,
          websocket: false,
          latency_ms: 2.4,
          state: 'DEMO MODE',
          mode: 'DEMO',
          packet_rate_hz: 30
        },
        position: {
          x: x,
          y: y,
          z: z,
          total_dist: this.totalTraveledDist
        },
        velocity: {
          vx: vx,
          vy: vy,
          vz: vz,
          norm: Math.sqrt(vx*vx + vy*vy + vz*vz)
        },
        acceleration: {
          ax: (Math.sin(this.demoIndex * 0.1) * 0.4),
          ay: (Math.cos(this.demoIndex * 0.1) * 0.3),
          az: 9.81 + (Math.sin(this.demoIndex * 0.15) * 0.2),
          norm: 9.81
        },
        orientation: {
          roll: euler.roll,
          pitch: euler.pitch,
          yaw: euler.yaw,
          roll_rate: (euler.roll * 0.05),
          pitch_rate: (euler.pitch * 0.05),
          yaw_rate: 1.41,
          qw: qw,
          qx: qx,
          qy: qy,
          qz: qz
        },
        camera_pose: camPose,
        vio: {
          status: 'ACTIVE (DEMO)',
          active_features: activeFeat,
          tracked_features: trackedFeat,
          tracking_quality: trackingQuality,
          clones: 10,
          update_rate_hz: 30.0,
          imu_rate_hz: 200.0,
          covariance_trace: 1.42e-4,
          scale_drift: 0.018,
          accel_bias: [0.012, -0.008, 0.024],
          gyro_bias: [-0.0004, 0.0011, -0.0002]
        },
        controller: {
          status: 'READY (DEMO)',
          mode: 'AUTO WAYPOINT [WP-04]',
          stability: 'STABLE (LQR DAMPED)',
          desired_pos: target,
          position_error: posErr,
          desired_att: { roll: 0.0, pitch: 0.0, yaw: target.yaw * 180 / Math.PI },
          attitude_error: attErr * (180 / Math.PI),
          m1_rpm: 7840 + Math.floor(Math.sin(this.demoIndex * 0.2) * 80),
          m2_rpm: 7820 + Math.floor(Math.cos(this.demoIndex * 0.2) * 70),
          m3_rpm: 7860 + Math.floor(Math.sin(this.demoIndex * 0.2) * 90),
          m4_rpm: 7830 + Math.floor(Math.cos(this.demoIndex * 0.2) * 60),
          currents: [11.4, 11.2, 11.5, 11.3]
        },
        health: {
          camera: 'ONLINE',
          imu: 'ONLINE',
          vio: 'ACTIVE',
          controller: 'READY',
          websocket: 'DEMO'
        }
      });
    }

    /**
     * Compute Camera Optical Pose T_WC from Body Pose T_WB using extrinsic transform T_BC
     * T_WC = T_WB * T_BC
     */
    computeCameraPose(bx, by, bz, bRoll, bPitch, bYaw) {
      // Small angle approximation for translation displacement
      const rRad = (bRoll || 0) * Math.PI / 180;
      const pRad = (bPitch || 0) * Math.PI / 180;
      const yRad = (bYaw || 0) * Math.PI / 180;

      // Body to camera offset vector
      const ox = EXTRINSIC_T_BC.translation.x;
      const oy = EXTRINSIC_T_BC.translation.y;
      const oz = EXTRINSIC_T_BC.translation.z;

      // Rotate offset by body attitude
      const cx = bx + (Math.cos(yRad)*Math.cos(pRad)*ox + (Math.cos(yRad)*Math.sin(pRad)*Math.sin(rRad) - Math.sin(yRad)*Math.cos(rRad))*oy);
      const cy = by + (Math.sin(yRad)*Math.cos(pRad)*ox + (Math.sin(yRad)*Math.sin(pRad)*Math.sin(rRad) + Math.cos(yRad)*Math.cos(rRad))*oy);
      const cz = bz + (-Math.sin(pRad)*ox + Math.cos(pRad)*Math.sin(rRad)*oy + oz);

      return {
        x: cx,
        y: cy,
        z: cz,
        roll: bRoll,
        pitch: bPitch - 15, // 15-degree down-tilt
        yaw: bYaw
      };
    }

    /**
     * Ingest normalized telemetry data from either live WebSocket bridge or Demo player
     */
    updateFromNormalizedData(data) {
      const now = Date.now();
      this.state.timestamp = data.timestamp || now;
      this.state.lastUpdateTimestamp = now;
      this.state.isStale = false;
      this.state.staleDuration = 0;

      // Update sub-structures cleanly
      if (data.connection) Object.assign(this.state.connection, data.connection);
      if (data.position) Object.assign(this.state.position, data.position);
      if (data.velocity) Object.assign(this.state.velocity, data.velocity);
      if (data.acceleration) Object.assign(this.state.acceleration, data.acceleration);
      if (data.orientation) Object.assign(this.state.orientation, data.orientation);
      if (data.camera_pose) Object.assign(this.state.camera_pose, data.camera_pose);
      if (data.vio) Object.assign(this.state.vio, data.vio);
      if (data.controller) Object.assign(this.state.controller, data.controller);
      if (data.health) Object.assign(this.state.health, data.health);

      // Append to trajectories buffer (keep max 1000 points)
      if (this.state.position.x !== null) {
        this.state.trajectories.body.push({
          x: this.state.position.x,
          y: this.state.position.y,
          z: this.state.position.z,
          t: this.state.timestamp
        });
        if (this.state.trajectories.body.length > 1000) {
          this.state.trajectories.body.shift();
        }
      }

      if (this.state.camera_pose.x !== null) {
        this.state.trajectories.camera.push({
          x: this.state.camera_pose.x,
          y: this.state.camera_pose.y,
          z: this.state.camera_pose.z,
          t: this.state.timestamp
        });
        if (this.state.trajectories.camera.length > 1000) {
          this.state.trajectories.camera.shift();
        }
      }

      // Track packet rate
      this.packetCounter++;

      // Emit telemetry update to all subscribers
      this._emit('telemetry', this.state);
    }

    /**
     * Sets telemetry to a clean disconnected state when LIVE connection drops
     */
    setDisconnectedState() {
      this.state.connection.ros = false;
      this.state.connection.websocket = false;
      this.state.connection.state = 'DISCONNECTED';
      this.state.connection.latency_ms = null;
      
      this.state.health = {
        camera: 'DISCONNECTED',
        imu: 'DISCONNECTED',
        vio: 'DISCONNECTED',
        controller: 'DISCONNECTED',
        websocket: 'DISCONNECTED'
      };

      this.state.vio.status = 'NOT CONNECTED';
      this.state.controller.status = 'NOT CONNECTED';

      this._emit('stateChange', this.state);
      this._emit('health', this.state.health);
    }

    /**
     * Stale Watchdog timer running at 5 Hz
     */
    _initWatchdog() {
      setInterval(() => {
        const now = Date.now();
        
        // Calculate packet rate
        const dtRate = (now - this.lastPacketCheck) / 1000;
        if (dtRate >= 1.0) {
          this.state.connection.packet_rate_hz = Math.round(this.packetCounter / dtRate);
          this.packetCounter = 0;
          this.lastPacketCheck = now;
        }

        // Check staleness
        if (this.state.lastUpdateTimestamp > 0) {
          const elapsedMs = now - this.state.lastUpdateTimestamp;
          if (elapsedMs > this.staleThresholdMs) {
            this.state.isStale = true;
            this.state.staleDuration = (elapsedMs / 1000).toFixed(1);
            this._emit('stale', { isStale: true, duration: this.state.staleDuration });
          } else {
            if (this.state.isStale) {
              this.state.isStale = false;
              this._emit('stale', { isStale: false, duration: 0 });
            }
          }
        }
      }, 200);
    }

    /**
     * Helper: Convert quaternion (w, x, y, z) to Euler angles (Roll, Pitch, Yaw in degrees)
     */
    quaternionToEuler(w, x, y, z) {
      if (w === undefined || x === undefined || y === undefined || z === undefined) {
        return { roll: 0, pitch: 0, yaw: 0 };
      }
      
      // Roll (x-axis rotation)
      const sinr_cosp = 2 * (w * x + y * z);
      const cosr_cosp = 1 - 2 * (x * x + y * y);
      const roll = Math.atan2(sinr_cosp, cosr_cosp);

      // Pitch (y-axis rotation)
      const sinp = 2 * (w * y - z * x);
      let pitch;
      if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * (Math.PI / 2);
      } else {
        pitch = Math.asin(sinp);
      }

      // Yaw (z-axis rotation)
      const siny_cosp = 2 * (w * z + x * y);
      const cosy_cosp = 1 - 2 * (y * y + z * z);
      const yaw = Math.atan2(siny_cosp, cosy_cosp);

      return {
        roll: +(roll * 180 / Math.PI).toFixed(2),
        pitch: +(pitch * 180 / Math.PI).toFixed(2),
        yaw: +(yaw * 180 / Math.PI).toFixed(2)
      };
    }

    _emit(event, data) {
      if (this.listeners[event]) {
        this.listeners[event].forEach(cb => {
          try { cb(data); } catch(err) { console.error(`[TelemetryEngine Error] in ${event} listener:`, err); }
        });
      }
    }
  }

  // Expose singleton on window
  window.telemetry = new TelemetryEngine();

})(window);
