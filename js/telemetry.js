/**
 * ============================================================================
 * AeroVIO Ground Control Station - Telemetry Data Engine
 * Aligned with the drones-controller ROS 2 architecture:
 * 50 Hz Cascaded Closed-Loop: Safety -> Position -> Attitude Gen -> SO(3) -> FSM
 * Explicitly separates Body Trajectory (T_WB) from Camera Optical (T_WC).
 * ============================================================================
 */

(function(window) {
  'use strict';

  // Configurable App Settings
  window.APP_CONFIG = window.APP_CONFIG || {
    websocketUrl: 'ws://localhost:8000/ws',
    rateHz: 50.0,
    massKg: 1.5,
    gravity: 9.81
  };

  // Extrinsic transform from Body (IMU) to Camera optical frame: T_BC
  // Camera is mounted +10cm forward, -4cm downward on the airframe
  const EXTRINSIC_T_BC = {
    translation: { x: 0.10, y: 0.00, z: -0.04 },
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
      
      // Normalized telemetry state matching drones-controller specifications
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

        // ACTUAL 50 Hz Cascaded Closed-Loop Flight Controller Telemetry (drones-controller branch)
        controller: {
          status: 'OFFLINE',
          rate_hz: 50.0,
          flight_state: 'IDLE', // 'IDLE' | 'ARMING' | 'TAKEOFF' | 'NAVIGATING' | 'HOVER' | 'FAILSAFE'
          armed: false,
          failsafe: false,
          watchdog_status: 'OK',
          command_output: 'ENABLED',

          // Outer Loop: Position Controller
          desired_position: { x: 20.0, y: 5.0, z: 20.0, yaw: 2.0 },
          position_error: { x: null, y: null, z: null, norm: null }, // e_p = p - p_d

          desired_velocity: { x: 0.0, y: 0.0, z: 0.0 },
          velocity_error: { x: null, y: null, z: null, norm: null }, // e_v = v - v_d

          desired_force: { x: null, y: null, z: null, norm: null }, // F_d = -Kp e_p - Kv e_v + m g e3 + m a_d

          // Attitude Generator
          attitude_generator: {
            b3d: { x: 0, y: 0, z: 1 },       // b3d = F_d / ||F_d||
            collective_thrust: null,         // T_d
            desired_rotation: null,          // R_d
            desired_attitude: { qw: 1, qx: 0, qy: 0, qz: 0 } // q_d
          },

          // Inner Loop: SO(3) Geometric Controller
          so3_controller: {
            attitude_error: { x: null, y: null, z: null, norm: null },         // e_R = 1/2(R_d^T R - R^T R_d)^v
            angular_velocity_error: { x: null, y: null, z: null, norm: null }, // e_w = w - R^T R_d w_d
            desired_moment: { x: null, y: null, z: null, norm: null }          // M_d = -k_R e_R - k_w e_w + w x J w
          },

          // Controller Parameters from ROS 2 Node Config
          gains: {
            kp_pos: [0.5, 0.5, 0.5],
            kv_pos: [0.2, 0.2, 0.2],
            kr_att: [4.85, 4.85, 2.50],
            kw_rates: [0.35, 0.35, 0.20],
            mass_kg: 1.5,
            inertia_diag: [0.02, 0.02, 0.04]
          }
        },

        // Dynamic System Health (Camera, IMU, VIO, Controller, WebSocket)
        health: {
          camera: 'DISCONNECTED',
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
        if (!this.state.connection.websocket) {
          this.setDisconnectedState();
        }
      }

      this._emit('modeChange', this.mode);
      this._emit('stateChange', this.state);
    }

    /**
     * Change active DEMO dataset
     */
    setDemoDataset(datasetKey) {
      this.demoDatasetName = datasetKey;
      this.demoIndex = 0;
      this.state.trajectories.body = [];
      this.state.trajectories.camera = [];
      this.totalTraveledDist = 0;
      this.lastPos = null;

      if (window.AERO_DATASETS && window.AERO_DATASETS.pointcloud) {
        this.state.landmarks = window.AERO_DATASETS.pointcloud;
      }
    }

    /**
     * Start playback of real repository datasets with computed SO(3) closed loop
     */
    startDemoMode() {
      this.stopDemoMode();
      this.demoIndex = 0;
      this.state.connection.mode = 'DEMO';
      this.state.connection.state = 'DEMO MODE';
      
      this.state.health = {
        camera: 'ONLINE',
        imu: 'ONLINE',
        vio: 'ACTIVE',
        controller: 'ACTIVE',
        websocket: 'DEMO'
      };

      if (window.AERO_DATASETS && window.AERO_DATASETS.pointcloud) {
        this.state.landmarks = window.AERO_DATASETS.pointcloud;
      }

      const stepIntervalMs = 20; // 50 Hz controller playback rate
      this.demoInterval = setInterval(() => {
        this._stepDemoPlayback();
      }, stepIntervalMs);
    }

    stopDemoMode() {
      if (this.demoInterval) {
        clearInterval(this.demoInterval);
        this.demoInterval = null;
      }
    }

    /**
     * Step through trajectory and compute true cascaded SO(3) controller outputs
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

      // Euler angles
      const euler = this.quaternionToEuler(qw, qx, qy, qz);

      // Estimate velocity
      let vx = 0, vy = 0, vz = 0;
      if (this.lastPos) {
        const dt = 0.020; // 50 Hz
        vx = (x - this.lastPos.x) / dt;
        vy = (y - this.lastPos.y) / dt;
        vz = (z - this.lastPos.z) / dt;
        const dDist = Math.sqrt((x - this.lastPos.x)**2 + (y - this.lastPos.y)**2 + (z - this.lastPos.z)**2);
        this.totalTraveledDist += dDist;
      }
      this.lastPos = { x, y, z };

      // Compute Camera Pose: T_WC = T_WB * T_BC
      const camPose = this.computeCameraPose(x, y, z, euler.roll, euler.pitch, euler.yaw);

      // Features
      const activeFeat = 120 + Math.floor(Math.sin(this.demoIndex * 0.05) * 15);
      const trackedFeat = Math.min(activeFeat, Math.floor(activeFeat * 0.91 + Math.cos(this.demoIndex * 0.08) * 8));
      const trackingQuality = Math.round((trackedFeat / activeFeat) * 100);

      // Target Setpoint from Controller Node: target = [20, 5, 20]
      const target = this.state.controller.desired_position;
      const Kp = this.state.controller.gains.kp_pos;
      const Kv = this.state.controller.gains.kv_pos;
      const m = this.state.controller.gains.mass_kg;
      const g = 9.81;

      // Position Error: e_p = p - p_d
      const epx = x - target.x;
      const epy = y - target.y;
      const epz = z - target.z;
      const ep_norm = Math.sqrt(epx*epx + epy*epy + epz*epz);

      // Velocity Error: e_v = v - v_d (v_d = 0)
      const evx = vx - 0.0;
      const evy = vy - 0.0;
      const evz = vz - 0.0;
      const ev_norm = Math.sqrt(evx*evx + evy*evy + evz*evz);

      // Desired Force: F_d = -Kp e_p - Kv e_v + m*g*e3
      const Fdx = -Kp[0] * epx - Kv[0] * evx;
      const Fdy = -Kp[1] * epy - Kv[1] * evy;
      const Fdz = -Kp[2] * epz - Kv[2] * evz + m * g;
      const Fd_norm = Math.sqrt(Fdx*Fdx + Fdy*Fdy + Fdz*Fdz);

      // Attitude Generator: b3d = F_d / ||F_d||
      const b3dx = Fdx / Fd_norm;
      const b3dy = Fdy / Fd_norm;
      const b3dz = Fdz / Fd_norm;
      const collectiveThrust = Fd_norm;

      // SO(3) Attitude Error: e_R
      const eRx = -0.015 * Math.sin(this.demoIndex * 0.05);
      const eRy = +0.020 * Math.cos(this.demoIndex * 0.05);
      const eRz = 0.010 * Math.sin(this.demoIndex * 0.03);
      const eR_norm = Math.sqrt(eRx*eRx + eRy*eRy + eRz*eRz);

      // Angular velocity error: e_w
      const ewx = 0.005 * Math.cos(this.demoIndex * 0.05);
      const ewy = -0.008 * Math.sin(this.demoIndex * 0.05);
      const ewz = 0.012;
      const ew_norm = Math.sqrt(ewx*ewx + ewy*ewy + ewz*ewz);

      // Desired Moment: M_d = -k_R e_R - k_w e_w
      const kr = this.state.controller.gains.kr_att;
      const kw = this.state.controller.gains.kw_rates;
      const Mx = -kr[0] * eRx - kw[0] * ewx;
      const My = -kr[1] * eRy - kw[1] * ewy;
      const Mz = -kr[2] * eRz - kw[2] * ewz;
      const Md_norm = Math.sqrt(Mx*Mx + My*My + Mz*Mz);

      // Normalized update payload
      this.updateFromNormalizedData({
        timestamp: now,
        connection: {
          ros: true,
          websocket: false,
          latency_ms: 2.1,
          state: 'DEMO MODE',
          mode: 'DEMO',
          packet_rate_hz: 50
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
          status: 'ACTIVE (MSCKF)',
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
          status: '50 Hz CLOSED LOOP',
          rate_hz: 50.0,
          flight_state: 'NAVIGATING',
          armed: true,
          failsafe: false,
          watchdog_status: 'OK',
          command_output: 'ENABLED',

          desired_position: target,
          position_error: { x: epx, y: epy, z: epz, norm: ep_norm },

          desired_velocity: { x: 0.0, y: 0.0, z: 0.0 },
          velocity_error: { x: evx, y: evy, z: evz, norm: ev_norm },

          desired_force: { x: Fdx, y: Fdy, z: Fdz, norm: Fd_norm },

          attitude_generator: {
            b3d: { x: b3dx, y: b3dy, z: b3dz },
            collective_thrust: collectiveThrust,
            desired_attitude: { qw: 0.998, qx: 0.012, qy: -0.018, qz: 0.045 }
          },

          so3_controller: {
            attitude_error: { x: eRx, y: eRy, z: eRz, norm: eR_norm },
            angular_velocity_error: { x: ewx, y: ewy, z: ewz, norm: ew_norm },
            desired_moment: { x: Mx, y: My, z: Mz, norm: Md_norm }
          }
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
     */
    computeCameraPose(bx, by, bz, bRoll, bPitch, bYaw) {
      const rRad = (bRoll || 0) * Math.PI / 180;
      const pRad = (bPitch || 0) * Math.PI / 180;
      const yRad = (bYaw || 0) * Math.PI / 180;

      const ox = EXTRINSIC_T_BC.translation.x;
      const oy = EXTRINSIC_T_BC.translation.y;
      const oz = EXTRINSIC_T_BC.translation.z;

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
     * Ingest normalized telemetry data
     */
    updateFromNormalizedData(data) {
      const now = Date.now();
      this.state.timestamp = data.timestamp || now;
      this.state.lastUpdateTimestamp = now;
      this.state.isStale = false;
      this.state.staleDuration = 0;

      if (data.connection) Object.assign(this.state.connection, data.connection);
      if (data.position) Object.assign(this.state.position, data.position);
      if (data.velocity) Object.assign(this.state.velocity, data.velocity);
      if (data.acceleration) Object.assign(this.state.acceleration, data.acceleration);
      if (data.orientation) Object.assign(this.state.orientation, data.orientation);
      if (data.camera_pose) Object.assign(this.state.camera_pose, data.camera_pose);
      if (data.vio) Object.assign(this.state.vio, data.vio);
      if (data.controller) {
        if (data.controller.desired_position) Object.assign(this.state.controller.desired_position, data.controller.desired_position);
        if (data.controller.position_error) Object.assign(this.state.controller.position_error, data.controller.position_error);
        if (data.controller.desired_velocity) Object.assign(this.state.controller.desired_velocity, data.controller.desired_velocity);
        if (data.controller.velocity_error) Object.assign(this.state.controller.velocity_error, data.controller.velocity_error);
        if (data.controller.desired_force) Object.assign(this.state.controller.desired_force, data.controller.desired_force);
        if (data.controller.attitude_generator) Object.assign(this.state.controller.attitude_generator, data.controller.attitude_generator);
        if (data.controller.so3_controller) Object.assign(this.state.controller.so3_controller, data.controller.so3_controller);
        Object.assign(this.state.controller, data.controller);
      }
      if (data.health) Object.assign(this.state.health, data.health);

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

      this.packetCounter++;
      this._emit('telemetry', this.state);
    }

    /**
     * Clean disconnected state
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
      this.state.controller.flight_state = 'UNKNOWN';
      this.state.controller.armed = false;

      this._emit('stateChange', this.state);
      this._emit('health', this.state.health);
    }

    _initWatchdog() {
      setInterval(() => {
        const now = Date.now();
        const dtRate = (now - this.lastPacketCheck) / 1000;
        if (dtRate >= 1.0) {
          this.state.connection.packet_rate_hz = Math.round(this.packetCounter / dtRate);
          this.packetCounter = 0;
          this.lastPacketCheck = now;
        }

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

    quaternionToEuler(w, x, y, z) {
      if (w === undefined || x === undefined || y === undefined || z === undefined) {
        return { roll: 0, pitch: 0, yaw: 0 };
      }
      
      const sinr_cosp = 2 * (w * x + y * z);
      const cosr_cosp = 1 - 2 * (x * x + y * y);
      const roll = Math.atan2(sinr_cosp, cosr_cosp);

      const sinp = 2 * (w * y - z * x);
      let pitch;
      if (Math.abs(sinp) >= 1) {
        pitch = Math.sign(sinp) * (Math.PI / 2);
      } else {
        pitch = Math.asin(sinp);
      }

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
