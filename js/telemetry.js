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

  // APP_CONFIG defined in js/config.js — consumed here

  function _syncHeaderBadges(stateOrMode){
    try {
      var ep = document.getElementById('top-endpoint-badge');
      if (ep && window.APP_CONFIG) ep.textContent = window.APP_CONFIG.websocketUrl || 'ws://localhost:8000/ws';
      var inp = document.getElementById('tune-ws-url');
      if (inp && window.APP_CONFIG && inp.value !== window.APP_CONFIG.websocketUrl) inp.value = window.APP_CONFIG.websocketUrl;
    } catch(e) {}
  }

  // Extrinsic transform from Body (IMU) to Camera optical frame: T_BC
  // Defined in reference repository config/synthetic.yaml & config/ros2.yaml:
  // p_BC: [0.10, 0.00, 0.00] m, R_BC maps OpenCV camera optical vectors to body ENU vectors
  const EXTRINSIC_T_BC = {
    translation: { x: 0.10, y: 0.00, z: 0.00 },
    rotation: [
      [ 0.0,  0.0,  1.0],
      [-1.0,  0.0,  0.0],
      [ 0.0, -1.0,  0.0]
    ]
  };

  function quatToMatrix(qx, qy, qz, qw) {
    var norm = Math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (!norm || norm < 1e-9) return [[1,0,0],[0,1,0],[0,0,1]];
    var x = qx/norm, y = qy/norm, z = qz/norm, w = qw/norm;
    return [
      [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
      [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
      [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ];
  }

  function matrixToQuat(R) {
    var tr = R[0][0] + R[1][1] + R[2][2];
    var qx, qy, qz, qw;
    if (tr > 0) {
      var s = 0.5 / Math.sqrt(tr + 1.0);
      qw = 0.25 / s;
      qx = (R[2][1] - R[1][2]) * s;
      qy = (R[0][2] - R[2][0]) * s;
      qz = (R[1][0] - R[0][1]) * s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
      var s = 2.0 * Math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
      qw = (R[2][1] - R[1][2]) / s;
      qx = 0.25 * s;
      qy = (R[0][1] + R[1][0]) / s;
      qz = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
      var s = 2.0 * Math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
      qw = (R[0][2] - R[2][0]) / s;
      qx = (R[0][1] + R[1][0]) / s;
      qy = 0.25 * s;
      qz = (R[1][2] + R[2][1]) / s;
    } else {
      var s = 2.0 * Math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
      qw = (R[1][0] - R[0][1]) / s;
      qx = (R[0][2] + R[2][0]) / s;
      qy = (R[1][2] + R[2][1]) / s;
      qz = 0.25 * s;
    }
    var qn = Math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) || 1;
    return { qx: qx/qn, qy: qy/qn, qz: qz/qn, qw: qw/qn };
  }

  function matrixMultiply(A, B) {
    var C = [[0,0,0],[0,0,0],[0,0,0]];
    for (var i=0; i<3; i++) {
      for (var j=0; j<3; j++) {
        C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
      }
    }
    return C;
  }

  function veeOp(S) {
    return [S[2][1], S[0][2], S[1][0]];
  }

  class TelemetryEngine {
    constructor() {
      // Current mode: 'LIVE' or 'DEMO'
      this.mode = 'LIVE'; // Default strictly to LIVE mode (truthful data, empty when awaiting telemetry)
      
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
          state: 'AWAITING LIVE TELEMETRY',
          mode: 'LIVE',
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

          // Controller Parameters from ROS 2 Node Config (config/controller.yaml & vehicle.yaml)
          gains: {
            kp_pos: [0.5, 0.5, 0.5],
            kv_pos: [0.2, 0.2, 0.2],
            kr_att: [4.0, 4.0, 2.0],
            kw_rates: [0.8, 0.8, 0.5],
            mass_kg: 1.5,
            inertia_diag: [0.02, 0.02, 0.04],
            max_velocity: 0.5,
            position_tolerance: 0.10,
            velocity_tolerance: 0.05
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

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function(){ _syncHeaderBadges(this.state.connection); }.bind(this));
      } else {
        setTimeout(function(){ _syncHeaderBadges(this.state.connection); }.bind(this), 0);
      }
      // Start internal watchdog and demo runner if initial mode is DEMO
      this._initWatchdog();
      if (this.mode === 'DEMO') {
        this.startDemoMode();
      } else {
        this.setDisconnectedState();
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
      _syncHeaderBadges(mode);

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
     * Aligned with position_controller.py, attitude_generator.py, and so3_controller.py
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

      // Euler angles of body
      const euler = this.quaternionToEuler(qw, qx, qy, qz);

      // Estimate linear velocity from displacement
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
      const v_norm = Math.sqrt(vx*vx + vy*vy + vz*vz);

      // Gyro rates from dataset or zero
      const wx = row.bgx || 0.0;
      const wy = row.bgy || 0.0;
      const wz = row.bgz || 0.0;

      // Compute Camera Pose: T_WC = T_WB * T_BC (matrix composition)
      const camPose = this.computeCameraPose(x, y, z, qw, qx, qy, qz);

      // Features
      const activeFeat = 120 + Math.floor(Math.sin(this.demoIndex * 0.05) * 15);
      const trackedFeat = Math.min(activeFeat, Math.floor(activeFeat * 0.91 + Math.cos(this.demoIndex * 0.08) * 8));
      const trackingQuality = Math.round((trackedFeat / activeFeat) * 100);

      // Controller Target & Gains (from reference repository)
      const target = this.state.controller.desired_position || { x: 20.0, y: 5.0, z: 20.0, yaw: 2.0 };
      const Kp = this.state.controller.gains.kp_pos || [0.5, 0.5, 0.5];
      const Kv = this.state.controller.gains.kv_pos || [0.2, 0.2, 0.2];
      const KR = this.state.controller.gains.kr_att || [4.0, 4.0, 2.0];
      const KOmega = this.state.controller.gains.kw_rates || [0.8, 0.8, 0.5];
      const m = this.state.controller.gains.mass_kg || 1.5;
      const g = 9.81;
      const J = this.state.controller.gains.inertia_diag || [0.02, 0.02, 0.04];
      const maxVel = this.state.controller.gains.max_velocity || 0.5;
      const posTol = this.state.controller.gains.position_tolerance || 0.10;
      const velTol = this.state.controller.gains.velocity_tolerance || 0.05;

      // 1. Position Controller (e_p = p_d - p, e_v = v_d - v)
      const epx = target.x - x;
      const epy = target.y - y;
      const epz = target.z - z;
      const ep_norm = Math.sqrt(epx*epx + epy*epy + epz*epz);

      const target_v = this.state.controller.desired_velocity || { x: 0, y: 0, z: 0 };
      const evx = target_v.x - vx;
      const evy = target_v.y - vy;
      const evz = target_v.z - vz;
      const ev_norm = Math.sqrt(evx*evx + evy*evy + evz*evz);

      const targetReached = (ep_norm < posTol && v_norm < velTol);

      // Commanded velocity (saturated)
      const rawVx = Kp[0] * epx + Kv[0] * evx;
      const rawVy = Kp[1] * epy + Kv[1] * evy;
      const rawVz = Kp[2] * epz + Kv[2] * evz;
      const rawSpeed = Math.sqrt(rawVx*rawVx + rawVy*rawVy + rawVz*rawVz);
      let v_cmd_x = 0, v_cmd_y = 0, v_cmd_z = 0;
      if (!targetReached) {
        if (rawSpeed > maxVel && rawSpeed > 1e-6) {
          const scale = maxVel / rawSpeed;
          v_cmd_x = rawVx * scale;
          v_cmd_y = rawVy * scale;
          v_cmd_z = rawVz * scale;
        } else {
          v_cmd_x = rawVx;
          v_cmd_y = rawVy;
          v_cmd_z = rawVz;
        }
      }

      // Desired force: F_d = m * (Kp*e_p + Kv*e_v + a_d + g*e3)
      const Fdx = m * (Kp[0] * epx + Kv[0] * evx);
      const Fdy = m * (Kp[1] * epy + Kv[1] * evy);
      const Fdz = m * (Kp[2] * epz + Kv[2] * evz + g);
      const Fd_norm = Math.sqrt(Fdx*Fdx + Fdy*Fdy + Fdz*Fdz);

      // 2. Attitude Generator
      let b3dx = 0, b3dy = 0, b3dz = 1;
      let collectiveThrust = 0;
      let R_d = [[1,0,0],[0,1,0],[0,0,1]];
      if (Fd_norm > 1e-6) {
        b3dx = Fdx / Fd_norm;
        b3dy = Fdy / Fd_norm;
        b3dz = Fdz / Fd_norm;
        collectiveThrust = Fd_norm;

        const yaw_d = target.yaw !== undefined ? target.yaw : 2.0;
        let hy_x = Math.cos(yaw_d);
        let hy_y = Math.sin(yaw_d);
        let hy_z = 0.0;

        // b2 = b3d x heading
        let b2x = b3dy * hy_z - b3dz * hy_y;
        let b2y = b3dz * hy_x - b3dx * hy_z;
        let b2z = b3dx * hy_y - b3dy * hy_x;
        let norm_b2 = Math.sqrt(b2x*b2x + b2y*b2y + b2z*b2z);
        if (norm_b2 < 1e-6) {
          const yaw_mod = yaw_d + 1e-3;
          hy_x = Math.cos(yaw_mod);
          hy_y = Math.sin(yaw_mod);
          b2x = b3dy * hy_z - b3dz * hy_y;
          b2y = b3dz * hy_x - b3dx * hy_z;
          b2z = b3dx * hy_y - b3dy * hy_x;
          norm_b2 = Math.sqrt(b2x*b2x + b2y*b2y + b2z*b2z);
        }
        b2x /= norm_b2; b2y /= norm_b2; b2z /= norm_b2;

        // b1 = b2 x b3d
        let b1x = b2y * b3dz - b2z * b3dy;
        let b1y = b2z * b3dx - b2x * b3dz;
        let b1z = b2x * b3dy - b2y * b3dx;
        const norm_b1 = Math.sqrt(b1x*b1x + b1y*b1y + b1z*b1z);
        b1x /= norm_b1; b1y /= norm_b1; b1z /= norm_b1;

        R_d = [
          [b1x, b2x, b3dx],
          [b1y, b2y, b3dy],
          [b1z, b2z, b3dz]
        ];
      }
      const q_d = matrixToQuat(R_d);

      // 3. SO(3) Attitude Controller
      const R_wb = quatToMatrix(qx, qy, qz, qw);
      const Rd_T = [
        [R_d[0][0], R_d[1][0], R_d[2][0]],
        [R_d[0][1], R_d[1][1], R_d[2][1]],
        [R_d[0][2], R_d[1][2], R_d[2][2]]
      ];
      const R_T = [
        [R_wb[0][0], R_wb[1][0], R_wb[2][0]],
        [R_wb[0][1], R_wb[1][1], R_wb[2][1]],
        [R_wb[0][2], R_wb[1][2], R_wb[2][2]]
      ];

      const RdT_R = matrixMultiply(Rd_T, R_wb);
      const RT_Rd = matrixMultiply(R_T, R_d);
      const E = [
        [RdT_R[0][0] - RT_Rd[0][0], RdT_R[0][1] - RT_Rd[0][1], RdT_R[0][2] - RT_Rd[0][2]],
        [RdT_R[1][0] - RT_Rd[1][0], RdT_R[1][1] - RT_Rd[1][1], RdT_R[1][2] - RT_Rd[1][2]],
        [RdT_R[2][0] - RT_Rd[2][0], RdT_R[2][1] - RT_Rd[2][1], RdT_R[2][2] - RT_Rd[2][2]]
      ];
      const eR_vec = veeOp(E);
      const eRx = 0.5 * eR_vec[0];
      const eRy = 0.5 * eR_vec[1];
      const eRz = 0.5 * eR_vec[2];
      const eR_norm = Math.sqrt(eRx*eRx + eRy*eRy + eRz*eRz);

      // Angular rate error: e_omega = omega - R^T * R_d * omega_d (with omega_d = 0)
      const ewx = wx;
      const ewy = wy;
      const ewz = wz;
      const ew_norm = Math.sqrt(ewx*ewx + ewy*ewy + ewz*ewz);

      // Coriolis = omega x (J * omega)
      const Jw_x = J[0] * wx;
      const Jw_y = J[1] * wy;
      const Jw_z = J[2] * wz;
      const coriolis_x = wy * Jw_z - wz * Jw_y;
      const coriolis_y = wz * Jw_x - wx * Jw_z;
      const coriolis_z = wx * Jw_y - wy * Jw_x;

      const Mx = -KR[0] * eRx - KOmega[0] * ewx + coriolis_x;
      const My = -KR[1] * eRy - KOmega[1] * ewy + coriolis_y;
      const Mz = -KR[2] * eRz - KOmega[2] * ewz + coriolis_z;
      const Md_norm = Math.sqrt(Mx*Mx + My*My + Mz*Mz);

      const flightState = targetReached ? 'TARGET_REACHED' : (v_norm > 0.05 ? 'FLYING' : 'POSITION_CONTROL');

      // Normalized update payload
      this.updateFromNormalizedData({
        timestamp: now,
        connection: {
          ros: false,
          websocket: false,
          latency_ms: 1.8,
          state: 'DEMO PLAYBACK',
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
          norm: v_norm
        },
        acceleration: {
          ax: (row.bax ? row.bax * 10 : 0.0),
          ay: (row.bay ? row.bay * 10 : 0.0),
          az: 9.81,
          norm: 9.81
        },
        orientation: {
          roll: euler.roll,
          pitch: euler.pitch,
          yaw: euler.yaw,
          roll_rate: wx,
          pitch_rate: wy,
          yaw_rate: wz,
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
          accel_bias: [row.bax || 0.0, row.bay || 0.0, row.baz || 0.0],
          gyro_bias: [row.bgx || 0.0, row.bgy || 0.0, row.bgz || 0.0]
        },
        controller: {
          status: '50 Hz CLOSED LOOP',
          rate_hz: 50.0,
          flight_state: flightState,
          armed: true,
          failsafe: false,
          watchdog_status: 'OK',
          command_output: 'ENABLED',

          desired_position: target,
          position_error: { x: epx, y: epy, z: epz, norm: ep_norm },

          desired_velocity: target_v,
          velocity_error: { x: evx, y: evy, z: evz, norm: ev_norm },
          command_velocity: { vx: v_cmd_x, vy: v_cmd_y, vz: v_cmd_z, speed: Math.sqrt(v_cmd_x*v_cmd_x + v_cmd_y*v_cmd_y + v_cmd_z*v_cmd_z) },

          desired_force: { x: Fdx, y: Fdy, z: Fdz, norm: Fd_norm },

          attitude_generator: {
            b3d: { x: b3dx, y: b3dy, z: b3dz },
            collective_thrust: collectiveThrust,
            desired_attitude: { qw: q_d.qw, qx: q_d.qx, qy: q_d.qy, qz: q_d.qz }
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
     * Compute Camera Optical Pose T_WC from Body Pose T_WB using extrinsic transform T_BC:
     * T_WC = T_WB * T_BC => p_WC = p_WB + R_WB * p_BC, R_WC = R_WB * R_BC
     */
    computeCameraPose(bx, by, bz, qw, qx, qy, qz) {
      const R_wb = quatToMatrix(qx || 0.0, qy || 0.0, qz || 0.0, qw !== undefined ? qw : 1.0);
      const ox = EXTRINSIC_T_BC.translation.x;
      const oy = EXTRINSIC_T_BC.translation.y;
      const oz = EXTRINSIC_T_BC.translation.z;

      // Translation: p_WC = p_WB + R_WB * p_BC
      const cx = bx + (R_wb[0][0]*ox + R_wb[0][1]*oy + R_wb[0][2]*oz);
      const cy = by + (R_wb[1][0]*ox + R_wb[1][1]*oy + R_wb[1][2]*oz);
      const cz = bz + (R_wb[2][0]*ox + R_wb[2][1]*oy + R_wb[2][2]*oz);

      // Rotation: R_WC = R_WB * R_BC
      const R_wc = matrixMultiply(R_wb, EXTRINSIC_T_BC.rotation);
      const q_wc = matrixToQuat(R_wc);
      const euler_wc = this.quaternionToEuler(q_wc.qw, q_wc.qx, q_wc.qy, q_wc.qz);

      return {
        x: cx,
        y: cy,
        z: cz,
        roll: euler_wc.roll,
        pitch: euler_wc.pitch,
        yaw: euler_wc.yaw,
        qw: q_wc.qw,
        qx: q_wc.qx,
        qy: q_wc.qy,
        qz: q_wc.qz
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

      if (data.connection) { Object.assign(this.state.connection, data.connection); _syncHeaderBadges(this.state.connection); }
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
      _syncHeaderBadges('DISCONNECTED');
      this.state.connection.ros = false;
      this.state.connection.websocket = false;
      this.state.connection.state = 'AWAITING LIVE TELEMETRY';
      this.state.connection.latency_ms = null;
      
      this.state.position = { x: null, y: null, z: null, total_dist: 0 };
      this.state.velocity = { vx: null, vy: null, vz: null, norm: null };
      this.state.acceleration = { ax: null, ay: null, az: null, norm: null };
      this.state.orientation = { roll: null, pitch: null, yaw: null, roll_rate: null, pitch_rate: null, yaw_rate: null, qw: null, qx: null, qy: null, qz: null };
      this.state.camera_pose = { x: null, y: null, z: null, roll: null, pitch: null, yaw: null };
      this.state.trajectories.body = [];
      this.state.trajectories.camera = [];
      this.state.landmarks = [];

      this.state.health = {
        camera: 'DISCONNECTED',
        imu: 'DISCONNECTED',
        vio: 'DISCONNECTED',
        controller: 'DISCONNECTED',
        websocket: 'DISCONNECTED'
      };

      this.state.vio.status = 'AWAITING VIO';
      this.state.controller.status = 'AWAITING TELEMETRY';
      this.state.controller.flight_state = 'AWAITING TELEMETRY';
      this.state.controller.armed = false;

      this._emit('stateChange', this.state);
      this._emit('health', this.state.health);
      this._emit('telemetry', this.state);
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
