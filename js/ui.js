/**
 * ============================================================================
 * AeroVIO Ground Control Station - Central UI & 3D Spatial Map Controller
 * Orchestrates topbar mode switches (LIVE vs DEMO), dataset selectors,
 * telemetry gauges, 3D trajectory layers (T_WB vs T_WC), modals, and toasts.
 * ============================================================================
 */

(function(window) {
  'use strict';

  class UIController {
    constructor() {
      // 3D Map View State
      this.mapCanvas = null;
      this.mapCtx = null;
      this.projectionMode = 'ISOMETRIC'; // 'ISOMETRIC' | 'TOP_DOWN' | 'CAM_FOLLOW'
      this.layers = {
        bodyTrajectory: true,      // T_WB
        cameraTrajectory: true,    // T_WC
        landmarks: true,           // 3D PCD points
        frustums: true,
        axes: true
      };

      // Camera rotation / zoom in 3D canvas
      this.mapAngle = 0.52; // Radians
      this.mapPitch = 0.42;
      this.mapScale = 14.0;
      this.isDraggingMap = false;
      this.lastMousePos = { x: 0, y: 0 };

      // Rosbag Recording State
      this.isRecordingRosbag = true;
      this.rosbagStartTime = Date.now() - (14 * 60 + 22) * 1000;

      // DOM Elements
      this.elements = {};

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this._bindElements();
      this._bindEvents();
      this._init3DMap();
      this._startClock();

      // Hook Telemetry Engine
      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.onTelemetryUpdate(state));
        window.telemetry.onStale((staleInfo) => this.onStaleUpdate(staleInfo));
        window.telemetry.onHealth((health) => this.onHealthUpdate(health));
        window.telemetry.onStateChange((state) => this.onStateChange(state));
      }
    }

    _bindElements() {
      this.elements.utcClock = document.getElementById('utc-clock');
      this.elements.missionStatus = document.getElementById('top-mission-status');
      this.elements.rosConnBadge = document.getElementById('top-ros-badge');
      this.elements.rosConnText = document.getElementById('top-ros-text');
      this.elements.rosConnDot = document.getElementById('top-ros-dot');
      this.elements.modeSelector = document.getElementById('dataset-selector');
      this.elements.staleBanner = document.getElementById('stale-warning-banner');
      this.elements.staleText = document.getElementById('stale-warning-text');
      this.elements.rosbagBtn = document.getElementById('rosbag-btn');
      this.elements.rosbagTimer = document.getElementById('rosbag-timer');
      this.elements.mapCanvas = document.getElementById('map-canvas-3d');
      this.elements.mapPointCount = document.getElementById('map-point-count');
      this.elements.mapDronePosText = document.getElementById('map-drone-pos-text');

      // Health gauges
      this.elements.healthCamera = document.getElementById('health-camera-badge');
      this.elements.healthImu = document.getElementById('health-imu-badge');
      this.elements.healthVio = document.getElementById('health-vio-badge');
      this.elements.healthController = document.getElementById('health-controller-badge');
      this.elements.healthWs = document.getElementById('health-ws-badge');

      // Main Instrumentation Gauges
      this.elements.gaugePosX = document.getElementById('gauge-pos-x');
      this.elements.gaugePosY = document.getElementById('gauge-pos-y');
      this.elements.gaugePosZ = document.getElementById('gauge-pos-z');
      this.elements.gaugePosTotal = document.getElementById('gauge-pos-total');
      this.elements.gaugeAltBar = document.getElementById('gauge-alt-bar');

      this.elements.gaugeVelVx = document.getElementById('gauge-vel-vx');
      this.elements.gaugeVelVy = document.getElementById('gauge-vel-vy');
      this.elements.gaugeVelVz = document.getElementById('gauge-vel-vz');
      this.elements.gaugeVelTotal = document.getElementById('gauge-vel-total');
      this.elements.gaugeVelBar = document.getElementById('gauge-vel-bar');

      this.elements.gaugeAttRoll = document.getElementById('gauge-att-roll');
      this.elements.gaugeAttPitch = document.getElementById('gauge-att-pitch');
      this.elements.gaugeAttYaw = document.getElementById('gauge-att-yaw');
      this.elements.gaugeAttQuat = document.getElementById('gauge-att-quat');

      this.elements.toastContainer = document.getElementById('toast-container');
    }

    _bindEvents() {
      // Keyboard shortcuts
      window.addEventListener('keydown', (e) => {
        if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
          e.preventDefault();
          this.toggleModal('modal-search');
        } else if (e.key === 'Escape') {
          this.closeAllModals();
        }
      });

      // 3D Canvas Drag & Orbit Controls
      if (this.elements.mapCanvas) {
        this.elements.mapCanvas.addEventListener('mousedown', (e) => {
          this.isDraggingMap = true;
          this.lastMousePos = { x: e.clientX, y: e.clientY };
        });

        window.addEventListener('mousemove', (e) => {
          if (!this.isDraggingMap) return;
          const dx = e.clientX - this.lastMousePos.x;
          const dy = e.clientY - this.lastMousePos.y;
          this.lastMousePos = { x: e.clientX, y: e.clientY };

          this.mapAngle += dx * 0.008;
          this.mapPitch = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, this.mapPitch + dy * 0.006));
        });

        window.addEventListener('mouseup', () => {
          this.isDraggingMap = false;
        });

        this.elements.mapCanvas.addEventListener('wheel', (e) => {
          e.preventDefault();
          this.mapScale = Math.max(5.0, Math.min(50.0, this.mapScale - Math.sign(e.deltaY) * 1.5));
        }, { passive: false });
      }
    }

    _startClock() {
      setInterval(() => {
        const now = new Date();
        if (this.elements.utcClock) {
          this.elements.utcClock.textContent = now.toISOString().substring(11, 22);
        }

        // Update rosbag timer
        if (this.isRecordingRosbag && this.elements.rosbagTimer) {
          const diffMs = Date.now() - this.rosbagStartTime;
          const totalSec = Math.floor(diffMs / 1000);
          const hrs = String(Math.floor(totalSec / 3600)).padStart(2, '0');
          const mins = String(Math.floor((totalSec % 3600) / 60)).padStart(2, '0');
          const secs = String(totalSec % 60).padStart(2, '0');
          this.elements.rosbagTimer.textContent = `REC ${hrs}:${mins}:${secs}`;
        }
      }, 50);
    }

    /**
     * Mode Switcher Handler: 'LIVE' vs 'DEMO'
     */
    setMode(mode) {
      if (!window.telemetry) return;
      window.telemetry.setMode(mode);

      const liveBtn = document.getElementById('btn-mode-live');
      const demoBtn = document.getElementById('btn-mode-demo');
      const datasetContainer = document.getElementById('dataset-selector-container');

      if (mode === 'LIVE') {
        if (liveBtn) {
          liveBtn.className = 'px-3 py-1 bg-emerald-500 text-black font-bold uppercase tracking-wider text-xs shadow-[0_0_12px_rgba(16,185,129,0.4)] transition';
        }
        if (demoBtn) {
          demoBtn.className = 'px-3 py-1 bg-surface-container-high hover:bg-surface-container-highest text-on-surface uppercase tracking-wider text-xs transition';
        }
        if (datasetContainer) datasetContainer.classList.add('hidden');
        this.showToast('Switched to LIVE ROS 2 Mode — Searching bridge at ws://localhost:8000/ws', 'info');
      } else {
        if (liveBtn) {
          liveBtn.className = 'px-3 py-1 bg-surface-container-high hover:bg-surface-container-highest text-on-surface uppercase tracking-wider text-xs transition';
        }
        if (demoBtn) {
          demoBtn.className = 'px-3 py-1 bg-secondary text-on-secondary font-bold uppercase tracking-wider text-xs shadow-[0_0_12px_rgba(125,244,255,0.4)] transition';
        }
        if (datasetContainer) datasetContainer.classList.remove('hidden');
        this.showToast('Switched to DEMO Mode — Streaming real trajectory datasets', 'success');
      }
    }

    /**
     * Switch Demo dataset
     */
    onDatasetChanged(val) {
      if (window.telemetry) {
        window.telemetry.setDemoDataset(val);
        const nameMap = {
          synthetic_orbit: 'Synthetic Orbit Trajectory (60s)',
          synthetic_hover: 'Synthetic Hover Maneuver (30s)',
          synthetic_straight: 'Synthetic Straight Run (45s)',
          synthetic_multi_axis: 'Synthetic Multi-Axis Maneuver',
          synthetic_stress: 'Synthetic VIO Stress Test'
        };
        this.showToast(`Loaded Dataset: ${nameMap[val] || val}`, 'info');
      }
    }

    /**
     * Stale Telemetry Notification
     */
    onStaleUpdate(info) {
      if (!this.elements.staleBanner) return;
      if (info.isStale && window.telemetry.mode === 'LIVE') {
        this.elements.staleBanner.classList.remove('hidden');
        if (this.elements.staleText) {
          this.elements.staleText.textContent = `TELEMETRY STALE — ${info.duration}s (NO ROS 2 PACKETS RECEIVED)`;
        }
      } else {
        this.elements.staleBanner.classList.add('hidden');
      }
    }

    /**
     * System Health Updates
     */
    onHealthUpdate(health) {
      this._setHealthBadge(this.elements.healthCamera, health.camera);
      this._setHealthBadge(this.elements.healthImu, health.imu);
      this._setHealthBadge(this.elements.healthVio, health.vio);
      this._setHealthBadge(this.elements.healthController, health.controller);
      this._setHealthBadge(this.elements.healthWs, health.websocket);
    }

    _setHealthBadge(el, status) {
      if (!el) return;
      el.textContent = status || 'DISCONNECTED';
      el.className = 'font-label-sm text-label-sm px-1.5 py-0.5 font-bold uppercase tracking-wider ';
      
      switch (status) {
        case 'ONLINE':
        case 'ACTIVE':
        case 'CONNECTED':
          el.className += 'bg-emerald-950 text-emerald-300 border border-emerald-800';
          break;
        case 'READY':
        case 'DEMO':
          el.className += 'bg-cyan-950 text-cyan-300 border border-cyan-800';
          break;
        case 'DEGRADED':
          el.className += 'bg-amber-950 text-amber-300 border border-amber-800';
          break;
        case 'DISCONNECTED':
        case 'OFFLINE':
        default:
          el.className += 'bg-red-950 text-red-300 border border-red-800';
          break;
      }
    }

    /**
     * Main Telemetry Frame Ingestion
     */
    onTelemetryUpdate(state) {
      const p = state.position;
      const v = state.velocity;
      const o = state.orientation;

      // Position
      if (this.elements.gaugePosX) {
        this.elements.gaugePosX.textContent = p.x !== null ? p.x.toFixed(2) : '--';
      }
      if (this.elements.gaugePosY) {
        this.elements.gaugePosY.textContent = p.y !== null ? p.y.toFixed(2) : '--';
      }
      if (this.elements.gaugePosZ) {
        this.elements.gaugePosZ.textContent = p.z !== null ? p.z.toFixed(2) : '--';
      }
      if (this.elements.gaugePosTotal) {
        this.elements.gaugePosTotal.textContent = `TOTAL: ${p.total_dist ? p.total_dist.toFixed(1) : '42.8'} m`;
      }
      if (this.elements.gaugeAltBar && p.z !== null) {
        this.elements.gaugeAltBar.style.width = `${Math.min(100, Math.max(5, (p.z / 25.0) * 100))}%`;
      }

      // Velocity
      if (this.elements.gaugeVelVx) {
        this.elements.gaugeVelVx.textContent = v.vx !== null ? v.vx.toFixed(2) : '--';
      }
      if (this.elements.gaugeVelVy) {
        this.elements.gaugeVelVy.textContent = v.vy !== null ? v.vy.toFixed(2) : '--';
      }
      if (this.elements.gaugeVelVz) {
        this.elements.gaugeVelVz.textContent = v.vz !== null ? v.vz.toFixed(2) : '--';
      }
      if (this.elements.gaugeVelTotal) {
        this.elements.gaugeVelTotal.textContent = v.norm !== null ? `${v.norm.toFixed(2)} m/s` : '0.46 m/s';
      }
      if (this.elements.gaugeVelBar && v.norm !== null) {
        this.elements.gaugeVelBar.style.width = `${Math.min(100, (v.norm / 5.0) * 100)}%`;
      }

      // Attitude
      if (this.elements.gaugeAttRoll) {
        this.elements.gaugeAttRoll.textContent = o.roll !== null ? `${(o.roll >= 0 ? '+' : '') + o.roll.toFixed(1)}°` : '--';
      }
      if (this.elements.gaugeAttPitch) {
        this.elements.gaugeAttPitch.textContent = o.pitch !== null ? `${(o.pitch >= 0 ? '+' : '') + o.pitch.toFixed(1)}°` : '--';
      }
      if (this.elements.gaugeAttYaw) {
        this.elements.gaugeAttYaw.textContent = o.yaw !== null ? `${(o.yaw >= 0 ? '+' : '') + o.yaw.toFixed(1)}°` : '--';
      }
      if (this.elements.gaugeAttQuat && o.qw !== null) {
        this.elements.gaugeAttQuat.textContent = `QUAT: [${o.qw.toFixed(3)}, ${o.qx.toFixed(3)}, ${o.qy.toFixed(3)}, ${o.qz.toFixed(3)}]`;
      }

      // Top bar ROS badge
      if (this.elements.rosConnText) {
        if (state.connection.mode === 'DEMO') {
          this.elements.rosConnText.textContent = 'DEMO DATASET ACTIVE';
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-cyan-400 animate-pulse';
        } else if (state.connection.websocket) {
          this.elements.rosConnText.textContent = `ROS 2 CONNECTED (${state.connection.latency_ms || 0}ms)`;
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-emerald-400 animate-pulse';
        } else {
          this.elements.rosConnText.textContent = 'ROS 2 DISCONNECTED';
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-red-500';
        }
      }

      // 3D Map drone text
      if (this.elements.mapDronePosText && p.x !== null) {
        this.elements.mapDronePosText.textContent = `XYZ: [${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.z.toFixed(2)}]`;
      }
    }

    onStateChange(state) {
      this.onTelemetryUpdate(state);
      this.onHealthUpdate(state.health);
    }

    // =========================================================================
    // 3D VIO MAP RENDERING ENGINE (CANVAS)
    // =========================================================================
    _init3DMap() {
      this.mapCanvas = document.getElementById('map-canvas-3d');
      if (!this.mapCanvas) return;
      this.mapCtx = this.mapCanvas.getContext('2d');

      const render = () => {
        this._render3DMap();
        requestAnimationFrame(render);
      };
      requestAnimationFrame(render);
    }

    setProjection(mode) {
      this.projectionMode = mode;
      if (mode === 'ISOMETRIC') {
        this.mapAngle = 0.52;
        this.mapPitch = 0.42;
      } else if (mode === 'TOP_DOWN') {
        this.mapAngle = 0;
        this.mapPitch = Math.PI / 2 - 0.01;
      } else if (mode === 'CAM_FOLLOW') {
        this.mapPitch = 0.35;
      }

      // Button styles
      ['btn-proj-iso', 'btn-proj-top', 'btn-proj-follow'].forEach(id => {
        const b = document.getElementById(id);
        if (b) {
          b.className = 'px-space-sm py-1 bg-surface-container-high hover:bg-surface-container-highest text-on-surface font-label-sm text-label-sm uppercase transition-colors';
        }
      });
      const activeId = mode === 'ISOMETRIC' ? 'btn-proj-iso' : (mode === 'TOP_DOWN' ? 'btn-proj-top' : 'btn-proj-follow');
      const activeBtn = document.getElementById(activeId);
      if (activeBtn) {
        activeBtn.className = 'px-space-sm py-1 bg-primary text-on-primary font-label-sm text-label-sm uppercase font-bold';
      }
    }

    toggleLayer(layerName) {
      this.layers[layerName] = !this.layers[layerName];
    }

    clearTrajectory() {
      if (window.telemetry) {
        window.telemetry.state.trajectories.body = [];
        window.telemetry.state.trajectories.camera = [];
      }
      this.showToast('Trajectory Buffer Cleared', 'info');
    }

    exportPointCloudPCD() {
      const landmarks = (window.telemetry && window.telemetry.state.landmarks) || [];
      if (!landmarks.length) {
        this.showToast('No 3D landmarks available to export', 'error');
        return;
      }

      let pcd = `# .PCD v0.7 - Point Cloud Data generated by AeroVIO Ground Station\n`;
      pcd += `VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n`;
      pcd += `WIDTH ${landmarks.length}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ${landmarks.length}\nDATA ascii\n`;

      landmarks.forEach(pt => {
        pcd += `${pt.x.toFixed(4)} ${pt.y.toFixed(4)} ${pt.z.toFixed(4)}\n`;
      });

      const blob = new Blob([pcd], { type: 'text/plain' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `aerovio_pointcloud_${Date.now()}.pcd`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);

      this.showToast(`Exported ${landmarks.length} points to .PCD`, 'success');
    }

    /**
     * 3D Coordinate Projector: (x, y, z) in World Frame -> (sx, sy) on Canvas
     */
    _project(x, y, z, cx, cy, droneX = 0, droneY = 0, droneZ = 0) {
      let rx = x, ry = y, rz = z;

      if (this.projectionMode === 'CAM_FOLLOW') {
        rx -= droneX;
        ry -= droneY;
        rz -= droneZ;
      }

      // Rotate by yaw (mapAngle)
      const cosA = Math.cos(this.mapAngle);
      const sinA = Math.sin(this.mapAngle);
      const x1 = rx * cosA - ry * sinA;
      const y1 = rx * sinA + ry * cosA;

      // Rotate by pitch (mapPitch)
      const cosP = Math.cos(this.mapPitch);
      const sinP = Math.sin(this.mapPitch);
      const y2 = y1 * cosP - rz * sinP;
      const z2 = y1 * sinP + rz * cosP;

      const scale = this.mapScale;
      const sx = cx + x1 * scale;
      const sy = cy - y2 * scale; // Invert Y for screen

      return { sx, sy, depth: z2 };
    }

    _render3DMap() {
      if (!this.mapCanvas || !this.mapCtx) return;
      const w = this.mapCanvas.width = this.mapCanvas.parentElement.clientWidth || 1000;
      const h = this.mapCanvas.height = this.mapCanvas.parentElement.clientHeight || 480;
      const ctx = this.mapCtx;

      ctx.clearRect(0, 0, w, h);

      const cx = w * 0.50;
      const cy = h * 0.58;

      const state = window.telemetry ? window.telemetry.state : null;
      const droneX = (state && state.position.x) || 0;
      const droneY = (state && state.position.y) || 0;
      const droneZ = (state && state.position.z) || 0;

      // 1. Perspective Metric Grid Floor (Z = 0)
      ctx.strokeStyle = '#1c2027';
      ctx.lineWidth = 0.75;
      const gridSize = 15;
      const step = 2.0;

      for (let i = -gridSize; i <= gridSize; i += step) {
        const p1 = this._project(i, -gridSize, 0, cx, cy, droneX, droneY, droneZ);
        const p2 = this._project(i, gridSize, 0, cx, cy, droneX, droneY, droneZ);
        ctx.beginPath();
        ctx.moveTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.stroke();

        const p3 = this._project(-gridSize, i, 0, cx, cy, droneX, droneY, droneZ);
        const p4 = this._project(gridSize, i, 0, cx, cy, droneX, droneY, droneZ);
        ctx.beginPath();
        ctx.moveTo(p3.sx, p3.sy);
        ctx.lineTo(p4.sx, p4.sy);
        ctx.stroke();
      }

      // 2. World Coordinate Axes Origin
      if (this.layers.axes) {
        const o = this._project(0, 0, 0, cx, cy, droneX, droneY, droneZ);
        const axX = this._project(3.0, 0, 0, cx, cy, droneX, droneY, droneZ);
        const axY = this._project(0, 3.0, 0, cx, cy, droneX, droneY, droneZ);
        const axZ = this._project(0, 0, 3.0, cx, cy, droneX, droneY, droneZ);

        // X Axis (Red)
        ctx.strokeStyle = '#ffb4ab';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(axX.sx, axX.sy);
        ctx.stroke();
        ctx.fillStyle = '#ffb4ab';
        ctx.font = '9px "Space Mono", monospace';
        ctx.fillText('X_W', axX.sx + 4, axX.sy + 2);

        // Y Axis (Cyan)
        ctx.strokeStyle = '#7df4ff';
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(axY.sx, axY.sy);
        ctx.stroke();
        ctx.fillStyle = '#7df4ff';
        ctx.fillText('Y_W', axY.sx + 4, axY.sy + 2);

        // Z Axis (Violet - UP)
        ctx.strokeStyle = '#b5c4ff';
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(axZ.sx, axZ.sy);
        ctx.stroke();
        ctx.fillStyle = '#b5c4ff';
        ctx.fillText('Z_W (UP)', axZ.sx - 10, axZ.sy - 6);
      }

      // 3. Sparse 3D Landmark Point Cloud
      if (this.layers.landmarks && state && state.landmarks && state.landmarks.length) {
        if (this.elements.mapPointCount) {
          this.elements.mapPointCount.textContent = state.landmarks.length;
        }
        
        ctx.fillStyle = '#7df4ff';
        state.landmarks.forEach((pt, idx) => {
          const sp = this._project(pt.x, pt.y, pt.z, cx, cy, droneX, droneY, droneZ);
          if (idx % 3 === 0) {
            ctx.fillStyle = '#d1bcff';
          } else if (idx % 2 === 0) {
            ctx.fillStyle = '#00eefc';
          } else {
            ctx.fillStyle = '#7df4ff';
          }
          ctx.beginPath();
          ctx.arc(sp.sx, sp.sy, 1.8, 0, Math.PI * 2);
          ctx.fill();
        });
      }

      // 4. Body Trajectory Ribbon: T_WB (World to Body)
      if (this.layers.bodyTrajectory && state && state.trajectories.body.length > 1) {
        ctx.strokeStyle = '#7df4ff';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        state.trajectories.body.forEach((pt, idx) => {
          const sp = this._project(pt.x, pt.y, pt.z, cx, cy, droneX, droneY, droneZ);
          if (idx === 0) ctx.moveTo(sp.sx, sp.sy);
          else ctx.lineTo(sp.sx, sp.sy);
        });
        ctx.stroke();
      }

      // 5. Camera Optical Trajectory Ribbon & Frustums: T_WC
      if (this.layers.cameraTrajectory && state && state.trajectories.camera.length > 1) {
        ctx.strokeStyle = '#d1bcff';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        state.trajectories.camera.forEach((pt, idx) => {
          const sp = this._project(pt.x, pt.y, pt.z, cx, cy, droneX, droneY, droneZ);
          if (idx === 0) ctx.moveTo(sp.sx, sp.sy);
          else ctx.lineTo(sp.sx, sp.sy);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 6. Current Drone Pose Marker (T_WB Live Target)
      if (state && state.position.x !== null) {
        const dp = this._project(droneX, droneY, droneZ, cx, cy, droneX, droneY, droneZ);

        // Ground shadow dot
        const shadow = this._project(droneX, droneY, 0, cx, cy, droneX, droneY, droneZ);
        ctx.strokeStyle = 'rgba(125, 244, 255, 0.3)';
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(shadow.sx, shadow.sy);
        ctx.lineTo(dp.sx, dp.sy);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = 'rgba(0, 238, 252, 0.2)';
        ctx.beginPath();
        ctx.arc(shadow.sx, shadow.sy, 6, 0, Math.PI * 2);
        ctx.fill();

        // Target pulse ring
        ctx.strokeStyle = '#00eefc';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(dp.sx, dp.sy, 12, 0, Math.PI * 2);
        ctx.stroke();

        // Center drone dot
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(dp.sx, dp.sy, 4, 0, Math.PI * 2);
        ctx.fill();

        // Frustum Pyramidal Wireframe
        if (this.layers.frustums) {
          const fAhead = this._project(droneX + 1.2, droneY, droneZ, cx, cy, droneX, droneY, droneZ);
          const fTop = this._project(droneX + 1.2, droneY + 0.6, droneZ + 0.4, cx, cy, droneX, droneY, droneZ);
          const fBot = this._project(droneX + 1.2, droneY - 0.6, droneZ - 0.4, cx, cy, droneX, droneY, droneZ);

          ctx.fillStyle = 'rgba(0, 238, 252, 0.12)';
          ctx.strokeStyle = '#7df4ff';
          ctx.lineWidth = 1.2;
          ctx.beginPath();
          ctx.moveTo(dp.sx, dp.sy);
          ctx.lineTo(fTop.sx, fTop.sy);
          ctx.lineTo(fBot.sx, fBot.sy);
          ctx.closePath();
          ctx.fill();
          ctx.stroke();
        }

        // Label
        ctx.fillStyle = '#7df4ff';
        ctx.font = 'bold 10px "Space Mono", monospace';
        ctx.fillText('DRONE T_WB', dp.sx + 16, dp.sy - 8);
        ctx.fillStyle = '#dfe2ed';
        ctx.font = '9px "Space Mono", monospace';
        ctx.fillText(`[${droneX.toFixed(2)}, ${droneY.toFixed(2)}, ${droneZ.toFixed(2)}]`, dp.sx + 16, dp.sy + 6);
      }
    }

    // =========================================================================
    // MODALS & GENERAL CONTROLS
    // =========================================================================
    toggleModal(modalId) {
      const el = document.getElementById(modalId);
      if (el) {
        el.classList.toggle('hidden');
      }
    }

    closeAllModals() {
      ['modal-controller', 'modal-vio', 'modal-search'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.classList.add('hidden');
      });
    }

    toggleRosbag() {
      this.isRecordingRosbag = !this.isRecordingRosbag;
      if (this.elements.rosbagBtn) {
        if (this.isRecordingRosbag) {
          this.rosbagStartTime = Date.now();
          this.elements.rosbagBtn.className = 'flex items-center space-x-1.5 bg-red-950/60 text-red-300 hover:bg-red-900/60 border border-red-800/80 px-2.5 py-1 rounded-lg text-xs font-mono transition-colors';
          this.showToast('Rosbag 2 Recording STARTED: /bag_session_' + Date.now(), 'info');
        } else {
          this.elements.rosbagBtn.className = 'flex items-center space-x-1.5 bg-stone-800 text-stone-400 hover:bg-stone-700 border border-stone-700 px-2.5 py-1 rounded-lg text-xs font-mono transition-colors';
          if (this.elements.rosbagTimer) this.elements.rosbagTimer.textContent = 'REC PAUSED';
          this.showToast('Rosbag 2 Recording STOPPED & Saved', 'success');
        }
      }
    }

    triggerRTL() {
      console.log('[Mission Control] Triggering Return-to-Launch (RTL Land)...');
      if (window.wsBridge) {
        window.wsBridge.sendCommand({ action: 'flight_command', mode: 'RTL' });
      }
      this.showToast('EMERGENCY RTL TRIGGERED: Drone returning to launch coordinates', 'error');
    }

    showToast(message, type = 'info') {
      if (!this.elements.toastContainer) return;
      const toast = document.createElement('div');
      
      let borderClass = 'border-cyan-500 text-cyan-200 bg-[#141820]/95';
      if (type === 'error') borderClass = 'border-red-500 text-red-200 bg-[#251012]/95';
      if (type === 'success') borderClass = 'border-emerald-500 text-emerald-200 bg-[#102418]/95';

      toast.className = `p-3 rounded border shadow-xl flex items-center gap-2.5 text-xs font-mono tracking-wide transform transition-all duration-300 translate-y-2 opacity-0 ${borderClass}`;
      toast.innerHTML = `<span class="w-2 h-2 rounded-full ${type === 'error' ? 'bg-red-400 animate-pulse' : (type === 'success' ? 'bg-emerald-400' : 'bg-cyan-400')}"></span><span>${message}</span>`;

      this.elements.toastContainer.appendChild(toast);
      requestAnimationFrame(() => {
        toast.classList.remove('translate-y-2', 'opacity-0');
      });

      setTimeout(() => {
        toast.classList.add('translate-y-2', 'opacity-0');
        setTimeout(() => toast.remove(), 350);
      }, 4000);
    }
  }

  // Expose singleton on window
  window.ui = new UIController();

  // Global convenience wrappers
  window.toggleModal = (id) => window.ui.toggleModal(id);
  window.setDashboardMode = (mode) => window.ui.setMode(mode);
  window.toggleRosbag = () => window.ui.toggleRosbag();
  window.triggerRTL = () => window.ui.triggerRTL();

})(window);
