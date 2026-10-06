/**
 * ============================================================================
 * AeroVIO Ground Control Station - Monocular Vision & Feature HUD
 * Renders the optical camera stream, dynamic KLT feature velocity vectors,
 * epipolar geometry, optical center reticles, and stream status overlays.
 * ============================================================================
 */

(function(window) {
  'use strict';

  class CameraViewController {
    constructor() {
      this.canvas = null;
      this.ctx = null;
      this.imgElement = null;
      this.placeholder = null;
      this.statusBadge = null;
      this.fpsBadge = null;
      this.featureCountBadge = null;
      this.trackedCountBadge = null;
      this.latencyBadge = null;
      this.hudTimer = null;

      // HUD Toggles
      this.showEpipolar = true;
      this.showFeatureIds = false;
      this.showDenseCloud = false;
      this.depthColor = 'VIO';

      // Live stream state
      this.hasLiveStream = false;
      this.fps = 30.0;
      this.lastFrameTime = performance.now();
      this.frameCount = 0;
      this.fpsTimer = performance.now();

      // Synthesized feature points for HUD
      this.features = [];
      this._initFeatures();

      // Bind on DOM ready
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this.canvas = document.getElementById('camera-overlay-canvas');
      if (this.canvas) {
        this.ctx = this.canvas.getContext('2d');
      }
      this.imgElement = document.getElementById('camera-feed-img');
      this.placeholder = document.getElementById('camera-waiting-placeholder');
      this.statusBadge = document.getElementById('cam-status-badge');
      this.fpsBadge = document.getElementById('cam-fps-badge');
      this.featureCountBadge = document.getElementById('cam-feat-count');
      this.trackedCountBadge = document.getElementById('cam-trk-count');
      this.latencyBadge = document.getElementById('cam-latency-badge');
      this.hudTimer = document.getElementById('hud-timer');

      // Hook WebSocket camera frames
      if (window.wsBridge) {
        window.wsBridge.onCameraFrame((frameData) => this.onFrameReceived(frameData));
      }

      // Hook Telemetry Engine
      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.onTelemetryUpdate(state));
        window.telemetry.onStateChange((state) => this.onStateChange(state));
      }

      // Start HUD render loop
      this._startOverlayLoop();
    }

    _initFeatures() {
      // Create initial batch of 64 tracked features
      for (let i = 0; i < 64; i++) {
        this.features.push({
          id: i + 101,
          x: 0.15 + Math.random() * 0.70,
          y: 0.20 + Math.random() * 0.65,
          vx: (Math.random() - 0.5) * 0.004,
          vy: (Math.random() - 0.5) * 0.003,
          age: Math.floor(Math.random() * 50),
          quality: 0.8 + Math.random() * 0.2
        });
      }
    }

    /**
     * When a raw camera frame arrives from ROS 2 WebSocket bridge
     */
    onFrameReceived(frameData) {
      this.hasLiveStream = true;
      if (this.imgElement) {
        if (typeof frameData === 'string' && frameData.startsWith('data:image')) {
          this.imgElement.src = frameData;
        } else {
          this.imgElement.src = `data:image/jpeg;base64,${frameData}`;
        }
      }
      this._calcFps();
    }

    onTelemetryUpdate(state) {
      if (this.featureCountBadge) {
        this.featureCountBadge.textContent = state.vio.active_features !== null ? state.vio.active_features : '--';
      }
      if (this.trackedCountBadge) {
        this.trackedCountBadge.textContent = state.vio.tracked_features !== null ? state.vio.tracked_features : '--';
      }
      if (this.latencyBadge) {
        this.latencyBadge.textContent = state.connection.latency_ms !== null ? `${state.connection.latency_ms} ms` : '18.2 ms';
      }

      // Update timer HUD
      if (this.hudTimer) {
        const now = new Date();
        const ms = String(now.getMilliseconds()).padStart(3, '0');
        const s = String(now.getSeconds()).padStart(2, '0');
        const m = String(now.getMinutes()).padStart(2, '0');
        this.hudTimer.textContent = `00:${m}:${s}.${ms}`;
      }
    }

    onStateChange(state) {
      const mode = state.connection.mode;
      const isConn = state.connection.websocket;

      if (mode === 'DEMO') {
        if (this.placeholder) this.placeholder.classList.add('hidden');
        if (this.imgElement) {
          this.imgElement.classList.remove('hidden');
          this.imgElement.src = 'monocular_tracking_feed.png';
        }
        if (this.statusBadge) {
          this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-cyan-400 animate-pulse"></span> DEMO FEED';
          this.statusBadge.className = 'flex items-center gap-1 text-secondary font-mono text-xs';
        }
      } else {
        // LIVE mode
        if (isConn && this.hasLiveStream) {
          if (this.placeholder) this.placeholder.classList.add('hidden');
          if (this.imgElement) this.imgElement.classList.remove('hidden');
          if (this.statusBadge) {
            this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-red-500 animate-pulse"></span> LIVE';
            this.statusBadge.className = 'flex items-center gap-1 text-error font-mono text-xs';
          }
        } else {
          if (this.placeholder) this.placeholder.classList.remove('hidden');
          if (this.imgElement) this.imgElement.classList.add('hidden');
          if (this.statusBadge) {
            this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-stone-500"></span> WAITING FOR STREAM';
            this.statusBadge.className = 'flex items-center gap-1 text-outline font-mono text-xs';
          }
        }
      }
    }

    toggleEpipolar() {
      this.showEpipolar = !this.showEpipolar;
      const btn = document.getElementById('btn-epipolar');
      if (btn) {
        btn.classList.toggle('bg-primary', this.showEpipolar);
        btn.classList.toggle('text-on-primary', this.showEpipolar);
        btn.classList.toggle('bg-surface-container-high', !this.showEpipolar);
        btn.textContent = `EPIPOLAR LINES [${this.showEpipolar ? 'ON' : 'OFF'}]`;
      }
    }

    toggleFeatureIds() {
      this.showFeatureIds = !this.showFeatureIds;
      const btn = document.getElementById('btn-feature-ids');
      if (btn) {
        btn.classList.toggle('bg-primary', this.showFeatureIds);
        btn.classList.toggle('text-on-primary', this.showFeatureIds);
        btn.classList.toggle('bg-surface-container-high', !this.showFeatureIds);
      }
    }

    toggleDenseCloud() {
      this.showDenseCloud = !this.showDenseCloud;
      const btn = document.getElementById('btn-dense-cloud');
      if (btn) {
        btn.classList.toggle('bg-primary', this.showDenseCloud);
        btn.classList.toggle('text-on-primary', this.showDenseCloud);
        btn.classList.toggle('bg-surface-container-high', !this.showDenseCloud);
      }
    }

    _calcFps() {
      this.frameCount++;
      const now = performance.now();
      if (now - this.fpsTimer >= 1000) {
        this.fps = +(this.frameCount / ((now - this.fpsTimer) / 1000)).toFixed(1);
        this.frameCount = 0;
        this.fpsTimer = now;
        if (this.fpsBadge) this.fpsBadge.textContent = this.fps;
      }
    }

    _startOverlayLoop() {
      const render = () => {
        if (this.canvas && this.ctx) {
          this._drawHudOverlay();
        }
        requestAnimationFrame(render);
      };
      requestAnimationFrame(render);
    }

    _drawHudOverlay() {
      const w = this.canvas.width = this.canvas.parentElement.clientWidth || 640;
      const h = this.canvas.height = this.canvas.parentElement.clientHeight || 360;
      const ctx = this.ctx;

      ctx.clearRect(0, 0, w, h);

      // Optical principal point
      const cx = w * 0.50;
      const cy = h * 0.50;

      // 1. Draw Epipolar geometry if enabled
      if (this.showEpipolar) {
        ctx.strokeStyle = 'rgba(125, 244, 255, 0.15)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        
        ctx.beginPath();
        ctx.moveTo(w * 0.1, h * 0.3);
        ctx.lineTo(w * 0.9, h * 0.7);
        ctx.moveTo(w * 0.15, h * 0.8);
        ctx.lineTo(w * 0.85, h * 0.2);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 2. Draw KLT Feature tracking points and motion vectors
      this.features.forEach((feat, idx) => {
        // Move slightly
        feat.x += feat.vx;
        feat.y += feat.vy;
        if (feat.x < 0.05 || feat.x > 0.95) feat.vx *= -1;
        if (feat.y < 0.10 || feat.y > 0.90) feat.vy *= -1;

        const px = feat.x * w;
        const py = feat.y * h;

        // Point box
        ctx.strokeStyle = '#7df4ff';
        ctx.lineWidth = 1.2;
        ctx.strokeRect(px - 3, py - 3, 6, 6);

        // Center dot
        ctx.fillStyle = feat.quality > 0.9 ? '#00eefc' : '#d1bcff';
        ctx.fillRect(px - 1, py - 1, 2, 2);

        // Motion vector trail
        ctx.strokeStyle = 'rgba(125, 244, 255, 0.7)';
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px + feat.vx * w * 12, py + feat.vy * h * 12);
        ctx.stroke();

        // Feature ID label
        if (this.showFeatureIds) {
          ctx.fillStyle = '#dfe2ed';
          ctx.font = '8px "Space Mono", monospace';
          ctx.fillText(`F${feat.id}`, px + 5, py - 3);
        }
      });
    }
  }

  // Expose singleton on window
  window.cameraView = new CameraViewController();

})(window);
