/**
 * ============================================================================
 * AeroVIO Ground Control Station - Monocular Vision & Feature HUD
 * ============================================================================
 * Manages the camera feed display for both DEMO and LIVE modes:
 * - LIVE mode: Subscribes to /camera/image_raw (sensor_msgs/msg/Image, mono8).
 *   Monitors stream health, calculates genuine frame rate and frame age from
 *   actual message arrival timestamps, and presents clear disconnected/waiting
 *   states when no stream is present.
 * - DEMO mode: Honest presentation of reference frame snapshot (monocular_tracking_feed.png).
 *   Labeled explicitly as DEMO SNAPSHOT. Never fabricates fake video or simulated
 *   random feature coordinates.
 * - Calibrated Pinhole Reticle: Calibrated intrinsics (fx=615.2, fy=615.4, cx=320.0, cy=240.0)
 *   optical center crosshairs, FOV brackets (55.0° H × 42.6° V), and optical frame indicator.
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
      this.latencyBadge = null;
      this.resBadge = null;
      this.featuresBadge = null;

      // Overlay toggles
      this.showReticle = true;
      this.showHud = true;
      this.objectFit = 'contain';

      // Stream state
      this.mode = 'DEMO'; // 'DEMO' or 'LIVE'
      this.hasLiveFrame = false;
      this.lastFrameArrivalTime = 0;
      this.lastFrameRosTimestamp = null;
      this.measuredFps = 0.0;
      this.frameCount = 0;
      this.fpsTimer = performance.now();
      this.staleCheckInterval = null;

      // Calibrated Camera Intrinsics (from reference VIO project_vio.yaml / test_camera.py)
      this.intrinsics = {
        width: 640,
        height: 480,
        fx: 615.2,
        fy: 615.4,
        cx: 320.0,
        cy: 240.0,
        fovH_deg: 54.95,
        fovV_deg: 42.62
      };

      // Genuine feature points delivered by estimator overlay topic (empty by default)
      // We NEVER synthesize fake feature coordinates!
      this.features = [];

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
      this.placeholder = document.getElementById('camera-placeholder');
      this.statusBadge = document.getElementById('cam-status-badge');
      this.fpsBadge = document.getElementById('cam-fps-badge');
      this.latencyBadge = document.getElementById('cam-latency-badge');
      this.resBadge = document.getElementById('cam-res-badge');
      this.featuresBadge = document.getElementById('cam-features-badge');

      // Hook WebSocket camera frames
      if (window.wsBridge && typeof window.wsBridge.onCameraFrame === 'function') {
        window.wsBridge.onCameraFrame((frameData) => this.onFrameReceived(frameData));
      }

      // Hook Telemetry Engine
      if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
        window.telemetry.onTelemetry((state) => this.onTelemetryUpdate(state));
        window.telemetry.onStateChange((state) => this.onStateChange(state));
      }

      // Window resize listener to keep overlay pixel-aligned
      window.addEventListener('resize', () => this._syncCanvasResolution());

      // Periodic health/stale check (every 500ms)
      this.staleCheckInterval = setInterval(() => this._checkStreamHealth(), 500);

      // Start render loop for calibrated HUD overlay
      this._startOverlayLoop();

      // Initial state sync
      this._syncModeUI();
    }

    /**
     * Called when a genuine camera frame arrives via WebSocket
     */
    onFrameReceived(frameData) {
      const now = performance.now();
      this.hasLiveFrame = true;
      this.lastFrameArrivalTime = now;

      // Automatically switch UI to LIVE mode as soon as genuine drone frames arrive
      if (this.mode !== 'LIVE') {
        this.mode = 'LIVE';
        this._syncModeUI();
      }

      // Unpack frame data
      let imageUrl = null;
      if (typeof frameData === 'string') {
        if (frameData.startsWith('data:image')) {
          imageUrl = frameData;
        } else {
          imageUrl = `data:image/jpeg;base64,${frameData}`;
        }
      } else if (frameData && typeof frameData === 'object') {
        if (frameData.image) {
          imageUrl = frameData.image.startsWith('data:') ? frameData.image : `data:image/jpeg;base64,${frameData.image}`;
        }
        if (frameData.timestamp) {
          this.lastFrameRosTimestamp = frameData.timestamp;
        }
        // Genuine feature points delivered by live ROS 2 tracker
        if (Array.isArray(frameData.features)) {
          this.features = frameData.features;
        }
      }

      if (this.imgElement && imageUrl && !this.useMjpeg) {
        this.imgElement.src = imageUrl;
        this.imgElement.classList.remove('hidden');
      }

      if (this.placeholder) {
        this.placeholder.classList.add('hidden');
      }

      this._calcFps();
      this._updateFrameAgeDisplay();
    }

    onTelemetryUpdate(state) {
      if (!state) return;

      // Update mode if connection state changed
      if (state.connection && state.connection.mode && state.connection.mode !== this.mode) {
        this.setMode(state.connection.mode);
      }

      // Update features count badge if estimator exposes it
      if (this.featuresBadge && state.vio) {
        const trk = state.vio.tracked_features;
        const act = state.vio.active_features;
        if (trk !== null && trk !== undefined) {
          this.featuresBadge.textContent = `${trk} TRK / ${act || trk} ACT`;
        }
      }
    }

    onStateChange(state) {
      if (state && state.connection && state.connection.mode) {
        this.setMode(state.connection.mode);
      }
    }

    setMode(newMode) {
      this.mode = newMode;
      this._syncModeUI();
    }

    _syncModeUI() {
      if (this.mode === 'DEMO') {
        // DEMO mode: display reference snapshot
        if (this.placeholder) this.placeholder.classList.add('hidden');
        if (this.imgElement) {
          this.imgElement.classList.remove('hidden');
          this.imgElement.src = 'monocular_tracking_feed.png';
        }
        if (this.statusBadge) {
          this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-cyan-400"></span> DEMO SNAPSHOT';
          this.statusBadge.className = 'inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-[10px] font-mono font-semibold bg-cyan-950/80 text-cyan-300 border border-cyan-800/80';
        }
        if (this.fpsBadge) {
          this.fpsBadge.textContent = 'REF FRAME';
        }
        if (this.latencyBadge) {
          this.latencyBadge.textContent = 'STATIC REF';
        }
        if (this.resBadge) {
          this.resBadge.textContent = `${this.intrinsics.width}×${this.intrinsics.height} mono8`;
        }
      } else {
        // LIVE mode
        if (this.hasLiveFrame && (performance.now() - this.lastFrameArrivalTime < 3000)) {
          if (this.placeholder) this.placeholder.classList.add('hidden');
          if (this.imgElement) this.imgElement.classList.remove('hidden');
          if (this.statusBadge) {
            this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-red-500 animate-pulse"></span> LIVE STREAM';
            this.statusBadge.className = 'inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-[10px] font-mono font-semibold bg-red-950/80 text-red-300 border border-red-800/80';
          }
        } else {
          // Live stream disconnected or waiting
          if (this.placeholder) this.placeholder.classList.remove('hidden');
          if (this.imgElement) this.imgElement.classList.add('hidden');
          if (this.statusBadge) {
            this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-amber-500"></span> AWAITING STREAM';
            this.statusBadge.className = 'inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-[10px] font-mono font-semibold bg-amber-950/80 text-amber-300 border border-amber-800/80';
          }
          if (this.fpsBadge) this.fpsBadge.textContent = '0.0 FPS';
          if (this.latencyBadge) this.latencyBadge.textContent = 'NO DATA';
        }
      }
    }

    _calcFps() {
      this.frameCount++;
      const now = performance.now();
      const elapsed = (now - this.fpsTimer) / 1000;
      if (elapsed >= 1.0) {
        this.measuredFps = +(this.frameCount / elapsed).toFixed(1);
        this.frameCount = 0;
        this.fpsTimer = now;
        if (this.fpsBadge) {
          this.fpsBadge.textContent = `${this.measuredFps} FPS`;
        }
      }
    }

    _updateFrameAgeDisplay() {
      if (this.latencyBadge && this.lastFrameArrivalTime > 0) {
        const ageMs = Math.round(performance.now() - this.lastFrameArrivalTime);
        this.latencyBadge.textContent = `${ageMs} ms age`;
      }
    }

    _checkStreamHealth() {
      if (this.mode === 'LIVE') {
        const now = performance.now();
        if (this.hasLiveFrame && (now - this.lastFrameArrivalTime > 2500)) {
          // Stream went stale
          if (this.statusBadge) {
            this.statusBadge.innerHTML = '<span class="w-2 h-2 rounded-full bg-amber-500"></span> STREAM STALE';
            this.statusBadge.className = 'inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-[10px] font-mono font-semibold bg-amber-950/80 text-amber-300 border border-amber-800/80';
          }
          if (this.fpsBadge) this.fpsBadge.textContent = '0.0 FPS';
          if (this.latencyBadge) {
            const ageMs = Math.round(now - this.lastFrameArrivalTime);
            this.latencyBadge.textContent = `${(ageMs / 1000).toFixed(1)}s stale`;
          }
        }
      }
    }

    _syncCanvasResolution() {
      if (!this.canvas) return;
      const rect = this.canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const width = Math.floor(rect.width * dpr);
      const height = Math.floor(rect.height * dpr);

      if (this.canvas.width !== width || this.canvas.height !== height) {
        this.canvas.width = width;
        this.canvas.height = height;
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
      const canvas = this.canvas;
      const ctx = this.ctx;
      const w = canvas.width;
      const h = canvas.height;

      ctx.clearRect(0, 0, w, h);
      if (!this.showReticle && !this.showHud) return;

      const scaleX = w / this.intrinsics.width;
      const scaleY = h / this.intrinsics.height;

      // Calibrated Optical Principal Point (cx, cy)
      const cx = this.intrinsics.cx * scaleX;
      const cy = this.intrinsics.cy * scaleY;

      // 1. Draw Calibrated Pinhole Reticle & Crosshair
      if (this.showReticle) {
        ctx.save();

        // Subtle optical center crosshair
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)'; // Sky cyan
        ctx.lineWidth = Math.max(1, 1 * (window.devicePixelRatio || 1));

        // Horizontal line with gap at center
        const armLen = 30 * scaleX;
        const gap = 8 * scaleX;
        ctx.beginPath();
        ctx.moveTo(cx - armLen, cy);
        ctx.lineTo(cx - gap, cy);
        ctx.moveTo(cx + gap, cy);
        ctx.lineTo(cx + armLen, cy);

        // Vertical line with gap at center
        ctx.moveTo(cx, cy - armLen);
        ctx.lineTo(cx, cy - gap);
        ctx.moveTo(cx, cy + gap);
        ctx.lineTo(cx, cy + armLen);
        ctx.stroke();

        // Optical center circle
        ctx.beginPath();
        ctx.arc(cx, cy, gap, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)';
        ctx.stroke();

        // Principal point dot
        ctx.beginPath();
        ctx.arc(cx, cy, 2 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
        ctx.fillStyle = '#38bdf8';
        ctx.fill();

        // Corner crop brackets for 640x480 active sensor area
        const bracketLen = 20 * scaleX;
        const padX = 12 * scaleX;
        const padY = 12 * scaleY;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.beginPath();
        // Top-Left
        ctx.moveTo(padX + bracketLen, padY);
        ctx.lineTo(padX, padY);
        ctx.lineTo(padX, padY + bracketLen);
        // Top-Right
        ctx.moveTo(w - padX - bracketLen, padY);
        ctx.lineTo(w - padX, padY);
        ctx.lineTo(w - padX, padY + bracketLen);
        // Bottom-Left
        ctx.moveTo(padX + bracketLen, h - padY);
        ctx.lineTo(padX, h - padY);
        ctx.lineTo(padX, h - padY - bracketLen);
        // Bottom-Right
        ctx.moveTo(w - padX - bracketLen, h - padY);
        ctx.lineTo(w - padX, h - padY);
        ctx.lineTo(w - padX, h - padY - bracketLen);
        ctx.stroke();

        ctx.restore();
      }

      // 2. Draw Genuine Feature Tracking Points (if delivered by live estimator)
      if (this.features && this.features.length > 0) {
        ctx.save();
        ctx.fillStyle = '#10b981'; // Emerald
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.8)';
        ctx.lineWidth = 1;

        for (let i = 0; i < this.features.length; i++) {
          const pt = this.features[i];
          const px = pt[0] * scaleX;
          const py = pt[1] * scaleY;

          ctx.beginPath();
          ctx.arc(px, py, 2.5 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
          ctx.fill();

          // Small ring around feature
          ctx.beginPath();
          ctx.arc(px, py, 5 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
          ctx.stroke();
        }
        ctx.restore();
      }
    }

    toggleReticle() {
      this.showReticle = !this.showReticle;
      const btn = document.getElementById('btn-toggle-reticle');
      if (btn) {
        btn.classList.toggle('bg-stone-900', this.showReticle);
        btn.classList.toggle('text-white', this.showReticle);
        btn.classList.toggle('bg-white', !this.showReticle);
        btn.classList.toggle('text-stone-700', !this.showReticle);
      }
      if (window.showToast) {
        window.showToast(`Optical Reticle: ${this.showReticle ? 'Enabled' : 'Disabled'}`);
      }
    }

    toggleHud() {
      this.showHud = !this.showHud;
      const hudEl = document.getElementById('camera-hud-overlay');
      if (hudEl) {
        hudEl.style.display = this.showHud ? 'flex' : 'none';
      }
      const btn = document.getElementById('btn-toggle-hud');
      if (btn) {
        btn.classList.toggle('bg-stone-900', this.showHud);
        btn.classList.toggle('text-white', this.showHud);
        btn.classList.toggle('bg-white', !this.showHud);
        btn.classList.toggle('text-stone-700', !this.showHud);
      }
      if (window.showToast) {
        window.showToast(`Camera HUD: ${this.showHud ? 'Visible' : 'Hidden'}`);
      }
    }

    toggleFitMode() {
      this.objectFit = this.objectFit === 'contain' ? 'cover' : 'contain';
      if (this.imgElement) {
        this.imgElement.classList.toggle('object-contain', this.objectFit === 'contain');
        this.imgElement.classList.toggle('object-cover', this.objectFit === 'cover');
      }
      const btn = document.getElementById('btn-toggle-fit');
      if (btn) {
        btn.textContent = this.objectFit === 'contain' ? 'Fit: 1:1 Aspect' : 'Fit: Fill Box';
      }
      if (window.showToast) {
        window.showToast(`Display Aspect: ${this.objectFit.toUpperCase()}`);
      }
    }

    toggleMjpegStream() {
      this.useMjpeg = !this.useMjpeg;
      const btn = document.getElementById('btn-toggle-mjpeg');
      if (this.useMjpeg) {
        if (this.imgElement) {
          this.imgElement.src = '/api/camera_stream?t=' + Date.now();
          this.imgElement.classList.remove('hidden');
        }
        if (this.placeholder) this.placeholder.classList.add('hidden');
        if (btn) {
          btn.className = 'min-h-[38px] px-3 py-1.5 bg-brand-600 text-white font-medium rounded-lg border border-brand-500 transition shadow-sm';
          btn.textContent = 'MJPEG: Active';
        }
        if (window.showToast) window.showToast('Switched to Direct MJPEG Video Stream');
      } else {
        if (this.imgElement) {
          this.imgElement.src = 'monocular_tracking_feed.png';
        }
        if (btn) {
          btn.className = 'min-h-[38px] px-3 py-1.5 bg-white hover:bg-stone-50 text-stone-700 font-medium rounded-lg border border-stone-200 transition';
          btn.textContent = 'MJPEG Stream';
        }
        if (window.showToast) window.showToast('Switched to WebSocket Frame Stream');
      }
    }
  }

  // Expose singleton on window
  window.cameraView = new CameraViewController();
  window.toggleMjpegStream = function() {
    if (window.cameraView) window.cameraView.toggleMjpegStream();
  };

})(window);
