/**
 * ============================================================================
 * AeroVIO Ground Control Station - Live Oscilloscope Graphs
 * Renders 3 real-time rolling 20-second window charts:
 * 1. Acceleration (IMU / m/s²) - Ax, Ay, Az + 1G Reference
 * 2. Odometry (Velocity / m/s) - Vx, Vy, Vz
 * 3. Position (World Frame / m) - X, Y, Z
 * Supports LIVE / PAUSE / CLEAR controls.
 * ============================================================================
 */

(function(window) {
  'use strict';

  class GraphController {
    constructor() {
      this.isPaused = false;
      this.windowDurationSec = 20.0;
      this.maxPoints = 400; // Rolling buffer size

      // Data buffers
      this.accelData = []; // { t, ax, ay, az }
      this.velData = [];   // { t, vx, vy, vz }
      this.posData = [];   // { t, x, y, z }

      // Elements
      this.elements = {
        // SVG paths
        accelPathX: null,
        accelPathY: null,
        accelPathZ: null,
        accelValX: null,
        accelValY: null,
        accelValZ: null,
        accelWaiting: null,

        velPathX: null,
        velPathY: null,
        velPathZ: null,
        velValX: null,
        velValY: null,
        velValZ: null,
        velWaiting: null,

        posPathX: null,
        posPathY: null,
        posPathZ: null,
        posValX: null,
        posValY: null,
        posValZ: null,
        posWaiting: null
      };

      // Animation frame handle
      this.animFrameId = null;
      this.lastRenderTime = 0;

      // Initialize on DOM ready
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this._bindElements();
      
      // Subscribe to telemetry engine
      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.pushTelemetry(state));
      }

      // Start render loop
      this._startRenderLoop();
    }

    _bindElements() {
      this.elements.accelPathX = document.getElementById('graph-accel-x');
      this.elements.accelPathY = document.getElementById('graph-accel-y');
      this.elements.accelPathZ = document.getElementById('graph-accel-z');
      this.elements.accelValX = document.getElementById('graph-accel-val-x');
      this.elements.accelValY = document.getElementById('graph-accel-val-y');
      this.elements.accelValZ = document.getElementById('graph-accel-val-z');
      this.elements.accelWaiting = document.getElementById('graph-accel-waiting');

      this.elements.velPathX = document.getElementById('graph-vel-x');
      this.elements.velPathY = document.getElementById('graph-vel-y');
      this.elements.velPathZ = document.getElementById('graph-vel-z');
      this.elements.velValX = document.getElementById('graph-vel-val-x');
      this.elements.velValY = document.getElementById('graph-vel-val-y');
      this.elements.velValZ = document.getElementById('graph-vel-val-z');
      this.elements.velWaiting = document.getElementById('graph-vel-waiting');

      this.elements.posPathX = document.getElementById('graph-pos-x');
      this.elements.posPathY = document.getElementById('graph-pos-y');
      this.elements.posPathZ = document.getElementById('graph-pos-z');
      this.elements.posValX = document.getElementById('graph-pos-val-x');
      this.elements.posValY = document.getElementById('graph-pos-val-y');
      this.elements.posValZ = document.getElementById('graph-pos-val-z');
      this.elements.posWaiting = document.getElementById('graph-pos-waiting');
    }

    /**
     * Push incoming normalized state into buffers
     */
    pushTelemetry(state) {
      if (this.isPaused) return;

      const t = state.timestamp || Date.now();

      // Accel
      if (state.acceleration.ax !== null) {
        this.accelData.push({
          t: t,
          ax: state.acceleration.ax,
          ay: state.acceleration.ay,
          az: state.acceleration.az
        });
        if (this.accelData.length > this.maxPoints) this.accelData.shift();
      }

      // Vel
      if (state.velocity.vx !== null) {
        this.velData.push({
          t: t,
          vx: state.velocity.vx,
          vy: state.velocity.vy,
          vz: state.velocity.vz
        });
        if (this.velData.length > this.maxPoints) this.velData.shift();
      }

      // Pos
      if (state.position.x !== null) {
        this.posData.push({
          t: t,
          x: state.position.x,
          y: state.position.y,
          z: state.position.z
        });
        if (this.posData.length > this.maxPoints) this.posData.shift();
      }

      // Update text values
      this._updateNumericReadouts(state);
    }

    _updateNumericReadouts(state) {
      // Accel
      if (this.elements.accelValX) {
        this.elements.accelValX.textContent = state.acceleration.ax !== null ? `AX: ${(state.acceleration.ax >= 0 ? '+' : '') + state.acceleration.ax.toFixed(2)} m/s²` : 'AX: --';
      }
      if (this.elements.accelValY) {
        this.elements.accelValY.textContent = state.acceleration.ay !== null ? `AY: ${(state.acceleration.ay >= 0 ? '+' : '') + state.acceleration.ay.toFixed(2)} m/s²` : 'AY: --';
      }
      if (this.elements.accelValZ) {
        this.elements.accelValZ.textContent = state.acceleration.az !== null ? `AZ: ${state.acceleration.az.toFixed(2)} m/s²` : 'AZ: --';
      }

      // Vel
      if (this.elements.velValX) {
        this.elements.velValX.textContent = state.velocity.vx !== null ? `VX: ${(state.velocity.vx >= 0 ? '+' : '') + state.velocity.vx.toFixed(2)} m/s` : 'VX: --';
      }
      if (this.elements.velValY) {
        this.elements.velValY.textContent = state.velocity.vy !== null ? `VY: ${(state.velocity.vy >= 0 ? '+' : '') + state.velocity.vy.toFixed(2)} m/s` : 'VY: --';
      }
      if (this.elements.velValZ) {
        this.elements.velValZ.textContent = state.velocity.vz !== null ? `VZ: ${(state.velocity.vz >= 0 ? '+' : '') + state.velocity.vz.toFixed(2)} m/s` : 'VZ: --';
      }

      // Pos
      if (this.elements.posValX) {
        this.elements.posValX.textContent = state.position.x !== null ? `X: ${(state.position.x >= 0 ? '+' : '') + state.position.x.toFixed(2)}m` : 'X: --';
      }
      if (this.elements.posValY) {
        this.elements.posValY.textContent = state.position.y !== null ? `Y: ${(state.position.y >= 0 ? '+' : '') + state.position.y.toFixed(2)}m` : 'Y: --';
      }
      if (this.elements.posValZ) {
        this.elements.posValZ.textContent = state.position.z !== null ? `ALT(Z): ${state.position.z.toFixed(2)}m` : 'ALT(Z): --';
      }
    }

    /**
     * Controls
     */
    togglePause() {
      this.isPaused = !this.isPaused;
      const pauseBtn = document.getElementById('graphs-pause-btn');
      if (pauseBtn) {
        pauseBtn.innerHTML = this.isPaused 
          ? '<span class="material-symbols-outlined text-[14px]">play_arrow</span> RESUME'
          : '<span class="material-symbols-outlined text-[14px]">pause</span> PAUSE';
        pauseBtn.classList.toggle('bg-primary', this.isPaused);
        pauseBtn.classList.toggle('text-on-primary', this.isPaused);
      }
    }

    clearBuffers() {
      this.accelData = [];
      this.velData = [];
      this.posData = [];
      this.render();
    }

    _startRenderLoop() {
      const renderTick = (time) => {
        // Limit rendering to ~30 FPS for optimal UI responsiveness
        if (time - this.lastRenderTime >= 30) {
          this.render();
          this.lastRenderTime = time;
        }
        this.animFrameId = requestAnimationFrame(renderTick);
      };
      this.animFrameId = requestAnimationFrame(renderTick);
    }

    /**
     * Generate SVG path string from series
     */
    _buildSvgPath(data, field, minY, maxY, width = 300, height = 100) {
      if (!data || data.length < 2) return '';
      
      const count = data.length;
      const rangeY = (maxY - minY) || 1.0;
      let d = '';

      for (let i = 0; i < count; i++) {
        const px = (i / (count - 1)) * width;
        const val = data[i][field];
        const normY = (val - minY) / rangeY;
        const py = height - (Math.max(0, Math.min(1, normY)) * height);

        if (i === 0) {
          d += `M ${px.toFixed(1)},${py.toFixed(1)}`;
        } else {
          d += ` L ${px.toFixed(1)},${py.toFixed(1)}`;
        }
      }

      return d;
    }

    /**
     * Render all three graphs
     */
    render() {
      // 1. Acceleration Graph
      if (this.accelData.length > 1) {
        if (this.elements.accelWaiting) this.elements.accelWaiting.classList.add('hidden');
        // Min/Max for Accel (centered around 0 for X, Y and 9.8 for Z)
        const pathX = this._buildSvgPath(this.accelData, 'ax', -2.5, 2.5);
        const pathY = this._buildSvgPath(this.accelData, 'ay', -2.5, 2.5);
        const pathZ = this._buildSvgPath(this.accelData, 'az', 5.0, 15.0);

        if (this.elements.accelPathX) this.elements.accelPathX.setAttribute('d', pathX);
        if (this.elements.accelPathY) this.elements.accelPathY.setAttribute('d', pathY);
        if (this.elements.accelPathZ) this.elements.accelPathZ.setAttribute('d', pathZ);
      } else {
        if (this.elements.accelWaiting) this.elements.accelWaiting.classList.remove('hidden');
      }

      // 2. Velocity Graph
      if (this.velData.length > 1) {
        if (this.elements.velWaiting) this.elements.velWaiting.classList.add('hidden');
        let minV = -1.5, maxV = 1.5;
        this.velData.forEach(d => {
          minV = Math.min(minV, d.vx, d.vy, d.vz);
          maxV = Math.max(maxV, d.vx, d.vy, d.vz);
        });
        const pathX = this._buildSvgPath(this.velData, 'vx', minV - 0.2, maxV + 0.2);
        const pathY = this._buildSvgPath(this.velData, 'vy', minV - 0.2, maxV + 0.2);
        const pathZ = this._buildSvgPath(this.velData, 'vz', minV - 0.2, maxV + 0.2);

        if (this.elements.velPathX) this.elements.velPathX.setAttribute('d', pathX);
        if (this.elements.velPathY) this.elements.velPathY.setAttribute('d', pathY);
        if (this.elements.velPathZ) this.elements.velPathZ.setAttribute('d', pathZ);
      } else {
        if (this.elements.velWaiting) this.elements.velWaiting.classList.remove('hidden');
      }

      // 3. Position Graph
      if (this.posData.length > 1) {
        if (this.elements.posWaiting) this.elements.posWaiting.classList.add('hidden');
        let minP = -5, maxP = 25;
        this.posData.forEach(d => {
          minP = Math.min(minP, d.x, d.y, d.z);
          maxP = Math.max(maxP, d.x, d.y, d.z);
        });
        const pathX = this._buildSvgPath(this.posData, 'x', minP - 1, maxP + 1);
        const pathY = this._buildSvgPath(this.posData, 'y', minP - 1, maxP + 1);
        const pathZ = this._buildSvgPath(this.posData, 'z', minP - 1, maxP + 1);

        if (this.elements.posPathX) this.elements.posPathX.setAttribute('d', pathX);
        if (this.elements.posPathY) this.elements.posPathY.setAttribute('d', pathY);
        if (this.elements.posPathZ) this.elements.posPathZ.setAttribute('d', pathZ);
      } else {
        if (this.elements.posWaiting) this.elements.posWaiting.classList.remove('hidden');
      }
    }
  }

  // Expose singleton on window
  window.graphs = new GraphController();

})(window);
