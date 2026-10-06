/**
 * ============================================================================
 * AeroVIO Ground Control Station - Precision Kinematic Waveform Oscilloscope
 * Renders 3 real-time rolling 20-second window charts:
 * 1. Linear Acceleration (IMU / m/s²) - Ax, Ay, Az + 1g Reference
 * 2. Linear Velocity (m/s) - Vx, Vy, Vz
 * 3. World Position (m) - X, Y, Z Altitude
 *
 * Features:
 * - Calibrated Cartesian grid with explicit Y-axis and X-axis ticks
 * - Semi-transparent gradient area fills and antialiased spline curves
 * - Interactive Channel Visibility Toggles (Ax, Ay, Az / Vx, Vy, Vz / X, Y, Z)
 * - Leading-edge live pulse dots
 * - Interactive mouse hover crosshair & instantaneous tooltip inspection
 * - LIVE / PAUSE / CLEAR buffer controls
 * ============================================================================
 */

(function(window) {
  'use strict';

  class KinematicGraphEngine {
    constructor() {
      this.isPaused = false;
      this.windowDurationSec = 20.0;
      this.maxPoints = 250; // Rolling buffer size (~12.5 Hz sampling over 20s)

      // Time series buffers: { t, x, y, z }
      this.buffers = {
        accel: [], // ax, ay, az
        vel: [],   // vx, vy, vz
        pos: []    // x, y, z
      };

      // Channel Visibility State
      this.channelVisibility = {
        accel: { x: true, y: true, z: true },
        vel: { x: true, y: true, z: true },
        pos: { x: true, y: true, z: true }
      };

      // Hover Inspection State
      this.hoverState = {
        activeChart: null,
        cursorX: null,
        hoverIndex: null
      };

      // Chart Configs & Fixed Engineering Ranges
      this.chartConfigs = {
        accel: {
          svgId: 'svg-graph-accel',
          yMin: -4.0,
          yMax: 14.0,
          unit: 'm/s²',
          colors: { x: '#10B981', y: '#0EA5E9', z: '#F59E0B' },
          fillGradients: { x: 'accelGradX', y: 'accelGradY', z: 'accelGradZ' }
        },
        vel: {
          svgId: 'svg-graph-vel',
          yMin: -3.0,
          yMax: 3.0,
          unit: 'm/s',
          colors: { x: '#10B981', y: '#0EA5E9', z: '#A855F7' },
          fillGradients: { x: 'velGradX', y: 'velGradY', z: 'velGradZ' }
        },
        pos: {
          svgId: 'svg-graph-pos',
          yMin: -10.0,
          yMax: 25.0,
          unit: 'm',
          colors: { x: '#10B981', y: '#0EA5E9', z: '#14B8A6' },
          fillGradients: { x: 'posGradX', y: 'posGradY', z: 'posGradZ' }
        }
      };

      this.animFrameId = null;
      this.lastRenderTime = 0;

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this._bindHoverEvents();

      // Subscribe to telemetry engine if available
      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.pushTelemetry(state));
      }

      this._startRenderLoop();
    }

    _bindHoverEvents() {
      ['accel', 'vel', 'pos'].forEach(chartKey => {
        const svg = document.getElementById(this.chartConfigs[chartKey].svgId);
        if (!svg) return;

        svg.addEventListener('mousemove', (e) => {
          const rect = svg.getBoundingClientRect();
          const svgX = ((e.clientX - rect.left) / rect.width) * 400;
          this.hoverState.activeChart = chartKey;
          this.hoverState.cursorX = Math.max(45, Math.min(390, svgX));
        });

        svg.addEventListener('mouseleave', () => {
          if (this.hoverState.activeChart === chartKey) {
            this.hoverState.activeChart = null;
            this.hoverState.cursorX = null;
          }
        });
      });
    }

    /**
     * Push incoming normalized telemetry frame into rolling buffers
     */
    pushTelemetry(state) {
      if (this.isPaused) return;

      const t = (state.timestamp || Date.now()) / 1000.0;

      // 1. Acceleration
      if (state.acceleration && state.acceleration.ax !== null) {
        this.buffers.accel.push({
          t: t,
          x: state.acceleration.ax,
          y: state.acceleration.ay,
          z: state.acceleration.az
        });
        if (this.buffers.accel.length > this.maxPoints) this.buffers.accel.shift();
      }

      // 2. Velocity
      if (state.velocity && state.velocity.vx !== null) {
        this.buffers.vel.push({
          t: t,
          x: state.velocity.vx,
          y: state.velocity.vy,
          z: state.velocity.vz
        });
        if (this.buffers.vel.length > this.maxPoints) this.buffers.vel.shift();
      }

      // 3. Position
      if (state.position && state.position.x !== null) {
        this.buffers.pos.push({
          t: t,
          x: state.position.x,
          y: state.position.y,
          z: state.position.z
        });
        if (this.buffers.pos.length > this.maxPoints) this.buffers.pos.shift();
      }

      this._updateValueBadges(state);
    }

    /**
     * Update numerical footer badges
     */
    _updateValueBadges(state) {
      // Acceleration
      if (state.acceleration) {
        const ax = state.acceleration.ax;
        const ay = state.acceleration.ay;
        const az = state.acceleration.az;
        const aNorm = Math.sqrt((ax||0)**2 + (ay||0)**2 + (az||0)**2);
        this._setText('graph-accel-val-x', `AX: ${(ax>=0?'+':'')+(ax||0).toFixed(2)} m/s²`);
        this._setText('graph-accel-val-y', `AY: ${(ay>=0?'+':'')+(ay||0).toFixed(2)} m/s²`);
        this._setText('graph-accel-val-z', `AZ: ${(az||0).toFixed(2)} m/s²`);
        this._setText('graph-accel-val-norm', `‖A‖: ${aNorm.toFixed(2)} m/s²`);
      }

      // Velocity
      if (state.velocity) {
        const vx = state.velocity.vx;
        const vy = state.velocity.vy;
        const vz = state.velocity.vz;
        const vNorm = Math.sqrt((vx||0)**2 + (vy||0)**2 + (vz||0)**2);
        this._setText('graph-vel-val-x', `VX: ${(vx>=0?'+':'')+(vx||0).toFixed(2)} m/s`);
        this._setText('graph-vel-val-y', `VY: ${(vy>=0?'+':'')+(vy||0).toFixed(2)} m/s`);
        this._setText('graph-vel-val-z', `VZ: ${(vz>=0?'+':'')+(vz||0).toFixed(2)} m/s`);
        this._setText('graph-vel-val-norm', `‖V‖: ${vNorm.toFixed(2)} m/s`);
      }

      // Position
      if (state.position) {
        const px = state.position.x;
        const py = state.position.y;
        const pz = state.position.z;
        const pNorm = Math.sqrt((px||0)**2 + (py||0)**2 + (pz||0)**2);
        this._setText('graph-pos-val-x', `X: ${(px>=0?'+':'')+(px||0).toFixed(2)} m`);
        this._setText('graph-pos-val-y', `Y: ${(py>=0?'+':'')+(py||0).toFixed(2)} m`);
        this._setText('graph-pos-val-z', `ALT(Z): ${(pz||0).toFixed(2)} m`);
        this._setText('graph-pos-val-norm', `‖P‖: ${pNorm.toFixed(2)} m`);
      }
    }

    _setText(id, text) {
      const el = document.getElementById(id);
      if (el) el.textContent = text;
    }

    /**
     * Toggle individual channel (X, Y, Z) visibility
     */
    toggleChannel(chartKey, axis) {
      if (!this.channelVisibility[chartKey]) return;
      this.channelVisibility[chartKey][axis] = !this.channelVisibility[chartKey][axis];

      const btnId = `btn-chip-${chartKey}-${axis}`;
      const btn = document.getElementById(btnId);
      if (btn) {
        const isVis = this.channelVisibility[chartKey][axis];
        btn.classList.toggle('opacity-40', !isVis);
        btn.classList.toggle('line-through', !isVis);
      }
      this.render();
    }

    /**
     * Pause / Resume controls
     */
    togglePause() {
      this.isPaused = !this.isPaused;
      const btn = document.getElementById('graphs-pause-btn');
      if (btn) {
        btn.textContent = this.isPaused ? 'RESUME' : 'PAUSE';
        btn.classList.toggle('bg-amber-100', this.isPaused);
        btn.classList.toggle('text-amber-800', this.isPaused);
      }
    }

    clearBuffers() {
      this.buffers.accel = [];
      this.buffers.vel = [];
      this.buffers.pos = [];
      this.render();
    }

    _startRenderLoop() {
      const loop = (timestamp) => {
        if (timestamp - this.lastRenderTime >= 25) { // ~40 FPS render limit
          this.render();
          this.lastRenderTime = timestamp;
        }
        this.animFrameId = requestAnimationFrame(loop);
      };
      this.animFrameId = requestAnimationFrame(loop);
    }

    /**
     * Primary Render Pipeline
     */
    render() {
      this._renderChart('accel');
      this._renderChart('vel');
      this._renderChart('pos');
    }

    _renderChart(chartKey) {
      const cfg = this.chartConfigs[chartKey];
      const data = this.buffers[chartKey];
      const vis = this.channelVisibility[chartKey];

      const plotLeft = 45;
      const plotRight = 390;
      const plotTop = 10;
      const plotBottom = 135;
      const plotWidth = plotRight - plotLeft;
      const plotHeight = plotBottom - plotTop;

      const yMin = cfg.yMin;
      const yMax = cfg.yMax;
      const yRange = (yMax - yMin) || 1.0;

      // Coordinate converter helper
      const toSvgCoords = (index, total, val) => {
        const x = plotLeft + (index / Math.max(1, total - 1)) * plotWidth;
        const normY = (val - yMin) / yRange;
        const clampedNormY = Math.max(0, Math.min(1, normY));
        const y = plotBottom - (clampedNormY * plotHeight);
        return { x, y };
      };

      const axes = ['x', 'y', 'z'];
      axes.forEach(axis => {
        const lineEl = document.getElementById(`graph-${chartKey}-${axis}`);
        const areaEl = document.getElementById(`graph-${chartKey}-area-${axis}`);
        const dotEl = document.getElementById(`graph-${chartKey}-dot-${axis}`);

        if (!vis[axis] || !data || data.length < 2) {
          if (lineEl) lineEl.setAttribute('d', '');
          if (areaEl) areaEl.setAttribute('d', '');
          if (dotEl) dotEl.setAttribute('opacity', '0');
          return;
        }

        let linePath = '';
        let lastPt = null;

        for (let i = 0; i < data.length; i++) {
          const pt = toSvgCoords(i, data.length, data[i][axis]);
          linePath += (i === 0 ? `M ${pt.x.toFixed(1)} ${pt.y.toFixed(1)}` : ` L ${pt.x.toFixed(1)} ${pt.y.toFixed(1)}`);
          if (i === data.length - 1) lastPt = pt;
        }

        if (lineEl) lineEl.setAttribute('d', linePath);

        // Area fill under curve
        if (areaEl && data.length > 1) {
          const firstPt = toSvgCoords(0, data.length, data[0][axis]);
          const zeroY = plotBottom - ((0 - yMin) / yRange) * plotHeight;
          const clampedZeroY = Math.max(plotTop, Math.min(plotBottom, zeroY));
          const areaPath = `${linePath} L ${lastPt.x.toFixed(1)} ${clampedZeroY.toFixed(1)} L ${firstPt.x.toFixed(1)} ${clampedZeroY.toFixed(1)} Z`;
          areaEl.setAttribute('d', areaPath);
        }

        // Leading edge live pulse dot
        if (dotEl && lastPt) {
          dotEl.setAttribute('cx', lastPt.x.toFixed(1));
          dotEl.setAttribute('cy', lastPt.y.toFixed(1));
          dotEl.setAttribute('opacity', '1');
        }
      });

      // Render Hover Inspection Crosshair
      const crosshair = document.getElementById(`graph-${chartKey}-crosshair`);
      const tooltip = document.getElementById(`graph-${chartKey}-tooltip`);
      const tooltipText = document.getElementById(`graph-${chartKey}-tooltip-text`);

      if (this.hoverState.activeChart === chartKey && this.hoverState.cursorX && data.length > 1) {
        const cursorRatio = (this.hoverState.cursorX - plotLeft) / plotWidth;
        const sampleIdx = Math.max(0, Math.min(data.length - 1, Math.round(cursorRatio * (data.length - 1))));
        const sample = data[sampleIdx];

        if (crosshair) {
          crosshair.setAttribute('x1', this.hoverState.cursorX.toFixed(1));
          crosshair.setAttribute('x2', this.hoverState.cursorX.toFixed(1));
          crosshair.setAttribute('opacity', '1');
        }

        if (tooltip && tooltipText && sample) {
          const tRel = ((sampleIdx - (data.length - 1)) * (20.0 / data.length)).toFixed(1);
          tooltipText.textContent = `T${tRel}s: [${sample.x.toFixed(2)}, ${sample.y.toFixed(2)}, ${sample.z.toFixed(2)}]`;
          const ttX = Math.min(260, Math.max(50, this.hoverState.cursorX - 60));
          tooltip.setAttribute('transform', `translate(${ttX}, 18)`);
          tooltip.setAttribute('opacity', '1');
        }
      } else {
        if (crosshair) crosshair.setAttribute('opacity', '0');
        if (tooltip) tooltip.setAttribute('opacity', '0');
      }
    }
  }

  // Expose singleton on window
  window.graphs = new KinematicGraphEngine();

})(window);
