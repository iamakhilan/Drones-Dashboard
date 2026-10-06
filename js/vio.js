/**
 * ============================================================================
 * AeroVIO Ground Control Station - Visual-Inertial Odometry Subsystem
 * Manages VIO estimator metrics, MSCKF sliding window camera clones,
 * covariance tracking, and dynamic tracking quality calculation.
 * ============================================================================
 */

(function(window) {
  'use strict';

  class VioController {
    constructor() {
      this.elements = {
        statusBadge: null,
        imuRate: null,
        camRate: null,
        featureSummary: null,
        cloneSummary: null,
        covarianceTrace: null,
        scaleDrift: null,
        trackingQualityBar: null,
        trackingQualityText: null,
        accelBias: null,
        gyroBias: null
      };

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this._bindElements();

      // Hook Telemetry Engine
      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.update(state));
      }
    }

    _bindElements() {
      this.elements.statusBadge = document.getElementById('vio-status-badge');
      this.elements.imuRate = document.getElementById('vio-imu-rate');
      this.elements.camRate = document.getElementById('vio-cam-rate');
      this.elements.featureSummary = document.getElementById('vio-feature-summary');
      this.elements.cloneSummary = document.getElementById('vio-clone-summary');
      this.elements.covarianceTrace = document.getElementById('vio-cov-trace');
      this.elements.scaleDrift = document.getElementById('vio-scale-drift');
      this.elements.trackingQualityBar = document.getElementById('vio-quality-bar');
      this.elements.trackingQualityText = document.getElementById('vio-quality-text');
      this.elements.accelBias = document.getElementById('vio-accel-bias');
      this.elements.gyroBias = document.getElementById('vio-gyro-bias');
    }

    update(state) {
      const v = state.vio;

      // Status badge
      if (this.elements.statusBadge) {
        this.elements.statusBadge.textContent = v.status || 'MSCKF ACT';
      }

      // Rates
      if (this.elements.imuRate) {
        this.elements.imuRate.textContent = v.imu_rate_hz ? `${v.imu_rate_hz} HZ [BMI088]` : '200 HZ [BMI088]';
      }
      if (this.elements.camRate) {
        this.elements.camRate.textContent = v.update_rate_hz ? `${v.update_rate_hz} FPS [18.2 ms]` : '30 FPS [18.2 ms]';
      }

      // Features
      if (this.elements.featureSummary) {
        const act = v.active_features !== null ? v.active_features : 128;
        const trk = v.tracked_features !== null ? v.tracked_features : 116;
        this.elements.featureSummary.textContent = `${act} (${trk} TRK)`;
      }

      // Clones
      if (this.elements.cloneSummary) {
        this.elements.cloneSummary.textContent = `${v.clones || 10} CLONES`;
      }

      // Covariance trace & scale drift
      if (this.elements.covarianceTrace) {
        this.elements.covarianceTrace.textContent = v.covariance_trace ? v.covariance_trace.toExponential(2) : '1.42e-4';
      }
      if (this.elements.scaleDrift) {
        this.elements.scaleDrift.textContent = v.scale_drift ? `${v.scale_drift} m/m (0.18%)` : '0.018 m/m (0.18%)';
      }

      // Tracking quality percentage calculation
      if (v.tracking_quality !== null) {
        const q = Math.max(0, Math.min(100, v.tracking_quality));
        if (this.elements.trackingQualityText) {
          this.elements.trackingQualityText.textContent = `${q}%`;
        }
        if (this.elements.trackingQualityBar) {
          this.elements.trackingQualityBar.style.width = `${q}%`;
        }
      }

      // Biases in modal
      if (this.elements.accelBias && v.accel_bias) {
        this.elements.accelBias.textContent = `[${v.accel_bias.map(b => (b >= 0 ? '+' : '') + b.toFixed(3)).join(', ')}] m/s²`;
      }
      if (this.elements.gyroBias && v.gyro_bias) {
        this.elements.gyroBias.textContent = `[${v.gyro_bias.map(b => (b >= 0 ? '+' : '') + b.toFixed(4)).join(', ')}] rad/s`;
      }
    }

    /**
     * Trigger MSCKF keyframe marginalization
     */
    forceMarginalization() {
      console.log('[VIO Engine] Requesting Keyframe Marginalization...');
      if (window.wsBridge) {
        window.wsBridge.sendCommand({ action: 'marginalize_keyframe' });
      }
      if (window.ui) {
        window.ui.showToast('MSCKF: Keyframe Marginalization Triggered', 'info');
      }
    }
  }

  // Expose singleton on window
  window.vio = new VioController();

})(window);
