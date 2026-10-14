/**
 * ============================================================================
 * AeroVIO — VIO Health Module
 * Covariance-trace sparkline (canvas, threshold line), features tracked/active,
 * tracking quality %, update rate, and tracking-loss transition counter
 * (rising transitions of state.vio.status into 'TRACKING_LOST').
 * N/A graceful fallbacks when fields are null.
 * ============================================================================
 */
!function(window){
  'use strict';

  var HISTORY_MAX = 300;
  var covHistory = [];           // numbers (non-null samples)
  var trackingLossCount = 0;
  var prevStatus = null;
  var lastRender = 0;

  function _threshold(){
    var t = (window.APP_CONFIG && window.APP_CONFIG.thresholds) || {};
    return (typeof t.covTrace === 'number' && isFinite(t.covTrace)) ? t.covTrace : 1e-3;
  }

  function _setText(id, txt){
    var el = document.getElementById(id);
    if (el) el.textContent = txt;
  }

  function _fmt(v, digits, suffix){
    if (v === null || v === undefined || !isFinite(v)) return 'N/A';
    return (+v).toFixed(digits) + (suffix || '');
  }

  function _fmtSci(v){
    if (v === null || v === undefined || !isFinite(v)) return 'N/A';
    return v.toExponential(2);
  }

  function _deriveQuality(vio){
    if (!vio) return null;
    if (typeof vio.tracking_quality === 'number' && isFinite(vio.tracking_quality)) return vio.tracking_quality;
    var a = vio.active_features, t = vio.tracked_features;
    if (typeof a === 'number' && a > 0 && typeof t === 'number') return (t / a) * 100;
    return null;
  }

  /* Exponential y-scale mapping for sparkline (values span decades) */
  function _drawSparkline(){
    var canvas = document.getElementById('vio-cov-canvas');
    if (!canvas) return;
    var dpr = window.devicePixelRatio || 1;
    var w = canvas.clientWidth || 300, h = canvas.clientHeight || 80;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr; canvas.height = h * dpr;
    }
    var ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    var th = _threshold();
    var pts = covHistory;
    // log-domain range
    var logs = pts.map(function(v){ return Math.log10(Math.max(v, 1e-12)); });
    var logTh = Math.log10(Math.max(th, 1e-12));
    var lo = Math.min(logTh, logs.length ? Math.min.apply(null, logs) : logTh);
    var hi = Math.max(logTh, logs.length ? Math.max.apply(null, logs) : logTh);
    if (hi - lo < 1e-6) { hi = lo + 1; }
    var pad = 0.12 * (hi - lo);
    lo -= pad; hi += pad;

    function yOf(v){ return h - 6 - ((Math.log10(Math.max(v, 1e-12)) - lo) / (hi - lo)) * (h - 16); }

    // threshold line (amber dashed)
    ctx.strokeStyle = '#D97706';
    ctx.setLineDash([4, 3]);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, yOf(th)); ctx.lineTo(w, yOf(th));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#B45309';
    ctx.font = '9px monospace';
    ctx.fillText('thr ' + th.toExponential(1), 4, yOf(th) - 3);

    if (pts.length < 2) {
      ctx.fillStyle = '#A8A29E';
      ctx.font = '10px monospace';
      ctx.fillText(pts.length ? 'collecting…' : 'no covariance samples (N/A)', 8, h / 2);
      return;
    }
    // trace (light-theme friendly brand/stone stroke; red beyond threshold)
    for (var i = 1; i < pts.length; i++) {
      var x0 = (i - 1) / (HISTORY_MAX - 1) * w;
      var x1 = i / (HISTORY_MAX - 1) * w;
      ctx.strokeStyle = (pts[i] > th) ? '#EF4444' : '#57534E';
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      ctx.moveTo(x0, yOf(pts[i - 1]));
      ctx.lineTo(x1, yOf(pts[i]));
      ctx.stroke();
    }
  }

  function render(state){
    var vio = (state && state.vio) || {};
    _setText('vio-h-status', vio.status || 'N/A');
    var af = vio.active_features, tf = vio.tracked_features;
    _setText('vio-h-features',
      (typeof tf === 'number' ? tf : 'N/A') + ' / ' + (typeof af === 'number' ? af : 'N/A'));
    _setText('vio-h-quality', _fmt(_deriveQuality(vio), 1, ' %'));
    _setText('vio-h-rate', _fmt(vio.update_rate_hz, 1, ' Hz'));
    _setText('vio-h-cov', _fmtSci(vio.covariance_trace));
    _setText('vio-h-loss-count', String(trackingLossCount));

    var statusEl = document.getElementById('vio-h-status');
    if (statusEl) {
      var s = vio.status || '';
      var cls = 'font-mono font-bold ';
      if (s.indexOf('TRACKING_LOST') === 0) cls += 'text-red-600';
      else if (s) cls += 'text-emerald-700';
      else cls += 'text-stone-400';
      statusEl.className = cls;
    }
    _drawSparkline();
  }

  function ingestState(state){
    if (!state || !state.vio) return;
    var vio = state.vio;
    // tracking-loss transition counter
    var s = vio.status || null;
    if (s && prevStatus && prevStatus !== 'TRACKING_LOST' && s === 'TRACKING_LOST') {
      trackingLossCount++;
    }
    if (s) prevStatus = s;

    if (typeof vio.covariance_trace === 'number' && isFinite(vio.covariance_trace)) {
      covHistory.push(vio.covariance_trace);
      if (covHistory.length > HISTORY_MAX) covHistory.shift();
    }
    var now = performance.now();
    if (now - lastRender > 200) { lastRender = now; render(state); }
  }

  /* DEMO frames carry no VIO fields — render N/A placeholders once, no errors. */
  function ingestDemoFrame(){ render(window.telemetry ? window.telemetry.state : null); }

  function reset(){
    covHistory = [];
    trackingLossCount = 0;
    prevStatus = null;
    render(window.telemetry ? window.telemetry.state : null);
  }

  function init(){
    if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
      window.telemetry.onTelemetry(ingestState);
    }
    render(window.telemetry ? window.telemetry.state : null);
    window.addEventListener('resize', _drawSparkline);
    var btn = document.getElementById('vio-h-reset-btn');
    if (btn) btn.addEventListener('click', reset);
  }

  window.AeroVIOVioHealth = {
    init: init,
    ingestState: ingestState,
    ingestDemoFrame: ingestDemoFrame,
    render: render,
    reset: reset,
    getTrackingLossCount: function(){ return trackingLossCount; }
  };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
}(window);
