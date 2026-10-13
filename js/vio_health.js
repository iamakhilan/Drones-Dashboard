/**
 * AeroVIO — VIO Health Panel
 * Covariance-trace trend sparkline, feature counts, keyframe rate,
 * and a tracking-loss event counter fed by the telemetry engine.
 * Degrades gracefully (shows N/A) when optional fields are absent.
 * Vanilla JS IIFE, no build step.
 */
!function(window){
  'use strict';

  var MAX_TREND = 60;          // cov-trace samples in the sparkline
  var RENDER_MS = 500;         // DOM update throttle
  var _trend = [];             // recent covariance_trace values
  var _trackingLossCount = 0;
  var _prevStatus = null;
  var _lastRender = 0;
  var _lastState = null;

  function _fmt(v, digits, suffix){
    if (v == null || !isFinite(v)) return 'N/A';
    return (+v).toFixed(digits != null ? digits : 2) + (suffix || '');
  }

  function _set(id, txt){
    var el = document.getElementById(id);
    if (el) el.textContent = txt;
  }

  function _drawSparkline(){
    var svg = document.getElementById('vio-cov-spark');
    if (!svg) return;
    while (svg.firstChild) svg.removeChild(svg.firstChild);
    if (_trend.length < 2) {
      var t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      t.setAttribute('x', '200'); t.setAttribute('y', '45');
      t.setAttribute('fill', '#94A3B8'); t.setAttribute('font-size', '11');
      t.setAttribute('font-family', 'monospace'); t.setAttribute('text-anchor', 'middle');
      t.textContent = 'N/A — no covariance data';
      svg.appendChild(t);
      return;
    }
    var th = (window.APP_CONFIG && window.APP_CONFIG.thresholds) || { covTrace: 1e-3 };
    var W = 400, H = 80, P = 6;
    var maxV = th.covTrace * 1.6;
    _trend.forEach(function(v){ if (v > maxV) maxV = v; });
    function pt(i, v){
      return { x: P + (i / (MAX_TREND - 1)) * (W - 2 * P),
               y: (H - P) - (v / maxV) * (H - 2 * P) };
    }
    // threshold line
    var thY = (H - P) - (th.covTrace / maxV) * (H - 2 * P);
    var line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('x1', P); line.setAttribute('x2', W - P);
    line.setAttribute('y1', thY); line.setAttribute('y2', thY);
    line.setAttribute('stroke', '#EF4444'); line.setAttribute('stroke-dasharray', '4 3');
    line.setAttribute('stroke-width', '1');
    svg.appendChild(line);
    var d = _trend.map(function(v, i){
      var p = pt(i, v);
      return (i === 0 ? 'M ' : 'L ') + p.x.toFixed(1) + ' ' + p.y.toFixed(1);
    }).join(' ');
    var path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', d);
    path.setAttribute('fill', 'none');
    path.setAttribute('stroke', '#10B981');
    path.setAttribute('stroke-width', '1.8');
    svg.appendChild(path);
  }

  function _render(){
    var s = _lastState;
    if (!s) return;
    var vio = s.vio || {};
    _set('vio-health-status', vio.status || 'N/A');
    _set('vio-health-cov',
      (vio.covariance_trace != null && isFinite(vio.covariance_trace))
        ? vio.covariance_trace.toExponential(2)
        : 'N/A');
    _set('vio-health-features',
      (vio.tracked_features != null && vio.active_features != null)
        ? (vio.tracked_features + ' / ' + vio.active_features)
        : 'N/A');
    _set('vio-health-quality', vio.tracking_quality != null ? (vio.tracking_quality + '%') : 'N/A');
    _set('vio-health-keyframe', _fmt(vio.keyframe_rate_hz, 1, ' Hz'));
    _set('vio-health-rate', _fmt(vio.update_rate_hz, 1, ' Hz'));
    _set('vio-health-tracking-loss', String(_trackingLossCount));
    _drawSparkline();
  }

  function _onTelemetry(state){
    _lastState = state;
    var vio = (state && state.vio) || {};

    if (vio.covariance_trace != null && isFinite(vio.covariance_trace)) {
      _trend.push(vio.covariance_trace);
      if (_trend.length > MAX_TREND) _trend.shift();
    }
    var cur = vio.status || null;
    if (cur && _prevStatus && cur !== _prevStatus && /TRACKING_LOST/i.test(cur)) {
      _trackingLossCount++;
    }
    if (cur) _prevStatus = cur;

    var now = Date.now();
    if (now - _lastRender >= RENDER_MS) {
      _lastRender = now;
      _render();
    }
  }

  window.AeroVIOVioHealth = {
    trackingLossCount: function(){ return _trackingLossCount; },
    trend: function(){ return _trend.slice(); }
  };

  function _wire(){
    if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
      window.telemetry.onTelemetry(_onTelemetry);
    }
    _render();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', _wire);
  } else {
    _wire();
  }
}(window);
