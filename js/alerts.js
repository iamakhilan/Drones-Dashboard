/**
 * AeroVIO — Alert Log + Playback Event Markers
 * Listens to the same frame stream the KPI threshold-pulse uses
 * (window.updateDashboardWithFrame) plus the live telemetry engine state.
 * Vanilla JS IIFE, no build step. Exposes window.AeroVIOAlerts.
 */
!function(window){
  'use strict';

  var STORAGE_KEY = 'aerovio.alertLog';
  var MAX_ENTRIES = 500;
  var COOLDOWN_MS = 10000; // min gap between log entries for the same metric

  var _entries = [];
  try {
    var raw = localStorage.getItem(STORAGE_KEY);
    if (raw) { var parsed = JSON.parse(raw); if (Array.isArray(parsed)) _entries = parsed.slice(-MAX_ENTRIES); }
  } catch(e) {}

  var _lastTripByMetric = {};   // metric -> ts of last logged crossing
  var _trippedState = {};       // metric -> bool (edge detection)
  var _markersByDataset = {};   // datasetKey -> [{t, pct, metric, severity}]
  var _currentDatasetKey = null;

  function _thresholds(){
    return (window.APP_CONFIG && window.APP_CONFIG.thresholds) || { posErrNorm: 2.0, attErrNorm: 0.10, covTrace: 1e-3 };
  }

  function _severity(value, threshold){
    return value >= threshold * 1.5 ? 'critical' : 'warning';
  }

  function _persist(){
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(_entries.slice(-MAX_ENTRIES))); } catch(e) {}
  }

  function _add(metric, value, threshold, playCtx){
    var now = Date.now();
    var sev = _severity(value, threshold);
    var entry = {
      time: new Date(now).toISOString(),
      metric: metric,
      value: +value.toFixed(6),
      threshold: threshold,
      severity: sev
    };
    _entries.push(entry);
    if (_entries.length > MAX_ENTRIES) _entries = _entries.slice(-MAX_ENTRIES);
    _persist();
    _render();

    // Playback event marker
    if (playCtx && playCtx.datasetKey != null && playCtx.pct != null) {
      var list = _markersByDataset[playCtx.datasetKey] || (_markersByDataset[playCtx.datasetKey] = []);
      // avoid marker spam: max 1 marker per metric per 2% of timeline
      var dup = list.some(function(m){ return m.metric === metric && Math.abs(m.pct - playCtx.pct) < 2; });
      if (!dup) {
        list.push({ t: playCtx.t, pct: playCtx.pct, metric: metric, severity: sev });
        _renderMarkers();
      }
    }
    return entry;
  }

  // Check a metric against its threshold with rising-edge + cooldown dedup.
  function _check(metric, value, threshold, playCtx){
    if (value == null || !isFinite(value) || threshold == null) return;
    var over = value > threshold;
    var now = Date.now();
    if (over) {
      var last = _lastTripByMetric[metric] || 0;
      if (!_trippedState[metric] || (now - last) > COOLDOWN_MS) {
        _lastTripByMetric[metric] = now;
        _add(metric, value, threshold, playCtx);
      }
    }
    _trippedState[metric] = over;
  }

  // ---- metric extraction -----------------------------------------------

  function _targetP(){
    return (window.APP_CONFIG && window.APP_CONFIG.targetP) || [20.0, 5.0, 20.0];
  }

  // Called from the wrapped updateDashboardWithFrame (playback scrubber stream).
  function onFrame(frame, datasetKey, frameIndex, totalFrames){
    if (!frame) return;
    var th = _thresholds();
    var tp = _targetP();
    var playCtx = null;
    if (typeof frameIndex === 'number' && totalFrames > 0) {
      playCtx = { datasetKey: datasetKey, pct: (frameIndex / totalFrames) * 100, t: frame.t };
    }
    if (frame.px != null) {
      var dx = frame.px - tp[0], dy = frame.py - tp[1], dz = frame.pz - tp[2];
      _check('posErrNorm', Math.sqrt(dx*dx + dy*dy + dz*dz), th.posErrNorm, playCtx);
    }
    if (frame.t != null) {
      // same synthesized attitude error the KPI card uses
      _check('attErrNorm', 0.015 + Math.abs(Math.sin(frame.t * 0.5)) * 0.01, th.attErrNorm);
    }
    if (frame.cov_trace != null) {
      _check('covTrace', frame.cov_trace, th.covTrace, playCtx);
    }
  }

  // Called from the telemetry engine (bridge / live path).
  function _onTelemetry(state){
    if (!state) return;
    var th = _thresholds();
    var c = state.controller || {};
    var vio = state.vio || {};
    var pe = c.position_error || {};
    var ae = (c.so3_controller && c.so3_controller.attitude_error) || {};
    _check('posErrNorm', pe.norm, th.posErrNorm);
    _check('attErrNorm', ae.norm, th.attErrNorm);
    _check('covTrace', vio.covariance_trace, th.covTrace);
  }

  // ---- panel rendering --------------------------------------------------

  function _els(){
    return {
      list: document.getElementById('alert-log-list'),
      badge: document.getElementById('alert-log-badge'),
      empty: document.getElementById('alert-log-empty'),
      track: document.getElementById('scrubber-marker-track'),
      panel: document.getElementById('alert-log-panel')
    };
  }

  function _render(){
    var el = _els();
    if (el.badge) {
      el.badge.textContent = String(_entries.length);
      el.badge.classList.toggle('av-hidden', _entries.length === 0);
    }
    if (!el.list) return;
    if (_entries.length === 0) {
      el.list.innerHTML = '';
      if (el.empty) el.empty.classList.remove('av-hidden');
      return;
    }
    if (el.empty) el.empty.classList.add('av-hidden');
    var rows = _entries.slice(-80).reverse().map(function(e){
      var cls = e.severity === 'critical' ? 'av-alert-crit' : 'av-alert-warn';
      return '<div class="av-alert-row ' + cls + '">' +
        '<span class="av-alert-time">' + e.time.slice(11, 19) + '</span>' +
        '<span class="av-alert-metric">' + e.metric + '</span>' +
        '<span class="av-alert-val">' + e.value + '</span>' +
        '<span class="av-alert-th">&gt; ' + e.threshold + '</span>' +
        '<span class="av-alert-sev">' + e.severity.toUpperCase() + '</span>' +
      '</div>';
    }).join('');
    el.list.innerHTML = rows;
  }

  function _renderMarkers(){
    var el = _els();
    if (!el.track) return;
    var key = _currentDatasetKey || (window.AeroVIOPlayback && window.AeroVIOPlayback.datasetKey) || 'orbit';
    var list = _markersByDataset[key] || [];
    el.track.innerHTML = list.map(function(m, i){
      return '<button type="button" class="av-marker av-marker-' + m.severity + '" ' +
        'style="left:' + m.pct.toFixed(2) + '%" data-idx="' + i + '" ' +
        'title="' + m.metric + ' ' + m.severity + ' @ t=' + (m.t != null ? m.t.toFixed(1) : '?') + 's — click to jump"></button>';
    }).join('');
  }

  // ---- public API --------------------------------------------------------

  var api = {
    onFrame: onFrame,
    add: function(entry){
      if (!entry || !entry.metric) return null;
      return _add(entry.metric, +entry.value || 0, +entry.threshold || 0, null);
    },
    list: function(){ return _entries.slice(); },
    clear: function(){
      _entries = [];
      _trippedState = {};
      _lastTripByMetric = {};
      _persist();
      _render();
    },
    exportCsv: function(){
      if (_entries.length === 0) return;
      var header = 'time,metric,value,threshold,severity';
      var rows = _entries.map(function(e){
        return [e.time, e.metric, e.value, e.threshold, e.severity].join(',');
      }).join('\n');
      var blob = new Blob([header + '\n' + rows], { type: 'text/csv' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url; a.download = 'aerovio_alerts_' + Date.now() + '.csv';
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
      URL.revokeObjectURL(url);
    },
    markers: function(datasetKey){ return (_markersByDataset[datasetKey] || []).slice(); },
    jumpTo: function(pct){ if (typeof window.onScrubberInput === 'function') window.onScrubberInput(pct); }
  };
  window.AeroVIOAlerts = api;

  // ---- wiring ------------------------------------------------------------

  function _wire(){
    // 1) Hook the telemetry engine (live bridge + engine-driven DEMO stream).
    if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
      window.telemetry.onTelemetry(_onTelemetry);
    }

    // 2) Wrap the playback frame renderer (the KPI threshold-pulse stream).
    if (typeof window.updateDashboardWithFrame === 'function' && !window.updateDashboardWithFrame._aerovioWrapped) {
      var _origUpdate = window.updateDashboardWithFrame;
      var wrapped = function(frame, allPoints){
        try {
          var key = _currentDatasetKey || (typeof currentDatasetKey !== 'undefined' ? currentDatasetKey : 'orbit');
          var total = (allPoints && allPoints.length) || 1;
          var idx = (typeof currentFrameIndex !== 'undefined' ? currentFrameIndex : 0);
          api.onFrame(frame, key, idx, total);
        } catch(e) {}
        return _origUpdate.apply(this, arguments);
      };
      wrapped._aerovioWrapped = true;
      window.updateDashboardWithFrame = wrapped;
    }

    // 3) Track dataset switches so markers render for the active dataset.
    if (typeof window.switchDataset === 'function' && !window.switchDataset._aerovioWrapped) {
      var _origSwitch = window.switchDataset;
      var wrappedSwitch = function(key){
        _currentDatasetKey = key;
        var r = _origSwitch.apply(this, arguments);
        _renderMarkers();
        return r;
      };
      wrappedSwitch._aerovioWrapped = true;
      window.switchDataset = wrappedSwitch;
    }

    // 4) Panel controls (delegated, guards inside).
    document.addEventListener('click', function(ev){
      var t = ev.target;
      if (!t || !t.closest) return;
      if (t.closest('#alert-log-clear')) { api.clear(); return; }
      if (t.closest('#alert-log-export')) { api.exportCsv(); return; }
      if (t.closest('#alert-log-toggle')) {
        var p = _els().panel;
        if (p) p.classList.toggle('av-collapsed');
        return;
      }
      var marker = t.closest('.av-marker');
      if (marker && marker.getAttribute('style')) {
        var m = (marker.getAttribute('style').match(/left:\s*([\d.]+)%/) || [])[1];
        if (m != null) api.jumpTo(parseFloat(m));
      }
    });

    _render();
    _renderMarkers();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', _wire);
  } else {
    _wire();
  }
}(window);
