/**
 * ============================================================================
 * AeroVIO — Alert Log + Playback Event Markers
 * Rising-edge threshold trips on:
 *   state.controller.position_error.norm            vs APP_CONFIG.thresholds.posErrNorm
 *   state.controller.so3_controller.attitude_error.norm vs APP_CONFIG.thresholds.attErrNorm
 *   state.vio.covariance_trace                      vs APP_CONFIG.thresholds.covTrace
 * 10s per-metric cooldown. Persists to localStorage 'aerovio.alertLog' (cap 500).
 * LIVE frames via window.telemetry.onTelemetry; DEMO frames via
 * AeroVIOAlerts.ingestDemoFrame(frame, index, len) hooked from index.html.
 * ============================================================================
 */
!function(window){
  'use strict';

  var STORAGE_KEY = 'aerovio.alertLog';
  var MAX_ENTRIES = 500;
  var COOLDOWN_MS = 10000;

  var METRICS = [
    { key: 'posErrNorm', label: '||e_p|| (m)',        statePath: ['controller','position_error','norm'],                              thresholdKey: 'posErrNorm' },
    { key: 'attErrNorm', label: '||e_R|| (rad)',      statePath: ['controller','so3_controller','attitude_error','norm'],             thresholdKey: 'attErrNorm' },
    { key: 'covTrace',   label: 'cov trace',          statePath: ['vio','covariance_trace'],                                          thresholdKey: 'covTrace'   }
  ];

  var alerts = [];
  var cooldownUntil = {};   // metricKey -> ts
  var above = {};           // metricKey -> bool (rising edge latch)
  var listeners = [];

  function _threshold(k){
    var t = (window.APP_CONFIG && window.APP_CONFIG.thresholds) || {};
    var dflt = { posErrNorm: 2.0, attErrNorm: 0.10, covTrace: 1e-3 };
    var v = t[k] !== undefined ? t[k] : dflt[k];
    return (typeof v === 'number' && isFinite(v)) ? v : dflt[k];
  }

  function _load(){
    try {
      var raw = localStorage.getItem(STORAGE_KEY);
      if (raw) {
        var arr = JSON.parse(raw);
        if (Array.isArray(arr)) alerts = arr.slice(-MAX_ENTRIES);
      }
    } catch(e) { alerts = []; }
  }

  function _save(){
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(alerts)); } catch(e) {}
  }

  function _fmtVal(v){
    if (v === null || v === undefined || !isFinite(v)) return 'N/A';
    if (Math.abs(v) < 1e-2 && v !== 0) return v.toExponential(2);
    return (+v).toFixed(3);
  }

  function _getPath(obj, path){
    var cur = obj;
    for (var i = 0; i < path.length; i++){
      if (cur === null || cur === undefined) return null;
      cur = cur[path[i]];
    }
    return (typeof cur === 'number' && isFinite(cur)) ? cur : null;
  }

  /**
   * Core threshold evaluation. Returns list of new alert entries.
   * values: {metricKey: number|null}, opts: {pct: 0-100|null (demo playback position)}
   */
  function evaluate(values, opts){
    opts = opts || {};
    var now = Date.now();
    var fired = [];
    METRICS.forEach(function(m){
      var v = values[m.key];
      if (v === null || v === undefined || !isFinite(v)) { above[m.key] = false; return; }
      var th = _threshold(m.thresholdKey);
      var isAbove = v > th;
      var wasAbove = !!above[m.key];
      above[m.key] = isAbove;
      if (isAbove && !wasAbove && now >= (cooldownUntil[m.key] || 0)) {
        cooldownUntil[m.key] = now + COOLDOWN_MS;
        var entry = {
          time: new Date(now).toISOString(),
          metric: m.key,
          label: m.label,
          value: +v,
          threshold: th,
          severity: (v > 1.5 * th) ? 'critical' : 'warning'
        };
        alerts.push(entry);
        if (alerts.length > MAX_ENTRIES) alerts = alerts.slice(alerts.length - MAX_ENTRIES);
        _save();
        fired.push(entry);
        if (opts.pct !== null && opts.pct !== undefined) _addMarker(entry, opts.pct);
        _notify(entry);
      }
    });
    if (fired.length) _render();
    return fired;
  }

  /* --- LIVE telemetry subscription --- */
  function _onTelemetry(state){
    if (!state) return;
    var values = {};
    METRICS.forEach(function(m){ values[m.key] = _getPath(state, m.statePath); });
    evaluate(values, {});
  }

  /* --- DEMO playback hook (called from index.html updateDashboardWithFrame) --- */
  function ingestDemoFrame(frame, index, len){
    if (!frame) return;
    var targetP = (window.APP_CONFIG && window.APP_CONFIG.targetP) || [20.0, 5.0, 20.0];
    var ex = targetP[0] - frame.px, ey = targetP[1] - frame.py, ez = targetP[2] - frame.pz;
    var epNorm = Math.sqrt(ex*ex + ey*ey + ez*ez);
    // matches inline attErr synthesis in index.html updateDashboardWithFrame
    var attErr = 0.015 + Math.abs(Math.sin((frame.t || 0) * 0.5)) * 0.01;
    var pct = (len > 0) ? (index / len) * 100 : null;
    evaluate({ posErrNorm: epNorm, attErrNorm: attErr, covTrace: null }, { pct: pct });
  }

  /* --- Playback event markers on the scrubber --- */
  function _addMarker(entry, pct){
    var layer = document.getElementById('scrubber-marker-layer');
    if (!layer) return;
    var el = document.createElement('button');
    el.type = 'button';
    el.className = 'aerovio-marker severity-' + entry.severity;
    el.style.left = Math.max(0, Math.min(100, pct)) + '%';
    el.title = entry.label + ' = ' + _fmtVal(entry.value) + ' (>' + _fmtVal(entry.threshold) + ') @ ' + entry.time + ' — click to seek';
    el.addEventListener('click', function(ev){
      ev.preventDefault();
      if (typeof window.onScrubberInput === 'function') window.onScrubberInput(pct);
    });
    layer.appendChild(el);
    // cap marker count
    while (layer.children.length > 120) layer.removeChild(layer.firstChild);
  }

  function clearMarkers(){
    var layer = document.getElementById('scrubber-marker-layer');
    if (layer) layer.innerHTML = '';
  }

  /* --- UI rendering --- */
  function _render(){
    var listEl = document.getElementById('alert-log-list');
    var badgeEl = document.getElementById('alert-badge');
    var emptyEl = document.getElementById('alert-log-empty');
    if (badgeEl) {
      badgeEl.textContent = String(alerts.length);
      badgeEl.classList.toggle('hidden', alerts.length === 0);
    }
    if (!listEl) return;
    if (emptyEl) emptyEl.classList.toggle('hidden', alerts.length > 0);
    var recent = alerts.slice(-50).reverse();
    listEl.innerHTML = recent.map(function(a){
      return '<div class="aerovio-alert-entry severity-' + a.severity + '">' +
        '<span class="sev-dot"></span>' +
        '<span class="text-stone-500 whitespace-nowrap">' + (a.time || '').slice(11, 19) + '</span>' +
        '<span class="font-bold">' + _esc(a.label || a.metric) + '</span>' +
        '<span>=' + _fmtVal(a.value) + ' &gt; ' + _fmtVal(a.threshold) + '</span>' +
        '<span class="ml-auto uppercase text-[9px] font-bold">' + a.severity + '</span>' +
      '</div>';
    }).join('');
  }

  function _esc(s){ return String(s).replace(/[&<>"]/g, function(c){ return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]; }); }

  function _notify(entry){
    listeners.forEach(function(cb){ try { cb(entry); } catch(e){} });
    if (typeof window.showToast === 'function') {
      try { window.showToast('ALERT: ' + entry.label + ' = ' + _fmtVal(entry.value) + ' (threshold ' + _fmtVal(entry.threshold) + ')'); } catch(e){}
    }
  }

  function clear(){
    alerts = [];
    _save();
    clearMarkers();
    _render();
  }

  function exportCSV(){
    var header = 'time,metric,value,threshold,severity';
    var rows = alerts.map(function(a){
      return [a.time, a.metric, a.value, a.threshold, a.severity].join(',');
    });
    var blob = new Blob([header + '\n' + rows.join('\n')], { type: 'text/csv' });
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url; a.download = 'aerovio_alert_log_' + Date.now() + '.csv';
    document.body.appendChild(a); a.click(); document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  function toggleCollapse(){
    var card = document.getElementById('alert-log-card');
    if (card) card.classList.toggle('aerovio-collapsed');
  }

  function init(){
    _load();
    _render();
    var btnClr = document.getElementById('alert-clear-btn');
    if (btnClr) btnClr.addEventListener('click', clear);
    var btnExp = document.getElementById('alert-export-btn');
    if (btnExp) btnExp.addEventListener('click', exportCSV);
    var btnCol = document.getElementById('alert-collapse-btn');
    if (btnCol) btnCol.addEventListener('click', toggleCollapse);
    if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
      window.telemetry.onTelemetry(_onTelemetry);
    }
  }

  window.AeroVIOAlerts = {
    init: init,
    ingestDemoFrame: ingestDemoFrame,
    evaluate: evaluate,
    clear: clear,
    exportCSV: exportCSV,
    clearMarkers: clearMarkers,
    toggleCollapse: toggleCollapse,
    getAlerts: function(){ return alerts.slice(); },
    onAlert: function(cb){ listeners.push(cb); }
  };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
}(window);
