/**
 * ============================================================================
 * AeroVIO — Session Comparison Module
 * Loads a record-export CSV (header: t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,...)
 * as a REFERENCE session and compares against the CURRENT session sampled
 * from DEMO playback frames (and LIVE telemetry positions).
 * Renders: top-view (X-Y) SVG trajectory overlay (reference dashed),
 * ||e_p|| overlay plot, and max/mean delta stats matched by nearest time.
 * ============================================================================
 */
!function(window){
  'use strict';

  var CURRENT_MAX = 4000;
  var current = [];     // {t, px, py, pz, ep}
  var reference = null; // {frames:[{t,px,py,pz,ep}], name}
  var lastRender = 0;

  function _targetP(){
    return (window.APP_CONFIG && window.APP_CONFIG.targetP) || [20.0, 5.0, 20.0];
  }

  function _epOf(p){
    var tp = _targetP();
    var dx = tp[0] - p.px, dy = tp[1] - p.py, dz = tp[2] - p.pz;
    return Math.sqrt(dx*dx + dy*dy + dz*dz);
  }

  function _setText(id, txt){
    var el = document.getElementById(id);
    if (el) el.textContent = txt;
  }

  function ingestDemoFrame(frame){
    if (!frame) return;
    current.push({ t: frame.t || 0, px: frame.px, py: frame.py, pz: frame.pz, ep: _epOf(frame) });
    if (current.length > CURRENT_MAX) current.shift();
    var now = performance.now();
    if (reference && now - lastRender > 500) { lastRender = now; _renderPlots(); }
  }

  function _onTelemetry(state){
    if (!state || !state.position) return;
    var p = state.position;
    if (p.x === null || p.x === undefined) return;
    var f = { t: (state.timestamp || Date.now()) / 1000, px: p.x, py: p.y, pz: p.z };
    f.ep = _epOf(f);
    current.push(f);
    if (current.length > CURRENT_MAX) current.shift();
  }

  /* --- CSV parsing (tolerant: matches record-export header from index.html) --- */
  function parseCSV(text){
    var lines = text.split(/\r?\n/).filter(function(l){ return l.trim().length; });
    if (lines.length < 2) return [];
    var header = lines[0].split(',').map(function(h){ return h.trim(); });
    var idx = {};
    ['t','px','py','pz'].forEach(function(k){ idx[k] = header.indexOf(k); });
    if (idx.t < 0 || idx.px < 0 || idx.py < 0 || idx.pz < 0) {
      // positional fallback: assume t,px,py,pz,...
      idx = { t: 0, px: 1, py: 2, pz: 3 };
    }
    var out = [];
    for (var i = 1; i < lines.length; i++){
      var cols = lines[i].split(',');
      var f = {
        t:  parseFloat(cols[idx.t]),
        px: parseFloat(cols[idx.px]),
        py: parseFloat(cols[idx.py]),
        pz: parseFloat(cols[idx.pz])
      };
      if (!isFinite(f.t) || !isFinite(f.px) || !isFinite(f.py) || !isFinite(f.pz)) continue;
      f.ep = _epOf(f);
      out.push(f);
    }
    return out;
  }

  function loadReferenceFile(file){
    if (!file) return;
    var reader = new FileReader();
    reader.onload = function(){
      try {
        var frames = parseCSV(String(reader.result || ''));
        if (!frames.length) {
          _setText('cmp-status', 'No valid rows parsed from ' + file.name);
          return;
        }
        reference = { frames: frames, name: file.name };
        _setText('cmp-status', 'Reference: ' + file.name + ' (' + frames.length + ' pts)');
        _renderPlots();
        if (typeof window.showToast === 'function') {
          try { window.showToast('Comparison reference loaded: ' + file.name); } catch(e){}
        }
      } catch(e) {
        _setText('cmp-status', 'Parse error: ' + e.message);
      }
    };
    reader.onerror = function(){ _setText('cmp-status', 'Failed to read file.'); };
    reader.readAsText(file);
  }

  function clearReference(){
    reference = null;
    _setText('cmp-status', 'No reference loaded.');
    _renderPlots();
  }

  /* --- stats: nearest-time deltas between current and reference positions --- */
  function _computeStats(){
    if (!reference || !reference.frames.length || current.length < 2) return null;
    var ref = reference.frames;
    var used = current.length > 800 ? current.slice(current.length - 800) : current;
    var sum = 0, max = 0, n = 0;
    for (var i = 0; i < used.length; i++){
      var c = used[i];
      // binary search nearest by t
      var lo = 0, hi = ref.length - 1;
      while (hi - lo > 1) {
        var mid = (lo + hi) >> 1;
        if (ref[mid].t < c.t) lo = mid; else hi = mid;
      }
      var r = (Math.abs(ref[lo].t - c.t) <= Math.abs(ref[hi].t - c.t)) ? ref[lo] : ref[hi];
      var dx = c.px - r.px, dy = c.py - r.py, dz = c.pz - r.pz;
      var d = Math.sqrt(dx*dx + dy*dy + dz*dz);
      sum += d; if (d > max) max = d; n++;
    }
    if (!n) return null;
    return { max: max, mean: sum / n, samples: n };
  }

  /* --- SVG plot helpers --- */
  function _trajSVG(){
    var cur = current, ref = reference ? reference.frames : [];
    if (!cur.length && !ref.length) {
      return '<text x="200" y="85" text-anchor="middle">awaiting position samples…</text>';
    }
    var xs = [], ys = [];
    cur.forEach(function(p){ xs.push(p.px); ys.push(p.py); });
    ref.forEach(function(p){ xs.push(p.px); ys.push(p.py); });
    var x0 = Math.min.apply(null, xs), x1 = Math.max.apply(null, xs);
    var y0 = Math.min.apply(null, ys), y1 = Math.max.apply(null, ys);
    if (x1 - x0 < 1e-6) x1 = x0 + 1;
    if (y1 - y0 < 1e-6) y1 = y0 + 1;
    var pad = 14, W = 400, H = 160;
    function sx(x){ return pad + (x - x0) / (x1 - x0) * (W - 2 * pad); }
    function sy(y){ return H - pad - (y - y0) / (y1 - y0) * (H - 2 * pad); }
    function path(pts){
      return pts.map(function(p, i){
        return (i ? 'L' : 'M') + sx(p.px).toFixed(1) + ',' + sy(p.py).toFixed(1);
      }).join(' ');
    }
    var s = '<text x="' + pad + '" y="12">X-Y top view (m)  x∈[' + x0.toFixed(1) + ',' + x1.toFixed(1) + '] y∈[' + y0.toFixed(1) + ',' + y1.toFixed(1) + ']</text>';
    if (ref.length) {
      s += '<path d="' + path(ref) + '" fill="none" stroke="#D97706" stroke-width="1.4" stroke-dasharray="5 4" opacity="0.9"/>';
    }
    if (cur.length) {
      s += '<path d="' + path(cur) + '" fill="none" stroke="#57534E" stroke-width="1.6"/>';
    }
    return s;
  }

  function _epSVG(){
    var cur = current, ref = reference ? reference.frames : [];
    var all = cur.concat(ref);
    if (!all.length) return '<text x="200" y="85" text-anchor="middle">no ||e_p|| samples…</text>';
    var t0 = Math.min.apply(null, all.map(function(p){ return p.t; }));
    var t1 = Math.max.apply(null, all.map(function(p){ return p.t; }));
    var eMax = Math.max.apply(null, all.map(function(p){ return p.ep; }));
    var th = (window.APP_CONFIG && window.APP_CONFIG.thresholds && window.APP_CONFIG.thresholds.posErrNorm);
    if (typeof th === 'number' && isFinite(th) && th > eMax) eMax = th;
    if (eMax <= 1e-9) eMax = 1;
    if (t1 - t0 < 1e-6) t1 = t0 + 1;
    var pad = 30, W = 400, H = 160, rpad = 12;
    function sx(t){ return pad + (t - t0) / (t1 - t0) * (W - pad - rpad); }
    function sy(e){ return H - 16 - (e / eMax) * (H - 30); }
    function path(pts){
      return pts.map(function(p, i){
        return (i ? 'L' : 'M') + sx(p.t).toFixed(1) + ',' + sy(p.ep).toFixed(1);
      }).join(' ');
    }
    var s = '';
    // gridlines
    [0, 0.5, 1].forEach(function(f){
      var y = sy(eMax * f);
      s += '<line x1="' + pad + '" x2="' + (W - rpad) + '" y1="' + y + '" y2="' + y + '" stroke="#E7E5E4" stroke-width="1"/>';
      s += '<text x="' + (pad - 3) + '" y="' + (y + 3) + '" text-anchor="end">' + (eMax * f).toFixed(1) + '</text>';
    });
    if (typeof th === 'number' && isFinite(th)) {
      s += '<line x1="' + pad + '" x2="' + (W - rpad) + '" y1="' + sy(th) + '" y2="' + sy(th) + '" stroke="#D97706" stroke-width="1" stroke-dasharray="4 3"/>';
      s += '<text x="' + (W - rpad) + '" y="' + (sy(th) - 3) + '" text-anchor="end" fill="#B45309">thr ' + th + 'm</text>';
    }
    if (ref.length) s += '<path d="' + path(ref) + '" fill="none" stroke="#D97706" stroke-width="1.3" stroke-dasharray="5 4"/>';
    if (cur.length) s += '<path d="' + path(cur) + '" fill="none" stroke="#57534E" stroke-width="1.5"/>';
    s += '<text x="' + pad + '" y="12">‖e_p‖ (m) vs t — current solid, reference dashed</text>';
    return s;
  }

  function _renderPlots(){
    var traj = document.getElementById('cmp-traj-plot');
    if (traj) traj.innerHTML = _trajSVG();
    var ep = document.getElementById('cmp-ep-plot');
    if (ep) ep.innerHTML = _epSVG();
    var stats = _computeStats();
    _setText('cmp-samples', String(current.length));
    if (stats) {
      _setText('cmp-max-delta', stats.max.toFixed(3) + ' m');
      _setText('cmp-mean-delta', stats.mean.toFixed(3) + ' m');
      _setText('cmp-matched', String(stats.samples));
    } else {
      _setText('cmp-max-delta', 'N/A');
      _setText('cmp-mean-delta', 'N/A');
      _setText('cmp-matched', '0');
    }
  }

  function init(){
    var input = document.getElementById('cmp-file-input');
    if (input) {
      input.addEventListener('change', function(){ loadReferenceFile(input.files && input.files[0]); });
    }
    var clr = document.getElementById('cmp-clear-btn');
    if (clr) clr.addEventListener('click', function(){
      if (input) input.value = '';
      clearReference();
    });
    if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
      window.telemetry.onTelemetry(_onTelemetry);
    }
    _renderPlots();
  }

  window.AeroVIOCompare = {
    init: init,
    ingestDemoFrame: ingestDemoFrame,
    loadReferenceFile: loadReferenceFile,
    clearReference: clearReference,
    parseCSV: parseCSV,
    render: _renderPlots,
    getCurrent: function(){ return current.slice(); },
    getReference: function(){ return reference ? reference.frames.slice() : null; }
  };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
}(window);
