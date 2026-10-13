/**
 * AeroVIO — Session Comparison
 * Loads a second recorded CSV (same format the Record button exports:
 * t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,...) and overlays its trajectory on the
 * 3D spatial map plus a position-error plot over the current session,
 * with a diff-stats box (max/mean position-error deltas).
 * Vanilla JS IIFE, no build step.
 */
!function(window){
  'use strict';

  var _ref = null; // { name, points:[{t,px,py,pz,...}], epNorm:[{t,e}] }

  function _targetP(){
    return (window.APP_CONFIG && window.APP_CONFIG.targetP) || [20.0, 5.0, 20.0];
  }

  function _epNorm(p){
    var tp = _targetP();
    var dx = p.px - tp[0], dy = p.py - tp[1], dz = p.pz - tp[2];
    return Math.sqrt(dx*dx + dy*dy + dz*dz);
  }

  // Tolerant CSV parser: header-driven, accepts the Record-export format
  // and the raw synthetic_*_estimated_trajectory.csv files (x,y,z aliases).
  function _parseCsv(text){
    var lines = text.split(/\r?\n/).filter(function(l){ return l.trim().length > 0; });
    if (lines.length < 2) return null;
    var header = lines[0].split(',').map(function(h){ return h.trim(); });
    function col(){
      for (var i = 0; i < arguments.length; i++) {
        for (var j = 0; j < header.length; j++) {
          if (header[j] === arguments[i]) return j;
        }
      }
      return -1;
    }
    var ci = {
      t:  col('t', 'time', 'timestamp'),
      px: col('px', 'x'),
      py: col('py', 'y'),
      pz: col('pz', 'z')
    };
    if (ci.px < 0 || ci.py < 0 || ci.pz < 0) return null;
    var pts = [];
    for (var i = 1; i < lines.length; i++) {
      var f = lines[i].split(',');
      var p = {
        t:  ci.t >= 0 ? parseFloat(f[ci.t]) : i,
        px: parseFloat(f[ci.px]),
        py: parseFloat(f[ci.py]),
        pz: parseFloat(f[ci.pz])
      };
      if (isFinite(p.px) && isFinite(p.py) && isFinite(p.pz)) pts.push(p);
    }
    return pts.length ? pts : null;
  }

  function _currentSessionPoints(){
    var rd = window.REAL_DRONE_DATA;
    var key = (typeof currentDatasetKey !== 'undefined') ? currentDatasetKey : 'orbit';
    var ds = rd && rd.trajectories && rd.trajectories[key];
    return (ds && ds.points) ? ds.points : [];
  }

  // ---- overlay: trajectory on the existing 3D SVG canvas ----------------

  var SVG_NS = 'http://www.w3.org/2000/svg';

  function _project(x, y, z){
    if (typeof window.project3DToSVG === 'function') return window.project3DToSVG(x, y, z);
    return { x: 300 + x * 18, y: 240 + y * 18 }; // iso-ish fallback
  }

  function _renderTrajOverlay(){
    var svg = document.getElementById('svg-spatial-canvas');
    if (!svg) return;
    var old = document.getElementById('svg-compare-traj');
    if (old) old.parentNode.removeChild(old);
    if (!_ref) return;
    var d = _ref.points.map(function(p, i){
      var pt = _project(p.px, p.py, p.pz);
      return (i === 0 ? 'M ' : 'L ') + pt.x + ' ' + pt.y;
    }).join(' ');
    var path = document.createElementNS(SVG_NS, 'path');
    path.setAttribute('id', 'svg-compare-traj');
    path.setAttribute('d', d);
    path.setAttribute('fill', 'none');
    path.setAttribute('stroke', '#F472B6');
    path.setAttribute('stroke-width', '2');
    path.setAttribute('stroke-dasharray', '6 4');
    path.setAttribute('opacity', '0.85');
    svg.appendChild(path);
  }

  // ---- position error comparison plot (own SVG in the compare card) ----

  function _seriesFromPoints(points){
    return points.map(function(p){ return { t: p.t != null ? p.t : 0, e: _epNorm(p) }; });
  }

  function _renderErrPlot(){
    var svg = document.getElementById('compare-err-plot');
    if (!svg) return;
    while (svg.firstChild) svg.removeChild(svg.firstChild);
    var cur = _seriesFromPoints(_currentSessionPoints());
    if (!cur.length && !_ref) return;
    var all = _ref ? cur.concat(_ref.epNorm) : cur;
    var maxE = 0, maxT = 1;
    all.forEach(function(p){ if (p.e > maxE) maxE = p.e; if (p.t > maxT) maxT = p.t; });
    if (maxE <= 0) maxE = 1;
    var W = 400, H = 140, L = 36, B = 16;
    function xy(p){
      return { x: L + (p.t / maxT) * (W - L - 8), y: (H - B) - (p.e / maxE) * (H - B - 10) };
    }
    // axes + threshold line
    var th = (window.APP_CONFIG && window.APP_CONFIG.thresholds) || { posErrNorm: 2.0 };
    var thY = (H - B) - (th.posErrNorm / maxE) * (H - B - 10);
    if (thY > 10 && thY < H - B) {
      var tl = document.createElementNS(SVG_NS, 'line');
      tl.setAttribute('x1', L); tl.setAttribute('x2', W - 8);
      tl.setAttribute('y1', thY); tl.setAttribute('y2', thY);
      tl.setAttribute('stroke', '#EF4444'); tl.setAttribute('stroke-dasharray', '3 3');
      svg.appendChild(tl);
      var tt = document.createElementNS(SVG_NS, 'text');
      tt.setAttribute('x', W - 10); tt.setAttribute('y', thY - 3);
      tt.setAttribute('fill', '#EF4444'); tt.setAttribute('font-size', '8');
      tt.setAttribute('font-family', 'monospace'); tt.setAttribute('text-anchor', 'end');
      tt.textContent = 'thr ' + th.posErrNorm;
      svg.appendChild(tt);
    }
    function draw(series, color, dash){
      if (!series || series.length < 2) return;
      var d = series.map(function(p, i){
        var pt = xy(p);
        return (i === 0 ? 'M ' : 'L ') + pt.x.toFixed(1) + ' ' + pt.y.toFixed(1);
      }).join(' ');
      var path = document.createElementNS(SVG_NS, 'path');
      path.setAttribute('d', d);
      path.setAttribute('fill', 'none');
      path.setAttribute('stroke', color);
      path.setAttribute('stroke-width', '1.8');
      if (dash) path.setAttribute('stroke-dasharray', '5 3');
      svg.appendChild(path);
    }
    draw(cur, '#0EA5E9', false);
    if (_ref) draw(_ref.epNorm, '#F472B6', true);
  }

  // ---- diff stats ---------------------------------------------------------

  function _stats(series){
    if (!series.length) return { max: 0, mean: 0 };
    var max = 0, sum = 0;
    series.forEach(function(p){ if (p.e > max) max = p.e; sum += p.e; });
    return { max: max, mean: sum / series.length };
  }

  function _renderStats(){
    var box = document.getElementById('compare-stats');
    if (!box) return;
    if (!_ref) { box.classList.add('av-hidden'); return; }
    box.classList.remove('av-hidden');
    var curS = _stats(_seriesFromPoints(_currentSessionPoints()));
    var refS = _stats(_ref.epNorm);
    function fmt(n){ return (n >= 0 ? '+' : '') + n.toFixed(3); }
    var dMax = curS.max - refS.max;
    var dMean = curS.mean - refS.mean;
    box.innerHTML =
      '<div class="av-cmp-row"><span>Ref: ' + (_ref.name || 'session') + ' (' + _ref.points.length + ' pts)</span></div>' +
      '<div class="av-cmp-row"><span>max ‖e_p‖</span><strong>' + curS.max.toFixed(3) + ' vs ' + refS.max.toFixed(3) + '</strong><span class="' + (dMax > 0 ? 'av-cmp-worse' : 'av-cmp-better') + '">Δ ' + fmt(dMax) + ' m</span></div>' +
      '<div class="av-cmp-row"><span>mean ‖e_p‖</span><strong>' + curS.mean.toFixed(3) + ' vs ' + refS.mean.toFixed(3) + '</strong><span class="' + (dMean > 0 ? 'av-cmp-worse' : 'av-cmp-better') + '">Δ ' + fmt(dMean) + ' m</span></div>';
  }

  function _renderAll(){ _renderTrajOverlay(); _renderErrPlot(); _renderStats(); }

  function _loadFile(file){
    var reader = new FileReader();
    reader.onload = function(){
      var pts = _parseCsv(String(reader.result || ''));
      if (!pts) {
        if (typeof showToast === 'function') showToast('Compare CSV: no usable t/px/py/pz columns found');
        return;
      }
      _ref = { name: file.name, points: pts, epNorm: _seriesFromPoints(pts) };
      _renderAll();
      var lbl = document.getElementById('compare-ref-name');
      if (lbl) lbl.textContent = file.name;
      if (typeof showToast === 'function') showToast('Overlay loaded: ' + file.name + ' (' + pts.length + ' pts)');
    };
    reader.onerror = function(){
      if (typeof showToast === 'function') showToast('Failed to read CSV file');
    };
    reader.readAsText(file);
  }

  var api = {
    loadFile: _loadFile,
    clear: function(){
      _ref = null;
      var lbl = document.getElementById('compare-ref-name');
      if (lbl) lbl.textContent = 'none';
      _renderAll();
      var inp = document.getElementById('compare-file-input');
      if (inp) inp.value = '';
    },
    stats: function(){
      if (!_ref) return null;
      var curS = _stats(_seriesFromPoints(_currentSessionPoints()));
      var refS = _stats(_ref.epNorm);
      return { current: curS, reference: refS, dMax: curS.max - refS.max, dMean: curS.mean - refS.mean };
    },
    _renderAll: _renderAll
  };
  window.AeroVIOCompare = api;

  function _wire(){
    var inp = document.getElementById('compare-file-input');
    if (inp) inp.addEventListener('change', function(){ if (inp.files && inp.files[0]) _loadFile(inp.files[0]); });
    var clr = document.getElementById('compare-clear-btn');
    if (clr) clr.addEventListener('click', api.clear);

    // Re-render overlay when dataset or perspective changes.
    if (typeof window.switchDataset === 'function' && !window.switchDataset._aerovioCmpWrapped) {
      var _os = window.switchDataset;
      var ws = function(){ var r = _os.apply(this, arguments); _renderErrPlot(); _renderStats(); return r; };
      ws._aerovioCmpWrapped = true;
      window.switchDataset = ws;
    }
    if (typeof window.setPerspective === 'function' && !window.setPerspective._aerovioCmpWrapped) {
      var _op = window.setPerspective;
      var wp = function(){ var r = _op.apply(this, arguments); setTimeout(_renderTrajOverlay, 50); return r; };
      wp._aerovioCmpWrapped = true;
      window.setPerspective = wp;
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', _wire);
  } else {
    _wire();
  }
}(window);
