/**
 * ============================================================================
 * AeroVIO — Snapshot Module
 * Dynamic-imports html2canvas from CDN with an 8s timeout and downloads a PNG
 * of the dashboard <main>. Offline / failure fallback: window.print()
 * (print CSS rules live in css/features.css).
 * ============================================================================
 */
!function(window){
  'use strict';

  var CDN_URL = 'https://cdn.jsdelivr.net/npm/html2canvas@1.4.1/dist/html2canvas.esm.js';
  var TIMEOUT_MS = 8000;
  var html2canvasPromise = null;
  var busy = false;

  function _toast(msg){
    if (typeof window.showToast === 'function') {
      try { window.showToast(msg); return; } catch(e){}
    }
  }

  function _importHtml2Canvas(){
    if (html2canvasPromise) return html2canvasPromise;
    html2canvasPromise = Promise.race([
      import(/* webpackIgnore: true */ CDN_URL).then(function(mod){
        return mod.default || mod.html2canvas || mod;
      }),
      new Promise(function(_, reject){
        setTimeout(function(){ reject(new Error('html2canvas load timed out after ' + (TIMEOUT_MS / 1000) + 's')); }, TIMEOUT_MS);
      })
    ]).catch(function(err){
      html2canvasPromise = null; // allow retry
      throw err;
    });
    return html2canvasPromise;
  }

  function capture(){
    if (busy) { _toast('Snapshot already in progress…'); return; }
    busy = true;
    _toast('Capturing snapshot…');
    var target = document.querySelector('main') || document.body;
    _importHtml2Canvas().then(function(html2canvas){
      return html2canvas(target, {
        backgroundColor: '#FAFAF9',
        scale: Math.min(2, window.devicePixelRatio || 1),
        useCORS: true,
        logging: false
      });
    }).then(function(canvas){
      var a = document.createElement('a');
      a.href = canvas.toDataURL('image/png');
      a.download = 'aerovio_snapshot_' + Date.now() + '.png';
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
      _toast('Snapshot saved as PNG.');
    }).catch(function(err){
      // Offline or CDN blocked: fall back to print stylesheet
      _toast('Snapshot capture unavailable (' + (err && err.message ? err.message : 'offline') + ') — opening print view.');
      try { window.print(); } catch(e) {}
    }).then(function(){ busy = false; });
  }

  window.AeroVIOSnapshot = { capture: capture };

  var btn = document.getElementById('snapshot-btn');
  if (btn) btn.addEventListener('click', capture);
  else if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){
      var b = document.getElementById('snapshot-btn');
      if (b) b.addEventListener('click', capture);
    });
  }
}(window);
