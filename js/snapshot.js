/**
 * AeroVIO — Dashboard Snapshot
 * One-click PNG export of the current dashboard view. Dynamically loads
 * html2canvas from CDN only when clicked (project otherwise avoids CDNs);
 * falls back to window.print() when the CDN is unreachable (offline),
 * the library is blocked, or rendering fails.
 * Vanilla JS IIFE, no build step.
 */
!function(window){
  'use strict';

  var CDN_URL = 'https://cdnjs.cloudflare.com/ajax/libs/html2canvas/1.4.1/html2canvas.min.js';
  var _loadingPromise = null;

  function _toast(msg){
    if (typeof showToast === 'function') showToast(msg);
  }

  function _loadHtml2Canvas(){
    if (window.html2canvas) return Promise.resolve(window.html2canvas);
    if (_loadingPromise) return _loadingPromise;
    _loadingPromise = new Promise(function(resolve, reject){
      var s = document.createElement('script');
      s.src = CDN_URL;
      s.async = true;
      var timer = setTimeout(function(){
        reject(new Error('html2canvas CDN load timed out'));
      }, 8000);
      s.onload = function(){
        clearTimeout(timer);
        if (window.html2canvas) resolve(window.html2canvas);
        else reject(new Error('html2canvas missing after load'));
      };
      s.onerror = function(){
        clearTimeout(timer);
        reject(new Error('html2canvas CDN unreachable'));
      };
      document.head.appendChild(s);
    });
    return _loadingPromise;
  }

  function snapshot(){
    var target = document.querySelector('main') || document.body;
    _toast('Rendering snapshot…');
    _loadHtml2Canvas()
      .then(function(h2c){
        return h2c(target, {
          backgroundColor: '#F6F6F7',
          scale: Math.min(2, (window.devicePixelRatio || 1)),
          useCORS: true,
          logging: false
        });
      })
      .then(function(canvas){
        return new Promise(function(resolve, reject){
          canvas.toBlob(function(blob){
            if (blob) resolve(blob); else reject(new Error('toBlob failed'));
          }, 'image/png');
        });
      })
      .then(function(blob){
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = 'aerovio_snapshot_' + Date.now() + '.png';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        _toast('Snapshot saved as PNG');
      })
      .catch(function(){
        // graceful offline fallback
        _toast('PNG renderer unavailable (offline?) — falling back to print');
        window.print();
      });
  }

  window.AeroVIOSnapshot = { capture: snapshot };

  function _wire(){
    var btn = document.getElementById('snapshot-btn');
    if (btn) btn.addEventListener('click', snapshot);
  }
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', _wire);
  } else {
    _wire();
  }
}(window);
