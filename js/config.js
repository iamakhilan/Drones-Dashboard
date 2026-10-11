/**
 * AeroVIO — Central configuration & constants
 * Loaded FIRST before any other js/*. This is the single source of truth
 * for APP_CONFIG. Other modules read window.APP_CONFIG, never redefine it.
 */
(function(window){
  'use strict';
  var STORAGE_KEY = 'aerovio.wsUrl';
  var storedUrl = null;
  try { storedUrl = localStorage.getItem(STORAGE_KEY); } catch(e) {}

  if (!window.APP_CONFIG) window.APP_CONFIG = {};

  var C = window.APP_CONFIG;
  C.websocketUrl = storedUrl || C.websocketUrl || 'ws://localhost:8000/ws';
  C.rateHz = C.rateHz || 50.0;
  C.massKg = C.massKg || 1.5;
  C.gravity = C.gravity || 9.81;
  // Configurable target setpoint (also settable via --target flag on bridge)
  if (!C.targetP) C.targetP = [20.0, 5.0, 20.0];
  if (!C.targetV) C.targetV = [0.0, 0.0, 0.0];
  // Alert thresholds for KPI red-pulse
  if (!C.thresholds) C.thresholds = {
    posErrNorm: 2.0,      // m
    attErrNorm: 0.10,     // rad (~5.7 deg)
    covTrace: 1e-3
  };
  // Playback
  if (!C.playbackSpeeds) C.playbackSpeeds = [0.5, 1.0, 2.0];

  // Helpers
  C._storageKey = STORAGE_KEY;
  C.persistWsUrl = function(url){
    try { localStorage.setItem(STORAGE_KEY, url); } catch(e){}
    C.websocketUrl = url;
    var badge = document.getElementById('top-endpoint-badge');
    if (badge) badge.textContent = url;
    var inp = document.getElementById('tune-ws-url');
    if (inp && inp.value !== url) inp.value = url;
  };
})(window);
