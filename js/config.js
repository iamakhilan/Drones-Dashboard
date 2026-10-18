/**
 * AeroVIO — Central configuration & constants
 * Loaded FIRST before any other js/*. This is the single source of truth
 * for APP_CONFIG, aligned strictly with reference repository:
 * - config/controller.yaml & config/vehicle.yaml (drones-controller branch)
 * - config/ros2.yaml & config/synthetic.yaml (VIO-Controller branch)
 */
(function(window){
  'use strict';
  var STORAGE_KEY = 'aerovio.wsUrl';
  var storedUrl = null;
  try { storedUrl = localStorage.getItem(STORAGE_KEY); } catch(e) {}

  if (!window.APP_CONFIG) window.APP_CONFIG = {};

  var defaultHost = (typeof window !== 'undefined' && window.location && window.location.host) ? window.location.host : 'localhost:8000';
  var defaultWs = (typeof window !== 'undefined' && window.location && window.location.protocol === 'https:' ? 'wss://' : 'ws://') + defaultHost + '/ws';
  if (storedUrl && storedUrl.indexOf('localhost') !== -1 && window.location && window.location.hostname && window.location.hostname !== 'localhost' && window.location.hostname !== '127.0.0.1') {
    storedUrl = defaultWs;
  }

  var C = window.APP_CONFIG;
  C.mode = 'LIVE'; // Default strictly to LIVE truthful mode
  C.websocketUrl = storedUrl || defaultWs;
  C.rateHz = 50.0;
  C.massKg = 1.5;
  C.gravity = 9.81;
  C.inertia = [0.02, 0.02, 0.04]; // [Jxx, Jyy, Jzz] kg*m^2

  // Gains from config/controller.yaml
  C.kp = [0.5, 0.5, 0.5];
  C.kv = [0.2, 0.2, 0.2];
  C.kr = [4.0, 4.0, 2.0];
  C.komega = [0.8, 0.8, 0.5];

  // Tolerances & limits
  C.maxVelocity = 0.5;       // m/s
  C.posTolerance = 0.10;     // m
  C.velTolerance = 0.05;     // m/s
  C.stateTimeout = 0.2;      // s

  // Target setpoint from config/controller.yaml
  if (!C.targetP) C.targetP = [20.0, 5.0, 20.0];
  if (!C.targetV) C.targetV = [0.0, 0.0, 0.0];
  if (C.targetYaw === undefined) C.targetYaw = 2.0;

  // Extrinsics from VIO config/ros2.yaml & synthetic.yaml
  // Camera optical (OpenCV: Z forward, X right, Y down) relative to drone body ENU
  C.extrinsics = {
    p_BC: [0.10, 0.0, 0.0],
    R_BC: [
      [ 0.0,  0.0,  1.0],
      [-1.0,  0.0,  0.0],
      [ 0.0, -1.0,  0.0]
    ]
  };

  // Alert thresholds for KPI red-pulse
  if (!C.thresholds) C.thresholds = {
    posErrNorm: 2.0,      // m
    attErrNorm: 0.10,     // rad (~5.7 deg)
    covTrace: 1e-3
  };

  // Playback speeds
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
