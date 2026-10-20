/**
 * ===============================================================================
 * AeroVIO Ground Station - 8BitDo Ultimate 2C & Gamepad Teleoperation Engine
 * ===============================================================================
 * Provides seamless RC Mode 2 flight control from any browser via HTML5 Gamepad API.
 * 
 * Hardware Mapping (8BitDo Ultimate 2C / Standard W3C / Linux evdev):
 *   - Left Stick (Vertical)   : Throttle / Climb (+Z) & Descend (-Z)
 *   - Left Stick (Horizontal) : Yaw Rate (+Yaw Left / -Yaw Right)
 *   - Right Stick (Vertical)  : Pitch / Forward (+X) & Reverse (-X)
 *   - Right Stick (Horizontal): Roll / Strafe Left (+Y) & Strafe Right (-Y)
 * 
 * Buttons:
 *   - Button Y (Top, btn[3])  : Switch Mode to GUIDED (Mode 4)
 *   - Button A (Bottom, btn[0]): ARM Motors
 *   - Button X (Left, btn[2]) : TAKEOFF to 5.0m
 *   - Button B (Right, btn[1]): BRAKE / HOVER HOLD (Zero velocity)
 *   - LB (btn[4])             : Precision Speed (0.5 m/s)
 *   - RB (btn[5])             : Turbo Speed (2.0 m/s)
 * 
 * D-Pad (btn[12..15] or Hat):
 *   - D-Pad UP                : Tilt Camera UP (towards horizon)
 *   - D-Pad DOWN              : Tilt Camera DOWN (towards ground)
 *   - D-Pad LEFT              : Pan Camera LEFT
 *   - D-Pad RIGHT             : Pan Camera RIGHT
 * ===============================================================================
 */

(function () {
  'use strict';

  var DEADZONE = 0.12;
  var DEFAULT_SPEED = 1.0;
  var PRECISION_SPEED = 0.5;
  var TURBO_SPEED = 2.0;
  var YAW_SPEED = 0.8;
  var VERTICAL_SPEED = 0.8;

  var savedAxisMode = 'auto';
  try {
    var stored = localStorage.getItem('aerovio_gamepad_axis_mode');
    if (stored && stored !== 'pair_2_3') savedAxisMode = stored;
    else savedAxisMode = 'auto';
  } catch (e) {
    savedAxisMode = 'auto';
  }

  var state = {
    connected: false,
    gamepadIndex: null,
    id: 'No Gamepad',
    speedMode: 'normal', // 'precision', 'normal', 'turbo'
    maxSpeed: DEFAULT_SPEED,
    lastSentTime: 0,
    isActivelyMoving: false,
    prevButtons: {},
    prevHats: {},
    axes: { vx: 0, vy: 0, vz: 0, yaw_rate: 0 },
    rawAxes: [],
    axisMode: savedAxisMode,
    detectedPair: [2, 5], // Default for 8BitDo on Linux Chrome [Roll=Axis 2, Pitch=Axis 5]
    baselines: {}
  };

  function applyDeadzone(val) {
    if (Math.abs(val) < DEADZONE) return 0.0;
    var sign = val > 0 ? 1 : -1;
    return sign * ((Math.abs(val) - DEADZONE) / (1.0 - DEADZONE));
  }

  function getActiveGamepad() {
    if (!navigator.getGamepads) return null;
    var gamepads = navigator.getGamepads();
    if (!gamepads || gamepads.length === 0) return null;

    // 1. If currently bound to a gamepad, verify it is not the idle input-remapper dummy
    if (state.gamepadIndex !== null && gamepads[state.gamepadIndex]) {
      var current = gamepads[state.gamepadIndex];
      var cid = (current.id || '').toLowerCase();
      if (cid.indexOf('input-remapper') === -1) {
        return current;
      }
      // If bound to input-remapper, check if a real hardware gamepad is connected
      for (var k = 0; k < gamepads.length; k++) {
        var gpK = gamepads[k];
        if (gpK && gpK.id && gpK.id.toLowerCase().indexOf('input-remapper') === -1) {
          state.gamepadIndex = k;
          return gpK;
        }
      }
      return current;
    }

    // 2. Select physical hardware gamepad (prioritize 8BitDo, Wireless, Xbox) over virtual input-remapper
    var preferredIndex = null;
    var fallbackIndex = null;

    for (var i = 0; i < gamepads.length; i++) {
      var gp = gamepads[i];
      if (!gp) continue;
      var gid = (gp.id || '').toLowerCase();
      if (gid.indexOf('8bitdo') !== -1 || gid.indexOf('wireless') !== -1 || gid.indexOf('xbox') !== -1) {
        preferredIndex = i;
        break;
      }
      if (gid.indexOf('input-remapper') === -1 && fallbackIndex === null) {
        fallbackIndex = i;
      }
    }

    var targetIndex = preferredIndex !== null ? preferredIndex : (fallbackIndex !== null ? fallbackIndex : 0);
    if (gamepads[targetIndex]) {
      state.gamepadIndex = targetIndex;
      return gamepads[targetIndex];
    }
    return null;
  }

  function determineRightStick(gp) {
    if (!gp || !gp.axes) return { rawX: 0, rawY: 0, pair: [2, 5] };

    var mode = state.axisMode;

    if (mode === 'pair_3_4' || mode === 'xpad') {
      state.detectedPair = [3, 4];
      return {
        rawX: gp.axes.length > 3 ? gp.axes[3] : 0,
        rawY: gp.axes.length > 4 ? gp.axes[4] : 0,
        pair: [3, 4]
      };
    }
    if (mode === 'pair_2_5' || mode === 'bt_linux') {
      state.detectedPair = [2, 5];
      return {
        rawX: gp.axes.length > 2 ? gp.axes[2] : 0,
        rawY: gp.axes.length > 5 ? gp.axes[5] : 0,
        pair: [2, 5]
      };
    }
    if (mode === 'pair_2_3' || mode === 'standard') {
      state.detectedPair = [2, 3];
      return {
        rawX: gp.axes.length > 2 ? gp.axes[2] : 0,
        rawY: gp.axes.length > 3 ? gp.axes[3] : 0,
        pair: [2, 3]
      };
    }

    // Default 'auto': Smart Baseline-Aware Hybrid Dynamic Detection
    // On Linux Chrome, 8BitDo Ultimate 2C in Bluetooth mode exposes:
    //   Axis 0 = Yaw (Left Stick X)
    //   Axis 1 = Throttle (Left Stick Y)
    //   Axis 2 = Roll (Right Stick X, evdev ABS_Z=2)
    //   Axis 5 = Pitch (Right Stick Y, evdev ABS_RZ=5)
    // On W3C standard gamepads, Pitch is on Axis 3.
    var a2 = gp.axes.length > 2 ? gp.axes[2] : 0;
    var a3 = gp.axes.length > 3 ? gp.axes[3] : 0;
    var a4 = gp.axes.length > 4 ? gp.axes[4] : 0;
    var a5 = gp.axes.length > 5 ? gp.axes[5] : 0;

    function getDelta(idx) {
      if (!gp.axes || idx >= gp.axes.length) return 0;
      var base = state.baselines[idx] !== undefined ? state.baselines[idx] : 0;
      return Math.abs(gp.axes[idx] - base);
    }

    var pair = state.detectedPair ? [state.detectedPair[0], state.detectedPair[1]] : [2, 5];

    // Resting baseline check: If Axis 5 rests near -1.0, it is a Trigger (so Pitch is Axis 3).
    // If Axis 5 rests near 0.0 (evdev ABS_RZ), it is Pitch!
    var b5 = state.baselines[5] !== undefined ? state.baselines[5] : (gp.axes.length > 5 ? gp.axes[5] : 0);
    var isAxis5Trigger = (b5 < -0.6);

    var d2 = getDelta(2);
    var d3 = getDelta(3);
    var d4 = getDelta(4);
    var d5 = getDelta(5);

    // Dynamic Active Deflection Priority:
    if (d5 > 0.08 && !isAxis5Trigger) {
      // Axis 5 moving and not a trigger -> Pitch is Axis 5
      pair[1] = 5;
    } else if (d4 > 0.08 && Math.abs(state.baselines[4] || 0) < 0.5) {
      // Axis 4 moving -> Xbox 360 mode (Pitch=4, Roll=3)
      pair[1] = 4;
      pair[0] = 3;
    } else if (d3 > 0.08 && Math.abs(state.baselines[3] || 0) < 0.5) {
      // Axis 3 moving -> if Pitch is not 4 or 5, it is Pitch
      if (pair[1] !== 4 && (isAxis5Trigger || Math.abs(a5) <= 0.05)) {
        pair[1] = 3;
      }
    }

    if (d2 > 0.08) {
      pair[0] = 2;
    } else if (d3 > 0.08 && pair[1] !== 3) {
      pair[0] = 3;
    }

    // Fallback resting defaults if sticks are centered:
    if (d2 <= 0.05 && d3 <= 0.05 && d4 <= 0.05 && d5 <= 0.05) {
      if (isAxis5Trigger) {
        pair = [2, 3];
      } else {
        pair = [2, 5];
      }
    }

    state.detectedPair = pair;
    var rollVal = gp.axes.length > pair[0] ? gp.axes[pair[0]] : 0;
    var pitchVal = gp.axes.length > pair[1] ? gp.axes[pair[1]] : 0;

    return {
      rawX: rollVal,
      rawY: pitchVal,
      pair: pair
    };
  }

  function updateHUD(gp) {
    var badge = document.getElementById('gp-status-badge');
    var nameEl = document.getElementById('gp-device-name');
    var speedEl = document.getElementById('gp-speed-mode');
    var btnY = document.getElementById('gp-btn-y');
    var btnA = document.getElementById('gp-btn-a');
    var btnX = document.getElementById('gp-btn-x');
    var btnB = document.getElementById('gp-btn-b');
    var stickL = document.getElementById('gp-stick-left-val');
    var stickR = document.getElementById('gp-stick-right-val');
    var pairBadge = document.getElementById('gp-pair-badge');
    var axesMonitor = document.getElementById('gp-axes-monitor');

    if (!gp) {
      if (badge) {
        badge.className = 'px-2 py-0.5 rounded text-[10px] font-mono bg-rose-100 text-rose-700 border border-rose-300 font-bold';
        badge.textContent = 'DISCONNECTED';
      }
      if (nameEl) nameEl.innerHTML = '<span class="text-rose-600 font-bold">🔴 Controller Disconnected / Asleep</span> (Press Home/Power on 8BitDo)';
      return;
    }

    if (badge) {
      badge.className = 'px-2 py-0.5 rounded text-[10px] font-mono bg-emerald-100 text-emerald-800 border border-emerald-300 font-bold';
      badge.textContent = 'ACTIVE';
    }
    if (nameEl) {
      var displayName = gp.id || 'Standard Gamepad';
      if (displayName.length > 28) displayName = displayName.substring(0, 26) + '...';
      nameEl.textContent = displayName;
    }

    if (speedEl) {
      speedEl.textContent = state.speedMode.toUpperCase() + ' (' + state.maxSpeed.toFixed(1) + ' m/s)';
    }

    // Update button highlight indicators
    if (btnY) setBtnActive(btnY, gp.buttons[3] && gp.buttons[3].pressed);
    if (btnA) setBtnActive(btnA, gp.buttons[0] && gp.buttons[0].pressed);
    if (btnX) setBtnActive(btnX, gp.buttons[2] && gp.buttons[2].pressed);
    if (btnB) setBtnActive(btnB, gp.buttons[1] && gp.buttons[1].pressed);

    if (stickL) {
      stickL.textContent = 'Thr: ' + state.axes.vz.toFixed(2) + ' | Yaw: ' + state.axes.yaw_rate.toFixed(2);
      if (Math.abs(state.axes.vz) > 0.05 || Math.abs(state.axes.yaw_rate) > 0.05) {
        stickL.className = 'text-emerald-700 font-bold bg-emerald-50 px-2 py-0.5 rounded border border-emerald-300';
      } else {
        stickL.className = 'text-stone-700 font-bold';
      }
    }
    if (stickR) {
      stickR.textContent = 'Vx (Pitch): ' + state.axes.vx.toFixed(2) + ' | Vy (Roll): ' + state.axes.vy.toFixed(2);
      if (Math.abs(state.axes.vx) > 0.05 || Math.abs(state.axes.vy) > 0.05) {
        stickR.className = 'text-emerald-700 font-black bg-emerald-100 px-2 py-0.5 rounded border border-emerald-400';
      } else {
        stickR.className = 'text-stone-700 font-bold';
      }
    }

    if (pairBadge) {
      var p = state.detectedPair || [2, 5];
      pairBadge.textContent = 'Right Stick: Axes ' + p[0] + ' & ' + p[1];
    }

    if (axesMonitor && gp.axes) {
      var spans = [];
      for (var a = 0; a < Math.min(gp.axes.length, 8); a++) {
        var val = gp.axes[a];
        var isMoving = Math.abs(val) > 0.08;
        var cls = isMoving ? 'bg-emerald-600 text-white font-bold px-1 rounded' : 'text-stone-600';
        spans.push('<span class="' + cls + '">A' + a + ':' + (val >= 0 ? '+' : '') + val.toFixed(2) + '</span>');
      }
      axesMonitor.innerHTML = spans.join(' ');
    }
  }

  function setBtnActive(el, active) {
    if (active) {
      el.classList.add('ring-2', 'ring-brand-500', 'scale-105', 'font-bold');
    } else {
      el.classList.remove('ring-2', 'ring-brand-500', 'scale-105', 'font-bold');
    }
  }

  function pollGamepad() {
    var gp = getActiveGamepad();
    if (!gp) {
      if (state.connected) {
        state.connected = false;
        updateHUD(null);
      }
      return;
    }

    if (!state.connected) {
      state.connected = true;
      console.log('[Gamepad] 8BitDo / Gamepad activated:', gp.id);
      // Sample resting baseline
      if (gp.axes) {
        for (var b = 0; b < gp.axes.length; b++) {
          state.baselines[b] = gp.axes[b];
        }
      }
    }

    // Read Speed Multipliers from Bumpers
    var lb = gp.buttons[4] && gp.buttons[4].pressed;
    var rb = gp.buttons[5] && gp.buttons[5].pressed;
    if (lb && !rb) {
      state.speedMode = 'precision';
      state.maxSpeed = PRECISION_SPEED;
    } else if (rb && !lb) {
      state.speedMode = 'turbo';
      state.maxSpeed = TURBO_SPEED;
    } else {
      state.speedMode = 'normal';
      state.maxSpeed = DEFAULT_SPEED;
    }

    // Button Edge Detection (Only trigger once when button is pushed down)
    handleButtonEdge(gp, 3, function () {
      console.log('[Gamepad] Button Y -> GUIDED mode');
      if (typeof window.triggerFlightAction === 'function') {
        window.triggerFlightAction('guided');
      }
    });

    handleButtonEdge(gp, 0, function () {
      console.log('[Gamepad] Button A -> ARM motors');
      if (typeof window.triggerFlightAction === 'function') {
        window.triggerFlightAction('arm');
      }
    });

    handleButtonEdge(gp, 2, function () {
      console.log('[Gamepad] Button X -> TAKEOFF to 5.0m');
      if (typeof window.triggerTakeoffWithAlt === 'function') {
        window.triggerTakeoffWithAlt(5.0);
      } else if (typeof window.triggerFlightAction === 'function') {
        window.triggerFlightAction('takeoff', 5.0);
      }
    });

    handleButtonEdge(gp, 1, function () {
      console.log('[Gamepad] Button B -> BRAKE / HOVER');
      if (typeof window.triggerFlightAction === 'function') {
        window.triggerFlightAction('hold');
      }
    });

    // D-Pad Camera Pan & Tilt Controls (Buttons 12=UP, 13=DOWN, 14=LEFT, 15=RIGHT)
    handleButtonEdge(gp, 12, function () {
      console.log('[Gamepad] D-pad UP -> Tilt camera UP');
      triggerGimbalAction('tilt_up');
    });
    handleButtonEdge(gp, 13, function () {
      console.log('[Gamepad] D-pad DOWN -> Tilt camera DOWN');
      triggerGimbalAction('tilt_down');
    });
    handleButtonEdge(gp, 14, function () {
      console.log('[Gamepad] D-pad LEFT -> Pan camera LEFT');
      triggerGimbalAction('pan_left');
    });
    handleButtonEdge(gp, 15, function () {
      console.log('[Gamepad] D-pad RIGHT -> Pan camera RIGHT');
      triggerGimbalAction('pan_right');
    });

    // Also support Hat axes for D-pad if exposed as axes[6] & axes[7]
    if (gp.axes && gp.axes.length > 7) {
      var hatX = gp.axes[6] !== undefined ? gp.axes[6] : 0;
      var hatY = gp.axes[7] !== undefined ? gp.axes[7] : 0;
      handleHatEdge('hat_up', hatY < -0.6, function() { triggerGimbalAction('tilt_up'); });
      handleHatEdge('hat_down', hatY > 0.6, function() { triggerGimbalAction('tilt_down'); });
      handleHatEdge('hat_left', hatX < -0.6, function() { triggerGimbalAction('pan_left'); });
      handleHatEdge('hat_right', hatX > 0.6, function() { triggerGimbalAction('pan_right'); });
    }

    // RC Mode 2 Analog Sticks Reading
    // Left Stick: axes[0] = Yaw (Left/Right), axes[1] = Throttle (Up/Down)
    var rawLeftX = gp.axes.length > 0 ? gp.axes[0] : 0;
    var rawLeftY = gp.axes.length > 1 ? gp.axes[1] : 0;

    // Right Stick: Adaptive detection for Roll (Left/Right) & Pitch (Up/Down)
    var rightStick = determineRightStick(gp);
    var rawRightX = rightStick.rawX;
    var rawRightY = rightStick.rawY;

    var normLeftX = applyDeadzone(rawLeftX);
    var normLeftY = applyDeadzone(rawLeftY);
    var normRightX = applyDeadzone(rawRightX);
    var normRightY = applyDeadzone(rawRightY);

    var vx = -normRightY * state.maxSpeed;          // Forward (+X) / Backward (-X)
    var vy = -normRightX * state.maxSpeed;          // Left (+Y) / Right (-Y) Strafe
    var vz = -normLeftY * VERTICAL_SPEED;           // Climb (+Z) / Descend (-Z)
    var yaw_rate = -normLeftX * YAW_SPEED;          // Yaw Rotate

    state.axes = { vx: vx, vy: vy, vz: vz, yaw_rate: yaw_rate };

    var isMoving = (Math.abs(vx) > 0.001 || Math.abs(vy) > 0.001 || Math.abs(vz) > 0.001 || Math.abs(yaw_rate) > 0.001);

    var now = performance.now();
    if (isMoving) {
      state.isActivelyMoving = true;
      // Stream at 25 Hz (every 40ms)
      if (now - state.lastSentTime >= 40) {
        sendTeleopCommand(vx, vy, vz, yaw_rate);
        state.lastSentTime = now;
      }
    } else if (state.isActivelyMoving) {
      // Sent stop command once when sticks return to center
      sendTeleopCommand(0.0, 0.0, 0.0, 0.0);
      state.isActivelyMoving = false;
      state.lastSentTime = now;
    }

    updateHUD(gp);
  }

  function handleButtonEdge(gp, btnIndex, callback) {
    var pressed = gp.buttons[btnIndex] && gp.buttons[btnIndex].pressed;
    var wasPressed = state.prevButtons[btnIndex] || false;
    if (pressed && !wasPressed) {
      callback();
    }
    state.prevButtons[btnIndex] = pressed;
  }

  function handleHatEdge(name, pressed, callback) {
    var wasPressed = state.prevHats[name] || false;
    if (pressed && !wasPressed) {
      callback();
    }
    state.prevHats[name] = pressed;
  }

  function sendTeleopCommand(vx, vy, vz, yaw_rate) {
    var payload = {
      type: 'teleop_cmd',
      vx: roundTo(vx, 3),
      vy: roundTo(vy, 3),
      vz: roundTo(vz, 3),
      yaw_rate: roundTo(yaw_rate, 3)
    };

    // 1. Send via WebSocket if open
    if (window.wsBridge && window.wsBridge.socket && window.wsBridge.socket.readyState === WebSocket.OPEN) {
      try {
        window.wsBridge.socket.send(JSON.stringify(payload));
        return;
      } catch (e) {}
    } else if (window.AeroVIOWebSocket && typeof window.AeroVIOWebSocket.send === 'function') {
      try {
        window.AeroVIOWebSocket.send(JSON.stringify(payload));
        return;
      } catch (e) {}
    }

    // 2. Fallback to POST /api/teleop
    if (window.fetch) {
      fetch('/api/teleop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      }).catch(function () {});
    }
  }

  function triggerGimbalAction(action) {
    var payload = { type: 'gimbal_cmd', action: action };
    if (window.wsBridge && window.wsBridge.socket && window.wsBridge.socket.readyState === WebSocket.OPEN) {
      window.wsBridge.socket.send(JSON.stringify(payload));
    } else if (window.fetch) {
      fetch('/api/gimbal', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: action })
      }).catch(function () {});
    }
  }

  function setGimbalAngles(pitch, yaw) {
    var payload = { type: 'gimbal_cmd', pitch: pitch, yaw: yaw };
    if (window.wsBridge && window.wsBridge.socket && window.wsBridge.socket.readyState === WebSocket.OPEN) {
      window.wsBridge.socket.send(JSON.stringify(payload));
    } else if (window.fetch) {
      fetch('/api/gimbal', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pitch: pitch, yaw: yaw })
      }).catch(function () {});
    }
  }

  function roundTo(val, decimals) {
    var mult = Math.pow(10, decimals);
    return Math.round(val * mult) / mult;
  }

  // Hook into Takeoff helper
  window.triggerTakeoffWithAlt = function (alt) {
    var inputAlt = document.getElementById('input-takeoff-alt');
    if (inputAlt) inputAlt.value = alt;
    if (typeof window.triggerTakeoff === 'function') {
      window.triggerTakeoff();
    } else if (typeof window.triggerFlightAction === 'function') {
      window.triggerFlightAction('takeoff', alt);
    }
  };

  // Keyboard Teleop Fallback (Standard ROS teleop_twist_keyboard + WASD + Arrows)
  var keyState = { vx: 0, vy: 0, vz: 0, yaw_rate: 0, active: false };
  window.addEventListener('keydown', function(e) {
    if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA')) return;
    var k = e.key;
    var kl = k.toLowerCase();
    var spd = state.maxSpeed;
    var handled = true;

    // Forward (+X)
    if (kl === 'i' || kl === 'w' || e.key === 'ArrowUp') keyState.vx = spd;
    // Backward (-X)
    else if (k === ',' || kl === 's' || e.key === 'ArrowDown') keyState.vx = -spd;
    // Strafe Left (+Y)
    else if (k === 'J' || kl === 'a' || e.key === 'ArrowLeft') keyState.vy = spd;
    // Strafe Right (-Y)
    else if (k === 'L' || kl === 'd' || e.key === 'ArrowRight') keyState.vy = -spd;
    // Yaw Left (+Yaw)
    else if (k === 'j' || kl === 'q') keyState.yaw_rate = YAW_SPEED;
    // Yaw Right (-Yaw)
    else if (k === 'l' || kl === 'e') keyState.yaw_rate = -YAW_SPEED;
    // Diagonal Turn + Forward / Backward
    else if (k === 'u') { keyState.vx = spd; keyState.yaw_rate = YAW_SPEED; }
    else if (k === 'o') { keyState.vx = spd; keyState.yaw_rate = -YAW_SPEED; }
    else if (k === 'm') { keyState.vx = -spd; keyState.yaw_rate = YAW_SPEED; }
    else if (k === '.') { keyState.vx = -spd; keyState.yaw_rate = -YAW_SPEED; }
    // Holonomic Diagonal Strafe (Shift)
    else if (k === 'U') { keyState.vx = spd; keyState.vy = spd; }
    else if (k === 'O') { keyState.vx = spd; keyState.vy = -spd; }
    else if (k === 'M') { keyState.vx = -spd; keyState.vy = spd; }
    else if (k === '>') { keyState.vx = -spd; keyState.vy = -spd; }
    // Altitude Climb (+Z) and Descend (-Z)
    else if (kl === 't') keyState.vz = VERTICAL_SPEED;
    else if (kl === 'b' || kl === 'z') keyState.vz = -VERTICAL_SPEED;
    // Stop / Hover Hold
    else if (kl === 'k' || k === ' ' || kl === 'h') {
      keyState.vx = 0; keyState.vy = 0; keyState.vz = 0; keyState.yaw_rate = 0;
      if (typeof window.triggerFlightAction === 'function') window.triggerFlightAction('hold');
    }
    // Vehicle shortcuts (matching 8BitDo buttons Y, A, X, B)
    else if (kl === 'y' || kl === 'g') {
      if (typeof window.triggerFlightAction === 'function') window.triggerFlightAction('guided');
    } else if (kl === 'x') {
      if (typeof window.triggerTakeoffWithAlt === 'function') window.triggerTakeoffWithAlt(5.0);
    } else if (k === 'A' || (kl === 'm' && !e.shiftKey)) {
      if (typeof window.triggerFlightAction === 'function') window.triggerFlightAction('arm');
    } else {
      handled = false;
    }

    if (handled) {
      e.preventDefault();
      keyState.active = true;
      sendTeleopCommand(keyState.vx, keyState.vy, keyState.vz, keyState.yaw_rate);
    }
  });

  window.addEventListener('keyup', function(e) {
    if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA')) return;
    var k = e.key;
    var kl = k.toLowerCase();
    var handled = true;

    if (kl === 'i' || kl === 'w' || e.key === 'ArrowUp' || k === ',' || kl === 's' || e.key === 'ArrowDown') keyState.vx = 0;
    else if (k === 'J' || k === 'L' || kl === 'a' || kl === 'd' || e.key === 'ArrowLeft' || e.key === 'ArrowRight') keyState.vy = 0;
    else if (k === 'j' || k === 'l' || kl === 'q' || kl === 'e') keyState.yaw_rate = 0;
    else if (kl === 'u' || kl === 'o' || kl === 'm' || k === '.') { keyState.vx = 0; keyState.yaw_rate = 0; keyState.vy = 0; }
    else if (k === 'U' || k === 'O' || k === 'M' || k === '>') { keyState.vx = 0; keyState.vy = 0; }
    else if (kl === 't' || kl === 'b' || kl === 'z') keyState.vz = 0;
    else handled = false;

    if (handled && keyState.active) {
      if (keyState.vx === 0 && keyState.vy === 0 && keyState.vz === 0 && keyState.yaw_rate === 0) {
        keyState.active = false;
        sendTeleopCommand(0.0, 0.0, 0.0, 0.0);
      } else {
        sendTeleopCommand(keyState.vx, keyState.vy, keyState.vz, keyState.yaw_rate);
      }
    }
  });

  // Event Listeners
  window.addEventListener('gamepadconnected', function (e) {
    console.log('[Gamepad] Connected:', e.gamepad.id, 'at index', e.gamepad.index);
    state.connected = true;
    state.gamepadIndex = e.gamepad.index;
    if (e.gamepad.axes) {
      for (var i = 0; i < e.gamepad.axes.length; i++) {
        state.baselines[i] = e.gamepad.axes[i];
      }
    }
    updateHUD(e.gamepad);
  });

  window.addEventListener('gamepaddisconnected', function (e) {
    console.log('[Gamepad] Disconnected:', e.gamepad.id);
    if (state.gamepadIndex === e.gamepad.index) {
      state.connected = false;
      state.gamepadIndex = null;
      updateHUD(null);
    }
  });

  // Start polling loop at 50 Hz
  setInterval(pollGamepad, 20);

  // Expose Global API
  window.AeroVIOGamepad = {
    getState: function () { return state; },
    setAxisMode: function (mode) {
      state.axisMode = mode;
      try {
        localStorage.setItem('aerovio_gamepad_axis_mode', mode);
      } catch (e) {}
      if (mode === 'pair_3_4' || mode === 'xpad') state.detectedPair = [3, 4];
      else if (mode === 'pair_2_3' || mode === 'standard') state.detectedPair = [2, 3];
      else if (mode === 'pair_2_5' || mode === 'bt_linux') state.detectedPair = [2, 5];
      else state.detectedPair = [2, 3];
      console.log('[Gamepad] Axis mode set to:', mode, 'Pair:', state.detectedPair);
      var badge = document.getElementById('gp-pair-badge');
      if (badge && state.detectedPair) {
        badge.textContent = 'Right Stick: Axes ' + state.detectedPair[0] + ' & ' + state.detectedPair[1];
      }
      var sel = document.getElementById('gp-axis-mode');
      if (sel && sel.value !== mode) sel.value = mode;
    },
    poll: pollGamepad
  };
  window.triggerGimbalAction = triggerGimbalAction;
  window.setGimbalAngles = setGimbalAngles;

  console.log('[AeroVIO] 8BitDo Ultimate 2C Gamepad & Gimbal Engine initialized.');
})();
