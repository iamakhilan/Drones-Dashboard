/**
 * ============================================================================
 * AeroVIO Ground Control Station - 50 Hz Cascaded Controller Subsystem
 * Matches the drones-controller ROS 2 architecture:
 * Safety Watchdog -> Position Loop -> Attitude Gen -> SO(3) Controller -> FSM
 * ============================================================================
 */

(function(window) {
  'use strict';

  class FlightController {
    constructor() {
      this.elements = {
        statusBadge: null,
        fsmStateText: null,
        fsmPill: null,
        rateBadge: null,
        pipelineText: null,
        
        // Position Error & Force (Outer Loop)
        posErrorNorm: null,
        posErrorX: null,
        posErrorY: null,
        posErrorZ: null,
        velErrorNorm: null,
        forceNorm: null,
        forceX: null,
        forceY: null,
        forceZ: null,

        // Attitude Generator
        thrustTd: null,
        b3dVec: null,
        desiredQuat: null,

        // SO(3) Geometric Controller (Inner Loop)
        attErrorNorm: null,
        attErrorX: null,
        attErrorY: null,
        attErrorZ: null,
        rateErrorNorm: null,
        rateErrorX: null,
        rateErrorY: null,
        rateErrorZ: null,
        momentNorm: null,
        momentX: null,
        momentY: null,
        momentZ: null,

        // Safety & Watchdog
        watchdogStatus: null,
        commandOutput: null,
        armedBadge: null,

        // Modal inputs
        setpointX: null,
        setpointY: null,
        setpointZ: null,
        setpointYaw: null,
        wsEndpointInput: null
      };

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this._bindElements();

      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.update(state));
      }
    }

    _bindElements() {
      // Main Card Elements
      this.elements.statusBadge = document.getElementById('ctrl-status-badge');
      this.elements.fsmStateText = document.getElementById('ctrl-fsm-state');
      this.elements.fsmPill = document.getElementById('ctrl-fsm-pill');
      this.elements.rateBadge = document.getElementById('ctrl-rate-badge');
      
      this.elements.posErrorNorm = document.getElementById('ctrl-pos-err-norm');
      this.elements.attErrorNorm = document.getElementById('ctrl-att-err-norm');
      this.elements.forceNorm = document.getElementById('ctrl-force-norm');
      this.elements.momentNorm = document.getElementById('ctrl-moment-norm');

      // Modal Elements
      this.elements.modalFsmState = document.getElementById('modal-fsm-state');
      this.elements.posErrorX = document.getElementById('modal-ep-x');
      this.elements.posErrorY = document.getElementById('modal-ep-y');
      this.elements.posErrorZ = document.getElementById('modal-ep-z');
      this.elements.modalPosNorm = document.getElementById('modal-ep-norm');

      this.elements.velErrorNorm = document.getElementById('modal-ev-norm');
      this.elements.forceX = document.getElementById('modal-fd-x');
      this.elements.forceY = document.getElementById('modal-fd-y');
      this.elements.forceZ = document.getElementById('modal-fd-z');
      this.elements.modalForceNorm = document.getElementById('modal-fd-norm');

      this.elements.thrustTd = document.getElementById('modal-td-thrust');
      this.elements.b3dVec = document.getElementById('modal-b3d-vec');
      this.elements.desiredQuat = document.getElementById('modal-qd-quat');

      this.elements.attErrorX = document.getElementById('modal-er-x');
      this.elements.attErrorY = document.getElementById('modal-er-y');
      this.elements.attErrorZ = document.getElementById('modal-er-z');
      this.elements.modalAttNorm = document.getElementById('modal-er-norm');

      this.elements.rateErrorX = document.getElementById('modal-ew-x');
      this.elements.rateErrorY = document.getElementById('modal-ew-y');
      this.elements.rateErrorZ = document.getElementById('modal-ew-z');
      this.elements.rateErrorNorm = document.getElementById('modal-ew-norm');

      this.elements.momentX = document.getElementById('modal-md-x');
      this.elements.momentY = document.getElementById('modal-md-y');
      this.elements.momentZ = document.getElementById('modal-md-z');
      this.elements.modalMomentNorm = document.getElementById('modal-md-norm');

      this.elements.watchdogStatus = document.getElementById('modal-watchdog-status');
      this.elements.commandOutput = document.getElementById('modal-cmd-output');
      this.elements.armedBadge = document.getElementById('modal-armed-badge');

      this.elements.setpointX = document.getElementById('modal-sp-x');
      this.elements.setpointY = document.getElementById('modal-sp-y');
      this.elements.setpointZ = document.getElementById('modal-sp-z');
      this.elements.setpointYaw = document.getElementById('modal-sp-yaw');
      this.elements.wsEndpointInput = document.getElementById('modal-ws-url-input');

      // Populate default endpoint input
      if (this.elements.wsEndpointInput && window.APP_CONFIG) {
        this.elements.wsEndpointInput.value = window.APP_CONFIG.websocketUrl;
      }
    }

    update(state) {
      const c = state.controller;
      const so3 = c.so3_controller || {};
      const ag = c.attitude_generator || {};
      const ep = c.position_error || {};
      const ev = c.velocity_error || {};
      const fd = c.desired_force || {};

      // 1. Controller Status & FSM State
      const fsm = c.flight_state || 'IDLE';
      if (this.elements.statusBadge) {
        this.elements.statusBadge.textContent = c.status || '50 Hz CLOSED LOOP';
      }
      if (this.elements.fsmStateText) {
        this.elements.fsmStateText.textContent = `STATE: ${fsm}`;
      }
      if (this.elements.fsmPill) {
        this.elements.fsmPill.textContent = fsm;
        this.elements.fsmPill.className = `font-mono text-xs px-2 py-0.5 rounded font-bold uppercase ${fsm === 'FAILSAFE' ? 'bg-red-500/20 text-red-400 border border-red-500/40' : (fsm === 'NAVIGATING' ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40' : 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/40')}`;
      }
      if (this.elements.modalFsmState) {
        this.elements.modalFsmState.textContent = fsm;
      }

      // Update horizontal FSM progression pills in modal
      this._updateFsmProgression(fsm);

      // 2. Position Loop Telemetry (Outer Loop)
      if (this.elements.posErrorNorm) {
        this.elements.posErrorNorm.textContent = ep.norm !== null ? `${ep.norm.toFixed(2)} m` : 'N/A';
      }
      if (this.elements.posErrorX) this.elements.posErrorX.textContent = ep.x !== null ? `${(ep.x >= 0 ? '+' : '') + ep.x.toFixed(3)} m` : 'N/A';
      if (this.elements.posErrorY) this.elements.posErrorY.textContent = ep.y !== null ? `${(ep.y >= 0 ? '+' : '') + ep.y.toFixed(3)} m` : 'N/A';
      if (this.elements.posErrorZ) this.elements.posErrorZ.textContent = ep.z !== null ? `${(ep.z >= 0 ? '+' : '') + ep.z.toFixed(3)} m` : 'N/A';
      if (this.elements.modalPosNorm) this.elements.modalPosNorm.textContent = ep.norm !== null ? `${ep.norm.toFixed(3)} m` : 'N/A';

      if (this.elements.velErrorNorm) this.elements.velErrorNorm.textContent = ev.norm !== null ? `${ev.norm.toFixed(3)} m/s` : 'N/A';

      // Desired Force F_d
      if (this.elements.forceNorm) {
        this.elements.forceNorm.textContent = fd.norm !== null ? `${fd.norm.toFixed(2)} N` : 'N/A';
      }
      if (this.elements.forceX) this.elements.forceX.textContent = fd.x !== null ? `${(fd.x >= 0 ? '+' : '') + fd.x.toFixed(2)} N` : 'N/A';
      if (this.elements.forceY) this.elements.forceY.textContent = fd.y !== null ? `${(fd.y >= 0 ? '+' : '') + fd.y.toFixed(2)} N` : 'N/A';
      if (this.elements.forceZ) this.elements.forceZ.textContent = fd.z !== null ? `${(fd.z >= 0 ? '+' : '') + fd.z.toFixed(2)} N` : 'N/A';
      if (this.elements.modalForceNorm) this.elements.modalForceNorm.textContent = fd.norm !== null ? `${fd.norm.toFixed(2)} N` : 'N/A';

      // 3. Attitude Generator Telemetry
      if (this.elements.thrustTd) {
        this.elements.thrustTd.textContent = ag.collective_thrust !== null ? `${ag.collective_thrust.toFixed(2)} N` : 'N/A';
      }
      if (this.elements.b3dVec && ag.b3d) {
        this.elements.b3dVec.textContent = `[${ag.b3d.x.toFixed(3)}, ${ag.b3d.y.toFixed(3)}, ${ag.b3d.z.toFixed(3)}]`;
      }
      if (this.elements.desiredQuat && ag.desired_attitude) {
        const q = ag.desired_attitude;
        this.elements.desiredQuat.textContent = `[${(q.qw||1).toFixed(3)}, ${(q.qx||0).toFixed(3)}, ${(q.qy||0).toFixed(3)}, ${(q.qz||0).toFixed(3)}]`;
      }

      // 4. SO(3) Geometric Attitude Controller Telemetry (Inner Loop)
      const er = so3.attitude_error || {};
      const ew = so3.angular_velocity_error || {};
      const md = so3.desired_moment || {};

      if (this.elements.attErrorNorm) {
        this.elements.attErrorNorm.textContent = er.norm !== null ? `${(er.norm * (180 / Math.PI)).toFixed(2)}°` : 'N/A';
      }
      if (this.elements.attErrorX) this.elements.attErrorX.textContent = er.x !== null ? `${er.x.toFixed(4)} rad` : 'N/A';
      if (this.elements.attErrorY) this.elements.attErrorY.textContent = er.y !== null ? `${er.y.toFixed(4)} rad` : 'N/A';
      if (this.elements.attErrorZ) this.elements.attErrorZ.textContent = er.z !== null ? `${er.z.toFixed(4)} rad` : 'N/A';
      if (this.elements.modalAttNorm) this.elements.modalAttNorm.textContent = er.norm !== null ? `${er.norm.toFixed(4)} rad (${(er.norm * (180/Math.PI)).toFixed(2)}°)` : 'N/A';

      if (this.elements.rateErrorX) this.elements.rateErrorX.textContent = ew.x !== null ? `${ew.x.toFixed(4)} rad/s` : 'N/A';
      if (this.elements.rateErrorY) this.elements.rateErrorY.textContent = ew.y !== null ? `${ew.y.toFixed(4)} rad/s` : 'N/A';
      if (this.elements.rateErrorZ) this.elements.rateErrorZ.textContent = ew.z !== null ? `${ew.z.toFixed(4)} rad/s` : 'N/A';
      if (this.elements.rateErrorNorm) this.elements.rateErrorNorm.textContent = ew.norm !== null ? `${ew.norm.toFixed(4)} rad/s` : 'N/A';

      // Desired Moment M_d
      if (this.elements.momentNorm) {
        this.elements.momentNorm.textContent = md.norm !== null ? `${md.norm.toFixed(3)} N·m` : 'N/A';
      }
      if (this.elements.momentX) this.elements.momentX.textContent = md.x !== null ? `${(md.x >= 0 ? '+' : '') + md.x.toFixed(4)} N·m` : 'N/A';
      if (this.elements.momentY) this.elements.momentY.textContent = md.y !== null ? `${(md.y >= 0 ? '+' : '') + md.y.toFixed(4)} N·m` : 'N/A';
      if (this.elements.momentZ) this.elements.momentZ.textContent = md.z !== null ? `${(md.z >= 0 ? '+' : '') + md.z.toFixed(4)} N·m` : 'N/A';
      if (this.elements.modalMomentNorm) this.elements.modalMomentNorm.textContent = md.norm !== null ? `${md.norm.toFixed(4)} N·m` : 'N/A';

      // 5. Safety & Watchdog
      if (this.elements.watchdogStatus) {
        this.elements.watchdogStatus.textContent = c.watchdog_status || 'OK';
        this.elements.watchdogStatus.className = `font-bold ${c.failsafe ? 'text-red-500' : 'text-emerald-400'}`;
      }
      if (this.elements.commandOutput) {
        this.elements.commandOutput.textContent = c.command_output || 'ENABLED';
      }
      if (this.elements.armedBadge) {
        this.elements.armedBadge.textContent = c.armed ? 'ARMED' : 'DISARMED';
        this.elements.armedBadge.className = `px-2 py-0.5 rounded font-mono font-bold text-xs ${c.armed ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40' : 'bg-stone-700 text-stone-300'}`;
      }
    }

    _updateFsmProgression(activeState) {
      const states = ['IDLE', 'ARMING', 'TAKEOFF', 'NAVIGATING', 'HOVER', 'FAILSAFE'];
      states.forEach(s => {
        const el = document.getElementById(`fsm-node-${s.toLowerCase()}`);
        if (el) {
          const isActive = (s === activeState);
          if (s === 'FAILSAFE' && isActive) {
            el.className = 'px-2 py-1 bg-red-600 text-white font-bold rounded shadow-[0_0_10px_rgba(239,68,68,0.5)]';
          } else if (isActive) {
            el.className = 'px-2 py-1 bg-emerald-500 text-black font-bold rounded shadow-[0_0_10px_rgba(16,185,129,0.5)]';
          } else {
            el.className = 'px-2 py-1 bg-stone-800 text-stone-400 rounded hover:text-stone-200';
          }
        }
      });
    }

    /**
     * Dispatch target setpoint to ROS 2 controller node
     */
    applySetpoint() {
      const x = parseFloat(this.elements.setpointX?.value) || 20.0;
      const y = parseFloat(this.elements.setpointY?.value) || 5.0;
      const z = parseFloat(this.elements.setpointZ?.value) || 20.0;
      const yaw = parseFloat(this.elements.setpointYaw?.value) || 2.0;

      const target = { x, y, z, yaw };
      console.log('[drones-controller] Dispatching setpoint:', target);

      if (window.telemetry) {
        window.telemetry.state.controller.desired_position = target;
      }

      if (window.wsBridge) {
        window.wsBridge.sendCommand({
          action: 'set_target_setpoint',
          setpoint: target
        });
      }

      if (window.ui) {
        window.ui.showToast(`Target Setpoint Dispatched: [${x}, ${y}, ${z}]m, Yaw: ${yaw} rad`, 'success');
      }
    }

    /**
     * Update Configured WebSocket Endpoint
     */
    updateWsEndpoint() {
      const newUrl = this.elements.wsEndpointInput?.value.trim();
      if (!newUrl) return;

      window.APP_CONFIG = window.APP_CONFIG || {};
      window.APP_CONFIG.websocketUrl = newUrl;

      if (window.wsBridge) {
        window.wsBridge.disconnect();
        window.wsBridge.connect(newUrl);
      }

      if (window.ui) {
        window.ui.showToast(`WebSocket Bridge Endpoint updated to: ${newUrl}`, 'info');
      }
    }
  }

  // Expose singleton on window
  window.controller = new FlightController();

})(window);
