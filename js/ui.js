/**
 * ============================================================================
 * AeroVIO Ground Control Station - Central UI & 3D Spatial Map Controller
 * Orchestrates topbar mode switches (LIVE vs DEMO), dataset selectors,
 * telemetry gauges, 3D trajectory layers (T_WB vs T_WC), modals, and toasts.
 * Patch: audit fixes — id mappings, projections, pointer/touch/pinch,
 * devicePixelRatio, stale-warning-banner, drawer+overlay, IntersectionObserver,
 * modal id search, dataset ?dataset= persistence, empty states, Undo CLEAR.
 * ============================================================================
 */

(function(window) {
  'use strict';

  // Ensure stale-warning-banner exists (audit: wire stale-warning-banner)
  function ensureStaleBanner() {
    var existing = document.getElementById('stale-warning-banner');
    if (existing) return existing;
    var banner = document.createElement('div');
    banner.id = 'stale-warning-banner';
    banner.className = 'hidden bg-amber-100 border border-amber-300 text-amber-900 text-xs font-mono px-4 py-2 flex items-center justify-between';
    banner.innerHTML = '<span id="stale-warning-text" class="font-bold">TELEMETRY STALE — NO ROS 2 PACKETS RECEIVED</span><button onclick="this.parentElement.classList.add(\'hidden\')" class="ml-3 text-amber-700 hover:text-amber-900 font-bold">×</button>';
    // insert after header or at top of main
    var main = document.querySelector('main') || document.body;
    if (main && main.parentElement) {
      var header = document.querySelector('header');
      if (header && header.nextElementSibling) header.insertAdjacentElement('afterend', banner);
      else document.body.prepend(banner);
    } else {
      document.body.prepend(banner);
    }
    return banner;
  }

  // Ensure top-endpoint-badge exists (audit: sync with telemetry header badge)
  function ensureEndpointBadge() {
    var existing = document.getElementById('top-endpoint-badge');
    if (existing) return existing;
    var badge = document.createElement('span');
    badge.id = 'top-endpoint-badge';
    badge.className = 'hidden md:inline text-[11px] font-mono text-stone-400 truncate max-w-[220px]';
    var parent = document.querySelector('header .flex.items-center.space-x-3');
    if (parent) parent.appendChild(badge);
    else {
      var hdr = document.querySelector('header');
      if (hdr) hdr.appendChild(badge);
    }
    if (window.APP_CONFIG && window.APP_CONFIG.websocketUrl) badge.textContent = window.APP_CONFIG.websocketUrl;
    return badge;
  }

  class UIController {
    constructor() {
      // 3D Map View State
      this.mapCanvas = null;
      this.mapCtx = null;
      // Also support SVG spatial canvas
      this.svgSpatial = null;
      this.projectionMode = 'ISOMETRIC'; // 'ISOMETRIC' | 'TOP_DOWN' | 'CAM_FOLLOW'
      this.layers = {
        bodyTrajectory: true,      // T_WB
        cameraTrajectory: true,    // T_WC
        landmarks: true,           // 3D PCD points
        frustums: true,
        axes: true
      };

      // Camera rotation / zoom in 3D canvas
      this.mapAngle = 0.52; // Radians
      this.mapPitch = 0.42;
      this.mapScale = 14.0;
      this.isDraggingMap = false;
      this.lastMousePos = { x: 0, y: 0 };
      this.pinchStartDist = 0;
      this.pinchStartScale = 14.0;

      // Rosbag Recording State
      this.isRecordingRosbag = true;
      this.rosbagStartTime = Date.now() - (14 * 60 + 22) * 1000;

      // Undo for CLEAR
      this.clearUndoSnapshot = null;
      this.clearUndoTimer = null;

      // DOM Elements
      this.elements = {};
      this._activeNavId = null;
      this.drawerOpen = false;

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      ensureStaleBanner();
      ensureEndpointBadge();
      this._bindElements();
      this._bindEvents();
      this._initDatasetPersistence();
      this._initDrawer();
      this._initActiveNavObserver();
      this._init3DMap();
      this._startClock();
      this._syncEndpointBadge();
      this._renderEmptyStates();

      // Hook Telemetry Engine
      if (window.telemetry) {
        window.telemetry.onTelemetry((state) => this.onTelemetryUpdate(state));
        window.telemetry.onStale((staleInfo) => this.onStaleUpdate(staleInfo));
        window.telemetry.onHealth((health) => this.onHealthUpdate(health));
        window.telemetry.onStateChange((state) => this.onStateChange(state));
      }

      // In LIVE mode, hide playback controls (scrubber and dataset selector)
      if (window.APP_CONFIG && window.APP_CONFIG.mode === 'LIVE') {
        const scrub = document.getElementById('scrubber-bar');
        if (scrub) scrub.classList.add('hidden');
        const dsBar = document.getElementById('dataset-selector-bar');
        if (dsBar) dsBar.classList.add('hidden');
      }
    }

    _bindElements() {
      this.elements.utcClock = document.getElementById('utc-clock');
      this.elements.missionStatus = document.getElementById('top-mission-status');
      this.elements.rosConnBadge = document.getElementById('top-ros-badge');
      this.elements.rosConnText = document.getElementById('top-ros-text');
      this.elements.rosConnDot = document.getElementById('top-ros-dot');
      // FIX: dataset-selector -> dataset-selector-bar
      this.elements.modeSelector = document.getElementById('dataset-selector-bar') || document.getElementById('dataset-selector');
      this.elements.staleBanner = document.getElementById('stale-warning-banner');
      if (!this.elements.staleBanner) this.elements.staleBanner = ensureStaleBanner();
      this.elements.staleText = document.getElementById('stale-warning-text');
      this.elements.rosbagBtn = document.getElementById('rosbag-btn');
      this.elements.rosbagTimer = document.getElementById('rosbag-timer');
      // FIX: map-canvas-3d -> svg-spatial-canvas
      this.elements.mapCanvas = document.getElementById('svg-spatial-canvas') || document.getElementById('map-canvas-3d');
      // Keep svg ref for projection helpers
      this.elements.svgCanvas = document.getElementById('svg-spatial-canvas');
      this.elements.mapPointCount = document.getElementById('map-point-count') || document.getElementById('hud-map-pts') || document.getElementById('sidebar-pointcount');
      this.elements.mapDronePosText = document.getElementById('map-drone-pos-text') || document.getElementById('hud-pw');
      this.elements.topEndpointBadge = document.getElementById('top-endpoint-badge');

      // Health gauges
      this.elements.healthCamera = document.getElementById('health-camera-badge');
      this.elements.healthImu = document.getElementById('health-imu-badge');
      this.elements.healthVio = document.getElementById('health-vio-badge');
      this.elements.healthController = document.getElementById('health-controller-badge');
      this.elements.healthWs = document.getElementById('health-ws-badge');

      // Main Instrumentation Gauges
      this.elements.gaugePosX = document.getElementById('gauge-pos-x') || document.getElementById('tel-pos-wb');
      this.elements.gaugePosY = document.getElementById('gauge-pos-y');
      this.elements.gaugePosZ = document.getElementById('gauge-pos-z');
      this.elements.gaugePosTotal = document.getElementById('gauge-pos-total') || document.getElementById('kpi-displacement');
      this.elements.gaugeAltBar = document.getElementById('gauge-alt-bar');

      this.elements.gaugeVelVx = document.getElementById('gauge-vel-vx');
      this.elements.gaugeVelVy = document.getElementById('gauge-vel-vy');
      this.elements.gaugeVelVz = document.getElementById('gauge-vel-vz');
      this.elements.gaugeVelTotal = document.getElementById('gauge-vel-total');
      this.elements.gaugeVelBar = document.getElementById('gauge-vel-bar');

      this.elements.gaugeAttRoll = document.getElementById('gauge-att-roll');
      this.elements.gaugeAttPitch = document.getElementById('gauge-att-pitch');
      this.elements.gaugeAttYaw = document.getElementById('gauge-att-yaw');
      this.elements.gaugeAttQuat = document.getElementById('gauge-att-quat') || document.getElementById('tel-quat');

      this.elements.toastContainer = document.getElementById('toast-container') || document.getElementById('toast-box');
    }

    _bindEvents() {
      // Keyboard shortcuts — fix modal id search: support both search-modal and modal-search
      window.addEventListener('keydown', (e) => {
        if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
          e.preventDefault();
          this.toggleModal('search-modal');
        } else if (e.key === 'Escape') {
          this.closeAllModals();
          this._closeDrawer();
        }
      });

      // 3D Canvas pointer/touch/pinch handlers for 3D view
      var canvasEl = this.elements.mapCanvas || this.elements.svgCanvas;
      if (canvasEl) {
        // Pointer events (covers mouse + pen)
        canvasEl.addEventListener('pointerdown', (e) => {
          try { canvasEl.setPointerCapture(e.pointerId); } catch(err) {}
          this.isDraggingMap = true;
          this.lastMousePos = { x: e.clientX, y: e.clientY };
          if (canvasEl.style) canvasEl.style.cursor = 'grabbing';
        });
        window.addEventListener('pointermove', (e) => {
          if (!this.isDraggingMap) return;
          var dx = e.clientX - this.lastMousePos.x;
          var dy = e.clientY - this.lastMousePos.y;
          this.lastMousePos = { x: e.clientX, y: e.clientY };
          this.mapAngle += dx * 0.008;
          this.mapPitch = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, this.mapPitch + dy * 0.006));
        });
        window.addEventListener('pointerup', (e) => {
          this.isDraggingMap = false;
          if (canvasEl.style) canvasEl.style.cursor = '';
          try { canvasEl.releasePointerCapture(e.pointerId); } catch(err) {}
        });
        canvasEl.addEventListener('pointercancel', () => { this.isDraggingMap = false; });

        // Touch fallback (for older browsers) — also handles pinch
        canvasEl.addEventListener('touchstart', (e) => {
          if (e.touches.length === 1) {
            this.isDraggingMap = true;
            this.lastMousePos = { x: e.touches[0].clientX, y: e.touches[0].clientY };
          } else if (e.touches.length === 2) {
            this.isDraggingMap = false;
            var dx = e.touches[0].clientX - e.touches[1].clientX;
            var dy = e.touches[0].clientY - e.touches[1].clientY;
            this.pinchStartDist = Math.hypot(dx, dy);
            this.pinchStartScale = this.mapScale;
          }
        }, { passive: true });
        canvasEl.addEventListener('touchmove', (e) => {
          if (e.touches.length === 1 && this.isDraggingMap) {
            e.preventDefault();
            var dx = e.touches[0].clientX - this.lastMousePos.x;
            var dy = e.touches[0].clientY - this.lastMousePos.y;
            this.lastMousePos = { x: e.touches[0].clientX, y: e.touches[0].clientY };
            this.mapAngle += dx * 0.008;
            this.mapPitch = Math.max(0.1, Math.min(Math.PI / 2 - 0.05, this.mapPitch + dy * 0.006));
          } else if (e.touches.length === 2) {
            e.preventDefault();
            var dx2 = e.touches[0].clientX - e.touches[1].clientX;
            var dy2 = e.touches[0].clientY - e.touches[1].clientY;
            var dist = Math.hypot(dx2, dy2);
            if (this.pinchStartDist > 0) {
              var factor = dist / this.pinchStartDist;
              // pinch scale: factor>1 zoom in (increase scale), <1 zoom out
              this.mapScale = Math.max(5.0, Math.min(50.0, this.pinchStartScale * factor));
            }
          }
        }, { passive: false });
        canvasEl.addEventListener('touchend', (e) => {
          if (e.touches.length === 0) this.isDraggingMap = false;
          if (e.touches.length < 2) this.pinchStartDist = 0;
        }, { passive: true });

        canvasEl.addEventListener('wheel', (e) => {
          e.preventDefault();
          this.mapScale = Math.max(5.0, Math.min(50.0, this.mapScale - Math.sign(e.deltaY) * 1.5));
        }, { passive: false });

        // prevent context menu on right drag
        canvasEl.addEventListener('contextmenu', (e) => e.preventDefault());
      }

      // Modal click-outside to close
      document.addEventListener('click', (e) => {
        var searchModal = document.getElementById('search-modal') || document.getElementById('modal-search');
        if (searchModal && !searchModal.classList.contains('hidden') && e.target === searchModal) {
          this.closeAllModals();
        }
        var tuningModal = document.getElementById('tuning-modal') || document.getElementById('modal-controller') || document.getElementById('modal-vio');
        if (tuningModal && !tuningModal.classList.contains('hidden') && e.target === tuningModal) {
          this.closeAllModals();
        }
        // drawer overlay click handled separately
      });
    }

    // Dataset persistence via history.replaceState and ?dataset=
    _initDatasetPersistence() {
      try {
        var params = new URLSearchParams(window.location.search);
        var ds = params.get('dataset');
        if (ds) {
          // sanitize to known keys
          var norm = ds.replace(/[^a-zA-Z0-9_\-]/g,'');
          // map short keys
          var map = { orbit: 'orbit', hover: 'hover', straight: 'straight', multi_axis: 'multi_axis', stress: 'stress',
                      synthetic_orbit: 'orbit', synthetic_hover: 'hover', synthetic_straight: 'straight', synthetic_multi_axis: 'multi_axis', synthetic_stress: 'stress' };
          if (map[norm]) {
            if (typeof window.switchDataset === 'function') window.switchDataset(map[norm]);
            if (window.telemetry) {
              var keyMap = { orbit: 'synthetic_orbit', hover: 'synthetic_hover', straight: 'synthetic_straight', multi_axis: 'synthetic_multi_axis', stress: 'synthetic_stress' };
              if (keyMap[map[norm]]) window.telemetry.setDemoDataset(keyMap[map[norm]]);
            }
            // highlight button
            document.querySelectorAll('.traj-sel-btn').forEach(function(b){ b.classList.remove('bg-stone-100','text-stone-900','shadow-inner','font-semibold'); });
            var active = document.getElementById('btn-traj-' + map[norm]);
            if (active) active.classList.add('bg-stone-100','text-stone-900','shadow-inner','font-semibold');
          }
        }
      } catch(e) {}
      // wrap switchDataset to persist
      if (window.switchDataset && !window.switchDataset._persistWrapped) {
        var orig = window.switchDataset;
        var self = this;
        window.switchDataset = function(key){
          var ret = orig.apply(this, arguments);
          try {
            var url = new URL(window.location.href);
            url.searchParams.set('dataset', key);
            history.replaceState(null, '', url.toString());
            if (window.telemetry) {
              var km = { orbit: 'synthetic_orbit', hover: 'synthetic_hover', straight: 'synthetic_straight', multi_axis: 'synthetic_multi_axis', stress: 'synthetic_stress' };
              if (km[key]) window.telemetry.setDemoDataset(km[key]);
            }
          } catch(e) {}
          return ret;
        };
        window.switchDataset._persistWrapped = true;
      }
    }

    _persistDataset(key) {
      try {
        var url = new URL(window.location.href);
        url.searchParams.set('dataset', key);
        history.replaceState(null, '', url.toString());
      } catch(e) {}
    }

    // Sidebar drawer toggle + overlay
    _initDrawer() {
      var sidebar = document.querySelector('aside');
      if (!sidebar) return;
      // ensure overlay exists
      var overlay = document.getElementById('sidebar-overlay');
      if (!overlay) {
        overlay = document.createElement('div');
        overlay.id = 'sidebar-overlay';
        overlay.className = 'fixed inset-0 bg-black/40 backdrop-blur-sm z-30 hidden lg:hidden';
        document.body.appendChild(overlay);
      }
      // ensure toggle button exists in header
      var toggleBtn = document.getElementById('sidebar-drawer-toggle');
      if (!toggleBtn) {
        var headerLeft = document.querySelector('header .flex.items-center.space-x-3');
        toggleBtn = document.createElement('button');
        toggleBtn.id = 'sidebar-drawer-toggle';
        toggleBtn.className = 'lg:hidden p-1.5 rounded-lg bg-stone-800 text-stone-200 border border-stone-700 mr-2';
        toggleBtn.innerHTML = '<svg class="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path d="M4 6h16M4 12h16M4 18h16" stroke-linecap="round" stroke-linejoin="round" stroke-width="2"/></svg>';
        if (headerLeft) headerLeft.prepend(toggleBtn);
        else {
          var hdr = document.querySelector('header');
          if (hdr) hdr.prepend(toggleBtn);
        }
      }
      // style sidebar for drawer on mobile
      sidebar.classList.add('fixed','inset-y-0','left-0','z-40','transform','transition-transform','duration-200','lg:static','lg:transform-none');
      // hide by default on mobile
      if (window.innerWidth < 1024) {
        sidebar.classList.add('-translate-x-full');
        sidebar.classList.remove('translate-x-0');
      } else {
        sidebar.classList.add('translate-x-0');
        sidebar.classList.remove('-translate-x-full');
      }
      var self = this;
      toggleBtn.addEventListener('click', function(){ self._toggleDrawer(); });
      overlay.addEventListener('click', function(){ self._closeDrawer(); });
      window.addEventListener('resize', function(){
        if (window.innerWidth >= 1024) {
          sidebar.classList.remove('-translate-x-full');
          sidebar.classList.add('translate-x-0');
          overlay.classList.add('hidden');
          self.drawerOpen = false;
        } else if (!self.drawerOpen) {
          sidebar.classList.add('-translate-x-full');
        }
      });
    }
    _toggleDrawer() {
      var sidebar = document.querySelector('aside');
      var overlay = document.getElementById('sidebar-overlay');
      if (!sidebar) return;
      this.drawerOpen = !this.drawerOpen;
      if (this.drawerOpen) {
        sidebar.classList.remove('-translate-x-full');
        sidebar.classList.add('translate-x-0');
        if (overlay) overlay.classList.remove('hidden');
      } else {
        sidebar.classList.add('-translate-x-full');
        sidebar.classList.remove('translate-x-0');
        if (overlay) overlay.classList.add('hidden');
      }
    }
    _closeDrawer() {
      var sidebar = document.querySelector('aside');
      var overlay = document.getElementById('sidebar-overlay');
      if (!sidebar) return;
      this.drawerOpen = false;
      sidebar.classList.add('-translate-x-full');
      sidebar.classList.remove('translate-x-0');
      if (overlay) overlay.classList.add('hidden');
    }

    // IntersectionObserver for active nav highlighting
    _initActiveNavObserver() {
      var navLinks = document.querySelectorAll('aside a.nav-btn[href^="#"]');
      if (!navLinks.length) return;
      var sectionIds = Array.from(navLinks).map(function(a){ return a.getAttribute('href'); }).filter(Boolean);
      var sections = sectionIds.map(function(id){ return document.querySelector(id); }).filter(Boolean);
      if (!sections.length || !('IntersectionObserver' in window)) return;
      var self = this;
      var observer = new IntersectionObserver(function(entries){
        entries.forEach(function(entry){
          if (entry.isIntersecting) {
            var id = '#' + entry.target.id;
            navLinks.forEach(function(a){
              if (a.getAttribute('href') === id) {
                a.classList.add('bg-[#E3E5E7]','text-stone-900','shadow-sm');
                a.classList.remove('text-stone-700','hover:bg-stone-200/60');
                self._activeNavId = id;
              } else {
                a.classList.remove('bg-[#E3E5E7]','text-stone-900','shadow-sm');
                a.classList.add('text-stone-700','hover:bg-stone-200/60');
              }
            });
          }
        });
      }, { root: null, rootMargin: '-20% 0px -60% 0px', threshold: 0.1 });
      sections.forEach(function(s){ observer.observe(s); });
    }

    _startClock() {
      setInterval(() => {
        const now = new Date();
        if (this.elements.utcClock) {
          this.elements.utcClock.textContent = now.toISOString().substring(11, 22);
        }
        if (this.isRecordingRosbag && this.elements.rosbagTimer) {
          const diffMs = Date.now() - this.rosbagStartTime;
          const totalSec = Math.floor(diffMs / 1000);
          const hrs = String(Math.floor(totalSec / 3600)).padStart(2, '0');
          const mins = String(Math.floor((totalSec % 3600) / 60)).padStart(2, '0');
          const secs = String(totalSec % 60).padStart(2, '0');
          this.elements.rosbagTimer.textContent = `REC ${hrs}:${mins}:${secs}`;
        }
      }, 50);
    }

    _syncEndpointBadge() {
      var badge = document.getElementById('top-endpoint-badge');
      if (!badge) badge = ensureEndpointBadge();
      try {
        if (badge && window.APP_CONFIG) badge.textContent = window.APP_CONFIG.websocketUrl || '';
      } catch(e) {}
    }

    /**
     * Mode Switcher Handler: 'LIVE' vs 'DEMO'
     */
    setMode(mode) {
      if (!window.telemetry) return;
      window.telemetry.setMode(mode);

      const liveBtn = document.getElementById('btn-mode-live');
      const demoBtn = document.getElementById('btn-mode-demo');
      // FIX: use dataset-selector-bar
      const datasetContainer = document.getElementById('dataset-selector-bar') || document.getElementById('dataset-selector-container') || document.getElementById('dataset-selector');

      if (mode === 'LIVE') {
        if (liveBtn) {
          liveBtn.className = 'px-3 py-1 bg-emerald-500 text-black font-bold uppercase tracking-wider text-xs shadow-[0_0_12px_rgba(16,185,129,0.4)] transition';
        }
        if (demoBtn) {
          demoBtn.className = 'px-3 py-1 bg-surface-container-high hover:bg-surface-container-highest text-on-surface uppercase tracking-wider text-xs transition';
        }
        if (datasetContainer) datasetContainer.classList.add('hidden');
        this.showToast('Switched to LIVE ROS 2 Mode — Searching bridge at ' + (window.APP_CONFIG ? window.APP_CONFIG.websocketUrl : 'ws://localhost:8000/ws'), 'info');
      } else {
        if (liveBtn) {
          liveBtn.className = 'px-3 py-1 bg-surface-container-high hover:bg-surface-container-highest text-on-surface uppercase tracking-wider text-xs transition';
        }
        if (demoBtn) {
          demoBtn.className = 'px-3 py-1 bg-secondary text-on-secondary font-bold uppercase tracking-wider text-xs shadow-[0_0_12px_rgba(125,244,255,0.4)] transition';
        }
        if (datasetContainer) datasetContainer.classList.remove('hidden');
        this.showToast('Switched to DEMO Mode — Streaming real trajectory datasets', 'success');
      }
    }

    /**
     * Switch Demo dataset — also persists via history.replaceState
     */
    onDatasetChanged(val) {
      if (window.telemetry) {
        window.telemetry.setDemoDataset(val);
        const nameMap = {
          synthetic_orbit: 'Synthetic Orbit Trajectory (60s)',
          synthetic_hover: 'Synthetic Hover Maneuver (30s)',
          synthetic_straight: 'Synthetic Straight Run (45s)',
          synthetic_multi_axis: 'Synthetic Multi-Axis Maneuver',
          synthetic_stress: 'Synthetic VIO Stress Test'
        };
        this.showToast(`Loaded Dataset: ${nameMap[val] || val}`, 'info');
        // persist short key
        var shortMap = { synthetic_orbit: 'orbit', synthetic_hover: 'hover', synthetic_straight: 'straight', synthetic_multi_axis: 'multi_axis', synthetic_stress: 'stress' };
        var shortKey = shortMap[val] || val;
        this._persistDataset(shortKey);
        this._renderEmptyStates();
      }
    }

    /**
     * Stale Telemetry Notification — wired to #stale-warning-banner + #stale-warning-text
     */
    onStaleUpdate(info) {
      if (!this.elements.staleBanner) this.elements.staleBanner = document.getElementById('stale-warning-banner') || ensureStaleBanner();
      if (!this.elements.staleText) this.elements.staleText = document.getElementById('stale-warning-text');
      if (info.isStale && window.telemetry.mode === 'LIVE') {
        this.elements.staleBanner.classList.remove('hidden');
        if (this.elements.staleText) {
          this.elements.staleText.textContent = `TELEMETRY STALE — ${info.duration}s (NO ROS 2 PACKETS RECEIVED)`;
        }
      } else {
        if (this.elements.staleBanner) this.elements.staleBanner.classList.add('hidden');
      }
    }

    /**
     * System Health Updates
     */
    onHealthUpdate(health) {
      this._setHealthBadge(this.elements.healthCamera, health.camera);
      this._setHealthBadge(this.elements.healthImu, health.imu);
      this._setHealthBadge(this.elements.healthVio, health.vio);
      this._setHealthBadge(this.elements.healthController, health.controller);
      this._setHealthBadge(this.elements.healthWs, health.websocket);
    }

    _setHealthBadge(el, status) {
      if (!el) return;
      el.textContent = status || 'DISCONNECTED';
      el.className = 'font-label-sm text-label-sm px-1.5 py-0.5 font-bold uppercase tracking-wider ';
      
      switch (status) {
        case 'ONLINE':
        case 'ACTIVE':
        case 'CONNECTED':
          el.className += 'bg-emerald-950 text-emerald-300 border border-emerald-800';
          break;
        case 'READY':
        case 'DEMO':
          el.className += 'bg-cyan-950 text-cyan-300 border border-cyan-800';
          break;
        case 'DEGRADED':
          el.className += 'bg-amber-950 text-amber-300 border border-amber-800';
          break;
        case 'DISCONNECTED':
        case 'OFFLINE':
        default:
          el.className += 'bg-red-950 text-red-300 border border-red-800';
          break;
      }
    }

    /**
     * Main Telemetry Frame Ingestion
     */
    onTelemetryUpdate(state) {
      const p = state.position;
      const v = state.velocity;
      const o = state.orientation;

      // Position
      if (this.elements.gaugePosX) {
        this.elements.gaugePosX.textContent = p.x !== null ? p.x.toFixed(2) : '--';
      }
      if (this.elements.gaugePosY) {
        this.elements.gaugePosY.textContent = p.y !== null ? p.y.toFixed(2) : '--';
      }
      if (this.elements.gaugePosZ) {
        this.elements.gaugePosZ.textContent = p.z !== null ? p.z.toFixed(2) : '--';
      }
      if (this.elements.gaugePosTotal) {
        this.elements.gaugePosTotal.textContent = `TOTAL: ${p.total_dist ? p.total_dist.toFixed(1) : (p.x !== null ? '0.0' : '--')} m`;
      }
      if (this.elements.gaugeAltBar) {
        this.elements.gaugeAltBar.style.width = p.z !== null ? `${Math.min(100, Math.max(5, (p.z / 25.0) * 100))}%` : '0%';
      }

      // Velocity
      if (this.elements.gaugeVelVx) {
        this.elements.gaugeVelVx.textContent = v.vx !== null ? v.vx.toFixed(2) : '--';
      }
      if (this.elements.gaugeVelVy) {
        this.elements.gaugeVelVy.textContent = v.vy !== null ? v.vy.toFixed(2) : '--';
      }
      if (this.elements.gaugeVelVz) {
        this.elements.gaugeVelVz.textContent = v.vz !== null ? v.vz.toFixed(2) : '--';
      }
      if (this.elements.gaugeVelTotal) {
        this.elements.gaugeVelTotal.textContent = v.norm !== null ? `${v.norm.toFixed(2)} m/s` : '-- m/s';
      }
      if (this.elements.gaugeVelBar) {
        this.elements.gaugeVelBar.style.width = v.norm !== null ? `${Math.min(100, (v.norm / 5.0) * 100)}%` : '0%';
      }

      // Attitude
      if (this.elements.gaugeAttRoll) {
        this.elements.gaugeAttRoll.textContent = o.roll !== null ? `${(o.roll >= 0 ? '+' : '') + o.roll.toFixed(1)}°` : '--';
      }
      if (this.elements.gaugeAttPitch) {
        this.elements.gaugeAttPitch.textContent = o.pitch !== null ? `${(o.pitch >= 0 ? '+' : '') + o.pitch.toFixed(1)}°` : '--';
      }
      if (this.elements.gaugeAttYaw) {
        this.elements.gaugeAttYaw.textContent = o.yaw !== null ? `${(o.yaw >= 0 ? '+' : '') + o.yaw.toFixed(1)}°` : '--';
      }
      if (this.elements.gaugeAttQuat) {
        this.elements.gaugeAttQuat.textContent = o.qw !== null ? `QUAT: [${o.qw.toFixed(3)}, ${o.qx.toFixed(3)}, ${o.qy.toFixed(3)}, ${o.qz.toFixed(3)}]` : 'QUAT: [--, --, --, --]';
      }

      // Matrix Cards Spans
      const elPosWb = document.getElementById('tel-pos-wb');
      if (elPosWb) {
        elPosWb.textContent = p.x !== null ? `[${p.x >= 0 ? '+' : ''}${p.x.toFixed(2)}, ${p.y >= 0 ? '+' : ''}${p.y.toFixed(2)}, ${p.z >= 0 ? '+' : ''}${p.z.toFixed(2)}] m` : '[--, --, --] m';
      }
      const elPosWc = document.getElementById('tel-pos-wc');
      if (elPosWc) {
        const c = state.camera_pose;
        elPosWc.textContent = (c && c.x !== null) ? `[${c.x >= 0 ? '+' : ''}${c.x.toFixed(2)}, ${c.y >= 0 ? '+' : ''}${c.y.toFixed(2)}, ${c.z >= 0 ? '+' : ''}${c.z.toFixed(2)}] m` : '[--, --, --] m';
      }
      const elVel = document.getElementById('tel-vel');
      if (elVel) {
        elVel.textContent = v.vx !== null ? `[${v.vx >= 0 ? '+' : ''}${v.vx.toFixed(2)}, ${v.vy >= 0 ? '+' : ''}${v.vy.toFixed(2)}, ${v.vz >= 0 ? '+' : ''}${v.vz.toFixed(2)}] m/s` : '[--, --, --] m/s';
      }
      const elSpeed = document.getElementById('tel-speed');
      if (elSpeed) {
        elSpeed.textContent = v.norm !== null ? `${v.norm.toFixed(2)} m/s (${(v.norm * 3.6).toFixed(1)} km/h)` : '-- m/s';
      }
      const elAccel = document.getElementById('tel-accel');
      if (elAccel) {
        const a = state.acceleration;
        elAccel.textContent = (a && a.ax !== null) ? `[${a.ax >= 0 ? '+' : ''}${a.ax.toFixed(2)}, ${a.ay >= 0 ? '+' : ''}${a.ay.toFixed(2)}, ${a.az >= 0 ? '+' : ''}${a.az.toFixed(2)}] m/s²` : '[--, --, --] m/s²';
      }
      const elEuler = document.getElementById('tel-euler');
      if (elEuler) {
        elEuler.textContent = o.roll !== null ? `R: ${o.roll >= 0 ? '+' : ''}${o.roll.toFixed(1)}° | P: ${o.pitch >= 0 ? '+' : ''}${o.pitch.toFixed(1)}° | Y: ${o.yaw >= 0 ? '+' : ''}${o.yaw.toFixed(1)}°` : 'R: --° | P: --° | Y: --°';
      }
      const elRates = document.getElementById('tel-rates');
      if (elRates) {
        elRates.textContent = o.roll_rate !== null ? `[${o.roll_rate.toFixed(2)}, ${o.pitch_rate.toFixed(2)}, ${o.yaw_rate.toFixed(2)}] rad/s` : '[--, --, --] rad/s';
      }
      const elQuat = document.getElementById('tel-quat');
      if (elQuat) {
        elQuat.textContent = o.qw !== null ? `[${o.qw.toFixed(3)}, ${o.qx.toFixed(3)}, ${o.qy.toFixed(3)}, ${o.qz.toFixed(3)}]` : '[--, --, --, --]';
      }
      const elDisp = document.getElementById('kpi-displacement');
      if (elDisp) {
        elDisp.textContent = p.total_dist ? p.total_dist.toFixed(1) : (p.x !== null ? '0.0' : '--');
      }

      // Top bar ROS badge + endpoint badge
      if (this.elements.rosConnText) {
        if (state.connection.mode === 'DEMO') {
          this.elements.rosConnText.textContent = 'DEMO DATASET ACTIVE';
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-cyan-400 animate-pulse';
        } else if (state.connection.websocket) {
          this.elements.rosConnText.textContent = `ROS 2 CONNECTED (${state.connection.latency_ms || 0}ms)`;
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-emerald-400 animate-pulse';
        } else if (state.connection.state && state.connection.state.indexOf('DEGRADED') !== -1) {
          this.elements.rosConnText.textContent = 'ROS 2 DEGRADED';
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-amber-400 animate-pulse';
        } else {
          this.elements.rosConnText.textContent = 'AWAITING LIVE TELEMETRY';
          if (this.elements.rosConnDot) this.elements.rosConnDot.className = 'w-2 h-2 rounded-full bg-amber-400 animate-pulse';
        }
      }
      // sync top-endpoint-badge
      var epBadge = document.getElementById('top-endpoint-badge');
      if (epBadge && window.APP_CONFIG) epBadge.textContent = window.APP_CONFIG.websocketUrl || '';

      // 3D Map drone text & HUD
      if (this.elements.mapDronePosText) {
        this.elements.mapDronePosText.textContent = p.x !== null ? `XYZ: [${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.z.toFixed(2)}]` : 'XYZ: [--, --, --]';
      }
      var hudPw = document.getElementById('hud-pw');
      if (hudPw) {
        hudPw.innerHTML = p.x !== null ? `P_WB: [ <span class="text-white">x: ${p.x.toFixed(2)}</span>, <span class="text-white">y: ${p.y.toFixed(2)}</span>, <span class="text-emerald-400">z: ${p.z.toFixed(2)}</span> ] m` : `P_WB: [ <span class="text-white">x: --</span>, <span class="text-white">y: --</span>, <span class="text-stone-500">z: --</span> ] m`;
      }
      var hudVw = document.getElementById('hud-vw');
      if (hudVw) {
        hudVw.innerHTML = v.vx !== null ? `V_WB: [ <span class="text-white">vx: ${v.vx.toFixed(2)}</span>, <span class="text-white">vy: ${v.vy.toFixed(2)}</span>, <span class="text-emerald-400">vz: ${v.vz.toFixed(2)}</span> ] m/s` : `V_WB: [ <span class="text-white">vx: --</span>, <span class="text-white">vy: --</span>, <span class="text-stone-500">vz: --</span> ] m/s`;
      }
      // update empty states visibility
      this._renderEmptyStates();
    }

    onStateChange(state) {
      this.onTelemetryUpdate(state);
      this.onHealthUpdate(state.health);
    }

    // =========================================================================
    // Empty states for graphs / PCD
    // =========================================================================
    _renderEmptyStates() {
      // Graphs empty state
      var graphBuffers = (window.graphs && window.graphs.buffers) || null;
      var graphsEmpty = true;
      if (graphBuffers) {
        graphsEmpty = (!graphBuffers.accel.length && !graphBuffers.vel.length && !graphBuffers.pos.length);
      } else {
        // fallback: check if telemetry has any position yet
        var st = window.telemetry ? window.telemetry.state : null;
        graphsEmpty = !(st && st.position && st.position.x !== null);
      }
      ['svg-graph-accel','svg-graph-vel','svg-graph-pos'].forEach(function(id){
        var svg = document.getElementById(id);
        if (!svg) return;
        var emptyEl = document.getElementById(id + '-empty');
        if (graphsEmpty) {
          if (!emptyEl) {
            emptyEl = document.createElementNS('http://www.w3.org/2000/svg','text');
            emptyEl.id = id + '-empty';
            emptyEl.setAttribute('x','200');
            emptyEl.setAttribute('y','80');
            emptyEl.setAttribute('text-anchor','middle');
            emptyEl.setAttribute('fill','#64748B');
            emptyEl.setAttribute('font-size','10');
            emptyEl.setAttribute('font-family','monospace');
            emptyEl.textContent = 'No data — awaiting telemetry';
            svg.appendChild(emptyEl);
          }
          emptyEl.style.display = 'block';
        } else if (emptyEl) {
          emptyEl.style.display = 'none';
        }
      });

      // PCD empty state
      var pcdGroup = document.getElementById('svg-pcd-points-group');
      if (pcdGroup) {
        var pcdEmpty = document.getElementById('pcd-empty-state');
        var hasPoints = pcdGroup.children.length > 0;
        var st2 = window.telemetry ? window.telemetry.state : null;
        var hasLandmarks = st2 && st2.landmarks && st2.landmarks.length > 0;
        if (!hasPoints && !hasLandmarks) {
          if (!pcdEmpty) {
            var svgRoot = document.getElementById('svg-spatial-canvas');
            if (svgRoot) {
              pcdEmpty = document.createElementNS('http://www.w3.org/2000/svg','text');
              pcdEmpty.id = 'pcd-empty-state';
              pcdEmpty.setAttribute('x','300');
              pcdEmpty.setAttribute('y','175');
              pcdEmpty.setAttribute('text-anchor','middle');
              pcdEmpty.setAttribute('fill','#94A3B8');
              pcdEmpty.setAttribute('font-size','11');
              pcdEmpty.setAttribute('font-family','monospace');
              pcdEmpty.textContent = 'No point cloud — load a dataset or connect LIVE';
              svgRoot.appendChild(pcdEmpty);
            }
          }
        } else if (pcdEmpty) {
          pcdEmpty.style.display = 'none';
          if (hasPoints || hasLandmarks) pcdEmpty.style.display = 'none';
        }
      }
    }

    // =========================================================================
    // 3D VIO MAP RENDERING ENGINE (SVG / Canvas hybrid with DPR handling)
    // =========================================================================
    _init3DMap() {
      // svg-spatial-canvas is an SVG element, not a <canvas>; we support both
      this.mapCanvas = document.getElementById('svg-spatial-canvas') || document.getElementById('map-canvas-3d');
      this.svgSpatial = document.getElementById('svg-spatial-canvas');
      // If it's actually a canvas element, init 2d context
      if (this.mapCanvas && this.mapCanvas.tagName && this.mapCanvas.tagName.toLowerCase() === 'canvas') {
        this.mapCtx = this.mapCanvas.getContext('2d');
        const render = () => {
          this._render3DMap();
          requestAnimationFrame(render);
        };
        requestAnimationFrame(render);
      } else {
        // SVG mode: sync telemetry to SVG drone marker
        // No canvas render loop needed; update via onTelemetryUpdate and CSS transforms
        // Still handle DPR for any overlay canvas siblings
        this._handleDPR();
        window.addEventListener('resize', () => this._handleDPR());
      }
    }

    _handleDPR() {
      var dpr = window.devicePixelRatio || 1;
      // Scale any canvas siblings of svg-spatial-canvas for crisp rendering
      var canvases = document.querySelectorAll('#svg-spatial-canvas + canvas, canvas#map-canvas-3d, #pointcloud canvas');
      canvases.forEach(function(c){
        var rect = c.getBoundingClientRect();
        if (!rect.width) return;
        c.width = Math.round(rect.width * dpr);
        c.height = Math.round(rect.height * dpr);
        c.style.width = rect.width + 'px';
        c.style.height = rect.height + 'px';
        var ctx = c.getContext('2d');
        if (ctx) ctx.setTransform(dpr,0,0,dpr,0,0);
      });
      // Also handle overlay canvas for graphs if needed (no-op if not present)
    }

    setProjection(mode) {
      this.projectionMode = mode;
      // FIX: normalize mode aliases to match perspective toggle ids
      var norm = mode;
      if (mode === 'iso' || mode === 'ISOMETRIC') norm = 'ISOMETRIC';
      else if (mode === 'top' || mode === 'TOP_DOWN') norm = 'TOP_DOWN';
      else if (mode === 'side' || mode === 'SIDE') norm = 'SIDE';
      else if (mode === 'follow' || mode === 'CAM_FOLLOW') norm = 'CAM_FOLLOW';
      this.projectionMode = norm;

      // Proper per-mode alignment
      if (norm === 'ISOMETRIC') {
        this.mapAngle = 0.52;
        this.mapPitch = 0.42;
        this.mapScale = 14.0;
      } else if (norm === 'TOP_DOWN') {
        // Top view: look straight down XY plane
        this.mapAngle = 0;
        this.mapPitch = Math.PI / 2 - 0.01;
        this.mapScale = 18.0;
      } else if (norm === 'SIDE') {
        // Side XZ plane
        this.mapAngle = 0;
        this.mapPitch = 0.05;
        this.mapScale = 16.0;
      } else if (norm === 'CAM_FOLLOW') {
        this.mapPitch = 0.35;
        // keep angle but pitch slight
      }

      // Button styles — support both id schemes
      var isoBtn = document.getElementById('btn-persp-iso');
      var topBtn = document.getElementById('btn-persp-top');
      var sideBtn = document.getElementById('btn-persp-side');
      var followBtn = document.getElementById('btn-persp-follow');
      [isoBtn, topBtn, sideBtn, followBtn].forEach(function(b){
        if (b) b.className = 'persp-btn text-stone-600 hover:text-stone-900 px-2 py-0.5 rounded';
      });
      var activeBtn = null;
      if (norm === 'ISOMETRIC' && isoBtn) activeBtn = isoBtn;
      else if (norm === 'TOP_DOWN' && topBtn) activeBtn = topBtn;
      else if (norm === 'SIDE' && sideBtn) activeBtn = sideBtn;
      else if (norm === 'CAM_FOLLOW' && followBtn) activeBtn = followBtn;
      if (activeBtn) {
        activeBtn.className = 'persp-btn bg-white text-stone-900 px-2 py-0.5 rounded shadow-sm';
      }
      // Also support legacy btn-proj-* ids
      ['btn-proj-iso', 'btn-proj-top', 'btn-proj-follow'].forEach(function(id){
        var b = document.getElementById(id);
        if (b) {
          var isActive = (id === 'btn-proj-iso' && norm==='ISOMETRIC') || (id === 'btn-proj-top' && norm==='TOP_DOWN') || (id === 'btn-proj-follow' && norm==='CAM_FOLLOW');
          b.className = isActive ? 'px-space-sm py-1 bg-primary text-on-primary font-label-sm text-label-sm uppercase font-bold' : 'px-space-sm py-1 bg-surface-container-high hover:bg-surface-container-highest text-on-surface font-label-sm text-label-sm uppercase transition-colors';
        }
      });
      // Sync to inline PCD projection helper if present
      if (typeof window.setPerspective === 'function') {
        try { window.setPerspective(mode.toLowerCase()); } catch(e) {}
      }
    }

    toggleLayer(layerName) {
      this.layers[layerName] = !this.layers[layerName];
    }

    clearTrajectory() {
      // Save undo snapshot
      var snapshot = null;
      if (window.telemetry) {
        try {
          snapshot = {
            body: (window.telemetry.state.trajectories.body || []).slice(),
            camera: (window.telemetry.state.trajectories.camera || []).slice()
          };
        } catch(e) { snapshot = null; }
        if (window.graphs && window.graphs.buffers) {
          snapshot.graphs = {
            accel: (window.graphs.buffers.accel || []).slice(),
            vel: (window.graphs.buffers.vel || []).slice(),
            pos: (window.graphs.buffers.pos || []).slice()
          };
        }
        window.telemetry.state.trajectories.body = [];
        window.telemetry.state.trajectories.camera = [];
        if (window.graphs && typeof window.graphs.clearBuffers === 'function') {
          window.graphs.clearBuffers(true); // silent, we handle toast
        }
      }
      if (snapshot) {
        this.clearUndoSnapshot = snapshot;
        if (this.clearUndoTimer) clearTimeout(this.clearUndoTimer);
        // expire undo after 10s
        this.clearUndoTimer = setTimeout(() => { this.clearUndoSnapshot = null; }, 10000);
        this.showToast('Trajectory Buffer Cleared', 'info', true);
      } else {
        this.showToast('Trajectory Buffer Cleared', 'info');
      }
      this._renderEmptyStates();
    }

    undoClear() {
      if (!this.clearUndoSnapshot) {
        this.showToast('Nothing to undo', 'info');
        return;
      }
      if (window.telemetry) {
        window.telemetry.state.trajectories.body = this.clearUndoSnapshot.body || [];
        window.telemetry.state.trajectories.camera = this.clearUndoSnapshot.camera || [];
      }
      if (window.graphs && this.clearUndoSnapshot.graphs) {
        window.graphs.buffers.accel = this.clearUndoSnapshot.graphs.accel || [];
        window.graphs.buffers.vel = this.clearUndoSnapshot.graphs.vel || [];
        window.graphs.buffers.pos = this.clearUndoSnapshot.graphs.pos || [];
        if (typeof window.graphs.render === 'function') window.graphs.render();
      }
      this.clearUndoSnapshot = null;
      if (this.clearUndoTimer) { clearTimeout(this.clearUndoTimer); this.clearUndoTimer = null; }
      this.showToast('Undo: trajectory restored', 'success');
      this._renderEmptyStates();
    }

    exportPointCloudPCD() {
      const landmarks = (window.telemetry && window.telemetry.state.landmarks) || [];
      if (!landmarks.length) {
        this.showToast('No 3D landmarks available to export', 'error');
        return;
      }

      let pcd = `# .PCD v0.7 - Point Cloud Data generated by AeroVIO Ground Station\n`;
      pcd += `VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n`;
      pcd += `WIDTH ${landmarks.length}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ${landmarks.length}\nDATA ascii\n`;

      landmarks.forEach(pt => {
        if (Array.isArray(pt)) pcd += `${pt[0].toFixed(4)} ${pt[1].toFixed(4)} ${pt[2].toFixed(4)}\n`;
        else pcd += `${pt.x.toFixed(4)} ${pt.y.toFixed(4)} ${pt.z.toFixed(4)}\n`;
      });

      const blob = new Blob([pcd], { type: 'text/plain' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `aerovio_pointcloud_${Date.now()}.pcd`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);

      this.showToast(`Exported ${landmarks.length} points to .PCD`, 'success');
    }

    /**
     * 3D Coordinate Projector: (x, y, z) in World Frame -> (sx, sy) on Canvas
     * Fixed iso/top/side alignment to match svg-spatial-canvas viewBox 600x350
     */
    _project(x, y, z, cx, cy, droneX, droneY, droneZ) {
      // Support TOP_DOWN and SIDE as absolute projections aligned to spatial canvas helpers
      if (this.projectionMode === 'TOP_DOWN') {
        // Top (XY): match inline project3DToSVG top: x:300+x*22, y:175+y*22 (viewBox 600x350)
        // Convert to canvas coords: cx/cy passed is w*0.5 / h*0.5
        var scaleTop = this.mapScale * 1.15;
        // center at cx,cy
        return { sx: cx + x * scaleTop, sy: cy + y * scaleTop, depth: -z };
      }
      if (this.projectionMode === 'SIDE') {
        var scaleSide = this.mapScale * 1.15;
        return { sx: cx + x * scaleSide, sy: cy - z * scaleSide, depth: y };
      }
      let rx = x, ry = y, rz = z;

      if (this.projectionMode === 'CAM_FOLLOW') {
        rx -= droneX || 0;
        ry -= droneY || 0;
        rz -= droneZ || 0;
      }

      // Rotate by yaw (mapAngle)
      const cosA = Math.cos(this.mapAngle);
      const sinA = Math.sin(this.mapAngle);
      const x1 = rx * cosA - ry * sinA;
      const y1 = rx * sinA + ry * cosA;

      // Rotate by pitch (mapPitch)
      const cosP = Math.cos(this.mapPitch);
      const sinP = Math.sin(this.mapPitch);
      const y2 = y1 * cosP - rz * sinP;
      const z2 = y1 * sinP + rz * cosP;

      const scale = this.mapScale;
      const sx = cx + x1 * scale;
      const sy = cy - y2 * scale; // Invert Y for screen

      return { sx, sy, depth: z2 };
    }

    _render3DMap() {
      if (!this.mapCanvas || !this.mapCtx) {
        // SVG fallback: nothing to canvas-render; update SVG via telemetry instead
        return;
      }
      // DevicePixelRatio handling
      var dpr = window.devicePixelRatio || 1;
      var parent = this.mapCanvas.parentElement;
      var cssW = (parent && parent.clientWidth) || 1000;
      var cssH = (parent && parent.clientHeight) || 480;
      // set canvas backing store for DPR
      if (this.mapCanvas.width !== Math.round(cssW * dpr) || this.mapCanvas.height !== Math.round(cssH * dpr)) {
        this.mapCanvas.width = Math.round(cssW * dpr);
        this.mapCanvas.height = Math.round(cssH * dpr);
        this.mapCanvas.style.width = cssW + 'px';
        this.mapCanvas.style.height = cssH + 'px';
        this.mapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
      }
      const w = cssW;
      const h = cssH;
      const ctx = this.mapCtx;

      ctx.clearRect(0, 0, w, h);

      const cx = w * 0.50;
      const cy = h * 0.58;

      const state = window.telemetry ? window.telemetry.state : null;
      const droneX = (state && state.position.x) || 0;
      const droneY = (state && state.position.y) || 0;
      const droneZ = (state && state.position.z) || 0;

      // 1. Perspective Metric Grid Floor (Z = 0)
      ctx.strokeStyle = '#1c2027';
      ctx.lineWidth = 0.75;
      const gridSize = 15;
      const step = 2.0;

      for (let i = -gridSize; i <= gridSize; i += step) {
        const p1 = this._project(i, -gridSize, 0, cx, cy, droneX, droneY, droneZ);
        const p2 = this._project(i, gridSize, 0, cx, cy, droneX, droneY, droneZ);
        ctx.beginPath();
        ctx.moveTo(p1.sx, p1.sy);
        ctx.lineTo(p2.sx, p2.sy);
        ctx.stroke();

        const p3 = this._project(-gridSize, i, 0, cx, cy, droneX, droneY, droneZ);
        const p4 = this._project(gridSize, i, 0, cx, cy, droneX, droneY, droneZ);
        ctx.beginPath();
        ctx.moveTo(p3.sx, p3.sy);
        ctx.lineTo(p4.sx, p4.sy);
        ctx.stroke();
      }

      // 2. World Coordinate Axes Origin
      if (this.layers.axes) {
        const o = this._project(0, 0, 0, cx, cy, droneX, droneY, droneZ);
        const axX = this._project(3.0, 0, 0, cx, cy, droneX, droneY, droneZ);
        const axY = this._project(0, 3.0, 0, cx, cy, droneX, droneY, droneZ);
        const axZ = this._project(0, 0, 3.0, cx, cy, droneX, droneY, droneZ);

        // X Axis (Red)
        ctx.strokeStyle = '#ffb4ab';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(axX.sx, axX.sy);
        ctx.stroke();
        ctx.fillStyle = '#ffb4ab';
        ctx.font = '9px "Space Mono", monospace';
        ctx.fillText('X_W', axX.sx + 4, axX.sy + 2);

        // Y Axis (Cyan)
        ctx.strokeStyle = '#7df4ff';
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(axY.sx, axY.sy);
        ctx.stroke();
        ctx.fillStyle = '#7df4ff';
        ctx.fillText('Y_W', axY.sx + 4, axY.sy + 2);

        // Z Axis (Violet - UP)
        ctx.strokeStyle = '#b5c4ff';
        ctx.beginPath();
        ctx.moveTo(o.sx, o.sy);
        ctx.lineTo(axZ.sx, axZ.sy);
        ctx.stroke();
        ctx.fillStyle = '#b5c4ff';
        ctx.fillText('Z_W (UP)', axZ.sx - 10, axZ.sy - 6);
      }

      // 3. Sparse 3D Landmark Point Cloud
      if (this.layers.landmarks && state && state.landmarks && state.landmarks.length) {
        if (this.elements.mapPointCount) {
          this.elements.mapPointCount.textContent = state.landmarks.length;
        }
        
        ctx.fillStyle = '#7df4ff';
        state.landmarks.forEach((pt, idx) => {
          var px = Array.isArray(pt) ? pt[0] : pt.x;
          var py = Array.isArray(pt) ? pt[1] : pt.y;
          var pz = Array.isArray(pt) ? pt[2] : pt.z;
          const sp = this._project(px, py, pz, cx, cy, droneX, droneY, droneZ);
          if (idx % 3 === 0) {
            ctx.fillStyle = '#d1bcff';
          } else if (idx % 2 === 0) {
            ctx.fillStyle = '#00eefc';
          } else {
            ctx.fillStyle = '#7df4ff';
          }
          ctx.beginPath();
          ctx.arc(sp.sx, sp.sy, 1.8, 0, Math.PI * 2);
          ctx.fill();
        });
      }

      // 4. Body Trajectory Ribbon: T_WB (World to Body)
      if (this.layers.bodyTrajectory && state && state.trajectories.body.length > 1) {
        ctx.strokeStyle = '#7df4ff';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        state.trajectories.body.forEach((pt, idx) => {
          const sp = this._project(pt.x, pt.y, pt.z, cx, cy, droneX, droneY, droneZ);
          if (idx === 0) ctx.moveTo(sp.sx, sp.sy);
          else ctx.lineTo(sp.sx, sp.sy);
        });
        ctx.stroke();
      }

      // 5. Camera Optical Trajectory Ribbon & Frustums: T_WC
      if (this.layers.cameraTrajectory && state && state.trajectories.camera.length > 1) {
        ctx.strokeStyle = '#d1bcff';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        state.trajectories.camera.forEach((pt, idx) => {
          const sp = this._project(pt.x, pt.y, pt.z, cx, cy, droneX, droneY, droneZ);
          if (idx === 0) ctx.moveTo(sp.sx, sp.sy);
          else ctx.lineTo(sp.sx, sp.sy);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 6. Current Drone Pose Marker (T_WB Live Target)
      if (state && state.position.x !== null) {
        const dp = this._project(droneX, droneY, droneZ, cx, cy, droneX, droneY, droneZ);

        // Ground shadow dot
        const shadow = this._project(droneX, droneY, 0, cx, cy, droneX, droneY, droneZ);
        ctx.strokeStyle = 'rgba(125, 244, 255, 0.3)';
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(shadow.sx, shadow.sy);
        ctx.lineTo(dp.sx, dp.sy);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = 'rgba(0, 238, 252, 0.2)';
        ctx.beginPath();
        ctx.arc(shadow.sx, shadow.sy, 6, 0, Math.PI * 2);
        ctx.fill();

        // Target pulse ring
        ctx.strokeStyle = '#00eefc';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(dp.sx, dp.sy, 12, 0, Math.PI * 2);
        ctx.stroke();

        // Center drone dot
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(dp.sx, dp.sy, 4, 0, Math.PI * 2);
        ctx.fill();

        // Frustum Pyramidal Wireframe
        if (this.layers.frustums) {
          const fTop = this._project(droneX + 1.2, droneY + 0.6, droneZ + 0.4, cx, cy, droneX, droneY, droneZ);
          const fBot = this._project(droneX + 1.2, droneY - 0.6, droneZ - 0.4, cx, cy, droneX, droneY, droneZ);

          ctx.fillStyle = 'rgba(0, 238, 252, 0.12)';
          ctx.strokeStyle = '#7df4ff';
          ctx.lineWidth = 1.2;
          ctx.beginPath();
          ctx.moveTo(dp.sx, dp.sy);
          ctx.lineTo(fTop.sx, fTop.sy);
          ctx.lineTo(fBot.sx, fBot.sy);
          ctx.closePath();
          ctx.fill();
          ctx.stroke();
        }

        // Label
        ctx.fillStyle = '#7df4ff';
        ctx.font = 'bold 10px "Space Mono", monospace';
        ctx.fillText('DRONE T_WB', dp.sx + 16, dp.sy - 8);
        ctx.fillStyle = '#dfe2ed';
        ctx.font = '9px "Space Mono", monospace';
        ctx.fillText(`[${droneX.toFixed(2)}, ${droneY.toFixed(2)}, ${droneZ.toFixed(2)}]`, dp.sx + 16, dp.sy + 6);
      }

      // Empty state for canvas PCD overlay
      if (!state || !state.landmarks || !state.landmarks.length) {
        var tot = (state && state.trajectories.body.length) || 0;
        if (tot === 0) {
          ctx.fillStyle = 'rgba(148,163,184,0.9)';
          ctx.font = '11px "Space Mono", monospace';
          ctx.textAlign = 'center';
          ctx.fillText('No trajectory — load a dataset or connect LIVE', w/2, h/2);
          ctx.textAlign = 'left';
        }
      }
    }

    // =========================================================================
    // MODALS & GENERAL CONTROLS
    // =========================================================================
    // Resolve modal id aliases (search-modal vs modal-search, tuning-modal vs modal-controller)
    _resolveModalId(id) {
      if (document.getElementById(id)) return id;
      var aliasMap = {
        'modal-search': 'search-modal',
        'search-modal': 'modal-search',
        'modal-controller': 'tuning-modal',
        'tuning-modal': 'modal-controller',
        'modal-vio': 'tuning-modal'
      };
      var alias = aliasMap[id];
      if (alias && document.getElementById(alias)) return alias;
      return id;
    }

    toggleModal(modalId) {
      var resolved = this._resolveModalId(modalId);
      const el = document.getElementById(resolved) || document.getElementById(modalId);
      if (el) {
        el.classList.toggle('hidden');
        // focus search input if opening search modal
        if (!el.classList.contains('hidden') && (resolved === 'search-modal' || resolved === 'modal-search')) {
          var inp = document.getElementById('search-modal-input') || document.getElementById('modal-search-input');
          if (inp) setTimeout(function(){ inp.focus(); }, 50);
        }
      } else if (modalId === 'modal-search' || modalId === 'search-modal') {
        // try global opener fallback
        if (typeof window.openSearchModal === 'function') window.openSearchModal();
      }
    }

    closeAllModals() {
      ['search-modal','modal-search','tuning-modal','modal-controller','modal-vio'].forEach(function(id){
        const el = document.getElementById(id);
        if (el) el.classList.add('hidden');
      });
    }

    toggleRosbag() {
      this.isRecordingRosbag = !this.isRecordingRosbag;
      if (this.elements.rosbagBtn) {
        if (this.isRecordingRosbag) {
          this.rosbagStartTime = Date.now();
          this.elements.rosbagBtn.className = 'flex items-center space-x-1.5 bg-red-950/60 text-red-300 hover:bg-red-900/60 border border-red-800/80 px-2.5 py-1 rounded-lg text-xs font-mono transition-colors';
          this.showToast('Rosbag 2 Recording STARTED: /bag_session_' + Date.now(), 'info');
        } else {
          this.elements.rosbagBtn.className = 'flex items-center space-x-1.5 bg-stone-800 text-stone-400 hover:bg-stone-700 border border-stone-700 px-2.5 py-1 rounded-lg text-xs font-mono transition-colors';
          if (this.elements.rosbagTimer) this.elements.rosbagTimer.textContent = 'REC PAUSED';
          this.showToast('Rosbag 2 Recording STOPPED & Saved', 'success');
        }
      }
    }

    triggerRTL() {
      console.log('[Mission Control] Triggering Return-to-Launch (RTL Land)...');
      if (window.wsBridge) {
        window.wsBridge.sendCommand({ action: 'flight_command', mode: 'RTL' });
      }
      this.showToast('EMERGENCY RTL TRIGGERED: Drone returning to launch coordinates', 'error');
    }

    showToast(message, type, withUndo) {
      var container = this.elements.toastContainer || document.getElementById('toast-box') || document.getElementById('toast-container');
      if (!container) return;
      const toast = document.createElement('div');
      
      let borderClass = 'border-cyan-500 text-cyan-200 bg-[#141820]/95';
      if (type === 'error') borderClass = 'border-red-500 text-red-200 bg-[#251012]/95';
      if (type === 'success') borderClass = 'border-emerald-500 text-emerald-200 bg-[#102418]/95';

      toast.className = `p-3 rounded border shadow-xl flex items-center gap-2.5 text-xs font-mono tracking-wide transform transition-all duration-300 translate-y-2 opacity-0 ${borderClass}`;
      var undoHtml = withUndo && this.clearUndoSnapshot ? ' <button onclick="window.ui.undoClear()" class="ml-2 underline font-bold hover:text-white">Undo</button>' : '';
      toast.innerHTML = `<span class="w-2 h-2 rounded-full ${type === 'error' ? 'bg-red-400 animate-pulse' : (type === 'success' ? 'bg-emerald-400' : 'bg-cyan-400')}"></span><span>${message}</span>${undoHtml}`;

      container.appendChild(toast);
      requestAnimationFrame(() => {
        toast.classList.remove('translate-y-2', 'opacity-0');
      });

      setTimeout(() => {
        toast.classList.add('translate-y-2', 'opacity-0');
        setTimeout(() => { try{toast.remove();}catch(e){} }, 350);
      }, withUndo ? 6000 : 4000);
    }
  }

  // Expose singleton on window
  window.ui = new UIController();

  // Global convenience wrappers — keep both modal id schemes
  window.toggleModal = (id) => window.ui.toggleModal(id);
  window.setDashboardMode = (mode) => window.ui.setMode(mode);
  window.toggleRosbag = () => window.ui.toggleRosbag();
  window.triggerRTL = () => window.ui.triggerRTL();
  // Bridge for external callers that expect these exact names
  window.openSearchModal = () => window.ui.toggleModal('search-modal');
  window.closeSearchModal = () => window.ui.closeAllModals();
  window.clearTrajectory = () => window.ui.clearTrajectory();
  window.undoClear = () => window.ui.undoClear();

})(window);
