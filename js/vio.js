/**
 * ============================================================================
 * AeroVIO Ground Control Station - 3D Spatial Map & MSCKF Visualizer
 * ============================================================================
 * High-performance, robotics-grade 3D visualization of:
 * - World Coordinate Frame: ENU (East-North-Up: +X East/Red, +Y North/Green, +Z Up/Blue)
 * - Calibrated Ground Plane Grid (XY elevation Z=0)
 * - 6-DOF Drone Body Pose (T_WB) with Quadrotor Frame & Heading Triad
 * - Camera Optical Frame (T_WC = T_WB * T_BC) with Viewing Frustum & Optical Axis
 * - Historical Trajectory Buffer (reconstructed from actual pose history, bounded buffer)
 * - Triangulated Sparse VIO Landmarks (442 genuine points from MSCKF point cloud)
 * - Interactive OrbitControls (orbit, pan, zoom) with Perspective Presets
 * ============================================================================
 */

(function(window) {
  'use strict';

  class VioSpatialVisualizer {
    constructor() {
      this.container = null;
      this.canvas = null;
      this.renderer = null;
      this.scene = null;
      this.camera = null;
      this.controls = null;
      this.isInitialized = false;
      this.webglAvailable = true;

      // Scene components
      this.gridHelper = null;
      this.worldTriad = null;
      this.droneGroup = null;
      this.cameraGroup = null;
      this.trajectoryLine = null;
      this.trajectoryGeo = null;
      this.landmarksPoints = null;

      // Trajectory buffer (bounded to max 1500 points for zero per-frame allocation)
      this.maxTrajectoryPoints = 1500;
      this.trajectoryPositions = new Float32Array(this.maxTrajectoryPoints * 3);
      this.trajectoryCount = 0;
      this.lastPoseTime = 0;

      // Camera view state
      this.perspectiveMode = 'iso'; // 'iso', 'top', 'side', 'follow'
      this.followDrone = false;
      this.lastDronePos = { x: 0, y: 0, z: 0 };

      // Visibility toggles
      this.showTrajectory = true;
      this.showLandmarks = true;
      this.showFrustum = true;
      this.showGrid = true;

      // Real landmarks array
      this.landmarks = [];

      // Extrinsics: p_BC = [0.10, 0, 0] m, OpenCV optical frame
      this.p_BC = [0.10, 0.0, 0.0];

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.init());
      } else {
        this.init();
      }
    }

    init() {
      this.container = document.getElementById('spatial-3d-viewport');
      this.canvas = document.getElementById('three-spatial-canvas');

      if (!this.container || !this.canvas) {
        console.warn('[VIO 3D] Container or canvas not found in DOM.');
        return;
      }

      // Verify WebGL and Three.js availability
      if (typeof window.THREE === 'undefined') {
        console.error('[VIO 3D] Three.js is not loaded. Cannot initialize 3D scene.');
        this._showWebglFallback('Three.js library not loaded. Check script imports.');
        return;
      }

      try {
        this._initThreeScene();
        this._initWorldFrame();
        this._initDroneModel();
        this._initCameraFrustum();
        this._initTrajectory();
        this._loadLandmarks();
        this._startRenderLoop();

        // Subscribe to real-time telemetry updates
        if (window.telemetry && typeof window.telemetry.onTelemetry === 'function') {
          window.telemetry.onTelemetry((state) => {
            if (!state || !state.position || state.position.x === null) {
              if (this.droneGroup) this.droneGroup.visible = false;
              if (this.trajectoryLine) this.trajectoryLine.visible = false;
              return;
            }
            if (this.droneGroup) this.droneGroup.visible = true;
            if (this.trajectoryLine) this.trajectoryLine.visible = this.showTrajectory;
            const p = state.position;
            const o = state.orientation;
            const v = state.velocity;
            this.updatePose(
              p.x, p.y, p.z,
              o.qw !== null ? o.qw : 1.0,
              o.qx !== null ? o.qx : 0.0,
              o.qy !== null ? o.qy : 0.0,
              o.qz !== null ? o.qz : 0.0,
              v.vx !== null ? v.vx : 0.0,
              v.vy !== null ? v.vy : 0.0,
              v.vz !== null ? v.vz : 0.0
            );
          });
        }

        // Listen for container resize
        window.addEventListener('resize', () => this._onResize());
        if (window.ResizeObserver) {
          const ro = new ResizeObserver(() => this._onResize());
          ro.observe(this.container);
        }

        this.isInitialized = true;
        console.log('[VIO 3D] Three.js Spatial Map initialized successfully in ENU frame.');
      } catch (err) {
        console.error('[VIO 3D] WebGL initialization failed:', err);
        this._showWebglFallback(`WebGL 3D Context Failed: ${err.message}`);
      }
    }

    _showWebglFallback(message) {
      this.webglAvailable = false;
      if (this.container) {
        this.container.innerHTML = `
          <div class="flex flex-col items-center justify-center p-6 text-center text-stone-300 space-y-2">
            <svg class="w-8 h-8 text-amber-500" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.964-.833-2.732 0L3.732 16.5c-.77.833.192 2.5 1.732 2.5z"/></svg>
            <div class="font-bold text-white text-sm">3D WebGL Unavailable</div>
            <div class="text-xs text-stone-400 font-mono">${message}</div>
          </div>
        `;
      }
    }

    _initThreeScene() {
      const width = this.container.clientWidth || 600;
      const height = this.container.clientHeight || 350;

      // 1. Scene
      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x070a0f); // Deep dark robotics slate

      // 2. Camera with ENU up vector (+Z Up)
      this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 200);
      this.camera.up.set(0, 0, 1); // Crucial: World Up is +Z in ENU
      this.camera.position.set(10, -14, 9); // Isometric viewpoint

      // 3. WebGL Renderer
      this.renderer = new THREE.WebGLRenderer({
        canvas: this.canvas,
        antialias: true,
        alpha: false,
        powerPreference: 'high-performance'
      });
      this.renderer.setSize(width, height);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

      // 4. OrbitControls
      if (typeof THREE.OrbitControls !== 'undefined') {
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0, 0, 2); // Look toward hover center
        this.controls.maxDistance = 80;
        this.controls.minDistance = 1;
        this.controls.update();
      }

      // 5. Lighting
      const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
      this.scene.add(ambientLight);

      const dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
      dirLight.position.set(15, -10, 25);
      this.scene.add(dirLight);
    }

    _initWorldFrame() {
      // Ground plane grid helper in XY plane at Z=0
      // GridHelper default is XZ plane; rotate by Math.PI / 2 around X to lay in XY plane
      const grid = new THREE.GridHelper(30, 30, 0x0ea5e9, 0x1e293b);
      grid.rotation.x = Math.PI / 2;
      grid.position.set(0, 0, 0);
      this.gridHelper = grid;
      this.scene.add(grid);

      // World Coordinate Triad at Origin (0,0,0)
      this.worldTriad = new THREE.Group();

      // Axis lengths: 2.0m
      const len = 2.0;

      // X-axis: East (Red)
      const xMat = new THREE.LineBasicMaterial({ color: 0xef4444, linewidth: 2 });
      const xGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(len, 0, 0)]);
      this.worldTriad.add(new THREE.Line(xGeo, xMat));

      // Y-axis: North (Green)
      const yMat = new THREE.LineBasicMaterial({ color: 0x10b981, linewidth: 2 });
      const yGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, len, 0)]);
      this.worldTriad.add(new THREE.Line(yGeo, yMat));

      // Z-axis: Up (Blue)
      const zMat = new THREE.LineBasicMaterial({ color: 0x3b82f6, linewidth: 2 });
      const zGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, len)]);
      this.worldTriad.add(new THREE.Line(zGeo, zMat));

      this.scene.add(this.worldTriad);
    }

    _initDroneModel() {
      this.droneGroup = new THREE.Group();

      // Center avionics fuselage hub
      const hubGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.06, 16);
      hubGeo.rotateX(Math.PI / 2); // align with body Z
      const hubMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.4 });
      const hub = new THREE.Mesh(hubGeo, hubMat);
      this.droneGroup.add(hub);

      // Quadrotor arms in X-configuration (arm radius ~0.28m)
      const armLen = 0.56;
      const armMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.5 });
      const armGeo = new THREE.CylinderGeometry(0.015, 0.015, armLen, 8);
      armGeo.rotateZ(Math.PI / 2);

      // Arm 1 (diagonal)
      const arm1 = new THREE.Mesh(armGeo, armMat);
      arm1.rotation.z = Math.PI / 4;
      this.droneGroup.add(arm1);

      // Arm 2 (orthogonal diagonal)
      const arm2 = new THREE.Mesh(armGeo, armMat);
      arm2.rotation.z = -Math.PI / 4;
      this.droneGroup.add(arm2);

      // 4 Rotor Discs
      const rotorR = 0.10;
      const d = 0.20; // offset in X and Y
      const rotorPos = [
        { x: d, y: d, color: 0x10b981 },  // Front-Left (Green)
        { x: d, y: -d, color: 0x10b981 }, // Front-Right (Green)
        { x: -d, y: d, color: 0x0284c7 }, // Rear-Left (Blue)
        { x: -d, y: -d, color: 0x0284c7 } // Rear-Right (Blue)
      ];

      rotorPos.forEach((rp) => {
        const rGeo = new THREE.CylinderGeometry(rotorR, rotorR, 0.008, 16);
        rGeo.rotateX(Math.PI / 2);
        const rMat = new THREE.MeshStandardMaterial({
          color: rp.color,
          transparent: true,
          opacity: 0.65
        });
        const rMesh = new THREE.Mesh(rGeo, rMat);
        rMesh.position.set(rp.x, rp.y, 0.03);
        this.droneGroup.add(rMesh);
      });

      // Body Frame Coordinate Triad (Body X Red Forward, Body Y Green Left, Body Z Blue Up)
      const bodyTriadLen = 0.35;
      // Body X (+X forward)
      const bxMat = new THREE.LineBasicMaterial({ color: 0xef4444, linewidth: 2 });
      const bxGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(bodyTriadLen, 0, 0)]);
      this.droneGroup.add(new THREE.Line(bxGeo, bxMat));

      // Body Y (+Y left)
      const byMat = new THREE.LineBasicMaterial({ color: 0x10b981, linewidth: 2 });
      const byGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, bodyTriadLen, 0)]);
      this.droneGroup.add(new THREE.Line(byGeo, byMat));

      // Body Z (+Z up)
      const bzMat = new THREE.LineBasicMaterial({ color: 0x3b82f6, linewidth: 2 });
      const bzGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, bodyTriadLen)]);
      this.droneGroup.add(new THREE.Line(bzGeo, bzMat));

      // Heading forward arrow tip
      const arrowMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
      const arrowGeo = new THREE.ConeGeometry(0.04, 0.12, 8);
      arrowGeo.rotateZ(-Math.PI / 2);
      const arrowMesh = new THREE.Mesh(arrowGeo, arrowMat);
      arrowMesh.position.set(bodyTriadLen + 0.06, 0, 0);
      this.droneGroup.add(arrowMesh);

      // Initial drone position at hover equilibrium
      this.droneGroup.position.set(0, 0, 2);
      this.scene.add(this.droneGroup);
    }

    _initCameraFrustum() {
      this.cameraGroup = new THREE.Group();

      // Camera body box
      const camBoxGeo = new THREE.BoxGeometry(0.05, 0.04, 0.04);
      const camBoxMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b }); // Amber
      const camBox = new THREE.Mesh(camBoxGeo, camBoxMat);
      this.cameraGroup.add(camBox);

      // Camera optical viewing frustum cone / wireframe
      // Optical frame convention: +Z optical is depth forward, +X is right, +Y is down
      // When aligned with OpenCV convention:
      const fovLen = 0.55;
      const fovW = 0.38;
      const fovH = 0.28;

      // Frustum 5 vertices: Origin, Top-Left, Top-Right, Bottom-Right, Bottom-Left
      // In Camera Optical Frame: Z forward, X right, Y down
      // Converting to Body ENU:
      // R_BC = [[0,0,1],[-1,0,0],[0,-1,0]] => x_body = z_cam, y_body = -x_cam, z_body = -y_cam
      const v0 = new THREE.Vector3(0, 0, 0);
      const vTL = new THREE.Vector3(fovLen, fovW, fovH);   // forward, left, up
      const vTR = new THREE.Vector3(fovLen, -fovW, fovH);  // forward, right, up
      const vBR = new THREE.Vector3(fovLen, -fovW, -fovH); // forward, right, down
      const vBL = new THREE.Vector3(fovLen, fovW, -fovH);  // forward, left, down

      const frustumPts = [
        v0, vTL, vTR, v0, vBR, vBL, v0,
        vTL, vBL, vBR, vTR, vTL
      ];

      const frustumGeo = new THREE.BufferGeometry().setFromPoints(frustumPts);
      const frustumMat = new THREE.LineBasicMaterial({
        color: 0x38bdf8, // Sky cyan
        transparent: true,
        opacity: 0.65
      });
      const frustumLine = new THREE.Line(frustumGeo, frustumMat);
      this.cameraGroup.add(frustumLine);

      // Offset camera by extrinsic p_BC = [+0.10, 0, 0] relative to drone
      this.cameraGroup.position.set(this.p_BC[0], this.p_BC[1], this.p_BC[2]);
      this.droneGroup.add(this.cameraGroup);
    }

    _initTrajectory() {
      this.trajectoryGeo = new THREE.BufferGeometry();
      this.trajectoryGeo.setAttribute('position', new THREE.BufferAttribute(this.trajectoryPositions, 3));
      this.trajectoryGeo.setDrawRange(0, 0);

      const trajMat = new THREE.LineBasicMaterial({
        color: 0x00f5ff, // Bright high-contrast cyan
        linewidth: 2,
        transparent: true,
        opacity: 0.85
      });

      this.trajectoryLine = new THREE.Line(this.trajectoryGeo, trajMat);
      this.scene.add(this.trajectoryLine);
    }

    _loadLandmarks() {
      // In LIVE mode, start with empty landmarks (awaiting live pointcloud)
      if (window.APP_CONFIG && window.APP_CONFIG.mode === 'LIVE') {
        this.landmarks = [];
        const sideCount = document.getElementById('sidebar-pointcount');
        if (sideCount) sideCount.textContent = '0 pts';
        const hudPts = document.getElementById('hud-map-pts');
        if (hudPts) hudPts.textContent = '0';
        return;
      }

      // In DEMO mode, load dataset PCD
      if (window.REAL_DRONE_DATA && Array.isArray(window.REAL_DRONE_DATA.pcd_points)) {
        this.landmarks = window.REAL_DRONE_DATA.pcd_points;
      }

      if (this.landmarks.length === 0) {
        fetch('/api/pointcloud')
          .then((res) => res.json())
          .then((pts) => {
            if (Array.isArray(pts) && pts.length > 0) {
              this.landmarks = pts;
              this._buildLandmarksMesh();
            }
          })
          .catch(() => {});
        return;
      }

      this._buildLandmarksMesh();
    }

    _buildLandmarksMesh() {
      if (!this.landmarks || this.landmarks.length === 0) return;

      const count = this.landmarks.length;
      const positions = new Float32Array(count * 3);
      const colors = new Float32Array(count * 3);

      // Height colormap (Z elevation from min to max)
      let minZ = Infinity;
      let maxZ = -Infinity;
      for (let i = 0; i < count; i++) {
        const z = this.landmarks[i][2];
        if (z < minZ) minZ = z;
        if (z > maxZ) maxZ = z;
      }
      const rangeZ = Math.max(0.1, maxZ - minZ);

      const colorLow = new THREE.Color(0x0284c7);  // Sky-blue
      const colorMid = new THREE.Color(0x14b8a6);  // Teal
      const colorHigh = new THREE.Color(0x10b981); // Emerald

      for (let i = 0; i < count; i++) {
        const pt = this.landmarks[i];
        positions[i * 3 + 0] = pt[0];
        positions[i * 3 + 1] = pt[1];
        positions[i * 3 + 2] = pt[2];

        // Normalized elevation [0, 1]
        const normZ = (pt[2] - minZ) / rangeZ;
        const col = new THREE.Color();
        if (normZ < 0.5) {
          col.lerpColors(colorLow, colorMid, normZ * 2);
        } else {
          col.lerpColors(colorMid, colorHigh, (normZ - 0.5) * 2);
        }

        colors[i * 3 + 0] = col.r;
        colors[i * 3 + 1] = col.g;
        colors[i * 3 + 2] = col.b;
      }

      const pcdGeo = new THREE.BufferGeometry();
      pcdGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      pcdGeo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

      const pcdMat = new THREE.PointsMaterial({
        size: 3.2,
        vertexColors: true,
        sizeAttenuation: false,
        transparent: true,
        opacity: 0.88
      });

      if (this.landmarksPoints) {
        this.scene.remove(this.landmarksPoints);
      }

      this.landmarksPoints = new THREE.Points(pcdGeo, pcdMat);
      this.scene.add(this.landmarksPoints);

      // Update sidebar and HUD counts
      const sideCount = document.getElementById('sidebar-pointcount');
      if (sideCount) sideCount.textContent = `${count} pts`;
      const hudPts = document.getElementById('hud-map-pts');
      if (hudPts) hudPts.textContent = `${count}`;
    }

    /**
     * Primary Real-time Pose Ingestion Method
     * Called by telemetry playback and WebSocket bridge
     */
    updatePose(px, py, pz, qw, qx, qy, qz, vx, vy, vz) {
      if (!this.isInitialized || !this.droneGroup) return;

      // 1. Update Drone Body Position
      this.droneGroup.position.set(px, py, pz);

      // 2. Update Drone Body Orientation
      // Three.js Quaternion constructor is (x, y, z, w)
      this.droneGroup.quaternion.set(qx, qy, qz, qw);

      this.lastDronePos = { x: px, y: py, z: pz };

      // 3. Append to Trajectory History (throttled to min 0.04s interval or distance delta)
      const now = performance.now();
      if (now - this.lastPoseTime >= 40) {
        this._appendTrajectory(px, py, pz);
        this.lastPoseTime = now;
      }

      // 4. Update HUD badges
      const hudPw = document.getElementById('hud-pw');
      if (hudPw) {
        hudPw.innerHTML = `P_WB: [ <span class="text-white">x: ${px.toFixed(2)}</span>, <span class="text-white">y: ${py.toFixed(2)}</span>, <span class="text-emerald-400">z: ${pz.toFixed(2)}</span> ] m`;
      }
      const hudVw = document.getElementById('hud-vw');
      if (hudVw && vx !== undefined) {
        hudVw.innerHTML = `V_WB: [ <span class="text-white">vx: ${vx.toFixed(2)}</span>, <span class="text-white">vy: ${vy.toFixed(2)}</span>, <span class="text-emerald-400">vz: ${vz.toFixed(2)}</span> ] m/s`;
      }

      // 5. Follow Mode update
      if (this.followDrone && this.controls) {
        this.controls.target.set(px, py, pz);
      }
    }

    _appendTrajectory(x, y, z) {
      if (!this.trajectoryGeo) return;

      if (this.trajectoryCount < this.maxTrajectoryPoints) {
        const idx = this.trajectoryCount * 3;
        this.trajectoryPositions[idx + 0] = x;
        this.trajectoryPositions[idx + 1] = y;
        this.trajectoryPositions[idx + 2] = z;
        this.trajectoryCount++;
      } else {
        // Shift left by 1 point to keep ring buffer
        this.trajectoryPositions.copyWithin(0, 3);
        const idx = (this.maxTrajectoryPoints - 1) * 3;
        this.trajectoryPositions[idx + 0] = x;
        this.trajectoryPositions[idx + 1] = y;
        this.trajectoryPositions[idx + 2] = z;
      }

      this.trajectoryGeo.attributes.position.needsUpdate = true;
      this.trajectoryGeo.setDrawRange(0, this.trajectoryCount);
    }

    clearTrajectory() {
      this.trajectoryCount = 0;
      if (this.trajectoryGeo) {
        this.trajectoryGeo.setDrawRange(0, 0);
      }
    }

    setPerspective(mode) {
      this.perspectiveMode = mode;
      this.followDrone = (mode === 'follow');

      // Update button styling in DOM
      ['iso', 'top', 'side', 'follow'].forEach((m) => {
        const btn = document.getElementById(`btn-persp-${m}`);
        if (btn) {
          const isActive = (m === mode);
          btn.classList.toggle('bg-white', isActive);
          btn.classList.toggle('text-stone-900', isActive);
          btn.classList.toggle('shadow-sm', isActive);
          btn.classList.toggle('text-stone-600', !isActive);
        }
      });

      const tx = this.lastDronePos.x || 0;
      const ty = this.lastDronePos.y || 0;
      const tz = this.lastDronePos.z || 2;

      if (mode === 'top') {
        // Bird's eye looking straight down (+Z)
        this.camera.position.set(tx, ty, tz + 20);
        if (this.controls) this.controls.target.set(tx, ty, tz);
      } else if (mode === 'side') {
        // Side view looking North along +Y
        this.camera.position.set(tx, ty - 22, tz + 2);
        if (this.controls) this.controls.target.set(tx, ty, tz);
      } else if (mode === 'follow') {
        // Chase view behind drone
        this.camera.position.set(tx - 6, ty - 6, tz + 4);
        if (this.controls) this.controls.target.set(tx, ty, tz);
      } else {
        // Default Isometric
        this.camera.position.set(tx + 10, ty - 14, tz + 9);
        if (this.controls) this.controls.target.set(tx, ty, tz);
      }

      if (this.controls) this.controls.update();
      if (window.showToast) {
        window.showToast(`3D Camera View: ${mode.toUpperCase()}`);
      }
    }

    centerOnDrone() {
      const tx = this.lastDronePos.x || 0;
      const ty = this.lastDronePos.y || 0;
      const tz = this.lastDronePos.z || 2;

      if (this.controls) {
        this.controls.target.set(tx, ty, tz);
        this.controls.update();
      }
      if (window.showToast) {
        window.showToast('Viewport centered on body pose T_WB.');
      }
    }

    resetView() {
      this.setPerspective('iso');
    }

    toggleTrajectory() {
      this.showTrajectory = !this.showTrajectory;
      if (this.trajectoryLine) {
        this.trajectoryLine.visible = this.showTrajectory;
      }
      if (window.showToast) {
        window.showToast(`Trajectory Trail: ${this.showTrajectory ? 'Visible' : 'Hidden'}`);
      }
    }

    toggleLandmarks() {
      this.showLandmarks = !this.showLandmarks;
      if (this.landmarksPoints) {
        this.landmarksPoints.visible = this.showLandmarks;
      }
      if (window.showToast) {
        window.showToast(`Landmark Point Cloud: ${this.showLandmarks ? 'Visible' : 'Hidden'}`);
      }
    }

    toggleCameraFrustum() {
      this.showFrustum = !this.showFrustum;
      if (this.cameraGroup) {
        this.cameraGroup.visible = this.showFrustum;
      }
      if (window.showToast) {
        window.showToast(`Camera Optical Frustum: ${this.showFrustum ? 'Visible' : 'Hidden'}`);
      }
    }

    _onResize() {
      if (!this.container || !this.renderer || !this.camera) return;
      const width = this.container.clientWidth || 600;
      const height = this.container.clientHeight || 350;

      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height);
    }

    _startRenderLoop() {
      const animate = () => {
        requestAnimationFrame(animate);

        if (this.controls) {
          this.controls.update();
        }

        if (this.renderer && this.scene && this.camera) {
          this.renderer.render(this.scene, this.camera);
        }
      };
      animate();
    }
  }

  // Expose singleton on window
  window.vio = new VioSpatialVisualizer();

})(window);
