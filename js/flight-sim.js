/**
 * Drone 3D Flight Simulator, Environment, Aerodynamics & Audio Synthesizer
 * Built with Three.js and Web Audio API
 */

class DroneFlightSimulator {
  constructor(canvasContainer) {
    this.container = canvasContainer;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.clock = new THREE.Clock();

    // 3D Objects
    this.droneMesh = null;
    this.droneBuilder = null;
    this.windsock = null;
    this.helipad = null;
    this.towerLight = null;
    this.checkpoints = [];
    this.activeCheckpoints = [];

    // Flight State
    this.isArmed = false;
    this.isLanded = true;
    this.isCrashed = false;
    this.isFPV = true; // Default to FPV (First-Person View)
    this.isHoldMode = false;
    this.isRTHMode = false;
    this.isMuted = false;

    // Physics 6DOF State
    this.position = new THREE.Vector3(0, 0.48, 0); // starts on helipad
    this.velocity = new THREE.Vector3(0, 0, 0);
    this.rotation = new THREE.Euler(0, 0, 0, 'YXZ');
    this.angularVelocity = new THREE.Vector3(0, 0, 0);

    // Realistic Quadrotor Propulsion, Aerodynamics, Battery & Flight Controller State
    this.physicsParams = null;
    this.motorRpm = [0, 0, 0, 0]; // normalized [0..1] RPM for [FL(CW), RL(CCW), RR(CW), FR(CCW)]
    this.holdAltitude = 0.48;
    this.altHoldLocked = true;
    this.holdTargetPos = new THREE.Vector3(0, 0.48, 0);
    this.holdTargetLocked = false;
    this.windEst = new THREE.Vector2(0, 0);
    this.rthPhase = 'IDLE'; // 'IDLE' | 'CLIMB' | 'PATH_RETURN' | 'RETURN' | 'DESCEND'
    this.rthCruiseAlt = 22;
    this.flightPathHistory = []; // Odometry breadcrumb trail [{ x, y, z }] recorded every 3.5m for sensor-loss backtracking!
    this.rthWaypoints = [];      // Calculated obstacle-avoiding return waypoints [{ x, y, z, kind }]
    this.rthWaypointIdx = 0;
    this.rthAlgoLabel = '';      // 'A* หลบสิ่งกีดขวาง' | 'Breadcrumb ย้อนรอย + A*' | 'IMU ย้อนรอยจุดเริ่มต้น'
    this.rthTotalPathDist = 0;
    this.isFailsafeReturn = false;
    this.sensorLostSim = false;
    this.signalLossTimer = 0;
    this.rthPathVisuals = null;
    this.liveWindVec = new THREE.Vector3(0, 0, 0);
    this.liveWindSpeed = 0;
    this.liveWindDirRad = Math.atan2(0.4, 0.7);
    this.liveWindDirDeg = 60;
    this.liveWindVertical = 0;
    this.windPreset = 'mission'; // 'mission' | 'calm' | 'moderate' | 'strong' | 'storm'
    this.windShelterFactor = 1.0; // 1.0 open sky -> 0.15 sheltered inside hangar
    this.windGustFactor = 1.0;
    this.windGustActive = false;
    this.windLevel = 'calm'; // 'calm' | 'moderate' | 'strong' | 'storm'
    this.windStreaks = null;
    this.windStreakData = [];
    this._lastSprayDriftWarnMs = 0;
    this.vrsIntensity = 0;
    this.igeFactor = 1.0;
    this.airDensity = 1.225;
    this.batteryVoltage = 16.8;
    this.currentAmps = 0;
    this.simTime = 0;
    this.surfaceSupportY = 0.48;
    this.hudAlertText = '';
    this.hudAlertUntil = 0;

    // 12-Part Subsystem Live State (RX RSSI, RF Signal Dead Zones, LiDAR, Agri Sprayer / Cargo, Recovery Parachute)
    this.payloadPercent = 100;
    this.isSpraying = false;
    this.cargoAttached = true;
    this.parachuteDeployed = false;
    this.rssiPercent = 100;
    this.smoothedRssi = 100;
    this.signalState = 'good'; // 'good' | 'weak' | 'critical'
    this.signalSource = 'clear'; // 'clear' | 'tree' | 'hangar' | 'building' | 'emi_5g' | 'emi_radar' | 'terrain' | 'range'
    this.signalObstruction = 0; // 0..1 combined obstruction/EMI severity
    this.gpsMultipathFactor = 1.0;
    this.insideMetalRoof = false;
    this._lastSignalState = 'good';
    this._lastSignalBeepMs = 0;
    this.lidarBraking = false;
    this.lidarZone = 'off'; // 'off' | 'clear' | 'detect' | 'brake'
    this.lidarNearestDist = null;
    this.lidarNearestDirTh = '';
    this.lidarNearestTag = '';
    this.lidarNearestPoint = new THREE.Vector3();
    this.groundClearanceM = 0; // Exact AGL distance from drone bottom to ground/roof below
    this.homeAltitudeM = 0;    // Barometric altitude relative to home takeoff pad
    this.surfaceBelowY = 0;
    this.altSensorType = 'BARO'; // 'RADAR+TOF' | 'SONAR' | 'BARO' | 'NONE'
    this.altSensorLocked = false;
    this.sensorVisuals = null;
    this.sprayPoints = null;
    this.sprayData = [];
    this.droppedCargoMesh = null;
    this.droppedCargoVel = new THREE.Vector3();
    this.droppedCargoLanded = true;

    // Live Thailand Drone Law (CAAT / NBTC) Compliance & Frame Altitude Ceiling Monitoring
    this.lawOverCeiling = false;
    this.lawAltitudeViolated = false;
    this.lawVlosWarned = false;
    this.lawVillageWarned = false;
    this._lastLawBeepMs = 0;
    this.hitAltitudeLimit = false;
    this._lastAltLimitAlertMs = 0;

    // Real-World Start / Stop Propulsion State Machine (DISARMED / PRE_ARMING / ARMED_IDLE / AUTO_TAKEOFF / FLYING / AUTO_LANDING)
    this.powerState = 'DISARMED';
    this.preArmTimer = 0;
    this.pendingAutoTakeoff = false;
    this.autoTakeoffTargetY = 3.48;
    this.autoLandSettleTimer = 0;
    this.touchdownHoldTimer = 0;
    this.emergencyKillArmUntil = 0;

    // Reusable scratch vectors (zero per-substep GC allocation!)
    this._bodyUp = new THREE.Vector3(0, 1, 0);
    this._relAir = new THREE.Vector3();
    this._prevPos = new THREE.Vector3();
    this._moveDelta = new THREE.Vector3();
    this._hDir = new THREE.Vector3();
    this._windTmp = new THREE.Vector3();

    // Drone Specs & Current telemetry
    this.stats = null;
    this.mission = null;
    this.altitude = 0;
    this.speedKmh = 0;
    this.maxAltitude = 0;
    this.flightTimeSeconds = 0;
    this.batteryPercent = 100;
    this.completedCheckpoints = 0;

    // Controls input states & critically-damped 2nd-order S-curve filter velocities
    this.keys = {};
    this.controlRateMode = 'normal'; // 'cine' | 'normal' | 'sport' (real DJI/RC flight rate switch!)
    this.input = {
      throttle: 0, // -1 to 1
      pitch: 0,    // -1 to 1
      roll: 0,     // -1 to 1
      yaw: 0       // -1 to 1
    };
    this.inputVel = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
    this.smoothTargetPitch = 0;
    this.smoothTargetRoll = 0;
    this.smoothTargetYawRate = 0;
    this.smoothTargetVz = 0;
    this.brakeRamp = 0;

    // Camera rig (Silky Polar Spring-Arm Chase Cam behind drone tail — zero jerks & zero house see-through!)
    this.camOffset = new THREE.Vector3(0, 1.45, -3.45);
    this.camLookOffset = new THREE.Vector3(0, 0.25, 3.5);
    this.camYaw = 0;
    this.camPitch = 0.16;
    this.targetCamYaw = 0;
    this.targetCamPitch = 0.16;
    this.smoothChasePos = new THREE.Vector3(0, 0.8, 0);
    this.smoothChaseYaw = 0;
    this.smoothChasePitch = 0.16;
    this.currentCamDist = 3.75;
    this.smoothFpvQuat = new THREE.Quaternion();
    this.camInitialized = false;
    this.smoothLookTarget = new THREE.Vector3(0, 0.6, 3.5);
    this.isDraggingMouse = false;
    this.lastMouseX = 0;
    this.lastMouseY = 0;

    // Performance & Graphics Quality State ('fast' | 'balanced' | 'high')
    this.graphicsQuality = 'balanced';
    try {
      const savedQ = localStorage.getItem('droneBuilder_gfx_v1');
      if (savedQ === 'fast' || savedQ === 'balanced' || savedQ === 'high') {
        this.graphicsQuality = savedQ;
      }
    } catch (e) {}
    this.loopActive = false;
    this.rafId = null;
    this._boundUpdate = this.update.bind(this);
    this._simScreenEl = null;
    this._shadowFrameCount = 0;
    this._fpsFrames = 0;
    this._fpsLastTime = performance.now();
    this.liveFps = 60;
    this.droneContactShadow = null;

    // Audio Engine
    this.audioCtx = null;
    this.motorOsc1 = null;
    this.motorOsc2 = null;
    this.motorGain = null;
    this.audioInitialized = false;

    // Callbacks
    this.onTelemetryUpdate = null;
    this.onFlightFinished = null;
    this.onCheckpointPassed = null;

    this.initScene();
    this.initControls();
  }

  initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x6fa8dc);
    this.scene.fog = new THREE.FogExp2(0xcbd9ea, 0.00072);

    const width = this.container.clientWidth || window.innerWidth;
    const height = this.container.clientHeight || window.innerHeight;

    this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1050);
    this.camera.position.set(0, 3, -6);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.0));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.autoUpdate = false; // Smart throttled shadow updates (cuts GPU shadow work 50-100%!)
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.02;
    if (THREE.sRGBEncoding) this.renderer.outputEncoding = THREE.sRGBEncoding;

    this.container.appendChild(this.renderer.domElement);

    // High-Resolution Atmospheric Sky Dome + Real PMREM Environment Reflections
    this.buildSkyDome();

    // Rich Natural Outdoor Lighting (Deep Shadow Contrast + Warm Sunlight + Cool Sky Bounce)
    const hemiLight = new THREE.HemisphereLight(0x93c5fd, 0x162918, 0.62);
    hemiLight.position.set(0, 140, 0);
    this.scene.add(hemiLight);

    // Follow-Camera 1024x1024 Texel-Snapped Sun Shadow (tight ±36m frustum = sharper shadows + 2x faster shadow pass!)
    this.sunOffset = new THREE.Vector3(95, 115, -75);
    this._lastShadowSnapX = null;
    this._lastShadowSnapZ = null;
    this._lastTelemetryTime = 0;
    this._lastTelemetryState = '';
    this.sunLight = new THREE.DirectionalLight(0xffe2bc, 1.52);
    this.sunLight.position.copy(this.sunOffset);
    this.sunLight.castShadow = true;
    this.sunLight.shadow.mapSize.width = 1024;
    this.sunLight.shadow.mapSize.height = 1024;
    this.sunLight.shadow.bias = -0.0004;
    this.sunLight.shadow.normalBias = 0.035;
    this.sunLight.shadow.camera.near = 20;
    this.sunLight.shadow.camera.far = 240;
    const d = 36;
    this.sunLight.shadow.camera.left = -d;
    this.sunLight.shadow.camera.right = d;
    this.sunLight.shadow.camera.top = d;
    this.sunLight.shadow.camera.bottom = -d;
    this.scene.add(this.sunLight);
    this.scene.add(this.sunLight.target);

    // Subtle cool sky rim fill from opposite side for crisp 3D edge definition
    const rimLight = new THREE.DirectionalLight(0x7dd3fc, 0.28);
    rimLight.position.set(-140, 55, 110);
    this.scene.add(rimLight);

    // Lightweight soft radial ground contact shadow beneath drone (always gives crisp depth cue, essential in Fast 60 FPS mode!)
    this.initDroneContactShadow();

    // Build World Environment (default procedural world + auto-upgrade if assets/map.glb exists)
    this.proceduralWorldGroup = new THREE.Group();
    this.scene.add(this.proceduralWorldGroup);
    this.customMapRoot = null;
    this.solidTerrainMeshes = [];
    this.colliders = [];
    this.powerWireSegments = [];
    this.groundRaycaster = new THREE.Raycaster();
    this.wallRaycaster = new THREE.Raycaster();
    this.buildWorld();
    this.tryLoadCustomMapGLB();
    this.setGraphicsQuality(this.graphicsQuality, true);

    window.addEventListener('resize', () => this.onWindowResize());
  }

  initDroneContactShadow() {
    if (typeof document === 'undefined' || !this.scene) return;
    const c = document.createElement('canvas');
    c.width = 64;
    c.height = 64;
    const ctx = c.getContext('2d');
    const g = ctx.createRadialGradient(32, 32, 4, 32, 32, 30);
    g.addColorStop(0, 'rgba(0, 0, 0, 0.52)');
    g.addColorStop(0.55, 'rgba(0, 0, 0, 0.22)');
    g.addColorStop(1, 'rgba(0, 0, 0, 0)');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, 64, 64);
    const tex = new THREE.CanvasTexture(c);
    const mat = new THREE.MeshBasicMaterial({
      map: tex,
      transparent: true,
      opacity: 0.72,
      depthWrite: false
    });
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1.7, 1.7), mat);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.set(0, 0.06, 0);
    mesh.renderOrder = 2;
    this.scene.add(mesh);
    this.droneContactShadow = mesh;
  }

  setGraphicsQuality(mode = 'balanced', silent = false) {
    const valid = (mode === 'fast' || mode === 'balanced' || mode === 'high') ? mode : 'balanced';
    this.graphicsQuality = valid;
    try { localStorage.setItem('droneBuilder_gfx_v1', valid); } catch (e) {}

    const dpr = (typeof window !== 'undefined' && window.devicePixelRatio) ? window.devicePixelRatio : 1;
    if (this.renderer && this.camera && this.sunLight) {
      if (valid === 'fast') {
        this.renderer.setPixelRatio(Math.min(dpr, 0.85));
        this.sunLight.castShadow = false;
        this.renderer.shadowMap.enabled = false;
        this.renderer.shadowMap.autoUpdate = false;
        this.camera.far = 680;
        if (this.scene && this.scene.fog) this.scene.fog.density = 0.0011;
      } else if (valid === 'high') {
        this.renderer.setPixelRatio(Math.min(dpr, 1.35));
        this.sunLight.castShadow = true;
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.autoUpdate = true;
        this.renderer.shadowMap.needsUpdate = true;
        const d = 48;
        this.sunLight.shadow.camera.left = -d;
        this.sunLight.shadow.camera.right = d;
        this.sunLight.shadow.camera.top = d;
        this.sunLight.shadow.camera.bottom = -d;
        this.sunLight.shadow.camera.updateProjectionMatrix();
        this.camera.far = 1400;
        if (this.scene && this.scene.fog) this.scene.fog.density = 0.00072;
      } else {
        // Balanced (Default): sharp shadows updated every 2nd frame, crisp 1x pixelRatio, Lambert matte world!
        this.renderer.setPixelRatio(Math.min(dpr, 1.0));
        this.sunLight.castShadow = true;
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.autoUpdate = false;
        this.renderer.shadowMap.needsUpdate = true;
        const d = 36;
        this.sunLight.shadow.camera.left = -d;
        this.sunLight.shadow.camera.right = d;
        this.sunLight.shadow.camera.top = d;
        this.sunLight.shadow.camera.bottom = -d;
        this.sunLight.shadow.camera.updateProjectionMatrix();
        this.camera.far = 1050;
        if (this.scene && this.scene.fog) this.scene.fog.density = 0.00078;
      }
      this.camera.updateProjectionMatrix();
      this.onWindowResize();
    }

    if (!silent) {
      const labels = {
        fast: '⚡ กราฟิก: ลื่นสุด (ปิดเงาเรียลไทม์ · เน้น 60 FPS)',
        balanced: '⚖️ กราฟิก: สมดุล (เงาชัด + เฟรมเรตลื่น)',
        high: '✨ กราฟิก: คมชัดสูงสุด (Full Shadows + High DPI)'
      };
      this.playTone(valid === 'fast' ? 820 : valid === 'high' ? 560 : 680, 0.1);
      this.setHudAlert(labels[valid], 2400);
    }
    return valid;
  }

  cycleGraphicsQuality() {
    const order = ['balanced', 'fast', 'high'];
    const idx = order.indexOf(this.graphicsQuality || 'balanced');
    return this.setGraphicsQuality(order[(idx + 1) % order.length], false);
  }

  startLoop() {
    if (this.loopActive) return;
    this.loopActive = true;
    this.clock.getDelta(); // reset clock delta so resuming never jumps!
    this._fpsFrames = 0;
    this._fpsLastTime = performance.now();
    this.rafId = requestAnimationFrame(this._boundUpdate);
  }

  stopLoop() {
    this.loopActive = false;
    if (this.rafId !== null) {
      cancelAnimationFrame(this.rafId);
      this.rafId = null;
    }
    this.keys = {};
    if (this.audioInitialized && this.audioCtx) {
      if (this.motorGain) this.motorGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.05);
      if (this.noiseGain) this.noiseGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.05);
      if (this.windGain) this.windGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.05);
    }
  }

  addBoxCollider(cx, cy, cz, sx, sy, sz, tag = 'building') {
    this.colliders.push({
      type: 'box',
      tag,
      minX: cx - sx / 2, maxX: cx + sx / 2,
      minY: cy - sy / 2, maxY: cy + sy / 2,
      minZ: cz - sz / 2, maxZ: cz + sz / 2
    });
  }

  addCylCollider(x, z, radius, minY, maxY, tag = 'building') {
    this.colliders.push({ type: 'cyl', tag, x, z, radius, minY, maxY });
  }

  addConeCollider(x, z, baseR, height, tag = 'terrain') {
    this.colliders.push({ type: 'cone', tag, x, z, baseR, height });
  }

  tryLoadCustomMapGLB() {
    if (typeof THREE.GLTFLoader === 'undefined') return;
    const loader = new THREE.GLTFLoader();
    const maxAniso = this.renderer ? this.renderer.capabilities.getMaxAnisotropy() : 4;
    loader.load(
      'assets/map.glb',
      (gltf) => {
        const mapRoot = gltf.scene;
        // Auto-center and scale to an epic 420m alpine expanse
        const box = new THREE.Box3().setFromObject(mapRoot);
        const size = new THREE.Vector3();
        const center = new THREE.Vector3();
        box.getSize(size);
        box.getCenter(center);
        const maxSpan = Math.max(size.x, size.z, 1);
        const scaleFactor = 420 / maxSpan;
        mapRoot.scale.setScalar(scaleFactor);
        mapRoot.position.set(-center.x * scaleFactor, -box.min.y * scaleFactor, -center.z * scaleFactor);

        mapRoot.traverse((obj) => {
          if (obj.isMesh) {
            // Keep drone shadows on the terrain, disable self-shadowing artifacts
            obj.castShadow = false;
            obj.receiveShadow = true;

            // Smooth terrain mesh geometry (Taubin Smoothing removes jagged photogrammetry spikes
            // without shrinking the mountain, and welds UV seams so zero cracks appear!)
            const matsList = obj.material ? (Array.isArray(obj.material) ? obj.material : [obj.material]) : [];
            const isCloudMesh = matsList.some((m) => {
              const mn = (m && m.name ? m.name : '').toLowerCase();
              return mn.includes('cloud') || mn.includes('mist') || mn.includes('fog') || Boolean(m && m.transparent);
            });

            if (obj.geometry && obj.geometry.attributes && obj.geometry.attributes.position) {
              const geom = obj.geometry;
              geom.computeBoundingBox();
              const posAttr = geom.attributes.position;

              if (!isCloudMesh && geom.index && posAttr.count < 120000 && geom.boundingBox) {
                const gSize = new THREE.Vector3();
                geom.boundingBox.getSize(gSize);
                const weldScale = 1500 / Math.max(gSize.x, gSize.y, gSize.z, 0.001);

                // Group coincident vertices (across UV seams) into welded IDs
                const keyToWelded = new Map();
                const vertToWelded = new Int32Array(posAttr.count);
                const weldedX = [];
                const weldedY = [];
                const weldedZ = [];

                for (let i = 0; i < posAttr.count; i++) {
                  const px = posAttr.getX(i);
                  const py = posAttr.getY(i);
                  const pz = posAttr.getZ(i);
                  const key = `${Math.round(px * weldScale)},${Math.round(py * weldScale)},${Math.round(pz * weldScale)}`;
                  let wid = keyToWelded.get(key);
                  if (wid === undefined) {
                    wid = weldedX.length;
                    keyToWelded.set(key, wid);
                    weldedX.push(px);
                    weldedY.push(py);
                    weldedZ.push(pz);
                  }
                  vertToWelded[i] = wid;
                }

                // Build neighbor sets from triangle indices
                const wCount = weldedX.length;
                const neighbors = Array.from({ length: wCount }, () => new Set());
                const idxArr = geom.index.array;
                for (let t = 0; t < idxArr.length; t += 3) {
                  const a = vertToWelded[idxArr[t]];
                  const b = vertToWelded[idxArr[t + 1]];
                  const c = vertToWelded[idxArr[t + 2]];
                  if (a !== b) { neighbors[a].add(b); neighbors[b].add(a); }
                  if (b !== c) { neighbors[b].add(c); neighbors[c].add(b); }
                  if (c !== a) { neighbors[c].add(a); neighbors[a].add(c); }
                }

                // 3 iterations of Taubin smoothing (shrink λ=+0.45 then inflate μ=-0.42 -> zero volume loss!)
                let curX = Float32Array.from(weldedX);
                let curY = Float32Array.from(weldedY);
                let curZ = Float32Array.from(weldedZ);
                let nxtX = new Float32Array(wCount);
                let nxtY = new Float32Array(wCount);
                let nxtZ = new Float32Array(wCount);

                const taubinSteps = [0.45, -0.42, 0.45, -0.42, 0.35, -0.32];
                for (let s = 0; s < taubinSteps.length; s++) {
                  const factor = taubinSteps[s];
                  for (let w = 0; w < wCount; w++) {
                    const nb = neighbors[w];
                    if (nb.size < 3) {
                      // Preserve outer boundary vertices
                      nxtX[w] = curX[w];
                      nxtY[w] = curY[w];
                      nxtZ[w] = curZ[w];
                      continue;
                    }
                    let ax = 0, ay = 0, az = 0;
                    nb.forEach((n) => { ax += curX[n]; ay += curY[n]; az += curZ[n]; });
                    const inv = 1 / nb.size;
                    nxtX[w] = curX[w] + (ax * inv - curX[w]) * factor;
                    nxtY[w] = curY[w] + (ay * inv - curY[w]) * factor;
                    nxtZ[w] = curZ[w] + (az * inv - curZ[w]) * factor;
                  }
                  [curX, nxtX] = [nxtX, curX];
                  [curY, nxtY] = [nxtY, curY];
                  [curZ, nxtZ] = [nxtZ, curZ];
                }

                // Write smoothed coordinates back to geometry
                for (let i = 0; i < posAttr.count; i++) {
                  const wid = vertToWelded[i];
                  posAttr.setXYZ(i, curX[wid], curY[wid], curZ[wid]);
                }
                posAttr.needsUpdate = true;
              }

              // Recompute smooth welded vertex normals across the whole surface
              geom.computeVertexNormals();
              const normAttr = geom.attributes.normal;
              if (normAttr && posAttr.count < 500000 && geom.boundingBox) {
                const gSize = new THREE.Vector3();
                geom.boundingBox.getSize(gSize);
                const cellScale = 2000 / Math.max(gSize.x, gSize.y, gSize.z, 0.001);
                const buckets = new Map();
                for (let i = 0; i < posAttr.count; i++) {
                  const key = `${Math.round(posAttr.getX(i) * cellScale)},${Math.round(posAttr.getY(i) * cellScale)},${Math.round(posAttr.getZ(i) * cellScale)}`;
                  let acc = buckets.get(key);
                  if (!acc) {
                    acc = [0, 0, 0];
                    buckets.set(key, acc);
                  }
                  acc[0] += normAttr.getX(i);
                  acc[1] += normAttr.getY(i);
                  acc[2] += normAttr.getZ(i);
                }
                for (let i = 0; i < posAttr.count; i++) {
                  const key = `${Math.round(posAttr.getX(i) * cellScale)},${Math.round(posAttr.getY(i) * cellScale)},${Math.round(posAttr.getZ(i) * cellScale)}`;
                  const acc = buckets.get(key);
                  const len = Math.hypot(acc[0], acc[1], acc[2]) || 1;
                  normAttr.setXYZ(i, acc[0] / len, acc[1] / len, acc[2] / len);
                }
                normAttr.needsUpdate = true;
              }
            }

            if (obj.material) {
              const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
              mats.forEach((m, idx) => {
                const matName = (m.name || '').toLowerCase();
                const isCloud = matName.includes('cloud') || matName.includes('mist') || matName.includes('fog') || m.transparent;

                // Apply Trilinear + Max Anisotropic filtering to all textures
                [m.map, m.normalMap, m.roughnessMap, m.emissiveMap].forEach((tex) => {
                  if (!tex) return;
                  tex.anisotropy = maxAniso;
                  tex.generateMipmaps = true;
                  tex.minFilter = THREE.LinearMipmapLinearFilter;
                  tex.magFilter = THREE.LinearFilter;
                  tex.needsUpdate = true;
                });

                if (isCloud) {
                  // Replace harsh faceted PBR lighting on cloud meshes with silky soft unlit mist shading
                  const smoothCloudMat = new THREE.MeshBasicMaterial({
                    map: m.map || null,
                    color: new THREE.Color(0xf8fafc),
                    transparent: true,
                    opacity: 0.78,
                    depthWrite: false,
                    side: THREE.DoubleSide,
                    fog: true
                  });
                  if (Array.isArray(obj.material)) obj.material[idx] = smoothCloudMat;
                  else obj.material = smoothCloudMat;
                  obj.receiveShadow = false;
                  obj.renderOrder = 2;
                } else {
                  // Cliff / Terrain material: preserve original baked lighting/emissive, soften normal bumps for smooth rock
                  m.flatShading = false;
                  if (m.normalScale && typeof m.normalScale.set === 'function') {
                    m.normalScale.set(0.32, 0.32);
                  }
                  if (typeof m.roughness === 'number') m.roughness = 0.90;
                  if (typeof m.metalness === 'number') m.metalness = 0.0;
                  m.needsUpdate = true;
                }
              });
            }
          }
        });

        // Hide procedural world when custom GLB map is loaded (keep Helipad + Sky)
        if (this.proceduralWorldGroup) this.proceduralWorldGroup.visible = false;
        this.customMapRoot = mapRoot;
        this.solidTerrainMeshes = [];
        mapRoot.traverse((obj) => {
          if (!obj.isMesh) return;
          const mats = obj.material ? (Array.isArray(obj.material) ? obj.material : [obj.material]) : [];
          const isCloud = mats.some((m) => {
            const mn = (m && m.name ? m.name : '').toLowerCase();
            return mn.includes('cloud') || mn.includes('mist') || mn.includes('fog') || Boolean(m && m.transparent);
          });
          if (!isCloud) {
            this.solidTerrainMeshes.push(obj);
          }
        });
        this.scene.add(mapRoot);
        mapRoot.updateMatrixWorld(true);

        // Seat Helipad & Drone cleanly on top of custom map center
        const centerY = this.getTerrainHeight(0, 0);
        if (this.helipad) this.helipad.position.y = centerY + 0.06;
        this.resetDronePosition();
      },
      undefined,
      () => {
        // assets/map.glb not provided yet — keep built-in procedural world silently
      }
    );
  }

  buildSkyDome() {
    // High-resolution atmospheric sky dome with realistic soft sunlit clouds (no blocky low-poly blobs!)
    const skyCanvas = document.createElement('canvas');
    skyCanvas.width = 1024;
    skyCanvas.height = 512;
    const ctx = skyCanvas.getContext('2d');

    // Rich alpine sky gradient (Deep Cobalt Zenith -> Clear Azure -> Warm Golden-Mist Horizon)
    const grad = ctx.createLinearGradient(0, 0, 0, 512);
    grad.addColorStop(0.0, '#0f2d6e');
    grad.addColorStop(0.24, '#2563eb');
    grad.addColorStop(0.44, '#60a5fa');
    grad.addColorStop(0.54, '#fed7aa'); // warm golden-hour horizon glow
    grad.addColorStop(0.62, '#cbd5e1');
    grad.addColorStop(1.0, '#1e293b');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 1024, 512);

    // Paint soft wispy high-altitude clouds across upper hemisphere
    for (let i = 0; i < 55; i++) {
      const cx = (i * 197 + 40) % 1024;
      const cy = 110 + ((i * 53) % 135);
      const rx = 55 + ((i * 31) % 95);
      const ry = 10 + ((i * 17) % 18);
      const cGrad = ctx.createRadialGradient(cx, cy, 2, cx, cy, rx);
      cGrad.addColorStop(0, 'rgba(255, 250, 240, 0.34)');
      cGrad.addColorStop(0.55, 'rgba(255, 245, 230, 0.14)');
      cGrad.addColorStop(1, 'rgba(255, 255, 255, 0)');
      ctx.save();
      ctx.translate(cx, cy);
      ctx.scale(1, ry / rx);
      ctx.beginPath();
      ctx.arc(0, 0, rx, 0, Math.PI * 2);
      ctx.fillStyle = cGrad;
      ctx.fill();
      ctx.restore();
    }

    // Soft glowing golden sun halo on the horizon
    const sunGrad = ctx.createRadialGradient(780, 228, 6, 780, 228, 120);
    sunGrad.addColorStop(0, 'rgba(255, 251, 235, 0.95)');
    sunGrad.addColorStop(0.25, 'rgba(253, 186, 116, 0.45)');
    sunGrad.addColorStop(1, 'rgba(253, 186, 116, 0)');
    ctx.fillStyle = sunGrad;
    ctx.fillRect(640, 90, 280, 280);

    const skyTex = new THREE.CanvasTexture(skyCanvas);
    if (THREE.sRGBEncoding) skyTex.encoding = THREE.sRGBEncoding;
    skyTex.mapping = THREE.EquirectangularReflectionMapping;
    const skyDome = new THREE.Mesh(
      new THREE.SphereGeometry(1450, 32, 20),
      new THREE.MeshBasicMaterial({ map: skyTex, side: THREE.BackSide, fog: false, depthWrite: false })
    );
    this.scene.add(skyDome);

    // Generate real IBL environment reflections from our sky dome so glass, water & drone metal gleam!
    if (typeof THREE.PMREMGenerator !== 'undefined' && this.renderer) {
      try {
        const pmrem = new THREE.PMREMGenerator(this.renderer);
        pmrem.compileEquirectangularShader();
        const envRT = pmrem.fromEquirectangular(skyTex);
        this.scene.environment = envRT.texture;
        pmrem.dispose();
      } catch (e) {
        // Fallback gracefully if PMREM unavailable
      }
    }
    this.clouds = [];
  }

  createGrassTexture() {
    const c = document.createElement('canvas');
    c.width = 512;
    c.height = 512;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#24471d';
    ctx.fillRect(0, 0, 512, 512);

    // Soft natural patches of moss, clover & warm meadow earth so ground never looks flat
    for (let i = 0; i < 90; i++) {
      const x = Math.random() * 512;
      const y = Math.random() * 512;
      const rad = 18 + Math.random() * 44;
      const g = ctx.createRadialGradient(x, y, 2, x, y, rad);
      g.addColorStop(0, Math.random() > 0.45 ? 'rgba(54, 98, 38, 0.24)' : 'rgba(26, 52, 22, 0.26)');
      g.addColorStop(1, 'rgba(36, 71, 29, 0)');
      ctx.fillStyle = g;
      ctx.beginPath();
      ctx.arc(x, y, rad, 0, Math.PI * 2);
      ctx.fill();
    }
    // Fine grass blades for crisp altitude & speed perception
    for (let i = 0; i < 6500; i++) {
      const x = Math.random() * 512;
      const y = Math.random() * 512;
      const w = 1 + Math.random() * 1.8;
      const h = 2 + Math.random() * 4;
      ctx.fillStyle = Math.random() > 0.5 ? 'rgba(72, 122, 46, 0.32)' : 'rgba(18, 42, 16, 0.34)';
      ctx.fillRect(x, y, w, h);
    }
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(72, 72);
    if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
    if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
    return tex;
  }

  createAsphaltTexture() {
    const c = document.createElement('canvas');
    c.width = 256;
    c.height = 256;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#1e222a';
    ctx.fillRect(0, 0, 256, 256);
    for (let i = 0; i < 3800; i++) {
      const v = 22 + Math.floor(Math.random() * 34);
      ctx.fillStyle = `rgba(${v}, ${v + 3}, ${v + 8}, 0.48)`;
      ctx.fillRect(Math.random() * 256, Math.random() * 256, 2, 2);
    }
    // Subtle dark rubber tire wear tracks along runway center
    ctx.fillStyle = 'rgba(10, 12, 16, 0.22)';
    ctx.fillRect(88, 0, 26, 256);
    ctx.fillRect(142, 0, 26, 256);

    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(4, 16);
    if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
    if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
    return tex;
  }

  createWaterTexture() {
    const c = document.createElement('canvas');
    c.width = 512;
    c.height = 512;
    const ctx = c.getContext('2d');
    // Rich crystal alpine water base
    const grad = ctx.createLinearGradient(0, 0, 512, 512);
    grad.addColorStop(0, '#0a6c9c');
    grad.addColorStop(0.5, '#0e85b8');
    grad.addColorStop(1, '#085b8a');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 512, 512);

    // Organic sinusoidal water wave crests & sunlight caustic shimmer (tiles seamlessly across 512x512!)
    for (let i = 0; i < 42; i++) {
      const y0 = (i / 42) * 512;
      ctx.beginPath();
      for (let x = 0; x <= 512; x += 8) {
        const wave = Math.sin((x / 512) * Math.PI * 4 + i * 0.9) * 6.5 + Math.cos((x / 512) * Math.PI * 8 - i * 1.4) * 2.5;
        if (x === 0) ctx.moveTo(x, y0 + wave);
        else ctx.lineTo(x, y0 + wave);
      }
      ctx.strokeStyle = i % 2 === 0 ? 'rgba(125, 211, 252, 0.22)' : 'rgba(3, 55, 92, 0.24)';
      ctx.lineWidth = i % 3 === 0 ? 3.2 : 1.8;
      ctx.stroke();
    }

    // Soft glowing caustic cells & sun-glint speckles
    for (let k = 0; k < 220; k++) {
      const cx = (k * 97 + (k % 7) * 31) % 512;
      const cy = (k * 61 + (k % 11) * 19) % 512;
      const rx = 8 + (k % 18);
      const ry = 3 + (k % 7);
      ctx.fillStyle = (k % 4 === 0) ? 'rgba(224, 242, 254, 0.22)' : 'rgba(56, 189, 248, 0.14)';
      ctx.beginPath();
      ctx.ellipse(cx, cy, rx, ry, (k % 5) * 0.18, 0, Math.PI * 2);
      ctx.fill();
    }

    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(5, 5);
    if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
    if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
    return tex;
  }

  createWaterBumpTexture() {
    const c = document.createElement('canvas');
    c.width = 256;
    c.height = 256;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#808080';
    ctx.fillRect(0, 0, 256, 256);
    for (let i = 0; i < 36; i++) {
      const y0 = (i / 36) * 256;
      ctx.beginPath();
      for (let x = 0; x <= 256; x += 6) {
        const w = Math.sin((x / 256) * Math.PI * 4 + i * 1.1) * 4.5 + Math.cos((x / 256) * Math.PI * 6 + i * 0.5) * 2;
        if (x === 0) ctx.moveTo(x, y0 + w);
        else ctx.lineTo(x, y0 + w);
      }
      ctx.strokeStyle = i % 2 === 0 ? 'rgba(245, 245, 245, 0.45)' : 'rgba(35, 35, 35, 0.45)';
      ctx.lineWidth = 3.5;
      ctx.stroke();
    }
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(7, 7);
    return tex;
  }

  // Shared height function so terrain mesh, objects, drone physics, and camera always agree
  // Only collides with SOLID terrain meshes (clouds are fly-through!)
  getTerrainHeight(x, z) {
    if (this.customMapRoot && this.groundRaycaster) {
      const targets = (this.solidTerrainMeshes && this.solidTerrainMeshes.length > 0)
        ? this.solidTerrainMeshes
        : [this.customMapRoot];
      this.groundRaycaster.set(new THREE.Vector3(x, 600, z), new THREE.Vector3(0, -1, 0));
      const hits = this.groundRaycaster.intersectObjects(targets, false);
      if (hits.length > 0) return hits[0].point.y;
      return 0;
    }
    const dist = Math.sqrt(x * x + z * z);
    // Expanded flat flying valley basin (dist <= 240m) with smooth gentle outer hills
    if (dist <= 240) return 0;
    const t = Math.min(1, (dist - 240) / 90);
    const blend = t * t * (3 - 2 * t); // smoothstep
    const rawHill = Math.sin(x * 0.024) * Math.cos(z * 0.024) * 5.5 + Math.sin(x * 0.014 + z * 0.012) * 3.0 + 3.5;
    return Math.max(0, rawHill * blend);
  }

  buildWorld() {
    // Route all procedural terrain/props into proceduralWorldGroup so a custom assets/map.glb cleanly replaces them
    const mainScene = this.scene;
    this.scene = this.proceduralWorldGroup;

    // 1. Expanded 1200x1200m Terrain Ground (64x64 = 3.5x fewer vertices, identical smooth terrain!)
    const groundGeo = new THREE.PlaneGeometry(1200, 1200, 64, 64);
    const pos = groundGeo.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      const vx = pos.getX(i);
      const vy = pos.getY(i);
      // PlaneGeometry rotated -PI/2 around X maps local (vx, vy) -> world (vx, -vy)
      const h = this.getTerrainHeight(vx, -vy);
      pos.setZ(i, h);
    }
    groundGeo.computeVertexNormals();

    const groundMat = new THREE.MeshStandardMaterial({
      map: this.createGrassTexture(),
      roughness: 0.92,
      metalness: 0.02
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = 0;
    ground.receiveShadow = true;
    this.scene.add(ground);

    // 2. Snow Mountains in Distance
    this.buildMountains();

    // 3. Airfield & Runway
    this.buildAirfield();

    // 5. Trees & Foliage
    this.buildTrees();

    // 6. Farm Fields & Crops
    this.buildAgriculturalFields();

    // 7. Village Houses
    this.buildVillage();

    // 8. Lake Water
    this.buildLake();

    // 9. Tall Telecom Tower (45m)
    this.buildTallTower();

    // 10. Aviation Geofence Perimeter & Southern Ridge Wind Turbines
    this.buildMapBoundary();

    // 11. Explorable Valley Landmarks (Ancient Ruins, Forest Campsite/Watchtower, Solar Farm, Lakeside Lighthouse)
    this.buildMapLandmarks();

    // 12. 4-Tier Interconnected Valley Road, Cobblestone Lane, Gravel Track & Scenic Footpath Network + Power Lines
    this.buildRoadNetwork();

    // 13. Purposeful Living Valley Settlements (Village Café, General Store, Town Square, Farmhouse, Greenhouse, Grain Mill, Boathouse, Grid Control House & Transit Shelter)
    this.buildLivingValleySettlements();

    // 14. Lossless Spatially-Chunked Static Geometry Batcher (collapses ~3,500+ static draw calls into ~160 frustum-cullable chunks for silky 60 FPS!)
    this.optimizeStaticProceduralWorld();

    // Restore main scene for Helipad, Drone, Checkpoints & Custom GLB Map
    this.scene = mainScene;
    this.buildHelipad();
  }

  optimizeStaticProceduralWorld() {
    if (!this.proceduralWorldGroup) return;

    // 1. Mark all animated/referenced subtrees so they are NEVER merged or frozen
    const animatedSet = new Set();
    const markSubtree = (root) => {
      if (!root) return;
      root.traverse((o) => animatedSet.add(o));
    };
    markSubtree(this.radarDish);
    markSubtree(this.towerLight);
    markSubtree(this.windsock);
    markSubtree(this.lighthouseBeam);
    markSubtree(this.windmillSail);
    if (this.windTurbineRotors) this.windTurbineRotors.forEach(markSubtree);
    if (this.lakeBoats) this.lakeBoats.forEach((b) => markSubtree(b.mesh));
    if (this.farmSpinners) this.farmSpinners.forEach((s) => markSubtree(s.mesh));
    if (this.clouds) this.clouds.forEach(markSubtree);

    const animatedTextures = new Set([
      this.waterTex, this.waterTex2, this.waterBumpTex, this.waterfallTex
    ].filter(Boolean));

    this.proceduralWorldGroup.updateMatrixWorld(true);

    // 2. Auto-convert matte non-metallic static PBR materials into fast MeshLambertMaterial (3x faster GPU fragment shader on grass/wood/stone/foliage/crops while keeping full PBR+IBL on water, glass, solar panels & the 3D drone!)
    const lambertCache = new Map();
    const toFastMaterial = (mat) => {
      if (!mat || !mat.isMeshStandardMaterial) return mat;
      if (mat.bumpMap || mat.normalMap || mat.roughnessMap || mat.metalnessMap || mat.envMap) return mat;
      if ((mat.metalness || 0) > 0.20 || (mat.roughness !== undefined && mat.roughness < 0.58)) return mat;
      if (mat.map && animatedTextures.has(mat.map)) return mat;
      const colHex = mat.color ? mat.color.getHex() : 0xffffff;
      const emiHex = mat.emissive ? mat.emissive.getHex() : 0x000000;
      const lKey = [
        mat.map ? mat.map.uuid : 'N',
        colHex, emiHex,
        Math.round((mat.emissiveIntensity ?? 1) * 100),
        Math.round((mat.opacity ?? 1) * 100),
        mat.transparent ? 1 : 0,
        mat.side,
        mat.flatShading ? 1 : 0,
        mat.vertexColors ? 1 : 0,
        mat.depthWrite === false ? 0 : 1,
        mat.polygonOffset ? 1 : 0,
        mat.polygonOffsetFactor || 0
      ].join('|');
      let cached = lambertCache.get(lKey);
      if (!cached) {
        cached = new THREE.MeshLambertMaterial({
          color: colHex,
          map: mat.map || null,
          emissive: emiHex,
          emissiveIntensity: mat.emissiveIntensity !== undefined ? mat.emissiveIntensity : 1,
          transparent: Boolean(mat.transparent),
          opacity: mat.opacity !== undefined ? mat.opacity : 1,
          side: mat.side,
          flatShading: Boolean(mat.flatShading),
          vertexColors: Boolean(mat.vertexColors),
          depthWrite: mat.depthWrite !== false,
          polygonOffset: Boolean(mat.polygonOffset),
          polygonOffsetFactor: mat.polygonOffsetFactor || 0,
          polygonOffsetUnits: mat.polygonOffsetUnits || 0
        });
        lambertCache.set(lKey, cached);
      }
      return cached;
    };

    // Upgrade matte materials across static & instanced world meshes (trees, crops, terrain, houses, roads)
    this.proceduralWorldGroup.traverse((obj) => {
      if (!obj.isMesh || animatedSet.has(obj)) return;
      if (obj.material && !Array.isArray(obj.material)) {
        obj.material = toFastMaterial(obj.material);
      }
    });

    // Canonical material key (deduplicates BOTH untextured AND shared-texture materials for maximum draw-call merging!)
    const getMaterialKey = (mat) => {
      const col = mat.color ? mat.color.getHexString() : 'none';
      const emi = mat.emissive ? mat.emissive.getHexString() : '000000';
      const ei = mat.emissiveIntensity !== undefined ? Math.round(mat.emissiveIntensity * 100) : 0;
      const r = mat.roughness !== undefined ? Math.round(mat.roughness * 50) : -1;
      const m = mat.metalness !== undefined ? Math.round(mat.metalness * 50) : -1;
      const op = mat.opacity !== undefined ? Math.round(mat.opacity * 100) : 100;
      const mapId = mat.map ? mat.map.uuid : '0';
      const extraTex = (mat.bumpMap ? mat.bumpMap.uuid : '') + (mat.normalMap ? mat.normalMap.uuid : '') + (mat.alphaMap ? mat.alphaMap.uuid : '') + (mat.emissiveMap ? mat.emissiveMap.uuid : '');
      if (extraTex) return 'X:' + mat.uuid;
      return [
        'M', mat.type, mapId, col, emi, ei, r, m, op,
        mat.transparent ? 1 : 0,
        mat.side,
        mat.flatShading ? 1 : 0,
        mat.depthWrite === false ? 0 : 1,
        mat.polygonOffset ? 1 : 0,
        mat.polygonOffsetFactor || 0,
        mat.polygonOffsetUnits || 0
      ].join('|');
    };

    // 3. Bucket static meshes by (180m spatial cell × material × shadow flags) for optimal frustum culling + tiny draw call count!
    const CELL_SIZE = 180;
    const buckets = new Map();
    const tmpCenter = new THREE.Vector3();
    const tmpScale = new THREE.Vector3();

    this.proceduralWorldGroup.traverse((obj) => {
      if (!obj.isMesh || obj.isInstancedMesh) return;
      if (animatedSet.has(obj)) return;
      const mat = obj.material;
      if (!mat || Array.isArray(mat)) return;
      if (mat.vertexColors) return; // Preserve custom vertex-colored terrain/mountains as-is
      if (mat.map && animatedTextures.has(mat.map)) return;
      if (mat.bumpMap && animatedTextures.has(mat.bumpMap)) return;
      const geo = obj.geometry;
      if (!geo || !geo.attributes || !geo.attributes.position) return;
      if (obj.matrixWorld.determinant() < 0) return; // Never flip winding

      if (!geo.boundingSphere) geo.computeBoundingSphere();
      const bs = geo.boundingSphere;
      if (!bs) return;
      tmpCenter.copy(bs.center).applyMatrix4(obj.matrixWorld);
      tmpScale.setFromMatrixScale(obj.matrixWorld);
      const maxS = Math.max(Math.abs(tmpScale.x), Math.abs(tmpScale.y), Math.abs(tmpScale.z));
      const worldRadius = bs.radius * maxS;

      // Skip enormous single terrain/sky/mountain meshes (> 55m radius) so they remain independent
      if (worldRadius > 55) return;

      // Smart Micro-Shadow Culling: tiny decorative sub-0.45m details skip shadow rasterization (huge GPU speedup, zero visible loss!)
      if (obj.castShadow && worldRadius < 0.45) {
        obj.castShadow = false;
      }

      const cx = Math.floor(tmpCenter.x / CELL_SIZE);
      const cz = Math.floor(tmpCenter.z / CELL_SIZE);
      const shadowBits = (obj.castShadow ? 1 : 0) | (obj.receiveShadow ? 2 : 0);
      const matKey = getMaterialKey(mat);
      const key = `${cx},${cz}|${shadowBits}|${matKey}`;

      let bucket = buckets.get(key);
      if (!bucket) {
        bucket = {
          material: mat,
          castShadow: obj.castShadow,
          receiveShadow: obj.receiveShadow,
          needsUV: Boolean(mat.map || mat.bumpMap || mat.normalMap || mat.alphaMap || mat.emissiveMap),
          meshes: []
        };
        buckets.set(key, bucket);
      }
      bucket.meshes.push(obj);
    });

    // 4. Merge each multi-mesh bucket into one single tightly-packed BufferGeometry!
    const mergedGroup = new THREE.Group();
    mergedGroup.name = 'OptimizedStaticChunks';
    mergedGroup.matrixAutoUpdate = false;
    mergedGroup.updateMatrix();

    buckets.forEach((b) => {
      if (b.meshes.length < 2) return;

      const baked = [];
      const includedMeshes = [];
      let totalVerts = 0;
      try {
        for (let i = 0; i < b.meshes.length; i++) {
          const mesh = b.meshes[i];
          const src = mesh.geometry;
          const g = src.index ? src.toNonIndexed() : src.clone();
          if (!g.attributes.normal) g.computeVertexNormals();
          g.applyMatrix4(mesh.matrixWorld);
          const vCount = g.attributes.position.count;
          if (vCount === 0 || !g.attributes.normal || g.attributes.normal.count !== vCount) {
            g.dispose();
            continue;
          }
          baked.push(g);
          includedMeshes.push(mesh);
          totalVerts += vCount;
        }
        if (baked.length < 2 || totalVerts === 0) {
          baked.forEach((g) => g.dispose());
          return;
        }

        const positions = new Float32Array(totalVerts * 3);
        const normals = new Float32Array(totalVerts * 3);
        const uvs = b.needsUV ? new Float32Array(totalVerts * 2) : null;

        let vOff = 0;
        for (let i = 0; i < baked.length; i++) {
          const g = baked[i];
          const pArr = g.attributes.position.array;
          const nArr = g.attributes.normal.array;
          const vCount = g.attributes.position.count;
          positions.set(pArr, vOff * 3);
          normals.set(nArr, vOff * 3);
          if (uvs) {
            const uvAttr = g.attributes.uv;
            if (uvAttr && uvAttr.itemSize === 2 && uvAttr.array.length === vCount * 2) {
              uvs.set(uvAttr.array, vOff * 2);
            }
          }
          vOff += vCount;
          g.dispose();
        }

        const mergedGeo = new THREE.BufferGeometry();
        mergedGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        mergedGeo.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
        if (uvs) mergedGeo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
        mergedGeo.computeBoundingBox();
        mergedGeo.computeBoundingSphere();

        const mergedMesh = new THREE.Mesh(mergedGeo, b.material);
        mergedMesh.castShadow = b.castShadow;
        mergedMesh.receiveShadow = b.receiveShadow;
        mergedMesh.matrixAutoUpdate = false;
        mergedMesh.updateMatrix();
        mergedGroup.add(mergedMesh);

        // Remove original unmerged static child meshes from their parent groups
        for (let i = 0; i < includedMeshes.length; i++) {
          const m = includedMeshes[i];
          if (m.parent) m.parent.remove(m);
        }
      } catch (err) {
        baked.forEach((g) => g.dispose());
      }
    });

    this.proceduralWorldGroup.add(mergedGroup);

    // 5. Prune empty leftover static Group wrappers & freeze matrices on all remaining static meshes/InstancedMeshes!
    const pruneEmpty = (node) => {
      for (let i = node.children.length - 1; i >= 0; i--) {
        const ch = node.children[i];
        if (animatedSet.has(ch) || ch === mergedGroup) continue;
        if (ch.isGroup && ch.children.length > 0) pruneEmpty(ch);
        if (ch.isGroup && ch.children.length === 0) {
          node.remove(ch);
        }
      }
    };
    pruneEmpty(this.proceduralWorldGroup);

    this.proceduralWorldGroup.traverse((obj) => {
      if (animatedSet.has(obj)) return;
      if (obj.isInstancedMesh) {
        obj.frustumCulled = false; // Prevent r128 origin-based culling of spread-out instanced runway lights, trees & crops
        obj.updateMatrix();
        obj.matrixAutoUpdate = false;
      } else if (obj.isMesh || obj.isLine) {
        obj.updateMatrix();
        obj.matrixAutoUpdate = false;
      }
    });
  }

  createMountainRockTexture() {
    const c = document.createElement('canvas');
    c.width = 512;
    c.height = 512;
    const ctx = c.getContext('2d');
    ctx.fillStyle = '#dcdcdc';
    ctx.fillRect(0, 0, 512, 512);

    // Layered horizontal geological cliff strata
    for (let y = 0; y < 512; y += 8) {
      const shade = 192 + Math.floor(Math.sin(y * 0.09) * 24 + Math.cos(y * 0.23) * 14);
      ctx.fillStyle = `rgba(${shade},${shade},${shade},0.45)`;
      ctx.fillRect(0, y, 512, 6);
    }
    // Vertical erosion gullies & rock fractures
    for (let i = 0; i < 420; i++) {
      const x = (i * 73) % 512;
      const y = (i * 137) % 512;
      const w = 2 + (i % 5);
      const h = 18 + (i % 46);
      ctx.fillStyle = i % 2 === 0 ? 'rgba(85, 92, 102, 0.22)' : 'rgba(255, 255, 255, 0.20)';
      ctx.fillRect(x, y, w, h);
    }
    // Fine rocky micro-grain
    for (let i = 0; i < 3600; i++) {
      const x = (i * 97 + 13) % 512;
      const y = (i * 53 + 29) % 512;
      const v = i % 3 === 0 ? 110 : 235;
      ctx.fillStyle = `rgba(${v},${v},${v},0.16)`;
      ctx.fillRect(x, y, 2, 2);
    }

    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(5, 3);
    if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
    if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
    return tex;
  }

  buildMountains() {
    // 14 High-Detail Ridged-Fractal Alpine Massifs encircling 360° (with slope-aware snow,
    // procedural rock strata textures, craggy buttresses, slope pines & alpine waterfall!)
    const mPositions = [
      // North Alpine Range
      { x: -305, z: -345, r: 108, h: 168, seed: 1.1, style: 'jagged_spire', sx: 1.15, sz: 0.92, tiltX: 0.16, tiltZ: -0.07 },
      { x: -120, z: -435, r: 138, h: 215, seed: 2.3, style: 'twin_peak',    sx: 1.28, sz: 0.96, tiltX: -0.11, tiltZ: 0.08 },
      { x: 110,  z: -425, r: 130, h: 198, seed: 3.7, style: 'broad_alpine', sx: 1.14, sz: 1.15, tiltX: 0.10, tiltZ: 0.11 },
      { x: 300,  z: -335, r: 108, h: 162, seed: 4.5, style: 'mesa_cliff',   sx: 1.06, sz: 1.16, tiltX: -0.06, tiltZ: -0.08 },
      // East Ridge
      { x: 430,  z: -145, r: 114, h: 156, seed: 7.1, style: 'twin_peak',    sx: 0.94, sz: 1.24, tiltX: -0.13, tiltZ: 0.05 },
      { x: 445,  z: 85,   r: 120, h: 148, seed: 8.3, style: 'mesa_cliff',   sx: 0.96, sz: 1.22, tiltX: 0.05, tiltZ: -0.09 },
      { x: 345,  z: 300,  r: 114, h: 158, seed: 9.2, style: 'jagged_spire', sx: 1.14, sz: 1.04, tiltX: -0.09, tiltZ: -0.11 },
      // South Range (well behind 232m Wind Turbines & 295m Geofence)
      { x: 155,  z: 420,  r: 122, h: 165, seed: 10.4, style: 'broad_alpine', sx: 1.24, sz: 0.94, tiltX: 0.12, tiltZ: -0.06 },
      { x: -20,  z: 448,  r: 135, h: 182, seed: 11.8, style: 'twin_peak',    sx: 1.30, sz: 0.98, tiltX: -0.10, tiltZ: -0.07 },
      { x: -195, z: 405,  r: 118, h: 158, seed: 12.6, style: 'mesa_cliff',   sx: 1.16, sz: 1.04, tiltX: 0.07, tiltZ: 0.08 },
      // West Ridge
      { x: -365, z: 265,  r: 116, h: 162, seed: 6.4,  style: 'jagged_spire', sx: 1.02, sz: 1.20, tiltX: 0.14, tiltZ: -0.05 },
      { x: -435, z: 55,   r: 118, h: 148, seed: 5.2,  style: 'broad_alpine', sx: 0.94, sz: 1.26, tiltX: 0.09, tiltZ: 0.10 },
      { x: -420, z: -165, r: 112, h: 160, seed: 13.5, style: 'twin_peak',    sx: 0.98, sz: 1.16, tiltX: 0.11, tiltZ: -0.08 },
      // Distant High Background Peak behind South-West
      { x: -425, z: 415,  r: 140, h: 208, seed: 14.2, style: 'jagged_spire', sx: 1.14, sz: 1.14, tiltX: -0.07, tiltZ: 0.07 }
    ];

    const rockTex = this.createMountainRockTexture();
    const mountainMat = new THREE.MeshStandardMaterial({
      map: rockTex,
      vertexColors: true,
      roughness: 0.92,
      metalness: 0.04,
      flatShading: false
    });

    const cForestDeep = new THREE.Color(0x1b3c1f);
    const cMeadowSlope = new THREE.Color(0x2f5329);
    const cEarth = new THREE.Color(0x5a4c3e);
    const cRockDark = new THREE.Color(0x364152);
    const cRockMid = new THREE.Color(0x526072);
    const cSnowShadow = new THREE.Color(0xdbe7f5);
    const cSnowBright = new THREE.Color(0xf8fafc);

    // 4-octave ridged multifractal function for authentic craggy alpine arêtes & gullies
    const ridgedNoise = (angle, hNorm, seed) => {
      const r1 = 1.0 - Math.abs(Math.sin(angle * 3.0 + seed + hNorm * 1.4));
      const r2 = 1.0 - Math.abs(Math.cos(angle * 6.0 - seed * 1.7 + hNorm * 3.2));
      const r3 = 1.0 - Math.abs(Math.sin(angle * 11.0 + seed * 0.9 - hNorm * 5.8));
      const r4 = Math.sin(angle * 17.0 + hNorm * 11.0 + seed * 2.3) * 0.5 + 0.5;
      return r1 * 0.45 + r2 * 0.28 + r3 * 0.17 + r4 * 0.10;
    };

    const buildSinglePeak = (x, z, r, h, seed, style, sx = 1, sz = 1, tiltX = 0, tiltZ = 0, addCollider = true) => {
      // High-resolution tapered ridge mesh (44 radial x 24 height segments — no pointy tent artifact!)
      const topR = style === 'mesa_cliff' ? r * 0.34 : (style === 'broad_alpine' ? r * 0.12 : r * 0.035);
      const geo = new THREE.CylinderGeometry(topR, r, h, 44, 24, false);
      const pos = geo.attributes.position;

      // Pass 1: Sculpt 3D vertex coordinates with ridged multifractal geology
      for (let i = 0; i < pos.count; i++) {
        let vx = pos.getX(i);
        let vy = pos.getY(i);
        let vz = pos.getZ(i);

        const hNorm = Math.max(0, Math.min(1, (vy + h / 2) / h));
        const angle = Math.atan2(vz, vx);
        const ridge = ridgedNoise(angle, hNorm, seed); // 0..1 sharp alpine ridges

        // Natural concave/convex profile by mountain style
        let profile = 1.0;
        if (style === 'jagged_spire') {
          profile = 1.0 - Math.pow(hNorm, 0.85) * 0.22;
        } else if (style === 'broad_alpine') {
          profile = 1.0 + Math.sin(hNorm * Math.PI) * 0.18;
        } else if (style === 'mesa_cliff') {
          profile = hNorm > 0.78 ? 1.18 : 1.0 + Math.sin(hNorm * Math.PI) * 0.12;
        }

        // Carve prominent sharp rock buttresses & recessed snow couloirs
        const carve = 0.72 + ridge * 0.58;
        const envelope = Math.sin(hNorm * Math.PI * 0.92); // strong mid-mountain, smooth into ground
        const radialMul = profile * (1.0 + (carve - 1.0) * (0.28 + 0.72 * envelope));

        vx = vx * radialMul * sx + hNorm * hNorm * r * tiltX;
        vz = vz * radialMul * sz + hNorm * hNorm * r * tiltZ;

        // Craggy summit silhouette (undulate top crest so even peaks & mesas look rocky!)
        if (hNorm > 0.75) {
          const crestBoost = (hNorm - 0.75) * 4.0;
          vy += (ridge - 0.5) * h * 0.16 * crestBoost;
        }

        pos.setXYZ(i, vx, vy, vz);
      }

      // Compute smooth sculpted vertex normals BEFORE slope-aware coloring!
      geo.computeVertexNormals();
      const norm = geo.attributes.normal;
      const colors = [];

      // Pass 2: Slope-aware & elevation-aware realistic Alpine coloring
      // (Snow accumulates on ridges/gullies & high elevations, while sheer steep cliffs expose dark granite!)
      for (let i = 0; i < pos.count; i++) {
        const vx = pos.getX(i);
        const vy = pos.getY(i);
        const vz = pos.getZ(i);
        const ny = norm.getY(i); // 1 = flat, 0 = vertical sheer cliff
        const hNorm = Math.max(0, Math.min(1, (vy + h / 2) / h));
        const angle = Math.atan2(vz, vx);
        const ridge = ridgedNoise(angle, hNorm, seed);

        const snowLine = (style === 'mesa_cliff' ? 0.64 : 0.54) + (ridge - 0.5) * 0.18;
        const rockLine = 0.24 + (ridge - 0.5) * 0.12;
        const earthLine = 0.15 + Math.sin(angle * 4 + seed) * 0.04;

        let col = new THREE.Color();
        if (hNorm >= snowLine) {
          // Steep cliff faces above snowline still show dark exposed granite streaks!
          if (ny < 0.32 && hNorm < 0.88) {
            col.copy(cRockDark).lerp(cSnowShadow, 0.25);
          } else {
            col.copy(ny > 0.55 ? cSnowBright : cSnowShadow);
          }
        } else if (hNorm >= rockLine) {
          // Layered dark/mid granite rock with occasional snow pockets tucked in flatter gullies
          const strata = Math.sin(hNorm * 34 + ridge * 4 + seed);
          col.copy(strata > 0.1 ? cRockMid : cRockDark);
          if (hNorm > snowLine - 0.14 && ny > 0.52) {
            col.lerp(cSnowShadow, 0.65);
          }
        } else if (hNorm >= earthLine) {
          col.copy(cEarth).lerp(cRockDark, ny < 0.45 ? 0.5 : 0.15);
        } else {
          col.copy(ridge > 0.52 ? cForestDeep : cMeadowSlope);
        }
        colors.push(col.r, col.g, col.b);
      }

      geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

      const mesh = new THREE.Mesh(geo, mountainMat);
      mesh.position.set(x, h / 2 - 5, z);
      mesh.receiveShadow = true;
      this.scene.add(mesh);
      if (addCollider) {
        this.addConeCollider(x, z, r * Math.max(sx, sz) * 0.86, h - 5);
      }
    };

    mPositions.forEach(m => {
      buildSinglePeak(m.x, m.z, m.r, m.h, m.seed, m.style, m.sx, m.sz, m.tiltX, m.tiltZ, true);

      // Every mountain gets 1–2 organic rocky shoulder sub-peaks / cliff spurs so NO peak looks like a lone cone!
      const ang1 = m.seed * 1.7;
      const subX1 = m.x + Math.cos(ang1) * (m.r * 0.52);
      const subZ1 = m.z + Math.sin(ang1) * (m.r * 0.52);
      buildSinglePeak(subX1, subZ1, m.r * 0.62, m.h * 0.72, m.seed + 5.1, 'jagged_spire', 1.05, 0.95, -m.tiltX, m.tiltZ, false);

      const ang2 = ang1 + 2.4;
      const subX2 = m.x + Math.cos(ang2) * (m.r * 0.58);
      const subZ2 = m.z + Math.sin(ang2) * (m.r * 0.58);
      buildSinglePeak(subX2, subZ2, m.r * 0.48, m.h * 0.48, m.seed + 8.7, 'broad_alpine', 1.15, 0.9, 0.06, -0.05, false);
    });

    // Alpine Waterfall cascading down the face of the North-East mountain peak (108, -322)
    const wfCanvas = document.createElement('canvas');
    wfCanvas.width = 64;
    wfCanvas.height = 256;
    const wCtx = wfCanvas.getContext('2d');
    const wGrad = wCtx.createLinearGradient(0, 0, 64, 0);
    wGrad.addColorStop(0, 'rgba(224,242,254,0.15)');
    wGrad.addColorStop(0.5, 'rgba(240,249,255,0.92)');
    wGrad.addColorStop(1, 'rgba(224,242,254,0.15)');
    wCtx.fillStyle = wGrad;
    wCtx.fillRect(0, 0, 64, 256);
    for (let i = 0; i < 90; i++) {
      wCtx.fillStyle = i % 2 === 0 ? 'rgba(56, 189, 248, 0.45)' : 'rgba(255, 255, 255, 0.75)';
      wCtx.fillRect((i * 19) % 58, (i * 41) % 240, 4, 26);
    }
    this.waterfallTex = new THREE.CanvasTexture(wfCanvas);
    this.waterfallTex.wrapS = THREE.RepeatWrapping;
    this.waterfallTex.wrapT = THREE.RepeatWrapping;
    this.waterfallTex.repeat.set(1, 3);
    const wfMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(14, 118),
      new THREE.MeshBasicMaterial({ map: this.waterfallTex, transparent: true, opacity: 0.85, side: THREE.DoubleSide })
    );
    wfMesh.position.set(106, 52, -338);
    wfMesh.rotation.x = -0.52;
    this.scene.add(wfMesh);

    // 140 Craggy Granite Foothill Boulders + 180 Alpine Slope Pine Trees climbing mountain bases!
    const boulderMat = new THREE.MeshStandardMaterial({ map: rockTex, color: 0x64748b, roughness: 0.92, flatShading: true });
    const boulderCount = 140;
    const boulderInst = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(1.0, 0), boulderMat, boulderCount);
    boulderInst.castShadow = true;
    boulderInst.receiveShadow = true;
    const dummy = new THREE.Object3D();
    for (let i = 0; i < boulderCount; i++) {
      const a = (i / boulderCount) * Math.PI * 2 + Math.sin(i * 3.1) * 0.14;
      const dist = 288 + ((i * 37) % 72);
      const bx = Math.cos(a) * dist;
      const bz = Math.sin(a) * dist;
      const by = this.getTerrainHeight(bx, bz);
      const s = 2.2 + ((i * 19) % 34) * 0.18;
      dummy.position.set(bx, by + s * 0.35, bz);
      dummy.rotation.set(i * 0.5, i * 1.2, i * 0.3);
      dummy.scale.set(s * 1.4, s * 0.85, s * 1.15);
      dummy.updateMatrix();
      boulderInst.setMatrixAt(i, dummy.matrix);
    }
    this.scene.add(boulderInst);

    // Alpine Slope Pines hugging the outer foothills (305..355m) for rich transition into mountain bases
    const slopePineCount = 180;
    const spMat = new THREE.MeshStandardMaterial({ color: 0x113820, roughness: 0.88, flatShading: true });
    const spTrunkMat = new THREE.MeshStandardMaterial({ color: 0x452712, roughness: 0.94 });
    const spTrunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.25, 0.45, 3.2, 5), spTrunkMat, slopePineCount);
    const spCones1 = new THREE.InstancedMesh(new THREE.ConeGeometry(2.8, 5.2, 6), spMat, slopePineCount);
    const spCones2 = new THREE.InstancedMesh(new THREE.ConeGeometry(1.9, 4.4, 6), spMat, slopePineCount);
    for (let i = 0; i < slopePineCount; i++) {
      const a = (i / slopePineCount) * Math.PI * 2 + Math.cos(i * 5.3) * 0.08;
      const dist = 304 + ((i * 29) % 52);
      const px = Math.cos(a) * dist;
      const pz = Math.sin(a) * dist;
      const py = this.getTerrainHeight(px, pz) + ((dist - 304) * 0.18);
      const s = 1.0 + ((i * 13) % 10) * 0.1;
      dummy.position.set(px, py + 1.6 * s, pz);
      dummy.rotation.set(0, i, 0);
      dummy.scale.setScalar(s);
      dummy.updateMatrix();
      spTrunks.setMatrixAt(i, dummy.matrix);
      dummy.position.set(px, py + 4.4 * s, pz);
      dummy.updateMatrix();
      spCones1.setMatrixAt(i, dummy.matrix);
      dummy.position.set(px, py + 7.2 * s, pz);
      dummy.updateMatrix();
      spCones2.setMatrixAt(i, dummy.matrix);
    }
    this.scene.add(spTrunks, spCones1, spCones2);
  }

  buildMapBoundary() {
    // 1. Expanded Visual Drone Aviation Geofence Ring (Radius = 295m, 32 Warning Pylons — 100% clear of mountains!)
    const boundaryR = 295;
    const pylonCount = 32;
    const poleMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.6, metalness: 0.3 });
    const stripeMat = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.5 });
    const beaconMat = new THREE.MeshBasicMaterial({ color: 0xff3b30 });

    const ringPts = [];
    for (let i = 0; i < pylonCount; i++) {
      const a = (i / pylonCount) * Math.PI * 2;
      const px = Math.cos(a) * boundaryR;
      const pz = Math.sin(a) * boundaryR;
      const py = this.getTerrainHeight(px, pz);

      const pylon = new THREE.Group();
      pylon.position.set(px, py, pz);

      const post = new THREE.Mesh(new THREE.CylinderGeometry(0.26, 0.36, 8.5, 8), poleMat);
      post.position.y = 4.25;
      post.castShadow = true;

      const band = new THREE.Mesh(new THREE.CylinderGeometry(0.30, 0.30, 1.5, 8), stripeMat);
      band.position.y = 6.1;

      const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.42, 8, 8), beaconMat);
      bulb.position.y = 8.7;

      pylon.add(post, band, bulb);
      this.scene.add(pylon);
      this.addCylCollider(px, pz, 0.42, py, py + 8.6, 'power_pole');

      ringPts.push(new THREE.Vector3(px, py + 5.5, pz));
    }

    // Connect pylons with a sleek glowing orange aviation geofence perimeter cable
    ringPts.push(ringPts[0].clone());
    const cableGeo = new THREE.BufferGeometry().setFromPoints(ringPts);
    const cableMat = new THREE.LineBasicMaterial({ color: 0xf97316, transparent: true, opacity: 0.72 });
    this.scene.add(new THREE.Line(cableGeo, cableMat));

    // 2. Three High-Detail 3.5 MW Modern Alpine Wind Turbines (Concrete Pad, Transformer, Flanged Mast, Aerodynamic Airfoil Blades & Nacelle!)
    this.windTurbineRotors = [];

    // Procedural bolted-flange tubular steel tower texture (realistic stacked turbine mast sections + eco-green base stripe!)
    const makeTurbineTowerTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 512;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#f1f5f9'; ctx.fillRect(0, 0, 128, 512);
      // Subtle vertical metallic sheen gradient
      const g = ctx.createLinearGradient(0, 0, 128, 0);
      g.addColorStop(0, 'rgba(148,163,184,0.16)');
      g.addColorStop(0.45, 'rgba(255,255,255,0.12)');
      g.addColorStop(1, 'rgba(100,116,139,0.2)');
      ctx.fillStyle = g; ctx.fillRect(0, 0, 128, 512);
      // 3 Horizontal bolted steel section flange seams along tower height
      [128, 256, 384].forEach((sy) => {
        ctx.fillStyle = 'rgba(71, 85, 105, 0.35)';
        ctx.fillRect(0, sy, 128, 4);
        ctx.fillStyle = 'rgba(255, 255, 255, 0.45)';
        ctx.fillRect(0, sy + 4, 128, 2);
      });
      // Eco-green gradient chevron base ring near tower bottom
      ctx.fillStyle = '#10b981'; ctx.fillRect(0, 456, 128, 12);
      ctx.fillStyle = '#059669'; ctx.fillRect(0, 474, 128, 16);
      ctx.fillStyle = '#047857'; ctx.fillRect(0, 496, 128, 16);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeTurbineBadgeTex = (code) => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 64;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#0f172a'; ctx.fillRect(0, 0, 128, 64);
      ctx.strokeStyle = '#10b981'; ctx.lineWidth = 4; ctx.strokeRect(3, 3, 122, 58);
      ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 28px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(code, 64, 34);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Sculpted Tapered Aerodynamic Airfoil Blade Geometry (wide chord root tapering cleanly to swept tip!)
    const makeAirfoilBladeGeo = (yStart, yEnd, wStart, wEnd, depth = 0.28) => {
      const s = new THREE.Shape();
      s.moveTo(-wStart * 0.32, yStart);
      s.quadraticCurveTo(-wStart * 0.48, yStart + (yEnd - yStart) * 0.35, -wEnd * 0.35, yEnd);
      s.lineTo(wEnd * 0.65, yEnd);
      s.quadraticCurveTo(wStart * 0.95, yStart + (yEnd - yStart) * 0.25, wStart * 0.68, yStart);
      s.closePath();
      const geo = new THREE.ExtrudeGeometry(s, { depth, bevelEnabled: true, bevelThickness: 0.05, bevelSize: 0.04, bevelSegments: 2 });
      geo.translate(0, 0, -depth / 2);
      return geo;
    };

    const turbineTowerMat = new THREE.MeshStandardMaterial({ map: makeTurbineTowerTex(), roughness: 0.38, metalness: 0.24 });
    const turbineWhite = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.34, metalness: 0.2 });
    const turbineGrey = new THREE.MeshStandardMaterial({ color: 0x475569, roughness: 0.55, metalness: 0.55 });
    const turbineDark = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.55, metalness: 0.6 });
    const turbineRedTip = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.45 });
    const turbineYel = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.48 });
    const turbineConcrete = new THREE.MeshStandardMaterial({ color: 0x7c848e, roughness: 0.92 });
    const turbineGreen = new THREE.MeshStandardMaterial({ color: 0x166534, roughness: 0.65, metalness: 0.25 });
    const turbineBeacon = new THREE.MeshBasicMaterial({ color: 0xff2200 });

    // Shared extruded sculpted blade segments (Root -> Mid White Airfoil -> Twin Red Aviation Safety Bands!)
    const bladeMainGeo = makeAirfoilBladeGeo(1.3, 12.2, 1.38, 0.72, 0.28);
    const bladeBand1Geo = makeAirfoilBladeGeo(12.2, 13.7, 0.72, 0.54, 0.24);
    const bladeMidWGeo = makeAirfoilBladeGeo(13.7, 14.8, 0.54, 0.42, 0.22);
    const bladeTipGeo = makeAirfoilBladeGeo(14.8, 16.4, 0.42, 0.16, 0.18);

    [[-95, 224], [0, 234], [95, 224]].forEach(([tx, tz], idx) => {
      const ty = this.getTerrainHeight(tx, tz);
      const wt = new THREE.Group();
      wt.position.set(tx, ty, tz);

      // 1. Gravel Maintenance Apron + Octagonal Concrete Foundation Pedestal + Bolted Steel Base Flange
      const gravelApron = new THREE.Mesh(new THREE.CylinderGeometry(6.2, 6.5, 0.12, 16), new THREE.MeshStandardMaterial({ color: 0x8b7e6a, roughness: 0.96 }));
      gravelApron.position.y = 0.06; gravelApron.receiveShadow = true;
      const foundPad = new THREE.Mesh(new THREE.CylinderGeometry(4.2, 4.65, 0.55, 8), turbineConcrete);
      foundPad.position.y = 0.28; foundPad.receiveShadow = true; foundPad.castShadow = true;
      const baseFlange = new THREE.Mesh(new THREE.CylinderGeometry(1.75, 1.95, 0.42, 20), turbineGrey);
      baseFlange.position.y = 0.72;
      wt.add(gravelApron, foundPad, baseFlange);

      // 2. Recessed North Steel Entry Door + Grated Steps + Yellow Handrails + Turbine ID Plaque ("WT-01..03")!
      const entryFrame = new THREE.Mesh(new THREE.BoxGeometry(1.32, 2.35, 0.65), turbineGrey);
      entryFrame.position.set(0, 2.15, -1.35);
      const entryDoor = new THREE.Mesh(new THREE.BoxGeometry(1.0, 2.05, 0.14), turbineDark);
      entryDoor.position.set(0, 2.12, -1.66);
      const entryDeck = new THREE.Mesh(new THREE.BoxGeometry(1.85, 0.14, 1.25), turbineGrey);
      entryDeck.position.set(0, 1.05, -2.15);
      const entryStep = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.5, 0.65), turbineConcrete);
      entryStep.position.set(0, 0.65, -2.85);
      [-0.85, 0.85].forEach((rx) => {
        const post = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 1.1, 6), turbineYel);
        post.position.set(rx, 1.55, -2.65);
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 1.25), turbineYel);
        rail.position.set(rx, 2.08, -2.05);
        wt.add(post, rail);
      });
      const badge = new THREE.Mesh(
        new THREE.BoxGeometry(1.25, 0.58, 0.1),
        new THREE.MeshStandardMaterial({ map: makeTurbineBadgeTex(`WT-0${idx + 1}`), roughness: 0.45 })
      );
      badge.position.set(0, 3.68, -1.48);
      badge.rotation.y = Math.PI;
      wt.add(entryFrame, entryDoor, entryDeck, entryStep, badge);

      // Pad-Mounted Step-Up Transformer Cabinet + Yellow Bollards right beside Turbine Base
      const padXf = new THREE.Group();
      padXf.position.set(3.4, 0.52, -0.6);
      const xfBox = new THREE.Mesh(new THREE.BoxGeometry(1.75, 1.55, 1.35), turbineGreen);
      xfBox.position.y = 0.78; xfBox.castShadow = true;
      const xfCap = new THREE.Mesh(new THREE.BoxGeometry(1.85, 0.14, 1.45), turbineDark);
      xfCap.position.y = 1.6;
      const xfStripe = new THREE.Mesh(new THREE.BoxGeometry(1.78, 0.18, 1.38), turbineYel);
      xfStripe.position.y = 1.35;
      [-0.4, 0, 0.4].forEach((fz) => {
        const fin = new THREE.Mesh(new THREE.BoxGeometry(0.24, 1.15, 0.1), turbineDark);
        fin.position.set(0.95, 0.78, fz); padXf.add(fin);
      });
      padXf.add(xfBox, xfCap, xfStripe);
      wt.add(padXf);

      // 3. 20-Sided Tapered Tubular Steel Mast + Circumferential Flange Rings + Yaw Bearing Gear Collar!
      const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.72, 1.52, 34.5, 20), turbineTowerMat);
      mast.position.y = 18.0;
      mast.castShadow = true;
      mast.receiveShadow = true;
      wt.add(mast);
      // Physical raised flange rings at stacked tubular steel joints
      [9.5, 18.5, 27.5].forEach((fy) => {
        const rAtY = 1.52 + (0.72 - 1.52) * ((fy - 0.75) / 34.5);
        const fRing = new THREE.Mesh(new THREE.CylinderGeometry(rAtY + 0.07, rAtY + 0.07, 0.22, 20), turbineGrey);
        fRing.position.y = fy;
        wt.add(fRing);
      });
      // Heavy steel Yaw Bearing Collar joining tower top to nacelle
      const yawCollar = new THREE.Mesh(new THREE.CylinderGeometry(1.05, 0.92, 0.85, 20), turbineDark);
      yawCollar.position.y = 35.35;
      wt.add(yawCollar);

      // 4. Sculpted Aerodynamic Nacelle + Heat-Exchanger Cooler + Heli-Hoist Roof Pad + Twin Red Aviation Beacons!
      const nacGrp = new THREE.Group();
      nacGrp.position.set(0, 36.6, 0.75);
      const nacBody = new THREE.Mesh(new THREE.BoxGeometry(2.55, 2.45, 5.4), turbineWhite);
      nacBody.castShadow = true;
      // Aerodynamic beveled roof cowling & rounded rear generator cap
      const nacTop = new THREE.Mesh(new THREE.BoxGeometry(2.15, 0.45, 5.1), turbineWhite);
      nacTop.position.y = 1.38;
      const nacRear = new THREE.Mesh(new THREE.CylinderGeometry(1.22, 1.22, 2.48, 12), turbineWhite);
      nacRear.rotation.z = Math.PI / 2; nacRear.position.set(0, 0, 2.65);
      // Dark side ventilation cooling louver grilles along both flanks
      [-1.3, 1.3].forEach((nx) => {
        const grille = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.15, 2.45), turbineDark);
        grille.position.set(nx, 0.1, 0.6); nacGrp.add(grille);
      });
      // Rooftop Heat-Exchanger Cooling Radiator + Yellow Heli-Hoist Winch Pad + Safety Guardrails
      const cooler = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.82, 1.85), turbineGrey);
      cooler.position.set(0, 1.88, 1.45); cooler.castShadow = true;
      const hoistPad = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.06, 1.6), turbineYel);
      hoistPad.position.set(0, 1.62, -0.65);
      [-0.95, 0.95].forEach((gx) => {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 2.1), turbineGrey);
        rail.position.set(gx, 2.25, -0.65);
        const p1 = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.65, 0.05), turbineGrey);
        p1.position.set(gx, 1.92, -1.65);
        const p2 = p1.clone(); p2.position.z = 0.35;
        nacGrp.add(rail, p1, p2);
      });
      // Twin Rear Aviation Obstruction Warning Beacons + Ultrasonic Wind Sensor Mast
      [-0.85, 0.85].forEach((bx) => {
        const bPole = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.05, 1.55, 6), turbineGrey);
        bPole.position.set(bx, 2.85, 2.2);
        const bLed = new THREE.Mesh(new THREE.SphereGeometry(0.24, 8, 8), turbineBeacon);
        bLed.position.set(bx, 3.68, 2.2);
        nacGrp.add(bPole, bLed);
      });
      const vanePole = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 1.1, 6), turbineDark);
      vanePole.position.set(0, 2.65, 2.55);
      const vaneFin = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.35, 0.72), turbineYel);
      vaneFin.position.set(0, 3.15, 2.75);
      nacGrp.add(nacBody, nacTop, nacRear, cooler, hoistPad, vanePole, vaneFin);
      wt.add(nacGrp);

      // 5. Rotating 3-Blade Sculpted Airfoil Rotor Hub facing North (-Z toward Airfield!)
      const rotor = new THREE.Group();
      rotor.position.set(0, 36.6, -2.35);
      // Aerodynamic rounded nose spinner + rear hub fairing collar
      const hubCollar = new THREE.Mesh(new THREE.CylinderGeometry(1.18, 1.26, 1.15, 16), turbineWhite);
      hubCollar.rotation.x = Math.PI / 2; hubCollar.position.z = 0.15;
      const hubNose = new THREE.Mesh(new THREE.SphereGeometry(1.18, 16, 12, 0, Math.PI * 2, 0, Math.PI / 2), turbineWhite);
      hubNose.rotation.x = -Math.PI / 2; hubNose.scale.set(1, 1.45, 1); hubNose.position.z = -0.4;
      const hubBand = new THREE.Mesh(new THREE.TorusGeometry(1.22, 0.07, 8, 20), turbineGrey);
      hubBand.position.z = 0.55;
      rotor.add(hubCollar, hubNose, hubBand);

      for (let b = 0; b < 3; b++) {
        const bWrap = new THREE.Group();
        bWrap.rotation.z = (b * Math.PI * 2) / 3 + idx * 0.7;
        // Circular Blade Pitch Bearing Ring + Cylindrical Transition Root
        const pitchRing = new THREE.Mesh(new THREE.CylinderGeometry(0.46, 0.52, 0.35, 12), turbineDark);
        pitchRing.position.y = 1.05;
        const bladeRoot = new THREE.Mesh(new THREE.CylinderGeometry(0.36, 0.46, 0.75, 12), turbineWhite);
        bladeRoot.position.y = 1.45;
        // Sculpted Airfoil Blade pitched at realistic 14° aerodynamic attack angle!
        const airfoil = new THREE.Group();
        airfoil.rotation.y = 0.24;
        const bMain = new THREE.Mesh(bladeMainGeo, turbineWhite);
        bMain.castShadow = true;
        const bBand1 = new THREE.Mesh(bladeBand1Geo, turbineRedTip);
        bBand1.castShadow = true;
        const bMidW = new THREE.Mesh(bladeMidWGeo, turbineWhite);
        const bTip = new THREE.Mesh(bladeTipGeo, turbineRedTip);
        bTip.castShadow = true;
        airfoil.add(bMain, bBand1, bMidW, bTip);
        bWrap.add(pitchRing, bladeRoot, airfoil);
        rotor.add(bWrap);
      }

      wt.add(rotor);
      this.scene.add(wt);
      this.windTurbineRotors.push(rotor);
      this.addCylCollider(tx, tz, 2.2, ty, ty + 38.5, 'wind_turbine');
    });
  }

  buildAirfield() {
    const asphaltMat = new THREE.MeshStandardMaterial({
      map: this.createAsphaltTexture(),
      roughness: 0.88,
      metalness: 0.04
    });

    // Main Runway strip (x = 12..48, z = -80..40)
    const runway = new THREE.Mesh(new THREE.PlaneGeometry(36, 120), asphaltMat);
    runway.rotation.x = -Math.PI / 2;
    runway.position.set(30, 0.04, -20);
    runway.receiveShadow = true;
    this.scene.add(runway);

    // Connecting Taxiway: Helipad (0,0) -> Runway (12,0) & Runway (48,-12) -> Hangar Apron (66,-12)
    const taxiLeft = new THREE.Mesh(new THREE.PlaneGeometry(14, 10), asphaltMat);
    taxiLeft.rotation.x = -Math.PI / 2;
    taxiLeft.position.set(6, 0.035, 0);
    taxiLeft.receiveShadow = true;
    this.scene.add(taxiLeft);

    const apron = new THREE.Mesh(new THREE.PlaneGeometry(24, 34), asphaltMat);
    apron.rotation.x = -Math.PI / 2;
    apron.position.set(60, 0.035, -12);
    apron.receiveShadow = true;
    this.scene.add(apron);

    // Runway white dashed centerline, edge lines & threshold "piano keys"
    const lineMat = new THREE.MeshBasicMaterial({ color: 0xf8fafc });
    const yellowLineMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 });
    for (let rz = -64; rz <= 24; rz += 12) {
      const dash = new THREE.Mesh(new THREE.PlaneGeometry(0.9, 6.2), lineMat);
      dash.rotation.x = -Math.PI / 2;
      dash.position.set(30, 0.06, rz);
      this.scene.add(dash);
    }
    [-16.5, 16.5].forEach(edgeX => {
      const edge = new THREE.Mesh(new THREE.PlaneGeometry(0.5, 118), lineMat);
      edge.rotation.x = -Math.PI / 2;
      edge.position.set(30 + edgeX, 0.06, -20);
      this.scene.add(edge);
    });
    // Threshold piano keys at North (-74) & South (+34) ends
    [-74, 34].forEach(tz => {
      for (let kx = -12; kx <= 12; kx += 4) {
        if (kx === 0) continue;
        const key = new THREE.Mesh(new THREE.PlaneGeometry(1.6, 8.5), lineMat);
        key.rotation.x = -Math.PI / 2;
        key.position.set(30 + kx, 0.06, tz);
        this.scene.add(key);
      }
    });
    // Yellow taxiway guideline
    const taxiLine = new THREE.Mesh(new THREE.PlaneGeometry(12, 0.35), yellowLineMat);
    taxiLine.rotation.x = -Math.PI / 2;
    taxiLine.position.set(6, 0.055, 0);
    this.scene.add(taxiLine);

    // Detailed ICAO/FAA Airport Runway Edge Light Fixtures (MIRL) + Threshold Wingbars — GPU-Instanced!
    const edgeSpecs = [];
    for (let lz = -75; lz <= 35; lz += 12) {
      const kind = (lz <= -63 || lz >= 23) ? 'amber' : 'white';
      edgeSpecs.push({ x: 11.2, z: lz, kind }, { x: 48.8, z: lz, kind });
    }
    // 4 Green Runway Threshold End Marker Lights flanking North (-80.4) & South (+40.4) piano-key thresholds
    [[15.5, -80.4], [44.5, -80.4], [15.5, 40.4], [44.5, 40.4]].forEach(([x, z]) => {
      edgeSpecs.push({ x, z, kind: 'green' });
    });

    const rwBaseMat = new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.9 });
    const rwStemMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.45, metalness: 0.2 }); // Aviation yellow frangible stem
    const rwCollarMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.5, metalness: 0.65 });
    const rwLensWhite = new THREE.MeshBasicMaterial({ color: 0xe0f2fe });
    const rwLensAmber = new THREE.MeshBasicMaterial({ color: 0xfbbf24 });
    const rwLensGreen = new THREE.MeshBasicMaterial({ color: 0x4ade80 });

    const nLights = edgeSpecs.length;
    const baseInst = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.3, 0.36, 0.09, 8), rwBaseMat, nLights);
    const stemInst = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.055, 0.085, 0.36, 8), rwStemMat, nLights);
    const collarInst = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.14, 0.165, 0.14, 10), rwCollarMat, nLights);
    const capRingInst = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.15, 0.15, 0.035, 10), rwCollarMat, nLights);
    const whiteSpecs = edgeSpecs.filter((e) => e.kind === 'white');
    const amberSpecs = edgeSpecs.filter((e) => e.kind === 'amber');
    const greenSpecs = edgeSpecs.filter((e) => e.kind === 'green');
    const domeGeo = new THREE.CylinderGeometry(0.1, 0.135, 0.2, 10);
    const whiteInst = new THREE.InstancedMesh(domeGeo, rwLensWhite, whiteSpecs.length);
    const amberInst = new THREE.InstancedMesh(domeGeo, rwLensAmber, amberSpecs.length);
    const greenInst = new THREE.InstancedMesh(domeGeo, rwLensGreen, greenSpecs.length);

    const rwDummy = new THREE.Object3D();
    edgeSpecs.forEach((e, i) => {
      rwDummy.rotation.set(0, 0, 0);
      rwDummy.scale.set(1, 1, 1);
      rwDummy.position.set(e.x, 0.045, e.z); rwDummy.updateMatrix(); baseInst.setMatrixAt(i, rwDummy.matrix);
      rwDummy.position.set(e.x, 0.26, e.z); rwDummy.updateMatrix(); stemInst.setMatrixAt(i, rwDummy.matrix);
      rwDummy.position.set(e.x, 0.47, e.z); rwDummy.updateMatrix(); collarInst.setMatrixAt(i, rwDummy.matrix);
      rwDummy.position.set(e.x, 0.69, e.z); rwDummy.updateMatrix(); capRingInst.setMatrixAt(i, rwDummy.matrix);
    });
    const fillLens = (arr, inst) => {
      arr.forEach((e, i) => {
        rwDummy.position.set(e.x, 0.6, e.z); rwDummy.updateMatrix(); inst.setMatrixAt(i, rwDummy.matrix);
      });
    };
    fillLens(whiteSpecs, whiteInst);
    fillLens(amberSpecs, amberInst);
    fillLens(greenSpecs, greenInst);
    this.scene.add(baseInst, stemInst, collarInst, capRingInst, whiteInst, amberInst, greenInst);

    // Solid High-Detail Aviation Hangar positioned off the runway at (66, 0, -12), opening facing -X (runway)
    const hangarGroup = new THREE.Group();
    hangarGroup.position.set(66, 0, -12);

    // Procedural Corrugated Galvanized Steel Siding Texture (vertical ribs + dado base band)
    const makeHangarSidingTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#475569';
      ctx.fillRect(0, 0, 256, 256);
      // Corrugated vertical steel sheet ribs
      for (let x = 0; x < 256; x += 8) {
        ctx.fillStyle = 'rgba(255, 255, 255, 0.12)';
        ctx.fillRect(x, 0, 2, 256);
        ctx.fillStyle = 'rgba(15, 23, 42, 0.34)';
        ctx.fillRect(x + 4, 0, 3, 256);
      }
      // Horizontal architectural purlin seam lines & darker concrete splash dado along lower wall
      ctx.fillStyle = 'rgba(15, 23, 42, 0.32)';
      ctx.fillRect(0, 84, 256, 3);
      ctx.fillRect(0, 168, 256, 3);
      ctx.fillStyle = 'rgba(30, 41, 59, 0.55)';
      ctx.fillRect(0, 206, 256, 50);
      ctx.fillStyle = '#f97316';
      ctx.fillRect(0, 202, 256, 4);
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.ClampToEdgeWrapping;
      tex.repeat.set(4, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural Standing-Seam Metal Roof Panel Texture
    const makeHangarRoofTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#526275';
      ctx.fillRect(0, 0, 256, 256);
      for (let x = 0; x < 256; x += 16) {
        ctx.fillStyle = 'rgba(241, 245, 249, 0.18)';
        ctx.fillRect(x, 0, 2, 256);
        ctx.fillStyle = 'rgba(15, 23, 42, 0.35)';
        ctx.fillRect(x + 2, 0, 2, 256);
      }
      for (let y = 0; y < 256; y += 64) {
        ctx.fillStyle = 'rgba(15, 23, 42, 0.24)';
        ctx.fillRect(0, y, 256, 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(4, 3);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural "HANGAR 01 — DRONE OPS" Aviation Front Signboard
    const makeHangarSignTex = () => {
      const c = document.createElement('canvas');
      c.width = 512;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(0, 0, 512, 128);
      ctx.fillStyle = '#f97316';
      ctx.fillRect(0, 0, 512, 12);
      ctx.fillRect(0, 116, 512, 12);
      // Hazard stripes at left & right ends
      ctx.fillStyle = '#facc15';
      ctx.fillRect(14, 22, 14, 84);
      ctx.fillRect(484, 22, 14, 84);
      ctx.fillStyle = '#f8fafc';
      ctx.font = 'bold 54px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('HANGAR 01', 256, 54);
      ctx.fillStyle = '#94a3b8';
      ctx.font = 'bold 20px sans-serif';
      ctx.fillText('UAV FLIGHT OPERATIONS & MAINTENANCE', 256, 95);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural Black-and-Yellow Aviation Caution Hazard Stripe Texture
    const makeHazardTex = () => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#facc15';
      ctx.fillRect(0, 0, 128, 128);
      ctx.fillStyle = '#18181b';
      for (let i = -128; i < 256; i += 32) {
        ctx.beginPath();
        ctx.moveTo(i, 0);
        ctx.lineTo(i + 16, 0);
        ctx.lineTo(i + 144, 128);
        ctx.lineTo(i + 128, 128);
        ctx.closePath();
        ctx.fill();
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(1, 4);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const hWallMat = new THREE.MeshStandardMaterial({ map: makeHangarSidingTex(), roughness: 0.65, metalness: 0.28, side: THREE.DoubleSide });
    const hGableMat = new THREE.MeshStandardMaterial({ color: 0x3b495c, roughness: 0.68, metalness: 0.28, side: THREE.DoubleSide });
    const hRoofMat = new THREE.MeshStandardMaterial({ map: makeHangarRoofTex(), roughness: 0.52, metalness: 0.38, side: THREE.DoubleSide });
    const hAccentMat = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.42, metalness: 0.22, side: THREE.DoubleSide });
    const hFloorMat = new THREE.MeshStandardMaterial({ color: 0x1e2530, roughness: 0.85, metalness: 0.08 });
    const hSteelMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.45, metalness: 0.68, side: THREE.DoubleSide });
    const hLightSteelMat = new THREE.MeshStandardMaterial({ color: 0xcbd5e1, roughness: 0.38, metalness: 0.62, side: THREE.DoubleSide });
    const hGlassMat = new THREE.MeshStandardMaterial({ color: 0x7dd3fc, roughness: 0.12, metalness: 0.65, emissive: 0x334155, emissiveIntensity: 0.45, side: THREE.DoubleSide });
    const hHazardMat = new THREE.MeshStandardMaterial({ map: makeHazardTex(), roughness: 0.55, side: THREE.DoubleSide });
    const hYellowMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.48, metalness: 0.25 });

    // Concrete floor with painted yellow bay safety lines, apron lead-in line & indoor Drone Service Pad!
    const hFloor = new THREE.Mesh(new THREE.BoxGeometry(16, 0.2, 24), hFloorMat);
    hFloor.position.set(0, 0.1, 0);
    hFloor.receiveShadow = true;
    hangarGroup.add(hFloor);

    const bayLineMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 });
    const bayFront = new THREE.Mesh(new THREE.PlaneGeometry(0.35, 21), bayLineMat);
    bayFront.rotation.x = -Math.PI / 2;
    bayFront.position.set(-7.1, 0.22, 0);
    hangarGroup.add(bayFront);

    [-8.5, 8.5].forEach(bz => {
      const baySide = new THREE.Mesh(new THREE.PlaneGeometry(12.5, 0.28), bayLineMat);
      baySide.rotation.x = -Math.PI / 2;
      baySide.position.set(-0.5, 0.22, bz);
      hangarGroup.add(baySide);
    });

    // Central painted Drone Service Circle & Apron Taxi Lead-in Line entering Hangar bay
    const indoorPadRing = new THREE.Mesh(new THREE.RingGeometry(2.8, 3.15, 24), bayLineMat);
    indoorPadRing.rotation.x = -Math.PI / 2;
    indoorPadRing.position.set(-0.8, 0.225, 0);
    const apronLeadLine = new THREE.Mesh(new THREE.PlaneGeometry(14.0, 0.3), bayLineMat);
    apronLeadLine.rotation.x = -Math.PI / 2;
    apronLeadLine.position.set(-14.5, 0.055, 0);
    hangarGroup.add(indoorPadRing, apronLeadLine);

    // North & South side walls (corrugated steel + external structural buttress I-beams + clerestory factory windows!)
    [-11.6, 11.6].forEach((wz, sideIdx) => {
      const wall = new THREE.Mesh(new THREE.BoxGeometry(16, 6.2, 0.7), hWallMat);
      wall.position.set(0, 3.1, wz);
      wall.castShadow = true;
      wall.receiveShadow = true;
      hangarGroup.add(wall);

      const outZ = wz > 0 ? wz + 0.42 : wz - 0.42;
      const inZ = wz > 0 ? wz - 0.5 : wz + 0.5;

      // Exterior & Interior structural steel I-beam columns + concrete foundation bumper curbs
      [-7.4, -3.7, 0, 3.7, 7.4].forEach(cx => {
        const extCol = new THREE.Mesh(new THREE.BoxGeometry(0.36, 6.25, 0.34), hSteelMat);
        extCol.position.set(cx, 3.12, outZ);
        extCol.castShadow = true;
        hangarGroup.add(extCol);
      });
      [-4.5, 0, 4.5].forEach(cx => {
        const col = new THREE.Mesh(new THREE.BoxGeometry(0.45, 6.1, 0.45), hSteelMat);
        col.position.set(cx, 3.1, inZ);
        hangarGroup.add(col);
      });

      // 4 High Clerestory Industrial Factory Windows along each exterior wall
      [-5.55, -1.85, 1.85, 5.55].forEach(wx => {
        const wf = new THREE.Mesh(new THREE.BoxGeometry(2.65, 1.18, 0.16), hSteelMat);
        wf.position.set(wx, 4.65, outZ - (wz > 0 ? 0.04 : -0.04));
        const wp = new THREE.Mesh(new THREE.BoxGeometry(2.42, 0.96, 0.2), hGlassMat);
        wp.position.set(wx, 4.65, outZ - (wz > 0 ? 0.04 : -0.04));
        const wm = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.0, 0.22), hSteelMat);
        wm.position.set(wx, 4.65, outZ - (wz > 0 ? 0.04 : -0.04));
        hangarGroup.add(wf, wp, wm);
      });

      // Side Personnel Steel Exit Door + Canopy + Protective Yellow Bollards on South wall (+11.6)
      if (sideIdx === 1) {
        const pFrame = new THREE.Mesh(new THREE.BoxGeometry(1.38, 2.35, 0.18), hSteelMat);
        pFrame.position.set(-5.4, 1.25, wz + 0.38);
        const pDoor = new THREE.Mesh(new THREE.BoxGeometry(1.16, 2.2, 0.22), hLightSteelMat);
        pDoor.position.set(-5.4, 1.22, wz + 0.4);
        const pWin = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.55, 0.25), hGlassMat);
        pWin.position.set(-5.4, 1.75, wz + 0.4);
        const pAwning = new THREE.Mesh(new THREE.BoxGeometry(1.85, 0.12, 0.85), hAccentMat);
        pAwning.position.set(-5.4, 2.65, wz + 0.72);
        pAwning.rotation.x = 0.2;
        [-6.35, -4.45].forEach(bx => {
          const bol = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.95, 8), hYellowMat);
          bol.position.set(bx, 0.48, wz + 0.88);
          hangarGroup.add(bol);
        });
        hangarGroup.add(pFrame, pDoor, pWin, pAwning);
      }
    });

    // East back wall (sealed solid) + Exterior Rear Steel Reinforcements & High Windows
    const backWall = new THREE.Mesh(new THREE.BoxGeometry(0.7, 6.2, 24), hWallMat);
    backWall.position.set(7.65, 3.1, 0);
    backWall.castShadow = true;
    backWall.receiveShadow = true;
    hangarGroup.add(backWall);

    [-6.0, 0, 6.0].forEach(bz => {
      const bwFrame = new THREE.Mesh(new THREE.BoxGeometry(0.16, 1.25, 3.6), hSteelMat);
      bwFrame.position.set(8.02, 4.65, bz);
      const bwPane = new THREE.Mesh(new THREE.BoxGeometry(0.2, 1.02, 3.35), hGlassMat);
      bwPane.position.set(8.02, 4.65, bz);
      hangarGroup.add(bwFrame, bwPane);
    });

    // Front header beam + orange aviation stripe + overhead sliding door track rail facing runway (-X)
    const frontHeader = new THREE.Mesh(new THREE.BoxGeometry(0.85, 1.5, 24), hSteelMat);
    frontHeader.position.set(-7.6, 5.5, 0);
    frontHeader.castShadow = true;
    const orangeStripe = new THREE.Mesh(new THREE.BoxGeometry(0.96, 0.4, 24.15), hAccentMat);
    orangeStripe.position.set(-7.6, 5.05, 0);
    const doorTrack = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.22, 24.4), hLightSteelMat);
    doorTrack.position.set(-8.12, 4.82, 0);
    hangarGroup.add(frontHeader, orangeStripe, doorTrack);

    // Retracted Open Sliding Bi-Fold Steel Hangar Doors parked at left & right bay corners (center 15.6m stays WIDE OPEN!)
    [-9.6, 9.6].forEach((dz) => {
      const doorLeaf = new THREE.Mesh(new THREE.BoxGeometry(0.28, 4.65, 3.6), hWallMat);
      doorLeaf.position.set(-8.08, 2.48, dz);
      doorLeaf.castShadow = true;
      const doorFrame = new THREE.Mesh(new THREE.BoxGeometry(0.34, 4.72, 0.16), hSteelMat);
      doorFrame.position.set(-8.08, 2.48, dz > 0 ? dz - 1.75 : dz + 1.75);
      // Black-and-yellow hazard safety strip on leading door edge!
      const hazStrip = new THREE.Mesh(new THREE.BoxGeometry(0.36, 4.5, 0.34), hHazardMat);
      hazStrip.position.set(-8.08, 2.46, dz > 0 ? dz - 1.56 : dz + 1.56);
      hangarGroup.add(doorLeaf, doorFrame, hazStrip);
    });

    // 4 Glowing Industrial Exterior Floodlights above Hangar Bay Entrance
    const ledBarMat = new THREE.MeshBasicMaterial({ color: 0xfef9c3 });
    [-6.6, -2.2, 2.2, 6.6].forEach((fz) => {
      const fArm = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.08, 0.12), hSteelMat);
      fArm.position.set(-8.22, 5.65, fz);
      const fHood = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.22, 0.72), hSteelMat);
      fHood.position.set(-8.5, 5.58, fz);
      fHood.rotation.z = 0.35;
      const fLamp = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.08, 0.62), ledBarMat);
      fLamp.position.set(-8.52, 5.46, fz);
      fLamp.rotation.z = 0.35;
      hangarGroup.add(fArm, fHood, fLamp);
    });

    // Pitched A-frame Standing-Seam Metal Roof + Skylights + Full-Length Ridge Ventilator Monitor + Rooftop HVAC & Solar!
    const roofSlopeL = new THREE.Mesh(new THREE.BoxGeometry(17.4, 0.38, 13.2), hRoofMat);
    roofSlopeL.position.set(0, 7.6, -6.15);
    roofSlopeL.rotation.x = -0.22;
    roofSlopeL.castShadow = true;
    roofSlopeL.receiveShadow = true;
    hangarGroup.add(roofSlopeL);

    const roofSlopeR = new THREE.Mesh(new THREE.BoxGeometry(17.4, 0.38, 13.2), hRoofMat);
    roofSlopeR.position.set(0, 7.6, 6.15);
    roofSlopeR.rotation.x = 0.22;
    roofSlopeR.castShadow = true;
    roofSlopeR.receiveShadow = true;
    hangarGroup.add(roofSlopeR);

    // Raised Louvered Roof Ridge Ventilator Monitor running cleanly along the roof peak (y = 9.0..9.75)
    const ridgeVentBase = new THREE.Mesh(new THREE.BoxGeometry(13.8, 0.62, 1.55), hSteelMat);
    ridgeVentBase.position.set(0, 9.22, 0);
    ridgeVentBase.castShadow = true;
    const ridgeVentCap = new THREE.Mesh(new THREE.BoxGeometry(14.4, 0.18, 2.05), hLightSteelMat);
    ridgeVentCap.position.set(0, 9.6, 0);
    ridgeVentCap.castShadow = true;
    hangarGroup.add(ridgeVentBase, ridgeVentCap);

    // 6 Translucent Roof Skylight Strips flush on North & South slopes + 3 Rooftop Turbine Exhaust Vents
    [-4.8, 0, 4.8].forEach((sx) => {
      [-5.2, 5.2].forEach((sz) => {
        const skyFrame = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.12, 2.1), hSteelMat);
        skyFrame.position.set(sx, 7.98, sz);
        skyFrame.rotation.x = sz < 0 ? -0.22 : 0.22;
        const skyPane = new THREE.Mesh(new THREE.BoxGeometry(2.38, 0.15, 1.88), hGlassMat);
        skyPane.position.set(sx, 7.99, sz);
        skyPane.rotation.x = sz < 0 ? -0.22 : 0.22;
        hangarGroup.add(skyFrame, skyPane);
      });
      // Spinning-style metallic roof turbine exhaust dome on North slope
      const tBase = new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.38, 0.45, 10), hSteelMat);
      tBase.position.set(sx, 7.22, -9.2);
      const tDome = new THREE.Mesh(new THREE.SphereGeometry(0.46, 10, 8), hLightSteelMat);
      tDome.position.set(sx, 7.62, -9.2);
      hangarGroup.add(tBase, tDome);
    });

    // Rooftop Solar Power Array mounted neatly along South roof slope (z = +8.8)
    const hSolarMat = new THREE.MeshStandardMaterial({ color: 0x0f2d6e, roughness: 0.18, metalness: 0.78 });
    [-4.8, 0, 4.8].forEach((sx) => {
      const sFrame = new THREE.Mesh(new THREE.BoxGeometry(3.8, 0.12, 2.2), hLightSteelMat);
      sFrame.position.set(sx, 7.28, 8.85);
      sFrame.rotation.x = 0.22;
      const sCells = new THREE.Mesh(new THREE.BoxGeometry(3.62, 0.15, 2.02), hSolarMat);
      sCells.position.set(sx, 7.29, 8.85);
      sCells.rotation.x = 0.22;
      hangarGroup.add(sFrame, sCells);
    });

    // Solid thick front & back triangular gable walls (ExtrudeGeometry -> 100% opaque from inside & outside!)
    const gableShape = new THREE.Shape([
      new THREE.Vector2(-12.0, -0.1),
      new THREE.Vector2(12.0, -0.1),
      new THREE.Vector2(0, 2.85)
    ]);
    const gableGeo = new THREE.ExtrudeGeometry(gableShape, { depth: 0.65, bevelEnabled: false });
    const frontGable = new THREE.Mesh(gableGeo, hGableMat);
    frontGable.rotation.y = -Math.PI / 2;
    frontGable.position.set(-7.3, 6.2, 0);
    hangarGroup.add(frontGable);

    const backGable = new THREE.Mesh(gableGeo, hGableMat);
    backGable.rotation.y = Math.PI / 2;
    backGable.position.set(7.35, 6.2, 0);
    hangarGroup.add(backGable);

    // Bold "HANGAR 01" Aviation Signboard on Front Gable facing the Runway (-X)
    const signBack = new THREE.Mesh(new THREE.BoxGeometry(0.18, 1.52, 7.1), hSteelMat);
    signBack.position.set(-8.02, 7.15, 0);
    const signFace = new THREE.Mesh(
      new THREE.PlaneGeometry(6.8, 1.32),
      new THREE.MeshStandardMaterial({ map: makeHangarSignTex(), roughness: 0.4, metalness: 0.2 })
    );
    signFace.rotation.y = -Math.PI / 2;
    signFace.position.set(-8.12, 7.15, 0);
    hangarGroup.add(signBack, signFace);

    // Exterior White Cylindrical Aviation Fuel Storage Tank (AVGAS) on Steel Saddles beside North Wall
    const fuelTankGrp = new THREE.Group();
    fuelTankGrp.position.set(2.2, 0, -14.2);
    const tankCyl = new THREE.Mesh(new THREE.CylinderGeometry(1.05, 1.05, 5.2, 16), hLightSteelMat);
    tankCyl.rotation.z = Math.PI / 2;
    tankCyl.position.y = 1.65;
    tankCyl.castShadow = true;
    const tankStripe = new THREE.Mesh(new THREE.CylinderGeometry(1.07, 1.07, 0.75, 16), hAccentMat);
    tankStripe.rotation.z = Math.PI / 2;
    tankStripe.position.y = 1.65;
    [-1.6, 1.6].forEach((lx) => {
      const saddle = new THREE.Mesh(new THREE.BoxGeometry(0.32, 1.25, 1.8), hSteelMat);
      saddle.position.set(lx, 0.62, 0);
      fuelTankGrp.add(saddle);
    });
    [-2.8, 0, 2.8].forEach((bx) => {
      const guard = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 1.0, 8), hYellowMat);
      guard.position.set(bx, 0.5, -1.55);
      fuelTankGrp.add(guard);
    });
    fuelTankGrp.add(tankCyl, tankStripe);
    hangarGroup.add(fuelTankGrp);

    // Exposed Steel Ceiling Trusses + Overhead Yellow Gantry Crane Beam + Glowing Industrial Shop Lights
    [-4.5, 0, 4.5].forEach(tx => {
      const trussBeam = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.35, 22.8), hSteelMat);
      trussBeam.position.set(tx, 6.1, 0);
      hangarGroup.add(trussBeam);

      // Vertical king post + angled web struts connecting truss to roof slopes
      const kingPost = new THREE.Mesh(new THREE.BoxGeometry(0.28, 2.6, 0.28), hSteelMat);
      kingPost.position.set(tx, 7.45, 0);
      hangarGroup.add(kingPost);
      [-1, 1].forEach((s) => {
        const strut = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.18, 5.2), hSteelMat);
        strut.position.set(tx, 7.1, s * 3.1);
        strut.rotation.x = s * 0.42;
        hangarGroup.add(strut);
      });

      // 2 Glowing overhead fluorescent LED shop lights per truss
      [-4.8, 4.8].forEach(lz => {
        const fixture = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.14, 3.2), hSteelMat);
        fixture.position.set(tx, 5.88, lz);
        const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.08, 3.0), ledBarMat);
        lamp.position.set(tx, 5.79, lz);
        hangarGroup.add(fixture, lamp);
      });
    });

    // Overhead Industrial Yellow Gantry Crane I-Beam + Hoist Trolley
    const craneBeam = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.48, 22.2), hYellowMat);
    craneBeam.position.set(-1.6, 5.62, 0);
    const craneTrolley = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.42, 1.15), hSteelMat);
    craneTrolley.position.set(-1.6, 5.32, 1.8);
    hangarGroup.add(craneBeam, craneTrolley);

    // Warm interior hangar work light
    // Warm additive overhead hangar glow strips (zero global PointLight shader overhead across the valley!)

    // ---- Hangar Interior Props (Workbench, Dual Monitors, Red Tool Chest, Battery Charging Station, Shelving, Drums, Cones) ----
    const benchWoodMat = new THREE.MeshStandardMaterial({ color: 0x9a6536, roughness: 0.75 });
    const toolRedMat = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.38, metalness: 0.45 });
    const drumBlueMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.42, metalness: 0.5 });
    const coneOrangeMat = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.5 });

    // Mechanic Workbench along East back wall + Pegboard Tool Wall + Dual Telemetry Diagnostic Monitors
    const benchTop = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.18, 6.5), benchWoodMat);
    benchTop.position.set(6.2, 1.35, -1.5);
    benchTop.castShadow = true;
    const pegboard = new THREE.Mesh(new THREE.BoxGeometry(0.1, 1.7, 5.6), hSteelMat);
    pegboard.position.set(7.24, 2.65, -1.5);
    hangarGroup.add(benchTop, pegboard);
    [-4.4, 1.4].forEach(bz => {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(1.6, 1.25, 0.18), hSteelMat);
      leg.position.set(6.2, 0.72, bz);
      hangarGroup.add(leg);
    });
    [-2.3, -0.7].forEach((mz) => {
      const mon = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.78, 1.25), hSteelMat);
      mon.position.set(6.7, 1.88, mz);
      const monScr = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.64, 1.1), new THREE.MeshBasicMaterial({ color: 0x38bdf8 }));
      monScr.position.set(6.68, 1.88, mz);
      hangarGroup.add(mon, monScr);
    });

    // Rolling Red Mechanic Tool Cabinet + LiPo Battery Charging Cabinet with Green Status LEDs
    const toolCab = new THREE.Mesh(new THREE.BoxGeometry(1.4, 1.65, 2.2), toolRedMat);
    toolCab.position.set(6.3, 1.02, 3.6);
    toolCab.castShadow = true;
    const chargeCab = new THREE.Mesh(new THREE.BoxGeometry(1.3, 2.1, 1.8), hSteelMat);
    chargeCab.position.set(6.35, 1.25, -6.4);
    const chargeLed = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.12, 1.1), new THREE.MeshBasicMaterial({ color: 0x22c55e }));
    chargeLed.position.set(5.68, 1.95, -6.4);
    // Wall-mounted Red Fire Extinguisher beside Hangar entrance column
    const fireExt = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.14, 0.75, 10), toolRedMat);
    fireExt.position.set(-7.1, 1.55, 10.9);
    hangarGroup.add(toolCab, chargeCab, chargeLed, fireExt);

    // Industrial Shelving Rack along South interior wall (z = 10.1)
    const shelfGroup = new THREE.Group();
    shelfGroup.position.set(1.5, 0.2, 10.1);
    [-3.2, 0, 3.2].forEach(sx => {
      const postF = new THREE.Mesh(new THREE.BoxGeometry(0.14, 3.6, 0.14), hSteelMat);
      postF.position.set(sx, 1.8, -0.7);
      const postB = postF.clone();
      postB.position.z = 0.7;
      shelfGroup.add(postF, postB);
    });
    [0.9, 2.0, 3.1].forEach(sy => {
      const plank = new THREE.Mesh(new THREE.BoxGeometry(6.8, 0.12, 1.5), benchWoodMat);
      plank.position.y = sy;
      shelfGroup.add(plank);
    });
    // Spare parts crates on shelves
    const boxMat = new THREE.MeshStandardMaterial({ color: 0xb45309, roughness: 0.82 });
    [[-2.0, 1.35], [0.6, 1.35], [-1.2, 2.45], [1.8, 2.45]].forEach(([bx, by]) => {
      const sb = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.75, 1.0), boxMat);
      sb.position.set(bx, by, 0);
      shelfGroup.add(sb);
    });
    hangarGroup.add(shelfGroup);

    // Aviation Fuel Barrels along North interior wall (z = -9.8)
    [[-3.0, -9.8, drumBlueMat], [-1.5, -9.9, hAccentMat], [-2.2, -8.5, drumBlueMat], [3.5, -9.8, hAccentMat]].forEach(([dx, dz, dm]) => {
      const drum = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.55, 1.55, 14), dm);
      drum.position.set(dx, 0.98, dz);
      drum.castShadow = true;
      hangarGroup.add(drum);
    });

    // Orange Traffic Safety Cones marking Hangar entrance corners
    [[-6.6, -7.2], [-6.6, 7.2]].forEach(([cx, cz]) => {
      const coneBase = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.08, 0.6), hSteelMat);
      coneBase.position.set(cx, 0.24, cz);
      const cone = new THREE.Mesh(new THREE.ConeGeometry(0.24, 0.8, 10), coneOrangeMat);
      cone.position.set(cx, 0.65, cz);
      hangarGroup.add(coneBase, cone);
    });

    this.scene.add(hangarGroup);

    // Hangar Solid Colliders (side walls, back wall, front header, retracted doors, roof & exterior fuel tank — center bay stays open!)
    this.addBoxCollider(66, 3.1, -12 - 11.6, 16.4, 6.2, 1.0, 'hangar'); // North wall
    this.addBoxCollider(66, 3.1, -12 + 11.6, 16.4, 6.2, 1.0, 'hangar'); // South wall
    this.addBoxCollider(66 + 7.65, 3.1, -12, 1.0, 6.2, 24.4, 'hangar'); // East back wall
    this.addBoxCollider(66 - 7.6, 5.5, -12, 1.1, 1.6, 24.4, 'hangar');  // Front upper header beam
    this.addBoxCollider(66 - 8.1, 2.5, -12 - 9.6, 0.8, 4.8, 3.7, 'hangar'); // North retracted door leaf
    this.addBoxCollider(66 - 8.1, 2.5, -12 + 9.6, 0.8, 4.8, 3.7, 'hangar'); // South retracted door leaf
    this.addBoxCollider(66, 7.8, -12, 17.8, 3.4, 25.0, 'hangar');       // Pitched A-frame roof + ridge ventilator mass
    this.addBoxCollider(68.2, 1.5, -26.2, 5.8, 3.0, 2.8, 'hangar');     // Exterior AVGAS fuel storage tank

    // Cargo Crates neatly stacked on hangar apron (not on the runway!)
    const crateMat = new THREE.MeshStandardMaterial({ color: 0xb45309, roughness: 0.8 });
    const cratePositions = [
      [56, 0.75, 2],
      [58, 0.75, 2.5],
      [57, 2.25, 2.2],
      [55.5, 0.75, 4.2]
    ];
    cratePositions.forEach(c => {
      const cMesh = new THREE.Mesh(new THREE.BoxGeometry(1.5, 1.5, 1.5), crateMat);
      cMesh.position.set(c[0], c[1], c[2]);
      cMesh.castShadow = true;
      this.scene.add(cMesh);
      this.addBoxCollider(c[0], c[1], c[2], 1.6, 1.6, 1.6);
    });

    // Detailed Air Traffic Control Tower cleanly positioned beside Airfield at (-16, 0, -34)
    // (20+ meters clear of the West Agricultural Field!)
    const towerGroup = new THREE.Group();
    towerGroup.position.set(-16, 0, -34);

    // Procedural architectural precast concrete panel cladding texture (seams, tie-rod dots & subtle weathering)
    const makeTowerConcreteTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 512;
      const ctx = c.getContext('2d');
      const grad = ctx.createLinearGradient(0, 0, 0, 512);
      grad.addColorStop(0, '#cfd8e3');
      grad.addColorStop(0.5, '#bcc7d6');
      grad.addColorStop(1, '#9aa7b8');
      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, 256, 512);
      // Precast architectural concrete slab joints (horizontal & vertical)
      for (let y = 0; y < 512; y += 32) {
        ctx.fillStyle = 'rgba(30, 41, 59, 0.28)';
        ctx.fillRect(0, y, 256, 2);
        ctx.fillStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.fillRect(0, y + 2, 256, 1);
      }
      for (let x = 0; x < 256; x += 32) {
        ctx.fillStyle = 'rgba(30, 41, 59, 0.18)';
        ctx.fillRect(x, 0, 1.5, 512);
      }
      // Formwork tie-rod dots inside each concrete panel
      ctx.fillStyle = 'rgba(51, 65, 85, 0.35)';
      for (let y = 16; y < 512; y += 32) {
        for (let x = 16; x < 256; x += 32) {
          ctx.beginPath();
          ctx.arc(x - 6, y, 1.8, 0, Math.PI * 2);
          ctx.arc(x + 6, y, 1.8, 0, Math.PI * 2);
          ctx.fill();
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(4, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural "ATC-01" aviation tower signage plaque
    const makeAtcSignTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(0, 0, 256, 128);
      ctx.fillStyle = '#f97316';
      ctx.fillRect(0, 0, 256, 14);
      ctx.fillRect(0, 114, 256, 14);
      ctx.fillStyle = '#f8fafc';
      ctx.font = 'bold 54px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('ATC-01', 128, 56);
      ctx.fillStyle = '#94a3b8';
      ctx.font = 'bold 18px sans-serif';
      ctx.fillText('AIR TRAFFIC CONTROL', 128, 96);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const concreteMat = new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.82, metalness: 0.08, side: THREE.DoubleSide });
    const concreteLightMat = new THREE.MeshStandardMaterial({ map: makeTowerConcreteTex(), roughness: 0.82, metalness: 0.06, side: THREE.DoubleSide });
    const ribConcreteMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.78, metalness: 0.1, side: THREE.DoubleSide });
    const darkMetalMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.45, metalness: 0.62, side: THREE.DoubleSide });
    const steelLightMat = new THREE.MeshStandardMaterial({ color: 0xcbd5e1, roughness: 0.35, metalness: 0.72, side: THREE.DoubleSide });
    const glassMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, roughness: 0.08, metalness: 0.88, side: THREE.DoubleSide });
    const darkGlassMat = new THREE.MeshStandardMaterial({ color: 0x0b4f78, roughness: 0.12, metalness: 0.85, emissive: 0x1c3d52, emissiveIntensity: 0.35, side: THREE.DoubleSide });
    const towerAccentMat = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.45, metalness: 0.25, side: THREE.DoubleSide });
    const beaconRedMat = new THREE.MeshBasicMaterial({ color: 0xff2200 });

    // Concrete plaza apron + perimeter curb around Control Tower base
    const towerApron = new THREE.Mesh(new THREE.CylinderGeometry(8.2, 8.5, 0.18, 24), darkMetalMat);
    towerApron.position.y = 0.09;
    towerApron.receiveShadow = true;
    towerGroup.add(towerApron);

    // Ground lobby base + ribbed pilasters + entrance canopy & double glass doors facing runway (+X)
    const lobby = new THREE.Mesh(new THREE.CylinderGeometry(5.15, 5.55, 3.4, 16), concreteMat);
    lobby.position.y = 1.78;
    lobby.castShadow = true;
    lobby.receiveShadow = true;
    towerGroup.add(lobby);

    const lobbyRoofTrim = new THREE.Mesh(new THREE.CylinderGeometry(5.32, 5.25, 0.32, 16), darkMetalMat);
    lobbyRoofTrim.position.y = 3.55;
    towerGroup.add(lobbyRoofTrim);

    const entryBox = new THREE.Mesh(new THREE.BoxGeometry(2.4, 2.85, 3.4), darkMetalMat);
    entryBox.position.set(4.8, 1.52, 0);
    entryBox.castShadow = true;
    towerGroup.add(entryBox);

    const entryGlass = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.2, 2.3), glassMat);
    entryGlass.position.set(6.0, 1.28, 0);
    const entryDoorMullion = new THREE.Mesh(new THREE.BoxGeometry(0.18, 2.25, 0.12), steelLightMat);
    entryDoorMullion.position.set(6.02, 1.28, 0);
    const entryCanopy = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.24, 4.0), towerAccentMat);
    entryCanopy.position.set(5.9, 2.95, 0);
    towerGroup.add(entryGlass, entryDoorMullion, entryCanopy);

    // Backup Power Generator / Electrical Transformer Box on West side of lobby apron
    const genBox = new THREE.Mesh(new THREE.BoxGeometry(1.8, 1.5, 2.5), darkMetalMat);
    genBox.position.set(-6.1, 0.92, 1.8);
    genBox.castShadow = true;
    const genVent = new THREE.Mesh(new THREE.BoxGeometry(1.86, 0.7, 1.4), ribConcreteMat);
    genVent.position.set(-6.1, 1.0, 1.8);
    const genStack = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.12, 1.2, 8), steelLightMat);
    genStack.position.set(-6.1, 2.2, 1.0);
    towerGroup.add(genBox, genVent, genStack);

    // =========================================================================
    // HIGH-DETAIL TAPERED CONTROL TOWER SHAFT (y = 3.4 .. 18.4, 16-sided concrete cladding!)
    // =========================================================================
    const shaftBotY = 3.4;
    const shaftH = 15.0;
    const shaftBotR = 4.05;
    const shaftTopR = 3.15;
    const shaftMidY = shaftBotY + shaftH / 2; // 10.9
    const shaftMidR = (shaftBotR + shaftTopR) / 2; // 3.60
    const taperRad = Math.atan2(shaftBotR - shaftTopR, shaftH); // exact inward taper angle (~3.43 deg)
    const shaftRAt = (y) => shaftBotR - ((y - shaftBotY) / shaftH) * (shaftBotR - shaftTopR);

    const tBase = new THREE.Mesh(new THREE.CylinderGeometry(shaftTopR, shaftBotR, shaftH, 16), concreteLightMat);
    tBase.position.y = shaftMidY;
    tBase.castShadow = true;
    tBase.receiveShadow = true;
    towerGroup.add(tBase);

    // 1. Three Multi-Tiered Structural Floor Collars (concrete slab + recessed dark steel shadow band)
    [6.8, 10.6, 14.4].forEach((by) => {
      const rB = shaftRAt(by);
      const collarMain = new THREE.Mesh(new THREE.CylinderGeometry(rB + 0.18, rB + 0.22, 0.44, 16), darkMetalMat);
      collarMain.position.y = by;
      collarMain.castShadow = true;
      const collarCap = new THREE.Mesh(new THREE.CylinderGeometry(rB + 0.24, rB + 0.24, 0.12, 16), steelLightMat);
      collarCap.position.y = by + 0.24;
      towerGroup.add(collarMain, collarCap);
    });

    // 2. Eight Flush Taper-Aligned External Concrete Buttress Ribs + Upper Cantilever Corbel Brackets!
    for (let i = 0; i < 8; i++) {
      const ang = (i * Math.PI) / 4 + Math.PI / 8; // 22.5° offset so ribs neatly frame all 4 cardinal window/elevator bays
      const ribGrp = new THREE.Group();
      ribGrp.rotation.y = ang;

      // Full-height tapered vertical concrete fin hugging the exact shaft slope (ZERO floating gap!)
      const rib = new THREE.Mesh(new THREE.BoxGeometry(0.42, shaftH - 0.1, 0.38), ribConcreteMat);
      rib.position.set(shaftMidR + 0.08, shaftMidY, 0);
      rib.rotation.z = taperRad;
      rib.castShadow = true;
      rib.receiveShadow = true;
      ribGrp.add(rib);

      // Dark metal base shoe anchoring the rib onto the lobby roof
      const ribFoot = new THREE.Mesh(new THREE.BoxGeometry(0.54, 0.55, 0.5), darkMetalMat);
      ribFoot.position.set(shaftBotR + 0.1, shaftBotY + 0.28, 0);
      ribGrp.add(ribFoot);

      // Angled structural corbel bracket flaring outward from upper shaft (y=15.4) to support the wide Observation Deck (y=18.0)!
      const corbel = new THREE.Mesh(new THREE.BoxGeometry(0.38, 3.2, 0.34), darkMetalMat);
      corbel.position.set(4.15, 16.75, 0);
      corbel.rotation.z = -0.62;
      corbel.castShadow = true;
      ribGrp.add(corbel);

      const corbelWeb = new THREE.Mesh(new THREE.BoxGeometry(0.22, 2.5, 0.22), ribConcreteMat);
      corbelWeb.position.set(3.9, 16.95, 0);
      corbelWeb.rotation.z = -0.45;
      ribGrp.add(corbelWeb);

      towerGroup.add(ribGrp);
    }

    // 3. Two Taper-Aligned Framed Glass Elevator / Stairwell Shafts (East +X runway side & West -X side)
    [0, Math.PI].forEach((ang) => {
      const elevGrp = new THREE.Group();
      elevGrp.rotation.y = ang;

      const shaftLen = 11.4;
      const elevCenterY = 9.4; // spans y = 3.7 .. 15.1 (cleanly below ATC-01 signage plaque!)
      const elevR = shaftRAt(elevCenterY);

      // Structural dark steel housing tilted at EXACT taperRad so it stays 100% flush from bottom to top!
      const elevFrame = new THREE.Group();
      elevFrame.position.set(elevR + 0.08, elevCenterY, 0);
      elevFrame.rotation.z = taperRad;

      const backCase = new THREE.Mesh(new THREE.BoxGeometry(0.34, shaftLen, 1.38), darkMetalMat);
      backCase.castShadow = true;
      const glassStrip = new THREE.Mesh(new THREE.BoxGeometry(0.14, shaftLen - 0.3, 1.06), darkGlassMat);
      glassStrip.position.x = 0.13;
      const topHood = new THREE.Mesh(new THREE.BoxGeometry(0.46, 0.32, 1.48), darkMetalMat);
      topHood.position.set(0.06, shaftLen / 2 + 0.1, 0);
      elevFrame.add(backCase, glassStrip, topHood);

      // Left & right steel vertical mullions + 7 horizontal floor transom bars across the glass
      [-0.58, 0.58].forEach((mz) => {
        const sideRail = new THREE.Mesh(new THREE.BoxGeometry(0.26, shaftLen, 0.12), steelLightMat);
        sideRail.position.set(0.12, 0, mz);
        elevFrame.add(sideRail);
      });
      for (let t = -2; t <= 2; t++) {
        const transom = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.14, 1.24), darkMetalMat);
        transom.position.set(0.13, t * 2.05, 0);
        elevFrame.add(transom);
      }
      // Subtle illuminated elevator cab window visible mid-shaft between transoms!
      const elevCabGlow = new THREE.Mesh(new THREE.BoxGeometry(0.06, 1.35, 0.9), new THREE.MeshBasicMaterial({ color: 0xfef3c7 }));
      elevCabGlow.position.set(0.19, 1.02, 0);
      elevFrame.add(elevCabGlow);

      elevGrp.add(elevFrame);
      towerGroup.add(elevGrp);
    });

    // 4. North (+Z) Equipment-Floor Windows & South (-Z) Caged Maintenance Ladder + HVAC Louver Grilles
    [Math.PI / 2, -Math.PI / 2].forEach((ang, idx) => {
      const sideGrp = new THREE.Group();
      sideGrp.rotation.y = ang;

      if (idx === 0) {
        // North (+Z) side: 4 levels of framed recessed horizontal equipment-room windows matching shaft taper
        [5.3, 8.7, 12.5, 16.0].forEach((wy) => {
          const rw = shaftRAt(wy);
          const wGrp = new THREE.Group();
          wGrp.position.set(rw + 0.04, wy, 0);
          wGrp.rotation.z = taperRad;
          const wFrame = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.92, 1.75), darkMetalMat);
          const wPane = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.68, 1.52), darkGlassMat);
          const wMidBar = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.72, 0.08), steelLightMat);
          wGrp.add(wFrame, wPane, wMidBar);
          sideGrp.add(wGrp);
        });
      } else {
        // South (-Z) service face: centered caged steel maintenance ladder + flanking dark HVAC louver vents
        const ladGrp = new THREE.Group();
        ladGrp.position.set(shaftMidR + 0.26, shaftMidY - 0.2, 0);
        ladGrp.rotation.z = taperRad;
        [-0.24, 0.24].forEach((lz) => {
          const lRail = new THREE.Mesh(new THREE.BoxGeometry(0.06, 13.8, 0.06), darkMetalMat);
          lRail.position.z = lz;
          ladGrp.add(lRail);
        });
        for (let r = -6.4; r <= 6.4; r += 0.55) {
          const rung = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.48), steelLightMat);
          rung.position.y = r;
          ladGrp.add(rung);
        }
        // Safety ladder hoops every 2.2m on upper section
        for (let h = -2.2; h <= 5.5; h += 2.2) {
          const hoop = new THREE.Mesh(new THREE.TorusGeometry(0.36, 0.032, 6, 12, Math.PI), darkMetalMat);
          hoop.rotation.x = Math.PI / 2;
          hoop.rotation.z = -Math.PI / 2;
          hoop.position.set(0.18, h, 0);
          ladGrp.add(hoop);
        }
        sideGrp.add(ladGrp);

        // Industrial HVAC air-intake louver boxes flanking the ladder
        [5.4, 12.6].forEach((vy) => {
          const rv = shaftRAt(vy);
          [-0.82, 0.82].forEach((vz) => {
            const vent = new THREE.Mesh(new THREE.BoxGeometry(0.22, 1.05, 0.62), darkMetalMat);
            vent.position.set(rv + 0.05, vy, vz);
            vent.rotation.z = taperRad;
            sideGrp.add(vent);
          });
        });
      }

      // External vertical steel cable conduit pipes on both sides
      [-1.12, 1.12].forEach((cz) => {
        const pipe = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 13.6, 8), steelLightMat);
        pipe.position.set(shaftMidR + 0.12, shaftMidY - 0.1, cz);
        pipe.rotation.z = taperRad;
        sideGrp.add(pipe);
      });

      // Red mid-tower aviation obstruction warning lamp on cantilever bracket at y = 10.8m
      const bArm = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.08, 0.08), darkMetalMat);
      bArm.position.set(shaftRAt(10.8) + 0.35, 10.8, 0.82);
      const bLamp = new THREE.Mesh(new THREE.SphereGeometry(0.16, 8, 8), beaconRedMat);
      bLamp.position.set(shaftRAt(10.8) + 0.68, 10.95, 0.82);
      sideGrp.add(bArm, bLamp);

      towerGroup.add(sideGrp);
    });

    // 5. Crisp "ATC-01" Aviation Identifier Plaque mounted cleanly above elevator bay on Runway-facing (+X) shaft
    const atcSignMat = new THREE.MeshStandardMaterial({ map: makeAtcSignTex(), roughness: 0.45, metalness: 0.25 });
    const atcSign = new THREE.Mesh(new THREE.BoxGeometry(0.16, 1.05, 2.05), atcSignMat);
    atcSign.position.set(shaftRAt(16.2) + 0.18, 16.2, 0);
    atcSign.rotation.z = taperRad;
    towerGroup.add(atcSign);

    // Observation deck collar + orange aviation stripe + grounded perimeter safety railing
    const deck = new THREE.Mesh(new THREE.CylinderGeometry(6.05, 4.55, 1.1, 16), darkMetalMat);
    deck.position.y = 18.2;
    deck.castShadow = true;
    deck.receiveShadow = true;
    towerGroup.add(deck);

    const deckStripe = new THREE.Mesh(new THREE.CylinderGeometry(6.12, 5.82, 0.32, 16), towerAccentMat);
    deckStripe.position.y = 18.48;
    towerGroup.add(deckStripe);

    // Perimeter top handrail + mid-rail + 16 grounded vertical steel posts (ZERO floating rail!)
    const railRing = new THREE.Mesh(new THREE.TorusGeometry(5.88, 0.06, 8, 32), steelLightMat);
    railRing.rotation.x = Math.PI / 2;
    railRing.position.y = 19.62;
    const midRailRing = new THREE.Mesh(new THREE.TorusGeometry(5.88, 0.038, 6, 32), steelLightMat);
    midRailRing.rotation.x = Math.PI / 2;
    midRailRing.position.y = 19.16;
    towerGroup.add(railRing, midRailRing);

    for (let p = 0; p < 16; p++) {
      const pa = (p / 16) * Math.PI * 2;
      const rPost = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.96, 6), steelLightMat);
      rPost.position.set(Math.cos(pa) * 5.88, 19.16, Math.sin(pa) * 5.88);
      towerGroup.add(rPost);
    }

    // Reflective angled 360-degree glass control room + heavy dark steel window mullions + interior console ring
    const cabin = new THREE.Mesh(new THREE.CylinderGeometry(5.45, 4.72, 3.55, 16), glassMat);
    cabin.position.y = 20.42;
    cabin.castShadow = true;
    towerGroup.add(cabin);

    for (let m = 0; m < 16; m++) {
      const a = (m / 16) * Math.PI * 2;
      const mullion = new THREE.Mesh(new THREE.BoxGeometry(0.15, 3.65, 0.15), darkMetalMat);
      mullion.position.set(Math.cos(a) * 5.1, 20.42, Math.sin(a) * 5.1);
      mullion.rotation.y = -a;
      mullion.rotation.z = -0.2;
      towerGroup.add(mullion);
    }

    // =========================================================================
    // HIGH-DETAIL ATC ROOFTOP EQUIPMENT DECK (Bitumen Deck, Parapet Railing, Penthouse, Parabolic Lattice ASR Radar, Geodesic Radome, HVAC & Red/White Comms Mast!)
    // =========================================================================
    const makeAtcRoofTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#334155';
      ctx.fillRect(0, 0, 256, 256);
      // Subtle gravel/EPDM roof texture speckles
      for (let i = 0; i < 650; i++) {
        const rx = (i * 67 + (i % 11) * 19) % 256;
        const ry = (i * 43 + (i % 13) * 17) % 256;
        ctx.fillStyle = (i % 2 === 0) ? 'rgba(255,255,255,0.06)' : 'rgba(15,23,42,0.16)';
        ctx.fillRect(rx, ry, 2, 2);
      }
      // Radial roof drainage seams
      ctx.strokeStyle = 'rgba(15, 23, 42, 0.35)';
      ctx.lineWidth = 2;
      for (let k = 0; k < 8; k++) {
        const a = (k / 8) * Math.PI * 2;
        ctx.beginPath();
        ctx.moveTo(128, 128);
        ctx.lineTo(128 + Math.cos(a) * 128, 128 + Math.sin(a) * 128);
        ctx.stroke();
      }
      // Crisp yellow safety perimeter walkway ring & inner equipment boundary
      ctx.strokeStyle = '#facc15';
      ctx.lineWidth = 4;
      ctx.setLineDash([12, 8]);
      ctx.beginPath();
      ctx.arc(128, 128, 108, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(128, 128, 34, 0, Math.PI * 2);
      ctx.stroke();
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const atcRedMat = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.45, metalness: 0.25, side: THREE.DoubleSide });
    const atcWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.42, metalness: 0.15, side: THREE.DoubleSide });
    const atcRadomeMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.55, metalness: 0.08, flatShading: true });
    const atcYellowMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.45 });

    // Overhanging roof structural slab + textured dark bitumen deck membrane + hollow perimeter parapet curb
    const roof = new THREE.Mesh(new THREE.CylinderGeometry(6.15, 5.75, 0.78, 24), darkMetalMat);
    roof.position.y = 22.52;
    roof.castShadow = true;
    roof.receiveShadow = true;
    const roofMembrane = new THREE.Mesh(
      new THREE.CircleGeometry(5.98, 24),
      new THREE.MeshStandardMaterial({ map: makeAtcRoofTex(), roughness: 0.88 })
    );
    roofMembrane.rotation.x = -Math.PI / 2;
    roofMembrane.position.y = 22.92;
    roofMembrane.receiveShadow = true;
    const roofParapet = new THREE.Mesh(new THREE.TorusGeometry(6.02, 0.16, 6, 24), concreteMat);
    roofParapet.rotation.x = Math.PI / 2;
    roofParapet.position.y = 23.02;
    towerGroup.add(roof, roofMembrane, roofParapet);

    // Grounded Rooftop Maintenance Safety Handrail around perimeter (top rail, mid rail & 16 posts!)
    const roofRailTop = new THREE.Mesh(new THREE.TorusGeometry(5.9, 0.045, 6, 32), steelLightMat);
    roofRailTop.rotation.x = Math.PI / 2;
    roofRailTop.position.y = 23.82;
    const roofRailMid = new THREE.Mesh(new THREE.TorusGeometry(5.9, 0.032, 6, 32), steelLightMat);
    roofRailMid.rotation.x = Math.PI / 2;
    roofRailMid.position.y = 23.44;
    towerGroup.add(roofRailTop, roofRailMid);
    for (let rp = 0; rp < 16; rp++) {
      const rpa = (rp / 16) * Math.PI * 2;
      const post = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 0.84, 6), steelLightMat);
      post.position.set(Math.cos(rpa) * 5.9, 23.42, Math.sin(rpa) * 5.9);
      towerGroup.add(post);
    }

    // 1. Central Stepped Radar Pedestal + Rotating Concave Parabolic Lattice ASR Radar & IFF Bar!
    const pedBase = new THREE.Mesh(new THREE.CylinderGeometry(1.18, 1.38, 0.72, 12), concreteMat);
    pedBase.position.y = 23.28;
    pedBase.castShadow = true;
    const pedStripe = new THREE.Mesh(new THREE.CylinderGeometry(1.22, 1.24, 0.16, 12), atcYellowMat);
    pedStripe.position.y = 23.58;
    const pedTurret = new THREE.Mesh(new THREE.CylinderGeometry(0.68, 0.88, 0.76, 12), darkMetalMat);
    pedTurret.position.y = 24.02;
    pedTurret.castShadow = true;
    const pedGearbox = new THREE.Mesh(new THREE.BoxGeometry(0.92, 0.56, 0.92), steelLightMat);
    pedGearbox.position.y = 24.6;
    towerGroup.add(pedBase, pedStripe, pedTurret, pedGearbox);

    // Rotating High-Detail Airport Surveillance Radar (ASR) Parabolic Lattice Reflector
    this.radarDish = new THREE.Group();
    this.radarDish.position.set(0, 25.15, 0);
    const radarMastHub = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.28, 0.75, 10), darkMetalMat);
    radarMastHub.position.y = -0.25;
    this.radarDish.add(radarMastHub);

    // Concave parabolic steel lattice reflector built from 5 curved horizontal ribs + 9 vertical lattice ribs!
    const dishHalfW = 2.45;
    const dishH = 1.38;
    const segs = 8;
    for (let row = -2; row <= 2; row++) {
      const ry = row * (dishH / 4.2);
      for (let s = 0; s < segs; s++) {
        const x1 = -dishHalfW + (s / segs) * dishHalfW * 2;
        const x2 = -dishHalfW + ((s + 1) / segs) * dishHalfW * 2;
        const z1 = -0.22 + Math.pow(x1 / dishHalfW, 2) * 0.52;
        const z2 = -0.22 + Math.pow(x2 / dishHalfW, 2) * 0.52;
        const segLen = Math.hypot(x2 - x1, z2 - z1);
        const segAng = Math.atan2(z2 - z1, x2 - x1);
        const isTip = (s === 0 || s === segs - 1);
        const rib = new THREE.Mesh(new THREE.BoxGeometry(segLen + 0.02, (Math.abs(row) === 2) ? 0.11 : 0.065, 0.07), isTip ? towerAccentMat : atcWhiteMat);
        rib.position.set((x1 + x2) / 2, ry, (z1 + z2) / 2);
        rib.rotation.y = -segAng;
        rib.castShadow = true;
        this.radarDish.add(rib);
      }
    }
    for (let s = 0; s <= segs; s++) {
      const vx = -dishHalfW + (s / segs) * dishHalfW * 2;
      const vz = -0.22 + Math.pow(vx / dishHalfW, 2) * 0.52;
      const isEdge = (s === 0 || s === segs);
      const vRib = new THREE.Mesh(new THREE.BoxGeometry(isEdge ? 0.12 : 0.065, dishH + 0.06, 0.08), isEdge ? towerAccentMat : steelLightMat);
      vRib.position.set(vx, 0, vz);
      this.radarDish.add(vRib);
    }
    // Secondary IFF Interrogator Bar mounted cleanly above the parabolic reflector
    const iffBar = new THREE.Mesh(new THREE.BoxGeometry(3.2, 0.24, 0.16), atcWhiteMat);
    iffBar.position.set(0, 1.02, -0.15);
    const iffTipL = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.26, 0.18), towerAccentMat);
    iffTipL.position.set(-1.55, 1.02, -0.15);
    const iffTipR = iffTipL.clone();
    iffTipR.position.x = 1.55;
    [-0.95, 0.95].forEach((ix) => {
      const iPost = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.38, 0.07), darkMetalMat);
      iPost.position.set(ix, 0.82, -0.15);
      this.radarDish.add(iPost);
    });
    // Forward Waveguide Feed Horn Assembly projecting in front of the parabolic dish
    const feedBoom = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 1.55), darkMetalMat);
    feedBoom.position.set(0, -0.12, 0.58);
    const feedStrutL = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 1.48), steelLightMat);
    feedStrutL.position.set(-0.48, 0.28, 0.58);
    feedStrutL.rotation.y = 0.34;
    feedStrutL.rotation.x = 0.22;
    const feedStrutR = feedStrutL.clone();
    feedStrutR.position.x = 0.48;
    feedStrutR.rotation.y = -0.34;
    const feedHorn = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.26, 0.36), towerAccentMat);
    feedHorn.position.set(0, 0.02, 1.32);
    this.radarDish.add(iffBar, iffTipL, iffTipR, feedBoom, feedStrutL, feedStrutR, feedHorn);
    towerGroup.add(this.radarDish);

    // 2. Rooftop Access Stairwell Penthouse Cabin + Cable Conduit Trays (-2.4, 22.9, -2.0)
    const pentGrp = new THREE.Group();
    pentGrp.position.set(-2.45, 22.9, -2.05);
    pentGrp.rotation.y = 0.65;
    const pentBody = new THREE.Mesh(new THREE.BoxGeometry(2.25, 1.38, 1.95), concreteMat);
    pentBody.position.y = 0.69;
    pentBody.castShadow = true;
    const pentCap = new THREE.Mesh(new THREE.BoxGeometry(2.42, 0.16, 2.12), darkMetalMat);
    pentCap.position.y = 1.42;
    const pentStripe = new THREE.Mesh(new THREE.BoxGeometry(2.3, 0.18, 2.0), towerAccentMat);
    pentStripe.position.y = 1.24;
    const pentDoor = new THREE.Mesh(new THREE.BoxGeometry(0.85, 1.15, 0.1), darkMetalMat);
    pentDoor.position.set(0, 0.58, 0.98);
    const pentVent = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.65, 0.95), darkMetalMat);
    pentVent.position.set(1.13, 0.72, 0);
    pentGrp.add(pentBody, pentCap, pentStripe, pentDoor, pentVent);
    towerGroup.add(pentGrp);

    // Steel rooftop cable conduit trays connecting Penthouse -> Radar Pedestal -> Radome
    const cableTray1 = new THREE.Mesh(new THREE.BoxGeometry(2.4, 0.12, 0.26), darkMetalMat);
    cableTray1.position.set(-1.35, 22.98, -1.05);
    cableTray1.rotation.y = -0.68;
    const cableTray2 = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.12, 3.2), darkMetalMat);
    cableTray2.position.set(-2.35, 22.98, 0.15);
    towerGroup.add(cableTray1, cableTray2);

    // 3. Faceted White Geodesic Radar Dome (Radome) seated cleanly on an Octagonal Steel Equipment Pedestal!
    const radomeGrp = new THREE.Group();
    radomeGrp.position.set(-2.35, 22.9, 2.35);
    const radBase = new THREE.Mesh(new THREE.CylinderGeometry(1.06, 1.16, 0.68, 8), darkMetalMat);
    radBase.position.y = 0.34;
    radBase.castShadow = true;
    const radFlange = new THREE.Mesh(new THREE.CylinderGeometry(1.14, 1.14, 0.12, 8), steelLightMat);
    radFlange.position.y = 0.7;
    const radome = new THREE.Mesh(new THREE.IcosahedronGeometry(1.04, 2), atcRadomeMat);
    radome.position.y = 1.44;
    radome.castShadow = true;
    const radFinial = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.05, 0.85, 6), steelLightMat);
    radFinial.position.y = 2.75;
    radomeGrp.add(radBase, radFlange, radome, radFinial);
    towerGroup.add(radomeGrp);

    // 4. Dual Rooftop Industrial HVAC Chiller Units with Circular Exhaust Fan Cowls & Air Ducts (+2.1, 22.9, -2.35)
    const hvacGrp = new THREE.Group();
    hvacGrp.position.set(2.1, 22.9, -2.35);
    hvacGrp.rotation.y = -0.55;
    const hvacBox = new THREE.Mesh(new THREE.BoxGeometry(1.95, 0.88, 1.25), steelLightMat);
    hvacBox.position.y = 0.44;
    hvacBox.castShadow = true;
    [-0.48, 0.48].forEach((fx) => {
      const cowl = new THREE.Mesh(new THREE.CylinderGeometry(0.38, 0.4, 0.18, 12), darkMetalMat);
      cowl.position.set(fx, 0.94, 0);
      const blade1 = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.04, 0.1), steelLightMat);
      blade1.position.set(fx, 0.98, 0);
      const blade2 = blade1.clone();
      blade2.rotation.y = Math.PI / 2;
      hvacGrp.add(cowl, blade1, blade2);
    });
    const hvacDuct = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.36, 0.36), concreteMat);
    hvacDuct.position.set(-1.65, 0.38, 0.1);
    hvacGrp.add(hvacBox, hvacDuct);
    towerGroup.add(hvacGrp);

    // 5. Red-and-White Striped VHF Comms & Weather Mast + Dipole Crossbars + Microwave Drum + Anemometer + Caged Red Beacon!
    const mastGrp = new THREE.Group();
    mastGrp.position.set(2.95, 22.9, 1.95);
    const mastFoot = new THREE.Mesh(new THREE.CylinderGeometry(0.38, 0.52, 0.38, 8), darkMetalMat);
    mastFoot.position.y = 0.19;
    mastGrp.add(mastFoot);
    // 5 alternating red & white tubular aviation mast segments (height = 6.2m)
    for (let seg = 0; seg < 5; seg++) {
      const rB = 0.15 - seg * 0.018;
      const rT = 0.15 - (seg + 1) * 0.018;
      const mSeg = new THREE.Mesh(new THREE.CylinderGeometry(rT, rB, 1.24, 10), (seg % 2 === 0) ? atcRedMat : atcWhiteMat);
      mSeg.position.y = 0.32 + seg * 1.24 + 0.62;
      mSeg.castShadow = true;
      mastGrp.add(mSeg);
    }
    // 2 Horizontal VHF Dipole Cross-Arm Arrays + Vertical Whip Rods
    [2.6, 4.3].forEach((cy, ci) => {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(1.45, 0.06, 0.06), steelLightMat);
      arm.position.y = cy;
      arm.rotation.y = ci * (Math.PI / 2);
      mastGrp.add(arm);
      [-0.68, 0.68].forEach((wx) => {
        const whip = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, 0.95, 6), steelLightMat);
        whip.position.set(ci === 0 ? wx : 0, cy + 0.15, ci === 1 ? wx : 0);
        mastGrp.add(whip);
      });
    });
    // Compact White Microwave Relay Drum mounted on Mast
    const mwDrum = new THREE.Mesh(new THREE.CylinderGeometry(0.34, 0.32, 0.24, 12), atcWhiteMat);
    mwDrum.rotation.z = Math.PI / 2;
    mwDrum.position.set(0.32, 3.45, 0);
    const mwRim = new THREE.Mesh(new THREE.TorusGeometry(0.33, 0.035, 6, 12), atcRedMat);
    mwRim.rotation.y = Math.PI / 2;
    mwRim.position.set(0.44, 3.45, 0);
    // Weather Station 3-Cup Anemometer Arm
    const wxArm = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.04, 0.04), darkMetalMat);
    wxArm.position.set(-0.38, 5.2, 0);
    const wxHub = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.25, 6), steelLightMat);
    wxHub.position.set(-0.72, 5.32, 0);
    mastGrp.add(mwDrum, mwRim, wxArm, wxHub);

    // Caged Red Aviation Obstruction Strobe Beacon at Mast Summit
    const beaconBase = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.22, 0.18, 10), darkMetalMat);
    beaconBase.position.y = 6.56;
    this.towerLight = new THREE.Mesh(new THREE.SphereGeometry(0.24, 10, 10), beaconRedMat);
    this.towerLight.position.set(0, 6.82, 0);
    const beaconCage = new THREE.Mesh(new THREE.TorusGeometry(0.28, 0.022, 6, 14), darkMetalMat);
    beaconCage.position.y = 6.82;
    mastGrp.add(beaconBase, this.towerLight, beaconCage);
    towerGroup.add(mastGrp);

    // 6. Backup Telemetry Solar Panel & Secondary Whip Antenna near South Roof Rim
    const tSolarStand = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.55, 0.14), darkMetalMat);
    tSolarStand.position.set(0.35, 23.18, 4.15);
    const tSolar = new THREE.Mesh(
      new THREE.BoxGeometry(1.55, 0.08, 1.05),
      new THREE.MeshStandardMaterial({ color: 0x0f2d6e, roughness: 0.2, metalness: 0.75 })
    );
    tSolar.position.set(0.35, 23.52, 4.15);
    tSolar.rotation.x = 0.35;
    const secAnt = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.055, 3.6, 8), steelLightMat);
    secAnt.position.set(-3.8, 24.7, -0.2);
    towerGroup.add(tSolarStand, tSolar, secAnt);

    this.scene.add(towerGroup);
    this.addCylCollider(-16, -34, 5.65, 0, 3.6, 'atc_tower');     // Control tower lobby base
    this.addCylCollider(-16, -34, 4.45, 3.4, 17.8, 'atc_tower');  // Control tower shaft + buttresses & elevator core
    this.addCylCollider(-16, -34, 6.25, 17.6, 23.85, 'atc_tower'); // Control tower cabin, roof deck & rooftop parapet

    // Detailed ICAO/FAA Illuminated Airport Windsock Assembly (Segmented Ground Circle, Hinged Striped Mast, Swivel Throat Ring & Rippling 5-Stripe Hollow Sock!)
    const wsGroup = new THREE.Group();
    wsGroup.position.set(-16.5, 0, 10.5); // Spaciously clear West of Asphalt Collector Road (x = -13.3)!

    const wsOrangeMat = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.78, side: THREE.DoubleSide });
    const wsWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.72, side: THREE.DoubleSide });
    const wsSteelMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.5, metalness: 0.65 });
    const wsConcreteMat = new THREE.MeshStandardMaterial({ color: 0x7c848e, roughness: 0.92 });
    const wsYellowMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.48 });
    const wsLampMat = new THREE.MeshBasicMaterial({ color: 0xfef9c3 });
    const wsRedBeaconMat = new THREE.MeshBasicMaterial({ color: 0xff1e00 });

    // 1. ICAO White Segmented Ground Wind-Circle Marker (8 curved marker panels) + Concrete Pedestal & Hinged Mast Base!
    for (let s = 0; s < 8; s++) {
      const seg = new THREE.Mesh(
        new THREE.RingGeometry(1.85, 2.22, 6, 1, (s * Math.PI) / 4 + 0.08, Math.PI / 4 - 0.16),
        wsWhiteMat
      );
      seg.rotation.x = -Math.PI / 2;
      seg.position.y = 0.04;
      seg.receiveShadow = true;
      wsGroup.add(seg);
    }
    const wsBase = new THREE.Mesh(new THREE.CylinderGeometry(0.45, 0.56, 0.36, 8), wsConcreteMat);
    wsBase.position.y = 0.18; wsBase.castShadow = true; wsBase.receiveShadow = true;
    const wsFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.28, 0.14, 10), wsSteelMat);
    wsFlange.position.y = 0.42;
    const wsHinge = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.24, 0.22), wsSteelMat);
    wsHinge.position.set(0, 0.62, 0);
    // 4 Low yellow safety bollards protecting the concrete base
    [[-0.85, -0.85], [0.85, -0.85], [-0.85, 0.85], [0.85, 0.85]].forEach(([bx, bz]) => {
      const bol = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.06, 0.62, 6), wsYellowMat);
      bol.position.set(bx, 0.31, bz); wsGroup.add(bol);
    });
    wsGroup.add(wsBase, wsFlange, wsHinge);

    // 2. Red-and-White Aviation Striped Tapered Steel Mast + Overhead Floodlight Hoods + Top Red Aviation Beacon!
    const mastBands = 6;
    const mastH = 6.2;
    const bandH = mastH / mastBands;
    for (let b = 0; b < mastBands; b++) {
      const y0 = 0.5 + b * bandH;
      const rBot = 0.11 - (b / mastBands) * 0.04;
      const rTop = 0.11 - ((b + 1) / mastBands) * 0.04;
      const seg = new THREE.Mesh(
        new THREE.CylinderGeometry(rTop, rBot, bandH + 0.01, 10),
        (b % 2 === 0) ? wsOrangeMat : wsWhiteMat
      );
      seg.position.y = y0 + bandH / 2;
      seg.castShadow = true;
      wsGroup.add(seg);
    }
    // Overhead 4-Arm Airport Windsock Illumination Rig + Top Red Obstruction Light (y = 6.85..7.35m)
    const lightHub = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.25, 8), wsSteelMat);
    lightHub.position.y = 6.82;
    const topPin = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.05, 0.42, 6), wsSteelMat);
    topPin.position.y = 7.12;
    const topBeacon = new THREE.Mesh(new THREE.SphereGeometry(0.11, 8, 8), wsRedBeaconMat);
    topBeacon.position.y = 7.36;
    wsGroup.add(lightHub, topPin, topBeacon);
    for (let k = 0; k < 4; k++) {
      const ang = (k * Math.PI) / 2 + Math.PI / 4;
      const armGrp = new THREE.Group();
      armGrp.position.y = 6.88; armGrp.rotation.y = ang;
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.04, 0.04), wsSteelMat);
      arm.position.x = 0.28;
      const shade = new THREE.Mesh(new THREE.ConeGeometry(0.13, 0.14, 8), wsWhiteMat);
      shade.position.set(0.54, -0.06, 0);
      const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.065, 6, 6), wsLampMat);
      bulb.position.set(0.54, -0.12, 0);
      armGrp.add(arm, shade, bulb);
      wsGroup.add(armGrp);
    }

    // 3. Properly Pivoted Swivel Bearing + Steel Throat Frame + Articulated 5-Stripe Hollow Open-Ended Windsock!
    this.windsock = new THREE.Group();
    this.windsock.position.set(0, 6.22, 0); // Centered directly on mast axis (ZERO pole skewering!)
    const swivelCollar = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.34, 10), wsSteelMat);
    const swivelArm = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.42), wsSteelMat);
    swivelArm.position.set(0, 0, 0.22);
    this.windsock.add(swivelCollar, swivelArm);

    // Rigid circular steel throat hoop + 3 steel support struts locked solidly to the swivel collar & arm!
    const throatRing = new THREE.Mesh(new THREE.TorusGeometry(0.42, 0.032, 8, 18), wsSteelMat);
    throatRing.position.set(0, 0, 0.42);
    for (let k = 0; k < 3; k++) {
      const a = (k * Math.PI * 2) / 3 + Math.PI / 6;
      const dir = new THREE.Vector3(Math.cos(a) * 0.42, Math.sin(a) * 0.42, 0.42);
      const origin = new THREE.Vector3(0, 0, 0.06);
      const v = new THREE.Vector3().subVectors(dir, origin);
      const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, v.length(), 5), wsSteelMat);
      strut.position.copy(origin).add(dir).multiplyScalar(0.5);
      strut.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v.normalize());
      this.windsock.add(strut);
    }
    this.windsock.add(throatRing);

    // Flexible hollow fabric pitch hinge right at the mouth hoop (droops in light breeze, stretches taut in strong wind!)
    this.windsockPitch = new THREE.Group();
    this.windsockPitch.position.set(0, 0, 0.42);

    // Helper: hollow open-ended truncated cone extending cleanly along +Z from z = 0 to z = len
    const makeSockBand = (rFront, rBack, len, zOff, mat) => {
      const geo = new THREE.CylinderGeometry(rBack, rFront, len, 16, 1, true);
      geo.rotateX(Math.PI / 2); // +Y -> +Z (rBack at +len/2, rFront at -len/2)
      const m = new THREE.Mesh(geo, mat);
      m.position.z = zOff + len / 2;
      m.castShadow = true;
      return m;
    };

    // Segment 1 (Bands 1 & 2: Orange + White attached right to throat hoop!)
    this.windsockPitch.add(makeSockBand(0.42, 0.365, 0.52, 0.0, wsOrangeMat));
    this.windsockPitch.add(makeSockBand(0.365, 0.31, 0.52, 0.52, wsWhiteMat));

    // Segment 2 (Bands 3 & 4: Orange + White articulated mid-sleeve for organic billow ripple!)
    this.windsockMid = new THREE.Group();
    this.windsockMid.position.z = 1.04;
    this.windsockMid.add(makeSockBand(0.31, 0.255, 0.52, 0.0, wsOrangeMat));
    this.windsockMid.add(makeSockBand(0.255, 0.20, 0.52, 0.52, wsWhiteMat));
    this.windsockPitch.add(this.windsockMid);

    // Segment 3 (Band 5: Open-ended International Orange tail vent fluttering freely!)
    this.windsockTail = new THREE.Group();
    this.windsockTail.position.z = 1.04;
    this.windsockTail.add(makeSockBand(0.20, 0.145, 0.54, 0.0, wsOrangeMat));
    this.windsockMid.add(this.windsockTail);

    this.windsock.add(this.windsockPitch);
    wsGroup.add(this.windsock);

    this.scene.add(wsGroup);
    this.addCylCollider(-16.5, 10.5, 0.22, 0, 7.45, 'power_pole');
  }

  buildHelipad() {
    this.helipad = new THREE.Group();
    this.helipad.position.set(0, 0.05, 0);

    // Dark round platform
    const padGeo = new THREE.CylinderGeometry(6, 6.2, 0.1, 32);
    const padMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.6 });
    const pad = new THREE.Mesh(padGeo, padMat);
    pad.receiveShadow = true;
    this.helipad.add(pad);

    // Outer Glowing Cyan Ring
    const ringGeo = new THREE.RingGeometry(5.2, 5.6, 36);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.06;
    this.helipad.add(ring);

    // Inner Ring
    const iRingGeo = new THREE.RingGeometry(3.6, 3.8, 32);
    const iRing = new THREE.Mesh(iRingGeo, ringMat);
    iRing.rotation.x = -Math.PI / 2;
    iRing.position.y = 0.06;
    this.helipad.add(iRing);

    // Giant White 'H'
    const hMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    // Left stem
    const lStem = new THREE.Mesh(new THREE.PlaneGeometry(0.7, 3.4), hMat);
    lStem.rotation.x = -Math.PI / 2;
    lStem.position.set(-1.1, 0.07, 0);
    this.helipad.add(lStem);

    // Right stem
    const rStem = new THREE.Mesh(new THREE.PlaneGeometry(0.7, 3.4), hMat);
    rStem.rotation.x = -Math.PI / 2;
    rStem.position.set(1.1, 0.07, 0);
    this.helipad.add(rStem);

    // Crossbar
    const cBar = new THREE.Mesh(new THREE.PlaneGeometry(1.6, 0.7), hMat);
    cBar.rotation.x = -Math.PI / 2;
    cBar.position.set(0, 0.07, 0);
    this.helipad.add(cBar);

    this.scene.add(this.helipad);
  }

  buildTrees() {
    // High-Detail Multi-Layered GPU InstancedMesh Forest (4-Tier Pines + 3-Cluster Oaks + Autumn Accent Trees!)
    const trunkMat = new THREE.MeshStandardMaterial({ color: 0x452712, roughness: 0.94, metalness: 0.0 });
    const pineL1Mat = new THREE.MeshStandardMaterial({ color: 0x0c301a, roughness: 0.88, flatShading: true });
    const pineL2Mat = new THREE.MeshStandardMaterial({ color: 0x124023, roughness: 0.86, flatShading: true });
    const pineL3Mat = new THREE.MeshStandardMaterial({ color: 0x19522d, roughness: 0.84, flatShading: true });
    const pineL4Mat = new THREE.MeshStandardMaterial({ color: 0x226638, roughness: 0.82, flatShading: true });
    const oakDarkMat = new THREE.MeshStandardMaterial({ color: 0x1c4a21, roughness: 0.86, flatShading: true });
    const oakSunMat = new THREE.MeshStandardMaterial({ color: 0x2f6829, roughness: 0.84, flatShading: true });
    const autumnMat = new THREE.MeshStandardMaterial({ color: 0xb4691e, roughness: 0.85, flatShading: true });
    const shadowDiscMat = new THREE.MeshBasicMaterial({ color: 0x081409, transparent: true, opacity: 0.34, depthWrite: false });

    // Helper: point-to-segment distance so NO tree ever spawns on top of any road, lane, gravel track or footpath!
    const distToSeg = (px, pz, x1, z1, x2, z2) => {
      const dx = x2 - x1, dz = z2 - z1;
      const lenSq = dx * dx + dz * dz;
      const t = lenSq > 0 ? Math.max(0, Math.min(1, ((px - x1) * dx + (pz - z1) * dz) / lenSq)) : 0;
      return Math.hypot(px - (x1 + t * dx), pz - (z1 + t * dz));
    };

    const roadClearSegments = [
      // Main Asphalt Collector Loop & Solar Branch
      [54, 5, 54, 12], [54, 12, 134, 12], [134, 12, 134, 78], [134, 78, -2, 78],
      [-2, 78, -2, 46], [-2, 46, -10.5, 36], [-10.5, 36, -10.5, -26.85],
      [136.8, 68, 148, 68], [148, 68, 148, 118], [148, 118, 154, 118],
      // Cobblestone Village Streets
      [71, 14.8, 71, 75.2], [73.5, 31, 131.2, 31], [73.5, 62, 131.2, 62],
      // Gravel Farm, 5G Tower, Ranger & Wind-Farm Service Roads
      [66, -29, 66, -54], [66, -54, 75, -65],
      [10.0, 80.8, 10.0, 85.5], [10.0, 85.5, -21.0, 85.5], [-21.0, 85.5, -26, 102], [-26, 102, -76, 102], [-76, 102, -76, 46], [-76, 46, -48, 46], [-48, 46, -48, 22], [-48, 22, -13.3, 22],
      [-40.5, 20.1, -40.5, -12.5],
      [-77.9, 86, -84, 86], [-84, 86, -110, 108], [-110, 108, -136, 124],
      [-38, 103.9, -42, 165], [-42, 165, -95, 214], [-95, 214, 0, 224], [0, 224, 95, 214], [95, 214, 138, 170], [138, 170, 152.2, 136], [152.2, 136, 152.2, 120.6],
      // Scenic Lakeside, Grand Strait Arched Bridge, Marina Pier, Lighthouse & Remote Ancient Ruins Footpaths
      [-16, -42.5, -16, -45.5], [-16, -45.5, -32.7, -45.5], [-32.7, -45.5, -32.7, -69.6],
      [-33.75, -49.0, -54.0, -49.0], [-54.0, -49.0, -54.0, -52.6],
      [-38.3, -100.4, -40, -110], [-40, -110, -60, -117], [-60, -117, -88, -115], [-88, -115, -101, -111], [-101, -111, -128, -122], [-128, -122, -168, -136.5]
    ];

    // Clean clearance around all buildings, spacious town blocks, farms, expanded twin-basin lake, turbines, roads & landmarks
    const isClearSpot = (x, z) => {
      if (Math.hypot(x, z) < 16) return false; // helipad
      if (Math.hypot(x + 16, z + 34) < 13) return false; // control tower plaza
      if (Math.hypot(x - 75, z + 70) < 14) return false; // 5G telecom tower compound
      if (Math.hypot(x + 95, z - 224) < 18 || Math.hypot(x, z - 234) < 18 || Math.hypot(x - 95, z - 224) < 18) return false; // wind turbines
      if (Math.hypot(x + 168, z + 148) < 25) return false; // ancient temple ruins
      if (Math.hypot(x + 145, z - 125) < 22) return false; // forest campsite & watchtower
      if (Math.hypot(x - 172, z - 118) < 27) return false; // solar power farm
      if (Math.hypot(x - 135, z - 124) < 14) return false; // green energy grid control house
      if (Math.hypot(x + 108, z + 110) < 18) return false; // lakeside lighthouse & cottage
      if (Math.hypot(x + 24, z + 56) < 11) return false; // lakeside boathouse & kayak cabin
      if (Math.hypot(x - 82, z - 6.4) < 9) return false; // airfield-village transit shelter
      if (Math.hypot(x + 10, z - 92) < 12) return false; // historic country grain windmill
      if (Math.hypot(x + 65, z - 86) < 13) return false; // valley country farmhouse & chicken coop
      if (Math.hypot(x - 30, z - 88) < 12) return false; // glass botanical greenhouse
      if (x > 2 && x < 84 && z > -90 && z < 50) return false; // runway + taxiway + hangar + apron
      if (x > -114 && x < 4 && z > -124 && z < -44) return false; // expanded twin-basin alpine lake & east lagoon shore
      if (x > -98 && x < -10 && z > -58 && z < 104) return false; // farm fields + barn + homestead
      if (x > 14 && x < 140 && z > 8 && z < 84) return false; // south field + spacious expanded village town
      for (let i = 0; i < roadClearSegments.length; i++) {
        const s = roadClearSegments[i];
        if (distToSeg(x, z, s[0], s[1], s[2], s[3]) < 6.8) return false;
      }
      return true;
    };

    const pineSpots = [];
    const oakSpots = [];
    const autumnSpots = [];
    let seed = 42;
    const rand = () => {
      seed = (seed * 1664525 + 1013904223) % 4294967296;
      return seed / 4294967296;
    };

    const scenic = [
      [-22, -10], [-26, -2], [-22, 15], [-29, 28], [-20, 34],
      [-36, 7], [-50, -35], [53, -52], [86, -24], [95, 2], [146, 34]
    ];
    scenic.forEach(([x, z], i) => {
      if (!isClearSpot(x, z)) return;
      const s = 0.92 + (i % 3) * 0.16;
      (i % 2 === 0 ? pineSpots : oakSpots).push({ x, z, s, ry: i * 1.1 });
    });

    for (let attempt = 0; attempt < 950 && pineSpots.length + oakSpots.length + autumnSpots.length < 255; attempt++) {
      const angle = rand() * Math.PI * 2;
      const dist = 24 + rand() * 242;
      const x = Math.cos(angle) * dist;
      const z = Math.sin(angle) * dist;
      if (!isClearSpot(x, z)) continue;
      const s = 0.82 + rand() * 0.78;
      const ry = rand() * Math.PI * 2;
      const roll = rand();
      if (roll < 0.60) pineSpots.push({ x, z, s, ry });
      else if (roll < 0.88) oakSpots.push({ x, z, s, ry });
      else autumnSpots.push({ x, z, s, ry });
    }

    const dummy = new THREE.Object3D();
    const aoGeo = new THREE.CircleGeometry(2.8, 12);
    aoGeo.rotateX(-Math.PI / 2);

    // 1. 4-Tiered Detailed Alpine Pines (flared trunk root + 4 staggered overlapping branch tiers + contact shadow)
    if (pineSpots.length) {
      const pTrunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.20, 0.52, 3.2, 7), trunkMat, pineSpots.length);
      const pL1 = new THREE.InstancedMesh(new THREE.ConeGeometry(2.9, 3.4, 8), pineL1Mat, pineSpots.length);
      const pL2 = new THREE.InstancedMesh(new THREE.ConeGeometry(2.35, 3.2, 8), pineL2Mat, pineSpots.length);
      const pL3 = new THREE.InstancedMesh(new THREE.ConeGeometry(1.75, 3.0, 8), pineL3Mat, pineSpots.length);
      const pL4 = new THREE.InstancedMesh(new THREE.ConeGeometry(1.12, 2.6, 8), pineL4Mat, pineSpots.length);
      const pAO = new THREE.InstancedMesh(aoGeo, shadowDiscMat, pineSpots.length);
      pL1.castShadow = true; // Base pine canopy casts shadow; upper tiers skip redundant shadow triangles (3x faster!)
      pL1.receiveShadow = true;

      pineSpots.forEach((t, i) => {
        const y0 = this.getTerrainHeight(t.x, t.z);
        dummy.position.set(t.x, y0 + 0.05, t.z);
        dummy.rotation.set(0, 0, 0);
        dummy.scale.setScalar(t.s);
        dummy.updateMatrix();
        pAO.setMatrixAt(i, dummy.matrix);

        dummy.position.set(t.x, y0 + 1.6 * t.s, t.z);
        dummy.rotation.set(0, t.ry, 0);
        dummy.updateMatrix();
        pTrunks.setMatrixAt(i, dummy.matrix);

        dummy.position.set(t.x, y0 + 3.4 * t.s, t.z);
        dummy.updateMatrix();
        pL1.setMatrixAt(i, dummy.matrix);

        dummy.position.set(t.x, y0 + 5.1 * t.s, t.z);
        dummy.rotation.set(0, t.ry + 0.4, 0);
        dummy.updateMatrix();
        pL2.setMatrixAt(i, dummy.matrix);

        dummy.position.set(t.x, y0 + 6.7 * t.s, t.z);
        dummy.rotation.set(0, t.ry + 0.85, 0);
        dummy.updateMatrix();
        pL3.setMatrixAt(i, dummy.matrix);

        dummy.position.set(t.x, y0 + 8.2 * t.s, t.z);
        dummy.rotation.set(0, t.ry + 1.25, 0);
        dummy.updateMatrix();
        pL4.setMatrixAt(i, dummy.matrix);

        this.addCylCollider(t.x, t.z, 1.6 * t.s, y0, y0 + 9.0 * t.s, 'tree');
      });
      this.scene.add(pAO, pTrunks, pL1, pL2, pL3, pL4);
    }

    // Helper: Instanced 3-Cluster Leafy Broadleaf Trees (forked trunk + 3 overlapping natural canopy puffs!)
    const buildBroadleafGroup = (spots, baseMat, topMat) => {
      if (!spots.length) return;
      const trunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.28, 0.56, 3.4, 7), trunkMat, spots.length);
      const branches = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.14, 0.22, 1.8, 5), trunkMat, spots.length);
      const puffMain = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(2.3, 0), baseMat, spots.length);
      const puffSide = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(1.75, 0), baseMat, spots.length);
      const puffTop = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(1.65, 0), topMat, spots.length);
      const ao = new THREE.InstancedMesh(aoGeo, shadowDiscMat, spots.length);
      puffMain.castShadow = true; // Main canopy casts shadow; side/top clusters skip redundant shadow pass!
      puffMain.receiveShadow = true;

      spots.forEach((t, i) => {
        const y0 = this.getTerrainHeight(t.x, t.z);
        const c = Math.cos(t.ry), sn = Math.sin(t.ry);

        dummy.position.set(t.x, y0 + 0.05, t.z);
        dummy.rotation.set(0, 0, 0);
        dummy.scale.setScalar(t.s);
        dummy.updateMatrix();
        ao.setMatrixAt(i, dummy.matrix);

        dummy.position.set(t.x, y0 + 1.7 * t.s, t.z);
        dummy.rotation.set(0, t.ry, 0);
        dummy.updateMatrix();
        trunks.setMatrixAt(i, dummy.matrix);

        // Angled limb branch reaching out to side cluster
        dummy.position.set(t.x + c * 0.65 * t.s, y0 + 3.0 * t.s, t.z + sn * 0.65 * t.s);
        dummy.rotation.set(0, -t.ry, -0.58);
        dummy.updateMatrix();
        branches.setMatrixAt(i, dummy.matrix);

        // Main lower-center canopy cluster
        dummy.position.set(t.x - c * 0.35 * t.s, y0 + 4.4 * t.s, t.z - sn * 0.35 * t.s);
        dummy.rotation.set(0, t.ry, 0);
        dummy.scale.set(t.s * 1.12, t.s * 0.85, t.s * 1.05);
        dummy.updateMatrix();
        puffMain.setMatrixAt(i, dummy.matrix);

        // Asymmetric side canopy cluster
        dummy.position.set(t.x + c * 1.35 * t.s, y0 + 4.15 * t.s, t.z + sn * 1.35 * t.s);
        dummy.scale.set(t.s * 0.98, t.s * 0.82, t.s * 0.95);
        dummy.updateMatrix();
        puffSide.setMatrixAt(i, dummy.matrix);

        // Sunlit upper crown puff
        dummy.position.set(t.x + c * 0.25 * t.s, y0 + 5.75 * t.s, t.z + sn * 0.25 * t.s);
        dummy.scale.set(t.s * 1.02, t.s * 0.82, t.s * 1.02);
        dummy.updateMatrix();
        puffTop.setMatrixAt(i, dummy.matrix);

        this.addCylCollider(t.x, t.z, 2.3 * t.s, y0, y0 + 7.0 * t.s, 'tree');
      });
      this.scene.add(ao, trunks, branches, puffMain, puffSide, puffTop);
    };

    buildBroadleafGroup(oakSpots, oakDarkMat, oakSunMat);
    buildBroadleafGroup(autumnSpots, autumnMat, new THREE.MeshStandardMaterial({ color: 0xd98a2b, roughness: 0.84, flatShading: true }));
  }

  createCropTexture(baseHex, stripeHex, furrowHex = null) {
    const c = document.createElement('canvas');
    c.width = 256;
    c.height = 256;
    const ctx = c.getContext('2d');
    ctx.fillStyle = baseHex;
    ctx.fillRect(0, 0, 256, 256);
    // Deep plowed furrow crop ridges with realistic sunlit crests & dark shaded troughs
    for (let y = 0; y < 256; y += 16) {
      ctx.fillStyle = stripeHex;
      ctx.fillRect(0, y, 256, 7);
      ctx.fillStyle = furrowHex || 'rgba(20, 14, 6, 0.36)';
      ctx.fillRect(0, y + 7, 256, 3);
      ctx.fillStyle = 'rgba(255, 255, 255, 0.06)';
      ctx.fillRect(0, y + 1, 256, 2);
    }
    // Organic tilled-loam soil grain & stubble speckles
    for (let i = 0; i < 950; i++) {
      const gx = (i * 67 + (i % 13) * 19) % 256;
      const gy = (i * 41 + (i % 17) * 11) % 256;
      ctx.fillStyle = (i % 3 === 0) ? 'rgba(255, 245, 220, 0.08)' : 'rgba(24, 16, 8, 0.16)';
      ctx.fillRect(gx, gy, 3, 2);
    }
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(4, 4);
    if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
    if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
    return tex;
  }

  buildAgriculturalFields() {
    // =========================================================================
    // 4 PRECISION SMART-AGRICULTURE PLOTS (ZONE A–D) DESIGNED FOR FUTURE FERTILIZER-SPRAYING DRONES!
    // Stores plot metadata in this.farmPlots so future spraying missions can track coverage & refill pads cleanly.
    // =========================================================================
    const fieldDefs = [
      {
        id: 'A', name: 'ZONE A · WHEAT', x: -56, z: -18, w: 30, d: 24,
        base: '#6f4818', stripe: '#9e702a', type: 'wheat',
        gateSide: 'E', gatePos: -14, refillPad: { x: -30.5, z: -15.5 }
      },
      {
        id: 'B', name: 'ZONE B · CABBAGE', x: -75, z: 25, w: 38, d: 34,
        base: '#3b2b18', stripe: '#523c22', type: 'cabbage',
        gateSide: 'E', gatePos: 30, refillPad: { x: -53.2, z: 24.0 }
      },
      {
        id: 'C', name: 'ZONE C · SWEET CORN', x: -26, z: 64, w: 30, d: 28,
        base: '#4a3219', stripe: '#664623', type: 'corn',
        gateSide: 'S', gatePos: -16, refillPad: null
      },
      {
        id: 'D', name: 'ZONE D · BERRY TRELLIS', x: 35, z: 56, w: 34, d: 28,
        base: '#362717', stripe: '#4d3821', type: 'trellis',
        gateSide: 'S', gatePos: 35, refillPad: { x: 26.5, z: 72.6 }
      }
    ];
    this.farmPlots = fieldDefs.map(f => ({
      id: f.id, name: f.name, x: f.x, z: f.z, w: f.w, d: f.d, cropType: f.type, refillPad: f.refillPad
    }));
    this.farmSpinners = [];

    // Shared Smart-Farm Materials
    const fenceMat = new THREE.MeshStandardMaterial({ color: 0x6b3a17, roughness: 0.92 });
    const postMat = new THREE.MeshStandardMaterial({ color: 0x522a10, roughness: 0.94 });
    const hayMat = new THREE.MeshStandardMaterial({ color: 0xd4a024, roughness: 0.92 });
    const soilRidgeMat = new THREE.MeshStandardMaterial({ color: 0x4b331c, roughness: 0.96 });
    const wheatGoldMat = new THREE.MeshStandardMaterial({ color: 0xd9a834, roughness: 0.88, flatShading: true });
    const wheatDryMat = new THREE.MeshStandardMaterial({ color: 0xc2b067, roughness: 0.9, flatShading: true }); // Pale nutrient-needed patch!
    const cabOuterMat = new THREE.MeshStandardMaterial({ color: 0x256322, roughness: 0.84, flatShading: true });
    const cabInnerMat = new THREE.MeshStandardMaterial({ color: 0x74b843, roughness: 0.78, flatShading: true });
    const cornStalkMat = new THREE.MeshStandardMaterial({ color: 0x2f7024, roughness: 0.86, flatShading: true });
    const cornTasselMat = new THREE.MeshStandardMaterial({ color: 0xeab308, roughness: 0.82, flatShading: true });
    const berryBushMat = new THREE.MeshStandardMaterial({ color: 0x1e5c26, roughness: 0.86, flatShading: true });
    const berryFruitMat = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.62 });
    const steelMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.42, metalness: 0.72 });
    const darkMetalMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.55, metalness: 0.55 });
    const whiteMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.72 });
    const orangeMat = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.55 });
    const fertTankMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, roughness: 0.25, metalness: 0.15, transparent: true, opacity: 0.82 });
    const fertGreenMat = new THREE.MeshStandardMaterial({ color: 0x22c55e, roughness: 0.35, metalness: 0.12, transparent: true, opacity: 0.84 });
    const solarCellMat = new THREE.MeshStandardMaterial({ color: 0x1e3a8a, roughness: 0.18, metalness: 0.8 });
    const beaconLedMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
    const npkBagMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.88 });
    const npkStripeMat = new THREE.MeshStandardMaterial({ color: 0x16a34a, roughness: 0.8 });
    const brassMat = new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.3, metalness: 0.8 });

    // Procedural Textures: Agri-Drone Refill Pad, Zone Signboard, Shed Planks & Stave Water Tank
    const makeSprayPadTex = (zoneLabel) => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#475569'; ctx.fillRect(0, 0, 256, 256);
      // Hazard green & yellow perimeter stripe for agricultural drone refill landing zone
      ctx.strokeStyle = '#facc15'; ctx.lineWidth = 14; ctx.strokeRect(12, 12, 232, 232);
      ctx.strokeStyle = '#22c55e'; ctx.lineWidth = 5;
      ctx.beginPath(); ctx.arc(128, 128, 76, 0, Math.PI * 2); ctx.stroke();
      // Bold "H" + green spray droplet emblem
      ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 74px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('F', 128, 114);
      ctx.fillStyle = '#86efac'; ctx.font = 'bold 22px sans-serif';
      ctx.fillText(zoneLabel, 128, 174);
      ctx.fillStyle = '#fde047'; ctx.font = 'bold 16px sans-serif';
      ctx.fillText('AGRI-DRONE REFILL', 128, 224);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeZoneSignTex = (title, sub) => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 96;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#14532d'; ctx.fillRect(0, 0, 256, 96);
      ctx.strokeStyle = '#facc15'; ctx.lineWidth = 6; ctx.strokeRect(4, 4, 248, 88);
      ctx.fillStyle = '#fefce8'; ctx.font = 'bold 30px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(title, 128, 36);
      ctx.fillStyle = '#86efac'; ctx.font = 'bold 17px sans-serif';
      ctx.fillText(sub, 128, 68);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeFarmWoodTex = (hex1, hex2) => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = hex1; ctx.fillRect(0, 0, 128, 128);
      for (let x = 0; x < 128; x += 16) {
        ctx.fillStyle = (x % 32 === 0) ? hex1 : hex2;
        ctx.fillRect(x + 1, 0, 14, 128);
        ctx.fillStyle = 'rgba(28, 16, 6, 0.4)';
        ctx.fillRect(x, 0, 2, 128);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Helper: Reusable IBC Liquid Fertilizer Tank in Steel Cage + Brass Outlet Valve
    const addIbcTank = (parent, x, z, liquidMat = fertTankMat, rotY = 0) => {
      const tg = new THREE.Group();
      tg.position.set(x, 0, z); tg.rotation.y = rotY;
      const pallet = new THREE.Mesh(new THREE.BoxGeometry(1.18, 0.14, 1.05), fenceMat);
      pallet.position.y = 0.07;
      const tank = new THREE.Mesh(new THREE.BoxGeometry(1.02, 1.02, 0.9), liquidMat);
      tank.position.y = 0.65; tank.castShadow = true;
      const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.14, 0.09, 8), darkMetalMat);
      cap.position.y = 1.2;
      const valve = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.18, 6), brassMat);
      valve.rotation.x = Math.PI / 2; valve.position.set(0, 0.25, 0.52);
      // Tubular steel protective IBC cage bars
      [0.32, 0.68, 1.02].forEach((cy) => {
        const ringN = new THREE.Mesh(new THREE.BoxGeometry(1.08, 0.035, 0.035), steelMat);
        ringN.position.set(0, cy, -0.47);
        const ringS = ringN.clone(); ringS.position.z = 0.47;
        const ringW = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.035, 0.96), steelMat);
        ringW.position.set(-0.53, cy, 0);
        const ringE = ringW.clone(); ringE.position.x = 0.53;
        tg.add(ringN, ringS, ringW, ringE);
      });
      [[-0.53, -0.47], [0.53, -0.47], [-0.53, 0.47], [0.53, 0.47]].forEach(([cx, cz]) => {
        const bar = new THREE.Mesh(new THREE.BoxGeometry(0.035, 1.04, 0.035), steelMat);
        bar.position.set(cx, 0.66, cz); tg.add(bar);
      });
      tg.add(pallet, tank, cap, valve);
      parent.add(tg);
    };

    // Helper: Wooden Pallet Stacked with NPK Granular Fertilizer Sacks
    const addNpkPallet = (parent, x, z, rotY = 0) => {
      const pg = new THREE.Group();
      pg.position.set(x, 0, z); pg.rotation.y = rotY;
      const base = new THREE.Mesh(new THREE.BoxGeometry(1.25, 0.14, 1.05), fenceMat);
      base.position.y = 0.07; pg.add(base);
      for (let layer = 0; layer < 3; layer++) {
        [-0.28, 0.28].forEach((bx) => {
          [-0.22, 0.22].forEach((bz) => {
            if (layer === 2 && bx > 0 && bz > 0) return;
            const bag = new THREE.Mesh(new THREE.BoxGeometry(0.48, 0.16, 0.36), npkBagMat);
            bag.position.set(bx, 0.22 + layer * 0.17, bz);
            bag.rotation.y = ((layer % 2) * 0.08) - 0.04;
            bag.castShadow = true;
            const label = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.17, 0.18), npkStripeMat);
            label.position.set(bx, 0.22 + layer * 0.17, bz);
            pg.add(bag, label);
          });
        });
      }
      parent.add(pg);
    };

    // Helper: Solar-Powered Precision Agriculture Weather & Soil-Moisture Telemetry Mast (with Spinning Anemometer!)
    const addSmartSensorMast = (x, z) => {
      const mg = new THREE.Group();
      mg.position.set(x, 0, z);
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.07, 3.2, 8), steelMat);
      pole.position.y = 1.6; pole.castShadow = true;
      const box = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.52, 0.26), whiteMat);
      box.position.set(0, 1.55, 0.15);
      const sol = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.05, 0.45), solarCellMat);
      sol.position.set(0, 2.38, 0.24); sol.rotation.x = 0.55;
      const led = new THREE.Mesh(new THREE.SphereGeometry(0.06, 6, 6), beaconLedMat);
      led.position.set(0, 3.24, 0);
      // Spinning 3-cup wind anemometer for drone spray drift monitoring!
      const anemo = new THREE.Group();
      anemo.position.set(0.32, 3.05, 0);
      const aArm = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.04, 0.04), steelMat);
      aArm.position.set(-0.16, -0.08, 0); mg.add(aArm);
      for (let k = 0; k < 3; k++) {
        const ang = (k * Math.PI * 2) / 3;
        const spoke = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.025, 0.025), darkMetalMat);
        spoke.rotation.y = ang;
        spoke.position.set(Math.cos(ang) * 0.14, 0, -Math.sin(ang) * 0.14);
        const cup = new THREE.Mesh(new THREE.SphereGeometry(0.055, 6, 6), orangeMat);
        cup.position.set(Math.cos(ang) * 0.28, 0, -Math.sin(ang) * 0.28);
        anemo.add(spoke, cup);
      }
      this.farmSpinners.push({ mesh: anemo, speed: 3.4 });
      mg.add(pole, box, sol, led, anemo);
      this.scene.add(mg);
    };

    // Helper: Field Impact Irrigation Sprinkler on Steel Riser (gently rotates in update()!)
    const addFieldSprinkler = (x, z) => {
      const sg = new THREE.Group();
      sg.position.set(x, 0, z);
      const riser = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.05, 1.15, 6), steelMat);
      riser.position.y = 0.575;
      const head = new THREE.Group();
      head.position.y = 1.18;
      const nozzle = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.07, 0.07), brassMat);
      const tip = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.04, 0.16, 6), brassMat);
      tip.rotation.z = -0.8; tip.position.set(0.18, 0.06, 0);
      head.add(nozzle, tip);
      this.farmSpinners.push({ mesh: head, speed: 1.6 });
      sg.add(riser, head);
      this.scene.add(sg);
    };

    // Helper: Concrete Agri-Drone Fertilizer Refill Pad ("F" Emblem)
    const addAgriRefillPad = (x, z, label) => {
      const pad = new THREE.Mesh(
        new THREE.BoxGeometry(3.8, 0.14, 3.8),
        [
          darkMetalMat, darkMetalMat,
          new THREE.MeshStandardMaterial({ map: makeSprayPadTex(label), roughness: 0.82 }),
          darkMetalMat, darkMetalMat, darkMetalMat
        ]
      );
      pad.position.set(x, 0.07, z);
      pad.receiveShadow = true;
      this.scene.add(pad);
      // 4 Low corner solar guide lights on refill pad
      [[-1.75, -1.75], [1.75, -1.75], [-1.75, 1.75], [1.75, 1.75]].forEach(([cx, cz]) => {
        const gl = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 0.1, 8), beaconLedMat);
        gl.position.set(x + cx, 0.18, z + cz);
        this.scene.add(gl);
      });
    };

    // Instanced Geometry Accumulators (thousands of 3D crops in < 10 GPU draw calls!)
    const allPosts = [];
    const allRidges = [];
    const allWheatGold = [];
    const allWheatDry = [];
    const allCabbages = [];
    const allCorn = [];
    const allBerries = [];

    fieldDefs.forEach((f) => {
      // 1. High-Detail Plowed Soil Surface + Slightly Raised Soil Border Berm
      const fMat = new THREE.MeshStandardMaterial({
        map: this.createCropTexture(f.base, f.stripe),
        roughness: 0.94
      });
      const fMesh = new THREE.Mesh(new THREE.PlaneGeometry(f.w, f.d), fMat);
      fMesh.rotation.x = -Math.PI / 2;
      fMesh.position.set(f.x, 0.03, f.z);
      fMesh.receiveShadow = true;
      this.scene.add(fMesh);

      const hw = f.w / 2;
      const hd = f.d / 2;

      // 2. Wooden Perimeter Fence with Real Open Entrance Gate (3.6m wide) + Zone Number Signboard!
      const addFenceSeg = (x1, z1, x2, z2) => {
        const len = Math.hypot(x2 - x1, z2 - z1);
        if (len < 0.2) return;
        const ang = Math.atan2(x2 - x1, z2 - z1);
        [0.42, 0.84].forEach((ry) => {
          const rail = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.13, len), fenceMat);
          rail.position.set((x1 + x2) / 2, ry, (z1 + z2) / 2);
          rail.rotation.y = ang;
          rail.castShadow = true;
          this.scene.add(rail);
        });
        const steps = Math.max(1, Math.round(len / 4.2));
        for (let i = 0; i <= steps; i++) {
          allPosts.push([x1 + (x2 - x1) * (i / steps), z1 + (z2 - z1) * (i / steps)]);
        }
      };

      const gateHalf = 1.9;
      // North edge
      addFenceSeg(f.x - hw, f.z - hd, f.x + hw, f.z - hd);
      // West edge
      addFenceSeg(f.x - hw, f.z - hd, f.x - hw, f.z + hd);
      // South edge (split for gate if gateSide === 'S')
      if (f.gateSide === 'S') {
        addFenceSeg(f.x - hw, f.z + hd, f.gatePos - gateHalf, f.z + hd);
        addFenceSeg(f.gatePos + gateHalf, f.z + hd, f.x + hw, f.z + hd);
      } else {
        addFenceSeg(f.x - hw, f.z + hd, f.x + hw, f.z + hd);
      }
      // East edge (split for gate if gateSide === 'E')
      if (f.gateSide === 'E') {
        addFenceSeg(f.x + hw, f.z - hd, f.x + hw, f.gatePos - gateHalf);
        addFenceSeg(f.x + hw, f.gatePos + gateHalf, f.x + hw, f.z + hd);
      } else {
        addFenceSeg(f.x + hw, f.z - hd, f.x + hw, f.z + hd);
      }

      // Open swung-in wooden gate leaf + Tall Zone Signboard right beside the field entrance!
      const gGrp = new THREE.Group();
      if (f.gateSide === 'S') {
        gGrp.position.set(f.gatePos - gateHalf, 0, f.z + hd);
        gGrp.rotation.y = 0.65; // Swung gracefully open inward
      } else {
        gGrp.position.set(f.x + hw, 0, f.gatePos - gateHalf);
        gGrp.rotation.y = Math.PI / 2 + 0.65;
      }
      [0.42, 0.84].forEach((ry) => {
        const gBar = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.1, 0.08), fenceMat);
        gBar.position.set(1.3, ry, 0); gGrp.add(gBar);
      });
      const gDiag = new THREE.Mesh(new THREE.BoxGeometry(2.7, 0.09, 0.08), fenceMat);
      gDiag.position.set(1.3, 0.63, 0); gDiag.rotation.z = 0.22;
      gGrp.add(gDiag);
      this.scene.add(gGrp);

      // Painted Zone Identification Signpost ("ZONE A · WHEAT" etc.) at field gate
      const zSign = new THREE.Group();
      if (f.gateSide === 'S') {
        zSign.position.set(f.gatePos + gateHalf + 1.4, 0, f.z + hd + 0.35);
      } else {
        zSign.position.set(f.x + hw + 0.35, 0, f.gatePos + gateHalf + 1.4);
        zSign.rotation.y = Math.PI / 2;
      }
      [-0.68, 0.68].forEach((sx) => {
        const sp = new THREE.Mesh(new THREE.BoxGeometry(0.1, 1.9, 0.1), postMat);
        sp.position.set(sx, 0.95, 0); zSign.add(sp);
      });
      const sBoard = new THREE.Mesh(
        new THREE.BoxGeometry(1.75, 0.66, 0.09),
        new THREE.MeshStandardMaterial({ map: makeZoneSignTex(f.name, 'SMART AGRI-SPRAY PLOT'), roughness: 0.6 })
      );
      sBoard.position.set(0, 1.55, 0); sBoard.castShadow = true;
      zSign.add(sBoard);
      this.scene.add(zSign);

      // 3. Four High-Visibility Orange/White RTK-GPS Spray-Boundary Corner Beacons on every farm plot!
      [[-hw, -hd], [hw, -hd], [-hw, hd], [hw, hd]].forEach(([ox, oz]) => {
        const mx = f.x + ox, mz = f.z + oz;
        const mPole = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.075, 2.35, 8), whiteMat);
        mPole.position.set(mx, 1.18, mz);
        const mBand = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.55, 8), orangeMat);
        mBand.position.set(mx, 1.85, mz);
        const mCap = new THREE.Mesh(new THREE.SphereGeometry(0.11, 8, 8), beaconLedMat);
        mCap.position.set(mx, 2.42, mz);
        this.scene.add(mPole, mBand, mCap);
      });

      // 4. Main Metal Irrigation Header Pipe running cleanly along the field edge with brass shutoff valve!
      const headerPipe = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, f.w - 1.2, 8), steelMat);
      headerPipe.rotation.z = Math.PI / 2;
      headerPipe.position.set(f.x, 0.14, f.z - hd + 0.55);
      const valveWheel = new THREE.Mesh(new THREE.TorusGeometry(0.14, 0.03, 6, 10), berryFruitMat);
      valveWheel.position.set(f.x + hw - 1.2, 0.36, f.z - hd + 0.55);
      this.scene.add(headerPipe, valveWheel);

      // 5. Crop-Specific 3D Agricultural Rows, Raised Soil Beds, Sprinklers & Fertilizer Target Patches!
      if (f.type === 'wheat') {
        // ZONE A: Dense Golden Wheat Rows + Pale Nutrient-Deficient Center-West Patch (ready for future drone fertilizing!)
        for (let rz = f.z - hd + 2.2; rz <= f.z + hd - 2.2; rz += 2.6) {
          allRidges.push([f.x, rz, f.w - 2.6, 0.9]);
          for (let rx = f.x - hw + 2.0; rx <= f.x + hw - 2.0; rx += 1.9) {
            if (Math.hypot(rx - f.x, rz - f.z) < 2.2) continue; // Keep Scarecrow center clear
            // Leave a neat harvested strip in SE corner for 3 golden hay bales
            if (rx > f.x + 5.5 && rz > f.z + 3.5) continue;
            const isDryPatch = (rx < f.x - 1.5 && rz > f.z - 5.5 && rz < f.z + 5.5);
            const sc = 0.85 + ((Math.abs(Math.round(rx * 5 + rz * 7))) % 10) * 0.04;
            (isDryPatch ? allWheatDry : allWheatGold).push([rx, rz, sc]);
          }
        }
        // 3 Neatly stacked golden hay bales on harvested SE corner
        [[f.x + 8.5, f.z + 7.2, 0.25], [f.x + 11.4, f.z + 6.4, 0.75], [f.x + 9.8, f.z + 9.0, 1.1]].forEach(([bx, bz, rot]) => {
          const bale = new THREE.Mesh(new THREE.CylinderGeometry(0.78, 0.78, 1.75, 12), hayMat);
          bale.rotation.z = Math.PI / 2; bale.rotation.y = rot;
          bale.position.set(bx, 0.78, bz); bale.castShadow = true; bale.receiveShadow = true;
          this.scene.add(bale);
        });
        addFieldSprinkler(f.x - 8, f.z - 5);
        addFieldSprinkler(f.x + 7, f.z - 5);
      } else if (f.type === 'cabbage') {
        // ZONE B: Raised Dark-Loam Vegetable Beds with Multi-Layered 3D Cabbages + 4 Rotating Impact Sprinklers!
        for (let rz = f.z - hd + 2.6; rz <= f.z + hd - 2.6; rz += 3.2) {
          allRidges.push([f.x, rz, f.w - 3.0, 1.45]);
          for (let rx = f.x - hw + 2.6; rx <= f.x + hw - 2.6; rx += 2.6) {
            const sc = 0.82 + ((Math.abs(Math.round(rx * 7 + rz * 3))) % 10) * 0.038;
            allCabbages.push([rx, rz, sc]);
          }
        }
        [[-9, -8], [9, -8], [-9, 8], [9, 8]].forEach(([sx, sz]) => addFieldSprinkler(f.x + sx, f.z + sz));
      } else if (f.type === 'corn') {
        // ZONE C: Tall Sweet Corn Stalks in Precision Drone-Spray Lanes + Harvested Hay Bales + NPK Pallet!
        for (let rz = f.z - hd + 2.4; rz <= f.z + hd - 2.4; rz += 2.9) {
          allRidges.push([f.x, rz, f.w - 2.8, 1.0]);
          for (let rx = f.x - hw + 2.2; rx <= f.x + hw - 2.2; rx += 2.1) {
            // Keep harvested corner near South gate clear for hay bales & staging
            if (rx > f.x + 3.5 && rz > f.z + 4.5) continue;
            const sc = 0.88 + ((Math.abs(Math.round(rx * 3 + rz * 11))) % 10) * 0.035;
            allCorn.push([rx, rz, sc]);
          }
        }
        [[f.x + 7.2, f.z + 8.5, 0.3], [f.x + 10.2, f.z + 7.4, 0.85], [f.x + 8.6, f.z + 10.6, 1.2]].forEach(([bx, bz, rot]) => {
          const bale = new THREE.Mesh(new THREE.CylinderGeometry(0.8, 0.8, 1.8, 12), hayMat);
          bale.rotation.z = Math.PI / 2; bale.rotation.y = rot;
          bale.position.set(bx, 0.8, bz); bale.castShadow = true; bale.receiveShadow = true;
          this.scene.add(bale);
        });
        addFieldSprinkler(f.x - 6, f.z - 4);
        addFieldSprinkler(f.x + 6, f.z - 4);
        addNpkPallet(this.scene, f.x + hw - 2.2, f.z + hd - 2.2, 0.15);
      } else if (f.type === 'trellis') {
        // ZONE D: High-Value Organic Berry / Tomato Wire Trellis Rows right across from the Greenhouse!
        for (let rz = f.z - hd + 3.0; rz <= f.z + hd - 3.0; rz += 3.6) {
          allRidges.push([f.x, rz, f.w - 3.2, 1.15]);
          // Continuous wooden trellis wire line + end/mid timber stakes per row
          const tWire = new THREE.Mesh(new THREE.BoxGeometry(f.w - 3.6, 0.04, 0.04), steelMat);
          tWire.position.set(f.x, 1.25, rz);
          this.scene.add(tWire);
          for (let px = f.x - hw + 2.0; px <= f.x + hw - 1.8; px += 7.5) {
            allPosts.push([px, rz]);
          }
          for (let rx = f.x - hw + 2.6; rx <= f.x + hw - 2.6; rx += 2.3) {
            const sc = 0.85 + ((Math.abs(Math.round(rx * 5 + rz * 7))) % 10) * 0.035;
            allBerries.push([rx, rz, sc]);
          }
        }
        addFieldSprinkler(f.x - 7, f.z);
        addFieldSprinkler(f.x + 7, f.z);
      }
    });

    const dummy = new THREE.Object3D();

    // Instanced Wooden Fence & Trellis Posts (1 draw call, skip tiny post shadow rasterization)
    if (allPosts.length) {
      const postInst = new THREE.InstancedMesh(new THREE.BoxGeometry(0.2, 1.25, 0.2), postMat, allPosts.length);
      allPosts.forEach(([px, pz], idx) => {
        dummy.position.set(px, 0.62, pz);
        dummy.rotation.set(0, 0, 0);
        dummy.scale.set(1, 1, 1);
        dummy.updateMatrix();
        postInst.setMatrixAt(idx, dummy.matrix);
      });
      this.scene.add(postInst);
    }

    // Instanced Raised Tilled-Soil Crop Ridges (1 draw call)
    if (allRidges.length) {
      const ridgeInst = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 0.16, 1), soilRidgeMat, allRidges.length);
      ridgeInst.receiveShadow = true;
      allRidges.forEach(([rx, rz, rw, rd], idx) => {
        dummy.position.set(rx, 0.1, rz);
        dummy.rotation.set(0, 0, 0);
        dummy.scale.set(rw, 1, rd);
        dummy.updateMatrix();
        ridgeInst.setMatrixAt(idx, dummy.matrix);
      });
      this.scene.add(ridgeInst);
    }

    // Instanced Golden Wheat Clusters & Nutrient-Needed Dry Patch (2 draw calls; skip tiny crop shadow pass for huge GPU gain!)
    const addWheatInst = (arr, mat) => {
      if (!arr.length) return;
      const inst = new THREE.InstancedMesh(new THREE.ConeGeometry(0.58, 1.15, 5), mat, arr.length);
      inst.receiveShadow = true;
      arr.forEach(([wx, wz, sc], idx) => {
        dummy.position.set(wx, 0.65 * sc, wz);
        dummy.rotation.set((idx % 3 - 1) * 0.06, idx * 0.85, (idx % 2 === 0 ? 0.06 : -0.06));
        dummy.scale.set(sc * 1.15, sc, sc * 1.15);
        dummy.updateMatrix();
        inst.setMatrixAt(idx, dummy.matrix);
      });
      this.scene.add(inst);
    };
    addWheatInst(allWheatGold, wheatGoldMat);
    addWheatInst(allWheatDry, wheatDryMat);

    // Instanced Two-Tone 3D Cabbages (Outer Leaf Rosette + Crisp Inner Head — 2 draw calls!)
    if (allCabbages.length) {
      const outerInst = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(0.68, 0), cabOuterMat, allCabbages.length);
      const innerInst = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(0.42, 0), cabInnerMat, allCabbages.length);
      outerInst.receiveShadow = true;
      allCabbages.forEach(([cx, cz, sc], idx) => {
        dummy.position.set(cx, 0.36 * sc, cz);
        dummy.rotation.set(0, idx * 0.7, 0);
        dummy.scale.set(sc * 1.25, sc * 0.52, sc * 1.25);
        dummy.updateMatrix();
        outerInst.setMatrixAt(idx, dummy.matrix);

        dummy.position.set(cx, 0.5 * sc, cz);
        dummy.scale.set(sc * 0.92, sc * 0.85, sc * 0.92);
        dummy.updateMatrix();
        innerInst.setMatrixAt(idx, dummy.matrix);
      });
      this.scene.add(outerInst, innerInst);
    }

    // Instanced Tall Sweet Corn Stalks + Golden Tassels (2 draw calls!)
    if (allCorn.length) {
      const stalkInst = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.18, 0.42, 1.95, 6), cornStalkMat, allCorn.length);
      const tasselInst = new THREE.InstancedMesh(new THREE.ConeGeometry(0.24, 0.55, 5), cornTasselMat, allCorn.length);
      stalkInst.receiveShadow = true;
      allCorn.forEach(([cx, cz, sc], idx) => {
        dummy.position.set(cx, 1.0 * sc, cz);
        dummy.rotation.set(0, idx * 0.9, 0);
        dummy.scale.set(sc, sc, sc);
        dummy.updateMatrix();
        stalkInst.setMatrixAt(idx, dummy.matrix);

        dummy.position.set(cx, 2.08 * sc, cz);
        dummy.scale.set(sc, sc, sc);
        dummy.updateMatrix();
        tasselInst.setMatrixAt(idx, dummy.matrix);
      });
      this.scene.add(stalkInst, tasselInst);
    }

    // Instanced Organic Berry / Tomato Trellis Bushes + Ripening Red Fruit Clusters (2 draw calls!)
    if (allBerries.length) {
      const bushInst = new THREE.InstancedMesh(new THREE.DodecahedronGeometry(0.62, 0), berryBushMat, allBerries.length);
      const fruitInst = new THREE.InstancedMesh(new THREE.SphereGeometry(0.14, 6, 6), berryFruitMat, allBerries.length * 2);
      bushInst.receiveShadow = true;
      allBerries.forEach(([bx, bz, sc], idx) => {
        dummy.position.set(bx, 0.82 * sc, bz);
        dummy.rotation.set(0, idx * 0.65, 0);
        dummy.scale.set(sc * 1.15, sc * 1.1, sc * 0.78);
        dummy.updateMatrix();
        bushInst.setMatrixAt(idx, dummy.matrix);

        [-0.24, 0.24].forEach((ox, k) => {
          dummy.position.set(bx + ox, 0.92 * sc + k * 0.14, bz + (k === 0 ? 0.34 : -0.34));
          dummy.scale.set(sc, sc, sc);
          dummy.updateMatrix();
          fruitInst.setMatrixAt(idx * 2 + k, dummy.matrix);
        });
      });
      this.scene.add(bushInst, fruitInst);
    }

    // =========================================================================
    // SMART-AGRICULTURE SUPPORT STATIONS (Refill Pads, Mixing Shed, Fertigation Water Tower & Weather Masts!)
    // =========================================================================

    // 1. ZONE A: Upgraded Agri-Drone Fertilizer Mixing Shed (-36.5, -15.5) + Concrete Drone Refill Pad (-31.5, -15.5)
    const fieldShed = new THREE.Group();
    fieldShed.position.set(-36.5, 0, -15.5);
    const shedWallTexMat = new THREE.MeshStandardMaterial({ map: makeFarmWoodTex('#84522a', '#734522'), roughness: 0.88, side: THREE.DoubleSide });
    const shedRoofMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.65, metalness: 0.35, side: THREE.DoubleSide });
    const shedPlinth = new THREE.Mesh(new THREE.BoxGeometry(4.2, 0.22, 3.3), steelMat);
    shedPlinth.position.y = 0.11;
    const shedBase = new THREE.Mesh(new THREE.BoxGeometry(3.8, 2.5, 2.9), shedWallTexMat);
    shedBase.position.y = 1.46; shedBase.castShadow = true; shedBase.receiveShadow = true;
    const shedRoof = new THREE.Mesh(new THREE.BoxGeometry(4.4, 0.2, 3.6), shedRoofMat);
    shedRoof.position.set(0, 2.86, 0); shedRoof.rotation.x = 0.16; shedRoof.castShadow = true;
    const shedDoor = new THREE.Mesh(new THREE.BoxGeometry(1.15, 2.0, 0.14), postMat);
    shedDoor.position.set(0, 1.22, 1.46);
    const shedSign = new THREE.Mesh(
      new THREE.BoxGeometry(2.6, 0.68, 0.1),
      new THREE.MeshStandardMaterial({ map: makeZoneSignTex('AGRI MIXING', 'FERTILIZER & DRONE HUB'), roughness: 0.55 })
    );
    shedSign.position.set(0, 2.48, 1.5);
    fieldShed.add(shedPlinth, shedBase, shedRoof, shedDoor, shedSign);
    addIbcTank(fieldShed, -1.35, 2.35, fertTankMat, 0);
    addIbcTank(fieldShed, 1.35, 2.35, fertGreenMat, 0);
    addNpkPallet(fieldShed, 2.95, 0.1, 0.08);
    this.scene.add(fieldShed);
    this.addBoxCollider(-36.5, 1.5, -15.5, 4.2, 3.1, 3.4);

    // Zone A Concrete Agri-Drone Refill Pad right beside the Mixing Shed!
    addAgriRefillPad(-30.5, -15.5, 'ZONE A');
    addSmartSensorMast(-41.8, -6.5);

    // 2. ZONE B: Upgraded Timber & Iron-Hooped Farm Fertigation Water Tower (-53.2, 0, 24)
    const waterTower = new THREE.Group();
    waterTower.position.set(-53.2, 0, 24);
    const tankStaveMat = new THREE.MeshStandardMaterial({ map: makeFarmWoodTex('#8c5a2b', '#764821'), roughness: 0.86, side: THREE.DoubleSide });
    const tankRoofMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.6, metalness: 0.38 });
    // 4 Concrete Footing Piers + Tapered Timber Legs + Cross-Bracing + Maintenance Ladder + Catwalk!
    [[-1.15, -1.15], [1.15, -1.15], [-1.15, 1.15], [1.15, 1.15]].forEach(([lx, lz]) => {
      const foot = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.35, 0.42), steelMat);
      foot.position.set(lx, 0.18, lz);
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.17, 5.4, 8), postMat);
      leg.position.set(lx, 2.85, lz); leg.castShadow = true;
      waterTower.add(foot, leg);
    });
    [1.8, 3.6, 5.2].forEach((by) => {
      const bN = new THREE.Mesh(new THREE.BoxGeometry(2.4, 0.14, 0.14), fenceMat);
      bN.position.set(0, by, -1.15);
      const bS = bN.clone(); bS.position.z = 1.15;
      const bW = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.14, 2.4), fenceMat);
      bW.position.set(-1.15, by, 0);
      const bE = bW.clone(); bE.position.x = 1.15;
      waterTower.add(bN, bS, bW, bE);
    });
    // Vertical Steel Supply Standpipe + Catwalk Deck + Guardrail Ring
    const standPipe = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.18, 5.4, 10), steelMat);
    standPipe.position.set(-0.55, 2.7, -0.55);
    const catwalk = new THREE.Mesh(new THREE.CylinderGeometry(2.35, 2.35, 0.16, 16), fenceMat);
    catwalk.position.y = 5.45; catwalk.castShadow = true;
    const cwRail = new THREE.Mesh(new THREE.TorusGeometry(2.28, 0.045, 6, 20), darkMetalMat);
    cwRail.rotation.x = Math.PI / 2; cwRail.position.y = 6.32;
    for (let k = 0; k < 8; k++) {
      const ka = (k / 8) * Math.PI * 2;
      const rp = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.88, 6), darkMetalMat);
      rp.position.set(Math.cos(ka) * 2.28, 5.88, Math.sin(ka) * 2.28);
      waterTower.add(rp);
    }
    // Weathered Wooden Stave Water Tank + 3 Black Iron Tension Hoops + Conical Metal Roof & Finial!
    const tank = new THREE.Mesh(new THREE.CylinderGeometry(1.78, 1.84, 3.0, 18), tankStaveMat);
    tank.position.y = 7.0; tank.castShadow = true;
    [6.0, 7.0, 8.0].forEach((hy) => {
      const hoop = new THREE.Mesh(new THREE.TorusGeometry(1.83, 0.045, 6, 22), darkMetalMat);
      hoop.rotation.x = Math.PI / 2; hoop.position.y = hy;
      waterTower.add(hoop);
    });
    const tankCap = new THREE.Mesh(new THREE.ConeGeometry(2.15, 1.55, 18), tankRoofMat);
    tankCap.position.y = 9.25; tankCap.castShadow = true;
    const tankSpire = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.06, 0.75, 6), steelMat);
    tankSpire.position.y = 10.32;
    // Wooden Access Ladder climbing the East side + Ground-Level Liquid Fertilizer Mixing Tank under Trestle!
    [-0.22, 0.22].forEach((lz) => {
      const lRail = new THREE.Mesh(new THREE.BoxGeometry(0.06, 5.6, 0.06), fenceMat);
      lRail.position.set(1.32, 2.8, lz); waterTower.add(lRail);
    });
    for (let ry = 0.6; ry <= 5.2; ry += 0.55) {
      const rung = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.48), fenceMat);
      rung.position.set(1.32, ry, 0); waterTower.add(rung);
    }
    addIbcTank(waterTower, 0.32, 0.32, fertGreenMat, 0);
    waterTower.add(standPipe, catwalk, cwRail, tank, tankCap, tankSpire);
    this.scene.add(waterTower);
    this.addCylCollider(-53.2, 24, 2.35, 0, 10.4);
    addSmartSensorMast(-55.2, 11.5);

    // 3. ZONE D: Concrete Agri-Drone Fertilizer Refill Pad (26.5, 72.6) + IBC Chemical Tanks & NPK Pallet (22.0, 72.5)
    addAgriRefillPad(26.5, 72.6, 'ZONE D');
    addIbcTank(this.scene, 21.8, 72.5, fertTankMat, 0);
    addIbcTank(this.scene, 23.2, 72.5, fertGreenMat, 0);
    addNpkPallet(this.scene, 20.3, 72.5, 0.12);
    addSmartSensorMast(50.8, 71.2);
    addSmartSensorMast(-11.8, 76.5);

    // 4. Detailed Classic Scarecrow guarding Zone A Golden Wheat Field (-56, 0, -18)
    const scarecrow = new THREE.Group();
    scarecrow.position.set(-56, 0, -18);
    const scPost = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.11, 2.65, 6), postMat);
    scPost.position.y = 1.32; scPost.castShadow = true;
    const scArms = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 2.15, 6), postMat);
    scArms.rotation.z = Math.PI / 2; scArms.position.y = 1.88;
    const scShirt = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.95, 0.36), new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.9 }));
    scShirt.position.y = 1.65;
    const scPatch = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.24, 0.39), berryFruitMat);
    scPatch.position.set(0.18, 1.55, 0);
    // Straw cuffs sticking out from shirt sleeves
    [-0.95, 0.95].forEach((sx) => {
      const cuff = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.36, 6), hayMat);
      cuff.rotation.z = sx < 0 ? Math.PI / 2 : -Math.PI / 2;
      cuff.position.set(sx, 1.88, 0); scarecrow.add(cuff);
    });
    const scHead = new THREE.Mesh(new THREE.SphereGeometry(0.32, 10, 10), hayMat);
    scHead.position.y = 2.4;
    const scHatBrim = new THREE.Mesh(new THREE.CylinderGeometry(0.62, 0.65, 0.06, 12), hayMat);
    scHatBrim.position.y = 2.64;
    const scHat = new THREE.Mesh(new THREE.ConeGeometry(0.42, 0.46, 10), fenceMat);
    scHat.position.y = 2.86;
    scarecrow.add(scPost, scArms, scShirt, scPatch, scHead, scHatBrim, scHat);
    this.scene.add(scarecrow);
  }

  buildVillage() {
    // Richly Detailed 5 Unique House Styles + Textured Village Street + Parked Car + Upgraded Red Farm Barn
    const trimDarkMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.78, side: THREE.DoubleSide });
    const trimWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.8, side: THREE.DoubleSide });
    const woodMat = new THREE.MeshStandardMaterial({ color: 0x6b3a17, roughness: 0.88, side: THREE.DoubleSide });
    const lightWoodMat = new THREE.MeshStandardMaterial({ color: 0x9a6536, roughness: 0.85, side: THREE.DoubleSide });
    const glassMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.12, metalness: 0.72, emissive: 0x453512, emissiveIntensity: 0.45, side: THREE.DoubleSide });
    const bushMat = new THREE.MeshStandardMaterial({ color: 0x1e5628, roughness: 0.88, flatShading: true });
    const flowerRedMat = new THREE.MeshStandardMaterial({ color: 0xef4444, roughness: 0.7 });
    const flowerYelMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.7 });

    // Procedural roof shingle texture so roofs never look like flat plastic slabs
    const makeRoofTex = (hexBase, hexLine) => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = hexBase;
      ctx.fillRect(0, 0, 128, 128);
      ctx.fillStyle = hexLine;
      for (let y = 0; y < 128; y += 16) {
        ctx.fillRect(0, y, 128, 3);
        const offset = (y / 16) % 2 === 0 ? 0 : 16;
        for (let x = offset; x < 128; x += 32) {
          ctx.fillRect(x, y, 2, 16);
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 3);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural cobblestone street texture
    const makeCobbleTex = () => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#4b5563';
      ctx.fillRect(0, 0, 128, 128);
      for (let y = 0; y < 128; y += 16) {
        const off = (y / 16) % 2 === 0 ? 0 : 12;
        for (let x = -12 + off; x < 128; x += 24) {
          ctx.fillStyle = ((x + y) % 3 === 0) ? '#64748b' : '#525b68';
          ctx.fillRect(x + 2, y + 2, 20, 12);
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 14);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural horizontal cedar timber log texture (for Alpine Log Cabin)
    const makeLogTex = (hexBase = '#8b5630', hexHighlight = '#a56b40', hexShadow = '#4e2b14') => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      const courses = 8;
      const rowH = 128 / courses;
      for (let r = 0; r < courses; r++) {
        const y = r * rowH;
        const grad = ctx.createLinearGradient(0, y, 0, y + rowH);
        grad.addColorStop(0, hexShadow);
        grad.addColorStop(0.22, hexHighlight);
        grad.addColorStop(0.65, hexBase);
        grad.addColorStop(1, hexShadow);
        ctx.fillStyle = grad;
        ctx.fillRect(0, y, 128, rowH);
        // Subtle vertical log seam
        const vx = ((r * 43 + 24) % 96) + 16;
        ctx.fillStyle = 'rgba(40, 20, 8, 0.45)';
        ctx.fillRect(vx, y + 1, 2, rowH - 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(2, 1.4);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural vertical barn siding texture (for Red Farm Barn)
    const makeBarnPlankTex = (hexBase = '#9f2a22', hexSeam = '#681611') => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = hexBase;
      ctx.fillRect(0, 0, 128, 128);
      for (let x = 0; x < 128; x += 16) {
        ctx.fillStyle = hexSeam;
        ctx.fillRect(x, 0, 2, 128);
        ctx.fillStyle = 'rgba(255,255,255,0.06)';
        ctx.fillRect(x + 2, 0, 3, 128);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const knobBrassMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.25, metalness: 0.85 });
    const warmLightMat = new THREE.MeshBasicMaterial({ color: 0xfef08a });
    const stoneChimneyMat = new THREE.MeshStandardMaterial({ color: 0x7a7f85, roughness: 0.94 });

    // Helper: realistic framed entrance door with recessed panel, upper glass pane, threshold & brass doorknob
    const addDoor = (parent, x, y, z, w = 1.05, h = 2.0, doorMat = woodMat, frameMat = trimWhiteMat, rotY = 0) => {
      const dg = new THREE.Group();
      dg.position.set(x, y, z);
      dg.rotation.y = rotY;
      const frame = new THREE.Mesh(new THREE.BoxGeometry(w + 0.2, h + 0.14, 0.14), frameMat);
      frame.position.y = 0.03;
      const leaf = new THREE.Mesh(new THREE.BoxGeometry(w, h, 0.18), doorMat);
      const sill = new THREE.Mesh(new THREE.BoxGeometry(w + 0.26, 0.08, 0.24), trimDarkMat);
      sill.position.set(0, -h / 2 + 0.02, 0.04);
      // Upper glazed window inset + lower decorative recessed trim panel
      const topGlassFrame = new THREE.Mesh(new THREE.BoxGeometry(w * 0.62, h * 0.3, 0.2), frameMat);
      topGlassFrame.position.set(0, h * 0.22, 0.01);
      const topGlass = new THREE.Mesh(new THREE.BoxGeometry(w * 0.52, h * 0.23, 0.22), glassMat);
      topGlass.position.set(0, h * 0.22, 0.01);
      const lowPanel = new THREE.Mesh(new THREE.BoxGeometry(w * 0.64, h * 0.32, 0.21), frameMat);
      lowPanel.position.set(0, -h * 0.22, 0.0);
      const lowPanelInner = new THREE.Mesh(new THREE.BoxGeometry(w * 0.54, h * 0.26, 0.22), doorMat);
      lowPanelInner.position.set(0, -h * 0.22, 0.005);
      // Brass doorknob
      const knob = new THREE.Mesh(new THREE.SphereGeometry(0.06, 8, 8), knobBrassMat);
      knob.position.set(w * 0.34, -0.05, 0.14);
      dg.add(frame, leaf, sill, topGlassFrame, topGlass, lowPanel, lowPanelInner, knob);
      parent.add(dg);
    };

    // Helper: realistic framed window + optional wooden shutters & flowerbox with blooms!
    const addWindow = (parent, x, y, z, w = 1.1, h = 1.1, rotY = 0, withDecor = true) => {
      const wg = new THREE.Group();
      wg.position.set(x, y, z);
      wg.rotation.y = rotY;
      const frame = new THREE.Mesh(new THREE.BoxGeometry(w + 0.18, h + 0.18, 0.14), trimWhiteMat);
      const pane = new THREE.Mesh(new THREE.BoxGeometry(w, h, 0.17), glassMat);
      const barH = new THREE.Mesh(new THREE.BoxGeometry(w + 0.04, 0.06, 0.19), trimWhiteMat);
      const barV = new THREE.Mesh(new THREE.BoxGeometry(0.06, h + 0.04, 0.19), trimWhiteMat);
      wg.add(frame, pane, barH, barV);

      if (withDecor) {
        // Wooden shutters flanking window + flowerbox underneath
        const shL = new THREE.Mesh(new THREE.BoxGeometry(0.24, h + 0.14, 0.08), woodMat);
        shL.position.set(-(w / 2 + 0.22), 0, 0.04);
        const shR = shL.clone();
        shR.position.x = w / 2 + 0.22;
        const fBox = new THREE.Mesh(new THREE.BoxGeometry(w + 0.24, 0.22, 0.28), woodMat);
        fBox.position.set(0, -(h / 2 + 0.14), 0.14);
        const leaf = new THREE.Mesh(new THREE.BoxGeometry(w + 0.12, 0.16, 0.2), bushMat);
        leaf.position.set(0, -(h / 2 + 0.01), 0.14);
        const fl1 = new THREE.Mesh(new THREE.SphereGeometry(0.09, 6, 6), flowerRedMat);
        fl1.position.set(-w * 0.25, -(h / 2 - 0.08), 0.16);
        const fl2 = new THREE.Mesh(new THREE.SphereGeometry(0.09, 6, 6), flowerYelMat);
        fl2.position.set(w * 0.22, -(h / 2 - 0.08), 0.16);
        wg.add(shL, shR, fBox, leaf, fl1, fl2);
      }
      parent.add(wg);
    };

    // Helper: watertight A-frame gable roof with exact-pitch eaves, solid extruded attic prism (flush with house walls!) & ridge cap
    const addGableRoof = (
      parent,
      width,
      wallLen,
      baseY,
      peakH,
      roofHex,
      lineHex,
      wallMat,
      centerZ = 0,
      frontOverhang = 0.5,
      backOverhang = 0.5,
      eaveX = 0.55,
      wallTexRefH = 3.0
    ) => {
      const rMat = new THREE.MeshStandardMaterial({
        map: makeRoofTex(roofHex, lineHex),
        roughness: 0.78,
        metalness: 0.06,
        side: THREE.DoubleSide
      });

      const halfWall = width / 2;
      // Exact roof pitch matching the gable triangle corners (±halfWall, baseY) -> (0, baseY + peakH)
      const pitchAngle = Math.atan2(peakH, halfWall);
      const run = halfWall + eaveX;
      const drop = run * Math.tan(pitchAngle);
      const slopeLen = Math.hypot(run, drop) + 0.14;
      const slabT = 0.24;
      const nOff = slabT / 2 - 0.04; // seats underside slightly into gable edges => ZERO gap!

      const roofLen = wallLen + frontOverhang + backOverhang;
      const roofCenterZ = centerZ + (frontOverhang - backOverhang) / 2;
      const midX = run / 2;
      const midY = (baseY + peakH) - drop / 2;

      const sL = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, slabT, roofLen), rMat);
      sL.position.set(-midX - Math.sin(pitchAngle) * nOff, midY + Math.cos(pitchAngle) * nOff, roofCenterZ);
      sL.rotation.z = pitchAngle;
      sL.castShadow = true;
      sL.receiveShadow = true;

      const sR = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, slabT, roofLen), rMat);
      sR.position.set(midX + Math.sin(pitchAngle) * nOff, midY + Math.cos(pitchAngle) * nOff, roofCenterZ);
      sR.rotation.z = -pitchAngle;
      sR.castShadow = true;
      sR.receiveShadow = true;

      // Top ridge cap beam sealing the apex seam cleanly
      const ridge = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.26, roofLen + 0.08), trimDarkMat);
      ridge.position.set(0, baseY + peakH + slabT / 2 + 0.02, roofCenterZ);
      ridge.castShadow = true;

      // Front & back fascia bargeboard trim boards under the overhanging roof edges
      [roofCenterZ + roofLen / 2 - 0.05, roofCenterZ - roofLen / 2 + 0.05].forEach((fz) => {
        const trimL = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, 0.14, 0.1), trimDarkMat);
        trimL.position.set(-midX - Math.sin(pitchAngle) * (nOff - 0.12), midY + Math.cos(pitchAngle) * (nOff - 0.12), fz);
        trimL.rotation.z = pitchAngle;
        const trimR = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, 0.14, 0.1), trimDarkMat);
        trimR.position.set(midX + Math.sin(pitchAngle) * (nOff - 0.12), midY + Math.cos(pitchAngle) * (nOff - 0.12), fz);
        trimR.rotation.z = -pitchAngle;
        parent.add(trimL, trimR);
      });

      // Solid 3D extruded gable attic prism sitting 100% flush with house front/back walls (zero sky-gap & zero Z-fighting!)
      const gShape = new THREE.Shape([
        new THREE.Vector2(-halfWall, 0),
        new THREE.Vector2(halfWall, 0),
        new THREE.Vector2(0, peakH + 0.02)
      ]);
      const gGeo = new THREE.ExtrudeGeometry(gShape, { depth: wallLen, bevelEnabled: false });
      // Normalize UVs so textured walls (e.g., horizontal logs / barn planks) match the house body seamlessly
      const uvAttr = gGeo.attributes.uv;
      const posAttr = gGeo.attributes.position;
      for (let i = 0; i < uvAttr.count; i++) {
        uvAttr.setXY(i, (posAttr.getX(i) + halfWall) / width, posAttr.getY(i) / wallTexRefH);
      }
      uvAttr.needsUpdate = true;

      const gablePrism = new THREE.Mesh(gGeo, wallMat);
      gablePrism.position.set(0, baseY, centerZ - wallLen / 2);
      gablePrism.castShadow = true;
      gablePrism.receiveShadow = true;

      parent.add(sL, sR, ridge, gablePrism);
    };

    // Helper: grounded wooden deck/porch railing with top rail, bottom rail & evenly spaced vertical balusters
    const addWoodRailing = (parent, x1, z1, x2, z2, deckY, railH = 0.82, railMat = lightWoodMat, postMat = woodMat) => {
      const dx = x2 - x1;
      const dz = z2 - z1;
      const len = Math.hypot(dx, dz);
      if (len < 0.05) return;
      const angle = Math.atan2(dx, dz);
      const rg = new THREE.Group();
      rg.position.set((x1 + x2) / 2, deckY, (z1 + z2) / 2);
      rg.rotation.y = angle;

      const topRail = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.09, len + 0.08), railMat);
      topRail.position.y = railH;
      const botRail = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.07, len), railMat);
      botRail.position.y = 0.16;
      rg.add(topRail, botRail);

      // End newel posts + evenly spaced vertical balusters anchored to deckY
      const count = Math.max(2, Math.round(len / 0.42) + 1);
      for (let i = 0; i < count; i++) {
        const t = i / (count - 1);
        const zPos = -len / 2 + t * len;
        const isEnd = (i === 0 || i === count - 1);
        const size = isEnd ? 0.11 : 0.065;
        const post = new THREE.Mesh(new THREE.BoxGeometry(size, railH + (isEnd ? 0.06 : 0), size), isEnd ? postMat : railMat);
        post.position.set(0, (railH + (isEnd ? 0.06 : 0)) / 2, zPos);
        rg.add(post);
      }
      parent.add(rg);
    };

    // Village Street Lamps along Main Street & spacious East Market Avenue (roads & curbs built seamlessly in buildRoadNetwork!)
    [[67.6, 19], [74.4, 26], [67.6, 36], [74.4, 46], [86, 27.6], [108, 27.6], [126, 34.4]].forEach(([lx, lz]) => {
      const lamp = new THREE.Group();
      lamp.position.set(lx, 0, lz);
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.12, 4.6, 8), trimDarkMat);
      pole.position.y = 2.3;
      pole.castShadow = true;
      const armDir = (lx > 80) ? 0 : (lx < 71 ? 1 : -1);
      const arm = new THREE.Mesh(new THREE.BoxGeometry(armDir === 0 ? 0.08 : 0.75, 0.08, armDir === 0 ? 0.75 : 0.08), trimDarkMat);
      arm.position.set(armDir * 0.35, 4.5, armDir === 0 ? (lz < 31 ? 0.35 : -0.35) : 0);
      const head = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.22, 0.36), warmLightMat);
      head.position.set(armDir * 0.65, 4.38, armDir === 0 ? (lz < 31 ? 0.65 : -0.65) : 0);
      lamp.add(pole, arm, head);
      this.scene.add(lamp);
      this.addCylCollider(lx, lz, 0.35, 0, 4.7, 'power_pole');
    });

    // ------------------------------------------------------------
    // HOUSE 1 (62, 22): Modern 2-Story Villa with Textured Stucco, Cedar Slats, Balcony Pergola & Planter
    // ------------------------------------------------------------
    const makeModernWallTex = () => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#ece6db';
      ctx.fillRect(0, 0, 128, 128);
      // Subtle architectural precast concrete / stucco panel grooves
      ctx.fillStyle = 'rgba(120, 113, 102, 0.22)';
      for (let y = 0; y < 128; y += 32) {
        ctx.fillRect(0, y, 128, 2);
      }
      for (let x = 0; x < 128; x += 64) {
        ctx.fillRect(x, 0, 1, 128);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(2, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const h1 = new THREE.Group();
    h1.position.set(62, 0, 22);
    h1.rotation.y = Math.PI / 2;
    const h1Wall = new THREE.MeshStandardMaterial({ map: makeModernWallTex(), roughness: 0.85, side: THREE.DoubleSide });
    const h1UpperMat = new THREE.MeshStandardMaterial({ map: makeLogTex('#b07d4f', '#c49060', '#7c512d'), roughness: 0.82, side: THREE.DoubleSide });
    const h1Plinth = new THREE.Mesh(new THREE.BoxGeometry(7.2, 0.4, 6.2), trimDarkMat);
    h1Plinth.position.y = 0.2;
    h1Plinth.receiveShadow = true;
    const h1Ground = new THREE.Mesh(new THREE.BoxGeometry(6.6, 3.0, 5.4), h1Wall);
    h1Ground.position.y = 1.9;
    h1Ground.castShadow = true;
    h1Ground.receiveShadow = true;
    const h1Upper = new THREE.Mesh(new THREE.BoxGeometry(4.4, 2.7, 5.4), h1UpperMat);
    h1Upper.position.set(-1.1, 4.75, 0);
    h1Upper.castShadow = true;
    h1Upper.receiveShadow = true;
    // Flush cantilevered modern flat roof slab + parapet fascia + rooftop A/C & solar panel (ZERO roof gap!)
    const h1Roof = new THREE.Mesh(new THREE.BoxGeometry(5.2, 0.28, 6.2), trimDarkMat);
    h1Roof.position.set(-1.1, 6.22, 0);
    h1Roof.castShadow = true;
    const h1RoofCap = new THREE.Mesh(new THREE.BoxGeometry(4.8, 0.16, 5.8), trimWhiteMat);
    h1RoofCap.position.set(-1.1, 6.42, 0);
    const h1AcUnit = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.55, 0.65), trimWhiteMat);
    h1AcUnit.position.set(-2.4, 6.75, -1.6);
    const h1SolarPanel = new THREE.Mesh(
      new THREE.BoxGeometry(1.7, 0.08, 1.15),
      new THREE.MeshStandardMaterial({ color: 0x0f2d6e, roughness: 0.22, metalness: 0.65, side: THREE.DoubleSide })
    );
    h1SolarPanel.position.set(-0.4, 6.72, -1.1);
    h1SolarPanel.rotation.x = 0.28;
    // Modern open cedar pergola shading the upper balcony terrace
    [[3.22, 2.58], [3.22, -2.58]].forEach(([px, pz]) => {
      const pCol = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.82, 0.14), trimDarkMat);
      pCol.position.set(px, 4.81, pz);
      pCol.castShadow = true;
      h1.add(pCol);
    });
    const h1PergolaOuter = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.2, 5.34), trimDarkMat);
    h1PergolaOuter.position.set(3.22, 6.18, 0);
    [-2.58, 2.58].forEach(pz => {
      const sideBeam = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.2, 0.14), trimDarkMat);
      sideBeam.position.set(2.16, 6.18, pz);
      h1.add(sideBeam);
    });
    for (let sz = -2.1; sz <= 2.11; sz += 0.7) {
      const slat = new THREE.Mesh(new THREE.BoxGeometry(2.12, 0.1, 0.12), woodMat);
      slat.position.set(2.16, 6.18, sz);
      slat.castShadow = true;
      h1.add(slat);
    }
    // 3-sided glass + dark metal handrail wraparound upper terrace balcony (x: 1.1..3.25, z: -2.62..+2.62)
    const h1BalcFront = new THREE.Mesh(new THREE.BoxGeometry(2.15, 0.9, 0.08), glassMat);
    h1BalcFront.position.set(2.15, 3.85, 2.6);
    const h1BalcBack = h1BalcFront.clone();
    h1BalcBack.position.z = -2.6;
    const h1BalcSide = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.9, 5.24), glassMat);
    h1BalcSide.position.set(3.22, 3.85, 0);
    const h1HandFront = new THREE.Mesh(new THREE.BoxGeometry(2.22, 0.08, 0.12), trimDarkMat);
    h1HandFront.position.set(2.15, 4.32, 2.6);
    const h1HandBack = h1HandFront.clone();
    h1HandBack.position.z = -2.6;
    const h1HandSide = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.08, 5.32), trimDarkMat);
    h1HandSide.position.set(3.22, 4.32, 0);
    [[3.22, 2.6], [3.22, -2.6], [3.22, 0], [1.15, 2.6], [1.15, -2.6]].forEach(([px, pz]) => {
      const bPost = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.96, 0.1), trimDarkMat);
      bPost.position.set(px, 3.88, pz);
      h1.add(bPost);
    });
    // Framed modern front door + entry canopy + architectural cedar privacy fins + stone planter box
    addDoor(h1, 1.6, 1.45, 2.71, 1.1, 2.1, woodMat, trimDarkMat, 0);
    const h1Canopy = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.14, 0.95), trimDarkMat);
    h1Canopy.position.set(1.5, 2.75, 3.12);
    h1Canopy.castShadow = true;
    [0.52, 0.68, 0.84].forEach(fx => {
      const fin = new THREE.Mesh(new THREE.BoxGeometry(0.07, 2.35, 0.24), woodMat);
      fin.position.set(fx, 1.58, 2.92);
      fin.castShadow = true;
      h1.add(fin);
    });
    const h1Planter = new THREE.Mesh(new THREE.BoxGeometry(2.3, 0.48, 0.48), stoneChimneyMat);
    h1Planter.position.set(-1.6, 0.64, 2.92);
    [-2.25, -1.6, -0.95].forEach(bx => {
      const pb = new THREE.Mesh(new THREE.DodecahedronGeometry(0.34, 0), bushMat);
      pb.position.set(bx, 1.02, 2.92);
      h1.add(pb);
    });
    const h1Step = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.24, 0.75), trimDarkMat);
    h1Step.position.set(1.6, 0.14, 3.15);
    const h1PorchLight = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.28, 0.18), warmLightMat);
    h1PorchLight.position.set(2.42, 2.2, 2.76);
    addWindow(h1, -1.6, 2.05, 2.71, 1.85, 1.2, 0, false);
    addWindow(h1, -1.2, 4.8, 2.71, 2.2, 1.2, 0, false);
    addWindow(h1, -1.2, 4.8, -2.71, 2.0, 1.1, Math.PI, false);
    addWindow(h1, -3.31, 2.0, 0, 1.4, 1.1, -Math.PI / 2, false);
    addWindow(h1, 1.12, 4.55, 0, 1.7, 2.0, Math.PI / 2, false); // Sliding glass balcony door
    h1.add(
      h1Plinth, h1Ground, h1Upper, h1Roof, h1RoofCap, h1AcUnit, h1SolarPanel, h1PergolaOuter,
      h1BalcFront, h1BalcBack, h1BalcSide, h1HandFront, h1HandBack, h1HandSide,
      h1Canopy, h1Planter, h1Step, h1PorchLight
    );
    this.scene.add(h1);

    // ------------------------------------------------------------
    // HOUSE 2 (79, 23): Cozy Terracotta Shingle Gable Cottage with Flush Gables, Grounded Porch & Chimney
    // ------------------------------------------------------------
    const h2 = new THREE.Group();
    h2.position.set(79, 0, 23);
    h2.rotation.y = -Math.PI / 2;
    const h2Wall = new THREE.MeshStandardMaterial({ color: 0xf4f0e8, roughness: 0.86, side: THREE.DoubleSide });
    const h2Base = new THREE.Mesh(new THREE.BoxGeometry(6.6, 0.4, 6.4), trimDarkMat);
    h2Base.position.y = 0.2;
    const h2Body = new THREE.Mesh(new THREE.BoxGeometry(6.0, 3.3, 5.2), h2Wall);
    h2Body.position.set(0, 2.05, -0.4);
    h2Body.castShadow = true;
    h2Body.receiveShadow = true;
    // Flush gable prism spanning z = -3.0 .. +2.2 matching h2Body!
    addGableRoof(h2, 6.0, 5.2, 3.7, 2.1, '#b84a28', '#8a3218', h2Wall, -0.4, 0.5, 0.5, 0.55, 3.3);
    const h2PorchRoof = new THREE.Mesh(
      new THREE.BoxGeometry(5.0, 0.2, 1.75),
      new THREE.MeshStandardMaterial({ map: makeRoofTex('#9a3412', '#71240a'), roughness: 0.78 })
    );
    h2PorchRoof.position.set(0, 3.12, 2.62);
    h2PorchRoof.rotation.x = 0.2;
    h2PorchRoof.castShadow = true;
    // Porch support posts seated squarely on h2Base (y = 0.4 .. 3.0) + grounded front porch railings & steps
    [-2.05, 2.05].forEach(px => {
      const post = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.12, 2.62, 8), trimWhiteMat);
      post.position.set(px, 1.71, 3.04);
      post.castShadow = true;
      h2.add(post);
    });
    addWoodRailing(h2, -2.05, 3.04, -0.78, 3.04, 0.4, 0.78, trimWhiteMat, trimWhiteMat);
    addWoodRailing(h2, 0.78, 3.04, 2.05, 3.04, 0.4, 0.78, trimWhiteMat, trimWhiteMat);
    const h2Step = new THREE.Mesh(new THREE.BoxGeometry(1.55, 0.22, 0.65), trimDarkMat);
    h2Step.position.set(0, 0.11, 3.48);
    addDoor(h2, 0, 1.4, 2.21, 1.05, 2.0, woodMat, trimWhiteMat, 0);
    const h2Light = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.24, 0.16), warmLightMat);
    h2Light.position.set(0.75, 2.15, 2.28);
    const h2Chimney = new THREE.Mesh(new THREE.BoxGeometry(0.8, 2.4, 0.8), new THREE.MeshStandardMaterial({ color: 0x8c3b2b, roughness: 0.9 }));
    h2Chimney.position.set(1.6, 5.15, -1.2);
    const h2ChimCap = new THREE.Mesh(new THREE.BoxGeometry(0.98, 0.16, 0.98), trimDarkMat);
    h2ChimCap.position.set(1.6, 6.38, -1.2);
    addWindow(h2, -1.8, 2.1, 2.22, 1.05, 1.05, 0, true);
    addWindow(h2, 1.8, 2.1, 2.22, 1.05, 1.05, 0, true);
    addWindow(h2, 0, 4.5, 2.22, 0.88, 0.82, 0, false); // Attic window flush on front gable!
    addWindow(h2, 3.02, 2.1, -0.4, 1.2, 1.05, Math.PI / 2, true);
    addWindow(h2, -3.02, 2.1, -0.4, 1.2, 1.05, -Math.PI / 2, true);
    h2.add(h2Base, h2Body, h2PorchRoof, h2Step, h2Light, h2Chimney, h2ChimCap);
    this.scene.add(h2);

    // ------------------------------------------------------------
    // HOUSE 3 (62, 39): L-Shaped Blue-Slate Suburban Home + Flush Gables, Awning Brackets & Mailbox
    // ------------------------------------------------------------
    const h3 = new THREE.Group();
    h3.position.set(62, 0, 39);
    h3.rotation.y = Math.PI / 2;
    const h3Wall = new THREE.MeshStandardMaterial({ color: 0xe6ecf2, roughness: 0.84, side: THREE.DoubleSide });
    const h3PlinthMain = new THREE.Mesh(new THREE.BoxGeometry(6.7, 0.34, 4.7), trimDarkMat);
    h3PlinthMain.position.set(0, 0.17, -0.6);
    const h3PlinthWing = new THREE.Mesh(new THREE.BoxGeometry(3.5, 0.34, 3.9), trimDarkMat);
    h3PlinthWing.position.set(-1.6, 0.17, 2.6);
    const h3Main = new THREE.Mesh(new THREE.BoxGeometry(6.4, 3.2, 4.4), h3Wall);
    h3Main.position.set(0, 1.94, -0.6);
    h3Main.castShadow = true;
    h3Main.receiveShadow = true;
    const h3Wing = new THREE.Mesh(new THREE.BoxGeometry(3.2, 2.86, 3.6), h3Wall);
    h3Wing.position.set(-1.6, 1.77, 2.6);
    h3Wing.castShadow = true;
    h3Wing.receiveShadow = true;
    // Flush gable prism spanning z = -2.8 .. +1.6 matching h3Main (top y = 3.54)!
    addGableRoof(h3, 6.4, 4.4, 3.54, 1.9, '#1e4068', '#142c4a', h3Wall, -0.6, 0.5, 0.5, 0.52, 3.2);
    const h3WingRoof = new THREE.Mesh(
      new THREE.ConeGeometry(2.85, 1.85, 4),
      new THREE.MeshStandardMaterial({ map: makeRoofTex('#1e4068', '#142c4a'), roughness: 0.74 })
    );
    h3WingRoof.position.set(-1.6, 4.06, 2.6);
    h3WingRoof.rotation.y = Math.PI / 4;
    h3WingRoof.scale.set(1.0, 1.0, 1.12);
    h3WingRoof.castShadow = true;
    addDoor(h3, 1.5, 1.34, 1.61, 1.0, 2.0, trimDarkMat, trimWhiteMat, 0);
    const h3Step = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.24, 0.72), trimDarkMat);
    h3Step.position.set(1.5, 0.12, 2.02);
    const h3Awning = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.14, 0.95), trimDarkMat);
    h3Awning.position.set(1.5, 2.58, 2.02);
    h3Awning.rotation.x = 0.22;
    // Support brackets anchoring the door awning cleanly to the wall
    [0.78, 2.22].forEach(bx => {
      const brk = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.55, 0.08), trimDarkMat);
      brk.position.set(bx, 2.32, 1.9);
      brk.rotation.x = 0.72;
      h3.add(brk);
    });
    // Cute roadside mailbox beside entry path
    const mbPost = new THREE.Mesh(new THREE.BoxGeometry(0.1, 1.05, 0.1), woodMat);
    mbPost.position.set(2.9, 0.52, 3.5);
    const mbBox = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.24, 0.44), new THREE.MeshStandardMaterial({ color: 0x1d4ed8, roughness: 0.45 }));
    mbBox.position.set(2.9, 1.12, 3.5);
    addWindow(h3, -1.6, 1.9, 4.42, 1.3, 1.05, 0, true);
    addWindow(h3, 1.6, 2.1, -2.82, 1.5, 1.05, Math.PI, true);
    addWindow(h3, 3.22, 2.1, -0.6, 1.2, 1.05, Math.PI / 2, true);
    [0.35, 2.65].forEach(bx => {
      const b = new THREE.Mesh(new THREE.DodecahedronGeometry(0.55, 0), bushMat);
      b.position.set(bx, 0.45, 2.05);
      h3.add(b);
    });
    h3.add(h3PlinthMain, h3PlinthWing, h3Main, h3Wing, h3WingRoof, h3Step, h3Awning, mbPost, mbBox);
    this.scene.add(h3);

    // ------------------------------------------------------------
    // HOUSE 4 (80, 40): Alpine Timber Log Cabin with Cedar Logs, Flush Gables, Porch Pillars, Deck Railing & Chimney
    // ------------------------------------------------------------
    const h4 = new THREE.Group();
    h4.position.set(80, 0, 40);
    h4.rotation.y = -Math.PI / 2;
    const h4Wall = new THREE.MeshStandardMaterial({ map: makeLogTex(), roughness: 0.9, side: THREE.DoubleSide });
    const h4Deck = new THREE.Mesh(new THREE.BoxGeometry(6.0, 0.45, 6.8), woodMat);
    h4Deck.position.y = 0.225;
    h4Deck.receiveShadow = true;
    const h4Body = new THREE.Mesh(new THREE.BoxGeometry(5.4, 3.0, 5.2), h4Wall);
    h4Body.position.set(0, 1.95, -0.4);
    h4Body.castShadow = true;
    h4Body.receiveShadow = true;

    // 4 Notched dark-timber corner posts framing the log cabin walls
    [[-2.7, -3.0], [2.7, -3.0], [-2.7, 2.2], [2.7, 2.2]].forEach(([cx, cz]) => {
      const cLog = new THREE.Mesh(new THREE.BoxGeometry(0.24, 3.05, 0.24), woodMat);
      cLog.position.set(cx, 1.95, cz);
      cLog.castShadow = true;
      h4.add(cLog);
    });

    // Watertight steep alpine roof: gable prism 100% flush with h4Body (z = -3.0 .. +2.2), roof overhangs front deck to z = +3.65!
    addGableRoof(h4, 5.4, 5.2, 3.45, 2.55, '#1c4a32', '#123222', h4Wall, -0.4, 1.45, 0.55, 0.58, 3.0);

    // 2 Stout front porch support pillars + header beam + angled knee braces supporting the overhanging roof
    [-2.65, 2.65].forEach(px => {
      const pillar = new THREE.Mesh(new THREE.BoxGeometry(0.22, 3.0, 0.22), woodMat);
      pillar.position.set(px, 1.95, 3.22);
      pillar.castShadow = true;
      const brace = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.72, 0.14), woodMat);
      brace.position.set(px > 0 ? px - 0.28 : px + 0.28, 3.12, 3.22);
      brace.rotation.z = px > 0 ? -0.78 : 0.78;
      h4.add(pillar, brace);
    });
    const h4HeaderBeam = new THREE.Mesh(new THREE.BoxGeometry(5.55, 0.22, 0.22), woodMat);
    h4HeaderBeam.position.set(0, 3.38, 3.22);
    h4HeaderBeam.castShadow = true;

    // Grounded wooden front deck railings with top/bottom rails & vertical balusters (ZERO floating rails!)
    addWoodRailing(h4, -2.65, 3.22, -1.92, 3.22, 0.45, 0.82, lightWoodMat, woodMat); // Left of steps
    addWoodRailing(h4, -0.25, 3.22, 2.65, 3.22, 0.45, 0.82, lightWoodMat, woodMat);  // Main front railing
    addWoodRailing(h4, 2.65, 2.22, 2.65, 3.22, 0.45, 0.82, lightWoodMat, woodMat);   // Right porch side return
    addWoodRailing(h4, -2.65, 2.22, -2.65, 3.22, 0.45, 0.82, lightWoodMat, woodMat); // Left porch side return

    // Front wooden porch steps descending cleanly from deck (y = 0.45) to grass (y = 0)
    const h4Step1 = new THREE.Mesh(new THREE.BoxGeometry(1.68, 0.28, 0.42), woodMat);
    h4Step1.position.set(-1.08, 0.14, 3.58);
    const h4Step2 = new THREE.Mesh(new THREE.BoxGeometry(1.68, 0.15, 0.4), lightWoodMat);
    h4Step2.position.set(-1.08, 0.075, 3.94);

    // Framed panelled cabin door + warm porch lantern + stone fireplace chimney + stacked firewood pile
    addDoor(h4, -1.1, 1.45, 2.21, 1.05, 2.0, lightWoodMat, woodMat, 0);
    const h4Lantern = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.28, 0.18), warmLightMat);
    h4Lantern.position.set(-0.32, 2.15, 2.28);
    const h4Chimney = new THREE.Mesh(new THREE.BoxGeometry(0.92, 5.9, 0.92), stoneChimneyMat);
    h4Chimney.position.set(-1.75, 3.4, -1.85);
    h4Chimney.castShadow = true;
    const h4ChimCap = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.16, 1.1), trimDarkMat);
    h4ChimCap.position.set(-1.75, 6.4, -1.85);

    // Stacked firewood logs resting cleanly on the right porch deck
    for (let row = 0; row < 3; row++) {
      for (let col = 0; col < 4 - row; col++) {
        const log = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 0.62, 6), woodMat);
        log.rotation.x = Math.PI / 2;
        log.position.set(1.75 + col * 0.17 + row * 0.085, 0.53 + row * 0.14, 2.55);
        h4.add(log);
      }
    }

    addWindow(h4, 1.1, 1.9, 2.22, 1.4, 1.15, 0, true);
    addWindow(h4, 0, 4.35, 2.22, 1.0, 0.85, 0, false); // Attic window flush & visible on log front gable!
    addWindow(h4, 2.72, 1.9, -0.4, 1.2, 1.05, Math.PI / 2, true);
    addWindow(h4, -2.72, 1.9, -0.4, 1.2, 1.05, -Math.PI / 2, true);
    h4.add(h4Deck, h4Body, h4HeaderBeam, h4Step1, h4Step2, h4Lantern, h4Chimney, h4ChimCap);
    this.scene.add(h4);

    // ------------------------------------------------------------
    // HOUSE 5 (-38, 85): Classic Red Farm Barn (Vertical Plank Siding, Flush Gables, X-Braced Doors, Silo & Grounded Corral!)
    // ------------------------------------------------------------
    const h5 = new THREE.Group();
    h5.position.set(-38, 0, 85);
    const barnRedMat = new THREE.MeshStandardMaterial({ map: makeBarnPlankTex(), roughness: 0.86, side: THREE.DoubleSide });
    const siloMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.35, metalness: 0.72 });
    const h5Body = new THREE.Mesh(new THREE.BoxGeometry(7.8, 4.5, 6.6), barnRedMat);
    h5Body.position.y = 2.25;
    h5Body.castShadow = true;
    h5Body.receiveShadow = true;
    // Flush gable prism spanning z = -3.3 .. +3.3 matching h5Body!
    addGableRoof(h5, 7.8, 6.6, 4.5, 2.8, '#334155', '#1e293b', barnRedMat, 0, 0.55, 0.55, 0.58, 4.5);

    // White corner trim posts on all 4 corners of the Red Barn
    [[-3.88, -3.28], [3.88, -3.28], [-3.88, 3.28], [3.88, 3.28]].forEach(([bx, bz]) => {
      const cTrim = new THREE.Mesh(new THREE.BoxGeometry(0.22, 4.55, 0.22), trimWhiteMat);
      cTrim.position.set(bx, 2.25, bz);
      h5.add(cTrim);
    });

    // Classic Double Barn Doors with Diagonal White X-Braces
    const barnDoorFrame = new THREE.Mesh(new THREE.BoxGeometry(3.0, 3.0, 0.16), trimWhiteMat);
    barnDoorFrame.position.set(0, 1.5, 3.32);
    const barnDoorPanel = new THREE.Mesh(new THREE.BoxGeometry(2.65, 2.7, 0.19), barnRedMat);
    barnDoorPanel.position.set(0, 1.5, 3.33);
    const xBrace1 = new THREE.Mesh(new THREE.BoxGeometry(0.16, 3.3, 0.22), trimWhiteMat);
    xBrace1.position.set(0, 1.5, 3.35);
    xBrace1.rotation.z = 0.76;
    const xBrace2 = xBrace1.clone();
    xBrace2.rotation.z = -0.76;
    const centerSplit = new THREE.Mesh(new THREE.BoxGeometry(0.12, 2.8, 0.22), trimWhiteMat);
    centerSplit.position.set(0, 1.5, 3.35);

    // Hayloft Window + Protruding Wooden Pulley Hoist Beam at Peak
    addWindow(h5, 0, 5.15, 3.32, 1.25, 1.25, 0, false);
    addWindow(h5, -3.92, 2.3, -1.2, 1.1, 1.0, -Math.PI / 2, false);
    addWindow(h5, -3.92, 2.3, 1.2, 1.1, 1.0, -Math.PI / 2, false);
    const hoistBeam = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.24, 1.8), woodMat);
    hoistBeam.position.set(0, 6.75, 3.9);

    // Grain Silo beside Barn with Steel Reinforcement Bands + Vertical Climb Ladder
    const siloBody = new THREE.Mesh(new THREE.CylinderGeometry(1.65, 1.65, 7.2, 16), siloMat);
    siloBody.position.set(5.7, 3.6, -0.6);
    siloBody.castShadow = true;
    const siloDome = new THREE.Mesh(new THREE.SphereGeometry(1.65, 16, 10, 0, Math.PI * 2, 0, Math.PI / 2), siloMat);
    siloDome.position.set(5.7, 7.2, -0.6);
    [1.6, 3.6, 5.6].forEach(ry => {
      const band = new THREE.Mesh(new THREE.TorusGeometry(1.68, 0.06, 6, 18), trimDarkMat);
      band.rotation.x = Math.PI / 2;
      band.position.set(5.7, ry, -0.6);
      h5.add(band);
    });
    const ladder = new THREE.Mesh(new THREE.BoxGeometry(0.45, 7.0, 0.14), trimDarkMat);
    ladder.position.set(5.7, 3.6, 1.08);

    // Grounded wooden corral fence beside the Red Barn (2 horizontal rails + grounded fence posts!)
    const addCorralRun = (x1, z1, x2, z2) => {
      const len = Math.hypot(x2 - x1, z2 - z1);
      const ang = Math.atan2(x2 - x1, z2 - z1);
      const cg = new THREE.Group();
      cg.position.set((x1 + x2) / 2, 0, (z1 + z2) / 2);
      cg.rotation.y = ang;
      [0.42, 0.92].forEach(ry => {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, len), woodMat);
        rail.position.y = ry;
        cg.add(rail);
      });
      const posts = Math.max(2, Math.round(len / 1.8) + 1);
      for (let i = 0; i < posts; i++) {
        const p = new THREE.Mesh(new THREE.BoxGeometry(0.16, 1.15, 0.16), woodMat);
        p.position.set(0, 0.575, -len / 2 + (i / (posts - 1)) * len);
        p.castShadow = true;
        cg.add(p);
      }
      h5.add(cg);
    };
    addCorralRun(-3.9, 2.8, -11.2, 2.8);
    addCorralRun(-3.9, -2.8, -11.2, -2.8);
    addCorralRun(-11.2, -2.8, -11.2, 2.8);

    h5.add(h5Body, barnDoorFrame, barnDoorPanel, xBrace1, xBrace2, centerSplit, hoistBeam, siloBody, siloDome, ladder);
    this.scene.add(h5);

    // Exact rotated world-space solid colliders for all 5 Village Houses & Silo (blocks both Drone & Camera!)
    this.addBoxCollider(62.2, 3.25, 22.0, 6.7, 6.5, 7.3);  // House 1 (Modern 2-story + pergola + canopy)
    this.addBoxCollider(78.85, 3.05, 23.0, 6.8, 6.1, 6.8); // House 2 (Terracotta cottage + porch)
    this.addBoxCollider(61.4, 2.85, 39.0, 5.2, 5.7, 7.0);  // House 3A (L-shaped main block)
    this.addBoxCollider(64.6, 2.5, 40.6, 4.1, 5.0, 3.8);   // House 3B (L-shaped front wing)
    this.addBoxCollider(79.9, 3.15, 40.0, 7.4, 6.3, 6.7);  // House 4 (Alpine log cabin + porch roof)
    this.addBoxCollider(-38.0, 3.8, 85.0, 8.8, 7.6, 7.6);  // House 5 (Red farm barn + overhanging eaves)
    this.addCylCollider(-32.3, 84.4, 1.85, 0, 8.8);        // Grain silo beside barn
  }

  buildLake() {
    // Richly Detailed Expanded Twin-Basin Crystal Alpine Lake (Main West Basin + Natural Bridge Strait + East Lagoon Bay!)
    const lakeGroup = new THREE.Group();
    lakeGroup.position.set(0, 0, 0); // World-space coordinates so shore, pier, bridge & footpaths align to 0.01m!

    // Smooth closed Catmull-Rom shoreline defining the larger Main Lake + pinched Strait + scenic East Lagoon Bay (~4,466 sq m!)
    const lakeCtrl = [
      [-104, -84], // Far West cove
      [ -96, -62], // SW shore
      [ -74, -53], // South-West shore (23m North of West Wheat Field)
      [ -50, -56], // South Marina Bay (T-Pier extends straight North into deep Main Basin)
      [ -33, -72], // South Bridge Headland (Arched Bridge South Abutment)
      [ -17, -68], // East Lagoon South cove
      [  -4, -81], // East Lagoon Far East shore (16m West of Runway x=12!)
      [  -6, -102],// East Lagoon NE shore
      [ -20, -111],// East Lagoon North shore
      [ -38, -98], // North Bridge Headland (Arched Bridge North Abutment)
      [ -60, -112],// North-Central deep bay
      [ -86, -110],// NW Lighthouse shore
      [-102, -98]  // Far NW shore
    ];
    const sampleClosedSpline = (ctrl, segsPerCtrl = 10) => {
      const n = ctrl.length;
      const out = [];
      for (let i = 0; i < n; i++) {
        const p0 = ctrl[(i - 1 + n) % n];
        const p1 = ctrl[i];
        const p2 = ctrl[(i + 1) % n];
        const p3 = ctrl[(i + 2) % n];
        for (let s = 0; s < segsPerCtrl; s++) {
          const t = s / segsPerCtrl;
          const t2 = t * t;
          const t3 = t2 * t;
          const x = 0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3);
          const z = 0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3);
          out.push([x, z]);
        }
      }
      return out;
    };
    const baseShore = sampleClosedSpline(lakeCtrl, 10);

    // Outward normal offset with optional gentle Laplacian smoothing (guarantees ZERO self-intersections on all 6 water/bank layers!)
    const offsetLakeContour = (pts, pad = 0, smoothPasses = 0) => {
      const n = pts.length;
      let area = 0;
      for (let i = 0; i < n; i++) {
        const nxPt = pts[(i + 1) % n];
        area += pts[i][0] * nxPt[1] - nxPt[0] * pts[i][1];
      }
      const s = area < 0 ? 1 : -1;
      let out = [];
      for (let i = 0; i < n; i++) {
        const prev = pts[(i - 1 + n) % n];
        const curr = pts[i];
        const next = pts[(i + 1) % n];
        const dx1 = curr[0] - prev[0], dz1 = curr[1] - prev[1], l1 = Math.hypot(dx1, dz1) || 1;
        const dx2 = next[0] - curr[0], dz2 = next[1] - curr[1], l2 = Math.hypot(dx2, dz2) || 1;
        const nx = s * (-dz1 / l1 - dz2 / l2);
        const nz = s * (dx1 / l1 + dx2 / l2);
        const nl = Math.hypot(nx, nz) || 1;
        out.push([curr[0] + (nx / nl) * pad, curr[1] + (nz / nl) * pad]);
      }
      for (let p = 0; p < smoothPasses; p++) {
        const nxt = [];
        for (let i = 0; i < n; i++) {
          const a = out[(i - 1 + n) % n], b = out[i], c = out[(i + 1) % n];
          nxt.push([a[0] * 0.25 + b[0] * 0.5 + c[0] * 0.25, a[1] * 0.25 + b[1] * 0.5 + c[1] * 0.25]);
        }
        out = nxt;
      }
      return out;
    };

    const makeLakeShape = (pad = 0, smoothPasses = 0) => {
      const poly = offsetLakeContour(baseShore, pad, smoothPasses);
      const pts = poly.map(([wx, wz]) => new THREE.Vector2(wx, -wz));
      const shape = new THREE.Shape(pts);
      const geo = new THREE.ShapeGeometry(shape);
      const uv = geo.attributes.uv;
      for (let i = 0; i < uv.count; i++) {
        uv.setXY(i, uv.getX(i) * 0.045, uv.getY(i) * 0.045);
      }
      return geo;
    };

    // Procedural earthy sand & river-pebble shoreline texture (replaces flat bright white band!)
    const makeShoreTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#9a8663';
      ctx.fillRect(0, 0, 256, 256);
      for (let i = 0; i < 650; i++) {
        const x = (i * 73 + (i % 13) * 17) % 256;
        const y = (i * 41 + (i % 19) * 23) % 256;
        const r = 1.5 + (i % 5);
        const shades = ['#7d6c4f', '#6b5c44', '#b19d79', '#5a5347', '#8a7a5d'];
        ctx.fillStyle = shades[i % shades.length];
        ctx.beginPath();
        ctx.ellipse(x, y, r * 1.2, r * 0.85, (i % 7) * 0.4, 0, Math.PI * 2);
        ctx.fill();
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(4, 4);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural weathered wooden marina deck planks texture
    const makePierPlankTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#5c3a21';
      ctx.fillRect(0, 0, 256, 256);
      const plankW = 16;
      for (let x = 0; x < 256; x += plankW) {
        const shade = ((x / plankW) % 3 === 0) ? '#7a4d2a' : (((x / plankW) % 2 === 0) ? '#6b4223' : '#865732');
        ctx.fillStyle = shade;
        ctx.fillRect(x + 1, 0, plankW - 2, 256);
        // Subtle wood grain & nail heads
        ctx.fillStyle = 'rgba(35, 18, 8, 0.35)';
        ctx.fillRect(x, 0, 1.5, 256);
        ctx.fillStyle = '#292524';
        ctx.fillRect(x + 4, 18, 2, 2);
        ctx.fillRect(x + 10, 18, 2, 2);
        ctx.fillRect(x + 4, 236, 2, 2);
        ctx.fillRect(x + 10, 236, 2, 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(5, 1.2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // 1. Natural Textured Sandy-Pebble Bank + Wet-Sand Waterline Rim
    const bankMat = new THREE.MeshStandardMaterial({ map: makeShoreTex(), roughness: 0.94, metalness: 0.0 });
    const bankMesh = new THREE.Mesh(makeLakeShape(3.6, 0), bankMat);
    bankMesh.rotation.x = -Math.PI / 2;
    bankMesh.position.y = 0.022;
    bankMesh.receiveShadow = true;
    lakeGroup.add(bankMesh);

    const wetSandMat = new THREE.MeshStandardMaterial({ map: makeShoreTex(), color: 0x6b5d47, roughness: 0.72, metalness: 0.06 });
    const wetSandMesh = new THREE.Mesh(makeLakeShape(1.15, 0), wetSandMat);
    wetSandMesh.rotation.x = -Math.PI / 2;
    wetSandMesh.position.y = 0.036;
    wetSandMesh.receiveShadow = true;
    lakeGroup.add(wetSandMesh);

    // 2. Multi-Layered Crystal Alpine Water (Shallow Turquoise Edge + Shimmering Cerulean + Deep Cross-Wave Core!)
    this.waterTex = this.createWaterTexture();
    this.waterBumpTex = this.createWaterBumpTexture();
    this.waterTex2 = this.createWaterTexture();
    this.waterTex2.repeat.set(7, 7);

    // Soft translucent shoreline wave-lap foam rim
    const foamMat = new THREE.MeshStandardMaterial({
      color: 0xdbeafe,
      roughness: 0.45,
      transparent: true,
      opacity: 0.36
    });
    const foamMesh = new THREE.Mesh(makeLakeShape(0.32, 0), foamMat);
    foamMesh.rotation.x = -Math.PI / 2;
    foamMesh.position.y = 0.048;
    lakeGroup.add(foamMesh);

    // Shallow crystal turquoise shoreline shelf
    const shallowMat = new THREE.MeshStandardMaterial({
      map: this.waterTex,
      bumpMap: this.waterBumpTex,
      bumpScale: 0.25,
      color: 0x22b8cf,
      roughness: 0.11,
      metalness: 0.52
    });
    const shallowMesh = new THREE.Mesh(makeLakeShape(0.0, 0), shallowMat);
    shallowMesh.rotation.x = -Math.PI / 2;
    shallowMesh.position.y = 0.062;
    shallowMesh.receiveShadow = true;
    lakeGroup.add(shallowMesh);

    // Mid-depth sparkling cerulean lake water (smoothly fills BOTH Main Basin & East Lagoon across the Bridge Strait!)
    const midWaterMat = new THREE.MeshStandardMaterial({
      map: this.waterTex,
      bumpMap: this.waterBumpTex,
      bumpScale: 0.34,
      color: 0x0b79ad,
      roughness: 0.07,
      metalness: 0.74,
      transparent: true,
      opacity: 0.88
    });
    const midWaterMesh = new THREE.Mesh(makeLakeShape(-3.4, 1), midWaterMat);
    midWaterMesh.rotation.x = -Math.PI / 2;
    midWaterMesh.position.y = 0.08;
    midWaterMesh.receiveShadow = true;
    lakeGroup.add(midWaterMesh);

    // Deep alpine sapphire core with cross-drifting shimmer layer (passes deep under the Grand Strait Bridge!)
    const deepWaterMat = new THREE.MeshStandardMaterial({
      map: this.waterTex2,
      bumpMap: this.waterBumpTex,
      bumpScale: 0.38,
      color: 0x064675,
      roughness: 0.05,
      metalness: 0.82,
      transparent: true,
      opacity: 0.62
    });
    const deepWaterMesh = new THREE.Mesh(makeLakeShape(-7.8, 6), deepWaterMat);
    deepWaterMesh.rotation.x = -Math.PI / 2;
    deepWaterMesh.position.y = 0.098;
    deepWaterMesh.receiveShadow = true;
    lakeGroup.add(deepWaterMesh);

    // 3. Natural Shoreline Details: River Boulders, Cattail Reeds, Water Lily Pads & Mooring Buoys!
    const rockMat = new THREE.MeshStandardMaterial({ color: 0x6b7280, roughness: 0.92, flatShading: true });
    const mossRockMat = new THREE.MeshStandardMaterial({ color: 0x5c6f58, roughness: 0.93, flatShading: true });
    const reedStemMat = new THREE.MeshStandardMaterial({ color: 0x2f6b2e, roughness: 0.85 });
    const cattailMat = new THREE.MeshStandardMaterial({ color: 0x4a2c15, roughness: 0.9 });
    const lilyMat = new THREE.MeshStandardMaterial({ color: 0x227a36, roughness: 0.72, side: THREE.DoubleSide });
    const lotusPinkMat = new THREE.MeshStandardMaterial({ color: 0xf472b6, roughness: 0.6 });
    const lotusWhiteMat = new THREE.MeshStandardMaterial({ color: 0xfef3c7, roughness: 0.6 });

    // 58 Shoreline river boulders & mossy pebble clusters around BOTH basins (keeping Pier & Bridge abutments clear!)
    const rockRingInner = offsetLakeContour(baseShore, 0.75, 0);
    const rockRingOuter = offsetLakeContour(baseShore, 2.35, 0);
    for (let i = 0; i < 58; i++) {
      const idx = Math.floor((i / 58) * baseShore.length) % baseShore.length;
      const mix = (i % 5) / 4;
      const bx = rockRingInner[idx][0] + (rockRingOuter[idx][0] - rockRingInner[idx][0]) * mix;
      const bz = rockRingInner[idx][1] + (rockRingOuter[idx][1] - rockRingInner[idx][1]) * mix;
      if (Math.hypot(bx + 54, bz + 54) < 6.5) continue; // Keep marina pier approach clear
      if (Math.hypot(bx + 33, bz + 71) < 5.8 || Math.hypot(bx + 38, bz + 99) < 5.8) continue; // Keep bridge headlands clear

      const s = 0.45 + ((i * 19) % 10) * 0.11;
      const rock = new THREE.Mesh(new THREE.DodecahedronGeometry(s, 0), (i % 3 === 0) ? mossRockMat : rockMat);
      rock.position.set(bx, s * 0.36, bz);
      rock.scale.set(1.25, 0.62, 1.05);
      rock.rotation.set(i * 0.4, i * 1.1, i * 0.2);
      rock.castShadow = true;
      rock.receiveShadow = true;
      lakeGroup.add(rock);
    }

    // 38 Clusters of lakeside cattail reeds framing the Main Basin & East Lagoon shores
    const reedRing = offsetLakeContour(baseShore, 0.35, 0);
    for (let i = 0; i < 38; i++) {
      const idx = Math.floor(((i + 0.35) / 38) * baseShore.length) % baseShore.length;
      const rx = reedRing[idx][0];
      const rz = reedRing[idx][1];
      if (Math.hypot(rx + 54, rz + 54) < 6.2) continue;
      if (Math.hypot(rx + 33, rz + 71) < 5.2 || Math.hypot(rx + 38, rz + 99) < 5.2) continue;

      const tuft = new THREE.Group();
      tuft.position.set(rx, 0.05, rz);
      for (let k = 0; k < 5; k++) {
        const ang = (k / 5) * Math.PI * 2 + i;
        const h = 1.05 + (k % 3) * 0.28;
        const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.04, h, 5), reedStemMat);
        stem.position.set(Math.cos(ang) * 0.28, h / 2, Math.sin(ang) * 0.28);
        stem.rotation.z = (k - 2) * 0.08;
        tuft.add(stem);
        if (k % 2 === 0) {
          const head = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.26, 6), cattailMat);
          head.position.set(Math.cos(ang) * 0.28, h - 0.16, Math.sin(ang) * 0.28);
          tuft.add(head);
        }
      }
      lakeGroup.add(tuft);
    }

    // 22 Floating water lily pads + delicate lotus blossoms across calm coves in BOTH Main Basin & East Lagoon Bay!
    [
      [-86, -98], [-88.5, -96.2], [-84.2, -100.4], [-87.4, -101.0], [-90.0, -98.5], [-83.0, -97.6],
      [-56, -104], [-58.2, -102.5], [-54.4, -105.6], [-57.0, -106.2],
      [-91, -72], [-89.2, -70.4], [-92.5, -74.0], [-87.8, -68.8],
      [-14, -74], [-11.5, -77.5], [-16.2, -73.0], [-15.0, -103.5], [-18.2, -104.8], [-12.2, -100.5], [-21.5, -104.0], [-10.5, -96.5]
    ].forEach(([lx, lz], idx) => {
      const padR = 0.44 + (idx % 3) * 0.14;
      const pad = new THREE.Mesh(new THREE.CircleGeometry(padR, 10, 0, Math.PI * 1.85), lilyMat);
      pad.rotation.x = -Math.PI / 2;
      pad.rotation.z = idx * 1.3;
      pad.position.set(lx, 0.115, lz);
      lakeGroup.add(pad);
      if (idx % 3 === 0) {
        const flower = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.16, 6), (idx % 6 === 0) ? lotusPinkMat : lotusWhiteMat);
        flower.rotation.x = Math.PI;
        flower.position.set(lx, 0.19, lz);
        lakeGroup.add(flower);
      }
    });

    // 3 Floating red-and-white nautical marker buoys bobbing in Main Basin & East Lagoon Bay
    const buoyRedMat = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.42 });
    const buoyWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.45 });
    const darkIronMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.45, metalness: 0.65, side: THREE.DoubleSide });
    const ropeMat = new THREE.MeshStandardMaterial({ color: 0xc2a06c, roughness: 0.92 });
    const warmLampMat = new THREE.MeshBasicMaterial({ color: 0xfef08a });

    this.lakeBoats = [];
    [[-84, -76], [-62, -96], [-16, -84]].forEach(([bx, bz], idx) => {
      const buoy = new THREE.Group();
      buoy.position.set(bx, 0.12, bz);
      const bBase = new THREE.Mesh(new THREE.CylinderGeometry(0.38, 0.44, 0.35, 10), buoyRedMat);
      bBase.position.y = 0.12;
      const bMid = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.36, 0.35, 10), buoyWhiteMat);
      bMid.position.y = 0.45;
      const bTop = new THREE.Mesh(new THREE.ConeGeometry(0.26, 0.42, 10), buoyRedMat);
      bTop.position.y = 0.82;
      const bLight = new THREE.Mesh(new THREE.SphereGeometry(0.08, 6, 6), warmLampMat);
      bLight.position.y = 1.06;
      buoy.add(bBase, bMid, bTop, bLight);
      lakeGroup.add(buoy);
      this.lakeBoats.push({ mesh: buoy, baseY: 0.12, phase: idx * 2.1, ampY: 0.045, ampRoll: 0.08 });
    });

    // =========================================================================
    // 4. DETAILED LAKESIDE T-SHAPED MARINA PIER (Oriented North from South Marina Bay x=-54 into Main Basin!)
    // =========================================================================
    const pierPlankMat = new THREE.MeshStandardMaterial({ map: makePierPlankTex(), roughness: 0.86, side: THREE.DoubleSide });
    const darkTimberMat = new THREE.MeshStandardMaterial({ color: 0x4a2c16, roughness: 0.9, side: THREE.DoubleSide });
    const lightTimberMat = new THREE.MeshStandardMaterial({ color: 0x8b5a2b, roughness: 0.85, side: THREE.DoubleSide });
    const pierRoofMat = new THREE.MeshStandardMaterial({ color: 0x1e3a5f, roughness: 0.72, side: THREE.DoubleSide });

    const pierGrp = new THREE.Group();
    pierGrp.position.set(-50, 0, -92.5); // Rotated -PI/2 -> centerline x = -54, ramp z = -52.3 (shore), T-head z = -72.9 (deep water!)
    pierGrp.rotation.y = -Math.PI / 2;
    // Main walkway deck extending cleanly from East shore (x=38.2) out into deep water (x=20.6) at z=4.0
    const mainWalk = new THREE.Mesh(new THREE.BoxGeometry(17.6, 0.22, 3.3), pierPlankMat);
    mainWalk.position.set(29.4, 0.56, 4.0);
    mainWalk.castShadow = true;
    mainWalk.receiveShadow = true;
    pierGrp.add(mainWalk);

    // Side structural timber curb stringers along main walkway
    [2.32, 5.68].forEach((cz) => {
      const curb = new THREE.Mesh(new THREE.BoxGeometry(17.7, 0.28, 0.16), darkTimberMat);
      curb.position.set(29.4, 0.58, cz);
      curb.castShadow = true;
      pierGrp.add(curb);
    });

    // Shore wooden ramp stepping smoothly from sandy bank up onto pier deck
    const ramp = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.18, 3.2), pierPlankMat);
    ramp.position.set(39.1, 0.3, 4.0);
    ramp.rotation.z = 0.22;
    ramp.receiveShadow = true;
    pierGrp.add(ramp);

    // Wide T-head docking platform at water end (x = 17.8 .. 21.4, z = -0.5 .. +8.5)
    const tDeck = new THREE.Mesh(new THREE.BoxGeometry(3.6, 0.24, 9.0), pierPlankMat);
    tDeck.position.set(19.6, 0.56, 4.0);
    tDeck.castShadow = true;
    tDeck.receiveShadow = true;
    pierGrp.add(tDeck);

    // T-head outer edge beams + black rubber boat fenders on West docking side (x = 17.75)
    [17.8, 21.4].forEach((ex) => {
      const eBeam = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.28, 9.1), darkTimberMat);
      eBeam.position.set(ex, 0.58, 4.0);
      pierGrp.add(eBeam);
    });
    [0.6, 2.2, 3.8, 5.4, 7.2].forEach((fz) => {
      const fender = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.55, 8), darkIronMat);
      fender.position.set(17.68, 0.45, fz);
      pierGrp.add(fender);
    });

    // 14 Heavy vertical timber pilings rising above deck height with iron bands & mooring rope wraps
    const pilingCoords = [
      [37.6, 2.25], [37.6, 5.75], [33.6, 2.25], [33.6, 5.75],
      [29.4, 2.25], [29.4, 5.75], [25.2, 2.25], [25.2, 5.75],
      [21.4, -0.45], [17.8, -0.45], [17.8, 2.4], [17.8, 5.6], [17.8, 8.45], [21.4, 8.45]
    ];
    pilingCoords.forEach(([px, pz], idx) => {
      const h = (idx >= 8) ? 1.52 : 1.38;
      const pile = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.2, h, 10), darkTimberMat);
      pile.position.set(px, h / 2 - 0.05, pz);
      pile.castShadow = true;
      const ironCap = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.1, 10), darkIronMat);
      ironCap.position.set(px, h - 0.06, pz);
      pierGrp.add(pile, ironCap);
      if (idx >= 8 || idx % 2 === 0) {
        const ropeWrap = new THREE.Mesh(new THREE.TorusGeometry(0.2, 0.05, 6, 12), ropeMat);
        ropeWrap.rotation.x = Math.PI / 2;
        ropeWrap.position.set(px, 0.98, pz);
        pierGrp.add(ropeWrap);
      }
    });

    // Grounded wooden safety handrail along North side of walkway (z = 2.32, x = 21.4 .. 37.6)
    const railTop = new THREE.Mesh(new THREE.BoxGeometry(16.2, 0.1, 0.1), lightTimberMat);
    railTop.position.set(29.5, 1.38, 2.32);
    const railMid = new THREE.Mesh(new THREE.BoxGeometry(16.2, 0.07, 0.07), lightTimberMat);
    railMid.position.set(29.5, 0.98, 2.32);
    pierGrp.add(railTop, railMid);
    for (let rx = 22.4; rx <= 36.8; rx += 2.05) {
      const rPost = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.82, 0.08), darkTimberMat);
      rPost.position.set(rx, 0.98, 2.32);
      pierGrp.add(rPost);
    }

    // Timber Dockside Shelter Gazebo / Boat-Rental Canopy near shore end (x = 34.4, z = 4.0)
    [[32.8, 2.45], [36.0, 2.45], [32.8, 5.55], [36.0, 5.55]].forEach(([gx, gz]) => {
      const gPost = new THREE.Mesh(new THREE.BoxGeometry(0.16, 2.45, 0.16), darkTimberMat);
      gPost.position.set(gx, 1.88, gz);
      gPost.castShadow = true;
      pierGrp.add(gPost);
    });
    const gazeboRoof = new THREE.Mesh(new THREE.ConeGeometry(2.85, 1.15, 4), pierRoofMat);
    gazeboRoof.position.set(34.4, 3.62, 4.0);
    gazeboRoof.rotation.y = Math.PI / 4;
    gazeboRoof.castShadow = true;
    pierGrp.add(gazeboRoof);

    // 2 Cast-Iron Dockside Lantern Posts with glowing warm bulbs (entrance & T-head corner)
    [[37.6, 5.75], [17.8, 8.45]].forEach(([lx, lz]) => {
      const lPole = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.08, 2.35, 8), darkIronMat);
      lPole.position.set(lx, 2.52, lz);
      const lHead = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.38, 0.32), warmLampMat);
      lHead.position.set(lx, 3.72, lz);
      const lCap = new THREE.Mesh(new THREE.ConeGeometry(0.28, 0.22, 4), darkIronMat);
      lCap.rotation.y = Math.PI / 4;
      lCap.position.set(lx, 4.0, lz);
      pierGrp.add(lPole, lHead, lCap);
    });

    // Marina Props: Wooden Lakeside Bench, Red-and-White Lifebuoy on Post, Wooden Gear Crate & Coiled Rope!
    const benchSeat = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.08, 2.1), lightTimberMat);
    benchSeat.position.set(20.6, 1.05, 6.8);
    const benchBack = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.45, 2.1), lightTimberMat);
    benchBack.position.set(20.88, 1.32, 6.8);
    [5.95, 7.65].forEach((bz) => {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.42, 0.1), darkIronMat);
      leg.position.set(20.6, 0.84, bz);
      pierGrp.add(leg);
    });
    pierGrp.add(benchSeat, benchBack);

    const lifeRing = new THREE.Mesh(new THREE.TorusGeometry(0.34, 0.09, 8, 16), buoyRedMat);
    lifeRing.position.set(21.4, 1.32, 0.05);
    lifeRing.rotation.y = Math.PI / 2;
    const lifeStripe = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.78, 0.16), buoyWhiteMat);
    lifeStripe.position.set(21.4, 1.32, 0.05);
    const tackleCrate = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.58, 0.65), lightTimberMat);
    tackleCrate.position.set(20.5, 0.96, 0.75);
    tackleCrate.castShadow = true;
    const ropeCoil = new THREE.Mesh(new THREE.TorusGeometry(0.26, 0.07, 6, 14), ropeMat);
    ropeCoil.rotation.x = Math.PI / 2;
    ropeCoil.position.set(19.0, 0.72, 7.4);
    pierGrp.add(lifeRing, lifeStripe, tackleCrate, ropeCoil);

    lakeGroup.add(pierGrp);
    // Solid world-space pier colliders (rotated -PI/2 pointing North along x = -54!) so drone can land cleanly on deck
    this.addBoxCollider(-54.0, 0.34, -63.1, 3.4, 0.68, 17.6);
    this.addBoxCollider(-54.0, 0.34, -72.9, 9.2, 0.68, 3.8);

    // =========================================================================
    // 5. SCULPTED CURVED BOATS (Moored Wooden Lake Skiff beside T-Pier + Graceful Sailboat in Lake!)
    // =========================================================================
    const makeBoatHullShape = (halfBeam, halfLen, inset = 0) => {
      const b = Math.max(0.15, halfBeam - inset);
      const l = Math.max(0.3, halfLen - inset);
      const s = new THREE.Shape();
      s.moveTo(-b * 0.66, -l);
      s.lineTo(b * 0.66, -l);
      s.quadraticCurveTo(b * 1.08, -l * 0.15, b * 0.96, l * 0.32);
      s.quadraticCurveTo(b * 0.76, l * 0.78, 0, l);
      s.quadraticCurveTo(-b * 0.76, l * 0.78, -b * 0.96, l * 0.32);
      s.quadraticCurveTo(-b * 1.08, -l * 0.15, -b * 0.66, -l);
      return s;
    };

    const makeHollowHullGeo = (halfBeam, halfLen, depth, wallT = 0.13) => {
      const outer = makeBoatHullShape(halfBeam, halfLen, 0);
      const innerPts = makeBoatHullShape(halfBeam, halfLen, wallT).getPoints(28);
      outer.holes.push(new THREE.Path(innerPts.reverse()));
      const geo = new THREE.ExtrudeGeometry(outer, { depth, bevelEnabled: false });
      geo.rotateX(-Math.PI / 2); // maps shape Y -> -Z, depth -> +Y
      return geo;
    };

    const makeSolidHullGeo = (halfBeam, halfLen, depth) => {
      const geo = new THREE.ExtrudeGeometry(makeBoatHullShape(halfBeam, halfLen, 0), {
        depth,
        bevelEnabled: true,
        bevelThickness: 0.05,
        bevelSize: 0.04,
        bevelSegments: 2
      });
      geo.rotateX(-Math.PI / 2);
      return geo;
    };

    // BOAT 1: Classic Curved Wooden Skiff / Rowboat moored cleanly alongside North face of T-Pier (-54.0, -76.5)
    const skiff = new THREE.Group();
    skiff.position.set(-54.0, 0.13, -76.5);
    skiff.rotation.y = -Math.PI / 2 + 0.06;
    const navyHullMat = new THREE.MeshStandardMaterial({ color: 0x1e3a5f, roughness: 0.35, metalness: 0.2, side: THREE.DoubleSide });
    const whiteHullMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.38, side: THREE.DoubleSide });
    const mahoganyMat = new THREE.MeshStandardMaterial({ color: 0x6b2f12, roughness: 0.5, side: THREE.DoubleSide });
    const brassMat = new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.25, metalness: 0.85 });

    // Sculpted solid keel bottom + hollow flared upper walls + mahogany gunwale trim
    const skiffKeel = new THREE.Mesh(makeSolidHullGeo(0.86, 2.15, 0.2), navyHullMat);
    skiffKeel.castShadow = true;
    const skiffSides = new THREE.Mesh(makeHollowHullGeo(0.95, 2.28, 0.44, 0.12), whiteHullMat);
    skiffSides.position.y = 0.16;
    skiffSides.castShadow = true;
    const skiffGunwale = new THREE.Mesh(makeHollowHullGeo(0.99, 2.32, 0.08, 0.15), mahoganyMat);
    skiffGunwale.position.y = 0.58;
    // Recessed wooden floorboards + 3 bench thwarts + bow foredeck
    const skiffFloor = new THREE.Mesh(makeSolidHullGeo(0.72, 1.85, 0.06), lightTimberMat);
    skiffFloor.position.y = 0.17;
    const bowDeck = new THREE.Mesh(new THREE.ConeGeometry(0.68, 0.95, 3), mahoganyMat);
    bowDeck.rotation.x = -Math.PI / 2;
    bowDeck.scale.set(1.1, 1.0, 0.08);
    bowDeck.position.set(0, 0.56, -1.72);
    [-1.05, 0.05, 1.15].forEach((sz, si) => {
      const bw = (si === 1) ? 1.72 : 1.38;
      const bench = new THREE.Mesh(new THREE.BoxGeometry(bw, 0.07, 0.38), mahoganyMat);
      bench.position.set(0, 0.44, sz);
      skiff.add(bench);
    });
    // Pair of detailed carved wooden oars with paddle blades resting in brass oarlocks
    [-1, 1].forEach((side) => {
      const oarGrp = new THREE.Group();
      oarGrp.position.set(side * 0.92, 0.64, 0.15);
      oarGrp.rotation.y = side * 0.22;
      oarGrp.rotation.z = side * -0.12;
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.038, 2.55, 8), lightTimberMat);
      shaft.rotation.z = Math.PI / 2;
      shaft.position.x = side * 0.35;
      const blade = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.035, 0.18), buoyRedMat);
      blade.position.x = side * 1.48;
      const lock = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.12, 6), brassMat);
      oarGrp.add(shaft, blade, lock);
      skiff.add(oarGrp);
    });
    // Compact vintage outboard motor mounted on stern transom (z = +2.25)
    const motorHead = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.34, 0.42), darkIronMat);
    motorHead.position.set(0, 0.78, 2.28);
    const motorShaft = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.72, 0.14), darkIronMat);
    motorShaft.position.set(0, 0.32, 2.34);
    const motorTiller = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.62, 6), darkIronMat);
    motorTiller.rotation.x = Math.PI / 2.2;
    motorTiller.position.set(-0.18, 0.74, 1.9);
    const skiffRing = new THREE.Mesh(new THREE.TorusGeometry(0.24, 0.06, 6, 12), buoyRedMat);
    skiffRing.rotation.x = Math.PI / 2;
    skiffRing.position.set(0.32, 0.5, 1.15);

    skiff.add(skiffKeel, skiffSides, skiffGunwale, skiffFloor, bowDeck, motorHead, motorShaft, motorTiller, skiffRing);
    lakeGroup.add(skiff);
    this.lakeBoats.push({ mesh: skiff, baseY: 0.13, phase: 0.4, ampY: 0.035, ampRoll: 0.035 });

    // Taut hemp mooring line connecting skiff bow (-51.9, 0.72, -76.4) to North T-Pier bollard (-52.4, 0.98, -74.7)
    const moorPts = [
      new THREE.Vector3(-51.9, 0.72, -76.4),
      new THREE.Vector3(-52.15, 0.68, -75.55),
      new THREE.Vector3(-52.4, 0.98, -74.7)
    ];
    const moorLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(moorPts),
      new THREE.LineBasicMaterial({ color: 0xc2a06c })
    );
    lakeGroup.add(moorLine);

    // BOAT 2: Classic Lakeside White-and-Navy Sailboat floating gracefully in Main Alpine Basin (-76, -84)
    const sail = new THREE.Group();
    sail.position.set(-76, 0.15, -84);
    sail.rotation.y = 0.65;
    const sailKeel = new THREE.Mesh(makeSolidHullGeo(1.22, 3.1, 0.28), navyHullMat);
    sailKeel.castShadow = true;
    const sailHull = new THREE.Mesh(makeHollowHullGeo(1.34, 3.3, 0.58, 0.14), whiteHullMat);
    sailHull.position.y = 0.22;
    sailHull.castShadow = true;
    const sailDeck = new THREE.Mesh(makeSolidHullGeo(1.26, 3.15, 0.08), lightTimberMat);
    sailDeck.position.y = 0.72;
    // Raised white cabin trunk with dark porthole windows + stern rudder tiller
    const sailCab = new THREE.Mesh(new THREE.BoxGeometry(1.42, 0.46, 2.25), whiteHullMat);
    sailCab.position.set(0, 0.98, -0.35);
    sailCab.castShadow = true;
    [-0.72, 0.72].forEach((px) => {
      [-0.75, 0.05].forEach((pz) => {
        const port = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.08, 8), navyHullMat);
        port.rotation.z = Math.PI / 2;
        port.position.set(px, 1.0, pz);
        sail.add(port);
      });
    });
    // Tapered aluminum mast, spreader bar, horizontal boom & crisp white billowing Mainsail + Jib Sail!
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.095, 7.6, 10), new THREE.MeshStandardMaterial({ color: 0xdbe4ee, roughness: 0.35, metalness: 0.75 }));
    mast.position.set(0, 4.6, -0.45);
    mast.castShadow = true;
    const spreader = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 1.35, 6), darkIronMat);
    spreader.rotation.z = Math.PI / 2;
    spreader.position.set(0, 5.4, -0.45);
    const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.055, 2.95, 8), darkIronMat);
    boom.rotation.x = Math.PI / 2;
    boom.position.set(0, 1.75, 1.0);

    const sailClothMat = new THREE.MeshStandardMaterial({ color: 0xfefefe, roughness: 0.75, side: THREE.DoubleSide });
    const sailStripeMat = new THREE.MeshStandardMaterial({ color: 0x1d4ed8, roughness: 0.7, side: THREE.DoubleSide });
    // Triangular Mainsail behind mast
    const mainShape = new THREE.Shape([
      new THREE.Vector2(0, 0),
      new THREE.Vector2(2.75, 0.08),
      new THREE.Vector2(0.05, 6.2)
    ]);
    const mainSail = new THREE.Mesh(new THREE.ShapeGeometry(mainShape), sailClothMat);
    mainSail.rotation.y = -Math.PI / 2 + 0.14;
    mainSail.position.set(0.04, 1.82, -0.35);
    mainSail.castShadow = true;
    // Navy accent racing band across mainsail
    const mainStripe = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.42, 1.68), sailStripeMat);
    mainStripe.position.set(0.18, 3.6, 0.5);
    mainStripe.rotation.y = 0.14;
    // Front Triangular Jib Sail in front of mast
    const jibShape = new THREE.Shape([
      new THREE.Vector2(0, 0),
      new THREE.Vector2(-2.25, 0.15),
      new THREE.Vector2(-0.04, 5.5)
    ]);
    const jibSail = new THREE.Mesh(new THREE.ShapeGeometry(jibShape), sailClothMat);
    jibSail.rotation.y = -Math.PI / 2 - 0.1;
    jibSail.position.set(-0.03, 1.45, -0.58);
    jibSail.castShadow = true;
    // Red triangular masthead wind pennant
    const pennant = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.65, 3), buoyRedMat);
    pennant.rotation.x = Math.PI / 2;
    pennant.position.set(0, 8.38, -0.15);

    sail.add(sailKeel, sailHull, sailDeck, sailCab, mast, spreader, boom, mainSail, mainStripe, jibSail, pennant);
    lakeGroup.add(sail);
    this.lakeBoats.push({ mesh: sail, baseY: 0.15, phase: 1.7, ampY: 0.04, ampRoll: 0.045 });

    // BOAT 3: Scenic Wooden Lake Canoe drifting peacefully in the East Lagoon Bay (-19.5, -89.5) facing toward the Arched Bridge!
    const canoe = new THREE.Group();
    canoe.position.set(-19.5, 0.12, -89.5);
    canoe.rotation.y = Math.PI / 2 + 0.16;
    const canoeKeel = new THREE.Mesh(makeSolidHullGeo(0.55, 2.25, 0.16), mahoganyMat);
    canoeKeel.castShadow = true;
    const canoeSides = new THREE.Mesh(makeHollowHullGeo(0.62, 2.38, 0.36, 0.09), lightTimberMat);
    canoeSides.position.y = 0.14;
    canoeSides.castShadow = true;
    const canoeTrim = new THREE.Mesh(makeHollowHullGeo(0.66, 2.42, 0.06, 0.11), mahoganyMat);
    canoeTrim.position.y = 0.48;
    [-0.95, 0.05, 1.0].forEach((cz) => {
      const seat = new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.06, 0.28), mahoganyMat);
      seat.position.set(0, 0.36, cz);
      canoe.add(seat);
    });
    const padShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.028, 2.1, 6), lightTimberMat);
    padShaft.rotation.x = Math.PI / 2;
    padShaft.rotation.z = 0.24;
    padShaft.position.set(0.28, 0.54, 0.1);
    const padBlade = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.03, 0.55), buoyRedMat);
    padBlade.position.set(0.05, 0.54, -0.95);
    padBlade.rotation.y = 0.24;
    canoe.add(canoeKeel, canoeSides, canoeTrim, padShaft, padBlade);
    lakeGroup.add(canoe);
    this.lakeBoats.push({ mesh: canoe, baseY: 0.12, phase: 2.9, ampY: 0.032, ampRoll: 0.04 });

    this.scene.add(lakeGroup);
  }

  buildTallTower() {
    // =========================================================================
    // HIGH-DETAIL 45m RED-AND-WHITE 5G CELLULAR & MICROWAVE TELECOM TOWER (MISSION 3 LANDMARK!)
    // (Concrete Piers, Caged Climb Ladder, RF Cable Ice-Bridge, Grounded Platforms, RRU Sector Arrays & Dual Beacons!)
    // =========================================================================
    const tower = new THREE.Group();
    tower.position.set(75, 0, -70);

    // Procedural textures: Gravel/Concrete Compound Pad, Corrugated Shelter Wall, Chain-Link Fence & 5G Warning Sign
    const makeTelecomPadTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#68717c'; ctx.fillRect(0, 0, 256, 256);
      for (let i = 0; i < 1200; i++) {
        const gx = (i * 71 + (i % 13) * 19) % 256;
        const gy = (i * 43 + (i % 17) * 11) % 256;
        ctx.fillStyle = (i % 2 === 0) ? 'rgba(255,255,255,0.08)' : 'rgba(15,23,42,0.16)';
        ctx.fillRect(gx, gy, 3, 2);
      }
      ctx.strokeStyle = '#facc15'; ctx.lineWidth = 8; ctx.strokeRect(8, 8, 240, 240);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeShelterTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#cbd5e1'; ctx.fillRect(0, 0, 128, 128);
      for (let x = 0; x < 128; x += 12) {
        ctx.fillStyle = 'rgba(255,255,255,0.22)'; ctx.fillRect(x, 0, 3, 128);
        ctx.fillStyle = 'rgba(51,65,85,0.22)'; ctx.fillRect(x + 6, 0, 3, 128);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeTelecomSignTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 96;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#0f172a'; ctx.fillRect(0, 0, 256, 96);
      ctx.strokeStyle = '#ef4444'; ctx.lineWidth = 6; ctx.strokeRect(4, 4, 248, 88);
      ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 30px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('5G NODE · 03', 128, 36);
      ctx.fillStyle = '#fde047'; ctx.font = 'bold 17px sans-serif';
      ctx.fillText('CAUTION · HIGH RF RADIATION', 128, 68);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeFenceMeshTex = () => {
      const c = document.createElement('canvas');
      c.width = 64; c.height = 64;
      const ctx = c.getContext('2d');
      ctx.clearRect(0, 0, 64, 64);
      ctx.strokeStyle = 'rgba(203, 213, 225, 0.72)'; ctx.lineWidth = 2;
      for (let i = -64; i <= 128; i += 16) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i + 64, 64); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(i + 64, 0); ctx.lineTo(i, 64); ctx.stroke();
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(8, 2);
      return tex;
    };

    const redSteelMat = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.45, metalness: 0.45 });
    const whiteSteelMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.42, metalness: 0.35 });
    const galvSteelMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.4, metalness: 0.72 });
    const darkSteelMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.55, metalness: 0.65 });
    const concreteMat = new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.88 });
    const shelterWallMat = new THREE.MeshStandardMaterial({ map: makeShelterTex(), roughness: 0.72, side: THREE.DoubleSide });
    const antennaWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf8fafc, roughness: 0.28, metalness: 0.16 });
    const yellowSafetyMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.48 });
    const beaconRedMat = new THREE.MeshBasicMaterial({ color: 0xff1e00 });
    const ledGreenMat = new THREE.MeshBasicMaterial({ color: 0x22c55e });
    const fenceMeshMat = new THREE.MeshBasicMaterial({ map: makeFenceMeshTex(), transparent: true, opacity: 0.82, side: THREE.DoubleSide });

    // Helper: connect any two 3D points cleanly with a steel structural member
    const addBeam = (p1, p2, radius, mat, cast = true) => {
      const dir = new THREE.Vector3().subVectors(p2, p1);
      const len = dir.length();
      if (len < 0.04) return;
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len, 6), mat);
      mesh.position.copy(p1).add(p2).multiplyScalar(0.5);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
      mesh.castShadow = cast;
      tower.add(mesh);
    };

    // 1. Concrete Compound Pad, 4 Corner Anchor Piers, Upgraded Base Station Shelter, RF Cable Ice-Bridge & Standby Generator!
    const pad = new THREE.Mesh(
      new THREE.BoxGeometry(11.8, 0.3, 11.8),
      [
        concreteMat, concreteMat,
        new THREE.MeshStandardMaterial({ map: makeTelecomPadTex(), roughness: 0.9 }),
        concreteMat, concreteMat, concreteMat
      ]
    );
    pad.position.y = 0.15; pad.receiveShadow = true;
    tower.add(pad);

    // 4 Reinforced Concrete Corner Footing Pedestals + Steel Anchor Baseplates supporting Tower Legs!
    const baseHalf = 3.2;
    const topHalf = 0.85;
    const totalH = 44;
    const halfAtY = (y) => baseHalf + (topHalf - baseHalf) * (y / totalH);

    [[-baseHalf, -baseHalf], [baseHalf, -baseHalf], [baseHalf, baseHalf], [-baseHalf, baseHalf]].forEach(([px, pz]) => {
      const pier = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.85, 1.15), concreteMat);
      pier.position.set(px, 0.58, pz); pier.castShadow = true; pier.receiveShadow = true;
      const plate = new THREE.Mesh(new THREE.BoxGeometry(0.82, 0.12, 0.82), darkSteelMat);
      plate.position.set(px, 1.04, pz);
      tower.add(pier, plate);
    });

    // Upgraded Corrugated 5G Base Station Control Shelter (-3.5, 0, 2.2) + Dual Split-AC Units + RF Sign + Door Canopy!
    const shelter = new THREE.Mesh(new THREE.BoxGeometry(3.6, 2.85, 4.4), shelterWallMat);
    shelter.position.set(-3.5, 1.72, 2.2); shelter.castShadow = true; shelter.receiveShadow = true;
    const shelterRoof = new THREE.Mesh(new THREE.BoxGeometry(3.95, 0.24, 4.75), darkSteelMat);
    shelterRoof.position.set(-3.5, 3.24, 2.2);
    const shelterBand = new THREE.Mesh(new THREE.BoxGeometry(3.66, 0.24, 4.46), redSteelMat);
    shelterBand.position.set(-3.5, 2.9, 2.2);
    const shelterDoor = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.05, 1.12), darkSteelMat);
    shelterDoor.position.set(-1.66, 1.35, 2.2);
    const doorCanopy = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.1, 1.45), darkSteelMat);
    doorCanopy.position.set(-1.35, 2.52, 2.2);
    const statusLed = new THREE.Mesh(new THREE.SphereGeometry(0.1, 8, 8), ledGreenMat);
    statusLed.position.set(-1.62, 2.48, 2.88);
    const sSign = new THREE.Mesh(
      new THREE.BoxGeometry(0.1, 0.72, 1.95),
      new THREE.MeshStandardMaterial({ map: makeTelecomSignTex(), roughness: 0.5 })
    );
    sSign.position.set(-1.65, 2.02, 0.65);
    // Twin Side-Mounted Telecom Shelter HVAC Cooling Units
    [1.1, 3.1].forEach((az) => {
      const ac = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.78, 0.92), whiteSteelMat);
      ac.position.set(-5.32, 1.55, az); ac.castShadow = true;
      const fan = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.28, 0.08, 10), darkSteelMat);
      fan.rotation.z = Math.PI / 2; fan.position.set(-5.62, 1.55, az);
      tower.add(ac, fan);
    });
    // Overhead Galvanized RF Coaxial Cable Ice-Bridge linking Shelter straight into Tower Center Spine!
    const bridgeDeck = new THREE.Mesh(new THREE.BoxGeometry(2.5, 0.12, 0.72), galvSteelMat);
    bridgeDeck.position.set(-0.95, 2.72, 0.55);
    [-1.7, -0.35].forEach((bx) => {
      const bLeg = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.055, 2.55, 6), galvSteelMat);
      bLeg.position.set(bx, 1.48, 0.55); tower.add(bLeg);
    });
    // Compact Standby Backup Generator Cabinet tucked cleanly in NE Compound Corner (+3.8, 0, -3.2)
    const genBox = new THREE.Mesh(new THREE.BoxGeometry(2.1, 1.45, 1.3), yellowSafetyMat);
    genBox.position.set(3.85, 1.02, -3.25); genBox.castShadow = true;
    const genCap = new THREE.Mesh(new THREE.BoxGeometry(2.2, 0.14, 1.4), darkSteelMat);
    genCap.position.set(3.85, 1.8, -3.25);
    const genVent = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.65, 1.35), darkSteelMat);
    genVent.position.set(3.85, 1.05, -3.25);
    const genStack = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.07, 1.1, 6), darkSteelMat);
    genStack.position.set(4.65, 2.35, -3.25);
    tower.add(shelter, shelterRoof, shelterBand, shelterDoor, doorCanopy, statusLed, sSign, bridgeDeck, genBox, genCap, genVent, genStack);

    // Grounded Chain-Link Perimeter Security Fence with Angled Top Security Outriggers & Yellow Gate Bollards!
    [[-5.7, -5.7, 5.7, -5.7], [-5.7, -5.7, -5.7, 5.7], [5.7, -5.7, 5.7, 5.7], [-5.7, 5.7, -2.1, 5.7], [2.1, 5.7, 5.7, 5.7]].forEach(([x1, z1, x2, z2]) => {
      const len = Math.hypot(x2 - x1, z2 - z1);
      const ang = Math.atan2(x2 - x1, z2 - z1);
      const fg = new THREE.Group();
      fg.position.set((x1 + x2) / 2, 0.3, (z1 + z2) / 2);
      fg.rotation.y = ang;
      const meshP = new THREE.Mesh(new THREE.PlaneGeometry(len, 1.8), fenceMeshMat);
      meshP.rotation.y = Math.PI / 2; meshP.position.y = 0.92;
      fg.add(meshP);
      [0.12, 0.95, 1.82].forEach((ry) => {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, len), galvSteelMat);
        rail.position.y = ry; fg.add(rail);
      });
      const nP = Math.max(2, Math.round(len / 2.8) + 1);
      for (let i = 0; i < nP; i++) {
        const pz = -len / 2 + (i / (nP - 1)) * len;
        const post = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 1.9, 6), darkSteelMat);
        post.position.set(0, 0.95, pz);
        const arm = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.42, 0.045), galvSteelMat);
        arm.position.set(0.11, 2.02, pz); arm.rotation.z = -0.55;
        fg.add(post, arm);
      }
      tower.add(fg);
    });
    [-2.2, 2.2].forEach((gx) => {
      const bol = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.92, 8), yellowSafetyMat);
      bol.position.set(gx, 0.72, 6.05); tower.add(bol);
    });

    // 2. Open 4-Legged Steel Lattice Truss (6 Red/White Aviation Bands + Plan Diaphragms + Caged Climb Ladder + RF Feeder Spine!)
    const tiers = 6;
    const tierH = totalH / tiers;

    for (let t = 0; t < tiers; t++) {
      const y0 = t * tierH + (t === 0 ? 1.0 : 0);
      const y1 = (t + 1) * tierH;
      const yMid = (y0 + y1) * 0.5;
      const h0 = halfAtY(y0);
      const h1 = halfAtY(y1);
      const hMid = halfAtY(yMid);
      const mat = t % 2 === 0 ? redSteelMat : whiteSteelMat;

      const c0 = [new THREE.Vector3(-h0, y0, -h0), new THREE.Vector3(h0, y0, -h0), new THREE.Vector3(h0, y0, h0), new THREE.Vector3(-h0, y0, h0)];
      const c1 = [new THREE.Vector3(-h1, y1, -h1), new THREE.Vector3(h1, y1, -h1), new THREE.Vector3(h1, y1, h1), new THREE.Vector3(-h1, y1, h1)];
      const cMid = [new THREE.Vector3(-hMid, yMid, -hMid), new THREE.Vector3(hMid, yMid, -hMid), new THREE.Vector3(hMid, yMid, hMid), new THREE.Vector3(-hMid, yMid, hMid)];

      for (let i = 0; i < 4; i++) {
        const nxt = (i + 1) % 4;
        // Heavy corner leg (tapers in steel thickness toward upper tiers)
        addBeam(c0[i], c1[i], 0.18 - t * 0.012, mat, true);
        // Horizontal girders at top & mid-bay for dense authentic telecom lattice silhouette!
        addBeam(c1[i], c1[nxt], 0.1, mat, false);
        addBeam(cMid[i], cMid[nxt], 0.065, mat, false);
        // Double-tier X-bracing (lower sub-bay & upper sub-bay!)
        addBeam(c0[i], cMid[nxt], 0.072, mat, false);
        addBeam(c0[nxt], cMid[i], 0.072, mat, false);
        addBeam(cMid[i], c1[nxt], 0.072, mat, false);
        addBeam(cMid[nxt], c1[i], 0.072, mat, false);
      }
      // Internal horizontal plan-view diagonal cross-brace every 2 tiers
      if (t % 2 === 1) {
        addBeam(c1[0], c1[2], 0.06, darkSteelMat, false);
        addBeam(c1[1], c1[3], 0.06, darkSteelMat, false);
      }
    }

    // Internal Vertical Steel Climbing Ladder with Safety Fall-Arrest Hoops + Bundled Black RF Feeder Cables (y = 1..44m!)
    [-0.24, 0.24].forEach((lx) => {
      const rail = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 43.2, 6), galvSteelMat);
      rail.position.set(lx, 22.6, 0.42); tower.add(rail);
    });
    for (let ry = 1.4; ry <= 43.4; ry += 0.65) {
      const rung = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.035, 0.035), galvSteelMat);
      rung.position.set(0, ry, 0.42); tower.add(rung);
    }
    // Safety fall-arrest hoops every 2.8m along the climbing ladder
    for (let hy = 3.5; hy <= 42.0; hy += 2.8) {
      const hoop = new THREE.Mesh(new THREE.TorusGeometry(0.42, 0.025, 5, 12, Math.PI * 1.45), galvSteelMat);
      hoop.rotation.x = Math.PI / 2; hoop.rotation.z = -Math.PI * 0.22;
      hoop.position.set(0, hy, 0.68);
      tower.add(hoop);
    }
    // 3 Thick Black RF Coaxial Feeder Cable Bundles running vertically alongside the ladder up to the 5G antennas!
    [-0.55, -0.42, -0.29].forEach((cx) => {
      const cable = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 42.2, 6), darkSteelMat);
      cable.position.set(cx, 22.8, 0.12); tower.add(cable);
    });

    // Mid-Height Dual Red Aviation Obstruction Lights on opposite corners at y = 22m
    const midH = halfAtY(22);
    [[-midH - 0.35, -midH - 0.35], [midH + 0.35, midH + 0.35]].forEach(([mx, mz]) => {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.06, 0.06), darkSteelMat);
      arm.position.set(mx * 0.92, 22.0, mz * 0.92); arm.rotation.y = Math.PI / 4;
      const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.22, 8, 8), beaconRedMat);
      bulb.position.set(mx, 22.25, mz);
      tower.add(arm, bulb);
    });

    // 3. GROUNDED Maintenance Service Platforms (y = 29.4m & y = 44m) + Stanchions + 5G Sector Panels w/ RRU & Booms + Microwave Dishes!
    [
      { y: 29.4, rad: 2.55, dishAngles: [-Math.PI * 0.75, Math.PI * 0.15] },
      { y: 44.0, rad: 2.15, dishAngles: [Math.PI * 0.25, -Math.PI * 0.45] }
    ].forEach((lvl) => {
      // Main steel grating deck + outer safety toe-kick rim
      const plat = new THREE.Mesh(new THREE.CylinderGeometry(lvl.rad, lvl.rad, 0.28, 16), darkSteelMat);
      plat.position.y = lvl.y; plat.castShadow = true;
      const kickRim = new THREE.Mesh(new THREE.CylinderGeometry(lvl.rad + 0.04, lvl.rad + 0.04, 0.2, 16, 1, true), yellowSafetyMat);
      kickRim.position.y = lvl.y + 0.22;
      tower.add(plat, kickRim);

      // 8 Diagonal Underside Cantilever Brackets supporting platform cleanly from tower legs!
      const hUnder = halfAtY(lvl.y - 1.6);
      for (let b = 0; b < 8; b++) {
        const ba = (b / 8) * Math.PI * 2;
        addBeam(
          new THREE.Vector3(Math.cos(ba) * hUnder, lvl.y - 1.55, Math.sin(ba) * hUnder),
          new THREE.Vector3(Math.cos(ba) * (lvl.rad - 0.15), lvl.y - 0.1, Math.sin(ba) * (lvl.rad - 0.15)),
          0.065, darkSteelMat, false
        );
      }

      // Grounded Double-Ring Safety Handrails (Top Rail + Mid Rail + 12 Vertical Stanchion Posts — ZERO floating rails!)
      [lvl.y + 0.62, lvl.y + 1.15].forEach((ry) => {
        const rail = new THREE.Mesh(new THREE.TorusGeometry(lvl.rad - 0.05, 0.045, 6, 24), whiteSteelMat);
        rail.rotation.x = Math.PI / 2; rail.position.y = ry;
        tower.add(rail);
      });
      for (let p = 0; p < 12; p++) {
        const pa = (p / 12) * Math.PI * 2;
        const post = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 1.18, 6), whiteSteelMat);
        post.position.set(Math.cos(pa) * (lvl.rad - 0.05), lvl.y + 0.6, Math.sin(pa) * (lvl.rad - 0.05));
        tower.add(post);
      }

      // 3 Dual-Panel 5G MIMO Sector Antenna Arrays (120° apart) mounted on Steel Outrigger Booms with Rear RRU Transceiver Boxes!
      for (let k = 0; k < 3; k++) {
        const ang = (k * Math.PI * 2) / 3 + (lvl.y > 40 ? 0.35 : 0);
        const secGrp = new THREE.Group();
        secGrp.rotation.y = -ang;
        // Twin horizontal steel mounting standoffs attaching pole securely to platform deck & handrail!
        [lvl.y + 0.25, lvl.y + 1.15].forEach((by) => {
          const boom = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.07, 0.07), galvSteelMat);
          boom.position.set(lvl.rad + 0.22, by, 0); secGrp.add(boom);
        });
        // Vertical antenna mount pipe + rear grey aluminum RRU (Remote Radio Unit) heatsink box
        const pipe = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 3.1, 6), galvSteelMat);
        pipe.position.set(lvl.rad + 0.56, lvl.y + 1.25, 0);
        const rru = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.62, 0.42), darkSteelMat);
        rru.position.set(lvl.rad + 0.38, lvl.y + 0.95, 0);
        // Primary Tall 5G Sector Antenna + Secondary Compact Massive-MIMO 5G Panel side-by-side!
        const panelMain = new THREE.Mesh(new THREE.BoxGeometry(0.26, 2.75, 0.58), antennaWhiteMat);
        panelMain.position.set(lvl.rad + 0.72, lvl.y + 1.35, -0.36); panelMain.castShadow = true;
        const panelSub = new THREE.Mesh(new THREE.BoxGeometry(0.22, 1.65, 0.52), antennaWhiteMat);
        panelSub.position.set(lvl.rad + 0.72, lvl.y + 1.55, 0.34); panelSub.castShadow = true;
        const crossBar = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 1.05), galvSteelMat);
        crossBar.position.set(lvl.rad + 0.58, lvl.y + 1.85, 0);
        const crossBar2 = crossBar.clone(); crossBar2.position.y = lvl.y + 0.75;
        secGrp.add(pipe, rru, panelMain, panelSub, crossBar, crossBar2);
        tower.add(secGrp);
      }

      // 2 Shrouded Microwave Point-to-Point Relay Dishes with Steel Mounting Brackets & Rear Waveguide Feeds!
      lvl.dishAngles.forEach((dAng, di) => {
        const dRad = di === 0 ? 1.08 : 0.72;
        const dishGrp = new THREE.Group();
        dishGrp.rotation.y = -dAng;
        // Mounting arm & diagonal stabilizer brace anchored solidly to the platform!
        const dArm = new THREE.Mesh(new THREE.BoxGeometry(0.88, 0.09, 0.09), galvSteelMat);
        dArm.position.set(lvl.rad + 0.25, lvl.y + 0.62, 0);
        const dPole = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 1.4, 6), galvSteelMat);
        dPole.position.set(lvl.rad + 0.52, lvl.y + 0.62, 0);
        const drum = new THREE.Mesh(new THREE.CylinderGeometry(dRad, dRad * 0.9, 0.52, 18), antennaWhiteMat);
        drum.rotation.z = Math.PI / 2; drum.position.set(lvl.rad + 0.78, lvl.y + 0.68, 0);
        const radome = new THREE.Mesh(new THREE.CircleGeometry(dRad * 0.96, 18), whiteSteelMat);
        radome.rotation.y = Math.PI / 2; radome.position.set(lvl.rad + 1.05, lvl.y + 0.68, 0);
        const rim = new THREE.Mesh(new THREE.TorusGeometry(dRad * 0.98, 0.075, 8, 18), redSteelMat);
        rim.rotation.y = Math.PI / 2; rim.position.set(lvl.rad + 1.04, lvl.y + 0.68, 0);
        const rearFeed = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.3, 0.35, 10), darkSteelMat);
        rearFeed.rotation.z = Math.PI / 2; rearFeed.position.set(lvl.rad + 0.45, lvl.y + 0.68, 0);
        dishGrp.add(dArm, dPole, drum, radome, rim, rearFeed);
        tower.add(dishGrp);
      });
    });

    // 4. Multi-Stage Broadcast Top Mast, Side-Mount VHF Dipoles, Lightning Arrestor Needles & Caged Dual Red Aviation Beacon!
    const mastBase = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.34, 1.4, 10), darkSteelMat);
    mastBase.position.y = 44.7;
    const spireRed = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.22, 2.2, 10), redSteelMat);
    spireRed.position.y = 46.4;
    const spireWhite = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.14, 2.2, 10), whiteSteelMat);
    spireWhite.position.y = 48.5;
    tower.add(mastBase, spireRed, spireWhite);

    // 4 Side-Mounted Stacked VHF/Broadcast Dipole Loops along Top Mast
    [45.8, 46.7, 47.6, 48.5].forEach((dy, idx) => {
      const dip = new THREE.Group();
      dip.position.y = dy; dip.rotation.y = (idx * Math.PI) / 2;
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.045, 0.045), galvSteelMat);
      arm.position.x = 0.32;
      const elem = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.72, 6), whiteSteelMat);
      elem.position.x = 0.6;
      dip.add(arm, elem);
      tower.add(dip);
    });

    // Caged Dual Red Aviation Obstruction Warning Beacon + 4 Lightning Arrestor Air-Terminal Needles at y = 49.7m!
    const beaconBase = new THREE.Mesh(new THREE.CylinderGeometry(0.38, 0.26, 0.22, 10), darkSteelMat);
    beaconBase.position.y = 49.62;
    [-0.18, 0.18].forEach((bx) => {
      const bBulb = new THREE.Mesh(new THREE.SphereGeometry(0.22, 10, 8), beaconRedMat);
      bBulb.position.set(bx, 49.92, 0);
      tower.add(bBulb);
    });
    // Wire guard cage + top lightning arrestor needle spire
    const cageRing = new THREE.Mesh(new THREE.TorusGeometry(0.42, 0.025, 6, 14), galvSteelMat);
    cageRing.rotation.x = Math.PI / 2; cageRing.position.y = 49.95;
    const lightningPin = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.035, 1.35, 6), galvSteelMat);
    lightningPin.position.y = 50.4;
    for (let n = 0; n < 4; n++) {
      const na = (n * Math.PI) / 2;
      const needle = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.022, 0.55, 5), galvSteelMat);
      needle.position.set(Math.cos(na) * 0.25, 50.15, Math.sin(na) * 0.25);
      needle.rotation.z = -Math.cos(na) * 0.55;
      needle.rotation.x = Math.sin(na) * 0.55;
      tower.add(needle);
    }
    tower.add(beaconBase, cageRing, lightningPin);

    this.scene.add(tower);
    this.addCylCollider(75, -70, 3.4, 0, 50.5, 'telecom_5g'); // 45m+ Tall 5G Telecom Tower solid collider
  }

  buildMapLandmarks() {
    // 4 Richly Detailed Realistic Landmarks across the Expanded Valley (procedural stone, solar grid, wood & ruins!)
    const makeStoneTex = () => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#73797f';
      ctx.fillRect(0, 0, 128, 128);
      for (let y = 0; y < 128; y += 22) {
        const off = ((y / 22) % 2) * 18;
        for (let x = -18 + off; x < 128; x += 36) {
          const shade = 104 + ((x * 7 + y * 13) & 24);
          ctx.fillStyle = `rgb(${shade},${shade + 2},${shade - 1})`;
          ctx.fillRect(x + 2, y + 2, 32, 18);
        }
      }
      // Subtle moss patches in stone mortar cracks
      for (let i = 0; i < 40; i++) {
        ctx.fillStyle = 'rgba(62, 92, 54, 0.38)';
        ctx.fillRect((i * 31) % 124, (i * 47) % 124, 8, 5);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 3);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeSolarTex = () => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#0c2a66';
      ctx.fillRect(0, 0, 128, 128);
      // Silver photovoltaic silicon cell grid
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 2;
      for (let x = 0; x <= 128; x += 16) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, 128); ctx.stroke();
      }
      for (let y = 0; y <= 128; y += 21.3) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(128, y); ctx.stroke();
      }
      // Metallic outer frame border
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 6;
      ctx.strokeRect(0, 0, 128, 128);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const stoneTex = makeStoneTex();
    const stoneMat = new THREE.MeshStandardMaterial({ map: stoneTex, roughness: 0.94 });
    const mossStoneMat = new THREE.MeshStandardMaterial({ map: stoneTex, color: 0x9eb098, roughness: 0.95 });
    const bushMat = new THREE.MeshStandardMaterial({ color: 0x225629, roughness: 0.88, flatShading: true });
    const woodMat = new THREE.MeshStandardMaterial({ color: 0x6b3a17, roughness: 0.9 });
    const lightWoodMat = new THREE.MeshStandardMaterial({ color: 0x9a6536, roughness: 0.86 });
    const darkMetalMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.5, metalness: 0.65 });
    const silverMat = new THREE.MeshStandardMaterial({ color: 0xcbd5e1, roughness: 0.4, metalness: 0.72 });

    // =========================================================================
    // LANDMARK 1 (-168, -148): HIGH-DETAIL ANCIENT CLASSICAL TEMPLE RUINS & ARCHAEOLOGICAL SANCTUARY!
    // =========================================================================
    const ruins = new THREE.Group();
    ruins.position.set(-168, 0, -148);

    // Procedural Textures: Fluted Weathered Marble Columns, Classical Floor Mosaic & Archaeological Heritage Plaque
    const makeFlutedMarbleTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#c8c4b8'; ctx.fillRect(0, 0, 256, 256);
      // 16 Classical fluted vertical concave column channels with warm highlights & shaded flutes
      for (let x = 0; x < 256; x += 16) {
        const g = ctx.createLinearGradient(x, 0, x + 16, 0);
        g.addColorStop(0, '#8d8b80');
        g.addColorStop(0.22, '#dcd8cc');
        g.addColorStop(0.55, '#b6b3a6');
        g.addColorStop(0.88, '#969488');
        g.addColorStop(1, '#78776e');
        ctx.fillStyle = g;
        ctx.fillRect(x, 0, 16, 256);
      }
      // Horizontal stacked drum joints + aged olive moss stains near base
      for (let y = 42; y < 256; y += 52) {
        ctx.fillStyle = 'rgba(42, 44, 38, 0.32)';
        ctx.fillRect(0, y, 256, 2);
      }
      for (let i = 0; i < 65; i++) {
        ctx.fillStyle = 'rgba(66, 98, 56, 0.28)';
        ctx.fillRect((i * 43) % 244, 165 + ((i * 29) % 86), 10, 7);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeMosaicTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#c2baa8'; ctx.fillRect(0, 0, 256, 256);
      // Concentric classical Greco-Roman sanctuary floor rings & 8-point compass star!
      const cx = 128, cy = 128;
      ctx.strokeStyle = '#475569'; ctx.lineWidth = 10;
      ctx.beginPath(); ctx.arc(cx, cy, 114, 0, Math.PI * 2); ctx.stroke();
      ctx.strokeStyle = '#9a3412'; ctx.lineWidth = 8;
      ctx.beginPath(); ctx.arc(cx, cy, 96, 0, Math.PI * 2); ctx.stroke();
      ctx.strokeStyle = '#334155'; ctx.lineWidth = 5;
      ctx.beginPath(); ctx.arc(cx, cy, 74, 0, Math.PI * 2); ctx.stroke();
      // Alternating checkerboard border tiles along outer ring
      for (let k = 0; k < 24; k++) {
        const a = (k / 24) * Math.PI * 2;
        ctx.fillStyle = (k % 2 === 0) ? '#334155' : '#e5dfd1';
        ctx.beginPath(); ctx.arc(cx + Math.cos(a) * 85, cy + Math.sin(a) * 85, 5, 0, Math.PI * 2); ctx.fill();
      }
      // 8-point ancient sun compass star
      for (let k = 0; k < 8; k++) {
        const a = (k / 8) * Math.PI * 2;
        ctx.fillStyle = (k % 2 === 0) ? '#9a3412' : '#475569';
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(a - 0.16) * 26, cy + Math.sin(a - 0.16) * 26);
        ctx.lineTo(cx + Math.cos(a) * 68, cy + Math.sin(a) * 68);
        ctx.lineTo(cx + Math.cos(a + 0.16) * 26, cy + Math.sin(a + 0.16) * 26);
        ctx.closePath(); ctx.fill();
      }
      // Moss cracks weathering the ancient tiles
      for (let i = 0; i < 45; i++) {
        ctx.fillStyle = 'rgba(56, 86, 48, 0.34)';
        ctx.fillRect((i * 59) % 240, (i * 37) % 240, 9, 6);
      }
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeRuinPlaqueTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#292524'; ctx.fillRect(0, 0, 256, 128);
      ctx.strokeStyle = '#d97706'; ctx.lineWidth = 6; ctx.strokeRect(5, 5, 246, 118);
      ctx.fillStyle = '#fef3c7'; ctx.font = 'bold 22px sans-serif';
      ctx.textAlign = 'center'; ctx.fillText('ANCIENT SANCTUARY', 128, 38);
      ctx.fillStyle = '#f59e0b'; ctx.font = 'bold 14px sans-serif';
      ctx.fillText('HERITAGE SITE · C. 400 BC', 128, 66);
      ctx.fillStyle = '#cbd5e1'; ctx.font = '12px sans-serif';
      ctx.fillText('SOLAR TEMPLE & CELESTIAL DIAL', 128, 96);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const marbleMat = new THREE.MeshStandardMaterial({ map: makeFlutedMarbleTex(), roughness: 0.88, side: THREE.DoubleSide });
    const carvedCreamMat = new THREE.MeshStandardMaterial({ map: stoneTex, color: 0xd8d2c5, roughness: 0.9, side: THREE.DoubleSide });
    const verdigrisMat = new THREE.MeshStandardMaterial({ color: 0x3b8c7e, roughness: 0.45, metalness: 0.62, side: THREE.DoubleSide }); // Oxidized ancient bronze!
    const ancientGoldMat = new THREE.MeshStandardMaterial({ color: 0xd99a26, roughness: 0.32, metalness: 0.78 });
    const amphoraMat = new THREE.MeshStandardMaterial({ color: 0xb85b35, roughness: 0.84, side: THREE.DoubleSide });
    const ivyMat = new THREE.MeshStandardMaterial({ color: 0x2a662f, roughness: 0.86, flatShading: true });
    const emberMat = new THREE.MeshBasicMaterial({ color: 0xff6b1a });

    // 1. Three-Tiered Classical Crepidoma Stylobate Platform + Inlaid Central Mosaic Courtyard!
    const step1 = new THREE.Mesh(new THREE.BoxGeometry(22.4, 0.55, 22.4), stoneMat);
    step1.position.y = 0.275; step1.receiveShadow = true;
    const step2 = new THREE.Mesh(new THREE.BoxGeometry(20.2, 0.46, 20.2), carvedCreamMat);
    step2.position.y = 0.78; step2.receiveShadow = true;
    const step3 = new THREE.Mesh(new THREE.BoxGeometry(18.2, 0.42, 18.2), mossStoneMat);
    step3.position.y = 1.22; step3.receiveShadow = true;
    const mosaicFloor = new THREE.Mesh(
      new THREE.CircleGeometry(5.6, 28),
      new THREE.MeshStandardMaterial({ map: makeMosaicTex(), roughness: 0.9 })
    );
    mosaicFloor.rotation.x = -Math.PI / 2;
    mosaicFloor.position.y = 1.44; mosaicFloor.receiveShadow = true;
    ruins.add(step1, step2, step3, mosaicFloor);

    // 2. Grand 5-Step Ceremonial Entrance Staircase + Carved Cheek Balustrades + Twin Bronze Fire Braziers!
    for (let s = 0; s < 5; s++) {
      const stair = new THREE.Mesh(new THREE.BoxGeometry(6.6, 0.28, 0.78), carvedCreamMat);
      stair.position.set(0, 0.14 + s * 0.28, 11.75 - s * 0.62);
      stair.receiveShadow = true; stair.castShadow = true;
      ruins.add(stair);
    }
    [-3.75, 3.75].forEach((cx) => {
      const cheek = new THREE.Mesh(new THREE.BoxGeometry(0.95, 1.42, 3.2), stoneMat);
      cheek.position.set(cx, 0.71, 10.6); cheek.castShadow = true; cheek.receiveShadow = true;
      const cap = new THREE.Mesh(new THREE.BoxGeometry(1.12, 0.16, 3.35), carvedCreamMat);
      cap.position.set(cx, 1.48, 10.6);
      // Ancient Verdigris-Bronze Tripod Brazier crowning each entrance cheek pedestal!
      const bStem = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.32, 0.58, 8), verdigrisMat);
      bStem.position.set(cx, 1.84, 11.45);
      const bBowl = new THREE.Mesh(new THREE.CylinderGeometry(0.54, 0.24, 0.34, 12), verdigrisMat);
      bBowl.position.set(cx, 2.25, 11.45); bBowl.castShadow = true;
      const bRim = new THREE.Mesh(new THREE.TorusGeometry(0.54, 0.05, 6, 14), ancientGoldMat);
      bRim.rotation.x = Math.PI / 2; bRim.position.set(cx, 2.42, 11.45);
      const bCoal = new THREE.Mesh(new THREE.SphereGeometry(0.32, 8, 6), emberMat);
      bCoal.scale.set(1, 0.38, 1); bCoal.position.set(cx, 2.38, 11.45);
      ruins.add(cheek, cap, bStem, bBowl, bRim, bCoal);
    });

    // 3. Stepped Crumbling Ashlar-Stone Sanctuary Side Walls & West Broken Classical Archway!
    // West stepped ruined sanctuary wall + semi-circular stone arch framing courtyard
    [
      [-8.3, 2.5, -3.6, 1.1, 2.2, 4.4],
      [-8.3, 2.1,  1.2, 1.1, 1.4, 3.8],
      [-8.3, 3.9, -4.6, 1.1, 1.0, 2.2],
      [ 8.3, 2.3, -3.2, 1.1, 1.8, 5.2],
      [ 8.3, 3.5, -4.6, 1.1, 0.8, 2.0],
      [ 8.3, 1.9,  1.5, 1.1, 1.0, 2.8]
    ].forEach(([wx, wy, wz, sw, sh, sd], idx) => {
      const blk = new THREE.Mesh(new THREE.BoxGeometry(sw, sh, sd), idx % 2 === 0 ? mossStoneMat : carvedCreamMat);
      blk.position.set(wx, wy, wz); blk.castShadow = true; blk.receiveShadow = true;
      ruins.add(blk);
    });
    // Broken West Classical Stone Archway (two ashlar piers + segmented voussoir arch ring!)
    [-4.2, -0.4].forEach((az) => {
      const pier = new THREE.Mesh(new THREE.BoxGeometry(1.35, 4.2, 1.15), carvedCreamMat);
      pier.position.set(-8.3, 3.5, az); pier.castShadow = true;
      const cap = new THREE.Mesh(new THREE.BoxGeometry(1.55, 0.25, 1.35), stoneMat);
      cap.position.set(-8.3, 5.65, az);
      ruins.add(pier, cap);
    });
    for (let k = 0; k < 7; k++) {
      const ang = (k / 6) * Math.PI;
      if (k === 5) continue; // Authentic missing fallen keystone block!
      const v = new THREE.Mesh(new THREE.BoxGeometry(1.28, 0.56, 0.52), k % 2 === 0 ? carvedCreamMat : mossStoneMat);
      v.position.set(-8.3, 5.72 + Math.sin(ang) * 1.85, -2.3 + Math.cos(ang) * 1.85);
      v.rotation.x = ang - Math.PI / 2;
      v.castShadow = true;
      ruins.add(v);
    }

    // 4. Eight Fluted Classical Doric Columns (Attic Molded Bases, Fluted Marble Shafts & Capitals — 4 Full North + 4 Varied Ruined!)
    const colSpecs = [
      { x: -7.2, z: -7.2, h: 8.4, intact: true },
      { x: -2.4, z: -7.2, h: 8.4, intact: true },
      { x:  2.4, z: -7.2, h: 8.4, intact: true },
      { x:  7.2, z: -7.2, h: 8.4, intact: true },
      { x: -7.2, z:  4.2, h: 5.4, intact: false },
      { x: -7.2, z:  7.4, h: 3.2, intact: false },
      { x:  7.2, z:  4.0, h: 2.3, intact: false },
      { x:  7.2, z:  7.4, h: 4.5, intact: false }
    ];
    colSpecs.forEach((c) => {
      // Stepped square plinth + double Attic torus base molding
      const plinth = new THREE.Mesh(new THREE.BoxGeometry(2.05, 0.36, 2.05), carvedCreamMat);
      plinth.position.set(c.x, 1.61, c.z);
      const torus1 = new THREE.Mesh(new THREE.CylinderGeometry(0.84, 0.92, 0.2, 16), carvedCreamMat);
      torus1.position.set(c.x, 1.88, c.z);
      const torus2 = new THREE.Mesh(new THREE.CylinderGeometry(0.76, 0.82, 0.16, 16), stoneMat);
      torus2.position.set(c.x, 2.06, c.z);
      // Fluted tapered marble column shaft
      const topR = 0.72 - (c.h / 8.4) * 0.1;
      const col = new THREE.Mesh(new THREE.CylinderGeometry(topR, 0.74, c.h, 16), marbleMat);
      col.position.set(c.x, 2.14 + c.h / 2, c.z);
      col.castShadow = true; col.receiveShadow = true;
      ruins.add(plinth, torus1, torus2, col);
      if (c.intact) {
        // Classical Doric Necking Ring + Flared Echinus + Square Abacus Capital!
        const neck = new THREE.Mesh(new THREE.TorusGeometry(topR + 0.04, 0.06, 6, 16), carvedCreamMat);
        neck.rotation.x = Math.PI / 2; neck.position.set(c.x, 2.14 + c.h - 0.08, c.z);
        const ech = new THREE.Mesh(new THREE.CylinderGeometry(0.95, topR, 0.36, 16), carvedCreamMat);
        ech.position.set(c.x, 2.14 + c.h + 0.14, c.z);
        const abacus = new THREE.Mesh(new THREE.BoxGeometry(2.05, 0.32, 2.05), carvedCreamMat);
        abacus.position.set(c.x, 2.14 + c.h + 0.46, c.z);
        ruins.add(neck, ech, abacus);
      } else {
        // Jagged fractured top drum rim on broken columns
        const stub = new THREE.Mesh(new THREE.CylinderGeometry(topR * 0.65, topR, 0.42, 8), mossStoneMat);
        stub.position.set(c.x + 0.08, 2.14 + c.h + 0.16, c.z - 0.06);
        stub.rotation.z = 0.28;
        ruins.add(stub);
      }
      this.addCylCollider(-168 + c.x, -148 + c.z, 0.95, 0, 2.14 + c.h + (c.intact ? 0.6 : 0.3));
    });

    // 5. Classical Stepped Doric Entablature (Architrave + Triglyph Frieze + Dentil Cornice + Broken Extruded Pediment!)
    const entY = 11.15;
    const architrave = new THREE.Mesh(new THREE.BoxGeometry(16.9, 0.72, 1.85), carvedCreamMat);
    architrave.position.set(0, entY, -7.2); architrave.castShadow = true;
    const frieze = new THREE.Mesh(new THREE.BoxGeometry(16.7, 0.78, 1.72), mossStoneMat);
    frieze.position.set(0, entY + 0.75, -7.2); frieze.castShadow = true;
    // Carved vertical Doric triglyph blocks rhythmically spaced across the frieze!
    for (let tx = -7.2; tx <= 7.25; tx += 1.6) {
      const trig = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.72, 1.86), carvedCreamMat);
      trig.position.set(tx, entY + 0.75, -7.2);
      ruins.add(trig);
    }
    const cornice = new THREE.Mesh(new THREE.BoxGeometry(17.5, 0.36, 2.35), carvedCreamMat);
    cornice.position.set(0, entY + 1.32, -7.2); cornice.castShadow = true;

    // Authentic Broken Triangular Classical Pediment (ExtrudeGeometry — jaggedly collapsed on East right edge!)
    const pedShape = new THREE.Shape([
      new THREE.Vector2(-8.4, 0),
      new THREE.Vector2(3.6, 0),
      new THREE.Vector2(2.8, 1.35),
      new THREE.Vector2(1.4, 2.35),
      new THREE.Vector2(0, 2.75)
    ]);
    const pedMesh = new THREE.Mesh(
      new THREE.ExtrudeGeometry(pedShape, { depth: 1.55, bevelEnabled: false }),
      mossStoneMat
    );
    pedMesh.position.set(0, entY + 1.48, -7.2 - 0.775);
    pedMesh.castShadow = true;
    // Left intact raking cornice slope capping the surviving West half of the pediment!
    const rakeSlope = Math.atan2(2.75, 8.4);
    const rakeLen = Math.hypot(8.7, 2.85);
    const rakeL = new THREE.Mesh(new THREE.BoxGeometry(rakeLen, 0.3, 2.25), carvedCreamMat);
    rakeL.position.set(-4.2, entY + 1.48 + 1.48, -7.2);
    rakeL.rotation.z = rakeSlope; rakeL.castShadow = true;
    ruins.add(architrave, frieze, cornice, pedMesh, rakeL);
    this.addBoxCollider(-168, 12.1, -155.2, 17.5, 3.2, 2.4);

    // 6. Hero Sanctuary Centerpiece: Carved Octagonal Altar Pedestal + Verdigris-Bronze Celestial Armillary Sphere!
    const altBase = new THREE.Mesh(new THREE.CylinderGeometry(2.55, 2.85, 0.42, 8), carvedCreamMat);
    altBase.position.y = 1.65; altBase.receiveShadow = true;
    const altShaft = new THREE.Mesh(new THREE.CylinderGeometry(1.55, 1.85, 1.15, 8), mossStoneMat);
    altShaft.position.y = 2.42; altShaft.castShadow = true;
    const altSlab = new THREE.Mesh(new THREE.CylinderGeometry(2.05, 1.85, 0.28, 8), carvedCreamMat);
    altSlab.position.y = 3.12; altSlab.castShadow = true;
    // Oxidized Verdigris-Bronze & Ancient Gold Celestial Armillary Sphere / Sundial!
    const armGrp = new THREE.Group();
    armGrp.position.set(0, 4.48, 0);
    const armStand = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.45, 0.52, 8), verdigrisMat);
    armStand.position.y = -1.0;
    const ring1 = new THREE.Mesh(new THREE.TorusGeometry(0.95, 0.065, 8, 24), verdigrisMat);
    const ring2 = new THREE.Mesh(new THREE.TorusGeometry(0.92, 0.055, 8, 24), verdigrisMat);
    ring2.rotation.y = Math.PI / 2;
    const ringEq = new THREE.Mesh(new THREE.TorusGeometry(0.86, 0.06, 8, 24), ancientGoldMat);
    ringEq.rotation.x = Math.PI / 2 - 0.42;
    const ringZod = new THREE.Mesh(new THREE.TorusGeometry(0.72, 0.045, 8, 20), verdigrisMat);
    ringZod.rotation.set(0.55, 0.7, 0.35);
    const gnomon = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.04, 2.35, 6), ancientGoldMat);
    gnomon.rotation.z = -0.45;
    const sunOrb = new THREE.Mesh(new THREE.SphereGeometry(0.18, 10, 10), ancientGoldMat);
    armGrp.add(armStand, ring1, ring2, ringEq, ringZod, gnomon, sunOrb);
    ruins.add(altBase, altShaft, altSlab, armGrp);
    this.addCylCollider(-168, -148, 2.2, 0, 5.5);

    // 7. Fallen Fluted Column Drums, Carved Rubble, Terracotta Greek Amphora Jars & Climbing Ivy Foliage!
    // Tumbled series of separated fluted column drums rolling off East terrace & inside courtyard
    [
      { x: 9.6, y: 0.72, z: 3.8, len: 2.3, ry: 0.45 },
      { x: 11.4, y: 0.72, z: 2.7, len: 1.9, ry: 0.58 },
      { x: -4.2, y: 2.02, z: 3.8, len: 1.7, ry: -0.35 },
      { x: 4.4, y: 2.02, z: 1.8, len: 1.5, ry: 1.1 }
    ].forEach((d) => {
      const drum = new THREE.Mesh(new THREE.CylinderGeometry(0.68, 0.72, d.len, 14), marbleMat);
      drum.rotation.z = Math.PI / 2; drum.rotation.y = d.ry;
      drum.position.set(d.x, d.y, d.z); drum.castShadow = true; drum.receiveShadow = true;
      ruins.add(drum);
    });

    // Scattered mossy ashlar stone rubble blocks
    [[-5.2, 1.72, 1.2, 1.0], [4.8, 1.72, -3.2, 0.9], [-2.8, 1.65, 5.6, 0.85], [6.4, 0.5, 9.2, 1.2]].forEach(([rx, ry, rz, rs]) => {
      const rub = new THREE.Mesh(new THREE.BoxGeometry(rs * 1.4, rs * 0.72, rs), mossStoneMat);
      rub.position.set(rx, ry, rz); rub.rotation.set(0.12, rx, 0.08); rub.castShadow = true;
      ruins.add(rub);
    });

    // 3 Classical Terracotta Greek Amphora Jars (2 standing by walls, 1 toppled on courtyard floor!)
    [
      { x: -6.2, y: 1.44, z: -5.4, toppled: false },
      { x: 6.1,  y: 1.44, z: -5.2, toppled: false },
      { x: -3.2, y: 1.74, z: -2.2, toppled: true }
    ].forEach((am) => {
      const ag = new THREE.Group();
      ag.position.set(am.x, am.y, am.z);
      if (am.toppled) { ag.rotation.z = Math.PI / 2 - 0.1; ag.rotation.y = 0.65; }
      const body = new THREE.Mesh(new THREE.SphereGeometry(0.36, 10, 8), amphoraMat);
      body.scale.set(0.85, 1.35, 0.85); body.position.y = 0.48; body.castShadow = true;
      const neck = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.12, 0.34, 8), amphoraMat);
      neck.position.y = 1.02;
      const lip = new THREE.Mesh(new THREE.TorusGeometry(0.15, 0.035, 6, 10), amphoraMat);
      lip.rotation.x = Math.PI / 2; lip.position.y = 1.18;
      [-0.24, 0.24].forEach((hx) => {
        const hdl = new THREE.Mesh(new THREE.TorusGeometry(0.14, 0.025, 5, 8, Math.PI), amphoraMat);
        hdl.position.set(hx, 0.86, 0); hdl.rotation.z = hx > 0 ? -Math.PI / 2 : Math.PI / 2;
        ag.add(hdl);
      });
      ag.add(body, neck, lip);
      ruins.add(ag);
    });

    // Lush Climbing Ivy Clusters & Mossy Shrubs draping organically over the temple walls, columns & pediment!
    [
      [-8.6, 1.9, -6.8, 1.15], [8.6, 1.7, -6.2, 1.05], [-7.9, 1.7, 6.4, 1.1], [6.2, 1.7, 8.0, 0.95],
      [-8.3, 4.6, -4.1, 0.72], [-7.2, 4.2, -7.2, 0.68], [-7.2, 6.6, -7.1, 0.62], [-2.4, 11.4, -7.0, 0.78],
      [7.2, 3.6, 7.4, 0.7], [-8.8, 1.0, 2.2, 0.92]
    ].forEach(([bx, by, bz, sc]) => {
      const ivy = new THREE.Mesh(new THREE.DodecahedronGeometry(sc, 0), ivyMat);
      ivy.position.set(bx, by, bz);
      ivy.scale.set(1.2, 0.78, 1.08);
      ruins.add(ivy);
    });

    // 8. Tasteful Archaeological Heritage Info Plaque & Preservation Survey Props right beside South Stair Entrance!
    const hSign = new THREE.Group();
    hSign.position.set(-5.6, 0, 12.2); hSign.rotation.y = 0.28;
    [-0.55, 0.55].forEach((px) => {
      const p = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 1.25, 6), woodMat);
      p.position.set(px, 0.62, 0); hSign.add(p);
    });
    const hPlate = new THREE.Mesh(
      new THREE.BoxGeometry(1.55, 0.82, 0.08),
      new THREE.MeshStandardMaterial({ map: makeRuinPlaqueTex(), roughness: 0.55 })
    );
    hPlate.position.set(0, 1.28, 0); hPlate.rotation.x = -0.58; hPlate.castShadow = true;
    hSign.add(hPlate);
    ruins.add(hSign);

    this.scene.add(ruins);

    // =========================================================================
    // LANDMARK 2 (-145, 125): HIGH-DETAIL FIREWATCH-STYLE RANGER LOOKOUT TOWER & NATIONAL PARK CAMPSITE
    // =========================================================================
    const camp = new THREE.Group();
    camp.position.set(-145, 0, 125);

    // Procedural Textures: Cedar Timber Planks, Deep Crimson Shingles & National Park Trailhead Map Kiosk
    const makeCampWoodTex = (hex1 = '#7c4c27', hex2 = '#6b3f1e') => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = hex1; ctx.fillRect(0, 0, 128, 128);
      for (let y = 0; y < 128; y += 16) {
        ctx.fillStyle = ((y / 16) % 2 === 0) ? hex1 : hex2;
        ctx.fillRect(0, y, 128, 14);
        ctx.fillStyle = 'rgba(30, 16, 6, 0.42)';
        ctx.fillRect(0, y + 14, 128, 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(2, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeCampRoofTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#7f231e'; ctx.fillRect(0, 0, 128, 128);
      for (let y = 0; y < 128; y += 16) {
        ctx.fillStyle = '#5c1713'; ctx.fillRect(0, y, 128, 3);
        const off = ((y / 16) % 2) * 16;
        for (let x = off; x < 128; x += 32) ctx.fillRect(x, y, 2, 16);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 3);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeTrailMapTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 160;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#fefce8'; ctx.fillRect(0, 0, 256, 160);
      ctx.fillStyle = '#14532d'; ctx.fillRect(0, 0, 256, 36);
      ctx.fillStyle = '#fef08a'; ctx.font = 'bold 18px sans-serif';
      ctx.textAlign = 'center'; ctx.fillText('PINE RIDGE LOOKOUT · 420M', 128, 24);
      // Stylized miniature trail map graphic (blue lake, green pines & red dashed hiking route)
      ctx.fillStyle = '#bbf7d0'; ctx.fillRect(14, 46, 152, 100);
      ctx.fillStyle = '#38bdf8';
      ctx.beginPath(); ctx.ellipse(58, 86, 28, 18, 0.2, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = '#dc2626'; ctx.lineWidth = 3; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(26, 134); ctx.quadraticCurveTo(95, 118, 142, 68); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = '#dc2626'; ctx.beginPath(); ctx.arc(142, 68, 5, 0, Math.PI * 2); ctx.fill();
      // Right legend sidebar
      ctx.fillStyle = '#334155'; ctx.font = 'bold 11px sans-serif'; ctx.textAlign = 'left';
      ctx.fillText('TRAIL INFO', 176, 62);
      ctx.fillStyle = '#475569'; ctx.font = '10px sans-serif';
      ctx.fillText('• Tower 04', 176, 82);
      ctx.fillText('• Basecamp', 176, 98);
      ctx.fillText('• Lake Trail', 176, 114);
      ctx.fillText('• Fire Safety', 176, 130);
      ctx.strokeStyle = '#78350f'; ctx.lineWidth = 6; ctx.strokeRect(3, 3, 250, 154);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeTowerPlaqueTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 64;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#3f2310'; ctx.fillRect(0, 0, 256, 64);
      ctx.strokeStyle = '#facc15'; ctx.lineWidth = 5; ctx.strokeRect(3, 3, 250, 58);
      ctx.fillStyle = '#fefce8'; ctx.font = 'bold 26px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('TOWER 04 · RANGER', 128, 34);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const campCedarMat = new THREE.MeshStandardMaterial({ map: makeCampWoodTex('#8c5830', '#774824'), roughness: 0.88, side: THREE.DoubleSide });
    const campDarkWood = new THREE.MeshStandardMaterial({ color: 0x4b2c15, roughness: 0.92, side: THREE.DoubleSide });
    const campRoofMat = new THREE.MeshStandardMaterial({ map: makeCampRoofTex(), roughness: 0.82, side: THREE.DoubleSide });
    const campShutterMat = new THREE.MeshStandardMaterial({ color: 0x1c4a32, roughness: 0.82, side: THREE.DoubleSide });
    const campGlassMat = new THREE.MeshStandardMaterial({
      color: 0x7dd3fc, roughness: 0.1, metalness: 0.45,
      transparent: true, opacity: 0.42, side: THREE.DoubleSide, depthWrite: false
    });
    const campWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.78, side: THREE.DoubleSide });
    const campGlowMat = new THREE.MeshBasicMaterial({ color: 0xfef08a });
    const campRedLed = new THREE.MeshBasicMaterial({ color: 0xff2200 });
    const campDirtMat = new THREE.MeshStandardMaterial({ color: 0x7c6a50, roughness: 0.96 });
    const campBrassMat = new THREE.MeshStandardMaterial({ color: 0xd97706, roughness: 0.3, metalness: 0.8 });

    // Helper: connect any two 3D points cleanly with a timber strut or steel rod
    const addCampBeam = (p1, p2, radius, mat) => {
      const dir = new THREE.Vector3().subVectors(p2, p1);
      const len = dir.length();
      if (len < 0.05) return;
      const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len, 6), mat);
      mesh.position.copy(p1).add(p2).multiplyScalar(0.5);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
      mesh.castShadow = true;
      camp.add(mesh);
    };

    // 1. Natural Packed-Earth Basecamp Clearing Pad (blends gravel road end at (+9,-1) smoothly into watchtower & campfire!)
    const campClearing = new THREE.Mesh(new THREE.CircleGeometry(10.5, 24), campDirtMat);
    campClearing.rotation.x = -Math.PI / 2;
    campClearing.scale.set(1.22, 0.92, 1);
    campClearing.position.set(4.5, 0.02, 1.5);
    campClearing.receiveShadow = true;
    camp.add(campClearing);

    // 2. ICONIC FIREWATCH-STYLE TAPERED TIMBER TOWER (Concrete Footings + 3-Tier X-Bracing + Switchback Stairs!)
    const towerH = 11.4;
    const baseSpread = 2.85;
    const topSpread = 2.15;
    const spreadAt = (y) => baseSpread + (topSpread - baseSpread) * (y / towerH);

    // 4 Concrete Footing Piers + 4 Tapered Heavy Timber Corner Legs
    [[-1, -1], [1, -1], [1, 1], [-1, 1]].forEach(([sx, sz]) => {
      const foot = new THREE.Mesh(new THREE.BoxGeometry(0.88, 0.72, 0.88), stoneMat);
      foot.position.set(sx * baseSpread, 0.36, sz * baseSpread);
      foot.castShadow = true; foot.receiveShadow = true;
      camp.add(foot);
      addCampBeam(
        new THREE.Vector3(sx * baseSpread, 0.65, sz * baseSpread),
        new THREE.Vector3(sx * topSpread, towerH, sz * topSpread),
        0.2, campDarkWood
      );
    });

    // 3 Structural Tiers: Horizontal Girts + Full Diagonal X-Bracing on ALL 4 Faces!
    const tierYs = [0.65, 3.9, 7.65, towerH - 0.15];
    for (let t = 0; t < 3; t++) {
      const y0 = tierYs[t], y1 = tierYs[t + 1];
      const s0 = spreadAt(y0), s1 = spreadAt(y1);
      const c0 = [
        new THREE.Vector3(-s0, y0, -s0), new THREE.Vector3(s0, y0, -s0),
        new THREE.Vector3(s0, y0, s0), new THREE.Vector3(-s0, y0, s0)
      ];
      const c1 = [
        new THREE.Vector3(-s1, y1, -s1), new THREE.Vector3(s1, y1, -s1),
        new THREE.Vector3(s1, y1, s1), new THREE.Vector3(-s1, y1, s1)
      ];
      for (let i = 0; i < 4; i++) {
        const nxt = (i + 1) % 4;
        if (t < 2) addCampBeam(c1[i], c1[nxt], 0.13, lightWoodMat); // Mid horizontal girts
        // Full diagonal X-bracing across every structural bay
        addCampBeam(c0[i], c1[nxt], 0.085, woodMat);
        addCampBeam(c0[nxt], c1[i], 0.085, woodMat);
      }
    }

    // 3-Flight Wooden Switchback Staircase with Intermediate Landings, Individual Treads & Safety Handrails!
    const addStairFlight = (pStart, pEnd, steps = 9) => {
      const dx = pEnd.x - pStart.x, dy = pEnd.y - pStart.y, dz = pEnd.z - pStart.z;
      const horiz = Math.hypot(dx, dz) || 1;
      const perpX = (-dz / horiz) * 0.45, perpZ = (dx / horiz) * 0.45;
      // Twin diagonal stringers & elevated safety handrails
      [-1, 1].forEach((s) => {
        const a = new THREE.Vector3(pStart.x + perpX * s, pStart.y, pStart.z + perpZ * s);
        const b = new THREE.Vector3(pEnd.x + perpX * s, pEnd.y, pEnd.z + perpZ * s);
        addCampBeam(a, b, 0.065, campDarkWood);
        addCampBeam(a.clone().add(new THREE.Vector3(0, 0.92, 0)), b.clone().add(new THREE.Vector3(0, 0.92, 0)), 0.04, lightWoodMat);
      });
      const stepAng = Math.atan2(dx, dz);
      for (let i = 1; i <= steps; i++) {
        const frac = i / (steps + 1);
        const tread = new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.08, 0.34), lightWoodMat);
        tread.position.set(pStart.x + dx * frac, pStart.y + dy * frac, pStart.z + dz * frac);
        tread.rotation.y = stepAng;
        tread.castShadow = true;
        camp.add(tread);
      }
    };
    // Intermediate Timber Stair Landings at y = 3.9m (SE) & y = 7.65m (SW)
    const land1 = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.22, 1.5), campCedarMat);
    land1.position.set(2.0, 3.85, 2.0); land1.castShadow = true;
    const land2 = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.22, 1.5), campCedarMat);
    land2.position.set(-1.7, 7.6, 1.7); land2.castShadow = true;
    camp.add(land1, land2);
    // Flight 1 (Ground East -> Landing 1 SE), Flight 2 (Landing 1 SE -> Landing 2 SW), Flight 3 (Landing 2 SW -> Catwalk North-West)
    addStairFlight(new THREE.Vector3(2.4, 0.1, -1.8), new THREE.Vector3(2.0, 3.85, 1.8), 9);
    addStairFlight(new THREE.Vector3(1.8, 3.85, 2.0), new THREE.Vector3(-1.5, 7.6, 1.7), 9);
    addStairFlight(new THREE.Vector3(-1.7, 7.6, 1.5), new THREE.Vector3(-1.6, 11.35, -2.55), 9);

    // 3. Cantilevered Catwalk Observation Deck + 8 Knee Braces + Grounded 24-Baluster Railing + Spotting Scope!
    const wtDeck = new THREE.Mesh(new THREE.BoxGeometry(7.2, 0.38, 7.2), campCedarMat);
    wtDeck.position.y = 11.4; wtDeck.castShadow = true; wtDeck.receiveShadow = true;
    const wtDeckRim = new THREE.Mesh(new THREE.BoxGeometry(7.34, 0.22, 7.34), campDarkWood);
    wtDeckRim.position.y = 11.32;
    camp.add(wtDeck, wtDeckRim);
    // 8 Diagonal Timber Knee Braces anchoring cantilevered deck to tower legs
    [[-2.15, -2.15], [2.15, -2.15], [-2.15, 2.15], [2.15, 2.15]].forEach(([lx, lz]) => {
      addCampBeam(new THREE.Vector3(lx, 9.65, lz), new THREE.Vector3(lx * 1.52, 11.25, lz), 0.11, campDarkWood);
      addCampBeam(new THREE.Vector3(lx, 9.65, lz), new THREE.Vector3(lx, 11.25, lz * 1.52), 0.11, campDarkWood);
    });
    // Grounded Catwalk Safety Railing (top rail + mid rail + 24 vertical posts)
    [12.02, 12.48].forEach((ry) => {
      const rN = new THREE.Mesh(new THREE.BoxGeometry(7.2, 0.09, 0.09), lightWoodMat);
      rN.position.set(0, ry, -3.52);
      const rS = rN.clone(); rS.position.z = 3.52;
      const rW = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.09, 7.2), lightWoodMat);
      rW.position.set(-3.52, ry, 0);
      const rE = rW.clone(); rE.position.x = 3.52;
      camp.add(rN, rS, rW, rE);
    });
    for (let i = 0; i <= 6; i++) {
      const v = -3.52 + (i / 6) * 7.04;
      [[v, -3.52], [v, 3.52], [-3.52, v], [3.52, v]].forEach(([rx, rz]) => {
        if (Math.abs(rx) > 3.5 && Math.abs(rz) > 3.5 && i > 0 && i < 6) return;
        const p = new THREE.Mesh(new THREE.BoxGeometry(0.11, 1.08, 0.11), campDarkWood);
        p.position.set(rx, 11.98, rz); camp.add(p);
      });
    }
    // "TOWER 04 · RANGER" Wooden Plaque mounted cleanly on East Catwalk Railing overlooking Campfire & Valley!
    const tPlaque = new THREE.Mesh(
      new THREE.BoxGeometry(0.1, 0.62, 2.35),
      new THREE.MeshStandardMaterial({ map: makeTowerPlaqueTex(), roughness: 0.6 })
    );
    tPlaque.position.set(3.6, 12.02, 0);
    // Brass Observation Spotting Scope on Tripod standing on East Catwalk!
    const scTripod = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.24, 1.25, 6), darkMetalMat);
    scTripod.position.set(2.85, 12.22, -1.65);
    const scTube = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, 0.72, 10), campBrassMat);
    scTube.rotation.z = Math.PI / 2; scTube.position.set(2.95, 12.92, -1.65);
    camp.add(tPlaque, scTripod, scTube);

    // 4. High-Detail Firewatch Lookout Cabin (Clapboard Base, 360° Framed Windows, Propped Shutters, Osborne Fire-Finder & Shingled Roof!)
    const wtWallLow = new THREE.Mesh(new THREE.BoxGeometry(4.65, 1.15, 4.65), campCedarMat);
    wtWallLow.position.y = 12.16; wtWallLow.castShadow = true; wtWallLow.receiveShadow = true;
    const wtSill = new THREE.Mesh(new THREE.BoxGeometry(4.82, 0.12, 4.82), campWhiteMat);
    wtSill.position.y = 12.76;
    const wtCabGlass = new THREE.Mesh(new THREE.BoxGeometry(4.48, 1.82, 4.48), campGlassMat);
    wtCabGlass.position.y = 13.72;
    const wtHeader = new THREE.Mesh(new THREE.BoxGeometry(4.82, 0.24, 4.82), campCedarMat);
    wtHeader.position.y = 14.72;
    // Framed Corner Posts & Intermediate Window Mullions around all 4 panoramic glass faces
    [[-2.28, -2.28], [2.28, -2.28], [-2.28, 2.28], [2.28, 2.28]].forEach(([cx, cz]) => {
      const cp = new THREE.Mesh(new THREE.BoxGeometry(0.18, 3.15, 0.18), campWhiteMat);
      cp.position.set(cx, 13.18, cz); camp.add(cp);
    });
    [-0.78, 0.78].forEach((m) => {
      [[m, -2.28], [m, 2.28], [-2.28, m], [2.28, m]].forEach(([mx, mz]) => {
        const mul = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.86, 0.08), campWhiteMat);
        mul.position.set(mx, 13.72, mz); camp.add(mul);
      });
    });
    // Framed Cabin Entry Door on East Catwalk side + Warm Glowing Interior Osborne Fire-Finder Pedestal!
    const cabDoor = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.15, 1.05), campDarkWood);
    cabDoor.position.set(2.3, 12.68, 1.25);
    const fireFinderBase = new THREE.Mesh(new THREE.CylinderGeometry(0.45, 0.52, 1.05, 12), campDarkWood);
    fireFinderBase.position.set(0, 12.15, 0);
    const fireFinderMap = new THREE.Mesh(new THREE.CylinderGeometry(0.62, 0.62, 0.1, 14), campWhiteMat);
    fireFinderMap.position.set(0, 12.72, 0);
    const fireFinderSight = new THREE.Mesh(new THREE.TorusGeometry(0.54, 0.035, 6, 16), campBrassMat);
    fireFinderSight.rotation.x = Math.PI / 2; fireFinderSight.position.set(0, 12.82, 0);
    const cabCeilingLamp = new THREE.Mesh(new THREE.SphereGeometry(0.24, 8, 8), campGlowMat);
    cabCeilingLamp.position.set(0, 14.42, 0);

    // Signature Propped-Open Firewatch Wooden Storm Shutters on North, West & South faces!
    [
      { x: 0, z: -2.35, ry: 0 },
      { x: 0, z: 2.35, ry: Math.PI },
      { x: -2.35, z: 0, ry: Math.PI / 2 }
    ].forEach((sh) => {
      const sg = new THREE.Group();
      sg.position.set(sh.x, 14.62, sh.z); sg.rotation.y = sh.ry;
      const shutter = new THREE.Mesh(new THREE.BoxGeometry(3.6, 0.09, 1.48), campShutterMat);
      shutter.position.set(0, -0.38, -0.62); shutter.rotation.x = -0.56; shutter.castShadow = true;
      [-1.55, 1.55].forEach((px) => {
        const prop = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.028, 1.45, 5), lightWoodMat);
        prop.position.set(px, -1.3, -0.55); prop.rotation.x = -0.78;
        sg.add(prop);
      });
      sg.add(shutter);
      camp.add(sg);
    });

    // Wide Overhanging Shingled Pyramidal Hip Roof + Stovepipe Flue + Rooftop Solar Panel + VHF Radio & Wind Mast!
    const wtRoof = new THREE.Mesh(new THREE.ConeGeometry(4.95, 2.35, 4), campRoofMat);
    wtRoof.position.y = 15.92; wtRoof.rotation.y = Math.PI / 4; wtRoof.castShadow = true;
    const wtRoofCap = new THREE.Mesh(new THREE.ConeGeometry(0.45, 0.35, 4), darkMetalMat);
    wtRoofCap.position.y = 17.15; wtRoofCap.rotation.y = Math.PI / 4;
    const wtFlue = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.16, 1.95, 8), darkMetalMat);
    wtFlue.position.set(-1.35, 16.55, 1.15);
    const wtFlueCap = new THREE.Mesh(new THREE.ConeGeometry(0.28, 0.22, 8), darkMetalMat);
    wtFlueCap.position.set(-1.35, 17.6, 1.15);
    // Rooftop Solar Power Panel powering the Ranger VHF radio
    const wtSolar = new THREE.Mesh(new THREE.BoxGeometry(1.55, 0.08, 1.05), new THREE.MeshStandardMaterial({ color: 0x1e3a8a, roughness: 0.16, metalness: 0.82 }));
    wtSolar.position.set(1.25, 16.12, 1.25); wtSolar.rotation.set(0.55, 0, -0.25);
    // Tall Lightning Rod, VHF Dipole Antenna, Spinning Anemometer & Red Warning Light!
    const wtAnt = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.075, 3.6, 8), silverMat);
    wtAnt.position.set(0, 18.85, 0);
    const wtDipole = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.05, 0.05), darkMetalMat);
    wtDipole.position.set(0, 19.65, 0);
    const wtBeacon = new THREE.Mesh(new THREE.SphereGeometry(0.14, 8, 8), campRedLed);
    wtBeacon.position.set(0, 20.72, 0);
    camp.add(wtWallLow, wtSill, wtCabGlass, wtHeader, cabDoor, fireFinderBase, fireFinderMap, fireFinderSight, cabCeilingLamp, wtRoof, wtRoofCap, wtFlue, wtFlueCap, wtSolar, wtAnt, wtDipole, wtBeacon);
    this.addBoxCollider(-145, 8.6, 125, 7.2, 17.2, 7.2);

    // 5. REAL SCULPTED CAMPING TENTS (Cleanly framing the Campfire & Clearing — ZERO cones, ZERO road overlap!)
    // TENT 1 (5.2, 0, 7.6): Classic Extruded A-Frame Expedition Canvas Tent (Burnt Orange + Rainfly + Open Vestibule + Guy-Lines!)
    const tentA = new THREE.Group();
    tentA.position.set(5.2, 0, 7.6);
    tentA.rotation.y = 2.65; // Faces North-East toward Campfire!
    const aW = 2.55, aH = 1.85, aLen = 3.35;
    const aTarp = new THREE.Mesh(new THREE.BoxGeometry(aW + 0.35, 0.05, aLen + 0.4), darkMetalMat);
    aTarp.position.y = 0.025; aTarp.receiveShadow = true;
    const aShape = new THREE.Shape([new THREE.Vector2(-aW / 2, 0), new THREE.Vector2(aW / 2, 0), new THREE.Vector2(0, aH)]);
    const aBody = new THREE.Mesh(
      new THREE.ExtrudeGeometry(aShape, { depth: aLen, bevelEnabled: false }),
      new THREE.MeshStandardMaterial({ color: 0xea580c, roughness: 0.86, side: THREE.DoubleSide })
    );
    aBody.position.set(0, 0.05, -aLen / 2); aBody.castShadow = true; aBody.receiveShadow = true;
    // Contrasting Cream Waterproof Rainfly draped cleanly over upper A-frame slopes!
    const aFlyLen = Math.hypot(aW / 2, aH) * 0.72;
    const aSlope = Math.atan2(aH, aW / 2);
    [-1, 1].forEach((side) => {
      const fly = new THREE.Mesh(new THREE.BoxGeometry(aFlyLen, 0.05, aLen + 0.36), campWhiteMat);
      fly.position.set(side * (aW * 0.19), aH - Math.sin(aSlope) * (aFlyLen / 2) + 0.08, 0);
      fly.rotation.z = -side * aSlope;
      fly.castShadow = true;
      tentA.add(fly);
    });
    // Dark recessed triangular entry door + rolled-back canvas flaps + timber ridge pole + 4 taut guy-lines!
    const aDoorShape = new THREE.Shape([new THREE.Vector2(-0.62, 0), new THREE.Vector2(0.62, 0), new THREE.Vector2(0, 1.32)]);
    const aDoor = new THREE.Mesh(new THREE.ShapeGeometry(aDoorShape), new THREE.MeshStandardMaterial({ color: 0x18181b, roughness: 0.95 }));
    aDoor.position.set(0, 0.06, aLen / 2 + 0.02);
    const aPole = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, aH + 0.12, 6), lightWoodMat);
    aPole.position.set(0, aH / 2 + 0.04, aLen / 2 + 0.06);
    const guyMat = new THREE.LineBasicMaterial({ color: 0xe2e8f0 });
    [[-1, -1], [1, -1], [-1, 1], [1, 1]].forEach(([gx, gz]) => {
      const gPts = [new THREE.Vector3(gx * (aW * 0.42), 0.75, gz * (aLen * 0.45)), new THREE.Vector3(gx * (aW * 0.78), 0.04, gz * (aLen * 0.62))];
      tentA.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(gPts), guyMat));
      const peg = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.01, 0.18, 5), silverMat);
      peg.position.set(gx * (aW * 0.78), 0.08, gz * (aLen * 0.62)); tentA.add(peg);
    });
    tentA.add(aTarp, aBody, aDoor, aPole);
    camp.add(tentA);

    // TENT 2 (12.6, 0, 4.4): Sculpted Geodesic Dome Alpine Tent (Deep Teal Dome + Amber Rainfly + Crossed Shock-Cord Poles!)
    const tentB = new THREE.Group();
    tentB.position.set(12.6, 0, 4.4);
    tentB.rotation.y = -1.95; // Faces West-Northwest toward Campfire!
    const bTarp = new THREE.Mesh(new THREE.CylinderGeometry(1.92, 1.98, 0.06, 12), darkMetalMat);
    bTarp.position.y = 0.03; bTarp.receiveShadow = true;
    const bDome = new THREE.Mesh(
      new THREE.SphereGeometry(1.78, 16, 10, 0, Math.PI * 2, 0, Math.PI * 0.52),
      new THREE.MeshStandardMaterial({ color: 0x0f766e, roughness: 0.82, side: THREE.DoubleSide })
    );
    bDome.scale.set(1.0, 0.88, 1.08); bDome.position.y = 0.04; bDome.castShadow = true;
    const bRainfly = new THREE.Mesh(
      new THREE.SphereGeometry(1.84, 16, 8, 0, Math.PI * 2, 0, Math.PI * 0.34),
      new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.76, side: THREE.DoubleSide })
    );
    bRainfly.scale.set(1.0, 0.88, 1.08); bRainfly.position.y = 0.08; bRainfly.castShadow = true;
    // Crossed external flexible fiberglass arch poles + front arched entrance vestibule
    [Math.PI / 4, -Math.PI / 4].forEach((pa) => {
      const arch = new THREE.Mesh(new THREE.TorusGeometry(1.82, 0.032, 6, 20, Math.PI), darkMetalMat);
      arch.rotation.y = pa; arch.scale.set(1.0, 0.88, 1.0); arch.position.y = 0.05;
      tentB.add(arch);
    });
    const bDoor = new THREE.Mesh(new THREE.CircleGeometry(0.68, 12, 0, Math.PI), new THREE.MeshStandardMaterial({ color: 0x18181b, roughness: 0.95 }));
    bDoor.position.set(0, 0.14, 1.88);
    // Red-and-white Camp Cooler Box & Rolled Sleeping Pad beside Dome Tent!
    const cooler = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.48, 0.46), new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.5 }));
    cooler.position.set(-2.3, 0.24, 0.85); cooler.castShadow = true;
    const coolerLid = new THREE.Mesh(new THREE.BoxGeometry(0.79, 0.09, 0.5), campWhiteMat);
    coolerLid.position.set(-2.3, 0.5, 0.85);
    tentB.add(bTarp, bDome, bRainfly, bDoor, cooler, coolerLid);
    camp.add(tentB);

    // 6. COZY CAMPFIRE CIRCLE (12 Individual Stone Boulders, Crossed Logs, Tripod Dutch Oven Pot, Multi-Layer Flame & Log Benches!)
    const fireGrp = new THREE.Group();
    fireGrp.position.set(6.8, 0, 2.6);
    const ashPit = new THREE.Mesh(new THREE.CircleGeometry(1.18, 14), new THREE.MeshStandardMaterial({ color: 0x262626, roughness: 0.98 }));
    ashPit.rotation.x = -Math.PI / 2; ashPit.position.y = 0.04;
    fireGrp.add(ashPit);
    // 12 Individual Cobblestone Fire-Ring Rocks
    for (let r = 0; r < 12; r++) {
      const ra = (r / 12) * Math.PI * 2;
      const rock = new THREE.Mesh(new THREE.DodecahedronGeometry(0.24 + (r % 3) * 0.05, 0), stoneMat);
      rock.scale.set(1.15, 0.72, 1.0);
      rock.position.set(Math.cos(ra) * 1.15, 0.14, Math.sin(ra) * 1.15);
      rock.rotation.set(r * 0.5, r * 1.1, 0);
      rock.castShadow = true;
      fireGrp.add(rock);
    }
    // Crossed Burning Firewood Logs + Glowing Ember Bed + Dual-Tone Sculpted Flame + Tripod Cooking Kettle!
    [0, 1.05, 2.1].forEach((la) => {
      const log = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 1.45, 6), campDarkWood);
      log.rotation.z = Math.PI / 2 - 0.18; log.rotation.y = la;
      log.position.y = 0.22;
      fireGrp.add(log);
    });
    const flameOuter = new THREE.Mesh(new THREE.ConeGeometry(0.45, 1.15, 7), new THREE.MeshBasicMaterial({ color: 0xff5a00 }));
    flameOuter.position.y = 0.62;
    const flameInner = new THREE.Mesh(new THREE.ConeGeometry(0.26, 0.78, 6), new THREE.MeshBasicMaterial({ color: 0xfef08a }));
    flameInner.position.y = 0.48;
    const fireGroundGlow = new THREE.Mesh(
      new THREE.CircleGeometry(2.35, 16),
      new THREE.MeshBasicMaterial({ color: 0xff8c24, transparent: true, opacity: 0.22, depthWrite: false })
    );
    fireGroundGlow.rotation.x = -Math.PI / 2;
    fireGroundGlow.position.y = 0.055;
    // 3-Legged Wooden Cooking Tripod suspending a Cast-Iron Stew Kettle above the Campfire!
    for (let k = 0; k < 3; k++) {
      const ka = (k * Math.PI * 2) / 3 + 0.4;
      addCampBeam(
        new THREE.Vector3(6.8 + Math.cos(ka) * 1.25, 0.02, 2.6 + Math.sin(ka) * 1.25),
        new THREE.Vector3(6.8, 2.15, 2.6),
        0.045, campDarkWood
      );
    }
    const potChain = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.82, 4), darkMetalMat);
    potChain.position.y = 1.65;
    const stewPot = new THREE.Mesh(new THREE.SphereGeometry(0.28, 10, 8), darkMetalMat);
    stewPot.scale.set(1.1, 0.85, 1.1); stewPot.position.y = 1.15;
    fireGrp.add(flameOuter, flameInner, fireGroundGlow, potChain, stewPot);

    // 2 Rustic Split-Log Camp Benches on Cradles + Hanging Camp Lantern Post
    [
      { x: 0, z: 2.75, ry: 0 },
      { x: -2.65, z: -0.2, ry: Math.PI / 2 - 0.15 }
    ].forEach((b) => {
      const bg = new THREE.Group(); bg.position.set(b.x, 0, b.z); bg.rotation.y = b.ry;
      const log = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.26, 2.9, 10), woodMat);
      log.rotation.z = Math.PI / 2; log.position.y = 0.42; log.castShadow = true;
      [-1.0, 1.0].forEach((cx) => {
        const cr = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.24, 0.52), campDarkWood);
        cr.position.set(cx, 0.12, 0); bg.add(cr);
      });
      bg.add(log); fireGrp.add(bg);
    });
    // Shepherd's Hook Camp Lantern Post
    const lPost = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.04, 2.2, 6), darkMetalMat);
    lPost.position.set(2.4, 1.1, -1.6);
    const lHook = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.04, 0.04), darkMetalMat);
    lHook.position.set(2.2, 2.18, -1.6);
    const cLantern = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.13, 0.26, 8), campGlowMat);
    cLantern.position.set(2.02, 2.02, -1.6);
    fireGrp.add(lPost, lHook, cLantern);
    camp.add(fireGrp);

    // 7. National Park Wooden Picnic Table (12.4, -4.6) + Roofed Trailhead Map Kiosk (4.6, -4.2) + Detailed Firewood Shed (-6.8, 4.2)!
    const pic = new THREE.Group();
    pic.position.set(12.4, 0, -4.6); pic.rotation.y = 0.22;
    const pTop = new THREE.Mesh(new THREE.BoxGeometry(2.35, 0.1, 1.15), campCedarMat);
    pTop.position.y = 0.78; pTop.castShadow = true;
    const pCloth = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.115, 1.18), new THREE.MeshStandardMaterial({ color: 0xb91c1c, roughness: 0.85 }));
    pCloth.position.y = 0.785;
    [-0.82, 0.82].forEach((pz) => {
      const seat = new THREE.Mesh(new THREE.BoxGeometry(2.35, 0.08, 0.34), lightWoodMat);
      seat.position.set(0, 0.44, pz); pic.add(seat);
    });
    [-0.85, 0.85].forEach((px) => {
      [-0.45, 0.45].forEach((sl) => {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.9, 0.09), campDarkWood);
        leg.position.set(px, 0.39, sl); leg.rotation.x = sl > 0 ? -0.35 : 0.35;
        pic.add(leg);
      });
    });
    const mug = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.12, 8), campWhiteMat);
    mug.position.set(0.32, 0.9, 0.18);
    pic.add(pTop, pCloth, mug);
    camp.add(pic);

    // Roofed Trailhead Map Information Kiosk standing cleanly beside the gravel arrival clearing (4.6, -4.2)
    const kiosk = new THREE.Group();
    kiosk.position.set(4.6, 0, -4.2); kiosk.rotation.y = 0.38;
    [-0.95, 0.95].forEach((kx) => {
      const kp = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.55, 0.14), campDarkWood);
      kp.position.set(kx, 1.28, 0); kp.castShadow = true; kiosk.add(kp);
    });
    const kBoard = new THREE.Mesh(
      new THREE.BoxGeometry(1.85, 1.18, 0.12),
      new THREE.MeshStandardMaterial({ map: makeTrailMapTex(), roughness: 0.65 })
    );
    kBoard.position.set(0, 1.58, 0); kBoard.castShadow = true;
    const kRoofF = new THREE.Mesh(new THREE.BoxGeometry(2.35, 0.12, 0.62), campRoofMat);
    kRoofF.position.set(0, 2.55, 0.24); kRoofF.rotation.x = 0.42;
    const kRoofB = kRoofF.clone(); kRoofB.position.z = -0.24; kRoofB.rotation.x = -0.42;
    kiosk.add(kBoard, kRoofF, kRoofB);
    camp.add(kiosk);

    // Upgraded 3-Sided Covered Ranger Firewood Shed, Splitting Stump with Axe & Supply Crates (-6.8, 4.2)
    const rShed = new THREE.Group();
    rShed.position.set(-6.8, 0, 4.2);
    rShed.rotation.y = 0.58;
    const rFloor = new THREE.Mesh(new THREE.BoxGeometry(3.3, 0.16, 2.2), campDarkWood);
    rFloor.position.y = 0.08;
    const rBack = new THREE.Mesh(new THREE.BoxGeometry(3.1, 2.05, 0.12), campCedarMat);
    rBack.position.set(0, 1.12, -0.95); rBack.castShadow = true;
    [-1.5, 1.5].forEach((sx) => {
      const sideW = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.95, 1.95), campCedarMat);
      sideW.position.set(sx, 1.08, 0); sideW.castShadow = true; rShed.add(sideW);
    });
    const rShedRoof = new THREE.Mesh(new THREE.BoxGeometry(3.7, 0.18, 2.65), campRoofMat);
    rShedRoof.position.set(0, 2.26, 0.05); rShedRoof.rotation.x = 0.16; rShedRoof.castShadow = true;
    // Stacked split firewood log rows tucked neatly inside the woodshed
    for (let row = 0; row < 4; row++) {
      for (let col = 0; col < 6; col++) {
        const log = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.12, 1.15, 6), (row + col) % 2 === 0 ? lightWoodMat : woodMat);
        log.rotation.x = Math.PI / 2;
        log.position.set(-1.12 + col * 0.44, 0.26 + row * 0.22, -0.15);
        rShed.add(log);
      }
    }
    // Chopping Stump with Steel Camp Axe Embedded + Olive Ranger Supply Crate & Red Water Jerrycan!
    const stump = new THREE.Mesh(new THREE.CylinderGeometry(0.36, 0.42, 0.68, 10), woodMat);
    stump.position.set(2.45, 0.34, 0.55); stump.castShadow = true;
    const axeHead = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.18, 0.06), silverMat);
    axeHead.position.set(2.45, 0.72, 0.55); axeHead.rotation.z = 0.25;
    const axeHandle = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.032, 0.78, 6), lightWoodMat);
    axeHandle.position.set(2.72, 0.96, 0.55); axeHandle.rotation.z = -0.72;
    const crate = new THREE.Mesh(new THREE.BoxGeometry(0.92, 0.58, 0.65), campShutterMat);
    crate.position.set(-2.35, 0.29, 0.45); crate.castShadow = true;
    const jerry = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.52, 0.36), new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.45 }));
    jerry.position.set(-2.35, 0.84, 0.45);
    rShed.add(rFloor, rBack, rShedRoof, stump, axeHead, axeHandle, crate, jerry);
    camp.add(rShed);

    this.scene.add(camp);

    // =========================================================================
    // LANDMARK 3 (172, 118): HIGH-DETAIL 12 MW SOLAR POWER PLANT, HV SUBSTATION, BESS BATTERY YARD & GANTRY!
    // =========================================================================
    const solar = new THREE.Group();
    solar.position.set(172, 0, 118);

    // Procedural Textures: Crushed Gravel Yard, Chain-Link Security Fence & Industrial Signage
    const makeSolarYardTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#6c737d'; ctx.fillRect(0, 0, 256, 256);
      for (let i = 0; i < 1600; i++) {
        const gx = (i * 73 + (i % 13) * 19) % 256;
        const gy = (i * 47 + (i % 17) * 13) % 256;
        const shades = ['#828994', '#585f69', '#949aa4', '#4d545e'];
        ctx.fillStyle = shades[i % shades.length];
        ctx.fillRect(gx, gy, 3, 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(8, 7);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeChainLinkTex = () => {
      const c = document.createElement('canvas');
      c.width = 64; c.height = 64;
      const ctx = c.getContext('2d');
      ctx.clearRect(0, 0, 64, 64);
      ctx.strokeStyle = 'rgba(203, 213, 225, 0.72)';
      ctx.lineWidth = 2;
      for (let i = -64; i <= 128; i += 16) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i + 64, 64); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(i + 64, 0); ctx.lineTo(i, 64); ctx.stroke();
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(16, 2);
      return tex;
    };

    const makeSolarSignTex = (title, sub, bg = '#0f172a', border = '#facc15') => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 96;
      const ctx = c.getContext('2d');
      ctx.fillStyle = bg; ctx.fillRect(0, 0, 256, 96);
      ctx.strokeStyle = border; ctx.lineWidth = 6; ctx.strokeRect(4, 4, 248, 88);
      ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 30px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(title, 128, 36);
      ctx.fillStyle = '#fde047'; ctx.font = 'bold 17px sans-serif';
      ctx.fillText(sub, 128, 68);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const solarPanelMat = new THREE.MeshStandardMaterial({
      map: makeSolarTex(), roughness: 0.14, metalness: 0.84, side: THREE.DoubleSide
    });
    const solarAlumMat = new THREE.MeshStandardMaterial({ color: 0xdbe2ea, roughness: 0.32, metalness: 0.78, side: THREE.DoubleSide });
    const solarBackMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.7, side: THREE.DoubleSide });
    const solarSafetyYel = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.48 });
    const solarEmeraldMat = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.45 });
    const solarCeramicMat = new THREE.MeshStandardMaterial({ color: 0x7c2d12, roughness: 0.35, metalness: 0.25 });
    const solarWhiteMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.72, side: THREE.DoubleSide });
    const solarLedGreen = new THREE.MeshBasicMaterial({ color: 0x4ade80 });
    const solarFloodMat = new THREE.MeshBasicMaterial({ color: 0xfef9c3 });
    const solarAsphaltMat = new THREE.MeshStandardMaterial({ color: 0x333b48, roughness: 0.9 });
    const chainMeshMat = new THREE.MeshBasicMaterial({ map: makeChainLinkTex(), transparent: true, opacity: 0.85, side: THREE.DoubleSide });

    // 1. Crushed-Gravel Compound Pad + Concrete Curbs + Internal Paved Access Driveway (CLEAR of Substation!)
    const solarPad = new THREE.Mesh(
      new THREE.PlaneGeometry(36, 30),
      new THREE.MeshStandardMaterial({ map: makeSolarYardTex(), roughness: 0.95 })
    );
    solarPad.rotation.x = -Math.PI / 2;
    solarPad.position.y = 0.03;
    solarPad.receiveShadow = true;
    // Internal asphalt driveway entering smoothly from West road (-18, 0) between Substation (North) & BESS Yard (South)!
    const innerRoad = new THREE.Mesh(new THREE.PlaneGeometry(13.2, 5.4), solarAsphaltMat);
    innerRoad.rotation.x = -Math.PI / 2;
    innerRoad.position.set(-11.4, 0.045, 0);
    innerRoad.receiveShadow = true;
    const innerRoadTee = new THREE.Mesh(new THREE.PlaneGeometry(3.4, 24.5), solarAsphaltMat);
    innerRoadTee.rotation.x = -Math.PI / 2;
    innerRoadTee.position.set(-6.5, 0.045, 0);
    innerRoadTee.receiveShadow = true;
    // Crisp yellow safety walkway border lines along the internal service road
    [-2.55, 2.55].forEach((lz) => {
      const stripe = new THREE.Mesh(new THREE.PlaneGeometry(10.0, 0.14), solarSafetyYel);
      stripe.rotation.x = -Math.PI / 2;
      stripe.position.set(-13.0, 0.052, lz);
      solar.add(stripe);
    });
    solar.add(solarPad, innerRoad, innerRoadTee);

    // 2. Nine Large Framed Dual-Module Photovoltaic Solar Arrays (3 Rows × 3 Cols on East Yard) + String Inverters & Cable Trays!
    [-9.2, 0.0, 9.2].forEach((pz) => {
      // Raised steel cable-tray conduit linking all 3 solar arrays in this row straight West to the Substation / BESS Yard!
      const tray = new THREE.Mesh(new THREE.BoxGeometry(22.5, 0.14, 0.28), silverMat);
      tray.position.set(4.8, 0.12, pz + 1.45);
      solar.add(tray);

      [-1.8, 5.6, 13.0].forEach((px) => {
        const rack = new THREE.Group();
        rack.position.set(px, 0, pz);
        // Twin concrete pier footings + galvanized steel H-posts + diagonal tilt brace struts + torque bar
        [-2.0, 2.0].forEach((ox) => {
          const foot = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.25, 0.45), stoneMat);
          foot.position.set(ox, 0.125, 0);
          const post = new THREE.Mesh(new THREE.BoxGeometry(0.16, 1.85, 0.16), silverMat);
          post.position.set(ox, 1.05, 0); post.castShadow = true;
          const brace = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.55, 0.08), darkMetalMat);
          brace.position.set(ox, 1.15, 0.55); brace.rotation.x = -0.68;
          rack.add(foot, post, brace);
        });
        const torqueTube = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 6.1, 8), darkMetalMat);
        torqueTube.rotation.z = Math.PI / 2;
        torqueTube.position.set(0, 1.92, 0);
        // Compact yellow-and-grey String Inverter / DC Combiner Box mounted beneath each solar rack
        const invBox = new THREE.Mesh(new THREE.BoxGeometry(0.68, 0.58, 0.34), solarWhiteMat);
        invBox.position.set(0, 1.25, 0.18);
        const invAccent = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.16, 0.36), solarSafetyYel);
        invAccent.position.set(0, 1.44, 0.18);
        const invPost = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 1.15, 6), darkMetalMat);
        invPost.position.set(0, 0.58, 0.18);
        // Tilted Silver-Framed Dual Photovoltaic Solar Table (crisp aluminum bezel + glistening blue PV cells + white backsheet)
        const pvTable = new THREE.Group();
        pvTable.position.set(0, 2.05, 0);
        pvTable.rotation.x = 0.42;
        const frame = new THREE.Mesh(new THREE.BoxGeometry(6.35, 0.11, 4.15), solarAlumMat);
        frame.castShadow = true; frame.receiveShadow = true;
        const cells = new THREE.Mesh(new THREE.PlaneGeometry(6.1, 3.92), solarPanelMat);
        cells.rotation.x = -Math.PI / 2; cells.position.y = 0.062;
        const back = new THREE.Mesh(new THREE.PlaneGeometry(6.1, 3.92), solarBackMat);
        back.rotation.x = Math.PI / 2; back.position.y = -0.06;
        // Center aluminum divider rib splitting upper & lower PV module rows
        const midRib = new THREE.Mesh(new THREE.BoxGeometry(6.35, 0.13, 0.07), solarAlumMat);
        pvTable.add(frame, cells, back, midRib);
        rack.add(torqueTube, invBox, invAccent, invPost, pvTable);
        solar.add(rack);
      });
    });

    // 3. NORTH-WEST QUADRANT: High-Voltage Substation Building + Finned Oil-Cooled Transformer + Steel H-Frame HV Gantry!
    const subPad = new THREE.Mesh(new THREE.BoxGeometry(9.6, 0.18, 9.2), stoneMat);
    subPad.position.set(-12.4, 0.09, -9.4); subPad.receiveShadow = true;
    const subBox = new THREE.Mesh(new THREE.BoxGeometry(5.2, 3.4, 4.4), solarWhiteMat);
    subBox.position.set(-10.5, 1.85, -10.4); subBox.castShadow = true; subBox.receiveShadow = true;
    const subRoof = new THREE.Mesh(new THREE.BoxGeometry(5.55, 0.28, 4.75), darkMetalMat);
    subRoof.position.set(-10.5, 3.68, -10.4);
    const subStripe = new THREE.Mesh(new THREE.BoxGeometry(5.28, 0.34, 4.48), solarEmeraldMat);
    subStripe.position.set(-10.5, 3.22, -10.4);
    // Double steel substation doors + louver grille + high-voltage warning sign facing South (+Z) toward driveway!
    const subDoor = new THREE.Mesh(new THREE.BoxGeometry(1.55, 2.25, 0.12), darkMetalMat);
    subDoor.position.set(-11.4, 1.3, -8.18);
    const subLouver = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.95, 0.12), darkMetalMat);
    subLouver.position.set(-9.2, 2.1, -8.18);
    const subSign = new THREE.Mesh(
      new THREE.BoxGeometry(1.3, 0.52, 0.1),
      new THREE.MeshStandardMaterial({ map: makeSolarSignTex('HIGH VOLTAGE', '33 kV SUBSTATION', '#1e293b', '#facc15'), roughness: 0.5 })
    );
    subSign.position.set(-11.4, 2.72, -8.16);
    const subAc = new THREE.Mesh(new THREE.BoxGeometry(1.25, 0.85, 0.72), silverMat);
    subAc.position.set(-9.2, 4.18, -10.2);
    solar.add(subPad, subBox, subRoof, subStripe, subDoor, subLouver, subSign, subAc);

    // Finned Oil-Cooled High-Voltage Power Transformer (-15.2, 0, -10.2) with Cooling Radiators, Conservator Drum & 3 Ceramic Bushings!
    const xfmr = new THREE.Group();
    xfmr.position.set(-15.2, 0.18, -10.2);
    const xfBody = new THREE.Mesh(new THREE.BoxGeometry(2.2, 2.15, 2.8), darkMetalMat);
    xfBody.position.y = 1.08; xfBody.castShadow = true;
    // 8 Steel radiator oil-cooling fins protruding along East & West sides
    for (let f = -1.0; f <= 1.01; f += 0.5) {
      [-1.32, 1.32].forEach((fx) => {
        const fin = new THREE.Mesh(new THREE.BoxGeometry(0.42, 1.72, 0.12), silverMat);
        fin.position.set(fx, 1.12, f); fin.castShadow = true;
        xfmr.add(fin);
      });
    }
    // Cylindrical top oil conservator drum on brackets
    const conserv = new THREE.Mesh(new THREE.CylinderGeometry(0.36, 0.36, 2.1, 10), silverMat);
    conserv.rotation.x = Math.PI / 2; conserv.position.set(-0.55, 2.75, 0);
    // 3 Ribbed Brown Ceramic HV Porcelain Bushings rising from transformer top
    [-0.85, 0, 0.85].forEach((bz) => {
      const bush = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.16, 1.15, 8), solarCeramicMat);
      bush.position.set(0.45, 2.72, bz);
      [2.45, 2.72, 2.98].forEach((ry) => {
        const disc = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.06, 8), solarCeramicMat);
        disc.position.set(0.45, ry, bz); xfmr.add(disc);
      });
      xfmr.add(bush);
    });
    xfmr.add(xfBody, conserv);
    solar.add(xfmr);

    // Steel H-Frame High-Voltage Gantry Portal (-15.2, -5.8) feeding overhead HV lines straight to Utility Pole (154, 114)!
    const gantry = new THREE.Group();
    gantry.position.set(-15.2, 0, -5.8);
    [-1.75, 1.75].forEach((gx) => {
      const col = new THREE.Mesh(new THREE.BoxGeometry(0.24, 7.6, 0.24), silverMat);
      col.position.set(gx, 3.8, 0); col.castShadow = true;
      gantry.add(col);
    });
    const gBeam = new THREE.Mesh(new THREE.BoxGeometry(4.1, 0.28, 0.28), silverMat);
    gBeam.position.set(0, 7.25, 0);
    gantry.add(gBeam);
    const hvWireMat = new THREE.LineBasicMaterial({ color: 0x1e293b });
    [-1.1, 0, 1.1].forEach((ix, idx) => {
      const ins = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.65, 8), solarCeramicMat);
      ins.position.set(ix, 6.82, 0); gantry.add(ins);
      // Overhead HV busbar cable connecting Transformer bushing -> Gantry insulator -> West Utility Pole (-18, 7.38, -4)
      const cPts = [
        new THREE.Vector3(-15.2 + 0.45, 3.45, -10.2 + (idx - 1) * 0.85),
        new THREE.Vector3(-15.2 + ix, 6.5, -5.8),
        new THREE.Vector3(-18.0, 7.35, -4.0 + (idx - 1) * 0.55)
      ];
      solar.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(cPts), hvWireMat));
    });
    solar.add(gantry);

    // 4. SOUTH-WEST QUADRANT: 2 Grid-Scale Battery Energy Storage (BESS Megapack) Containers + Central Inverter!
    const bessPad = new THREE.Mesh(new THREE.BoxGeometry(9.8, 0.18, 9.2), stoneMat);
    bessPad.position.set(-12.2, 0.09, 9.2); bessPad.receiveShadow = true;
    solar.add(bessPad);
    [7.2, 11.2].forEach((bz) => {
      const bess = new THREE.Group();
      bess.position.set(-12.5, 0.18, bz);
      const body = new THREE.Mesh(new THREE.BoxGeometry(6.4, 2.35, 2.15), solarWhiteMat);
      body.position.y = 1.18; body.castShadow = true; body.receiveShadow = true;
      const topCap = new THREE.Mesh(new THREE.BoxGeometry(6.5, 0.18, 2.25), darkMetalMat);
      topCap.position.y = 2.42;
      const stripe = new THREE.Mesh(new THREE.BoxGeometry(6.46, 0.24, 2.2), solarEmeraldMat);
      stripe.position.y = 2.08;
      // 4 Cabinet doors + status green LED bar + 3 rooftop cooling HVAC fan units!
      [-2.1, -0.7, 0.7, 2.1].forEach((dx) => {
        const door = new THREE.Mesh(new THREE.BoxGeometry(1.15, 1.55, 2.2), silverMat);
        door.position.set(dx, 0.98, 0);
        const vent = new THREE.Mesh(new THREE.BoxGeometry(0.85, 0.35, 2.24), darkMetalMat);
        vent.position.set(dx, 0.45, 0);
        bess.add(door, vent);
      });
      [-1.8, 0, 1.8].forEach((fx) => {
        const fan = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.45, 0.26, 10), darkMetalMat);
        fan.position.set(fx, 2.62, 0); bess.add(fan);
      });
      const ledStrip = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.65, 0.14), solarLedGreen);
      ledStrip.position.set(-3.22, 1.55, -0.75);
      bess.add(body, topCap, stripe, ledStrip);
      solar.add(bess);
    });
    // Yellow safety bollards protecting BESS battery yard along driveway edge
    [-15.8, -13.2, -10.6, -8.2].forEach((bx) => {
      const bol = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.95, 8), solarSafetyYel);
      bol.position.set(bx, 0.55, 5.1); solar.add(bol);
    });

    // 5. Industrial West Entrance Gate Arch ("VALLEY SOLAR · 12 MW"), Open Sliding Gate, Chain-Link Fence & 4 Floodlight Towers!
    [-4.2, 4.2].forEach((gz) => {
      const gPost = new THREE.Mesh(new THREE.BoxGeometry(0.28, 4.5, 0.28), darkMetalMat);
      gPost.position.set(-18.0, 2.25, gz); gPost.castShadow = true;
      const gBol = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 1.0, 8), solarSafetyYel);
      gBol.position.set(-18.6, 0.5, gz * 0.85);
      solar.add(gPost, gBol);
    });
    const gateHeader = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.32, 8.7), darkMetalMat);
    gateHeader.position.set(-18.0, 4.45, 0);
    const gateSign = new THREE.Mesh(
      new THREE.BoxGeometry(0.16, 1.05, 4.6),
      new THREE.MeshStandardMaterial({ map: makeSolarSignTex('VALLEY SOLAR', '12 MW CLEAN ENERGY PLANT', '#0f172a', '#10b981'), roughness: 0.45 })
    );
    gateSign.position.set(-18.14, 4.45, 0);
    // Open retracted yellow-accented sliding steel gate parked neatly along South fence
    const slideGate = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.75, 7.6), darkMetalMat);
    slideGate.position.set(-17.65, 0.95, 8.2);
    const slideStripe = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.22, 7.6), solarSafetyYel);
    slideStripe.position.set(-17.65, 1.0, 8.2);
    solar.add(gateHeader, gateSign, slideGate, slideStripe);

    // Pyranometer Solar Irradiance Weather Mast inside Solar Compound (-6.2, -5.0)
    const pyPole = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.07, 3.1, 8), silverMat);
    pyPole.position.set(-6.2, 1.55, -5.0);
    const pySensor = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.22, 0.16, 10), solarWhiteMat);
    pySensor.position.set(-6.2, 3.16, -5.0);
    const pySol = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.05, 0.4), solarPanelMat);
    pySol.position.set(-6.2, 2.25, -4.8); pySol.rotation.x = 0.5;
    solar.add(pyPole, pySensor, pySol);

    // Grounded Chain-Link Perimeter Security Fence with Angled Top Security Outriggers + 4 Corner Security Floodlight Towers!
    const addSolarFenceRun = (x1, z1, x2, z2) => {
      const len = Math.hypot(x2 - x1, z2 - z1);
      const ang = Math.atan2(x2 - x1, z2 - z1);
      const fg = new THREE.Group();
      fg.position.set((x1 + x2) / 2, 0, (z1 + z2) / 2);
      fg.rotation.y = ang;
      const meshPanel = new THREE.Mesh(new THREE.PlaneGeometry(len, 1.85), chainMeshMat);
      meshPanel.rotation.y = Math.PI / 2;
      meshPanel.position.y = 0.95;
      fg.add(meshPanel);
      [0.12, 0.98, 1.86].forEach((ry) => {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.07, len), silverMat);
        rail.position.y = ry; fg.add(rail);
      });
      const nPosts = Math.max(2, Math.round(len / 4.5) + 1);
      for (let i = 0; i < nPosts; i++) {
        const pz = -len / 2 + (i / (nPosts - 1)) * len;
        const p = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.075, 1.95, 6), silverMat);
        p.position.set(0, 0.975, pz);
        const arm = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.45, 0.05), silverMat);
        arm.position.set(0.12, 2.08, pz); arm.rotation.z = -0.55;
        fg.add(p, arm);
      }
      solar.add(fg);
    };
    addSolarFenceRun(-18, -15, 18, -15);
    addSolarFenceRun(-18, 15, 18, 15);
    addSolarFenceRun(18, -15, 18, 15);
    addSolarFenceRun(-18, -15, -18, -4.2);
    addSolarFenceRun(-18, 4.2, -18, 15);

    // 4 Tall Compound Security Floodlight Poles at all 4 Corners
    [[-17.4, -14.4], [17.4, -14.4], [-17.4, 14.4], [17.4, 14.4]].forEach(([fx, fz]) => {
      const fPole = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.14, 7.2, 8), darkMetalMat);
      fPole.position.set(fx, 3.6, fz); fPole.castShadow = true;
      const fHead = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.25, 0.42), darkMetalMat);
      fHead.position.set(fx, 7.25, fz); fHead.rotation.y = Math.atan2(-fx, -fz);
      const fGlow = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.1, 0.34), solarFloodMat);
      fGlow.position.set(fx, 7.12, fz); fGlow.rotation.y = fHead.rotation.y;
      solar.add(fPole, fHead, fGlow);
    });

    this.scene.add(solar);
    // Accurate solid colliders for NW Substation Building & SW BESS Battery Containers (West entrance driveway stays WIDE OPEN!)
    this.addBoxCollider(172 - 10.5, 1.9, 118 - 10.4, 5.6, 3.8, 4.8);
    this.addBoxCollider(172 - 12.5, 1.4, 118 + 9.2, 6.8, 2.8, 6.6);

    // =========================================================================
    // LANDMARK 4 (-108, -110): Coastal Masonry Lighthouse (Overlooking Expanded Twin-Basin Alpine Lake!)
    // =========================================================================
    const lh = new THREE.Group();
    lh.position.set(-108, 0, -110);

    // Procedural red-and-white painted stone-brick masonry texture for the lighthouse tower
    const makeLighthouseTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 512;
      const ctx = c.getContext('2d');
      // 4 bold alternating nautical maritime bands (White / Crimson / White / Crimson)
      const bands = ['#f4f5f7', '#c81e1e', '#f4f5f7', '#c81e1e'];
      const bandH = 512 / bands.length;
      bands.forEach((col, bi) => {
        ctx.fillStyle = col;
        ctx.fillRect(0, bi * bandH, 256, bandH);
      });
      // Stone-brick masonry courses & subtle coastal weathering mortar lines
      for (let y = 0; y < 512; y += 16) {
        ctx.fillStyle = 'rgba(30, 41, 59, 0.16)';
        ctx.fillRect(0, y, 256, 1.5);
        const off = ((y / 16) % 2) * 16;
        for (let x = off; x < 256; x += 32) {
          ctx.fillRect(x, y, 1.5, 16);
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.ClampToEdgeWrapping;
      tex.repeat.set(3, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const lhMasonryMat = new THREE.MeshStandardMaterial({ map: makeLighthouseTex(), roughness: 0.78, side: THREE.DoubleSide });
    const lhWhite = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.78, side: THREE.DoubleSide });
    const lhRed = new THREE.MeshStandardMaterial({ color: 0xb91c1c, roughness: 0.62, side: THREE.DoubleSide });
    const lhCopperMat = new THREE.MeshStandardMaterial({ color: 0x991b1b, roughness: 0.42, metalness: 0.45, side: THREE.DoubleSide });
    const lhBrassMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.28, metalness: 0.85 });
    const lhGlassMat = new THREE.MeshStandardMaterial({ color: 0x7dd3fc, roughness: 0.08, metalness: 0.6, transparent: true, opacity: 0.42, side: THREE.DoubleSide });
    const lhWarmWindowMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.15, metalness: 0.65, emissive: 0x854d0e, emissiveIntensity: 0.55 });
    const lhGlowMat = new THREE.MeshBasicMaterial({ color: 0xfef08a });

    // 1. Stone Terrace Apron + Coastal Outcrop Boulders + Stepped Masonry Foundation Base
    const lhApron = new THREE.Mesh(new THREE.CylinderGeometry(7.2, 7.8, 0.36, 20), stoneMat);
    lhApron.position.y = 0.18;
    lhApron.receiveShadow = true;
    const lhBase = new THREE.Mesh(new THREE.CylinderGeometry(3.25, 4.1, 2.45, 20), stoneMat);
    lhBase.position.y = 1.42;
    lhBase.castShadow = true;
    lhBase.receiveShadow = true;
    const lhBaseTrim = new THREE.Mesh(new THREE.CylinderGeometry(3.42, 3.45, 0.34, 20), mossStoneMat);
    lhBaseTrim.position.y = 2.72;
    lh.add(lhApron, lhBase, lhBaseTrim);

    // Coastal outcrop boulders framing the lighthouse rocky promontory
    for (let b = 0; b < 9; b++) {
      const ba = (b / 9) * Math.PI * 2 + 0.3;
      const dist = 5.4 + (b % 3) * 0.9;
      if (Math.cos(ba) < -0.35 && Math.abs(Math.sin(ba)) < 0.55) continue; // Keep Keeper's Cottage side clear
      const bs = 0.55 + (b % 4) * 0.22;
      const boulder = new THREE.Mesh(new THREE.DodecahedronGeometry(bs, 0), (b % 2 === 0) ? mossStoneMat : stoneMat);
      boulder.position.set(Math.cos(ba) * dist, bs * 0.42 + 0.1, Math.sin(ba) * dist);
      boulder.scale.set(1.3, 0.72, 1.1);
      boulder.castShadow = true;
      lh.add(boulder);
    }

    // 2. Smooth 24-Sided Painted Stone-Brick Lighthouse Tower Shaft (y = 2.7 .. 14.5)
    const lhBotY = 2.7;
    const lhShaftH = 11.8;
    const lhBotR = 3.12;
    const lhTopR = 1.88;
    const lhTaper = Math.atan2(lhBotR - lhTopR, lhShaftH);
    const lhRadAt = (y) => lhBotR - ((y - lhBotY) / lhShaftH) * (lhBotR - lhTopR);

    const lhShaft = new THREE.Mesh(new THREE.CylinderGeometry(lhTopR, lhBotR, lhShaftH, 24), lhMasonryMat);
    lhShaft.position.y = lhBotY + lhShaftH / 2;
    lhShaft.castShadow = true;
    lhShaft.receiveShadow = true;
    lh.add(lhShaft);

    // 3 Carved Stone Stringcourse Molding Rings dividing the maritime stripes cleanly
    [5.65, 8.6, 11.55].forEach((ry) => {
      const rr = lhRadAt(ry);
      const ring = new THREE.Mesh(new THREE.CylinderGeometry(rr + 0.14, rr + 0.16, 0.24, 24), stoneMat);
      ring.position.y = ry;
      lh.add(ring);
    });

    // Protruding Stone Entrance Portico with Arched Door, Steps & Warm Brass Lantern facing the Lake!
    const portico = new THREE.Group();
    portico.rotation.y = -0.68; // Points Southeast (+X/+Z) directly toward the Lake!
    const portBody = new THREE.Mesh(new THREE.BoxGeometry(1.75, 2.55, 1.85), stoneMat);
    portBody.position.set(3.55, 1.48, 0);
    portBody.castShadow = true;
    const portRoof = new THREE.Mesh(new THREE.ConeGeometry(1.45, 0.75, 4), lhRed);
    portRoof.position.set(3.55, 3.12, 0);
    portRoof.rotation.y = Math.PI / 4;
    const portFrame = new THREE.Mesh(new THREE.BoxGeometry(0.18, 2.05, 1.18), lhWhite);
    portFrame.position.set(4.42, 1.32, 0);
    const portDoor = new THREE.Mesh(new THREE.BoxGeometry(0.22, 1.9, 0.98), lhRed);
    portDoor.position.set(4.44, 1.28, 0);
    const portKnob = new THREE.Mesh(new THREE.SphereGeometry(0.06, 6, 6), lhBrassMat);
    portKnob.position.set(4.58, 1.24, 0.3);
    const portLantern = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.28, 0.18), lhGlowMat);
    portLantern.position.set(4.56, 2.18, -0.72);
    const portStep = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.26, 1.55), stoneMat);
    portStep.position.set(4.78, 0.28, 0);
    portico.add(portBody, portRoof, portFrame, portDoor, portKnob, portLantern, portStep);
    lh.add(portico);

    // 6 Framed Multi-Pane Lighthouse Windows with Projecting Stone Sills (tilted flush to shaft taper!)
    [0.65, -0.85, 2.25].forEach((ang, idx) => {
      const wYList = (idx === 0) ? [4.4, 7.4, 10.4, 13.0] : [6.8, 10.2];
      wYList.forEach((wy) => {
        const wr = lhRadAt(wy);
        const wg = new THREE.Group();
        wg.rotation.y = ang;
        const winBox = new THREE.Group();
        winBox.position.set(wr + 0.04, wy, 0);
        winBox.rotation.z = lhTaper;
        const frame = new THREE.Mesh(new THREE.BoxGeometry(0.2, 1.05, 0.78), darkMetalMat);
        const pane = new THREE.Mesh(new THREE.BoxGeometry(0.23, 0.85, 0.58), lhWarmWindowMat);
        const barH = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.06, 0.62), lhWhite);
        const barV = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.88, 0.06), lhWhite);
        const sill = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.12, 0.92), stoneMat);
        sill.position.set(0.06, -0.56, 0);
        winBox.add(frame, pane, barH, barV, sill);
        wg.add(winBox);
        lh.add(wg);
      });
    });

    // 3. Twelve Carved Stone Corbel Brackets + Cantilevered Gallery Walkway + Grounded Iron Railing!
    for (let c = 0; c < 12; c++) {
      const ca = (c / 12) * Math.PI * 2;
      const cg = new THREE.Group();
      cg.rotation.y = ca;
      const corb = new THREE.Mesh(new THREE.BoxGeometry(0.3, 1.35, 0.24), stoneMat);
      corb.position.set(2.22, 13.9, 0);
      corb.rotation.z = -0.52;
      cg.add(corb);
      lh.add(cg);
    }

    const lhDeck = new THREE.Mesh(new THREE.CylinderGeometry(2.92, 2.32, 0.52, 24), darkMetalMat);
    lhDeck.position.y = 14.55;
    lhDeck.castShadow = true;
    const lhDeckEdge = new THREE.Mesh(new THREE.CylinderGeometry(2.98, 2.98, 0.14, 24), lhWhite);
    lhDeckEdge.position.y = 14.8;
    lh.add(lhDeck, lhDeckEdge);

    // Grounded Wrought-Iron Gallery Railing (Top Ring + Mid Ring + 16 Vertical Posts!)
    const lhRailTop = new THREE.Mesh(new THREE.TorusGeometry(2.76, 0.055, 8, 28), darkMetalMat);
    lhRailTop.rotation.x = Math.PI / 2;
    lhRailTop.position.y = 15.72;
    const lhRailMid = new THREE.Mesh(new THREE.TorusGeometry(2.76, 0.035, 6, 28), darkMetalMat);
    lhRailMid.rotation.x = Math.PI / 2;
    lhRailMid.position.y = 15.28;
    lh.add(lhRailTop, lhRailMid);
    for (let p = 0; p < 16; p++) {
      const pa = (p / 16) * Math.PI * 2;
      const rPost = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.94, 6), darkMetalMat);
      rPost.position.set(Math.cos(pa) * 2.76, 15.28, Math.sin(pa) * 2.76);
      lh.add(rPost);
    }

    // 4. 360-Degree Faceted Glass Lantern Room + Rotating Fresnel Light Beam + Ribbed Cupola Dome!
    const lhWatchBase = new THREE.Mesh(new THREE.CylinderGeometry(1.88, 1.94, 0.92, 24), lhRed);
    lhWatchBase.position.y = 15.25;
    const lhGlassRoom = new THREE.Mesh(new THREE.CylinderGeometry(1.72, 1.72, 2.35, 24), lhGlassMat);
    lhGlassRoom.position.y = 16.88;
    lh.add(lhWatchBase, lhGlassRoom);

    // 12 Wrought-Iron Vertical Lantern Room Mullion Bars + Mid Ring
    for (let m = 0; m < 12; m++) {
      const ma = (m / 12) * Math.PI * 2;
      const bar = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 2.42, 6), darkMetalMat);
      bar.position.set(Math.cos(ma) * 1.75, 16.88, Math.sin(ma) * 1.75);
      lh.add(bar);
    }
    const lhGlassMidRing = new THREE.Mesh(new THREE.TorusGeometry(1.75, 0.045, 6, 24), darkMetalMat);
    lhGlassMidRing.rotation.x = Math.PI / 2;
    lhGlassMidRing.position.y = 16.88;
    lh.add(lhGlassMidRing);

    // Rotating Golden Fresnel Lens Core & Twin Sweeping Light Cones!
    this.lighthouseBeam = new THREE.Group();
    this.lighthouseBeam.position.set(0, 16.88, 0);
    const fresnelCore = new THREE.Mesh(new THREE.OctahedronGeometry(0.82, 1), lhGlowMat);
    const fresnelPedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.72, 0.9, 10), lhBrassMat);
    fresnelPedestal.position.y = -0.82;
    this.lighthouseBeam.add(fresnelCore, fresnelPedestal);

    const beamMat = new THREE.MeshBasicMaterial({
      color: 0xfef08a,
      transparent: true,
      opacity: 0.14,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    [-1, 1].forEach((dir) => {
      const cone = new THREE.Mesh(new THREE.ConeGeometry(3.8, 22, 14, 1, true), beamMat);
      cone.rotation.z = dir * (Math.PI / 2);
      cone.position.x = dir * 11.4;
      this.lighthouseBeam.add(cone);
    });
    lh.add(this.lighthouseBeam);

    // Overhanging Cornice + Ribbed Crimson-Copper Cupola Dome + Brass Ball + Lightning Rod & Weathervane
    const lhCornice = new THREE.Mesh(new THREE.CylinderGeometry(2.15, 1.92, 0.34, 24), darkMetalMat);
    lhCornice.position.y = 18.18;
    const lhDome = new THREE.Mesh(new THREE.SphereGeometry(1.88, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2), lhCopperMat);
    lhDome.position.y = 18.32;
    lhDome.scale.set(1, 0.88, 1);
    lhDome.castShadow = true;
    const lhVentBall = new THREE.Mesh(new THREE.SphereGeometry(0.32, 10, 10), lhBrassMat);
    lhVentBall.position.y = 20.12;
    const lhSpire = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.07, 1.75, 8), lhBrassMat);
    lhSpire.position.y = 21.05;
    const lhVane = new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.12, 0.04), lhBrassMat);
    lhVane.position.y = 21.35;
    lh.add(lhCornice, lhDome, lhVentBall, lhSpire, lhVane);

    // 5. Upgraded Lighthouse Keeper's Cottage attached beside Lighthouse Base (-5.4, 0, 0.6)
    const cotGrp = new THREE.Group();
    cotGrp.position.set(-5.4, 0, 0.6);
    const cotPlinth = new THREE.Mesh(new THREE.BoxGeometry(5.2, 0.42, 4.4), stoneMat);
    cotPlinth.position.y = 0.21;
    const cotBody = new THREE.Mesh(new THREE.BoxGeometry(4.8, 2.8, 4.0), lhWhite);
    cotBody.position.y = 1.82;
    cotBody.castShadow = true;
    cotBody.receiveShadow = true;

    // Watertight pitched crimson gable roof with solid extruded gable prism (ZERO roof gaps!)
    const cotHalfW = 2.4;
    const cotPeakH = 1.65;
    const cotPitch = Math.atan2(cotPeakH, cotHalfW);
    const cotRun = cotHalfW + 0.42;
    const cotDrop = cotRun * Math.tan(cotPitch);
    const cotSlopeLen = Math.hypot(cotRun, cotDrop) + 0.1;
    const cotMidY = (3.22 + cotPeakH) - cotDrop / 2;
    const cotRoofL = new THREE.Mesh(new THREE.BoxGeometry(cotSlopeLen, 0.2, 4.8), lhRed);
    cotRoofL.position.set(-cotRun / 2 - Math.sin(cotPitch) * 0.06, cotMidY + Math.cos(cotPitch) * 0.06, 0);
    cotRoofL.rotation.z = cotPitch;
    cotRoofL.castShadow = true;
    const cotRoofR = new THREE.Mesh(new THREE.BoxGeometry(cotSlopeLen, 0.2, 4.8), lhRed);
    cotRoofR.position.set(cotRun / 2 + Math.sin(cotPitch) * 0.06, cotMidY + Math.cos(cotPitch) * 0.06, 0);
    cotRoofR.rotation.z = -cotPitch;
    cotRoofR.castShadow = true;
    const cotRidge = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.22, 4.86), darkMetalMat);
    cotRidge.position.set(0, 3.22 + cotPeakH + 0.1, 0);
    const cotGableShape = new THREE.Shape([
      new THREE.Vector2(-cotHalfW, 0),
      new THREE.Vector2(cotHalfW, 0),
      new THREE.Vector2(0, cotPeakH + 0.02)
    ]);
    const cotGable = new THREE.Mesh(new THREE.ExtrudeGeometry(cotGableShape, { depth: 4.0, bevelEnabled: false }), lhWhite);
    cotGable.position.set(0, 3.22, -2.0);
    cotGable.castShadow = true;

    // Cottage Chimney, Framed Front Door + Canopy + Steps, & Shuttered Multi-Pane Windows!
    const cotChim = new THREE.Mesh(new THREE.BoxGeometry(0.75, 2.2, 0.75), stoneMat);
    cotChim.position.set(-1.35, 4.65, -0.8);
    const cotChimCap = new THREE.Mesh(new THREE.BoxGeometry(0.92, 0.14, 0.92), darkMetalMat);
    cotChimCap.position.set(-1.35, 5.78, -0.8);
    const cotDoorFrame = new THREE.Mesh(new THREE.BoxGeometry(1.1, 1.98, 0.16), darkMetalMat);
    cotDoorFrame.position.set(-0.9, 1.4, 2.02);
    const cotDoor = new THREE.Mesh(new THREE.BoxGeometry(0.92, 1.85, 0.2), lhRed);
    cotDoor.position.set(-0.9, 1.38, 2.03);
    const cotDoorAwning = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.14, 0.75), lhRed);
    cotDoorAwning.position.set(-0.9, 2.52, 2.34);
    cotDoorAwning.rotation.x = 0.22;
    const cotStep = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.24, 0.65), stoneMat);
    cotStep.position.set(-0.9, 0.12, 2.42);

    // Cottage shuttered windows (front, back & west wall)
    [[1.05, 1.85, 2.02, 0], [-0.4, 1.85, -2.02, Math.PI], [-2.42, 1.85, 0, -Math.PI / 2]].forEach(([wx, wy, wz, ry]) => {
      const cw = new THREE.Group();
      cw.position.set(wx, wy, wz);
      cw.rotation.y = ry;
      const cf = new THREE.Mesh(new THREE.BoxGeometry(1.18, 1.05, 0.14), darkMetalMat);
      const cp = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.88, 0.18), lhWarmWindowMat);
      const csL = new THREE.Mesh(new THREE.BoxGeometry(0.24, 1.02, 0.08), lhRed);
      csL.position.set(-0.72, 0, 0.04);
      const csR = csL.clone();
      csR.position.x = 0.72;
      cw.add(cf, cp, csL, csR);
      cotGrp.add(cw);
    });

    cotGrp.add(cotPlinth, cotBody, cotRoofL, cotRoofR, cotRidge, cotGable, cotChim, cotChimCap, cotDoorFrame, cotDoor, cotDoorAwning, cotStep);
    lh.add(cotGrp);
    this.scene.add(lh);

    // Accurate solid colliders for both Lighthouse Tower & Keeper's Cottage
    this.addCylCollider(-108, -110, 3.85, 0, 19.6);
    this.addBoxCollider(-113.4, 2.5, -109.4, 5.4, 5.1, 4.8);
  }

  buildRoadNetwork() {
    // =========================================================================
    // 4-TIER SEAMLESSLY INTERCONNECTED VALLEY ROAD, LANE, GRAVEL & FOOTPATH NETWORK
    // (Every single house, farm, pier, tower, landmark & energy site is linked!)
    // =========================================================================

    // 1. Procedural Textures for All 4 Road Tiers
    const makeMainAsphaltRoadTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#333b48';
      ctx.fillRect(0, 0, 256, 256);
      // Subtle weathered aggregate grain
      for (let i = 0; i < 900; i++) {
        const x = (i * 53 + (i % 13) * 19) % 256;
        const y = (i * 37 + (i % 17) * 11) % 256;
        ctx.fillStyle = (i % 2 === 0) ? 'rgba(255,255,255,0.05)' : 'rgba(10,14,20,0.14)';
        ctx.fillRect(x, y, 2, 2);
      }
      // Subtle asphalt shoulder curbs & crisp white edge stripes
      ctx.fillStyle = '#525b68';
      ctx.fillRect(0, 0, 12, 256);
      ctx.fillRect(244, 0, 12, 256);
      ctx.fillStyle = 'rgba(241, 245, 249, 0.78)';
      ctx.fillRect(16, 0, 5, 256);
      ctx.fillRect(235, 0, 5, 256);
      // Warm amber/white dashed centerline
      ctx.fillStyle = '#facc15';
      ctx.fillRect(125, 20, 6, 92);
      ctx.fillRect(125, 148, 6, 92);
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.ClampToEdgeWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeVillageCobbleTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#57606f';
      ctx.fillRect(0, 0, 256, 256);
      // Stone borders along both edges
      ctx.fillStyle = '#94a3b8';
      ctx.fillRect(0, 0, 18, 256);
      ctx.fillRect(238, 0, 18, 256);
      for (let y = 0; y < 256; y += 16) {
        const off = ((y / 16) % 2) * 14;
        for (let x = 18; x < 238; x += 28) {
          const sx = x + off;
          if (sx + 24 <= 240) {
            const shades = ['#7c8696', '#6c7686', '#8a94a4', '#636c7a'];
            ctx.fillStyle = shades[((x + y) / 16) & 3];
            ctx.fillRect(sx + 2, y + 2, 22, 12);
          }
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.ClampToEdgeWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeGravelRoadTex = () => {
      const c = document.createElement('canvas');
      c.width = 256;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#968367';
      ctx.fillRect(0, 0, 256, 256);
      // Soft grassy shoulders blending naturally into valley grass
      ctx.fillStyle = 'rgba(72, 92, 54, 0.45)';
      ctx.fillRect(0, 0, 18, 256);
      ctx.fillRect(238, 0, 18, 256);
      // Subtle packed dual rural wheel ruts
      ctx.fillStyle = 'rgba(110, 90, 62, 0.42)';
      ctx.fillRect(54, 0, 38, 256);
      ctx.fillRect(164, 0, 38, 256);
      // Gravel pebbles & earth speckles
      for (let i = 0; i < 850; i++) {
        const x = (i * 71 + (i % 11) * 23) % 256;
        const y = (i * 43 + (i % 19) * 13) % 256;
        const cols = ['#b8a586', '#7a6950', '#c7b89d', '#685942'];
        ctx.fillStyle = cols[i % 4];
        ctx.fillRect(x, y, 3, 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.ClampToEdgeWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      if (this.renderer) tex.anisotropy = Math.min(4, this.renderer.capabilities.getMaxAnisotropy());
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeFootpathTex = () => {
      const c = document.createElement('canvas');
      c.width = 128;
      c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#8b7b62';
      ctx.fillRect(0, 0, 128, 256);
      // Natural warm stone flagstones embedded in packed earth trail
      for (let y = 8; y < 256; y += 36) {
        const sx = ((y / 36) % 2 === 0) ? 24 : 38;
        ctx.fillStyle = '#b4aba0';
        ctx.beginPath();
        ctx.ellipse(sx + 28, y + 10, 34, 13, ((y % 72 === 8) ? 0.08 : -0.08), 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = 'rgba(72, 62, 48, 0.45)';
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.ClampToEdgeWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const asphaltRoadMat = new THREE.MeshStandardMaterial({
      map: makeMainAsphaltRoadTex(), roughness: 0.88, metalness: 0.04,
      polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4
    });
    const cobbleRoadMat = new THREE.MeshStandardMaterial({
      map: makeVillageCobbleTex(), roughness: 0.9,
      polygonOffset: true, polygonOffsetFactor: -3, polygonOffsetUnits: -3
    });
    const footpathMat = new THREE.MeshStandardMaterial({
      map: makeFootpathTex(), roughness: 0.94,
      polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2
    });
    const gravelRoadMat = new THREE.MeshStandardMaterial({
      map: makeGravelRoadTex(), roughness: 0.95,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1
    });

    // Smoothly rounds corners of any polyline using non-overlapping quadratic Bezier fillets!
    const filletPolyline = (pts, filletDist = 6.0) => {
      if (pts.length <= 2) return pts.slice();
      const out = [pts[0]];
      for (let i = 1; i < pts.length - 1; i++) {
        const pPrev = pts[i - 1];
        const pCurr = pts[i];
        const pNext = pts[i + 1];
        const inDx = pCurr[0] - pPrev[0], inDz = pCurr[1] - pPrev[1];
        const outDx = pNext[0] - pCurr[0], outDz = pNext[1] - pCurr[1];
        const lenIn = Math.hypot(inDx, inDz);
        const lenOut = Math.hypot(outDx, outDz);
        if (lenIn < 0.1 || lenOut < 0.1) continue;
        const uInX = inDx / lenIn, uInZ = inDz / lenIn;
        const uOutX = outDx / lenOut, uOutZ = outDz / lenOut;
        const dot = uInX * uOutX + uInZ * uOutZ;
        if (dot > 0.992) {
          out.push(pCurr);
          continue;
        }
        const d = Math.min(filletDist, lenIn * 0.42, lenOut * 0.42);
        const ax = pCurr[0] - uInX * d, az = pCurr[1] - uInZ * d;
        const bx = pCurr[0] + uOutX * d, bz = pCurr[1] + uOutZ * d;
        const arcSteps = 6;
        for (let s = 0; s <= arcSteps; s++) {
          const t = s / arcSteps;
          const mt = 1 - t;
          const qx = mt * mt * ax + 2 * mt * t * pCurr[0] + t * t * bx;
          const qz = mt * mt * az + 2 * mt * t * pCurr[1] + t * t * bz;
          out.push([qx, qz]);
        }
      }
      out.push(pts[pts.length - 1]);
      return out;
    };

    // Single continuous non-overlapping terrain-conforming road ribbon (ZERO Z-fighting caps or corner overlap!)
    const addRoadPolyline = (rawPts, width, yOff, mat, uvTileLen = 8.0, filletDist = 5.5) => {
      const smooth = filletPolyline(rawPts, filletDist);
      // Subdivide long straight stretches so outer hills conform seamlessly to getTerrainHeight()
      const pts = [smooth[0]];
      for (let i = 0; i < smooth.length - 1; i++) {
        const [x1, z1] = smooth[i];
        const [x2, z2] = smooth[i + 1];
        const len = Math.hypot(x2 - x1, z2 - z1);
        if (len < 0.05) continue;
        const subs = Math.max(1, Math.ceil(len / 8));
        for (let s = 1; s <= subs; s++) {
          const t = s / subs;
          pts.push([x1 + (x2 - x1) * t, z1 + (z2 - z1) * t]);
        }
      }
      if (pts.length < 2) return;

      const halfW = width / 2;
      const positions = [];
      const uvs = [];
      const indices = [];
      let cumDist = 0;

      for (let i = 0; i < pts.length; i++) {
        const [cx, cz] = pts[i];
        if (i > 0) {
          cumDist += Math.hypot(cx - pts[i - 1][0], cz - pts[i - 1][1]);
        }

        let tx = 0, tz = 0;
        if (i === 0) {
          const dx = pts[1][0] - cx, dz = pts[1][1] - cz;
          const l = Math.hypot(dx, dz) || 1;
          tx = dx / l; tz = dz / l;
        } else if (i === pts.length - 1) {
          const dx = cx - pts[i - 1][0], dz = cz - pts[i - 1][1];
          const l = Math.hypot(dx, dz) || 1;
          tx = dx / l; tz = dz / l;
        } else {
          const dx1 = cx - pts[i - 1][0], dz1 = cz - pts[i - 1][1];
          const l1 = Math.hypot(dx1, dz1) || 1;
          const dx2 = pts[i + 1][0] - cx, dz2 = pts[i + 1][1] - cz;
          const l2 = Math.hypot(dx2, dz2) || 1;
          tx = dx1 / l1 + dx2 / l2;
          tz = dz1 / l1 + dz2 / l2;
          const tl = Math.hypot(tx, tz) || 1;
          tx /= tl; tz /= tl;
        }

        const nx = -tz;
        const nz = tx;
        const lx = cx - nx * halfW;
        const lz = cz - nz * halfW;
        const rx = cx + nx * halfW;
        const rz = cz + nz * halfW;
        const ly = this.getTerrainHeight(lx, lz) + yOff;
        const ry = this.getTerrainHeight(rx, rz) + yOff;
        const v = cumDist / uvTileLen;

        positions.push(lx, ly, lz, rx, ry, rz);
        uvs.push(0, v, 1, v);

        if (i < pts.length - 1) {
          const b = i * 2;
          indices.push(b, b + 1, b + 2, b + 1, b + 3, b + 2);
        }
      }

      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
      geo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
      geo.setIndex(indices);
      geo.computeVertexNormals();

      const mesh = new THREE.Mesh(geo, mat);
      mesh.receiveShadow = true;
      this.scene.add(mesh);
    };

    // --- TIER 1: MAIN PAVED ASPHALT COLLECTOR ROAD (y = 0.056, width = 5.6m, halfW = 2.8m — Spacious & ZERO overlap!) ---
    // ONE continuous smooth filleted grand loop: Hangar Apron South edge (54, 5.05) -> North & East Village (134, 12..78)
    // -> South Valley Corridor (-2, 78) -> West of Runway (-10.5) -> Control Tower Apron edge (-11.3, -26.85)!
    addRoadPolyline([
      [54, 5.05], [54, 12], [134, 12], [134, 78], [-2, 78], [-2, 46], [-10.5, 36], [-10.5, -24], [-11.3, -26.85]
    ], 5.6, 0.056, asphaltRoadMat, 9.0, 6.2);

    // Solar Power Farm Paved Branch: starts STRICTLY FLUSH at East edge of straight x = 134 (136.8, 68) -> Solar West Gate (154.0, 118)!
    addRoadPolyline([
      [136.8, 68], [148.0, 68], [148.0, 118], [154.0, 118]
    ], 5.2, 0.056, asphaltRoadMat, 9.0, 4.5);

    // Short Paved Helipad Access Spur: starts STRICTLY FLUSH at East edge of x = -10.5 (-7.7, 0) -> Helipad West rim (-6.2, 0)
    addRoadPolyline([[-7.7, 0], [-6.2, 0]], 3.6, 0.056, asphaltRoadMat, 6.0, 0);

    // --- TIER 2: COBBLESTONE VILLAGE STREETS (y = 0.044 — Spacious blocks, STRICTLY FLUSH edge-to-edge!) ---
    // Main Street (North-South along x = 71, flush between North Asphalt South edge 14.8 & South Asphalt North edge 75.2!)
    addRoadPolyline([[71, 14.8], [71, 75.2]], 5.0, 0.044, cobbleRoadMat, 6.5, 0);
    // East Market Avenue (along z = 31, flush between Main St East edge 73.5 & East Asphalt West edge 131.2!)
    addRoadPolyline([[73.5, 31], [131.2, 31]], 4.8, 0.044, cobbleRoadMat, 6.5, 0);
    // South Village Residential Lane (along z = 62, flush between Main St East edge 73.5 & East Asphalt West edge 131.2!)
    addRoadPolyline([[73.5, 62], [131.2, 62]], 4.4, 0.044, cobbleRoadMat, 6.5, 0);

    // --- TIER 3: PACKED GRAVEL / DIRT ACCESS ROADS (y = 0.024 — STRICTLY FLUSH edge-to-edge on straight parent edges!) ---
    // North-East 5G Telecom Tower Road: starts flush at Hangar Apron North edge (66, -29.0) -> 5G Pad South edge (75, -64.25)!
    addRoadPolyline([[66, -29.0], [66, -54], [75, -64.25]], 3.6, 0.024, gravelRoadMat, 7.0, 5.0);

    // South-West Farming & Homestead Loop: starts FLUSH at South edge of straight z = 78 (10.0, 80.8) -> sweeps spaciously past
    // Windmill, Red Barn & Farmhouse (z = 102) -> loops North around SW fields -> joins FLUSH at West edge of x = -10.5 (-13.3, 22)!
    addRoadPolyline([
      [10.0, 80.8], [10.0, 85.5], [-21.0, 85.5], [-26, 102], [-76, 102], [-76, 46], [-48, 46], [-48, 22], [-13.3, 22]
    ], 3.8, 0.024, gravelRoadMat, 7.0, 4.2);

    // Flush Rural Driveways & Spurs (every start point sits cleanly on a STRAIGHT outer edge of its parent road!)
    addRoadPolyline([[-38, 100.1], [-38, 88.5]], 3.4, 0.024, gravelRoadMat, 6.0, 0); // Red Barn entrance
    addRoadPolyline([[-65, 100.1], [-65, 90.0]], 3.0, 0.024, gravelRoadMat, 6.0, 0); // Country Farmhouse entrance
    addRoadPolyline([[-40.5, 20.1], [-40.5, -12.5]], 3.4, 0.024, gravelRoadMat, 7.0, 0); // West Wheat Field Shed
    addRoadPolyline([[145.4, 112], [135, 112], [135, 120.9]], 3.4, 0.024, gravelRoadMat, 6.0, 2.5); // Grid Control House

    // Forest Ranger Watchtower Dirt Track: starts FLUSH on STRAIGHT West edge of x = -76 (-77.9, 86) -> Ranger Camp!
    addRoadPolyline([[-77.9, 86], [-84, 86], [-110, 108], [-136, 124]], 3.4, 0.024, gravelRoadMat, 7.0, 6.0);

    // Southern Wind Turbines Service Loop: starts FLUSH at South edge of straight z = 102 (-38, 103.9) -> 3 Wind Turbines -> Solar Asphalt (152.2, 120.6)!
    addRoadPolyline([
      [-38, 103.9], [-42, 165], [-95, 214], [0, 224], [95, 214], [138, 170], [152.2, 136], [152.2, 120.6]
    ], 3.8, 0.024, gravelRoadMat, 8.0, 9.0);

    // --- TIER 4: SCENIC FLAGSTONE FOOTPATHS + GRAND STRAIT ARCHED TIMBER FOOTBRIDGE OVER TWIN-BASIN LAKE ---
    // Trail 4A: Control Tower Apron -> cleanly past Boathouse -> South Bridge Peninsula -> Grand Strait Bridge South Ramp (-32.7, -69.6)!
    addRoadPolyline([
      [-16, -42.5], [-16, -45.5], [-32.7, -45.5], [-32.7, -69.6]
    ], 2.1, 0.034, footpathMat, 4.5, 2.4);
    // Crisp flush Marina Pier branch leaving straight West edge of Trail 4A (-33.75, -49.0) -> South Marina Pier ramp (-54.0, -52.6)!
    addRoadPolyline([
      [-33.75, -49.0], [-54.0, -49.0], [-54.0, -52.6]
    ], 2.1, 0.034, footpathMat, 4.0, 1.4);

    // Trail 4B: Starts seamlessly at Grand Bridge North Ramp (-38.3, -100.4) -> Expanded North Shore -> Lighthouse (-108,-110) -> Ruins (-168,-148)!
    addRoadPolyline([
      [-38.3, -100.4], [-40, -110], [-60, -117], [-88, -115], [-101, -111], [-128, -122], [-168, -136.5]
    ], 2.0, 0.034, footpathMat, 4.5, 5.0);

    // GRAND ARCHED TIMBER STRAIT FOOTBRIDGE spanning 28.4m across deep open water between Main Basin & East Lagoon Bay!
    const buildArchedLakeBridge = (x1, z1, x2, z2) => {
      const dx = x2 - x1;
      const dz = z2 - z1;
      const L = Math.hypot(dx, dz);
      const halfL = L / 2;
      const bridgeW = 2.75;
      const archRise = 2.45; // Tall graceful arch allowing lake boats & kayaks to pass underneath!
      const baseH = 0.24;

      const brGrp = new THREE.Group();
      brGrp.position.set((x1 + x2) / 2, 0, (z1 + z2) / 2);
      brGrp.rotation.y = Math.atan2(dx, dz);

      const brDarkWood = new THREE.MeshStandardMaterial({ color: 0x4a2c16, roughness: 0.9, side: THREE.DoubleSide });
      const brPlankWood = new THREE.MeshStandardMaterial({ color: 0x8b5a2b, roughness: 0.84, side: THREE.DoubleSide });
      const brLightWood = new THREE.MeshStandardMaterial({ color: 0xa67342, roughness: 0.85, side: THREE.DoubleSide });
      const brStone = new THREE.MeshStandardMaterial({ color: 0x7a7f86, roughness: 0.94, side: THREE.DoubleSide });
      const brLamp = new THREE.MeshBasicMaterial({ color: 0xfef08a });
      const brIron = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.45, metalness: 0.65 });

      const archY = (z) => {
        const u = z / halfL;
        return baseH + (1 - u * u) * archRise;
      };
      const archPitch = (z) => {
        const u = z / halfL;
        return Math.atan((-2 * u * archRise) / halfL);
      };

      // 1. Stone Masonry Bank Abutments & Sloped Timber Entry Ramps at Both Headlands
      [-1, 1].forEach((end) => {
        const abut = new THREE.Mesh(new THREE.BoxGeometry(bridgeW + 0.55, 0.55, 1.35), brStone);
        abut.position.set(0, 0.16, end * (halfL + 0.35));
        abut.receiveShadow = true;
        abut.castShadow = true;
        const ramp = new THREE.Mesh(new THREE.BoxGeometry(bridgeW - 0.15, 0.12, 1.4), brPlankWood);
        ramp.position.set(0, 0.16, end * (halfL + 1.05));
        ramp.rotation.x = end * 0.18;
        brGrp.add(abut, ramp);
      });

      // 2. Weathered Wooden Deck Planks + Twin Curved Arch Stringer Beams Underneath
      const nPlanks = Math.max(24, Math.round(L * 1.15));
      const stepZ = L / nPlanks;
      for (let i = 0; i < nPlanks; i++) {
        const pz = -halfL + (i + 0.5) * stepZ;
        const py = archY(pz);
        const pAng = archPitch(pz);
        const plank = new THREE.Mesh(
          new THREE.BoxGeometry(bridgeW, 0.14, stepZ - 0.07),
          (i % 2 === 0) ? brPlankWood : brLightWood
        );
        plank.position.set(0, py, pz);
        plank.rotation.x = -pAng;
        plank.castShadow = true;
        plank.receiveShadow = true;
        brGrp.add(plank);

        // Twin curved dark-timber structural arch stringer segments underneath deck
        [-bridgeW / 2 + 0.14, bridgeW / 2 - 0.14].forEach((sx) => {
          const beam = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.34, stepZ + 0.06), brDarkWood);
          beam.position.set(sx, py - 0.19, pz);
          beam.rotation.x = -pAng;
          beam.castShadow = true;
          brGrp.add(beam);
        });
      }

      // 3. Eight Heavy Timber Support Pilings with Wide Open Central Boat-Passage Arch Under Crest!
      [-0.37 * L, -0.21 * L, 0.21 * L, 0.37 * L].forEach((pz) => {
        const topY = archY(pz) - 0.16;
        const pileH = topY + 0.38;
        [-bridgeW / 2 + 0.18, bridgeW / 2 - 0.18].forEach((px) => {
          const pile = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.2, pileH, 8), brDarkWood);
          pile.position.set(px, topY - pileH / 2, pz);
          pile.castShadow = true;
          brGrp.add(pile);
        });
        const cross = new THREE.Mesh(new THREE.BoxGeometry(bridgeW - 0.2, 0.2, 0.2), brDarkWood);
        cross.position.set(0, topY - 0.12, pz);
        brGrp.add(cross);
      });

      // 4. Graceful Arched Wooden Handrails (Top Rail + Mid Rail + Vertical Balusters) & 10 Glowing Bridge Lanterns!
      const nRailSegs = 24;
      const rStep = L / nRailSegs;
      [-bridgeW / 2 + 0.1, bridgeW / 2 - 0.1].forEach((rx) => {
        for (let i = 0; i < nRailSegs; i++) {
          const rz = -halfL + (i + 0.5) * rStep;
          const ry = archY(rz);
          const rAng = archPitch(rz);
          const segLen = rStep / Math.cos(rAng) + 0.05;
          const topR = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.1, segLen), brDarkWood);
          topR.position.set(rx, ry + 0.98, rz);
          topR.rotation.x = -rAng;
          const midR = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.07, segLen), brLightWood);
          midR.position.set(rx, ry + 0.52, rz);
          midR.rotation.x = -rAng;
          brGrp.add(topR, midR);
        }
        // Vertical handrail balusters + tall glowing lantern posts at ends, quarter-spans & arch crest
        for (let i = 0; i <= nRailSegs; i++) {
          const pz = -halfL + i * rStep;
          const py = archY(pz);
          const isLantern = (i === 0 || i === nRailSegs || i === nRailSegs / 2 || i === nRailSegs / 4 || i === (nRailSegs * 3) / 4);
          const pH = isLantern ? 1.55 : 0.98;
          const pSize = isLantern ? 0.15 : 0.075;
          const post = new THREE.Mesh(new THREE.BoxGeometry(pSize, pH, pSize), brDarkWood);
          post.position.set(rx, py + pH / 2, pz);
          post.castShadow = true;
          brGrp.add(post);
          if (isLantern) {
            const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.28, 0.24), brLamp);
            lamp.position.set(rx, py + pH + 0.12, pz);
            const cap = new THREE.Mesh(new THREE.ConeGeometry(0.24, 0.18, 4), brIron);
            cap.rotation.y = Math.PI / 4;
            cap.position.set(rx, py + pH + 0.34, pz);
            brGrp.add(lamp, cap);
          }
        }
      });

      this.scene.add(brGrp);

      // 5 Stepped solid colliders along the 28.4m arched bridge so drones can land smoothly anywhere along the deck!
      [-0.36, -0.18, 0, 0.18, 0.36].forEach((frac) => {
        const cx = (x1 + x2) / 2 + dx * frac;
        const cz = (z1 + z2) / 2 + dz * frac;
        const cy = archY(frac * L) * 0.5;
        this.addBoxCollider(cx, cy, cz, 3.2, archY(frac * L) + 0.15, 5.8);
      });
    };
    buildArchedLakeBridge(-33.0, -71.0, -38.0, -99.0);

    // Individual Front-Door & Field Gate Walkways (ALL trimmed STRICTLY FLUSH edge-to-edge, 0.00m overlap!)
    const shortPaths = [
      [[65.5, 20.4], [68.5, 20.4]],     // House 1 steps -> Main St West edge (68.5)
      [[75.5, 23.0], [73.5, 23.0]],     // House 2 steps -> Main St East edge (73.5)
      [[63.8, 37.5], [68.5, 37.5]],     // House 3 door -> Main St West edge (68.5)
      [[76.0, 41.1], [73.5, 41.1]],     // House 4 log cabin steps -> Main St East edge (73.5)
      [[82, 7.7], [82, 9.2]],           // Transit Shelter -> North Main Asphalt North edge (9.2)
      [[98, 26.25], [98, 28.6]],        // Village Café patio -> Market Ave North edge (28.6)
      [[118, 23.6], [118, 28.6]],       // Valley Market veranda -> Market Ave North edge (28.6)
      [[108, 34.7], [108, 33.4]],       // Village Square Plaza -> Market Ave South edge (33.4)
      [[98, 57.4], [98, 59.8]],         // Ochre Cottage -> South Lane North edge (59.8)
      [[118, 57.7], [118, 59.8]],       // Sage Chalet -> South Lane North edge (59.8)
      [[30, 84.6], [30, 80.8]],         // Glass Greenhouse porch steps -> South Valley Asphalt South edge (80.8)
      [[-10, 88.6], [-10, 87.4]],       // Grain Windmill door -> Gravel road South edge (87.4)
      [[35, 70.0], [35, 75.2]],         // SE Cabbage Field gate -> South Valley Asphalt North edge (75.2)
      [[-16, 78.0], [-16, 83.6]],       // South Wheat Field gate -> Gravel Homestead road North edge (83.6)
      [[-27.0, -56.0], [-31.65, -56.0]] // Lakeside Boathouse porch -> Lakeside Trail 4A East edge (-31.65)
    ];
    shortPaths.forEach((sp) => addRoadPolyline(sp, 1.65, 0.034, footpathMat, 3.2, 0));

    // --- WOODEN UTILITY POWER POLES & OVERHEAD CABLES (Solar Farm -> Grid Control House -> East Village -> Hangar!) ---
    const poleWoodMat = new THREE.MeshStandardMaterial({ color: 0x5c3a21, roughness: 0.92 });
    const insWhiteMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.35 });
    const wireMat = new THREE.LineBasicMaterial({ color: 0x1e293b });

    const powerPoles = [
      [154, 114], [142, 115], [140, 88], [138.5, 58], [138.5, 28],
      [135, 8.2], [108, 8.2], [74, 8.2], [58, 8.2], [58, -4]
    ];
    for (let i = 0; i < powerPoles.length; i++) {
      const [px, pz] = powerPoles[i];
      const py = this.getTerrainHeight(px, pz);
      const next = powerPoles[Math.min(i + 1, powerPoles.length - 1)];
      const prev = powerPoles[Math.max(i - 1, 0)];
      const dirAng = Math.atan2(next[0] - prev[0], next[1] - prev[1]);

      const pg = new THREE.Group();
      pg.position.set(px, py, pz);
      pg.rotation.y = dirAng;
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.16, 7.6, 8), poleWoodMat);
      pole.position.y = 3.8;
      pole.castShadow = true;
      const cross = new THREE.Mesh(new THREE.BoxGeometry(1.9, 0.14, 0.14), poleWoodMat);
      cross.position.y = 7.15;
      pg.add(pole, cross);
      [-0.78, 0.78].forEach((ix) => {
        const ins = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.08, 0.24, 6), insWhiteMat);
        ins.position.set(ix, 7.32, 0);
        pg.add(ins);
      });
      this.scene.add(pg);
      this.addCylCollider(px, pz, 0.26, py, py + 7.6, 'power_pole');

      // Pair of gently sagging overhead power cables connecting to next pole
      if (i < powerPoles.length - 1) {
        const [nx, nz] = powerPoles[i + 1];
        const ny = this.getTerrainHeight(nx, nz);
        if (this.powerWireSegments) {
          this.powerWireSegments.push({ x1: px, y1: py + 7.25, z1: pz, x2: nx, y2: ny + 7.25, z2: nz, sag: 0.65 });
        }
        const segLen = Math.hypot(nx - px, nz - pz);
        const perpX = -(nz - pz) / segLen;
        const perpZ = (nx - px) / segLen;
        [-0.76, 0.76].forEach((side) => {
          const pts = [];
          for (let s = 0; s <= 6; s++) {
            const t = s / 6;
            const sag = Math.sin(t * Math.PI) * 0.65;
            pts.push(new THREE.Vector3(
              px + (nx - px) * t + perpX * side,
              py + (ny - py) * t + 7.38 - sag,
              pz + (nz - pz) * t + perpZ * side
            ));
          }
          this.scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), wireMat));
        });
      }
    }

    // --- WOODEN DIRECTIONAL SIGNPOSTS AT KEY CROSSROADS ---
    const signWoodMat = new THREE.MeshStandardMaterial({ color: 0x6b3a17, roughness: 0.9 });
    const signBoardMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.75 });
    [
      [57.2, 15.2, 0.4], [74.4, 27.6, -0.3], [-5.2, 70.8, 0.6],
      [-30.8, -47.8, -0.5], [-99.5, -108.5, 0.2], [-71.5, 92.8, 0.8]
    ].forEach(([sx, sz, rot]) => {
      const sg = new THREE.Group();
      sg.position.set(sx, this.getTerrainHeight(sx, sz), sz);
      sg.rotation.y = rot;
      const post = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.08, 2.35, 6), signWoodMat);
      post.position.y = 1.18;
      const arm1 = new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.22, 0.06), signBoardMat);
      arm1.position.set(0.32, 2.08, 0);
      const arm2 = new THREE.Mesh(new THREE.BoxGeometry(0.88, 0.22, 0.06), signBoardMat);
      arm2.position.set(-0.26, 1.78, 0.05);
      arm2.rotation.y = Math.PI / 2;
      sg.add(post, arm1, arm2);
      this.scene.add(sg);
    });
  }

  buildLivingValleySettlements() {
    // =========================================================================
    // PURPOSEFUL, LIVED-IN VALLEY ARCHITECTURE & STORYTELLING PROPS (NO CARS, NO NPCs!)
    // Every building has a clear role in the valley's ecosystem and connects to the road/trail network!
    // =========================================================================
    const darkTrimMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.78, side: THREE.DoubleSide });
    const whiteTrimMat = new THREE.MeshStandardMaterial({ color: 0xf1f5f9, roughness: 0.8, side: THREE.DoubleSide });
    const darkWoodMat = new THREE.MeshStandardMaterial({ color: 0x5c3415, roughness: 0.88, side: THREE.DoubleSide });
    const warmWoodMat = new THREE.MeshStandardMaterial({ color: 0x9a6536, roughness: 0.85, side: THREE.DoubleSide });
    const stoneMat = new THREE.MeshStandardMaterial({ color: 0x7a7f86, roughness: 0.94, side: THREE.DoubleSide });
    const brickMat = new THREE.MeshStandardMaterial({ color: 0x9b3b2c, roughness: 0.88, side: THREE.DoubleSide });
    const bushMat = new THREE.MeshStandardMaterial({ color: 0x1e5c2a, roughness: 0.88, flatShading: true });
    const glassWarmMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.14, metalness: 0.65, emissive: 0x854d0e, emissiveIntensity: 0.5, side: THREE.DoubleSide });
    const clearGlassMat = new THREE.MeshStandardMaterial({ color: 0xbae6fd, roughness: 0.08, metalness: 0.3, transparent: true, opacity: 0.42, side: THREE.DoubleSide });
    const lampGlowMat = new THREE.MeshBasicMaterial({ color: 0xfef08a });
    const redMat = new THREE.MeshStandardMaterial({ color: 0xdc2626, roughness: 0.65, side: THREE.DoubleSide });
    const blueMat = new THREE.MeshStandardMaterial({ color: 0x1d4ed8, roughness: 0.62, side: THREE.DoubleSide });
    const yellowMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.65, side: THREE.DoubleSide });
    const brassMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.25, metalness: 0.85 });

    // Procedural helpers for signage, awnings & shingled roofs
    const makeRoofMat = (hexBase, hexLine) => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = hexBase; ctx.fillRect(0, 0, 128, 128);
      ctx.fillStyle = hexLine;
      for (let y = 0; y < 128; y += 16) {
        ctx.fillRect(0, y, 128, 3);
        const off = ((y / 16) % 2) * 16;
        for (let x = off; x < 128; x += 32) ctx.fillRect(x, y, 2, 16);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 3);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return new THREE.MeshStandardMaterial({ map: tex, roughness: 0.78, side: THREE.DoubleSide });
    };

    const makeStripeAwningMat = (c1 = '#dc2626', c2 = '#f8fafc') => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 64;
      const ctx = c.getContext('2d');
      for (let x = 0; x < 128; x += 16) {
        ctx.fillStyle = ((x / 16) % 2 === 0) ? c1 : c2;
        ctx.fillRect(x, 0, 16, 64);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = THREE.RepeatWrapping;
      tex.repeat.set(2, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return new THREE.MeshStandardMaterial({ map: tex, roughness: 0.82, side: THREE.DoubleSide });
    };

    const makeSignTex = (title, sub, bg = '#1e293b', accent = '#f97316') => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 96;
      const ctx = c.getContext('2d');
      ctx.fillStyle = bg; ctx.fillRect(0, 0, 256, 96);
      ctx.strokeStyle = accent; ctx.lineWidth = 6; ctx.strokeRect(4, 4, 248, 88);
      ctx.fillStyle = '#f8fafc'; ctx.font = 'bold 34px sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(title, 128, 38);
      ctx.fillStyle = '#cbd5e1'; ctx.font = 'bold 16px sans-serif';
      ctx.fillText(sub, 128, 70);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Watertight extruded A-frame gable roof helper (ZERO gaps, ZERO see-through!)
    const addFlushGable = (parent, width, wallLen, baseY, peakH, roofHex, lineHex, wallMat, centerZ = 0, fOver = 0.5, bOver = 0.5, eaveX = 0.5) => {
      const rMat = makeRoofMat(roofHex, lineHex);
      const halfW = width / 2;
      const pitch = Math.atan2(peakH, halfW);
      const run = halfW + eaveX;
      const drop = run * Math.tan(pitch);
      const slopeLen = Math.hypot(run, drop) + 0.14;
      const slabT = 0.22;
      const nOff = slabT / 2 - 0.04;
      const roofLen = wallLen + fOver + bOver;
      const roofCZ = centerZ + (fOver - bOver) / 2;
      const midY = (baseY + peakH) - drop / 2;

      const sL = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, slabT, roofLen), rMat);
      sL.position.set(-run / 2 - Math.sin(pitch) * nOff, midY + Math.cos(pitch) * nOff, roofCZ);
      sL.rotation.z = pitch; sL.castShadow = true;
      const sR = new THREE.Mesh(new THREE.BoxGeometry(slopeLen, slabT, roofLen), rMat);
      sR.position.set(run / 2 + Math.sin(pitch) * nOff, midY + Math.cos(pitch) * nOff, roofCZ);
      sR.rotation.z = -pitch; sR.castShadow = true;
      const ridge = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.24, roofLen + 0.06), darkTrimMat);
      ridge.position.set(0, baseY + peakH + slabT / 2 + 0.02, roofCZ);

      const gShape = new THREE.Shape([
        new THREE.Vector2(-halfW, 0),
        new THREE.Vector2(halfW, 0),
        new THREE.Vector2(0, peakH + 0.02)
      ]);
      const gable = new THREE.Mesh(new THREE.ExtrudeGeometry(gShape, { depth: wallLen, bevelEnabled: false }), wallMat);
      gable.position.set(0, baseY, centerZ - wallLen / 2);
      gable.castShadow = true; gable.receiveShadow = true;
      parent.add(sL, sR, ridge, gable);
    };

    const addFramedWin = (parent, x, y, z, w, h, ry = 0, shutterMat = null) => {
      const wg = new THREE.Group();
      wg.position.set(x, y, z); wg.rotation.y = ry;
      const fr = new THREE.Mesh(new THREE.BoxGeometry(w + 0.16, h + 0.16, 0.14), whiteTrimMat);
      const pn = new THREE.Mesh(new THREE.BoxGeometry(w, h, 0.18), glassWarmMat);
      const bh = new THREE.Mesh(new THREE.BoxGeometry(w + 0.04, 0.06, 0.2), whiteTrimMat);
      const bv = new THREE.Mesh(new THREE.BoxGeometry(0.06, h + 0.04, 0.2), whiteTrimMat);
      wg.add(fr, pn, bh, bv);
      if (shutterMat) {
        const sL = new THREE.Mesh(new THREE.BoxGeometry(0.24, h + 0.12, 0.08), shutterMat);
        sL.position.set(-(w / 2 + 0.2), 0, 0.04);
        const sR = sL.clone(); sR.position.x = w / 2 + 0.2;
        wg.add(sL, sR);
      }
      parent.add(wg);
    };

    const addEntryDoor = (parent, x, y, z, w, h, dMat, fMat = whiteTrimMat, ry = 0) => {
      const dg = new THREE.Group();
      dg.position.set(x, y, z); dg.rotation.y = ry;
      const fr = new THREE.Mesh(new THREE.BoxGeometry(w + 0.18, h + 0.12, 0.14), fMat);
      const lf = new THREE.Mesh(new THREE.BoxGeometry(w, h, 0.18), dMat);
      const gl = new THREE.Mesh(new THREE.BoxGeometry(w * 0.54, h * 0.28, 0.21), glassWarmMat);
      gl.position.y = h * 0.22;
      const kn = new THREE.Mesh(new THREE.SphereGeometry(0.055, 6, 6), brassMat);
      kn.position.set(w * 0.34, -0.04, 0.13);
      dg.add(fr, lf, gl, kn);
      parent.add(dg);
    };

    const addGroundedFence = (parent, x1, z1, x2, z2, y0 = 0, h = 0.85, mat = whiteTrimMat) => {
      const len = Math.hypot(x2 - x1, z2 - z1);
      if (len < 0.1) return;
      const fg = new THREE.Group();
      fg.position.set((x1 + x2) / 2, y0, (z1 + z2) / 2);
      fg.rotation.y = Math.atan2(x2 - x1, z2 - z1);
      [0.24, h - 0.1].forEach((ry) => {
        const rail = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.08, len), mat);
        rail.position.y = ry; fg.add(rail);
      });
      const count = Math.max(2, Math.round(len / 0.48) + 1);
      for (let i = 0; i < count; i++) {
        const p = new THREE.Mesh(new THREE.BoxGeometry(0.08, h + 0.05, 0.08), mat);
        p.position.set(0, (h + 0.05) / 2, -len / 2 + (i / (count - 1)) * len);
        fg.add(p);
      }
      parent.add(fg);
    };

    const addClothesline = (parent, x, z, rotY = 0) => {
      const cl = new THREE.Group();
      cl.position.set(x, 0, z); cl.rotation.y = rotY;
      [-1.5, 1.5].forEach((px) => {
        const post = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.055, 2.0, 6), darkWoodMat);
        post.position.set(px, 1.0, 0);
        const bar = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.65), darkWoodMat);
        bar.position.set(px, 1.92, 0);
        cl.add(post, bar);
      });
      const line = new THREE.Mesh(new THREE.BoxGeometry(3.0, 0.02, 0.02), whiteTrimMat);
      line.position.y = 1.88;
      cl.add(line);
      // Hanging colorful linen sheets & shirts (lived-in vibe without NPCs!)
      [[-0.95, 0xef4444, 0.55, 0.62], [-0.25, 0xf8fafc, 0.62, 0.72], [0.45, 0x38bdf8, 0.52, 0.58], [1.05, 0xfacc15, 0.46, 0.52]].forEach(([cx, col, cw, ch]) => {
        const cloth = new THREE.Mesh(new THREE.BoxGeometry(cw, ch, 0.04), new THREE.MeshStandardMaterial({ color: col, roughness: 0.9, side: THREE.DoubleSide }));
        cloth.position.set(cx, 1.88 - ch / 2, 0);
        cloth.rotation.x = 0.08;
        cl.add(cloth);
      });
      parent.add(cl);
    };

    const addPatioTableUmbrella = (parent, x, z, umbMat = redMat) => {
      const tg = new THREE.Group();
      tg.position.set(x, 0, z);
      const top = new THREE.Mesh(new THREE.CylinderGeometry(0.68, 0.68, 0.08, 12), warmWoodMat);
      top.position.y = 0.78;
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.14, 0.78, 8), darkTrimMat);
      leg.position.y = 0.39;
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 2.35, 6), darkWoodMat);
      pole.position.y = 1.18;
      const umb = new THREE.Mesh(new THREE.ConeGeometry(1.18, 0.45, 8), umbMat);
      umb.position.y = 2.38;
      umb.castShadow = true;
      tg.add(top, leg, pole, umb);
      [0, Math.PI].forEach((a) => {
        const chair = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.46, 0.42), warmWoodMat);
        chair.position.set(Math.cos(a) * 0.92, 0.23, Math.sin(a) * 0.92);
        tg.add(chair);
      });
      parent.add(tg);
    };

    // =========================================================================
    // 1. LIVED-IN YARD UPGRADES FOR ORIGINAL VILLAGE HOUSES 1–4
    // =========================================================================
    // House 1 courtyard patio table + blue umbrella (South lawn at 61.5, 27.6)
    const yardProps = new THREE.Group();
    addPatioTableUmbrella(yardProps, 61.5, 27.6, blueMat);
    // House 2 white picket front-yard garden fence & vegetable planter bed (East side at 79, 23)
    addGroundedFence(yardProps, 75.5, 19.2, 83.2, 19.2, 0, 0.75, whiteTrimMat);
    addGroundedFence(yardProps, 83.2, 19.2, 83.2, 26.8, 0, 0.75, whiteTrimMat);
    addGroundedFence(yardProps, 75.5, 26.8, 83.2, 26.8, 0, 0.75, whiteTrimMat);
    const h2Bed = new THREE.Mesh(new THREE.BoxGeometry(2.6, 0.35, 1.2), darkWoodMat);
    h2Bed.position.set(81.0, 0.18, 17.6);
    const h2BedLeaf = new THREE.Mesh(new THREE.BoxGeometry(2.4, 0.25, 1.0), bushMat);
    h2BedLeaf.position.set(81.0, 0.42, 17.6);
    yardProps.add(h2Bed, h2BedLeaf);
    // House 3 backyard clothesline & garden bench (57.2, 39.0)
    addClothesline(yardProps, 57.2, 38.8, Math.PI / 2);
    // House 4 side firewood chopping block & lean-to woodshed (80.5, 45.2)
    const h4ShedRoof = new THREE.Mesh(new THREE.BoxGeometry(2.4, 0.14, 1.5), darkTrimMat);
    h4ShedRoof.position.set(80.6, 1.95, 45.2);
    h4ShedRoof.rotation.x = 0.16;
    [[-1.0, -0.55], [1.0, -0.55], [-1.0, 0.55], [1.0, 0.55]].forEach(([px, pz]) => {
      const sp = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.9, 0.12), darkWoodMat);
      sp.position.set(80.6 + px, 0.95, 45.2 + pz);
      yardProps.add(sp);
    });
    const chopBlock = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.32, 0.62, 8), darkWoodMat);
    chopBlock.position.set(78.6, 0.31, 45.2);
    yardProps.add(h4ShedRoof, chopBlock);
    this.scene.add(yardProps);

    // =========================================================================
    // 2. BUILDING 6 (98, 0, 20): VALLEY CAFÉ & BAKERY (Striped Awning, Patio Umbrellas & Menu Board)
    // =========================================================================
    const cafe = new THREE.Group();
    cafe.position.set(98, 0, 20);
    const cafeWallMat = new THREE.MeshStandardMaterial({ color: 0xf5ead8, roughness: 0.85, side: THREE.DoubleSide });
    const cafePlinth = new THREE.Mesh(new THREE.BoxGeometry(8.2, 0.4, 6.4), stoneMat);
    cafePlinth.position.y = 0.2;
    const cafeBase = new THREE.Mesh(new THREE.BoxGeometry(7.8, 1.2, 5.8), brickMat);
    cafeBase.position.y = 1.0;
    const cafeBody = new THREE.Mesh(new THREE.BoxGeometry(7.7, 2.6, 5.7), cafeWallMat);
    cafeBody.position.y = 2.9;
    cafeBody.castShadow = true; cafeBody.receiveShadow = true;
    addFlushGable(cafe, 7.7, 5.7, 4.2, 2.2, '#9a3412', '#71240a', cafeWallMat, 0, 0.55, 0.55);
    // Red-and-white striped café storefront awning + warm display window + framed door + signage!
    const awning = new THREE.Mesh(new THREE.BoxGeometry(6.8, 0.14, 1.85), makeStripeAwningMat('#dc2626', '#f8fafc'));
    awning.position.set(0, 2.85, 3.65);
    awning.rotation.x = 0.32;
    awning.castShadow = true;
    const cafeSign = new THREE.Mesh(
      new THREE.BoxGeometry(3.8, 0.95, 0.14),
      new THREE.MeshStandardMaterial({ map: makeSignTex('VALLEY CAFÉ', 'FRESH BAKERY & COFFEE', '#451a03', '#f59e0b'), roughness: 0.5 })
    );
    cafeSign.position.set(0, 3.72, 2.94);
    addEntryDoor(cafe, -2.0, 1.4, 2.92, 1.1, 2.0, darkWoodMat, whiteTrimMat, 0);
    addFramedWin(cafe, 1.45, 1.75, 2.92, 2.6, 1.35, 0); // Large bakery display window
    addFramedWin(cafe, 0, 5.05, 2.9, 1.0, 0.85, 0);
    addFramedWin(cafe, 3.9, 2.0, 0, 1.4, 1.1, Math.PI / 2, brickMat);
    addFramedWin(cafe, -3.9, 2.0, 0, 1.4, 1.1, -Math.PI / 2, brickMat);
    const cafeChim = new THREE.Mesh(new THREE.BoxGeometry(0.85, 2.6, 0.85), brickMat);
    cafeChim.position.set(-2.2, 5.8, -1.2);
    // Outdoor timber café patio deck + 2 outdoor café tables with umbrellas + A-frame menu board
    const patioDeck = new THREE.Mesh(new THREE.BoxGeometry(8.2, 0.16, 3.2), warmWoodMat);
    patioDeck.position.set(0, 0.08, 4.65);
    patioDeck.receiveShadow = true;
    addPatioTableUmbrella(cafe, -1.9, 5.1, redMat);
    addPatioTableUmbrella(cafe, 2.1, 5.1, yellowMat);
    cafe.add(cafePlinth, cafeBase, cafeBody, awning, cafeSign, cafeChim, patioDeck);
    this.scene.add(cafe);
    this.addBoxCollider(98, 3.2, 20, 8.2, 6.4, 6.4);

    // =========================================================================
    // 3. BUILDING 7 (118, 0, 20): VALLEY GENERAL STORE & PRODUCE MARKET (Veranda, Farm Crates & Postbox)
    // =========================================================================
    const market = new THREE.Group();
    market.position.set(118, 0, 20);
    const mktWallMat = new THREE.MeshStandardMaterial({ color: 0xe6decb, roughness: 0.86, side: THREE.DoubleSide });
    const mktPlinth = new THREE.Mesh(new THREE.BoxGeometry(8.6, 0.4, 7.2), darkTrimMat);
    mktPlinth.position.y = 0.2;
    const mktBody = new THREE.Mesh(new THREE.BoxGeometry(8.0, 3.6, 5.4), mktWallMat);
    mktBody.position.set(0, 2.2, -0.7);
    mktBody.castShadow = true; mktBody.receiveShadow = true;
    addFlushGable(market, 8.0, 5.4, 4.0, 2.2, '#1e3a5f', '#142640', mktWallMat, -0.7, 2.3, 0.55);
    // Covered timber veranda columns supporting the front overhang + MARKET sign
    [-3.6, -1.2, 1.2, 3.6].forEach((px) => {
      const col = new THREE.Mesh(new THREE.BoxGeometry(0.18, 3.45, 0.18), darkWoodMat);
      col.position.set(px, 2.12, 3.25);
      col.castShadow = true;
      market.add(col);
    });
    const mktSign = new THREE.Mesh(
      new THREE.BoxGeometry(4.2, 0.92, 0.16),
      new THREE.MeshStandardMaterial({ map: makeSignTex('VALLEY MARKET', 'FARM PRODUCE & POST', '#14532d', '#facc15'), roughness: 0.5 })
    );
    mktSign.position.set(0, 3.82, 3.38);
    addEntryDoor(market, 0, 1.4, 2.02, 1.25, 2.05, darkWoodMat, whiteTrimMat, 0);
    addFramedWin(market, -2.4, 1.85, 2.02, 1.5, 1.2, 0);
    addFramedWin(market, 2.4, 1.85, 2.02, 1.5, 1.2, 0);
    // Wooden produce display crates overflowing with valley harvest (apples, pumpkins, greens!)
    [[-2.5, redMat], [2.4, yellowMat], [3.35, bushMat]].forEach(([cx, pMat]) => {
      const crate = new THREE.Mesh(new THREE.BoxGeometry(0.92, 0.65, 0.72), warmWoodMat);
      crate.position.set(cx, 0.72, 2.65);
      market.add(crate);
      for (let k = -1; k <= 1; k++) {
        const fruit = new THREE.Mesh(new THREE.SphereGeometry(0.14, 6, 6), pMat);
        fruit.position.set(cx + k * 0.24, 1.1, 2.65);
        market.add(fruit);
      }
    });
    // Classic Red Community Postbox standing outside the store entrance
    const postBox = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.24, 1.15, 10), redMat);
    postBox.position.set(-4.35, 0.58, 3.4);
    const postCap = new THREE.Mesh(new THREE.SphereGeometry(0.25, 10, 8, 0, Math.PI * 2, 0, Math.PI / 2), redMat);
    postCap.position.set(-4.35, 1.15, 3.4);
    market.add(mktPlinth, mktBody, mktSign, postBox, postCap);
    this.scene.add(market);
    this.addBoxCollider(118, 3.2, 20, 8.6, 6.4, 7.4);

    // =========================================================================
    // 4. BUILDING 8 (108, 0, 39.5): VILLAGE SQUARE PLAZA (Covered Wishing Well, Benches & Notice Board)
    // =========================================================================
    const plaza = new THREE.Group();
    plaza.position.set(108, 0, 39.5);
    const plazaPad = new THREE.Mesh(new THREE.CylinderGeometry(4.6, 4.8, 0.12, 20), stoneMat);
    plazaPad.position.y = 0.06; plazaPad.receiveShadow = true;
    // Covered Stone Wishing Well centerpiece
    const wellBase = new THREE.Mesh(new THREE.CylinderGeometry(1.05, 1.18, 0.95, 14), stoneMat);
    wellBase.position.y = 0.54; wellBase.castShadow = true;
    const wellWater = new THREE.Mesh(new THREE.CircleGeometry(0.84, 12), new THREE.MeshStandardMaterial({ color: 0x0c74a8, roughness: 0.1 }));
    wellWater.rotation.x = -Math.PI / 2; wellWater.position.y = 0.92;
    [-0.95, 0.95].forEach((wx) => {
      const wp = new THREE.Mesh(new THREE.BoxGeometry(0.14, 1.65, 0.14), darkWoodMat);
      wp.position.set(wx, 1.65, 0);
      plaza.add(wp);
    });
    const wellRoof = new THREE.Mesh(new THREE.ConeGeometry(1.65, 1.15, 4), makeRoofMat('#6b3a17', '#4a250e'));
    wellRoof.position.y = 2.95; wellRoof.rotation.y = Math.PI / 4; wellRoof.castShadow = true;
    // 3 Wooden Park Benches facing the Wishing Well + Community Notice Board with flyers
    [0, 2.1, -2.1].forEach((ang) => {
      const bg = new THREE.Group();
      bg.rotation.y = ang;
      const bSeat = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.1, 0.55), warmWoodMat);
      bSeat.position.set(0, 0.52, 3.2);
      const bBack = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.48, 0.09), warmWoodMat);
      bBack.position.set(0, 0.82, 3.45);
      [-0.75, 0.75].forEach((lx) => {
        const leg = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.48, 0.5), darkTrimMat);
        leg.position.set(lx, 0.28, 3.2);
        bg.add(leg);
      });
      bg.add(bSeat, bBack);
      plaza.add(bg);
    });
    // Wooden Community Notice Board with colorful paper flyers
    const nb = new THREE.Group();
    nb.position.set(-3.6, 0, -2.4); nb.rotation.y = 0.45;
    [-0.75, 0.75].forEach((nx) => {
      const np = new THREE.Mesh(new THREE.BoxGeometry(0.1, 2.2, 0.1), darkWoodMat);
      np.position.set(nx, 1.1, 0); nb.add(np);
    });
    const nBoard = new THREE.Mesh(new THREE.BoxGeometry(1.65, 1.05, 0.1), warmWoodMat);
    nBoard.position.set(0, 1.55, 0);
    const nRoof = new THREE.Mesh(new THREE.BoxGeometry(1.9, 0.14, 0.55), darkWoodMat);
    nRoof.position.set(0, 2.22, 0);
    [[-0.4, 1.65, whiteTrimMat], [0.05, 1.52, yellowMat], [0.45, 1.62, whiteTrimMat]].forEach(([fx, fy, fm]) => {
      const flyer = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.34, 0.12), fm);
      flyer.position.set(fx, fy, 0.01); nb.add(flyer);
    });
    nb.add(nBoard, nRoof);
    plaza.add(plazaPad, wellBase, wellWater, wellRoof, nb);
    this.scene.add(plaza);
    this.addCylCollider(108, 39.5, 1.35, 0, 3.6);

    // =========================================================================
    // 5. BUILDING 9A (98, 0, 54.5) & 9B (118, 0, 54.5): SOLAR ENGINEER'S COTTAGE & BOTANIST'S CHALET
    // =========================================================================
    // 9A: Ochre Stucco Cottage with Picket Yard Fence & Backyard Clothesline
    const cotA = new THREE.Group();
    cotA.position.set(98, 0, 54.5);
    const ochreMat = new THREE.MeshStandardMaterial({ color: 0xe2b866, roughness: 0.86, side: THREE.DoubleSide });
    const cotAPlinth = new THREE.Mesh(new THREE.BoxGeometry(6.8, 0.38, 5.8), stoneMat);
    cotAPlinth.position.y = 0.19;
    const cotABody = new THREE.Mesh(new THREE.BoxGeometry(6.2, 3.1, 5.2), ochreMat);
    cotABody.position.y = 1.93; cotABody.castShadow = true; cotABody.receiveShadow = true;
    addFlushGable(cotA, 6.2, 5.2, 3.48, 2.0, '#334155', '#1e293b', ochreMat, 0, 0.5, 0.5);
    addEntryDoor(cotA, -1.2, 1.38, 2.62, 1.05, 2.0, darkWoodMat, whiteTrimMat, 0);
    const cotAAwning = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.14, 0.88), darkTrimMat);
    cotAAwning.position.set(-1.2, 2.55, 2.98); cotAAwning.rotation.x = 0.22;
    addFramedWin(cotA, 1.45, 1.95, 2.62, 1.35, 1.1, 0, darkWoodMat);
    addFramedWin(cotA, 0, 4.3, 2.62, 0.9, 0.8, 0);
    addFramedWin(cotA, 3.12, 1.95, 0, 1.2, 1.05, Math.PI / 2, darkWoodMat);
    const cotAChim = new THREE.Mesh(new THREE.BoxGeometry(0.75, 2.4, 0.75), brickMat);
    cotAChim.position.set(1.7, 4.9, -1.0);
    addGroundedFence(cotA, -3.9, 3.5, -2.0, 3.5, 0, 0.75, whiteTrimMat);
    addGroundedFence(cotA, -0.3, 3.5, 3.9, 3.5, 0, 0.75, whiteTrimMat);
    addClothesline(cotA, 0, -4.1, 0);
    cotA.add(cotAPlinth, cotABody, cotAAwning, cotAChim);
    this.scene.add(cotA);
    this.addBoxCollider(98, 2.8, 54.5, 6.8, 5.7, 5.8);

    // 9B: Botanist's Sage-Green Timber Chalet with Covered Porch & Raised Herb Garden Beds
    const cotB = new THREE.Group();
    cotB.position.set(118, 0, 54.5);
    const sageMat = new THREE.MeshStandardMaterial({ color: 0x6b8c71, roughness: 0.86, side: THREE.DoubleSide });
    const cotBDeck = new THREE.Mesh(new THREE.BoxGeometry(6.6, 0.42, 6.4), darkWoodMat);
    cotBDeck.position.y = 0.21;
    const cotBBody = new THREE.Mesh(new THREE.BoxGeometry(6.0, 3.1, 5.0), sageMat);
    cotBBody.position.set(0, 1.97, -0.45); cotBBody.castShadow = true; cotBBody.receiveShadow = true;
    addFlushGable(cotB, 6.0, 5.0, 3.52, 2.35, '#1c4a32', '#123222', sageMat, -0.45, 1.4, 0.5);
    [-2.85, 2.85].forEach((px) => {
      const col = new THREE.Mesh(new THREE.BoxGeometry(0.2, 3.1, 0.2), darkWoodMat);
      col.position.set(px, 1.97, 3.05); cotB.add(col);
    });
    addGroundedFence(cotB, -2.85, 3.05, -0.8, 3.05, 0.42, 0.78, warmWoodMat);
    addGroundedFence(cotB, 0.8, 3.05, 2.85, 3.05, 0.42, 0.78, warmWoodMat);
    addEntryDoor(cotB, 0, 1.42, 2.06, 1.05, 2.0, warmWoodMat, whiteTrimMat, 0);
    addFramedWin(cotB, -1.85, 2.0, 2.07, 1.05, 1.05, 0, darkWoodMat);
    addFramedWin(cotB, 1.85, 2.0, 2.07, 1.05, 1.05, 0, darkWoodMat);
    addFramedWin(cotB, 0, 4.45, 2.07, 0.95, 0.85, 0);
    // Backyard raised wooden herb garden beds & rainwater barrel
    [-1.4, 1.4].forEach((hx) => {
      const hb = new THREE.Mesh(new THREE.BoxGeometry(2.1, 0.42, 1.25), darkWoodMat);
      hb.position.set(hx, 0.21, -4.2);
      const hl = new THREE.Mesh(new THREE.BoxGeometry(1.9, 0.3, 1.05), bushMat);
      hl.position.set(hx, 0.5, -4.2);
      cotB.add(hb, hl);
    });
    const rBarrel = new THREE.Mesh(new THREE.CylinderGeometry(0.36, 0.38, 0.95, 10), blueMat);
    rBarrel.position.set(3.45, 0.48, -2.4);
    cotB.add(cotBDeck, cotBBody, rBarrel);
    this.scene.add(cotB);
    this.addBoxCollider(118, 3.0, 54.5, 6.8, 6.1, 6.6);

    // =========================================================================
    // 6. BUILDING 10 (-65, 0, 86): VALLEY COUNTRY FARMHOUSE, CHICKEN COOP & HARVEST HAND-CART
    // =========================================================================
    const farmH = new THREE.Group();
    farmH.position.set(-65, 0, 86);
    const farmWallMat = new THREE.MeshStandardMaterial({ color: 0xf4f1ea, roughness: 0.86, side: THREE.DoubleSide });
    const fhDeck = new THREE.Mesh(new THREE.BoxGeometry(7.8, 0.45, 7.2), darkWoodMat);
    fhDeck.position.y = 0.225;
    const fhBody = new THREE.Mesh(new THREE.BoxGeometry(7.2, 3.6, 5.4), farmWallMat);
    fhBody.position.set(0, 2.25, -0.6); fhBody.castShadow = true; fhBody.receiveShadow = true;
    addFlushGable(farmH, 7.2, 5.4, 4.05, 2.4, '#b84a28', '#8a3218', farmWallMat, -0.6, 1.8, 0.55);
    [-3.4, -1.2, 1.2, 3.4].forEach((px) => {
      const col = new THREE.Mesh(new THREE.BoxGeometry(0.18, 3.55, 0.18), whiteTrimMat);
      col.position.set(px, 2.22, 3.38); farmH.add(col);
    });
    addGroundedFence(farmH, -3.4, 3.38, -0.85, 3.38, 0.45, 0.8, whiteTrimMat);
    addGroundedFence(farmH, 0.85, 3.38, 3.4, 3.38, 0.45, 0.8, whiteTrimMat);
    addEntryDoor(farmH, 0, 1.45, 2.12, 1.1, 2.0, redMat, whiteTrimMat, 0);
    addFramedWin(farmH, -2.2, 2.1, 2.12, 1.2, 1.15, 0, sageMat);
    addFramedWin(farmH, 2.2, 2.1, 2.12, 1.2, 1.15, 0, sageMat);
    addFramedWin(farmH, 0, 4.95, 2.12, 1.0, 0.9, 0);
    const fhChim = new THREE.Mesh(new THREE.BoxGeometry(0.95, 6.2, 0.95), stoneMat);
    fhChim.position.set(-2.3, 3.6, -2.0);
    const fhStep = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.28, 0.72), darkWoodMat);
    fhStep.position.set(0, 0.14, 3.85);
    // Small wooden chicken coop + fenced chicken run beside the Farmhouse
    const coop = new THREE.Mesh(new THREE.BoxGeometry(2.1, 1.5, 1.8), warmWoodMat);
    coop.position.set(-5.6, 0.75, -0.5); coop.castShadow = true;
    const coopRoof = new THREE.Mesh(new THREE.BoxGeometry(2.5, 0.16, 2.2), brickMat);
    coopRoof.position.set(-5.6, 1.62, -0.5); coopRoof.rotation.z = 0.18;
    addGroundedFence(farmH, -6.8, 0.5, -4.4, 0.5, 0, 0.75, darkWoodMat);
    addGroundedFence(farmH, -6.8, 0.5, -6.8, 3.2, 0, 0.75, darkWoodMat);
    addGroundedFence(farmH, -6.8, 3.2, -4.4, 3.2, 0, 0.75, darkWoodMat);
    // Rustic wooden harvest hand-cart loaded with hay (no engine!)
    const cartBed = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.35, 2.1), warmWoodMat);
    cartBed.position.set(5.2, 0.65, 1.4);
    const cartHay = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.75, 1.7), yellowMat);
    cartHay.position.set(5.2, 1.12, 1.4);
    [-0.78, 0.78].forEach((wx) => {
      const wh = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.42, 0.12, 10), darkWoodMat);
      wh.rotation.z = Math.PI / 2; wh.position.set(5.2 + wx, 0.42, 1.4);
      farmH.add(wh);
    });
    addClothesline(farmH, 4.8, -2.5, 0.3);
    farmH.add(fhDeck, fhBody, fhChim, fhStep, coop, coopRoof, cartBed, cartHay);
    this.scene.add(farmH);
    this.addBoxCollider(-65, 3.2, 86, 8.2, 6.6, 7.6);

    // =========================================================================
    // 7. BUILDING 11 (30, 0, 88): HIGH-DETAIL VICTORIAN GLASS BOTANICAL GREENHOUSE & PACKING SHED
    // =========================================================================
    const gh = new THREE.Group();
    gh.position.set(30, 0, 88);

    // Procedural weathered red-brick masonry texture for greenhouse knee-wall
    const makeGhBrickTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#d6cfc4'; ctx.fillRect(0, 0, 128, 128);
      const bShades = ['#9c4230', '#8b3727', '#ab4c38', '#7d3223', '#a34834'];
      for (let row = 0; row < 8; row++) {
        const y = row * 16;
        const off = (row % 2) * 16;
        for (let col = -1; col < 5; col++) {
          const x = col * 32 + off;
          ctx.fillStyle = bShades[(row * 3 + col + 5) % bShades.length];
          ctx.fillRect(x + 2, y + 2, 28, 12);
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(4, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural warm horizontal cedar-plank siding texture for attached potting & packing shed
    const makeShedPlankTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#8b5e36'; ctx.fillRect(0, 0, 128, 128);
      const shades = ['#9b6b3e', '#865830', '#a67547', '#7b4f29'];
      for (let i = 0; i < 8; i++) {
        ctx.fillStyle = shades[i % shades.length];
        ctx.fillRect(0, i * 16, 128, 14);
        ctx.fillStyle = 'rgba(38, 22, 10, 0.42)';
        ctx.fillRect(0, i * 16 + 14, 128, 2);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(2, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    // Procedural standing-seam dark zinc roof texture for packing shed
    const makeShedRoofTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#334155'; ctx.fillRect(0, 0, 128, 128);
      for (let x = 0; x < 128; x += 16) {
        ctx.fillStyle = '#475569'; ctx.fillRect(x, 0, 3, 128);
        ctx.fillStyle = '#1e293b'; ctx.fillRect(x + 3, 0, 2, 128);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const ghBrickMat = new THREE.MeshStandardMaterial({ map: makeGhBrickTex(), roughness: 0.9, side: THREE.DoubleSide });
    const ghFrameMat = new THREE.MeshStandardMaterial({ color: 0x1e3a2f, roughness: 0.52, metalness: 0.45, side: THREE.DoubleSide }); // Deep Victorian botanical green steel
    const ghGlassMat = new THREE.MeshStandardMaterial({
      color: 0xdcf4ff, roughness: 0.06, metalness: 0.25,
      transparent: true, opacity: 0.34, side: THREE.DoubleSide, depthWrite: false
    });
    const ghSoilMat = new THREE.MeshStandardMaterial({ color: 0x3b2616, roughness: 0.96 });
    const ghPotMat = new THREE.MeshStandardMaterial({ color: 0xc25a34, roughness: 0.85, side: THREE.DoubleSide }); // Terracotta clay
    const ghLeafBright = new THREE.MeshStandardMaterial({ color: 0x388e3c, roughness: 0.82, flatShading: true });
    const ghLeafDeep = new THREE.MeshStandardMaterial({ color: 0x1b5e20, roughness: 0.86, flatShading: true });
    const ghFlowerPink = new THREE.MeshStandardMaterial({ color: 0xf472b6, roughness: 0.65 });
    const ghFlowerGold = new THREE.MeshStandardMaterial({ color: 0xfacc15, roughness: 0.65 });
    const ghFlowerPurp = new THREE.MeshStandardMaterial({ color: 0xa855f7, roughness: 0.65 });
    const ghZincMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.38, metalness: 0.72, side: THREE.DoubleSide });
    const shedWallMat = new THREE.MeshStandardMaterial({ map: makeShedPlankTex(), roughness: 0.86, side: THREE.DoubleSide });
    const shedRoofMat = new THREE.MeshStandardMaterial({ map: makeShedRoofTex(), roughness: 0.62, metalness: 0.4, side: THREE.DoubleSide });

    // 1. Stone Foundation Plinth + Brick Knee-Wall (with Open North Entrance Gap!) + Stone Coping Sill + Interior Floor
    const ghPlinth = new THREE.Mesh(new THREE.BoxGeometry(8.65, 0.16, 5.45), stoneMat);
    ghPlinth.position.y = 0.08; ghPlinth.receiveShadow = true;
    const ghFloor = new THREE.Mesh(new THREE.BoxGeometry(8.1, 0.08, 4.9), new THREE.MeshStandardMaterial({ color: 0x9ca3af, roughness: 0.92 }));
    ghFloor.position.y = 0.16; ghFloor.receiveShadow = true;
    // Warm brick central stone aisle running through interior
    const ghAisle = new THREE.Mesh(new THREE.BoxGeometry(7.9, 0.03, 1.35), ghBrickMat);
    ghAisle.position.y = 0.215; ghAisle.receiveShadow = true;
    const ghAisleEntry = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.03, 2.35), ghBrickMat);
    ghAisleEntry.position.set(0, 0.215, -1.55);

    // 4 Brick Knee-Walls (split on North road side z = -2.55 to frame the glazed entrance vestibule!)
    const kwS = new THREE.Mesh(new THREE.BoxGeometry(8.4, 0.72, 0.24), ghBrickMat);
    kwS.position.set(0, 0.52, 2.52); kwS.castShadow = true; kwS.receiveShadow = true;
    const kwW = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.72, 5.2), ghBrickMat);
    kwW.position.set(-4.12, 0.52, 0); kwW.castShadow = true;
    const kwE = kwW.clone(); kwE.position.x = 4.12;
    const kwNL = new THREE.Mesh(new THREE.BoxGeometry(3.25, 0.72, 0.24), ghBrickMat);
    kwNL.position.set(-2.58, 0.52, -2.52); kwNL.castShadow = true; kwNL.receiveShadow = true;
    const kwNR = kwNL.clone(); kwNR.position.x = 2.58;

    // Limestone Coping Sills crowning all brick knee-walls
    const sillS = new THREE.Mesh(new THREE.BoxGeometry(8.56, 0.1, 0.34), stoneMat);
    sillS.position.set(0, 0.91, 2.52);
    const sillW = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.1, 5.36), stoneMat);
    sillW.position.set(-4.12, 0.91, 0);
    const sillE = sillW.clone(); sillE.position.x = 4.12;
    const sillNL = new THREE.Mesh(new THREE.BoxGeometry(3.38, 0.1, 0.34), stoneMat);
    sillNL.position.set(-2.58, 0.91, -2.52);
    const sillNR = sillNL.clone(); sillNR.position.x = 2.58;
    gh.add(ghPlinth, ghFloor, ghAisle, ghAisleEntry, kwS, kwW, kwE, kwNL, kwNR, sillS, sillW, sillE, sillNL, sillNR);

    // 2. Crystal Glazed Walls, Closed Glass Gable Triangles, Pitched Roof & Hinged Roof Vents!
    const wallY = 1.98;
    const wallH = 2.08;
    const gWallS = new THREE.Mesh(new THREE.PlaneGeometry(8.2, wallH), ghGlassMat);
    gWallS.position.set(0, wallY, 2.52);
    const gWallNL = new THREE.Mesh(new THREE.PlaneGeometry(3.2, wallH), ghGlassMat);
    gWallNL.position.set(-2.55, wallY, -2.52);
    const gWallNR = gWallNL.clone(); gWallNR.position.x = 2.55;
    const gWallW = new THREE.Mesh(new THREE.PlaneGeometry(5.04, wallH), ghGlassMat);
    gWallW.rotation.y = Math.PI / 2; gWallW.position.set(-4.12, wallY, 0);
    const gWallE = gWallW.clone(); gWallE.position.x = 4.12;

    // Closed triangular glass gable ends (ZERO open sky gaps at East & West peaks!)
    const ghHalfD = 2.52;
    const ghPeakH = 1.55;
    const ghEaveY = 3.02;
    const ghGableShape = new THREE.Shape([
      new THREE.Vector2(-ghHalfD, 0),
      new THREE.Vector2(ghHalfD, 0),
      new THREE.Vector2(0, ghPeakH)
    ]);
    const ghGableGeo = new THREE.ShapeGeometry(ghGableShape);
    const gGableW = new THREE.Mesh(ghGableGeo, ghGlassMat);
    gGableW.rotation.y = Math.PI / 2; gGableW.position.set(-4.12, ghEaveY, 0);
    const gGableE = gGableW.clone(); gGableE.position.x = 4.12;

    // Pitched roof glass slopes
    const ghPitch = Math.atan2(ghPeakH, ghHalfD);
    const ghSlopeLen = Math.hypot(ghHalfD + 0.16, ghPeakH + 0.1);
    const roofMidY = ghEaveY + ghPeakH / 2;
    const ghRoofS = new THREE.Mesh(new THREE.PlaneGeometry(8.36, ghSlopeLen), ghGlassMat);
    ghRoofS.position.set(0, roofMidY, ghHalfD / 2 + 0.04);
    ghRoofS.rotation.x = -Math.PI / 2 + ghPitch;
    const ghRoofN = new THREE.Mesh(new THREE.PlaneGeometry(8.36, ghSlopeLen), ghGlassMat);
    ghRoofN.position.set(0, roofMidY, -(ghHalfD / 2 + 0.04));
    ghRoofN.rotation.x = -Math.PI / 2 - ghPitch;
    gh.add(gWallS, gWallNL, gWallNR, gWallW, gWallE, gGableW, gGableE, ghRoofS, ghRoofN);

    // 3. Detailed Victorian Botanical-Green Steel Conservatory Structure (Mullions, Transoms, Purlins, Gutters & Cresting!)
    // Vertical wall mullions & arched roof rafters every 1.37m (7 bays)
    for (let i = 0; i <= 6; i++) {
      const gx = -4.12 + (i / 6) * 8.24;
      const isEnd = (i === 0 || i === 6);
      const isDoorPost = (i === 2 || i === 4);
      const colW = isEnd ? 0.13 : 0.08;
      // South wall mullion
      const postS = new THREE.Mesh(new THREE.BoxGeometry(colW, wallH + 0.06, colW), ghFrameMat);
      postS.position.set(gx, wallY, 2.52); postS.castShadow = true; gh.add(postS);
      // North wall mullion (skip center i=3 since entrance doors occupy center bay!)
      if (i !== 3) {
        const postN = new THREE.Mesh(new THREE.BoxGeometry(isDoorPost ? 0.12 : colW, wallH + 0.06, isDoorPost ? 0.12 : colW), ghFrameMat);
        postN.position.set(gx, wallY, -2.52); postN.castShadow = true; gh.add(postN);
      }
      // Sloped roof rafters meeting cleanly at peak
      const rafS = new THREE.Mesh(new THREE.BoxGeometry(isEnd ? 0.12 : 0.075, 0.09, ghSlopeLen), ghFrameMat);
      rafS.position.set(gx, roofMidY, ghHalfD / 2 + 0.04); rafS.rotation.x = ghPitch;
      const rafN = new THREE.Mesh(new THREE.BoxGeometry(isEnd ? 0.12 : 0.075, 0.09, ghSlopeLen), ghFrameMat);
      rafN.position.set(gx, roofMidY, -(ghHalfD / 2 + 0.04)); rafN.rotation.x = -ghPitch;
      gh.add(rafS, rafN);
      // Horizontal steel tie-beam truss across interior eaves on major bays
      if (i === 0 || i === 2 || i === 4 || i === 6) {
        const tie = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.08, 5.04), ghFrameMat);
        tie.position.set(gx, ghEaveY, 0);
        const king = new THREE.Mesh(new THREE.BoxGeometry(0.07, ghPeakH, 0.07), ghFrameMat);
        king.position.set(gx, ghEaveY + ghPeakH / 2, 0);
        gh.add(tie, king);
      }
    }
    // End-wall intermediate vertical mullions (West & East walls)
    [-1.26, 0, 1.26].forEach((gz) => {
      [-4.12, 4.12].forEach((gx) => {
        const ep = new THREE.Mesh(new THREE.BoxGeometry(0.07, wallH, 0.07), ghFrameMat);
        ep.position.set(gx, wallY, gz);
        gh.add(ep);
      });
    });
    // Horizontal glazing transoms (mid-wall bars), eave beams, roof purlins & galvanized rain gutters!
    [1.96, ghEaveY].forEach((hy) => {
      const barS = new THREE.Mesh(new THREE.BoxGeometry(8.32, 0.07, 0.09), ghFrameMat);
      barS.position.set(0, hy, 2.52);
      const barW = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.07, 5.12), ghFrameMat);
      barW.position.set(-4.12, hy, 0);
      const barE = barW.clone(); barE.position.x = 4.12;
      gh.add(barS, barW, barE);
      if (hy === ghEaveY) {
        const barN = barS.clone(); barN.position.z = -2.52; gh.add(barN);
      } else {
        const bNL = new THREE.Mesh(new THREE.BoxGeometry(3.2, 0.07, 0.09), ghFrameMat);
        bNL.position.set(-2.55, hy, -2.52);
        const bNR = bNL.clone(); bNR.position.x = 2.55;
        gh.add(bNL, bNR);
      }
    });
    // Roof purlins & zinc eave rain gutters on North and South sides
    [-1, 1].forEach((side) => {
      const purlin = new THREE.Mesh(new THREE.BoxGeometry(8.3, 0.06, 0.06), ghFrameMat);
      purlin.position.set(0, ghEaveY + ghPeakH * 0.52, side * ghHalfD * 0.48);
      const gutter = new THREE.Mesh(new THREE.BoxGeometry(8.55, 0.1, 0.14), ghZincMat);
      gutter.position.set(0, ghEaveY - 0.06, side * (ghHalfD + 0.14));
      gh.add(purlin, gutter);
    });
    // Ornamental Victorian Ridge Beam & Decorative Cresting Finials along Roof Peak!
    const ghRidge = new THREE.Mesh(new THREE.BoxGeometry(8.46, 0.14, 0.12), ghFrameMat);
    ghRidge.position.y = ghEaveY + ghPeakH + 0.04;
    gh.add(ghRidge);
    for (let fx = -4.0; fx <= 4.01; fx += 0.5) {
      const isSpire = (Math.abs(fx) > 3.8 || Math.abs(fx) < 0.1);
      const finial = new THREE.Mesh(
        new THREE.ConeGeometry(isSpire ? 0.075 : 0.04, isSpire ? 0.36 : 0.2, 4),
        ghFrameMat
      );
      finial.position.set(fx, ghEaveY + ghPeakH + (isSpire ? 0.28 : 0.19), 0);
      gh.add(finial);
    }
    // 2 Tilted Openable Roof Ventilation Sash Windows popped ajar on North & South roof slopes!
    [[-2.05, -1], [2.05, 1]].forEach(([vx, side]) => {
      const vent = new THREE.Group();
      vent.position.set(vx, ghEaveY + ghPeakH - 0.04, side * 0.08);
      vent.rotation.x = side * (ghPitch - 0.36); // Tilted open upwards for greenhouse ventilation!
      const vFrame = new THREE.Mesh(new THREE.BoxGeometry(1.32, 0.06, 1.05), ghFrameMat);
      vFrame.position.set(0, 0.04, side * 0.52);
      const vGlass = new THREE.Mesh(new THREE.BoxGeometry(1.18, 0.03, 0.92), ghGlassMat);
      vGlass.position.set(0, 0.04, side * 0.52);
      vent.add(vFrame, vGlass);
      gh.add(vent);
    });

    // 4. Glazed Victorian Entrance Vestibule, Double Glass Doors, Stone Steps, Lantern & Signboard facing Road (-Z)!
    const porchStep1 = new THREE.Mesh(new THREE.BoxGeometry(2.15, 0.16, 0.95), stoneMat);
    porchStep1.position.set(0, 0.08, -2.95); porchStep1.receiveShadow = true;
    const porchStep2 = new THREE.Mesh(new THREE.BoxGeometry(1.9, 0.12, 0.55), stoneMat);
    porchStep2.position.set(0, 0.18, -2.75);
    // Double Victorian glazed entrance doors (left & right door leaves with brass handles)
    [-0.45, 0.45].forEach((dx) => {
      const dFrame = new THREE.Mesh(new THREE.BoxGeometry(0.86, 2.25, 0.1), ghFrameMat);
      dFrame.position.set(dx, 1.35, -2.54);
      const dBot = new THREE.Mesh(new THREE.BoxGeometry(0.74, 0.72, 0.12), ghFrameMat);
      dBot.position.set(dx, 0.62, -2.54);
      const dPane = new THREE.Mesh(new THREE.BoxGeometry(0.68, 1.32, 0.13), ghGlassMat);
      dPane.position.set(dx, 1.72, -2.54);
      const dHandle = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.22, 6), brassMat);
      dHandle.position.set(dx + (dx < 0 ? 0.34 : -0.34), 1.25, -2.63);
      gh.add(dFrame, dBot, dPane, dHandle);
    });
    // Overhead glazed entrance canopy awning + Victorian support brackets + warm hanging porch lantern + Signboard!
    const eCanopyL = new THREE.Mesh(new THREE.BoxGeometry(1.25, 0.08, 1.15), ghFrameMat);
    eCanopyL.position.set(-0.56, 2.86, -3.02); eCanopyL.rotation.z = 0.35;
    const eCanopyR = new THREE.Mesh(new THREE.BoxGeometry(1.25, 0.08, 1.15), ghFrameMat);
    eCanopyR.position.set(0.56, 2.86, -3.02); eCanopyR.rotation.z = -0.35;
    [-0.95, 0.95].forEach((px) => {
      const ep = new THREE.Mesh(new THREE.BoxGeometry(0.1, 2.6, 0.1), ghFrameMat);
      ep.position.set(px, 1.42, -3.45); ep.castShadow = true;
      gh.add(ep);
    });
    const porchLamp = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.28, 0.22), lampGlowMat);
    porchLamp.position.set(0, 2.66, -3.12);
    const ghSign = new THREE.Mesh(
      new THREE.BoxGeometry(2.65, 0.68, 0.1),
      new THREE.MeshStandardMaterial({ map: makeSignTex('GREENHOUSE', 'BOTANICAL NURSERY', '#14532d', '#facc15'), roughness: 0.5 })
    );
    ghSign.position.set(0, 3.42, -2.58);
    ghSign.rotation.y = Math.PI; // Faces cleanly North (-Z) toward the South Valley road!
    gh.add(porchStep1, porchStep2, eCanopyL, eCanopyR, porchLamp, ghSign);

    // 5. Lush Layered Interior: Cedar Potting Benches, Terracotta Pots, Flowers, Tomato Trellises & Overhead Grow Lamps!
    const addBotanicalBed = (cx, cz, len, hasTrellis = false) => {
      // Cedar table top, rimmed soil box & 4 wooden support legs
      const frame = new THREE.Mesh(new THREE.BoxGeometry(len, 0.22, 1.25), warmWoodMat);
      frame.position.set(cx, 0.92, cz);
      const soil = new THREE.Mesh(new THREE.BoxGeometry(len - 0.16, 0.1, 1.08), ghSoilMat);
      soil.position.set(cx, 1.02, cz);
      [-len / 2 + 0.18, len / 2 - 0.18].forEach((lx) => {
        [-0.5, 0.5].forEach((lz) => {
          const leg = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.85, 0.1), darkWoodMat);
          leg.position.set(cx + lx, 0.45, cz + lz);
          gh.add(leg);
        });
      });
      // Lower wooden storage shelf under bench with spare terracotta pots
      const shelf = new THREE.Mesh(new THREE.BoxGeometry(len - 0.24, 0.06, 0.95), warmWoodMat);
      shelf.position.set(cx, 0.42, cz);
      gh.add(frame, soil, shelf);

      // Rows of individual terracotta pots, varied leafy seedlings & vibrant flower blooms!
      const nCols = Math.max(3, Math.round(len / 0.72));
      const fMats = [ghFlowerPink, ghFlowerGold, ghFlowerPurp, redMat];
      for (let c = 0; c < nCols; c++) {
        const px = cx - (len - 0.7) / 2 + (nCols > 1 ? (c / (nCols - 1)) * (len - 0.7) : 0);
        [-0.26, 0.26].forEach((pzOff, rIdx) => {
          const pz = cz + pzOff;
          const pot = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.11, 0.22, 7), ghPotMat);
          pot.position.set(px, 1.15, pz);
          const leaf = new THREE.Mesh(
            new THREE.DodecahedronGeometry(0.22 + ((c + rIdx) % 3) * 0.045, 0),
            ((c + rIdx) % 2 === 0) ? ghLeafBright : ghLeafDeep
          );
          leaf.scale.set(1.1, 0.85, 1.1);
          leaf.position.set(px, 1.36, pz);
          gh.add(pot, leaf);
          if ((c + rIdx) % 2 === 0) {
            const bloom = new THREE.Mesh(new THREE.SphereGeometry(0.085, 6, 6), fMats[(c + rIdx) % fMats.length]);
            bloom.position.set(px + 0.04, 1.56, pz);
            gh.add(bloom);
          }
        });
      }
      // Tall wooden climbing tomato trellises with lush vines & bright red tomatoes along back bench!
      if (hasTrellis) {
        for (let tx = cx - len / 2 + 0.75; tx <= cx + len / 2 - 0.6; tx += 1.65) {
          [-0.28, 0.28].forEach((ox) => {
            const stake = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.035, 1.65, 5), darkWoodMat);
            stake.position.set(tx + ox, 1.85, cz + 0.25);
            gh.add(stake);
          });
          [1.55, 2.05, 2.48].forEach((ty, bi) => {
            const bar = new THREE.Mesh(new THREE.BoxGeometry(0.66, 0.04, 0.04), warmWoodMat);
            bar.position.set(tx, ty, cz + 0.25);
            const vine = new THREE.Mesh(new THREE.DodecahedronGeometry(0.26, 0), bi % 2 === 0 ? ghLeafBright : ghLeafDeep);
            vine.position.set(tx + (bi - 1) * 0.12, ty + 0.05, cz + 0.22);
            const tomato = new THREE.Mesh(new THREE.SphereGeometry(0.08, 6, 6), redMat);
            tomato.position.set(tx + (bi % 2 === 0 ? 0.16 : -0.14), ty - 0.04, cz + 0.04);
            gh.add(bar, vine, tomato);
          });
        }
      }
    };
    addBotanicalBed(0, 1.55, 7.2, true);     // Full-length sunny South bed with tall climbing tomato trellises!
    addBotanicalBed(-2.45, -1.55, 2.55, false); // North-West seedling & flower bench (left of entrance)
    addBotanicalBed(2.45, -1.55, 2.55, false);  // North-East herb & flower bench (right of entrance)

    // Overhead galvanized irrigation pipe & 3 suspended warm horticultural grow-light bars
    const irrPipe = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 7.8, 8), ghZincMat);
    irrPipe.rotation.z = Math.PI / 2;
    irrPipe.position.set(0, 3.62, 0);
    gh.add(irrPipe);
    [-2.5, 0, 2.5].forEach((lx) => {
      const cord = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.85, 4), darkTrimMat);
      cord.position.set(lx, 4.02, 0);
      const hood = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.12, 0.36), ghFrameMat);
      hood.position.set(lx, 3.55, 0);
      const bulb = new THREE.Mesh(new THREE.BoxGeometry(1.02, 0.05, 0.22), lampGlowMat);
      bulb.position.set(lx, 3.48, 0);
      gh.add(cord, hood, bulb);
    });

    // 6. Upgraded Cedar-Plank Packing & Potting Shed (East Wing x = +5.7) with Door, Window, Metal Roof, Chimney & Rainwater Barrel!
    const shedPlinth = new THREE.Mesh(new THREE.BoxGeometry(3.45, 0.26, 4.9), stoneMat);
    shedPlinth.position.set(5.8, 0.13, 0); shedPlinth.receiveShadow = true;
    const packShed = new THREE.Mesh(new THREE.BoxGeometry(3.25, 2.75, 4.65), shedWallMat);
    packShed.position.set(5.8, 1.62, 0); packShed.castShadow = true; packShed.receiveShadow = true;
    // Corner cedar trim battens on the packing shed
    [[4.18, -2.3], [7.42, -2.3], [4.18, 2.3], [7.42, 2.3]].forEach(([sx, sz]) => {
      const tr = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.8, 0.14), darkWoodMat);
      tr.position.set(sx, 1.62, sz); gh.add(tr);
    });
    // Standing-seam zinc shed roof with fascia trim, metal stove flue & rainwater gutter!
    const packRoof = new THREE.Mesh(new THREE.BoxGeometry(3.85, 0.18, 5.25), shedRoofMat);
    packRoof.position.set(5.88, 3.12, 0); packRoof.rotation.z = -0.18; packRoof.castShadow = true;
    const shedFlue = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.16, 1.45, 8), darkTrimMat);
    shedFlue.position.set(5.35, 3.85, 1.25); shedFlue.castShadow = true;
    const shedFlueCap = new THREE.Mesh(new THREE.ConeGeometry(0.24, 0.18, 8), darkTrimMat);
    shedFlueCap.position.set(5.35, 4.62, 1.25);
    // Framed North Workshop Door + Steps + Awning + Wall Lantern on Packing Shed facing road (-Z)!
    addEntryDoor(gh, 5.75, 1.22, -2.34, 1.05, 1.95, darkWoodMat, whiteTrimMat, Math.PI);
    const shedStep = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.22, 0.55), stoneMat);
    shedStep.position.set(5.75, 0.11, -2.62);
    const shedAwning = new THREE.Mesh(new THREE.BoxGeometry(1.65, 0.1, 0.72), ghFrameMat);
    shedAwning.position.set(5.75, 2.38, -2.68); shedAwning.rotation.x = -0.22;
    const shedLamp = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.22, 0.16), lampGlowMat);
    shedLamp.position.set(6.55, 2.08, -2.42);
    // Framed Multi-Pane Workshop Windows on East & South walls with Flower Box
    addFramedWin(gh, 7.44, 1.75, 0, 1.35, 1.05, Math.PI / 2, ghFrameMat);
    addFramedWin(gh, 5.8, 1.75, 2.34, 1.35, 1.05, 0, ghFrameMat);
    const wBox = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.24, 1.45), darkWoodMat);
    wBox.position.set(7.6, 1.12, 0);
    const wBloom = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.22, 1.28), ghLeafBright);
    wBloom.position.set(7.6, 1.3, 0);
    gh.add(shedPlinth, packShed, packRoof, shedFlue, shedFlueCap, shedStep, shedAwning, shedLamp, wBox, wBloom);

    // Blue Rainwater Harvesting Cistern + Downspout Pipe catching water from greenhouse/shed roof!
    const rainTank = new THREE.Mesh(new THREE.CylinderGeometry(0.58, 0.62, 1.38, 14), blueMat);
    rainTank.position.set(7.92, 0.72, -1.65); rainTank.castShadow = true;
    const rainRim = new THREE.Mesh(new THREE.TorusGeometry(0.6, 0.045, 6, 14), darkTrimMat);
    rainRim.rotation.x = Math.PI / 2; rainRim.position.set(7.92, 1.4, -1.65);
    const downSpout = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 1.55, 6), ghZincMat);
    downSpout.position.set(7.56, 2.18, -1.65);
    const spoutElbow = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.42, 6), ghZincMat);
    spoutElbow.rotation.z = Math.PI / 2;
    spoutElbow.position.set(7.74, 1.45, -1.65);
    gh.add(rainTank, rainRim, downSpout, spoutElbow);

    // 7. Rich Storytelling Exterior Gardening Props (Harvest Crates, Wheelbarrow, Potting Table & Cold-Frame Cloche!)
    // Slatted wooden harvest crates overflowing with fresh valley tomatoes, pumpkins & cabbages beside entrance!
    [
      { x: -2.45, z: -3.12, y: 0.28, prod: redMat },
      { x: -1.55, z: -3.18, y: 0.28, prod: yellowMat },
      { x: -2.0,  z: -3.15, y: 0.84, prod: ghLeafBright },
      { x: 4.45,  z: -2.95, y: 0.28, prod: redMat }
    ].forEach((c) => {
      const cr = new THREE.Mesh(new THREE.BoxGeometry(0.82, 0.56, 0.66), warmWoodMat);
      cr.position.set(c.x, c.y, c.z); cr.castShadow = true;
      const crRim = new THREE.Mesh(new THREE.BoxGeometry(0.86, 0.07, 0.7), darkWoodMat);
      crRim.position.set(c.x, c.y + 0.26, c.z);
      gh.add(cr, crRim);
      for (let k = -1; k <= 1; k++) {
        const veg = new THREE.Mesh(new THREE.SphereGeometry(0.14, 7, 7), c.prod);
        veg.position.set(c.x + k * 0.22, c.y + 0.34, c.z + (k === 0 ? 0.06 : -0.06));
        gh.add(veg);
      }
    });

    // Exterior Wooden Potting Bench on North-East front of greenhouse (+2.55, -3.12) with Stacked Terracotta Pots & Watering Can
    const outBench = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.1, 0.72), warmWoodMat);
    outBench.position.set(2.55, 0.78, -3.12);
    [-0.75, 0.75].forEach((bx) => {
      const bLeg = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.76, 0.62), darkWoodMat);
      bLeg.position.set(2.55 + bx, 0.38, -3.12); gh.add(bLeg);
    });
    // Stacked terracotta pots + green metal watering can resting on outdoor potting table
    for (let p = 0; p < 3; p++) {
      const sp = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.1, 0.2, 8), ghPotMat);
      sp.position.set(2.05, 0.92 + p * 0.14, -3.12); gh.add(sp);
    }
    const wCan = new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.15, 0.28, 8), ghFrameMat);
    wCan.position.set(2.95, 0.97, -3.12);
    const wSpout = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.04, 0.32, 6), ghFrameMat);
    wSpout.rotation.z = -0.8; wSpout.position.set(3.15, 1.02, -3.12);
    gh.add(outBench, wCan, wSpout);

    // Classic Single-Wheel Garden Wheelbarrow filled with rich soil parked near Packing Shed (+7.0, -3.2)
    const wb = new THREE.Group();
    wb.position.set(7.05, 0, -3.25); wb.rotation.y = -0.52;
    const wbTray = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.36, 1.05), ghFrameMat);
    wbTray.position.set(0, 0.56, 0); wbTray.castShadow = true;
    const wbDirt = new THREE.Mesh(new THREE.SphereGeometry(0.36, 7, 6), ghSoilMat);
    wbDirt.scale.set(0.95, 0.45, 1.2); wbDirt.position.set(0, 0.74, 0);
    const wbWheel = new THREE.Mesh(new THREE.TorusGeometry(0.22, 0.055, 6, 12), darkTrimMat);
    wbWheel.rotation.y = Math.PI / 2; wbWheel.position.set(0, 0.24, 0.62);
    [-0.28, 0.28].forEach((hx) => {
      const handle = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 1.55), warmWoodMat);
      handle.position.set(hx, 0.44, -0.15);
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.42, 0.06), darkTrimMat);
      leg.position.set(hx, 0.21, -0.45);
      wb.add(handle, leg);
    });
    wb.add(wbTray, wbDirt, wbWheel);
    gh.add(wb);

    // Sunny West Mini Cold-Frame Glass Cloche Planter Box (-5.35, 0, 0.6) for hardening young valley seedlings!
    const cfBox = new THREE.Mesh(new THREE.BoxGeometry(1.45, 0.48, 2.4), warmWoodMat);
    cfBox.position.set(-5.35, 0.24, 0.55); cfBox.castShadow = true;
    const cfSoil = new THREE.Mesh(new THREE.BoxGeometry(1.3, 0.08, 2.24), ghSoilMat);
    cfSoil.position.set(-5.35, 0.46, 0.55);
    const cfLid = new THREE.Mesh(new THREE.BoxGeometry(1.52, 0.06, 2.46), ghGlassMat);
    cfLid.position.set(-5.35, 0.6, 0.55); cfLid.rotation.z = 0.16;
    const cfRim = new THREE.Mesh(new THREE.BoxGeometry(1.56, 0.06, 0.08), ghFrameMat);
    cfRim.position.set(-5.35, 0.6, -0.68); cfRim.rotation.z = 0.16;
    const cfRim2 = cfRim.clone(); cfRim2.position.z = 1.78;
    [-0.65, 0, 0.65].forEach((sz) => {
      const sprout = new THREE.Mesh(new THREE.DodecahedronGeometry(0.2, 0), ghLeafBright);
      sprout.position.set(-5.35, 0.56, 0.55 + sz); gh.add(sprout);
    });
    gh.add(cfBox, cfSoil, cfLid, cfRim, cfRim2);

    this.scene.add(gh);
    this.addBoxCollider(31.2, 2.35, 88, 13.6, 4.7, 5.8);

    // =========================================================================
    // 8. BUILDING 12 (-10, 0, 92): HIGH-DETAIL HISTORIC STONE TOWER WINDMILL (Gallery Stage, Fantail, Lattice Sails & Millstone!)
    // =========================================================================
    const mill = new THREE.Group();
    mill.position.set(-10, 0, 92);

    // Procedural Textures: Cobble Fieldstone Masonry Base, Aged Lime-Plaster Shaft & Woven Sailcloth
    const makeMillStoneTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#5f6369'; ctx.fillRect(0, 0, 256, 128);
      const shades = ['#7b7f86', '#6b7077', '#8b8e91', '#5c6168', '#837f77'];
      for (let row = 0; row < 6; row++) {
        const y = row * 22;
        const off = (row % 2) * 16;
        for (let col = -1; col < 9; col++) {
          ctx.fillStyle = shades[(row * 3 + col + 7) % shades.length];
          ctx.fillRect(col * 32 + off + 2, y + 2, 28, 18);
        }
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeMillPlasterTex = () => {
      const c = document.createElement('canvas');
      c.width = 256; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#efe7d8'; ctx.fillRect(0, 0, 256, 256);
      // Warm weathered plaster stucco blotches & subtle horizontal stone courses beneath limewash
      for (let y = 0; y < 256; y += 32) {
        ctx.fillStyle = 'rgba(130, 115, 92, 0.14)';
        ctx.fillRect(0, y, 256, 2);
      }
      for (let i = 0; i < 420; i++) {
        const x = (i * 73 + (i % 13) * 19) % 256;
        const y = (i * 41 + (i % 17) * 13) % 256;
        ctx.fillStyle = (i % 2 === 0) ? 'rgba(255,252,245,0.18)' : 'rgba(120,104,80,0.12)';
        ctx.fillRect(x, y, 5, 4);
      }
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 2);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const makeSailClothTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 256;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#f5ebd4'; ctx.fillRect(0, 0, 128, 256);
      // Woven linen vertical seams & reefing tie lines
      ctx.fillStyle = 'rgba(140, 115, 80, 0.28)';
      for (let y = 0; y < 256; y += 32) ctx.fillRect(0, y, 128, 2);
      for (let x = 0; x < 128; x += 32) ctx.fillRect(x, 0, 2, 256);
      // Classic crimson woven outer border trim along sail edge
      ctx.fillStyle = '#991b1b';
      ctx.fillRect(114, 0, 14, 256);
      ctx.fillRect(0, 0, 128, 12);
      ctx.fillRect(0, 244, 128, 12);
      const tex = new THREE.CanvasTexture(c);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };

    const millStoneMat = new THREE.MeshStandardMaterial({ map: makeMillStoneTex(), roughness: 0.94, side: THREE.DoubleSide });
    const millPlasterMat = new THREE.MeshStandardMaterial({ map: makeMillPlasterTex(), roughness: 0.88, side: THREE.DoubleSide });
    const millRoofMat = makeRoofMat('#633316', '#3d1d0a');
    const millShutterMat = new THREE.MeshStandardMaterial({ color: 0x1e4d30, roughness: 0.8, side: THREE.DoubleSide });
    const sailCanvasMat = new THREE.MeshStandardMaterial({ map: makeSailClothTex(), roughness: 0.86, side: THREE.DoubleSide });

    // 1. Cobble Apron, Fieldstone Masonry Base, Moulded Drip Ring & Tapered Limewashed Tower (24-sided!)
    const millApron = new THREE.Mesh(new THREE.CylinderGeometry(4.4, 4.6, 0.1, 24), stoneMat);
    millApron.position.y = 0.05; millApron.receiveShadow = true;
    const millBase = new THREE.Mesh(new THREE.CylinderGeometry(3.2, 3.68, 2.8, 24), millStoneMat);
    millBase.position.y = 1.4; millBase.castShadow = true; millBase.receiveShadow = true;
    const millBelt = new THREE.Mesh(new THREE.CylinderGeometry(3.32, 3.35, 0.22, 24), stoneMat);
    millBelt.position.y = 2.82;
    const millTower = new THREE.Mesh(new THREE.CylinderGeometry(2.12, 3.16, 7.25, 24), millPlasterMat);
    millTower.position.y = 6.45; millTower.castShadow = true; millTower.receiveShadow = true;
    const millTaper = Math.atan2(3.16 - 2.12, 7.25);
    const millRadAt = (y) => 3.16 + (2.12 - 3.16) * ((y - 2.82) / 7.25);

    // 2. Iconic Wraparound Wooden Mid-Level Reefing Gallery ("Stage") with 12 Knee Braces & Grounded 16-Post Handrail!
    const stageY = 2.95;
    const stageRing = new THREE.Mesh(new THREE.RingGeometry(3.05, 4.18, 24), warmWoodMat);
    stageRing.rotation.x = -Math.PI / 2; stageRing.position.y = stageY;
    const stageRim = new THREE.Mesh(new THREE.CylinderGeometry(4.2, 4.2, 0.18, 24, 1, true), darkWoodMat);
    stageRim.position.y = stageY - 0.06;
    const stageRailTop = new THREE.Mesh(new THREE.TorusGeometry(4.12, 0.055, 6, 28), darkWoodMat);
    stageRailTop.rotation.x = Math.PI / 2; stageRailTop.position.y = stageY + 0.88;
    const stageRailMid = new THREE.Mesh(new THREE.TorusGeometry(4.12, 0.035, 6, 28), warmWoodMat);
    stageRailMid.rotation.x = Math.PI / 2; stageRailMid.position.y = stageY + 0.45;
    mill.add(stageRing, stageRim, stageRailTop, stageRailMid);
    // 12 Diagonal Timber Corbel Braces supporting the cantilevered stage + 16 Grounded Vertical Balusters
    for (let k = 0; k < 12; k++) {
      const ka = (k / 12) * Math.PI * 2;
      const cg = new THREE.Group(); cg.rotation.y = ka;
      const strut = new THREE.Mesh(new THREE.BoxGeometry(0.14, 1.45, 0.14), darkWoodMat);
      strut.position.set(3.62, stageY - 0.58, 0); strut.rotation.z = -0.62;
      strut.castShadow = true;
      cg.add(strut); mill.add(cg);
    }
    for (let p = 0; p < 16; p++) {
      const pa = (p / 16) * Math.PI * 2;
      const rp = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.92, 0.08), darkWoodMat);
      rp.position.set(Math.cos(pa) * 4.12, stageY + 0.45, Math.sin(pa) * 4.12);
      mill.add(rp);
    }

    // 3. Stone Entrance Portico, Arched Plank Door, Steps, Lantern, Mill Signboard & Taper-Aligned Shuttered Windows!
    const mPortico = new THREE.Mesh(new THREE.BoxGeometry(1.85, 2.45, 0.85), millStoneMat);
    mPortico.position.set(0, 1.28, -3.45); mPortico.castShadow = true;
    const mPortRoof = new THREE.Mesh(new THREE.BoxGeometry(2.15, 0.16, 1.15), millRoofMat);
    mPortRoof.position.set(0, 2.55, -3.55); mPortRoof.rotation.x = -0.22;
    const mStep = new THREE.Mesh(new THREE.BoxGeometry(1.75, 0.22, 0.65), stoneMat);
    mStep.position.set(0, 0.11, -4.05);
    addEntryDoor(mill, 0, 1.15, -3.88, 1.08, 1.98, darkWoodMat, whiteTrimMat, Math.PI);
    const mLantern = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.26, 0.18), lampGlowMat);
    mLantern.position.set(0.75, 2.15, -3.95);
    const mSign = new THREE.Mesh(
      new THREE.BoxGeometry(1.95, 0.56, 0.09),
      new THREE.MeshStandardMaterial({ map: makeSignTex('VALLEY MILL', 'STONE GROUND FLOUR', '#451a03', '#f59e0b'), roughness: 0.55 })
    );
    mSign.position.set(0, 3.38, -4.22); mSign.rotation.y = Math.PI;
    mill.add(mPortico, mPortRoof, mStep, mLantern, mSign);

    // 4 Staggered Multi-Pane Shuttered Windows tilted flush to the tapered tower wall!
    [[Math.PI, 4.9], [Math.PI / 2, 6.2], [0, 5.4], [-Math.PI / 2, 7.6]].forEach(([ang, wy]) => {
      const wr = millRadAt(wy);
      const wg = new THREE.Group(); wg.rotation.y = ang;
      const winHolder = new THREE.Group();
      winHolder.position.set(0, wy, wr + 0.03);
      winHolder.rotation.x = -millTaper;
      addFramedWin(winHolder, 0, 0, 0, 0.88, 1.0, 0, millShutterMat);
      const sill = new THREE.Mesh(new THREE.BoxGeometry(1.15, 0.12, 0.24), stoneMat);
      sill.position.set(0, -0.58, 0.06); winHolder.add(sill);
      wg.add(winHolder); mill.add(wg);
    });
    // Protruding Wooden Sack-Hoist Beam & Rope Pulley jutting from East Upper Loft
    const hoistArm = new THREE.Mesh(new THREE.BoxGeometry(1.35, 0.16, 0.16), darkWoodMat);
    hoistArm.position.set(2.65, 8.75, 0);
    const hoistRope = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 2.6, 5), warmWoodMat);
    hoistRope.position.set(3.2, 7.45, 0);
    mill.add(hoistArm, hoistRope);

    // 4. Shingled Boat/Dome Cap + Wooden Gear Curb + Rear Tailpole & Wind-Turning Fantail Wheel!
    const millCollar = new THREE.Mesh(new THREE.CylinderGeometry(2.38, 2.22, 0.48, 24), darkWoodMat);
    millCollar.position.y = 10.18;
    const millCapBase = new THREE.Mesh(new THREE.CylinderGeometry(2.48, 2.62, 0.72, 24), millRoofMat);
    millCapBase.position.y = 10.75; millCapBase.castShadow = true;
    const millCap = new THREE.Mesh(new THREE.ConeGeometry(2.55, 2.35, 24), millRoofMat);
    millCap.position.y = 12.25; millCap.castShadow = true;
    const millFinial = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.08, 1.15, 8), brassMat);
    millFinial.position.y = 13.75;
    const millBall = new THREE.Mesh(new THREE.SphereGeometry(0.18, 8, 8), brassMat);
    millBall.position.y = 13.52;
    // Iconic Rear Timber Tailpole & Cap-Turning Wheel (+Z back of cap)
    const tailBeam = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.2, 3.2), darkWoodMat);
    tailBeam.position.set(0, 10.65, 2.65); tailBeam.rotation.x = -0.22;
    const fantailWheel = new THREE.Mesh(new THREE.TorusGeometry(0.92, 0.06, 6, 16), darkWoodMat);
    fantailWheel.position.set(0, 11.2, 4.15);
    for (let s = 0; s < 6; s++) {
      const spoke = new THREE.Mesh(new THREE.BoxGeometry(0.06, 1.8, 0.14), warmWoodMat);
      spoke.rotation.z = (s * Math.PI) / 6;
      spoke.position.set(0, 11.2, 4.15);
      mill.add(spoke);
    }
    mill.add(millApron, millBase, millBelt, millTower, millCollar, millCapBase, millCap, millFinial, millBall, tailBeam, fantailWheel);

    // 5. Rotating 4-Blade Tapered Timber Lattice Sails (Realistically pitched 9° upward to clear gallery & tapered walls!)
    this.windmillSail = new THREE.Group();
    this.windmillSail.position.set(0, 10.45, -2.55);
    this.windmillSail.rotation.x = 0.16; // Authentic 9° upward windshaft tilt!
    const hubCanister = new THREE.Mesh(new THREE.CylinderGeometry(0.46, 0.56, 1.25, 12), darkTrimMat);
    hubCanister.rotation.x = Math.PI / 2;
    const hubNoseCap = new THREE.Mesh(new THREE.SphereGeometry(0.45, 10, 8), brassMat);
    hubNoseCap.position.z = -0.62;
    this.windmillSail.add(hubCanister, hubNoseCap);

    for (let b = 0; b < 4; b++) {
      const armGrp = new THREE.Group();
      armGrp.rotation.z = (b * Math.PI) / 2 + 0.35;
      // Heavy tapered main wooden stock/whip spar + outer hemlath edge rail + leading wind-board!
      const spar = new THREE.Mesh(new THREE.BoxGeometry(0.18, 6.25, 0.16), darkWoodMat);
      spar.position.y = 3.12; spar.castShadow = true;
      const outerRail = new THREE.Mesh(new THREE.BoxGeometry(0.07, 4.85, 0.07), darkWoodMat);
      outerRail.position.set(1.26, 3.8, 0.06);
      const midRail = new THREE.Mesh(new THREE.BoxGeometry(0.055, 4.85, 0.055), warmWoodMat);
      midRail.position.set(0.65, 3.8, 0.04);
      const leadBoard = new THREE.Mesh(new THREE.BoxGeometry(0.24, 4.8, 0.05), millRoofMat);
      leadBoard.position.set(-0.18, 3.8, -0.02);
      // Woven linen sailcloth with crimson border laced across 80% of lattice frame (revealing crisp open timber rungs!)
      const latticeCloth = new THREE.Mesh(new THREE.BoxGeometry(1.16, 4.55, 0.035), sailCanvasMat);
      latticeCloth.position.set(0.66, 3.85, 0.08);
      latticeCloth.rotation.y = 0.14; // Realistic sail weather-angle pitch!
      latticeCloth.castShadow = true;
      armGrp.add(spar, outerRail, midRail, leadBoard, latticeCloth);
      // 9 Individual wooden cross-rungs forming classic open Dutch lattice grid
      for (let r = 1.45; r <= 6.15; r += 0.58) {
        const rung = new THREE.Mesh(new THREE.BoxGeometry(1.36, 0.06, 0.08), darkWoodMat);
        rung.position.set(0.62, r, 0.03);
        armGrp.add(rung);
      }
      this.windmillSail.add(armGrp);
    }
    mill.add(this.windmillSail);

    // 6. Authentic Miller's Yard Storytelling Props: Carved Granite Millstone, Flour Sacks & Wooden Grain Barrels!
    const millstone = new THREE.Mesh(new THREE.CylinderGeometry(0.78, 0.78, 0.26, 16), stoneMat);
    millstone.rotation.x = Math.PI / 2 - 0.25;
    millstone.position.set(-2.45, 0.75, -3.15); millstone.castShadow = true;
    const msHole = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.3, 8), darkWoodMat);
    msHole.rotation.copy(millstone.rotation); msHole.position.copy(millstone.position);
    mill.add(millstone, msHole);

    // Stacked tied burlap flour sacks & wooden grain barrels beside portico steps
    [[-1.45, -4.05, 0.42], [-0.95, -4.25, 0.4], [-1.22, -4.12, 0.98], [1.45, -4.05, 0.42]].forEach(([sx, sz, sy]) => {
      const sack = new THREE.Mesh(new THREE.SphereGeometry(0.35, 8, 8), new THREE.MeshStandardMaterial({ color: 0xcab38b, roughness: 0.95 }));
      sack.scale.set(1.0, 1.25, 0.85); sack.position.set(sx, sy, sz); sack.castShadow = true;
      const tie = new THREE.Mesh(new THREE.TorusGeometry(0.14, 0.03, 6, 10), redMat);
      tie.rotation.x = Math.PI / 2; tie.position.set(sx, sy + 0.32, sz);
      mill.add(sack, tie);
    });
    [[2.15, -3.55], [2.85, -3.15]].forEach(([bx, bz]) => {
      const bar = new THREE.Mesh(new THREE.CylinderGeometry(0.34, 0.38, 0.86, 10), warmWoodMat);
      bar.position.set(bx, 0.43, bz); bar.castShadow = true;
      [0.24, 0.62].forEach((hy) => {
        const hp = new THREE.Mesh(new THREE.TorusGeometry(0.36, 0.025, 6, 12), darkTrimMat);
        hp.rotation.x = Math.PI / 2; hp.position.set(bx, hy, bz); mill.add(hp);
      });
      mill.add(bar);
    });

    this.scene.add(mill);
    this.addCylCollider(-10, 92, 3.9, 0, 13.8);

    // =========================================================================
    // 9. BUILDING 13 (-24.0, 0, -56.0): LAKESIDE BOATHOUSE & KAYAK RENTAL CABIN (Overlooking Twin-Basin Lake & Strait Bridge!)
    // =========================================================================
    const bh = new THREE.Group();
    bh.position.set(-24.0, 0, -56.0);
    bh.rotation.y = -Math.PI * 0.5; // Faces cleanly West toward Lakeside Trail, Marina Pier & Lake!
    const bhWallMat = new THREE.MeshStandardMaterial({ color: 0x7c5334, roughness: 0.9, side: THREE.DoubleSide });
    const bhDeck = new THREE.Mesh(new THREE.BoxGeometry(5.6, 0.35, 6.0), darkWoodMat);
    bhDeck.position.y = 0.38;
    [[-2.5, -2.7], [2.5, -2.7], [-2.5, 2.7], [2.5, 2.7]].forEach(([px, pz]) => {
      const p = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.16, 0.6, 8), darkWoodMat);
      p.position.set(px, 0.25, pz); bh.add(p);
    });
    const bhBody = new THREE.Mesh(new THREE.BoxGeometry(4.8, 2.75, 4.2), bhWallMat);
    bhBody.position.set(0, 1.92, -0.55); bhBody.castShadow = true; bhBody.receiveShadow = true;
    addFlushGable(bh, 4.8, 4.2, 3.3, 1.95, '#1e3a5f', '#142640', bhWallMat, -0.55, 1.55, 0.45);
    [-2.35, 2.35].forEach((px) => {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.18, 2.75, 0.18), darkWoodMat);
      post.position.set(px, 1.92, 2.78); bh.add(post);
    });
    addGroundedFence(bh, -2.35, 2.78, -0.65, 2.78, 0.55, 0.75, warmWoodMat);
    addEntryDoor(bh, 0, 1.5, 1.56, 1.0, 1.9, warmWoodMat, whiteTrimMat, 0);
    addFramedWin(bh, 1.55, 1.9, 1.56, 1.05, 0.95, 0);
    // Horizontal Timber Kayak Rack holding 2 Colorful Kayaks (Yellow & Red) beside Boathouse!
    [-1.1, 1.1].forEach((rx) => {
      const rk = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.55, 0.85), darkWoodMat);
      rk.position.set(3.55 + rx * 0.0, 0.78, -0.8 + rx); bh.add(rk);
    });
    [[0.65, yellowMat], [1.22, redMat]].forEach(([ky, km]) => {
      const kayak = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.24, 3.3, 8), km);
      kayak.rotation.x = Math.PI / 2;
      kayak.scale.set(1.25, 1.0, 0.52);
      kayak.position.set(3.55, ky, -0.8);
      kayak.castShadow = true;
      bh.add(kayak);
    });
    // Wall-mounted Red Lifebuoy + Lakeside Wooden Picnic Table
    const bhRing = new THREE.Mesh(new THREE.TorusGeometry(0.32, 0.08, 6, 14), redMat);
    bhRing.position.set(-1.55, 2.0, 1.62);
    const picTop = new THREE.Mesh(new THREE.BoxGeometry(1.9, 0.1, 1.05), warmWoodMat);
    picTop.position.set(-3.9, 0.75, 1.2);
    const picSeatL = new THREE.Mesh(new THREE.BoxGeometry(1.9, 0.08, 0.35), warmWoodMat);
    picSeatL.position.set(-3.9, 0.44, 1.98);
    const picSeatR = picSeatL.clone(); picSeatR.position.z = 0.42;
    bh.add(bhDeck, bhBody, bhRing, picTop, picSeatL, picSeatR);
    this.scene.add(bh);
    this.addBoxCollider(-24.0, 2.7, -56.0, 6.2, 5.4, 6.2);

    // =========================================================================
    // 10. BUILDING 14 (135, 0, 124): UPGRADED GREEN ENERGY SCADA GRID CONTROL STATION (Rooftop PV, Standby Gen & Telemetry!)
    // =========================================================================
    const gridH = new THREE.Group();
    gridH.position.set(135, 0, 124);
    const makeGridWallTex = () => {
      const c = document.createElement('canvas');
      c.width = 128; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#dde4ec'; ctx.fillRect(0, 0, 128, 128);
      ctx.fillStyle = 'rgba(71, 85, 105, 0.22)';
      for (let y = 0; y < 128; y += 32) ctx.fillRect(0, y, 128, 2);
      for (let x = 0; x < 128; x += 64) ctx.fillRect(x, 0, 2, 128);
      const tex = new THREE.CanvasTexture(c);
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(3, 1);
      if (THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    };
    const gridWallMat = new THREE.MeshStandardMaterial({ map: makeGridWallTex(), roughness: 0.72, side: THREE.DoubleSide });
    const gridEmeraldMat = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.45 });
    const gridSolarMat = new THREE.MeshStandardMaterial({ color: 0x133880, roughness: 0.16, metalness: 0.82, side: THREE.DoubleSide });
    const gridAmberMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.45 });
    const gridBeaconMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    const gridCyanMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });

    // Stepped concrete compound pad with crisp yellow safety border stripes & entrance landing steps
    const gridPad = new THREE.Mesh(new THREE.BoxGeometry(11.6, 0.25, 9.2), stoneMat);
    gridPad.position.y = 0.125; gridPad.receiveShadow = true;
    const gridCurbN = new THREE.Mesh(new THREE.BoxGeometry(11.6, 0.06, 0.18), yellowMat);
    gridCurbN.position.set(0, 0.26, -4.52);
    const gridCurbS = gridCurbN.clone(); gridCurbS.position.z = 4.52;
    const gridBody = new THREE.Mesh(new THREE.BoxGeometry(7.4, 3.45, 5.6), gridWallMat);
    gridBody.position.set(-0.8, 1.98, 0.1); gridBody.castShadow = true; gridBody.receiveShadow = true;
    // Deep architectural roof parapet + bold Navy & Emerald clean-energy branding stripe!
    const gridRoof = new THREE.Mesh(new THREE.BoxGeometry(7.9, 0.34, 6.1), darkTrimMat);
    gridRoof.position.set(-0.8, 3.84, 0.1);
    const gridParapet = new THREE.Mesh(new THREE.BoxGeometry(7.5, 0.22, 5.7), stoneMat);
    gridParapet.position.set(-0.8, 4.02, 0.1);
    const gridBand = new THREE.Mesh(new THREE.BoxGeometry(7.5, 0.28, 5.7), blueMat);
    gridBand.position.set(-0.8, 3.48, 0.1);
    const gridSubBand = new THREE.Mesh(new THREE.BoxGeometry(7.52, 0.12, 5.72), gridEmeraldMat);
    gridSubBand.position.set(-0.8, 3.24, 0.1);

    // Cantilevered Steel Entrance Canopy + Illuminated "GRID CONTROL · SCADA" Sign + Steps + Entry Lamp!
    addEntryDoor(gridH, -2.8, 1.32, -2.72, 1.15, 2.05, darkTrimMat, whiteTrimMat, Math.PI);
    const gCanopy = new THREE.Mesh(new THREE.BoxGeometry(2.3, 0.16, 1.25), darkTrimMat);
    gCanopy.position.set(-2.8, 2.68, -3.28); gCanopy.castShadow = true;
    const gSign = new THREE.Mesh(
      new THREE.BoxGeometry(3.2, 0.72, 0.12),
      new THREE.MeshStandardMaterial({ map: makeSignTex('GRID CONTROL', 'SCADA TELEMETRY HUB', '#0f172a', '#10b981'), roughness: 0.45 })
    );
    gSign.position.set(0.5, 3.05, -2.78);
    gSign.rotation.y = Math.PI;
    const gLamp = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.08, 0.18), lampGlowMat);
    gLamp.position.set(-2.8, 2.56, -3.25);
    const gStatusBar = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.55, 0.08), gridCyanMat);
    gStatusBar.position.set(-1.95, 1.85, -2.76);
    // Large panoramic SCADA Control-Room Windows (North, East & West)
    addFramedWin(gridH, 0.65, 1.9, -2.72, 2.85, 1.2, Math.PI);
    addFramedWin(gridH, 2.92, 1.9, 0.1, 2.35, 1.2, Math.PI / 2);
    addFramedWin(gridH, -4.52, 1.9, 0.1, 1.9, 1.1, -Math.PI / 2);

    // Rooftop Clean-Energy Gear: 3 Tilted Self-Powering PV Solar Panels + Dual HVAC Chillers + Parabolic Dish + Lattice Comms Mast!
    [-2.5, -0.8, 0.9].forEach((sx) => {
      const rLeg = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.35, 0.12), darkTrimMat);
      rLeg.position.set(sx, 4.25, 1.55);
      const rPv = new THREE.Mesh(new THREE.BoxGeometry(1.45, 0.07, 1.15), gridSolarMat);
      rPv.position.set(sx, 4.46, 1.55); rPv.rotation.x = 0.36; rPv.castShadow = true;
      gridH.add(rLeg, rPv);
    });
    // Twin Rooftop HVAC Cooling Chillers with circular exhaust fan shrouds
    [-1.4, -0.15].forEach((hx) => {
      const hvac = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.68, 0.95), whiteTrimMat);
      hvac.position.set(hx, 4.45, -1.15); hvac.castShadow = true;
      const fan = new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.34, 0.1, 10), darkTrimMat);
      fan.position.set(hx, 4.82, -1.15);
      gridH.add(hvac, fan);
    });
    // Upgraded Parabolic Microwave Telemetry Dish with Feed Horn facing Solar Plant
    const satPost = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.18, 1.15, 8), darkTrimMat);
    satPost.position.set(1.75, 4.65, -1.05);
    const satDish = new THREE.Mesh(new THREE.SphereGeometry(1.05, 16, 10, 0, Math.PI * 2, 0, Math.PI * 0.45), whiteTrimMat);
    satDish.position.set(1.75, 5.42, -1.05); satDish.rotation.set(2.35, -0.4, 0);
    const satFeed = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.72, 6), darkTrimMat);
    satFeed.position.set(1.92, 5.55, -0.82); satFeed.rotation.z = -0.6;
    // Tall Lattice VHF / Weather Telemetry Mast with Dipole Arms & Red Warning Beacon
    const gMast = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.11, 4.2, 8), whiteTrimMat);
    gMast.position.set(-3.45, 6.15, -1.35);
    const gDipole1 = new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.05, 0.05), darkTrimMat);
    gDipole1.position.set(-3.45, 7.65, -1.35);
    const gDipole2 = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.85), darkTrimMat);
    gDipole2.position.set(-3.45, 7.15, -1.35);
    const gBeacon = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 8), gridBeaconMat);
    gBeacon.position.set(-3.45, 8.32, -1.35);

    // Exterior Compound Equipment: Standby Industrial Backup Generator + Switchgear Metering Rack + Dual Cable Reels + Bollards!
    const genBody = new THREE.Mesh(new THREE.BoxGeometry(2.05, 1.55, 1.35), gridAmberMat);
    genBody.position.set(4.15, 1.02, -1.35); genBody.castShadow = true;
    const genRoof = new THREE.Mesh(new THREE.BoxGeometry(2.15, 0.14, 1.45), darkTrimMat);
    genRoof.position.set(4.15, 1.84, -1.35);
    const genVent = new THREE.Mesh(new THREE.BoxGeometry(1.45, 0.75, 1.4), darkTrimMat);
    genVent.position.set(4.15, 1.08, -1.35);
    const genExh = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 1.25, 8), darkTrimMat);
    genExh.position.set(4.85, 2.48, -1.35);
    // Switchgear Metering Cabinet + Twin Heavy Wooden HV Cable Reels
    const swCab = new THREE.Mesh(new THREE.BoxGeometry(0.85, 1.65, 1.55), whiteTrimMat);
    swCab.position.set(4.15, 1.08, 0.85); swCab.castShadow = true;
    const swTrim = new THREE.Mesh(new THREE.BoxGeometry(0.88, 0.2, 1.58), gridEmeraldMat);
    swTrim.position.set(4.15, 1.72, 0.85);
    [2.55, 3.45].forEach((rz, idx) => {
      const reel = new THREE.Mesh(new THREE.CylinderGeometry(0.68, 0.68, 0.56, 12), warmWoodMat);
      reel.rotation.z = Math.PI / 2; reel.position.set(4.15 + idx * 0.15, 0.92, rz);
      const cableWrap = new THREE.Mesh(new THREE.CylinderGeometry(0.52, 0.52, 0.44, 12), darkTrimMat);
      cableWrap.rotation.z = Math.PI / 2; cableWrap.position.copy(reel.position);
      gridH.add(reel, cableWrap);
    });
    // Yellow Safety Bollards guarding generator & building corners + Compound Floodlight Pole
    [[-4.2, -3.8], [-1.3, -3.8], [3.0, -3.4], [5.2, -3.4]].forEach(([bx, bz]) => {
      const b = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 0.88, 8), yellowMat);
      b.position.set(bx, 0.68, bz); gridH.add(b);
    });
    const gLightPole = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.09, 5.4, 8), darkTrimMat);
    gLightPole.position.set(-5.35, 2.7, -3.95);
    const gLightHead = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.16, 0.35), lampGlowMat);
    gLightHead.position.set(-5.15, 5.38, -3.95);

    gridH.add(
      gridPad, gridCurbN, gridCurbS, gridBody, gridRoof, gridParapet, gridBand, gridSubBand,
      gCanopy, gSign, gLamp, gStatusBar, satPost, satDish, satFeed, gMast, gDipole1, gDipole2, gBeacon,
      genBody, genRoof, genVent, genExh, swCab, swTrim, gLightPole, gLightHead
    );
    this.scene.add(gridH);
    this.addBoxCollider(134.2, 2.2, 124.1, 8.6, 4.4, 6.4);

    // =========================================================================
    // 11. BUILDING 15 (82, 0, 6.4): AIRFIELD–VILLAGE TRANSIT SHELTER & BICYCLE RACK (Spaciously North of Main Road!)
    // =========================================================================
    const sh = new THREE.Group();
    sh.position.set(82, 0, 6.4);
    sh.rotation.y = Math.PI; // Open front & bench face South (+Z) cleanly toward North Main Road (z = 12)!
    const shPad = new THREE.Mesh(new THREE.BoxGeometry(4.8, 0.16, 2.6), stoneMat);
    shPad.position.y = 0.08;
    [[-1.8, -0.9], [1.8, -0.9], [-1.8, 0.9], [1.8, 0.9]].forEach(([px, pz]) => {
      const p = new THREE.Mesh(new THREE.BoxGeometry(0.14, 2.45, 0.14), darkWoodMat);
      p.position.set(px, 1.38, pz); sh.add(p);
    });
    const shRoof = new THREE.Mesh(new THREE.BoxGeometry(4.4, 0.18, 2.5), darkTrimMat);
    shRoof.position.set(0, 2.65, 0); shRoof.rotation.x = 0.12; shRoof.castShadow = true;
    const shBackGlass = new THREE.Mesh(new THREE.BoxGeometry(3.55, 1.75, 0.08), clearGlassMat);
    shBackGlass.position.set(0, 1.38, 0.9);
    const shBench = new THREE.Mesh(new THREE.BoxGeometry(2.8, 0.1, 0.52), warmWoodMat);
    shBench.position.set(0, 0.58, 0.45);
    const shLamp = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.08, 0.25), lampGlowMat);
    shLamp.position.set(0, 2.52, 0);
    // Steel Bicycle Rack with 2 Parked Bicycles beside the Shelter!
    const rackBar = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.06, 0.5), darkTrimMat);
    rackBar.position.set(3.45, 0.12, 0);
    [3.05, 3.85].forEach((bx, bi) => {
      const bCol = (bi === 0) ? redMat : blueMat;
      const wF = new THREE.Mesh(new THREE.TorusGeometry(0.32, 0.04, 6, 12), darkTrimMat);
      wF.rotation.y = Math.PI / 2; wF.position.set(bx, 0.36, -0.5);
      const wB = wF.clone(); wB.position.z = 0.5;
      const frame = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.36, 0.98), bCol);
      frame.position.set(bx, 0.62, 0);
      const handle = new THREE.Mesh(new THREE.BoxGeometry(0.45, 0.05, 0.05), darkTrimMat);
      handle.position.set(bx, 0.96, -0.45);
      sh.add(wF, wB, frame, handle);
    });
    sh.add(shPad, shRoof, shBackGlass, shBench, shLamp, rackBar);
    this.scene.add(sh);
    this.addBoxCollider(82, 1.4, 6.4, 4.6, 2.8, 2.6);
  }

  // Derive real physical quadrotor parameters directly from all 12 assembled parts
  computeDronePhysicsParams() {
    const stats = this.stats || {};
    const rp = stats.resolvedParts || {};
    const frame = rp.frame || {};
    const motor = rp.motor || {};
    const prop = rp.propeller || {};
    const esc = rp.esc || {};
    const battery = rp.battery || {};
    const fc = rp.fc || {};
    const camera = rp.camera || {};
    const gps = rp.gps || {};
    const rx = rp.rx || {};
    const sensor = rp.sensor || {};
    const payload = rp.payload || {};
    const safety = rp.safety || {};

    const gravity = 9.80665;
    const totalMassKg = Math.max(0.28, (stats.totalWeight || 1126) / 1000);
    const payloadTankKg = Math.max(0, payload.tankKg || 0);
    const dryMass = Math.max(0.26, totalMassKg - payloadTankKg);
    const mass = dryMass + payloadTankKg; // full takeoff mass
    const rawThrustN = ((stats.totalThrust || 3520) / 1000) * gravity;
    // Authentic thrust physics: NO fake 1.12x lift cheat! Overloaded builds (T/W <= 1.0) genuinely cannot climb!
    const maxThrustN = Math.max(0.5, rawThrustN);
    const twRatio = maxThrustN / (mass * gravity);

    const wheelbase = Math.max(0.20, ((frame.wheelbase || 450) / 1000)); // 0.25, 0.45, 0.65, 0.80 m
    const armLength = wheelbase * 0.5;
    const armXY = armLength * Math.SQRT1_2;
    const refMassKg = wheelbase <= 0.30 ? 0.58 : wheelbase <= 0.50 ? 1.05 : wheelbase <= 0.70 ? 1.95 : 3.20;

    const propSizeIn = prop.size || 10;
    const propPitchIn = prop.pitch || 4.5;
    const propDiameter = propSizeIn * 0.0254;
    const propRadius = propDiameter * 0.5;
    const diskArea = 4 * Math.PI * propRadius * propRadius;
    const hoverInducedVel = Math.sqrt((mass * gravity) / (2 * 1.225 * Math.max(0.02, diskArea)));

    const motorKv = motor.kv || 920;
    const cells = battery.cells || 4;
    const capacityAh = Math.max(0.8, ((battery.capacity || 3300) / 1000));
    const nomVoltage = cells * 3.7;
    const cRating = Math.max(12, battery.cRating || 45);
    // Real C-Rating / Chemistry internal resistance (15C Li-Ion sags more under heavy climbs than 45C LiPo!)
    const chemResMult = Math.min(3.2, 45 / cRating);
    const rInternal = cells * (0.018 * (2.2 / Math.max(1.0, capacityAh))) * chemResMult;
    const estHoverMinutes = Math.max(0.8, stats.estHoverMinutes || 6.2);
    const escTotalAmps = (esc.rating || 30) * 4;
    const hoverAmpsEst = (capacityAh * 0.85 * 60) / estHoverMinutes;
    const maxBurstAmps = Math.max(hoverAmpsEst * 3.2, escTotalAmps * 0.95);

    let escThrottleLimit = 1.0;
    if (esc.rating && motor.maxAmpsPerMotor && esc.rating < motor.maxAmpsPerMotor) {
      escThrottleLimit = Math.max(0.76, Math.sqrt(esc.rating / motor.maxAmpsPerMotor));
    }

    // Real 3-axis moments of inertia (kg*m^2): separate dry chassis + belly payload (parallel-axis theorem I = I_cm + m*d^2)
    const IxxDry = dryMass * (0.042 * wheelbase * wheelbase + 0.34 * armXY * armXY);
    const IzzDry = IxxDry * 0.96;
    const IyyDry = IxxDry * 1.75;
    const Ixx = IxxDry + payloadTankKg * (0.008 + 0.040);
    const Izz = IzzDry + payloadTankKg * (0.008 + 0.040);
    const Iyy = IyyDry + payloadTankKg * 0.012;

    // Brushless motor + propeller first-order spin-up time constant (s)
    const tauMotor = Math.max(0.026, Math.min(0.092,
      0.018 + (propSizeIn / 14) * 0.048 * (1000 / Math.max(460, motorKv))
    ));

    const kYawArm = propDiameter * (0.12 + (propPitchIn / 5.5) * 0.05);

    // Aerodynamic drag area (Cd*A) + prop guards + belly tank + camera drag!
    const cameraType = camera.type || 'none';
    const hasGimbal = cameraType === 'gimbal_4k' || cameraType === 'ndvi' || cameraType === 'thermal';
    const camDragBonus = hasGimbal ? 0.008 : cameraType === 'fpv' ? 0.003 : 0;
    const payloadDragBonus = payloadTankKg > 0 ? 0.014 : 0;
    const guardDragMult = safety.hasGuards ? 1.12 : 1.0;
    const cdAHoriz = (0.16 * wheelbase * wheelbase + 0.006 + camDragBonus + payloadDragBonus) * guardDragMult;
    const cdAVert = cdAHoriz * 1.95;
    const kRotorDrag = 0.14 * 1.225 * diskArea * hoverInducedVel;

    // Frame-authentic agility baseline (further modulated live every sub-step by real curMass & curTw!)
    const isSmall = wheelbase <= 0.30;
    const isAgri = wheelbase >= 0.74;
    const isLarge = wheelbase >= 0.56;
    const maxTiltRad = isSmall ? 0.68 : isAgri ? 0.38 : isLarge ? 0.44 : 0.56;
    const maxYawRate = isSmall ? 3.2 : isAgri ? 1.45 : isLarge ? 1.65 : 2.25;
    const omegaAtt = isSmall ? 10.2 : isAgri ? 5.6 : isLarge ? 6.4 : 8.0;
    const zetaAtt = 1.04; // Slightly over-damped: crisp zero-overshoot, zero bounce-back attitude control!
    const kYaw = isSmall ? 8.0 : isAgri ? 5.0 : isLarge ? 5.6 : 6.6;

    const maxClimbRate = twRatio > 1.02 ? Math.min(13.0, Math.max(0.32, (twRatio - 1.0) * 2.75)) : 0.12;
    const maxDescentRate = Math.min(6.4, Math.max(2.6, hoverInducedVel * 1.15));
    // Real-World Drone Class Operational Altitude Ceiling (FPV 50m | Aerial 90m | Heavy Survey 120m | Agri Sprayer 30m!)
    const maxAltitudeLimitM = frame.maxAltitudeLimitM
      || (isAgri ? 30 : isSmall ? 50 : isLarge ? 120 : 90);
    const frameClassLabel = frame.frameClassLabel
      || (isAgri ? 'โดรนเกษตร (800 มม.)' : isSmall ? 'โดรน FPV (250 มม.)' : isLarge ? 'โดรนสำรวจหนัก (650 มม.)' : 'โดรนถ่ายภาพ (450 มม.)');
    const altitudeLimitReason = frame.altitudeLimitReason || `จำกัดเพดานบินสูงสุด ${maxAltitudeLimitM} ม.`;

    // New subsystem capabilities
    const clearance = 0.48 + (safety.clearanceExtra || 0);
    const vertCrashSpeed = safety.gearType === 'tall' ? 9.6 : 7.5;
    const sideCrashSpeed = safety.hasGuards ? 22.0 : 13.5;

    this.liveTwRatio = twRatio;
    this.liveTwOge = twRatio;
    this.autoTakeoffTimer = 0;

    this.physicsParams = {
      gravity, mass, dryMass, payloadTankKg, refMassKg, maxThrustN, twRatio,
      wheelbase, armLength, armXY, kYawArm,
      propSizeIn, propPitchIn, propDiameter, propRadius, diskArea, hoverInducedVel,
      motorKv, cells, capacityAh, nomVoltage, rInternal, estHoverMinutes, maxBurstAmps, escThrottleLimit,
      IxxDry, IyyDry, IzzDry, Ixx, Iyy, Izz, tauMotor,
      cdAHoriz, cdAVert, kRotorDrag,
      maxTiltRad, maxYawRate, omegaAtt, zetaAtt, kYaw,
      maxClimbRate, maxDescentRate,
      maxAltitudeLimitM, frameClassLabel, altitudeLimitReason,
      hasBaro: Boolean(fc.hasBaro),
      dualImu: Boolean(fc.dualImu),
      gustRejectFactor: fc.dualImu ? 0.45 : 1.0,
      hasGps: Boolean(gps.hasGps),
      isRtk: Boolean(gps.isRtk),
      cameraType, hasGimbal,
      rxType: rx.rxType || 'elrs',
      rxRangeMeters: rx.rangeMeters || 340,
      rxFailsafeRth: Boolean(rx.failsafeRth),
      hasOpticalFlow: Boolean(sensor.hasOpticalFlow),
      hasLidar: Boolean(sensor.hasLidar),
      payloadType: payload.payloadType || 'none',
      hasGuards: Boolean(safety.hasGuards),
      hasParachute: Boolean(safety.hasParachute),
      clearance, vertCrashSpeed, sideCrashSpeed
    };
    return this.physicsParams;
  }

  // Realistic nonlinear LiPo per-cell Open-Circuit Voltage (OCV) vs State of Charge [0..1]
  getLipoOpenCircuitVoltage(soc) {
    const s = Math.max(0, Math.min(1, soc));
    if (s >= 0.85) return 4.00 + ((s - 0.85) / 0.15) * 0.20;
    if (s >= 0.50) return 3.82 + ((s - 0.50) / 0.35) * 0.18;
    if (s >= 0.20) return 3.69 + ((s - 0.20) / 0.30) * 0.13;
    if (s >= 0.05) return 3.42 + ((s - 0.05) / 0.15) * 0.27;
    return 3.20 + (s / 0.05) * 0.22;
  }

  // Wind Preset Profiles (KeyN / HUD Wind Switch) — live atmospheric wind tuning for teaching & demo!
  getWindPresetInfo() {
    const missionSpd = this.mission ? Math.max(0.35, this.mission.windSpeed) : 1.0;
    const preset = this.windPreset || 'mission';
    if (preset === 'calm') {
      return { id: 'calm', baseSpeed: 0.6, gustScale: 0.16, squallAmp: 0.08, label: '🍃 ลมสงบ (0.6 ม./วิ)', shortLabel: '🍃 ลมสงบ (N)' };
    }
    if (preset === 'moderate') {
      return { id: 'moderate', baseSpeed: 4.5, gustScale: 0.28, squallAmp: 0.22, label: '🌬️ ลมปานกลาง (4.5 ม./วิ)', shortLabel: '🌬️ ลมกลาง (N)' };
    }
    if (preset === 'strong') {
      return { id: 'strong', baseSpeed: 8.5, gustScale: 0.34, squallAmp: 0.30, label: '💨 ลมแรง (8.5 ม./วิ)', shortLabel: '💨 ลมแรง (N)' };
    }
    if (preset === 'storm') {
      return { id: 'storm', baseSpeed: 12.5, gustScale: 0.40, squallAmp: 0.36, label: '🌪️ พายุลมกระโชก (12.5 ม./วิ)', shortLabel: '🌪️ พายุลม (N)' };
    }
    return {
      id: 'mission',
      baseSpeed: missionSpd,
      gustScale: 0.24,
      squallAmp: missionSpd >= 3.5 ? 0.24 : 0.12,
      label: `🍃 ลมตามฉาก (${missionSpd.toFixed(1)} ม./วิ)`,
      shortLabel: `🌬️ ลม ${missionSpd.toFixed(1)} (N)`
    };
  }

  cycleWindPreset() {
    this.initAudio();
    const order = ['mission', 'moderate', 'strong', 'storm', 'calm'];
    const idx = order.indexOf(this.windPreset || 'mission');
    this.windPreset = order[(idx + 1) % order.length];
    const info = this.getWindPresetInfo();
    this.playTone(this.windPreset === 'storm' ? 420 : this.windPreset === 'strong' ? 540 : 680, 0.11);
    const tip = this.windPreset === 'storm'
      ? ' — สังเกตโดรนเอียงสู้ลม กระแสไฟพุ่ง และถุงลม/กังหันหมุนเร็ว!'
      : this.windPreset === 'strong'
        ? ' — ทดสอบต้านลมขวาง (กด H สลับ ATTI เพื่อดูโดรนปลิวตามลม!)'
        : this.windPreset === 'moderate'
          ? ' — ลมระดับปฏิบัติงานจริง สังเกตความต่างระหว่างโดรนเบาและหนัก'
          : '';
    this.setHudAlert(`ปรับสภาพลม: ${info.label}${tip}`, 3200);
    return info;
  }

  // Live 3D atmospheric wind field: logarithmic boundary-layer shear + Dryden-style rolling gust squalls + leeward wind shelter + vertical thermals!
  sampleWindField(x, y, z, t, outVec, applyShelter = false) {
    const wProf = this.getWindPresetInfo();
    const baseSpeed = wProf.baseSpeed;

    // 1. Atmospheric Boundary-Layer Wind Shear (Hellmann power-law: calmer near ground, ~1.5x stronger at 45m tower & ridges!)
    const groundY = (typeof this.getTerrainHeight === 'function') ? this.getTerrainHeight(x, z) : 0;
    const agl = Math.max(0.2, y - groundY);
    const altShear = Math.max(0.52, Math.min(1.68, 0.42 + 0.58 * Math.pow((agl + 0.8) / 14.0, 0.30)));

    // 2. Dryden-style multi-frequency turbulence + rolling gust squall pulses
    const harmonic = 0.55 * Math.sin(t * 0.62 + x * 0.018 + z * 0.014)
      + 0.30 * Math.sin(t * 1.47 - x * 0.025 + z * 0.021)
      + 0.15 * Math.cos(t * 2.85 + y * 0.045);
    const squallWave = 0.5 + 0.5 * Math.sin(t * 0.42 + x * 0.011 - z * 0.009);
    const squallPulse = Math.pow(squallWave, 4.0) * wProf.squallAmp;
    const gust = Math.max(0.55, 1.0 + harmonic * wProf.gustScale + squallPulse);

    // 3. Leeward Wind Shelter (calmer indoors inside steel hangar or tucked behind trees/buildings!)
    const shelter = applyShelter ? (this.windShelterFactor ?? 1.0) : 1.0;

    const speed = Math.max(0.04, baseSpeed * altShear * gust * shelter);

    // 4. Prevailing wind vector matches airfield windsock (~29.7° -> blows ENE/SE) + organic ±14° wind veer
    const baseAngle = Math.atan2(0.4, 0.7);
    const dir = baseAngle
      + 0.16 * Math.sin(t * 0.31 + z * 0.012)
      + 0.07 * Math.cos(t * 0.83 - x * 0.015);

    // 5. Vertical air currents (thermals & ridge updrafts/downdrafts, suppressed indoors)
    const wy = baseSpeed * (0.072 + 0.04 * squallPulse) * altShear * shelter
      * (Math.sin(t * 1.15 + x * 0.03) + 0.5 * Math.cos(t * 2.2 - z * 0.03));

    outVec.set(Math.cos(dir) * speed, wy, Math.sin(dir) * speed);
    if (applyShelter) {
      this.windGustFactor = gust;
      this.windGustActive = baseSpeed >= 3.2 && gust >= 1.24;
      this.liveWindDirRad = dir;
      // Compass heading wind is blowing TOWARD (0°=N/-Z, 90°=E/+X, 180°=S/+Z, 270°=W/-X)
      const heading = (Math.atan2(outVec.x, -outVec.z) * (180 / Math.PI) + 360) % 360;
      this.liveWindDirDeg = Math.round(heading);
      this.liveWindVertical = wy;
    }
    return outVec;
  }

  // Lightweight 1-draw-call 3D Visual Wind Airflow Streaks around the drone (shows live wind direction & gust speed!)
  initWindStreaks() {
    if (this.windStreaks || !this.scene || typeof THREE === 'undefined') return;
    const count = 44;
    const positions = new Float32Array(count * 2 * 3);
    this.windStreakData = [];
    for (let i = 0; i < count; i++) {
      this.windStreakData.push({
        x: (Math.random() - 0.5) * 28,
        y: (Math.random() - 0.35) * 14,
        z: (Math.random() - 0.5) * 28,
        lenScale: 0.75 + Math.random() * 0.65,
        spdScale: 0.85 + Math.random() * 0.35
      });
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const mat = new THREE.LineBasicMaterial({
      color: 0xe0f2fe,
      transparent: true,
      opacity: 0.0,
      depthWrite: false
    });
    this.windStreaks = new THREE.LineSegments(geo, mat);
    this.windStreaks.frustumCulled = false;
    this.windStreaks.visible = false;
    this.scene.add(this.windStreaks);
  }

  updateWindStreaks(delta) {
    if (!this.windStreaks) this.initWindStreaks();
    if (!this.windStreaks) return;
    const spd = this.liveWindSpeed || 0;
    if (spd < 1.8 || this.graphicsQuality === 'fast' && spd < 4.5) {
      this.windStreaks.visible = false;
      return;
    }
    this.windStreaks.visible = true;
    const gustBoost = this.windGustActive ? 0.14 : 0;
    this.windStreaks.material.opacity = Math.min(0.56, Math.max(0.08, (spd - 1.5) / 14.0) + gustBoost);
    this.windStreaks.material.color.setHex(spd >= 10.5 ? 0xfecaca : spd >= 7.5 ? 0xfde68a : 0xe0f2fe);

    const wx = this.liveWindVec.x;
    const wy = this.liveWindVec.y;
    const wz = this.liveWindVec.z;
    const invSpd = 1 / Math.max(0.1, Math.hypot(wx, wy, wz));
    const dx = wx * invSpd;
    const dy = wy * invSpd;
    const dz = wz * invSpd;
    const baseLen = Math.min(3.8, 0.55 + spd * 0.24);
    const px = this.position.x;
    const py = this.position.y;
    const pz = this.position.z;

    const posAttr = this.windStreaks.geometry.attributes.position;
    const arr = posAttr.array;
    const halfSpan = 15;
    const halfY = 7.5;

    for (let i = 0; i < this.windStreakData.length; i++) {
      const s = this.windStreakData[i];
      const adv = (spd * 1.15 * s.spdScale) * delta;
      s.x += dx * adv;
      s.y += dy * adv;
      s.z += dz * adv;

      // Wrap seamlessly inside moving volume centered around drone
      if (s.x > halfSpan) s.x -= halfSpan * 2;
      else if (s.x < -halfSpan) s.x += halfSpan * 2;
      if (s.z > halfSpan) s.z -= halfSpan * 2;
      else if (s.z < -halfSpan) s.z += halfSpan * 2;
      if (s.y > halfY) s.y -= halfY * 2;
      else if (s.y < -halfY) s.y += halfY * 2;

      const x0 = px + s.x;
      const y0 = Math.max(this.getTerrainHeight(x0, pz + s.z) + 0.4, py + s.y);
      const z0 = pz + s.z;
      const len = baseLen * s.lenScale;

      const idx = i * 6;
      arr[idx] = x0;
      arr[idx + 1] = y0;
      arr[idx + 2] = z0;
      arr[idx + 3] = x0 - dx * len;
      arr[idx + 4] = y0 - dy * len;
      arr[idx + 5] = z0 - dz * len;
    }
    posAttr.needsUpdate = true;
  }

  setHudAlert(msg, durationMs = 2600) {
    this.hudAlertText = msg;
    this.hudAlertUntil = performance.now() + durationMs;
  }

  // Load / Setup current drone model (with custom paint livery!)
  setDroneAssembly(assembly, stats, paintConfig = null) {
    this.stats = stats;
    this.paintConfig = paintConfig || this.paintConfig || null;
    this.computeDronePhysicsParams();
    if (this.droneMesh) {
      this.scene.remove(this.droneMesh);
    }
    this.droneBuilder = new DroneModelBuilder();
    this.droneMesh = this.droneBuilder.buildDrone(assembly, THREE, this.paintConfig);
    // Bake all static drone sub-meshes by material (~220 -> ~14 draw calls, 15x faster, 100% identical look!)
    if (typeof this.droneBuilder.bakeForFlight === 'function') {
      this.droneBuilder.bakeForFlight(THREE);
    }
    this.scene.add(this.droneMesh);
    this.resetDronePosition();
  }

  // Setup Mission
  setMission(mission) {
    this.mission = mission;
    this.windPreset = 'mission';
    this.clearCheckpoints();

    if (mission.checkpoints && mission.checkpoints.length > 0) {
      mission.checkpoints.forEach((cp, idx) => {
        const ringGeo = new THREE.TorusGeometry(cp.radius || 4, 0.35, 12, 24);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
        const ringMesh = new THREE.Mesh(ringGeo, ringMat);
        ringMesh.position.set(cp.x, cp.y, cp.z);
        ringMesh.rotation.y = Math.PI / 2;
        this.scene.add(ringMesh);

        this.checkpoints.push({
          mesh: ringMesh,
          pos: new THREE.Vector3(cp.x, cp.y, cp.z),
          radius: cp.radius || 4,
          name: cp.name,
          passed: false,
          index: idx
        });
      });
    }

    this.completedCheckpoints = 0;
    this.resetDronePosition();
  }

  clearCheckpoints() {
    this.checkpoints.forEach(cp => {
      this.scene.remove(cp.mesh);
    });
    this.checkpoints = [];
  }

  resetDronePosition() {
    const p = this.physicsParams || this.computeDronePhysicsParams();
    const startY = this.getTerrainHeight(0, 0) + (p.clearance || 0.48);
    this.position.set(0, startY, 0);
    this.velocity.set(0, 0, 0);
    this.rotation.set(0, 0, 0);
    this.angularVelocity.set(0, 0, 0);
    this.isArmed = false;
    this.isLanded = true;
    this.isCrashed = false;
    // Commercial GPS drones default to GPS Position Hold (P-Mode); no-GPS FPV builds fly manual ATTI!
    this.isHoldMode = Boolean(p && p.hasGps);
    this.isRTHMode = false;
    this.rthPhase = 'IDLE';
    this.rthCruiseAlt = startY + 22;
    this.flightPathHistory = [{ x: 0, y: startY, z: 0 }];
    this.rthWaypoints = [];
    this.rthWaypointIdx = 0;
    this.rthAlgoLabel = '';
    this.rthTotalPathDist = 0;
    this.isFailsafeReturn = false;
    this.sensorLostSim = false;
    this.signalLossTimer = 0;
    if (typeof this.clearRthPathVisuals === 'function') this.clearRthPathVisuals();
    this.holdAltitude = startY;
    this.altHoldLocked = true;
    this.holdTargetPos.set(0, startY, 0);
    this.holdTargetLocked = true;
    this.windEst.set(0, 0);
    this.motorRpm = [0, 0, 0, 0];
    this.input = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
    this.inputVel = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
    this.smoothTargetPitch = 0;
    this.smoothTargetRoll = 0;
    this.smoothTargetYawRate = 0;
    this.smoothTargetVz = 0;
    this.brakeRamp = 0;
    this.vrsIntensity = 0;
    this.igeFactor = 1.0;
    this.airDensity = 1.225;
    this.surfaceSupportY = startY;
    this.batteryVoltage = (p ? p.cells : 4) * 4.2;
    this.currentAmps = 0;
    this.payloadPercent = 100;
    this.isSpraying = false;
    this.cargoAttached = true;
    this.parachuteDeployed = false;
    this.rssiPercent = 100;
    this.smoothedRssi = 100;
    this.signalState = 'good';
    this.signalSource = 'clear';
    this.signalObstruction = 0;
    this.gpsMultipathFactor = 1.0;
    this.insideMetalRoof = false;
    this._lastSignalState = 'good';
    this._lastSignalBeepMs = 0;
    this.lidarBraking = false;
    this.lidarZone = 'off';
    this.lidarNearestDist = null;
    this.lidarNearestDirTh = '';
    this.lidarNearestTag = '';
    this.groundClearanceM = 0;
    this.homeAltitudeM = 0;
    this.surfaceBelowY = startY - (p ? (p.clearance || 0.48) : 0.48);
    this.altSensorLocked = false;
    if (this.droppedCargoMesh) {
      this.droppedCargoMesh.visible = false;
      this.droppedCargoLanded = true;
    }
    this.hudAlertText = '';
    this.hudAlertUntil = 0;
    this.lawOverCeiling = false;
    this.lawAltitudeViolated = false;
    this.lawVlosWarned = false;
    this.lawVillageWarned = false;
    this._lastLawBeepMs = 0;
    this.hitAltitudeLimit = false;
    this._lastAltLimitAlertMs = 0;
    this.powerState = 'DISARMED';
    this.preArmTimer = 0;
    this.pendingAutoTakeoff = false;
    this.autoTakeoffTargetY = startY + 3.0;
    this.autoLandSettleTimer = 0;
    this.touchdownHoldTimer = 0;
    this.emergencyKillArmUntil = 0;
    this.flightTimeSeconds = 0;
    this.batteryPercent = 100;
    this.maxAltitude = 0;
    this.altitude = 0;
    this.speedKmh = 0;
    this.camYaw = 0;
    this.camPitch = 0.18;
    this.targetCamYaw = 0;
    this.targetCamPitch = 0.18;
    this.camInitialized = false;

    if (this.droneMesh) {
      this.droneMesh.position.copy(this.position);
      this.droneMesh.rotation.copy(this.rotation);
    }
  }

  // Trigger Agri Sprayer / Spreader / Cargo Release (KeyF)
  togglePayloadAction() {
    const p = this.physicsParams || this.computeDronePhysicsParams();
    if (p.payloadType === 'sprayer' || p.payloadType === 'spreader') {
      if (this.payloadPercent <= 0.5) {
        this.isSpraying = false;
        this.playTone(240, 0.15);
        this.setHudAlert('ถังปุ๋ยหมดแล้ว! ลงจอดที่แท่น AGRI-DRONE REFILL เพื่อเติม');
        return;
      }
      this.isSpraying = !this.isSpraying;
      this.playTone(this.isSpraying ? 720 : 420, 0.12);
      const actionTxt = p.payloadType === 'spreader' ? 'จานหว่านปุ๋ยเม็ด' : 'หัวฉีดพ่นปุ๋ยน้ำ';
      this.setHudAlert(this.isSpraying ? `💧 เปิด${actionTxt} (น้ำหนักลดลงเรียลไทม์)` : `⏹ ปิด${actionTxt}`);
    } else if (p.payloadType === 'cargo') {
      if (!this.cargoAttached) {
        this.playTone(260, 0.12);
        this.setHudAlert('ปลดกล่องพัสดุไปแล้ว (ลงจอดลาน H หรือกด RESET เพื่อรับกล่องใหม่)');
        return;
      }
      this.cargoAttached = false;
      this.payloadPercent = 0;
      this.spawnDroppedCargo();
      this.playTone(820, 0.18);
      const newTw = (p.maxThrustN / Math.max(0.5, p.dryMass * p.gravity)).toFixed(2);
      this.setHudAlert(`📦 ปลดกล่องเวชภัณฑ์แล้ว! เบาลง 500 ก. ทันที (T/W พุ่งเป็น ${newTw}x)`, 3200);
    } else {
      this.playTone(240, 0.12);
      this.setHudAlert('ยังไม่ได้ติดถังพ่นปุ๋ย/ตะขอพัสดุ (ติดตั้งได้ที่ช่อง 11)');
    }
  }

  initPayloadEffects() {
    if (this.sprayPoints || !this.scene) return;
    const count = 160;
    const positions = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
      positions[i * 3 + 1] = -999;
      this.sprayData.push({ active: false, life: 0, maxLife: 1, vx: 0, vy: 0, vz: 0 });
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const mat = new THREE.PointsMaterial({
      color: 0x7dd3fc,
      size: 0.24,
      transparent: true,
      opacity: 0.78,
      depthWrite: false
    });
    this.sprayPoints = new THREE.Points(geo, mat);
    this.sprayPoints.frustumCulled = false;
    this.scene.add(this.sprayPoints);
  }

  spawnDroppedCargo() {
    if (!this.scene) return;
    if (!this.droppedCargoMesh) {
      const g = new THREE.Group();
      const box = new THREE.Mesh(
        new THREE.BoxGeometry(0.36, 0.24, 0.30),
        new THREE.MeshStandardMaterial({ color: 0xff6a00, roughness: 0.4 })
      );
      box.castShadow = true;
      g.add(box);
      const b1 = new THREE.Mesh(
        new THREE.BoxGeometry(0.37, 0.05, 0.31),
        new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.5 })
      );
      g.add(b1);
      this.droppedCargoMesh = g;
      this.scene.add(g);
    }
    this.droppedCargoMesh.visible = true;
    this.droppedCargoMesh.position.set(this.position.x, this.position.y - 0.28, this.position.z);
    this.droppedCargoMesh.rotation.set(0, this.rotation.y, 0);
    this.droppedCargoVel.copy(this.velocity);
    this.droppedCargoLanded = false;
  }

  updatePayloadEffects(delta) {
    const p = this.physicsParams;
    if (!p) return;

    // 1. Falling Emergency Relief Crate Physics
    if (this.droppedCargoMesh && this.droppedCargoMesh.visible && !this.droppedCargoLanded) {
      this.droppedCargoVel.y -= 9.81 * delta;
      this.droppedCargoVel.multiplyScalar(Math.exp(-0.45 * delta));
      this.droppedCargoMesh.position.addScaledVector(this.droppedCargoVel, delta);
      const gY = this.getTerrainHeight(this.droppedCargoMesh.position.x, this.droppedCargoMesh.position.z) + 0.12;
      if (this.droppedCargoMesh.position.y <= gY) {
        this.droppedCargoMesh.position.y = gY;
        this.droppedCargoVel.set(0, 0, 0);
        this.droppedCargoLanded = true;
        this.playTone(320, 0.12);
      }
    }

    // 2. Pooled 3D Fertilizer Spray Mist / Flung Granules
    const canEmit = this.isSpraying && this.isArmed && !this.isLanded && this.payloadPercent > 0
      && (p.payloadType === 'sprayer' || p.payloadType === 'spreader');
    if (canEmit && !this.sprayPoints) {
      this.initPayloadEffects();
    }
    if (!this.sprayPoints) return;

    const isSpreader = p.payloadType === 'spreader';
    this.sprayPoints.material.color.setHex(isSpreader ? 0xfbbf24 : 0x7dd3fc);
    this.sprayPoints.material.size = isSpreader ? 0.18 : 0.28;

    const posAttr = this.sprayPoints.geometry.attributes.position;
    const arr = posAttr.array;
    let toEmit = canEmit ? Math.max(2, Math.round(delta * 110)) : 0;
    const offsets = (this.droneBuilder && this.droneBuilder.nozzleLocalOffsets && this.droneBuilder.nozzleLocalOffsets.length > 0)
      ? this.droneBuilder.nozzleLocalOffsets
      : [new THREE.Vector3(-0.45, -0.36, 0), new THREE.Vector3(0.45, -0.36, 0)];

    let anyActive = false;
    for (let i = 0; i < this.sprayData.length; i++) {
      const pt = this.sprayData[i];
      if (!pt.active && toEmit > 0) {
        toEmit--;
        const off = offsets[i % offsets.length];
        this._moveDelta.copy(off).applyEuler(this.rotation).add(this.position);
        arr[i * 3] = this._moveDelta.x;
        arr[i * 3 + 1] = this._moveDelta.y;
        arr[i * 3 + 2] = this._moveDelta.z;
        if (isSpreader) {
          const ang = Math.random() * Math.PI * 2;
          const spd = 2.8 + Math.random() * 3.8;
          pt.vx = this.velocity.x * 0.5 + Math.cos(ang) * spd;
          pt.vy = -1.2 - Math.random() * 1.4;
          pt.vz = this.velocity.z * 0.5 + Math.sin(ang) * spd;
          pt.maxLife = 1.4;
        } else {
          // Atomized mist blown downward by rotor downwash + drifting with live wind!
          pt.vx = this.velocity.x * 0.35 + this.liveWindVec.x * 0.8 + (Math.random() - 0.5) * 1.4;
          pt.vy = -3.4 - Math.random() * 2.2;
          pt.vz = this.velocity.z * 0.35 + this.liveWindVec.z * 0.8 + (Math.random() - 0.5) * 1.4;
          pt.maxLife = 1.2;
        }
        pt.life = 0;
        pt.active = true;
      }

      if (pt.active) {
        pt.life += delta;
        if (isSpreader) {
          pt.vy -= 7.5 * delta;
        } else {
          // Mist slows down and drifts with live wind
          pt.vx += (this.liveWindVec.x - pt.vx) * Math.min(1, delta * 2.2);
          pt.vz += (this.liveWindVec.z - pt.vz) * Math.min(1, delta * 2.2);
          pt.vy += (-1.2 - pt.vy) * Math.min(1, delta * 1.5);
        }
        arr[i * 3] += pt.vx * delta;
        arr[i * 3 + 1] += pt.vy * delta;
        arr[i * 3 + 2] += pt.vz * delta;

        if (pt.life >= pt.maxLife || arr[i * 3 + 1] <= 0.08) {
          pt.active = false;
          arr[i * 3 + 1] = -999;
        } else {
          anyActive = true;
        }
      }
    }
    if (anyActive || canEmit) {
      posAttr.needsUpdate = true;
    }
  }

  // Play staggered Web Audio notes (authentic BLHeli/DJI ESC startup & shutdown melodies!)
  playMelody(notes) {
    if (!this.audioCtx || this.isMuted || !Array.isArray(notes)) return;
    const baseTime = this.audioCtx.currentTime;
    notes.forEach(([freq, delaySec, durSec]) => {
      try {
        const osc = this.audioCtx.createOscillator();
        const gain = this.audioCtx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(freq, baseTime + delaySec);
        gain.gain.setValueAtTime(0.075, baseTime + delaySec);
        gain.gain.exponentialRampToValueAtTime(0.001, baseTime + delaySec + durSec);
        osc.connect(gain);
        gain.connect(this.audioCtx.destination);
        osc.start(baseTime + delaySec);
        osc.stop(baseTime + delaySec + durSec + 0.01);
      } catch (e) {}
    });
  }

  // Real Pre-Arm Safety Diagnostics (battery, upright tilt, assembly health)
  runPreArmCheck() {
    if (this.batteryPercent <= 2) {
      return { ok: false, reason: 'แบตเตอรี่หมด (≤ 2%) — กด RESET เพื่อเปลี่ยนแบตก้อนใหม่' };
    }
    if (Math.abs(this.rotation.x) > 0.52 || Math.abs(this.rotation.z) > 0.52) {
      return { ok: false, reason: 'ตัวเครื่องเอียงเกิน 30° — วางโดรนบนพื้นราบหรือกด RESET ก่อนสตาร์ท' };
    }
    if (this.stats && this.stats.isCompatible === false) {
      return { ok: false, reason: 'ชิ้นส่วนไม่เข้ากัน — ตรวจสอบหน้าประกอบก่อนสตาร์ท' };
    }
    return { ok: true, reason: '' };
  }

  // Realistic Start / ARM Sequence (with Pre-Arm Check & ESC Startup Melody)
  startDrone({ instant = false, autoTakeoff = false } = {}) {
    this.initAudio();
    const check = this.runPreArmCheck();
    if (!check.ok) {
      this.playMelody([[260, 0, 0.12], [200, 0.14, 0.18]]);
      this.setHudAlert(`❌ PRE-ARM ไม่ผ่าน: ${check.reason}`, 3400);
      return false;
    }

    this.isCrashed = false;
    this.parachuteDeployed = false;
    this.isRTHMode = false;
    this.rthPhase = 'IDLE';
    this.autoLandSettleTimer = 0;
    this.touchdownHoldTimer = 0;
    this.emergencyKillArmUntil = 0;

    if (autoTakeoff) {
      this.autoTakeoffTargetY = this.position.y + 3.0;
      this.autoTakeoffTimer = 0;
    }

    // Authentic 4-note brushless ESC initialization melody
    this.playMelody([
      [523, 0.00, 0.08],
      [659, 0.09, 0.08],
      [784, 0.18, 0.09],
      [1046, 0.30, 0.16]
    ]);

    if (instant) {
      this.isArmed = true;
      this.preArmTimer = 0;
      this.pendingAutoTakeoff = false;
      this.powerState = autoTakeoff ? 'AUTO_TAKEOFF' : (this.isLanded ? 'ARMED_IDLE' : 'FLYING');
      if (autoTakeoff) {
        this.setHudAlert('🛫 Auto-Takeoff — กำลังบินขึ้นอัตโนมัติสู่ความสูง 3.0 ม.', 2800);
      } else {
        this.setHudAlert('🟢 ARMED — สตาร์ทมอเตอร์เรียบร้อย!', 1800);
      }
      return true;
    }

    this.isArmed = false;
    this.powerState = 'PRE_ARMING';
    this.preArmTimer = 0.72;
    this.pendingAutoTakeoff = autoTakeoff;
    this.setHudAlert('⚙️ PRE-ARM — ตรวจ Gyro · ESC · แบตเตอรี่ · สัญญาณ RX...', 1600);
    return true;
  }

  // Safe Stop / Disarm (Anti-Accidental Mid-Air Stop Guard: triggers Safe Auto-Land if airborne unless forceEmergency!)
  stopDrone({ forceEmergency = false } = {}) {
    this.initAudio();
    if (this.powerState === 'PRE_ARMING') {
      this.powerState = 'DISARMED';
      this.preArmTimer = 0;
      this.pendingAutoTakeoff = false;
      this.playTone(320, 0.12);
      this.setHudAlert('ยกเลิกการสตาร์ท (DISARMED)', 1800);
      return false;
    }

    if (!this.isArmed) return false;

    // Real-world safety guard: normal Stop while flying triggers Safe Auto-Land instead of dropping from sky!
    if (!this.isLanded && !forceEmergency) {
      this.startAutoLand(true);
      return true;
    }

    this.isArmed = false;
    this.isRTHMode = false;
    this.isFailsafeReturn = false;
    this.sensorLostSim = false;
    this.signalLossTimer = 0;
    this.rthPhase = 'IDLE';
    this.rthWaypoints = [];
    this.rthWaypointIdx = 0;
    if (typeof this.clearRthPathVisuals === 'function') this.clearRthPathVisuals();
    this.isSpraying = false;
    this.pendingAutoTakeoff = false;
    this.autoLandSettleTimer = 0;
    this.touchdownHoldTimer = 0;
    this.emergencyKillArmUntil = 0;
    this.powerState = 'DISARMED';

    if (forceEmergency && !this.isLanded) {
      this.playMelody([[440, 0.00, 0.10], [220, 0.11, 0.26]]);
      this.setHudAlert('🛑 EMERGENCY KILL! ตัดมอเตอร์กลางอากาศทันที!', 3200);
    } else {
      // Authentic descending ESC power-down chime
      this.playMelody([[784, 0.00, 0.09], [587, 0.10, 0.09], [392, 0.20, 0.18]]);
      this.setHudAlert('⏹ ดับมอเตอร์เรียบร้อย (DISARMED — ใบพัดค่อย ๆ หยุดหมุน)', 2400);
    }
    return false;
  }

  // Context-aware Start/Stop toggle (KeyB / HUD Start-Stop / Gamepad Button A)
  armDrone() {
    if (this.powerState === 'DISARMED') {
      return this.startDrone({ instant: false, autoTakeoff: false });
    }
    if (this.powerState === 'PRE_ARMING' || this.isLanded) {
      return this.stopDrone({ forceEmergency: false });
    }
    if (this.powerState === 'AUTO_LANDING') {
      this.powerState = 'FLYING';
      this.holdAltitude = this.position.y;
      this.altHoldLocked = true;
      this.playTone(580, 0.12);
      this.setHudAlert('⏸ ยกเลิก Auto-Land — นักบินรับช่วงควบคุม', 2200);
      return true;
    }
    // Airborne normal Stop -> Safe Auto-Landing!
    return this.stopDrone({ forceEmergency: false });
  }

  // One-Click / KeyT Auto-Takeoff to 3.0m Safe Hover
  startAutoTakeoff() {
    this.initAudio();
    if (this.isArmed && !this.isLanded) {
      if (this.powerState === 'AUTO_LANDING') {
        this.powerState = 'FLYING';
        this.holdAltitude = this.position.y;
        this.altHoldLocked = true;
        this.playTone(620, 0.12);
        this.setHudAlert('⏸ ยกเลิกการลงจอด — รักษาระดับกลางอากาศ', 2000);
      } else {
        this.setHudAlert('เครื่องกำลังลอยอยู่กลางอากาศแล้ว', 1600);
      }
      return;
    }
    if (this.powerState === 'DISARMED') {
      this.startDrone({ instant: false, autoTakeoff: true });
      return;
    }
    if (this.powerState === 'PRE_ARMING') {
      this.pendingAutoTakeoff = true;
      return;
    }
    // ARMED_IDLE on ground -> smooth Auto-Takeoff!
    this.autoTakeoffTargetY = this.position.y + 3.0;
    this.autoTakeoffTimer = 0;
    this.powerState = 'AUTO_TAKEOFF';
    this.isRTHMode = false;
    this.rthPhase = 'IDLE';
    this.playMelody([[587, 0, 0.08], [880, 0.09, 0.14]]);
    this.setHudAlert('🛫 Auto-Takeoff — กำลังบินขึ้นอัตโนมัติสู่ความสูง 3.0 ม.', 2600);
  }

  // One-Click / KeyL Smooth Auto-Land In-Place & Auto-Disarm
  startAutoLand(fromStopButton = false) {
    this.initAudio();
    if (this.powerState === 'AUTO_LANDING') {
      this.powerState = 'FLYING';
      this.holdAltitude = this.position.y;
      this.altHoldLocked = true;
      this.playTone(540, 0.12);
      this.setHudAlert('⏸ ยกเลิก Auto-Land — นักบินรับช่วงควบคุม', 2000);
      return;
    }
    if (!this.isArmed) {
      this.setHudAlert('เครื่องดับมอเตอร์อยู่บนพื้นแล้ว (DISARMED)', 1800);
      return;
    }
    if (this.isLanded) {
      this.stopDrone({ forceEmergency: false });
      return;
    }
    this.isRTHMode = false;
    this.rthPhase = 'IDLE';
    this.powerState = 'AUTO_LANDING';
    this.autoLandSettleTimer = 0;
    this.holdTargetPos.copy(this.position);
    this.holdTargetLocked = Math.hypot(this.velocity.x, this.velocity.z) < 0.8;
    this.playMelody([[659, 0, 0.09], [494, 0.10, 0.14]]);
    this.setHudAlert(
      fromStopButton
        ? '🛬 ป้องกันตกกลางอากาศ — กำลังลงจอดอัตโนมัติเพื่อดับมอเตอร์ (กด X ย้ำ 2 ครั้งเพื่อตัดฉุกเฉิน)'
        : '🛬 Auto-Land — กำลังลดระดับลงจอดและดับมอเตอร์อัตโนมัติ (กด Space หรือ L ยกเลิก)',
      3600
    );
  }

  // Double-Tap KeyX (or HUD Emergency Kill button) Instant Mid-Air Motor Cut!
  triggerEmergencyKill(directFromButton = false) {
    this.initAudio();
    if (!this.isArmed && this.powerState !== 'PRE_ARMING') {
      this.setHudAlert('มอเตอร์ดับอยู่แล้ว (DISARMED)', 1500);
      return;
    }
    if (this.isLanded || this.powerState === 'PRE_ARMING') {
      this.stopDrone({ forceEmergency: true });
      return;
    }
    const nowMs = performance.now();
    if (directFromButton || nowMs < this.emergencyKillArmUntil) {
      this.emergencyKillArmUntil = 0;
      this.stopDrone({ forceEmergency: true });
    } else {
      this.emergencyKillArmUntil = nowMs + 1800;
      this.playTone(880, 0.10);
      this.setHudAlert('⚠️ กด X ซ้ำภายใน 1.8 วิ เพื่อตัดมอเตอร์กลางอากาศทันที (EMERGENCY KILL)!', 2000);
    }
  }

  initAudio() {
    if (this.audioInitialized) return;
    try {
      const AudioContext = window.AudioContext || window.webkitAudioContext;
      if (AudioContext) {
        this.audioCtx = new AudioContext();

        // 1. Master Output Gain
        this.masterGain = this.audioCtx.createGain();
        this.masterGain.gain.setValueAtTime(0.22, this.audioCtx.currentTime);
        this.masterGain.connect(this.audioCtx.destination);

        // 2. Master Lowpass Filter (Eliminates all harsh high frequencies)
        this.audioFilter = this.audioCtx.createBiquadFilter();
        this.audioFilter.type = 'lowpass';
        this.audioFilter.frequency.setValueAtTime(380, this.audioCtx.currentTime);
        this.audioFilter.Q.setValueAtTime(0.8, this.audioCtx.currentTime);
        this.audioFilter.connect(this.masterGain);

        // 3. Motor Drone Oscillators (Soft Sine / Triangle Waves only)
        this.motorGain = this.audioCtx.createGain();
        this.motorGain.gain.setValueAtTime(0.0001, this.audioCtx.currentTime);

        this.motorOsc1 = this.audioCtx.createOscillator();
        this.motorOsc1.type = 'sine';
        this.motorOsc1.frequency.setValueAtTime(80, this.audioCtx.currentTime);

        this.motorOsc2 = this.audioCtx.createOscillator();
        this.motorOsc2.type = 'triangle';
        this.motorOsc2.frequency.setValueAtTime(160, this.audioCtx.currentTime);

        this.motorOsc1.connect(this.motorGain);
        this.motorOsc2.connect(this.motorGain);
        this.motorGain.connect(this.audioFilter);

        this.motorOsc1.start();
        this.motorOsc2.start();

        // 4. Pink Noise Airflow Generator (Soft carbon propeller slicing air)
        const bufferSize = this.audioCtx.sampleRate * 2;
        const noiseBuffer = this.audioCtx.createBuffer(1, bufferSize, this.audioCtx.sampleRate);
        const output = noiseBuffer.getChannelData(0);
        let b0 = 0, b1 = 0, b2 = 0;
        for (let i = 0; i < bufferSize; i++) {
          const white = Math.random() * 2 - 1;
          b0 = 0.99886 * b0 + white * 0.0555179;
          b1 = 0.99332 * b1 + white * 0.0750759;
          b2 = 0.96900 * b2 + white * 0.1538520;
          output[i] = (b0 + b1 + b2) * 0.06;
        }

        this.noiseNode = this.audioCtx.createBufferSource();
        this.noiseNode.buffer = noiseBuffer;
        this.noiseNode.loop = true;

        this.noiseGain = this.audioCtx.createGain();
        this.noiseGain.gain.setValueAtTime(0.0001, this.audioCtx.currentTime);

        this.noiseFilter = this.audioCtx.createBiquadFilter();
        this.noiseFilter.type = 'lowpass';
        this.noiseFilter.frequency.setValueAtTime(260, this.audioCtx.currentTime);

        this.noiseNode.connect(this.noiseFilter);
        this.noiseFilter.connect(this.noiseGain);
        this.noiseGain.connect(this.audioFilter);
        this.noiseNode.start();

        // 5. Ambient Atmospheric Wind Whoosh (bandpass-filtered airflow swells naturally in strong crosswind & gusts!)
        this.windNode = this.audioCtx.createBufferSource();
        this.windNode.buffer = noiseBuffer;
        this.windNode.loop = true;

        this.windFilter = this.audioCtx.createBiquadFilter();
        this.windFilter.type = 'bandpass';
        this.windFilter.frequency.setValueAtTime(340, this.audioCtx.currentTime);
        this.windFilter.Q.setValueAtTime(1.4, this.audioCtx.currentTime);

        this.windGain = this.audioCtx.createGain();
        this.windGain.gain.setValueAtTime(0.0001, this.audioCtx.currentTime);

        this.windNode.connect(this.windFilter);
        this.windFilter.connect(this.windGain);
        this.windGain.connect(this.masterGain);
        this.windNode.start();

        this.audioInitialized = true;
      }
    } catch (e) {
      console.warn('Web Audio not allowed or error:', e);
    }
  }

  toggleMute() {
    this.isMuted = !this.isMuted;
    if (this.masterGain && this.audioCtx) {
      this.masterGain.gain.setTargetAtTime(this.isMuted ? 0.0 : 0.22, this.audioCtx.currentTime, 0.05);
    }
    return this.isMuted;
  }

  playTone(freq, duration = 0.12) {
    if (!this.audioCtx || this.isMuted) return;
    try {
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = 'sine';
      osc.frequency.setValueAtTime(freq, this.audioCtx.currentTime);
      gain.gain.setValueAtTime(0.08, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + duration);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + duration);
    } catch (e) {}
  }

  toggleCamera() {
    this.isFPV = !this.isFPV;
    // Sync chase camera state cleanly to current drone pose so switching views NEVER jerks or swings!
    this.camInitialized = false;
    const btnCam = document.getElementById('btn-toggle-camera');
    if (btnCam) {
      btnCam.innerText = this.isFPV ? '📷 FPV (V)' : '📷 มุมมองที่ 3 (V)';
      btnCam.classList.toggle('primary', this.isFPV);
    }
    const reticle = document.getElementById('fpv-crosshair-hud');
    if (reticle) reticle.classList.toggle('hidden', !this.isFPV);
    if (typeof this.setHudAlert === 'function') {
      const p = this.physicsParams;
      const camModeTxt = this.isFPV
        ? (p && p.hasGimbal ? '📷 มุมมองบุคคลที่ 1 (FPV · กิมบอลกันสั่น 3 แกน)' : '📷 มุมมองบุคคลที่ 1 (FPV · มุมกล้องนักบิน)')
        : '🎥 มุมมองบุคคลที่ 3 (กล้องติดตามหลังลำ)';
      this.setHudAlert(camModeTxt, 2000);
    }
    return this.isFPV;
  }

  isSimulatorActive() {
    if (!this._simScreenEl) this._simScreenEl = document.getElementById('screen-simulator');
    if (!this._simScreenEl || !this._simScreenEl.classList.contains('active')) return false;
    const tag = document.activeElement && document.activeElement.tagName;
    if (tag === 'INPUT' || tag === 'TEXTAREA') return false;
    return true;
  }

  initControls() {
    window.addEventListener('keydown', (e) => {
      if (!this.isSimulatorActive()) return;
      if (e.code === 'Space') e.preventDefault();
      this.keys[e.code] = true;
      if (e.code === 'KeyC') {
        // Wrap current orbit angle to [-PI, PI] so pressing C always glides smoothly via the shortest arc back behind the drone!
        this.camYaw = Math.atan2(Math.sin(this.camYaw), Math.cos(this.camYaw));
        this.targetCamYaw = 0;
        this.targetCamPitch = 0.18;
      }
      if (e.code === 'KeyV' && !e.repeat) {
        this.toggleCamera();
      }
      if (e.code === 'KeyM') {
        this.toggleMute();
      }
      if (e.code === 'KeyH') {
        const p = this.physicsParams || this.computeDronePhysicsParams();
        if (!p.hasGps) {
          this.isHoldMode = false;
          this.playTone(240, 0.16);
          this.setHudAlert('ไม่มีโมดูล GPS — บินโหมดควบคุมเอง (ATTI)');
        } else {
          this.isHoldMode = !this.isHoldMode;
          if (this.isHoldMode) {
            this.holdTargetPos.copy(this.position);
            this.holdTargetLocked = Math.hypot(this.velocity.x, this.velocity.z) < 1.0;
            this.windEst.set(0, 0);
            this.playTone(780, 0.12);
            this.setHudAlert(p.isRtk ? '📡 เปิด RTK GPS Hold (ล็อกระดับเซนติเมตร)' : '📡 เปิด GPS Position Hold (ต้านลมอัตโนมัติ)');
          } else {
            this.isRTHMode = false;
            this.rthPhase = 'IDLE';
            this.playTone(460, 0.12);
            this.setHudAlert('✈️ ปิด GPS Hold — โหมด ATTI (ลอยตามลมและแรงเฉื่อย)');
          }
        }
      }
      if (e.code === 'KeyR' && !e.repeat) {
        const p = this.physicsParams || this.computeDronePhysicsParams();
        if (this.isRTHMode) {
          this.cancelSmartAutoReturn('ยกเลิก RTH — นักบินรับช่วงควบคุม');
        } else if (!p.hasGps && (!this.flightPathHistory || this.flightPathHistory.length < 2)) {
          this.playTone(240, 0.16);
          this.setHudAlert('ต้องติดตั้ง GPS หรือบินออกจากจุดเริ่มต้นก่อนจึงจะคำนวณเส้นทางกลับได้');
        } else {
          if (!this.isArmed) this.startDrone({ instant: true, autoTakeoff: false });
          this.triggerSmartAutoReturn('สั่ง RTH คำนวณเส้นทางกลับฐานอัตโนมัติ', false, false);
        }
      }
      if (e.code === 'KeyP' && !e.repeat) {
        this.toggleSimulatedSensorLoss();
      }
      if (e.code === 'KeyF' && !e.repeat) {
        this.togglePayloadAction();
      }
      if (e.code === 'KeyB' && !e.repeat) {
        this.armDrone();
      }
      if (e.code === 'KeyT' && !e.repeat) {
        this.startAutoTakeoff();
      }
      if (e.code === 'KeyL' && !e.repeat) {
        this.startAutoLand(false);
      }
      if (e.code === 'KeyX' && !e.repeat) {
        this.triggerEmergencyKill(false);
      }
      if (e.code === 'KeyG' && !e.repeat) {
        this.cycleControlRateMode();
      }
      if (e.code === 'KeyN' && !e.repeat) {
        this.cycleWindPreset();
      }
      if (e.code === 'Space' && !this.isArmed && this.powerState !== 'PRE_ARMING') {
        this.startDrone({ instant: true, autoTakeoff: false });
      }
    });

    window.addEventListener('keyup', (e) => {
      this.keys[e.code] = false;
    });

    // Mouse drag for silky camera orbit (only on canvas left-click, with spike clamping & focus safety!)
    this.container.addEventListener('mousedown', (e) => {
      if (e.button !== 0) return;
      if (this.renderer && e.target !== this.renderer.domElement) return;
      this.isDraggingMouse = true;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
    });

    window.addEventListener('mousemove', (e) => {
      if (!this.isDraggingMouse) return;
      // Release immediately if primary mouse button was released outside the window
      if ((e.buttons & 1) === 0) {
        this.isDraggingMouse = false;
        return;
      }
      const rawDx = e.clientX - this.lastMouseX;
      const rawDy = e.clientY - this.lastMouseY;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
      // Clamp sudden OS pointer-jump spikes so camera can NEVER whip 180° in one frame!
      const dx = Math.max(-55, Math.min(55, rawDx));
      const dy = Math.max(-55, Math.min(55, rawDy));

      this.targetCamYaw -= dx * 0.0048;
      this.targetCamPitch = Math.max(-0.18, Math.min(0.78, this.targetCamPitch + dy * 0.0045));
    });

    const stopDrag = () => { this.isDraggingMouse = false; };
    window.addEventListener('mouseup', stopDrag);
    window.addEventListener('blur', stopDrag);
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        this.isDraggingMouse = false;
        this.keys = {};
      }
    });

    // Touch controls support (with spike clamping)
    this.container.addEventListener('touchstart', (e) => {
      if (e.touches.length === 1 && (!this.renderer || e.target === this.renderer.domElement)) {
        this.isDraggingMouse = true;
        this.lastMouseX = e.touches[0].clientX;
        this.lastMouseY = e.touches[0].clientY;
      }
    }, { passive: true });

    window.addEventListener('touchmove', (e) => {
      if (!this.isDraggingMouse || e.touches.length !== 1) return;
      const dx = Math.max(-55, Math.min(55, e.touches[0].clientX - this.lastMouseX));
      const dy = Math.max(-55, Math.min(55, e.touches[0].clientY - this.lastMouseY));
      this.lastMouseX = e.touches[0].clientX;
      this.lastMouseY = e.touches[0].clientY;

      this.targetCamYaw -= dx * 0.0065;
      this.targetCamPitch = Math.max(-0.18, Math.min(0.78, this.targetCamPitch + dy * 0.006));
    }, { passive: true });

    window.addEventListener('touchend', stopDrag);
    window.addEventListener('touchcancel', stopDrag);
  }

  // Update loop (runs ONLY while simulator screen is active -> 0% wasted background CPU/GPU!)
  update() {
    if (!this.loopActive) return;
    this.rafId = requestAnimationFrame(this._boundUpdate);

    const delta = Math.min(this.clock.getDelta(), 0.05);
    if (!this.isSimulatorActive()) {
      this.keys = {};
      if (this.audioInitialized && this.audioCtx) {
        if (this.motorGain) this.motorGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.05);
        if (this.noiseGain) this.noiseGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.05);
      }
      return;
    }

    // Measure live rolling FPS (updated every 450ms, zero overhead!)
    const nowMs = performance.now();
    this._fpsFrames = (this._fpsFrames || 0) + 1;
    const fpsElapsed = nowMs - (this._fpsLastTime || nowMs);
    if (fpsElapsed >= 450) {
      this.liveFps = Math.min(240, Math.max(1, Math.round((this._fpsFrames * 1000) / fpsElapsed)));
      this._fpsFrames = 0;
      this._fpsLastTime = nowMs;
    }

    this.processInputs(delta);
    this.updatePhysics(delta);
    this.updatePayloadEffects(delta);
    this.updateWindsock(delta);
    this.updateWindStreaks(delta);
    this.updateSensorVisuals();
    this.updateRthPathVisuals();
    this.updateCheckpoints();
    this.updateCamera(delta);
    this.updateAudio();

    // Update lightweight soft ground contact shadow directly beneath drone (especially valuable in Fast 60 FPS mode!)
    if (this.droneContactShadow) {
      const gY = this.getTerrainHeight(this.position.x, this.position.z);
      const agl = Math.max(0, this.position.y - gY);
      const isFast = this.graphicsQuality === 'fast';
      const showContact = isFast || agl < 5.5;
      this.droneContactShadow.visible = showContact;
      if (showContact) {
        const sc = 1.0 + Math.min(2.2, agl * 0.14);
        this.droneContactShadow.position.set(this.position.x, gY + 0.055, this.position.z);
        this.droneContactShadow.scale.set(sc, sc, 1);
        const fade = Math.max(0, 1.0 - agl / (isFast ? 22.0 : 6.0));
        this.droneContactShadow.material.opacity = (isFast ? 0.68 : 0.34) * fade;
      }
    }

    // Throttle HUD DOM updates to ~10 Hz (instant on state/mode/alert/signal/wind/checkpoint changes) to eliminate DOM reflow overhead!
    const vrsWarn = this.vrsIntensity > 0.35;
    const alertActive = nowMs < (this.hudAlertUntil || 0);
    const stateSig = `${this.isArmed}|${this.powerState}|${this.controlRateMode}|${this.graphicsQuality}|${this.isLanded}|${this.isCrashed}|${this.isHoldMode}|${this.isRTHMode}|${this.rthPhase}|${this.isFailsafeReturn}|${this.sensorLostSim}|${this.rthWaypointIdx}|${vrsWarn}|${this.lidarBraking}|${this.lidarZone}|${this.altSensorType}|${this.altSensorLocked}|${this.isSpraying}|${this.parachuteDeployed}|${this.lawOverCeiling}|${this.hitAltitudeLimit}|${this.signalState}|${this.signalSource}|${this.windPreset}|${this.windLevel}|${this.windGustActive}|${alertActive}|${this.completedCheckpoints}`;
    if (nowMs - (this._lastTelemetryTime || 0) >= 100 || stateSig !== this._lastTelemetryState) {
      this._lastTelemetryTime = nowMs;
      this._lastTelemetryState = stateSig;
      this.triggerTelemetry();
    }

    // Texel-snapped 2m grid shadow follow + Smart Throttled Shadow Map updates (every 2nd frame in Balanced = 2x faster shadow pass!)
    const isFastMode = this.graphicsQuality === 'fast';
    if (!isFastMode && this.sunLight && this.sunOffset) {
      const snapX = Math.round(this.position.x * 0.5) * 2;
      const snapZ = Math.round(this.position.z * 0.5) * 2;
      if (snapX !== this._lastShadowSnapX || snapZ !== this._lastShadowSnapZ) {
        this._lastShadowSnapX = snapX;
        this._lastShadowSnapZ = snapZ;
        this.sunLight.position.set(snapX + this.sunOffset.x, this.sunOffset.y, snapZ + this.sunOffset.z);
        this.sunLight.target.position.set(snapX, 0, snapZ);
        this.sunLight.target.updateMatrixWorld();
        if (this.renderer && this.renderer.shadowMap.enabled) {
          this.renderer.shadowMap.needsUpdate = true;
        }
      } else if (this.renderer && this.renderer.shadowMap.enabled && !this.renderer.shadowMap.autoUpdate) {
        this._shadowFrameCount = (this._shadowFrameCount + 1) % 2;
        if (this._shadowFrameCount === 0) {
          this.renderer.shadowMap.needsUpdate = true;
        }
      }
    }

    // Animate airport radar dish, southern wind turbines & gentle shimmering lake ripples (skip distant micro-anims in Fast mode)
    if (this.radarDish) {
      this.radarDish.rotation.y += delta * 1.4;
    }
    const wSpin = Math.max(0.45, Math.min(4.8, 0.55 + (this.liveWindSpeed || 1.0) * 0.32));
    if (this.windTurbineRotors) {
      for (let i = 0; i < this.windTurbineRotors.length; i++) {
        this.windTurbineRotors[i].rotation.z -= delta * (wSpin + i * 0.14);
      }
    }
    if (this.windmillSail) {
      this.windmillSail.rotation.z -= delta * (0.28 + wSpin * 0.35);
    }
    if (!isFastMode) {
      if (this.waterTex) {
        this.waterTex.offset.x = (this.waterTex.offset.x + delta * 0.022) % 1;
        this.waterTex.offset.y = (this.waterTex.offset.y + delta * 0.013) % 1;
      }
      if (this.waterTex2) {
        this.waterTex2.offset.x = (this.waterTex2.offset.x - delta * 0.015) % 1;
        this.waterTex2.offset.y = (this.waterTex2.offset.y + delta * 0.019) % 1;
      }
      if (this.waterBumpTex) {
        this.waterBumpTex.offset.x = (this.waterBumpTex.offset.x + delta * 0.028) % 1;
        this.waterBumpTex.offset.y = (this.waterBumpTex.offset.y - delta * 0.014) % 1;
      }
      if (this.lakeBoats) {
        const nowSec = nowMs * 0.001;
        for (let i = 0; i < this.lakeBoats.length; i++) {
          const b = this.lakeBoats[i];
          b.mesh.position.y = b.baseY + Math.sin(nowSec * 1.8 + b.phase) * b.ampY;
          b.mesh.rotation.z = Math.sin(nowSec * 1.35 + b.phase * 1.3) * b.ampRoll;
          b.mesh.rotation.x = Math.cos(nowSec * 1.1 + b.phase) * (b.ampRoll * 0.55);
        }
      }
      if (this.lighthouseBeam) {
        this.lighthouseBeam.rotation.y += delta * 0.85;
      }
      if (this.farmSpinners) {
        for (let i = 0; i < this.farmSpinners.length; i++) {
          this.farmSpinners[i].mesh.rotation.y += delta * this.farmSpinners[i].speed;
        }
      }
      if (this.waterfallTex) {
        this.waterfallTex.offset.y = (this.waterfallTex.offset.y + delta * 0.55) % 1;
      }
      if (this.clouds) {
        const cloudDrift = Math.max(0.7, Math.min(6.5, 0.65 + (this.liveWindSpeed || 1.0) * 0.42));
        for (let i = 0; i < this.clouds.length; i++) {
          this.clouds[i].position.x += delta * cloudDrift;
          if (this.clouds[i].position.x > 260) this.clouds[i].position.x = -260;
        }
      }
    }

    if (this.droneBuilder) {
      const p = this.physicsParams || this.computeDronePhysicsParams();
      // Scale visual blade angular speed realistically with motor KV & pack voltage
      const maxSpinVisual = Math.min(76, Math.max(34, (p.motorKv * p.nomVoltage) / 260));
      const anySpin = this.motorRpm[0] + this.motorRpm[1] + this.motorRpm[2] + this.motorRpm[3] > 0.01;
      this.droneBuilder.update(
        delta,
        anySpin,
        maxSpinVisual,
        this.motorRpm,
        { pitch: this.rotation.x, roll: this.rotation.z },
        {
          payloadFraction: this.payloadPercent / 100,
          isSpraying: this.isSpraying && this.isArmed && !this.isLanded && this.payloadPercent > 0,
          cargoAttached: this.cargoAttached,
          parachuteDeployed: this.parachuteDeployed,
          signalState: this.signalState,
          lawOverCeiling: this.lawOverCeiling,
          isWarningLed: this.isCrashed || this.batteryPercent <= 25 || this.lawOverCeiling || this.vrsIntensity > 0.35 || this.signalState === 'critical'
        }
      );
    }

    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }

  // Real DJI/RC Flight-Rate Switch (Cine / Normal Smooth / Sport) — same 6-DOF physics, tunable stick shaping!
  getControlRateProfile() {
    if (this.controlRateMode === 'cine') {
      return {
        id: 'cine', label: '🎯 นุ่มนวล (Cine)',
        stickOmega: 5.2, yawOmega: 4.6, thrOmega: 5.8,
        tiltScale: 0.56, yawScale: 0.58, climbScale: 0.62,
        expo: 0.36, setpointSmooth: 8.5, brakeMult: 0.74
      };
    }
    if (this.controlRateMode === 'sport') {
      return {
        id: 'sport', label: '⚡ สปอร์ต (Sport)',
        stickOmega: 11.8, yawOmega: 10.8, thrOmega: 12.2,
        tiltScale: 1.04, yawScale: 1.08, climbScale: 1.05,
        expo: 0.22, setpointSmooth: 18.0, brakeMult: 1.15
      };
    }
    return {
      id: 'normal', label: '⚖️ นุ่มสมดุล (Smooth)',
      stickOmega: 7.6, yawOmega: 6.8, thrOmega: 8.2,
      tiltScale: 0.90, yawScale: 0.88, climbScale: 0.92,
      expo: 0.30, setpointSmooth: 11.5, brakeMult: 0.94
    };
  }

  cycleControlRateMode() {
    const order = ['normal', 'cine', 'sport'];
    const idx = order.indexOf(this.controlRateMode || 'normal');
    this.controlRateMode = order[(idx + 1) % order.length];
    const prof = this.getControlRateProfile();
    this.playTone(this.controlRateMode === 'sport' ? 880 : this.controlRateMode === 'cine' ? 520 : 680, 0.11);
    this.setHudAlert(`🎛️ โหมดความนุ่มนวลการบังคับ: ${prof.label}`, 2200);
    return prof;
  }

  // Exact analytical critically-damped 2nd-order S-curve filter (C1-continuous, zero overshoot, framerate-independent!)
  stepSmoothAxis(axis, target, omega, dt) {
    if (!this.inputVel) this.inputVel = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
    const x = this.input[axis] || 0;
    const v = this.inputVel[axis] || 0;
    const e = x - target;
    const wDt = omega * dt;
    const decay = Math.exp(-wDt);
    const vTerm = v + omega * e;
    const nextX = target + (e + vTerm * dt) * decay;
    const nextV = (v - omega * vTerm * dt) * decay;
    this.input[axis] = Math.abs(nextX) < 0.0005 && target === 0 ? 0 : nextX;
    this.inputVel[axis] = Math.abs(nextV) < 0.0005 && target === 0 ? 0 : nextV;
  }

  processInputs(delta = 0.016) {
    // Read keyboard
    let throttleIn = 0;
    let pitchIn = 0;
    let rollIn = 0;
    let yawIn = 0;
    this.activeInputDevice = 'KEYBOARD';

    if (this.keys['Space']) throttleIn += 1;
    if (this.keys['ShiftLeft'] || this.keys['ShiftRight']) throttleIn -= 1;

    if (this.keys['KeyW']) pitchIn += 1;
    if (this.keys['KeyS']) pitchIn -= 1;

    if (this.keys['KeyA']) yawIn -= 1;
    if (this.keys['KeyD']) yawIn += 1;

    if (this.keys['KeyQ']) rollIn -= 1;
    if (this.keys['KeyE']) rollIn += 1;

    // Normalize diagonal W+Q / W+E keyboard input so diagonal flight isn't 1.41x faster than straight flight!
    const diagH = Math.hypot(pitchIn, rollIn);
    if (diagH > 1.0) {
      pitchIn /= diagH;
      rollIn /= diagH;
    }

    // Read HTML5 Gamepad API (Xbox / PlayStation / USB RC Controller — Mode 2)
    if (typeof navigator !== 'undefined' && navigator.getGamepads) {
      const pads = navigator.getGamepads();
      let activePad = null;
      for (let i = 0; i < pads.length; i++) {
        if (pads[i] && pads[i].connected) {
          activePad = pads[i];
          break;
        }
      }
      if (activePad) {
        this.activeInputDevice = 'GAMEPAD';
        // Smooth remapped deadzone (eliminates jump from 0 -> 0.14 at stick center!)
        const dz = 0.12;
        const deadzone = (v) => {
          const val = v || 0;
          const mag = Math.abs(val);
          if (mag <= dz) return 0;
          return Math.sign(val) * ((mag - dz) / (1.0 - dz));
        };
        // Mode 2 standard: Left Stick (axes[0]=Yaw, axes[1]=Throttle), Right Stick (axes[2]=Roll, axes[3]=Pitch)
        const gpYaw = deadzone(activePad.axes[0]);
        const gpThrottle = -deadzone(activePad.axes[1]);
        const gpRoll = deadzone(activePad.axes[2]);
        const gpPitch = -deadzone(activePad.axes[3]);

        if (gpThrottle !== 0) throttleIn = gpThrottle;
        if (gpYaw !== 0) yawIn = gpYaw;
        if (gpRoll !== 0) rollIn = gpRoll;
        if (gpPitch !== 0) pitchIn = gpPitch;

        // Button 0 (A / Cross) to ARM/DISARM on press edge
        const armBtn = activePad.buttons[0] && activePad.buttons[0].pressed;
        if (armBtn && !this._gpArmPrev) {
          this.armDrone();
        }
        this._gpArmPrev = Boolean(armBtn);

        // Button 3 (Y / Triangle) to toggle camera
        const camBtn = activePad.buttons[3] && activePad.buttons[3].pressed;
        if (camBtn && !this._gpCamPrev) {
          this.toggleCamera();
        }
        this._gpCamPrev = Boolean(camBtn);
      }
    }

    // Critically-damped 2nd-order S-curve shaping: zero digital keyboard twitchiness, buttery smooth at any FPS!
    const prof = this.getControlRateProfile();
    const gpMult = this.activeInputDevice === 'GAMEPAD' ? 1.65 : 1.0;
    const dtClamp = Math.min(0.05, Math.max(0.001, delta));
    this.stepSmoothAxis('throttle', throttleIn, prof.thrOmega * gpMult, dtClamp);
    this.stepSmoothAxis('pitch', pitchIn, prof.stickOmega * gpMult, dtClamp);
    this.stepSmoothAxis('roll', rollIn, prof.stickOmega * gpMult, dtClamp);
    this.stepSmoothAxis('yaw', yawIn, prof.yawOmega * gpMult, dtClamp);
  }

  // RC Expo shaping: surgical precision near stick center, full authority at stick edge
  applyStickExpo(val, expo = 0.28) {
    return (1 - expo) * val + expo * val * val * val;
  }

  // Exact downward surface height directly under the drone (terrain, raised Helipad, OR solid building/hangar roof!)
  sampleSurfaceBelowY(px, py, pz) {
    let surfY = (typeof this.getTerrainHeight === 'function') ? this.getTerrainHeight(px, pz) : 0;
    if (this.helipad) {
      const hx = px - this.helipad.position.x;
      const hz = pz - this.helipad.position.z;
      if (hx * hx + hz * hz <= 6.2 * 6.2) {
        const padTop = this.helipad.position.y + 0.075;
        if (padTop <= py + 0.35 && padTop > surfY) surfY = padTop;
      }
    }
    if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
      for (let i = 0; i < this.colliders.length; i++) {
        const c = this.colliders[i];
        if (c.type === 'box') {
          if (px >= c.minX && px <= c.maxX && pz >= c.minZ && pz <= c.maxZ && c.maxY <= py + 0.35 && c.maxY > surfY) {
            surfY = c.maxY;
          }
        } else if (c.type === 'cyl' && c.tag !== 'tree') {
          const d = Math.hypot(px - c.x, pz - c.z);
          if (d <= c.radius && c.maxY <= py + 0.35 && c.maxY > surfY) {
            surfY = c.maxY;
          }
        }
      }
    }
    return surfY;
  }

  // Evaluate ground-to-drone clearance (AGL) vs Barometer Home Altitude + active rangefinder mode
  updateAltitudeSensors(p) {
    const clearance = p.clearance || 0.48;
    const surfY = this.sampleSurfaceBelowY(this.position.x, this.position.y, this.position.z);
    const homeBaseY = (typeof this.getTerrainHeight === 'function' ? this.getTerrainHeight(0, 0) : 0) + clearance;
    const aglRaw = Math.max(0, this.position.y - surfY - clearance);
    const homeAltRaw = this.position.y - homeBaseY;

    this.surfaceBelowY = surfY;
    this.groundClearanceM = Number(aglRaw.toFixed(2));
    this.homeAltitudeM = Number(homeAltRaw.toFixed(1));

    if (p.hasLidar) {
      this.altSensorType = 'RADAR+TOF';
      this.altSensorLocked = aglRaw <= 35.0;
    } else if (p.hasOpticalFlow) {
      this.altSensorType = 'SONAR';
      this.altSensorLocked = aglRaw <= 6.0;
    } else if (p.hasBaro) {
      this.altSensorType = 'BARO';
      this.altSensorLocked = false;
    } else {
      this.altSensorType = 'NONE';
      this.altSensorLocked = false;
    }
  }

  // 360° LiDAR + mmWave Radar Proximity Scanner (14m Detection + 4.0m Active Repulsive Collision Braking!)
  computeLidarAvoidance() {
    let ax = 0, az = 0;
    let triggered = false;
    let nearestDist = Infinity;
    let nearestDx = 0, nearestDz = 0;
    let nearestTag = '';
    let nearestPtX = 0, nearestPtY = 0, nearestPtZ = 0;

    if (this.customMapRoot || !this.colliders || this.colliders.length === 0) {
      this.lidarNearestDist = null;
      this.lidarNearestDirTh = '';
      this.lidarNearestTag = '';
      this.lidarZone = 'clear';
      return { ax, az, triggered, nearestDist: null, zone: 'clear' };
    }

    const scanR = 14.0;   // 360° Radar + LiDAR early detection radius
    const brakeR = 4.0;   // Active autonomous obstacle-avoidance braking radius
    const h = 0.72;
    const px = this.position.x, py = this.position.y, pz = this.position.z;
    const vx = this.velocity.x, vz = this.velocity.z;

    for (let i = 0; i < this.colliders.length; i++) {
      const c = this.colliders[i];
      let nx = 0, nz = 0, dist = 999;
      let hitX = px, hitY = py, hitZ = pz;

      if (c.type === 'box') {
        if (py + h < c.minY || py - h > c.maxY) continue;
        const cx = Math.max(c.minX, Math.min(c.maxX, px));
        const cz = Math.max(c.minZ, Math.min(c.maxZ, pz));
        const dx = px - cx, dz = pz - cz;
        dist = Math.hypot(dx, dz);
        if (dist > 0.01 && dist < scanR) {
          nx = dx / dist;
          nz = dz / dist;
          hitX = cx;
          hitY = Math.max(c.minY, Math.min(c.maxY, py));
          hitZ = cz;
        }
      } else if (c.type === 'cyl' && (c.radius >= 0.75 || c.tag === 'power_pole')) {
        if (py + h < c.minY || py - h > c.maxY) continue;
        const dx = px - c.x, dz = pz - c.z;
        const centerDist = Math.hypot(dx, dz);
        dist = Math.max(0.02, centerDist - c.radius);
        if (centerDist > 0.01 && dist < scanR) {
          nx = dx / centerDist;
          nz = dz / centerDist;
          hitX = c.x + nx * c.radius;
          hitY = Math.max(c.minY, Math.min(c.maxY, py));
          hitZ = c.z + nz * c.radius;
        }
      }

      if (dist < scanR && (nx !== 0 || nz !== 0)) {
        if (dist < nearestDist) {
          nearestDist = dist;
          nearestDx = -nx; // vector from drone -> obstacle
          nearestDz = -nz;
          nearestTag = c.tag || 'building';
          nearestPtX = hitX;
          nearestPtY = hitY;
          nearestPtZ = hitZ;
        }
        // Active repulsive braking within 4.0m when approaching or very close (<1.6m)
        if (dist < brakeR) {
          const approach = -(vx * nx + vz * nz);
          if (approach > 0.35 || dist < 1.6) {
            const proximity = Math.max(0, 1 - dist / brakeR);
            const mag = proximity * (3.6 + Math.max(0, approach) * 1.35);
            ax += nx * mag;
            az += nz * mag;
            triggered = true;
          }
        }
      }
    }

    // Also scan overhead electric power cable spans (สายไฟฟ้า) between utility poles!
    if (this.powerWireSegments && this.powerWireSegments.length > 0) {
      for (let i = 0; i < this.powerWireSegments.length; i++) {
        const w = this.powerWireSegments[i];
        const sdx = w.x2 - w.x1, sdz = w.z2 - w.z1;
        const lenSq = sdx * sdx + sdz * sdz;
        if (lenSq < 0.01) continue;
        const t = Math.max(0, Math.min(1, ((px - w.x1) * sdx + (pz - w.z1) * sdz) / lenSq));
        const wx = w.x1 + sdx * t;
        const wz = w.z1 + sdz * t;
        const wy = w.y1 + (w.y2 - w.y1) * t - Math.sin(t * Math.PI) * (w.sag || 0.65);
        const dx = px - wx, dy = py - wy, dz = pz - wz;
        const dist3d = Math.hypot(dx, dy, dz);
        if (dist3d > 0.01 && dist3d < scanR && Math.abs(dy) < 2.2) {
          const hDist = Math.hypot(dx, dz);
          const nx = hDist > 0.01 ? dx / hDist : 0;
          const nz = hDist > 0.01 ? dz / hDist : 0;
          if (dist3d < nearestDist) {
            nearestDist = dist3d;
            nearestDx = -nx;
            nearestDz = -nz;
            nearestTag = 'power_wire';
            nearestPtX = wx;
            nearestPtY = wy;
            nearestPtZ = wz;
          }
          if (dist3d < brakeR && hDist > 0.01) {
            const approach = -(vx * nx + vz * nz);
            if (approach > 0.30 || dist3d < 1.8) {
              const proximity = Math.max(0, 1 - dist3d / brakeR);
              const mag = proximity * (3.6 + Math.max(0, approach) * 1.35);
              ax += nx * mag;
              az += nz * mag;
              triggered = true;
            }
          }
        }
      }
    }

    const totalMag = Math.hypot(ax, az);
    if (totalMag > 6.5) {
      ax = (ax / totalMag) * 6.5;
      az = (az / totalMag) * 6.5;
    }

    if (nearestDist < scanR) {
      this.lidarNearestDist = Number(nearestDist.toFixed(1));
      if (this.lidarNearestPoint) this.lidarNearestPoint.set(nearestPtX, nearestPtY, nearestPtZ);
      // Express obstacle direction in the drone's own body frame (หน้า/หลัง/ซ้าย/ขวา)
      const yaw = (this.rotation && this.rotation.y) || 0;
      const sinY = Math.sin(yaw);
      const cosY = Math.cos(yaw);
      const obsFwd = nearestDx * sinY + nearestDz * cosY;
      const obsRight = -nearestDx * cosY + nearestDz * sinY;
      const relDeg = (Math.atan2(obsRight, obsFwd) * (180 / Math.PI) + 360) % 360;
      const dirLabels = ['หน้า', 'ขวาหน้า', 'ขวา', 'ขวาหลัง', 'หลัง', 'ซ้ายหลัง', 'ซ้าย', 'ซ้ายหน้า'];
      this.lidarNearestDirTh = dirLabels[Math.round(relDeg / 45) % 8];
      const tagMap = {
        tree: 'ต้นไม้',
        power_pole: 'เสาไฟฟ้า',
        power_wire: 'สายไฟฟ้า',
        hangar: 'โรงเก็บ',
        atc_tower: 'หอบังคับการ',
        telecom_5g: 'เสา 5G',
        wind_turbine: 'กังหันลม',
        solar_grid: 'สถานีไฟ',
        building: 'อาคาร'
      };
      this.lidarNearestTag = tagMap[nearestTag] || 'สิ่งกีดขวาง';
      this.lidarZone = (triggered || nearestDist < 4.0) ? 'brake' : (nearestDist <= 9.0 ? 'detect' : 'clear');
    } else {
      this.lidarNearestDist = null;
      this.lidarNearestDirTh = '';
      this.lidarNearestTag = '';
      this.lidarZone = 'clear';
    }

    return { ax, az, triggered, nearestDist: this.lidarNearestDist, zone: this.lidarZone };
  }

  // Procedural 512x512 Glowing Red Circular Laser Grid Texture (projects onto ground when close to surface!)
  createGroundLaserGridTexture() {
    if (typeof document === 'undefined') return null;
    const size = 512;
    const can = document.createElement('canvas');
    can.width = size;
    can.height = size;
    const ctx = can.getContext('2d');
    const cx = size / 2;
    const cy = size / 2;
    const R = 224;

    ctx.clearRect(0, 0, size, size);

    // 1. Soft crimson radial ground illumination wash (matches realistic red downward structured-light glow!)
    const wash = ctx.createRadialGradient(cx, cy, 12, cx, cy, R + 24);
    wash.addColorStop(0.0, 'rgba(255, 22, 16, 0.34)');
    wash.addColorStop(0.62, 'rgba(255, 18, 12, 0.22)');
    wash.addColorStop(0.88, 'rgba(255, 14, 10, 0.14)');
    wash.addColorStop(1.0, 'rgba(255, 0, 0, 0.0)');
    ctx.fillStyle = wash;
    ctx.beginPath();
    ctx.arc(cx, cy, R + 24, 0, Math.PI * 2);
    ctx.fill();

    // 2. Clipped crisp circular square-lattice laser grid (3-pass neon glow + bright hot core!)
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.clip();

    const step = 36;
    const drawLattice = (lineWidth, strokeStyle, blur) => {
      ctx.lineWidth = lineWidth;
      ctx.strokeStyle = strokeStyle;
      ctx.shadowBlur = blur;
      ctx.shadowColor = '#ff1808';
      ctx.beginPath();
      for (let offset = -R; offset <= R; offset += step) {
        const x = cx + offset;
        ctx.moveTo(x, cy - R);
        ctx.lineTo(x, cy + R);
        const y = cy + offset;
        ctx.moveTo(cx - R, y);
        ctx.lineTo(cx + R, y);
      }
      ctx.stroke();
    };

    drawLattice(11.0, 'rgba(255, 16, 10, 0.36)', 16);
    drawLattice(4.4, 'rgba(255, 42, 24, 0.90)', 8);
    drawLattice(1.7, 'rgba(255, 195, 175, 0.96)', 0);
    ctx.restore();

    // 3. Crisp glowing circular perimeter rim
    const drawRim = (lineWidth, strokeStyle, blur) => {
      ctx.lineWidth = lineWidth;
      ctx.strokeStyle = strokeStyle;
      ctx.shadowBlur = blur;
      ctx.shadowColor = '#ff1808';
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.stroke();
    };
    drawRim(12.0, 'rgba(255, 18, 12, 0.42)', 18);
    drawRim(4.6, 'rgba(255, 44, 24, 0.95)', 8);
    drawRim(1.8, 'rgba(255, 200, 180, 0.98)', 0);

    const tex = new THREE.CanvasTexture(can);
    tex.anisotropy = 4;
    tex.needsUpdate = true;
    return tex;
  }

  // Lightweight 3D Sensor Visualizers:
  // 1) Projected Red Circular Laser Grid on ground (visible ONLY when close to ground <= 4.5m!)
  // 2) LiDAR Obstacle Laser Beam + Impact Dot (NO horizontal circle ring; beam shows ONLY near obstacles including electric poles!)
  initSensorVisuals() {
    if (this.sensorVisuals || !this.scene || typeof THREE === 'undefined') return;

    // 1. Red Circular Laser Grid projected onto ground/roof surface when close to ground!
    const gridTex = this.createGroundLaserGridTexture();
    const gridGeo = new THREE.PlaneGeometry(2.6, 2.6);
    gridGeo.rotateX(-Math.PI / 2);
    const gridMat = new THREE.MeshBasicMaterial({
      map: gridTex,
      transparent: true,
      opacity: 0.96,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      side: THREE.DoubleSide
    });
    const groundGrid = new THREE.Mesh(gridGeo, gridMat);
    groundGrid.renderOrder = 3;
    groundGrid.visible = false;

    // 2. LiDAR Obstacle Lock-On Laser Beam (NO horizontal circle — ONLY shows crisp 3D laser line when close to obstacle!)
    const beamGeo = new THREE.CylinderGeometry(0.020, 0.020, 1.0, 6, 1, true);
    const beamMat = new THREE.MeshBasicMaterial({
      color: 0xff2a1f,
      transparent: true,
      opacity: 0.92,
      depthWrite: false
    });
    const targetLine = new THREE.Mesh(beamGeo, beamMat);
    targetLine.renderOrder = 4;
    targetLine.visible = false;

    // 3. Glowing Laser Impact Point Dot directly on the detected obstacle / electric pole!
    const dotGeo = new THREE.SphereGeometry(0.085, 10, 8);
    const dotMat = new THREE.MeshBasicMaterial({ color: 0xff2a1f, transparent: true, opacity: 0.96, depthWrite: false });
    const targetDot = new THREE.Mesh(dotGeo, dotMat);
    targetDot.renderOrder = 5;
    targetDot.visible = false;

    this.scene.add(groundGrid, targetLine, targetDot);
    this.sensorVisuals = {
      groundGrid,
      targetLine,
      targetDot,
      _start: new THREE.Vector3(),
      _dir: new THREE.Vector3(),
      _up: new THREE.Vector3(0, 1, 0)
    };
  }

  updateSensorVisuals() {
    if (!this.sensorVisuals) this.initSensorVisuals();
    if (!this.sensorVisuals) return;
    const p = this.physicsParams;
    const sv = this.sensorVisuals;
    if (!p || !this.droneMesh) {
      sv.groundGrid.visible = false;
      sv.targetLine.visible = false;
      sv.targetDot.visible = false;
      return;
    }

    const px = this.position.x;
    const py = this.position.y;
    const pz = this.position.z;
    const agl = this.groundClearanceM || 0;

    // 1. Projected Red Circular Laser Grid — visible ONLY when close to the ground/surface below (<= 4.5m AGL)!
    const hasGroundSensor = Boolean(p.hasOpticalFlow || p.hasLidar);
    const nearGroundMax = 4.5;
    const showGroundGrid = hasGroundSensor && this.isArmed && agl <= nearGroundMax;
    sv.groundGrid.visible = showGroundGrid;
    if (showGroundGrid) {
      const surfY = (this.surfaceBelowY ?? 0) + 0.055;
      sv.groundGrid.position.set(px, surfY, pz);
      sv.groundGrid.rotation.y = (this.rotation && this.rotation.y) || 0;

      // Cone projection expands gently with height; vivid <= 3.1m (including 3.0m auto-takeoff!), fades smoothly out 3.1m -> 4.5m!
      const coneScale = 0.92 + agl * 0.22;
      sv.groundGrid.scale.set(coneScale, 1, coneScale);
      const heightFade = agl <= 3.1 ? 1.0 : Math.max(0, 1.0 - (agl - 3.1) / (nearGroundMax - 3.1));
      const pulse = 0.92 + 0.08 * Math.sin(performance.now() * 0.008);
      sv.groundGrid.material.opacity = (this.isLanded ? 0.72 : 0.96) * heightFade * pulse;
    }

    // 2. LiDAR Obstacle Laser Line ONLY (no circle ring!) — appears ONLY when approaching obstacles/trees/electric poles (<= 9.0m)
    const showTarget = Boolean(
      p.hasLidar && !this.isLanded && this.isArmed &&
      this.lidarNearestDist !== null && this.lidarNearestDist <= 9.0 && this.lidarNearestPoint
    );
    sv.targetLine.visible = showTarget;
    sv.targetDot.visible = showTarget;
    if (showTarget) {
      sv._start.set(px, py + 0.14, pz);
      sv._dir.subVectors(this.lidarNearestPoint, sv._start);
      const beamLen = sv._dir.length();
      if (beamLen > 0.05) {
        sv.targetLine.position.lerpVectors(sv._start, this.lidarNearestPoint, 0.5);
        sv._dir.multiplyScalar(1 / beamLen);
        sv.targetLine.quaternion.setFromUnitVectors(sv._up, sv._dir);
        const isBrake = this.lidarZone === 'brake';
        const beamRadiusScale = isBrake ? 1.35 : 0.95;
        sv.targetLine.scale.set(beamRadiusScale, beamLen, beamRadiusScale);

        const laserHex = isBrake ? 0xff2a1f : 0xf59e0b;
        sv.targetLine.material.color.setHex(laserHex);
        sv.targetLine.material.opacity = isBrake ? 0.96 : 0.85;
        sv.targetDot.material.color.setHex(laserHex);
        sv.targetDot.position.copy(this.lidarNearestPoint);
        const dotPulse = isBrake ? (1.15 + 0.35 * Math.abs(Math.sin(performance.now() * 0.016))) : 0.95;
        sv.targetDot.scale.setScalar(dotPulse);
      } else {
        sv.targetLine.visible = false;
      }
    }
  }

  // =========================================================================
  // AUTONOMOUS SENSOR/SIGNAL-LOSS RETURN PATH PLANNER (Breadcrumb Backtrack + 2D A* Obstacle Avoidance!)
  // =========================================================================

  recordFlightBreadcrumb() {
    if (!this.isArmed || this.isLanded || this.isRTHMode || this.isCrashed) return;
    if (!this.flightPathHistory) this.flightPathHistory = [];
    const px = this.position.x, py = this.position.y, pz = this.position.z;
    const last = this.flightPathHistory[this.flightPathHistory.length - 1];
    if (!last || Math.hypot(px - last.x, py - last.y, pz - last.z) >= 3.5) {
      this.flightPathHistory.push({ x: px, y: py, z: pz });
      if (this.flightPathHistory.length > 140) {
        // Keep index 0 (Home start) and compact oldest breadcrumb cleanly
        this.flightPathHistory.splice(1, 1);
      }
    }
  }

  // Check whether horizontal point (wx, wz) at altitude flyY collides with any tall obstacle
  isReturnPointBlocked(wx, wz, flyY, pad = 4.4) {
    if (this.customMapRoot || !this.colliders || this.colliders.length === 0) return false;
    if (Math.hypot(wx, wz) > 286) return true;
    for (let i = 0; i < this.colliders.length; i++) {
      const c = this.colliders[i];
      if (c.type === 'box') {
        if (flyY - 1.4 > c.maxY || flyY + 1.2 < c.minY) continue;
        if (wx >= c.minX - pad && wx <= c.maxX + pad && wz >= c.minZ - pad && wz <= c.maxZ + pad) {
          return true;
        }
      } else if (c.type === 'cyl' && (c.radius >= 0.75 || c.tag === 'power_pole')) {
        if (flyY - 1.4 > c.maxY || flyY + 1.2 < c.minY) continue;
        if (Math.hypot(wx - c.x, wz - c.z) < c.radius + pad) {
          return true;
        }
      }
    }
    return false;
  }

  // Verify straight corridor from (x1, z1) -> (x2, z2) at altitude flyY is obstacle-free (for LOS string-pulling!)
  hasClearReturnCorridor(x1, z1, x2, z2, flyY, pad = 4.2) {
    const dist = Math.hypot(x2 - x1, z2 - z1);
    const steps = Math.max(2, Math.ceil(dist / 3.0));
    for (let i = 1; i < steps; i++) {
      const t = i / steps;
      const sx = x1 + (x2 - x1) * t;
      const sz = z1 + (z2 - z1) * t;
      if (this.isReturnPointBlocked(sx, sz, flyY, pad)) return false;
    }
    return true;
  }

  // Fast bounded 2D Grid A* Pathfinding from (startX, startZ) -> Home (0, 0) avoiding all tall obstacles at cruiseY!
  runAStarReturnGrid(startX, startZ, cruiseY) {
    const cell = 8.0;
    const toCell = (v) => Math.round(v / cell);
    const sGx = toCell(startX), sGz = toCell(startZ);
    const gGx = 0, gGz = 0;

    if (sGx === gGx && sGz === gGz) {
      return [{ x: 0, z: 0 }];
    }
    if (this.hasClearReturnCorridor(startX, startZ, 0, 0, cruiseY, 4.4)) {
      return [{ x: 0, z: 0 }];
    }

    const keyOf = (gx, gz) => `${gx},${gz}`;
    const startKey = keyOf(sGx, sGz);
    const goalKey = keyOf(gGx, gGz);

    const open = [{ gx: sGx, gz: sGz, key: startKey, g: 0, f: Math.hypot(sGx - gGx, sGz - gGz) }];
    const gScore = new Map([[startKey, 0]]);
    const cameFrom = new Map();
    const closed = new Set();
    const blockedCache = new Map();

    const checkBlocked = (gx, gz) => {
      if (gx === gGx && gz === gGz) return false;
      if (gx === sGx && gz === sGz) return false;
      const k = keyOf(gx, gz);
      if (blockedCache.has(k)) return blockedCache.get(k);
      const b = this.isReturnPointBlocked(gx * cell, gz * cell, cruiseY, 4.4);
      blockedCache.set(k, b);
      return b;
    };

    const dirs = [
      [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
      [1, 1, 1.414], [1, -1, 1.414], [-1, 1, 1.414], [-1, -1, 1.414]
    ];

    let found = false;
    let bestFallbackKey = startKey;
    let bestFallbackH = Math.hypot(sGx, sGz);
    let iterations = 0;

    while (open.length > 0 && iterations < 1200) {
      iterations++;
      let bestIdx = 0;
      for (let i = 1; i < open.length; i++) {
        if (open[i].f < open[bestIdx].f) bestIdx = i;
      }
      const cur = open.splice(bestIdx, 1)[0];
      if (closed.has(cur.key)) continue;
      closed.add(cur.key);

      if (cur.key === goalKey) {
        found = true;
        break;
      }

      const hCur = Math.hypot(cur.gx - gGx, cur.gz - gGz);
      if (hCur < bestFallbackH) {
        bestFallbackH = hCur;
        bestFallbackKey = cur.key;
      }

      for (let d = 0; d < dirs.length; d++) {
        const nx = cur.gx + dirs[d][0];
        const nz = cur.gz + dirs[d][1];
        if (Math.abs(nx) > 36 || Math.abs(nz) > 36) continue;
        const nKey = keyOf(nx, nz);
        if (closed.has(nKey)) continue;
        if (checkBlocked(nx, nz)) continue;

        const tentG = cur.g + dirs[d][2];
        const prevG = gScore.has(nKey) ? gScore.get(nKey) : Infinity;
        if (tentG < prevG) {
          gScore.set(nKey, tentG);
          cameFrom.set(nKey, cur.key);
          const h = Math.hypot(nx - gGx, nz - gGz);
          open.push({ gx: nx, gz: nz, key: nKey, g: tentG, f: tentG + h * 1.08 });
        }
      }
    }

    // Reconstruct raw grid path
    const rawPts = [];
    let traceKey = found ? goalKey : bestFallbackKey;
    while (traceKey && traceKey !== startKey) {
      const [gx, gz] = traceKey.split(',').map(Number);
      rawPts.push({ x: gx * cell, z: gz * cell });
      traceKey = cameFrom.get(traceKey);
    }
    rawPts.reverse();
    if (rawPts.length === 0 || Math.hypot(rawPts[rawPts.length - 1].x, rawPts[rawPts.length - 1].z) > 2) {
      rawPts.push({ x: 0, z: 0 });
    }

    // Line-of-Sight String-Pulling Smoothing (produces sleek minimal waypoints around obstacles!)
    const smoothed = [];
    let anchorX = startX, anchorZ = startZ;
    let idx = 0;
    while (idx < rawPts.length) {
      let farthest = idx;
      for (let j = rawPts.length - 1; j > idx; j--) {
        if (this.hasClearReturnCorridor(anchorX, anchorZ, rawPts[j].x, rawPts[j].z, cruiseY, 4.2)) {
          farthest = j;
          break;
        }
      }
      smoothed.push(rawPts[farthest]);
      anchorX = rawPts[farthest].x;
      anchorZ = rawPts[farthest].z;
      idx = farthest + 1;
    }
    return smoothed;
  }

  // Hybrid Autonomous Return Path Planner: Combines IMU Breadcrumb Escape + Obstacle-Aware 2D/3D A* Pathfinding!
  computeSmartReturnPath(preferBacktrack = false) {
    const p = this.physicsParams || this.computeDronePhysicsParams();
    const clearance = p.clearance || 0.48;
    const px = this.position.x, py = this.position.y, pz = this.position.z;
    const homeGroundY = (typeof this.getTerrainHeight === 'function') ? this.getTerrainHeight(0, 0) : 0;
    const homePadY = homeGroundY + clearance;
    const distHome = Math.hypot(px, pz);

    const waypoints = [];
    let usedBacktrack = false;
    let usedAStar = false;

    const isInsideHangar = (x, z, y) => (x >= 56.5 && x <= 75.5 && z >= -24.8 && z <= 0.8 && y <= 9.2);

    // Stage 1: Safe Exit / Odometry Breadcrumb Backtracking (essential inside Steel Hangar OR when GPS/Sensor is lost!)
    let planStartX = px;
    let planStartY = py;
    let planStartZ = pz;

    const hist = this.flightPathHistory || [];
    const needIndoorEscape = this.insideMetalRoof || isInsideHangar(px, pz, py);
    const fullBacktrackMode = Boolean(preferBacktrack && (!p.hasGps || this.sensorLostSim) && hist.length >= 2);

    if (needIndoorEscape) {
      usedBacktrack = true;
      // Walk backward through recorded flight breadcrumbs until exiting hangar roof safely!
      let escaped = false;
      for (let i = hist.length - 1; i >= 0; i--) {
        const bp = hist[i];
        if (!isInsideHangar(bp.x, bp.z, bp.y)) {
          waypoints.push({ x: bp.x, y: Math.min(4.2, Math.max(2.6, bp.y)), z: bp.z, kind: 'backtrack' });
          planStartX = bp.x;
          planStartY = Math.min(4.2, Math.max(2.6, bp.y));
          planStartZ = bp.z;
          escaped = true;
          break;
        } else if (waypoints.length === 0 || Math.hypot(bp.x - waypoints[waypoints.length - 1].x, bp.z - waypoints[waypoints.length - 1].z) >= 5.0) {
          waypoints.push({ x: bp.x, y: Math.min(4.0, Math.max(2.4, py)), z: bp.z, kind: 'backtrack' });
        }
      }
      if (!escaped) {
        // Direct safe exit out the open West hangar doorway (x=50, z=-12)
        waypoints.push({ x: 50.0, y: Math.min(4.0, Math.max(2.5, py)), z: -12.0, kind: 'backtrack' });
        planStartX = 50.0;
        planStartY = Math.min(4.0, Math.max(2.5, py));
        planStartZ = -12.0;
      }
    } else if (fullBacktrackMode && distHome > 6.0) {
      // Sensor/GPS Lost mid-flight: retrace recent IMU odometry trail for safe immediate escape, then bridge via A* to Home!
      usedBacktrack = true;
      let lastAdded = { x: px, y: py, z: pz };
      const maxBackPts = Math.min(5, hist.length);
      let added = 0;
      for (let i = hist.length - 1; i >= 0 && added < maxBackPts; i--) {
        const bp = hist[i];
        const stepDist = Math.hypot(bp.x - lastAdded.x, bp.z - lastAdded.z);
        if (stepDist >= 7.5 || i === 0) {
          const safeY = Math.max(this.getTerrainHeight(bp.x, bp.z) + 3.5, bp.y);
          waypoints.push({ x: bp.x, y: safeY, z: bp.z, kind: 'backtrack' });
          lastAdded = { x: bp.x, y: safeY, z: bp.z };
          planStartX = bp.x;
          planStartY = safeY;
          planStartZ = bp.z;
          added++;
          if (Math.hypot(bp.x, bp.z) < 6.0) break;
        }
      }
    }

    // Stage 2: Obstacle-Aware 2D/3D A* Pathfinding from escape point to Home Helipad (0, 0)
    const remDist = Math.hypot(planStartX, planStartZ);
    const maxSafeCruiseY = homePadY + Math.max(8.0, (p.maxAltitudeLimitM || 90) - 3.0);
    const cruiseY = Math.min(maxSafeCruiseY, Math.max(12.5, Math.min(22.0, Math.max(planStartY + 2.5, homePadY + 12.5))));
    this.rthCruiseAlt = cruiseY;

    if (remDist > 4.0) {
      // Climb clear of low obstacles before horizontal transit (unless already high enough)
      if (planStartY < cruiseY - 1.5 && !isInsideHangar(planStartX, planStartZ, planStartY)) {
        waypoints.push({ x: planStartX, y: cruiseY, z: planStartZ, kind: 'climb' });
      }
      const gridPts = this.runAStarReturnGrid(planStartX, planStartZ, cruiseY);
      usedAStar = true;
      for (let i = 0; i < gridPts.length; i++) {
        const gp = gridPts[i];
        const minSafeY = ((typeof this.getTerrainHeight === 'function') ? this.getTerrainHeight(gp.x, gp.z) : 0) + 4.5;
        const isLast = i === gridPts.length - 1;
        waypoints.push({
          x: gp.x,
          y: isLast ? Math.max(homePadY + 6.0, Math.min(cruiseY, homePadY + 13.0)) : Math.max(cruiseY, minSafeY),
          z: gp.z,
          kind: isLast ? 'home' : 'astar'
        });
      }
    } else {
      waypoints.push({ x: 0, y: Math.max(homePadY + 4.0, Math.min(py, homePadY + 12.0)), z: 0, kind: 'home' });
    }

    // Compute total 3D planned route length
    let totalDist = 0;
    let prevX = px, prevY = py, prevZ = pz;
    for (let i = 0; i < waypoints.length; i++) {
      const w = waypoints[i];
      totalDist += Math.hypot(w.x - prevX, w.y - prevY, w.z - prevZ);
      prevX = w.x; prevY = w.y; prevZ = w.z;
    }

    this.rthWaypoints = waypoints;
    this.rthWaypointIdx = 0;
    this.rthTotalPathDist = Math.round(totalDist);
    this.rthAlgoLabel = (usedBacktrack && usedAStar)
      ? 'Breadcrumb ย้อนรอย + A* หลบสิ่งกีดขวาง'
      : usedBacktrack
        ? 'IMU Breadcrumb ย้อนรอยจุดเริ่มต้น'
        : 'A* คำนวณเส้นทางหลบสิ่งกีดขวาง';
    return waypoints;
  }

  triggerSmartAutoReturn(reason = 'สัญญาณขาดหาย', isFailsafe = true, simulateLoss = false) {
    this.initAudio();
    this.isRTHMode = true;
    this.isHoldMode = true;
    this.isFailsafeReturn = Boolean(isFailsafe);
    this.sensorLostSim = Boolean(simulateLoss);
    this.signalLossTimer = 0;
    this.rthPhase = 'PATH_RETURN';

    const pts = this.computeSmartReturnPath(Boolean(isFailsafe || simulateLoss));
    this.playMelody(
      isFailsafe
        ? [[980, 0.00, 0.07], [740, 0.09, 0.07], [980, 0.18, 0.12]]
        : [[660, 0.00, 0.07], [880, 0.09, 0.11]]
    );
    this.setHudAlert(
      `🧭 ${reason} — คำนวณเส้นทางกลับจุดเริ่มต้นอัตโนมัติ (${pts.length} จุด · ${this.rthTotalPathDist} ม. · ${this.rthAlgoLabel})`,
      4600
    );
    return pts;
  }

  cancelSmartAutoReturn(msg = 'ยกเลิกเส้นทางกลับอัตโนมัติ — นักบินรับช่วงควบคุม') {
    this.isRTHMode = false;
    this.rthPhase = 'IDLE';
    this.isFailsafeReturn = false;
    this.sensorLostSim = false;
    this.signalLossTimer = 0;
    this.rthWaypoints = [];
    this.rthWaypointIdx = 0;
    this.holdAltitude = this.position.y;
    this.holdTargetPos.copy(this.position);
    this.holdTargetLocked = false;
    this.clearRthPathVisuals();
    this.playTone(520, 0.12);
    this.setHudAlert(msg, 2400);
  }

  // Interactive Demo Button (KeyP / #btn-hud-failsafe): Simulates sudden Sensor/Signal Loss mid-flight -> Auto-Path Return!
  toggleSimulatedSensorLoss() {
    this.initAudio();
    if (this.sensorLostSim || (this.isRTHMode && this.isFailsafeReturn)) {
      this.cancelSmartAutoReturn('📡 กู้คืนสัญญาณและเซนเซอร์สำเร็จ — นักบินกลับมาควบคุมปกติ');
      return false;
    }
    if (!this.isArmed || this.isLanded) {
      this.playTone(260, 0.14);
      this.setHudAlert('บินขึ้นจากพื้นก่อนเพื่อทดสอบจำลองเซนเซอร์/สัญญาณหายกลางทาง (กด T บินขึ้น)', 2800);
      return false;
    }
    this.triggerSmartAutoReturn('จำลองเซนเซอร์/สัญญาณหายกลางทาง!', true, true);
    return true;
  }

  // 3D Glowing Calculated Auto-Return Path Polyline + Waypoint Beacons + Home Pad Landing Beam!
  initRthPathVisuals() {
    if (this.rthPathVisuals || !this.scene || typeof THREE === 'undefined') return;
    const maxPts = 24;
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(maxPts * 3), 3));
    lineGeo.setDrawRange(0, 0);
    const lineMat = new THREE.LineBasicMaterial({ color: 0xff6a00, transparent: true, opacity: 0.92, depthWrite: false });
    const pathLine = new THREE.Line(lineGeo, lineMat);
    pathLine.frustumCulled = false;
    pathLine.visible = false;

    const markers = [];
    const sphereGeo = new THREE.SphereGeometry(0.42, 12, 10);
    for (let i = 0; i < 16; i++) {
      const m = new THREE.Mesh(
        sphereGeo,
        new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.82, depthWrite: false })
      );
      m.visible = false;
      this.scene.add(m);
      markers.push(m);
    }

    // Translucent Home Helipad Landing Beacon Column at (0, 0)
    const homeBeamGeo = new THREE.CylinderGeometry(0.32, 0.72, 24, 16, 1, true);
    const homeBeamMat = new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.24, side: THREE.DoubleSide, depthWrite: false });
    const homeBeam = new THREE.Mesh(homeBeamGeo, homeBeamMat);
    homeBeam.position.set(0, 12, 0);
    homeBeam.visible = false;

    this.scene.add(pathLine, homeBeam);
    this.rthPathVisuals = { pathLine, markers, homeBeam, maxPts };
  }

  clearRthPathVisuals() {
    if (!this.rthPathVisuals) return;
    this.rthPathVisuals.pathLine.visible = false;
    this.rthPathVisuals.homeBeam.visible = false;
    for (let i = 0; i < this.rthPathVisuals.markers.length; i++) {
      this.rthPathVisuals.markers[i].visible = false;
    }
  }

  updateRthPathVisuals() {
    if (!this.isRTHMode || !this.rthWaypoints || this.rthWaypoints.length === 0) {
      if (this.rthPathVisuals) this.clearRthPathVisuals();
      return;
    }
    if (!this.rthPathVisuals) this.initRthPathVisuals();
    if (!this.rthPathVisuals) return;

    const rv = this.rthPathVisuals;
    const isFail = Boolean(this.isFailsafeReturn || this.sensorLostSim);
    rv.pathLine.material.color.setHex(isFail ? 0xff6a00 : 0x38bdf8);

    const posArr = rv.pathLine.geometry.attributes.position.array;
    let count = 0;
    const pushPt = (x, y, z) => {
      if (count >= rv.maxPts) return;
      posArr[count * 3] = x;
      posArr[count * 3 + 1] = y;
      posArr[count * 3 + 2] = z;
      count++;
    };

    // Start polyline from drone's live position -> all remaining waypoints -> Home Helipad (0, homeY, 0)
    pushPt(this.position.x, this.position.y, this.position.z);
    let mIdx = 0;
    const pulse = 1.0 + 0.18 * Math.sin(performance.now() * 0.008);
    for (let i = this.rthWaypointIdx; i < this.rthWaypoints.length; i++) {
      const wp = this.rthWaypoints[i];
      pushPt(wp.x, wp.y, wp.z);
      if (mIdx < rv.markers.length) {
        const mk = rv.markers[mIdx++];
        mk.visible = true;
        mk.position.set(wp.x, wp.y, wp.z);
        mk.scale.setScalar(i === this.rthWaypointIdx ? pulse * 1.25 : 0.85);
        mk.material.color.setHex(
          wp.kind === 'home' ? 0x22c55e : wp.kind === 'backtrack' ? 0xf59e0b : (isFail ? 0xff6a00 : 0x38bdf8)
        );
      }
    }
    const homeY = ((typeof this.getTerrainHeight === 'function') ? this.getTerrainHeight(0, 0) : 0) + 0.35;
    pushPt(0, homeY, 0);

    for (let i = mIdx; i < rv.markers.length; i++) {
      rv.markers[i].visible = false;
    }

    rv.pathLine.geometry.setDrawRange(0, count);
    rv.pathLine.geometry.attributes.position.needsUpdate = true;
    rv.pathLine.visible = count >= 2;
    rv.homeBeam.visible = true;
    rv.homeBeam.position.set(0, homeY + 12, 0);
  }

  // Real-World RF Signal Obstruction, Foliage Attenuation, Steel Faraday Cage, NLOS Shadowing & EMI Simulator!
  evaluateSignalEnvironment(p, delta = 0.016) {
    const px = this.position.x;
    const py = this.position.y;
    const pz = this.position.z;
    const pilotY = this.getTerrainHeight(0, 0) + 1.8;

    const distFromHome = Math.hypot(px, pz);
    const rxRangeFrac = distFromHome / Math.max(90, p.rxRangeMeters || 340);
    const distAtten = Math.max(0, Math.min(0.96, (rxRangeFrac - 0.14) * 0.86));

    // Hardware-authentic RF penetration & EMI immunity by installed RX receiver!
    const rxType = p.rxType || 'elrs';
    const rxPenFactor = rxType === '24g' ? 1.24 : rxType === 'dual' ? 0.62 : 0.88;
    const emiFactor = rxType === '24g' ? 1.28 : rxType === 'dual' ? 0.45 : 0.85;

    let treeAtten = 0;
    let hangarAtten = 0;
    let buildingAtten = 0;
    let emi5gAtten = 0;
    let emiRadarAtten = 0;
    let terrainAtten = 0;
    let insideMetalRoof = false;

    if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
      const dxRay = px;
      const dyRay = py - pilotY;
      const dzRay = pz;
      const rayHorizSq = dxRay * dxRay + dzRay * dzRay;

      // Helper: fast 3D ray-vs-AABB slab test along Pilot(0, pilotY, 0) -> Drone(px, py, pz)
      const rayHitsBox = (c) => {
        let tMin = 0.03;
        let tMax = 0.985;
        if (Math.abs(dxRay) < 1e-6) {
          if (0 < c.minX || 0 > c.maxX) return false;
        } else {
          const inv = 1 / dxRay;
          let t1 = c.minX * inv;
          let t2 = c.maxX * inv;
          if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
          if (t1 > tMin) tMin = t1;
          if (t2 < tMax) tMax = t2;
          if (tMin > tMax) return false;
        }
        if (Math.abs(dyRay) < 1e-6) {
          if (pilotY < c.minY || pilotY > c.maxY) return false;
        } else {
          const inv = 1 / dyRay;
          let t1 = (c.minY - pilotY) * inv;
          let t2 = (c.maxY - pilotY) * inv;
          if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
          if (t1 > tMin) tMin = t1;
          if (t2 < tMax) tMax = t2;
          if (tMin > tMax) return false;
        }
        if (Math.abs(dzRay) < 1e-6) {
          if (0 < c.minZ || 0 > c.maxZ) return false;
        } else {
          const inv = 1 / dzRay;
          let t1 = c.minZ * inv;
          let t2 = c.maxZ * inv;
          if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
          if (t1 > tMin) tMin = t1;
          if (t2 < tMax) tMax = t2;
          if (tMin > tMax) return false;
        }
        return tMax >= tMin;
      };

      for (let i = 0; i < this.colliders.length; i++) {
        const c = this.colliders[i];
        const tag = c.tag || 'building';

        if (c.type === 'cyl') {
          if (tag === 'tree') {
            // 1. Proximity / Under-Canopy Foliage Attenuation (water in leaves absorbs 2.4GHz / RF strongly!)
            if (py <= c.maxY + 2.5) {
              const dEdge = Math.hypot(px - c.x, pz - c.z) - c.radius;
              if (dEdge < 5.2) {
                const prox = Math.max(0, 1 - Math.max(0, dEdge) / 5.2);
                const canopyBoost = (dEdge < 1.8 && py <= c.maxY + 0.9) ? 0.22 * prox : 0;
                treeAtten += 0.42 * prox + canopyBoost;
              }
            }
            // 2. NLOS Ray passing through tree foliage between pilot & drone
            if (rayHorizSq > 25) {
              const t = (c.x * dxRay + c.z * dzRay) / rayHorizSq;
              if (t > 0.04 && t < 0.98) {
                const qx = t * dxRay;
                const qz = t * dzRay;
                const qy = pilotY + t * dyRay;
                const rRf = c.radius * 1.32;
                const dh = Math.hypot(c.x - qx, c.z - qz);
                if (dh < rRf && qy >= c.minY - 0.5 && qy <= c.maxY + 0.8) {
                  const depth = 1 - dh / rRf;
                  treeAtten += 0.16 * (0.4 + 0.6 * depth);
                }
              }
            }
          } else if (tag === 'atc_tower' || tag === 'building' || tag === 'wind_turbine') {
            // Solid concrete/steel cylinder NLOS shadowing + close multipath
            if (py <= c.maxY + 1.5) {
              const dEdge = Math.hypot(px - c.x, pz - c.z) - c.radius;
              if (dEdge < 3.4) {
                buildingAtten += 0.28 * (1 - Math.max(0, dEdge) / 3.4);
              }
            }
            if (rayHorizSq > 25) {
              const t = (c.x * dxRay + c.z * dzRay) / rayHorizSq;
              if (t > 0.04 && t < 0.98) {
                const qx = t * dxRay;
                const qz = t * dzRay;
                const qy = pilotY + t * dyRay;
                const rRf = c.radius * 1.08;
                const dh = Math.hypot(c.x - qx, c.z - qz);
                if (dh < rRf && qy >= c.minY - 0.5 && qy <= c.maxY + 0.8) {
                  buildingAtten += 0.44;
                }
              }
            }
          }
        } else if (c.type === 'box') {
          // Box structures (Metal Hangar, Houses, Barns, Ruins, Bridges)
          const cx = Math.max(c.minX, Math.min(c.maxX, px));
          const cy = Math.max(c.minY, Math.min(c.maxY, py));
          const cz = Math.max(c.minZ, Math.min(c.maxZ, pz));
          const dBox = Math.hypot(px - cx, py - cy, pz - cz);

          if (tag === 'hangar') {
            if (dBox < 4.0) {
              hangarAtten = Math.max(hangarAtten, 0.46 * (1 - dBox / 4.0));
            }
            if (rayHorizSq > 25 && rayHitsBox(c)) {
              hangarAtten += 0.52;
            }
          } else {
            if (dBox < 3.4 && py <= c.maxY + 1.2) {
              buildingAtten += 0.28 * (1 - dBox / 3.4);
            }
            if (rayHorizSq > 25 && rayHitsBox(c)) {
              buildingAtten += 0.44;
            }
          }
        }
      }

      // 3. Steel Airfield Hangar Faraday-Cage Bay (x: 57.5..74.8, z: -24.2..0.2, y < 8.8)
      if (px >= 57.5 && px <= 74.8 && pz >= -24.2 && pz <= 0.2 && py <= 8.8) {
        insideMetalRoof = true;
        hangarAtten = Math.max(hangarAtten, 0.84);
      } else if (px >= 53.5 && px <= 79.0 && pz >= -27.5 && pz <= 3.0 && py <= 10.2) {
        const edgeX = Math.max(0, 57.5 - px, px - 74.8);
        const edgeZ = Math.max(0, -24.2 - pz, pz - 0.2);
        const dBay = Math.hypot(edgeX, edgeZ);
        hangarAtten = Math.max(hangarAtten, Math.max(0, 1 - dBay / 4.5) * 0.56);
      }

      // 4. High-Power RF / Electromagnetic Interference (EMI) Zones
      // 4a. 45m 5G Telecom Tower at (75, -70)
      const d5g = Math.hypot(px - 75, pz + 70);
      if (d5g < 22 && py >= 2 && py <= 56) {
        const heightGain = py > 24 ? 0.82 : 0.62;
        emi5gAtten = Math.max(emi5gAtten, (1 - d5g / 22) * heightGain);
      }
      // 4b. Solar Farm Inverters (172, 118) & Grid Control Substation (134, 124)
      const dSolar = Math.hypot(px - 172, pz - 118);
      if (dSolar < 20 && py < 14) {
        emi5gAtten = Math.max(emi5gAtten, (1 - dSolar / 20) * 0.48);
      }
      const dGrid = Math.hypot(px - 134.2, pz - 124.1);
      if (dGrid < 11 && py < 11) {
        emi5gAtten = Math.max(emi5gAtten, (1 - dGrid / 11) * 0.46);
      }
      // 4c. ATC Control Tower Rotating Radar Dish at (-16, -34, y: 12..31)
      const dAtc = Math.hypot(px + 16, pz + 34);
      if (dAtc < 15 && py >= 12 && py <= 31) {
        emiRadarAtten = (1 - dAtc / 15) * 0.64;
      }

      // 5. Terrain / Hill Fresnel-Zone NLOS Shadowing (sample 6 points along Pilot -> Drone ray)
      if (distFromHome > 26) {
        for (let k = 1; k <= 6; k++) {
          const t = k / 7;
          const sx = px * t;
          const sz = pz * t;
          const sy = pilotY + (py - pilotY) * t;
          const terrainY = this.getTerrainHeight(sx, sz);
          if (terrainY > sy + 0.75) {
            terrainAtten += Math.min(0.32, (terrainY - sy) * 0.08);
          }
        }
      }
    }

    const effTree = Math.min(0.84, Math.min(0.86, treeAtten) * rxPenFactor);
    const effHangar = Math.min(0.90, Math.min(0.88, hangarAtten) * Math.max(0.84, rxPenFactor));
    const effBuild = Math.min(0.82, Math.min(0.80, buildingAtten) * rxPenFactor);
    const effEmi5g = Math.min(0.82, emi5gAtten * emiFactor);
    const effEmiRadar = Math.min(0.72, emiRadarAtten * emiFactor);
    const effTerrain = Math.min(0.84, Math.min(0.78, terrainAtten) * Math.max(0.85, rxPenFactor));

    const envObstruction = 1 - (
      (1 - effTree) *
      (1 - effHangar) *
      (1 - effBuild) *
      (1 - effEmi5g) *
      (1 - effEmiRadar) *
      (1 - effTerrain)
    );

    // Determine dominant RF obstruction source for clear Thai HUD explanation
    const candidates = [
      { key: 'hangar', val: effHangar },
      { key: 'tree', val: effTree },
      { key: 'emi_5g', val: effEmi5g },
      { key: 'emi_radar', val: effEmiRadar },
      { key: 'building', val: effBuild },
      { key: 'terrain', val: effTerrain }
    ];
    let topSource = 'clear';
    let topVal = 0;
    for (let i = 0; i < candidates.length; i++) {
      if (candidates[i].val > topVal) {
        topVal = candidates[i].val;
        topSource = candidates[i].key;
      }
    }
    if (topVal < 0.20) {
      topSource = distAtten >= 0.45 ? 'range' : 'clear';
    }

    const jitter = envObstruction > 0.14
      ? envObstruction * 0.028 * (Math.sin(this.simTime * 11.3) + 0.6 * Math.cos(this.simTime * 19.7))
      : 0;
    const baseSignal = Math.max(0, 1.0 - distAtten) * (1.0 - envObstruction * 0.86);
    const targetRssi = Math.max(4, Math.min(100, (baseSignal + jitter) * 100));

    const dtClamp = Math.min(0.05, Math.max(0.001, delta));
    const rssiAlpha = 1 - Math.exp(-5.5 * dtClamp);
    const obsAlpha = 1 - Math.exp(-5.0 * dtClamp);
    this.smoothedRssi = (this.smoothedRssi ?? 100) + (targetRssi - (this.smoothedRssi ?? 100)) * rssiAlpha;
    this.signalObstruction = (this.signalObstruction || 0) + (envObstruction - (this.signalObstruction || 0)) * obsAlpha;
    this.rssiPercent = Math.round(Math.max(0, Math.min(100, this.smoothedRssi)));
    this.signalSource = topSource;
    this.insideMetalRoof = insideMetalRoof;
    this.gpsMultipathFactor = 1.0 + this.signalObstruction * (insideMetalRoof ? 3.5 : 2.2);

    // Real-World Wind Shelter / Leeward Wake (calmer inside metal hangar or tucked behind trees/buildings!)
    let targetShelter = 1.0;
    if (insideMetalRoof) {
      targetShelter = 0.15; // 85% calmer indoors inside the hangar bay!
    } else {
      const shelterDrag = Math.min(0.45, Math.min(0.32, treeAtten * 0.42) + Math.min(0.30, buildingAtten * 0.38) + Math.min(0.34, hangarAtten * 0.45));
      targetShelter = Math.max(0.52, 1.0 - shelterDrag);
    }
    this.windShelterFactor = (this.windShelterFactor ?? 1.0) + (targetShelter - (this.windShelterFactor ?? 1.0)) * (1 - Math.exp(-3.5 * dtClamp));

    if (this.sensorLostSim) {
      this.rssiPercent = 0;
      this.signalState = 'critical';
      this.signalSource = 'sensor_lost';
      this.signalObstruction = 1.0;
      this.gpsMultipathFactor = 3.5;
      this._lastSignalState = 'critical';
      return { distFromHome, rxRangeFrac, envObstruction: 1.0, insideMetalRoof };
    }

    // Hysteresis prevents rapid blinking right on the threshold edge
    const prevState = this.signalState || 'good';
    let nextState = 'good';
    if (this.rssiPercent <= 34 || this.signalObstruction >= 0.56 || (prevState === 'critical' && (this.rssiPercent <= 38 || this.signalObstruction >= 0.50))) {
      nextState = 'critical';
    } else if (this.rssiPercent <= 64 || this.signalObstruction >= 0.25 || (prevState === 'weak' && (this.rssiPercent <= 68 || this.signalObstruction >= 0.20))) {
      nextState = 'weak';
    }
    this.signalState = nextState;

    // Subtle telemetry warning chirp on entering weak/critical RF dead zones while armed
    if (this.isArmed && !this.isCrashed) {
      const nowMs = performance.now();
      if (nextState === 'critical' && (prevState !== 'critical' || nowMs - (this._lastSignalBeepMs || 0) > 3600)) {
        this._lastSignalBeepMs = nowMs;
        this.playMelody([[940, 0.00, 0.05], [700, 0.07, 0.05], [940, 0.14, 0.07]]);
      } else if (nextState === 'weak' && prevState === 'good' && nowMs - (this._lastSignalBeepMs || 0) > 2800) {
        this._lastSignalBeepMs = nowMs;
        this.playMelody([[760, 0.00, 0.06], [620, 0.08, 0.08]]);
      }
    }
    this._lastSignalState = nextState;

    return { distFromHome, rxRangeFrac, envObstruction, insideMetalRoof };
  }

  updatePhysics(delta) {
    if (!this.droneMesh) return;
    const p = this.physicsParams || this.computeDronePhysicsParams();
    // Continuously evaluate real-world RF obstruction (trees, metal hangar, buildings, 5G EMI, terrain) even while landed!
    const rfInfo = this.evaluateSignalEnvironment(p, delta);
    const clearance = p.clearance || 0.48;
    // Live Dynamic Mass & Parallel-Axis Inertia (decreases in real time as fertilizer is sprayed or cargo is dropped!)
    const curPayloadKg = Math.max(0, p.payloadTankKg * (this.payloadPercent / 100));
    const curMass = Math.max(0.25, p.dryMass + curPayloadKg);
    const curIxx = p.IxxDry + curPayloadKg * (0.008 + 0.040);
    const curIzz = p.IzzDry + curPayloadKg * (0.008 + 0.040);
    const curIyy = p.IyyDry + curPayloadKg * 0.012;
    const massRatio = curMass / Math.max(0.30, p.refMassKg || 1.05);
    // Heavier mass increases touchdown kinetic energy (0.5*m*v^2), requiring softer landings unless tall shock gear is fitted!
    const curVertCrashSpeed = p.vertCrashSpeed * Math.max(0.72, Math.min(1.22, Math.pow(1.2 / curMass, 0.22)));

    // Sub-stepped integration (dt <= 0.008s) for frame-rate-independent PID stability & collision precision!
    const subSteps = Math.max(2, Math.min(6, Math.ceil(delta / 0.008)));
    const dt = delta / subSteps;

    // Sample terrain once at start of frame (plus 4 skid points on custom GLB terrain)
    const sampleMinGroundY = (px, pz) => {
      let ty = this.getTerrainHeight(px, pz);
      if (this.customMapRoot && this.solidTerrainMeshes && this.solidTerrainMeshes.length > 0) {
        const padR = 0.55;
        ty = Math.max(
          ty,
          this.getTerrainHeight(px + padR, pz),
          this.getTerrainHeight(px - padR, pz),
          this.getTerrainHeight(px, pz + padR),
          this.getTerrainHeight(px, pz - padR)
        );
      }
      return ty + clearance;
    };

    // PRE_ARMING SELF-CHECK SEQUENCE (~0.72s: authentic ESC startup blip + Gyro/Battery diagnostics -> ARMED_IDLE / AUTO_TAKEOFF!)
    if (this.powerState === 'PRE_ARMING') {
      this.preArmTimer -= delta;
      // Subtle brushless ESC beep motor-test twitch pulse
      const testPulse = (this.preArmTimer > 0.20 && this.preArmTimer < 0.55) ? 0.11 : 0.04;
      for (let m = 0; m < 4; m++) {
        this.motorRpm[m] += (testPulse - this.motorRpm[m]) * (1 - Math.exp(-16 * delta));
      }
      if (this.preArmTimer <= 0) {
        this.preArmTimer = 0;
        this.isArmed = true;
        if (this.pendingAutoTakeoff) {
          this.pendingAutoTakeoff = false;
          this.powerState = 'AUTO_TAKEOFF';
          this.setHudAlert('🛫 PRE-ARM ผ่าน — กำลังบินขึ้นอัตโนมัติสู่ความสูง 3.0 ม.', 2600);
        } else {
          this.powerState = this.isLanded ? 'ARMED_IDLE' : 'FLYING';
          this.setHudAlert('🟢 ARMED — มอเตอร์พร้อมทำงาน! กด Space หรือ T เพื่อบินขึ้น (B ดับเครื่อง)', 2600);
        }
      }
      this.currentAmps = 0.8;
      this.batteryVoltage = p.cells * this.getLipoOpenCircuitVoltage(this.batteryPercent / 100);
      this.droneMesh.position.copy(this.position);
      this.droneMesh.rotation.copy(this.rotation);
      return;
    }

    // DISARMED OR EMPTY BATTERY: rotor coast-down, gravity, air drag, emergency 3D parachute & solid collision
    if (!this.isArmed || this.batteryPercent <= 0) {
      if (this.batteryPercent <= 0) this.powerState = 'DISARMED';
      this.isRTHMode = false;
      this.rthPhase = 'IDLE';
      this.isSpraying = false;
      this.lidarBraking = false;
      this.vrsIntensity *= Math.exp(-5.0 * delta);
      this.igeFactor = 1.0;

      for (let s = 0; s < subSteps; s++) {
        this.simTime += dt;
        const spinDecay = Math.exp(-dt / 0.26);
        for (let m = 0; m < 4; m++) {
          this.motorRpm[m] *= spinDecay;
          if (this.motorRpm[m] < 0.004) this.motorRpm[m] = 0;
        }

        const minGroundY = sampleMinGroundY(this.position.x, this.position.z);
        this.sampleWindField(this.position.x, this.position.y, this.position.z, this.simTime, this.liveWindVec, true);
        this.liveWindSpeed = Math.hypot(this.liveWindVec.x, this.liveWindVec.z);
        this.windLevel = this.liveWindSpeed >= 11.0 ? 'storm' : this.liveWindSpeed >= 7.5 ? 'strong' : this.liveWindSpeed >= 3.5 ? 'moderate' : 'calm';

        // Auto-deploy Pyrotechnic 3D Recovery Parachute when falling from > 5m!
        if (p.hasParachute && !this.parachuteDeployed && !this.isLanded && (this.position.y - minGroundY) > 5.0 && this.velocity.y < -2.2) {
          this.parachuteDeployed = true;
          this.playTone(960, 0.22);
          this.setHudAlert('🪂 กางร่มชูชีพฉุกเฉินอัตโนมัติ — ชะลอความเร็วตกปลอดภัย!', 3800);
        }

        if (this.position.y > minGroundY + 0.005 || !this.isLanded) {
          this.airDensity = 1.225 * Math.exp(-Math.max(0, this.position.y) / 8500);
          this._relAir.subVectors(this.velocity, this.liveWindVec);
          const vMag = this._relAir.length();
          // Deployed parachute adds massive aerodynamic canopy drag (terminal descent ~3.0 m/s!)
          const chuteVertMult = this.parachuteDeployed ? 28.0 : 1.0;
          const chuteHorizMult = this.parachuteDeployed ? 3.2 : 1.0;
          const qH = 0.5 * this.airDensity * p.cdAHoriz * chuteHorizMult * vMag;
          const qV = 0.5 * this.airDensity * p.cdAVert * chuteVertMult * vMag;

          this.velocity.x -= (qH / curMass) * this._relAir.x * dt;
          this.velocity.z -= (qH / curMass) * this._relAir.z * dt;
          this.velocity.y += (-p.gravity - (qV / curMass) * this._relAir.y) * dt;

          this.position.addScaledVector(this.velocity, dt);
          this.surfaceSupportY = minGroundY;

          if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
            this.resolveObstacleCollisions(dt);
          }

          if (this.position.y <= minGroundY) {
            const impactV = Math.max(0, -this.velocity.y);
            this.position.y = minGroundY;
            if (impactV > curVertCrashSpeed && !this.isCrashed) {
              this.isCrashed = true;
              this.playTone(Math.round(135 / Math.pow(curMass, 0.2)), 0.45);
            }
            this.velocity.y = impactV > 3.2 ? impactV * 0.14 : 0;
            const skidDamp = Math.exp(-8.5 * dt);
            this.velocity.x *= skidDamp;
            this.velocity.z *= skidDamp;
            if (impactV <= 3.2) {
              this.isLanded = true;
            }
          }
        } else {
          this.position.y = minGroundY;
          this.velocity.set(0, 0, 0);
          this.isLanded = true;
          if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
            this.resolveObstacleCollisions(dt);
          }
        }

        // Parachute keeps drone swinging gently upright; landed levels skids smoothly
        const angDecay = Math.exp((this.isLanded || this.parachuteDeployed ? -10.0 : -1.8) * dt);
        this.angularVelocity.multiplyScalar(angDecay);
        if (this.isLanded || this.parachuteDeployed) {
          const lvl = 1 - Math.exp(-8.0 * dt);
          this.rotation.x += (0 - this.rotation.x) * lvl;
          this.rotation.z += (0 - this.rotation.z) * lvl;
        } else {
          this.rotation.x = Math.max(-1.2, Math.min(1.2, this.rotation.x + this.angularVelocity.x * dt));
          this.rotation.y += this.angularVelocity.y * dt;
          this.rotation.z = Math.max(-1.2, Math.min(1.2, this.rotation.z + this.angularVelocity.z * dt));
        }
      }

      this.currentAmps = 0.4;
      this.batteryVoltage = p.cells * this.getLipoOpenCircuitVoltage(this.batteryPercent / 100);
      const disarmedVFactor = Math.max(0.72, Math.min(1.11, this.batteryVoltage / p.nomVoltage));
      this.liveTwOge = (p.maxThrustN * disarmedVFactor) / Math.max(0.5, curMass * p.gravity);
      this.liveTwRatio = this.liveTwOge;
      this.droneMesh.position.copy(this.position);
      this.droneMesh.rotation.copy(this.rotation);

      const finalGroundY = sampleMinGroundY(this.position.x, this.position.z);
      this.altitude = Math.max(0, Number((this.position.y - finalGroundY).toFixed(1)));
      if (typeof this.updateAltitudeSensors === 'function') this.updateAltitudeSensors(p);
      this.lidarZone = p.hasLidar ? 'clear' : 'off';
      this.lidarNearestDist = null;
      this.speedKmh = Math.round(Math.hypot(this.velocity.x, this.velocity.z) * 3.6);
      return;
    }

    // ARMED FLIGHT — TRUE 6-DOF QUADROTOR DYNAMICS, AERODYNAMICS & 12-PART SUBSYSTEMS
    this.parachuteDeployed = false;
    this.flightTimeSeconds += delta;

    // Live Agricultural Sprayer / Spreader Drain & Auto-Refill at Farm Refill Pads or Home Helipad!
    if (p.payloadType === 'sprayer' || p.payloadType === 'spreader') {
      if (this.isSpraying && !this.isLanded && this.payloadPercent > 0) {
        this.payloadPercent = Math.max(0, this.payloadPercent - 3.8 * delta);
        if (this.payloadPercent <= 0) {
          this.isSpraying = false;
          this.playTone(300, 0.22);
          this.setHudAlert('💧 ถังปุ๋ยหมด (0%) — ลงจอดที่แท่น AGRI-DRONE REFILL เพื่อเติม!');
        }
      } else if (this.isLanded && this.payloadPercent < 100) {
        let nearRefill = Math.hypot(this.position.x, this.position.z) < 5.0;
        if (!nearRefill && this.farmPlots) {
          for (let i = 0; i < this.farmPlots.length; i++) {
            const rp = this.farmPlots[i].refillPad;
            if (rp && Math.hypot(this.position.x - rp.x, this.position.z - rp.z) < 5.5) {
              nearRefill = true;
              break;
            }
          }
        }
        if (nearRefill) {
          const prevFull = this.payloadPercent >= 99.5;
          this.payloadPercent = Math.min(100, this.payloadPercent + 40.0 * delta);
          if (!prevFull && this.payloadPercent >= 100) {
            this.playTone(840, 0.16);
            this.setHudAlert('✅ เติมถังปุ๋ยเต็ม 100% เรียบร้อย!');
          }
        }
      }
    } else if (p.payloadType === 'cargo' && !this.cargoAttached && this.isLanded && Math.hypot(this.position.x, this.position.z) < 5.0) {
      this.cargoAttached = true;
      this.payloadPercent = 100;
      this.playTone(760, 0.14);
      this.setHudAlert('📦 บรรจุกล่องเวชภัณฑ์ใหม่เรียบร้อย!');
    }

    // Record live 3D flight breadcrumbs (every 3.5m) for automatic sensor/signal-loss backtracking!
    if (typeof this.recordFlightBreadcrumb === 'function') {
      this.recordFlightBreadcrumb();
    }

    // Live RC Receiver (RX) Signal Strength (RSSI), Obstruction Attenuation & Smart Auto-Return Failsafe!
    const rxRangeFrac = rfInfo.rxRangeFrac;
    let rxSignalScale = 1.0;
    const isRangeLost = rxRangeFrac > 1.02;
    const isDeadZoneLost = !this.isLanded && !this.isCrashed && (this.rssiPercent <= 24 || (this.insideMetalRoof && this.signalState === 'critical'));

    if (!this.isRTHMode) {
      if (isRangeLost) {
        if (p.rxFailsafeRth || p.hasGps || (this.flightPathHistory && this.flightPathHistory.length >= 2)) {
          this.triggerSmartAutoReturn('หลุดระยะคลื่น RX!', true, false);
        } else {
          rxSignalScale = Math.max(0.32, Math.min(1.0, this.rssiPercent / 35));
        }
      } else if (isDeadZoneLost) {
        this.signalLossTimer = (this.signalLossTimer || 0) + delta;
        if (this.signalLossTimer >= 2.2) {
          this.triggerSmartAutoReturn('สัญญาณ/เซนเซอร์ขาดหายในจุดอับ!', true, false);
        } else {
          rxSignalScale = Math.max(0.42, 0.42 + (this.rssiPercent / 24) * 0.58);
        }
      } else {
        this.signalLossTimer = Math.max(0, (this.signalLossTimer || 0) - delta * 2.5);
        if (this.rssiPercent < 30) {
          rxSignalScale = Math.max(0.58, 0.58 + (this.rssiPercent / 30) * 0.42);
        }
      }
    } else {
      this.signalLossTimer = 0;
    }

    // Hover calibration constants for LiPo/Li-Ion current draw (True Propeller Power Law: P ∝ ω^3 ∝ T^1.5)
    const hoverRpmNom = Math.sqrt(1.0 / Math.max(0.25, p.twRatio));
    const hoverPowerMetric = 4.0 * Math.pow(hoverRpmNom, 3.0);
    const usableAh = p.capacityAh * 0.85;
    const hoverAmps = (usableAh * 60.0) / Math.max(0.8, p.estHoverMinutes);

    // Check pilot manual override to cleanly cancel RTH (only when link/sensor is back online!)
    if (this.isRTHMode && !this.sensorLostSim && rxRangeFrac < 0.98 && this.rssiPercent >= 45 && (
      Math.abs(this.input.pitch) > 0.36 ||
      Math.abs(this.input.roll) > 0.36 ||
      Math.abs(this.input.throttle) > 0.48
    )) {
      this.cancelSmartAutoReturn('ยกเลิก RTH — นักบินรับช่วงควบคุม');
    }

    // Compute 360° LiDAR + mmWave Radar proximity scan & active obstacle avoidance once per frame!
    let lidarAvoid = { ax: 0, az: 0, triggered: false };
    if (p.hasLidar && !this.isLanded && !this.isRTHMode) {
      lidarAvoid = this.computeLidarAvoidance();
    } else {
      this.lidarZone = p.hasLidar ? 'clear' : 'off';
      this.lidarNearestDist = null;
      this.lidarNearestDirTh = '';
      this.lidarNearestTag = '';
    }
    this.lidarBraking = lidarAvoid.triggered;

    let lastMinGroundY = sampleMinGroundY(this.position.x, this.position.z);

    for (let s = 0; s < subSteps; s++) {
      this.simTime += dt;

      // 1. Atmosphere (ISA Exponential Air Density Lapse) & Live 3D Wind Field
      this.airDensity = 1.225 * Math.exp(-Math.max(0, this.position.y) / 8500);
      const densityRatio = this.airDensity / 1.225;
      this.sampleWindField(this.position.x, this.position.y, this.position.z, this.simTime, this.liveWindVec, true);
      this.liveWindSpeed = Math.hypot(this.liveWindVec.x, this.liveWindVec.z);
      this.windLevel = this.liveWindSpeed >= 11.0 ? 'storm' : this.liveWindSpeed >= 7.5 ? 'strong' : this.liveWindSpeed >= 3.5 ? 'moderate' : 'calm';

      // Relative airspeed vector: v_air = v_ground - v_wind
      this._relAir.subVectors(this.velocity, this.liveWindVec);
      const vAirH = Math.hypot(this._relAir.x, this._relAir.z);
      const vAirMag = this._relAir.length();

      // 2. Real Rotorcraft Aerodynamics: Live Mass-Driven Downwash (v_i), Ground Effect (IGE), ETL, Vortex Ring State (VRS)
      const curHoverInducedVel = Math.sqrt((curMass * p.gravity) / (2.0 * this.airDensity * Math.max(0.02, p.diskArea)));
      const hAgl = Math.max(0.04, this.position.y - this.surfaceSupportY);
      const igeMaxHeight = 2.8 * p.propDiameter;
      if (hAgl < igeMaxHeight) {
        const rEff = p.propRadius * 1.75;
        const hEff = Math.max(hAgl + 0.14, 0.62 * rEff);
        const rRatio = rEff / (4.0 * hEff);
        const rawIge = Math.min(1.24, 1.0 / Math.max(0.76, 1.0 - rRatio * rRatio));
        const igeBlend = Math.max(0, Math.min(1, 1.0 - hAgl / igeMaxHeight));
        this.igeFactor = 1.0 + (rawIge - 1.0) * igeBlend;
      } else {
        this.igeFactor = 1.0;
      }

      const uEtl = vAirH / (1.8 * curHoverInducedVel);
      const etlFactor = 1.0 + 0.075 * ((2.0 * uEtl) / (1.0 + uEtl * uEtl));

      let targetVrs = 0;
      if (!this.isLanded && hAgl > 1.3 && -this._relAir.y > 0.58 * curHoverInducedVel) {
        const descRatio = (-this._relAir.y) / curHoverInducedVel;
        const vrsVert = descRatio <= 1.12
          ? (descRatio - 0.58) / 0.54
          : Math.max(0.25, 1.0 - Math.max(0, descRatio - 1.45) * 0.5);
        const vrsHoriz = Math.max(0, 1.0 - vAirH / (0.68 * curHoverInducedVel));
        targetVrs = Math.max(0, Math.min(1, vrsVert * vrsHoriz));
      }
      this.vrsIntensity += (targetVrs - this.vrsIntensity) * (1 - Math.exp(-6.0 * dt));
      const vrsFactor = 1.0 - 0.18 * this.vrsIntensity;

      // 3. LiPo / Li-Ion Electrochemistry: True Cubic Rotor Shaft Power Law (P ∝ ω^3 ∝ W^1.5) & C-Rating Voltage Sag
      const powerMetric =
        Math.pow(this.motorRpm[0], 3.0) +
        Math.pow(this.motorRpm[1], 3.0) +
        Math.pow(this.motorRpm[2], 3.0) +
        Math.pow(this.motorRpm[3], 3.0);
      const pumpAmps = (this.isSpraying && !this.isLanded) ? 2.4 : 0;
      this.currentAmps = Math.min(
        p.maxBurstAmps,
        1.1 + pumpAmps + hoverAmps * (powerMetric / Math.max(0.12, hoverPowerMetric))
      );
      const vOcv = p.cells * this.getLipoOpenCircuitVoltage(this.batteryPercent / 100);
      this.batteryVoltage = Math.max(p.cells * 3.0, vOcv - this.currentAmps * p.rInternal);
      const voltageFactor = Math.max(0.72, Math.min(1.11, this.batteryVoltage / p.nomVoltage));

      const ahStep = (this.currentAmps * dt) / 3600.0;
      this.batteryPercent = Math.max(0, this.batteryPercent - (ahStep / usableAh) * 100.0);

      const tEffMax = p.maxThrustN * densityRatio * voltageFactor * this.igeFactor * etlFactor * vrsFactor;
      const weightN = curMass * p.gravity;
      const curTw = tEffMax / Math.max(0.5, weightN);
      const curTwOge = (p.maxThrustN * densityRatio * voltageFactor) / Math.max(0.5, weightN);
      this.liveTwRatio = curTw;
      this.liveTwOge = curTwOge;

      // Live Weight-Governed Flight Performance Envelope (heavy = sluggish tilt, slow climb, longer momentum braking!)
      const twMarginFactor = Math.max(0.42, Math.min(1.16, (curTwOge - 0.85) / 1.45));
      const massAgility = Math.max(0.50, Math.min(1.22, Math.pow(1.0 / Math.max(0.45, massRatio), 0.38)));
      const curOmegaAtt = p.omegaAtt * Math.max(0.42, Math.min(1.22, massAgility * twMarginFactor));
      const curKYaw = p.kYaw * Math.max(0.48, Math.min(1.18, massAgility));
      const curMaxYawRate = p.maxYawRate * Math.max(0.55, Math.min(1.18, Math.pow(1.0 / Math.max(0.45, massRatio), 0.30)));
      // Real Physics Bank Limit: vertical thrust component T*cos(tilt) must support weight -> max safe tilt shrinks as T/W nears 1!
      const thrustSafeTilt = curTwOge > 1.06 ? Math.acos(Math.min(0.995, 1.03 / curTwOge)) : 0.14;
      const curMaxTiltRad = Math.min(
        p.maxTiltRad * Math.max(0.66, Math.min(1.14, Math.pow(1.0 / Math.max(0.45, massRatio), 0.24))),
        Math.max(0.14, thrustSafeTilt)
      );
      const curMaxClimbRate = curTwOge > 1.02 ? Math.min(13.0, Math.max(0.32, (curTwOge - 1.0) * 2.75)) : 0.12;
      const curMaxDescentRate = Math.min(6.4, Math.max(2.6, curHoverInducedVel * 1.15));
      const brakeScale = Math.max(0.42, Math.min(1.18, Math.pow(1.0 / Math.max(0.45, massRatio), 0.42) * twMarginFactor));

      const cosTilt = Math.max(0.54, Math.cos(this.rotation.x) * Math.cos(this.rotation.z));
      const sinYaw = Math.sin(this.rotation.y);
      const cosYaw = Math.cos(this.rotation.y);

      const accelToAttitude = (axCmd, azCmd, maxAutoTilt) => {
        const aFwd = axCmd * sinYaw + azCmd * cosYaw;
        const aRight = -axCmd * cosYaw + azCmd * sinYaw;
        const pitch = Math.max(-maxAutoTilt, Math.min(maxAutoTilt, Math.atan2(aFwd, p.gravity)));
        const roll = Math.max(-maxAutoTilt, Math.min(maxAutoTilt, Math.atan2(aRight, p.gravity)));
        return { pitch, roll };
      };

      // 4. Cascaded Flight Controller (RTH / GPS Hold / Optical Flow VPS / LiDAR Brake / Baro / Manual ATTI)
      let targetPitch = 0;
      let targetRoll = 0;
      let targetYawRate = 0;
      let thrustCmdN = weightN * 0.16;

      if (this.isRTHMode && (p.hasGps || (this.rthWaypoints && this.rthWaypoints.length > 0))) {
        const dxHome = -this.position.x;
        const dzHome = -this.position.z;
        const distHome = Math.hypot(dxHome, dzHome);
        const hSpeed = Math.hypot(this.velocity.x, this.velocity.z);
        const homePadY = this.getTerrainHeight(0, 0) + clearance;
        const maxRthTilt = Math.min(0.42, curMaxTiltRad);

        let targetVz = 0;
        let axCmd = 0;
        let azCmd = 0;

        if (this.rthPhase === 'PATH_RETURN') {
          const wps = this.rthWaypoints || [];
          const wp = wps[this.rthWaypointIdx];
          if (!wp) {
            this.rthPhase = distHome < 3.5 ? 'DESCEND' : 'RETURN';
          } else {
            const dxW = wp.x - this.position.x;
            const dyW = wp.y - this.position.y;
            const dzW = wp.z - this.position.z;
            const distH = Math.hypot(dxW, dzW);

            if (distH > 2.2) {
              const desiredYaw = Math.atan2(dxW, dzW);
              const yawErr = Math.atan2(Math.sin(desiredYaw - this.rotation.y), Math.cos(desiredYaw - this.rotation.y));
              targetYawRate = Math.max(-1.8, Math.min(1.8, yawErr * 2.4));
            }

            // Backtrack/indoor waypoints fly slower and extra carefully; open A* corridor cruises smoothly up to 8.5 m/s!
            const maxLegSpd = wp.kind === 'backtrack' ? 4.2 : wp.kind === 'climb' ? 2.5 : 8.5;
            const legSpd = Math.min(maxLegSpd, distH * 0.95);
            const dirX = distH > 0.01 ? dxW / distH : 0;
            const dirZ = distH > 0.01 ? dzW / distH : 0;
            const vxDes = dirX * legSpd;
            const vzDes = dirZ * legSpd;
            axCmd = (vxDes - this.velocity.x) * 1.85 + this.windEst.x * 0.85;
            azCmd = (vzDes - this.velocity.z) * 1.85 + this.windEst.y * 0.85;
            targetVz = Math.max(-3.0, Math.min(3.8, dyW * 1.75));

            const isFinalWp = this.rthWaypointIdx >= wps.length - 1;
            const tolH = isFinalWp ? 1.35 : (wp.kind === 'climb' ? 2.8 : 2.3);
            const tolY = wp.kind === 'climb' ? 1.15 : 2.4;
            if (distH < tolH && Math.abs(dyW) < tolY && (!isFinalWp || hSpeed < 1.6)) {
              this.rthWaypointIdx++;
              if (this.rthWaypointIdx >= wps.length) {
                this.rthPhase = 'DESCEND';
                this.playTone(720, 0.10);
              }
            }
          }
        }

        if (this.rthPhase === 'CLIMB') {
          if (distHome < 3.5 && this.position.y > homePadY + 2.0) {
            this.rthPhase = 'DESCEND';
          } else {
            targetVz = Math.min(4.2, Math.max(1.2, (this.rthCruiseAlt - this.position.y) * 1.4));
            axCmd = -1.8 * this.velocity.x + this.windEst.x;
            azCmd = -1.8 * this.velocity.z + this.windEst.y;
            if (this.position.y >= this.rthCruiseAlt - 0.65) {
              this.rthPhase = 'RETURN';
            }
          }
        }

        if (this.rthPhase === 'RETURN') {
          if (distHome > 3.0) {
            const desiredYaw = Math.atan2(dxHome, dzHome);
            const yawErr = Math.atan2(Math.sin(desiredYaw - this.rotation.y), Math.cos(desiredYaw - this.rotation.y));
            targetYawRate = Math.max(-1.6, Math.min(1.6, yawErr * 2.2));
          }
          const cruiseSpd = Math.min(9.5, distHome * 0.92);
          const dirX = distHome > 0.01 ? dxHome / distHome : 0;
          const dirZ = distHome > 0.01 ? dzHome / distHome : 0;
          const vxDes = dirX * cruiseSpd;
          const vzDes = dirZ * cruiseSpd;
          axCmd = (vxDes - this.velocity.x) * 1.75 + this.windEst.x;
          azCmd = (vzDes - this.velocity.z) * 1.75 + this.windEst.y;
          targetVz = Math.max(-3.0, Math.min(3.5, (this.rthCruiseAlt - this.position.y) * 1.8));
          if (distHome < 1.15 && hSpeed < 1.25) {
            this.rthPhase = 'DESCEND';
          }
        }

        if (this.rthPhase === 'DESCEND') {
          const errX = 0 - this.position.x;
          const errZ = 0 - this.position.z;
          this.windEst.x = Math.max(-5.5, Math.min(5.5, this.windEst.x + errX * 0.45 * dt));
          this.windEst.y = Math.max(-5.5, Math.min(5.5, this.windEst.y + errZ * 0.45 * dt));
          axCmd = 1.9 * errX - 2.4 * this.velocity.x + this.windEst.x;
          azCmd = 1.9 * errZ - 2.4 * this.velocity.z + this.windEst.y;
          const aglHome = Math.max(0, this.position.y - homePadY);
          targetVz = aglHome > 3.2 ? -1.85 : aglHome > 1.0 ? -1.05 : -0.58;
          if (this.isLanded && aglHome < 0.28) {
            const wasFail = this.isFailsafeReturn || this.sensorLostSim;
            this.isArmed = false;
            this.isRTHMode = false;
            this.isFailsafeReturn = false;
            this.sensorLostSim = false;
            this.rthPhase = 'IDLE';
            this.rthWaypoints = [];
            this.rthWaypointIdx = 0;
            this.powerState = 'DISARMED';
            if (typeof this.clearRthPathVisuals === 'function') this.clearRthPathVisuals();
            this.playMelody([[784, 0, 0.09], [587, 0.10, 0.09], [392, 0.20, 0.18]]);
            this.setHudAlert(
              wasFail
                ? '✅ คำนวณเส้นทางและบินกลับจุดเริ่มต้นสำเร็จ (ลงจอดและดับมอเตอร์แล้ว)!'
                : '✅ RTH ลงจอดและดับมอเตอร์ที่ลาน H สำเร็จ!',
              3800
            );
            break;
          }
        }

        const att = accelToAttitude(axCmd, azCmd, maxRthTilt);
        targetPitch = att.pitch;
        targetRoll = att.roll;
        const azVert = (targetVz - this.velocity.y) * 4.4;
        thrustCmdN = (curMass * (p.gravity + azVert)) / cosTilt;
        this.holdAltitude = this.position.y;
      } else if (this.powerState === 'AUTO_TAKEOFF') {
        // AUTO-TAKEOFF: Smoothly lifts through Ground Effect (IGE) to 3.0m safe hover (aborts if overloaded T/W <= 1.02!)
        this.autoTakeoffTimer = (this.autoTakeoffTimer || 0) + dt;
        if (Math.abs(this.input.pitch) > 0.35 || Math.abs(this.input.roll) > 0.35 || Math.abs(this.input.throttle) > 0.45) {
          this.powerState = 'FLYING';
          this.holdAltitude = this.position.y;
          this.altHoldLocked = false;
          this.setHudAlert('ยกเลิก Auto-Takeoff — นักบินรับช่วงควบคุม', 2000);
        } else if (this.autoTakeoffTimer >= 1.65 && curTwOge <= 1.02 && hAgl < 0.75) {
          this.powerState = 'ARMED_IDLE';
          this.autoTakeoffTimer = 0;
          this.playMelody([[300, 0, 0.14], [210, 0.16, 0.22]]);
          this.setHudAlert(`⚠️ น้ำหนัก ${(curMass).toFixed(2)} กก. เกินแรงยก (T/W ${curTwOge.toFixed(2)} ≤ 1.0) — ยกตัวไม่ขึ้น!`, 4200);
        } else {
          const remAlt = this.autoTakeoffTargetY - this.position.y;
          const targetVz = Math.min(curMaxClimbRate, Math.min(2.6, Math.max(0.45, remAlt * 1.45)));
          const axCmd = -2.0 * brakeScale * this.velocity.x;
          const azCmd = -2.0 * brakeScale * this.velocity.z;
          const att = accelToAttitude(axCmd, azCmd, Math.min(0.24, curMaxTiltRad));
          targetPitch = att.pitch;
          targetRoll = att.roll;
          targetYawRate = 0;
          const azVert = (targetVz - this.velocity.y) * (4.4 * Math.max(0.65, brakeScale));
          thrustCmdN = (curMass * (p.gravity + azVert)) / cosTilt;
          this.holdAltitude = this.autoTakeoffTargetY;
          if (remAlt <= 0.14) {
            this.powerState = 'FLYING';
            this.autoTakeoffTimer = 0;
            this.holdAltitude = this.position.y;
            this.altHoldLocked = true;
            this.holdTargetPos.copy(this.position);
            this.holdTargetLocked = true;
            this.playTone(840, 0.14);
            this.setHudAlert('✅ บินขึ้นอัตโนมัติสำเร็จ (สูง 3.0 ม.) — พร้อมบังคับ!', 2600);
          }
        }
      } else if (this.powerState === 'AUTO_LANDING') {
        // AUTO-LANDING IN-PLACE: Arrests drift, descends smoothly through Ground Effect, settles & auto-disarms!
        if (this.input.throttle > 0.32 || Math.abs(this.input.pitch) > 0.35 || Math.abs(this.input.roll) > 0.35) {
          this.powerState = 'FLYING';
          this.autoLandSettleTimer = 0;
          this.holdAltitude = this.position.y;
          this.altHoldLocked = false;
          this.setHudAlert('⏸ ยกเลิก Auto-Land — นักบินรับช่วงควบคุม', 2000);
        } else if (this.isLanded) {
          targetPitch = 0;
          targetRoll = 0;
          targetYawRate = 0;
          thrustCmdN = weightN * 0.14;
          this.autoLandSettleTimer += dt;
          if (this.autoLandSettleTimer >= 0.42) {
            this.stopDrone({ forceEmergency: false });
            this.setHudAlert('✅ ลงจอดและดับมอเตอร์อัตโนมัติเรียบร้อย! (DISARMED)', 3200);
            break;
          }
        } else {
          this.autoLandSettleTimer = 0;
          const axCmd = -2.2 * this.velocity.x + (p.hasGps ? this.windEst.x * 0.7 : 0);
          const azCmd = -2.2 * this.velocity.z + (p.hasGps ? this.windEst.y * 0.7 : 0);
          const att = accelToAttitude(axCmd, azCmd, 0.28);
          targetPitch = att.pitch;
          targetRoll = att.roll;
          targetYawRate = 0;
          const targetVz = hAgl > 3.2 ? -1.75 : hAgl > 0.9 ? -0.95 : -0.54;
          const azVert = (targetVz - this.velocity.y) * 4.4;
          thrustCmdN = (curMass * (p.gravity + azVert)) / cosTilt;
          this.holdAltitude = this.position.y;
        }
      } else {
        // PILOTED MODE — Zero-Discontinuity Hermite Blending, Cross-Track Glide & Rate-Shaped Setpoints (100% Real 6-DOF Physics!)
        const rateProf = this.getControlRateProfile();
        const smoothstep = (e0, e1, v) => {
          const t = Math.max(0, Math.min(1, (v - e0) / (e1 - e0)));
          return t * t * (3 - 2 * t);
        };
        const pPitch = this.applyStickExpo(this.input.pitch, rateProf.expo) * rxSignalScale;
        const pRoll = this.applyStickExpo(this.input.roll, rateProf.expo) * rxSignalScale;
        const pYaw = this.applyStickExpo(this.input.yaw, rateProf.expo * 0.92) * rxSignalScale;
        const pThr = this.applyStickExpo(this.input.throttle, rateProf.expo * 0.82) * rxSignalScale;

        const isGroundedIdle = this.isLanded && this.input.throttle <= 0.04;

        if (isGroundedIdle) {
          this.powerState = 'ARMED_IDLE';
          targetPitch = 0;
          targetRoll = 0;
          targetYawRate = 0;
          thrustCmdN = weightN * 0.16;
          this.brakeRamp = 0;
          this.smoothTargetVz = 0;
          this.holdAltitude = this.position.y;
          this.holdTargetPos.copy(this.position);
          this.holdTargetLocked = true;
          this.windEst.multiplyScalar(Math.exp(-3.0 * dt));

          // Authentic Real-Drone Touchdown Auto-Disarm: holding Shift down on ground for 1.1s shuts off motors!
          if (this.input.throttle < -0.55) {
            this.touchdownHoldTimer += dt;
            if (this.touchdownHoldTimer >= 1.1) {
              this.stopDrone({ forceEmergency: false });
              this.setHudAlert('⏹ ตรวจพบการลงจอด (กด Shift ค้าง) — ดับมอเตอร์อัตโนมัติ!', 3000);
              break;
            }
          } else {
            this.touchdownHoldTimer = 0;
          }
        } else {
          this.powerState = 'FLYING';
          this.touchdownHoldTimer = 0;
          targetYawRate = -pYaw * curMaxYawRate * rateProf.yawScale;
          const stickHorizMag = Math.hypot(this.input.pitch, this.input.roll);

          // Educational overload feedback when pilot holds full throttle on an overweight build (T/W <= 1.02)!
          if (this.input.throttle > 0.55 && curTwOge <= 1.02 && hAgl < 0.70) {
            const nowOv = performance.now();
            if (nowOv - (this._lastOverloadAlertMs || 0) > 3600) {
              this._lastOverloadAlertMs = nowOv;
              this.playTone(230, 0.16);
              this.setHudAlert(`⚠️ น้ำหนักเกินแรงยก (T/W ${curTwOge.toFixed(2)} ≤ 1.0) — เร่งเต็ม 100% แต่ยกตัวไม่ขึ้น! ลดน้ำหนักหรือเพิ่มขนาดมอเตอร์`, 3500);
            }
          }

          // Smooth Hermite cross-fade between Manual Pilot Tilt and Active GPS/VPS Braking (ZERO release snap!)
          const effMaxTilt = Math.min(curMaxTiltRad, curMaxTiltRad * rateProf.tiltScale);
          const pilotPitch = pPitch * effMaxTilt;
          let pilotRoll = pRoll * effMaxTilt;

          // Subtle lateral cross-track stabilization in GPS Hold when gliding straight forward/back (feels on-rails in crosswind!)
          if (this.isHoldMode && p.hasGps && !this.isLanded && Math.abs(this.input.pitch) > 0.14 && Math.abs(this.input.roll) < 0.08) {
            const vRight = -this.velocity.x * cosYaw + this.velocity.z * sinYaw;
            const latDamp = Math.max(-0.14, Math.min(0.14, -vRight * 0.045 * brakeScale));
            pilotRoll += latDamp;
          }

          const manualBlend = smoothstep(0.015, 0.30, stickHorizMag);
          const curBrakeRamp = this.brakeRamp || 0;
          if (manualBlend > 0.35) {
            this.holdTargetLocked = false;
            this.windEst.multiplyScalar(Math.exp(-0.55 * dt));
            this.brakeRamp = curBrakeRamp + (0 - curBrakeRamp) * (1 - Math.exp(-8.5 * dt));
          } else {
            // Progressive silky brake ramp over ~0.24s after stick release (no sudden nose-up whiplash!)
            this.brakeRamp = curBrakeRamp + (1 - curBrakeRamp) * (1 - Math.exp(-5.4 * dt));
          }

          let holdPitch = 0;
          let holdRoll = 0;

          if (manualBlend < 0.995) {
            if (this.isHoldMode && p.hasGps && !this.isLanded) {
              const hSpd = Math.hypot(this.velocity.x, this.velocity.z);
              let axCmd = 0;
              let azCmd = 0;

              if (!this.holdTargetLocked) {
                // Heavier drones carry real horizontal momentum -> noticeably longer smooth braking glide!
                const hBrake = 1.65 * brakeScale * rateProf.brakeMult * (0.28 + 0.72 * this.brakeRamp);
                axCmd = -hBrake * this.velocity.x + this.windEst.x * 0.4;
                azCmd = -hBrake * this.velocity.z + this.windEst.y * 0.4;
                if (hSpd < 0.65 && manualBlend < 0.05) {
                  this.holdTargetPos.copy(this.position);
                  this.holdTargetLocked = true;
                }
              } else {
                const gpsNoiseAmp = (p.isRtk ? 0.020 : 0.22) * (this.gpsMultipathFactor || 1.0);
                const gpsErrX = gpsNoiseAmp * (0.68 * Math.sin(this.simTime * 0.37) + 0.32 * Math.cos(this.simTime * 0.71));
                const gpsErrZ = gpsNoiseAmp * (0.68 * Math.cos(this.simTime * 0.41) + 0.32 * Math.sin(this.simTime * 0.67));
                const ex = (this.holdTargetPos.x + gpsErrX) - this.position.x;
                const ez = (this.holdTargetPos.z + gpsErrZ) - this.position.z;

                const kiWind = (p.isRtk ? 0.68 : 0.48) * (p.dualImu ? 1.18 : 1.0);
                this.windEst.x = Math.max(-8.5, Math.min(8.5, this.windEst.x + ex * kiWind * dt));
                this.windEst.y = Math.max(-8.5, Math.min(8.5, this.windEst.y + ez * kiWind * dt));

                const kpPos = (p.isRtk ? 2.05 : 1.42) * Math.max(0.65, brakeScale);
                const kdPos = (p.isRtk ? 2.55 : 2.15) * Math.max(0.65, brakeScale);
                axCmd = kpPos * ex - kdPos * this.velocity.x + this.windEst.x;
                azCmd = kpPos * ez - kdPos * this.velocity.z + this.windEst.y;
              }

              const att = accelToAttitude(axCmd, azCmd, Math.min(0.42, curMaxTiltRad));
              holdPitch = att.pitch;
              holdRoll = att.roll;
            } else if (p.hasOpticalFlow && !this.isLanded && hAgl < 6.0) {
              // Optical Flow + Sonar VPS: locks low-altitude horizontal drift smoothly (scaled by mass momentum!)
              const vpsBrake = 1.55 * brakeScale * (0.30 + 0.70 * this.brakeRamp);
              const att = accelToAttitude(-vpsBrake * this.velocity.x, -vpsBrake * this.velocity.z, Math.min(0.26, curMaxTiltRad));
              holdPitch = att.pitch;
              holdRoll = att.roll;
              this.holdTargetLocked = false;
            } else {
              this.holdTargetLocked = false;
            }
          }

          targetPitch = holdPitch * (1 - manualBlend) + pilotPitch * manualBlend;
          targetRoll = holdRoll * (1 - manualBlend) + pilotRoll * manualBlend;

          // Superimpose 360° LiDAR Obstacle Avoidance Repulsive Brake Attitude!
          if (lidarAvoid.triggered) {
            const avoidAtt = accelToAttitude(lidarAvoid.ax, lidarAvoid.az, Math.min(0.44, curMaxTiltRad));
            targetPitch = Math.max(-curMaxTiltRad, Math.min(curMaxTiltRad, targetPitch * 0.35 + avoidAtt.pitch));
            targetRoll = Math.max(-curMaxTiltRad, Math.min(curMaxTiltRad, targetRoll * 0.35 + avoidAtt.roll));
          }

          // Vertical Collective Thrust Control (Velvet-Smooth Barometer Alt-Hold vs Authentic Manual No-Baro F4!)
          if (p.hasBaro) {
            const thrMag = Math.abs(this.input.throttle);
            const altManualBlend = smoothstep(0.015, 0.16, thrMag);
            const manualVz = this.input.throttle > 0
              ? pThr * curMaxClimbRate * rateProf.climbScale
              : pThr * curMaxDescentRate * rateProf.climbScale;

            let holdVz = 0;
            if (altManualBlend > 0.20) {
              this.altHoldLocked = false;
              this.holdAltitude = this.position.y;
            } else if (!this.altHoldLocked) {
              this.holdAltitude = this.position.y;
              if (Math.abs(this.velocity.y) < 0.30) {
                this.altHoldLocked = true;
              }
            } else {
              // Downward mmWave Terrain-Following Radar (when sensor_lidar is fitted): maintains safe >=1.6m ground clearance over rising slopes!
              if (p.hasLidar && !this.isLanded && this.input.throttle >= -0.05) {
                const safeFloorY = this.surfaceSupportY + 1.6;
                if (this.holdAltitude < safeFloorY && this.position.y < safeFloorY + 0.4 && Math.hypot(this.velocity.x, this.velocity.z) > 0.6) {
                  this.holdAltitude += (safeFloorY - this.holdAltitude) * (1 - Math.exp(-3.2 * dt));
                }
              }
              holdVz = Math.max(-3.2, Math.min(3.2, (this.holdAltitude - this.position.y) * (1.95 * Math.max(0.60, brakeScale))));
            }

            const rawTargetVz = holdVz * (1 - altManualBlend) + manualVz * altManualBlend;
            const vzAlpha = 1 - Math.exp(-rateProf.setpointSmooth * 0.85 * dt);
            this.smoothTargetVz = (this.smoothTargetVz || 0) + (rawTargetVz - (this.smoothTargetVz || 0)) * vzAlpha;

            const azVert = (this.smoothTargetVz - this.velocity.y) * (3.9 * Math.max(0.55, brakeScale));
            // When overloaded and holding throttle up, push full available thrust so motors spin at 100% straining!
            thrustCmdN = (this.input.throttle > 0.08 && curTwOge <= 1.05)
              ? Math.max(tEffMax * 1.15, (curMass * (p.gravity + azVert)) / cosTilt)
              : (curMass * (p.gravity + azVert)) / cosTilt;
          } else {
            const partialTiltComp = 1.0 + (1.0 / cosTilt - 1.0) * 0.32;
            const baroLessDrift = 0.024 * Math.sin(this.simTime * 0.72) + 0.014 * Math.cos(this.simTime * 1.19);
            const hoverBaseN = weightN * (1.0 + baroLessDrift) * partialTiltComp;

            if (this.input.throttle > 0.03) {
              const climbSpanN = Math.max(weightN * 0.25, tEffMax * 0.94 - weightN);
              thrustCmdN = hoverBaseN + pThr * climbSpanN;
            } else if (this.input.throttle < -0.03) {
              thrustCmdN = hoverBaseN + pThr * (weightN * 0.78);
            } else {
              thrustCmdN = hoverBaseN - curMass * this.velocity.y * (1.15 * Math.max(0.55, brakeScale));
            }
          }
        }
      }

      // 5. PX4/DJI Attitude Setpoint Pre-Filter -> Inner Torque Controller (Live Inertia & Agility) -> Quad-X Mixer -> Motor Lag
      const spRate = this.getControlRateProfile().setpointSmooth;
      const spAlpha = 1 - Math.exp(-spRate * dt);
      this.smoothTargetPitch = (this.smoothTargetPitch || 0) + (targetPitch - (this.smoothTargetPitch || 0)) * spAlpha;
      this.smoothTargetRoll = (this.smoothTargetRoll || 0) + (targetRoll - (this.smoothTargetRoll || 0)) * spAlpha;
      this.smoothTargetYawRate = (this.smoothTargetYawRate || 0) + (targetYawRate - (this.smoothTargetYawRate || 0)) * spAlpha;

      const tauXCmd = curIxx * (
        curOmegaAtt * curOmegaAtt * (this.smoothTargetPitch - this.rotation.x)
        - 2.0 * p.zetaAtt * curOmegaAtt * this.angularVelocity.x
      );
      const tauZCmd = curIzz * (
        curOmegaAtt * curOmegaAtt * (this.smoothTargetRoll - this.rotation.z)
        - 2.0 * p.zetaAtt * curOmegaAtt * this.angularVelocity.z
      );
      const tauYCmd = curIyy * (curKYaw * (this.smoothTargetYawRate - this.angularVelocity.y));

      const invT = 1.0 / Math.max(4.0, tEffMax);
      const fT = Math.max(0.045, Math.min(1.0, thrustCmdN * invT));
      const uP = Math.max(-0.34, Math.min(0.34, (tauXCmd * invT) / p.armXY));
      const uR = Math.max(-0.34, Math.min(0.34, (tauZCmd * invT) / p.armXY));
      const uY = Math.max(-0.22, Math.min(0.22, (tauYCmd * invT) / p.kYawArm));

      let f0 = fT - uP + uR + uY;
      let f1 = fT + uP + uR - uY;
      let f2 = fT + uP - uR + uY;
      let f3 = fT - uP - uR - uY;

      const minIdleF = 0.045;
      const maxCapF = p.escThrottleLimit * p.escThrottleLimit;
      const maxF = Math.max(f0, f1, f2, f3);
      if (maxF > maxCapF) {
        const shift = maxF - maxCapF;
        f0 -= shift; f1 -= shift; f2 -= shift; f3 -= shift;
      }
      const minF = Math.min(f0, f1, f2, f3);
      if (minF < minIdleF) {
        const shift = minIdleF - minF;
        f0 += shift; f1 += shift; f2 += shift; f3 += shift;
      }

      const fCmd0 = Math.max(minIdleF, Math.min(maxCapF, f0));
      const fCmd1 = Math.max(minIdleF, Math.min(maxCapF, f1));
      const fCmd2 = Math.max(minIdleF, Math.min(maxCapF, f2));
      const fCmd3 = Math.max(minIdleF, Math.min(maxCapF, f3));

      const mAlpha = 1 - Math.exp(-dt / p.tauMotor);
      this.motorRpm[0] += (Math.sqrt(fCmd0) - this.motorRpm[0]) * mAlpha;
      this.motorRpm[1] += (Math.sqrt(fCmd1) - this.motorRpm[1]) * mAlpha;
      this.motorRpm[2] += (Math.sqrt(fCmd2) - this.motorRpm[2]) * mAlpha;
      this.motorRpm[3] += (Math.sqrt(fCmd3) - this.motorRpm[3]) * mAlpha;

      const a0 = this.motorRpm[0] * this.motorRpm[0];
      const a1 = this.motorRpm[1] * this.motorRpm[1];
      const a2 = this.motorRpm[2] * this.motorRpm[2];
      const a3 = this.motorRpm[3] * this.motorRpm[3];

      const tActualN = tEffMax * 0.25 * (a0 + a1 + a2 + a3);
      const tauXAct = tEffMax * 0.25 * ((a1 + a2) - (a0 + a3)) * p.armXY;
      const tauZAct = tEffMax * 0.25 * ((a0 + a1) - (a2 + a3)) * p.armXY;
      const tauYAct = tEffMax * 0.25 * ((a0 + a2) - (a1 + a3)) * p.kYawArm;

      // 6. True 6-DOF Newton-Euler Translational Dynamics (Live Mass + Thrust Vectoring + Quadratic & Live Rotor Drag)
      this._bodyUp.set(0, 1, 0).applyEuler(this.rotation);
      let fx = tActualN * this._bodyUp.x;
      let fy = tActualN * this._bodyUp.y;
      let fz = tActualN * this._bodyUp.z;

      const qHoriz = 0.5 * this.airDensity * p.cdAHoriz * vAirMag;
      const qVert = 0.5 * this.airDensity * p.cdAVert * vAirMag;
      const rotorDrag = 0.14 * this.airDensity * p.diskArea * curHoverInducedVel;

      fx -= (qHoriz + rotorDrag) * this._relAir.x;
      fz -= (qHoriz + rotorDrag) * this._relAir.z;
      fy -= qVert * this._relAir.y;

      this.velocity.x += (fx / curMass) * dt;
      this.velocity.y += (fy / curMass - p.gravity) * dt;
      this.velocity.z += (fz / curMass) * dt;

      // 7. True 6-DOF Euler Rotational Dynamics (Live Inertia + Mass-Stabilized Wind Gusts + VRS Buffet + Gyro Coupling)
      const wx = this.angularVelocity.x;
      const wy = this.angularVelocity.y;
      const wz = this.angularVelocity.z;

      const vrsTx = this.vrsIntensity * curIxx * (8.5 * Math.sin(this.simTime * 23.7) + 5.2 * Math.cos(this.simTime * 37.1));
      const vrsTz = this.vrsIntensity * curIzz * (8.5 * Math.cos(this.simTime * 27.3) + 5.2 * Math.sin(this.simTime * 41.3));

      // Heavier drones resist rotational wind gusts far better than featherweight 500g FPV quads!
      const massWindStability = Math.max(0.32, Math.min(1.65, 1.05 / Math.pow(curMass, 0.62)));
      const gustAmp = this.isLanded ? 0 : Math.min(1.8, this.liveWindSpeed * 0.14) * p.gustRejectFactor * massWindStability;
      const gustTx = gustAmp * curIxx * Math.sin(this.simTime * 5.3 + this.position.x * 0.11);
      const gustTz = gustAmp * curIzz * Math.cos(this.simTime * 4.7 + this.position.z * 0.11);

      const gyroX = (curIyy - curIzz) * wy * wz;
      const gyroY = (curIzz - curIxx) * wz * wx;
      const gyroZ = (curIxx - curIyy) * wx * wy;

      const rotDragX = -curIxx * 0.45 * wx;
      const rotDragY = -curIyy * 0.35 * wy;
      const rotDragZ = -curIzz * 0.45 * wz;

      this.angularVelocity.x += ((tauXAct + vrsTx + gustTx + gyroX + rotDragX) / curIxx) * dt;
      this.angularVelocity.y += ((tauYAct + gyroY + rotDragY) / curIyy) * dt;
      this.angularVelocity.z += ((tauZAct + vrsTz + gustTz + gyroZ + rotDragZ) / curIzz) * dt;

      this.rotation.x = Math.max(-1.12, Math.min(1.12, this.rotation.x + this.angularVelocity.x * dt));
      this.rotation.y += this.angularVelocity.y * dt;
      this.rotation.z = Math.max(-1.12, Math.min(1.12, this.rotation.z + this.angularVelocity.z * dt));

      // 8. Integrate Position & Resolve Solid Terrain / Cliff / Building Collisions
      this._prevPos.copy(this.position);
      this.position.addScaledVector(this.velocity, dt);

      this._moveDelta.subVectors(this.position, this._prevPos);
      const horizDist = Math.hypot(this._moveDelta.x, this._moveDelta.z);
      if (this.customMapRoot && this.solidTerrainMeshes && this.solidTerrainMeshes.length > 0 && horizDist > 0.001) {
        this._hDir.set(this._moveDelta.x, 0, this._moveDelta.z).normalize();
        this.wallRaycaster.set(this._prevPos, this._hDir);
        this.wallRaycaster.far = horizDist + 0.95;
        const wallHits = this.wallRaycaster.intersectObjects(this.solidTerrainMeshes, false);
        if (wallHits.length > 0) {
          this.position.x = this._prevPos.x;
          this.position.z = this._prevPos.z;
          this.velocity.x *= -0.28;
          this.velocity.z *= -0.28;
        }
      }

      const minDroneY = sampleMinGroundY(this.position.x, this.position.z);
      lastMinGroundY = minDroneY;
      this.surfaceSupportY = minDroneY;

      if (minDroneY > this._prevPos.y + 1.8) {
        this.position.x = this._prevPos.x;
        this.position.z = this._prevPos.z;
        this.velocity.x *= -0.28;
        this.velocity.z *= -0.28;
        const safeGroundY = this.getTerrainHeight(this.position.x, this.position.z) + clearance;
        if (this.position.y < safeGroundY) {
          this.position.y = safeGroundY;
          this.velocity.y = Math.max(0, this.velocity.y);
        }
        this.surfaceSupportY = safeGroundY;
        this.isLanded = (this.position.y <= safeGroundY + 0.04 && this.input.throttle <= 0.04);
      } else if (this.position.y <= minDroneY) {
        const impactVert = Math.max(0, -this.velocity.y);
        this.position.y = minDroneY;

        if (impactVert > curVertCrashSpeed) {
          this.isCrashed = true;
          this.isRTHMode = false;
          this.rthPhase = 'IDLE';
          this.playTone(Math.round(135 / Math.pow(curMass, 0.2)), 0.45);
        } else if (impactVert > 2.9) {
          this.velocity.y = impactVert * 0.15;
          this.playTone(Math.round(195 / Math.pow(curMass, 0.25)), 0.09);
        } else {
          this.velocity.y = Math.max(0, this.velocity.y);
        }

        const skidFric = Math.exp(-8.5 * dt);
        this.velocity.x *= skidFric;
        this.velocity.z *= skidFric;

        if (impactVert <= 2.9 && this.input.throttle <= 0.04 && !(this.isRTHMode && this.rthPhase === 'CLIMB') && this.powerState !== 'AUTO_TAKEOFF') {
          this.isLanded = true;
          const lvl = 1 - Math.exp(-14.0 * dt);
          this.rotation.x += (0 - this.rotation.x) * lvl;
          this.rotation.z += (0 - this.rotation.z) * lvl;
          this.angularVelocity.x *= Math.exp(-12.0 * dt);
          this.angularVelocity.z *= Math.exp(-12.0 * dt);
        } else {
          this.isLanded = false;
        }
      } else {
        this.isLanded = false;
        if (this.isCrashed && this.isArmed && this.input.throttle > 0.25) {
          this.isCrashed = false;
        }
      }

      if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
        this.resolveObstacleCollisions(dt);
        const mapDist = Math.hypot(this.position.x, this.position.z);
        const maxFlyRadius = 292;
        if (mapDist > maxFlyRadius) {
          const nx = this.position.x / mapDist;
          const nz = this.position.z / mapDist;
          this.position.x = nx * maxFlyRadius;
          this.position.z = nz * maxFlyRadius;
          const outVel = this.velocity.x * nx + this.velocity.z * nz;
          if (outVel > 0) {
            this.velocity.x -= 1.25 * outVel * nx;
            this.velocity.z -= 1.25 * outVel * nz;
          }
        }
      }

      // Realistic Drone-Class Firmware Altitude Geofence Ceiling (Agri 30m | FPV 50m | Aerial 90m | Heavy Survey 120m!)
      const frameCeilingM = p.maxAltitudeLimitM || 90;
      const maxAllowedY = minDroneY + frameCeilingM;
      const remToCeiling = maxAllowedY - this.position.y;
      if (remToCeiling < 2.5 && this.velocity.y > 0) {
        // Soft firmware vertical brake approaching the ceiling (zero harsh bounce!)
        const maxUpV = Math.max(0, remToCeiling * 1.6);
        if (this.velocity.y > maxUpV) {
          this.velocity.y += (maxUpV - this.velocity.y) * (1 - Math.exp(-12 * dt));
        }
      }
      if (this.position.y >= maxAllowedY) {
        this.position.y = maxAllowedY;
        this.velocity.y = Math.min(0, this.velocity.y);
        this.holdAltitude = Math.min(this.holdAltitude, maxAllowedY);
      }
    }

    this.droneMesh.position.copy(this.position);
    this.droneMesh.rotation.copy(this.rotation);

    this.altitude = Math.max(0, Number((this.position.y - lastMinGroundY).toFixed(1)));
    if (typeof this.updateAltitudeSensors === 'function') this.updateAltitudeSensors(p);
    if (this.altitude > this.maxAltitude) {
      this.maxAltitude = this.altitude;
    }
    const hSpeed = Math.hypot(this.velocity.x, this.velocity.z);
    this.speedKmh = Math.round(hSpeed * 3.6);

    // Live Frame-Class Altitude Ceiling Feedback (notifies pilot when hitting 30m / 50m / 90m / 120m limit!)
    const ceilingLim = p.maxAltitudeLimitM || 90;
    const atCeiling = !this.isLanded && (this.altitude >= ceilingLim - 0.25 && (this.input.throttle > 0.04 || this.altitude >= ceilingLim - 0.05));
    this.hitAltitudeLimit = atCeiling;
    if (atCeiling && this.input.throttle > 0.08) {
      const nowAltMs = performance.now();
      if (nowAltMs - (this._lastAltLimitAlertMs || 0) > 3600) {
        this._lastAltLimitAlertMs = nowAltMs;
        this.playTone(620, 0.12);
        this.setHudAlert(`🛑 ถึงเพดานสูงสุดของ${p.frameClassLabel} (${ceilingLim} ม.) — ${p.altitudeLimitReason}`, 3400);
      }
    }

    // Live Thailand Drone Law (CAAT) Monitoring: 90m Ceiling, Community Overflight & VLOS Range
    const nowLawMs = performance.now();
    if (this.altitude > 90.0) {
      this.lawOverCeiling = true;
      this.lawAltitudeViolated = true;
      if (nowLawMs - this._lastLawBeepMs > 4200) {
        this._lastLawBeepMs = nowLawMs;
        this.playTone(920, 0.14);
      }
    } else {
      this.lawOverCeiling = false;
    }

    if (!this.lawVillageWarned && !this.isLanded && this.altitude > 2.5 && this.altitude < 38
      && this.position.x > 22 && this.position.x < 96 && this.position.z > -32 && this.position.z < 36) {
      this.lawVillageWarned = true;
      this.setHudAlert('⚖️ เตือนกฎหมายไทย (กพท.): ห้ามบินเหนือชุมชน/บ้านเรือน และต้องห่างบุคคล-อาคาร ≥ 30/50 ม.', 4500);
    }

    if (!this.lawVlosWarned && !this.isLanded && Math.hypot(this.position.x, this.position.z) > 205) {
      this.lawVlosWarned = true;
      this.setHudAlert('👁️ เตือนกฎ VLOS (กพท.): บินไกลกว่า 200 ม. — กฎหมายไทยกำหนดให้เห็นตัวโดรนด้วยตาเปล่าตลอดเวลา', 4500);
    }
  }

  resolveObstacleCollisions(dt = 0.008) {
    const p = this.physicsParams;
    const r = p && p.wheelbase >= 0.74 ? 1.15 : p && p.wheelbase >= 0.56 ? 0.92 : p && p.wheelbase <= 0.30 ? 0.58 : 0.76;
    const h = p && p.clearance ? p.clearance - 0.06 : 0.42;
    const curMass = p ? Math.max(0.25, p.dryMass + p.payloadTankKg * (this.payloadPercent / 100)) : 1.1;
    const vertCrash = (p ? p.vertCrashSpeed : 7.5) * Math.max(0.72, Math.min(1.22, Math.pow(1.2 / curMass, 0.22)));
    const sideCrash = p ? p.sideCrashSpeed : 13.5;
    const roofFric = Math.exp(-8.0 * dt);

    for (let i = 0; i < this.colliders.length; i++) {
      const c = this.colliders[i];

      if (c.type === 'box') {
        const exMinX = c.minX - r, exMaxX = c.maxX + r;
        const exMinY = c.minY - h, exMaxY = c.maxY + h;
        const exMinZ = c.minZ - r, exMaxZ = c.maxZ + r;

        if (
          this.position.x > exMinX && this.position.x < exMaxX &&
          this.position.z > exMinZ && this.position.z < exMaxZ &&
          this.position.y >= exMaxY - 0.05 && exMaxY > this.surfaceSupportY
        ) {
          this.surfaceSupportY = exMaxY;
        }

        if (
          this.position.x > exMinX && this.position.x < exMaxX &&
          this.position.y > exMinY && this.position.y < exMaxY &&
          this.position.z > exMinZ && this.position.z < exMaxZ
        ) {
          const penLeft = this.position.x - exMinX;
          const penRight = exMaxX - this.position.x;
          const penDown = this.position.y - exMinY;
          const penUp = exMaxY - this.position.y;
          const penBack = this.position.z - exMinZ;
          const penFront = exMaxZ - this.position.z;

          const minPen = Math.min(penLeft, penRight, penDown, penUp, penBack, penFront);
          if (minPen === penUp) {
            const impactV = Math.max(0, -this.velocity.y);
            this.position.y = exMaxY;
            this.surfaceSupportY = Math.max(this.surfaceSupportY, exMaxY);
            if (impactV > vertCrash && !this.isCrashed) {
              this.isCrashed = true;
              this.playTone(135, 0.45);
            }
            this.velocity.y = impactV > 3.0 ? impactV * 0.14 : Math.max(0, this.velocity.y);
            this.velocity.x *= roofFric;
            this.velocity.z *= roofFric;
            if (impactV <= 3.0 && this.input.throttle <= 0.04 && !(this.isRTHMode && this.rthPhase === 'CLIMB') && this.powerState !== 'AUTO_TAKEOFF') {
              this.isLanded = true;
            }
          } else if (minPen === penDown) {
            this.position.y = exMinY;
            this.velocity.y = Math.min(0, this.velocity.y) * -0.3;
          } else if (minPen === penLeft) {
            const impactH = Math.max(0, this.velocity.x);
            if (impactH > sideCrash && !this.isCrashed) { this.isCrashed = true; this.playTone(135, 0.45); }
            this.position.x = exMinX;
            this.velocity.x = Math.min(0, this.velocity.x) * -0.32;
          } else if (minPen === penRight) {
            const impactH = Math.max(0, -this.velocity.x);
            if (impactH > sideCrash && !this.isCrashed) { this.isCrashed = true; this.playTone(135, 0.45); }
            this.position.x = exMaxX;
            this.velocity.x = Math.max(0, this.velocity.x) * -0.32;
          } else if (minPen === penBack) {
            const impactH = Math.max(0, this.velocity.z);
            if (impactH > sideCrash && !this.isCrashed) { this.isCrashed = true; this.playTone(135, 0.45); }
            this.position.z = exMinZ;
            this.velocity.z = Math.min(0, this.velocity.z) * -0.32;
          } else {
            const impactH = Math.max(0, -this.velocity.z);
            if (impactH > sideCrash && !this.isCrashed) { this.isCrashed = true; this.playTone(135, 0.45); }
            this.position.z = exMaxZ;
            this.velocity.z = Math.max(0, this.velocity.z) * -0.32;
          }
        }
      } else if (c.type === 'cyl') {
        const dx = this.position.x - c.x;
        const dz = this.position.z - c.z;
        const dist = Math.hypot(dx, dz);
        const minDist = c.radius + r;
        const roofY = c.maxY + h;

        if (dist < minDist && this.position.y >= roofY - 0.05 && roofY > this.surfaceSupportY) {
          this.surfaceSupportY = roofY;
        }

        if (this.position.y + h > c.minY && this.position.y - h < c.maxY && dist < minDist) {
          const penUp = roofY - this.position.y;
          const penSide = minDist - dist;
          if (penUp < penSide && this.velocity.y <= 0.5) {
            const impactV = Math.max(0, -this.velocity.y);
            this.position.y = roofY;
            this.surfaceSupportY = Math.max(this.surfaceSupportY, roofY);
            if (impactV > vertCrash && !this.isCrashed) {
              this.isCrashed = true;
              this.playTone(135, 0.45);
            }
            this.velocity.y = impactV > 3.0 ? impactV * 0.14 : Math.max(0, this.velocity.y);
            this.velocity.x *= roofFric;
            this.velocity.z *= roofFric;
            if (impactV <= 3.0 && this.input.throttle <= 0.04 && !(this.isRTHMode && this.rthPhase === 'CLIMB') && this.powerState !== 'AUTO_TAKEOFF') {
              this.isLanded = true;
            }
          } else {
            const nx = dist > 0.001 ? dx / dist : 1;
            const nz = dist > 0.001 ? dz / dist : 0;
            this.position.x = c.x + nx * minDist;
            this.position.z = c.z + nz * minDist;
            const vn = this.velocity.x * nx + this.velocity.z * nz;
            if (vn < -sideCrash && !this.isCrashed) { this.isCrashed = true; this.playTone(135, 0.45); }
            if (vn < 0) {
              this.velocity.x -= 1.32 * vn * nx;
              this.velocity.z -= 1.32 * vn * nz;
            }
          }
        }
      } else if (c.type === 'cone') {
        if (this.position.y >= 0 && this.position.y < c.height) {
          const t = Math.max(0, 1 - this.position.y / c.height);
          const radAtY = c.baseR * t;
          const dx = this.position.x - c.x;
          const dz = this.position.z - c.z;
          const dist = Math.hypot(dx, dz);
          const minDist = radAtY + r;
          if (dist < minDist) {
            const nx = dist > 0.001 ? dx / dist : 1;
            const nz = dist > 0.001 ? dz / dist : 0;
            this.position.x = c.x + nx * minDist;
            this.position.z = c.z + nz * minDist;
            const vn = this.velocity.x * nx + this.velocity.z * nz;
            if (vn < 0) {
              this.velocity.x -= 1.28 * vn * nx;
              this.velocity.z -= 1.28 * vn * nz;
            }
          }
        }
      }
    }
  }

  updateWindsock(delta) {
    if (!this.windsock) return;
    // Synchronized directly with the live 3D atmospheric wind field at the airfield windsock pole!
    const w = this.sampleWindField(-16.5, 5.5, 10.5, this.simTime || performance.now() * 0.001, this._windTmp);
    const windSpeed = Math.hypot(w.x, w.z);
    const t = this.simTime || performance.now() * 0.001;
    // Local +Z sock aligns with wind direction vector (w.x, w.z) via atan2(w.x, w.z)
    const windAngle = Math.atan2(w.x, w.z) + Math.sin(t * 2.1) * 0.04;
    this.windsock.rotation.y = windAngle;
    if (this.windsockPitch) {
      // Droops gracefully downward (+X rotation tilts +Z down!) in light breeze, stretches taut in strong wind!
      const droop = Math.max(0.06, 0.62 - Math.min(0.54, windSpeed * 0.125));
      this.windsockPitch.rotation.x = droop + Math.sin(t * 3.4) * (0.04 + Math.min(0.06, windSpeed * 0.01));
    }
    if (this.windsockMid) {
      this.windsockMid.rotation.x = 0.07 + Math.sin(t * 4.8 - 1.0) * (0.06 + windSpeed * 0.01);
      this.windsockMid.rotation.y = Math.sin(t * 3.8 - 0.8) * (0.07 + windSpeed * 0.012);
    }
    if (this.windsockTail) {
      this.windsockTail.rotation.x = 0.05 + Math.sin(t * 5.8 - 2.1) * (0.08 + windSpeed * 0.014);
      this.windsockTail.rotation.y = Math.cos(t * 4.6 - 1.6) * (0.10 + windSpeed * 0.015);
    }
  }

  updateCheckpoints() {
    if (this.checkpoints.length === 0) return;

    this.checkpoints.forEach(cp => {
      if (!cp.passed) {
        const dist = this.position.distanceTo(cp.pos);
        if (dist <= cp.radius) {
          cp.passed = true;
          this.completedCheckpoints++;
          cp.mesh.material.color.setHex(0x22c55e); // turn green
          this.playTone(880, 0.25); // pleasant chime

          if (this.onCheckpointPassed) {
            this.onCheckpointPassed(this.completedCheckpoints, this.checkpoints.length);
          }
        }
      }
    });
  }

  // Exact scalar Spring-Arm Raycast against SOLID buildings/structures (ignores tree foliage & thin poles so camera NEVER randomly jerks in flight, while NEVER seeing through houses/walls!)
  resolveCameraRayDist(pivot, dir, maxDist, camPad = 0.5) {
    let hitDist = maxDist;

    if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
      for (let i = 0; i < this.colliders.length; i++) {
        const c = this.colliders[i];

        if (c.type === 'box') {
          const minX = c.minX - camPad, maxX = c.maxX + camPad;
          const minY = c.minY - camPad, maxY = c.maxY + camPad;
          const minZ = c.minZ - camPad, maxZ = c.maxZ + camPad;

          let tMin = 0;
          let tMax = hitDist;
          let miss = false;

          if (Math.abs(dir.x) < 1e-6) {
            if (pivot.x < minX || pivot.x > maxX) miss = true;
          } else {
            let t1 = (minX - pivot.x) / dir.x;
            let t2 = (maxX - pivot.x) / dir.x;
            if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tMin) tMin = t1;
            if (t2 < tMax) tMax = t2;
            if (tMin > tMax) miss = true;
          }
          if (miss) continue;

          if (Math.abs(dir.y) < 1e-6) {
            if (pivot.y < minY || pivot.y > maxY) miss = true;
          } else {
            let t1 = (minY - pivot.y) / dir.y;
            let t2 = (maxY - pivot.y) / dir.y;
            if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tMin) tMin = t1;
            if (t2 < tMax) tMax = t2;
            if (tMin > tMax) miss = true;
          }
          if (miss) continue;

          if (Math.abs(dir.z) < 1e-6) {
            if (pivot.z < minZ || pivot.z > maxZ) miss = true;
          } else {
            let t1 = (minZ - pivot.z) / dir.z;
            let t2 = (maxZ - pivot.z) / dir.z;
            if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tMin) tMin = t1;
            if (t2 < tMax) tMax = t2;
            if (tMin > tMax) miss = true;
          }
          if (miss) continue;

          if (tMin > 0.01 && tMin < hitDist) {
            hitDist = tMin;
          }
        } else if (c.type === 'cyl' && c.radius >= 1.1) {
          // Only thick solid structures (Towers, Lighthouse, Silo, Windmill) — ignore skinny poles so passing a pole never snaps camera!
          const R = c.radius + camPad;
          const ox = pivot.x - c.x;
          const oz = pivot.z - c.z;
          const a = dir.x * dir.x + dir.z * dir.z;
          if (a > 1e-6) {
            const b = 2 * (ox * dir.x + oz * dir.z);
            const cc = ox * ox + oz * oz - R * R;
            const disc = b * b - 4 * a * cc;
            if (disc >= 0) {
              const tHit = (-b - Math.sqrt(disc)) / (2 * a);
              if (tHit > 0.01 && tHit < hitDist) {
                const hy = pivot.y + tHit * dir.y;
                if (hy >= c.minY - camPad && hy <= c.maxY + camPad) {
                  hitDist = tHit;
                }
              }
            }
          }
        }
      }
    } else if (this.customMapRoot && this.solidTerrainMeshes && this.solidTerrainMeshes.length > 0) {
      this.wallRaycaster.set(pivot, dir);
      this.wallRaycaster.far = hitDist;
      const hits = this.wallRaycaster.intersectObjects(this.solidTerrainMeshes, false);
      if (hits.length > 0 && hits[0].distance < hitDist) {
        hitDist = Math.max(0.4, hits[0].distance - camPad);
      }
    }

    return Math.max(0.45, hitDist);
  }

  // Soft FPV/interior barrier: keeps FPV camera strictly outside solid buildings & above terrain
  pushCameraOutsideColliders(pt, camPad = 0.28) {
    if (!this.customMapRoot && this.colliders && this.colliders.length > 0) {
      for (let i = 0; i < this.colliders.length; i++) {
        const c = this.colliders[i];

        if (c.type === 'box') {
          const exMinX = c.minX - camPad, exMaxX = c.maxX + camPad;
          const exMinY = c.minY - camPad, exMaxY = c.maxY + camPad;
          const exMinZ = c.minZ - camPad, exMaxZ = c.maxZ + camPad;
          if (
            pt.x > exMinX && pt.x < exMaxX &&
            pt.y > exMinY && pt.y < exMaxY &&
            pt.z > exMinZ && pt.z < exMaxZ
          ) {
            const pL = pt.x - exMinX;
            const pR = exMaxX - pt.x;
            const pD = pt.y - exMinY;
            const pU = exMaxY - pt.y;
            const pB = pt.z - exMinZ;
            const pF = exMaxZ - pt.z;
            const m = Math.min(pL, pR, pD, pU, pB, pF);
            if (m === pU) pt.y = exMaxY;
            else if (m === pD) pt.y = exMinY;
            else if (m === pL) pt.x = exMinX;
            else if (m === pR) pt.x = exMaxX;
            else if (m === pB) pt.z = exMinZ;
            else pt.z = exMaxZ;
          }
        } else if (c.type === 'cyl' && c.radius >= 1.1) {
          if (pt.y > c.minY - camPad && pt.y < c.maxY + camPad) {
            const dx = pt.x - c.x;
            const dz = pt.z - c.z;
            const dist = Math.hypot(dx, dz);
            const minD = c.radius + camPad;
            if (dist < minD) {
              const nx = dist > 0.001 ? dx / dist : 1;
              const nz = dist > 0.001 ? dz / dist : 0;
              pt.x = c.x + nx * minD;
              pt.z = c.z + nz * minD;
            }
          }
        }
      }
    }

    const minCamY = this.getTerrainHeight(pt.x, pt.z) + 0.45;
    if (pt.y < minCamY) pt.y = minCamY;
  }

  updateCamera(delta = 0.016) {
    // Clamp camera dt so single-frame GC spikes NEVER cause visible camera jumps!
    const dt = Math.min(Math.max(delta, 0.001), 0.033);

    // Smooth manual orbit offset & C-reset glide (exponential damping = zero snap!)
    const orbitAlpha = 1 - Math.exp(-14.0 * dt);
    this.camYaw += (this.targetCamYaw - this.camYaw) * orbitAlpha;
    this.camPitch += (this.targetCamPitch - this.camPitch) * orbitAlpha;

    // In FPV mode hide own drone mesh so arms/plates/props can NEVER clip into the camera lens at any speed or pitch!
    if (this.droneMesh) {
      this.droneMesh.visible = !this.isFPV;
    }

    if (this.isFPV) {
      // First Person Cockpit View (Rigidly mounted to nose/gimbal = ZERO high-speed trailing lag + silky gimbal rotation damping!)
      if (!this._fpvCamPos) {
        this._fpvCamPos = new THREE.Vector3();
        this._fpvNose = new THREE.Vector3();
        this._fpvEuler = new THREE.Euler(0, 0, 0, 'YXZ');
        this._fpvTargetQuat = new THREE.Quaternion();
        this._fpvLookQuat = new THREE.Quaternion();
        this._fpvFlipY = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI);
      }
      const p = this.physicsParams;
      const hasGimbal = Boolean(p && p.hasGimbal);
      const wbM = (p && p.wheelbase) || 0.45;
      const noseZ = wbM >= 0.74 ? 0.56 : wbM >= 0.56 ? 0.48 : wbM <= 0.30 ? 0.32 : 0.42;
      const noseY = hasGimbal ? -0.06 : 0.10;
      this._fpvNose.set(0, noseY, noseZ).applyEuler(this.rotation);
      this._fpvCamPos.copy(this.position).add(this._fpvNose);
      this.pushCameraOutsideColliders(this._fpvCamPos, 0.25);

      // Desired FPV orientation: 4K/NDVI/Thermal Gimbal stabilizes horizon; Fixed FPV locks to frame with 20° uptilt!
      const fpvPitch = hasGimbal ? (0.08 + this.rotation.x * 0.20) : (this.rotation.x + 0.34);
      const fpvRoll = hasGimbal ? (this.rotation.z * 0.06) : this.rotation.z;
      this._fpvEuler.set(fpvPitch, this.rotation.y, fpvRoll, 'YXZ');
      this._fpvTargetQuat.setFromEuler(this._fpvEuler).multiply(this._fpvFlipY);

      // Lock camera position rigidly to drone mount (no exponential trailing lag into fuselage when flying fast!)
      this.camera.position.copy(this._fpvCamPos);
      if (!this.camInitialized) {
        this.smoothFpvQuat.copy(this._fpvTargetQuat);
        this.camInitialized = true;
      } else {
        const rotA = 1 - Math.exp(-18.0 * dt);
        this.smoothFpvQuat.slerp(this._fpvTargetQuat, rotA);
      }
      this.camera.quaternion.copy(this.smoothFpvQuat);

      // Authentic wide-angle FPV lens FOV (84° on agile FPV cam, 74° on 4K/NDVI/Thermal gimbal + subtle speed kick!)
      const targetFpvFov = (hasGimbal ? 74 : 84) + Math.min(8, (this.speedKmh || 0) * 0.10);
      if (Math.abs(this.camera.fov - targetFpvFov) > 0.08) {
        this.camera.fov += (targetFpvFov - this.camera.fov) * (1 - Math.exp(-10.0 * dt));
        this.camera.updateProjectionMatrix();
      }
    } else {
      // Smoothly return to crisp 60° chase-cam FOV when in 3rd-person view
      if (Math.abs(this.camera.fov - 60) > 0.08) {
        this.camera.fov += (60 - this.camera.fov) * (1 - Math.exp(-10.0 * dt));
        this.camera.updateProjectionMatrix();
      }
      // AAA Polar Spring-Arm 3rd-Person Chase Camera (ZERO Cartesian chord wobble, ZERO tree-cone snaps, ZERO wall see-through!)
      if (!this._camPivot) {
        this._camPivot = new THREE.Vector3();
        this._camDir = new THREE.Vector3();
        this._camLook = new THREE.Vector3();
        this._upAxis = new THREE.Vector3(0, 1, 0);
      }

      const desiredTotalYaw = this.rotation.y + this.camYaw;
      const desiredPitch = this.camPitch;
      this._camPivot.set(this.position.x, this.position.y + 0.38, this.position.z);

      // Base spring-arm length & pitch elevation angle (scales cleanly with large 650/800mm wide-arm frames!)
      const pCam = this.physicsParams;
      const camScale = pCam && pCam.wheelbase >= 0.74 ? 1.20 : pCam && pCam.wheelbase >= 0.56 ? 1.09 : 1.0;
      const baseHoriz = 3.60 * camScale;
      const baseHeight = (1.18 + desiredPitch * 2.15) * camScale;
      const fullDist = Math.hypot(baseHoriz, baseHeight);

      if (!this.camInitialized) {
        this.smoothChasePos.copy(this._camPivot);
        this.smoothChaseYaw = desiredTotalYaw;
        this.smoothChasePitch = desiredPitch;
        this.currentCamDist = fullDist;
        this.camInitialized = true;
      } else {
        // Smooth follow pivot + shortest-angle polar yaw & pitch (cinema-smooth turning glide!)
        const pivotAlpha = 1 - Math.exp(-16.0 * dt);
        const yawAlpha = 1 - Math.exp(-8.8 * dt);
        const pitchAlpha = 1 - Math.exp(-10.5 * dt);

        this.smoothChasePos.lerp(this._camPivot, pivotAlpha);
        const yawDiff = Math.atan2(Math.sin(desiredTotalYaw - this.smoothChaseYaw), Math.cos(desiredTotalYaw - this.smoothChaseYaw));
        this.smoothChaseYaw += yawDiff * yawAlpha;
        this.smoothChasePitch += (desiredPitch - this.smoothChasePitch) * pitchAlpha;
      }

      // Compute exact unit spring-arm direction pointing behind & above the drone in polar coordinates!
      const elevY = (1.18 + this.smoothChasePitch * 2.15) * camScale;
      const armFullDist = Math.hypot(baseHoriz, elevY);
      const sinY = Math.sin(this.smoothChaseYaw);
      const cosY = Math.cos(this.smoothChaseYaw);
      // Local offset (0, elevY, -baseHoriz) rotated by smoothChaseYaw around +Y:
      // x = -baseHoriz * sinY, z = -baseHoriz * cosY
      this._camDir.set(-baseHoriz * sinY, elevY, -baseHoriz * cosY).divideScalar(armFullDist);

      // Single exact raycast along spring arm against solid buildings (0.55m cushion outside walls so pull-in glides smoothly!)
      const safeDist = this.resolveCameraRayDist(this.smoothChasePos, this._camDir, armFullDist, 0.55);

      // Pull in smoothly across the exterior cushion (fast enough to NEVER enter walls!), ease gently back out when clear!
      if (safeDist < this.currentCamDist) {
        const pullAlpha = 1 - Math.exp(-18.0 * dt);
        this.currentCamDist += (safeDist - this.currentCamDist) * pullAlpha;
        // Hard safety ceiling stays 0.23m strictly OUTSIDE the physical wall (zero see-through, zero violent snap!)
        if (this.currentCamDist > safeDist + 0.32) {
          this.currentCamDist = safeDist + 0.32;
        }
      } else {
        const extendAlpha = 1 - Math.exp(-5.0 * dt);
        this.currentCamDist += (safeDist - this.currentCamDist) * extendAlpha;
      }

      // Place camera cleanly on the smoothed polar spring arm
      this.camera.position.copy(this.smoothChasePos).addScaledVector(this._camDir, this.currentCamDist);

      // Smooth ground terrain clearance floor (prevents sharp vertical jumps over rolling hills!)
      const minTerrainY = this.getTerrainHeight(this.camera.position.x, this.camera.position.z) + 0.65;
      if (this.camera.position.y < minTerrainY) {
        this.camera.position.y = minTerrainY;
      }

      // Phase-locked look target ahead of the SAME smoothed pivot & yaw (ZERO look-at oscillation or wobble!)
      const lookFwd = 2.6;
      this._camLook.set(
        this.smoothChasePos.x + sinY * lookFwd,
        this.smoothChasePos.y + 0.05,
        this.smoothChasePos.z + cosY * lookFwd
      );
      this.camera.up.copy(this._upAxis);
      this.camera.lookAt(this._camLook);
    }
  }

  updateAudio() {
    if (!this.audioInitialized || !this.motorGain || !this.audioCtx) return;

    if (this.isMuted) {
      if (this.masterGain) {
        this.masterGain.gain.setTargetAtTime(0.0, this.audioCtx.currentTime, 0.05);
      }
      return;
    }

    const avgRpm = 0.25 * (this.motorRpm[0] + this.motorRpm[1] + this.motorRpm[2] + this.motorRpm[3]);
    if (avgRpm > 0.025) {
      const p = this.physicsParams || this.computeDronePhysicsParams();
      // Pitch signature reflects real prop diameter & motor KV (5" FPV crisp & higher; 12" heavy deep & throaty!)
      const kvPitchRatio = Math.max(0.76, Math.min(1.38, (p.motorKv / 920) * (10 / Math.max(5, p.propSizeIn)) * 0.72 + 0.28));
      // Motor differential spread adds authentic harmonic detune during aggressive maneuvers or VRS!
      const rpmSpread = Math.max(this.motorRpm[0], this.motorRpm[1], this.motorRpm[2], this.motorRpm[3])
        - Math.min(this.motorRpm[0], this.motorRpm[1], this.motorRpm[2], this.motorRpm[3]);
      const vrsFlutter = this.vrsIntensity * (0.5 + 0.5 * Math.sin(this.simTime * 29.0));

      const loadNorm = Math.min(1.0, avgRpm * avgRpm * 1.15);
      const targetGain = 0.008 + avgRpm * 0.018 + loadNorm * 0.020;
      this.motorGain.gain.setTargetAtTime(targetGain, this.audioCtx.currentTime, 0.06);

      // Filtered pink noise for authentic propeller blade-passing airflow & VRS propwash buffet
      if (this.noiseGain) {
        const targetNoise = 0.005 + loadNorm * 0.030 + vrsFlutter * 0.022;
        this.noiseGain.gain.setTargetAtTime(targetNoise, this.audioCtx.currentTime, 0.06);
      }

      // Dynamic smooth motor hum driven by actual lagged RPMs (pleasant soft pitch, never harsh)
      const f1 = (58 + avgRpm * 138) * kvPitchRatio;
      const f2 = f1 * (2.0 + rpmSpread * 0.32 + vrsFlutter * 0.08);
      this.motorOsc1.frequency.setTargetAtTime(f1, this.audioCtx.currentTime, 0.06);
      this.motorOsc2.frequency.setTargetAtTime(f2, this.audioCtx.currentTime, 0.06);

      if (this.audioFilter) {
        this.audioFilter.frequency.setTargetAtTime(280 + loadNorm * 420 + rpmSpread * 160, this.audioCtx.currentTime, 0.06);
      }
      if (this.noiseFilter) {
        this.noiseFilter.frequency.setTargetAtTime(220 + loadNorm * 560 + vrsFlutter * 240, this.audioCtx.currentTime, 0.06);
      }

      if (this.masterGain) {
        this.masterGain.gain.setTargetAtTime(0.22, this.audioCtx.currentTime, 0.08);
      }
    } else {
      // Smooth fade out once rotors coast to a halt
      this.motorGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.08);
      if (this.noiseGain) {
        this.noiseGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.08);
      }
    }

    // Ambient atmospheric wind whoosh (swells gently in strong crosswind & rolling gusts!)
    if (this.windGain && this.windFilter) {
      const wspd = this.liveWindSpeed || 0;
      if (wspd > 3.2) {
        const wNorm = Math.min(1.0, (wspd - 3.2) / 11.5);
        const gustBoost = this.windGustActive ? 1.35 : 1.0;
        this.windGain.gain.setTargetAtTime(Math.min(0.24, wNorm * 0.16 * gustBoost), this.audioCtx.currentTime, 0.12);
        this.windFilter.frequency.setTargetAtTime(240 + wNorm * 580 + (this.windGustActive ? 140 : 0), this.audioCtx.currentTime, 0.12);
      } else {
        this.windGain.gain.setTargetAtTime(0.0001, this.audioCtx.currentTime, 0.12);
      }
    }
  }

  triggerTelemetry() {
    if (this.onTelemetryUpdate) {
      const p = this.physicsParams || this.computeDronePhysicsParams();
      const mins = Math.floor(this.flightTimeSeconds / 60);
      const secs = Math.floor(this.flightTimeSeconds % 60);
      const timeStr = `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;

      let flightModeLabel = 'ATTI';
      if (this.isRTHMode) {
        const phaseTh = this.rthPhase === 'PATH_RETURN'
          ? `ตามเส้นทาง ${Math.max(1, (this.rthWaypoints ? this.rthWaypoints.length : 1) - this.rthWaypointIdx)} จุด`
          : this.rthPhase === 'CLIMB' ? 'ไต่ระดับ' : this.rthPhase === 'RETURN' ? 'บินกลับ' : 'ลงจอด';
        flightModeLabel = (this.isFailsafeReturn || this.sensorLostSim)
          ? `AUTO-RETURN (${phaseTh})`
          : `RTH (${phaseTh})`;
      } else if (this.isHoldMode && p.hasGps) {
        flightModeLabel = p.isRtk ? 'RTK HOLD' : 'GPS HOLD';
      } else if (p.hasBaro) {
        flightModeLabel = 'BARO ATTI';
      } else {
        flightModeLabel = 'MANUAL F4';
      }

      const activeAlert = performance.now() < (this.hudAlertUntil || 0) ? this.hudAlertText : '';
      const curMassKg = p.dryMass + p.payloadTankKg * (this.payloadPercent / 100);
      const curMassGrams = Math.round(curMassKg * 1000);
      const liveTwOge = Number(((this.liveTwOge !== undefined ? this.liveTwOge : (p.maxThrustN / (curMassKg * p.gravity)))).toFixed(2));
      const liveTwRatio = Number(((this.liveTwRatio !== undefined ? this.liveTwRatio : liveTwOge)).toFixed(2));

      this.onTelemetryUpdate({
        altitude: this.altitude,
        maxAltitudeLimitM: p.maxAltitudeLimitM || 90,
        frameClassLabel: p.frameClassLabel || 'โดรนมาตรฐาน',
        hitAltitudeLimit: Boolean(this.hitAltitudeLimit),
        groundClearanceM: this.groundClearanceM || 0,
        homeAltitudeM: this.homeAltitudeM || 0,
        altSensorType: this.altSensorType || (p.hasLidar ? 'RADAR+TOF' : p.hasOpticalFlow ? 'SONAR' : p.hasBaro ? 'BARO' : 'NONE'),
        altSensorLocked: Boolean(this.altSensorLocked),
        speedKmh: this.speedKmh,
        timeStr,
        flightTimeSeconds: this.flightTimeSeconds,
        batteryPercent: Math.round(this.batteryPercent),
        batteryVoltage: Number(this.batteryVoltage.toFixed(1)),
        currentAmps: Number(this.currentAmps.toFixed(1)),
        rssiPercent: this.rssiPercent,
        signalState: this.signalState || 'good',
        signalSource: this.signalSource || 'clear',
        signalObstruction: Number((this.signalObstruction || 0).toFixed(2)),
        insideMetalRoof: Boolean(this.insideMetalRoof),
        rxType: p.rxType || 'elrs',
        payloadType: p.payloadType,
        payloadPercent: Math.round(this.payloadPercent),
        isSpraying: this.isSpraying,
        cargoAttached: this.cargoAttached,
        lidarBraking: this.lidarBraking,
        lidarZone: this.lidarZone || (p.hasLidar ? 'clear' : 'off'),
        lidarNearestDist: this.lidarNearestDist,
        lidarNearestDirTh: this.lidarNearestDirTh || '',
        lidarNearestTag: this.lidarNearestTag || '',
        parachuteDeployed: this.parachuteDeployed,
        hasLidar: p.hasLidar,
        hasOpticalFlow: p.hasOpticalFlow,
        hasParachute: p.hasParachute,
        curMassGrams,
        liveTwRatio,
        liveTwOge,
        isOverloaded: liveTwOge <= 1.02,
        isHeavyLoad: liveTwOge > 1.02 && liveTwOge < 1.55,
        controlRateMode: this.controlRateMode || 'normal',
        controlRateLabel: this.getControlRateProfile().label,
        graphicsQuality: this.graphicsQuality || 'balanced',
        fps: this.liveFps || 60,
        maxAltitude: this.maxAltitude,
        isArmed: this.isArmed,
        powerState: this.powerState,
        touchdownHoldPct: Math.min(100, Math.round((this.touchdownHoldTimer / 1.1) * 100)),
        isLanded: this.isLanded,
        isCrashed: this.isCrashed,
        isHoldMode: this.isHoldMode,
        isRTHMode: this.isRTHMode,
        rthPhase: this.rthPhase,
        isFailsafeReturn: Boolean(this.isFailsafeReturn),
        sensorLostSim: Boolean(this.sensorLostSim),
        rthAlgoLabel: this.rthAlgoLabel || '',
        rthWaypointsRemaining: this.rthWaypoints ? Math.max(0, this.rthWaypoints.length - (this.rthWaypointIdx || 0)) : 0,
        rthTotalPathDist: this.rthTotalPathDist || 0,
        signalLossCountdown: this.signalLossTimer > 0.25 ? Number(Math.max(0, 2.2 - this.signalLossTimer).toFixed(1)) : 0,
        distFromHomeM: Math.round(Math.hypot(this.position.x, this.position.z)),
        hasGps: p.hasGps,
        hasBaro: p.hasBaro,
        flightModeLabel,
        vrsIntensity: this.vrsIntensity,
        igeFactor: this.igeFactor,
        liveWindSpeed: Number(this.liveWindSpeed.toFixed(1)),
        liveWindDirDeg: this.liveWindDirDeg || 0,
        liveWindVertical: Number((this.liveWindVertical || 0).toFixed(1)),
        windPreset: this.windPreset || 'mission',
        windPresetShort: this.getWindPresetInfo().shortLabel,
        windLevel: this.windLevel || 'calm',
        windGustActive: Boolean(this.windGustActive),
        windShelterPct: Math.round((1 - (this.windShelterFactor ?? 1.0)) * 100),
        windScreenDeg: (() => {
          const yView = this.isFPV ? (this.rotation.y || 0) : (this.smoothChaseYaw ?? (this.rotation.y + (this.camYaw || 0)));
          const sY = Math.sin(yView);
          const cY = Math.cos(yView);
          const wRight = -this.liveWindVec.x * cY + this.liveWindVec.z * sY;
          const wUp = this.liveWindVec.x * sY + this.liveWindVec.z * cY;
          return Math.round(Math.atan2(-wUp, wRight) * (180 / Math.PI));
        })(),
        isFPV: Boolean(this.isFPV),
        yawDeg: Math.round((((-(this.rotation.y || 0)) * (180 / Math.PI)) % 360 + 360) % 360),
        verticalSpeedMs: Number(((this.velocity && this.velocity.y) || 0).toFixed(1)),
        homeRelBearingDeg: (() => {
          const toHomeX = -(this.position ? this.position.x : 0);
          const toHomeZ = -(this.position ? this.position.z : 0);
          if (Math.hypot(toHomeX, toHomeZ) < 1.2) return 0;
          const y = (this.rotation && this.rotation.y) || 0;
          const sY = Math.sin(y), cY = Math.cos(y);
          const fwd = toHomeX * sY + toHomeZ * cY;
          const right = -toHomeX * cY + toHomeZ * sY;
          return Math.round(Math.atan2(right, fwd) * (180 / Math.PI));
        })(),
        cells: p.cells || 4,
        cellVoltage: Number(((this.batteryVoltage || ((p.cells || 4) * 3.8)) / (p.cells || 4)).toFixed(2)),
        throttlePct: (() => {
          if (!this.isArmed) return 0;
          const rpms = this.motorRpm || [0, 0, 0, 0];
          const avg = (rpms[0] + rpms[1] + rpms[2] + rpms[3]) * 0.25;
          return Math.round(Math.max(0, Math.min(100, avg * 100)));
        })(),
        hasGimbal: Boolean(p.hasGimbal),
        cameraType: (this.stats && this.stats.resolvedParts && this.stats.resolvedParts.camera && this.stats.resolvedParts.camera.type) || 'gimbal_4k',
        cameraName: (this.stats && this.stats.resolvedParts && this.stats.resolvedParts.camera && this.stats.resolvedParts.camera.name) || '4K Gimbal',
        isSprayDrift: Boolean(this.isSpraying && !this.isLanded && this.liveWindSpeed >= 5.5),
        lawOverCeiling: this.lawOverCeiling,
        lawAltitudeViolated: this.lawAltitudeViolated,
        hudAlertText: activeAlert,
        checkpointsPassed: this.completedCheckpoints,
        totalCheckpoints: this.checkpoints.length,
        pitch: this.rotation.x,
        roll: this.rotation.z,
        inputDevice: this.activeInputDevice || 'KEYBOARD'
      });
    }
  }

  // End flight and trigger debrief
  finishFlight() {
    if (this.onFlightFinished) {
      const mins = Math.floor(this.flightTimeSeconds / 60);
      const secs = Math.floor(this.flightTimeSeconds % 60);
      const timeStr = `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;

      this.onFlightFinished({
        timeStr,
        flightTimeSeconds: this.flightTimeSeconds,
        batteryRemainPercent: this.batteryPercent,
        maxAltitude: this.maxAltitude,
        lawAltitudeViolated: this.lawAltitudeViolated,
        isCrashed: this.isCrashed,
        checkpointsCompleted: this.completedCheckpoints,
        totalCheckpoints: this.checkpoints.length,
        isSuccess: !this.isCrashed && (this.checkpoints.length === 0 || this.completedCheckpoints === this.checkpoints.length)
      });
    }
  }

  onWindowResize() {
    const width = this.container.clientWidth || window.innerWidth;
    const height = this.container.clientHeight || window.innerHeight;
    if (this.camera && this.renderer) {
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height);
    }
  }
}

window.DroneFlightSimulator = DroneFlightSimulator;
