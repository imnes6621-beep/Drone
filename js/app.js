/**
 * Drone Builder — Main Application State Machine & UI Controller
 * 100% Parity with https://drone-builder.surge.sh/
 */

class DroneApp {
  constructor() {
    // Start with empty assembly (0/8) like drone-builder.surge.sh, or load recommended on Free Flight
    this.assembly = { ...DRONE_PRESETS.default_aerial.parts };
    this.currentScreen = 'home';
    this.currentViewAngle = 'iso';
    this.isExploded = false;
    this.lastSelectedPartKey = 'gps';

    this.workbenchScene = null;
    this.workbenchCamera = null;
    this.workbenchRenderer = null;
    this.workbenchDroneBuilder = null;
    this.workbenchDroneMesh = null;
    this.workbenchControls = null;

    this.flightSim = null;
    this.selectedMission = MISSIONS[0];

    this.activeSelectingPartKey = null;
    this.pendingOptionId = null;

    // Learning Module, Pre/Post-test Quiz, Student Profile & Mission Progress
    this.activeUnitId = 'unit1';
    this.quizMode = 'pre'; // 'pre' | 'post' | 'compare' | 'eval'
    this.quizAnswers = {};
    this.quizSubmitted = false;
    this.savedScores = this.loadSavedScores();
    this.evalRecords = this.loadEvalRecords();
    this.studentProfile = this.loadStudentProfile();
    this.completedMissions = this.loadCompletedMissions();
    this.evalForm = {
      name: this.studentProfile.name || '',
      studentId: this.studentProfile.studentId || '',
      ratings: { c1: 5, c2: 5, c3: 5, c4: 5, c5: 5, c6: 5 }
    };

    this.homeScene = null;
    this.homeCamera = null;
    this.homeRenderer = null;
    this.homeDroneMesh = null;
    this.homeDroneBuilder = null;
    this.homeRafId = null;
    this.workbenchRafId = null;

    this.showCallouts = false; // Default clean hover-to-inspect mode
    this.isPaintOpen = false;
    this.workbenchSpinTest = false;
    this.workbenchSpinRpm = 0.08;
    this._uiAudioCtx = null;
    this.paintConfig = this.loadPaintConfig();
    this.paintLiveries = [
      { id: 'factory', name: 'Factory ส้ม', colors: { frame: '#343a46', accent: '#ff6a00', propFront: '#ff6a00', propRear: '#f1f5f9', led: '#38bdf8', finish: 'carbon' } },
      { id: 'arctic', name: 'Arctic ขาวมุก', colors: { frame: '#e2e8f0', accent: '#06b6d4', propFront: '#06b6d4', propRear: '#f8fafc', led: '#38bdf8', finish: 'gloss' } },
      { id: 'agri', name: 'Agri เกษตร', colors: { frame: '#15803d', accent: '#facc15', propFront: '#22c55e', propRear: '#facc15', led: '#22c55e', finish: 'matte' } },
      { id: 'stealth', name: 'Stealth ทอง', colors: { frame: '#1e222b', accent: '#f59e0b', propFront: '#f59e0b', propRear: '#334155', led: '#f59e0b', finish: 'matte' } },
      { id: 'rescue', name: 'Rescue กู้ภัย', colors: { frame: '#b91c1c', accent: '#f8fafc', propFront: '#ef4444', propRear: '#f8fafc', led: '#facc15', finish: 'gloss' } },
      { id: 'cyber', name: 'Cyber นีออน', colors: { frame: '#1e3a8a', accent: '#a855f7', propFront: '#ec4899', propRear: '#06b6d4', led: '#a855f7', finish: 'metallic' } }
    ];
    this.paintZones = [
      { key: 'frame', label: 'โครง / แขน', swatches: ['#343a46', '#1e222b', '#e2e8f0', '#15803d', '#b91c1c', '#1d4ed8', '#c29b61'] },
      { key: 'accent', label: 'อะไหล่ CNC / กิมบอล', swatches: ['#ff6a00', '#06b6d4', '#facc15', '#84cc16', '#ef4444', '#a855f7', '#cbd5e1'] },
      { key: 'propFront', label: 'ใบพัดคู่หน้า', swatches: ['#ff6a00', '#22c55e', '#06b6d4', '#facc15', '#ef4444', '#ec4899', '#f1f5f9'] },
      { key: 'propRear', label: 'ใบพัดคู่หลัง', swatches: ['#f1f5f9', '#ff6a00', '#06b6d4', '#facc15', '#a855f7', '#22c55e', '#334155'] },
      { key: 'led', label: 'ไฟ LED นำทาง', swatches: ['#38bdf8', '#22c55e', '#facc15', '#ff6a00', '#ec4899', '#a855f7', '#ffffff'] }
    ];
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.hoveredPartKey = null;
    this.sidebarHoveredPartKey = null;
    this.calloutDefinitions = [
      { key: 'frame', label: 'A · โครงโดรน/แขน', pos: new THREE.Vector3(-0.38, 0.02, 0.36) },
      { key: 'motor', label: 'B · มอเตอร์', pos: new THREE.Vector3(0.62, 0.09, 0.62) },
      { key: 'propeller', label: 'C · ใบพัด CW/CCW', pos: new THREE.Vector3(-0.78, 0.22, 0.78) },
      { key: 'esc', label: 'D · ESC 4-in-1', pos: new THREE.Vector3(0.20, 0.05, -0.04) },
      { key: 'battery', label: 'E · แบตเตอรี่', pos: new THREE.Vector3(0.0, 0.34, -0.02) },
      { key: 'fc', label: 'F · FC + IMU', pos: new THREE.Vector3(-0.24, 0.12, -0.06) },
      { key: 'camera', label: 'G · กล้อง/เซนเซอร์ภาพ', pos: new THREE.Vector3(0.0, -0.16, 0.38) },
      { key: 'gps', label: 'H · GPS/RTK', pos: new THREE.Vector3(0.14, 0.54, -0.22) },
      { key: 'rx', label: 'I · ตัวรับสัญญาณ RX', pos: new THREE.Vector3(-0.18, 0.16, -0.38) },
      { key: 'sensor', label: 'J · LiDAR/Optical Flow', pos: new THREE.Vector3(0.10, 0.30, 0.24) },
      { key: 'payload', label: 'K · ถังปุ๋ย/เพย์โหลด', pos: new THREE.Vector3(0.0, -0.28, -0.02) },
      { key: 'safety', label: 'L · ขาตั้ง/กันชน/ร่ม', pos: new THREE.Vector3(0.28, -0.30, 0.12) }
    ];

    this.init();
  }

  init() {
    this.bindEvents();
    this.initPaintStudio();
    this.initHomePreview3D();
    this.initWorkbench3D();
    this.initFlightSim();
    this.updateAssemblyUI();
    this.showScreen('home');
  }

  loadPaintConfig() {
    const def = (typeof DroneModelBuilder !== 'undefined' && DroneModelBuilder.getDefaultPaint)
      ? DroneModelBuilder.getDefaultPaint()
      : { frame: '#343a46', accent: '#ff6a00', propFront: '#ff6a00', propRear: '#f1f5f9', led: '#38bdf8', finish: 'carbon' };
    try {
      const raw = localStorage.getItem('droneBuilder_paint_v1');
      if (raw) return Object.assign({}, def, JSON.parse(raw));
    } catch (e) {}
    return def;
  }

  savePaintConfig() {
    try {
      localStorage.setItem('droneBuilder_paint_v1', JSON.stringify(this.paintConfig));
    } catch (e) {}
  }

  initPaintStudio() {
    const panel = document.getElementById('paint-studio-panel');
    const btnToggle = document.getElementById('btn-toggle-paint');
    const btnClose = document.getElementById('btn-paint-close');
    const btnReset = document.getElementById('btn-paint-reset');
    const liveryGrid = document.getElementById('paint-livery-grid');
    const finishRow = document.getElementById('paint-finish-row');
    const zonesList = document.getElementById('paint-zones-list');
    if (!panel || !liveryGrid || !zonesList) return;

    const setOpen = (open) => {
      this.isPaintOpen = open;
      panel.classList.toggle('hidden', !open);
      if (btnToggle) btnToggle.classList.toggle('active', open);
    };

    if (btnToggle) btnToggle.addEventListener('click', () => setOpen(!this.isPaintOpen));
    if (btnClose) btnClose.addEventListener('click', () => setOpen(false));
    if (btnReset) {
      btnReset.addEventListener('click', () => {
        this.paintConfig = DroneModelBuilder.getDefaultPaint();
        this.applyPaintLive();
        this.showStageToast('🎨 คืนค่าสีเริ่มต้น (Factory Orange)');
      });
    }

    // 1. Render 6 Pro Livery Chips
    liveryGrid.innerHTML = '';
    this.paintLiveries.forEach(liv => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn-livery-chip';
      btn.dataset.livery = liv.id;
      btn.innerHTML = `
        <span class="livery-dots">
          <span class="livery-dot" style="background:${liv.colors.frame}"></span>
          <span class="livery-dot" style="background:${liv.colors.accent}"></span>
        </span>
        <span>${liv.name}</span>
      `;
      btn.addEventListener('click', () => {
        this.paintConfig = Object.assign({}, liv.colors);
        this.applyPaintLive();
        this.showStageToast(`🎨 ใช้ธีมสี: ${liv.name}`);
      });
      liveryGrid.appendChild(btn);
    });

    // 2. Bind 4 Finish Chips
    if (finishRow) {
      finishRow.querySelectorAll('.btn-finish-chip').forEach(btn => {
        btn.addEventListener('click', () => {
          this.paintConfig.finish = btn.dataset.finish;
          this.applyPaintLive();
        });
      });
    }

    // 3. Render 5 Color Zone Rows (7 curated swatches + native custom rainbow color picker!)
    zonesList.innerHTML = '';
    this.paintZones.forEach(zone => {
      const row = document.createElement('div');
      row.className = 'paint-zone-row';
      row.dataset.zone = zone.key;

      const head = document.createElement('div');
      head.className = 'paint-zone-head';
      head.innerHTML = `<span>${zone.label}</span><span class="paint-zone-hex" data-hex-label="${zone.key}">${this.paintConfig[zone.key]}</span>`;
      row.appendChild(head);

      const bar = document.createElement('div');
      bar.className = 'paint-swatches-bar';

      zone.swatches.forEach(hex => {
        const sw = document.createElement('button');
        sw.type = 'button';
        sw.className = 'paint-swatch-btn';
        sw.style.backgroundColor = hex;
        sw.dataset.zone = zone.key;
        sw.dataset.hex = hex.toLowerCase();
        sw.title = hex;
        sw.addEventListener('click', () => {
          this.paintConfig[zone.key] = hex;
          this.applyPaintLive();
        });
        bar.appendChild(sw);
      });

      const picker = document.createElement('input');
      picker.type = 'color';
      picker.className = 'paint-color-wheel';
      picker.dataset.zone = zone.key;
      picker.value = this.paintConfig[zone.key];
      picker.title = 'เลือกสีอิสระ';
      picker.addEventListener('input', (e) => {
        this.paintConfig[zone.key] = e.target.value;
        this.applyPaintLive();
      });
      bar.appendChild(picker);

      row.appendChild(bar);
      zonesList.appendChild(row);
    });

    this.syncPaintStudioUI();
  }

  syncPaintStudioUI() {
    const cfg = this.paintConfig;
    // Sync livery active state
    document.querySelectorAll('.btn-livery-chip').forEach(btn => {
      const liv = this.paintLiveries.find(l => l.id === btn.dataset.livery);
      const match = liv
        && liv.colors.frame.toLowerCase() === cfg.frame.toLowerCase()
        && liv.colors.accent.toLowerCase() === cfg.accent.toLowerCase()
        && liv.colors.propFront.toLowerCase() === cfg.propFront.toLowerCase()
        && liv.colors.finish === cfg.finish;
      btn.classList.toggle('active', Boolean(match));
    });
    // Sync finish chips
    document.querySelectorAll('.btn-finish-chip').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.finish === cfg.finish);
    });
    // Sync swatches, hex labels & color inputs
    this.paintZones.forEach(z => {
      const curHex = (cfg[z.key] || '').toLowerCase();
      const lbl = document.querySelector(`[data-hex-label="${z.key}"]`);
      if (lbl) lbl.innerText = curHex;
      const inp = document.querySelector(`.paint-color-wheel[data-zone="${z.key}"]`);
      if (inp && inp.value.toLowerCase() !== curHex) inp.value = curHex;
      document.querySelectorAll(`.paint-swatch-btn[data-zone="${z.key}"]`).forEach(sw => {
        sw.classList.toggle('active', sw.dataset.hex === curHex);
      });
    });
  }

  applyPaintLive() {
    this.savePaintConfig();
    this.syncPaintStudioUI();
    if (this.workbenchDroneBuilder) {
      this.workbenchDroneBuilder.setHighlightedPart(null);
      this.workbenchDroneBuilder.applyPaint(this.paintConfig);
    }
    if (this.homeDroneBuilder) {
      this.homeDroneBuilder.applyPaint(this.paintConfig);
    }
    if (this.flightSim && this.flightSim.droneBuilder) {
      this.flightSim.paintConfig = this.paintConfig;
      this.flightSim.droneBuilder.applyPaint(this.paintConfig);
    }
  }

  // 3D Preview Scene for Home Screen (Helipad + Snow Mountains + Hovering Drone)
  initHomePreview3D() {
    const container = document.getElementById('home-canvas-viewport');
    if (!container) return;

    this.homeScene = new THREE.Scene();
    this.homeScene.background = new THREE.Color(0x3b7fb8);
    this.homeScene.fog = new THREE.FogExp2(0x4b8cc4, 0.0068);

    const w = container.clientWidth || 720;
    const h = container.clientHeight || 390;

    this.homeCamera = new THREE.PerspectiveCamera(42, w / h, 0.1, 500);
    this.homeCamera.position.set(0, 2.35, -4.6);
    this.homeCamera.lookAt(0, 1.15, 0.6);

    this.homeRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    this.homeRenderer.setSize(w, h);
    this.homeRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.35));
    this.homeRenderer.shadowMap.enabled = true;
    container.appendChild(this.homeRenderer.domElement);

    this.homeScene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e3a20, 0.9));
    const dirLight = new THREE.DirectionalLight(0xfff1dc, 1.25);
    dirLight.position.set(35, 55, -35);
    dirLight.castShadow = true;
    this.homeScene.add(dirLight);

    // Soft natural alpine meadow ground (harmonizes with Dark Telemetry UI)
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(240, 240),
      new THREE.MeshStandardMaterial({ color: 0x2d4f28, roughness: 0.92, metalness: 0.04 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.homeScene.add(ground);

    // Helipad with concentric cyan rings & H mark
    const pad = new THREE.Mesh(
      new THREE.CylinderGeometry(3.8, 4.0, 0.08, 48),
      new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.65 })
    );
    pad.position.set(0, 0.04, 0);
    this.homeScene.add(pad);

    const outerRing = new THREE.Mesh(
      new THREE.RingGeometry(3.35, 3.55, 48),
      new THREE.MeshBasicMaterial({ color: 0x5ce1e6, side: THREE.DoubleSide })
    );
    outerRing.rotation.x = -Math.PI / 2;
    outerRing.position.set(0, 0.09, 0);
    this.homeScene.add(outerRing);

    const innerRing = new THREE.Mesh(
      new THREE.RingGeometry(2.55, 2.7, 48),
      new THREE.MeshBasicMaterial({ color: 0x5ce1e6, side: THREE.DoubleSide })
    );
    innerRing.rotation.x = -Math.PI / 2;
    innerRing.position.set(0, 0.09, 0);
    this.homeScene.add(innerRing);

    const hMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    [[-0.75, 0.45, 2.3], [0.75, 0.45, 2.3], [0, 1.1, 0.45]].forEach(([x, bw, bh]) => {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(bw, bh), hMat);
      m.rotation.x = -Math.PI / 2;
      m.position.set(x, 0.1, 0);
      this.homeScene.add(m);
    });

    // Snowy Mountains (deeper slate blue + crisp snow)
    const mountMat = new THREE.MeshStandardMaterial({ color: 0x4b5d75, roughness: 0.92, flatShading: true });
    const snowMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.75, flatShading: true });
    [[-45, 70, 32, 46], [-10, 82, 36, 52], [35, 75, 30, 44], [65, 68, 28, 38]].forEach(([mx, mz, mr, mh]) => {
      const m = new THREE.Mesh(new THREE.ConeGeometry(mr, mh, 6), mountMat);
      m.position.set(mx, mh / 2, mz);
      this.homeScene.add(m);
      const cap = new THREE.Mesh(new THREE.ConeGeometry(mr * 0.38, mh * 0.36, 6), snowMat);
      cap.position.set(mx, mh * 0.82, mz);
      this.homeScene.add(cap);
    });

    // Trees (softer forest green)
    const trunkMat = new THREE.MeshStandardMaterial({ color: 0x5c361a });
    const folMat = new THREE.MeshStandardMaterial({ color: 0x1b4328, flatShading: true });
    [[-9, 10], [-14, 6], [10, 14], [15, 8], [-11, -3], [13, -5]].forEach(([tx, tz]) => {
      const t = new THREE.Group();
      t.position.set(tx, 0, tz);
      const tr = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.26, 1.3, 6), trunkMat);
      tr.position.y = 0.65;
      t.add(tr);
      const fo = new THREE.Mesh(new THREE.DodecahedronGeometry(1.3, 0), folMat);
      fo.position.y = 2.1;
      t.add(fo);
      this.homeScene.add(t);
    });

    // Drone (baked for silky Home preview!)
    this.homeDroneBuilder = new DroneModelBuilder();
    this.homeDroneMesh = this.homeDroneBuilder.buildDrone(this.assembly, THREE, this.paintConfig);
    if (typeof this.homeDroneBuilder.bakeForFlight === 'function') this.homeDroneBuilder.bakeForFlight(THREE);
    this.homeDroneMesh.position.set(0, 1.5, 0);
    this.homeScene.add(this.homeDroneMesh);

    this._animateHome = () => {
      if (this.currentScreen !== 'home') {
        this.homeRafId = null;
        return;
      }
      this.homeRafId = requestAnimationFrame(this._animateHome);
      const t = Date.now() * 0.002;
      if (this.homeDroneMesh) {
        this.homeDroneMesh.position.y = 1.45 + Math.sin(t * 2) * 0.08;
        this.homeDroneMesh.rotation.y = Math.sin(t * 0.4) * 0.38;
        this.homeDroneMesh.rotation.z = Math.sin(t) * 0.025;
        this.homeDroneMesh.rotation.x = Math.cos(t * 1.4) * 0.02;
      }
      if (this.homeDroneBuilder) {
        this.homeDroneBuilder.update(0.016, true, 30);
      }
      this.homeRenderer.render(this.homeScene, this.homeCamera);
    };

    window.addEventListener('resize', () => {
      if (this.homeRenderer && this.homeCamera && container.clientWidth > 0) {
        this.homeCamera.aspect = container.clientWidth / container.clientHeight;
        this.homeCamera.updateProjectionMatrix();
        this.homeRenderer.setSize(container.clientWidth, container.clientHeight);
      }
    });
  }

  syncActiveRenderLoops(screenId) {
    // Only run the single WebGL render loop for the currently visible screen (0% background CPU/GPU waste!)
    if (screenId === 'home') {
      if (this._animateHome && this.homeRafId === null) {
        this.homeRafId = requestAnimationFrame(this._animateHome);
      }
    } else if (this.homeRafId !== null) {
      cancelAnimationFrame(this.homeRafId);
      this.homeRafId = null;
    }

    if (screenId === 'assembly') {
      if (this._animateWorkbench && this.workbenchRafId === null) {
        this.workbenchRafId = requestAnimationFrame(this._animateWorkbench);
      }
    } else if (this.workbenchRafId !== null) {
      cancelAnimationFrame(this.workbenchRafId);
      this.workbenchRafId = null;
    }

    if (this.flightSim) {
      if (screenId === 'simulator') {
        if (typeof this.flightSim.startLoop === 'function') this.flightSim.startLoop();
      } else {
        if (typeof this.flightSim.stopLoop === 'function') this.flightSim.stopLoop();
      }
    }
  }

  showScreen(screenId) {
    const stats = calculateDroneStats(this.assembly);
    if ((screenId === 'readiness' || screenId === 'missions') && !stats.isComplete) {
      const totalSlots = stats.totalSlots || Object.keys(DRONE_PARTS).length;
      this.showStageToast(`ต้องประกอบให้ครบ ${totalSlots} ชิ้นก่อน`);
      screenId = 'assembly';
    }

    this.currentScreen = screenId;

    // Toggle header visibility: hidden on Home and Mission Flight, visible on Tabs 1–4 and Free Flight
    const header = document.getElementById('main-app-header');
    if (header) {
      const isMissionFlight = screenId === 'simulator' && this.selectedMission && this.selectedMission.id !== 'free_flight';
      header.classList.toggle('home-hidden', screenId === 'home' || isMissionFlight);
    }

    // Full height simulator when header hidden
    const simScreen = document.querySelector('.flight-sim-screen');
    if (simScreen) {
      const isMissionFlight = screenId === 'simulator' && this.selectedMission && this.selectedMission.id !== 'free_flight';
      simScreen.style.height = isMissionFlight ? '100vh' : 'calc(100vh - 58px)';
    }

    document.querySelectorAll('.screen-view').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.nav-tab-btn').forEach(btn => {
      if (btn.id === 'btn-nav-thai-law') {
        btn.classList.toggle('active', screenId === 'lessons' && this.activeUnitId === 'unit6');
      } else if (btn.dataset.screen === 'lessons') {
        btn.classList.toggle('active', screenId === 'lessons' && this.activeUnitId !== 'unit6');
      } else {
        btn.classList.toggle('active', btn.dataset.screen === screenId);
      }
    });

    // Highlight Free Flight shortcut button only when in Free Flight
    const btnTopFree = document.getElementById('btn-top-freeflight');
    if (btnTopFree) {
      const inFreeFlight = screenId === 'simulator' && this.selectedMission && this.selectedMission.id === 'free_flight';
      btnTopFree.classList.toggle('primary', inFreeFlight);
    }

    const target = document.getElementById(`screen-${screenId}`);
    if (target) target.classList.add('active');

    if (screenId !== 'simulator' && this.flightSim) {
      this.flightSim.isArmed = false;
      this.flightSim.keys = {};
    }

    if (screenId === 'home') {
      this.updateHomeDashboard();
    } else if (screenId === 'assembly') {
      this.refreshWorkbench();
    } else if (screenId === 'readiness') {
      this.renderReadinessScreen();
    } else if (screenId === 'missions') {
      this.renderMissionsScreen();
    } else if (screenId === 'lessons') {
      this.renderLessonsScreen();
    } else if (screenId === 'quiz') {
      this.renderQuizScreen();
    } else if (screenId === 'simulator' && this.flightSim) {
      this.flightSim.onWindowResize();
    }

    this.syncActiveRenderLoops(screenId);
  }

  bindEvents() {
    document.querySelectorAll('.nav-tab-btn').forEach(btn => {
      if (!btn.dataset.screen) return;
      btn.addEventListener('click', () => {
        if (btn.dataset.screen === 'lessons' && this.activeUnitId === 'unit6') {
          this.activeUnitId = 'unit1';
        }
        this.showScreen(btn.dataset.screen);
      });
    });

    const logo = document.getElementById('btn-logo-home');
    if (logo) logo.addEventListener('click', () => this.showScreen('home'));

    const btnTopHome = document.getElementById('btn-top-home');
    if (btnTopHome) btnTopHome.addEventListener('click', () => this.showScreen('home'));
    const btnTopAsm = document.getElementById('btn-top-assembly');
    if (btnTopAsm) btnTopAsm.addEventListener('click', () => this.showScreen('assembly'));
    document.getElementById('btn-top-freeflight').addEventListener('click', () => this.launchFreeFlightSafe());
    document.getElementById('btn-top-reset').addEventListener('click', () => this.resetToEmpty());

    document.getElementById('btn-home-assembly').addEventListener('click', () => this.showScreen('assembly'));
    document.getElementById('btn-home-freeflight').addEventListener('click', () => this.launchFreeFlightSafe());
    document.getElementById('btn-home-missions').addEventListener('click', () => this.showScreen('missions'));
    document.getElementById('btn-home-lessons').addEventListener('click', () => this.showScreen('lessons'));
    const btnHomeLaw = document.getElementById('btn-home-thai-law');
    if (btnHomeLaw) {
      btnHomeLaw.addEventListener('click', () => {
        this.activeUnitId = 'unit6';
        this.showScreen('lessons');
      });
    }
    const btnNavLaw = document.getElementById('btn-nav-thai-law');
    if (btnNavLaw) {
      btnNavLaw.addEventListener('click', () => {
        this.activeUnitId = 'unit6';
        this.showScreen('lessons');
      });
    }

    document.getElementById('btn-view-iso').addEventListener('click', () => this.setWorkbenchView('iso'));
    document.getElementById('btn-view-top').addEventListener('click', () => this.setWorkbenchView('top'));
    document.getElementById('btn-view-rear').addEventListener('click', () => this.setWorkbenchView('rear'));
    document.getElementById('btn-view-exploded').addEventListener('click', () => this.toggleExplodedView());

    const btnWorkbenchSpin = document.getElementById('btn-workbench-spin');
    if (btnWorkbenchSpin) {
      btnWorkbenchSpin.addEventListener('click', () => {
        this.workbenchSpinTest = !this.workbenchSpinTest;
        btnWorkbenchSpin.classList.toggle('active', this.workbenchSpinTest);
        btnWorkbenchSpin.innerText = this.workbenchSpinTest ? '⏹ หยุดเทสต์' : '🌀 เทสต์มอเตอร์';
        this.playInstallChime(false, this.workbenchSpinTest ? 740 : 420);
        this.showStageToast(this.workbenchSpinTest ? '🌀 กำลังทดสอบเร่งรอบมอเตอร์และใบพัดบนโต๊ะประกอบ' : '⏹ ลดรอบมอเตอร์เพื่อชมรายละเอียดใบพัด 3 มิติ');
      });
    }

    const btnCallouts = document.getElementById('btn-toggle-callouts');
    if (btnCallouts) {
      btnCallouts.addEventListener('click', () => {
        this.showCallouts = !this.showCallouts;
        btnCallouts.classList.toggle('active', this.showCallouts);
        btnCallouts.innerText = this.showCallouts ? '🏷 ซ่อนป้ายทั้งหมด' : '🏷 แสดงป้ายทั้งหมด';
        this.update3DCallouts();
      });
    }

    // 3D Workbench Interactive Raycaster Hover Highlight + Click-to-Edit
    const wbCanvas = document.getElementById('workbench-canvas');
    if (wbCanvas) {
      let downX = 0;
      let downY = 0;
      wbCanvas.addEventListener('pointerdown', (e) => {
        downX = e.clientX;
        downY = e.clientY;
      });
      wbCanvas.addEventListener('pointermove', (e) => {
        if (this.currentScreen !== 'assembly' || !this.workbenchCamera || !this.workbenchDroneMesh) return;
        const rect = wbCanvas.getBoundingClientRect();
        this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        this.raycaster.setFromCamera(this.pointer, this.workbenchCamera);

        const hits = this.raycaster.intersectObjects(this.workbenchDroneMesh.children, true);
        let hitPartKey = null;
        for (let i = 0; i < hits.length; i++) {
          const pk = hits[i].object && hits[i].object.userData && hits[i].object.userData.partKey;
          if (pk) {
            hitPartKey = pk;
            break;
          }
        }

        if (hitPartKey) {
          this.setWorkbenchHoverPart(hitPartKey, e.clientX, e.clientY);
          wbCanvas.style.cursor = 'pointer';
        } else {
          this.clearWorkbenchHoverPart();
          wbCanvas.style.cursor = 'grab';
        }
      });
      wbCanvas.addEventListener('pointerleave', () => {
        this.clearWorkbenchHoverPart();
        wbCanvas.style.cursor = 'default';
      });
      wbCanvas.addEventListener('pointerup', (e) => {
        const dist = Math.hypot(e.clientX - downX, e.clientY - downY);
        if (dist < 6 && this.hoveredPartKey && this.currentScreen === 'assembly') {
          this.openPartSelector(this.hoveredPartKey);
        }
      });
    }

    document.querySelectorAll('.btn-preset-chip').forEach(btn => {
      btn.addEventListener('click', () => {
        const presetKey = btn.dataset.preset;
        const preset = DRONE_PRESETS[presetKey];
        if (!preset) return;
        this.assembly = { ...preset.parts };
        this.updateAssemblyUI();
        if (this.workbenchDroneBuilder) {
          ['battery', 'motor', 'propeller', 'camera', 'payload', 'safety'].forEach(k => {
            this.workbenchDroneBuilder.triggerInstallAnimation(k);
          });
        }
        this.playInstallChime(true);
        this.showStageToast(`โหลดชุดตัวอย่าง: ${preset.name}`);
      });
    });

    document.getElementById('btn-next-step-float').addEventListener('click', () => {
      const next = this.getNextUninstalledPart();
      if (next) {
        this.openPartSelector(next.id);
      } else {
        const stats = calculateDroneStats(this.assembly);
        this.showScreen(stats.isCompatible ? 'missions' : 'readiness');
      }
    });

    // Flight simulator HUD buttons
    const btnCam = document.getElementById('btn-toggle-camera');
    if (btnCam) {
      btnCam.addEventListener('click', () => {
        if (this.flightSim) {
          const isFpv = this.flightSim.toggleCamera();
          btnCam.innerText = isFpv ? '📷 FPV' : '📷 มุมมองที่ 3';
          btnCam.classList.toggle('primary', isFpv);
          const reticle = document.getElementById('fpv-crosshair-hud');
          if (reticle) reticle.classList.toggle('hidden', !isFpv);
        }
      });
    }

    const btnSound = document.getElementById('btn-toggle-sound');
    if (btnSound) {
      btnSound.addEventListener('click', () => {
        if (this.flightSim) {
          const isMuted = this.flightSim.toggleMute();
          btnSound.innerText = isMuted ? '🔇' : '🔊';
        }
      });
    }

    document.getElementById('btn-hud-land').addEventListener('click', () => {
      if (this.flightSim) this.flightSim.finishFlight();
    });
    const btnHudStartStop = document.getElementById('btn-hud-start-stop');
    if (btnHudStartStop) {
      btnHudStartStop.addEventListener('click', () => {
        if (this.flightSim) this.flightSim.armDrone();
      });
    }
    const btnHudAutoTakeoff = document.getElementById('btn-hud-auto-takeoff');
    if (btnHudAutoTakeoff) {
      btnHudAutoTakeoff.addEventListener('click', () => {
        if (!this.flightSim) return;
        if (this.flightSim.isArmed && !this.flightSim.isLanded) {
          this.flightSim.triggerEmergencyKill(true);
        } else {
          this.flightSim.startAutoTakeoff();
        }
      });
    }
    const btnHudRateMode = document.getElementById('btn-hud-rate-mode');
    if (btnHudRateMode) {
      btnHudRateMode.addEventListener('click', () => {
        if (this.flightSim) {
          this.flightSim.cycleControlRateMode();
        }
      });
    }

    const btnHudGraphics = document.getElementById('btn-hud-graphics');
    if (btnHudGraphics) {
      btnHudGraphics.addEventListener('click', () => {
        if (this.flightSim && typeof this.flightSim.cycleGraphicsQuality === 'function') {
          this.flightSim.cycleGraphicsQuality();
        }
      });
    }

    document.getElementById('btn-hud-home').addEventListener('click', () => this.showScreen('home'));
    document.getElementById('btn-arm-toggle').addEventListener('click', () => {
      if (this.flightSim) {
        const armed = this.flightSim.armDrone();
        document.getElementById('btn-arm-toggle').classList.toggle('armed', armed);
      }
    });
    document.getElementById('btn-flight-reset').addEventListener('click', () => {
      if (this.flightSim) this.flightSim.resetDronePosition();
    });

    // Debrief Modal
    document.getElementById('btn-debrief-retry').addEventListener('click', () => {
      document.getElementById('modal-debrief').classList.remove('active');
      this.launchFlight(this.selectedMission);
    });
    document.getElementById('btn-debrief-missions').addEventListener('click', () => {
      document.getElementById('modal-debrief').classList.remove('active');
      this.showScreen('missions');
    });
    document.getElementById('btn-debrief-edit').addEventListener('click', () => {
      document.getElementById('modal-debrief').classList.remove('active');
      this.showScreen('assembly');
    });

    // Part Picker Modal (restores actual installed assembly if closed without confirming!)
    document.getElementById('btn-close-part-modal').addEventListener('click', () => {
      document.getElementById('modal-part-selector').classList.remove('active');
      if (this.workbenchDroneBuilder && this.workbenchScene) {
        this.workbenchDroneMesh = this.workbenchDroneBuilder.buildDrone(this.assembly, THREE, this.paintConfig);
      }
    });
    document.getElementById('btn-confirm-install-part').addEventListener('click', () => {
      this.confirmInstallPendingPart();
    });

    // Learning Module -> Quiz shortcut & Quiz mode switcher
    const btnGoQuiz = document.getElementById('btn-lessons-go-quiz');
    if (btnGoQuiz) btnGoQuiz.addEventListener('click', () => this.showScreen('quiz'));

    const btnQuizPre = document.getElementById('btn-quiz-pre');
    const btnQuizPost = document.getElementById('btn-quiz-post');
    const btnQuizComp = document.getElementById('btn-quiz-compare');
    const btnQuizEval = document.getElementById('btn-quiz-eval');
    if (btnQuizPre) {
      btnQuizPre.addEventListener('click', () => {
        this.quizMode = 'pre';
        this.quizAnswers = {};
        this.quizSubmitted = false;
        this.renderQuizScreen();
      });
    }
    if (btnQuizPost) {
      btnQuizPost.addEventListener('click', () => {
        this.quizMode = 'post';
        this.quizAnswers = {};
        this.quizSubmitted = false;
        this.renderQuizScreen();
      });
    }
    if (btnQuizComp) {
      btnQuizComp.addEventListener('click', () => {
        this.quizMode = 'compare';
        this.renderQuizScreen();
      });
    }
    if (btnQuizEval) {
      btnQuizEval.addEventListener('click', () => {
        this.quizMode = 'eval';
        this.renderQuizScreen();
      });
    }

    // Home Profile Modal & Sound Toggle & Live Gamepad Detector
    const btnHomeProfile = document.getElementById('btn-home-profile');
    const modalProfile = document.getElementById('modal-home-profile');
    const closeProfile = () => modalProfile && modalProfile.classList.remove('active');
    if (btnHomeProfile && modalProfile) {
      btnHomeProfile.addEventListener('click', () => {
        const nameEl = document.getElementById('profile-input-name');
        const idEl = document.getElementById('profile-input-id');
        if (nameEl) nameEl.value = this.studentProfile.name || '';
        if (idEl) idEl.value = this.studentProfile.studentId || '';
        modalProfile.classList.add('active');
      });
    }
    const btnCloseProf = document.getElementById('btn-close-profile-modal');
    const btnCancelProf = document.getElementById('btn-cancel-profile');
    const btnSaveProf = document.getElementById('btn-save-profile');
    if (btnCloseProf) btnCloseProf.addEventListener('click', closeProfile);
    if (btnCancelProf) btnCancelProf.addEventListener('click', closeProfile);
    if (btnSaveProf) {
      btnSaveProf.addEventListener('click', () => {
        const nameVal = (document.getElementById('profile-input-name')?.value || '').trim();
        const idVal = (document.getElementById('profile-input-id')?.value || '').trim();
        this.studentProfile = { name: nameVal, studentId: idVal };
        this.saveStudentProfile();
        this.evalForm.name = nameVal;
        this.evalForm.studentId = idVal;
        this.updateHomeDashboard();
        closeProfile();
      });
    }

    const btnHomeSound = document.getElementById('btn-home-sound');
    if (btnHomeSound) {
      btnHomeSound.addEventListener('click', () => {
        if (this.flightSim) {
          const isMuted = this.flightSim.toggleMute();
          btnHomeSound.innerText = isMuted ? '🔇' : '🔊';
          const simSound = document.getElementById('btn-toggle-sound');
          if (simSound) simSound.innerText = isMuted ? '🔇' : '🔊';
        }
      });
    }

    const refreshGamepadUI = () => {
      const badge = document.getElementById('home-gamepad-status');
      if (!badge || !navigator.getGamepads) return;
      const pads = navigator.getGamepads();
      let connectedPad = null;
      for (let i = 0; i < pads.length; i++) {
        if (pads[i] && pads[i].connected) {
          connectedPad = pads[i];
          break;
        }
      }
      if (connectedPad) {
        const shortName = connectedPad.id.split('(')[0].trim().slice(0, 22);
        badge.innerText = `🎮 เชื่อมต่อจอยแล้ว: ${shortName}`;
        badge.classList.add('connected');
      } else {
        badge.innerText = '🎮 ยังไม่ได้เสียบจอย (รองรับ Mode 2)';
        badge.classList.remove('connected');
      }
    };
    window.addEventListener('gamepadconnected', refreshGamepadUI);
    window.addEventListener('gamepaddisconnected', refreshGamepadUI);
    refreshGamepadUI();
  }

  launchFreeFlightSafe() {
    const stats = calculateDroneStats(this.assembly);
    if (!stats.isComplete) {
      // Auto-load recommended preset so user can immediately try Free Flight
      this.assembly = { ...DRONE_PRESETS.default_aerial.parts };
      this.updateAssemblyUI();
    }
    this.launchFlight(MISSIONS[0]);
  }

  initWorkbench3D() {
    const container = document.getElementById('workbench-canvas');
    if (!container) return;

    this.workbenchScene = new THREE.Scene();
    this.workbenchScene.background = new THREE.Color(0x07080b);

    const width = container.clientWidth || 720;
    const height = container.clientHeight || 600;

    this.workbenchCamera = new THREE.PerspectiveCamera(42, width / height, 0.1, 100);
    this.workbenchCamera.position.set(2.3, 1.7, 2.7);

    this.workbenchRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.workbenchRenderer.setSize(width, height);
    this.workbenchRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.35));
    this.workbenchRenderer.shadowMap.enabled = true;
    container.appendChild(this.workbenchRenderer.domElement);

    // Subtle dark floor grid
    const gridHelper = new THREE.GridHelper(6, 24, 0x222630, 0x111318);
    gridHelper.position.y = -0.18;
    this.workbenchScene.add(gridHelper);

    // Concentric Orange & White Circular Pad (sized comfortably for all 250–800mm zero-clipping frames!)
    const outerOrangeRing = new THREE.Mesh(
      new THREE.RingGeometry(1.88, 1.92, 72),
      new THREE.MeshBasicMaterial({ color: 0xff6a00, side: THREE.DoubleSide })
    );
    outerOrangeRing.rotation.x = -Math.PI / 2;
    outerOrangeRing.position.y = -0.17;
    this.workbenchScene.add(outerOrangeRing);

    const innerGrayRing = new THREE.Mesh(
      new THREE.RingGeometry(1.52, 1.55, 72),
      new THREE.MeshBasicMaterial({ color: 0x3b3f4a, side: THREE.DoubleSide })
    );
    innerGrayRing.rotation.x = -Math.PI / 2;
    innerGrayRing.position.y = -0.17;
    this.workbenchScene.add(innerGrayRing);

    // 3-Point Studio Lighting (Key + Cool Rim + Warm Orange Fill so dark carbon pops crisply)
    this.workbenchScene.add(new THREE.HemisphereLight(0xeef6ff, 0x1e222a, 1.1));
    const dirLight = new THREE.DirectionalLight(0xffffff, 1.65);
    dirLight.position.set(4.5, 7.5, 5.5);
    dirLight.castShadow = true;
    this.workbenchScene.add(dirLight);

    const fillLight = new THREE.DirectionalLight(0x93c5fd, 0.85);
    fillLight.position.set(-5, 3.5, -4);
    this.workbenchScene.add(fillLight);

    const orangeRim = new THREE.PointLight(0xff6a00, 1.4, 9);
    orangeRim.position.set(2.8, 1.6, -2.8);
    this.workbenchScene.add(orangeRim);

    if (window.THREE && THREE.OrbitControls) {
      this.workbenchControls = new THREE.OrbitControls(this.workbenchCamera, this.workbenchRenderer.domElement);
      this.workbenchControls.enableDamping = true;
      this.workbenchControls.dampingFactor = 0.07;
      this.workbenchControls.minDistance = 1.6;
      this.workbenchControls.maxDistance = 6.5;
      this.workbenchControls.maxPolarAngle = Math.PI / 2.05;
      this.workbenchControls.target.set(0, 0.08, 0);
    }

    this.workbenchDroneBuilder = new DroneModelBuilder();
    this.workbenchDroneMesh = this.workbenchDroneBuilder.buildDrone(this.assembly, THREE, this.paintConfig);
    this.workbenchScene.add(this.workbenchDroneMesh);

    this._animateWorkbench = () => {
      if (this.currentScreen !== 'assembly') {
        this.workbenchRafId = null;
        return;
      }
      this.workbenchRafId = requestAnimationFrame(this._animateWorkbench);
      if (this.workbenchControls) this.workbenchControls.update();
      if (this.workbenchDroneBuilder) {
        const targetRpm = this.workbenchSpinTest ? 0.90 : 0.045;
        this.workbenchSpinRpm += (targetRpm - this.workbenchSpinRpm) * 0.08;
        const r = this.workbenchSpinRpm;
        this.workbenchDroneBuilder.update(0.016, true, 16, [r, r, r, r]);
      }
      this.workbenchRenderer.render(this.workbenchScene, this.workbenchCamera);
      this.update3DCallouts();
    };

    window.addEventListener('resize', () => {
      if (this.currentScreen === 'assembly') this.refreshWorkbench();
    });
  }

  update3DCallouts() {
    const layer = document.getElementById('workbench-callouts');
    const container = document.getElementById('workbench-canvas');
    if (!layer || !container || !this.workbenchCamera) return;

    const w = container.clientWidth;
    const h = container.clientHeight;
    if (!w || !h) return;

    const explodeBoost = this.isExploded ? 1.55 : 1.0;

    this.calloutDefinitions.forEach(def => {
      let el = layer.querySelector(`[data-callout="${def.key}"]`);
      const hasMesh = Boolean(this.workbenchDroneBuilder && this.workbenchDroneBuilder.parts && this.workbenchDroneBuilder.parts[def.key]);
      const isInstalled = Boolean(this.assembly[def.key]) && hasMesh;
      const isSidebarTarget = this.sidebarHoveredPartKey === def.key;
      if (!isInstalled || (!this.showCallouts && !isSidebarTarget)) {
        if (el) el.style.display = 'none';
        return;
      }
      if (!el) {
        el = document.createElement('div');
        el.className = 'drone-callout-pill';
        el.dataset.callout = def.key;
        el.textContent = def.label;
        layer.appendChild(el);
      }
      el.style.display = 'inline-flex';
      const v = def.pos.clone().multiplyScalar(explodeBoost);
      v.project(this.workbenchCamera);
      const sx = Math.max(82, Math.min(w - 82, (v.x * 0.5 + 0.5) * w));
      const sy = Math.max(26, Math.min(h - 72, (-v.y * 0.5 + 0.5) * h));
      el.style.left = `${sx}px`;
      el.style.top = `${sy}px`;
    });
  }

  setWorkbenchHoverPart(partKey, clientX, clientY) {
    this.hoveredPartKey = partKey;
    if (this.workbenchDroneBuilder) {
      this.workbenchDroneBuilder.setHighlightedPart(partKey);
    }

    // Sync highlight with right sidebar card
    document.querySelectorAll('.part-item-card').forEach(c => {
      c.classList.toggle('hover-sync', c.dataset.partkey === partKey);
    });

    // Position and populate floating hover tooltip
    const tooltip = document.getElementById('workbench-hover-tooltip');
    const partDef = DRONE_PARTS[partKey];
    if (tooltip && partDef) {
      const selectedOptId = this.assembly[partKey];
      const selectedOpt = selectedOptId ? partDef.options.find(o => o.id === selectedOptId) : null;
      tooltip.innerHTML = `
        <span class="dht-code">${partDef.calloutCode || `${partDef.num} · ${partDef.enName}`}</span>
        <span class="dht-name">${partDef.name}</span>
        <span class="dht-opt">${selectedOpt ? `${selectedOpt.name} (${selectedOpt.weight} ก.)` : 'ยังไม่ได้ติดตั้ง'}</span>
        <span class="dht-hint">คลิกเพื่อเปลี่ยนชิ้นส่วน →</span>
      `;
      tooltip.style.left = `${Math.min(window.innerWidth - 270, clientX + 18)}px`;
      tooltip.style.top = `${Math.max(70, clientY - 18)}px`;
      tooltip.classList.add('visible');
    }
  }

  clearWorkbenchHoverPart() {
    if (!this.hoveredPartKey) return;
    this.hoveredPartKey = null;
    if (this.workbenchDroneBuilder) {
      this.workbenchDroneBuilder.setHighlightedPart(null);
    }
    document.querySelectorAll('.part-item-card.hover-sync').forEach(c => {
      c.classList.remove('hover-sync');
    });
    const tooltip = document.getElementById('workbench-hover-tooltip');
    if (tooltip) tooltip.classList.remove('visible');
  }

  showStageToast(text) {
    const pill = document.getElementById('stage-toast-pill');
    if (!pill) return;
    pill.textContent = text;
    pill.style.display = 'block';
    clearTimeout(this._toastTimer);
    this._toastTimer = setTimeout(() => {
      if (!this.isExploded) pill.style.display = 'none';
    }, 2800);
  }

  initFlightSim() {
    const container = document.getElementById('flight-sim-canvas');
    if (!container) return;

    this.flightSim = new DroneFlightSimulator(container);
    this.flightSim.isFPV = false; // Default 3rd person chase view like step_26_shot_1
    this.flightSim.onTelemetryUpdate = (data) => this.updateFlightHUD(data);
    this.flightSim.onFlightFinished = (result) => this.showDebriefModal(result);
    this.flightSim.onCheckpointPassed = (passed, total) => {
      document.getElementById('hud-checkpoint-val').innerText = `${passed} / ${total}`;
    };
  }

  setWorkbenchView(angle) {
    this.currentViewAngle = angle;
    document.querySelectorAll('.btn-view').forEach(b => b.classList.remove('active'));
    this.isExploded = false;
    if (this.workbenchDroneBuilder) this.workbenchDroneBuilder.setExploded(false);
    const toast = document.getElementById('stage-toast-pill');
    if (toast) toast.style.display = 'none';

    const frameOpt = this.assembly && this.assembly.frame
      ? DRONE_PARTS.frame.options.find(o => o.id === this.assembly.frame)
      : null;
    const wb = (frameOpt && frameOpt.wheelbase) || 450;
    const camScale = wb >= 750 ? 1.32 : wb >= 600 ? 1.15 : wb <= 260 ? 0.85 : 1.0;

    if (angle === 'iso') {
      document.getElementById('btn-view-iso').classList.add('active');
      this.workbenchCamera.position.set(2.35 * camScale, 1.55 * camScale, 2.70 * camScale);
      if (this.workbenchControls) this.workbenchControls.target.set(0, 0.08, 0);
      this.workbenchCamera.lookAt(0, 0.08, 0);
    } else if (angle === 'top') {
      document.getElementById('btn-view-top').classList.add('active');
      this.workbenchCamera.position.set(0, 3.65 * camScale, 0.01);
      if (this.workbenchControls) this.workbenchControls.target.set(0, 0, 0);
      this.workbenchCamera.lookAt(0, 0, 0);
    } else if (angle === 'rear') {
      document.getElementById('btn-view-rear').classList.add('active');
      this.workbenchCamera.position.set(0, 0.92 * camScale, -2.95 * camScale);
      if (this.workbenchControls) this.workbenchControls.target.set(0, 0.1, 0);
      this.workbenchCamera.lookAt(0, 0.1, 0);
    }
  }

  toggleExplodedView() {
    this.isExploded = !this.isExploded;
    document.getElementById('btn-view-exploded').classList.toggle('active', this.isExploded);
    if (this.workbenchDroneBuilder) {
      this.workbenchDroneBuilder.setExploded(this.isExploded);
    }
    const toast = document.getElementById('stage-toast-pill');
    if (toast) {
      if (this.isExploded) {
        toast.textContent = 'แยกชิ้นส่วน — เห็นตำแหน่งจริงของทุกชิ้น';
        toast.style.display = 'block';
      } else {
        toast.style.display = 'none';
      }
    }
  }

  updateAssemblyUI() {
    const stats = calculateDroneStats(this.assembly);

    // 1. Left HUD Performance Specs
    document.getElementById('stat-weight').innerText = `${stats.totalWeight.toLocaleString()} ก.`;
    document.getElementById('stat-thrust').innerText = stats.isComplete ? `${stats.totalThrust.toLocaleString()} ก.` : '—';
    document.getElementById('stat-tw').innerText = stats.isComplete ? `${stats.twRatio}` : '—';
    document.getElementById('stat-flight-time').innerText = stats.isComplete ? stats.estHoverSecStr : '—';
    document.getElementById('stat-current').innerText = stats.isComplete ? `${stats.maxCurrentAmps} A` : '—';
    document.getElementById('stat-price').innerText = `฿${stats.totalPrice.toLocaleString()}`;

    const twFill = document.getElementById('tw-progress-fill');
    if (twFill) {
      twFill.style.width = stats.isComplete ? `${Math.min(100, (stats.twRatio / 5.0) * 100)}%` : '0%';
    }

    // Lock / unlock nav tabs 2 & 3 when incomplete
    document.querySelectorAll('.nav-tab-btn').forEach(btn => {
      if (btn.dataset.screen === 'readiness' || btn.dataset.screen === 'missions') {
        btn.classList.toggle('locked', !stats.isComplete);
      }
    });

    // 2. Compatibility Card
    const compBadge = document.getElementById('badge-compatibility');
    const verifyBox = document.getElementById('text-verify-result');
    const readyBox = document.getElementById('text-flight-ready');

    if (stats.isComplete && stats.isCompatible) {
      if (stats.twRatio <= 1.02) {
        compBadge.className = 'badge-tag warning';
        compBadge.innerText = 'น้ำหนักเกินแรงยก';
        verifyBox.className = 'compat-box has-issue';
        verifyBox.innerText = `⚠ น้ำหนักรวม ${stats.totalWeight.toLocaleString()} ก. เกินแรงยกสูงสุด ${stats.totalThrust.toLocaleString()} ก. (T/W ${stats.twRatio} ≤ 1.0) — ลองบินอิสระได้แต่เครื่องจะยกตัวไม่ขึ้น!`;
      } else if (stats.twRatio < 1.6) {
        compBadge.className = 'badge-tag warning';
        compBadge.innerText = 'หนัก · บินช้า';
        verifyBox.className = 'compat-box';
        verifyBox.innerText = `⚠ โดรนค่อนข้างหนัก (T/W ${stats.twRatio}) — บินได้แต่ไต่ช้า กินไฟสูง และระยะเบรกยาว`;
      } else {
        compBadge.className = 'badge-tag success';
        compBadge.innerText = 'บินได้';
        verifyBox.className = 'compat-box';
        verifyBox.innerText = '✓ ชิ้นส่วนเข้ากันได้ทั้งหมด — โครงรับมอเตอร์/ใบพัดได้พอดี';
      }

      const basicCheck = checkMissionReadiness(stats, MISSIONS[1]);
      const law = typeof evaluateThailandDroneLaw === 'function' ? evaluateThailandDroneLaw(stats) : null;
      const lawLine = law
        ? `<div class="law-mini-pill" id="btn-workbench-law-link" title="คลิกเพื่ออ่านกฎหมายโดรนไทยฉบับเต็ม">⚖️ ${law.needCAAT ? 'ต้องขึ้นทะเบียน กสทช.+กพท.' : 'ขึ้นทะเบียน กสทช.'} · ห่างคน ≥ ${law.minHorizDistanceM} ม. · ≤ 90 ม.</div>`
        : '';
      readyBox.innerHTML = basicCheck.checks
        .map(c => `<div class="${c.pass ? '' : 'fail-line'}">${c.pass ? '✓' : '✕'} ${c.label}</div>`)
        .join('') + lawLine;
      const lawLink = document.getElementById('btn-workbench-law-link');
      if (lawLink) {
        lawLink.addEventListener('click', () => {
          this.activeUnitId = 'unit6';
          this.showScreen('lessons');
        });
      }
    } else if (!stats.isCompatible) {
      compBadge.className = 'badge-tag warning';
      compBadge.innerText = 'ไม่เข้ากัน';
      verifyBox.className = 'compat-box has-issue';
      verifyBox.innerText = stats.compatibilityIssues.join(' • ');
      readyBox.innerHTML = `<div class="fail-line">✕ แก้ไขชิ้นส่วนที่ไม่เข้ากันก่อนออกบิน</div>`;
    } else {
      const totalSlots = stats.totalSlots || Object.keys(DRONE_PARTS).length;
      compBadge.className = 'badge-tag warning';
      compBadge.innerText = 'ยังประกอบไม่ครบ';
      verifyBox.className = 'compat-box';
      verifyBox.innerText = `ยังตรวจไม่ครบ — ติดตั้งให้ครบ ${totalSlots} ชิ้นก่อน`;
      readyBox.innerHTML = `<div class="fail-line">✕ ต้องประกอบให้ครบ ${totalSlots} ชิ้นก่อน (ขาด ${totalSlots - stats.installedCount} ชิ้น)</div>`;
    }

    // 3. Right Sidebar Progress & 12 Slot Cards
    const totalSlots = stats.totalSlots || Object.keys(DRONE_PARTS).length;
    document.getElementById('val-installed-count').innerText = `ติดตั้งแล้ว ${stats.installedCount} / ${totalSlots}`;
    const nextPart = this.getNextUninstalledPart();
    document.getElementById('val-next-part').innerText = nextPart ? `ขั้นถัดไป: ${nextPart.name}` : 'ครบทุกชิ้น';
    const progFill = document.getElementById('parts-progress-fill');
    if (progFill) progFill.style.width = `${(stats.installedCount / totalSlots) * 100}%`;

    const floatBtn = document.getElementById('btn-next-step-float');
    if (nextPart) {
      floatBtn.innerText = `${nextPart.num} ติดตั้ง ${nextPart.name}`;
    } else if (stats.isCompatible) {
      floatBtn.innerText = '🚀 พร้อมบินแล้ว! ไปหน้าออกบิน →';
    } else {
      floatBtn.innerText = '⚠ ตรวจความเข้ากันได้ →';
    }

    const partsListEl = document.getElementById('parts-list-container');
    partsListEl.innerHTML = '';

    for (const partKey in DRONE_PARTS) {
      const partDef = DRONE_PARTS[partKey];
      const selectedOptId = this.assembly[partKey];
      const selectedOpt = selectedOptId ? partDef.options.find(o => o.id === selectedOptId) : null;
      const isCurrent = Boolean(nextPart && partKey === nextPart.id);

      const card = document.createElement('div');
      card.className = `part-item-card ${selectedOpt ? 'installed' : 'uninstalled'} ${isCurrent ? 'active-slot' : ''}`;
      card.dataset.partkey = partKey;
      card.innerHTML = `
        <div class="part-number-badge">${selectedOpt ? '✓' : partDef.num}</div>
        <div class="part-item-info">
          <div class="part-item-title">${partDef.name}</div>
          <div class="part-item-selected">${selectedOpt ? selectedOpt.name : 'ยังไม่ได้ติดตั้ง'}</div>
        </div>
      `;
      card.addEventListener('click', () => this.openPartSelector(partKey));
      card.addEventListener('mouseenter', () => {
        if (selectedOpt) {
          this.sidebarHoveredPartKey = partKey;
          if (this.workbenchDroneBuilder) {
            this.workbenchDroneBuilder.setHighlightedPart(partKey);
          }
          this.update3DCallouts();
        }
      });
      card.addEventListener('mouseleave', () => {
        this.sidebarHoveredPartKey = null;
        if (this.workbenchDroneBuilder) {
          this.workbenchDroneBuilder.setHighlightedPart(this.hoveredPartKey);
        }
        this.update3DCallouts();
      });
      partsListEl.appendChild(card);
    }

    // Rebuild 3D Meshes (preserving active custom paint livery!)
    if (this.workbenchDroneBuilder && this.workbenchScene) {
      this.workbenchDroneMesh = this.workbenchDroneBuilder.buildDrone(this.assembly, THREE, this.paintConfig);
      if (this.workbenchCamera && this._lastWorkbenchFrameId !== this.assembly.frame) {
        this._lastWorkbenchFrameId = this.assembly.frame;
        this.setWorkbenchView(this.currentViewAngle || 'iso');
      }
    }
    if (this.homeDroneBuilder && this.homeScene) {
      this.homeDroneMesh = this.homeDroneBuilder.buildDrone(this.assembly, THREE, this.paintConfig);
      if (typeof this.homeDroneBuilder.bakeForFlight === 'function') this.homeDroneBuilder.bakeForFlight(THREE);
      this.homeDroneMesh.position.set(0, 1.5, 0);
    }

    this.updateHomeDashboard();
  }

  getNextUninstalledPart() {
    for (const partKey in DRONE_PARTS) {
      if (!this.assembly[partKey]) return DRONE_PARTS[partKey];
    }
    return null;
  }

  openPartSelector(partKey) {
    this.activeSelectingPartKey = partKey;
    const partDef = DRONE_PARTS[partKey];
    const currentVal = this.assembly[partKey];

    // Pre-select current or recommended option
    const recOpt = partDef.options.find(o => o.recommended) || partDef.options[0];
    this.pendingOptionId = currentVal || recOpt.id;

    const isNext = this.getNextUninstalledPart()?.id === partKey;
    document.getElementById('part-modal-kicker').innerText = `${partDef.num} · ${partDef.enName}${isNext ? ' · ขั้นถัดไป' : ''}`;
    document.getElementById('part-modal-title').innerText = partDef.name;
    document.getElementById('part-modal-lesson').innerText = partDef.lesson;

    this.renderPartModalOptions();
    document.getElementById('modal-part-selector').classList.add('active');
  }

  renderPartModalOptions() {
    const partKey = this.activeSelectingPartKey;
    const partDef = DRONE_PARTS[partKey];
    const listEl = document.getElementById('part-modal-options-list');
    const confirmBtn = document.getElementById('btn-confirm-install-part');
    listEl.innerHTML = '';

    let pendingCompatible = true;

    partDef.options.forEach(opt => {
      const compat = checkOptionCompatibility(partKey, opt, this.assembly);
      const isSelected = opt.id === this.pendingOptionId;
      if (isSelected && !compat.compatible) pendingCompatible = false;

      const optCard = document.createElement('div');
      optCard.className = `part-opt-card ${isSelected ? 'selected' : ''} ${!compat.compatible ? 'incompatible' : ''}`;

      const chipsHtml = (opt.chips || [`${opt.weight} ก.`, `฿${opt.price}`])
        .map(c => `<span class="spec-chip">${c}</span>`)
        .join('');

      let badgeHtml = '';
      if (!compat.compatible) {
        badgeHtml = `<span class="part-opt-badge bad">ใส่ไม่ได้</span>`;
      } else if (opt.badge) {
        badgeHtml = `<span class="part-opt-badge">${opt.badge}</span>`;
      }

      optCard.innerHTML = `
        <div class="part-opt-header">
          <span class="part-opt-name">${opt.name}</span>
          ${badgeHtml}
        </div>
        <div class="part-opt-specs">${chipsHtml}</div>
        <div class="part-opt-desc">${!compat.compatible ? compat.reason : opt.desc}</div>
      `;

      optCard.addEventListener('click', () => {
        this.pendingOptionId = opt.id;
        this.renderPartModalOptions();
        // Live 3D Ghost-Preview right on the Workbench drone as user compares options!
        if (compat.compatible && this.workbenchDroneBuilder && this.workbenchScene) {
          const previewAsm = { ...this.assembly, [partKey]: opt.id };
          this.workbenchDroneMesh = this.workbenchDroneBuilder.buildDrone(previewAsm, THREE, this.paintConfig);
          this.workbenchDroneBuilder.triggerInstallAnimation(partKey);
          this.playInstallChime(false, 540);
        }
      });

      listEl.appendChild(optCard);
    });

    confirmBtn.disabled = !pendingCompatible;
    if (pendingCompatible && this.pendingOptionId) {
      const previewStats = calculateDroneStats({ ...this.assembly, [partKey]: this.pendingOptionId });
      const curStats = calculateDroneStats(this.assembly);
      const wDiff = previewStats.totalWeight - curStats.totalWeight;
      const wSign = wDiff > 0 ? `+${wDiff}` : `${wDiff}`;
      const deltaNote = previewStats.isComplete
        ? ` (${wDiff !== 0 ? `${wSign}ก. · ` : ''}T/W ${previewStats.twRatio})`
        : (wDiff !== 0 ? ` (${wSign} ก.)` : '');
      confirmBtn.innerText = `ติดตั้งชิ้นนี้${deltaNote} →`;
    } else {
      confirmBtn.innerText = 'ไม่สามารถติดตั้งชิ้นนี้ได้';
    }
  }

  // Crisp tactile mechanical snap + harmonic chime when installing parts in Workbench!
  playInstallChime(isComplete = false, baseFreq = 620) {
    try {
      if (this.flightSim && this.flightSim.isMuted) return;
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      if (!this._uiAudioCtx) this._uiAudioCtx = new AudioCtx();
      if (this._uiAudioCtx.state === 'suspended') this._uiAudioCtx.resume();
      const now = this._uiAudioCtx.currentTime;
      const notes = isComplete
        ? [[523, 0, 0.09], [659, 0.08, 0.09], [784, 0.16, 0.12], [1046, 0.25, 0.20]]
        : [[baseFreq * 0.78, 0, 0.045], [baseFreq * 1.18, 0.04, 0.09]];
      notes.forEach(([freq, delay, dur]) => {
        const osc = this._uiAudioCtx.createOscillator();
        const g = this._uiAudioCtx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(freq, now + delay);
        g.gain.setValueAtTime(0.06, now + delay);
        g.gain.exponentialRampToValueAtTime(0.001, now + delay + dur);
        osc.connect(g);
        g.connect(this._uiAudioCtx.destination);
        osc.start(now + delay);
        osc.stop(now + delay + dur + 0.01);
      });
    } catch (e) {}
  }

  confirmInstallPendingPart() {
    if (!this.activeSelectingPartKey || !this.pendingOptionId) return;
    const partKey = this.activeSelectingPartKey;
    this.assembly[partKey] = this.pendingOptionId;
    this.lastSelectedPartKey = partKey;

    this.updateAssemblyUI();
    const stats = calculateDroneStats(this.assembly);
    const totalSlots = stats.totalSlots || Object.keys(DRONE_PARTS).length;

    // Trigger tactile magnetic spring-snap animation + brief warm CNC highlight + snap chime!
    if (this.workbenchDroneBuilder) {
      this.workbenchDroneBuilder.triggerInstallAnimation(partKey);
      this.workbenchDroneBuilder.setHighlightedPart(partKey);
      clearTimeout(this._installHlTimer);
      this._installHlTimer = setTimeout(() => {
        if (this.workbenchDroneBuilder && this.hoveredPartKey !== partKey) {
          this.workbenchDroneBuilder.setHighlightedPart(this.hoveredPartKey);
        }
      }, 650);
    }
    this.playInstallChime(stats.installedCount === totalSlots);
    this.showStageToast(
      stats.installedCount === totalSlots
        ? `🎉 ประกอบครบ ${totalSlots}/${totalSlots} ชิ้นแล้ว! (T/W ${stats.twRatio} · พร้อมออกบิน)`
        : `✓ ติดตั้ง ${DRONE_PARTS[partKey].name} แล้ว (${stats.installedCount}/${totalSlots})`
    );

    // Auto-advance sequentially to next uninstalled part like surge.sh
    const nextUninstalled = this.getNextUninstalledPart();
    if (nextUninstalled) {
      this.openPartSelector(nextUninstalled.id);
    } else {
      document.getElementById('modal-part-selector').classList.remove('active');
    }
  }

  resetToEmpty() {
    this.assembly = { ...DRONE_PRESETS.empty_build.parts };
    this.isExploded = false;
    if (this.workbenchDroneBuilder) this.workbenchDroneBuilder.setExploded(false);
    this.updateAssemblyUI();
    this.showScreen('assembly');
    this.openPartSelector('frame');
  }

  renderReadinessScreen() {
    const stats = calculateDroneStats(this.assembly);
    const container = document.getElementById('readiness-cards-container');
    container.innerHTML = '';

    // Prepend Thailand Drone Law (NBTC + CAAT) Compliance Card for the user's assembled drone!
    if (typeof evaluateThailandDroneLaw === 'function') {
      const law = evaluateThailandDroneLaw(stats);
      const lawCard = document.createElement('div');
      lawCard.className = 'readiness-card law-readiness-card';
      lawCard.innerHTML = `
        <div class="readiness-card-header">
          <div>
            <div class="readiness-card-title">⚖️ เกณฑ์กฎหมายโดรนประเทศไทยสำหรับชุดที่คุณประกอบ (${law.weightKg} กก.)</div>
            <div class="readiness-card-desc">${law.weightClassLabel} — ${law.summaryShort}</div>
          </div>
          <span class="badge-tag ${law.needCAAT ? 'warning' : 'success'}">${law.needCAAT ? 'ต้องขึ้นทะเบียน 2 หน่วยงาน' : 'ขึ้นทะเบียน กสทช.'}</span>
        </div>
        <div class="readiness-checklist">
          ${law.requirements.map(r => `
            <div class="check-item">
              <span class="check-icon ${r.required ? 'pass' : 'pass'}" style="color:#38bdf8;">⚖</span>
              <span style="color:#e2e8f0;"><b style="color:#fff;">${r.agency}:</b> ${r.detail}</span>
            </div>
          `).join('')}
        </div>
        <div style="margin-top:10px; display:flex; justify-content:flex-end;">
          <button type="button" class="btn-header-action primary" id="btn-readiness-open-law">📖 ศึกษากฎหมายโดรนไทยฉบับเต็ม (หน่วยที่ 6) →</button>
        </div>
      `;
      container.appendChild(lawCard);
      const btnOpenLaw = lawCard.querySelector('#btn-readiness-open-law');
      if (btnOpenLaw) {
        btnOpenLaw.addEventListener('click', () => {
          this.activeUnitId = 'unit6';
          this.showScreen('lessons');
        });
      }
    }

    [MISSIONS[1], MISSIONS[2], MISSIONS[3]].forEach(m => {
      const res = checkMissionReadiness(stats, m);
      const card = document.createElement('div');
      card.className = `readiness-card ${res.ready ? '' : 'not-ready'}`;

      const checksHtml = res.checks.map(c => `
        <div class="check-item">
          <span class="check-icon ${c.pass ? 'pass' : 'fail'}">${c.pass ? '✓' : '✕'}</span>
          <span style="color: ${c.pass ? '#ffffff' : '#ff6b6b'};">${c.label}</span>
        </div>
      `).join('');

      card.innerHTML = `
        <div class="readiness-card-header">
          <div>
            <div class="readiness-card-title">${m.title}</div>
            <div class="readiness-card-desc">${m.desc}</div>
          </div>
          <span class="badge-tag ${res.ready ? 'success' : 'warning'}">${res.ready ? 'พร้อมบิน' : 'ยังไม่พร้อม'}</span>
        </div>
        <div class="readiness-checklist">${checksHtml}</div>
      `;
      container.appendChild(card);
    });
  }

  renderMissionsScreen() {
    const stats = calculateDroneStats(this.assembly);
    document.getElementById('btn-launch-free').onclick = () => this.launchFlight(MISSIONS[0]);

    const subContainer = document.getElementById('mission-subcards-container');
    subContainer.innerHTML = '';

    for (let i = 1; i < MISSIONS.length; i++) {
      const m = MISSIONS[i];
      const res = checkMissionReadiness(stats, m);
      const card = document.createElement('div');
      card.className = 'mission-sub-card';
      card.innerHTML = `
        <span class="mission-cat">${m.category}</span>
        <h3>${m.title}</h3>
        <p>${m.statsPill}</p>
        <span class="btn-launch-orange">${res.ready ? '▶ พร้อมออกบิน' : '✕ ชุดยังไม่ผ่านเกณฑ์'}</span>
      `;
      card.addEventListener('click', () => {
        if (res.ready) {
          this.launchFlight(m);
        } else {
          this.showScreen('readiness');
        }
      });
      subContainer.appendChild(card);
    }
  }

  loadSavedScores() {
    try {
      const raw = localStorage.getItem('drone_builder_quiz_scores');
      return raw ? JSON.parse(raw) : { pre: null, post: null };
    } catch {
      return { pre: null, post: null };
    }
  }

  saveScores() {
    try {
      localStorage.setItem('drone_builder_quiz_scores', JSON.stringify(this.savedScores));
    } catch {
      // ignore storage quota errors
    }
  }

  loadEvalRecords() {
    try {
      const raw = localStorage.getItem('drone_builder_eval_records');
      return raw ? JSON.parse(raw) : [];
    } catch {
      return [];
    }
  }

  saveEvalRecords() {
    try {
      localStorage.setItem('drone_builder_eval_records', JSON.stringify(this.evalRecords));
    } catch {
      // ignore storage quota errors
    }
  }

  loadStudentProfile() {
    try {
      const raw = localStorage.getItem('drone_builder_student_profile');
      return raw ? JSON.parse(raw) : { name: '', studentId: '' };
    } catch {
      return { name: '', studentId: '' };
    }
  }

  saveStudentProfile() {
    try {
      localStorage.setItem('drone_builder_student_profile', JSON.stringify(this.studentProfile));
    } catch {
      // ignore storage quota errors
    }
  }

  loadCompletedMissions() {
    try {
      const raw = localStorage.getItem('drone_builder_completed_missions');
      return raw ? JSON.parse(raw) : [];
    } catch {
      return [];
    }
  }

  saveCompletedMissions() {
    try {
      localStorage.setItem('drone_builder_completed_missions', JSON.stringify(this.completedMissions));
    } catch {
      // ignore storage quota errors
    }
  }

  updateHomeDashboard() {
    // 1. Topbar student profile pill
    const profLabel = document.getElementById('home-profile-label');
    if (profLabel) {
      if (this.studentProfile && this.studentProfile.name) {
        profLabel.innerText = this.studentProfile.studentId
          ? `${this.studentProfile.name} (${this.studentProfile.studentId})`
          : this.studentProfile.name;
      } else {
        profLabel.innerText = 'ตั้งชื่อผู้เรียน (สำหรับ CSV)';
      }
    }

    // 2. Lessons & Quiz progress indicator
    const progLessons = document.getElementById('home-progress-lessons');
    if (progLessons) {
      const totalQ = typeof QUIZ_QUESTIONS !== 'undefined' ? QUIZ_QUESTIONS.length : 10;
      const preTxt = this.savedScores.pre !== null ? `ทำแล้ว (${this.savedScores.pre}/${totalQ})` : 'ยังไม่ทำ';
      const postTxt = this.savedScores.post !== null ? `ทำแล้ว (${this.savedScores.post}/${totalQ})` : 'ยังไม่ทำ';
      progLessons.innerText = `Pre-test: ${preTxt} · Post-test: ${postTxt}`;
    }

    // 3. Assembly progress & live 3D telemetry chips
    const stats = calculateDroneStats(this.assembly);
    const progAsm = document.getElementById('home-progress-assembly');
    const totalSlots = stats.totalSlots || Object.keys(DRONE_PARTS).length;
    if (progAsm) {
      progAsm.innerText = stats.isComplete
        ? `ติดตั้งแล้ว ${stats.installedCount}/${totalSlots} ชิ้น · T/W ${stats.twRatio}x (${stats.isCompatible ? 'พร้อมบิน' : 'เช็กความเข้ากันได้'})`
        : `ติดตั้งแล้ว ${stats.installedCount}/${totalSlots} ชิ้น · ยังไม่ครบ`;
    }
    const chipTw = document.getElementById('home-telem-tw');
    if (chipTw) chipTw.innerText = stats.isComplete ? `T/W ${stats.twRatio}x` : 'T/W —';
    const chipBat = document.getElementById('home-telem-bat');
    if (chipBat && stats.resolvedParts && stats.resolvedParts.battery) {
      const chem = stats.resolvedParts.battery.chemistry === 'liion' ? 'Li-Ion' : 'LiPo';
      chipBat.innerText = `${stats.resolvedParts.battery.cells || 4}S ${chem} 100%`;
    }

    // 4. Missions progress indicator
    const progMis = document.getElementById('home-progress-missions');
    if (progMis) {
      const doneCount = this.completedMissions ? this.completedMissions.length : 0;
      progMis.innerText = `ผ่านภารกิจแล้ว ${doneCount}/3 ด่าน`;
    }
  }

  exportEvalCSV() {
    if (!this.evalRecords.length) {
      alert('ยังไม่มีข้อมูลผลประเมินที่บันทึกไว้ กรุณากรอกและกด "บันทึกผลประเมิน" อย่างน้อย 1 รายการก่อนส่งออก CSV');
      return;
    }
    const totalQ = typeof QUIZ_QUESTIONS !== 'undefined' ? QUIZ_QUESTIONS.length : 14;
    const headers = [
      'วันเวลา',
      'ชื่อ-นามสกุล',
      'รหัสนักศึกษา',
      `คะแนน Pre-test (/${totalQ})`,
      `คะแนน Post-test (/${totalQ})`,
      'หัวข้อที่ 1 (ความสวยงาม/ใช้งานง่าย)',
      'หัวข้อที่ 2 (ความครบถ้วนเนื้อหา)',
      'หัวข้อที่ 3 (ความสมจริงโมเดล 3D)',
      'หัวข้อที่ 4 (ระบบจำลองการบิน)',
      'หัวข้อที่ 5 (เข้าใจระบบโดรนมากขึ้น)',
      'หัวข้อที่ 6 (ความพึงพอใจโดยรวม)',
      'ค่าเฉลี่ย (0-5)'
    ];
    const escapeCell = (val) => `"${String(val ?? '').replace(/"/g, '""')}"`;
    const rows = this.evalRecords.map(r => [
      r.timestamp,
      r.name || 'ไม่ระบุชื่อ',
      r.studentId || '-',
      r.pre !== null && r.pre !== undefined ? r.pre : '-',
      r.post !== null && r.post !== undefined ? r.post : '-',
      r.ratings.c1,
      r.ratings.c2,
      r.ratings.c3,
      r.ratings.c4,
      r.ratings.c5,
      r.ratings.c6,
      r.avg
    ].map(escapeCell).join(','));

    // UTF-8 BOM (\uFEFF) ensures Thai text renders cleanly in Microsoft Excel
    const csvContent = '\uFEFF' + [headers.map(escapeCell).join(','), ...rows].join('\r\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `drone_builder_evaluation_${new Date().toISOString().slice(0, 10)}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  renderLessonsScreen() {
    const tabsEl = document.getElementById('learning-unit-tabs');
    const bodyEl = document.getElementById('learning-unit-content');
    const partsRowsEl = document.getElementById('lesson-rows-container');
    if (!tabsEl || !bodyEl || typeof LEARNING_UNITS === 'undefined') return;

    tabsEl.innerHTML = '';
    LEARNING_UNITS.forEach(u => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = `unit-tab-btn ${this.activeUnitId === u.id ? 'active' : ''}`;
      btn.innerHTML = `<b>${u.num}</b> <span>${u.title}</span>`;
      btn.addEventListener('click', () => {
        this.activeUnitId = u.id;
        this.renderLessonsScreen();
        document.querySelectorAll('.nav-tab-btn').forEach(nb => {
          if (nb.id === 'btn-nav-thai-law') nb.classList.toggle('active', this.activeUnitId === 'unit6');
          else if (nb.dataset.screen === 'lessons') nb.classList.toggle('active', this.activeUnitId !== 'unit6');
        });
      });
      tabsEl.appendChild(btn);
    });

    const unit = LEARNING_UNITS.find(u => u.id === this.activeUnitId) || LEARNING_UNITS[0];
    bodyEl.innerHTML = `
      <div class="unit-banner">
        <small>${unit.num} · ${unit.enTitle}</small>
        <h3>${unit.title}</h3>
        <p>${unit.summary}</p>
      </div>
    `;

    if (unit.isDynamicPartsUnit) {
      partsRowsEl.style.display = 'flex';
      partsRowsEl.innerHTML = '';
      for (const partKey in DRONE_PARTS) {
        const partDef = DRONE_PARTS[partKey];
        const selectedOptId = this.assembly[partKey];
        const selectedOpt = selectedOptId ? partDef.options.find(o => o.id === selectedOptId) : null;
        const row = document.createElement('div');
        row.className = 'lesson-row-card';
        row.innerHTML = `
          <div class="lesson-part-number">${partDef.num} ${partDef.name}</div>
          <div class="lesson-part-choice">${selectedOpt ? selectedOpt.name : '— ยังไม่ได้เลือก —'}</div>
          <div class="lesson-part-text">${selectedOpt ? selectedOpt.lesson : partDef.lesson}</div>
        `;
        partsRowsEl.appendChild(row);
      }
      return;
    }

    partsRowsEl.style.display = 'none';

    if (unit.cards) {
      const grid = document.createElement('div');
      grid.className = 'drone-types-grid';
      unit.cards.forEach(c => {
        const card = document.createElement('div');
        card.className = 'drone-type-card';
        card.innerHTML = `
          <span class="type-tag">${c.tag}</span>
          <h4>${c.name}</h4>
          <p><b>จุดเด่น:</b> ${c.pros}</p>
          <p><b>ข้อจำกัด:</b> ${c.cons}</p>
          <p><b>การใช้งาน:</b> ${c.useCase}</p>
        `;
        grid.appendChild(card);
      });
      bodyEl.appendChild(grid);
    }

    if (unit.ledLegend) {
      const ledGrid = document.createElement('div');
      ledGrid.className = 'led-legend-grid';
      unit.ledLegend.forEach(item => {
        const card = document.createElement('div');
        card.className = 'led-legend-card';
        card.innerHTML = `
          <div class="led-legend-swatch" style="background:${item.color}; box-shadow:0 0 12px ${item.color};"></div>
          <div>
            <h4>${item.icon} ${item.name}</h4>
            <p>${item.meaning}</p>
          </div>
        `;
        ledGrid.appendChild(card);
      });
      bodyEl.appendChild(ledGrid);
    }

    if (unit.isThaiLawUnit && typeof evaluateThailandDroneLaw === 'function') {
      const stats = calculateDroneStats(this.assembly);
      const law = evaluateThailandDroneLaw(stats);
      const liveBox = document.createElement('div');
      liveBox.className = 'law-live-check-card';
      liveBox.innerHTML = `
        <div class="law-live-head">
          <div>
            <span class="law-live-kicker">⚖️ วิเคราะห์กฎหมายไทยแบบ Real-Time จากโดรนที่คุณประกอบตอนนี้</span>
            <h4>น้ำหนักรวม ${law.weightKg} กก. · ${law.weightClassLabel}</h4>
          </div>
          <span class="badge-tag ${law.needCAAT ? 'warning' : 'success'}">${law.needCAAT ? 'ต้องขึ้นทะเบียน กสทช. + กพท.' : 'ขึ้นทะเบียน กสทช.'}</span>
        </div>
        <p class="law-live-summary">${law.summaryShort}</p>
        <div class="law-live-chips">
          <span class="law-chip">🏛️ กสทช.: ภายใน 30 วัน</span>
          <span class="law-chip">${law.needCAAT ? '📋 กพท. (CAAT): บังคับขึ้นทะเบียน + ประกัน ≥ 1 ล้านบาท' : '📋 กพท.: ยกเว้นสันทนาการ (ไร้กล้อง ≤ 2 กก.)'}</span>
          <span class="law-chip">👥 ห่างบุคคล/อาคาร ≥ ${law.minHorizDistanceM} ม.</span>
          <span class="law-chip">📏 เพดานบิน ≤ 90 ม.</span>
          <span class="law-chip">✈️ ห่างสนามบิน ≥ 9 กม.</span>
        </div>
      `;
      bodyEl.appendChild(liveBox);
    }

    if (unit.lawRulesGrid) {
      const rulesGrid = document.createElement('div');
      rulesGrid.className = 'law-rules-grid';
      unit.lawRulesGrid.forEach(r => {
        const card = document.createElement('div');
        card.className = 'law-rule-card';
        card.innerHTML = `
          <div class="law-rule-top">
            <span class="law-rule-icon">${r.icon}</span>
            <span class="law-rule-badge">${r.badge}</span>
          </div>
          <h4>${r.title}</h4>
          <p>${r.desc}</p>
        `;
        rulesGrid.appendChild(card);
      });
      bodyEl.appendChild(rulesGrid);
    }

    if (unit.lawPenalties) {
      const penTitle = document.createElement('div');
      penTitle.className = 'law-penalty-heading';
      penTitle.innerText = '🚨 บทลงโทษตามกฎหมายไทย (แยกตามหน่วยงาน — โทษร้ายแรงห้ามละเลย)';
      bodyEl.appendChild(penTitle);

      const penGrid = document.createElement('div');
      penGrid.className = 'law-penalties-grid';
      unit.lawPenalties.forEach(item => {
        const card = document.createElement('div');
        card.className = 'law-penalty-card';
        card.innerHTML = `
          <div class="law-penalty-agency">${item.agency}</div>
          <div class="law-penalty-act">${item.act}</div>
          <p class="law-penalty-offense"><b>ความผิด:</b> ${item.offense}</p>
          <div class="law-penalty-punish">⚖️ ${item.penalty}</div>
        `;
        penGrid.appendChild(card);
      });
      bodyEl.appendChild(penGrid);
    }

    if (unit.sections) {
      unit.sections.forEach(sec => {
        const card = document.createElement('div');
        card.className = 'unit-section-card';
        let inner = `<h4>${sec.heading}</h4>`;
        if (sec.body) inner += `<p>${sec.body}</p>`;
        if (sec.bullets) {
          inner += `<ul>${sec.bullets.map(b => `<li>${b}</li>`).join('')}</ul>`;
        }
        card.innerHTML = inner;
        bodyEl.appendChild(card);
      });
    }
  }

  renderQuizScreen() {
    if (typeof QUIZ_QUESTIONS === 'undefined') return;

    const btnPre = document.getElementById('btn-quiz-pre');
    const btnPost = document.getElementById('btn-quiz-post');
    const btnComp = document.getElementById('btn-quiz-compare');
    const btnEval = document.getElementById('btn-quiz-eval');
    if (btnPre) btnPre.classList.toggle('primary', this.quizMode === 'pre');
    if (btnPost) btnPost.classList.toggle('primary', this.quizMode === 'post');
    if (btnComp) btnComp.classList.toggle('primary', this.quizMode === 'compare');
    if (btnEval) btnEval.classList.toggle('primary', this.quizMode === 'eval');

    // Score Comparison Header Cards
    const compEl = document.getElementById('quiz-compare-panel');
    const preVal = this.savedScores.pre;
    const postVal = this.savedScores.post;
    const total = QUIZ_QUESTIONS.length;
    let diffText = 'รอทำครบทั้ง 2 รอบ';
    if (preVal !== null && postVal !== null) {
      const diff = postVal - preVal;
      const pct = Math.round((diff / total) * 100);
      diffText = diff >= 0 ? `+${diff} คะแนน (+${pct}%)` : `${diff} คะแนน (${pct}%)`;
    }

    compEl.innerHTML = `
      <div class="quiz-score-card">
        <small>คะแนนก่อนเรียน (Pre-test)</small>
        <strong>${preVal !== null ? `${preVal} / ${total}` : 'ยังไม่ทำ'}</strong>
      </div>
      <div class="quiz-score-card">
        <small>คะแนนหลังเรียน (Post-test)</small>
        <strong>${postVal !== null ? `${postVal} / ${total}` : 'ยังไม่ทำ'}</strong>
      </div>
      <div class="quiz-score-card highlight">
        <small>พัฒนาการเรียนรู้ (เปรียบเทียบ)</small>
        <strong>${diffText}</strong>
      </div>
    `;

    const qBox = document.getElementById('quiz-questions-box');
    qBox.innerHTML = '';

    if (this.quizMode === 'compare') {
      const summary = document.createElement('div');
      summary.className = 'unit-section-card';
      summary.innerHTML = `
        <h4>สรุปผลการเปรียบเทียบคะแนนก่อนเรียนและหลังเรียน</h4>
        <p>${
          preVal === null
            ? 'กรุณาทำแบบทดสอบก่อนเรียน (Pre-test) ก่อนเริ่มศึกษาบทเรียนทั้ง 6 หน่วย (รวมกฎหมายโดรนไทย)'
            : postVal === null
              ? `คุณได้คะแนนก่อนเรียน ${preVal}/${total} คะแนน — เมื่อศึกษาบทเรียน 6 หน่วย (รวมกฎหมายไทย CAAT/กสทช.) และทดลองประกอบ-บินโดรนแล้ว ให้กดทำแบบทดสอบหลังเรียน (Post-test) เพื่อเปรียบเทียบผล`
              : `คะแนนก่อนเรียน: ${preVal}/${total} (${Math.round((preVal / total) * 100)}%) • คะแนนหลังเรียน: ${postVal}/${total} (${Math.round((postVal / total) * 100)}%) • ผลต่างพัฒนาการ: ${diffText}`
        }</p>
        <div style="display:flex; gap:10px; margin-top:8px;">
          <button class="btn-header-action primary" id="btn-cmp-go-lessons">📖 กลับไปทบทวนบทเรียน 6 หน่วย</button>
          <button class="btn-header-action" id="btn-cmp-reset-scores">↺ ล้างคะแนนเพื่อทดสอบใหม่</button>
        </div>
      `;
      qBox.appendChild(summary);
      summary.querySelector('#btn-cmp-go-lessons').addEventListener('click', () => this.showScreen('lessons'));
      summary.querySelector('#btn-cmp-reset-scores').addEventListener('click', () => {
        this.savedScores = { pre: null, post: null };
        this.saveScores();
        this.renderQuizScreen();
      });
      return;
    }

    if (this.quizMode === 'eval') {
      const criteriaList = typeof EVALUATION_CRITERIA !== 'undefined' ? EVALUATION_CRITERIA : [];
      const evalCard = document.createElement('div');
      evalCard.className = 'unit-section-card';

      const avgAll = this.evalRecords.length
        ? (this.evalRecords.reduce((sum, r) => sum + parseFloat(r.avg), 0) / this.evalRecords.length).toFixed(2)
        : '—';

      evalCard.innerHTML = `
        <h4>⭐ แบบประเมินความพึงพอใจสื่อการเรียนรู้โดรน (ระดับคะแนน 0 – 5)</h4>
        <p>เกณฑ์การให้คะแนน: <b>5</b> = มากที่สุด · <b>4</b> = มาก · <b>3</b> = ปานกลาง · <b>2</b> = น้อย · <b>1</b> = น้อยที่สุด · <b>0</b> = ควรปรับปรุงอย่างยิ่ง (ระบบจะบันทึกคู่กับคะแนน Pre-test / Post-test ลงในฐานข้อมูลเครื่องอัตโนมัติ)</p>
        <div class="eval-student-row">
          <input type="text" class="eval-input" id="eval-input-name" placeholder="ชื่อ-นามสกุลผู้ประเมิน (เช่น นายสมชาย ใจดี)" value="${this.evalForm.name}">
          <input type="text" class="eval-input" id="eval-input-id" placeholder="รหัสนักศึกษา / ระดับชั้น (เช่น ปวช.3 ชค.)" value="${this.evalForm.studentId}">
        </div>
        <div id="eval-criteria-rows"></div>
        <div class="quiz-submit-row" style="margin-top:16px; flex-wrap:wrap; gap:10px;">
          <span style="font-size:12.5px; color:var(--muted);">
            บันทึกข้อมูลประชากรแล้ว <b>${this.evalRecords.length}</b> คน · ค่าเฉลี่ยรวม (x̄): <b style="color:var(--accent);">${avgAll} / 5.00</b>
          </span>
          <div style="display:flex; gap:8px; flex-wrap:wrap;">
            <button type="button" class="btn-header-action primary" id="btn-save-eval">💾 บันทึกผลประเมิน</button>
            <button type="button" class="btn-header-action" id="btn-export-csv">📥 ดาวน์โหลดไฟล์ CSV (Excel)</button>
            ${this.evalRecords.length ? '<button type="button" class="btn-header-action" id="btn-clear-eval">🗑 ล้างประวัติทั้งหมด</button>' : ''}
          </div>
        </div>
      `;
      qBox.appendChild(evalCard);

      const nameInput = evalCard.querySelector('#eval-input-name');
      const idInput = evalCard.querySelector('#eval-input-id');
      nameInput.addEventListener('input', (e) => { this.evalForm.name = e.target.value; });
      idInput.addEventListener('input', (e) => { this.evalForm.studentId = e.target.value; });

      const rowsBox = evalCard.querySelector('#eval-criteria-rows');
      criteriaList.forEach((c, idx) => {
        const currentVal = this.evalForm.ratings[c.id] ?? 5;
        const row = document.createElement('div');
        row.className = 'quiz-q-card';
        row.style.marginBottom = '10px';
        const btnsHtml = [0, 1, 2, 3, 4, 5].map(score => `
          <button type="button" class="eval-score-btn ${currentVal === score ? 'active' : ''}" data-cid="${c.id}" data-score="${score}">${score}</button>
        `).join('');
        row.innerHTML = `
          <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:10px;">
            <span style="font-size:13.5px; font-weight:600;">${idx + 1}. ${c.label}</span>
            <div class="eval-scale-row">${btnsHtml}</div>
          </div>
        `;
        row.querySelectorAll('.eval-score-btn').forEach(btn => {
          btn.addEventListener('click', () => {
            this.evalForm.ratings[btn.dataset.cid] = parseInt(btn.dataset.score, 10);
            this.renderQuizScreen();
          });
        });
        rowsBox.appendChild(row);
      });

      evalCard.querySelector('#btn-save-eval').addEventListener('click', () => {
        const vals = criteriaList.map(c => this.evalForm.ratings[c.id] ?? 0);
        const avg = vals.length ? (vals.reduce((a, b) => a + b, 0) / vals.length).toFixed(2) : '0.00';
        this.evalRecords.push({
          timestamp: new Date().toLocaleString('th-TH'),
          name: this.evalForm.name.trim() || `ผู้ทดสอบคนที่ ${this.evalRecords.length + 1}`,
          studentId: this.evalForm.studentId.trim() || '-',
          pre: this.savedScores.pre,
          post: this.savedScores.post,
          ratings: { ...this.evalForm.ratings },
          avg
        });
        this.saveEvalRecords();
        this.evalForm.name = '';
        this.evalForm.studentId = '';
        alert(`บันทึกผลประเมินเรียบร้อย! ค่าเฉลี่ยของคุณคือ ${avg} / 5.00 (มีข้อมูลสะสมทั้งหมด ${this.evalRecords.length} คน)`);
        this.renderQuizScreen();
      });

      evalCard.querySelector('#btn-export-csv').addEventListener('click', () => this.exportEvalCSV());

      const btnClear = evalCard.querySelector('#btn-clear-eval');
      if (btnClear) {
        btnClear.addEventListener('click', () => {
          if (confirm('ต้องการล้างประวัติผลประเมินที่บันทึกไว้ทั้งหมดใช่หรือไม่?')) {
            this.evalRecords = [];
            this.saveEvalRecords();
            this.renderQuizScreen();
          }
        });
      }
      return;
    }

    QUIZ_QUESTIONS.forEach((q, idx) => {
      const card = document.createElement('div');
      card.className = 'quiz-q-card';
      const chosen = this.quizAnswers[q.id];

      const optsHtml = q.options.map((optText, oIdx) => {
        let cls = 'quiz-opt-btn';
        if (this.quizSubmitted) {
          if (oIdx === q.answer) cls += ' correct';
          else if (chosen === oIdx) cls += ' wrong';
        } else if (chosen === oIdx) {
          cls += ' selected';
        }
        return `<button type="button" class="${cls}" data-qid="${q.id}" data-oidx="${oIdx}">${['ก', 'ข', 'ค', 'ง'][oIdx]}. ${optText}</button>`;
      }).join('');

      card.innerHTML = `
        <div class="quiz-q-header">
          <span class="quiz-q-unit">${q.unit}</span>
          <span>ข้อ ${idx + 1}. ${q.question}</span>
        </div>
        <div class="quiz-options-grid">${optsHtml}</div>
      `;

      card.querySelectorAll('.quiz-opt-btn').forEach(btn => {
        btn.addEventListener('click', () => {
          if (this.quizSubmitted) return;
          this.quizAnswers[q.id] = parseInt(btn.dataset.oidx, 10);
          this.renderQuizScreen();
        });
      });

      qBox.appendChild(card);
    });

    const submitRow = document.createElement('div');
    submitRow.className = 'quiz-submit-row';
    const answeredCount = Object.keys(this.quizAnswers).length;
    submitRow.innerHTML = `
      <span style="font-size:12.5px; color:var(--muted);">ตอบแล้ว ${answeredCount} / ${total} ข้อ</span>
      <button type="button" class="btn-header-action primary" id="btn-submit-quiz">
        ${this.quizSubmitted ? '📊 ดูผลเปรียบเทียบคะแนน →' : `ส่งคำตอบ${this.quizMode === 'pre' ? 'ก่อนเรียน (Pre-test)' : 'หลังเรียน (Post-test)'}`}
      </button>
    `;
    qBox.appendChild(submitRow);

    submitRow.querySelector('#btn-submit-quiz').addEventListener('click', () => {
      if (this.quizSubmitted) {
        this.quizMode = 'compare';
        this.renderQuizScreen();
        return;
      }
      if (answeredCount < total) {
        alert(`กรุณาตอบให้ครบทั้ง ${total} ข้อก่อนส่งคำตอบ (ตอบแล้ว ${answeredCount}/${total} ข้อ)`);
        return;
      }
      let score = 0;
      QUIZ_QUESTIONS.forEach(q => {
        if (this.quizAnswers[q.id] === q.answer) score++;
      });
      this.savedScores[this.quizMode] = score;
      this.saveScores();
      this.quizSubmitted = true;
      this.renderQuizScreen();
    });
  }

  launchFlight(mission) {
    const stats = calculateDroneStats(this.assembly);
    this.selectedMission = mission;

    this.flightSim.setDroneAssembly(this.assembly, stats, this.paintConfig);
    this.flightSim.setMission(mission);

    document.getElementById('hud-mission-title').innerText = mission.title;
    document.getElementById('hud-time-label').innerText = mission.timeLimit ? 'เวลาเหลือ' : 'เวลาบิน';
    document.getElementById('hud-checkpoint-val').innerText = mission.checkpoints.length > 0 ? `0 / ${mission.checkpoints.length}` : '—';

    // Hide RC widget in Free Flight (matches surge.sh step_30), show in Mission Flight (step_26)
    const rcWidget = document.getElementById('rc-widget-osd');
    if (rcWidget) {
      rcWidget.style.display = mission.id === 'free_flight' ? 'none' : 'flex';
    }

    const btnCam = document.getElementById('btn-toggle-camera');
    if (btnCam) {
      btnCam.innerText = this.flightSim.isFPV ? '📷 FPV' : '📷 มุมมองที่ 3';
      btnCam.classList.toggle('primary', this.flightSim.isFPV);
    }
    const reticle = document.getElementById('fpv-crosshair-hud');
    if (reticle) reticle.classList.toggle('hidden', !this.flightSim.isFPV);

    this.showScreen('simulator');
  }

  updateFlightHUD(telemetry) {
    const targetAlt = this.selectedMission && this.selectedMission.targetAlt;
    document.getElementById('hud-alt-val').innerText = targetAlt
      ? `${telemetry.altitude} ม. / ${targetAlt}`
      : `${telemetry.altitude} ม.`;
    document.getElementById('hud-speed-val').innerText = `${telemetry.speedKmh} กม./ชม.`;

    if (this.selectedMission && this.selectedMission.timeLimit) {
      const rem = Math.max(0, Math.floor(this.selectedMission.timeLimit - telemetry.flightTimeSeconds));
      const mins = Math.floor(rem / 60);
      const secs = rem % 60;
      document.getElementById('hud-time-val').innerText = `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;
    } else {
      document.getElementById('hud-time-val').innerText = telemetry.timeStr;
    }

    const voltStr = telemetry.batteryVoltage ? ` (${telemetry.batteryVoltage.toFixed(1)}V)` : '';
    document.getElementById('hud-bat-val').innerText = `${telemetry.batteryPercent}%${voltStr}`;
    document.getElementById('hud-bat-bar').style.width = `${telemetry.batteryPercent}%`;

    const devLabel = document.getElementById('osd-input-device');
    if (devLabel && telemetry.inputDevice) {
      const modeTag = telemetry.flightModeLabel ? ` · ${telemetry.flightModeLabel}` : '';
      devLabel.innerText = `${telemetry.inputDevice}${modeTag}`;
    }

    const windTag = telemetry.liveWindSpeed ? ` · ลม ${telemetry.liveWindSpeed} ม./วิ` : '';
    const rssiTag = typeof telemetry.rssiPercent === 'number' ? ` · RX ${telemetry.rssiPercent}%` : '';
    const massKgTxt = telemetry.curMassGrams ? `${(telemetry.curMassGrams / 1000).toFixed(2)} กก.` : '';
    const twTxt = typeof telemetry.liveTwOge === 'number' ? `T/W ${telemetry.liveTwOge.toFixed(2)}x` : '';
    const massTag = massKgTxt ? ` · ${massKgTxt} (${twTxt})` : '';
    const pState = telemetry.powerState || (telemetry.isArmed ? (telemetry.isLanded ? 'ARMED_IDLE' : 'FLYING') : 'DISARMED');
    document.getElementById('osd-status-text').innerText = pState === 'PRE_ARMING'
      ? 'PRE-ARM — ตรวจระบบ...'
      : telemetry.isArmed
        ? `${pState === 'ARMED_IDLE' ? 'IDLE' : pState === 'AUTO_TAKEOFF' ? 'TAKEOFF' : pState === 'AUTO_LANDING' ? 'LANDING' : 'ARMED'} · ${telemetry.altitude} ม. · ${telemetry.currentAmps || 0}A${massTag}${rssiTag}${windTag}`
        : `DISARM — ล็อกสติ๊ก${massTag}`;

    // Sync HUD Start/Stop, Auto-Takeoff/Kill & RC ARM buttons with real powerState!
    const btnStartStop = document.getElementById('btn-hud-start-stop');
    const btnAutoFlight = document.getElementById('btn-hud-auto-takeoff');
    const btnRcArm = document.getElementById('btn-arm-toggle');
    if (btnStartStop) {
      btnStartStop.classList.remove('start-ready', 'pre-arming', 'stop-armed', 'auto-landing');
      if (pState === 'DISARMED') {
        btnStartStop.innerText = '▶ START (B)';
        btnStartStop.classList.add('start-ready');
      } else if (pState === 'PRE_ARMING') {
        btnStartStop.innerText = '⚙️ ตรวจระบบ...';
        btnStartStop.classList.add('pre-arming');
      } else if (pState === 'ARMED_IDLE') {
        btnStartStop.innerText = '⏹ STOP (B)';
        btnStartStop.classList.add('stop-armed');
      } else if (pState === 'AUTO_LANDING') {
        btnStartStop.innerText = '⏸ ยกเลิกลงจอด';
        btnStartStop.classList.add('auto-landing');
      } else {
        btnStartStop.innerText = '🛬 ลงจอด (L)';
        btnStartStop.classList.add('auto-landing');
      }
    }
    if (btnAutoFlight) {
      const airborne = telemetry.isArmed && !telemetry.isLanded;
      btnAutoFlight.classList.toggle('danger-kill', airborne);
      btnAutoFlight.innerText = airborne ? '🛑 ตัดฉุกเฉิน' : '🛫 บินขึ้น (T)';
      btnAutoFlight.title = airborne
        ? 'ตัดมอเตอร์กลางอากาศทันที (Emergency Kill — ปุ่ม X×2)'
        : 'บินขึ้นอัตโนมัติสู่ความสูง 3.0 ม. (ปุ่ม T)';
    }
    const btnRate = document.getElementById('btn-hud-rate-mode');
    if (btnRate && telemetry.controlRateMode) {
      const rateTexts = {
        cine: '🎯 นุ่มนวล (G)',
        normal: '⚖️ นุ่มสมดุล (G)',
        sport: '⚡ สปอร์ต (G)'
      };
      btnRate.innerText = rateTexts[telemetry.controlRateMode] || '⚖️ นุ่มสมดุล (G)';
      btnRate.classList.toggle('primary', telemetry.controlRateMode === 'cine');
    }
    const btnGfx = document.getElementById('btn-hud-graphics');
    if (btnGfx && telemetry.graphicsQuality) {
      const gfxTexts = {
        fast: '⚡ ลื่นสุด',
        balanced: '⚖️ สมดุล',
        high: '✨ คมชัด'
      };
      btnGfx.innerText = gfxTexts[telemetry.graphicsQuality] || '⚖️ สมดุล';
      btnGfx.classList.toggle('primary', telemetry.graphicsQuality === 'fast');
    }
    const fpsEl = document.getElementById('hud-fps-badge');
    if (fpsEl && telemetry.fps) {
      fpsEl.innerText = `${telemetry.fps} FPS`;
    }
    if (btnRcArm) {
      btnRcArm.classList.toggle('armed', telemetry.isArmed);
      btnRcArm.innerText = pState === 'PRE_ARMING' ? '⚙️ CHECK' : telemetry.isArmed ? '⏹ STOP' : '▶ START';
    }

    // Update live Drone LED status indicator & Start/Stop + 12-subsystem alerts!
    const ledDot = document.getElementById('hud-led-dot');
    const ledText = document.getElementById('hud-led-text');
    if (ledDot && ledText) {
      const massShort = telemetry.curMassGrams ? ` · ${(telemetry.curMassGrams / 1000).toFixed(2)}กก. (T/W ${(telemetry.liveTwOge || 0).toFixed(2)})` : '';
      const payloadTag = (telemetry.payloadType === 'sprayer' || telemetry.payloadType === 'spreader')
        ? ` · ${telemetry.isSpraying ? '💧พ่น' : 'ถัง'} ${telemetry.payloadPercent}% (${telemetry.curMassGrams || 0}ก. · T/W ${(telemetry.liveTwOge || 0).toFixed(2)})`
        : (telemetry.payloadType === 'cargo'
          ? `${telemetry.cargoAttached ? ' · 📦พร้อมปลด(F)' : ' · 📦ปลดแล้ว'}${massShort}`
          : massShort);

      if (telemetry.parachuteDeployed) {
        ledDot.className = 'led-dot ready';
        ledText.innerText = '🪂 PARACHUTE — กางร่มชูชีพชะลอตกปลอดภัย!';
      } else if (telemetry.isArmed && telemetry.lawOverCeiling) {
        ledDot.className = 'led-dot danger';
        ledText.innerText = `⚖️ ผิดกฎหมาย กพท.! เกินเพดาน 90 ม. (${telemetry.altitude} ม. · ปรับสูงสุด 40,000 ฿) ลดระดับลง!`;
      } else if (telemetry.isCrashed || telemetry.batteryPercent <= 25) {
        ledDot.className = 'led-dot danger';
        ledText.innerText = telemetry.isCrashed
          ? '🔴 ERROR — กระแทกแรง/เครื่องตก (กด RESET หรือ START ใหม่)'
          : `🔴 LOW BAT — แบต ${telemetry.batteryPercent}% (${telemetry.batteryVoltage || 0}V) แรงดันตก ควรลงจอด`;
      } else if (telemetry.isOverloaded) {
        ledDot.className = 'led-dot danger';
        ledText.innerText = `🔴 OVERLOAD — น้ำหนัก ${(telemetry.curMassGrams / 1000).toFixed(2)} กก. เกินแรงยก (T/W ${(telemetry.liveTwOge || 0).toFixed(2)} ≤ 1.0) ยกตัวไม่ขึ้น!`;
      } else if (pState === 'PRE_ARMING') {
        ledDot.className = 'led-dot standby';
        ledText.innerText = '⚙️ PRE-ARM — ตรวจ Gyro · ESC · แบตเตอรี่ · ล็อกพิกัด...';
      } else if (telemetry.hudAlertText) {
        ledDot.className = 'led-dot ready';
        ledText.innerText = telemetry.hudAlertText;
      } else if (telemetry.isArmed && telemetry.vrsIntensity > 0.35) {
        ledDot.className = 'led-dot danger';
        ledText.innerText = '🟠 VRS — ลมวนใต้ใบพัด! เอียงบินออกด้านข้าง (W/Q/E)';
      } else if (telemetry.isArmed && telemetry.lidarBraking) {
        ledDot.className = 'led-dot ready';
        ledText.innerText = '🔵 LIDAR — ตรวจพบสิ่งกีดขวาง กำลังเบรกกันชนอัตโนมัติ!';
      } else if (telemetry.isArmed && typeof telemetry.rssiPercent === 'number' && telemetry.rssiPercent <= 30) {
        ledDot.className = 'led-dot danger';
        ledText.innerText = `🟠 RX LOW (${telemetry.rssiPercent}%) — สัญญาณรีโมตอ่อน บินกลับมาใกล้ฐาน!`;
      } else if (telemetry.isArmed && pState === 'AUTO_TAKEOFF') {
        ledDot.className = 'led-dot ready';
        ledText.innerText = '🔵 AUTO-TAKEOFF — กำลังบินขึ้นอัตโนมัติสู่ความสูง 3.0 ม.';
      } else if (telemetry.isArmed && pState === 'AUTO_LANDING') {
        ledDot.className = 'led-dot ready';
        ledText.innerText = '🔵 AUTO-LAND — กำลังลดระดับลงจอดและดับมอเตอร์อัตโนมัติ (กด L ยกเลิก)';
      } else if (telemetry.isArmed && telemetry.isRTHMode) {
        ledDot.className = 'led-dot ready';
        ledText.innerText = `🔵 ${telemetry.flightModeLabel} — กำลังบินกลับลานจอด H (กด R ยกเลิก)`;
      } else if (telemetry.isArmed && pState === 'ARMED_IDLE') {
        ledDot.className = 'led-dot ready';
        const holdNote = (telemetry.touchdownHoldPct || 0) > 10
          ? `⏹ กำลังดับมอเตอร์อัตโนมัติ (${telemetry.touchdownHoldPct}%)...`
          : `🟢 ARMED (เดินเบา) — กด Space/T บินขึ้น · B หรือ Shift ค้าง ดับเครื่อง${payloadTag}`;
        ledText.innerText = holdNote;
      } else if (telemetry.isArmed) {
        ledDot.className = 'led-dot ready';
        const igeTag = (!telemetry.isLanded && telemetry.igeFactor > 1.04) ? ' · IGE' : '';
        const modeTxt = telemetry.flightModeLabel || 'ARMED';
        ledText.innerText = `🟢 ${modeTxt} · RX ${telemetry.rssiPercent ?? 100}% · ลม ${telemetry.liveWindSpeed || 0} ม./วิ${igeTag}${payloadTag}`;
      } else {
        ledDot.className = 'led-dot standby';
        const readyMode = telemetry.hasGps
          ? (telemetry.flightModeLabel === 'RTK HOLD' ? 'RTK GPS พร้อม' : 'GPS พร้อม')
          : (telemetry.hasBaro ? 'BARO พร้อม (ไม่มี GPS)' : 'MANUAL F4 (ไม่มี Baro/GPS)');
        ledText.innerText = `🟡 STANDBY — ${readyMode}${payloadTag} · กด B สตาร์ท หรือ T บินขึ้น`;
      }
    }

    const horizon = document.getElementById('horizon-line');
    if (horizon) {
      const pitchOffset = telemetry.pitch * 30;
      const rollDeg = -telemetry.roll * (180 / Math.PI);
      horizon.style.transform = `translateY(calc(-50% + ${pitchOffset}px)) rotate(${rollDeg}deg)`;
    }

    const leftThumb = document.getElementById('joystick-left-thumb');
    const rightThumb = document.getElementById('joystick-right-thumb');
    if (leftThumb && this.flightSim) {
      leftThumb.style.transform = `translate(${this.flightSim.input.yaw * 16}px, ${-this.flightSim.input.throttle * 16}px)`;
    }
    if (rightThumb && this.flightSim) {
      rightThumb.style.transform = `translate(${this.flightSim.input.roll * 16}px, ${-this.flightSim.input.pitch * 16}px)`;
    }
  }

  showDebriefModal(result) {
    const stats = calculateDroneStats(this.assembly);
    const debrief = generateDebrief(result, stats);

    if (result.isSuccess && this.selectedMission && this.selectedMission.id !== 'free_flight') {
      if (!this.completedMissions.includes(this.selectedMission.id)) {
        this.completedMissions.push(this.selectedMission.id);
        this.saveCompletedMissions();
        this.updateHomeDashboard();
      }
    }

    document.getElementById('debrief-mode-sub').innerText = this.selectedMission.title;
    document.getElementById('debrief-main-title').innerText = result.isSuccess ? 'ภารกิจสำเร็จ!' : 'จบเที่ยวบินอิสระ';
    document.getElementById('debrief-time-val').innerText = debrief.timeStr;
    document.getElementById('debrief-bat-val').innerText = `${debrief.batteryRemainPercent}%`;
    document.getElementById('debrief-alt-val').innerText = `${debrief.maxAlt} ม.`;
    document.getElementById('debrief-advice-text').innerText = debrief.advice;

    document.getElementById('modal-debrief').classList.add('active');
  }

  refreshWorkbench() {
    if (this.workbenchRenderer && this.workbenchCamera) {
      const container = document.getElementById('workbench-canvas');
      if (container) {
        const w = container.clientWidth || 720;
        const h = container.clientHeight || 600;
        this.workbenchCamera.aspect = w / h;
        this.workbenchCamera.updateProjectionMatrix();
        this.workbenchRenderer.setSize(w, h);
      }
      this.setWorkbenchView(this.currentViewAngle || 'iso');
    }
  }
}

window.addEventListener('DOMContentLoaded', () => {
  window.droneApp = new DroneApp();
});
