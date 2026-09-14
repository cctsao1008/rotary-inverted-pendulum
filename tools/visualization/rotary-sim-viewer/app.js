import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/controls/OrbitControls.js';

const REQUIRED_STATE_ORDER = ['theta', 'theta_dot', 'phi', 'phi_dot'];
const sceneHost = document.querySelector('#scene');
const timeline = document.querySelector('#timeline');
const playButton = document.querySelector('#play');
const resetButton = document.querySelector('#reset');
const traceFile = document.querySelector('#trace-file');
const liveStartButton = document.querySelector('#live-start');
const liveStopButton = document.querySelector('#live-stop');
const liveScenario = document.querySelector('#live-scenario');
const liveSpeed = document.querySelector('#live-speed');
const liveChip = document.querySelector('#live-chip');

let trace = null;
let frame = 0;
let playing = false;
let lastAdvanceMs = 0;
let liveSource = null;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x303438);

const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 100);
camera.position.set(2.7, 2.05, 3.1);
camera.lookAt(0.45, 0.75, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
sceneHost.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.45, 0.7, 0);
controls.enableDamping = true;
controls.minDistance = 1.8;
controls.maxDistance = 7;

scene.add(new THREE.HemisphereLight(0xf7f0df, 0x303236, 2.1));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
keyLight.position.set(3, 5, 2);
keyLight.castShadow = true;
scene.add(keyLight);

const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(9, 9),
  new THREE.MeshStandardMaterial({ color: 0x35393d, roughness: 0.95, metalness: 0.0 })
);
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);

const grid = new THREE.GridHelper(8, 24, 0x666b70, 0x44494e);
grid.position.y = 0.001;
scene.add(grid);

const dark = new THREE.MeshStandardMaterial({ color: 0x2c3034, roughness: 0.38, metalness: 0.62 });
const metal = new THREE.MeshStandardMaterial({ color: 0x737980, roughness: 0.32, metalness: 0.72 });
const cream = new THREE.MeshStandardMaterial({ color: 0xf1dfc4, roughness: 0.55, metalness: 0.12 });
const accent = new THREE.MeshStandardMaterial({ color: 0xe0a52f, roughness: 0.45, metalness: 0.25 });

function mesh(geometry, material) {
  const object = new THREE.Mesh(geometry, material);
  object.castShadow = true;
  object.receiveShadow = true;
  return object;
}

// Viewer mapping of the project rigid-body contract into Three.js coordinates:
// project +x -> viewer +X, project +y -> viewer -Z, project +z -> viewer +Y.
// phi -> viewer +Y yaw; theta -> viewer -X rotation.
const base = mesh(new THREE.CylinderGeometry(0.52, 0.56, 0.22, 64), cream);
base.position.y = 0.11;
scene.add(base);

const baseRing = mesh(new THREE.CylinderGeometry(0.43, 0.43, 0.06, 64), dark);
baseRing.position.y = 0.25;
scene.add(baseRing);

const rotaryGroup = new THREE.Group();
rotaryGroup.position.y = 0.28;
scene.add(rotaryGroup);

const shaft = mesh(new THREE.CylinderGeometry(0.105, 0.12, 0.34, 40), dark);
shaft.position.y = 0.17;
rotaryGroup.add(shaft);

const hub = mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.08, 48), metal);
hub.position.y = 0.37;
rotaryGroup.add(hub);

const armLength = 1.22;
const arm = mesh(new THREE.BoxGeometry(armLength, 0.12, 0.16), dark);
arm.position.set(armLength / 2, 0.38, 0);
rotaryGroup.add(arm);

const armHighlight = mesh(new THREE.BoxGeometry(armLength * 0.86, 0.018, 0.03), metal);
armHighlight.position.set(armLength * 0.48, 0.448, -0.085);
rotaryGroup.add(armHighlight);

const hinge = new THREE.Group();
hinge.position.set(armLength, 0.38, 0);
rotaryGroup.add(hinge);

const yokeInner = mesh(new THREE.BoxGeometry(0.13, 0.34, 0.14), dark);
yokeInner.position.x = -0.16;
hinge.add(yokeInner);
const yokeOuter = yokeInner.clone();
yokeOuter.position.x = 0.16;
hinge.add(yokeOuter);

const axle = mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.48, 32), metal);
axle.rotation.z = Math.PI / 2;
hinge.add(axle);

const pendulumPivot = new THREE.Group();
hinge.add(pendulumPivot);

const pendulumLength = 1.18;
const rod = mesh(new THREE.BoxGeometry(0.13, pendulumLength, 0.13), cream);
rod.position.y = pendulumLength / 2;
pendulumPivot.add(rod);

const pendulumCap = mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.09, 32), dark);
pendulumCap.position.y = pendulumLength;
pendulumPivot.add(pendulumCap);

const pivotDisk = mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.10, 32), accent);
pivotDisk.rotation.z = Math.PI / 2;
pendulumPivot.add(pivotDisk);

const axisMaterial = new THREE.LineBasicMaterial({ color: 0xf4b63d });
const verticalAxis = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0.02, 0),
    new THREE.Vector3(0, 0.92, 0),
  ]),
  axisMaterial
);
scene.add(verticalAxis);

function resize() {
  const width = sceneHost.clientWidth;
  const height = sceneHost.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = Math.max(width, 1) / Math.max(height, 1);
  camera.updateProjectionMatrix();
}

function degrees(rad) {
  return rad * 180 / Math.PI;
}

function validateStateOrder(order) {
  if (!Array.isArray(order) || order.join('|') !== REQUIRED_STATE_ORDER.join('|')) {
    throw new Error(`state_order must be [${REQUIRED_STATE_ORDER.join(', ')}]`);
  }
}

function validateSample(sample, index = 0) {
  if (!Number.isFinite(sample.t_s)) throw new Error(`sample ${index}: invalid t_s`);
  if (!Array.isArray(sample.state) || sample.state.length !== 4 || !sample.state.every(Number.isFinite)) {
    throw new Error(`sample ${index}: state must contain four finite numbers`);
  }
  return sample;
}

function validateTrace(candidate) {
  if (!candidate || candidate.schema !== 1) throw new Error('viewer trace schema must be 1');
  validateStateOrder(candidate.state_order);
  if (!Array.isArray(candidate.samples) || candidate.samples.length === 0) {
    throw new Error('trace must contain at least one sample');
  }
  candidate.samples.forEach(validateSample);
  return candidate;
}

function stopLive(label = 'LIVE: disconnected') {
  if (liveSource) {
    liveSource.close();
    liveSource = null;
  }
  liveStartButton.disabled = false;
  liveStopButton.disabled = true;
  liveScenario.disabled = false;
  liveSpeed.disabled = false;
  liveChip.textContent = label;
}

function setTrace(nextTrace, label) {
  stopLive();
  trace = validateTrace(nextTrace);
  frame = 0;
  playing = false;
  playButton.disabled = false;
  resetButton.disabled = false;
  timeline.disabled = false;
  playButton.textContent = '▶ Play';
  timeline.min = '0';
  timeline.max = String(trace.samples.length - 1);
  timeline.value = '0';
  document.querySelector('#trace-status').textContent = label;
  renderFrame();
}

function sourceField(name, fallback = '—') {
  return trace?.source?.[name] ?? fallback;
}

function torqueText(value) {
  return Number.isFinite(value) ? `${Number(value).toFixed(4)} Nm` : '—';
}

function renderFrame() {
  if (!trace || trace.samples.length === 0) return;
  const sample = trace.samples[frame];
  const [theta, thetaDot, phi, phiDot] = sample.state;

  rotaryGroup.rotation.y = phi;
  pendulumPivot.rotation.x = -theta;

  document.querySelector('#theta').textContent = `${degrees(theta).toFixed(3)}°`;
  document.querySelector('#theta-dot').textContent = `${thetaDot.toFixed(3)} rad/s`;
  document.querySelector('#phi').textContent = `${degrees(phi).toFixed(3)}°`;
  document.querySelector('#phi-dot').textContent = `${phiDot.toFixed(3)} rad/s`;
  document.querySelector('#torque').textContent = torqueText(sample.arm_torque_nm ?? 0);
  document.querySelector('#requested-torque').textContent = torqueText(sample.requested_arm_torque_nm);
  document.querySelector('#applied-torque').textContent = torqueText(sample.applied_arm_torque_nm);
  document.querySelector('#regime').textContent = sample.control_regime ?? '—';
  document.querySelector('#runtime-state').textContent = sample.runtime_state ?? '—';
  document.querySelector('#authority').textContent = sample.authority ?? '—';
  document.querySelector('#model-class').textContent = sourceField('model_class');
  document.querySelector('#backend').textContent = sourceField('backend');
  document.querySelector('#scope').textContent = sourceField('scope', 'No evidence scope declared.');
  document.querySelector('#model-chip').textContent = `model: ${sourceField('model_class', 'unknown')}`;
  document.querySelector('#backend-chip').textContent = `backend: ${sourceField('backend', 'unknown')}`;
  document.querySelector('#time-readout').textContent = `t = ${sample.t_s.toFixed(3)} s`;
  document.querySelector('#frame-status').textContent = `frame ${frame + 1} / ${trace.samples.length}`;
  timeline.value = String(frame);
}

async function loadDefaultTrace() {
  const response = await fetch('demo-trace.json', { cache: 'no-store' });
  if (!response.ok) throw new Error(`cannot load demo trace: HTTP ${response.status}`);
  setTrace(await response.json(), 'synthetic UI demo');
}

function startLive() {
  stopLive();
  playing = false;
  playButton.textContent = '▶ Play';
  playButton.disabled = true;
  resetButton.disabled = true;
  timeline.disabled = true;
  liveStartButton.disabled = true;
  liveStopButton.disabled = false;
  liveScenario.disabled = true;
  liveSpeed.disabled = true;
  liveChip.textContent = 'LIVE: connecting';
  document.querySelector('#trace-status').textContent = 'live SITL stream';

  const params = new URLSearchParams({
    scenario: liveScenario.value,
    speed: liveSpeed.value,
    fps: '60',
  });
  liveSource = new EventSource(`/api/live?${params.toString()}`);

  liveSource.addEventListener('status', (event) => {
    const status = JSON.parse(event.data);
    if (status.phase === 'simulating') {
      liveChip.textContent = `LIVE: SITL run ${status.run}`;
    } else if (status.phase === 'run-complete') {
      liveChip.textContent = `LIVE: loop ${status.run} complete`;
    }
  });

  liveSource.addEventListener('meta', (event) => {
    const meta = JSON.parse(event.data);
    validateStateOrder(meta.state_order);
    trace = {
      schema: 1,
      source: meta.source,
      state_order: meta.state_order,
      samples: [],
    };
    frame = 0;
    timeline.min = '0';
    timeline.max = '0';
    timeline.value = '0';
    liveChip.textContent = `LIVE: ${meta.scenario} · run ${meta.run}`;
  });

  liveSource.addEventListener('sample', (event) => {
    const sample = validateSample(JSON.parse(event.data), trace?.samples?.length ?? 0);
    if (!trace) return;
    trace.samples.push(sample);
    frame = trace.samples.length - 1;
    timeline.max = String(frame);
    renderFrame();
  });

  liveSource.addEventListener('stream-error', (event) => {
    const detail = JSON.parse(event.data);
    document.querySelector('#scope').textContent = `Live stream error: ${detail.message}`;
    stopLive('LIVE: error');
  });

  liveSource.onerror = () => {
    if (liveSource) {
      document.querySelector('#trace-status').textContent = 'live server unavailable or disconnected';
      stopLive('LIVE: disconnected');
    }
  };
}

playButton.addEventListener('click', () => {
  if (!trace || liveSource) return;
  playing = !playing;
  playButton.textContent = playing ? '❚❚ Pause' : '▶ Play';
  lastAdvanceMs = performance.now();
});

resetButton.addEventListener('click', () => {
  if (liveSource) return;
  playing = false;
  playButton.textContent = '▶ Play';
  frame = 0;
  renderFrame();
});

timeline.addEventListener('input', () => {
  if (liveSource) return;
  playing = false;
  playButton.textContent = '▶ Play';
  frame = Number(timeline.value);
  renderFrame();
});

traceFile.addEventListener('change', async () => {
  const file = traceFile.files?.[0];
  if (!file) return;
  try {
    const parsed = JSON.parse(await file.text());
    setTrace(parsed, `loaded: ${file.name}`);
  } catch (error) {
    alert(`Trace rejected: ${error.message}`);
  } finally {
    traceFile.value = '';
  }
});

liveStartButton.addEventListener('click', startLive);
liveStopButton.addEventListener('click', () => stopLive('LIVE: stopped'));

function animate(now) {
  requestAnimationFrame(animate);
  controls.update();

  if (playing && trace && trace.samples.length > 1 && !liveSource) {
    const current = trace.samples[frame];
    const nextIndex = frame + 1;
    if (nextIndex >= trace.samples.length) {
      playing = false;
      playButton.textContent = '▶ Play';
    } else {
      const next = trace.samples[nextIndex];
      const delayMs = Math.max(16, (next.t_s - current.t_s) * 1000);
      if (now - lastAdvanceMs >= delayMs) {
        frame = nextIndex;
        lastAdvanceMs = now;
        renderFrame();
      }
    }
  }

  renderer.render(scene, camera);
}

window.addEventListener('resize', resize);
window.addEventListener('beforeunload', () => stopLive());
resize();
loadDefaultTrace().catch((error) => {
  document.querySelector('#scope').textContent = error.message;
  document.querySelector('#trace-status').textContent = 'trace load failed';
});
requestAnimationFrame(animate);
