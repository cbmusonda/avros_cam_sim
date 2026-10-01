import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { TransformControls } from './vendor/TransformControls.js';

// World is ROS-style so URDF numbers can be used as-is: x forward, y left, z up.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const D2R = Math.PI / 180;
const STORE_KEY = 'avros-cam-sim-v2';

// ───────────────────────── URDF data (avros.urdf.xacro) ─────────────────────────
const XS = 0.5556; // xsens_height
const CAR_BOXES = [
  { name: 'main chassis',    c: [0.3143, 0, XS - 0.2191],  s: [0.7430, 0.6795, 0.3747], color: 0x8c8c8c },
  { name: 'sensor platform', c: [0.1143, 0, XS + 0.0413],  s: [0.5842, 0.5715, 0.1461], color: 0xb3b3b3 },
  { name: 'left tread',      c: [0.3143, 0.3653, XS - 0.4763],  s: [0.8255, 0.0889, 0.1397], color: 0x404040 },
  { name: 'right tread',     c: [0.3143, -0.3653, XS - 0.4763], s: [0.8255, 0.0889, 0.1397], color: 0x404040 },
  { name: 'IMU',             c: [0, 0, XS],                s: [0.06, 0.06, 0.03],       color: 0x2255cc },
];
const VELODYNE = { c: [0.089, 0, XS + 0.159], r: 0.052, len: 0.072 };
// VLP-16: 16 rings, -15..+15 deg in 2 deg steps, 360 deg azimuth. Range/min-range are display settings.
const LIDAR_ELEV = Array.from({ length: 16 }, (_, i) => -15 + 2 * i);
const LIDAR_RANGE = 30, LIDAR_MIN = 0.4;
const FIGS = [[1.5, 0], [2.5, 0], [3.2, 0], [0.1, 1.5], [0.1, -1.5], [0.1, 3], [0.1, -3]]; // reference figures (x, y)

// ZED X sensor: AR0234, 1920x1200 @ 3 um => 5.76 x 3.6 mm
const SENSOR = { w: 5.76, h: 3.6 };
const rect = (f) => ({ h: 2 * Math.atan(SENSOR.w / 2 / f) / D2R, v: 2 * Math.atan(SENSOR.h / 2 / f) / D2R });
const LENSES = {
  '2.2mm-calc': { label: '2.2 mm (calculated, rectilinear)', ...rect(2.2) },
  '2.2mm-ds':   { label: '2.2 mm (datasheet ~110 x 80, verify)', h: 110, v: 80 },
  '4mm-calc':   { label: '4 mm (calculated, rectilinear)', ...rect(4) },
  '4mm-ds':     { label: '4 mm (datasheet ~80 x 55, verify)', h: 80, v: 55 },
  custom:       { label: 'Custom (imported values)' },
};

const COLORS = ['#e53935', '#1e88e5', '#43a047', '#fb8c00', '#8e24aa', '#00acc1', '#fdd835'];

function defaultCams() {
  const base = { roll: 0, range: 5, visible: true };
  return [
    { id: 1, name: 'zed_front', color: COLORS[0], x: 0.6795, y: 0, z: XS - 0.108, yaw: 0, pitch: 15, lens: '2.2mm-calc', ...base, hfov: LENSES['2.2mm-calc'].h, vfov: LENSES['2.2mm-calc'].v },
    { id: 2, name: 'zed_left', color: COLORS[1], x: 0.098, y: 0.286, z: XS + 0.057, yaw: 90, pitch: 0, lens: '4mm-calc', ...base, hfov: LENSES['4mm-calc'].h, vfov: LENSES['4mm-calc'].v },
    { id: 3, name: 'zed_right', color: COLORS[2], x: 0.098, y: -0.286, z: XS + 0.057, yaw: -90, pitch: 0, lens: '4mm-calc', ...base, hfov: LENSES['4mm-calc'].h, vfov: LENSES['4mm-calc'].v },
  ];
}

const FIELDS = [
  { k: 'x', label: 'X fwd', min: -1, max: 1.5, step: 0.005, unit: 'm' },
  { k: 'y', label: 'Y left', min: -0.8, max: 0.8, step: 0.005, unit: 'm' },
  { k: 'z', label: 'Z up', min: 0, max: 1.3, step: 0.005, unit: 'm' },
  { k: 'yaw', label: 'Yaw', min: -180, max: 180, step: 0.5, unit: '°' },
  { k: 'pitch', label: 'Pitch ↓', min: -90, max: 90, step: 0.5, unit: '°' },
  { k: 'roll', label: 'Roll', min: -45, max: 45, step: 0.5, unit: '°' },
  { k: 'hfov', label: 'H FOV', min: 10, max: 170, step: 0.5, unit: '°', fov: true },
  { k: 'vfov', label: 'V FOV', min: 10, max: 150, step: 0.5, unit: '°', fov: true },
  { k: 'range', label: 'Range', min: 0.5, max: 15, step: 0.1, unit: 'm' },
];

// ───────────────────────── state ─────────────────────────
let cams = defaultCams();
let selectedId = 1;
let nextId = 4;
const flags = { car: true, frustum: true, foot: true, fig: false, lane: false, pov: true, lidar: true, lidarBeams: true };
const lidarDefault = () => ({ x: VELODYNE.c[0], y: VELODYNE.c[1], z: VELODYNE.c[2] });
const lidar = lidarDefault();
let selKind = 'cam'; // 'cam' | 'lidar': what the gizmo is attached to

function save() {
  try { localStorage.setItem(STORE_KEY, JSON.stringify({ cams, selectedId, nextId, flags, lidar })); } catch (e) { /* ignore */ }
}
function load() {
  try {
    const raw = localStorage.getItem(STORE_KEY);
    if (!raw) return;
    const d = JSON.parse(raw);
    if (Array.isArray(d.cams) && d.cams.length) { cams = d.cams; selectedId = d.selectedId; nextId = d.nextId || cams.length + 1; }
    if (d.flags) Object.assign(flags, d.flags);
    if (d.lidar) Object.assign(lidar, d.lidar);
  } catch (e) { /* ignore */ }
}
load();
const sel = () => cams.find((c) => c.id === selectedId) || cams[0];

// ───────────────────────── three.js scene ─────────────────────────
const view = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
view.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12161b);

const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
camera.layers.enable(1); // layer 1 = editor overlays (frustums, camera bodies, gizmo)
camera.position.set(-3, -3.6, 2.6);

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0.2747, 0, 0.4);
orbit.enableDamping = true;
orbit.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.75));
const sun = new THREE.DirectionalLight(0xffffff, 1.1);
sun.position.set(-4, -6, 10);
scene.add(sun);

// ground: 20 ft x 20 ft (6.096 m) pad centred under the car, grid = 1 ft cells
const PAD = 20 * 0.3048, PAD_CX = (-0.1778 + 0.7271) / 2; // centre of the car's full x-extent (platform back .. tread front), y is symmetric
const ground = new THREE.Mesh(new THREE.PlaneGeometry(PAD, PAD), new THREE.MeshStandardMaterial({ color: 0x2b3a2f }));
ground.position.set(PAD_CX, 0, -0.002);
scene.add(ground);
const grid = new THREE.GridHelper(PAD, 20, 0x7a8b7f, 0x475549);
grid.rotation.x = Math.PI / 2;
grid.position.set(PAD_CX, 0, 0);
scene.add(grid);
const axes = new THREE.AxesHelper(0.6); // x red, y green, z blue == ROS convention
axes.position.z = 0.003;
axes.layers.set(1);
scene.add(axes);

// car
const carGroup = new THREE.Group();
scene.add(carGroup);
for (const b of CAR_BOXES) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(...b.s), new THREE.MeshStandardMaterial({ color: b.color }));
  m.position.set(...b.c);
  const e = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0x111111 }));
  m.add(e);
  carGroup.add(m);
}
// reference figures + lane lines
const figGroup = new THREE.Group();
for (const [x, y] of FIGS) {
  const p = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.7, 16), new THREE.MeshStandardMaterial({ color: 0xd9a066 }));
  p.rotation.x = Math.PI / 2;
  p.position.set(x, y, 0.85);
  figGroup.add(p);
}
scene.add(figGroup);
const laneGroup = new THREE.Group();
for (const y of [-1.5, 1.5]) {
  const l = new THREE.Mesh(new THREE.PlaneGeometry(PAD, 0.1), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  l.position.set(PAD_CX, y, 0.002);
  laneGroup.add(l);
}
scene.add(laneGroup);


// ───────────────────────── lidar (VLP-16) ─────────────────────────
const lidarGroup = new THREE.Group();
{
  const v = new THREE.Mesh(new THREE.CylinderGeometry(VELODYNE.r, VELODYNE.r, VELODYNE.len, 24), new THREE.MeshStandardMaterial({ color: 0x222222, emissive: 0xff8800, emissiveIntensity: 0.15 }));
  v.rotation.x = Math.PI / 2;
  v.layers.enable(1); // pickable, still visible in camera POVs
  v.userData.lidar = true;
  lidarGroup.add(v);
  scene.add(lidarGroup);
}
const lidarRings = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ vertexColors: true }));
const lidarBeams = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffb04d, transparent: true, opacity: 0.3 }));
const lidarHits = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ size: 0.05, vertexColors: true }));
for (const m of [lidarRings, lidarBeams, lidarHits]) { m.layers.set(1); m.frustumCulled = false; scene.add(m); }

const OCCLUDERS = CAR_BOXES.map((b) => ({ min: b.c.map((c, i) => c - b.s[i] / 2), max: b.c.map((c, i) => c + b.s[i] / 2) }));

// nearest hit along a ray from o (dir d): ground, car body, or a reference figure; null = nothing within range
function traceRay(o, d) {
  let t = Infinity, type = null;
  if (d[2] < -1e-9) { t = -o[2] / d[2]; type = 'ground'; }
  for (const b of OCCLUDERS) {
    let t0 = -Infinity, t1 = Infinity;
    for (let i = 0; i < 3; i++) {
      if (Math.abs(d[i]) < 1e-12) { if (o[i] < b.min[i] || o[i] > b.max[i]) { t0 = Infinity; break; } continue; }
      const a = (b.min[i] - o[i]) / d[i], c = (b.max[i] - o[i]) / d[i];
      t0 = Math.max(t0, Math.min(a, c)); t1 = Math.min(t1, Math.max(a, c));
    }
    if (t0 <= t1 && t0 >= LIDAR_MIN && t0 < t) { t = t0; type = 'car'; }
  }
  if (flags.fig) {
    for (const [fx, fy] of FIGS) {
      const px = o[0] - fx, py = o[1] - fy, a = d[0] * d[0] + d[1] * d[1];
      if (a < 1e-12) continue;
      const bq = px * d[0] + py * d[1], disc = bq * bq - a * (px * px + py * py - 0.22 * 0.22);
      if (disc < 0) continue;
      const tc = (-bq - Math.sqrt(disc)) / a, z = o[2] + d[2] * tc;
      if (tc >= LIDAR_MIN && tc < t && z >= 0 && z <= 1.7) { t = tc; type = 'fig'; }
    }
  }
  return t >= LIDAR_MIN && t <= LIDAR_RANGE ? { t, type } : null;
}

function updateLidar() {
  lidarGroup.position.set(lidar.x, lidar.y, lidar.z);
  lidarRings.visible = lidarHits.visible = flags.lidar;
  lidarBeams.visible = flags.lidar && flags.lidarBeams;
  const o = [lidar.x, lidar.y, lidar.z];
  const ringPos = [], ringCol = [], beamPos = [], hitPos = [], hitCol = [];
  const lines = [];
  const col = new THREE.Color();
  LIDAR_ELEV.forEach((el, ri) => {
    const ce = Math.cos(el * D2R), se = Math.sin(el * D2R);
    col.setHSL(0.02 + (ri / 15) * 0.6, 1, 0.55);
    let prev = null, first = null, blocked = 0, ground = 0;
    for (let j = 0; j <= 360; j++) {
      const a = (j % 360) * D2R;
      const d = [ce * Math.cos(a), ce * Math.sin(a), se];
      const h = traceRay(o, d);
      const p = h ? [o[0] + d[0] * h.t, o[1] + d[1] * h.t, o[2] + d[2] * h.t] : null;
      const g = h && h.type === 'ground' ? [p[0], p[1], 0.006] : null;
      if (j < 360) {
        if (h && h.type === 'car') blocked++;
        if (g) ground++;
        if (h && h.type !== 'ground') hitPos.push(...p), hitCol.push(...(h.type === 'car' ? [1, 0.25, 0.25] : [1, 0.9, 0.2]));
        if (j % 45 === 0) beamPos.push(...o, ...(p || [o[0] + d[0] * LIDAR_RANGE, o[1] + d[1] * LIDAR_RANGE, o[2] + d[2] * LIDAR_RANGE]));
      }
      if (g && prev) ringPos.push(...prev, ...g), ringCol.push(col.r, col.g, col.b, col.r, col.g, col.b);
      prev = g;
    }
    lines.push({ el, r: el < 0 ? lidar.z / Math.tan(-el * D2R) : null, blocked, ground });
  });
  const setGeo = (m, pos, colors) => {
    m.geometry.dispose();
    m.geometry = new THREE.BufferGeometry();
    m.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    if (colors) m.geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  };
  setGeo(lidarRings, ringPos, ringCol);
  setGeo(lidarBeams, beamPos);
  setGeo(lidarHits, hitPos, hitCol);

  const down = lines.filter((l) => l.r !== null);
  let txt = `Position (${lidar.x.toFixed(3)}, ${lidar.y.toFixed(3)}, ${lidar.z.toFixed(3)}) m, level\n` +
    `Range ${LIDAR_RANGE} m (min ${LIDAR_MIN} m)\n` +
    `Blind radius (steepest beam, -15°): ${down[0].r.toFixed(2)} m\n\n` +
    `ring  elev   ground r   gap    car-blocked\n`;
  down.forEach((l, i) => {
    const gap = i ? (l.r - down[i - 1].r) : null;
    txt += `${String(LIDAR_ELEV.indexOf(l.el)).padStart(3)}  ${String(l.el).padStart(4)}°  ${(l.r > LIDAR_RANGE ? '>' + LIDAR_RANGE : l.r.toFixed(2)).padStart(7)} m  ${gap === null || l.r > LIDAR_RANGE ? '   -  ' : gap.toFixed(2).padStart(5) + 'm'}  ${Math.round(l.blocked / 3.6)}%\n`;
  });
  const up = lines.filter((l) => l.r === null);
  txt += `${up.length} rings above horizon (${up.map((l) => l.el + '°').join(', ')}): ` +
    `${Math.round(up.reduce((s, l) => s + l.blocked, 0) / (up.length * 3.6))}% blocked by car`;
  $('lidarInfo').textContent = txt;
}

// ───────────────────────── camera objects ─────────────────────────
const objs = new Map(); // id -> { group, body, fr, edges, footMesh, footLine }

const frMat = (color) => new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false });

function buildCamObj(c) {
  const group = new THREE.Group();
  group.rotation.order = 'ZYX'; // == URDF rpy (extrinsic XYZ)
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.031, 0.165, 0.036), new THREE.MeshStandardMaterial({ color: c.color, emissive: c.color, emissiveIntensity: 0.25 }));
  body.userData.camId = c.id;
  body.layers.set(1);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.22, 0xffffff, 0.06, 0.04);
  arrow.traverse((o) => o.layers.set(1));
  const fr = new THREE.Mesh(new THREE.BufferGeometry(), frMat(c.color));
  fr.layers.set(1);
  const edges = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: c.color }));
  edges.layers.set(1);
  group.add(body, arrow, fr, edges);
  scene.add(group);

  const footMesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: c.color, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false }));
  const footLine = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: c.color }));
  footMesh.layers.set(1);
  footLine.layers.set(1);
  scene.add(footMesh, footLine);

  const o = { group, body, fr, edges, footMesh, footLine, arrow };
  objs.set(c.id, o);
  return o;
}

function frustumCorners(c) {
  const r = c.range;
  const y = r * Math.tan(c.hfov * D2R / 2);
  const z = r * Math.tan(c.vfov * D2R / 2);
  return [[r, y, z], [r, -y, z], [r, -y, -z], [r, y, -z]];
}

function refresh(c) {
  const o = objs.get(c.id) || buildCamObj(c);
  o.group.position.set(c.x, c.y, c.z);
  o.group.rotation.set(c.roll * D2R, c.pitch * D2R, c.yaw * D2R);
  o.group.visible = c.visible;

  const k = frustumCorners(c);
  const A = [0, 0, 0];
  const v = [A, k[0], k[1], A, k[1], k[2], A, k[2], k[3], A, k[3], k[0], k[0], k[1], k[2], k[0], k[2], k[3]].flat();
  o.fr.geometry.dispose();
  o.fr.geometry = new THREE.BufferGeometry();
  o.fr.geometry.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
  const l = [A, k[0], A, k[1], A, k[2], A, k[3], k[0], k[1], k[1], k[2], k[2], k[3], k[3], k[0]].flat();
  o.edges.geometry.dispose();
  o.edges.geometry = new THREE.BufferGeometry();
  o.edges.geometry.setAttribute('position', new THREE.Float32BufferAttribute(l, 3));
  o.fr.material.color.set(c.color);
  o.edges.material.color.set(c.color);
  o.fr.visible = o.edges.visible = flags.frustum;

  o.group.updateMatrixWorld(true);
  const p = o.group.position.clone();
  const pts = k.map((a) => {
    const q = o.group.localToWorld(new THREE.Vector3(...a));
    if (q.z < 0 && p.z > 0) { // clip the edge ray at the ground
      const t = p.z / (p.z - q.z);
      return new THREE.Vector3(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t, 0.004);
    }
    return new THREE.Vector3(q.x, q.y, 0.004);
  });
  const fp = [pts[0], pts[1], pts[2], pts[0], pts[2], pts[3]].flatMap((q) => [q.x, q.y, q.z]);
  o.footMesh.geometry.dispose();
  o.footMesh.geometry = new THREE.BufferGeometry();
  o.footMesh.geometry.setAttribute('position', new THREE.Float32BufferAttribute(fp, 3));
  o.footLine.geometry.dispose();
  o.footLine.geometry = new THREE.BufferGeometry();
  o.footLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts.flatMap((q) => [q.x, q.y, q.z]), 3));
  o.footMesh.visible = o.footLine.visible = flags.foot && c.visible;
}

function rebuildAll() {
  for (const o of objs.values()) {
    scene.remove(o.group, o.footMesh, o.footLine);
    o.group.traverse((m) => m.geometry && m.geometry.dispose());
  }
  objs.clear();
  transform.detach();
  for (const c of cams) refresh(c);
  attachGizmo();
}

// ───────────────────────── gizmo ─────────────────────────
const transform = new TransformControls(camera, renderer.domElement);
transform.setSize(0.6);
scene.add(transform);
transform.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
transform.addEventListener('objectChange', () => {
  if (selKind === 'lidar') {
    const p = lidarGroup.position;
    lidar.x = p.x; lidar.y = p.y; lidar.z = p.z;
    updateLidar(); syncLidarFields();
    return;
  }
  const c = sel();
  const g = objs.get(c.id).group;
  c.x = g.position.x; c.y = g.position.y; c.z = g.position.z;
  c.roll = g.rotation.x / D2R; c.pitch = g.rotation.y / D2R; c.yaw = g.rotation.z / D2R;
  refresh(c);
  syncFields();
  updateInfo();
});
transform.addEventListener('mouseUp', save);

function attachGizmo() {
  if (selKind === 'lidar') { transform.attach(lidarGroup); return; }
  const o = objs.get(sel().id);
  if (o) transform.attach(o.group);
}
function setMode(m) {
  if (selKind === 'lidar') m = 'translate'; // lidar is position-only for now
  transform.setMode(m);
  transform.setSpace(m === 'rotate' ? 'local' : 'world');
  document.getElementById('modeT').classList.toggle('on', m === 'translate');
  document.getElementById('modeR').classList.toggle('on', m === 'rotate');
}
setMode('translate');

// pick cameras by clicking their body
const ray = new THREE.Raycaster();
ray.layers.set(1);
let down = null;
renderer.domElement.addEventListener('pointerdown', (e) => { down = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4 || transform.dragging) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
  const lh = ray.intersectObject(lidarGroup, true)[0];
  const hit = ray.intersectObjects([...objs.values()].filter((o) => o.group.visible).map((o) => o.body), false)[0];
  if (hit && (!lh || hit.distance < lh.distance)) select(hit.object.userData.camId);
  else if (lh) selectLidar();
});

// ───────────────────────── UI ─────────────────────────
const $ = (id) => document.getElementById(id);
const fieldsEl = $('fields');
const inputs = {};
for (const f of FIELDS) {
  const row = document.createElement('div');
  row.className = 'field';
  if (f.fov) { // read-only: FOV comes from the lens preset
    row.innerHTML = `<label>${f.label}</label><output style="font-weight:600"></output><span style="opacity:.6">${f.unit === '°' ? 'degrees' : f.unit}</span>`;
    inputs[f.k] = { out: row.children[1] };
    fieldsEl.appendChild(row);
    continue;
  }
  row.innerHTML = `<label>${f.label} <span style="opacity:.6">(${f.unit})</span></label><input type="range" min="${f.min}" max="${f.max}" step="${f.step}"><input type="number" step="${f.step}">`;
  const [, range, num] = row.children;
  const set = (val, typing) => {
    val = parseFloat(val);
    if (!Number.isFinite(val)) return;
    const c = sel();
    c[f.k] = val;
        refresh(c); syncFields(typing ? f.k : undefined); updateInfo(); save();
  };
  range.addEventListener('input', () => set(range.value, false));
  num.addEventListener('change', () => set(num.value, true));
  inputs[f.k] = { range, num };
  fieldsEl.appendChild(row);
}
function syncFields(except) {
  const c = sel();
  for (const f of FIELDS) {
    const v = c[f.k];
    if (f.fov) { inputs[f.k].out.textContent = v.toFixed(1); continue; }
    inputs[f.k].range.value = v;
    if (f.k !== except) inputs[f.k].num.value = Math.round(v * 1000) / 1000;
  }
}

const lensSel = $('f-lens');
for (const [k, l] of Object.entries(LENSES)) lensSel.add(new Option(l.label, k));
lensSel.addEventListener('change', () => {
  const c = sel();
  c.lens = lensSel.value;
  if (LENSES[c.lens].h) { c.hfov = LENSES[c.lens].h; c.vfov = LENSES[c.lens].v; }
  refresh(c); syncFields(); updateInfo(); save();
});
$('f-name').addEventListener('input', (e) => { sel().name = e.target.value; renderList(); save(); });

function renderList() {
  const el = $('camlist');
  el.innerHTML = '';
  for (const c of cams) {
    const d = document.createElement('div');
    d.className = 'cam' + (selKind === 'cam' && c.id === selectedId ? ' sel' : '');
    d.innerHTML = `<span class="dot" style="background:${c.color}"></span><span class="nm"></span><input type="checkbox" title="visible" ${c.visible ? 'checked' : ''}>`;
    d.querySelector('.nm').textContent = c.name;
    d.addEventListener('click', () => select(c.id));
    d.querySelector('input').addEventListener('click', (e) => e.stopPropagation());
    d.querySelector('input').addEventListener('change', (e) => { c.visible = e.target.checked; refresh(c); save(); });
    el.appendChild(d);
  }
}

function select(id) {
  selKind = 'cam';
  $('lidarSel').classList.remove('sel');
  selectedId = id;
  renderList();
  $('f-name').value = sel().name;
  lensSel.value = sel().lens in LENSES ? sel().lens : 'custom';
  syncFields();
  attachGizmo();
  updateInfo();
  save();
}

// lidar fields (x/y/z only)
const LIDAR_FIELDS = [
  { k: 'x', label: 'X fwd', min: -1, max: 1.5 },
  { k: 'y', label: 'Y left', min: -0.8, max: 0.8 },
  { k: 'z', label: 'Z up', min: 0, max: 1.3 },
];
const lidarInputs = {};
for (const f of LIDAR_FIELDS) {
  const row = document.createElement('div');
  row.className = 'field';
  row.innerHTML = `<label>${f.label} <span style="opacity:.6">(m)</span></label><input type="range" min="${f.min}" max="${f.max}" step="0.005"><input type="number" step="0.005">`;
  const [, range, num] = row.children;
  const set = (val) => {
    val = parseFloat(val);
    if (!Number.isFinite(val)) return;
    lidar[f.k] = val;
    updateLidar(); syncLidarFields(); save();
  };
  range.addEventListener('input', () => set(range.value));
  num.addEventListener('change', () => set(num.value));
  lidarInputs[f.k] = { range, num };
  $('lidarFields').appendChild(row);
}
function syncLidarFields() {
  for (const f of LIDAR_FIELDS) {
    lidarInputs[f.k].range.value = lidar[f.k];
    lidarInputs[f.k].num.value = Math.round(lidar[f.k] * 1000) / 1000;
  }
}
function selectLidar() {
  selKind = 'lidar';
  $('lidarSel').classList.add('sel');
  renderList();
  setMode('translate');
  attachGizmo();
}
$('lidarSel').addEventListener('click', selectLidar);
$('lidarReset').addEventListener('click', () => { Object.assign(lidar, lidarDefault()); updateLidar(); syncLidarFields(); save(); });

// ground coverage of the selected camera (unclipped by "range")
function updateInfo() {
  const c = sel();
  const o = objs.get(c.id);
  o.group.updateMatrixWorld(true);
  const p = o.group.position;
  const rayHit = (vAng) => {
    const d = new THREE.Vector3(1, 0, -Math.tan(vAng * D2R)).transformDirection(o.group.matrixWorld);
    if (d.z >= -1e-6 || p.z <= 0) return null;
    const t = -p.z / d.z;
    return new THREE.Vector3(p.x + d.x * t, p.y + d.y * t, 0);
  };
  const near = rayHit(c.vfov / 2);
  const far = rayHit(-c.vfov / 2);
  const f = (SENSOR.w / 2) / Math.tan(c.hfov * D2R / 2);
  const fmt = (q) => (q ? `(${q.x.toFixed(2)}, ${q.y.toFixed(2)}) m` : 'none');
  $('info').textContent =
    `${c.name}\n` +
    `FOV ${c.hfov.toFixed(1)}° × ${c.vfov.toFixed(1)}°  (≈ ${f.toFixed(2)} mm rectilinear)\n` +
    `Mount height ${c.z.toFixed(3)} m\n` +
    `Lower edge hits ground: ${fmt(near)}\n` +
    (near ? `  ${Math.hypot(near.x - p.x, near.y - p.y).toFixed(2)} m from camera\n` : '') +
    `Upper edge hits ground: ${far ? fmt(far) : 'never (sees horizon)'}\n` +
    (far ? `  ${Math.hypot(far.x - p.x, far.y - p.y).toFixed(2)} m from camera` : '');
  povLabel();
}

$('addCam').addEventListener('click', () => {
  const c = { id: nextId++, name: `cam_${nextId - 1}`, color: COLORS[(nextId - 2) % COLORS.length], x: 0.3, y: 0, z: 0.75, yaw: 0, pitch: 10, roll: 0, range: 5, visible: true, lens: '4mm-calc', hfov: LENSES['4mm-calc'].h, vfov: LENSES['4mm-calc'].v };
  cams.push(c);
  refresh(c);
  select(c.id);
});
$('delCam').addEventListener('click', () => {
  if (cams.length <= 1) return;
  const o = objs.get(selectedId);
  transform.detach();
  scene.remove(o.group, o.footMesh, o.footLine);
  objs.delete(selectedId);
  cams = cams.filter((c) => c.id !== selectedId);
  select(cams[0].id);
});
$('dupCam').addEventListener('click', () => {
  const s = sel();
  const c = { ...s, id: nextId++, name: `${s.name}_mirror`, color: COLORS[(nextId - 2) % COLORS.length], y: -s.y, yaw: -s.yaw, roll: -s.roll };
  cams.push(c);
  refresh(c);
  select(c.id);
});

$('modeT').addEventListener('click', () => setMode('translate'));
$('modeR').addEventListener('click', () => { if (selKind === 'cam') setMode('rotate'); });
addEventListener('keydown', (e) => {
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) return;
  if (e.key === 't' || e.key === 'T') setMode('translate');
  if ((e.key === 'r' || e.key === 'R') && selKind === 'cam') setMode('rotate');
});

// scene toggles
const applyFlags = () => {
  carGroup.visible = flags.car;
  figGroup.visible = flags.fig;
  laneGroup.visible = flags.lane;
  for (const c of cams) refresh(c);
  updateLidar();
  $('povFrame').style.display = flags.pov ? 'block' : 'none';
};
for (const k of Object.keys(flags)) {
  const el = $('t-' + k);
  el.checked = flags[k];
  el.addEventListener('change', () => { flags[k] = el.checked; applyFlags(); save(); });
}

// camera presets
const VIEWS = {
  persp: [[-2.7, -3.6, 2.6], [0.2747, 0, 0.4]],
  top: [[0.2647, 0, 9], [0.2747, 0, 0]],
  side: [[0.2747, -6, 0.9], [0.2747, 0, 0.5]],
  front: [[7, 0.001, 0.9], [0, 0, 0.5]],
  rear: [[-7, 0.001, 0.9], [0, 0, 0.5]],
};
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => {
  const [pos, tgt] = VIEWS[b.dataset.view];
  camera.position.set(...pos);
  orbit.target.set(...tgt);
  orbit.update();
}));

// export / import
function download(name, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'application/json' }));
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}
$('exp').addEventListener('click', () => download('avros-cameras.json', JSON.stringify({ format: 'avros-cam-sim', version: 1, cameras: cams, lidar }, null, 2)));
$('imp').addEventListener('click', () => $('file').click());
$('file').addEventListener('change', async (e) => {
  try {
    const d = JSON.parse(await e.target.files[0].text());
    if (!Array.isArray(d.cameras) || !d.cameras.length) throw new Error('no cameras');
    cams = d.cameras.map((c, i) => ({ roll: 0, range: 5, visible: true, lens: 'custom', color: COLORS[i % COLORS.length], ...c, id: i + 1 }));
    nextId = cams.length + 1;
    if (d.lidar) Object.assign(lidar, d.lidar);
    updateLidar(); syncLidarFields();
    rebuildAll();
    select(cams[0].id);
  } catch (err) { alert('Could not import: ' + err.message); }
  e.target.value = '';
});
$('urdf').addEventListener('click', () => {
  const r = (n) => Math.round(n * 1e4) / 1e4;
  const text = cams.map((c) =>
    `<!-- ${c.name}: FOV ${c.hfov.toFixed(1)} x ${c.vfov.toFixed(1)} deg -->\n` +
    `<origin xyz="${r(c.x)} ${r(c.y)} \${xsens_height ${c.z - XS >= 0 ? '+' : '-'} ${r(Math.abs(c.z - XS))}}" rpy="${r(c.roll * D2R)} ${r(c.pitch * D2R)} ${r(c.yaw * D2R)}"/>\n` +
    `<!-- absolute z above ground: ${r(c.z)} m -->`).join('\n\n') +
    `\n\n<!-- velodyne (VLP-16) -->\n<origin xyz="${r(lidar.x)} ${r(lidar.y)} \${xsens_height ${lidar.z - XS >= 0 ? '+' : '-'} ${r(Math.abs(lidar.z - XS))}}" rpy="0 0 0"/>\n` +
    `<!-- absolute z above ground: ${r(lidar.z)} m -->`;
  $('out').value = text;
  navigator.clipboard?.writeText(text).catch(() => {});
});
$('reset').addEventListener('click', () => {
  if (!confirm('Reset all cameras and the lidar to the URDF positions?')) return;
  cams = defaultCams(); nextId = 4;
  Object.assign(lidar, lidarDefault()); updateLidar(); syncLidarFields();
  rebuildAll();
  select(1);
});

// ───────────────────────── POV inset + render loop ─────────────────────────
const pov = new THREE.PerspectiveCamera(50, 1, 0.02, 80); // layer 0 only: no overlays, no camera bodies
let povRect = { x: 0, y: 0, w: 0, h: 0 };
function povLabel() {
  const c = sel();
  $('povLabel').textContent = `${c.name} view  ${c.hfov.toFixed(0)}°×${c.vfov.toFixed(0)}°`;
}

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  const c = sel();
  const aspect = Math.tan(c.hfov * D2R / 2) / Math.tan(c.vfov * D2R / 2);
  let pw = Math.min(380, w * 0.42), ph = pw / aspect;
  if (ph > h * 0.45) { ph = h * 0.45; pw = ph * aspect; }
  povRect = { x: w - pw - 10, y: 10, w: pw, h: ph }; // y measured from the bottom
  const f = $('povFrame').style;
  f.left = povRect.x + 'px'; f.bottom = povRect.y + 'px'; f.width = pw + 'px'; f.height = ph + 'px';
}
addEventListener('resize', resize);

function renderPov() {
  const c = sel();
  const o = objs.get(c.id);
  if (!o || !flags.pov) return;
  o.group.updateMatrixWorld(true);
  const pos = o.group.getWorldPosition(new THREE.Vector3());
  const fwd = new THREE.Vector3(1, 0, 0).transformDirection(o.group.matrixWorld);
  pov.up.set(0, 0, 1).transformDirection(o.group.matrixWorld);
  pov.position.copy(pos);
  pov.lookAt(pos.clone().add(fwd));
  pov.fov = c.vfov;
  pov.aspect = Math.tan(c.hfov * D2R / 2) / Math.tan(c.vfov * D2R / 2);
  pov.updateProjectionMatrix();
  const wasVisible = transform.visible;
  transform.visible = false;
  renderer.setScissorTest(true);
  renderer.setViewport(povRect.x, povRect.y, povRect.w, povRect.h);
  renderer.setScissor(povRect.x, povRect.y, povRect.w, povRect.h);
  renderer.render(scene, pov);
  renderer.setScissorTest(false);
  transform.visible = wasVisible;
}

let lastPovKey = '';
function loop() {
  requestAnimationFrame(loop);
  orbit.update();
  const c = sel();
  const key = `${c.hfov}|${c.vfov}`;
  if (key !== lastPovKey) { lastPovKey = key; resize(); }
  renderer.setViewport(0, 0, view.clientWidth, view.clientHeight);
  renderer.render(scene, camera);
  renderPov();
}

// ───────────────────────── boot ─────────────────────────
for (const c of cams) refresh(c);
applyFlags();
syncLidarFields();
renderList();
select(sel().id);
resize();
loop();
