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
const flags = { car: true, frustum: true, foot: true, fig: false, lane: false, pov: true };

function save() {
  try { localStorage.setItem(STORE_KEY, JSON.stringify({ cams, selectedId, nextId, flags })); } catch (e) { /* ignore */ }
}
function load() {
  try {
    const raw = localStorage.getItem(STORE_KEY);
    if (!raw) return;
    const d = JSON.parse(raw);
    if (Array.isArray(d.cams) && d.cams.length) { cams = d.cams; selectedId = d.selectedId; nextId = d.nextId || cams.length + 1; }
    if (d.flags) Object.assign(flags, d.flags);
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
orbit.target.set(1, 0, 0.4);
orbit.enableDamping = true;
orbit.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.75));
const sun = new THREE.DirectionalLight(0xffffff, 1.1);
sun.position.set(-4, -6, 10);
scene.add(sun);

// ground + grid (1 m cells)
const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), new THREE.MeshStandardMaterial({ color: 0x2b3a2f }));
ground.position.z = -0.002;
scene.add(ground);
const grid = new THREE.GridHelper(40, 40, 0x7a8b7f, 0x475549);
grid.rotation.x = Math.PI / 2;
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
{
  const v = new THREE.Mesh(new THREE.CylinderGeometry(VELODYNE.r, VELODYNE.r, VELODYNE.len, 24), new THREE.MeshStandardMaterial({ color: 0x222222 }));
  v.rotation.x = Math.PI / 2;
  v.position.set(...VELODYNE.c);
  carGroup.add(v);
}

// reference figures + lane lines
const figGroup = new THREE.Group();
for (const [x, y] of [[2, 0], [4, 0], [6, 0], [0.1, 1.5], [0.1, -1.5], [0.1, 3], [0.1, -3]]) {
  const p = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 1.7, 16), new THREE.MeshStandardMaterial({ color: 0xd9a066 }));
  p.rotation.x = Math.PI / 2;
  p.position.set(x, y, 0.85);
  figGroup.add(p);
}
scene.add(figGroup);
const laneGroup = new THREE.Group();
for (const y of [-1.5, 1.5]) {
  const l = new THREE.Mesh(new THREE.PlaneGeometry(14, 0.1), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  l.position.set(5, y, 0.002);
  laneGroup.add(l);
}
scene.add(laneGroup);

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
  const o = objs.get(sel().id);
  if (o) transform.attach(o.group);
}
function setMode(m) {
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
  const hit = ray.intersectObjects([...objs.values()].filter((o) => o.group.visible).map((o) => o.body), false)[0];
  if (hit) select(hit.object.userData.camId);
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
    d.className = 'cam' + (c.id === selectedId ? ' sel' : '');
    d.innerHTML = `<span class="dot" style="background:${c.color}"></span><span class="nm"></span><input type="checkbox" title="visible" ${c.visible ? 'checked' : ''}>`;
    d.querySelector('.nm').textContent = c.name;
    d.addEventListener('click', () => select(c.id));
    d.querySelector('input').addEventListener('click', (e) => e.stopPropagation());
    d.querySelector('input').addEventListener('change', (e) => { c.visible = e.target.checked; refresh(c); save(); });
    el.appendChild(d);
  }
}

function select(id) {
  selectedId = id;
  renderList();
  $('f-name').value = sel().name;
  lensSel.value = sel().lens in LENSES ? sel().lens : 'custom';
  syncFields();
  attachGizmo();
  updateInfo();
  save();
}

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
$('modeR').addEventListener('click', () => setMode('rotate'));
addEventListener('keydown', (e) => {
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) return;
  if (e.key === 't' || e.key === 'T') setMode('translate');
  if (e.key === 'r' || e.key === 'R') setMode('rotate');
});

// scene toggles
const applyFlags = () => {
  carGroup.visible = flags.car;
  figGroup.visible = flags.fig;
  laneGroup.visible = flags.lane;
  for (const c of cams) refresh(c);
  $('povFrame').style.display = flags.pov ? 'block' : 'none';
};
for (const k of Object.keys(flags)) {
  const el = $('t-' + k);
  el.checked = flags[k];
  el.addEventListener('change', () => { flags[k] = el.checked; applyFlags(); save(); });
}

// camera presets
const VIEWS = {
  persp: [[-3, -3.6, 2.6], [1, 0, 0.4]],
  top: [[0.99, 0, 9], [1, 0, 0]],
  side: [[0.3, -6, 0.9], [0.3, 0, 0.5]],
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
$('exp').addEventListener('click', () => download('avros-cameras.json', JSON.stringify({ format: 'avros-cam-sim', version: 1, cameras: cams }, null, 2)));
$('imp').addEventListener('click', () => $('file').click());
$('file').addEventListener('change', async (e) => {
  try {
    const d = JSON.parse(await e.target.files[0].text());
    if (!Array.isArray(d.cameras) || !d.cameras.length) throw new Error('no cameras');
    cams = d.cameras.map((c, i) => ({ roll: 0, range: 5, visible: true, lens: 'custom', color: COLORS[i % COLORS.length], ...c, id: i + 1 }));
    nextId = cams.length + 1;
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
    `<!-- absolute z above ground: ${r(c.z)} m -->`).join('\n\n');
  $('out').value = text;
  navigator.clipboard?.writeText(text).catch(() => {});
});
$('reset').addEventListener('click', () => {
  if (!confirm('Reset all cameras to the URDF positions?')) return;
  cams = defaultCams(); nextId = 4;
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
renderList();
select(sel().id);
resize();
loop();
