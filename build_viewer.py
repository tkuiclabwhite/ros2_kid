#!/usr/bin/env python3
"""
KID Robot 3D Viewer - Build Script
===================================
讀取 URDF + STL 檔，產生單一自含 HTML 檔案。
每次 URDF 或 STL 有更新時重跑一次即可。

用法：
    python build_viewer.py

前置：
    hurocup_interface/js/three.min.js  (需手動下載，r128)
    例：wget https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js
        mv three.min.js ~/ros2_kid/hurocup_interface/js/
"""

import base64
import json
import pathlib
import xml.etree.ElementTree as ET

HOME      = pathlib.Path.home()
URDF_PATH = HOME / "isaac_projects/KID_sim/KID/urdf/KID.urdf"
MESH_DIR  = HOME / "isaac_projects/KID_sim/KID/meshes"
OUTPUT    = HOME / "ros2_kid/hurocup_interface/RobotViewer.html"
THREEJS   = HOME / "ros2_kid/hurocup_interface/js/three.min.js"
CANNON_JS = HOME / "ros2_kid/hurocup_interface/js/cannon.min.js"
EE2_JS    = HOME / "ros2_kid/hurocup_interface/js/roslib/eventemitter2.min.js"
ROSLIB_JS = HOME / "ros2_kid/hurocup_interface/js/roslib/roslib.min.js"


# ── URDF 解析 ─────────────────────────────────────────────────────────────────

def _floats(s, default="0 0 0"):
    return [float(v) for v in (s or default).split()]

def parse_urdf(path):
    root = ET.parse(path).getroot()

    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        mesh_file, vo = None, {"xyz": [0,0,0], "rpy": [0,0,0]}
        visual = link.find("visual")
        if visual is not None:
            o = visual.find("origin")
            if o is not None:
                vo = {"xyz": _floats(o.get("xyz")), "rpy": _floats(o.get("rpy"))}
            m = visual.find("geometry/mesh")
            if m is not None:
                mesh_file = pathlib.Path(m.get("filename", "")).name
        links[name] = {"mesh": mesh_file, "vo": vo}

    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        p  = joint.find("parent")
        c  = joint.find("child")
        o  = joint.find("origin")
        ax = joint.find("axis")
        origin = {"xyz": [0,0,0], "rpy": [0,0,0]}
        if o is not None:
            origin = {"xyz": _floats(o.get("xyz")), "rpy": _floats(o.get("rpy"))}
        axis = _floats(ax.get("xyz") if ax is not None else None, "1 0 0")
        joints[name] = {
            "type":   joint.get("type"),
            "parent": p.get("link") if p is not None else None,
            "child":  c.get("link") if c is not None else None,
            "origin": origin,
            "axis":   axis,
        }

    return links, joints


# ── STL 載入（base64）────────────────────────────────────────────────────────

def load_meshes(mesh_dir, links):
    meshes, seen = {}, set()
    for data in links.values():
        fname = data["mesh"]
        if not fname or fname in seen:
            continue
        fpath = mesh_dir / fname
        if fpath.exists():
            raw = fpath.read_bytes()
            meshes[fname] = base64.b64encode(raw).decode()
            seen.add(fname)
            print(f"  ✓ {fname:35s} {len(raw)//1024:4d} KB")
        else:
            print(f"  ✗ {fname} 找不到")
    return meshes


# ── HTML 產生 ─────────────────────────────────────────────────────────────────

def read_js(path):
    if path.exists():
        return path.read_text(encoding="utf-8")
    return f"/* 找不到 {path.name} — 請確認檔案存在 */"

def generate_html(links, joints, meshes, three_js, cannon_js, ee2_js, roslib_js):
    links_json  = json.dumps(links,  separators=(",", ":"))
    joints_json = json.dumps(joints, separators=(",", ":"))
    meshes_json = json.dumps(meshes, separators=(",", ":"))

    return f"""<!DOCTYPE html>
<html lang="zh-TW">
<head>
<meta charset="utf-8">
<title>KID Robot Viewer</title>
<style>
*{{margin:0;padding:0;box-sizing:border-box}}
body{{background:#12121f;color:#ddd;font-family:monospace;overflow:hidden}}
canvas{{display:block}}
#hud{{position:absolute;top:12px;left:12px;background:rgba(0,0,0,.65);
      padding:8px 14px;border-radius:6px;font-size:12px;line-height:1.8;
      pointer-events:none}}
.dot-on{{color:#4caf50}}.dot-off{{color:#f44336}}.dot-warn{{color:#ff9800}}
#ctrl-bar{{position:absolute;top:12px;right:12px;display:flex;gap:6px;align-items:center}}
#ctrl-bar select,#ctrl-bar button{{
  height:28px;padding:0 10px;border:none;border-radius:4px;
  font-size:12px;cursor:pointer}}
#ctrl-bar select{{background:#1e2a3a;color:#ddd}}
#btn-connect{{background:#2255aa;color:#fff}}
#btn-connect:hover{{background:#3366cc}}
#btn-stand{{background:#1a4a2a;color:#adf}}
#btn-stand:hover{{background:#1f6035}}
</style>
</head>
<body>
<div id="hud">
  <div>KID Robot Viewer</div>
  <div id="ros-line"><span id="ros-dot" class="dot-off">●</span> <span id="ros-txt">未連線 rosbridge</span></div>
  <div id="js-line"><span id="js-dot" class="dot-off">●</span> <span id="js-txt">/joint_commands: 等待中</span></div>
  <div id="stand-line"><span id="stand-dot" class="dot-warn">●</span> <span id="stand-txt">stand.ini: 使用預設值</span></div>
</div>
<div id="ctrl-bar">
  <select style="height:25px; margin-right:5px;" id="addressSelect">
    <option value="192.168.1.58">192.168.1.58</option>
    <option value="localhost">localhost</option>
  </select>
  <button type="button" style="width:110px;height:25px;" onclick="enterAddress()">Enter Address</button>
  <button id="btn-stand"   onclick="reloadStandIni()">重讀 stand.ini</button>
  <button type="button" style="height:25px;background:#2a3a5a;color:#adf;border:none;border-radius:4px;padding:0 10px;font-size:12px;cursor:pointer;" onclick="openSGPPanel()">調整 STAND_GP</button>
  <button id="btn-phys" type="button" style="height:25px;background:#3a2a1a;color:#fca;border:none;border-radius:4px;padding:0 10px;font-size:12px;cursor:pointer;" onclick="togglePhysics()">物理 OFF</button>
</div>

<!-- STAND_GP 側邊欄 -->
<div id="sgp-panel" style="position:fixed;top:0;right:-260px;width:250px;height:100%;
  background:#1a2030;color:#ddd;border-left:1px solid #446;z-index:9999;
  overflow-y:auto;padding:12px 10px;font-size:12px;
  transition:right 0.2s ease;box-shadow:-4px 0 16px #0009;">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;">
    <b style="font-size:13px;">STAND_GP 理想刻度</b>
    <button onclick="closeSGPPanel()" style="background:none;border:none;color:#aaa;font-size:16px;cursor:pointer;line-height:1;">✕</button>
  </div>
  <div id="sgp-grid" style="display:flex;flex-direction:column;gap:5px;"></div>
  <div style="margin-top:12px;">
    <button onclick="applySGP()" style="width:100%;height:28px;background:#2255aa;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:12px;">套用</button>
  </div>
</div>

<!-- Three.js -->
<script>
{three_js}
</script>
<!-- Cannon.js physics -->
<script>
{cannon_js}
</script>
<!-- EventEmitter2 + roslib -->
<script>
{ee2_js}
</script>
<script>
{roslib_js}
</script>

<script>
// ════════════════════════════════════════════════════════════
//  嵌入資料（由 build_viewer.py 產生）
// ════════════════════════════════════════════════════════════
const URDF_LINKS  = {links_json};
const URDF_JOINTS = {joints_json};
const MESH_DATA   = {meshes_json};   // filename → base64 binary STL

// ════════════════════════════════════════════════════════════
//  換算常數（來源：sim_bridge.py）
// ════════════════════════════════════════════════════════════
const TPR    = 4096.0 / (2 * Math.PI);  // ticks per radian ≈ 651.9
const CENTER = 2048;

const JOINT_TO_MOTOR = {{
  LshouderJoint:1, LarmJoint:2, LforarmJoint:3, LhandJoint:4,
  RshouderJoint:5, RarmJoint:6, RforarmJoint:7, RhandJoint:8,
  WaistJoint:9,
  Lhip_jointJoint:10, LassJoint:11, LthightJoint:12,
  LcalvesJoint:13,   LankleJoint:14, Lsoles_of_feetJoint:15,
  Rhip_jointJoint:16, RassJoint:17, RthightJoint:18,
  RcalvesJoint:19,   RankleJoint:20, Rsoles_of_feetJoint:21,
  NeckJoint:22, HeadJoint:23,
}};

// 方向修正（+1 / -1）
const DIR = {{
  1:1, 2:-1, 3:1,  4:-1,
  5:1, 6:-1, 7:1,  8:-1,
  9:-1,
  10:1, 11:1,  12:-1, 13:1, 14:-1, 15:-1,
  16:1, 17:1,  18:-1, 19:-1, 20:1,  21:-1,
  22:1, 23:1,
}};

// 前臂 URDF 模型零點偏 180°（來源：sim_bridge.py OFFSET）
const JOINT_OFFSET = {{ 3: Math.PI, 7: Math.PI }};

// 理想站姿刻度（sim STAND_GP）
const STAND_GP = {{
  1:2048,2:1925,3:2048,4:2048,
  5:2048,6:2171,7:2048,8:2048,
  9:2048,
  10:2048,11:2048,12:1716,13:2684,14:2366,15:2048,
  16:2048,17:2048,18:2380,19:1412,20:1730,21:2048,
  22:2048,23:2048,
}};

// 真實站姿刻度（從 stand.ini 讀取；初始使用 STAND_GP）
let STAND_REAL = Object.assign({{}}, STAND_GP);

function ticksToRad(tick, motorId) {{
  const dir    = DIR[motorId]          ?? 1;
  const offset = JOINT_OFFSET[motorId] ?? 0;
  const standReal = STAND_REAL[motorId] ?? CENTER;
  const standGp   = STAND_GP[motorId]   ?? CENTER;
  // 修正機械裝配偏移，換算到 URDF 角度
  const corrected = tick - standReal + standGp;
  return dir * (corrected - CENTER) / TPR + offset;
}}

// ════════════════════════════════════════════════════════════
//  Three.js 初始化
// ════════════════════════════════════════════════════════════
const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12121f);

// 格線（Y-up Three.js 地板，XZ 平面）
const grid = new THREE.GridHelper(2, 20, 0x334466, 0x223355);
scene.add(grid);

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.005, 50);
camera.position.set(1.5, 0.8, 1.5);
camera.lookAt(0, 0.4, 0);

// 燈光
scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const sun = new THREE.DirectionalLight(0xffffff, 0.9);
sun.position.set(2, 5, 3);
sun.castShadow = true;
scene.add(sun);
const fillLight = new THREE.DirectionalLight(0x8899ff, 0.3);
fillLight.position.set(-2, 3, -2);
scene.add(fillLight);

// 簡易 Orbit 控制（左鍵=旋轉，右/中鍵=平移）
let drag = false, pan = false, lastX = 0, lastY = 0;
const sph = {{ theta: Math.PI, phi: 1.1, r: 2.0 }};  // 正前方視角
const tgt = new THREE.Vector3(0, 0.4, 0);
function updateCam() {{
  camera.position.set(
    tgt.x + sph.r * Math.sin(sph.phi) * Math.sin(sph.theta),
    tgt.y + sph.r * Math.cos(sph.phi),
    tgt.z + sph.r * Math.sin(sph.phi) * Math.cos(sph.theta)
  );
  camera.lookAt(tgt);
}}
updateCam();
const cvs = renderer.domElement;
cvs.addEventListener('contextmenu', e => e.preventDefault());
cvs.addEventListener('mousedown', e => {{
  if (e.button === 2 || e.button === 1) {{ pan=true; e.preventDefault(); }}
  else {{ drag=true; }}
  lastX=e.clientX; lastY=e.clientY;
}});
cvs.addEventListener('mouseup',   () => {{ drag=false; pan=false; }});
cvs.addEventListener('mouseleave',() => {{ drag=false; pan=false; }});
cvs.addEventListener('mousemove', e => {{
  if (!drag && !pan) return;
  const dx=e.clientX-lastX, dy=e.clientY-lastY;
  if (pan) {{
    const ps = sph.r * 0.001;
    const rx= Math.cos(sph.theta), rz=-Math.sin(sph.theta);
    const ux=-Math.sin(sph.theta)*Math.cos(sph.phi), uy=Math.sin(sph.phi), uz=-Math.cos(sph.theta)*Math.cos(sph.phi);
    tgt.x -= dx*ps*rx - dy*ps*ux;
    tgt.y += dy*ps*uy;
    tgt.z -= dx*ps*rz - dy*ps*uz;
  }} else {{
    sph.theta -= dx * 0.008;
    sph.phi = Math.max(0.05, Math.min(Math.PI-0.05, sph.phi - dy * 0.008));
  }}
  lastX=e.clientX; lastY=e.clientY; updateCam();
}});
cvs.addEventListener('wheel', e => {{
  sph.r = Math.max(0.2, Math.min(6, sph.r * (1 + e.deltaY * 0.001)));
  updateCam(); e.preventDefault();
}}, {{passive:false}});
window.addEventListener('resize', () => {{
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
}});

// ════════════════════════════════════════════════════════════
//  STL 解析器（binary STL → Float32Array）
// ════════════════════════════════════════════════════════════
function base64ToBuffer(b64) {{
  const bin = atob(b64), buf = new ArrayBuffer(bin.length);
  const v = new Uint8Array(buf);
  for (let i=0; i<bin.length; i++) v[i] = bin.charCodeAt(i);
  return buf;
}}
function parseBinarySTL(buf) {{
  const view = new DataView(buf);
  const n    = view.getUint32(80, true);
  const pos  = new Float32Array(n * 9);
  let off = 84;
  for (let i=0; i<n; i++) {{
    off += 12;                         // 跳過法向量
    for (let j=0; j<9; j++) {{ pos[i*9+j]=view.getFloat32(off,true); off+=4; }}
    off += 2;                          // 跳過 attribute
  }}
  return pos;
}}
function makeMesh(filename) {{
  const b64 = MESH_DATA[filename];
  if (!b64) return null;
  const pos = parseBinarySTL(base64ToBuffer(b64));
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.computeVertexNormals();
  const mat = new THREE.MeshPhongMaterial({{
    color: 0x3a7bd5, specular: 0x1a2a4a, shininess: 40, side: THREE.DoubleSide
  }});
  const m = new THREE.Mesh(geo, mat);
  m.castShadow = true;
  return m;
}}

// ════════════════════════════════════════════════════════════
//  建立 URDF scene graph
// ════════════════════════════════════════════════════════════
// linkObjects[linkName]   = 對應 link 的 Group（放 mesh）
// jointPivots[jointName]  = 旋轉用 Group（setRotationFromAxisAngle 改這裡）
const linkObjects  = {{}};
const jointPivots  = {{}};

function setRPY(obj, rpy) {{
  // URDF fixed-axis XYZ (extrinsic) = Three.js intrinsic 'ZYX'
  obj.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');
}}

function buildLink(linkName, parentGroup) {{
  const grp = new THREE.Group();
  grp.name  = linkName;
  linkObjects[linkName] = grp;
  parentGroup.add(grp);

  const ld = URDF_LINKS[linkName];
  if (ld?.mesh) {{
    const mesh = makeMesh(ld.mesh);
    if (mesh) {{
      mesh.position.set(...ld.vo.xyz);
      setRPY(mesh, ld.vo.rpy);
      mesh.userData.linkName = linkName;  // 標記所屬 link（物理初始化用）
      grp.add(mesh);
    }}
  }}

  // 子關節
  for (const [jname, jd] of Object.entries(URDF_JOINTS)) {{
    if (jd.parent !== linkName) continue;

    // 靜態偏移（URDF origin xyz + rpy）
    const offsetGrp = new THREE.Group();
    offsetGrp.name  = jname + '_offset';
    offsetGrp.position.set(...jd.origin.xyz);
    setRPY(offsetGrp, jd.origin.rpy);
    grp.add(offsetGrp);

    // 動態旋轉（關節角度）
    const pivot = new THREE.Group();
    pivot.name  = jname + '_pivot';
    offsetGrp.add(pivot);
    jointPivots[jname] = pivot;

    if (jd.child) buildLink(jd.child, pivot);
  }}
}}

// worldGroup：把 URDF Z-up 轉成 Three.js Y-up
// position.y 補正 base_link 到腳底的距離，讓腳底接地
const worldGroup = new THREE.Group();
worldGroup.rotation.x = -Math.PI / 2;
worldGroup.position.y = 0.45;
scene.add(worldGroup);

// 找根節點（不是任何 joint 的 child）
const childSet = new Set(Object.values(URDF_JOINTS).map(j=>j.child).filter(Boolean));
const rootLink = Object.keys(URDF_LINKS).find(n => !childSet.has(n));
buildLink(rootLink, worldGroup);
console.log('根節點:', rootLink, '關節數:', Object.keys(jointPivots).length);

// ── Waist 視覺修正：上半身以腰關節為圓心旋轉，下半身固定 ─────────────
// WaistJoint origin 在 base_link 坐標系: xyz=(-0.0334,-0.2678,-0.0872)
// 把上身子物件放入 upperBodyGroup（pivot 設在腰關節位置）
// 設定 WaistJoint 角度時只轉 upperBodyGroup，腿鏈不動。
const _waistOff = [-0.0333615582037253, -0.267800041292008, -0.0871849554864445];
const upperBodyGroup = new THREE.Group();
upperBodyGroup.name = 'upperBodyGroup';
upperBodyGroup.position.set(..._waistOff);
const _baseLinkGrp = linkObjects['base_link'];
_baseLinkGrp.children.slice().forEach(child => {{
  if (child.name !== 'WaistJoint_offset') {{
    upperBodyGroup.add(child);
    // 補回 upperBodyGroup 的 offset，讓各子物件世界位置不變
    child.position.x -= _waistOff[0];
    child.position.y -= _waistOff[1];
    child.position.z -= _waistOff[2];
  }}
}});
_baseLinkGrp.add(upperBodyGroup);

// ════════════════════════════════════════════════════════════
//  關節角度設定
// ════════════════════════════════════════════════════════════
function setJointAngle(jname, rad) {{
  if (jname === 'WaistJoint') {{
    // 軸 [0,1,0] 在 joint frame，joint rpy=(π/2,0,0) → 在 base_link 坐標系是 Z 軸
    upperBodyGroup.setRotationFromAxisAngle(new THREE.Vector3(0, 0, 1), rad);
    return;
  }}
  const pivot = jointPivots[jname];
  if (!pivot) return;
  const axis  = new THREE.Vector3(...URDF_JOINTS[jname].axis).normalize();
  pivot.setRotationFromAxisAngle(axis, rad);
}}

// velMap: motorId → Dynamixel profile velocity units (0.229 rpm/unit)
function applyTickMap(tickMap, velMap={{}}) {{
  for (const [jname, mid] of Object.entries(JOINT_TO_MOTOR)) {{
    if (tickMap[mid] != null) {{
      const rad = ticksToRad(tickMap[mid], mid);
      // 0.229 rpm × 2π/60 ≈ 0.02399 rad/s per unit; 0 = 最大速度（snap）
      const vel = (velMap[mid]??0) > 0 ? (velMap[mid]*0.02399) : 0;
      animTarget[jname]  = {{rad, vel}};
      physJTarget[jname] = rad;
    }}
  }}
}}

// ── STAND_GP 編輯面板 ────────────────────────────────────────────────────────
const SGP_LABELS = {{
  1:'LshouderJoint',2:'LarmJoint',3:'LforarmJoint',4:'LhandJoint',
  5:'RshouderJoint',6:'RarmJoint',7:'RforarmJoint',8:'RhandJoint',
  9:'WaistJoint',
  10:'Lhip_jointJoint',11:'LassJoint',12:'LthightJoint',
  13:'LcalvesJoint',14:'LankleJoint',15:'Lsoles_of_feetJoint',
  16:'Rhip_jointJoint',17:'RassJoint',18:'RthightJoint',
  19:'RcalvesJoint',20:'RankleJoint',21:'Rsoles_of_feetJoint',
  22:'NeckJoint',23:'HeadJoint',
}};
function openSGPPanel() {{
  const panel = document.getElementById('sgp-panel');
  if (panel.style.right === '0px') {{
    panel.style.right = '-260px';
    return;
  }}
  const grid = document.getElementById('sgp-grid');
  grid.innerHTML = '';
  const ids = Object.keys(STAND_GP).map(Number).sort((a,b)=>a-b);
  for (const mid of ids) {{
    const row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;gap:4px;';
    const lbl = SGP_LABELS[mid] || ('M'+mid);
    row.innerHTML = `<span style="width:22px;color:#7af;font-weight:bold;font-size:11px;flex-shrink:0;">${{mid}}</span>`+
                    `<span style="flex:1;color:#aaa;font-size:10px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;" title="${{lbl}}">${{lbl}}</span>`;
    const inp = document.createElement('input');
    inp.type='number'; inp.min=0; inp.max=4096; inp.step=1;
    inp.value = STAND_GP[mid];
    inp.dataset.mid = mid;
    inp.style.cssText='width:58px;height:20px;background:#111;color:#fff;border:1px solid #446;border-radius:3px;text-align:center;font-size:11px;flex-shrink:0;';
    row.appendChild(inp);
    grid.appendChild(row);
  }}
  panel.style.right = '0px';
}}
function closeSGPPanel() {{
  document.getElementById('sgp-panel').style.right = '-260px';
}}
function applySGP() {{
  const inputs = document.querySelectorAll('#sgp-grid input[data-mid]');
  inputs.forEach(inp => {{
    const mid = Number(inp.dataset.mid);
    const val = Math.max(0, Math.min(4096, parseInt(inp.value)||0));
    STAND_GP[mid] = val;
    inp.value = val;
  }});
  applyIdealStandPose();
  adjustGroundHeight();
  centerOnGrid();
}}

// 直接從 STAND_GP 換算理想站姿，不受 STAND_REAL 影響（立即 snap，不插值）
function applyIdealStandPose() {{
  for (const [jname, mid] of Object.entries(JOINT_TO_MOTOR)) {{
    const tick = STAND_GP[mid] ?? CENTER;
    const rad  = (DIR[mid] ?? 1) * (tick - CENTER) / TPR + (JOINT_OFFSET[mid] ?? 0);
    animCurrent[jname] = rad;
    animTarget[jname]  = {{rad, vel:0}};
    physJTarget[jname] = rad;
    setJointAngle(jname, rad);
  }}
}}

// 以右腳底 mesh 最低點接地（用 BBox 而非 group origin）
function adjustGroundHeight() {{
  worldGroup.updateWorldMatrix(true, true);
  const obj = linkObjects['Rsoles_of_feetLink'];
  if (obj) {{
    const box = new THREE.Box3().setFromObject(obj);
    if (isFinite(box.min.y)) worldGroup.position.y -= box.min.y - 0.03;
  }}
}}

// 以整體 BBox 水平中心對齊網格
function centerOnGrid() {{
  worldGroup.updateWorldMatrix(true, true);
  const box = new THREE.Box3().setFromObject(worldGroup);
  const center = new THREE.Vector3();
  box.getCenter(center);
  worldGroup.position.x -= center.x;
  worldGroup.position.z -= center.z;
}}

// 動畫 / 物理狀態（在 applyIdealStandPose 呼叫前宣告，避免 const TDZ 錯誤）
var animCurrent = {{}}, animTarget = {{}}, physJTarget = {{}};

// 預設顯示理想站姿（STAND_GP）
applyIdealStandPose();
adjustGroundHeight();
centerOnGrid();

// ════════════════════════════════════════════════════════════
//  絲滑動畫：每個關節獨立按速度插值到目標角度
// ════════════════════════════════════════════════════════════
const animClock = new THREE.Clock();  // animCurrent/animTarget/physJTarget 已在上方宣告

// ════════════════════════════════════════════════════════════
//  物理引擎 (cannon.js 0.6.2)
// ════════════════════════════════════════════════════════════
let physicsEnabled = false;
let physInited     = false;
let physWorld      = null;
const physBodies      = {{}};  // linkName → CANNON.Body
const physGroups      = {{}};  // linkName → THREE.Group (flat)
const physConstraints = {{}};  // jname    → CANNON.HingeConstraint
const physJointData   = {{}};  // jname    → {{q_rel_init, axisA_local, theta_init}}
// physJTarget 已在上方以 var 宣告

// pre-allocated temporaries for getActualDeltaAngle (avoid GC churn)
const _pq1 = new CANNON.Quaternion(), _pq2 = new CANNON.Quaternion(), _pq3 = new CANNON.Quaternion();
const _pv1 = new CANNON.Vec3();

// 每個 link 的質量（kg）
const LINK_MASS = {{
  'base_link':5,'WaistLink':2,
  'Lhip_jointLink':0.3,'LassLink':0.8,'LthightLink':1.5,'LcalvesLink':1,'LankleLink':0.8,'Lsoles_of_feetLink':0.4,
  'Rhip_jointLink':0.3,'RassLink':0.8,'RthightLink':1.5,'RcalvesLink':1,'RankleLink':0.8,'Rsoles_of_feetLink':0.4,
  'LshouderLink':0.5,'LarmLink':0.3,'LforarmLink':0.3,'LhandLink':0.15,
  'RshouderLink':0.5,'RarmLink':0.3,'RforarmLink':0.3,'RhandLink':0.15,
  'NeckLink':0.3,'HeadLink':0.5,
}};

function initPhysics() {{
  physInited = true;
  physWorld  = new CANNON.World();
  physWorld.gravity.set(0, -9.81, 0);
  physWorld.broadphase = new CANNON.SAPBroadphase(physWorld);
  physWorld.solver.iterations = 50;
  physWorld.solver.tolerance  = 0;
  physWorld.defaultContactMaterial.friction    = 0.5;
  physWorld.defaultContactMaterial.restitution = 0.02;

  // 地板（碰撞群組 1）
  const gnd = new CANNON.Body({{ mass: 0 }});
  gnd.addShape(new CANNON.Plane());
  gnd.quaternion.setFromAxisAngle(new CANNON.Vec3(1,0,0), -Math.PI/2);
  gnd.collisionFilterGroup = 1;
  gnd.collisionFilterMask  = 2;  // 只和機器人身體碰
  physWorld.addBody(gnd);

  worldGroup.updateWorldMatrix(true, true);

  // ── Pass 1：為每個 link 建立可視群組（只複製該 link 自己的 mesh）
  for (const [lname, lgrp] of Object.entries(linkObjects)) {{
    const lWP = new THREE.Vector3(), lWQ = new THREE.Quaternion();
    lgrp.getWorldPosition(lWP); lgrp.getWorldQuaternion(lWQ);

    const pg = new THREE.Group();
    pg.name = 'phys_'+lname;
    pg.position.copy(lWP); pg.quaternion.copy(lWQ);
    pg.visible = false;
    scene.add(pg);
    physGroups[lname] = pg;

    // 只取 userData.linkName === lname 的 mesh（避免把子 link 的 mesh 也帶入）
    lgrp.traverse(obj => {{
      if (!obj.isMesh || obj.userData.linkName !== lname) return;
      const mWP = new THREE.Vector3(), mWQ = new THREE.Quaternion();
      obj.getWorldPosition(mWP); obj.getWorldQuaternion(mWQ);
      const clone = obj.clone();
      clone.position.copy(mWP.clone().sub(lWP).applyQuaternion(lWQ.clone().invert()));
      clone.quaternion.copy(lWQ.clone().invert().multiply(mWQ));
      pg.add(clone);
    }});
  }}

  // ── Pass 2：用 physGroups 的 bbox 建立 Cannon body（碰撞群組 2，只和地板碰）
  for (const [lname, pg] of Object.entries(physGroups)) {{
    const mass = LINK_MASS[lname] ?? 0.5;
    const body = new CANNON.Body({{ mass }});
    body.position.set(pg.position.x, pg.position.y, pg.position.z);
    body.quaternion.set(pg.quaternion.x, pg.quaternion.y, pg.quaternion.z, pg.quaternion.w);
    body.linearDamping  = 0.6;
    body.angularDamping = 0.85;
    body.collisionFilterGroup = 2;
    body.collisionFilterMask  = 1;  // 只和地板碰，各 link 之間不碰撞（避免爆炸）

    pg.updateMatrixWorld(true);
    const bbox = new THREE.Box3().setFromObject(pg);
    if (!bbox.isEmpty()) {{
      const sz = new THREE.Vector3(); bbox.getSize(sz);
      const ct = new THREE.Vector3(); bbox.getCenter(ct);
      const off = ct.clone().sub(pg.position).applyQuaternion(pg.quaternion.clone().invert());
      body.addShape(
        new CANNON.Box(new CANNON.Vec3(Math.max(.015,sz.x/2), Math.max(.015,sz.y/2), Math.max(.015,sz.z/2))),
        new CANNON.Vec3(off.x, off.y, off.z)
      );
    }} else {{
      body.addShape(new CANNON.Sphere(0.02));
    }}
    physBodies[lname] = body;
    physWorld.addBody(body);
  }}

  // ── Pass 3：關節 HingeConstraint
  worldGroup.updateWorldMatrix(true, true);
  for (const [jname, jd] of Object.entries(URDF_JOINTS)) {{
    const bA = physBodies[jd.parent], bB = physBodies[jd.child];
    if (!bA || !bB || !jointPivots[jname]) continue;

    const pvWP = new THREE.Vector3();
    jointPivots[jname].getWorldPosition(pvWP);

    const axL = new THREE.Vector3(...jd.axis).normalize();
    const oWQ = new THREE.Quaternion();
    jointPivots[jname].parent.getWorldQuaternion(oWQ);
    const axW = axL.clone().applyQuaternion(oWQ);

    const pA  = new THREE.Vector3(bA.position.x, bA.position.y, bA.position.z);
    const pB  = new THREE.Vector3(bB.position.x, bB.position.y, bB.position.z);
    const qAi = new THREE.Quaternion(bA.quaternion.x, bA.quaternion.y, bA.quaternion.z, bA.quaternion.w).invert();
    const qBi = new THREE.Quaternion(bB.quaternion.x, bB.quaternion.y, bB.quaternion.z, bB.quaternion.w).invert();

    const pvA = pvWP.clone().sub(pA).applyQuaternion(qAi);
    const pvB = pvWP.clone().sub(pB).applyQuaternion(qBi);
    const axA = axW.clone().applyQuaternion(qAi);
    const axB = axW.clone().applyQuaternion(qBi);

    const c = new CANNON.HingeConstraint(bA, bB, {{
      pivotA: new CANNON.Vec3(pvA.x, pvA.y, pvA.z),
      axisA:  new CANNON.Vec3(axA.x, axA.y, axA.z),
      pivotB: new CANNON.Vec3(pvB.x, pvB.y, pvB.z),
      axisB:  new CANNON.Vec3(axB.x, axB.y, axB.z),
    }});
    c.enableMotor();
    c.setMotorMaxForce(5000);
    c.setMotorSpeed(0);
    physConstraints[jname] = c;
    physWorld.addConstraint(c);

    // Store initial relative quaternion and axis (for angle computation)
    bA.quaternion.conjugate(_pq1);
    _pq1.mult(bB.quaternion, _pq2);
    physJointData[jname] = {{
      q_rel_init: new CANNON.Quaternion(_pq2.x, _pq2.y, _pq2.z, _pq2.w),
      axisA_local: new CANNON.Vec3(axA.x, axA.y, axA.z),
      theta_init: animCurrent[jname] ?? 0,
    }};
    physJTarget[jname] = animCurrent[jname] ?? 0;
  }}
}}

function togglePhysics() {{
  physicsEnabled = !physicsEnabled;
  document.getElementById('btn-phys').textContent = physicsEnabled ? '物理 ON' : '物理 OFF';
  document.getElementById('btn-phys').style.background = physicsEnabled ? '#1a3a1a' : '#3a2a1a';
  document.getElementById('btn-phys').style.color       = physicsEnabled ? '#afa'   : '#fca';

  if (physicsEnabled) {{
    if (!physInited) initPhysics();
    // 同步目前 FK 角度到物理關節目標
    for (const jname of Object.keys(physJointData)) {{
      physJTarget[jname] = animTarget[jname]?.rad ?? animCurrent[jname] ?? 0;
    }}
    worldGroup.visible = false;
    for (const pg of Object.values(physGroups)) pg.visible = true;
  }} else {{
    worldGroup.visible = true;
    for (const pg of Object.values(physGroups)) pg.visible = false;
    // 把 Cannon body 目前的關節角度同步回 FK animCurrent
    applyIdealStandPose();
    adjustGroundHeight();
  }}
}}

// Compute actual hinge angle from Cannon body quaternions.
// Returns delta from the initial pose (not absolute angle).
function getActualDeltaAngle(jname) {{
  const data = physJointData[jname];
  if (!data) return 0;
  const jd = URDF_JOINTS[jname];
  const bA = physBodies[jd.parent], bB = physBodies[jd.child];
  if (!bA || !bB) return 0;
  // q_rel_curr = bA.quat.conj * bB.quat  (B relative to A)
  bA.quaternion.conjugate(_pq1);
  _pq1.mult(bB.quaternion, _pq2);
  // dq = q_rel_init.conj * q_rel_curr
  data.q_rel_init.conjugate(_pq1); // q_rel_init^-1 → _pq1
  _pq1.mult(_pq2, _pq3);           // dq → _pq3
  // Correct projection axis: q_rel_init^-1 * axisA_local
  // (dq = Rot(q_rel_init^-1 * axisA_local, θ), so axis of dq is the rotated axis)
  _pq1.vmult(data.axisA_local, _pv1); // axis_ref → _pv1
  return 2 * Math.atan2(_pq3.x*_pv1.x + _pq3.y*_pv1.y + _pq3.z*_pv1.z, _pq3.w);
}}

function physicsStep(dt) {{
  // Drive joint motors using actual Cannon angles
  for (const [jname, c] of Object.entries(physConstraints)) {{
    const data  = physJointData[jname];
    const delta = getActualDeltaAngle(jname);
    const actual = (data?.theta_init ?? 0) + delta;
    const tgt    = physJTarget[jname] ?? (data?.theta_init ?? 0);
    const err    = tgt - actual;
    const spd    = Math.sign(err) * Math.min(2.0, Math.abs(err) * 4.0);
    c.setMotorSpeed(spd);
  }}
  // 步進物理模擬 (smaller fixed dt → more stable stiff constraints)
  physWorld.step(1/120, dt, 8);
  // 同步 Three.js
  for (const [lname, body] of Object.entries(physBodies)) {{
    const pg = physGroups[lname];
    if (pg) {{
      pg.position.set(body.position.x, body.position.y, body.position.z);
      pg.quaternion.set(body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w);
    }}
  }}
}}

// ════════════════════════════════════════════════════════════
//  Render loop
// ════════════════════════════════════════════════════════════
(function animate() {{
  requestAnimationFrame(animate);
  const dt = Math.min(animClock.getDelta(), 0.05);

  if (!physicsEnabled) {{
    // FK 絲滑插值
    let dirty = false;
    for (const [jname, tgt] of Object.entries(animTarget)) {{
      const cur  = animCurrent[jname] ?? tgt.rad;
      const diff = tgt.rad - cur;
      if (Math.abs(diff) < 1e-5) continue;
      const next = tgt.vel > 0
        ? (Math.abs(diff) <= tgt.vel*dt ? tgt.rad : cur + Math.sign(diff)*tgt.vel*dt)
        : tgt.rad;  // vel=0 → snap
      if (next !== cur) {{ animCurrent[jname]=next; setJointAngle(jname,next); dirty=true; }}
    }}
    // adjustGroundHeight 只在初始化時跑，動畫中不跟著動
  }} else {{
    physicsStep(dt);
  }}

  renderer.render(scene, camera);
}})();

// ════════════════════════════════════════════════════════════
//  ROS 連線（IP 可從 select 切換）
// ════════════════════════════════════════════════════════════
var myAddress   = '192.168.1.58';
var connectFlag = false;
const ros = new ROSLIB.Ros({{ url: 'ws://' + myAddress + ':9090' }});
let jsSub = null;

function setHud(id, dotClass, txt) {{
  document.getElementById(id + '-dot').className = dotClass;
  document.getElementById(id + '-txt').textContent = txt;
}}

ros.on('connection', () => {{
  connectFlag = true;
  setHud('ros', 'dot-on', '已連線 ' + myAddress);
  loadStandIni();   // 讀取站姿偏移（失敗時自動 fallback 到 STAND_GP）
  startJointSub();
}});
ros.on('error', () => {{
  connectFlag = false;
  setHud('ros', 'dot-off', '連線錯誤 (' + myAddress + ')');
}});
ros.on('close', () => {{
  connectFlag = false;
  setHud('ros', 'dot-off', '已斷線');
  setHud('js',  'dot-off', '/joint_commands: 已斷線');
}});

function enterAddress() {{
  if (connectFlag) {{ ros.close(); connectFlag = false; }}
  myAddress = document.getElementById('addressSelect').value;
  setHud('ros', 'dot-warn', '連線中 ' + myAddress + '...');
  const state = ros.socket ? ros.socket.readyState : WebSocket.CLOSED;
  if (state === WebSocket.OPEN || state === WebSocket.CONNECTING || state === WebSocket.CLOSING) {{
    ros.close();
    ros.once('close', () => ros.connect('ws://' + myAddress + ':9090'));
  }} else {{
    ros.connect('ws://' + myAddress + ':9090');
  }}
}}

// ── 讀取 stand.ini ────────────────────────────────────────────────────────────
function loadStandIni() {{
  const svc = new ROSLIB.Service({{
    ros, name: '/package/InterfaceReadSaveMotion',
    serviceType: 'tku_msgs/ReadMotion'
  }});
  svc.callService(
    new ROSLIB.ServiceRequest({{ read: true, name: 'stand', readstate: 1 }}),
    function(data) {{
      // absolutedata 依序包含 motionstate=3(位置) 和 motionstate=4(速度) 的資料
      // 找到第一個 motionstate=3，取其 21 個值作為真實站姿刻度
      let absIdx = 0;
      let found  = false;
      for (let i = 0; i < data.vectorcnt; i++) {{
        const ms = data.motionstate[i];
        if (ms === 3) {{
          for (let j = 1; j <= 21; j++) {{
            STAND_REAL[j] = data.absolutedata[absIdx + (j - 1)];
          }}
          found = true;
          break;
        }} else if (ms === 4) {{
          absIdx += 21;
        }}
      }}
      if (found) {{
        setHud('stand', 'dot-on', 'stand.ini: 已載入 (motionstate=3)');
        applyIdealStandPose();
        adjustGroundHeight();
      }} else {{
        setHud('stand', 'dot-warn', 'stand.ini: 找不到 motionstate=3，使用預設');
      }}
    }},
    () => setHud('stand', 'dot-warn', 'stand.ini: 服務呼叫失敗，使用預設值')
  );
}}

function reloadStandIni() {{
  if (ros.isConnected) {{
    setHud('stand', 'dot-warn', 'stand.ini: 重讀中...');
    loadStandIni();
  }} else {{
    setHud('stand', 'dot-off', 'stand.ini: 未連線');
  }}
}}

// ── 訂閱 /joint_commands ─────────────────────────────────────────────────────
function startJointSub() {{
  if (jsSub) {{ jsSub.unsubscribe(); }}
  jsSub = new ROSLIB.Topic({{
    ros, name: '/joint_commands', messageType: 'sensor_msgs/JointState'
  }});
  let lastTime = 0;
  jsSub.subscribe(msg => {{
    const now = Date.now();
    if (now - lastTime < 50) return;  // 限流 20Hz
    lastTime = now;

    const ticks = {{}}, vels = {{}};
    for (let i = 0; i < msg.name.length; i++) {{
      const id = parseInt(msg.name[i]);
      if (!isNaN(id)) {{
        ticks[id] = Math.round(msg.position[i]);
        if (msg.velocity && msg.velocity[i]) vels[id] = msg.velocity[i];
      }}
    }}
    applyTickMap(ticks, vels);
    setHud('js', 'dot-on', '/joint_commands: 接收中');
  }});
}}

// 頁面載入時立刻顯示連線中（Ros 建構子已自動連線）
setHud('ros', 'dot-warn', '連線中 ' + myAddress + '...');
</script>
</body>
</html>"""


# ── 主程式 ────────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("KID Robot Viewer — Build Script")
    print("=" * 50)

    if not URDF_PATH.exists():
        print(f"錯誤：找不到 URDF：{URDF_PATH}")
        return
    if not THREEJS.exists():
        print(f"\n⚠  找不到 Three.js：{THREEJS}")
        print("   請下載 r128：")
        print("   wget https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js")
        print(f"   mv three.min.js {THREEJS}\n")
        print("   或在機器人有網路時先下載好再執行此腳本。\n")
        return

    print("\n[1/3] 解析 URDF...")
    links, joints = parse_urdf(URDF_PATH)
    print(f"      {len(links)} links, {len(joints)} joints")


    print("\n[2/3] 載入 STL meshes...")
    meshes = load_meshes(MESH_DIR, links)
    total_kb = sum(len(b) * 3 // 4 // 1024 for b in meshes.values())
    print(f"      共 {len(meshes)} 個 mesh，原始約 {total_kb} KB")

    print("\n[3/3] 產生 HTML...")
    three_js  = read_js(THREEJS)
    cannon_js = read_js(CANNON_JS)
    ee2_js    = read_js(EE2_JS)
    roslib_js = read_js(ROSLIB_JS)

    html = generate_html(links, joints, meshes, three_js, cannon_js, ee2_js, roslib_js)
    OUTPUT.write_text(html, encoding="utf-8")

    size_mb = OUTPUT.stat().st_size / 1024 / 1024
    print(f"\n✓ 輸出：{OUTPUT}")
    print(f"  檔案大小：{size_mb:.1f} MB")
    print(f"\n開啟方式：用瀏覽器直接開啟 hurocup_interface/RobotViewer.html")
    print("需要 rosbridge 在線才能顯示即時姿態（ws://192.168.1.58:9090）")


if __name__ == "__main__":
    main()
