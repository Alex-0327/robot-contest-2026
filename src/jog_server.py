"""网页点动面板 —— 浏览器按钮控制机械臂在 x/y/z 方向微动。

架构：单文件 HTTP 服务（标准库 http.server，零新依赖），跑在连着机械臂的
Linux 机上，绑定 127.0.0.1，经 SSH 端口转发访问，不暴露局域网：

    ssh -L 8080:127.0.0.1:8080 user@linux机
    python src/jog_server.py            # 真机（连接机械臂，首次点击时建立连接）
    python src/jog_server.py --mock     # 无硬件调试 UI（Mac 上可跑）

然后浏览器打开 http://localhost:8080

比赛当天快速标定流程：页面顶部选已有航点 →「去该航点」（原地抬高→平移→降下，
全程 move_ik 肘朝下构型全局劣后）→ 点动微调 →「覆盖保存到该航点」（新格式，
pose+joints 同时落盘）。

注意：控制器同时只应由一个程序下发运动指令，且 CNDE 状态流只支持单连接。
运行本面板前必须退出 teach_daemon（echo quit > /tmp/teach_cmd），反之亦然。
急停后必须点「复位+自动模式」才能继续运动（StopMotion 会锁存停止态）。
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_STEP_MM = 20.0   # 单次微动步长硬上限
DEFAULT_VEL = 5.0    # 微动速度 mm/s，宁慢勿快
AXES = ("x", "y", "z", "rx", "ry", "rz")

INDEX_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>机械臂点动面板</title>
<style>
  body { font-family: -apple-system, sans-serif; max-width: 560px; margin: 24px auto; padding: 0 12px; }
  h1 { font-size: 20px; }
  #pose { font-family: monospace; background: #f4f4f4; padding: 8px 12px; border-radius: 6px; white-space: pre; }
  .row { margin: 10px 0; }
  button { font-size: 18px; padding: 12px 20px; margin: 4px; border: 1px solid #ccc;
           border-radius: 8px; background: #fff; cursor: pointer; min-width: 72px; }
  button:active { background: #dde; }
  button:disabled { opacity: .4; cursor: not-allowed; }
  #estop { background: #d33; color: #fff; border-color: #d33; font-weight: bold; }
  #msg { min-height: 1.4em; color: #555; }
  #msg.err { color: #d33; }
  label { margin-right: 8px; }
</style>
</head>
<body>
<h1>机械臂点动面板</h1>
<div id="pose">位姿加载中…</div>
<div class="row">
  <label>航点
    <select id="wpsel" style="font-size:16px;padding:6px;max-width:240px"></select>
  </label>
  <button onclick="loadWps()">刷新航点列表</button>
  <button onclick="gotoWp()">去该航点</button>
  <label style="font-size:14px"><input type="checkbox" id="goexact">降到底(默认停上方15mm)</label>
  <button onclick="saveOver()">覆盖保存到该航点</button>
</div>
<div class="row">
  <label>步长
    <select id="step">
      <option value="1">1 mm</option>
      <option value="5" selected>5 mm</option>
      <option value="10">10 mm</option>
    </select>
  </label>
  <span style="color:#999;font-size:13px">键盘：←→=X ±↑↓=Y，PgUp/PgDn=Z</span>
</div>
<div class="row">
  <button onclick="move('x',-1)">X−</button>
  <button onclick="move('x', 1)">X+</button>
</div>
<div class="row">
  <button onclick="move('y',-1)">Y−</button>
  <button onclick="move('y', 1)">Y+</button>
</div>
<div class="row">
  <button onclick="move('z',-1)">Z−</button>
  <button onclick="move('z', 1)">Z+</button>
</div>
<div class="row">
  <label>角度步长
    <select id="rstep">
      <option value="1">1°</option>
      <option value="3" selected>3°</option>
      <option value="5">5°</option>
    </select>
  </label>
</div>
<div class="row">
  <button onclick="rmove('rx',-1)">RX−</button>
  <button onclick="rmove('rx', 1)">RX+</button>
  <button onclick="rmove('ry',-1)">RY−</button>
  <button onclick="rmove('ry', 1)">RY+</button>
  <button onclick="rmove('rz',-1)">RZ−</button>
  <button onclick="rmove('rz', 1)">RZ+</button>
</div>
<div class="row">
  <input id="wpname" placeholder="航点名，如 paper1_tl" style="font-size:16px;padding:8px;width:220px">
  <button onclick="savewp()">存航点</button>
</div>
<div class="row">
  <button onclick="resetCtl()">复位+自动模式</button>
  <button onclick="gohome()">回 home</button>
  <button id="estop" onclick="estop()">急停</button>
</div>
<div class="row">灵巧手：
  <button onclick="handCmd('open')">伸直</button>
  <button onclick="handCmd('pre')">预抓握</button>
  <button onclick="handCmd('grasp')">抓握</button>
</div>
<div id="msg"></div>
<script>
let busy = false;
const msgEl = document.getElementById('msg');
const poseEl = document.getElementById('pose');

function setMsg(t, isErr) { msgEl.textContent = t; msgEl.className = isErr ? 'err' : ''; }
function showPose(p) {
  poseEl.textContent = 'x=' + p[0].toFixed(1) + '  y=' + p[1].toFixed(1) +
    '  z=' + p[2].toFixed(1) + '\\nrx=' + p[3].toFixed(1) + '  ry=' + p[4].toFixed(1) +
    '  rz=' + p[5].toFixed(1);
}
function setButtons(disabled) {
  document.querySelectorAll('button').forEach(b => { if (b.id !== 'estop') b.disabled = disabled; });
}

async function move(axis, dir) {
  if (busy) return;
  busy = true; setButtons(true); setMsg('运动中…');
  try {
    const step = parseFloat(document.getElementById('step').value);
    const r = await fetch('/api/move', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({axis, dir, step})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose); setMsg('完成');
  } catch (e) { setMsg('出错：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function rmove(axis, dir) {
  if (busy) return;
  busy = true; setButtons(true); setMsg('旋转中…');
  try {
    const step = parseFloat(document.getElementById('rstep').value);
    const r = await fetch('/api/move', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({axis, dir, step})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose); setMsg('完成');
  } catch (e) { setMsg('出错：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function savewp() {
  const name = document.getElementById('wpname').value.trim();
  if (!name) { setMsg('先输入航点名', true); return; }
  try {
    const r = await fetch('/api/save', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    setMsg('已存航点 [' + name + ']');
    loadWps();
  } catch (e) { setMsg('保存失败：' + e.message, true); }
}

async function loadWps() {
  try {
    const r = await fetch('/api/waypoints');
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    const sel = document.getElementById('wpsel');
    const prev = sel.value;
    sel.innerHTML = '';
    j.waypoints.forEach(w => {
      const o = document.createElement('option');
      o.value = w.name;
      o.textContent = w.name + (w.has_joints ? '' : '（旧格式）');
      sel.appendChild(o);
    });
    if (prev) sel.value = prev;
  } catch (e) { setMsg('航点列表加载失败：' + e.message, true); }
}

async function gotoWp() {
  const name = document.getElementById('wpsel').value;
  if (!name) { setMsg('先选一个航点', true); return; }
  if (busy) return;
  const exact = document.getElementById('goexact').checked;
  busy = true; setButtons(true); setMsg('正在去航点 [' + name + ']（抬高→平移→降下）…');
  try {
    const r = await fetch('/api/goto_waypoint', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name, exact})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose);
    setMsg(exact ? '已到航点 [' + name + '] 原位置（降到底）'
                 : '已到航点 [' + name + '] 上方 15mm，按 Z− 压到纸面后微调');
  } catch (e) { setMsg('去航点失败：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function saveOver() {
  const name = document.getElementById('wpsel').value;
  if (!name) { setMsg('先选一个航点', true); return; }
  if (!confirm('确认用当前位姿覆盖航点 [' + name + ']？旧值将被替换（含关节角）')) return;
  try {
    const r = await fetch('/api/save', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    setMsg('已覆盖保存航点 [' + name + ']');
    loadWps();
  } catch (e) { setMsg('覆盖保存失败：' + e.message, true); }
}

async function resetCtl() {
  try {
    const r = await fetch('/api/reset', {method: 'POST'});
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    setMsg('已复位，自动模式');
  } catch (e) { setMsg('复位失败：' + e.message, true); }
}

async function gohome() {
  // 关节空间回 home：不走 IK，任意怪姿态都能回来（奇异/限位恢复用）
  if (busy) return;
  busy = true; setButtons(true); setMsg('正在回 home（关节空间）…');
  try {
    const r = await fetch('/api/home', {method: 'POST'});
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose); setMsg('已回 home');
  } catch (e) { setMsg('回 home 失败：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function estop() {
  try { await fetch('/api/stop', {method: 'POST'}); setMsg('已发送急停（再动之前先点「复位」）', true); }
  catch (e) { setMsg('急停请求失败：' + e.message, true); }
}

async function handCmd(action) {
  const names = {open: '伸直', pre: '预抓握', grasp: '抓握'};
  setMsg('灵巧手：' + names[action] + '…');
  try {
    const r = await fetch('/api/hand', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    setMsg('灵巧手已' + names[action]);
  } catch (e) { setMsg('手部动作失败：' + e.message, true); }
}

async function refreshPose() {
  if (busy) return;
  try {
    const r = await fetch('/api/pose');
    const j = await r.json();
    if (r.ok) showPose(j.pose);
  } catch (e) { /* 静默，下轮再试 */ }
}

document.addEventListener('keydown', e => {
  const map = {ArrowLeft: ['x', -1], ArrowRight: ['x', 1],
               ArrowDown: ['y', -1], ArrowUp: ['y', 1],
               PageDown: ['z', -1], PageUp: ['z', 1]};
  const m = map[e.key];
  if (m) { e.preventDefault(); move(m[0], m[1]); }
});

refreshPose();
setInterval(refreshPose, 2000);
loadWps();
</script>
</body>
</html>
"""


class MockArm:
    """无硬件调试用的假机械臂：内存里维护一个位姿。"""

    def __init__(self):
        self.pose = [400.0, 0.0, 300.0, 180.0, 0.0, 0.0]

    def enable(self, on=True):
        pass

    def get_pose(self):
        return list(self.pose)

    def move_rel(self, dx=0.0, dy=0.0, dz=0.0, drx=0.0, dry=0.0, drz=0.0,
                 vel=DEFAULT_VEL, blocking=True):
        time.sleep(0.3)  # 模拟运动耗时
        self.pose[0] += dx
        self.pose[1] += dy
        self.pose[2] += dz
        self.pose[3] += drx
        self.pose[4] += dry
        self.pose[5] += drz

    def stop(self):
        pass

    def move_rel_ik(self, **kwargs):
        self.move_rel(**kwargs)

    def reset_errors(self):
        return 0

    def set_mode(self, manual=False):
        pass

    def save_waypoint(self, name):
        print(f"[mock] 存航点 {name} = {self.pose}")

    def list_waypoints(self):
        return {
            "home": {"pose": [400.0, 0.0, 300.0, 180.0, 0.0, 0.0], "joints": None},
            "eye_dot": {"pose": [300.0, -100.0, 200.0, 180.0, 0.0, 0.0],
                        "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        }

    def get_waypoint(self, name):
        wps = self.list_waypoints()
        if name not in wps:
            raise KeyError(f"航点 [{name}] 不存在，已有: {list(wps)}")
        return list(wps[name]["pose"])

    def move_ik(self, pose, vel=10.0, blocking=True):
        time.sleep(0.3)
        self.pose = list(pose)

    def move_to(self, pose, vel=10.0, blocking=True):
        self.move_ik(pose, vel, blocking)

    def get_waypoint_joints(self, name):
        return None


_arm = None
_arm_lock = threading.Lock()    # 建立连接互斥
_move_lock = threading.Lock()   # 运动互斥：同一时刻只允许一个微动在执行
_mock = False
_hand = None


def get_hand():
    """懒加载连接灵巧手（CAN）。mock 模式返回 None，调用方跳过手部动作。"""
    global _hand
    if _mock:
        return None
    if _hand is None:
        with _arm_lock:
            if _hand is None:
                from hand_api import Hand
                _hand = Hand()
    return _hand


def get_arm():
    """懒加载连接机械臂，首次请求时才建立 RPC 连接。"""
    global _arm
    if _arm is None:
        with _arm_lock:
            if _arm is None:
                if _mock:
                    _arm = MockArm()
                else:
                    from arm_api import Arm
                    _arm = Arm()
                    _arm.reset_errors()
                    time.sleep(1)
                    _arm.set_mode(False)  # 自动模式才能接受程序运动
                    _arm.enable(True)
    return _arm


class Handler(BaseHTTPRequestHandler):
    server_version = "ArmJog/0.1"

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, html):
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass  # 静默访问日志，错误会显式打印

    def do_GET(self):
        if self.path == "/":
            self._send_html(INDEX_HTML)
        elif self.path == "/api/pose":
            try:
                self._send_json({"pose": get_arm().get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/waypoints":
            try:
                wps = get_arm().list_waypoints()
                self._send_json({"waypoints": [
                    {"name": n, "pose": w["pose"], "has_joints": w["joints"] is not None}
                    for n, w in wps.items()]})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        else:
            self._send_json({"error": "not found"}, 404)

    def do_POST(self):
        if self.path == "/api/move":
            self._handle_move()
        elif self.path == "/api/stop":
            try:
                get_arm().stop()
                self._send_json({"ok": True})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/reset":
            # 急停/WebApp停止后控制器锁存停止态，必须先复位再切自动才能继续
            try:
                arm = get_arm()
                arm.reset_errors()
                time.sleep(1)
                arm.set_mode(False)
                arm.enable(True)
                self._send_json({"ok": True, "pose": arm.get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/goto":
            # 推算点粗定位：原地抬高→平移→降到悬停高度（纸面附近最后几毫米由人微调）
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                gx, gy = float(req["x"]), float(req["y"])
                hover = float(req.get("z", 195.0))
                travel = float(req.get("travel_z", 300.0))
                rz = req.get("rz")  # 可选：绕笔轴换手位（远角不可达时转270°）
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            if not _move_lock.acquire(blocking=False):
                self._send_json({"error": "上一个运动尚未完成，请稍候"}, 409)
                return
            try:
                arm = get_arm()
                cur = arm.get_pose()
                rpy = [cur[3], cur[4], float(rz) if rz is not None else cur[5]]
                high = cur[:]; high[2] = travel
                arm.move_ik(high, vel=10)
                over = [gx, gy, travel] + rpy
                arm.move_ik(over, vel=10)
                arm.move_ik([gx, gy, hover] + rpy, vel=5)
                self._send_json({"ok": True, "pose": arm.get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            finally:
                _move_lock.release()
        elif self.path == "/api/goto_waypoint":
            # 现场标定主流程第一步：去已保存航点。
            # 三段式：原地抬高 → 平移 → 降下；全程 move_ik（肘朝下构型全局劣后，防翻肘）。
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                name = str(req["name"]).strip()
                if not name:
                    raise ValueError("航点名不能为空")
                travel = float(req.get("travel_z", 300.0))
                exact = bool(req.get("exact", False))
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            if not _move_lock.acquire(blocking=False):
                self._send_json({"error": "上一个运动尚未完成，请稍候"}, 409)
                return
            try:
                arm = get_arm()
                wp = arm.get_waypoint(name)
                cur = arm.get_pose()
                safe_z = max(travel, cur[2], wp[2] + 50.0)  # 转场高度不低于目标上方 50mm
                high = cur[:]; high[2] = safe_z
                arm.move_ik(high, vel=10)
                arm.move_ik([wp[0], wp[1], safe_z, wp[3], wp[4], wp[5]], vel=10)
                # 默认只降到航点上方 15mm 悬停（防姿态修正后原 z 低于纸面戳笔），
                # 勾选「降到底」才回保存的精确位置（标定复验用）
                final_z = wp[2] if exact else wp[2] + 15.0
                arm.move_ik([wp[0], wp[1], final_z, wp[3], wp[4], wp[5]], vel=5)
                self._send_json({"ok": True, "pose": arm.get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            finally:
                _move_lock.release()
        elif self.path == "/api/home":
            # 关节空间回 home：不依赖 IK（当前姿态怪异时 move_ik 会在第一段就失败）
            if not _move_lock.acquire(blocking=False):
                self._send_json({"error": "上一个运动尚未完成，请稍候"}, 409)
                return
            try:
                arm = get_arm()
                hj = arm.get_waypoint_joints("home")
                if hj is None:
                    raise RuntimeError("home 航点没有关节角（旧格式），先示教重存 home")
                arm.move_joints(hj, vel=10)
                self._send_json({"ok": True, "pose": arm.get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            finally:
                _move_lock.release()
        elif self.path == "/api/hand":
            # 灵巧手三态：伸直 / 预抓握 / 抓握（握笔参数与 write_engine 同源）
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                action = str(req["action"]).strip()
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            try:
                hand = get_hand()
                if hand is not None:
                    if action == "open":
                        hand.open()
                    elif action == "pre":
                        hand.pre_grasp_pen()
                    elif action == "grasp":
                        hand.grasp_pen()
                    else:
                        raise ValueError(f"未知手部动作 {action!r}（可选 open/pre/grasp）")
                elif action not in ("open", "pre", "grasp"):
                    raise ValueError(f"未知手部动作 {action!r}（可选 open/pre/grasp）")
                self._send_json({"ok": True})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/save":
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                name = str(req["name"]).strip()
                if not name:
                    raise ValueError("航点名不能为空")
                get_arm().save_waypoint(name)
                self._send_json({"ok": True})
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        else:
            self._send_json({"error": "not found"}, 404)

    def _handle_move(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(length) or b"{}")
            axis = req["axis"]
            direction = 1 if float(req["dir"]) > 0 else -1
            step = min(abs(float(req["step"])), MAX_STEP_MM)
            if axis not in AXES:
                raise ValueError(f"未知轴 {axis!r}")
        except (KeyError, ValueError, TypeError) as e:
            self._send_json({"error": f"请求参数错误: {e}"}, 400)
            return

        if not _move_lock.acquire(blocking=False):
            self._send_json({"error": "上一个运动尚未完成，请稍候"}, 409)
            return
        try:
            arm = get_arm()
            kwargs = {"d" + axis: direction * step, "vel": DEFAULT_VEL}
            arm.move_rel_ik(**kwargs)  # IK+关节空间，避免 MoveL 奇异点/超限
            self._send_json({"ok": True, "pose": arm.get_pose()})
        except Exception as e:
            self._send_json({"error": str(e)}, 500)
        finally:
            _move_lock.release()


def main():
    global _mock
    parser = argparse.ArgumentParser(description="机械臂网页点动面板")
    parser.add_argument("--mock", action="store_true", help="无硬件调试模式")
    parser.add_argument("--host", default="127.0.0.1", help="绑定地址（默认仅本机，配合 SSH 转发）")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    _mock = args.mock

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    mode = "MOCK（无硬件）" if _mock else "真机"
    print(f"点动面板已启动 [{mode}] http://{args.host}:{args.port}")
    print("SSH 转发：ssh -L {0}:127.0.0.1:{0} user@linux机".format(args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
