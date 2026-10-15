"""赛场调试面板 —— 浏览器访问，从上往下点一遍就是完整调试流程。

架构：单文件 HTTP 服务（标准库 http.server，零新依赖），跑在连着机械臂的
Linux 机上，绑定 127.0.0.1，经 SSH 端口转发访问，不暴露局域网：

    ssh -L 8080:127.0.0.1:8080 user@linux机
    python src/jog_server.py            # 真机（连接机械臂，首次点击时建立连接）
    python src/jog_server.py --mock     # 无硬件调试 UI（Mac 上可跑）

然后浏览器打开 http://localhost:8080

页面 6 个编号区块（操作员不需要参加过调试）：
    第 1 步 通路测试：手三态 + 臂抬/降 5cm
    第 2 步 取笔/还笔：自动编排（home→高位去预抓握点→预抓握→降抓握位→抓握→抬回）
    第 3 步 航点标定向导：有序点位清单（home → 笔/墨/点睛 9 点 → 点睛跑通后再纸面 6 点），
             保存前连读 3 次 + 写盘后回读核对
    第 4 步 画龙点睛：全流程（取笔→蘸墨+抖墨→点睛→还笔）/ 仅点睛
    第 5 步 写字：试写一个字 + 检查字库 + 预览 + 开始书写
    第 6 步 精细微调：jog 点动（第 3 步对不准时用）

10-14 官方布局更新：笔筒与砚台相邻（间隔约 17.5cm），都在远离基座一侧，
且筒内竖着的毛笔顶部高过砚台悬停位（ink_pre）。纪律：笔筒区只从正上方垂直
进出——任何横向进入/离开笔筒区之前，先抬到高过筒内笔顶的高度（面板内建
PEN_TRAVEL_Z=340mm 下限）；低位横向挪向笔筒 = 必撞筒内笔。

注意：控制器同时只应由一个程序下发运动指令，且 CNDE 状态流只支持单连接。
运行本面板前必须退出 teach_daemon（echo quit > /tmp/teach_cmd），反之亦然。
急停后必须点「复位+自动模式」才能继续运动（StopMotion 会锁存停止态）。
任何编排/写字任务运行期间，jog/航点/其它流程全部被锁（服务端 409），
「急停」和「停止当前任务」永远可用。
"""
import argparse
import contextlib
import json
import math
import os
import sys
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
  #wtext { width: 100%; font-size: 16px; padding: 8px; box-sizing: border-box; }
  #wresult { font-size: 14px; line-height: 1.9; }
  #wlog { background: #111; color: #7f7; padding: 8px 12px; border-radius: 6px;
          height: 180px; overflow: auto; font-size: 12px; display: none; white-space: pre-wrap; }
  #wstop { background: #e80; color: #fff; border-color: #e80; font-weight: bold; }
  h2 { font-size: 17px; margin: 6px 0 2px; }
  .tip { font-size: 13px; color: #666; background: #f7f7f0; border-left: 3px solid #cc6;
         padding: 6px 10px; margin: 6px 0; line-height: 1.6; }
  .hint { font-size: 13px; color: #d33; }
  .crow { display: flex; align-items: center; gap: 6px; margin: 4px 0; font-size: 14px; }
  .crow .cname { width: 250px; }
  .crow .cst { width: 110px; font-size: 13px; }
  .crow .cdev { color: #282; font-size: 13px; }
  .crow button { font-size: 14px; padding: 6px 10px; min-width: 0; }
  .ok { color: #282; } .miss { color: #d33; font-weight: bold; }
</style>
</head>
<body>
<h1>赛场调试面板</h1>
<div id="pose">位姿加载中…</div>
<div class="row">
  <button id="estop" onclick="estop()">急停</button>
  <button id="wstop" onclick="wstop()" disabled>停止当前任务</button>
  <button id="btnreset" onclick="resetCtl()">复位+自动模式</button>
  <span style="color:#999;font-size:13px">急停后先「停止当前任务」再点「复位+自动模式」才能继续动</span>
</div>
<div id="msg"></div>

<hr>
<h2>第 1 步 · 通路测试</h2>
<p class="tip">这一步是干嘛的：先确认手和臂都能动。手没反应检查 CAN 灯，臂没反应检查控制柜绿灯。</p>
<div class="row">灵巧手：
  <button onclick="handCmd('open')">伸直</button>
  <button onclick="handCmd('pre')">预抓握</button>
  <button onclick="handCmd('grasp')">抓握</button>
</div>
<div class="row">机械臂：
  <button onclick="moveZ(50)">抬 5cm</button>
  <button onclick="moveZ(-50)">降 5cm</button>
</div>
<div class="row">
  <button onclick="alignPose()">对正写字姿态</button>
  <span style="color:#999;font-size:13px">通路测试通过后立刻点：位置不动，腕部一次性转到写字姿态
  （rx≈-96 ry≈-6 rz≈-120），之后标定只动 xyz。要求：空手、手指已伸直、臂周无障碍。
  若报错不可达，先用第 6 步把臂抬高点再试。</span>
</div>

<hr>
<h2>第 2 步 · 取笔 / 还笔</h2>
<p class="tip">这一步是干嘛的：让臂自动去笔筒抓笔 / 放回笔。路线：抬到高位（越过筒内笔顶）
→ 笔筒正上方 → 垂直下降抓/放（10-14 新布局：笔筒贴着砚台、筒内笔比砚台高，笔筒区只从
正上方垂直进出，面板已自动抬高，不用管）。<b>空手也能点，先空跑一遍确认轨迹。</b>
按钮灰的说明缺航点（旁边会写缺哪个），先去第 3 步补齐。</p>
<div class="row">
  <button id="btn_pick" onclick="flowStart('pick_pen')">取笔</button>
  <button id="btn_return" onclick="flowStart('return_pen')">还笔</button>
  <span id="hint_pick" class="hint"></span>
  <span id="hint_return" class="hint"></span>
</div>

<hr>
<h2>第 3 步 · 航点标定向导</h2>
<p class="tip">这一步是干嘛的：按从上到下的顺序标定。<b>顺序：home → 笔/墨/点睛 9 点 →
（点睛全流程跑通后）纸面 6 点。</b>笔墨点睛 9 点的标定手法（对着下面清单做）：<br>
① 第 6 步「飞到坐标」到笔筒上方（悬停Z≥250）→ 微调到预抓握悬停 → 存 pen_lift<br>
② 点 预抓握（第 1 步）→ Z− 微调下降到抓握位 → 存 pen_grasp<br>
③ 点 抓握 → Z+ 竖直抬高到笔完全脱离笔筒 → <b>覆盖存 pen_lift</b>（这步之后笔不离手）<br>
④ 保持 z 不变，XY 微调到砚台上方 → 存 ink_pre → Z− 下降到笔尖浸入墨约 5mm → 存 ink_dip<br>
⑤ Z+ 抬到刚脱离砚台（不用回 ink_pre 高度）→ 存 ink_post<br>
⑥「飞到坐标」到龙眼睛上方 → 微调 → 存 eye_pre → Z− 下降到笔尖刚好触纸 → 存 eye_dot<br>
⑦ Z+ 抬到与 pen_lift 的 z 等高（看顶部位姿显示的 z 数值对齐）→ 存 eye_post<br>
保存时会连读 3 次确认臂没动（晃动 ≥1mm 不让存），存完自动回读核对。
<b>手动微调时严禁在低位横向挪向笔筒——先 Z+ 抬到高过筒内笔顶再横移。</b></p>
<div class="row">
  <label style="font-size:14px"><input type="checkbox" id="goexact">去微调时降到底（默认停在点位上方 15mm）</label>
  <button onclick="loadCalib()">刷新清单</button>
  <span id="wpfile" style="color:#999;font-size:13px"></span>
</div>
<div id="calib"></div>

<hr>
<h2>第 4 步 · 画龙点睛</h2>
<p class="tip">这一步是干嘛的：决赛「画龙点睛」环节演练。「点睛全流程」= 回home → 取笔 →
蘸墨前悬停 → 蘸墨+抖墨 → 预点睛 → 点睛 → 抬到点睛后 → 等高回笔筒还笔 → 回home；
「仅点睛」= 手里已经有笔时用（蘸墨 → 点睛 → 抬到点睛后）。<b>先空手空跑一遍确认轨迹，
跑通全流程后再去第 3 步标纸面 6 点。</b></p>
<div class="row">
  <button id="btn_dotfull" onclick="flowStart('dot_eye_full')">点睛全流程</button>
  <button id="btn_dotonly" onclick="flowStart('dot_eye_only')">仅点睛</button>
  <span id="hint_dotfull" class="hint"></span>
  <span id="hint_dotonly" class="hint"></span>
</div>

<hr>
<h2>第 5 步 · 写字</h2>
<p class="tip">这一步是干嘛的：正式写字。<b>正式写之前先点「检查字库」，全绿才能写。</b>
建议先「试写一个字」确认墨色和位置没问题。</p>
<div class="row">试写：
  <input id="trychar" value="烟" maxlength="2" style="font-size:18px;width:52px;text-align:center;padding:6px">
  <button onclick="wtry()">试写这个字（先蘸墨，写在起始格）</button>
</div>
<div class="row">
  <textarea id="wtext" rows="3" placeholder="输入诗文，如：一去二三里烟村四五家（不要带标点）"></textarea>
</div>
<div class="row">
  <label>起始纸
    <select id="wsheet" style="font-size:16px;padding:4px">
      <option value="1" selected>1</option><option value="2">2</option>
      <option value="3">3</option><option value="4">4</option>
    </select>
  </label>
  <label>起始行
    <select id="wrow" style="font-size:16px;padding:4px">
      <option value="1" selected>1</option><option value="2">2</option>
    </select>
  </label>
  <label>起始列
    <select id="wcol" style="font-size:16px;padding:4px">
      <option value="1" selected>1</option><option value="2">2</option>
      <option value="3">3</option><option value="4">4</option><option value="5">5</option>
    </select>
  </label>
  <label style="font-size:14px"><input type="checkbox" id="wcomplex" checked>含复杂字</label>
</div>
<div class="row">
  <button onclick="wcheck()">检查字库</button>
  <button onclick="wpreview()">预览</button>
  <button onclick="wdip()">蘸墨</button>
  <button onclick="wstart()">开始书写</button>
</div>
<div id="wresult"></div>
<pre id="wlog"></pre>

<hr>
<h2>第 6 步 · 精细微调</h2>
<p class="tip">这一步是干嘛的：第 3 步对不准时，用这里的小步按钮微调机械臂。键盘：←→=X，↑↓=Y，PgUp/PgDn=Z。</p>
<div class="row">
  <label>步长
    <select id="step">
      <option value="1">1 mm</option>
      <option value="5" selected>5 mm</option>
      <option value="10">10 mm</option>
    </select>
  </label>
</div>
<div class="row">
  <button onclick="move('x',-1)">X−</button>
  <button onclick="move('x', 1)">X+</button>
  <button onclick="move('y',-1)">Y−</button>
  <button onclick="move('y', 1)">Y+</button>
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
<hr style="margin:10px 0">
<p class="tip"><b>手指微调</b>（拇指/食指/中指）。显示的是最后一次指令值，括弧里是点「读实际角度」
读到的电机真实角度。<b>电机顶死会锁死：一次只动几度，觉得紧了往反方向退。</b></p>
<div class="row">
  <label>手指步长
    <select id="fstep">
      <option value="1">1°</option>
      <option value="3" selected>3°</option>
      <option value="5">5°</option>
    </select>
  </label>
  <button onclick="handFb()">读实际角度</button>
</div>
<div id="fjog"></div>
<div class="row">
  <button onclick="handSave('pre')">把当前手型存为预抓握</button>
  <button onclick="handSave('grasp')">把当前手型存为抓握</button>
</div>
<div class="row">
  <input id="wpname" placeholder="航点名，如 paper1_tl" style="font-size:16px;padding:8px;width:220px">
  <button onclick="savewp()">存航点（自由命名，不走三读核对）</button>
</div>
<div class="row">
  <input id="gx" placeholder="X mm" style="font-size:16px;padding:8px;width:76px">
  <input id="gy" placeholder="Y mm" style="font-size:16px;padding:8px;width:76px">
  <input id="gz" placeholder="悬停Z(默认250)" style="font-size:16px;padding:8px;width:140px">
  <input id="gtz" placeholder="转场Z(默认300)" style="font-size:16px;padding:8px;width:140px">
  <button onclick="gotoXYZ()">飞到坐标</button>
</div>
<p class="tip">「飞到坐标」= 一次性三段式（抬到转场Z → 平移 → 降到悬停 Z），<b>首次标定够不到的点用它</b>：
估算个大概坐标先飞到目标上方悬停，再用上面的按钮微调对准。比一点一点挪可靠（点动逼近限位会锁死，
直飞是重新解算不会）。坐标估算错了没关系，反正是落在高位。去笔筒区时悬停 Z 填 250 以上。
<b>第一次飞每条新路线时：眼睛盯肘部（不是盯手），一只手放在面板急停旁。</b>关节空间运动的
TCP 不走直线，肘部可能下沉；担心就先把转场 Z 填 350 飞一遍看看。眼看要撞 → 点急停，
恢复流程见一页纸说明应急卡。</p>
<div class="row">
  <button onclick="gohome()">回 home</button>
</div>
<script>
let busy = false;
let writeActive = false;   // 写字/蘸墨后台任务运行中：锁住一切会动机械臂的操作
const msgEl = document.getElementById('msg');
const poseEl = document.getElementById('pose');

function setMsg(t, isErr) { msgEl.textContent = t; msgEl.className = isErr ? 'err' : ''; }
function showPose(p) {
  poseEl.textContent = 'x=' + p[0].toFixed(1) + '  y=' + p[1].toFixed(1) +
    '  z=' + p[2].toFixed(1) + '\\nrx=' + p[3].toFixed(1) + '  ry=' + p[4].toFixed(1) +
    '  rz=' + p[5].toFixed(1);
}
function setButtons(disabled) {
  // estop=硬件急停、wstop=停止当前任务、btnreset=复位，任何时候都不能禁用
  document.querySelectorAll('button').forEach(b => {
    if (b.id !== 'estop' && b.id !== 'wstop' && b.id !== 'btnreset') b.disabled = disabled;
  });
}

async function move(axis, dir) {
  if (busy || writeActive) return;
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
  if (busy || writeActive) return;
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
    loadCalib();
  } catch (e) { setMsg('保存失败：' + e.message, true); }
}

// ---------------- 第 1 步：通路测试 ----------------
async function moveZ(dz) {
  if (busy || writeActive) return;
  busy = true; setButtons(true); setMsg((dz > 0 ? '抬' : '降') + ' 5cm 中…');
  try {
    const r = await fetch('/api/move_z', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({dz})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose); setMsg('完成');
  } catch (e) { setMsg('出错：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

// ---------------- 第 2/4 步：编排流程 ----------------
const flowNames = {pick_pen: '取笔', return_pen: '还笔',
                   dot_eye_full: '点睛全流程', dot_eye_only: '仅点睛'};
const flowDesc = {
  pick_pen: '回 home → 抬高位去预抓握点 → 手预抓握 → 降到抓握位 → 抓握 → 抬回预抓握点',
  return_pen: '去预抓握点 → 降到抓握位 → 手松成预抓握 → 抬回 → 手伸直 → 回 home',
  dot_eye_full: '取笔 → 蘸墨前悬停 → 蘸墨+抖墨 → 预点睛 → 点睛 → 点睛后 → 还笔（整套）',
  dot_eye_only: '蘸墨前悬停 → 蘸墨+抖墨 → 预点睛 → 点睛 → 抬到点睛后（手里已有笔时用）'
};

async function flowStart(flow) {
  if (writeActive) { setMsg('有任务正在进行中', true); return; }
  if (!confirm(flowNames[flow] + ' 会运动机械臂和灵巧手：\\n' + flowDesc[flow] + '\\n确认开始？')) return;
  try {
    await wpost('/api/flow_start', {flow});
    setMsg(flowNames[flow] + ' 已启动');
    pollWrite();
  } catch (e) { setMsg(flowNames[flow] + ' 启动失败：' + e.message, true); }
}

// ---------------- 第 3 步：航点标定向导 ----------------
let calibData = null;

async function loadCalib() {
  try {
    const r = await fetch('/api/calib_points');
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    calibData = j;
    const box = document.getElementById('calib');
    box.innerHTML = '';
    j.points.forEach((p, i) => {
      const row = document.createElement('div');
      row.className = 'crow';
      const st = !p.exists ? '<span class="miss">✗ 缺失</span>'
               : (p.has_joints ? '<span class="ok">✓ 已存</span>'
                               : '<span style="color:#e80">已存(旧格式)</span>');
      row.innerHTML = '<span class="cname">' + (i + 1) + '. ' + esc(p.key) +
                      '　' + esc(p.label) + '</span>' +
                      '<span class="cst">' + st + '</span>';
      const bGo = document.createElement('button');
      bGo.textContent = '去微调';
      bGo.disabled = !p.exists;
      bGo.onclick = () => gotoPoint(p.key);
      const bSave = document.createElement('button');
      bSave.textContent = '保存到该点';
      bSave.onclick = () => saveChecked(p.key);
      const dev = document.createElement('span');
      dev.className = 'cdev';
      dev.id = 'dev_' + p.key;
      row.appendChild(bGo);
      row.appendChild(bSave);
      row.appendChild(dev);
      box.appendChild(row);
    });
    // 流程按钮：缺航点直接禁用并写出缺哪个
    const flowBtn = {pick_pen: 'btn_pick', return_pen: 'btn_return',
                     dot_eye_full: 'btn_dotfull', dot_eye_only: 'btn_dotonly'};
    const flowHint = {pick_pen: 'hint_pick', return_pen: 'hint_return',
                      dot_eye_full: 'hint_dotfull', dot_eye_only: 'hint_dotonly'};
    Object.keys(flowBtn).forEach(f => {
      const info = j.flows[f];
      const btn = document.getElementById(flowBtn[f]);
      const hint = document.getElementById(flowHint[f]);
      if (info && !info.ok) {
        btn.disabled = true;
        hint.textContent = flowNames[f] + '缺：' + info.missing.join(' ');
      } else {
        btn.disabled = false;
        hint.textContent = '';
      }
    });
    document.getElementById('wpfile').textContent =
      j.file_mtime ? ('航点文件最后保存：' + j.file_mtime) : '';
  } catch (e) { setMsg('点位清单加载失败：' + e.message, true); }
}

async function gotoPoint(name) {
  if (busy || writeActive) return;
  const exact = document.getElementById('goexact').checked;
  busy = true; setButtons(true); setMsg('正在去 [' + name + ']（抬高→平移→降下）…');
  try {
    const r = await fetch('/api/goto_waypoint', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name, exact})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose);
    // 到位后算一下与保存点的偏差（当前偏差）
    const p = calibData && calibData.points.find(q => q.key === name);
    if (p && p.pose) {
      const d = Math.hypot(j.pose[0] - p.pose[0], j.pose[1] - p.pose[1],
                           j.pose[2] - (exact ? p.pose[2] : p.pose[2] + 15));
      const el = document.getElementById('dev_' + name);
      if (el) el.textContent = '当前偏差 ' + d.toFixed(1) + ' mm';
    }
    setMsg(exact ? '已到 [' + name + '] 原位置（降到底）'
                 : '已到 [' + name + '] 上方 15mm，要压下去用第 6 步的 Z−');
  } catch (e) { setMsg('去点位失败：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function saveChecked(name) {
  if (busy || writeActive) { setMsg('有运动/任务在进行，不能保存', true); return; }
  if (!confirm('用当前位姿覆盖保存 [' + name + ']？\\n会先连读 3 次确认臂没动，存完自动回读核对。')) return;
  busy = true; setButtons(true); setMsg('正在保存 [' + name + ']（连读 3 次确认臂停稳）…');
  try {
    const r = await fetch('/api/save_checked', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    const el = document.getElementById('dev_' + name);
    if (el) el.textContent = '✓已保存并核对（读数晃动 ' + j.spread_mm.toFixed(2) +
                             'mm，回读差 ' + j.verify_mm.toFixed(2) + 'mm）';
    setMsg('[' + name + '] ✓已保存并核对');
    loadCalib();
  } catch (e) { setMsg('保存失败：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function resetCtl() {
  try {
    const r = await fetch('/api/reset', {method: 'POST'});
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    setMsg('已复位，自动模式');
  } catch (e) { setMsg('复位失败：' + e.message, true); }
}

async function alignPose() {
  // 对正写字姿态：保持当前 xyz，rpy 一次性转到写字姿态（10-12 定稿值）。
  // 从运输收拢构型直飞可能不可达 → 报错是安全失败，抬高臂再试即可
  if (busy || writeActive) return;
  if (!confirm('把腕部姿态对正到写字姿态（rx≈-96 ry≈-6 rz≈-120），位置不动。\\n确认：空手、手指已伸直、臂周无障碍？')) return;
  busy = true; setButtons(true); setMsg('对正写字姿态中…');
  try {
    const r = await fetch('/api/align_pose', {method: 'POST'});
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose); setMsg('已对正写字姿态，之后标定只动 xyz');
  } catch (e) { setMsg('对正失败：' + e.message + '（先把臂抬高/挪开再试）', true); }
  finally { busy = false; setButtons(false); }
}

async function gotoXYZ() {
  // 首次标定的粗定位：一次性三段式直飞目标坐标上方悬停（增量点动逼近限位会锁死，
  // 直飞是重新 IK 解算，没有这个毛病）
  if (busy || writeActive) return;
  const gx = parseFloat(document.getElementById('gx').value);
  const gy = parseFloat(document.getElementById('gy').value);
  const gz = parseFloat(document.getElementById('gz').value) || 250.0;
  const gtz = parseFloat(document.getElementById('gtz').value) || 300.0;
  if (isNaN(gx) || isNaN(gy)) { setMsg('先填 X 和 Y（mm），Z 不填默认悬停 250', true); return; }
  if (!confirm('飞到 (' + gx + ', ' + gy + ') 上方悬停 z=' + gz +
               '？\\n转场高度 ' + gtz + '。第一次飞这条路线请盯肘部、手放急停旁。')) return;
  busy = true; setButtons(true); setMsg('飞到坐标中…');
  try {
    const r = await fetch('/api/goto', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({x: gx, y: gy, z: gz, travel_z: gtz})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    showPose(j.pose); setMsg('已到 (' + gx + ', ' + gy + ') 上方悬停，用微调对准后存点');
  } catch (e) { setMsg('飞到坐标失败：' + e.message, true); }
  finally { busy = false; setButtons(false); }
}

async function gohome() {
  // 关节空间回 home：不走 IK，任意怪姿态都能回来（奇异/限位恢复用）
  if (busy || writeActive) return;
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
  if (writeActive) { setMsg('写字进行中，禁止手部动作（手握着笔）', true); return; }
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

// ---------------- 手指微调 ----------------
const FJOINTS = [
  ['th_rot', '拇指根旋'], ['th_mcp', '拇指根弯'], ['th_dip', '拇指尖'],
  ['ff_mcp', '食指根'],   ['ff_dip', '食指尖'],
  ['mf_mcp', '中指根'],   ['mf_dip', '中指尖'],
];
let fstate = {};   // 最后一次指令值（服务端 Hand.state）
let ffb = {};      // 最近一次读到的电机实际角度

function buildFjog() {
  const box = document.getElementById('fjog');
  box.innerHTML = '';
  FJOINTS.forEach(function(pair) {
    const j = pair[0], label = pair[1];
    const row = document.createElement('div');
    row.className = 'row';
    row.style.cssText = 'gap:6px;font-size:15px';
    const span = document.createElement('span');
    span.style.cssText = 'display:inline-block;width:150px';
    span.textContent = label + ' ' + j;
    const minus = document.createElement('button');
    minus.textContent = '−';
    minus.onclick = function() { handJog(j, -1); };
    const val = document.createElement('span');
    val.id = 'fv_' + j;
    val.style.cssText = 'display:inline-block;width:150px;text-align:center';
    val.textContent = '?';
    const plus = document.createElement('button');
    plus.textContent = '＋';
    plus.onclick = function() { handJog(j, 1); };
    row.appendChild(span);
    row.appendChild(minus);
    row.appendChild(val);
    row.appendChild(plus);
    box.appendChild(row);
  });
  renderFjog();
}

function renderFjog() {
  FJOINTS.forEach(([j]) => {
    const el = document.getElementById('fv_' + j);
    if (!el) return;
    const cmd = (j in fstate) ? fstate[j].toFixed(0) + '°' : '?';
    const act = (j in ffb) ? '（实际 ' + ffb[j].toFixed(1) + '°）' : '';
    el.textContent = cmd + act;
  });
}

async function handJog(joint, dir) {
  if (busy || writeActive) return;
  const step = parseFloat(document.getElementById('fstep').value);
  try {
    const r = await fetch('/api/hand_jog', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({joint, delta: dir * step})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    fstate = j.state;
    renderFjog();
    setMsg(joint + ' → ' + j.value.toFixed(0) + '°');
  } catch (e) { setMsg('手指微调失败：' + e.message, true); }
}

async function handFb() {
  try {
    const r = await fetch('/api/hand_fb');
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    fstate = j.state || fstate;
    ffb = j.feedback || {};
    renderFjog();
    setMsg('已读取手指实际角度');
  } catch (e) { setMsg('读取失败：' + e.message, true); }
}

async function handSave(which) {
  const label = which === 'pre' ? '预抓握' : '抓握';
  if (!confirm('把当前手型（最后一次指令值）存为「' + label + '」参数？\\n' +
               '会覆盖 data/grasp_params.json 里的旧值并立即生效。')) return;
  try {
    const r = await fetch('/api/hand_save', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({which})
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
    setMsg('已存为' + label + '：' + j.saved);
  } catch (e) { setMsg('存握姿失败：' + e.message, true); }
}

async function loadHandState() {
  try {
    const r = await fetch('/api/hand_state');
    const j = await r.json();
    if (r.ok && j.state) { fstate = j.state; renderFjog(); }
  } catch (e) { /* 静默 */ }
}

buildFjog();
loadHandState();

async function refreshPose() {
  // 写字期间不并发读位姿：fairino RPC 与写字线程共用一条连接，避免交错
  if (busy || writeActive) return;
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

// ---------------- 写字区块 ----------------
const wlogEl = document.getElementById('wlog');
const wresEl = document.getElementById('wresult');
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
function wres(html) { wresEl.innerHTML = html; }

function wparams() {
  return {
    text: document.getElementById('wtext').value,
    sheet: parseInt(document.getElementById('wsheet').value),
    row: parseInt(document.getElementById('wrow').value),
    col: parseInt(document.getElementById('wcol').value),
    allow_complex: document.getElementById('wcomplex').checked
  };
}

async function wpost(path, body) {
  const r = await fetch(path, {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {})
  });
  const j = await r.json();
  if (!r.ok) throw new Error(j.error || ('HTTP ' + r.status));
  return j;
}

// ---------------- 第 5 步：写字 ----------------
async function wtry() {
  const ch = document.getElementById('trychar').value.trim();
  if (!ch) { setMsg('先输入要试写的字', true); return; }
  const p = wparams();
  if (!confirm('试写「' + ch + '」：先蘸墨一次，再写在 paper' + p.sheet +
               ' r' + p.row + 'c' + p.col + '（约 35 秒）。\\n确认手已握稳笔、笔尖朝下？')) return;
  try {
    await wpost('/api/write_start', {text: ch, sheet: p.sheet, row: p.row,
                                     col: p.col, allow_complex: true});
    setMsg('试写「' + ch + '」已启动');
    pollWrite();
  } catch (e) { setMsg('试写启动失败：' + e.message, true); }
}

async function wcheck() {
  const p = wparams();
  if (!p.text.trim()) { wres('<span style="color:#d33">先输入诗文</span>'); return; }
  wres('检查字库中…');
  try {
    const j = await wpost('/api/write_check', {text: p.text});
    let html = '共 ' + j.unique + ' 个不同字：';
    j.chars.forEach(c => {
      html += c.ok ? '<span style="color:#282">' + esc(c.ch) + '</span> '
                   : '<span style="color:#d33;font-weight:bold;background:#fdd">'
                     + esc(c.ch) + '</span> ';
    });
    if (j.missing.length) {
      html += '<br><b style="color:#d33">缺字无法书写，需联网预抓或换字：'
              + j.missing.map(esc).join(' ') + '</b>';
    } else {
      html += '<br><b style="color:#282">✓ 字库齐全，可以书写</b>';
    }
    wres(html);
  } catch (e) { wres('<span style="color:#d33">检查失败：' + esc(e.message) + '</span>'); }
}

async function wpreview() {
  const p = wparams();
  if (!p.text.trim()) { wres('<span style="color:#d33">先输入诗文</span>'); return; }
  wres('生成预览（dry_run，不动机械臂）…');
  try {
    const j = await wpost('/api/write_preview', p);
    let html = '预览：共 ' + j.total_chars + ' 字，墨迹总长 ' + j.total_mm +
               'mm，预计约 ' + j.est_seconds + ' 秒（含蘸墨，按 20s/字 估）<br>';
    j.chars.forEach(c => {
      html += esc(c.char) + ' → paper' + c.sheet + ' r' + c.row + 'c' + c.col +
              '　' + c.strokes + ' 笔 ' + c.length_mm.toFixed(0) + 'mm<br>';
    });
    wres(html);
  } catch (e) { wres('<span style="color:#d33">预览失败：' + esc(e.message) + '</span>'); }
}

async function wdip() {
  if (!confirm('蘸墨会运动机械臂（抬到转场高度 → 走高空走廊 → 下探墨碟），确认？')) return;
  try { await wpost('/api/write_dip'); setMsg('蘸墨中…'); pollWrite(); }
  catch (e) { setMsg('蘸墨启动失败：' + e.message, true); }
}

async function wstart() {
  const p = wparams();
  const n = [...p.text].filter(c => c.trim()).length;
  if (!n) { wres('<span style="color:#d33">先输入诗文</span>'); return; }
  if (!confirm('流程：先蘸墨一次，再从 paper' + p.sheet + ' r' + p.row + 'c' + p.col +
               ' 起写 ' + n + ' 个字（约 ' + (15 + 20 * n) + ' 秒）' +
               (p.allow_complex ? '，已放行复杂字' : '') +
               '。\\n确认：手已握稳笔、笔尖朝下、纸已铺好？')) return;
  try {
    await wpost('/api/write_start', p);
    setMsg('书写任务已启动');
    pollWrite();
  } catch (e) {
    setMsg('书写启动失败：' + e.message, true);
    wres('<span style="color:#d33">' + esc(e.message) + '</span>');
  }
}

async function wstop() {
  try { await wpost('/api/write_stop'); setMsg('已请求停止：当前动作完成后停', true); }
  catch (e) { setMsg('停止请求失败：' + e.message, true); }
}

const taskNames = {write: '书写', dip: '蘸墨', ...flowNames};

async function pollWrite() {
  try {
    const r = await fetch('/api/write_status');
    const j = await r.json();
    if (!r.ok) return;
    const active = (j.status === 'running' || j.status === 'stopping');
    const tname = taskNames[j.mode] || '任务';
    if (active && !writeActive) setButtons(true);          // 锁住所有运动按钮
    if (!active && writeActive) {                          // 刚结束：解锁 + 总结
      setButtons(false);
      if (j.status === 'done') setMsg(tname + '完成');
      else if (j.status === 'stopped') setMsg(tname + '已停止', true);
      else if (j.status === 'error') setMsg(tname + '出错：' + (j.error || '未知'), true);
    }
    writeActive = active;
    document.getElementById('wstop').disabled = !active;
    if (j.log && j.log.length) {
      wlogEl.style.display = 'block';
      wlogEl.textContent = j.log.slice(-60).join('\\n');
      wlogEl.scrollTop = wlogEl.scrollHeight;
    }
    if (active && j.current) setMsg(j.current);
  } catch (e) { /* 静默，下轮再试 */ }
}

refreshPose();
setInterval(refreshPose, 2000);
setInterval(pollWrite, 1000);
loadCalib();
</script>
</body>
</html>
"""


class MockArm:
    """无硬件调试用的假机械臂：内存里维护一个位姿和一张航点表。

    航点表故意只放流程需要的点（home/pen_*/ink_*/eye_dot），纸阵 paper1_* 不放，
    这样第 3 步向导正好演示「缺失」状态。保存只进内存，绝不写真实 waypoints.json。"""

    def __init__(self):
        self.pose = [400.0, 0.0, 300.0, 180.0, 0.0, 0.0]
        # 注意：刻意不设 waypoint_file 属性 —— WriteEngine 会 getattr 它来决定读哪个
        # 航点文件，mock 下要落到默认的 src/waypoints.json（写字/蘸墨靠它）
        self._wps = {
            "home": {"pose": [400.0, 0.0, 300.0, 180.0, 0.0, 0.0],
                     "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            # 新布局：笔筒与砚台相邻（约 17.5cm），mock 里也摆在一起
            "pen_lift": {"pose": [500.0, -375.0, 250.0, 180.0, 0.0, 0.0], "joints": None},
            "pen_grasp": {"pose": [500.0, -375.0, 180.0, 180.0, 0.0, 0.0], "joints": None},
            "ink_pre": {"pose": [500.0, -200.0, 250.0, 180.0, 0.0, 0.0], "joints": None},
            "ink_dip": {"pose": [500.0, -200.0, 195.0, 180.0, 0.0, 0.0], "joints": None},
            "ink_post": {"pose": [500.0, -200.0, 215.0, 180.0, 0.0, 0.0], "joints": None},
            "eye_pre": {"pose": [300.0, -100.0, 250.0, 180.0, 0.0, 0.0], "joints": None},
            "eye_dot": {"pose": [300.0, -100.0, 200.0, 180.0, 0.0, 0.0], "joints": None},
            "eye_post": {"pose": [300.0, -100.0, 250.0, 180.0, 0.0, 0.0], "joints": None},
        }

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
        self._wps[name] = {"pose": list(self.pose), "joints": [0.0] * 6}
        print(f"[mock] 存航点 {name} = {self.pose}")

    def list_waypoints(self):
        return {k: {"pose": list(v["pose"]), "joints": v["joints"]}
                for k, v in self._wps.items()}

    def get_waypoint(self, name):
        if name not in self._wps:
            raise KeyError(f"航点 [{name}] 不存在，已有: {list(self._wps)}")
        return list(self._wps[name]["pose"])

    def move_ik(self, pose, vel=10.0, blocking=True):
        time.sleep(0.3)
        self.pose = list(pose)

    def move_to(self, pose, vel=10.0, blocking=True):
        self.move_ik(pose, vel, blocking)

    def move_joints(self, joints, vel=20.0, blocking=True):
        time.sleep(0.3)  # mock 里关节角和位姿不挂钩，假装动完了

    def get_waypoint_joints(self, name):
        wp = self._wps.get(name)
        return wp["joints"] if wp else None


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


# ============================================================================
# 写字（决赛流程：公布诗题 → 面板输入诗文 → 检查字库 → 预览 → 蘸墨 → 整首写完）
# ============================================================================
WRITE_SEC_PER_CHAR = 20    # 总时长估算：约 20s/字（10-12 赛场实测）
WRITE_DIP_S = 15           # 蘸墨一次约 15s

_write_lock = threading.Lock()
_write_stop = threading.Event()
_write = {"status": "idle", "mode": None, "current": "", "error": None, "log": []}
_stroke_lib = None
_engine_mod = None


def get_stroke_lib():
    """懒加载笔画库：offline=True 只读本地缓存，现场无网也能用，绝不联网。"""
    global _stroke_lib
    if _stroke_lib is None:
        from stroke_lib import StrokeLib
        _stroke_lib = StrokeLib(offline=True, verbose=False)
    return _stroke_lib


def get_engine_mod():
    """懒加载 write_engine（带 numpy 等重依赖，只在用写字功能时才 import）。"""
    global _engine_mod
    if _engine_mod is None:
        import write_engine
        _engine_mod = write_engine
    return _engine_mod


def _write_log(line):
    with _write_lock:
        _write["log"].append(line)
        if len(_write["log"]) > 400:          # 封顶，防止长任务把状态撑爆
            del _write["log"][:-400]


def _write_progress(msg):
    with _write_lock:
        _write["current"] = msg
    _write_log("== " + msg + " ==")


def _write_finish(status, error=None):
    tail = {"done": "== 全部完成 ==", "stopped": "== 已停止（当前动作完成后停）==",
            "error": f"== 出错中止：{error} =="}.get(status, status)
    with _write_lock:
        _write["status"] = status
        _write["error"] = error
        _write["current"] = ""
    _write_log(tail)


class _LogTee:
    """把写字引擎 print 的每一行收进面板进度日志（引擎本身不改）。"""

    def __init__(self):
        self._buf = ""

    def write(self, s):
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                _write_log(line)

    def flush(self):
        if self._buf.strip():
            _write_log(self._buf)
        self._buf = ""


class _WriteAbort(Exception):
    """操作员在面板点了「停止」。"""


def _make_engine_once(allow_complex):
    """复用本进程已有的 arm/hand 连接造写字引擎（CNDE 只允许单客户端，不能再起 Arm()）。"""
    we = get_engine_mod()

    class _JogWriteEngine(we.WriteEngine):
        def _vertical(self, pose, why, vel=None):
            # 停止检查点：只拦「落笔」。落笔前笔一定在纸面上方（5/20/50mm 高位），
            # 此刻中止绝对安全 → 实现「当前笔画完成后停」，不用毫秒级急停
            if _write_stop.is_set() and "落笔" in why:
                raise _WriteAbort()
            return super()._vertical(pose, why, we.DESCEND_VEL if vel is None else vel)

    return _JogWriteEngine(get_arm(), get_hand(), allow_complex=allow_complex)


def _make_engine(allow_complex):
    """造引擎，失败重试一次：初始化要读 strokes_cache.json/waypoints.json，
    预抓进程正在写缓存时可能读到半截 JSON，等 2s 再试。"""
    try:
        return _make_engine_once(allow_complex)
    except Exception:                           # noqa: BLE001
        time.sleep(2)
        return _make_engine_once(allow_complex)


# ============================================================================
# 编排流程（第 2 步取笔/还笔、第 4 步画龙点睛）：后台线程 + _move_lock 互斥，
# 步与步之间检查停止标志。竖直移动一律 move_ik（MoveL 会撞奇异点）。
# ============================================================================
GOTO_TRAVEL_Z = 300.0     # 三段式转场高度（与 /api/goto_waypoint 一致）
WRITE_RPY = (-95.8, -5.8, -120.0)   # 写字姿态（10-12 定稿：home/纸角/蘸墨全在这个姿态
                                    # 附近，rz≈-120 是 rz 扫描全纸可达性最优解）
PEN_AREA_WPS = ("pen_lift", "pen_grasp")   # 笔筒区航点
PEN_TRAVEL_Z = 340.0      # 笔筒区横移高度下限：必须高过筒内竖笔的顶部（笔筒贴着砚台，
                          # 筒内笔顶高过 ink_pre 悬停位，低位横移必撞笔——10-15 布局复核）。
                          # 若报 err=38（该位置抬不到这么高）：先用第 6 步 Z+ 手动抬高，
                          # 再点流程/去微调，并把情况告诉程序组调这个数

CALIB_POINTS = [          # 第 3 步向导的有序点位清单（从上到下 = 示教顺序）
    # 10-15 点睛优先：先标 home + 笔/墨/点睛共 9 点，点睛全流程跑通后再标纸面 6 点。
    # 笔筒区只从正上方垂直进出：去笔筒区面板自动抬到 PEN_TRAVEL_Z 越过筒内笔顶
    ("home", "回零点（★开场第一个标它：x≈0 高位收拢，别罩桌面任何东西上方）"),
    ("pen_lift", "预抓握点（笔筒上方悬停；抓笔后抬到笔脱筒，要再覆盖存一次）"),
    ("pen_grasp", "抓握位（笔尖进筒、手合拢的位置）"),
    ("ink_pre", "蘸墨前悬停（砚台上方，从预抓握点等高平移过来）"),
    ("ink_dip", "蘸墨点（笔尖浸入墨汁约 5mm）"),
    ("ink_post", "蘸墨后悬停（刚脱离砚台即可，不用回到蘸墨前高度）"),
    ("eye_pre", "预点睛（龙眼睛上方悬停，飞到坐标+微调）"),
    ("eye_dot", "点睛位（笔尖触纸的精确位置）"),
    ("eye_post", "点睛后（竖直抬到与预抓握点等高，看 z 数值对齐）"),
    ("paper1_tl", "纸1 左上角（★点睛全流程跑通后才标纸面点）"),
    ("paper1_tr", "纸1 右上角"),
    ("paper1_bl", "纸1 左下角"),
    ("paper1_br", "纸1 右下角"),
    ("paper1_cell00_tl", "第 0 格十字中心"),
    ("paper1_cell00_corner_tr", "第 0 格田字格右上角"),
]

FLOW_LABELS = {"pick_pen": "取笔", "return_pen": "还笔",
               "dot_eye_full": "点睛全流程", "dot_eye_only": "仅点睛"}
FLOW_WAYPOINTS = {        # 每个流程缺哪个航点就不许跑（按钮禁用 + 服务端再拦一次）
    "pick_pen": ["home", "pen_lift", "pen_grasp"],
    "return_pen": ["home", "pen_lift", "pen_grasp"],
    "dot_eye_full": ["home", "pen_lift", "pen_grasp",
                     "ink_pre", "ink_dip", "ink_post",
                     "eye_pre", "eye_dot", "eye_post"],
    "dot_eye_only": ["ink_pre", "ink_dip", "ink_post", "eye_pre", "eye_dot", "eye_post"],
}


def _goto_wp(arm, name, final_dz=0.0):
    """三段式去航点：原地抬高 → 平移 → 竖直降下，全程 move_ik（与「去微调」同逻辑）。
    final_dz>0 时停在点位上方（默认降到底）。笔筒区航点横移高度不低于 PEN_TRAVEL_Z。"""
    wp = arm.get_waypoint(name)
    cur = arm.get_pose()
    safe_z = max(GOTO_TRAVEL_Z, cur[2], wp[2] + 50.0)  # 转场高度不低于目标上方 50mm
    if name in PEN_AREA_WPS:
        safe_z = max(safe_z, PEN_TRAVEL_Z)             # 越筒内笔顶
    high = cur[:]; high[2] = safe_z
    arm.move_ik(high, vel=10)
    arm.move_ik([wp[0], wp[1], safe_z, wp[3], wp[4], wp[5]], vel=10)
    arm.move_ik([wp[0], wp[1], wp[2] + final_dz, wp[3], wp[4], wp[5]], vel=5)


def _build_flow_steps(flow):
    """把流程展开成 [(步骤名, 动作), ...]，每步之间会检查停止标志。"""
    arm = get_arm()
    hand = get_hand()

    def go_home():
        hj = arm.get_waypoint_joints("home")
        if hj is not None:
            arm.move_joints(hj, vel=10)      # 有关节角走关节空间：任意姿态都回得来
        else:
            _goto_wp(arm, "home")            # 旧格式兜底：三段式 move_ik

    def vertical_to(name, why):
        # 竖直移动一律 move_ik（翻腕姿态下 MoveL 撞奇异点，write_engine 同款教训）
        arm.move_ik(arm.get_waypoint(name), vel=5)

    def grasp():
        if hand is None:
            print("[mock] 无灵巧手，跳过抓握动作")
        else:
            hand.grasp_pen()
        time.sleep(1.0)                      # 等手指收拢再抬笔

    def dip():
        """蘸墨（10-15 新航点方案，调用前已到 ink_pre）：下探蘸墨点停 1.5s →
        抬到蘸墨后悬停（刚离砚台即可）→ 原地 ±8mm 抖墨往返 2 次甩掉浮墨。"""
        vertical_to("ink_dip", "下探蘸墨")
        time.sleep(1.5)
        vertical_to("ink_post", "抬到蘸墨后悬停")
        base = arm.get_pose()
        for i, dx in enumerate((8.0, -16.0, 16.0, -8.0), 1):  # ±8mm 往返 2 次
            nxt = base[:]; nxt[0] += dx
            arm.move_ik(nxt, vel=5)
            time.sleep(0.15)
            print(f"  [抖墨] 第 {i}/4 段")

    def dot_down():
        vertical_to("eye_dot", "下降点睛")
        time.sleep(0.5)                      # 停半秒让墨点上纸

    def dot_up():
        # 竖直提升到「点睛后」（与预抓握点等高，标定时已对齐 z）
        vertical_to("eye_post", "抬到点睛后")

    def level_to(name, why):
        """等高水平移动 + 竖直对齐到目标航点（点睛后→预抓握点用这种：
        等高平移不抬转场高度，省时间；用户确认这段路径不经过笔筒）。"""
        wp = arm.get_waypoint(name)
        cur = arm.get_pose()
        arm.move_ik([wp[0], wp[1], cur[2], wp[3], wp[4], wp[5]], vel=10)
        arm.move_ik(wp, vel=5)

    def pre_grasp():
        if hand is None:
            print("[mock] 无灵巧手，跳过预抓握动作")
        else:
            hand.pre_grasp_pen()
        time.sleep(1.0)

    def hand_open():
        if hand is None:
            print("[mock] 无灵巧手，跳过伸直动作")
        else:
            hand.open()
        time.sleep(1.0)

    steps = []
    if flow in ("pick_pen", "dot_eye_full"):
        # 笔筒区只从正上方垂直进出（_goto_wp 对笔筒区航点自动抬到 PEN_TRAVEL_Z）
        steps += [("回 home", go_home),
                  ("抬到高位，去预抓握点 pen_lift", lambda: _goto_wp(arm, "pen_lift")),
                  ("手预抓握", pre_grasp),
                  ("竖直下降到抓握位 pen_grasp", lambda: vertical_to("pen_grasp", "下降到抓握位")),
                  ("手抓握（抓笔）", grasp),
                  ("竖直抬回预抓握点 pen_lift", lambda: vertical_to("pen_lift", "抬回预抓握点"))]
    if flow == "dot_eye_only":
        # 从任意位置起（手里已有笔）：三段式安全到蘸墨前悬停
        steps += [("移到蘸墨前悬停 ink_pre", lambda: _goto_wp(arm, "ink_pre"))]
    if flow == "dot_eye_full":
        # 刚从预抓握点抬笔出来：等高平移过去（笔已脱筒，路径不经过笔筒，省时间）
        steps += [("等高平移到蘸墨前悬停 ink_pre", lambda: level_to("ink_pre", "去蘸墨前悬停"))]
    if flow in ("dot_eye_full", "dot_eye_only"):
        steps += [("蘸墨+抖墨", dip),
                  ("平移到预点睛 eye_pre（不抬转场高，路径不经过笔筒）",
                   lambda: level_to("eye_pre", "去预点睛")),
                  ("竖直下降到点睛位", dot_down),
                  ("竖直提升到点睛后 eye_post", dot_up)]
    if flow in ("return_pen", "dot_eye_full"):
        # dot_eye_full 从「点睛后」出发：与预抓握点等高，直接平移（用户确认路径不过笔筒）。
        # 单独还笔起点不定（可能在纸面低位）：必须三段式先抬高位
        first = [("等高平移回预抓握点 pen_lift", lambda: level_to("pen_lift", "回预抓握点"))] \
            if flow == "dot_eye_full" else \
            [("抬到高位，去预抓握点 pen_lift", lambda: _goto_wp(arm, "pen_lift"))]
        steps += first + [
                  ("竖直下降到抓握位放笔", lambda: vertical_to("pen_grasp", "下降到放笔位")),
                  ("手松开成预抓握（留笔在筒里）", pre_grasp),
                  ("竖直抬回预抓握点 pen_lift", lambda: vertical_to("pen_lift", "抬回预抓握点")),
                  ("手伸直", hand_open),
                  ("回 home", go_home)]
    return steps


def _flow_worker(flow):
    """流程后台线程：逐步行进，步间响应停止；崩溃只置 error，绝不搞死 server。"""
    label = FLOW_LABELS.get(flow, flow)
    tee = _LogTee()
    try:
        with contextlib.redirect_stdout(tee):
            steps = _build_flow_steps(flow)
            n = len(steps)
            for i, (slabel, fn) in enumerate(steps, 1):
                if _write_stop.is_set():
                    raise _WriteAbort()
                _write_progress(f"{label} 第 {i}/{n} 步：{slabel}")
                fn()
        tee.flush()
        _write_finish("stopped" if _write_stop.is_set() else "done")
    except _WriteAbort:
        tee.flush()
        _write_finish("stopped")
    except Exception as e:                          # noqa: BLE001
        tee.flush()
        _write_finish("error", str(e))
    finally:
        _move_lock.release()


def _start_task(mode, target, args=()):
    """启动后台任务（写字/蘸墨/流程共用）。返回错误字符串，None = 成功。"""
    with _write_lock:
        if _write["status"] in ("running", "stopping"):
            return "有任务正在进行中（看页面顶部状态）"
    if not _move_lock.acquire(blocking=False):      # 与点动/航点/回home互斥
        return "有点动或航点运动正在进行，等它结束再启动"
    _write_stop.clear()
    with _write_lock:
        _write.update({"status": "running", "mode": mode, "current": "启动中…",
                       "error": None, "log": []})
    threading.Thread(target=target, args=args, daemon=True).start()
    return None


def _checked_save(arm, name):
    """保存航点的纪律（修下拉框误存 + 脏数据两个真实事故）：
    连读 3 次当前位姿（间隔 0.5s），三次相差 <1mm 才允许保存；
    写盘后立刻回读 waypoints 比对。返回 (晃动mm, 回读差mm, 落盘位姿)。"""
    reads = []
    for i in range(3):
        reads.append(arm.get_pose())
        if i < 2:
            time.sleep(0.5)
    spread = max(math.dist(reads[a][:3], reads[b][:3])
                 for a, b in ((0, 1), (0, 2), (1, 2)))
    if spread >= 1.0:
        raise RuntimeError(
            f"三次读数晃动 {spread:.2f}mm ≥ 1mm：手臂还在动或没停稳，没保存。"
            f"等臂完全停稳再点")
    arm.save_waypoint(name)
    back = arm.get_waypoint(name)
    verify = math.dist(back[:3], reads[-1][:3])
    if verify >= 1.0:
        raise RuntimeError(f"写盘后回读比对失败：差 {verify:.2f}mm（航点文件写入有问题）")
    return spread, verify, back


def _write_worker(mode, text, sheet, start_row, start_col, allow_complex):
    """写字后台线程：先蘸墨再逐字写。逐字循环的布局逻辑与 WriteEngine.write_text
    一致（写满一行换行、写满一页翻纸），逐字调用是为了报进度和在字间响应停止。
    任何异常只把状态置为 error 并记录原因，绝不搞死 server。"""
    tee = _LogTee()
    try:
        with contextlib.redirect_stdout(tee):
            eng = _make_engine(allow_complex)
            _write_progress("蘸墨…")
            eng.dip_ink()
            if mode == "write":
                chars = [c for c in text if c.strip()]
                n = len(chars)
                g = eng.grid(sheet)
                row, col, cur = start_row, start_col, sheet
                for i, ch in enumerate(chars, 1):
                    if _write_stop.is_set():
                        raise _WriteAbort()
                    if col > g.cols:
                        row += 1
                        col = 1
                        if row > g.rows:
                            cur += 1
                            row = 1
                            eng._need(f"paper{cur}_tl")   # 没示教下一张纸就提前炸掉
                            g = eng.grid(cur)
                            _write_log(f"→ 本页写完，翻到 paper{cur}")
                    _write_progress(f"第 {i}/{n} 字「{ch}」 paper{cur} r{row}c{col}")
                    eng.write_char(ch, cur, row, col)
                    col += 1
        tee.flush()
        _write_finish("stopped" if _write_stop.is_set() else "done")
    except _WriteAbort:
        tee.flush()
        _write_finish("stopped")
    except Exception as e:                          # noqa: BLE001
        tee.flush()
        _write_finish("error", str(e))
    finally:
        _move_lock.release()


def _start_write_task(mode, text="", sheet=1, row=1, col=1, allow_complex=True):
    """启动写字/蘸墨后台线程。返回错误字符串，None = 成功。"""
    return _start_task(mode, _write_worker,
                       (mode, text, sheet, row, col, allow_complex))


def _write_preview(text, sheet, start_row, start_col, allow_complex):
    """dry_run 生成整首计划：用 write_engine._PlanOnlyArm 纯计算，绝不碰硬件。
    布局逻辑与 WriteEngine.write_text 一致。"""
    we = get_engine_mod()
    eng = we.WriteEngine(we._PlanOnlyArm(we.DEFAULT_WP_FILE), None,
                         allow_complex=allow_complex, verbose=False)
    chars = [c for c in text if c.strip()]
    if not chars:
        raise ValueError("诗文为空（空白字符不算）")
    g = eng.grid(sheet)
    row, col, cur = start_row, start_col, sheet
    out = []
    for ch in chars:
        if col > g.cols:
            row += 1
            col = 1
            if row > g.rows:
                cur += 1
                row = 1
                eng._need(f"paper{cur}_tl")
                g = eng.grid(cur)
        plan = eng.plan_char(ch, cur, row, col)
        out.append({"char": ch, "sheet": cur, "row": row, "col": col,
                    "strokes": len(plan["strokes"]), "length_mm": plan["total_mm"]})
        col += 1
    total_mm = round(sum(c["length_mm"] for c in out), 1)
    return {"chars": out, "total_chars": len(out), "total_mm": total_mm,
            "est_seconds": WRITE_DIP_S + WRITE_SEC_PER_CHAR * len(out)}


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
        elif self.path == "/api/hand_state":
            # 手指微调：最后一次指令值（无需连手也可查；未连过则为 None）
            hand = get_hand()
            self._send_json({"state": dict(hand.state) if hand is not None else None})
        elif self.path == "/api/hand_fb":
            # 手指微调：读电机实际角度（需要 CAN 连接）
            try:
                hand = get_hand()
                if hand is None:
                    raise RuntimeError("mock 模式无灵巧手")
                fb = hand._hand.get_feedback()
                self._send_json({"state": dict(hand.state),
                                 "feedback": {k: v.angle for k, v in fb.joints.items()}})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/write_status":
            with _write_lock:
                self._send_json(dict(_write))
        elif self.path == "/api/calib_points":
            # 第 3 步向导：有序点位清单 + 各流程缺航点情况（前端禁用缺点的流程按钮）
            try:
                arm = get_arm()
                wps = arm.list_waypoints()
                points = [{"key": k, "label": lbl, "exists": k in wps,
                           "has_joints": bool(wps.get(k, {}).get("joints")) if k in wps else False,
                           "pose": wps[k]["pose"] if k in wps else None}
                          for k, lbl in CALIB_POINTS]
                flows = {f: {"ok": not [w for w in names if w not in wps],
                             "missing": [w for w in names if w not in wps]}
                         for f, names in FLOW_WAYPOINTS.items()}
                mtime = None
                wpf = getattr(arm, "waypoint_file", None)
                if wpf and os.path.exists(wpf):
                    mtime = time.strftime("%m-%d %H:%M:%S", time.localtime(os.path.getmtime(wpf)))
                self._send_json({"points": points, "flows": flows, "file_mtime": mtime})
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
                if name in PEN_AREA_WPS:        # 笔筒区：横移高度必须越筒内笔顶
                    safe_z = max(safe_z, PEN_TRAVEL_Z)
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
        elif self.path == "/api/hand_jog":
            # 手指微调：单关节相对调整（基于最后指令值，步长由前端给，限 ±10°）
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                joint = str(req["joint"]).strip()
                delta = float(req["delta"])
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            allow = {"th_rot", "th_mcp", "th_dip",
                     "ff_mcp", "ff_dip", "mf_mcp", "mf_dip"}
            if joint not in allow:
                self._send_json({"error": f"不允许微调的关节 {joint!r}"}, 400)
                return
            if abs(delta) > 10:
                self._send_json({"error": "单步不得超过 10°（防顶死电机）"}, 400)
                return
            try:
                with _write_lock:
                    writing = _write["status"] in ("running", "stopping")
                if writing:
                    raise RuntimeError("有任务进行中，禁止手部动作")
                hand = get_hand()
                if hand is None:
                    raise RuntimeError("mock 模式无灵巧手")
                hand.adjust_joint(joint, delta)
                self._send_json({"ok": True, "state": dict(hand.state),
                                 "joint": joint, "value": hand.state[joint]})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/hand_save":
            # 把当前手型（最后指令值）存为预抓握/抓握参数，写 grasp_params.json 并热更新
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                which = str(req["which"]).strip()
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            if which not in ("pre", "grasp"):
                self._send_json({"error": f"which 只能是 pre/grasp，收到 {which!r}"}, 400)
                return
            try:
                with _write_lock:
                    writing = _write["status"] in ("running", "stopping")
                if writing:
                    raise RuntimeError("有任务进行中，禁止改握姿参数")
                hand = get_hand()
                if hand is None:
                    raise RuntimeError("mock 模式无灵巧手")
                import hand_api
                path = hand_api._PARAMS_FILE
                try:
                    with open(path, encoding="utf-8") as f:
                        d = json.load(f)
                except Exception:
                    d = {}
                s = hand.state
                if which == "grasp":
                    d.setdefault("grasp", {})["joints"] = {
                        k: s.get(k, 0) for k in
                        ("th_rot", "th_mcp", "th_dip", "ff_mcp", "ff_dip", "mf_mcp", "mf_dip")}
                else:
                    d.setdefault("pre_grasp", {})["step1_thumb_rotate"] = {"th_rot": s.get("th_rot", 70)}
                    d["pre_grasp"]["step2_fingers"] = {
                        k: s.get(k, 0) for k in ("ff_mcp", "ff_dip", "mf_mcp", "mf_dip")}
                d.setdefault("caution", []).append(
                    time.strftime("%m-%d %H:%M") + " 面板「存为"
                    + ("预抓握" if which == "pre" else "抓握") + "」")
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(d, f, ensure_ascii=False, indent=2)
                hand_api.reload_params()
                saved = d["grasp"]["joints"] if which == "grasp" else d["pre_grasp"]
                self._send_json({"ok": True, "saved": json.dumps(saved, ensure_ascii=False)})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
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
                with _write_lock:           # 任务运行期间手必须保持不动，任何手部动作都禁止
                    writing = _write["status"] in ("running", "stopping")
                if writing:
                    raise RuntimeError("有任务（写字/取笔/点睛等）进行中，禁止手部动作")
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
        elif self.path == "/api/align_pose":
            # 对正写字姿态：保持当前 xyz，rpy 一次性转到 WRITE_RPY（move_ik 重解，
            # 不可达会报错而不是乱动，安全失败）
            if not _move_lock.acquire(blocking=False):
                self._send_json({"error": "上一个运动尚未完成，请稍候"}, 409)
                return
            try:
                arm = get_arm()
                cur = arm.get_pose()
                arm.move_ik(list(cur[:3]) + list(WRITE_RPY), vel=5)
                self._send_json({"ok": True, "pose": arm.get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            finally:
                _move_lock.release()
        elif self.path == "/api/move_z":
            # 第 1 步通路测试：抬/降 5cm（上限 ±50mm，比 jog 微调的 20mm 宽）
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                dz = max(-50.0, min(50.0, float(req["dz"])))
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            if not _move_lock.acquire(blocking=False):
                self._send_json({"error": "上一个运动尚未完成，请稍候"}, 409)
                return
            try:
                arm = get_arm()
                arm.move_rel_ik(dz=dz, vel=DEFAULT_VEL)   # IK+关节空间，避免 MoveL 奇异点
                self._send_json({"ok": True, "pose": arm.get_pose()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            finally:
                _move_lock.release()
        elif self.path == "/api/save_checked":
            # 第 3 步向导的保存：连读 3 次确认臂停稳 + 写盘后回读核对
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                name = str(req["name"]).strip()
                if not name:
                    raise ValueError("航点名不能为空")
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            if not _move_lock.acquire(blocking=False):
                # 有运动在跑时读数必然不稳，直接拒（也算一层防误存）
                self._send_json({"error": "有运动正在进行，不能保存航点"}, 409)
                return
            try:
                spread, verify, pose = _checked_save(get_arm(), name)
                self._send_json({"ok": True, "spread_mm": round(spread, 3),
                                 "verify_mm": round(verify, 3), "pose": pose})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            finally:
                _move_lock.release()
        elif self.path == "/api/flow_start":
            # 第 2/4 步编排流程：缺航点明确报错禁止执行（前端按钮也是灰的）
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                flow = str(req["flow"]).strip()
                if flow not in FLOW_WAYPOINTS:
                    raise ValueError(f"未知流程 {flow!r}（可选 {'/'.join(FLOW_WAYPOINTS)}）")
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            try:
                wps = get_arm().list_waypoints()
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
                return
            missing = [w for w in FLOW_WAYPOINTS[flow] if w not in wps]
            if missing:
                self._send_json({"error": f"缺航点，不能执行{FLOW_LABELS[flow]}："
                                          f"{' '.join(missing)}（先去第 3 步补齐）",
                                 "missing": missing}, 400)
                return
            err = _start_task(flow, _flow_worker, (flow,))
            if err:
                self._send_json({"error": err}, 409)
            else:
                self._send_json({"ok": True, "flow": flow})
        elif self.path == "/api/write_check":
            # 检查字库：逐字查本地缓存（offline，不联网），缺字列出来
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                text = str(req["text"])
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            try:
                seen, chars = set(), []
                for c in text:
                    if c.strip() and c not in seen:
                        seen.add(c)
                        chars.append(c)
                lib = get_stroke_lib()
                missing = [c for c in chars if not lib.has(c)]
                self._send_json({"chars": [{"ch": c, "ok": c not in missing} for c in chars],
                                 "missing": missing, "unique": len(chars)})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/write_preview":
            # 预览：dry_run 纯计算（_PlanOnlyArm），绝不碰硬件
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                result = _write_preview(str(req["text"]),
                                        int(req.get("sheet", 1)),
                                        int(req.get("row", 1)),
                                        int(req.get("col", 1)),
                                        bool(req.get("allow_complex", True)))
                self._send_json(result)
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
            except RuntimeError as e:
                self._send_json({"error": str(e)}, 400)
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
        elif self.path == "/api/write_dip":
            err = _start_write_task("dip")
            if err:
                self._send_json({"error": err}, 409)
            else:
                self._send_json({"ok": True})
        elif self.path == "/api/write_start":
            try:
                length = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(length) or b"{}")
                text = str(req["text"])
                sheet = int(req.get("sheet", 1))
                row = int(req.get("row", 1))
                col = int(req.get("col", 1))
                allow_complex = bool(req.get("allow_complex", True))
            except (KeyError, ValueError, TypeError) as e:
                self._send_json({"error": f"请求参数错误: {e}"}, 400)
                return
            chars = [c for c in text if c.strip()]
            if not chars:
                self._send_json({"error": "诗文为空（空白字符不算）"}, 400)
                return
            try:
                lib = get_stroke_lib()      # 服务端再查一次缺字，不依赖前端先点「检查字库」
                missing = sorted({c for c in chars if not lib.has(c)})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
                return
            if missing:
                self._send_json({"error": "缺字无法书写，需联网预抓或换字："
                                          + " ".join(missing),
                                 "missing": missing}, 400)
                return
            err = _start_write_task("write", text, sheet, row, col, allow_complex)
            if err:
                self._send_json({"error": err}, 409)
            else:
                self._send_json({"ok": True, "chars": len(chars)})
        elif self.path == "/api/write_stop":
            _write_stop.set()
            with _write_lock:
                if _write["status"] == "running":
                    _write["status"] = "stopping"
            _write_log("== 收到停止请求：当前动作完成后停 ==")
            self._send_json({"ok": True})
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
