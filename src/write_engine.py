"""描红写字轨迹生成 + 执行引擎（决赛「古诗书法」核心模块）。

定位：**只做轨迹生成 + 调用 Arm/Hand 已有接口，绝不碰 SDK 底层**。
能在完全不动机械臂的情况下把整段轨迹算出来给人审（`plan_char` / 命令行默认只读模式）。

依赖（全部复用现有代码）
    arm_api.Arm    move_to / move_ik / move_rel / get_waypoint / goto_waypoint /
                   reset_errors / set_mode / enable
    hand_api.Hand  grasp_pen / open（本模块不主动动手：写字期间手必须保持握笔不动）
    stroke_lib     汉字 → 笔画折线（0~1 归一化的田字格坐标，含真实笔顺）

依赖的航点（src/waypoints.json）
    每张纸 4 个角：paper{1..4}_{tl,tr,bl,br}      ← 建系用；第 4 角做校验（误差>5mm 报错）
    田字格标定：  paper{i}_cell00_tl              ← 第 0 个田字格的【十字中心】或【左上小格中心】
                  paper{i}_cell00_corner_tr       ← 第 0 个田字格的【右上角】
                  （cell00_tl 两种语义都支持，_derive_cell 自动选与图纸最接近的解释；
                   推荐点十字中心——有线可对，比目测小格中心准得多）
    既有：        home / pen_grasp / pen_lift / ink_pre / ink_dip

坐标系约定
    用 tl/tr/bl 三个角建"纸面局部坐标系"：tl 为原点，tr-tl 方向为 X 轴，bl-tl 方向为 Y 轴
    局部坐标单位是 mm（沿纸边的真实距离）；第 4 角 tr+bl-tl 与实测 br 偏差 >5mm 直接报错
    抬落笔一律沿纸面法向（X×Y，强制取 +z 方向），纸稍微倾斜也不怕

已验证的坑，代码里都守着（别删）
    1. 跨区移动必须三段式：原地抬高 → 平移 → 下降，禁止点对点直飞
    2. 翻转姿态（笔尖朝下写字）下【升降】一律 move_ik，不用 MoveL
    3. 运动前 reset_errors() + set_mode(False) + enable(True)；报错看错误码
       （14=未复位报警，99/112=奇异点，185=超工作空间，186=运动中碰撞停止）
    4. CNDE 状态流只允许一个客户端，本程序运行时不要开别的脚本
    5. 重启控制柜后激活的工具坐标系会重置 → check_ready() 会打印 tool/wobj 让你看一眼
    6. 写字 vel≤5（本模块 3），转场 vel≤10（本模块 10）

用法（默认只读出方案，人工审核后才允许上机）
    python src/write_engine.py --char 日月山水火田 --sheet 1 --row 1 --col 1
    python src/write_engine.py --char 日 --sheet 1 --row 1 --col 1 --execute   # 真写
    python src/write_engine.py --list-cells                                    # 打印格心
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_WP_FILE = os.path.join(HERE, "waypoints.json")

# ============================================================================
# 常量：现场就调这几个
# ============================================================================
RETRACT_MM = 5.0          # 笔画间抬笔高度（任务书要求 5mm 换笔）
SAFE_HOP_MM = 50.0        # 进入/离开一个字时的高度（任务书要求先到落点上方 5cm）
TRANSIT_Z_MM = 280.0      # 进字转场高度：已高于此就不再抬（墨碟上方 z≈291，再抬 50 不可达 err=38）
SHORT_MOVE_MM = 30.0      # 笔画间距小于此值：就在 5mm 高度直接平移，不来回升降
                          # （12→30：5mm 平移模式 10-12 已验收；字内多数笔距 <30mm，
                          #  少一次 50mm 慢速落笔每字省 ~3-6s）
INTRA_HOP_MM = 20.0       # 字内远笔画（>SHORT_MOVE_MM）的转场高度：仍走「抬-移-落」三段式，
                          # 但 50→20mm。5cm 是任务书对【进字】的要求，字内无此约束；
                          # 纸面平坦、上方无障碍，落笔慢速段缩短 30mm 每处省 ~1-2s
WRITE_VEL = 4.0           # 走笔速度（≤5；3→4：10-12 整纸压测 347s 后提速）
TRACE_BLEND_MM = 1.2      # 走笔 MoveL 转角平滑半径（非阻塞连发，10-11 write_api 已验证模式）；
                          # 折线顶点不停顿：省每段 ~0.2-0.4s 阻塞开销，拐角也不积墨。设 0 退回逐段阻塞
TRACE_CHUNK = 8           # 每连发多少段 MoveL 等一次（与 write_api.chunk 一致，防爆队列）
DESCEND_VEL = 5.0         # 落笔/抬笔垂直速度（≤5，走 move_ik）
TRANSIT_VEL = 14.0        # 转场速度（10→14：走廊高空段很安全，提速压总时长）
CORNER_TOL_MM = 5.0       # 第 4 角校验误差上限
PRESS_MM = -4.0           # 落笔下压量（正=往纸里压，负=抬高）。10-12 实测 PRESS=0 墨色太深 → 抬 4mm
SHEET_MM = (420.0, 297.0)  # A3 横向（4 张拼成 A1 阵列）

# 官方《方格.pdf》实测值（mm，A3 放大打印）：没有标定航点时用它兜底
MEAS_CELL_MM = (76.86, 70.52)
MEAS_COL_PITCH = 80.20
MEAS_ROW_PITCH = 108.28
MEAS_MARGIN_LR = 11.44
MEAS_MARGIN_TB = 59.08
GRID_ROWS, GRID_COLS = 2, 5

# 暂只支持单体字：先用这批字把流程验通，多部件字拆完再放开
SIMPLE_CHARS = ("日月山水火田一二三人口木禾竹土王上下中大小干工川牛午天元目白玉石立"
                "丁十千左右寸尺牛羊古老米来")


# ============================================================================
# 纸面局部坐标系
# ============================================================================
class SheetFrame:
    """单张纸的局部坐标系：3 角建系 + 第 4 角校验。"""

    def __init__(self, tl, tr, bl, br, tol_mm=CORNER_TOL_MM, name="sheet"):
        self.name = name
        self.origin = np.asarray(tl[:3], dtype=float)
        p_tr, p_bl, p_br = (np.asarray(p[:3], dtype=float) for p in (tr, bl, br))
        vx, vy = p_tr - self.origin, p_bl - self.origin
        self.width = float(np.linalg.norm(vx))
        self.height = float(np.linalg.norm(vy))
        if self.width < 1e-6 or self.height < 1e-6:
            raise RuntimeError(f"[{name}] tl/tr 或 tl/bl 重合了，角点肯定没点对")
        self.ex, self.ey = vx / self.width, vy / self.height
        n = np.cross(self.ex, self.ey)
        n /= np.linalg.norm(n)
        if n[2] < 0:                                   # 法向强制朝上
            n = -n
        self.n = n
        self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(n[2])))))
        self.square_err_deg = 90.0 - math.degrees(
            math.acos(max(-1.0, min(1.0, abs(float(np.dot(self.ex, self.ey)))))))
        # 第 4 角校验：tl + (tr-tl) + (bl-tl) 应当等于 br
        self.corner_err_mm = float(np.linalg.norm(self.origin + vx + vy - p_br))
        if self.corner_err_mm > tol_mm:
            raise RuntimeError(
                f"[{name}] 第 4 角校验失败：由 tl/tr/bl 推出来的角点与实测 br 差 "
                f"{self.corner_err_mm:.1f}mm > {tol_mm}mm。某个角点没点准，或纸不平/不是矩形"
                f" → 重采 {name}_tl/tr/bl/br")
        self.rpy = tuple(float(v) for v in tl[3:6])

    def point(self, x_mm, y_mm, dz_mm=0.0):
        """局部 (x,y) + 沿法向抬 dz → 完整位姿。"""
        p = self.origin + x_mm * self.ex + y_mm * self.ey + dz_mm * self.n
        return [float(v) for v in p] + list(self.rpy)

    def air(self, pose, dz_mm):
        """把一个点沿纸面法向抬高 dz（不要裸加 +z，纸有倾角时那样会偏）。"""
        p = np.asarray(pose[:3], dtype=float) + dz_mm * self.n
        return [float(v) for v in p] + list(self.rpy)

    def to_local(self, pose):
        """base 位姿 → 局部平面坐标 (x,y)（由标定航点反算格心用）。"""
        d = np.asarray(pose[:3], dtype=float) - self.origin
        A = np.stack([self.ex, self.ey], axis=1)
        xy, *_ = np.linalg.lstsq(A, d, rcond=None)
        return float(xy[0]), float(xy[1])

    def offset_from(self, base_point, dx_mm, dy_mm):
        """从已知 base 点出发沿纸面基走 (dx,dy) mm。

        ★ 千万别写 base_x + dx：纸一旦有旋转，直接相加就会把字写歪。"""
        p = np.asarray(base_point[:3], dtype=float) + dx_mm * self.ex + dy_mm * self.ey
        return [float(v) for v in p] + list(self.rpy)

    def summary(self):
        return {"sheet": self.name, "size_mm": [round(self.width, 1), round(self.height, 1)],
                "corner_err_mm": round(self.corner_err_mm, 2),
                "tilt_deg": round(self.tilt_deg, 2), "square_err_deg": round(self.square_err_deg, 2)}


# ============================================================================
# 田字格几何
# ============================================================================
class Grid:
    def __init__(self, cell_w, cell_h, col_pitch, row_pitch, center00, rows, cols, source):
        self.cell_w, self.cell_h = cell_w, cell_h
        self.col_pitch, self.row_pitch = col_pitch, row_pitch
        self.center00 = center00
        self.rows, self.cols = rows, cols
        self.source = source

    def center(self, row, col):
        """row/col 从 1 开始。"""
        if not (1 <= row <= self.rows and 1 <= col <= self.cols):
            raise RuntimeError(f"格号越界：只有 {self.rows} 行 × {self.cols} 列，收到 r{row}c{col}")
        return (self.center00[0] + (col - 1) * self.col_pitch,
                self.center00[1] + (row - 1) * self.row_pitch)

    def summary(self):
        return {"cell_mm": [round(self.cell_w, 2), round(self.cell_h, 2)],
                "col_pitch": round(self.col_pitch, 2), "row_pitch": round(self.row_pitch, 2),
                "grid": f"{self.rows}x{self.cols}",
                "center00_mm": [round(v, 2) for v in self.center00], "source": self.source}


# ============================================================================
# 引擎
# ============================================================================
class WriteEngine:
    def __init__(self, arm, hand=None, waypoint_file=None, allow_complex=False, verbose=True):
        self.arm = arm
        self.hand = hand
        self.verbose = verbose
        self.allow_complex = allow_complex
        self.wp_file = waypoint_file or getattr(arm, "waypoint_file", DEFAULT_WP_FILE)
        self.wp = self._load_waypoints()
        self.lib = self._load_stroke_lib()
        self._frames, self._grids = {}, {}
        self._ready = False

    # ---------------------------------------------------------------- 载入
    def _load_waypoints(self):
        if not os.path.exists(self.wp_file):
            raise RuntimeError(f"航点文件不存在：{self.wp_file}（先跑 teach_waypoints.py 采点）")
        with open(self.wp_file, encoding="utf-8") as f:
            return json.load(f)

    def _load_stroke_lib(self):
        """笔画数据源：优先 stroke_lib（有缓存/可联网补字），退回直接读缓存 JSON。"""
        try:
            from stroke_lib import StrokeLib
            return StrokeLib(offline=True)
        except Exception:                                       # noqa: BLE001
            cache = os.path.join(ROOT, "data", "strokes_cache.json")
            if not os.path.exists(cache):
                raise RuntimeError(
                    "缺笔画数据源：没找到 src/stroke_lib.py，也没有 data/strokes_cache.json。"
                    "先跑 python src/stroke_lib.py --fetch 日月山水火田")
            with open(cache, encoding="utf-8") as f:
                data = json.load(f)

            class _CacheLib:
                def __init__(self, d):
                    self.d = d.get("chars", {})

                def has(self, ch):
                    return ch in self.d

                def strokes(self, ch, rebuild=False):
                    return [[tuple(p) for p in s] for s in self.d.get(ch, [])]
            print(f"[笔画] 直接读缓存 {cache}（{len(data.get('chars', {}))} 字）")
            return _CacheLib(data)

    def _need(self, name):
        if name not in self.wp:
            raise RuntimeError(f"航点缺失：{name}。当前 waypoints.json 有 {sorted(self.wp)}"
                               f"（纸阵四角命名规则 paper{{1-4}}_{{tl,tr,bl,br}}）")
        p = self.wp[name]
        if isinstance(p, dict):          # 新格式 {"pose": [...], "joints": [...]} → 取 pose
            p = p.get("pose")
        if not isinstance(p, list) or len(p) != 6:
            raise RuntimeError(f"航点 {name} 不是 6 维位姿：{self.wp[name]}")
        return p

    # ---------------------------------------------------------------- 纸面 / 格
    def frame(self, sheet=1):
        if sheet not in self._frames:
            pts = [self._need(f"paper{sheet}_{k}") for k in ("tl", "tr", "bl", "br")]
            fr = SheetFrame(pts[0], pts[1], pts[2], pts[3], name=f"paper{sheet}")
            if abs(fr.width - SHEET_MM[0]) > 40 or abs(fr.height - SHEET_MM[1]) > 40:
                print(f"★ [paper{sheet}] 纸 {fr.width:.0f}×{fr.height:.0f}mm 与 A3"
                      f"({SHEET_MM[0]:.0f}×{SHEET_MM[1]:.0f}) 差太多，确认角点对不对")
            if fr.tilt_deg > 3.0:
                print(f"★ [paper{sheet}] 纸面倾角 {fr.tilt_deg:.1f}°，台面/纸没放平？")
            if fr.square_err_deg > 4.0:
                print(f"★ [paper{sheet}] 两轴不垂直 {fr.square_err_deg:.1f}°，角点点歪了")
            self._frames[sheet] = fr
        return self._frames[sheet]

    def grid(self, sheet=1):
        """格心几何：有标定航点就实算（按官方实测比例缩放），没有就用官方实测值兜底。"""
        if sheet in self._grids:
            return self._grids[sheet]
        fr = self.frame(sheet)
        sc_name = f"paper{sheet}_cell00_tl"           # 第 0 个田字格的左上小格中心
        cr_name = f"paper{sheet}_cell00_corner_tr"    # 第 0 个田字格的右上角
        if sc_name in self.wp and cr_name in self.wp:
            sc, cr = self._need(sc_name), self._need(cr_name)   # 走 _need 解包新格式 dict
            cell_w, cell_h, interp, dev, center00 = self._derive_cell(fr, sc, cr)
            print(f"   [paper{sheet}] 标定实算：格 {cell_w:.2f}×{cell_h:.2f}mm（按「{interp}」反推，"
                  f"与图纸实测偏差 {dev * 100:.1f}%）")
            # 纸角实测 = A3 标准 420mm → 打印比例 1:1，格尺寸/格距直接用图纸值；
            # 标定反推的格宽偏小（十字中心目测误差 2-3mm），列距误差逐列累积会把
            # 后几列顶出格（10-12 整纸压测：第一行后两字靠左出格）。
            # 标定点只用来定 center00（第 0 格中心锚点），不用来定格尺寸。
            g = Grid(MEAS_CELL_MM[0], MEAS_CELL_MM[1], MEAS_COL_PITCH, MEAS_ROW_PITCH,
                     center00, GRID_ROWS, GRID_COLS, "from_calib+pdf_grid")
        else:
            cell_w, cell_h = MEAS_CELL_MM
            center00 = (MEAS_MARGIN_LR + cell_w / 2.0, MEAS_MARGIN_TB + cell_h / 2.0)
            print(f"★ [paper{sheet}] 缺标定航点 {sc_name}/{cr_name} → 用官方图纸实测兜底"
                  f"（格 {cell_w}×{cell_h}，留边 {MEAS_MARGIN_LR}/{MEAS_MARGIN_TB}）。"
                  f"强烈建议补采这两个点：纸一摆歪，兜底值整体就偏")
            g = Grid(cell_w, cell_h, MEAS_COL_PITCH, MEAS_ROW_PITCH,
                     center00, GRID_ROWS, GRID_COLS, "from_pdf")
        # 所有格必须落在纸里（留 10mm 容差）：这一步能挡住"格距算错/角点点歪"
        last = g.center(g.rows, g.cols)
        over_x = (last[0] + g.cell_w / 2) - fr.width
        over_y = (last[1] + g.cell_h / 2) - fr.height
        if over_x > 10 or over_y > 10:
            raise RuntimeError(
                f"[paper{sheet}] 末格 r{g.rows}c{g.cols} 跑到纸外了："
                f"右边界超出 {over_x:.1f}mm、下边界超出 {over_y:.1f}mm"
                f"（纸 {fr.width:.0f}×{fr.height:.0f}mm，格 {g.cell_w:.1f}×{g.cell_h:.1f}，"
                f"行/列距 {g.row_pitch:.1f}/{g.col_pitch:.1f}）→ 查格距标定点和纸角点")
        self._grids[sheet] = g
        return g

    @staticmethod
    def _derive_cell(fr, sub_center_pose, corner_pose):
        """由【标定点】+【右上角点】反推田字格大小，并给出第 0 格田字格中心。

        cell00_tl 的语义现场容易点错（目测"小格中心"没有线可对，10-12 实测点成
        十字中心的概率很大），所以三种解释都算一遍，取与官方图纸实测值最接近的。
        返回 (cell_w, cell_h, 解释名, 偏差, center00局部坐标)。"""
        sx, sy = fr.to_local(sub_center_pose)
        cx, cy = fr.to_local(corner_pose)
        dx, dy_up = cx - sx, sy - cy                 # 中心→右边框 / 中心→上边框
        cands = [("小格中心→田字格右上角", 4.0 / 3.0 * dx, 4.0 * dy_up),
                 ("小格中心→小格右上角", 4.0 * dx, 4.0 * dy_up),
                 ("田字格十字中心→田字格右上角", 2.0 * dx, 2.0 * dy_up)]
        scored = []
        for name, w, h in cands:
            if w > 5 and h > 5:
                dev = max(abs(w / MEAS_CELL_MM[0] - 1.0), abs(h / MEAS_CELL_MM[1] - 1.0))
                c00 = (sx, sy) if name.startswith("田字格") else (sx + w / 4.0, sy + h / 4.0)
                scored.append((dev, name, w, h, c00))
        if not scored:
            raise RuntimeError(
                f"标定关系对不上：两个点反推出的格尺寸是负数/零（dx={dx:.1f}, dy_up={dy_up:.1f}）。"
                f"检查 cell00_tl（十字中心或小格中心）、cell00_corner_tr（右上角点）")
        scored.sort(key=lambda t: t[0])
        dev, name, w, h, c00 = scored[0]
        if dev > 0.30:
            print(f"★ 格尺寸反推与图纸差 {dev * 100:.0f}%（{name}）→ 先复核这两个标定点的位置定义")
        return w, h, name, dev, c00

    def cell_center_pose(self, sheet=1, row=1, col=1):
        fr, g = self.frame(sheet), self.grid(sheet)
        x, y = g.center(row, col)
        return fr.point(x, y, dz_mm=-PRESS_MM)      # 负 = 往纸里压 PRESS_MM

    # ---------------------------------------------------------------- 规划
    def strokes_of(self, ch):
        if not self.allow_complex and ch not in SIMPLE_CHARS:
            raise RuntimeError(
                f"「{ch}」不在单体字白名单里（当前 {len(set(SIMPLE_CHARS))} 个字）。"
                f"确认字形没问题就传 allow_complex=True 放行")
        st = self.lib.strokes(ch)
        if not st:
            raise RuntimeError(f"「{ch}」取不到笔画：缓存里没有这个字 → "
                               f"python src/stroke_lib.py --fetch {ch}")
        return st

    def plan_char(self, ch, sheet=1, row=1, col=1):
        """纯计算：把「写这个字」变成一串 base 位姿。不动机械臂。"""
        fr, g = self.frame(sheet), self.grid(sheet)
        base_cell = self.cell_center_pose(sheet, row, col)
        # 字形自动居中：笔画库的字在 0~1 框里不一定居中（如「日」偏左下 1.3/2.4mm），
        # 按本字 bbox 中心对齐格心，消掉系统性偏移（10-12 实测「日」偏左下）
        all_uv = np.array([p for poly in self.strokes_of(ch) for p in poly])
        cu, cv = (all_uv.min(axis=0) + all_uv.max(axis=0)) / 2.0
        plan, total = [], 0.0
        for poly in self.strokes_of(ch):
            pts = [fr.offset_from(base_cell, (u - cu) * g.cell_w, (v - cv) * g.cell_h)
                   for u, v in poly]
            a = np.asarray([p[:3] for p in pts])
            length = float(np.hypot(*np.diff(a, axis=0).T).sum()) if len(a) > 1 else 0.0
            plan.append({"points": pts, "length_mm": round(length, 2),
                         "local_start": [round(v, 1) for v in fr.to_local(pts[0])],
                         "local_end": [round(v, 1) for v in fr.to_local(pts[-1])]})
            total += length
        return {"char": ch, "sheet": sheet, "row": row, "col": col,
                "cell_mm": [round(g.cell_w, 2), round(g.cell_h, 2)],
                "press_mm": PRESS_MM, "strokes": plan, "total_mm": round(total, 2)}

    def print_plan(self, plan):
        print(f"── 计划：写「{plan['char']}」于 paper{plan['sheet']} r{plan['row']}c{plan['col']}"
              f"（格 {plan['cell_mm'][0]}×{plan['cell_mm'][1]}mm，下压 {plan['press_mm']}mm）──")
        print(f"   {len(plan['strokes'])} 笔，墨迹总长 {plan['total_mm']:.1f}mm")
        for i, s in enumerate(plan["strokes"], 1):
            p0, p1 = s["points"][0], s["points"][-1]
            print(f"   笔{i}/{len(plan['strokes'])}  {len(s['points'])}点 长{s['length_mm']:6.1f}mm  "
                  f"起({p0[0]:7.1f},{p0[1]:7.1f},{p0[2]:7.1f}) → 终({p1[0]:7.1f},{p1[1]:7.1f},{p1[2]:7.1f})")

    # ---------------------------------------------------------------- 运动
    def check_ready(self):
        """运动前标准动作 + 报警/工具号检查。有未复位报警不许开跑。"""
        self.arm.reset_errors()
        self.arm.set_mode(False)             # False = 自动模式
        self.arm.enable(True)
        try:
            err, pkg = self.arm.robot.GetRobotRealTimeState()
        except Exception:                    # noqa: BLE001
            print("★ 读不到控制器状态（CNDE 没起来？）→ 按 SOP 重连/复位后再来")
            return
        if err != 0:
            raise RuntimeError(f"读取控制器状态失败 err={err}")
        if getattr(pkg, "main_code", 0) != 0:
            raise RuntimeError(f"控制器有未复位报警 main_code={pkg.main_code}（SOP：先复位再运动）")
        print(f"控制器就绪 main_code=0  tool={getattr(pkg, 'tool_id', self.arm.tool)} "
              f"wobj={getattr(pkg, 'wobj_id', self.arm.user)}（tool 必须和你标定用的一致）")
        self._ready = True

    def _move(self, pose, vel, why):
        """直线运动：先 MoveL，撞奇异点(99/112)自动转 move_ik；两个都失败必须抛出来。"""
        try:
            self.arm.move_to(pose, vel=vel)
            return
        except RuntimeError as e:
            print(f"  [运动] MoveL 失败（{e}）→ 复位故障后改走 move_ik")
            self.arm.reset_errors()   # MoveL 报错会锁存故障，不复位后续全报 err=14
            time.sleep(1)
            self.arm.enable(True)
        try:
            self.arm.move_ik(pose, vel=vel)
        except Exception as e:                                 # noqa: BLE001
            raise RuntimeError(f"{why} 失败：MoveL 与 move_ik 都走不通 → {e}") from e

    def _trace(self, pts, why):
        """走笔：MoveL 带 blendR 平滑半径【非阻塞连发】，一笔只在末尾等一次，
        省掉逐段阻塞的通讯/等待开销（645 点/20 字，逐段阻塞是最大的隐性耗时）。
        任一段下发失败（IK 预检不过）就排空队列、复位后把剩余段退回逐段阻塞老路
        （保留 _move 的 move_ik 奇异点兜底），绝不少写一段。"""
        if len(pts) < 2:
            return
        if TRACE_BLEND_MM <= 0:
            for j, p in enumerate(pts[1:], 1):
                self._move(p, WRITE_VEL, f"{why}第{j}段")
            return
        segs = pts[1:]
        sent = 0
        try:
            for i in range(0, len(segs), TRACE_CHUNK):
                grp = segs[i:i + TRACE_CHUNK]
                for k, p in enumerate(grp):
                    last = (i + k) == len(segs) - 1          # 末段不混叠：精确停笔再抬
                    err = self.arm.robot.MoveL(list(p), self.arm.tool, self.arm.user,
                                               vel=WRITE_VEL, ovl=100.0,
                                               blendR=(0.0 if last else TRACE_BLEND_MM))
                    if err != 0:
                        raise RuntimeError(f"MoveL 下发 err={err}")
                    sent += 1
                self.arm.wait_done()
            return
        except Exception as e:                                 # noqa: BLE001
            print(f"  [走笔] 混合连发中断（{e}）→ 排空队列后退回逐段阻塞")
            try:
                self.arm.wait_done()
            except Exception:                                  # noqa: BLE001
                pass
            self.arm.reset_errors()
            self.arm.enable(True)
            for j, p in enumerate(segs[sent:], sent + 1):
                self._move(p, WRITE_VEL, f"{why}第{j}段(阻塞兜底)")

    def _vertical(self, pose, why, vel=DESCEND_VEL):
        """垂直升降一律 move_ik（翻腕姿态下 MoveL 会撞奇异点）。"""
        try:
            self.arm.move_ik(pose, vel=vel)
        except Exception as e:                                 # noqa: BLE001
            raise RuntimeError(f"{why} 失败（垂直升降 move_ik）→ {e}") from e

    # ---------------------------------------------------------------- 执行
    def write_char(self, ch, sheet=1, row=1, col=1, dry_run=False):
        """写一个字。dry_run=True 只打印计划不动硬件。"""
        fr = self.frame(sheet)
        plan = self.plan_char(ch, sheet, row, col)
        self.print_plan(plan)
        if dry_run:
            return plan
        self.check_ready()          # 每个字开写前都做一遍：复位 + 自动模式 + 使能 + 查报警

        prev_end = None
        for i, s in enumerate(plan["strokes"], 1):
            pts = s["points"]
            start, end = pts[0], pts[-1]
            print(f"[写] 第 {i}/{len(plan['strokes'])} 笔：{len(pts)} 点 / {s['length_mm']:.1f}mm "
                  f"起 ({start[0]:.1f},{start[1]:.1f},{start[2]:.1f})")
            if prev_end is None:
                # 进字：①原地抬到转场高度（保持当前姿态：墨碟位置用写字姿态超限；
                # 已经在高位就不抬——继续硬抬会撞顶部位姿不可达 err=38，10-12 实测）
                # ②平移到落点上方 ③下降（三段式，禁止直飞）
                cur = self.arm.get_pose()
                if cur[2] < TRANSIT_Z_MM - 1.0:
                    lifted = list(cur); lifted[2] = TRANSIT_Z_MM
                    self._vertical(lifted, "进入前抬到转场高度", vel=TRANSIT_VEL)
                # 绕笔筒走廊：笔挂在 TCP 下约 18cm，墨碟→纸直飞会在笔筒顶部剐蹭
                # （10-12 实测撞笔筒）。只有从远处（墨碟侧）进字才走走廊；
                # 格间短距转移不绕（10-12 连写「日月」：每字绕走廊浪费 ~10s）
                entry = fr.air(start, SAFE_HOP_MM)
                far = math.dist(cur[:2], entry[:2]) > 200.0
                if far:
                    for cn in ("ink_corridor_1", "ink_corridor_2"):
                        if cn in self.wp:
                            self.arm.move_ik(self._need(cn), vel=TRANSIT_VEL)
                            print(f"  [转场] 过走廊点 {cn}")
                self._move(entry, TRANSIT_VEL, "平移到落点上方 5cm")
            else:
                d = math.dist(start[:2], prev_end[:2])
                if d <= SHORT_MOVE_MM:                  # 相邻笔画：保持 5mm 高度平移
                    self._move(fr.air(start, RETRACT_MM), TRANSIT_VEL, "笔画间 5mm 平移")
                else:                                   # 字内远笔画：抬到 20mm 再平移
                    self._vertical(fr.air(prev_end, INTRA_HOP_MM), "跨笔画抬笔", vel=TRANSIT_VEL)
                    self._move(fr.air(start, INTRA_HOP_MM), TRANSIT_VEL, "平移到下一笔上方")
            self._vertical(start, f"第{i}笔落笔")        # ③下降
            self._trace(pts, f"第{i}笔")
            self._vertical(fr.air(end, RETRACT_MM), f"第{i}笔抬笔")
            prev_end = list(end)
        print(f"[写] 「{ch}」完成：{len(plan['strokes'])} 笔 / {plan['total_mm']:.0f}mm")
        return plan

    def write_text(self, text, sheet=1, start_row=1, start_col=1, dry_run=False):
        """按阅读顺序写字：写满一行换行，写完一页翻下一张纸。"""
        g = self.grid(sheet)
        row, col, cur_sheet = start_row, start_col, sheet
        for ch in text:
            if not ch.strip():
                continue
            if col > g.cols:
                row += 1
                col = 1
                if row > g.rows:
                    cur_sheet += 1
                    row = 1
                    self._need(f"paper{cur_sheet}_tl")   # 提前炸掉，别写到一半才发现没示教
                    g = self.grid(cur_sheet)
                    print(f"→ 本页写完，翻到 paper{cur_sheet}")
            self.write_char(ch, cur_sheet, row, col, dry_run=dry_run)
            col += 1


# ============================================================================
# 命令行：默认只读出方案；加 --execute 才真写（且要求人工确认）
# ============================================================================
class _PlanOnlyArm:
    """不连硬件也能出方案：只提供航点文件读取，所有运动直接报错。"""

    def __init__(self, wp_file):
        self.waypoint_file = wp_file
        self.tool, self.user = 1, 0
        self.robot = None

    def reset_errors(self):
        pass

    def set_mode(self, manual):
        pass

    def enable(self, on):
        pass

    def move_to(self, pose, vel=20.0, blocking=True):
        raise RuntimeError("只读模式：未连接机械臂")

    def move_ik(self, pose, vel=10.0, blocking=True):
        raise RuntimeError("只读模式：未连接机械臂")

    def get_pose(self):
        raise RuntimeError("只读模式：未连接机械臂")

    def close(self):
        pass


def main():
    ap = argparse.ArgumentParser(description="描红写字：轨迹生成 + 执行")
    ap.add_argument("--char", default="日月山水火田", help="要写的字；多个字按阅读顺序排到相邻格")
    ap.add_argument("--sheet", type=int, default=1)
    ap.add_argument("--row", type=int, default=1)
    ap.add_argument("--col", type=int, default=1)
    ap.add_argument("--waypoints", default="")
    ap.add_argument("--allow-complex", action="store_true", help="放行非单体字")
    ap.add_argument("--list-cells", action="store_true", help="打印所有格心 base 坐标")
    ap.add_argument("--execute", action="store_true", help="★真机执行（默认只读）")
    args = ap.parse_args()

    wp_file = args.waypoints or DEFAULT_WP_FILE
    if not os.path.isabs(wp_file):
        wp_file = os.path.join(ROOT, wp_file)

    if args.execute:
        from arm_api import Arm
        from hand_api import Hand
        arm, hand = Arm(waypoint_file=wp_file), Hand()
        print("★ 真机模式（已连 arm/hand）：请先确认手已经握稳笔、笔尖朝下")
        if input("输入 y 回车开始写，其它任意输入取消: ").strip().lower() != "y":
            print("已取消")
            return 0
    else:
        arm, hand = _PlanOnlyArm(wp_file), None
        print("只读模式：只出方案，不动机械臂（加 --execute 才真写）")

    try:
        eng = WriteEngine(arm, hand, waypoint_file=wp_file, allow_complex=args.allow_complex)
    except RuntimeError as e:
        print(f"★ 初始化失败：{e}")
        return 1

    print(f"航点文件: {wp_file}")
    print(f"  home={'有' if 'home' in eng.wp else '缺'}  "
          f"pen_grasp={'有' if 'pen_grasp' in eng.wp else '缺'}  "
          f"ink_dip={'有' if 'ink_dip' in eng.wp else '缺'}")

    for s in range(1, 5):
        try:
            fr = eng.frame(s)
            print(f"paper{s}: {fr.summary()}")
            print(f"   grid: {eng.grid(s).summary()}")
        except RuntimeError as e:
            print(f"paper{s}: 未就绪 —— {e}")

    if args.list_cells:
        for s in range(1, 5):
            try:
                g = eng.grid(s)
                fr = eng.frame(s)
            except RuntimeError:
                continue
            print(f"\npaper{s} 格心：")
            for r in range(1, g.rows + 1):
                for c in range(1, g.cols + 1):
                    x, y = g.center(r, c)
                    print(f"  r{r}c{c}  局部({x:7.2f},{y:7.2f})  base={[round(v, 1) for v in fr.point(x, y)]}")

    try:
        eng.write_text(args.char, args.sheet, args.row, args.col, dry_run=not args.execute)
    except RuntimeError as e:
        print(f"★ 中止：{e}")
        return 1
    finally:
        if args.execute:
            arm.close()
            if hand:
                hand.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
