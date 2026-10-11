"""毛笔书写执行层 —— 分层架构第 3 层：把"一个字"落到纸面上。

依赖下面两层（都已就绪）：
    arm_api.Arm   机械臂原语（MoveL/MoveJ/逆解/航点）
    hand_api.Hand 灵巧手（握笔姿态，写字期间保持不动）
本层只负责：抬笔/落笔、按折线走笔、蘸墨、时间熔断、以及"机器人也会失误"的兜底。

三个关键设计（都是现场踩出来的）：
1) ★ 落笔不能靠固定关节角。纸面高度会变（纸厚/换纸/治具），落笔深度必须由纸面坐标系
   算出来（poem_layout.Paper），再 MoveL 下探。本层的落笔就是这么做的。
2) ★ 翻腕姿态下笛卡尔 MoveL 可能报 err=99/112（奇异点），所以直线运动统一走 _movel()：
   先试 MoveL，失败自动退到逆解 + MoveJ，仍失败抛异常（绝不静默吞掉）。
3) ★ 参数从 data/write_plan.json 的 motion 段读进来，不要在代码里写死：
   现场调的就是"抬笔多高、落笔多慢、走笔多快"，这些数字直接决定 300s 能写几个字。

时间账（10-11 实测估算，务必按自己机器复算）：
    每笔的固定开销 ≈ 抬笔高度/下降速度 + 抬笔高度/抬起速度 + 点位开销
    8mm/12mm·s⁻¹ + 8mm/30mm·s⁻¹ + 0.15s ≈ 1.1s/笔
    54 笔 ≈ 59s；墨迹 1900mm/15mm·s⁻¹ ≈ 127s；格间空移 ≈ 35s；蘸墨 4 次 ≈ 36s → 约 257s
    ⇒ 300s 预算是"10 字刚好、12 字必超"。想多写就压蘸墨次数、提 write_vel。

用法（一般由 demo_write_poem.py 调用）：
    from write_api import Writer
    w = Writer(arm, hand, lib, motion=plan["motion"])
    w.load_paper_from_waypoints(arm.get_waypoint)
    w.write_char("一", w.cell_pose(x_mm, y_mm), cell_mm)
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

# 默认参数（会被 plan["motion"] 覆盖；这几个值是 10-11 晚按"300s 写 10 字"倒推的实战档）
DEFAULTS = {
    "clearance_mm": 8.0,          # 笔画间抬笔高度（够跨过墨迹，又不像 15mm 那样费时间）
    "travel_clearance_mm": 30.0,  # 跨格/换行抬笔高度
    "descend_vel": 12.0,          # 落笔下降速度 mm/s
    "lift_vel": 30.0,             # 抬笔速度
    "write_vel": 15.0,            # 走笔速度 mm/s（>20 会甩墨、笔画变虚）
    "travel_vel": 40.0,           # 空移速度
    "blend_mm": 1.2,              # MoveL 转角过渡半径：折线拐角不出现停顿积墨
    "press_mm": 0.3,              # 落笔后额外下压量（墨淡加大，晕染减小）
    "chunk": 8,                   # 一笔内连续发多少条 MoveL 后等一次
}


class MoveError(RuntimeError):
    pass


class Writer:
    def __init__(self, arm, hand, lib, motion=None, verbose=True):
        self.arm = arm
        self.hand = hand
        self.lib = lib
        self.verbose = verbose
        for k, v in DEFAULTS.items():
            setattr(self, k, (motion or {}).get(k, v))
        self.paper = None
        self._blend_ok = True                 # SDK 是否支持 blendR/ovl 关键字
        self.ink_mm_since_dip = 0.0           # 距上次蘸墨走过的墨迹长度（mm）
        self.strokes_since_dip = 0
        self.t0 = time.time()

    # ------------------------------------------------------------------ 基础
    def log(self, msg):
        if self.verbose:
            print(msg)
        sys.stdout.flush()

    def elapsed(self):
        return time.time() - self.t0

    def reset_clock(self):
        self.t0 = time.time()

    def _movel(self, target, vel, blend=None, wait=True):
        """直线运动，带奇异点兜底。target 为 6 维位姿。"""
        blend = self.blend_mm if blend is None else blend
        r = self.arm.robot
        if self._blend_ok:
            try:
                err = r.MoveL(list(target), self.arm.tool, self.arm.user,
                              vel=vel, ovl=100.0, blendR=blend)
            except TypeError:                 # 老版 SDK 没有 blendR/ovl 关键字
                self._blend_ok = False
                err = r.MoveL(list(target), self.arm.tool, self.arm.user, vel=vel)
        else:
            err = r.MoveL(list(target), self.arm.tool, self.arm.user, vel=vel)
        if err != 0:
            # 常见 err=99/112：目标不可达或经过奇异点 → 退到逆解 + 关节空间
            self.log(f"  [write] MoveL err={err}，改走逆解 MoveJ")
            self.arm.move_ik(list(target), vel=vel, blocking=wait)
            return
        if wait:
            self.arm.wait_done()

    # ------------------------------------------------------------------ 纸面
    def load_paper_from_waypoints(self, getter, sheet_mm=(420.0, 297.0)):
        """从示教航点 paper_a / paper_b / paper_c 建立纸面坐标系。"""
        from poem_layout import Paper
        p = [getter(k) for k in ("paper_a", "paper_b", "paper_c")]
        self.paper = Paper(p[0], p[1], p[2], sheet_mm=sheet_mm)
        self.log(f"[write] 纸面坐标系：{self.paper.size_check()}")
        if self.paper.tilt_deg > 3.0:
            self.log(f"[write] ★ 纸面倾角 {self.paper.tilt_deg:.1f}° 偏大，台面/纸没放平？")
        if abs(self.paper.square_err_deg) > 4.0:
            self.log(f"[write] ★ 三点不垂直（{self.paper.square_err_deg:.1f}°），纸角点是不是点歪了？")
        return self.paper

    def cell_pose(self, x_mm, y_mm):
        """阵列坐标（mm）→ 笔尖触纸的 base 位姿。"""
        if self.paper is None:
            raise MoveError("还没建立纸面坐标系：先 load_paper_from_waypoints()")
        return self.paper.pose(x_mm, y_mm, dz=-self.press_mm)

    def cell_pose_local(self, sheet, row, col, layout="a3_scaled"):
        """第几张纸 / 第几行 / 第几列 → base 位姿。"""
        from poem_layout import SHEET_OFFSET, cell_local_mm
        ox, oy = SHEET_OFFSET[sheet]
        x, y = cell_local_mm(sheet, row, col, layout)
        return self.cell_pose(ox + x, oy + y)

    # ------------------------------------------------------------------ 蘸墨
    def dip(self, dip_pose, hover_pose, depth_mm=3.0, dwell_s=1.2, drip_s=0.8):
        """蘸墨：到墨面正上方 → 慢速下潜 depth_mm → 停 dwell_s → 抬起 → 悬停 drip_s 防滴。

        墨碟/磨台是固定件，直接示教 ink_hover（笔尖离墨面约 20mm）和 ink_dip（笔尖刚触墨面）。"""
        self.log(f"[write] 蘸墨（距上次 {self.strokes_since_dip} 笔 / {self.ink_mm_since_dip:.0f}mm）")
        self._movel(list(hover_pose), vel=self.travel_vel, blend=0.0)
        target = list(dip_pose)
        target[2] -= abs(depth_mm)                 # 沿 base z 下潜（墨面水平，够用）
        self._movel(target, vel=max(3.0, self.descend_vel / 2), blend=0.0)
        time.sleep(dwell_s)
        self._movel(list(dip_pose), vel=6.0, blend=0.0)
        self._movel(list(hover_pose), vel=self.travel_vel, blend=0.0)
        time.sleep(drip_s)
        if hasattr(self.hand, "check"):
            self.hand.check()
        self.ink_mm_since_dip = 0.0
        self.strokes_since_dip = 0

    # ------------------------------------------------------------------ 写字
    def stroke(self, points_base):
        """写一笔：走到起点上方 → 落笔 → 沿折线走完 → 抬笔。"""
        if len(points_base) < 2:
            return 0.0
        start = list(points_base[0])
        air = list(start)
        air[2] += self.clearance_mm
        self._movel(air, vel=self.travel_vel, blend=0.0)          # 空移到起点上方
        self._movel(start, vel=self.descend_vel, blend=0.0)       # 落笔
        for i in range(1, len(points_base), self.chunk):
            grp = points_base[i:i + self.chunk]
            for j, p in enumerate(grp):
                last = (i + j) == len(points_base) - 1
                self._movel(list(p), vel=self.write_vel,
                            blend=(0.0 if last else self.blend_mm), wait=False)
            self.arm.wait_done()
        end_air = list(points_base[-1])
        end_air[2] += self.clearance_mm
        self._movel(end_air, vel=self.lift_vel, blend=0.0)        # 抬笔
        a = np.asarray(points_base, dtype=float)
        length = float(np.hypot(*np.diff(a, axis=0).T).sum())
        self.ink_mm_since_dip += length
        self.strokes_since_dip += 1
        return length

    def write_char(self, ch, cell_pose, cell_mm, char_index=None, total=None):
        """把一个字写到指定格心。返回该字实际墨迹长度（mm）。"""
        strokes = self.lib.strokes(ch)
        if not strokes:
            self.log(f"[write] ★ {ch} 没有笔画数据，跳过")
            return 0.0
        cw, chh = cell_mm
        base = list(cell_pose[:3])
        if not all(math.isfinite(v) for v in base):
            raise MoveError(f"{ch} 的格心位姿非法：{cell_pose}")
        tag = f"{ch}" + (f" ({char_index}/{total})" if char_index else "")
        self.log(f"[write] {tag}: {len(strokes)} 笔")
        total_len = 0.0
        for si, s in enumerate(strokes, 1):
            # 格内 0~1 坐标 → 纸内 mm → base；用纸面基换算，纸有旋转也不会写歪
            if self.paper is not None:
                pts = [self.paper.offset_from(base, x * cw, y * chh) for x, y in s]
            else:
                pts = [[base[0] + x * cw, base[1] + y * chh, base[2]] for x, y in s]
            total_len += self.stroke(pts)
        self.log(f"  → {tag} 完成，墨迹 {total_len:.0f}mm，已写 {self.strokes_since_dip} 笔未蘸墨")
        return total_len

    def travel_to_cell(self, cell_pose):
        """跨格移动：抬到跨格高度再走，避免蹭到刚写的字。"""
        air = list(cell_pose[:3])
        air[2] += self.travel_clearance_mm
        self._movel(air + list(cell_pose[3:]), vel=self.travel_vel, blend=0.0)
