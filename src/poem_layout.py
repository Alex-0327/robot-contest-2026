"""纸面几何与写字计划 —— 把"第几行第几列"翻译成机械臂 base 坐标下的位姿。

三块内容：
  1) 田字格实测尺寸（来源：官方矢量文件 方格.pdf，见下）；A3 放大打印 vs 1:1 打印两套坐标
  2) 纸面坐标系：用示教的 3 个纸角点建立局部基（u 沿纸宽、v 沿纸高、n 指向纸面上方），
     之后任何"纸内 mm 坐标"都能换算成 base 位姿 —— 纸摆得略有旋转/倾斜都不怕
  3) 写字计划：诗歌文本 → 每字对应哪个格子（纸内 mm）→ 逐字的可达性检查

坐标约定
    纸内坐标：原点 = 纸的【左上角】，x 向右（沿纸宽 420mm），y 向下（沿纸高 297mm）
    田字格：每张 A3 有 2 行 × 5 列 = 10 格，1 格 1 字
    4 张 A3 组成 2×2 阵列（=A1 大小）：backL/backR/frontL/frontR

用法：
    python src/poem_layout.py --sheet backL --row 2            # 打印第 2 行 5 个格心
    python src/poem_layout.py --sheet backL --row 2 --dry-run  # 不带纸角点时只能看纸内坐标
"""
from __future__ import annotations

import argparse
import math
import json
import os
import sys

import numpy as np

# ============================================================================
# 1. 官方田字格的实测尺寸（单位 mm）
# ============================================================================
# 原始文件 方格.pdf 是 A4 横向 297x210，通知要求 A3 打印 → 放大 √2 = 1.41421 倍
SHEET_MM = (420.0, 297.0)            # A3 横向（放大后）
ARRAY_MM = (840.0, 594.0)            # 2x2 无间隙 = A1

LAYOUTS = {
    # A3 放大打印（默认）：格 76.86 x 70.52
    "a3_scaled": {
        "cell_mm": (76.86, 70.52),
        "col_centers_x": [49.87, 130.09, 209.59, 289.82, 370.03],
        "row_centers_y": [94.36, 202.59],
    },
    # 若发现是 1:1 打印（内容只占 A3 左上角，量一格 ≈54.4x49.9 就是这种）：
    "a4_raw": {
        "cell_mm": (54.35, 49.87),
        "col_centers_x": [35.26, 91.99, 148.20, 204.93, 261.65],
        "row_centers_y": [66.72, 143.25],
    },
}
SHEET_OFFSET = {                     # 每张纸在阵列里的左上角偏移
    "backL": (0.0, 0.0), "backR": (SHEET_MM[0], 0.0),
    "frontL": (0.0, SHEET_MM[1]), "frontR": SHEET_MM,
}


# ============================================================================
# 2. 纸面坐标系（3 点建系）
# ============================================================================
def entry_pose(entry):
    """兼容航点两种格式：新格式 {"pose": [...], "joints": [...]} 取 pose；旧格式就是 6 维列表。"""
    return entry["pose"] if isinstance(entry, dict) else entry


class Paper:
    """由 3 个示教点建立的纸面坐标系。

    示教要求（重点，错这里全盘皆错）：
        握着笔把【笔尖】分别点到目标 A3 纸的 左上角 / 右上角 / 左下角，保存为
        paper_a / paper_b / paper_c（纸张边缘即可，不用对准格线）。
        三个点必须在同一张纸上，且笔的姿态一致（用写字姿态去点）。
    """

    def __init__(self, p_a, p_b, p_c, sheet_mm=SHEET_MM, rpy=None):
        self.a = np.asarray(p_a, dtype=float)
        self.b = np.asarray(p_b, dtype=float)
        self.c = np.asarray(p_c, dtype=float)
        self.sheet_mm = sheet_mm
        if self.a.size >= 6:                       # 示教点自带姿态 → 写字姿态沿用它
            self.rpy = tuple(float(v) for v in self.a[3:6])
        elif rpy is not None:
            self.rpy = tuple(float(v) for v in rpy)
        else:
            raise ValueError("示教点只有位置：请传 6 维位姿，或显式给 rpy=(rx,ry,rz)")
        pa, pb, pc = self.a[:3], self.b[:3], self.c[:3]
        self.u = (pb - pa) / sheet_mm[0]           # 每 1mm 纸内 x 对应的 base 位移
        self.v = (pc - pa) / sheet_mm[1]
        n = np.cross(self.u, self.v)
        norm = float(np.linalg.norm(n))
        if norm < 1e-9:
            raise ValueError("三个示教点共线，无法建立纸面坐标系")
        self.n = n / norm
        if self.n[2] < 0:                          # 保证法向朝上（+z）
            self.n = -self.n
        self.tilt_deg = self._tilt()
        self.square_err_deg = self._square_err()

    # ---- 自检 ----
    def _tilt(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, float(self.n[2])))))

    def _square_err(self):
        u = self.u / np.linalg.norm(self.u)
        v = self.v / np.linalg.norm(self.v)
        return 90.0 - math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, v))))))

    def size_check(self):
        """返回实际示教跨度（mm），用于判断纸角点是不是点歪了。"""
        return {"u_span_mm": round(float(np.linalg.norm(self.b[:3] - self.a[:3])), 1),
                "v_span_mm": round(float(np.linalg.norm(self.c[:3] - self.a[:3])), 1),
                "expected_mm": self.sheet_mm,
                "tilt_deg": round(self.tilt_deg, 2),
                "square_err_deg": round(self.square_err_deg, 2)}

    def point(self, x_mm, y_mm, dz=0.0):
        """纸内 (x, y) 且沿法向抬 dz → base 坐标 [x, y, z]。"""
        p = self.a[:3] + x_mm * self.u + y_mm * self.v + dz * self.n
        return [float(v) for v in p]

    def pose(self, x_mm, y_mm, dz=0.0):
        """纸内 (x, y) → base 位姿 [x,y,z,rx,ry,rz]（姿态沿用示教点）。"""
        p = self.point(x_mm, y_mm, dz)
        return p + list(self.rpy)

    def offset_from(self, base_point, dx_mm, dy_mm):
        """从一个已知 base 点出发，沿纸面基走 (dx_mm, dy_mm)（写字时"格内偏移"用这个）。

        ★ 不要用 base_x + dx 这种写法：纸一旦有旋转，xy 直接相加就会把字写歪。"""
        p = np.asarray(base_point, dtype=float)[:3] + dx_mm * self.u + dy_mm * self.v
        return [float(v) for v in p]


# ============================================================================
# 3. 写字计划：文本 → 格子
# ============================================================================
def cell_local_mm(sheet, row, col, layout="a3_scaled"):
    """第 row 行第 col 列（1 起）格心在【该张纸】内的 mm 坐标。"""
    lay = LAYOUTS[layout]
    return (lay["col_centers_x"][col - 1], lay["row_centers_y"][row - 1])


def cell_origin_mm(sheet, row, col, layout="a3_scaled"):
    """格心相对【阵列左上角】的 mm 坐标（含这张纸在阵列里的偏移）。"""
    x, y = cell_local_mm(sheet, row, col, layout)
    ox, oy = SHEET_OFFSET[sheet]
    return (ox + x, oy + y)


def make_plan(text, sheet="backL", row=2, col_start=1, layout="a3_scaled",
              max_cols=5, allow_wrap=True):
    """把文本排到格子上，返回 [{"char","sheet","row","col","x_mm","y_mm","cell_mm"}, ...]。

    排版规则：同一行从左到右填满 5 格；折行默认只往【同一张纸的另一行】走，
    纸用完了就停（不会擅自跑到够不着的纸上）。规则要求"连续 5 字"才有加分，
    所以宁可一行写满，也不要东一个西一个。
    """
    items = []
    lay = LAYOUTS[layout]
    n_col, n_row = len(lay["col_centers_x"]), len(lay["row_centers_y"])
    row_cycle = [row] + [r for r in range(1, n_row + 1) if r != row]
    ri, ci = 0, col_start
    for ch in text:
        if not ch.strip():
            continue
        if ci > n_col:
            if not allow_wrap or ri + 1 >= len(row_cycle):
                break
            ri += 1
            ci = 1
        r = row_cycle[ri]
        x, y = cell_origin_mm(sheet, r, ci, layout)
        items.append({"char": ch, "sheet": sheet, "row": r, "col": ci,
                      "x_mm": round(x, 2), "y_mm": round(y, 2),
                      "cell_mm": lay["cell_mm"]})
        ci += 1
        if len(items) >= max_cols * n_row:
            break
    return items


def reachability(items, paper: Paper, arm_base_mm=(0.0, 0.0), band_mm=(150.0, 700.0)):
    """逐格算到臂基座的水平距离（base 系里基座就在原点），标出太近/太远的格子。

    太近（<150mm）不是够不到，而是腕关节容易超限、姿态别扭；太远（>700mm）可能真伸不到。
    这两个阈值按 FR3（臂展 622mm）给的保守值，实测后按 PLAN 里的数据改。"""
    bx, by = arm_base_mm
    out = []
    for it in items:
        p = paper.point(it["x_mm"], it["y_mm"])
        d = math.hypot(p[0] - bx, p[1] - by)
        flag = "OK"
        if d < band_mm[0]:
            flag = "★太近：腕关节易超限，建议先空跑试姿态"
        elif d > band_mm[1]:
            flag = "★可能够不到"
        out.append((it["char"], it["sheet"], it["row"], it["col"], round(d, 1), flag))
    return out


# ============================================================================
# 4. 命令行：只看几何，不动硬件
# ============================================================================
def main():
    ap = argparse.ArgumentParser(description="纸面几何 / 写字计划（纯计算，不连硬件）")
    ap.add_argument("--sheet", default="backL", choices=list(SHEET_OFFSET))
    ap.add_argument("--row", type=int, default=2)
    ap.add_argument("--col", type=int, default=1)
    ap.add_argument("--text", default="", help="给了就按文本排版，否则只打印该行的格心")
    ap.add_argument("--layout", default="a3_scaled", choices=list(LAYOUTS))
    ap.add_argument("--waypoints", default="", help="waypoints.json 路径（有就给 base 坐标+可达性）")
    args = ap.parse_args()

    if args.text:
        items = make_plan(args.text, args.sheet, args.row, args.col, args.layout)
        print(f"排版：{len(items)} 字")
        for it in items:
            print(f"  {it['char']}  {it['sheet']} r{it['row']}c{it['col']}  纸内({it['x_mm']:.1f}, {it['y_mm']:.1f})mm")
    else:
        print(f"{args.sheet} 第 {args.row} 行格心（纸内 mm，{args.layout}）：")
        for c in range(1, 6):
            x, y = cell_local_mm(args.sheet, args.row, c, args.layout)
            xa, ya = cell_origin_mm(args.sheet, args.row, c, args.layout)
            print(f"  c{c}: 纸内({x:7.2f}, {y:7.2f})   阵列({xa:7.2f}, {ya:7.2f})")

    if args.waypoints and os.path.exists(args.waypoints):
        with open(args.waypoints, encoding="utf-8") as f:
            wp = json.load(f)
        need = ["paper_a", "paper_b", "paper_c"]
        if all(k in wp for k in need):
            paper = Paper(entry_pose(wp["paper_a"]), entry_pose(wp["paper_b"]), entry_pose(wp["paper_c"]))
            print("\n纸面坐标系自检:", paper.size_check())
            print("笔的姿态(rx,ry,rz) =", paper.rpy)
            if args.text:
                print("\n可达性（距臂基座水平距离）:")
                for row in reachability(make_plan(args.text, args.sheet, args.row, args.col, args.layout), paper):
                    print("  ", row)
                print("\nbase 坐标:")
                for it in make_plan(args.text, args.sheet, args.row, args.col, args.layout):
                    print(f"  {it['char']}  {[round(v, 1) for v in paper.pose(it['x_mm'], it['y_mm'])]}")
        else:
            print(f"\n★ 还没示教完整：需要 {need}，当前有 {list(wp)}")


if __name__ == "__main__":
    main()
