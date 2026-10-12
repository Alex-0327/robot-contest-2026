#!/usr/bin/env python3
"""写字前空跑校验：笔尖悬在纸面上方描格子边框，肉眼核对标定系是否对准。

用途：换纸/重新标定后、正式书写前，先跑这个脚本。笔尖会悬在纸面上方
HOVER_MM 处，沿田字格区域的四条长边缓慢走直线。标定系若有旋转/平移误差，
笔尖会明显偏离印刷线（长边末端最敏感，1°≈6mm），当场就能看出来。

用法：
    python src/verify_grid.py [--sheet 1] [--dry]

前置：手里已握笔（z 高度是按握笔状态标定的）；纸已用胶带固定。
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from write_engine import (SheetFrame, Grid, WriteEngine,
                          MEAS_CELL_MM, MEAS_COL_PITCH, MEAS_ROW_PITCH,
                          GRID_ROWS, GRID_COLS)

HOVER_MM = 3.0      # 描线时笔尖离纸高度
LIFT_MM = 40.0      # 角点间转移的抬高
TRACE_VEL = 5.0     # 描线速度（慢，方便肉眼跟）
MOVE_VEL = 10.0


def load_grid(sheet, wp_file):
    wp = json.load(open(wp_file, encoding="utf-8"))

    def need(k):
        v = wp[k]
        return v["pose"] if isinstance(v, dict) else v

    fr = SheetFrame(need(f"paper{sheet}_tl"), need(f"paper{sheet}_tr"),
                    need(f"paper{sheet}_bl"), need(f"paper{sheet}_br"),
                    name=f"paper{sheet}")
    sc, cr = need(f"paper{sheet}_cell00_tl"), need(f"paper{sheet}_cell00_corner_tr")
    _, _, interp, dev, center00 = WriteEngine._derive_cell(fr, sc, cr)
    g = Grid(MEAS_CELL_MM[0], MEAS_CELL_MM[1], MEAS_COL_PITCH, MEAS_ROW_PITCH,
             center00, GRID_ROWS, GRID_COLS, "from_calib+pdf_grid")
    return fr, g, interp, dev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sheet", type=int, default=1)
    ap.add_argument("--dry", action="store_true", help="只打印路径，不动机械臂")
    args = ap.parse_args()

    wp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints.json")
    fr, g, interp, dev = load_grid(args.sheet, wp_file)

    print(f"纸面: {fr.width:.1f}x{fr.height:.1f}mm  第4角误差 {fr.corner_err_mm:.2f}mm  "
          f"倾角 {fr.tilt_deg:.2f}°  垂直度误差 {fr.square_err_deg:.2f}°")
    print(f"格子: center00=({g.center00[0]:.1f},{g.center00[1]:.1f})  解释「{interp}」偏差 {dev*100:.1f}%")

    # 整个格区的四条外边框（局部坐标）
    x0 = g.center00[0] - g.cell_w / 2.0
    y0 = g.center00[1] - g.cell_h / 2.0
    x1 = g.center00[0] + (g.cols - 1) * g.col_pitch + g.cell_w / 2.0
    y1 = g.center00[1] + (g.rows - 1) * g.row_pitch + g.cell_h / 2.0
    corners = [("左上", x0, y0), ("右上", x1, y0), ("右下", x1, y1), ("左下", x0, y1)]
    print("描线路径: " + " -> ".join(n for n, _, _ in corners) + " -> 左上")
    for n, x, y in corners:
        print(f"   {n}: 局部({x:.1f},{y:.1f})")
    if args.dry:
        print("--dry，不动机械臂")
        return

    from arm_api import Arm
    arm = Arm()
    try:
        arm.reset_errors()
        arm.set_mode(False)
        arm.enable(True)
        # 到第一个角点上方
        arm.move_ik(fr.point(corners[0][1], corners[0][2], dz_mm=LIFT_MM), vel=MOVE_VEL)
        arm.move_ik(fr.point(corners[0][1], corners[0][2], dz_mm=HOVER_MM), vel=MOVE_VEL)
        for i in range(len(corners)):
            nxt = corners[(i + 1) % len(corners)]
            print(f"描 {corners[i][0]} -> {nxt[0]}，看笔尖是否贴着印刷线走", flush=True)
            arm.move_ik(fr.point(nxt[1], nxt[2], dz_mm=HOVER_MM), vel=TRACE_VEL)
            time.sleep(2.0)
            if i < len(corners) - 1:
                arm.move_ik(fr.point(nxt[1], nxt[2], dz_mm=LIFT_MM), vel=MOVE_VEL)
                arm.move_ik(fr.point(nxt[1], nxt[2], dz_mm=HOVER_MM), vel=MOVE_VEL)
        # 收尾抬高
        p = arm.get_pose()
        arm.move_ik(fr.point(corners[0][1], corners[0][2], dz_mm=LIFT_MM), vel=MOVE_VEL)
        print("校验完成。四条边都贴线 = 标定系正确；哪条边飘 = 重采对应角点")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
