#!/usr/bin/env python3
"""安全存航点：连读 3 次位姿交叉验证，读数稳定才落盘。

背景：CNDE 状态流偶发垃圾值，单次读数直接保存可能存进错误坐标
（10-12 赛场：页面存角点，复到时 xy 偏 10cm+）。本工具绕开页面，
SSH 直连控制柜读数，三次读数最大偏差超阈值就拒绝保存。

用法：
    python src/save_wp.py <航点名> [--tol 1.0]

只读位姿+关节角并写文件，不发任何运动/模式指令，webapp 手动/拖动示教时可用。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm_api import Arm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="航点名，如 paper1_tl")
    ap.add_argument("--tol", type=float, default=1.0, help="三次读数最大允许偏差(mm/°)")
    args = ap.parse_args()

    arm = Arm()
    try:
        reads = []
        for _ in range(3):
            reads.append(arm.get_pose())
            time.sleep(0.8)
        dev = max(abs(a - b) for r in reads[1:] for a, b in zip(reads[0], r))
        for i, r in enumerate(reads):
            print(f"读数{i + 1}: {[round(v, 2) for v in r]}")
        print(f"最大偏差: {dev:.2f}（阈值 {args.tol}）")
        if dev > args.tol:
            print("!! 读数不稳定，未保存。机械臂可能还在动/被扶着，停稳后重试")
            sys.exit(1)
        arm.save_waypoint(args.name)
        print(f"OK [{args.name}] 已落盘")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
