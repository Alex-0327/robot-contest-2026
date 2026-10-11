"""决赛"古诗书法"主程序 —— 抓笔蘸墨之后，把诗写到田字格里。

★ 2026-10-11 晚：写字执行统一走 **`src/write_engine.py`**（航点命名是
  paper{1-4}_{tl,tr,bl,br}，与现场采集一致）。本文件保留给**离线预演/工时估算**
  （它自己用的 paper_a/b/c 命名与 write_engine 不同，别混着用）。

前置状态（本脚本不负责）：
    ✅ 笔已经在灵巧手里握稳（照 demo_cycle.py 走完：抓笔 → 抬升 → J6 翻转 180°）
    ✅ 田字格纸已铺好并用镇纸压住（或治具定位）
    ✅ 已示教 5 个航点：paper_a / paper_b / paper_c / ink_hover / ink_dip
    ✅ 手是握笔姿态（本脚本全程不动手，避免写到一半把笔松了）

跑法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/demo_write_poem.py            # 正式跑
    PYTHONPATH=... python src/demo_write_poem.py --dry-run                                 # 只算不跑（先跑这个！）
    PYTHONPATH=... python src/demo_write_poem.py --chars 5                                 # 只写 5 个字
    PYTHONPATH=... python src/demo_write_poem.py --plan data/write_plan.json --yes          # 跳过确认

安全约定：
    - 每写一个字前后都查一次控制器报警（main_code），有报警立即停手退出
    - 剩余时间低于 guard_s 就不再起新字，抬笔收尾（规则：超时 -50 分，且超时部分不计分）
    - 任何异常都会先把笔抬起来再退出，不会把笔停在纸上
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from stroke_lib import StrokeLib               # noqa: E402
from poem_layout import Paper, make_plan, reachability  # noqa: E402
from write_api import DEFAULTS                 # noqa: E402

DEFAULT_PLAN = os.path.join(ROOT, "data", "write_plan.json")


def load_plan(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def build_items(plan, args):
    text = plan["poem"]
    if args.text:
        text = args.text
    if args.chars:
        text = text[:args.chars]
    return make_plan(text, args.sheet or plan["sheet"], args.row or plan["row"],
                     plan.get("col_start", 1), args.layout or plan.get("layout", "a3_scaled"),
                     allow_wrap=bool(plan.get("allow_wrap", False)))


def estimate(plan, items, lib):
    """粗估耗时（秒）：每笔固定开销 + 走笔 + 格间空移 + 蘸墨。

    ★ 参数与执行层共用 write_api.DEFAULTS，避免"估算说够、现场写不完"。"""
    m = {**DEFAULTS, **plan.get("motion", {})}
    total_ink, total_strokes = 0.0, 0
    for it in items:
        s = lib.stats(it["char"])
        total_strokes += s["strokes"]
        total_ink += s["ink_len"] * it["cell_mm"][0]        # 归一化笔长 → mm
    per_stroke = (m["clearance_mm"] / m["descend_vel"] + m["clearance_mm"] / m["lift_vel"]
                  + m.get("point_overhead_s", 0.15))
    write_s = total_ink / m["write_vel"]
    travel_s = len(items) * 0.5 + len(items) * (it_pitch(plan) / m["travel_vel"])
    dip = plan["dip"]
    n_dip = max(1, int(round(total_strokes / max(1, dip["every_strokes"])))) if dip.get("pre_dip", True) else 0
    dip_s = n_dip * dip.get("dip_cost_s", 9.0)
    return {
        "chars": len(items), "strokes": total_strokes, "ink_mm": round(total_ink, 1),
        "write_s": round(write_s, 1), "stroke_overhead_s": round(total_strokes * per_stroke, 1),
        "travel_s": round(travel_s, 1), "dip_count": n_dip, "dip_s": round(dip_s, 1),
        "total_s": round(write_s + total_strokes * per_stroke + travel_s + dip_s, 1),
    }


def it_pitch(plan):
    from poem_layout import LAYOUTS
    lay = LAYOUTS[plan.get("layout", "a3_scaled")]
    return lay["col_centers_x"][1] - lay["col_centers_x"][0]


def main():
    ap = argparse.ArgumentParser(description="决赛：古诗书法")
    ap.add_argument("--plan", default=DEFAULT_PLAN)
    ap.add_argument("--text", default="", help="临时换文本（按顺序填格）")
    ap.add_argument("--chars", type=int, default=0, help="只写前 N 个字")
    ap.add_argument("--sheet", default="", help="backL/backR/frontL/frontR")
    ap.add_argument("--row", type=int, default=0)
    ap.add_argument("--layout", default="", choices=["", "a3_scaled", "a4_raw"])
    ap.add_argument("--waypoints", default="", help="waypoints.json 路径（默认用 arm_api 的）")
    ap.add_argument("--suffix", default="", help="航点草稿后缀，等价于 --waypoints src/waypoints_<后缀>.json")
    ap.add_argument("--dry-run", action="store_true", help="只算位姿和工时，不连硬件")
    ap.add_argument("--yes", action="store_true", help="不等待人工确认")
    args = ap.parse_args()

    plan = load_plan(args.plan)
    items = build_items(plan, args)
    if not items:
        print("★ 计划里没有字，检查 data/write_plan.json 的 poem 字段")
        return 1

    lib = StrokeLib(offline=True)     # 现场不联网：全部走 data/strokes_cache.json
    est = estimate(plan, items, lib)

    print(f"=== 书写计划：{len(items)} 字（{plan.get('poem_title', '')}）===")
    for it in items:
        st = lib.stats(it["char"])
        flag = "" if st["strokes"] else "   ★ 无笔画数据！"
        print(f"  {it['char']}  {it['sheet']} r{it['row']}c{it['col']}"
              f"  阵列({it['x_mm']:.1f}, {it['y_mm']:.1f})mm  {st['strokes']}笔{flag}")
    print(f"\n预估工时：{est['total_s']:.0f}s / 预算 {plan['time_budget_s']}s"
          f"（走笔 {est['write_s']:.0f}s + 落抬笔 {est['stroke_overhead_s']:.0f}s"
          f" + 空移 {est['travel_s']:.0f}s + 蘸墨 {est['dip_count']}次 {est['dip_s']:.0f}s）")
    limit = plan["time_budget_s"] - plan["guard_s"]
    if est["total_s"] > limit:
        print(f"★ 预估 {est['total_s']:.0f}s 已吃掉熔断余量（上限 {limit:.0f}s）：先减字"
              f"（--chars N）或提 write_vel，别到现场才发现写不完")
    else:
        print(f"→ 剩余 {limit - est['total_s']:.0f}s 余量，够用来处理突发（重蘸墨/补一笔）")

    # ---------------- 读航点 ----------------
    if args.waypoints:
        wp_file = args.waypoints
    elif args.suffix:
        wp_file = os.path.join(HERE, f"waypoints_{args.suffix}.json")
    else:
        wp_file = plan.get("waypoint_file") or os.path.join(HERE, "waypoints.json")
    wp_file = wp_file if os.path.isabs(wp_file) else os.path.join(ROOT, wp_file)
    if not os.path.exists(wp_file):
        print(f"★ 找不到航点文件 {wp_file}：先跑 teach_waypoints.py 示教")
        return 1
    with open(wp_file, encoding="utf-8") as f:
        wp = json.load(f)
    need = ["paper_a", "paper_b", "paper_c"]
    missing = [k for k in need if k not in wp]
    if missing:
        print(f"★ 缺少航点 {missing}（需要在纸的三个角各示教一次），当前只有 {list(wp)}")
        return 1
    paper = Paper(wp["paper_a"], wp["paper_b"], wp["paper_c"])
    print("\n纸面坐标系自检:", paper.size_check())
    band = plan.get("reachability", {}).get("band_mm", (150.0, 700.0))
    for row in reachability(items, paper, band_mm=band):
        print("  ", row)
    for it in items[:3]:
        print(f"  {it['char']} base ≈ {[round(v, 1) for v in paper.pose(it['x_mm'], it['y_mm'])]}")

    if args.dry_run:
        print("\n[dry-run] 只算不跑，结束")
        return 0

    # ---------------- 真机 ----------------
    from arm_api import Arm
    from hand_api import Hand
    from write_api import Writer

    print(f"\n航点文件: {wp_file}")
    if not args.yes:
        print("即将开始书写。确认：① 纸已铺好压住  ② 笔已握在手里  ③ 手保持握笔姿态不动")
        if input("回车开始，输入其它任意字符取消: ").strip():
            print("已取消")
            return 0

    arm = Arm(waypoint_file=wp_file)
    hand = Hand()
    arm.reset_errors()
    arm.enable(True)
    hand.check()

    writer = Writer(arm, hand, lib, motion=plan["motion"])
    writer.load_paper_from_waypoints(wp.__getitem__)

    dip = plan["dip"]
    budget = plan["time_budget_s"]
    guard = plan["guard_s"]
    done = 0
    aborted = False
    try:
        if dip.get("pre_dip", True):
            writer.dip(wp["ink_dip"], wp["ink_hover"], dip["depth_mm"], dip["dwell_s"], dip["drip_s"])
        writer.reset_clock()
        for i, it in enumerate(items, 1):
            left = budget - writer.elapsed()
            if left < guard:
                print(f"\n★ 剩余 {left:.0f}s < 熔断阈值 {guard}s → 停止起新字，抬笔收尾")
                aborted = True
                break
            if (writer.strokes_since_dip >= dip["every_strokes"]
                    or (dip.get("every_mm") and writer.ink_mm_since_dip >= dip["every_mm"])):
                if left - dip["dip_cost_s"] > guard:
                    writer.dip(wp["ink_dip"], wp["ink_hover"], dip["depth_mm"],
                               dip["dwell_s"], dip["drip_s"])
                else:
                    print("  （时间不够，跳过这次蘸墨）")
            pose = writer.cell_pose(it["x_mm"], it["y_mm"])
            writer.travel_to_cell(pose)                 # 抬到跨格高度再过去，别蹭到刚写的字
            writer.write_char(it["char"], pose, it["cell_mm"], i, len(items))
            done += 1
            err, pkg = arm.robot.GetRobotRealTimeState()
            if err == 0 and pkg.main_code != 0:
                print(f"★ 控制器报警 main={pkg.main_code}，立即停止")
                aborted = True
                break
    except Exception as e:                                  # noqa: BLE001
        print(f"★ 异常中断：{type(e).__name__}: {e}")
        aborted = True
    finally:
        # 无论如何先把笔抬起来，再收尾
        try:
            p = arm.get_pose()
            up = list(p)
            up[2] += 80
            arm.move_ik(up, vel=10)
            if not aborted and plan.get("return_home") and "home" in wp:
                arm.move_to(wp["home"], vel=20)
            print(f"\n收尾完成，笔已抬起。当前位姿 {[round(v, 1) for v in arm.get_pose()]}")
        except Exception as e:                              # noqa: BLE001
            print(f"★ 收尾抬笔失败（手动接管！）：{e}")
        print(f"本轮写完 {done}/{len(items)} 字，用时 {writer.elapsed():.0f}s")
        arm.close()
        hand.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
