"""书写预演 —— 不碰硬件，把整首诗按真实格距画出来 + 估算工时。

为什么必须先跑这个：
  1. 看字形对不对（字帖风格、笔画是否连着、有没有超出田字格）
  2. 看工时够不够（300s 能写几个字，一眼定字数）
  3. 出图留档，赛前答辩/复盘直接拿去用

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/preview_poem.py
    PYTHONPATH=... python src/preview_poem.py --chars 5 --out out/preview_5chars.png
    PYTHONPATH=... python src/preview_poem.py --text 一去二三里 --show   # 弹窗看（需图形界面）
"""
from __future__ import annotations

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文标题要能显示：把系统里常见的中文字体排前面（否则 matplotlib 只认 DejaVu，出方块）
matplotlib.rcParams["font.sans-serif"] = [
    "PingFang SC", "Heiti SC", "STHeiti", "Hiragino Sans GB",   # macOS
    "Microsoft YaHei", "SimHei",                                 # Windows
    "Noto Sans CJK SC", "WenQuanYi Zen Hei", "Source Han Sans SC",  # Linux
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.patches import Rectangle
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from stroke_lib import StrokeLib                 # noqa: E402
from poem_layout import LAYOUTS, SHEET_OFFSET    # noqa: E402
from demo_write_poem import load_plan, estimate, DEFAULT_PLAN   # noqa: E402


def draw(plan, items, lib, out_path):
    lay = LAYOUTS[plan.get("layout", "a3_scaled")]
    cw, ch = lay["cell_mm"]
    fig, ax = plt.subplots(figsize=(4 + 1.15 * len(items), 4.2))
    for it in items:
        ox = it["x_mm"] - cw / 2.0
        oy = it["y_mm"] - ch / 2.0
        ax.add_patch(Rectangle((ox, oy), cw, ch, fill=False, lw=1.0, ec="#e0b3b3"))
        # 田字格十字虚线（浅灰，和官方格线一个意思）
        ax.plot([ox + cw / 2, ox + cw / 2], [oy, oy + ch], lw=0.5, ls=":", color="#efcaca")
        ax.plot([ox, ox + cw], [oy + ch / 2, oy + ch / 2], lw=0.5, ls=":", color="#efcaca")
        for k, poly in enumerate(lib.strokes(it["char"])):
            a = np.asarray(poly, dtype=float)
            xs = ox + a[:, 0] * cw
            ys = oy + a[:, 1] * ch
            ax.plot(xs, ys, "-", color="#1a1a1a", lw=3.4, solid_capstyle="round")
            ax.annotate(str(k + 1), (xs[0], ys[0]), fontsize=5, color="#c0392b")
        ax.text(ox + cw / 2, oy - 3.0, f"r{it['row']}c{it['col']}", ha="center", fontsize=6, color="#888")

    ax.set_aspect("equal")
    ax.invert_yaxis()                     # 纸内 y 向下，和实际看纸一致
    ax.set_title(f"{plan.get('poem_title', '')}  {len(items)} 字 / 纸面 {plan['sheet']}",
                 fontsize=10)
    ax.set_xlabel("纸内 x (mm)")
    ax.set_ylabel("纸内 y (mm)")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print(f"预览图已保存: {out_path}")


def main():
    ap = argparse.ArgumentParser(description="书写预演（不连硬件）")
    ap.add_argument("--plan", default=DEFAULT_PLAN)
    ap.add_argument("--text", default="")
    ap.add_argument("--chars", type=int, default=0)
    ap.add_argument("--sheet", default="")
    ap.add_argument("--row", type=int, default=0)
    ap.add_argument("--layout", default="", choices=["", "a3_scaled", "a4_raw"])
    ap.add_argument("--out", default="")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    plan = dict(load_plan(args.plan))
    if args.text:
        plan["poem"] = args.text
    if args.sheet:
        plan["sheet"] = args.sheet
    if args.row:
        plan["row"] = args.row
    if args.layout:
        plan["layout"] = args.layout
    if args.chars:
        plan["poem"] = plan["poem"][:args.chars]

    from poem_layout import make_plan
    items = make_plan(plan["poem"], plan["sheet"], plan["row"], plan.get("col_start", 1),
                      plan.get("layout", "a3_scaled"),
                      allow_wrap=bool(plan.get("allow_wrap", False)))
    lib = StrokeLib(offline=True)
    est = estimate(plan, items, lib)

    print(f"=== {plan.get('poem_title', '')} ===")
    for it in items:
        s = lib.stats(it["char"])
        print(f"  {it['char']} r{it['row']}c{it['col']}  {s['strokes']:>2} 笔 / 笔长 {s['ink_len']:.2f} 格")
    print(f"\n预估工时 {est['total_s']:.0f}s（预算 {plan['time_budget_s']}s，熔断留 {plan['guard_s']}s）")
    print(f"  写 {est['chars']} 字 / {est['strokes']} 笔 / 墨迹总长 {est['ink_mm']:.0f}mm"
          f" | 走笔 {est['write_s']:.0f}s + 落抬笔 {est['stroke_overhead_s']:.0f}s"
          f" + 空移 {est['travel_s']:.0f}s + 蘸墨 {est['dip_count']} 次 {est['dip_s']:.0f}s")
    if est["total_s"] > plan["time_budget_s"] - plan["guard_s"]:
        print("★ 超预算：减字 / 提 write_vel / 减少蘸墨次数")
    miss = [c for c in plan["poem"] if c.strip() and not lib.has(c)]
    if miss:
        print(f"★ 缓存里没有这些字：{miss} → 先跑 python src/stroke_lib.py --fetch {plan['poem']}")

    out = args.out or os.path.join(ROOT, "out", f"preview_{plan['poem'][:8]}.png")
    draw(plan, items, lib, out)
    if args.show:
        import subprocess
        subprocess.run(["open" if sys.platform == "darwin" else "xdg-open", out], check=False)


if __name__ == "__main__":
    main()
