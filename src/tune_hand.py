"""单关节调试器：一次只动一个关节，安全摸索每个关节的可用角度。

用法：
    python src/tune_hand.py        （需先 conda activate robot）

输入格式：关节名 角度，回车执行，如：
    th_mcp 30
输入 q 退出。

关节名一览：
    th_rot 拇指旋转   th_mcp 拇指根   th_dip 拇指尖
    ff_spr 食指横摆   ff_mcp 食指根   ff_dip 食指尖
    mf_mcp 中指根     mf_dip 中指尖
    rf_mcp 无名指根   rf_dip 无名指尖
    lf_mcp 小指根     lf_dip 小指尖

调试纪律（10-11 手指碰撞教训）：
    - 每次只动一个关节，每次增减不超过 10°
    - 动之前目测这个手指的运动路线上有没有其他手指
    - 出现错误码先 clear：输入 c 回车
"""
from pyzlg_dexhand import RightDexHand

VALID = ["th_rot", "th_mcp", "th_dip", "ff_spr", "ff_mcp", "ff_dip",
         "mf_mcp", "mf_dip", "rf_mcp", "rf_dip", "lf_mcp", "lf_dip"]

h = RightDexHand()
h.clear_errors()
print("单关节调试器。输入：关节名 角度（如 th_mcp 30）；c=清错误；q=退出")

while True:
    s = input("> ").strip()
    if s in ("q", "Q"):
        break
    if s == "c":
        h.clear_errors()
        print("错误已清除")
        continue
    parts = s.split()
    if len(parts) != 2 or parts[0] not in VALID:
        print("格式不对。关节名须为:", ", ".join(VALID))
        continue
    try:
        angle = float(parts[1])
    except ValueError:
        print("角度要是数字")
        continue
    h.move_joints(**{parts[0]: angle})
    errors = h.get_errors()
    if any(errors.values()):
        print("出现错误码:", errors)

h.close()
print("已退出")
