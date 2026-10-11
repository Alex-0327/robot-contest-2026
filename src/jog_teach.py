"""键盘点动示教：不依赖拖动示教模式，用键盘把臂一步步挪到目标位并保存航点。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/jog_teach.py

按键（输入后回车）：
    w/s  x 前进/后退      a/d  y 左/右       r/f  z 升/降
    i/k  绕x俯仰          j/l  绕y偏摆       u/o  绕z旋转
    1/2/3  步长切为 1mm / 5mm / 20mm（姿态固定 2°）
    p    打印当前位姿
    直接输入名字（如 pen_grasp）回车 = 保存当前位姿为航点
    q    退出

接近目标时记得把步长切到 1。臂周清空，急停在手边。
"""
from arm_api import Arm
from hand_api import Hand

KEYMAP = {
    "w": (1, +1), "s": (1, -1),
    "a": (2, +1), "d": (2, -1),
    "r": (3, +1), "f": (3, -1),
    "i": (4, +1), "k": (4, -1),
    "j": (5, +1), "l": (5, -1),
    "u": (6, +1), "o": (6, -1),
}
STEPS = {"1": 1, "2": 5, "3": 20}

arm = Arm()
arm.enable(True)

ans = input("先让手进入握笔预成形？(y/n): ").strip().lower()
if ans == "y":
    hand = Hand()
    hand.pre_grasp_pen()
    print("预成形完成")

step = 5
print("当前步长 %dmm。开始点动（h 查看按键帮助）" % step)

while True:
    cmd = input("> ").strip().lower()
    if cmd == "q":
        break
    if cmd == "h":
        print(__doc__)
        continue
    if cmd == "p":
        print("位姿:", [round(x, 1) for x in arm.get_pose()])
        continue
    if cmd in STEPS:
        step = STEPS[cmd]
        print("步长 -> %dmm" % step)
        continue
    if cmd in KEYMAP:
        axis, direction = KEYMAP[cmd]
        arm.jog(axis, direction, step if axis <= 3 else 2.0)
        continue
    if cmd:
        arm.save_waypoint(cmd)

arm.close()
print("已退出，航点保存在 src/waypoints.json")
