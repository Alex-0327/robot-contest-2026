"""拖动示教记录航点：把固定位置（笔、墨碟、纸面）存进 waypoints.json。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/teach_waypoints.py [后缀]

带后缀（如 test）时航点存到 waypoints_test.json 草稿文件，不污染正式
waypoints.json——机器没摆到比赛位时练习示教就用它。

流程：
    1. 脚本先让手进入握笔预成形（先弯手指后转拇指）
    2. 开启拖动示教，手把臂拖到目标位——让预成形的手指恰好环住笔筒
    3. 输入航点名回车保存；输入 q 结束（手张开、关闭拖动示教）

抓笔需要记录的航点：
    pen_grasp  手指恰好环住笔筒的夹取位姿（demo_pick_pen.py 会自己算上方悬停点）
后续写书法还需要：ink_dish（蘸墨点）、paper_start（起笔点）等。

注意：拖动中臂处于无力随动状态，动作要慢，别让手碰桌面。
"""
import os
import sys

from arm_api import Arm
from hand_api import Hand

suffix = sys.argv[1] if len(sys.argv) > 1 else ""
if suffix:
    waypoint_file = os.path.join(os.path.dirname(__file__), f"waypoints_{suffix}.json")
    print(f"草稿模式：航点保存到 waypoints_{suffix}.json")
else:
    waypoint_file = None

arm = Arm(waypoint_file=waypoint_file)
hand = Hand()
arm.enable(True)

print("手进入握笔预成形（先手指后拇指）…")
hand.pre_grasp_pen()

print("手进入握笔预成形（先手指后拇指）…")
hand.pre_grasp_pen()

arm.set_mode(True)   # 拖动示教要求手动模式（robot_mode=1）
arm.drag_teach(True)
print("拖动示教已开启：手把机械臂拖到目标位置")
print("输入航点名称回车保存；输入 q 结束")

while True:
    name = input("航点名称: ").strip()
    if name in ("q", "Q"):
        break
    if not name:
        continue
    arm.save_waypoint(name)

arm.drag_teach(False)
arm.set_mode(False)  # 切回自动模式，后续程序控制用
hand.open()
arm.close()
hand.close()
print("已退出，航点保存在 src/waypoints.json")
