"""抓笔全流程 —— 今天的里程碑。

前提：已用 teach_waypoints.py 记录航点 pen_grasp（手指恰好环住笔杆的
夹取位姿）。脚本自动算笔上方 60mm 的悬停点，不用单独记录。

流程：手张开 → 走到笔上方 → 预成形 → 慢速下降 → 低握力捏合 → 抬起。
抓不稳就调 hand_api.py 里的 PEN_GRASP_EXTRA / 本文件的 GRASP_PRESSURE，
每次只改一个量，记下改了什么。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/demo_pick_pen.py [后缀]

带后缀（如 test）时从 waypoints_test.json 草稿文件读航点，配合
teach_waypoints.py 的后缀使用。
"""
import os
import sys

from arm_api import Arm
from hand_api import Hand

ABOVE_MM = 60       # 悬停点在夹取位上方的高度

suffix = sys.argv[1] if len(sys.argv) > 1 else ""
waypoint_file = (os.path.join(os.path.dirname(__file__), f"waypoints_{suffix}.json")
                 if suffix else None)

arm = Arm(waypoint_file=waypoint_file)
hand = Hand()
arm.enable(True)
hand.open()

grasp_pose = arm.get_waypoint("pen_grasp")
above = list(grasp_pose)
above[2] += ABOVE_MM

print("走到笔上方")
arm.move_ik(above, vel=10)   # 大范围转移走关节空间：home→笔架的 MoveL 会撞奇异/超限(err=185)
hand.pre_grasp_pen()

print("慢速下降到夹取位")
arm.move_to(grasp_pose, vel=5)

print("握紧")
hand.grasp_pen()

print("抬起")
arm.move_to(above, vel=5)

print("完成。手动取笔前请先确认捏持牢固")
arm.close()
hand.close()
