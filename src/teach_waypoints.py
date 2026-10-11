"""拖动示教记录航点：把固定位置（笔、墨碟、纸面）存进 waypoints.json。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/teach_waypoints.py

流程：脚本开启拖动示教 → 手把臂拖到目标位 → 输入航点名回车保存 →
输入 q 结束并关闭拖动示教。航点保存在 src/waypoints.json。

抓笔需要记录的航点：
    pen_grasp  手指恰好环住笔杆的夹取位姿（demo_pick_pen.py 会自己算上方悬停点）
后续写书法还需要：ink_dish（蘸墨点）、paper_start（起笔点）等。
"""
from arm_api import Arm

arm = Arm()
arm.enable(True)
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
arm.close()
print("已退出，航点保存在 src/waypoints.json")
