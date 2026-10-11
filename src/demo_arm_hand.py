"""臂+手联合空跑测试（不抓笔）—— 第一条全链路验证。

预期现象：
    1. 臂缓慢上移 20mm → 落回原位
    2. 手 张开 → 握笔预成形 → 捏合 → 张开
任何一步异常就 Ctrl+C，必要时拍急停。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/demo_arm_hand.py
"""
from arm_api import Arm
from hand_api import Hand

arm = Arm()
hand = Hand()
arm.enable(True)

print("臂：上移 20mm → 落回")
arm.move_rel(dz=20, vel=5)
arm.move_rel(dz=-20, vel=5)

print("手：张开 → 预成形 → 握紧 → 张开")
hand.open()
hand.pre_grasp_pen()
hand.grasp_pen()
hand.open()

print("最终位姿:", [round(p, 1) for p in arm.get_pose()])
hand.check()
arm.close()
hand.close()
print("完成")
