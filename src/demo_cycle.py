"""全流程彩排：放回 → 抬升归位 → 重新抓取 → 翻转 → 模拟落笔 → 放回。

高度规则（10-11 现场定）：抓取点是笔筒而非笔尖，所有悬停/抬升高度
额外 +5cm 为笔尖留余量。模拟落笔分 3 段执行，每段停顿可人工叫停。
每步后校验控制器报警（main_code），有报警立即停止。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/demo_cycle.py [后缀]
"""
import os
import sys
import time

from arm_api import Arm
from hand_api import Hand

ABOVE_MM = 110   # 笔上方悬停（60 + 笔尖余量 50）
LIFT_FLIP = 130  # 抓后抬起再翻转（80 + 50）
LIFT_HOME = 300  # 收尾抬升（250 + 50）
DIP_MM = 150     # 模拟落笔下降量（分 3 段）

suffix = sys.argv[1] if len(sys.argv) > 1 else "test"
wf = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"waypoints_{suffix}.json")

arm = Arm(waypoint_file=wf)
hand = Hand()
arm.reset_errors()
arm.enable(True)

grasp = arm.get_waypoint("pen_grasp")
above = list(grasp)
above[2] += ABOVE_MM


def j6(deg):
    err, j = arm.robot.GetActualJointPosDegree()
    t = list(j)
    t[5] += deg
    arm.move_joints(t, vel=10)


def check(step):
    pose = arm.get_pose()
    err, pkg = arm.robot.GetRobotRealTimeState()
    print(f"[{step}] z={pose[2]:.1f} main={pkg.main_code}")
    if pkg.main_code != 0:
        raise RuntimeError(f"控制器报警 main={pkg.main_code}，停止")


print("1. 手腕回旋180（笔尖朝上）")
j6(-180)
check(1)
print("2. 下降到夹取位")
arm.move_to(grasp, vel=5)
check(2)
print("3. 松手")
hand.open()
print("4. 抬升 30cm 到 home")
arm.move_rel(dz=LIFT_HOME, vel=10)
check(4)
print("home:", [round(p, 1) for p in arm.get_pose()])
time.sleep(1)

print("5. 到笔上方 11cm")
arm.move_to(above, vel=10)
check(5)
print("6. 预成形（拇指先转位）")
hand.pre_grasp_pen()
print("7. 下降到夹取位")
arm.move_to(grasp, vel=5)
check(7)
print("8. 三指同时握紧")
hand.grasp_pen()
print("9. 抬起 13cm")
arm.move_rel(dz=LIFT_FLIP, vel=5)
check(9)
print("10. J6 翻转180（笔尖朝下）")
j6(180)
check(10)

print("11. 模拟落笔：3段下降共15cm，段间停顿")
for i in range(3):
    arm.move_rel(dz=-DIP_MM // 3, vel=5)
    check(f"11-{i+1}")
    time.sleep(1.5)
print("12. 3段升回")
for i in range(3):
    arm.move_rel(dz=DIP_MM // 3, vel=5)
    check(f"12-{i+1}")

print("13. 回旋180")
j6(-180)
check(13)
print("14. 放回夹取位")
arm.move_to(grasp, vel=5)
check(14)
print("15. 松手")
hand.open()
print("16. 抬升 30cm 收工")
arm.move_rel(dz=LIFT_HOME, vel=10)
check(16)

arm.close()
hand.close()
print("彩排完成")
