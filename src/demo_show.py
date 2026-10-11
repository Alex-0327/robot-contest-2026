"""机械臂展示动作：上移 80mm → 水平面走正方形 → 落回原位。

幅度大、速度慢（vel=10），纯臂不动手，用于演示和鼓舞士气 :)

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/demo_show.py

安全前提：臂周 1 米无人无障碍，控制柜绿色常亮，手放急停旁。
"""
from arm_api import Arm

STEP = 80   # 每步幅度 mm
VEL = 10    # 速度，越小越慢

arm = Arm()
arm.enable(True)
print("起始位姿:", [round(p, 1) for p in arm.get_pose()])

print("上移 80mm")
arm.move_rel(dz=STEP, vel=VEL)

print("走正方形")
arm.move_rel(dx=STEP, vel=VEL)
arm.move_rel(dy=STEP, vel=VEL)
arm.move_rel(dx=-STEP, vel=VEL)
arm.move_rel(dy=-STEP, vel=VEL)

print("落回原位")
arm.move_rel(dz=-STEP, vel=VEL)

print("结束位姿:", [round(p, 1) for p in arm.get_pose()])
arm.close()
print("完成")
