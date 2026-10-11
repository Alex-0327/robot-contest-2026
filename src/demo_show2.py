"""展示动作 B：关节空间"挥手"——逐关节小角度摆动。

用 MoveJ（关节空间），不经过笛卡尔奇异点，几乎不会报超限。
动作：基座左右摆 → 腕部摆 → 末端旋转 → 回起始位。幅度大、速度慢。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/demo_show2.py
"""
from arm_api import Arm

DELTAS = [(0, 35), (0, -35), (1, 12), (1, -12), (2, 15), (2, -15),
          (3, 20), (3, -20), (4, 20), (4, -20), (5, 60), (5, -60)]  # (关节序号, 增量°)
VEL = 15

arm = Arm()
arm.enable(True)
err, j0 = arm.robot.GetActualJointPosDegree()
print("起始关节角:", [round(a, 1) for a in j0])

for idx, d in DELTAS:
    print("J%d %+d°" % (idx + 1, d))
    target = list(j0)
    target[idx] += d
    arm.move_joints(target, vel=VEL)

print("回到起始位")
arm.move_joints(list(j0), vel=VEL)
arm.close()
print("完成")
