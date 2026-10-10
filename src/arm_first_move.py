"""机械臂第一次点动测试：上移 20mm → 停 → 落回原位。

用法（现场 Linux 机，在任意目录下）：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/arm_first_move.py

安全前提：臂周 1 米无人无障碍，控制柜绿色常亮，手放急停旁。
"""
import time

import Robot

r = Robot.RPC("192.168.58.2")
err, pose = r.GetActualTCPPose()
print("pose:", [round(p, 1) for p in pose])
r.RobotEnable(1)
time.sleep(1)
t = list(pose)
t[2] += 20
print("up err:", r.MoveL(t, 0, 0, vel=5))
time.sleep(5)
t[2] -= 20
print("down err:", r.MoveL(t, 0, 0, vel=5))
time.sleep(5)
err, p2 = r.GetActualTCPPose()
print("final:", [round(p, 1) for p in p2])
r.CloseRPC()
