"""开机自检：只读状态，不产生任何运动。确认臂、手两条链路都活着。

用法：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/check_all.py

通过标准：打印出臂位姿 + 手固件/关节反馈，无错误码。
"""
from arm_api import Arm
from hand_api import Hand

print("== 机械臂 ==")
arm = Arm()
print("臂位姿:", [round(p, 1) for p in arm.get_pose()])
arm.close()

print("== 灵巧手 ==")
hand = Hand()
ok = hand.check()
hand.close()

print("自检通过，可以开始干活" if ok else "手有错误码，见上方输出")
