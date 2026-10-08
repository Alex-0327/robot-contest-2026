"""法奥 FAIRINO 机械臂最小连接测试。

用法：
    1. 笔记本网线直连机械臂控制柜（或同一局域网），本机 IP 设为 192.168.58.x 网段
    2. 机械臂上电，等第六轴光圈绿色常亮
    3. python src/demo_arm.py            # 只连接并读取状态，不产生任何运动
       python src/demo_arm.py --move     # 额外执行一次小幅关节运动（确认周围安全后再用）
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sdk", "fairino-python-sdk", "linux"))

from fairino import Robot  # noqa: E402

ROBOT_IP = "192.168.58.2"  # 法奥控制柜默认 IP


def main():
    do_move = "--move" in sys.argv

    print(f"连接机械臂 {ROBOT_IP} ...")
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        print("连接失败：检查网线、本机 IP 是否在 192.168.58.x 网段、机械臂是否已上电")
        sys.exit(1)

    error, version = robot.GetSDKVersion()
    print("SDK/固件版本:", version)

    error, ip = robot.GetControllerIP()
    print("控制器 IP:", ip if error == 0 else f"读取失败(err={error})")

    error, joints = robot.GetActualJointPosDegree()
    if error == 0:
        print("当前关节角(度):", [round(j, 2) for j in joints])
    else:
        print(f"读取关节角失败(err={error})")

    if do_move:
        print("机械臂使能...")
        print("RobotEnable:", robot.RobotEnable(1))
        # 在当前位置基础上，仅让 6 轴转 5 度，验证运动链路
        target = list(joints)
        target[5] += 5.0
        print("MoveJ 到:", [round(j, 2) for j in target])
        # tool/user 坐标系编号需与示教器实际配置一致，默认为 0/0 时按基坐标系
        err = robot.MoveJ(target, tool=0, user=0, vel=10.0)
        print("MoveJ 返回:", err, "(0 为成功)")

    robot.CloseRPC()
    print("完成，连接已关闭")


if __name__ == "__main__":
    main()
