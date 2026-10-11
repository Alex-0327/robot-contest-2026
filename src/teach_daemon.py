"""示教守护进程：常驻保持 RPC 连接（SSH 断开后控制器会自动退出拖动态，

所以示教期间必须有一个进程一直连着）。通过 /tmp 文件接收指令。

启动（在现场机）：
    PYTHONPATH=~/fairino-python-sdk/linux/fairino \
    nohup ~/miniconda3/envs/robot/bin/python src/teach_daemon.py [后缀] \
    > /tmp/teach_daemon.log 2>&1 < /dev/null &

控制（任何人、任何终端都可发）：
    echo "pose" > /tmp/teach_cmd          # 读当前位姿
    echo "save pen_grasp" > /tmp/teach_cmd  # 保存航点
    echo "quit" > /tmp/teach_cmd          # 退出拖动态、切回自动、收尾
结果读取：
    cat /tmp/teach_result
"""
import os
import sys
import time

from arm_api import Arm
from hand_api import Hand

CMD = "/tmp/teach_cmd"
RESULT = "/tmp/teach_result"

suffix = sys.argv[1] if len(sys.argv) > 1 else ""
wf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  f"waypoints_{suffix}.json" if suffix else "waypoints.json")


def report(msg):
    with open(RESULT, "w") as f:
        f.write(msg + "\n")
    print(msg, flush=True)


arm = Arm(waypoint_file=wf)
hand = Hand()
arm.enable(True)
hand.pre_grasp_pen()
arm.set_mode(True)
arm.drag_teach(True)
report(f"READY: 手已预成形，臂可拖动。航点文件 {os.path.basename(wf)}")

try:
    while True:
        if os.path.exists(CMD):
            with open(CMD) as f:
                cmd = f.read().strip()
            os.remove(CMD)
            if cmd == "quit":
                break
            elif cmd == "pose":
                report("pose: " + str([round(p, 2) for p in arm.get_pose()]))
            elif cmd.startswith("save "):
                name = cmd[5:].strip()
                arm.save_waypoint(name)
                report(f"saved [{name}]")
            else:
                report(f"unknown cmd: {cmd}")
        time.sleep(0.3)
finally:
    arm.drag_teach(False)
    arm.set_mode(False)
    hand.open()
    arm.close()
    hand.close()
    report("DONE: 已退出拖动态、切回自动模式、手已张开")
