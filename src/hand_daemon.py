"""调手守护进程：常驻持有 CAN，通过 /tmp 文件接收微调指令，远程逐度调手型。

启动（现场机）：
    nohup ~/miniconda3/envs/robot/bin/python src/hand_daemon.py \
    > /tmp/hand_daemon.log 2>&1 < /dev/null &

指令（写入 /tmp/hand_cmd）：
    a th_mcp -5      相对调整：th_mcp 减 5 度（松一点）
    s th_mcp 30      绝对设定：th_mcp = 30 度
    open             全部归零张开
    pre              应用握笔预成形（先手指后拇指）
    p                打印当前目标角度
    q                退出（手保持当前姿态不动）
结果读 /tmp/hand_result。

关节名：th_rot th_mcp th_dip ff_spr ff_mcp ff_dip
        mf_mcp mf_dip rf_mcp rf_dip lf_mcp lf_dip
"""
import os
import time

from pyzlg_dexhand import RightDexHand
from hand_api import OPEN, PEN_PRE_GRASP_FINGERS, PEN_PRE_GRASP_THUMB

CMD = "/tmp/hand_cmd"
RESULT = "/tmp/hand_result"

VALID = ["th_rot", "th_mcp", "th_dip", "ff_spr", "ff_mcp", "ff_dip",
         "mf_mcp", "mf_dip", "rf_mcp", "rf_dip", "lf_mcp", "lf_dip"]

h = RightDexHand()
h.clear_errors()

targets = {k: 0.0 for k in VALID}


def report(msg):
    with open(RESULT, "w") as f:
        f.write(msg + "\n")
    print(msg, flush=True)


def apply():
    h.move_joints(**{k: v for k, v in targets.items() if v != 0})
    errors = h.get_errors()
    if any(errors.values()):
        report("错误码: " + str(errors))


report("READY: 调手进程就绪，当前全张开")

while True:
    if os.path.exists(CMD):
        with open(CMD) as f:
            cmd = f.read().strip()
        os.remove(CMD)
        parts = cmd.split()
        try:
            if cmd == "q":
                break
            elif cmd == "open":
                targets = {k: 0.0 for k in VALID}
                h.move_joints(**OPEN)
                report("已张开")
            elif cmd == "pre":
                for k, v in PEN_PRE_GRASP_FINGERS.items():
                    targets[k] = float(v)
                h.move_joints(**PEN_PRE_GRASP_FINGERS)
                time.sleep(0.8)
                for k, v in PEN_PRE_GRASP_THUMB.items():
                    targets[k] = float(v)
                h.move_joints(**PEN_PRE_GRASP_THUMB)
                report("预成形完成: " + str({k: v for k, v in targets.items() if v}))
            elif cmd == "p":
                report(str({k: v for k, v in targets.items() if v}))
            elif parts[0] == "a" and len(parts) == 3 and parts[1] in VALID:
                targets[parts[1]] = max(-30.0, min(90.0, targets[parts[1]] + float(parts[2])))
                apply()
                report(f"{parts[1]} -> {targets[parts[1]]}")
            elif parts[0] == "s" and len(parts) == 3 and parts[1] in VALID:
                targets[parts[1]] = max(-30.0, min(90.0, float(parts[2])))
                apply()
                report(f"{parts[1]} -> {targets[parts[1]]}")
            else:
                report("unknown: " + cmd)
        except Exception as e:
            report("error: " + str(e))
    time.sleep(0.2)

h.close()
report("DONE: 调手进程退出（手保持当前姿态）")
