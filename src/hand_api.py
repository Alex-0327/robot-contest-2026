"""DexHand021 灵巧手动作原语库 —— 分层架构第 2 层。

预成形/捏合角度为真机实测值（见各常量注释）。纯位置控制：
set_pressure_limit 在本固件上会导致电机停在半路，不要用。

握笔参数的唯一权威来源是 data/grasp_params.json（面板「存为预抓握/抓握」
按钮写它）；下面的常量只是文件缺失时的兜底默认值。改参数后调
reload_params() 热更新，不必重启进程。
"""
import json
import os
import time

from pyzlg_dexhand import RightDexHand  # 左手换 LeftDexHand

OPEN = dict(th_rot=0, th_mcp=0, th_dip=0, ff_spr=0,
            ff_mcp=0, ff_dip=0, mf_mcp=0, mf_dip=0,
            rf_mcp=0, rf_dip=0, lf_mcp=0, lf_dip=0)

_PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'data', 'grasp_params.json')

# 兜底默认值（grasp_params.json 缺失/损坏时使用）
_DEFAULTS = {
    "pre_thumb":   dict(th_rot=70, th_mcp=0, th_dip=0),
    "pre_fingers": dict(ff_mcp=35, ff_dip=25, mf_mcp=35, mf_dip=25),
    "grasp_thumb": dict(th_rot=70, th_mcp=40, th_dip=0),
    "grasp_fingers": dict(ff_mcp=75, ff_dip=30, mf_mcp=80, mf_dip=30),
}


def _read_params():
    try:
        with open(_PARAMS_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def reload_params():
    """从 data/grasp_params.json 重读握笔参数，原地更新四个姿态字典。"""
    d = _read_params()
    pre = d.get("pre_grasp", {})
    grasp = d.get("grasp", {}).get("joints", {})
    PEN_PRE_GRASP_THUMB.clear()
    PEN_PRE_GRASP_THUMB.update(dict(
        th_rot=pre.get("step1_thumb_rotate", {}).get("th_rot", _DEFAULTS["pre_thumb"]["th_rot"]),
        th_mcp=0, th_dip=0))
    PEN_PRE_GRASP_FINGERS.clear()
    PEN_PRE_GRASP_FINGERS.update(pre.get("step2_fingers") or _DEFAULTS["pre_fingers"])
    PEN_GRASP_THUMB.clear()
    PEN_GRASP_THUMB.update({k: grasp.get(k, v) for k, v in _DEFAULTS["grasp_thumb"].items()})
    PEN_GRASP_FINGERS.clear()
    PEN_GRASP_FINGERS.update({k: grasp.get(k, v) for k, v in _DEFAULTS["grasp_fingers"].items()})


# 握笔姿态（真机实测调定，抓取加粗圆筒笔杆稳定悬吊不滑脱）：
# 顺序要点：拇指先旋转到位（保持伸直），再弯曲——旋转和弯曲同时进行会顶到食指。
# 每次只调 5°，防电机顶死。
PEN_PRE_GRASP_THUMB = dict(_DEFAULTS["pre_thumb"])
PEN_PRE_GRASP_FINGERS = dict(_DEFAULTS["pre_fingers"])
PEN_GRASP_THUMB = dict(_DEFAULTS["grasp_thumb"])
PEN_GRASP_FINGERS = dict(_DEFAULTS["grasp_fingers"])
reload_params()


class Hand:
    def __init__(self):
        self._hand = RightDexHand()  # auto_init=True
        if not self._hand._initialized:
            raise ConnectionError("CAN 设备初始化失败：检查 USBCANFD 连接与 config.yaml")
        self._hand.clear_errors()
        self.state = dict(OPEN)  # 固件读不回实际角度，记录最后一次指令值

    def _send(self, cmd, settle=0.0):
        """发指令 + 幂等补发：CAN 单帧广播偶发丢帧（10-12 伸直丢帧、10-16 预赛
        预抓握丢帧致拇指未伸直卡筒），同一指令发两遍，无副作用。"""
        self._hand.move_joints(**cmd)
        time.sleep(0.3)
        self._hand.move_joints(**cmd)
        self.state.update(cmd)
        if settle:
            time.sleep(settle)

    def open(self, settle=1.0):
        """五指完全张开。"""
        self._send(OPEN, settle)

    def pre_grasp_pen(self, settle=0.5):
        """握笔预成形：拇指先伸直旋转到位（其他手指张开），再预弯食指中指。
        根部先就位避免捏合时卡到筒口。"""
        self._send(PEN_PRE_GRASP_THUMB, 0.8)
        self._send(PEN_PRE_GRASP_FINGERS, settle)

    def grasp_pen(self, settle=1.5):
        """握紧：拇指/食指/中指同时闭合（单指令同时启动，避免中间姿态碰倒笔筒）。
        纯位置控制——set_pressure_limit 在本固件上会导致电机停在半路，已弃用。"""
        self._send({**PEN_GRASP_FINGERS, **PEN_GRASP_THUMB}, settle)

    def set_joint(self, joint, deg, settle=0.6):
        """绝对设定某关节角度（度）。"""
        self._send({joint: deg}, settle)

    def adjust_joint(self, joint, delta, settle=0.6):
        """相对微调某关节（基于最后指令值，固件无实际角度回读）。"""
        return self.set_joint(joint, self.state.get(joint, 0) + delta, settle)

    def check(self):
        """打印关节反馈与错误码，返回是否有错误。"""
        print("关节反馈:", self._hand.get_feedback())
        errors = self._hand.get_errors()
        if any(errors.values()):
            print("存在错误码:", errors)
            return False
        return True

    def close(self):
        self._hand.close()
