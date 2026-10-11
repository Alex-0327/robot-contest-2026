"""DexHand021 灵巧手动作原语库 —— 分层架构第 2 层。

预成形/捏合角度来自 demo_hand_real.py 的实测起始值，握力上限用
set_pressure_limit_value（0~20 N）保护毛笔不被捏坏或滑脱。
所有 *_EXTRA 闭合增量和握力都是现场可调参数，调一次记一次。
"""
import time

from pyzlg_dexhand import RightDexHand  # 左手换 LeftDexHand

OPEN = dict(th_rot=0, th_mcp=0, th_dip=0, ff_spr=0,
            ff_mcp=0, ff_dip=0, mf_mcp=0, mf_dip=0,
            rf_mcp=0, rf_dip=0, lf_mcp=0, lf_dip=0)

# 三指捏（拇指+食指+中指）预成形：手指围成笔的通道，尚未接触
# 10-11 真机教训：拇指内旋和食指外张同时进行会相互对顶，
# 所以预成形拆成两步——先弯手指，再转拇指。角度≤40° 为实测安全值。
PEN_PRE_GRASP_FINGERS = dict(ff_mcp=40, ff_dip=30, mf_mcp=40, mf_dip=30)
PEN_PRE_GRASP_THUMB = dict(th_rot=20, th_mcp=40, th_dip=30)

# 在预成形基础上的闭合增量：让指腹贴住笔杆/圆筒（保守值，逐步上调）
PEN_GRASP_EXTRA = dict(th_mcp=8, th_dip=5, ff_mcp=10, ff_dip=8,
                       mf_mcp=8, mf_dip=5)


class Hand:
    def __init__(self):
        self._hand = RightDexHand()  # auto_init=True
        if not self._hand._initialized:
            raise ConnectionError("CAN 设备初始化失败：检查 USBCANFD 连接与 config.yaml")
        self._hand.clear_errors()

    def open(self, settle=1.0):
        """五指完全张开。"""
        self._hand.move_joints(**OPEN)
        time.sleep(settle)

    def pre_grasp_pen(self, settle=1.0):
        """握笔预成形：先弯食指中指，再把拇指转进来（分步避免对顶）。"""
        self._hand.move_joints(**PEN_PRE_GRASP_FINGERS)
        time.sleep(0.8)
        self._hand.move_joints(**PEN_PRE_GRASP_THUMB)
        time.sleep(settle)

    def grasp_pen(self, pressure=5, settle=1.5):
        """在预成形基础上捏合，同样分步：先手指后拇指。
        pressure 为握力上限（N，0~20），笔滑就调大 PEN_GRASP_EXTRA，
        笔被捏变形就调小 pressure。"""
        self._hand.set_pressure_limit_value(pressure)
        self._hand.set_pressure_limit_enable(True)
        fingers = dict(PEN_PRE_GRASP_FINGERS)
        thumb = dict(PEN_PRE_GRASP_THUMB)
        for k, v in PEN_GRASP_EXTRA.items():
            if k.startswith("th_"):
                thumb[k] = thumb.get(k, 0) + v
            else:
                fingers[k] = fingers.get(k, 0) + v
        self._hand.move_joints(**fingers)
        time.sleep(0.8)
        self._hand.move_joints(**thumb)
        time.sleep(settle)

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
