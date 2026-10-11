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

# 握笔姿态（10-11 真机实测调定，抓取加粗圆筒笔杆稳定悬吊不滑脱）：
# 顺序要点：拇指先旋转到位（保持伸直），再弯曲——旋转和弯曲同时进行会顶到食指。
PEN_PRE_GRASP_FINGERS = dict(ff_mcp=40, ff_dip=30, mf_mcp=40, mf_dip=30)
PEN_PRE_GRASP_THUMB = dict(th_rot=60)  # 预成形时拇指只旋转、保持伸直，给下降留通道

# 最终握紧姿态（绝对角度）
PEN_GRASP_FINGERS = dict(ff_mcp=60, ff_dip=30, mf_mcp=75, mf_dip=30)
PEN_GRASP_THUMB = dict(th_rot=60, th_mcp=55, th_dip=0)


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
        """从预成形到握紧（10-11 实测角度）：先收食指中指，再弯拇指。
        pressure 为握力上限（N，0~20）。"""
        self._hand.set_pressure_limit_value(pressure)
        self._hand.set_pressure_limit_enable(True)
        self._hand.move_joints(**PEN_GRASP_FINGERS)
        time.sleep(0.8)
        self._hand.move_joints(**PEN_GRASP_THUMB)
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
