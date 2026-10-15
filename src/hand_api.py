"""DexHand021 灵巧手动作原语库 —— 分层架构第 2 层。

预成形/捏合角度为真机实测值（见各常量注释）。纯位置控制：
set_pressure_limit 在本固件上会导致电机停在半路，不要用。
"""
import time

from pyzlg_dexhand import RightDexHand  # 左手换 LeftDexHand

OPEN = dict(th_rot=0, th_mcp=0, th_dip=0, ff_spr=0,
            ff_mcp=0, ff_dip=0, mf_mcp=0, mf_dip=0,
            rf_mcp=0, rf_dip=0, lf_mcp=0, lf_dip=0)

# 握笔姿态（10-11 真机实测调定，抓取加粗圆筒笔杆稳定悬吊不滑脱）：
# 顺序要点：拇指先旋转到位（保持伸直），再弯曲——旋转和弯曲同时进行会顶到食指。
# 与 data/grasp_params.json 一致，改这里要同步改数据集。
# 10-15 预抓握弯曲指统一松 5°（40/30→35/25）：进筒通道更宽，下降不易刮筒口
PEN_PRE_GRASP_FINGERS = dict(ff_mcp=35, ff_dip=25, mf_mcp=35, mf_dip=25)
PEN_PRE_GRASP_THUMB = dict(th_rot=70, th_mcp=0, th_dip=0)  # 预成形时拇指只旋转、保持伸直，给下降留通道

# 最终握紧姿态（绝对角度）
# 10-12 写字实测笔杆微滑 → ff_mcp 70→75 / mf_mcp 75→80 / th_mcp 55→60（每次只加 5°，防电机顶死）
PEN_GRASP_FINGERS = dict(ff_mcp=75, ff_dip=30, mf_mcp=80, mf_dip=30)
PEN_GRASP_THUMB = dict(th_rot=70, th_mcp=50, th_dip=0)


class Hand:
    def __init__(self):
        self._hand = RightDexHand()  # auto_init=True
        if not self._hand._initialized:
            raise ConnectionError("CAN 设备初始化失败：检查 USBCANFD 连接与 config.yaml")
        self._hand.clear_errors()
        self.state = dict(OPEN)  # 固件读不回实际角度，记录最后一次指令值

    def open(self, settle=1.0):
        """五指完全张开。同一广播发两遍：12 关节单帧广播偶发个别关节丢帧
        （10-12 现场：仅中指张开、拇指/食指未动），幂等补发无副作用。"""
        self._hand.move_joints(**OPEN)
        time.sleep(0.5)
        self._hand.move_joints(**OPEN)
        self.state.update(OPEN)
        time.sleep(settle)

    def pre_grasp_pen(self, settle=0.5):
        """握笔预成形：拇指先伸直旋转到位（其他手指张开），再预弯食指中指。
        根部先就位避免捏合时卡到筒口。"""
        self._hand.move_joints(**PEN_PRE_GRASP_THUMB)
        self.state.update(PEN_PRE_GRASP_THUMB)
        time.sleep(0.8)
        self._hand.move_joints(**PEN_PRE_GRASP_FINGERS)
        self.state.update(PEN_PRE_GRASP_FINGERS)
        time.sleep(settle)

    def grasp_pen(self, settle=1.5):
        """握紧：拇指/食指/中指同时闭合（单指令同时启动，避免中间姿态碰倒笔筒）。
        纯位置控制——set_pressure_limit 在本固件上会导致电机停在半路，已弃用。"""
        self._hand.move_joints(**PEN_GRASP_FINGERS, **PEN_GRASP_THUMB)
        self.state.update(PEN_GRASP_FINGERS)
        self.state.update(PEN_GRASP_THUMB)
        time.sleep(settle)

    def set_joint(self, joint, deg, settle=0.6):
        """绝对设定某关节角度（度）。"""
        self._hand.move_joints(**{joint: deg})
        self.state[joint] = deg
        time.sleep(settle)

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
