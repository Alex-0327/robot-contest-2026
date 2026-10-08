"""DexHand021 五指灵巧手 Mock 测试（无硬件、跨平台，可在 Mac 上跑）。

用 MockZCANWrapper 替代真实 CANFD 设备，验证代码逻辑和 API 调用，
不接硬件也能开发调试手部动作序列。
"""
import time

from pyzlg_dexhand import RightDexHand
from pyzlg_dexhand.zcan_wrapper import MockZCANWrapper


def main():
    zcan = MockZCANWrapper()
    zcan.open()  # 外部传入的 zcan 由我们负责 open，否则所有发送都会静默失败
    hand = RightDexHand(zcan=zcan)
    print("Mock 灵巧手初始化完成")

    # 张开 -> 半握 -> 张开，12 个可控自由度按关节名给角度（度）
    print("全部手指张开")
    hand.move_joints(
        th_rot=0, th_mcp=0, th_dip=0, ff_spr=0,
        ff_mcp=0, ff_dip=0, mf_mcp=0, mf_dip=0,
        rf_mcp=0, rf_dip=0, lf_mcp=0, lf_dip=0,
    )
    time.sleep(0.5)

    print("食指+拇指半握（模拟捏笔）")
    hand.move_joints(th_rot=30, th_mcp=40, th_dip=30, ff_mcp=60, ff_dip=50)
    time.sleep(0.5)

    feedback = hand.get_feedback()
    print("关节反馈:", feedback)

    print("重新张开")
    hand.move_joints(th_rot=0, th_mcp=0, th_dip=0, ff_mcp=0, ff_dip=0)

    hand.close()

    history = zcan.get_message_history()
    print(f"Mock 共记录 {len(history)} 条 CAN 报文（真机上这些就是发给手的指令）")
    print("完成")


if __name__ == "__main__":
    main()
