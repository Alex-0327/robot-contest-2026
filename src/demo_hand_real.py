"""DexHand021 五指灵巧手真机控制（仅在比赛笔记本上运行）。

运行前提：
    - 系统：Ubuntu 20.04/22.04（x86_64 或 aarch64），Windows 请改用官方 C++ SDK
    - 硬件：ZLG USBCANFD-200U 适配器，CANFD 通道接灵巧手
    - 首次使用执行过 usb 权限配置：sudo sdk/pyzlg_dexhand/tools/setup_usb_can.sh
    - sdk/pyzlg_dexhand/config/config.yaml 中的 channel / 设备类型与现场一致
"""
import time

from pyzlg_dexhand import RightDexHand  # 左手用 LeftDexHand


def main():
    hand = RightDexHand()  # auto_init=True，自动打开 CAN 设备并配置通道
    if not hand._initialized:
        print("CAN 设备初始化失败：检查 USBCANFD-200U 是否插好、udev 规则是否生效")
        return
    print("灵巧手连接成功")

    hand.clear_errors()

    print("张开")
    hand.move_joints(
        th_rot=0, th_mcp=0, th_dip=0, ff_spr=0,
        ff_mcp=0, ff_dip=0, mf_mcp=0, mf_dip=0,
        rf_mcp=0, rf_dip=0, lf_mcp=0, lf_dip=0,
    )
    time.sleep(1.0)

    print("三指捏（拇指+食指+中指，握笔预备姿态）")
    hand.move_joints(th_rot=35, th_mcp=45, th_dip=35, ff_mcp=65, ff_dip=55, mf_mcp=55, mf_dip=45)
    time.sleep(1.0)

    feedback = hand.get_feedback()
    print("关节反馈:", feedback)

    errors = hand.get_errors()
    if any(errors.values()):
        print("存在错误码:", errors)

    print("张开复位")
    hand.move_joints(th_rot=0, th_mcp=0, th_dip=0, ff_mcp=0, ff_dip=0, mf_mcp=0, mf_dip=0)
    time.sleep(1.0)

    hand.close()
    print("完成")


if __name__ == "__main__":
    main()
