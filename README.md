# 五指灵巧操作赛项 — 开发环境

中国机器人大赛 2026「五指灵巧操作——艺术技能挑战」控制程序工程。
硬件：法奥 FAIRINO 六轴协作机械臂 + 灵巧智能 DexHand021 五指灵巧手 + 组委会深度相机。

## 目录结构

```
.venv/                  # Python 3.12 虚拟环境（uv 创建）
sdk/
  fairino-python-sdk/   # 法奥机械臂 SDK（官方开源纯 Python 版，XMLRPC 协议，跨平台）
  pyzlg_dexhand/        # 灵巧智能五指手 Python SDK（ZLG USBCANFD 通信，主用）
  dexhand_sdk_python/   # 灵巧官方 Python SDK（目前只支持三指 021S，五指请用 pyzlg_dexhand）
src/
  demo_arm.py           # 机械臂最小连接测试
  demo_hand_mock.py     # 灵巧手 Mock 测试（无硬件，Mac 上可直接跑）
  demo_hand_real.py     # 灵巧手真机控制（仅比赛笔记本 Linux + USBCANFD-200U）
五指规则-更新2026(1).pdf  # 赛项规则
```

## 环境激活

```bash
source .venv/bin/activate
```

已安装：python-mbedtls（法奥 SDK 依赖）、numpy、pyyaml、opencv-python、
pyzlg_dexhand（editable 安装，含 matplotlib/pandas/filterpy/ipython/ros_compat）。

## 平台说明（重要）

| 组件 | Mac（本机） | 比赛笔记本（Ubuntu/Windows） |
|---|---|---|
| 法奥机械臂 SDK | 可开发调试（纯 Python，TCP/XMLRPC 走网线） | 可用 |
| 灵巧手 pyzlg_dexhand | 仅 Mock 模式（libusbcanfd 只有 Linux/Windows 版） | 可用，需 ZLG USBCANFD-200U |
| 深度相机 | 待组委会公布型号后装对应 SDK | 可用 |

规则要求所有感知与计算在本地笔记本完成，禁止云端算力；笔记本京东自营价 ≤8000 元。

## 三个 Demo

```bash
python src/demo_hand_mock.py   # 先跑这个：无硬件验证手部控制代码逻辑
python src/demo_arm.py         # 接上机械臂后跑：本机 IP 设 192.168.58.x 网段
python src/demo_arm.py --move  # 确认安全后：额外做一次 6 轴 +5° 小幅运动
python src/demo_hand_real.py   # 比赛笔记本上跑：真机手部张合
```

## 真机接入检查单

机械臂（法奥）：
1. 网线连控制柜，本机 IP 设为 192.168.58.x（如 192.168.58.100），掩码 255.255.255.0
2. 上电等第六轴光圈绿色常亮，`ping 192.168.58.2` 通后再跑 demo_arm.py
3. SDK 文档：https://fr-documentation.readthedocs.io/zh_CN/latest/SDKManual/python_intro.html

灵巧手（DexHand021）：
1. 需要周立功 ZLG USBCANFD-200U 适配器（确认组委会是否随设备提供，没有需自购）
2. 首次使用：`sudo sdk/pyzlg_dexhand/tools/setup_usb_can.sh` 配置 udev 权限，然后重新登录
3. 按现场接线改 `sdk/pyzlg_dexhand/config/config.yaml` 的 channel 和设备类型
4. SDK 文档：sdk/pyzlg_dexhand/README_zh.md

## 待办 / 待确认

- [ ] 相机型号与 SDK（组委会统一提供，等 QQ 群 1062438256 公布）
- [ ] USBCANFD-200U 适配器是否随设备发放
- [ ] 手眼标定参数（规则说明会公布，精度不满意可在调试阶段自行标定）
- [ ] 比赛笔记本到位后，按本 README 在其上重装一遍环境（Windows 需改用官方 C++ SDK 编译 dexhand）
- [ ] 灵巧手 C++ SDK（github.com/DexRobot/dexhand_sdk_cpp，GitHub 直连不稳时从官网
      dex-robot.com 下载中心获取）；Python 路线优先用 pyzlg_dexhand
