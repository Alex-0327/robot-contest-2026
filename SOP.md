# 开机激活 SOP（每天开工按此顺序）

## 一、硬件检查（断电状态下）

1. CANFD 端子：用手指轻拽每根线，确认不松动（10-10 虚接教训：会导致手完全无响应）
2. 臂网线：电脑 ↔ 控制柜插紧
3. USBCANFD 适配器：插在主机 USB 口上
4. **急停按钮：确认处于旋出复位状态**（按下 = 控制柜闪灯即灭）

## 二、上电顺序

1. 控制柜上电 → 灯环**变色循环 = 自检中**（约 1~2 分钟）→ **绿色常亮 = 就绪**
2. 灵巧手上电（能听到掌内风扇声 = 已供电；此手本无指示灯，属正常）
3. 开电脑

## 三、软件激活

```bash
conda activate robot
cd ~/robot-contest-2026
git pull   # 需要联网才拉得到；没网可跳过，用本地代码
PYTHONPATH=~/fairino-python-sdk/linux/fairino python src/check_all.py
```

看到臂位姿 + 手关节反馈、无错误码 = 链路全通，可以跑任何运动脚本。

## 四、收工关机顺序

1. 确认运动脚本已全部跑完
2. 关控制柜电源（臂有关节抱闸，断电自锁，不会塌）
3. 关灵巧手供电
4. 关电脑

## 故障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| 控制柜闪灯即灭 | 急停被按下 | 旋出急停，重新上电 |
| 手完全无响应 | CANFD 端子虚接 | 断电后重新剥线/压紧端子 |
| `import Robot` 报 unknown location | fairino 包缺 `__init__.py` | 命令前加 `PYTHONPATH=~/fairino-python-sdk/linux/fairino` |
| git pull 失败 | GitHub 国内不稳 | 重试几次；或改用 gitee 镜像 / U 盘 |
