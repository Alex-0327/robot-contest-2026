"""机械臂运动原语库 —— 分层架构第 1 层：笛卡尔空间基本动作。

设计要点：
- 所有位置指令都是末端 TCP 位姿 [x,y,z,rx,ry,rz]（mm / °），不用关心 6 个关节角，
  逆运动学由控制器解算（MoveL 内部自动调 GetInverseKin）
- move_rel 用 MoveL 自带的 offset 功能在基坐标系下做相对偏移，由控制器端完成
- 大范围转移用 MoveJ（关节空间，不怕奇异点），精细直线用 MoveL（笛卡尔直线插补）
- 固定位置（墨碟、笔架等）用拖动示教记录成 waypoint 存 JSON，不硬编码在代码里

用法见文件末尾 main 自测段。
"""
import json
import os
import sys
import time

_sdk_dir = os.path.join(os.path.dirname(__file__), "..", "sdk", "fairino-python-sdk", "linux")
if os.path.isdir(_sdk_dir):
    sys.path.insert(0, _sdk_dir)

try:
    from fairino import Robot  # noqa: E402
except ImportError:  # 现场机：SDK 不在仓库内，用 PYTHONPATH=fairino-python-sdk/linux/fairino 提供
    import Robot  # noqa: E402

DEFAULT_IP = "192.168.58.2"
WAYPOINT_FILE = os.path.join(os.path.dirname(__file__), "waypoints.json")

# FR5 关节软限位（°），2026-10-12 从控制器 GetJointSoftLimitDeg 实读。
# 注意 J2/J4 不对称（-265~+85）。GetInverseKin 不做限位检查，会返回超界解，
# move_ik 选解前必须自己过滤，否则 MoveJ 报 err=14（10-12 取笔失败的教训）。
JOINT_MIN = [-175.0, -265.0, -160.0, -265.0, -175.0, -175.0]
JOINT_MAX = [175.0, 85.0, 160.0, 85.0, 175.0, 175.0]
LIMIT_MARGIN = 3.0  # 离软限位留 3° 余量


class Arm:
    def __init__(self, ip=DEFAULT_IP, tool=None, user=None, waypoint_file=None):
        """连接机械臂。tool/user 默认自动读取控制器当前激活的工具/工件坐标系
        （10-11 教训：控制柜上激活的是 1 号工具，硬编码 0 会报 err=154 工具不符）。
        waypoint_file 默认 waypoints.json，练习时传别的路径避免污染正式航点。
        注意：CNDE 状态流连接后需预热，所有状态读取都要重试（10-11 教训：
        不预热会读到垃圾位姿或直接 TypeError）。"""
        self.robot = Robot.RPC(ip)
        if self.robot is None:
            raise ConnectionError(f"无法连接机械臂 {ip}")
        self.waypoint_file = waypoint_file or WAYPOINT_FILE
        time.sleep(2)  # CNDE 状态流预热
        if tool is None or user is None:
            pkg = None
            for _ in range(5):
                res = self.robot.GetRobotRealTimeState()
                if isinstance(res, tuple) and res[0] == 0:
                    pkg = res[1]
                    break
                time.sleep(1)
            if pkg is None:
                raise RuntimeError("控制器状态流未就绪，重连试试")
            if tool is None:
                tool = pkg.tool_id
            if user is None:
                user = pkg.wobj_id
        self.tool = tool
        self.user = user

    def reset_errors(self):
        """清除控制器上的可复位报警（运动报错后必须复位才能继续）。"""
        return self.robot.ResetAllError()

    # ---------- 状态读取 ----------

    def get_pose(self):
        """当前 TCP 位姿 [x,y,z,rx,ry,rz]，单位 mm / °。读一次校验一次（状态流偶发垃圾值）。"""
        error, pose = self.robot.GetActualTCPPose()
        if error != 0:
            raise RuntimeError(f"读取 TCP 位姿失败 err={error}")
        return pose

    def _current_config(self):
        """当前关节角所属的 IK 构型分支号（0-7）：对当前位姿逐构型求解，
        取与实读关节角最近的分支。点动锁分支用。"""
        err, cur = self.robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"读取关节角失败 err={err}")
        pose = self.get_pose()
        best = None
        for cfg in range(8):
            err, j = self.robot.GetInverseKin(0, list(pose), cfg)
            if err == 0 and j is not None:
                dist = sum((a - b) ** 2 for a, b in zip(cur, j))
                if best is None or dist < best[0]:
                    best = (dist, cfg)
        if best is None:
            raise RuntimeError("当前位姿无法判定构型分支")
        return best[1]

    def move_ik(self, pose, vel=10.0, blocking=True, config=None):
        """经逆解走关节空间到目标位姿——大姿态变化（如翻转后升降）用这条，
        笛卡尔 MoveL 在 ry≈±90 奇异点附近会报 err=99/112。
        config=-1（保持当前构型）在臂处于非常规姿态后会假报不可达（err=38），
        因此解出全部 8 种构型，选与当前关节角距离最小的解（10-11 晚教训：
        取"第一个成功"的构型会翻肘，TCP 轨迹安全但大臂扫到桌面）。
        肘朝下解（J3<0）全局劣后：手动拖动/碰撞冻结会把臂停在肘朝下构型，
        只锚定"最近"会把错误构型一路继承下去（10-11 深夜两次撞桌的教训）。
        config=None 时全分支择优；指定 config（0-7）时只用该分支——点动必须
        锁分支，否则逼近限位时选解跳分支，臂会突然做大动作（10-12 br 卡位教训）。"""
        err, cur = self.robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"读取关节角失败 err={err}")
        best = None
        configs = (config,) if config is not None else (-1, 0, 1, 2, 3, 4, 5, 6, 7)
        for cfg in configs:
            err, j = self.robot.GetInverseKin(0, list(pose), cfg)
            if err == 0 and j is not None:
                if any(v < lo + LIMIT_MARGIN or v > hi - LIMIT_MARGIN
                       for v, lo, hi in zip(j, JOINT_MIN, JOINT_MAX)):
                    continue  # GetInverseKin 不查限位，超界解直接丢弃
                dist = sum((a - b) ** 2 for a, b in zip(cur, j))
                if j[2] <= 0:
                    dist += 1e6  # 肘朝下构型劣后，无肘上解时才用
                if best is None or dist < best[0]:
                    best = (dist, list(j))
        if best is None:
            raise RuntimeError("逆解失败（8 种构型均不可达或超限位）")
        self.move_joints(best[1], vel=vel, blocking=blocking)

    def wait_done(self, timeout=30.0):
        """阻塞等待当前运动指令执行完成。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            error, done = self.robot.GetRobotMotionDone()
            if error == 0 and done == 1:
                return True
            time.sleep(0.05)
        raise TimeoutError("等待运动完成超时")

    # ---------- 基础运动 ----------

    def enable(self, on=True):
        """使能/下使能。运动前必须 enable(True)。"""
        error = self.robot.RobotEnable(1 if on else 0)
        if error != 0:
            raise RuntimeError(f"使能失败 err={error}")

    def move_rel(self, dx=0.0, dy=0.0, dz=0.0, drx=0.0, dry=0.0, drz=0.0,
                 vel=10.0, blocking=True):
        """相对移动：在当前位姿基础上偏移（mm / °），笛卡尔直线插补。
        水平移动 dz=0 即可；竖直下降只给 dz 负值。"""
        error, pose = self.robot.GetActualTCPPose()
        if error != 0:
            raise RuntimeError(f"读取位姿失败 err={error}")
        target = [pose[0] + dx, pose[1] + dy, pose[2] + dz,
                  pose[3] + drx, pose[4] + dry, pose[5] + drz]
        error = self.robot.MoveL(target, self.tool, self.user, vel=vel)
        if error != 0:
            raise RuntimeError(f"MoveL 失败 err={error}（目标可能超限或经过奇异点）")
        if blocking:
            self.wait_done()

    def move_rel_ik(self, dx=0.0, dy=0.0, dz=0.0, drx=0.0, dry=0.0, drz=0.0,
                    vel=10.0, blocking=True):
        """相对移动（IK+关节空间）：MoveL 撞奇异点/超限（err=99/112/185）时的替代。
        锁定当前构型分支：逼近限位时宁可报错也不跳分支做大动作。"""
        pose = self.get_pose()
        target = [pose[0] + dx, pose[1] + dy, pose[2] + dz,
                  pose[3] + drx, pose[4] + dry, pose[5] + drz]
        self.move_ik(target, vel=vel, blocking=blocking, config=self._current_config())

    def move_to(self, pose, vel=20.0, blocking=True):
        """绝对移动：笛卡尔直线到指定位姿 [x,y,z,rx,ry,rz]。"""
        error = self.robot.MoveL(list(pose), self.tool, self.user, vel=vel)
        if error != 0:
            raise RuntimeError(f"MoveL 失败 err={error}")
        if blocking:
            self.wait_done()

    def move_joints(self, joints, vel=20.0, blocking=True):
        """关节空间移动（大范围转移/换姿态用），joints 为 6 个关节角（°）。"""
        error = self.robot.MoveJ(list(joints), self.tool, self.user, vel=vel)
        if error != 0:
            raise RuntimeError(f"MoveJ 失败 err={error}")
        if blocking:
            self.wait_done()

    def stop(self):
        """紧急停止当前运动。"""
        return self.robot.StopMotion()

    # ---------- 示教与航点 ----------

    def jog(self, axis, direction, step, vel=10.0):
        """键盘单步点动：axis 1~6（x,y,z,rx,ry,rz），direction ±1，step mm 或 °。
        在基坐标系下点动，单次最大位移被 max_dis=step 限制。"""
        error = self.robot.StartJOG(2, axis, 1 if direction > 0 else 0, step, vel=vel)
        if error != 0:
            raise RuntimeError(f"StartJOG 失败 err={error}")
        time.sleep(0.4)
        self.robot.StopJOG(3)

    def set_mode(self, manual=True):
        """切换手动/自动模式。拖动示教必须在手动模式（robot_mode=1）下进行。
        幂等：已是目标模式时直接返回（重复切换会报 err=14）。"""
        err, pkg = self.robot.GetRobotRealTimeState()
        if err == 0 and pkg.robot_mode == (1 if manual else 0):
            return
        error = self.robot.Mode(1 if manual else 0)
        for _ in range(3):
            if error == 0:
                return
            time.sleep(1)  # 关拖动态后立刻切模式会报 err=123，缓 1s 重试
            error = self.robot.Mode(1 if manual else 0)
        raise RuntimeError(f"模式切换失败 err={error}（先检查控制器是否有未复位报警）")

    def drag_teach(self, on=True):
        """拖动示教开关：on=True 后可以手把机械臂拖到目标位置。
        注意：进入前必须 set_mode(True) 切手动模式，否则 err=-1。"""
        error = self.robot.DragTeachSwitch(1 if on else 0)
        if error != 0:
            raise RuntimeError(f"拖动示教切换失败 err={error}")

    def save_waypoint(self, name):
        """把当前位姿记为命名航点（配合拖动示教使用）。
        同时记录 TCP 位姿和 6 个关节角——关节角可用于走 MoveJ，
        保证回到示教时的同一构型（不翻肘）。"""
        pose = self.get_pose()
        err, joints = self.robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"读取关节角失败 err={err}")
        waypoints = {}
        if os.path.exists(self.waypoint_file):
            with open(self.waypoint_file) as f:
                waypoints = json.load(f)
        waypoints[name] = {"pose": pose, "joints": list(joints)}
        with open(self.waypoint_file, "w") as f:
            json.dump(waypoints, f, indent=2, ensure_ascii=False)
        print(f"航点 [{name}] 已保存到 {os.path.basename(self.waypoint_file)}: {[round(p, 1) for p in pose]}")

    @staticmethod
    def _entry_pose(entry):
        """兼容旧格式（纯位姿列表）与新格式（{pose, joints}）。"""
        return entry["pose"] if isinstance(entry, dict) else entry

    def list_waypoints(self):
        """列出全部航点：{name: {"pose": [...], "joints": [...] | None}}（只读取，不运动）。
        旧格式航点 joints 为 None。"""
        if not os.path.exists(self.waypoint_file):
            return {}
        with open(self.waypoint_file) as f:
            raw = json.load(f)
        return {name: {"pose": self._entry_pose(entry),
                       "joints": entry.get("joints") if isinstance(entry, dict) else None}
                for name, entry in raw.items()}

    def get_waypoint(self, name):
        """读取已保存航点的位姿 [x,y,z,rx,ry,rz]（只读取，不运动）。"""
        with open(self.waypoint_file) as f:
            waypoints = json.load(f)
        if name not in waypoints:
            raise KeyError(f"航点 [{name}] 不存在，已有: {list(waypoints)}")
        return self._entry_pose(waypoints[name])

    def get_waypoint_joints(self, name):
        """读取航点保存时的 6 关节角；旧格式航点没有关节角则返回 None。"""
        with open(self.waypoint_file) as f:
            waypoints = json.load(f)
        if name not in waypoints:
            raise KeyError(f"航点 [{name}] 不存在，已有: {list(waypoints)}")
        entry = waypoints[name]
        return entry.get("joints") if isinstance(entry, dict) else None

    def goto_waypoint(self, name, vel=20.0):
        """运动到已保存的命名航点。"""
        self.move_to(self.get_waypoint(name), vel=vel)

    def close(self):
        self.robot.CloseRPC()


if __name__ == "__main__":
    # 自测：连接 → 读位姿 → 上移 20mm → 下移 20mm 复位
    arm = Arm()
    print("当前位姿:", [round(p, 1) for p in arm.get_pose()])
    arm.enable(True)
    print("上移 20mm")
    arm.move_rel(dz=20, vel=5)
    print("下移 20mm 复位")
    arm.move_rel(dz=-20, vel=5)
    print("完成:", [round(p, 1) for p in arm.get_pose()])
    arm.close()
