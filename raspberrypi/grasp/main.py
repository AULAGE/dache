#!/usr/bin/env python3
"""
全自动抓取 总控程序（运行在树莓派上）

  仿真（电脑上也能跑，不需要硬件）：
      python3 main.py --sim 50
  只读自检（不会让任何东西动）：
      python3 main.py --check
  第一次真机：单步模式，每个动作前按回车确认；先不让底盘动
      python3 main.py --once --step --no-chassis
  真机，抓一个就停：
      python3 main.py --once
  真机，一直抓（开机自启用这个）：
      python3 main.py

  运行日志自动保存在 logs/run_日期_时间.log
"""
import argparse
import os
import statistics
import subprocess
import sys
import time

import config as cfg
from devices import (Arm, DeviceError, K230, Lift, ManualGuard, ManualOverride, NullChassis,
                     RealClock, WheeltecChassis)
from task import GraspTask

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, cfg.LOG_DIR)


# ============================================================ 仿真
def run_sim(n, seed, learn, no_chassis=False):
    import random
    import tempfile
    from sim import SIM_POSES, make_car, make_sim

    cfg.POSES = SIM_POSES                 # 仿真用可区分的姿态值
    if no_chassis:
        cfg.USE_CHASSIS = False
    rng = random.Random(seed)
    bias = rng.uniform(0.88, 1.12)        # 同一台车：深度标定误差固定
    r_cam = cfg.R_CAM_MM * rng.uniform(0.8, 1.25)   # 相机半径量得不准
    print(f"仿真：{'无底盘' if not cfg.USE_CHASSIS else '有底盘'}  深度系统误差 {100 * (bias - 1):+.1f}%  "
          f"相机真实半径 {r_cam:.0f}mm（配置 {cfg.R_CAM_MM:.0f}mm）  学习 {'开' if learn else '关'}")

    log_dir = tempfile.mkdtemp(prefix="grasp_sim_") if learn else None
    car = make_car(cfg, seed)              # 同一台车：滑行 τ、测深框偏差等固定
    rows, truth = [], []
    task = None
    for i in range(n):
        world, clock, cam, lift, arm, chassis = make_sim(cfg, seed * 1000 + i, bias, r_cam, car)
        if task is not None and hasattr(task.chassis, "coast_tau") and hasattr(chassis, "coast_tau"):
            chassis.coast_tau = task.chassis.coast_tau     # 真机把学到的 τ 存文件，下次接着用
        if task is None:
            task = GraspTask(cfg, clock, cam, lift, arm, chassis, log_dir=log_dir, learn=learn,
                             verbose=(n == 1))
        else:       # 换一个新场景，但保留学到的灵敏度（模拟同一台车反复使用）
            task.clock, task.cam, task.lift, task.arm, task.chassis = clock, cam, lift, arm, chassis
            task.base_deg = 0.0
        ok_end = True
        try:
            rows.append(task.run_once())
        except Exception as e:      # 真机上 DeviceError 会重开设备再来，其它异常会让程序退出：都算这次失败
            kind = "device_error" if isinstance(e, DeviceError) else "crash"
            if n == 1:
                print(f"异常（{kind}）：{e!r}")
            rows.append({"episode": i + 1, "result": kind, "attempts": 0, "steps": 0,
                         "time_s": round(clock.now(), 1), "err_norm": 0.0,
                         "score": round(-cfg.SCORE_TIME_COST * clock.now(), 1)})
            task._lifted = False
            ok_end = world.removed          # 中途出错：已经放进筐里才算抓到
        truth.append(world.picked if ok_end else world.removed)

    def stats(rs, tr, title):
        ok = sum(tr)
        print(f"{title}: 真实抓到 {ok}/{len(rs)} = {100 * ok / len(rs):.0f}%   "
              f"平均对准步数 {statistics.mean(r['steps'] for r in rs):.1f}   "
              f"平均用时 {statistics.mean(r['time_s'] for r in rs):.1f}s   "
              f"平均评分 {statistics.mean(r['score'] for r in rs):.1f}")

    stats(rows, truth, "全部")
    if n >= 20:
        h = n // 2
        stats(rows[:h], truth[:h], "前一半")
        stats(rows[h:], truth[h:], "后一半")
    print("最终学到的灵敏度 J：", {k: round(a.j, 3) for k, a in task.axes.items()},
          " 初值：", {k: cfg.AXES[k]["j0"] for k in task.axes})
    fails, false_ok = {}, 0
    for r, t in zip(rows, truth):
        if not t:
            fails[r["result"]] = fails.get(r["result"], 0) + 1
        if r["result"] == "success" and not t:
            false_ok += 1
    retries = sum(1 for r in rows if r["attempts"] > 1)
    print(f"重试过的次数：{retries}   程序报成功但实际没抓到：{false_ok}")
    if fails:
        print("未抓到的原因：", fails)
    return sum(truth) / len(truth)


# ============================================================ 真机设备
def open_devices(clock):
    cam = K230(cfg.PORTS["k230"], cfg.BAUD["k230"], clock, cfg.TARGET_TIMEOUT_S, cfg.K230_ROTATE)
    lift = Lift(cfg.PORTS["lift"], cfg.BAUD["lift"], cfg.LIFT_MAX_CM)
    arm = Arm(cfg.PORTS["arm"], cfg.BAUD["arm"], clock, robust=cfg.ARM_USB_ROBUST)
    chassis = WheeltecChassis(cfg.PORTS["chassis"], cfg.BAUD["chassis"], cfg.CHASSIS_SPEED_MM_S,
                              cfg.ACK_MIN_RADIUS_MM, cfg.CHASSIS_ACKERMANN,
                              os.path.join(LOG_DIR, "chassis_tau.json")) \
        if cfg.USE_CHASSIS else NullChassis()
    return cam, lift, arm, chassis


def close_devices(devs):
    for d in devs:
        try:
            if hasattr(d, "stop"):
                d.stop()
        except Exception:
            pass
        try:
            if hasattr(d, "close"):
                d.close()
            if hasattr(d, "ser"):
                d.ser.close()
        except Exception:
            pass


class StepProxy:
    """单步模式：每个会让东西动的调用之前，打印要做什么，等操作员按回车"""
    MOTION = {"move_to", "move_by", "start_move_by", "drive", "forward", "turn", "move"}

    def __init__(self, dev, name):
        self._dev, self._name = dev, name

    def __getattr__(self, k):
        attr = getattr(self._dev, k)
        if k not in self.MOTION or not callable(attr):
            return attr

        def wrapped(*a, **kw):
            args = ", ".join([repr(x) if not isinstance(x, float) else f"{x:.2f}" for x in a] +
                             [f"{kk}={vv!r}" for kk, vv in kw.items()])
            ans = input(f"  ▶ {self._name}.{k}({args})   回车=执行  q=停止：").strip().lower()
            if ans == "q":
                raise KeyboardInterrupt
            return attr(*a, **kw)
        return wrapped


class Tee:
    """同时输出到屏幕和日志文件"""

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "a", encoding="utf-8")
        self.out = sys.stdout

    def write(self, s):
        self.out.write(s)
        self.f.write(s)
        self.f.flush()

    def flush(self):
        self.out.flush()


def manual_running():
    # 直接调用 pgrep（不经过 shell），否则 shell 自己的命令行里含有进程名，会误报
    return subprocess.run(["pgrep", "-f", cfg.MANUAL_PROCESS], capture_output=True).returncode == 0


def preflight(cam, lift, arm, chassis):
    """真机启动前检查，返回问题列表（空 = 全部通过）"""
    problems = []

    def item(ok, text, fix=""):
        print(("  ✅ " if ok else "  ❌ ") + text + ("" if ok or not fix else f"  → {fix}"))
        if not ok:
            problems.append(text)

    print("启动前检查：")
    item(cfg.POSES_CALIBRATED, "机械臂姿态已标定", "用 calib.py 标定后把 POSES_CALIBRATED 设为 true")
    item(not manual_running(), "手柄控制程序没在运行", "断开手柄，或 systemctl --user stop pad-watchdog")
    t0 = time.time()
    while not cam.link_ok() and time.time() - t0 < 2.0:
        time.sleep(0.1)
    item(cam.link_ok(), "K230 有数据（位置帧或心跳）",
         "检查 K230 是否在运行、GPIO3→树莓派物理脚 29、GND→物理脚 30")
    try:
        p = lift.position()
        item(True, f"升降轴位置 {p:.2f}cm")
    except DeviceError as e:
        item(False, f"升降轴：{e}")
    if cfg.USE_CHASSIS:
        fb = chassis.has_feedback()
        item(fb, "底盘有回传", "检查 /dev/ttyACM0、底盘电源开关")
        if fb:
            v = chassis.voltage_mv / 1000
            item(v >= cfg.BATTERY_MIN_V, f"底盘电池 {v:.2f}V（下限 {cfg.BATTERY_MIN_V}V）", "充电")
    print("  ℹ 机械臂控制板不回传状态，无法自检；第一次运行建议加 --step")
    return problems


# ============================================================ 只读自检
def run_check():
    """只读自检：不发任何运动指令"""
    import glob
    print("串口：")
    for p in sorted(glob.glob("/dev/serial/by-path/*") + glob.glob("/dev/serial/by-id/*")):
        if "usbv2" not in p:
            print("  ", p, "->", os.path.realpath(p))
    print("手柄控制程序：", "运行中（会占用底盘和升降轴串口！）" if manual_running() else "未运行")
    print("calib.json 覆盖的参数：", sorted(cfg.CALIB_LOADED) or "（没有，全部用 config.py 默认值）")
    for name in ("lift", "chassis", "k230", "arm"):
        print(f"{name:8s}", cfg.PORTS[name], "存在" if os.path.exists(cfg.PORTS[name]) else "【不存在】")
    if os.path.exists(cfg.PORTS["lift"]):
        try:
            lift = Lift(cfg.PORTS["lift"], cfg.BAUD["lift"])
            print("  升降轴：", lift.cmd("P"), "|", lift.cmd("V"))
            lift.ser.close()
        except Exception as e:
            print("  升降轴：", e)
    if cfg.USE_CHASSIS and os.path.exists(cfg.PORTS["chassis"]):
        import serial
        s = serial.Serial(cfg.PORTS["chassis"], cfg.BAUD["chassis"], timeout=0.2)
        t, buf = time.time(), b""
        while time.time() - t < 1.0:
            buf += s.read(256)
        s.close()
        i = buf.find(b"\x7B")
        volt = None
        while 0 <= i <= len(buf) - 24:
            f = buf[i:i + 24]
            b = 0
            for x in f[:22]:
                b ^= x
            if f[23] == 0x7D and b == f[22]:
                volt = int.from_bytes(f[20:22], "big") / 1000
                break
            i = buf.find(b"\x7B", i + 1)
        print(f"  底盘：1 秒收到 {len(buf)} 字节" + (f"，电池 {volt:.2f}V" if volt else "，没有解析到状态帧"))
    if os.path.exists(cfg.PORTS["k230"]):
        clock = RealClock()
        cam = K230(cfg.PORTS["k230"], cfg.BAUD["k230"], clock, cfg.TARGET_TIMEOUT_S, cfg.K230_ROTATE)
        time.sleep(2.0)
        print(f"  K230：2 秒收到 位置帧 {len(cam.frames)}、心跳 {cam.heartbeats}、坏帧 {cam.bad_frames}",
              ("，最新 x=%d y=%d d=%d" % cam.frames[-1][1:]) if cam.frames else "")
        if not cam.frames and not cam.heartbeats:
            print("  ⚠ K230 什么都没发：检查 K230 是否上电、程序是否在运行、TX→GPIO5 和 GND 是否接好")
        elif not cam.frames:
            print("  K230 链路正常，但画面里没识别到物块（或物块碰到画面边缘）")
        cam.close()


# ============================================================ 真机运行
def run_real(once, step, force):
    clock = RealClock()
    guard = ManualGuard(cfg.MANUAL_PROCESS)
    time.sleep(1.2)
    task = None
    devs = None
    checked = False
    try:
        while True:
            if guard.active:
                if devs:
                    close_devices(devs)          # 让出串口给手柄程序
                    devs = None
                    print("手柄接管中，串口已释放，等待手柄断开……")
                time.sleep(1.0)
                continue
            if devs is None:
                devs = open_devices(clock)
                if not checked:
                    problems = preflight(*devs)
                    checked = True
                    if problems and not force:
                        print(f"有 {len(problems)} 项检查没通过，不启动（确认安全的话可以加 --force 跳过）")
                        return
                cam, lift, arm, chassis = devs
                if step:
                    lift, arm, chassis = (StepProxy(lift, "升降轴"), StepProxy(arm, "机械臂"),
                                          StepProxy(chassis, "底盘"))
                if task is None:
                    task = GraspTask(cfg, clock, cam, lift, arm, chassis, log_dir=LOG_DIR, guard=guard)
                else:
                    task.cam, task.lift, task.arm, task.chassis = cam, lift, arm, chassis
            try:
                r = task.run_once()
            except ManualOverride as e:
                print(e)
                continue
            except DeviceError as e:
                print("设备错误：", e)
                close_devices(devs)
                devs = None
                if once:
                    raise
                time.sleep(3)
                continue
            if once:
                break
            if r["result"] == "not_found":
                time.sleep(2)       # 周围没有物块，等一会再找
    except KeyboardInterrupt:
        print("\n手动停止：升降轴急停、底盘停车")
    finally:
        if devs:
            close_devices(devs)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="全自动抓取总控")
    ap.add_argument("--sim", type=int, metavar="N", help="仿真 N 次抓取")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-learn", action="store_true", help="关闭自适应学习（对比用）")
    ap.add_argument("--once", action="store_true", help="真机只抓一次")
    ap.add_argument("--step", action="store_true", help="单步模式：每个动作前按回车确认")
    ap.add_argument("--no-chassis", action="store_true", help="不用底盘（固定工位，或者先只调机械臂和升降轴）")
    ap.add_argument("--force", action="store_true", help="启动前检查没通过也继续（自己确认安全）")
    ap.add_argument("--check", action="store_true", help="只读自检：看各串口和下位机是否正常，不动任何东西")
    a = ap.parse_args()
    if a.check:
        run_check()
    elif a.sim:
        run_sim(a.sim, a.seed, not a.no_learn, a.no_chassis)
    else:
        if a.no_chassis:
            cfg.USE_CHASSIS = False
        sys.stdout = Tee(os.path.join(LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.log")))
        print("启动：", " ".join(sys.argv), " calib.json 覆盖：", sorted(cfg.CALIB_LOADED))
        run_real(a.once, a.step, a.force)
