#!/usr/bin/env python3
"""
任务 13 对比（仿真，未在真机验证）：抓取提速 + 确认防误报 + 灵敏度学习去偏。无底盘、低平台，每档 N 台车 × M 次。

仿真环境 = 任务 09 的真实因素（sim_accuracy.ENV：落点偏右 3~5mm、横向容差 ±5mm、底座回差 20us、伸出推底座、R_CAM=0）
          + 真机现在的做法（VERIFY="observe"、固定微调 trim 4）
          + 任务 13 新因素：合爪没夹住时有 SIM_CLAW_PUSH_P 的概率把物块推开 10~40mm

  python3 sim_speed_verify.py                 默认 10 台车 × 100 次
  python3 sim_speed_verify.py --cars 3 --n 40 快速看一眼
  python3 sim_speed_verify.py --set SIM_CLAW_PUSH_P=0.3     改仿真环境（敏感性分析）
  python3 sim_speed_verify.py --backlash      底座回差 0 / 20 / 40us 下 J 的偏差
"""
import argparse
import importlib
import os
import random
import statistics
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("GRASP_IGNORE_CALIB", "1")      # 按 config.py 默认值，不受这台车 calib.json 影响

import config  # noqa: E402
from devices import DeviceError  # noqa: E402
from sim_accuracy import ENV as ENV09, FIXED_TRIM, LEARN  # noqa: E402
from task import GraspTask  # noqa: E402

ENV = {**ENV09, **FIXED_TRIM, "VERIFY": "observe", "SIM_CLAW_PUSH_P": 0.5}
J_TRUE_X = -1.0           # R_CAM_MM = 0、相机真实半径 0：底座转 1°，方位角正好变 −1°（没有回差时）

WHOLE = {"VERIFY_WHOLE_FRAME": True}
SKIP = {"VERIFY_SKIP_CARRY": True}
SCALE = {"ARM_MOVE_SCALE": True}
SAME = {"LEARN_SAME_DIR_ONLY": True}
ALL13 = {**WHOLE, **SKIP, **SCALE, **SAME}

ROWS = [
    ("① 现状（任务 13 全关）", {}),
    ("② 整画面确认 VERIFY_WHOLE_FRAME", WHOLE),
    ("③ 去掉确认前 carry VERIFY_SKIP_CARRY", SKIP),
    ("④ 动作时间缩放 ARM_MOVE_SCALE", SCALE),
    ("⑤ 同向才学 J LEARN_SAME_DIR_ONLY", SAME),
    ("⑥ 任务 13 全开", ALL13),
    ("⑦ 现状 + 任务 09 试探学习", LEARN),
    ("⑧ 任务 13 全开 + 任务 09 试探学习", {**ALL13, **LEARN}),
]


def make_cfg(over):
    importlib.reload(config)
    for k, v in over.items():
        setattr(config, k, v)
    return config


def run_car(over, seed, n):
    from sim import SIM_POSES, make_car, make_sim
    cfg = make_cfg(over)
    cfg.USE_CHASSIS = False
    cfg.POSES = dict(SIM_POSES)
    rng = random.Random(seed)
    bias = rng.uniform(0.88, 1.12)
    r_cam = cfg.R_CAM_MM * rng.uniform(0.8, 1.25)
    car = make_car(cfg, seed)
    log_dir = tempfile.mkdtemp(prefix="grasp_sv_")
    task, rows = None, []
    for i in range(n):
        world, clock, cam, lift, arm, chassis = make_sim(cfg, seed * 1000 + i, bias, r_cam, car)
        if task is None:
            task = GraspTask(cfg, clock, cam, lift, arm, chassis, log_dir=log_dir, learn=True, verbose=False)
        else:
            task.clock, task.cam, task.lift, task.arm, task.chassis = clock, cam, lift, arm, chassis
            task.base_deg = 0.0
            task._base_sent, task._base_dir = None, 0
        ok_end = True
        try:
            r = task.run_once()
        except Exception as e:
            r = {"result": "device_error" if isinstance(e, DeviceError) else "crash", "attempts": 0, "steps": 0,
                 "time_s": round(clock.now(), 1)}
            task._lifted = False
            ok_end = world.removed
        truth = world.picked if ok_end else world.removed
        rows.append({**r, "truth": truth, "claw_pushes": world.claw_pushes})
    return rows, task.axes["x_deg"].j, task.learn_skipped_reversal


def summarize(title, over, cars, n, extra=None, env=None):
    allover = {**(ENV if env is None else env), **over, **(extra or {})}
    rows, js = [], []
    for seed in range(1, cars + 1):
        rs, j, _ = run_car(allover, seed, n)
        rows += rs
        js.append(j)
    tot = len(rows)
    ok = sum(r["truth"] for r in rows) / tot
    false_ok = sum(1 for r in rows if r["result"] == "success" and not r["truth"])
    t = statistics.mean(r["time_s"] for r in rows)
    t_ok = [r["time_s"] for r in rows if r["truth"] and r["result"] == "success"]
    pushed_ep = sum(1 for r in rows if r["claw_pushes"])
    found = sum(1 for r in rows if r.get("pushes"))
    jb = statistics.mean(j / J_TRUE_X - 1 for j in js)
    jr = (min(js) / J_TRUE_X - 1, max(js) / J_TRUE_X - 1)
    print(f"| {title} | {ok:.1%} | {false_ok} ({false_ok / tot:.1%}) | {t:.1f} | "
          f"{statistics.mean(t_ok) if t_ok else 0:.1f} | {pushed_ep} | {found} | "
          f"{100 * jb:+.1f}%（{100 * min(jr):+.0f}~{100 * max(jr):+.0f}%） |", flush=True)
    return {"ok": ok, "false_ok": false_ok, "t": t, "jb": jb}


def arm_time(over):
    """
    用 config.py 里真机的姿态（arm.txt 录的脉宽，不是仿真那套可区分的假姿态）走一遍
    "抓取 → 观察确认 → 放置"，统计机械臂动作一共要等多久（Arm.move 每帧等 ms + 0.15s）。
    返回 (成功一次的机械臂时间, 没抓到一次的机械臂时间, 每帧明细)
    """
    import devices
    cfg = make_cfg({"VERIFY": "observe", **over})
    cfg.USE_CHASSIS = False

    class Clock:
        t = 0.0
        def now(self): return self.t
        def sleep(self, s): self.t += max(0.0, s)

    class Arm:
        def __init__(self, clock): self.clock, self.log = clock, []
        def move(self, pulses, ms):
            self.log.append((dict(pulses), int(ms)))
            self.clock.sleep(ms / 1000.0 + 0.15)

    class Lift:
        pos = 10.0
        def move_by(self, cm, timeout=30): pass

    class Cam:
        def __init__(self, see): self.see = see
        def measure(self, since, n, timeout):
            return devices.Measurement(0.0, 0.0, 180.0, n) if self.see else None

    out = []
    for see in (True, False):              # 看得到（原位置）= 没抓到；看不到 = 抓到
        clock = Clock()
        arm = Arm(clock)
        t = GraspTask(cfg, clock, Cam(see), Lift(), arm, None, verbose=False, learn=False)
        t.go_observe(0)                    # 先让程序知道舵机在哪（真机上一次动作之后就知道）
        n0 = len(arm.log)
        ok = t.grasp(devices.Measurement(0.0, 0.0, 180.0, 5))
        if ok:
            t.place()
        else:
            t.release_and_back_off()
        log = arm.log[n0:]
        out.append((sum(ms / 1000.0 + 0.15 for _, ms in log), log))
    return out[1][0], out[0][0], out[1][1]


HEAD = ("| 配置 | 真实抓到率 | 误报成功（占全部） | 平均用时 s | 成功那次平均用时 s | 被推开过的物块 | "
        "程序发现推开 | 学到的 J 偏差（车间范围） |\n|---|---|---|---|---|---|---|---|")


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cars", type=int, default=10)
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--backlash", action="store_true")
    ap.add_argument("--arm-time", action="store_true", help="按真机姿态估算每次抓取的机械臂动作时间")
    ap.add_argument("--set", action="append", default=[], help="额外覆盖 config，例如 --set SIM_CLAW_PUSH_P=0.3")
    a = ap.parse_args()
    extra = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        extra[k] = eval(v)
    print(f"无底盘，每档 {a.cars} 台车 × {a.n} 次" + (f"，额外 {extra}" if extra else "") + "（仿真，未在真机验证）\n")
    if a.arm_time:
        print("| 配置 | 抓到一次（伸出→确认→放置）机械臂动作 s | 没抓到一次（伸出→确认→松开回观察）s | 动作帧数（抓到） |")
        print("|---|---|---|---|")
        for title, over in (("现状", {}), ("去掉确认前 carry", SKIP), ("动作时间缩放", SCALE),
                            ("两项都开", {**SKIP, **SCALE})):
            ok_t, fail_t, log = arm_time(over)
            print(f"| {title} | {ok_t:.2f} | {fail_t:.2f} | {len(log)} |")
        print("\n抓到一次的动作明细（两项都开）：")
        for pulses, ms in arm_time({**SKIP, **SCALE})[2]:
            print(f"  {ms:5d}ms  {pulses}")
        return
    print(HEAD)
    if a.backlash:
        for bl in (0, 20, 40):
            e = {**extra, "SIM_SERVO_BACKLASH_US": bl}
            summarize(f"底座回差 {bl}us，每步都学", {}, a.cars, a.n, extra=e)
            summarize(f"底座回差 {bl}us，同向才学", SAME, a.cars, a.n, extra=e)
        return
    for title, over in ROWS:
        summarize(title, over, a.cars, a.n, extra=extra)


if __name__ == "__main__":
    main_()
