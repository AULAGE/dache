#!/usr/bin/env python3
"""
任务 09 抓取精度对比（仿真，未在真机验证）：无底盘、低平台，每档 N 台车 × M 次。

仿真环境在默认仿真的基础上打开任务 09 的真实因素（见 sim.py 文件头 8~10）：
  每台车固定的落点偏右 3~5mm（随深度再变 ±1.5mm）、横向容差 ±5mm、底座回差 20us、伸出时底座被推到回差一侧，
  相机在底座转轴上（R_CAM_MM=0，和真机标定一致）

  python3 sim_accuracy.py               默认 10 台车 × 100 次
  python3 sim_accuracy.py --cars 3 --n 40   快速看一眼
  python3 sim_accuracy.py --probe-sweep  试探量 p 和学习率扫一遍
  python3 sim_accuracy.py --mid          中间深度高度偏差：两点插值 vs 标了 reach_mid 的三点插值
  python3 sim_accuracy.py --set SIM_GRIP_TOL_MM=4.0    改仿真环境（敏感性分析）
"""
import argparse
import importlib
import math
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
from task import GraspTask  # noqa: E402

# 仿真环境：任务 09 的真实因素（估计值，见报告）
ENV = {
    "SIM_GRASP_BIAS_MM": (3.0, 5.0),
    "SIM_GRASP_BIAS_VAR_MM": 1.5,
    "SIM_GRIP_TOL_MM": 5.0,   # 爪子张开约 34mm（估计）−物块 24mm → ±5mm；报告里也给了 4 / 6 的结果
    "SIM_SERVO_BACKLASH_US": 20,
    "SIM_REACH_SHIFT": True,
    "R_CAM_MM": 0.0,          # 真机标定：相机几乎在底座转轴上（calib.json R=0），calib.py trim 的换算也按这个
}
BACKLASH = {"BASE_BACKLASH_DEG": 2.0, "BASE_MIN_STEP_DEG": 1.0}
PROBE = {"GRASP_PROBE_MM": 3.0}
LEARN = {"GRASP_PROBE_MM": 3.0, "TRIM_LEARN": True}
FIXED_TRIM = {"GRASP_BASE_TRIM_DEG": -1.27}            # 用户现在的做法：calib.py trim 4（按深度 180mm）
IDEAL = {"SIM_GRASP_BIAS_MM": 0.0, "SIM_GRASP_BIAS_VAR_MM": 0.0, "SIM_REACH_SHIFT": False}

ROWS = [
    ("原仿真（不加任务 09 的真实因素）", {}, False),
    ("参考上限：没有落点偏差、伸出不推底座（其余同下）", IDEAL, True),
    ("① 现状（真实因素全开，改进全关）", {}, True),
    ("② 现状 + 固定微调 trim 4（−1.27°，真机现在的做法）", FIXED_TRIM, True),
    ("③ + 消回差（BASE_BACKLASH_DEG=2, BASE_MIN_STEP_DEG=1）", BACKLASH, True),
    ("④ + 消回差 + 固定微调 trim 4", {**BACKLASH, **FIXED_TRIM}, True),
    ("⑤ + 失败左右试探（GRASP_PROBE_MM=3）", PROBE, True),
    ("⑥ + 试探 + 学习（TRIM_LEARN）", LEARN, True),
    ("⑦ + 试探 + 学习 + 固定微调 trim 4", {**LEARN, **FIXED_TRIM}, True),
    ("⑧ 全开（消回差 + 试探 + 学习）", {**BACKLASH, **LEARN}, True),
    ("⑨ 全开 + 固定微调 trim 4", {**BACKLASH, **LEARN, **FIXED_TRIM}, True),
]


def make_cfg(over):
    importlib.reload(config)
    for k, v in over.items():
        setattr(config, k, v)
    return config


def run_car(over, seed, n, mid=False):
    """一台车跑 n 次，返回每次的记录和最后学到的微调"""
    from sim import SIM_POSES, make_car, make_sim
    cfg = make_cfg(over)
    cfg.USE_CHASSIS = False
    cfg.POSES = dict(SIM_POSES)
    if mid:                                   # 标了中间姿态：p3 = 抛物线在 t=0.5 处的真值（见 sim._height_err）
        sag = cfg.SIM_REACH_SAG_MM
        cfg.POSES["reach_mid"] = {2: 1500, 3: int(round(1250 - sag)), 4: 1500, 5: 1500}
        cfg.REACH_MID_D = (cfg.REACH_NEAR_D + cfg.REACH_FAR_D) / 2.0
    rng = random.Random(seed)
    bias = rng.uniform(0.88, 1.12)
    r_cam = cfg.R_CAM_MM * rng.uniform(0.8, 1.25)
    car = make_car(cfg, seed)
    log_dir = tempfile.mkdtemp(prefix="grasp_acc_")
    task, rows = None, []
    for i in range(n):
        world, clock, cam, lift, arm, chassis = make_sim(cfg, seed * 1000 + i, bias, r_cam, car)
        if task is None:
            task = GraspTask(cfg, clock, cam, lift, arm, chassis, log_dir=log_dir, learn=True, verbose=False)
        else:
            task.clock, task.cam, task.lift, task.arm, task.chassis = clock, cam, lift, arm, chassis
            task.base_deg = 0.0
            task._base_sent, task._base_dir = None, 0      # 新场景：底座状态未知（和 run_sim 一样每次重建机械臂）
        ok_end = True
        try:
            r = task.run_once()
        except Exception as e:
            r = {"result": "device_error" if isinstance(e, DeviceError) else "crash", "attempts": 0, "steps": 0,
                 "time_s": round(clock.now(), 1)}
            task._lifted = False
            ok_end = world.removed
        truth = world.picked if ok_end else world.removed
        rows.append({**r, "truth": truth})

    # 学到的微调 vs 这台车真正需要的微调（每段中心深度处）
    conv = None
    if task.trim_state is not None:
        edges = task._trim_edges()
        lo, hi = sorted((cfg.REACH_NEAR_D, cfg.REACH_FAR_D))
        bounds = [lo] + edges + [hi]
        a = cfg.BASE_APPROACH_DIR * cfg.BASE_DIR
        bl = cfg.SIM_SERVO_BACKLASH_US
        if cfg.SIM_REACH_SHIFT and cfg.BASE_BACKLASH_DEG > 0:
            shift_deg = (car["reach_side"] + a) * bl / 2.0 / (cfg.BASE_US_PER_DEG * cfg.BASE_DIR)
        elif cfg.SIM_REACH_SHIFT:
            shift_deg = car["reach_side"] * bl / 2.0 / (cfg.BASE_US_PER_DEG * cfg.BASE_DIR)   # 方向随机，取期望
        else:
            shift_deg = 0.0
        conv = []
        for s, seg in enumerate(task.trim_state["segments"]):
            dm = (bounds[s] + bounds[s + 1]) / 2.0                   # K230 读数下的深度
            kk = (dm - (lo + hi) / 2.0) / max((hi - lo) / 2.0, 1.0)
            b_mm = car["grasp_bias_mm"] + car["grasp_bias_var_mm"] * max(-1.0, min(1.0, kk))
            dt = dm / bias
            best = -math.degrees(b_mm / (bias * (dt + r_cam))) - shift_deg - cfg.GRASP_BASE_TRIM_DEG
            conv.append((seg["trim_deg"], best, seg["probe_ok_left"] + seg["probe_ok_right"]))
    return rows, conv


def summarize(title, over, env, cars, n, mid=False, extra=None):
    allover = {**(ENV if env else {}), **over, **(extra or {})}
    rows, convs = [], []
    halves = [[], []]
    for seed in range(1, cars + 1):
        rs, conv = run_car(allover, seed, n, mid)
        rows += rs
        halves[0] += rs[:n // 2]
        halves[1] += rs[n // 2:]
        if conv:
            convs += conv
    ok = sum(r["truth"] for r in rows) / len(rows)
    retry = statistics.mean(max(0, r["attempts"] - 1) for r in rows)
    steps = statistics.mean(r["steps"] for r in rows)
    t = statistics.mean(r["time_s"] for r in rows)
    false_ok = sum(1 for r in rows if r["result"] == "success" and not r["truth"])
    h2 = sum(r["truth"] for r in halves[1]) / max(1, len(halves[1]))
    conv_txt = ""
    if convs:
        err0 = statistics.mean(abs(b) for _, b, _ in convs)
        err1 = statistics.mean(abs(l - b) for l, b, _ in convs)
        upd = statistics.mean(k for _, _, k in convs)
        conv_txt = f"{err0:.2f}° → {err1:.2f}°（每段平均学了 {upd:.1f} 次）"
    print(f"| {title} | {ok:.1%} | {h2:.1%} | {retry:.2f} | {steps:.2f} | {t:.1f} | {false_ok} | {conv_txt} |",
          flush=True)
    return ok


HEAD = ("| 配置 | 真实抓到率 | 后一半抓到率 | 平均重试次数 | 平均对准步数 | 平均用时 s | 误报成功 | "
        "微调误差：初始 → 学完（|学到 − 应有|） |\n|---|---|---|---|---|---|---|---|")


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cars", type=int, default=10)
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--probe-sweep", action="store_true")
    ap.add_argument("--mid", action="store_true")
    ap.add_argument("--set", action="append", default=[], help="额外覆盖 config，例如 --set SIM_SERVO_BACKLASH_US=10")
    a = ap.parse_args()
    extra = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        extra[k] = eval(v)
    print(f"无底盘，每档 {a.cars} 台车 × {a.n} 次" + (f"，额外 {extra}" if extra else "") + "（仿真，未在真机验证）\n")
    print(HEAD)
    if a.probe_sweep:
        for p in (2.0, 3.0, 4.0, 6.0):
            for lr in (0.3, 0.5, 1.0):
                summarize(f"全开 p={p:.0f}mm 学习率 {lr}", {**BACKLASH, **LEARN, "GRASP_PROBE_MM": p,
                                                         "TRIM_LEARN_RATE": lr}, True, a.cars, a.n, extra=extra)
        return
    if a.mid:
        for sag in (0.0, 6.0, 10.0):
            e = {**extra, "SIM_REACH_SAG_MM": sag}
            summarize(f"中间高度偏 {sag:.0f}mm，两点插值", {**BACKLASH, **LEARN}, True, a.cars, a.n, extra=e)
            if sag:
                summarize(f"中间高度偏 {sag:.0f}mm，标了 reach_mid 三点插值", {**BACKLASH, **LEARN}, True,
                          a.cars, a.n, mid=True, extra=e)
        return
    for title, over, env in ROWS:
        summarize(title, over, env, a.cars, a.n, extra=extra)


if __name__ == "__main__":
    main_()
