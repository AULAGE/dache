#!/usr/bin/env python3
"""
仿真真实度逐项对比（任务 02）：从"原版仿真"开始，一项一项打开 config.py 里的 SIM_* 因素，
每一档跑 有底盘 / 无底盘 各 N 次 × 若干台车（seed），打印 markdown 表格。

  python3 sim_compare.py                 默认 每档 5 台车 × 100 次
  python3 sim_compare.py --n 40 --seeds 2   快速看一眼
  python3 sim_compare.py --only-all      只跑"全开"一档（看当前默认配置）
"""
import argparse
import contextlib
import importlib
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config  # noqa: E402
import main  # noqa: E402

# 全部关掉 = 原版仿真
OFF = {
    "SIM_K230_TRACKER": False, "SIM_CHASSIS_DYNAMICS": False,
    "SIM_SERVO_DEADBAND_US": 0, "SIM_SERVO_BACKLASH_US": 0, "SIM_PUSH_P": 0.0,
    "SIM_DEPTH_NONLINEAR": False, "SIM_ARM_USB_DROP_P": 0.0, "SIM_FALSE_TARGET_P": 0.0,
}


def _defaults():
    importlib.reload(config)
    return {k: getattr(config, k) for k in OFF}


STEPS = [
    ("原版仿真", []),
    ("+1 K230 逐帧 + 跟踪器滞后", ["SIM_K230_TRACKER"]),
    ("+2 底盘控制器平滑（真机 drive 逻辑）", ["SIM_CHASSIS_DYNAMICS"]),
    ("+3 舵机死区 / 回差", ["SIM_SERVO_DEADBAND_US", "SIM_SERVO_BACKLASH_US"]),
    ("+4 伸出时碰歪物块", ["SIM_PUSH_P"]),
    ("+5 深度随距离非线性", ["SIM_DEPTH_NONLINEAR"]),
    ("+6 机械臂 USB 掉线", ["SIM_ARM_USB_DROP_P"]),
    ("+7 误识别假目标", ["SIM_FALSE_TARGET_P"]),
]

# task.py / devices.py 里的改进（默认关闭），在"全开"基础上逐项打开
FIXES = [
    ("全开 + 改进 A：ARM_USB_ROBUST", {"ARM_USB_ROBUST": True}),
    ("全开 + 改进 A+B：SEARCH_CONFIRM", {"SEARCH_CONFIRM": True}),
    ("全开 + 改进 A+B+C：BASE_LIMIT_BACKUP_MM=150", {"BASE_LIMIT_BACKUP_MM": 150}),
]
NEED_FRAMES = {"SIM_DEPTH_NONLINEAR", "SIM_FALSE_TARGET_P"}   # 这两项在逐帧模型里才完整（单项对比时一起打开 1）

LINE = re.compile(r"真实抓到 (\d+)/(\d+).*平均对准步数 ([\d.]+)\s+平均用时 ([\d.]+)s\s+平均评分 (-?[\d.]+)")


def run(over, n, seeds, no_chassis, extra=None):
    ok = tot = 0
    steps = time_s = score = 0.0
    reasons = {}
    for seed in seeds:
        importlib.reload(config)
        for k, v in {**over, **(extra or {})}.items():
            setattr(config, k, v)
        main.cfg = config
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            main.run_sim(n, seed, True, no_chassis)
        out = buf.getvalue()
        m = LINE.search([ln for ln in out.splitlines() if ln.startswith("全部")][0])
        a, b = int(m.group(1)), int(m.group(2))
        ok += a
        tot += b
        steps += float(m.group(3)) * b
        time_s += float(m.group(4)) * b
        score += float(m.group(5)) * b
        for ln in out.splitlines():
            if ln.startswith("未抓到的原因："):
                for k, v in eval(ln.split("：", 1)[1]).items():     # 自己打印的 dict
                    reasons[k] = reasons.get(k, 0) + v
    importlib.reload(config)
    main.cfg = config
    return ok / tot, steps / tot, time_s / tot, score / tot, reasons


def table(rows_spec, n, seeds, extra=None):
    print("| 仿真因素（累加） | 有底盘 成功率 | 步数 | 用时 s | 评分 | 无底盘 成功率 | 步数 | 用时 s | 评分 | 失败原因（有底盘 / 无底盘） |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    res = []
    for title, over in rows_spec:
        a = run(over, n, seeds, False, extra)
        b = run(over, n, seeds, True, extra)
        res.append((title, a, b))
        print(f"| {title} | {a[0]:.1%} | {a[1]:.2f} | {a[2]:.1f} | {a[3]:.1f} | "
              f"{b[0]:.1%} | {b[1]:.2f} | {b[2]:.1f} | {b[3]:.1f} | {a[4]} / {b[4]} |", flush=True)
    return res


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--only-all", action="store_true")
    ap.add_argument("--single", action="store_true", help="每项单独打开（其它全关），看单项影响")
    ap.add_argument("--fixes", action="store_true", help="最后再加上改进项的对比行")
    ap.add_argument("--set", action="append", default=[], help="额外覆盖 config，例如 --set ARM_USB_ROBUST=True")
    a = ap.parse_args()
    extra = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        extra[k] = eval(v)
    seeds = list(range(1, a.seeds + 1))
    dflt = _defaults()
    spec, cur = [], dict(OFF)
    for title, keys in STEPS:
        if a.single:
            cur = dict(OFF)
            if NEED_FRAMES & set(keys):
                cur["SIM_K230_TRACKER"] = True
                title = title.replace("+", "+1 +", 1)
        for k in keys:
            cur[k] = dflt[k]
        spec.append((title if not a.single or not keys else title.replace("+", "仅 ", 1), dict(cur)))
    if a.only_all:
        spec = [("全开（当前默认）", dflt)]
    if a.fixes:
        cur = dict(dflt)
        for title, over in FIXES:
            cur.update(over)
            spec.append((title, dict(cur)))
    print(f"每档：{len(seeds)} 台车 × {a.n} 次，seed {seeds[0]}~{seeds[-1]}" + (f"，额外 {extra}" if extra else ""))
    table(spec, a.n, seeds, extra)


if __name__ == "__main__":
    main_()
