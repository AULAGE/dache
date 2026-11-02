#!/usr/bin/env python3
"""
看日志：每次抓取的结果统计 + 某一次抓取的逐步对准过程

  python3 report.py            总体统计 + 最近一次抓取的每一步
  python3 report.py 3          第 3 次抓取的每一步
  python3 report.py all        所有抓取的每一步
"""
import csv
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "logs")


def read_csv(name):
    path = os.path.join(LOG, name)
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(x, d=1):
    try:
        return f"{float(x):+.{d}f}"
    except (TypeError, ValueError):
        return "   -"


def summary(eps):
    if not eps:
        print("还没有 logs/episodes.csv（还没在真机上跑过 main.py）")
        return
    n = len(eps)
    ok = [e for e in eps if e["result"] == "success"]
    print(f"共 {n} 次抓取：成功 {len(ok)}（{100 * len(ok) / n:.0f}%）")
    reasons = {}
    for e in eps:
        reasons[e["result"]] = reasons.get(e["result"], 0) + 1
    print("  结果分布：", reasons)
    for key, name in (("steps", "对准步数"), ("time_s", "用时(s)"), ("score", "评分")):
        vals = [float(e[key]) for e in eps if e.get(key)]
        if vals:
            print(f"  {name}：平均 {statistics.mean(vals):.1f}  最少 {min(vals):.1f}  最多 {max(vals):.1f}")
    last = eps[-1]
    js = {k: v for k, v in last.items() if k.startswith("J_")}
    print("  最新学到的灵敏度：", js)
    if len(eps) >= 4:
        h = n // 2
        a = statistics.mean(float(e["steps"]) for e in eps[:h])
        b = statistics.mean(float(e["steps"]) for e in eps[h:])
        print(f"  前一半平均 {a:.1f} 步 → 后一半 {b:.1f} 步" + ("（在变好）" if b < a else ""))
    for fn in ("learned_gains.json", "chassis_tau.json"):
        p = os.path.join(LOG, fn)
        if os.path.exists(p):
            print(f"  {fn}：", json.load(open(p, encoding="utf-8")))


def steps_of(steps, ep):
    rows = [s for s in steps if s["episode"] == str(ep)]
    if not rows:
        print(f"第 {ep} 次抓取没有对准记录（可能没找到物块，或者一开始就在容差内）")
        return
    print(f"\n第 {ep} 次抓取的对准过程（误差 → 动作）：")
    print("  步  底座°   误差x°   误差y(mm) 误差d(mm) |  升降cm  底座°   前进mm  拐弯 |  J_y    J_x    J_d")
    for s in rows:
        print(f"  {int(s['step']):2d} {f(s.get('base_deg'))} {f(s.get('e_x_deg')):>8} {f(s.get('e_y')):>10} "
              f"{f(s.get('e_d'), 0):>9} | {f(s.get('u_y'), 2):>7} {f(s.get('u_x')):>6} {f(s.get('u_d'), 0):>7} "
              f"{'  是' if s.get('recenter') == '1' else '    '} | {f(s.get('J_y'), 2):>6} {f(s.get('J_x_deg'), 2):>6} "
              f"{f(s.get('J_d'), 2):>6}")
    # 简单诊断
    ey = [abs(float(s["e_y"])) for s in rows if s.get("e_y")]
    if len(ey) >= 4 and ey[-1] > ey[0] * 0.8:
        print("  ⚠ y 误差几乎没变小：检查 AXES_J0['y'] 的符号（calib.py auto）")
    ex = [abs(float(s["e_x_deg"])) for s in rows if s.get("e_x_deg")]
    if len(ex) >= 4 and ex[-1] > ex[0] * 0.8:
        print("  ⚠ x 误差几乎没变小：检查 BASE_DIR（calib.py auto）")
    signs = [float(s["e_x_deg"]) > 0 for s in rows if s.get("e_x_deg") and abs(float(s["e_x_deg"])) > 1.5]
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    if flips >= 3:
        print("  ⚠ x 来回过冲：可以把 STEP_ALPHA 调小一点（例如 0.7）")


def main(argv):
    eps = read_csv("episodes.csv")
    steps = read_csv("steps.csv")
    summary(eps)
    if not eps:
        return
    if argv and argv[0] == "all":
        for e in eps:
            steps_of(steps, e["episode"])
    else:
        steps_of(steps, argv[0] if argv else eps[-1]["episode"])


if __name__ == "__main__":
    main(sys.argv[1:])
