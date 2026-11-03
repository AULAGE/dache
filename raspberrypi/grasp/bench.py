#!/usr/bin/env python3
"""
真机试验记录工具：按测试矩阵引导你放物块 → 自动抓一次 → 问你"真的抓到了吗" → 记到 logs/bench.csv

程序自己报的"抓到了"不一定可信，这个工具记录的是**人眼看到的真实结果**，并按物块位置统计偏差。

  python3 bench.py                     按 9 个位置（近/中/远 × 左/中/右）各抓 2 次，默认不用底盘
  python3 bench.py --positions 2,5,8   只测这几个位置
  python3 bench.py --repeat 3          每个位置抓 3 次
  python3 bench.py --resume            上次中途 Ctrl+C 退出了：从上一轮没测完的位置接着测
  python3 bench.py --chassis           用底盘（默认 --no-chassis）
  python3 bench.py --step              单步模式（每个动作回车确认），同 main.py --step
  python3 bench.py report              统计：按位置的真实成功率、误报、偏差方向 + 调参建议
  python3 bench.py report --last       只统计最近一轮

测试矩阵（见 docs/抓取与调试方案.md 3.2 节；左右和 config 一致：底座右转为正，所以"左"是负角度）：

              左（约 −10°）  中      右（约 +10°）
  近（约 15cm）   ①          ②         ③
  中（约 18cm）   ④          ⑤         ⑥
  远（约 21cm）   ⑦          ⑧         ⑨

近 / 远的深度取自标定的 REACH_NEAR_D / REACH_FAR_D（机械臂两个伸出姿态对应的深度），中 = 两者中间。

中途 Ctrl+C：升降轴急停、底盘停车，已经记下的结果都在 bench.csv 里，下次 --resume 接着测。
"""
import argparse
import csv
import math
import os
import re
import statistics
import sys
import time
import unicodedata
from collections import Counter, OrderedDict

import config as cfg

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_NAME = "bench.csv"
CIRCLED = "①②③④⑤⑥⑦⑧⑨"
ROWS = ("近", "中", "远")
COLS = ("左", "中", "右")
SIDE_DEG = 10.0              # 左 / 右位置偏多少度

FIELDS = ["time", "round", "pos", "row", "col", "depth_mm", "angle_deg", "chassis",
          "result", "truth", "dev_dirs", "dev_lr_mm", "dev_fb_mm", "dev_ud_mm", "note",
          "x_deg", "y", "d", "base_deg", "lift_cm", "t", "trim_deg", "attempts", "steps", "time_s"]

# 偏差方向：轴、符号。lr 右为正（和底座右转为正一致）；fb 前（离车更远）为正；ud 高为正
DIRS = OrderedDict([("左", ("lr", -1)), ("右", ("lr", 1)), ("前", ("fb", 1)), ("后", ("fb", -1)),
                    ("高", ("ud", 1)), ("低", ("ud", -1))])
ALIAS = {"l": "左", "r": "右", "f": "前", "b": "后", "u": "高", "h": "高", "d": "低"}
AXIS_WORD = {"lr": ("右", "左"), "fb": ("前", "后"), "ud": ("高", "低")}


def default_csv():
    return os.path.join(HERE, cfg.LOG_DIR, CSV_NAME)


# ============================================================ 测试矩阵
def depths_mm(c=cfg):
    """近 / 中 / 远三档物块深度（mm）：取标定的两个伸出姿态对应的深度"""
    near, far = sorted((float(c.REACH_NEAR_D), float(c.REACH_FAR_D)))
    if far - near < 20:                      # 只标了一个伸出姿态：按文档的 15 / 18 / 21cm
        return 150.0, 180.0, 210.0
    return near, (near + far) / 2, far


def positions(side_deg=SIDE_DEG, c=cfg):
    """9 个位置的说明。编号 1~9：行 = 近/中/远，列 = 左/中/右"""
    out = []
    ds = depths_mm(c)
    for i in range(9):
        r, k = divmod(i, 3)
        d = ds[r]
        ang = (k - 1) * side_deg              # 底座右转为正：左 = 负角度
        lat = d * math.tan(math.radians(abs(ang)))
        if k == 1:
            where = "正前方"
        else:
            where = (f"往{'左' if ang < 0 else '右'}偏约 {abs(ang):.0f}°（横向离中线约 {lat / 10:.1f}cm；"
                     f"对准时底座会转到约 {ang:+.0f}°）")
        text = (f"{CIRCLED[i]} {ROWS[r]}·{COLS[k]}：离相机（底座转轴正上方）约 {d / 10:.1f}cm、{where}")
        out.append({"no": i + 1, "row": ROWS[r], "col": COLS[k], "depth_mm": round(d), "angle_deg": ang,
                    "lateral_mm": round(lat), "text": text})
    return out


def parse_positions(s):
    if not s:
        return list(range(1, 10))
    out = []
    for tok in re.split(r"[,，\s]+", s.strip()):
        if not tok:
            continue
        if tok in CIRCLED:
            n = CIRCLED.index(tok) + 1
        else:
            n = int(tok)
        if not 1 <= n <= 9:
            raise ValueError(f"位置编号要在 1~9 之间：{tok}")
        out.append(n)
    return out


def make_plan(pos_list, repeat):
    """先把每个位置都测一遍，再测第二遍……（避免同一个位置连续抓，摆放习惯带来的偏差会被摊开）"""
    return [p for _ in range(repeat) for p in pos_list]


# ============================================================ CSV
def read_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def append_row(path, row):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    new = not os.path.exists(path)
    if not new:
        with open(path, newline="", encoding="utf-8") as f:
            header = next(csv.reader(f), [])
        if header != FIELDS:                 # 程序升级改了列：旧文件改名留档
            base, ext = os.path.splitext(path)
            os.replace(path, f"{base}_{time.strftime('%Y%m%d_%H%M%S')}{ext}")
            new = True
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in FIELDS})


def resume_plan(path, plan):
    """--resume：找到上一轮，去掉已经测过的次数。返回 (轮次 id 或 None, 剩下的计划)"""
    rows = read_rows(path)
    if not rows:
        return None, plan
    last = rows[-1].get("round")
    done = Counter(int(r["pos"]) for r in rows if r.get("round") == last)
    left = []
    for p in plan:
        if done[p] > 0:
            done[p] -= 1
        else:
            left.append(p)
    if not left:
        return None, plan                   # 上一轮已经测完：开新一轮
    return last, left


# ============================================================ 问用户
def ask_truth(ask):
    while True:
        a = ask("真的抓到了吗？[y=抓到 / n=没抓到 / x=这次不算，重来]：").strip().lower()
        if a in ("y", "yes", "是", "1", "抓到"):
            return 1
        if a in ("n", "no", "否", "0", "没", "没有"):
            return 0
        if a in ("x", "不算"):
            return None
        print("  请输入 y、n 或 x")


def parse_dev(s):
    """'右4 前3' / '右前' / 'r4' → [(方向, 毫米或 None), ...]；'不知道' / 空 → []"""
    s = s.strip().lower()
    if not s or s in ("?", "？", "不知道", "不清楚", "x"):
        return []
    out = []
    for word, num in re.findall(r"([左右前后高低lrfbuhd])\s*(\d+(?:\.\d+)?)?", s):
        word = ALIAS.get(word, word)
        if word in [w for w, _ in out]:
            continue
        out.append((word, float(num) if num else None))
    return out


def ask_deviation(ask):
    """返回 (方向文字, {lr, fb, ud} 带符号的毫米，没量的是 None)"""
    vals = {"lr": None, "fb": None, "ud": None}
    while True:
        a = ask("  爪子偏哪边？左/右/前/后/高/低，可以写多个、可以带毫米（例：右4 前3），不知道直接回车\n"
                "  （前 = 爪子落点比物块离车更远，后 = 更靠近车；高/低 = 爪子比物块高/低）：")
        devs = parse_dev(a)
        if devs or not a.strip() or a.strip() in ("?", "？", "不知道", "不清楚", "x"):
            break
        print("  没看懂，请用 左/右/前/后/高/低（或 l/r/f/b/u/d）")
    words = []
    for w, mm in devs:
        if mm is None:
            r = ask(f"  偏{w}大概几毫米？（回车 = 不知道）：").strip()
            try:
                mm = abs(float(r)) if r else None
            except ValueError:
                mm = None
        axis, sign = DIRS[w]
        words.append(w)
        if mm is not None:
            vals[axis] = sign * abs(mm)
    return "".join(words), vals


# ============================================================ 跑一次
def run_trial(runner):
    """调 main.RealRunner 跑一次真机抓取（和 main.py --once 同一套代码）。返回 (结果行, 伸出时的记录)"""
    from devices import DeviceError
    t0 = runner.clock.now()
    while True:
        try:
            r = runner.attempt()
        except DeviceError as e:
            return ({"result": "device_error", "attempts": "", "steps": "",
                     "time_s": round(runner.clock.now() - t0, 1), "error": str(e)}, None)
        if r is not None:
            task = runner.task
            return r, getattr(task, "last_grasp", None)


def run_bench(runner, plan, csv_path, ask=input, round_id=None, side_deg=SIDE_DEG, chassis=False):
    """按计划依次测；每次测完立刻写一行。返回写了几行。KeyboardInterrupt 原样抛出（调用方负责急停）"""
    pos_info = {p["no"]: p for p in positions(side_deg)}
    round_id = round_id or time.strftime("%Y%m%d_%H%M%S")
    done = 0
    print(f"本轮 {round_id}：共 {len(plan)} 次，位置顺序 "
          + " ".join(CIRCLED[p - 1] for p in plan) + f"   底座微调 GRASP_BASE_TRIM_DEG = {cfg.GRASP_BASE_TRIM_DEG:+.2f}°")
    i = 0
    while i < len(plan):
        p = pos_info[plan[i]]
        a = ask(f"\n[{i + 1}/{len(plan)}] 把物块放到 {p['text']}\n"
                f"    放好后回车开始（s = 跳过这次，q = 退出）：").strip().lower()
        if a == "q":
            break
        if a == "s":
            i += 1
            continue
        r, g = run_trial(runner)
        g = g or {}
        print(f"  程序结果：{r['result']}" + (f"（{r['error']}）" if r.get("error") else "")
              + (f"   伸出时物块 x={g['x_deg']:+.1f}° y={g['y']:+.0f}mm d={g['d']:.0f}mm，底座 {g['base_deg']:+.1f}°"
                 if g else ""))
        truth = ask_truth(ask)
        if truth is None:
            print("  这次不算，同一个位置重来")
            continue
        dirs, dev = "", {"lr": None, "fb": None, "ud": None}
        if not truth and g:                  # 伸出去抓过才问偏差（没找到 / 够不着时没有落点）
            dirs, dev = ask_deviation(ask)
        note = ask("  备注（可选，回车跳过）：").strip()
        append_row(csv_path, {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "round": round_id, "pos": p["no"], "row": p["row"],
            "col": p["col"], "depth_mm": p["depth_mm"], "angle_deg": p["angle_deg"], "chassis": int(chassis),
            "result": r["result"], "truth": truth, "dev_dirs": dirs,
            "dev_lr_mm": dev["lr"], "dev_fb_mm": dev["fb"], "dev_ud_mm": dev["ud"], "note": note,
            "x_deg": g.get("x_deg"), "y": g.get("y"), "d": g.get("d"), "base_deg": g.get("base_deg"),
            "lift_cm": g.get("lift_cm"), "t": g.get("t"),
            "trim_deg": g.get("trim_deg", cfg.GRASP_BASE_TRIM_DEG),
            "attempts": r.get("attempts"), "steps": r.get("steps"), "time_s": r.get("time_s")})
        done += 1
        flag = "✅ 真抓到" if truth else "❌ 没抓到"
        if r["result"] == "success" and not truth:
            flag += "（程序误报成功）"
        print(f"  已记录：{CIRCLED[p['no'] - 1]} {flag}" + (f"，偏{dirs}" if dirs else ""))
        i += 1
    return done


# ============================================================ 统计
def _w(s):
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def pad(s, w):
    return s + " " * max(0, w - _w(s))


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _votes(rows, axis):
    """某个轴上的偏差投票：每条没抓到的记录 +1 / −1 / 0（方向文字或毫米都算）"""
    v = []
    for r in rows:
        if str(r.get("truth")) != "0":
            continue
        mm = _f(r.get(f"dev_{axis}_mm"))
        if mm:
            v.append(1 if mm > 0 else -1)
            continue
        dirs = r.get("dev_dirs") or ""
        pos_w, neg_w = AXIS_WORD[axis]
        if pos_w in dirs:
            v.append(1)
        elif neg_w in dirs:
            v.append(-1)
    return v


def _mm(rows, axis):
    return [_f(r.get(f"dev_{axis}_mm")) for r in rows
            if str(r.get("truth")) == "0" and _f(r.get(f"dev_{axis}_mm"))]


def _sign(votes):
    """多数一致才算有方向：同号票数 > 一半"""
    if not votes:
        return 0
    s = sum(1 for v in votes if v > 0)
    n = sum(1 for v in votes if v < 0)
    if s > len(votes) / 2:
        return 1
    if n > len(votes) / 2:
        return -1
    return 0


def suggestions(rows):
    """根据记录给出调参建议（文字列表）"""
    out = []
    by_pos = {p: [r for r in rows if int(r["pos"]) == p] for p in range(1, 10)}
    n = len(rows)
    ok = sum(1 for r in rows if str(r.get("truth")) == "1")
    fp = sum(1 for r in rows if r.get("result") == "success" and str(r.get("truth")) == "0")

    # ---- 左右：先看偏差朝哪边、分布在几列
    sign = {p: _sign(_votes(rs, "lr")) for p, rs in by_pos.items()}
    tested = [p for p in range(1, 10) if by_pos[p]]
    right = [p for p in tested if sign[p] > 0]
    left = [p for p in tested if sign[p] < 0]
    for side, ps, other, sgn in (("右", right, left, 1), ("左", left, right, -1)):
        if len(ps) < 2 or len(other) * 3 > len(ps):     # 两边都有 / 只有一个位置：说明不了什么
            continue
        mms = [abs(m) for p in ps for m in _mm(by_pos[p], "lr") if m * sgn > 0]
        amt = f"约 {statistics.median(mms):.0f}mm" if mms else ""
        cols = sorted({(p - 1) % 3 for p in ps})
        names = "、".join(CIRCLED[p - 1] for p in ps)
        if len(cols) >= 2:
            head = (f"多数位置偏{side}{amt}" if len(ps) * 2 > len(tested)
                    else f"有偏差的位置（{names}）都偏{side}{amt}")
            if mms:
                out.append(f"{head} → 运行 `python3 calib.py trim {sgn * statistics.median(mms):.0f}`"
                           f"（偏右写正数，可累加）")
            else:
                out.append(f"{head}，但没记毫米 → 下次量一下偏几毫米，再 `calib.py trim <偏右毫米>`")
        elif cols == [1]:
            out.append(f"只有中间一列（{names}）偏{side}{amt} → 和底座角度无关，可能是 GOAL 的左右零点不准；"
                       f"多测几次确认后可以 `calib.py trim`")
        else:
            col = "左" if cols == [0] else "右"
            out.append(f"只有{col}侧位置（{names}）偏{side}{amt} → 和底座角度有关，可能是底座回差 / 小角度转不到位，"
                       f"见任务 09（先不要 trim，先把物块摆中间测）")

    # ---- 前后、高低：按行看
    for axis, name in (("fb", "前后"), ("ud", "高低")):
        row_sign = {}
        for ri, rn in enumerate(ROWS):
            v = [x for p in range(ri * 3 + 1, ri * 3 + 4) for x in _votes(by_pos[p], axis)]
            row_sign[rn] = (_sign(v) if len(v) >= 2 else 0)
        signed = {k: s for k, s in row_sign.items() if s}
        if len(signed) >= 2 and len(set(signed.values())) == 1:
            s = next(iter(signed.values()))
            w = AXIS_WORD[axis][0 if s > 0 else 1]
            mms = [abs(m) for rs in by_pos.values() for m in _mm(rs, axis) if m * s > 0]
            amt = f"约 {statistics.median(mms):.0f}mm" if mms else ""
            hint = ("两个伸出姿态都偏，重标 `calib.py teach near` 和 `calib.py teach far`"
                    if axis == "ud" else "深度标定（REACH_NEAR_D / REACH_FAR_D）可能整体偏了，重新 `calib.py teach`")
            out.append(f"多数深度都偏{w}{amt} → {hint}")
        else:
            for rn, s in signed.items():
                w = AXIS_WORD[axis][0 if s > 0 else 1]
                if rn == "远":
                    out.append(f"远处偏{w} → reach_far 标得不准，`calib.py teach far` 重标")
                elif rn == "近":
                    out.append(f"近处偏{w} → reach_near 标得不准，`calib.py teach near` 重标")
                else:
                    out.append(f"只有中间深度偏{w} → 两个姿态线性插值的误差，标一个 reach_mid（任务 09）")

    # ---- 成功率按行
    rate = {}
    for ri, rn in enumerate(ROWS):
        rs = [r for p in range(ri * 3 + 1, ri * 3 + 4) for r in by_pos[p]]
        if rs:
            rate[rn] = sum(1 for r in rs if str(r.get("truth")) == "1") / len(rs)
    if len(rate) == 3 and rate["中"] + 0.3 <= min(rate["近"], rate["远"]):
        out.append(f"中间深度成功率（{rate['中']:.0%}）明显低于两端 → 插值误差，标 reach_mid（任务 09）")

    if fp:
        out.append(f"程序误报成功 {fp} 次（说抓到、实际没抓到）→ 确认被骗了，把对应时间的 logs/run_*.log 里"
                   f"\"观察确认\"那几行发给我")
    if n >= 9 and ok / n >= 0.9 and not fp:
        out.append(f"真实成功率 {ok / n:.0%} ≥ 90% 且没有误报 → 可以进入底盘联调（方案第 4 节）")
    return out


def report(path, last=False):
    rows = read_rows(path)
    if last and rows:
        rid = rows[-1].get("round")
        rows = [r for r in rows if r.get("round") == rid]
    if not rows:
        print(f"没有记录（{path}）。先运行 python3 bench.py")
        return []
    rounds = sorted({r.get("round") for r in rows})
    n = len(rows)
    ok = sum(1 for r in rows if str(r.get("truth")) == "1")
    fp = sum(1 for r in rows if r.get("result") == "success" and str(r.get("truth")) == "0")
    claim = sum(1 for r in rows if r.get("result") == "success")
    print(f"试验记录：{path}")
    print(f"  共 {n} 次（{len(rounds)} 轮：{', '.join(rounds)}）  真实抓到 {ok}/{n} = {ok / n:.0%}   "
          f"程序报成功 {claim} 次，其中误报 {fp} 次")
    trims = sorted({r.get("trim_deg") for r in rows if r.get("trim_deg") not in (None, "")})
    if len(trims) > 1:
        print(f"  ⚠ 记录里的底座微调不止一个值（{', '.join(trims)}），偏差统计混在一起了；可以用 --last 只看最近一轮")

    by_pos = {p: [r for r in rows if int(r["pos"]) == p] for p in range(1, 10)}
    ds = {}
    for r in rows:
        ds.setdefault(r.get("row"), r.get("depth_mm"))

    W0, W = 14, 24
    print("\n按位置：真实成功率（误报次数）")
    print(pad("", W0) + "".join(pad(f"{c}（{a}）", W) for c, a in zip(COLS, ("约−10°", "0°", "约+10°"))))
    for ri, rn in enumerate(ROWS):
        dtxt = _f(ds.get(rn))
        line = pad(f"{rn}" + (f"（{dtxt / 10:.0f}cm）" if dtxt else ""), W0)
        for k in range(3):
            p = ri * 3 + k + 1
            rs = by_pos[p]
            if not rs:
                cell = f"{CIRCLED[p - 1]} —"
            else:
                o = sum(1 for r in rs if str(r.get("truth")) == "1")
                f = sum(1 for r in rs if r.get("result") == "success" and str(r.get("truth")) == "0")
                cell = f"{CIRCLED[p - 1]} {o}/{len(rs)} {100 * o / len(rs):.0f}%" + (f" 误报{f}" if f else "")
            line += pad(cell, W)
        print(line)

    print("\n按位置：没抓到时爪子的平均偏差（右 / 前 / 高为正，mm；括号里是方向记录次数）")
    print(pad("", W0) + "".join(pad(c, W) for c in COLS))
    for ri, rn in enumerate(ROWS):
        line = pad(rn, W0)
        for k in range(3):
            p = ri * 3 + k + 1
            parts = []
            for axis in ("lr", "fb", "ud"):
                v = _votes(by_pos[p], axis)
                if not v:
                    continue
                mms = _mm(by_pos[p], axis)
                s = _sign(v)
                if mms:
                    m = statistics.mean(mms)
                    w = AXIS_WORD[axis][0 if m > 0 else 1]
                    parts.append(f"{w}{abs(m):.0f}")
                elif s:
                    parts.append(AXIS_WORD[axis][0 if s > 0 else 1] + "?")
                else:
                    parts.append(f"{AXIS_WORD[axis][0]}{AXIS_WORD[axis][1]}不一")
                parts[-1] += f"({len(v)})"
            line += pad(f"{CIRCLED[p - 1]} " + (" ".join(parts) if parts else "—"), W)
        print(line)

    print("\n程序结果 vs 真实结果：")
    conf = OrderedDict()
    for r in rows:
        c = conf.setdefault(r.get("result") or "?", [0, 0])
        c[0 if str(r.get("truth")) == "1" else 1] += 1
    print("  " + pad("程序结果", 16) + pad("真抓到", 10) + pad("没抓到", 10))
    for res, (a, b) in sorted(conf.items(), key=lambda kv: (kv[0] != "success", kv[0])):
        tag = "  ← 误报" if res == "success" and b else ("  ← 漏报（程序说失败但其实抓到了）" if res != "success" and a else "")
        print("  " + pad(res, 16) + pad(str(a), 10) + pad(str(b), 10) + tag)

    sug = suggestions(rows)
    print("\n建议：")
    for s in sug or ["（数据还不够或没有一致的偏差，继续测）"]:
        print("  · " + s)
    return sug


# ============================================================ 入口
def main(argv=None):
    ap = argparse.ArgumentParser(description="真机试验记录工具（记录真实抓取结果，按位置统计）")
    ap.add_argument("cmd", nargs="?", default="run", choices=["run", "report"], help="run（默认）或 report")
    ap.add_argument("--positions", default="", help="只测这些位置，例如 2,5,8（默认 1~9 全部）")
    ap.add_argument("--repeat", type=int, default=2, help="每个位置测几次（默认 2）")
    ap.add_argument("--resume", action="store_true", help="接着上一轮没测完的位置继续")
    ap.add_argument("--chassis", action="store_true", help="用底盘（默认不用，等价于 main.py --no-chassis）")
    ap.add_argument("--no-chassis", action="store_true", help="不用底盘（默认就是这样，写上也行）")
    ap.add_argument("--step", action="store_true", help="单步模式：每个动作前回车确认")
    ap.add_argument("--force", action="store_true", help="启动前检查没通过也继续（自己确认安全）")
    ap.add_argument("--side-deg", type=float, default=SIDE_DEG, help="左 / 右位置偏多少度（默认 10）")
    ap.add_argument("--last", action="store_true", help="report：只统计最近一轮")
    ap.add_argument("--file", default=None, help="记录文件（默认 logs/bench.csv）")
    a = ap.parse_args(argv)
    path = a.file or default_csv()

    if a.cmd == "report":
        report(path, a.last)
        return

    plan = make_plan(parse_positions(a.positions), max(1, a.repeat))
    round_id = None
    if a.resume:
        round_id, plan = resume_plan(path, plan)
        if round_id:
            print(f"接着上一轮 {round_id}：还剩 {len(plan)} 次")
        else:
            print("没有没测完的一轮，开始新一轮")

    import main as M
    use_chassis = a.chassis and not a.no_chassis
    if not use_chassis:
        cfg.USE_CHASSIS = False
    sys.stdout = M.Tee(os.path.join(M.LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.log")))
    print("启动：", " ".join(sys.argv), " calib.json 覆盖：", sorted(cfg.CALIB_LOADED))
    print("位置说明：")
    for p in positions(a.side_deg):
        print("  " + p["text"])
    runner = None
    n = 0
    try:
        runner = M.RealRunner(a.step, a.force)
        n = run_bench(runner, plan, path, round_id=round_id, side_deg=a.side_deg, chassis=use_chassis)
        print(f"\n本次记录了 {n} 次。看统计：python3 bench.py report")
    except M.PreflightFailed as e:
        print(e)
    except KeyboardInterrupt:
        print("\n手动停止：升降轴急停、底盘停车。已测完的都记在", path,
              "\n下次接着测：python3 bench.py --resume" + (f" --positions {a.positions}" if a.positions else "")
              + (f" --repeat {a.repeat}" if a.repeat != 2 else ""))
    finally:
        if runner is not None:
            runner.close()


if __name__ == "__main__":
    main()
