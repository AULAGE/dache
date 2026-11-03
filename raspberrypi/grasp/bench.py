#!/usr/bin/env python3
"""
真机试验记录工具：按测试矩阵引导放物块 → 自动抓一次 → 问你"真的抓到了吗" → 记到 logs/bench.csv

  python3 bench.py                     按 9 个位置各测 2 次（默认无底盘）
  python3 bench.py --positions 2,5,8   只测这几个位置
  python3 bench.py --repeat 3          每个位置测 3 次
  python3 bench.py --resume            接着上一轮没测完的位置继续（Ctrl+C 退出后用）
  python3 bench.py --chassis           用底盘（默认不用，等价于 main.py --no-chassis）
  python3 bench.py --step              每个动作前回车确认（同 main.py --step）
  python3 bench.py --sim               不接硬件，用仿真试用这个工具
  python3 bench.py report              最近一轮的统计：按位置的真实成功率、误报、偏差方向、建议
  python3 bench.py report all          所有记录一起统计

测试矩阵（物块放在低平台上，摆正；左右以底座右转为正，和 config 一致）：

            左（约 -10°）  中      右（约 +10°）
  近 15cm      ①          ②         ③
  中 18cm      ④          ⑤         ⑥
  远 21cm      ⑦          ⑧         ⑨

"离车多远"按离机械臂底座转轴（≈ 观察姿态下的相机，真机上相机几乎在转轴上）的水平距离量。
"""
import argparse
import csv
import math
import os
import sys
import time
import unicodedata

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config as cfg  # noqa: E402
from devices import DeviceError, ManualOverride  # noqa: E402

BENCH_CSV = os.path.join(HERE, cfg.LOG_DIR, "bench.csv")

# 测试矩阵（docs/抓取与调试方案.md 3.2 节）
DEPTHS_CM = (15, 18, 21)                 # 近 / 中 / 远
SIDE_DEG = 10.0                          # 左右偏多少度
ROW_NAMES = ("近", "中", "远")
COL_NAMES = ("左", "中", "右")
MARKS = "①②③④⑤⑥⑦⑧⑨"
D_WARN_MM = 35                           # 测到的物块深度和这个位置差太多就提醒（可能放错位置）

# 偏差方向：爪子落点相对物块偏哪边。每个方向归到一个轴，带符号
DIRS = {"右": ("x", 1), "左": ("x", -1), "前": ("d", 1), "后": ("d", -1), "高": ("h", 1), "低": ("h", -1)}
DIR_KEYS = {"r": "右", "l": "左", "f": "前", "b": "后", "u": "高", "h": "高", "d": "低",
            "1": "左", "2": "右", "3": "前", "4": "后", "5": "高", "6": "低", "7": "不知道", "?": "不知道",
            "": "不知道"}
AXIS_NAMES = {"x": ("右", "左"), "d": ("前", "后"), "h": ("高", "低")}
BIAS_MM = 1.5                            # 平均偏差小于这个值算"不偏"

FIELDS = ["time", "round", "pos", "pos_name", "trial", "prog_result", "real", "dev_dir", "dev_mm", "note",
          "x_deg", "y", "d", "base_deg", "lift_cm", "t", "trim_deg", "steps", "attempts", "time_s",
          "chassis", "log"]


# ============================================================ 位置
def pos_spec(p):
    """位置编号 1~9 → (行, 列, 深度 mm, 左右角度°，右为正)"""
    r, c = (p - 1) // 3, (p - 1) % 3
    return r, c, DEPTHS_CM[r] * 10.0, (-SIDE_DEG, 0.0, SIDE_DEG)[c]


def pos_name(p):
    r, c, _, _ = pos_spec(p)
    return f"{ROW_NAMES[r]}·{COL_NAMES[c]}"


def pos_text(p):
    r, c, d_mm, side = pos_spec(p)
    cm = d_mm / 10
    s = f"{MARKS[p - 1]} {ROW_NAMES[r]}·{COL_NAMES[c]}（离底座转轴约 {cm:.0f}cm，"
    if side == 0:
        return s + "正前方）"
    off = cm * math.tan(math.radians(abs(side)))
    lr = "左" if side < 0 else "右"
    return s + f"往{lr}约 {abs(side):.0f}°，即正前方那条线往{lr}约 {off:.1f}cm）"


def parse_positions(text):
    ps = []
    for x in text.replace("，", ",").split(","):
        x = x.strip()
        if not x:
            continue
        p = MARKS.index(x) + 1 if x in MARKS else int(x)
        if not 1 <= p <= 9:
            raise ValueError(f"位置编号只能是 1~9：{x}")
        if p not in ps:
            ps.append(p)
    return ps


# ============================================================ CSV
def read_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def append_row(path, row):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    new = not os.path.exists(path)
    if not new:
        with open(path, newline="", encoding="utf-8") as f:
            header = next(csv.reader(f), [])
        if header != FIELDS:      # 列变了（程序升级）：旧文件改名留档
            base, ext = os.path.splitext(path)
            os.replace(path, f"{base}_{time.strftime('%Y%m%d_%H%M%S')}{ext}")
            new = True
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in FIELDS})


def make_plan(positions, repeat, done=None):
    """测试顺序：先把每个位置各测一遍，再测第二遍……（done = 本轮每个位置已测次数，--resume 用）"""
    done = done or {}
    return [(p, k + 1) for k in range(repeat) for p in positions if done.get(p, 0) <= k]


# ============================================================ 运行设备
class RealRunner:
    """真机：走 main.py 的 RealSession（open_devices / preflight / GraspTask.run_once）"""
    sim = False

    def __init__(self, chassis=False, step=False, force=False):
        import main
        cfg.USE_CHASSIS = bool(chassis)
        self.chassis = bool(chassis)
        self.session = main.RealSession(step=step, force=force)
        time.sleep(1.2)             # 让手柄检测线程先跑一次

    def start(self):
        if self.session.guard.active:
            print("手柄控制程序在运行（会占用串口）：先断开手柄 / systemctl --user stop pad-watchdog")
            return False
        return self.session.open()

    def run(self, pos):
        row = self.session.run_once()
        return row, self.session.task.last_grasp

    def close(self):
        self.session.close()        # 升降轴急停（E）、底盘停车、关串口


class SimRunner:
    """仿真：物块按位置放好，走同一个 GraspTask（selftest 和 --sim 用）"""
    sim = True

    def __init__(self, seed=1, chassis=False, verbose=False):
        from sim import SIM_POSES, make_car
        cfg.POSES = SIM_POSES
        cfg.USE_CHASSIS = bool(chassis)
        self.chassis = bool(chassis)
        self.seed, self.verbose = seed, verbose
        self.car = make_car(cfg, seed)
        self.task = None
        self.world = None
        self.i = 0
        self.closed = False

    def start(self):
        return True

    def run(self, pos):
        from sim import make_sim
        from task import GraspTask
        self.i += 1
        world, clock, cam, lift, arm, ch = make_sim(cfg, self.seed * 1000 + self.i, 1.0, cfg.R_CAM_MM, self.car)
        _, _, d_mm, side = pos_spec(pos)
        r = world.r_cam + d_mm
        a = math.radians(-side)                       # 世界坐标 y 向左；底座右转为正
        world.block[0] = world.PIVOT_FWD + r * math.cos(a)
        world.block[1] = r * math.sin(a)
        self.world = world
        if self.task is None:
            self.task = GraspTask(cfg, clock, cam, lift, arm, ch, verbose=self.verbose)
        else:
            self.task.clock, self.task.cam, self.task.lift, self.task.arm, self.task.chassis = \
                clock, cam, lift, arm, ch
            self.task.base_deg = 0.0
        row = self.task.run_once()
        return row, self.task.last_grasp

    def close(self):
        self.closed = True


# ============================================================ 交互
def _ask(ask, prompt, valid=None):
    while True:
        a = ask(prompt).strip()
        if valid is None or a.lower() in valid:
            return a
        print("  没看懂，再输一次")


def ask_result(ask, row, info):
    """问真实结果。返回 dict（real/dev_dir/dev_mm/note），或者 None = 这次作废不记"""
    if info is None:
        print(f"  程序没有伸爪（结果 {row['result']}），记为没抓到")
        real, dev, mm = "n", "未伸爪", None
    else:
        a = _ask(ask, "  真的抓到了吗？[y=抓到 / n=没抓到 / x=这次作废不记]：",
                 {"y", "n", "x", "是", "否", "1", "0"}).lower()
        if a == "x":
            return None
        real = "y" if a in ("y", "是", "1") else "n"
        dev, mm = "", None
        if real == "n":
            while True:
                a = ask("  爪子落点偏哪边？[左 l / 右 r / 前 f（伸过头）/ 后 b（没伸到）/ 高 u / 低 d / "
                        "回车=不知道]：").strip().lower()
                dev = a if a in DIRS or a == "不知道" else DIR_KEYS.get(a)
                if dev:
                    break
                print("  没看懂，再输一次")
            if dev != "不知道":
                while True:
                    a = ask("  大概偏几毫米？[数字，回车=不知道]：").strip()
                    if not a:
                        break
                    try:
                        mm = abs(float(a))
                        break
                    except ValueError:
                        print("  请输数字")
    note = ask("  备注（可空，回车跳过）：").strip()
    return {"real": real, "dev_dir": dev, "dev_mm": mm, "note": note}


def run_bench(runner, plan, csv_path, round_id, ask=None, log_name=""):
    """按计划逐个测。返回本次记录的条数。Ctrl+C / 输入结束：急停、已完成的都已经写进 CSV"""
    ask = ask or input
    n_rec = 0
    total = len(plan)
    try:
        if not runner.start():
            return 0
        i = 0
        while i < total:
            p, trial = plan[i]
            print(f"\n【{i + 1}/{total}】把物块放到 {pos_text(p)}，第 {trial} 次")
            a = _ask(ask, "  放好后回车开始（s=跳过这个位置这一次，q=退出）：", {"", "s", "q"}).lower()
            if a == "q":
                print("退出。没测的下次用 python3 bench.py --resume 继续")
                break
            if a == "s":
                i += 1
                continue
            try:
                row, info = runner.run(p)
            except ManualOverride as e:
                print(f"  {e}：这次不记，等手柄断开后重测")
                continue
            except DeviceError as e:
                row, info = {"result": "device_error", "steps": "", "attempts": "", "time_s": ""}, None
                print(f"  设备错误：{e}")
            print(f"  程序结果：{row['result']}  步数 {row.get('steps')}  用时 {row.get('time_s')}s")
            if info is not None and abs(info["d"] - pos_spec(p)[2]) > D_WARN_MM:
                print(f"  ⚠ 测到物块深度 {info['d']:.0f}mm，和这个位置（约 {pos_spec(p)[2]:.0f}mm）差得多，放对位置了吗？")
            if getattr(runner, "sim", False) and runner.world is not None:
                print(f"  （仿真真实结果：{'抓到' if runner.world.picked else '没抓到'}）")
            ans = ask_result(ask, row, info)
            if ans is None:
                print("  作废，重测这一次")
                continue
            g = info or {}
            append_row(csv_path, {
                "time": time.strftime("%Y-%m-%d %H:%M:%S"), "round": round_id, "pos": p, "pos_name": pos_name(p),
                "trial": trial, "prog_result": row["result"], **ans,
                **{k: g.get(k) for k in ("x_deg", "y", "d", "base_deg", "lift_cm", "t")},
                "trim_deg": g.get("trim_deg", cfg.GRASP_BASE_TRIM_DEG),
                "steps": row.get("steps"), "attempts": row.get("attempts"), "time_s": row.get("time_s"),
                "chassis": int(runner.chassis), "log": log_name})
            n_rec += 1
            i += 1
        else:
            print(f"\n本轮测完（记录 {n_rec} 次）。看统计：python3 bench.py report")
    except (KeyboardInterrupt, EOFError):
        print(f"\n中断：升降轴急停、底盘停车。本次已记录 {n_rec} 次（{csv_path}），"
              f"下次 python3 bench.py --resume 从没测的位置继续")
    finally:
        runner.close()
    return n_rec


# ============================================================ 统计
def _w(s):
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def _pad(s, w):
    return s + " " * max(0, w - _w(s))


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def select_round(rows, which=None):
    """which=None：最近一轮；'all'：全部；其它：指定轮次"""
    if not rows or which == "all":
        return rows, "全部记录"
    rid = which or rows[-1]["round"]
    return [r for r in rows if r["round"] == rid], f"第 {rid} 轮"


def axis_dev(rows):
    """按位置、按轴的带符号偏差：{axis: {pos: [mm...]}}（只算报了方向和毫米数的没抓到记录）"""
    out = {a: {} for a in AXIS_NAMES}
    for r in rows:
        if r["real"] != "n" or r["dev_dir"] not in DIRS:
            continue
        mm = _f(r["dev_mm"])
        if mm is None:
            continue
        ax, sg = DIRS[r["dev_dir"]]
        out[ax].setdefault(int(r["pos"]), []).append(sg * mm)
    return out


def suggest(rows):
    """根据偏差和成功率给建议，返回文字列表"""
    tips = []
    dev = axis_dev(rows)
    mean = lambda v: sum(v) / len(v)                      # noqa: E731

    # 左右
    lat = {p: mean(v) for p, v in dev["x"].items()}
    if len(lat) >= 2:
        right = [p for p, m in lat.items() if m >= BIAS_MM]
        left = [p for p, m in lat.items() if m <= -BIAS_MM]
        dom, sign = (right, 1) if len(right) >= len(left) else (left, -1)
        word = "右" if sign > 0 else "左"
        cols = {pos_spec(p)[1] for p in dom}
        if len(dom) >= 2 and len(dom) > len(lat) / 2 and len(cols) >= 2:
            vals = [x for p in dom for x in dev["x"][p]]
            mm = mean(vals)
            ds = [_f(r["d"]) for r in rows if r["real"] == "n" and r["dev_dir"] in ("左", "右")
                  and int(r["pos"]) in dom and _f(r["d"])]
            depth = round(mean(ds)) if ds else 180
            tips.append(f"多数位置（{len(dom)}/{len(lat)}）偏{word}约 {abs(mm):.0f}mm → 运行 "
                        f"python3 calib.py trim {mm:.0f} {depth}（落点偏{word}，可累加）")
        elif dom and cols in ({0}, {2}):
            side = "右" if cols == {2} else "左"
            tips.append(f"只有{side}侧位置偏{word}（{'、'.join(MARKS[p - 1] for p in sorted(dom))}）→ "
                        f"和底座角度有关，可能是底座回差或爪子脚遮挡，见任务 09（先别用 trim 补）")
        elif right and left:
            tips.append("左右偏差方向不一致：先看是不是对准没到位（report.py 看最后几步的 x 误差）")
    elif len(lat) == 1:
        tips.append("左右偏差只有一个位置有数据，太少，多测几个位置再下结论")

    # 前后 / 高低：按行（深度）看
    rows_name = {0: "近处", 1: "中间深度", 2: "远处"}
    for ax, what in (("d", "前后"), ("h", "高低")):
        pm = {p: mean(v) for p, v in dev[ax].items()}
        if len(pm) < 2:
            continue
        pos_ = [p for p, m in pm.items() if m >= BIAS_MM]
        neg = [p for p, m in pm.items() if m <= -BIAS_MM]
        dom, sign = (pos_, 1) if len(pos_) >= len(neg) else (neg, -1)
        if len(dom) < 2:
            continue
        word = AXIS_NAMES[ax][0 if sign > 0 else 1]
        rset = {pos_spec(p)[0] for p in dom}
        mm = mean([x for p in dom for x in dev[ax][p]])
        if len(rset) == 1:
            r = rset.pop()
            fix = {0: "python3 calib.py teach near 重标近处姿态", 2: "python3 calib.py teach far 重标远处姿态",
                   1: "两个姿态之间插值不准，标 reach_mid（任务 09）"}[r]
            tips.append(f"{rows_name[r]}偏{word}约 {abs(mm):.0f}mm → {fix}")
        elif len(dom) > len(pm) / 2:
            hint = ("伸出姿态整体伸过头 / 没伸到，或深度测得偏：重标 teach near / far"
                    if ax == "d" else "伸出姿态整体偏高 / 偏低：重标 teach near / far，或平台高度变了")
            tips.append(f"多数位置偏{word}约 {abs(mm):.0f}mm → {hint}")

    # 按深度的成功率：中间差、两头好 → 插值
    rate = {}
    for r in rows:
        k = pos_spec(int(r["pos"]))[0]
        a = rate.setdefault(k, [0, 0])
        a[0] += r["real"] == "y"
        a[1] += 1
    if all(k in rate for k in (0, 1, 2)) and min(rate[0][1], rate[1][1], rate[2][1]) >= 2:
        s = {k: a[0] / a[1] for k, a in rate.items()}
        if s[1] + 0.25 <= min(s[0], s[2]):
            tips.append(f"中间深度成功率（{s[1]:.0%}）明显比近处（{s[0]:.0%}）、远处（{s[2]:.0%}）低 → "
                        f"插值误差，标 reach_mid（任务 09）")

    fp = [r for r in rows if r["prog_result"] == "success" and r["real"] == "n"]
    if fp:
        tips.append(f"误报 {len(fp)} 次（程序说抓到、实际没有）：把那几次的 logs/run_*.log 里"
                    f"'观察确认'前后几行发给我（日志：{', '.join(sorted({r['log'] for r in fp if r['log']})) or '见 log 列'}）")
    return tips


def report(rows, which=None):
    rows, title = select_round(rows, which)
    if not rows:
        print("还没有 logs/bench.csv（先跑 python3 bench.py）")
        return
    rounds = sorted({r["round"] for r in rows})
    trims = sorted({r["trim_deg"] for r in rows})
    n = len(rows)
    ok = sum(r["real"] == "y" for r in rows)
    fp = sum(r["prog_result"] == "success" and r["real"] == "n" for r in rows)
    print(f"{title}：共 {n} 次，真实抓到 {ok}（{100 * ok / n:.0f}%），误报 {fp} 次"
          + (f"   （包含 {len(rounds)} 轮）" if len(rounds) > 1 else ""))
    if len(trims) > 1:
        print(f"  ⚠ 这些记录的左右微调不一样（{', '.join(trims)}°），偏差统计会混在一起；看单独一轮用 report <轮次>")

    by = {}
    for r in rows:
        by.setdefault(int(r["pos"]), []).append(r)
    W = 20

    def table(cell, head):
        print(f"\n{head}")
        print("  " + _pad("", 10) + "".join(_pad(COL_NAMES[c] + (f"（{(-SIDE_DEG, 0, SIDE_DEG)[c]:+.0f}°）" if c != 1 else ""), W)
                                            for c in range(3)))
        for ri in range(3):
            line = "  " + _pad(f"{ROW_NAMES[ri]} {DEPTHS_CM[ri]}cm", 10)
            for c in range(3):
                p = ri * 3 + c + 1
                line += _pad(f"{MARKS[p - 1]} " + (cell(by[p]) if p in by else "-"), W)
            print(line)

    def c_rate(rs):
        k = sum(r["real"] == "y" for r in rs)
        f = sum(r["prog_result"] == "success" and r["real"] == "n" for r in rs)
        return f"{k}/{len(rs)}" + (f" 误报{f}" if f else "")

    def c_dev(rs):
        out = []
        for ax, (pw, nw) in AXIS_NAMES.items():
            v = [DIRS[r["dev_dir"]][1] * _f(r["dev_mm"]) for r in rs
                 if r["real"] == "n" and r["dev_dir"] in DIRS and DIRS[r["dev_dir"]][0] == ax
                 and _f(r["dev_mm"]) is not None]
            if v:
                m = sum(v) / len(v)
                out.append(f"{pw if m >= 0 else nw}{abs(m):.0f}" + (f"×{len(v)}" if len(v) > 1 else ""))
        unk = sum(r["real"] == "n" and (r["dev_dir"] in ("不知道", "") or
                                        (r["dev_dir"] in DIRS and _f(r["dev_mm"]) is None)) for r in rs)
        if unk:
            out.append(f"?×{unk}")
        return " ".join(out) or "-"

    def c_meas(rs):
        ds = [_f(r["d"]) for r in rs if _f(r["d"]) is not None]
        bs = [_f(r["base_deg"]) for r in rs if _f(r["base_deg"]) is not None]
        if not ds:
            return "-"
        return f"{sum(ds) / len(ds):.0f}mm {sum(bs) / len(bs):+.0f}°"

    table(c_rate, "真实成功率（抓到/次数），误报 = 程序说抓到但实际没有：")
    table(c_dev, "没抓到时爪子落点的平均偏差（mm；右/左、前=伸过头/后=没伸到、高/低；?=不知道）：")
    table(c_meas, "对准时测到的物块深度、底座角度（检查物块是不是放在了该放的位置）：")

    # 混淆表
    print("\n程序结果 vs 真实结果：")
    res = sorted({r["prog_result"] for r in rows}, key=lambda x: (x != "success", x))
    print("  " + _pad("程序结果", 16) + _pad("真抓到", 8) + _pad("没抓到", 8))
    for x in res:
        y = sum(r["prog_result"] == x and r["real"] == "y" for r in rows)
        nn = sum(r["prog_result"] == x and r["real"] == "n" for r in rows)
        print("  " + _pad(x, 16) + _pad(str(y), 8) + _pad(str(nn), 8)
              + ("  ← 误报" if x == "success" and nn else "") + ("  ← 漏报" if x != "success" and y else ""))

    ts = [_f(r["time_s"]) for r in rows if _f(r["time_s"]) is not None]
    if ts:
        print(f"\n用时：平均 {sum(ts) / len(ts):.1f}s，最长 {max(ts):.1f}s")
    notes = [(r["pos"], r["note"]) for r in rows if r.get("note")]
    if notes:
        print("备注：" + "；".join(f"{MARKS[int(p) - 1]} {t}" for p, t in notes))

    tips = suggest(rows)
    print("\n建议：")
    for t in tips or ["没有看出一致的偏差方向（数据少的话多测几轮）"]:
        print("  → " + t)


# ============================================================ 入口
def main(argv=None):
    ap = argparse.ArgumentParser(description="真机试验记录工具（测试矩阵 + 人工记录真实结果）")
    ap.add_argument("cmd", nargs="?", default="run", choices=["run", "report"], help="run（默认）或 report")
    ap.add_argument("which", nargs="?", help="report 的范围：省略 = 最近一轮，all = 全部，或者轮次编号")
    ap.add_argument("--positions", default="1,2,3,4,5,6,7,8,9", help="只测这些位置，例如 2,5,8")
    ap.add_argument("--repeat", type=int, default=2, help="每个位置测几次（默认 2）")
    ap.add_argument("--resume", action="store_true", help="接着最近一轮没测完的继续")
    ap.add_argument("--chassis", action="store_true", help="用底盘（默认不用）")
    ap.add_argument("--step", action="store_true", help="单步模式：每个动作前回车确认")
    ap.add_argument("--force", action="store_true", help="启动前检查没通过也继续（自己确认安全）")
    ap.add_argument("--sim", nargs="?", type=int, const=1, metavar="SEED", help="不接硬件，用仿真试用")
    ap.add_argument("--csv", default=BENCH_CSV, help=argparse.SUPPRESS)
    a = ap.parse_args(argv)

    if a.cmd == "report":
        report(read_rows(a.csv), a.which)
        return

    positions = parse_positions(a.positions)
    rows = read_rows(a.csv)
    if a.resume and rows:
        round_id = rows[-1]["round"]
        done = {}
        for r in rows:
            if r["round"] == round_id:
                done[int(r["pos"])] = done.get(int(r["pos"]), 0) + 1
        plan = make_plan(positions, a.repeat, done)
        print(f"接着第 {round_id} 轮：已测 {sum(done.values())} 次，还剩 {len(plan)} 次")
        if not plan:
            print("这一轮已经测完了。看统计：python3 bench.py report")
            return
    else:
        round_id = time.strftime("%Y%m%d_%H%M%S")
        plan = make_plan(positions, a.repeat)
        print(f"新的一轮 {round_id}：{len(positions)} 个位置 × {a.repeat} 次 = {len(plan)} 次"
              f"（{'有' if a.chassis else '无'}底盘，左右微调 {cfg.GRASP_BASE_TRIM_DEG:+.2f}°）")
    print("物块放在低平台上、摆正。中途 Ctrl+C 可以安全退出（升降轴急停），下次加 --resume 继续")

    log_name = ""
    if a.sim is not None:
        runner = SimRunner(a.sim, a.chassis)
    else:
        import main as main_mod
        log_name = time.strftime("run_%Y%m%d_%H%M%S.log")
        sys.stdout = main_mod.Tee(os.path.join(main_mod.LOG_DIR, log_name))
        print("启动：", " ".join(sys.argv), " calib.json 覆盖：", sorted(cfg.CALIB_LOADED))
        runner = RealRunner(a.chassis, a.step, a.force)
    run_bench(runner, plan, a.csv, round_id, log_name=log_name)


if __name__ == "__main__":
    main()
