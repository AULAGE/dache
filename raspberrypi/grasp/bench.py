#!/usr/bin/env python3
"""
真机试验记录工具：引导你把物块放到测试矩阵的位置 → 自动抓一次 → 问你"真的抓到了吗" → 记到 logs/bench.csv

  python3 bench.py                    按测试矩阵 9 个位置 × 每个 2 次（默认无底盘）
  python3 bench.py --positions 2,5,8  只测这几个位置
  python3 bench.py --repeat 3         每个位置测 3 次
  python3 bench.py --resume           接着上一轮没测完的位置继续（Ctrl+C 退出后用）
  python3 bench.py --chassis          用底盘（默认不用，相当于 main.py --no-chassis）
  python3 bench.py --step             单步模式（同 main.py --step）
  python3 bench.py --sim              用仿真代替真机，练习这个工具怎么用（不需要硬件）

  python3 bench.py report             统计最新一轮：按位置的真实成功率、误报、偏差方向，并给出调参建议
  python3 bench.py report --all       统计全部记录
  python3 bench.py report --round R   统计指定的一轮

测试矩阵（俯视，车头朝上；左右和 config 一致：底座右转为正）：
              左（底座约 −10°）  中（正前方）  右（底座约 +10°）
  近 约 15cm        ①              ②             ③
  中 约 18cm        ④              ⑤             ⑥
  远 约 21cm        ⑦              ⑧             ⑨
距离从机械臂底座中心（底座舵机转轴）量到物块中心；相机几乎在转轴上，所以约等于 K230 读到的深度。

中途按 Ctrl+C：升降轴急停、已经完成的记录都在 bench.csv 里，下次加 --resume 接着测。
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

import config as cfg

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH_CSV = os.path.join(HERE, cfg.LOG_DIR, "bench.csv")

# ------------------------------------------------------------ 测试矩阵
ROW_DEPTH_MM = {"近": 150.0, "中": 180.0, "远": 210.0}       # 离底座转轴的距离
COL_SIDE_DEG = {"左": -10.0, "中": 0.0, "右": 10.0}          # 底座角度，右转为正
CIRCLED = "①②③④⑤⑥⑦⑧⑨"
POSITIONS = {}
for _i, _r in enumerate(ROW_DEPTH_MM):
    for _j, _c in enumerate(COL_SIDE_DEG):
        POSITIONS[_i * 3 + _j + 1] = (_r, _c)

BIAS_MM = 2.0          # 平均偏差超过这么多毫米才算"偏"

FIELDS = ["time", "round", "pos", "pos_name", "chassis", "prog_result", "truth", "dev_dir", "dev_mm", "note",
          "x_deg", "y", "d", "base_deg", "lift_cm", "t", "trim_deg", "steps", "attempts", "time_s", "log"]

# 偏差方向：落点相对物块中心。lat 右为正，dep 远（离车更远）为正，hgt 高为正
DIRS = {"左": ("lat", -1), "右": ("lat", 1), "前": ("dep", 1), "后": ("dep", -1), "高": ("hgt", 1), "低": ("hgt", -1)}
DIR_KEYS = {"l": "左", "r": "右", "f": "前", "b": "后", "u": "高", "d": "低"}


def pos_label(p):
    r, c = POSITIONS[p]
    return f"{CIRCLED[p - 1]} {r}·{c}"


def pos_text(p):
    """给人看的位置说明：离多远、往左/右多少"""
    r, c = POSITIONS[p]
    d, a = ROW_DEPTH_MM[r], COL_SIDE_DEG[c]
    side = "正前方" if not a else \
        f"往{c}约 {abs(d * math.tan(math.radians(a))) / 10:.1f}cm（底座要{'右' if a > 0 else '左'}转约 {abs(a):.0f}°，即 {a:+.0f}°）"
    return f"{pos_label(p)}：离机械臂底座中心约 {d / 10:.0f}cm，{side}"


def parse_positions(s):
    out = []
    for tok in re.split(r"[,，\s]+", s.strip()):
        if not tok:
            continue
        p = CIRCLED.index(tok) + 1 if tok in CIRCLED else int(tok)
        if p not in POSITIONS:
            raise ValueError(f"位置编号只能是 1~9：{tok}")
        if p not in out:
            out.append(p)
    return out


# ------------------------------------------------------------ CSV
def load_rows(path=None):
    path = path or BENCH_CSV
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def append_row(row, path=None):
    path = path or BENCH_CSV
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path)
    if not new:
        with open(path, newline="", encoding="utf-8") as f:
            header = next(csv.reader(f), [])
        if header != FIELDS:            # 程序升级改了列：旧文件改名留档
            base, ext = os.path.splitext(path)
            os.replace(path, f"{base}_{time.strftime('%Y%m%d_%H%M%S')}{ext}")
            new = True
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in FIELDS})


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# ------------------------------------------------------------ 真机 / 仿真
def _fail_row(kind, t0, now):
    return {"result": kind, "attempts": 0, "steps": 0, "time_s": round(now - t0, 1)}


class RealRig:
    """真机：完全走 main.py 的 open_devices / preflight / attach_task 和 GraspTask.run_once"""
    name = "真机"

    def __init__(self, step=False, force=False):
        self.step, self.force = step, force
        self.devs = self.task = None
        self.log = ""

    def open(self):
        import main
        from devices import ManualGuard, RealClock
        self.main = main
        f = getattr(sys.stdout, "f", None)            # main.Tee 写的运行日志（误报时要看确认那几行）
        self.log = os.path.basename(f.name) if f is not None else ""
        self.clock = RealClock()
        self.guard = ManualGuard(cfg.MANUAL_PROCESS)
        time.sleep(1.2)
        self.devs = main.open_devices(self.clock)
        problems = main.preflight(*self.devs)
        if problems and not self.force:
            print(f"有 {len(problems)} 项检查没通过，不启动（确认安全的话可以加 --force 跳过）")
            return False
        self.task = main.attach_task(None, self.devs, self.clock, self.step, self.guard)
        return True

    def run_once(self, p=None):
        from devices import DeviceError, DeviceResetError, ManualOverride
        t0 = self.clock.now()
        try:
            if self.devs is None:                      # 上一次设备出错关掉了：重开
                self.devs = self.main.open_devices(self.clock)
                self.task = self.main.attach_task(self.task, self.devs, self.clock, self.step, self.guard)
            return self.task.run_once()
        except DeviceResetError:                       # 升降轴复位：不能接着测（位置可能不对），退出
            self.main.close_devices(self.devs or [])
            self.devs = None
            raise
        except DeviceError as e:                       # 含手柄接管（ManualOverride）
            kind = "manual" if isinstance(e, ManualOverride) else "device_error"
            print(("手柄接管：" if kind == "manual" else "设备错误：") + str(e))
            self.main.close_devices(self.devs or [])
            self.devs = None
            return _fail_row(kind, t0, self.clock.now())

    def emergency(self):
        """Ctrl+C：升降轴急停、底盘停车（机械臂控制板没有急停指令，停在当前姿态）"""
        if not self.devs:
            return
        cam, lift, arm, chassis = self.devs
        for d in (lift, chassis):
            try:
                d.stop()
            except Exception as e:
                print("  急停指令发送失败：", e)

    def close(self):
        if self.devs:
            self.main.close_devices(self.devs)
            self.devs = None


class SimRig:
    """仿真：用 sim.py 的假设备跑同一个 GraspTask，物块按测试矩阵的位置摆"""
    name = "仿真"

    def __init__(self, seed=1, bias=1.0):
        self.seed, self.bias = seed, bias
        self.task = None
        self.n = 0
        self.truth = None      # 仿真里的真实结果（真机没有这个）
        self.log = ""
        self.emergencies = 0

    def open(self):
        from sim import SIM_POSES, make_car
        cfg.POSES = SIM_POSES
        self.car = make_car(cfg, self.seed)
        return True

    def place(self, p):
        """把物块摆到位置 p（深度按这台车标定的可抓范围等比例换算，仿真的标定值和真机不一样）"""
        r, c = POSITIONS[p]
        lo, hi = cfg.REACH_NEAR_D, cfg.REACH_FAR_D
        d = lo + (hi - lo) * {"近": 0.2, "中": 0.5, "远": 0.8}[r]
        ang = -math.radians(COL_SIDE_DEG[c])           # 世界坐标左为正，底座右转为正
        w = self.world
        reach = w.r_cam + d / w.depth_bias
        w.block[0] = w.PIVOT_FWD + reach * math.cos(ang)
        w.block[1] = reach * math.sin(ang)

    def run_once(self, p=None):
        from devices import DeviceError
        from sim import make_sim
        from task import GraspTask
        self.n += 1
        world, clock, cam, lift, arm, chassis = make_sim(cfg, self.seed * 1000 + self.n, self.bias, cfg.R_CAM_MM,
                                                         self.car)
        self.world = world
        if p is not None:
            self.place(p)
        if self.task is None:
            self.task = GraspTask(cfg, clock, cam, lift, arm, chassis, verbose=False)
        else:
            self.task.clock, self.task.cam, self.task.lift, self.task.arm, self.task.chassis = \
                clock, cam, lift, arm, chassis
            self.task.base_deg = 0.0
        try:
            row = self.task.run_once()
        except DeviceError as e:
            row = _fail_row("device_error", 0.0, clock.now())
            print("设备错误：", e)
        self.truth = world.removed or (world.picked and row["result"] == "success")
        return row

    def emergency(self):
        self.emergencies += 1

    def close(self):
        pass


# ------------------------------------------------------------ 交互
def _ask(ask, prompt, valid=None, default=None):
    """问到有效回答为止；valid 为 None 时任何回答都行"""
    while True:
        a = ask(prompt).strip()
        if not a and default is not None:
            return default
        if valid is None or a.lower() in valid:
            return a.lower() if valid is not None else a
        print("  没看懂，请按提示输入")


def parse_dir(s):
    """'右' / 'r' / '右前' / 'rf' → '右前'；空或'不知道' → ''；看不懂 → None"""
    s = s.strip().lower().replace("偏", "")
    if s in ("", "?", "？", "不知道", "x"):
        return ""
    out = ""
    for ch in s:
        ch = DIR_KEYS.get(ch, ch)
        if ch not in DIRS:
            return None
        if ch not in out:
            out += ch
    axes = [DIRS[c][0] for c in out]
    return out if len(axes) == len(set(axes)) else None      # 不能同时"左右"


def parse_mm(s):
    m = re.search(r"-?\d+(\.\d+)?", s)
    return abs(float(m.group())) if m else None


def ask_outcome(ask, prog, grasped_tried):
    """问真实结果。返回 dict(truth, dev_dir, dev_mm, note)；返回 None = 这次不算、重测"""
    hint = "" if grasped_tried else "（这次机械臂没有伸出去抓，回车=没抓到）"
    a = _ask(ask, f"真的抓到了吗？{hint} [y=抓到 / n=没抓到 / r=这次不算，重测]：",
             {"y", "n", "r", "是", "否"}, None if grasped_tried else "n")
    if a == "r":
        return None
    truth = a in ("y", "是")
    dev_dir, dev_mm = "", None
    if grasped_tried:
        if truth:
            q = "爪子居中吗？回车=基本居中；偏了就输入方向（同下）："
        else:
            q = ("爪子偏哪边？[左l 右r 前f(落得比物块远) 后b(落得近) 高u(没够到) 低d(压到)，"
                 "可组合如 右前/rf；回车=不知道]：")
        while True:
            d = parse_dir(ask(q))
            if d is not None:
                break
            print("  没看懂，方向用 左右前后高低 或 l r f b u d")
        if d:
            dev_dir = d
            while True:
                s = ask("大概偏几毫米？（回车=不知道）：").strip()
                if not s or parse_mm(s) is not None:
                    dev_mm = parse_mm(s) if s else None
                    break
                print("  请输入数字")
        elif truth:
            dev_dir, dev_mm = "中", 0.0
    note = ask("备注（可选，回车跳过）：").strip()
    return {"truth": "y" if truth else "n", "dev_dir": dev_dir, "dev_mm": dev_mm, "note": note}


def make_plan(positions, repeat, done=None):
    """先把每个位置测一遍，再测第二遍……（同一位置的几次分散开，避免一段时间里的漂移集中在一个位置）"""
    done = dict(done or {})
    plan = []
    for k in range(repeat):
        for p in positions:
            if done.get(p, 0) > k:
                continue
            plan.append((p, k + 1))
    return plan


def run_bench(rig, positions, repeat, resume=False, ask=input, path=None):
    """主流程。返回本次记录了几条"""
    rows = load_rows(path)
    rnd = time.strftime("%m%d_%H%M%S")
    done = {}
    if resume:
        if rows:
            rnd = rows[-1]["round"]
            for r in rows:
                if r["round"] == rnd:
                    done[int(r["pos"])] = done.get(int(r["pos"]), 0) + 1
            print(f"接着第 {rnd} 轮测，已经测过：" +
                  ("、".join(f"{CIRCLED[p - 1]}×{n}" for p, n in sorted(done.items())) or "无"))
        else:
            print("bench.csv 里没有记录，从头开始")
    plan = make_plan(positions, repeat, done)
    if not plan:
        print("这一轮要测的位置都测完了。看结果：python3 bench.py report")
        return 0
    lo, hi = (cfg.REACH_NEAR_D, cfg.REACH_FAR_D)
    if rig.name == "真机" and not cfg.USE_CHASSIS:
        for name, d in ROW_DEPTH_MM.items():
            if not lo - cfg.REACH_MARGIN_MM <= d <= hi + cfg.REACH_MARGIN_MM:
                print(f"⚠ {name}排的距离约 {d / 10:.0f}cm 不在这台车标定的可抓范围 "
                      f"{(lo - cfg.REACH_MARGIN_MM) / 10:.1f}~{(hi + cfg.REACH_MARGIN_MM) / 10:.1f}cm 内，可能会报 out_of_reach")
    print(f"{rig.name}，{'有' if cfg.USE_CHASSIS else '无'}底盘，第 {rnd} 轮，共 {len(plan)} 次。"
          f"Ctrl+C 随时退出（升降轴急停，已完成的都已记录）")
    n_rec = 0
    i = 0
    try:
        while i < len(plan):
            p, k = plan[i]
            print(f"\n==== 第 {i + 1}/{len(plan)} 次   {pos_text(p)}   （这个位置第 {k}/{repeat} 次）")
            a = _ask(ask, "把物块放好，手离开 → 回车开始（s=跳过这次  q=退出）：", {"", "s", "q"})
            if a == "q":
                break
            if a == "s":
                i += 1
                continue
            try:
                row = rig.run_once(p)
            except KeyboardInterrupt:
                print("\n手动停止：升降轴急停、底盘停车。这一次不记录")
                rig.emergency()
                raise
            task = rig.task
            g = getattr(task, "last_grasp", None) if task else None
            m = getattr(task, "last_aligned", None) if task else None
            print(f"程序结果：{row['result']}   步数 {row['steps']}   用时 {row['time_s']}s"
                  + ("" if g else "   （没有伸出去抓）"))
            if isinstance(rig, SimRig):
                print(f"  （仿真的真实结果：{'抓到' if rig.truth else '没抓到'}）")
            out = ask_outcome(ask, row["result"], g is not None)
            if out is None:
                print("这次不算，同一个位置再来一次")
                continue
            rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "round": rnd, "pos": p, "pos_name": pos_label(p),
                   "chassis": int(bool(cfg.USE_CHASSIS)), "prog_result": row["result"], **out,
                   "x_deg": g["x_deg"] if g else (round(m.x_deg, 2) if m else None),
                   "y": g["y"] if g else (round(m.y, 1) if m else None),
                   "d": g["d"] if g else (round(m.d, 1) if m else None),
                   "base_deg": g["base_deg"] if g else None, "lift_cm": g["lift_cm"] if g else None,
                   "t": g["t"] if g else None, "trim_deg": cfg.GRASP_BASE_TRIM_DEG,
                   "steps": row["steps"], "attempts": row["attempts"], "time_s": row["time_s"], "log": rig.log}
            append_row(rec, path)
            n_rec += 1
            print(f"已记录：{pos_label(p)} 程序 {row['result']} / 真实 {'抓到' if out['truth'] == 'y' else '没抓到'}"
                  + (f" 偏{out['dev_dir']}" if out["dev_dir"] and out["dev_dir"] != "中" else "")
                  + (f" {out['dev_mm']:.0f}mm" if out["dev_mm"] else "") + (f"  备注：{out['note']}" if out["note"] else ""))
            i += 1
    except (KeyboardInterrupt, EOFError):
        print("\n退出。")
    left = len(plan) - i
    print(f"本次记录了 {n_rec} 次" + (f"，还剩 {left} 次没测：下次 python3 bench.py --resume 接着测"
                                     "（--positions / --repeat 要和这次一样）" if left > 0 else ""))
    print("看统计：python3 bench.py report")
    return n_rec


# ------------------------------------------------------------ 统计
def _w(s):
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def _pad(s, w):
    return s + " " * max(0, w - _w(s))


def _table(rows, widths=None):
    widths = widths or [max(_w(r[i]) for r in rows) + 2 for i in range(len(rows[0]))]
    return "\n".join("  " + "".join(_pad(c, w) for c, w in zip(r, widths)).rstrip() for r in rows)


def dev_vec(r):
    """一条记录的偏差向量 {lat, dep, hgt}（mm）；没有方向或毫米 → None"""
    d = r.get("dev_dir", "")
    if d == "中":
        return {"lat": 0.0, "dep": 0.0, "hgt": 0.0}
    mm = _f(r.get("dev_mm"))
    if not d or mm is None:
        return None
    v = {}
    for ch in d:
        if ch in DIRS:
            ax, sgn = DIRS[ch]
            v[ax] = sgn * mm
    return v


AX_NAME = {"lat": ("右", "左"), "dep": ("前", "后"), "hgt": ("高", "低")}


def _fmt_dev(means):
    parts = []
    for ax in ("lat", "dep", "hgt"):
        v = means.get(ax)
        if v is not None and abs(v) >= 0.5:
            parts.append(f"{AX_NAME[ax][0 if v > 0 else 1]}{abs(v):.0f}")
    return " ".join(parts) or "居中"


def _means(rs):
    """按轴平均偏差。某条记录只报了"右"，就只算进 lat；报"居中"的三个轴都算 0"""
    acc = {}
    for r in rs:
        v = dev_vec(r)
        if v is None:
            continue
        for ax, x in v.items():
            acc.setdefault(ax, []).append(x)
    return {ax: sum(x) / len(x) for ax, x in acc.items()}, {ax: len(x) for ax, x in acc.items()}


def analyze(rows):
    """统计 + 建议；返回 (文本行列表, 建议列表)"""
    out, tips = [], []
    by = {p: [r for r in rows if int(r["pos"]) == p] for p in POSITIONS}
    ok = lambda rs: sum(r["truth"] == "y" for r in rs)                     # noqa: E731
    fp = lambda rs: sum(r["truth"] != "y" and r["prog_result"] == "success" for r in rs)   # noqa: E731
    n = len(rows)
    n_ok, n_fp = ok(rows), fp(rows)
    n_fn = sum(r["truth"] == "y" and r["prog_result"] != "success" for r in rows)
    ts = [_f(r["time_s"]) for r in rows if _f(r["time_s"]) is not None]
    out.append(f"共 {n} 次：真实抓到 {n_ok} 次 = {100 * n_ok / n:.0f}%   误报（程序说抓到、实际没有）{n_fp} 次   "
               f"漏报（实际抓到、程序说没有）{n_fn} 次" + (f"   平均用时 {sum(ts) / len(ts):.1f}s" if ts else ""))
    trims = sorted({r["trim_deg"] for r in rows})
    if len(trims) > 1:
        out.append(f"⚠ 这些记录里左右微调不止一个值（{', '.join(trims)}°），偏差统计混在一起了；"
                   "改过 trim 之后建议只看新的一轮（report --round）")

    # 3×3 成功率
    out.append("\n按位置的真实成功率（抓到/次数，误报次数）：")
    tab = [["", "左", "中", "右"]]
    for ri, rn in enumerate(ROW_DEPTH_MM):
        line = [f"{rn} {ROW_DEPTH_MM[rn] / 10:.0f}cm"]
        for ci in range(3):
            p = ri * 3 + ci + 1
            rs = by[p]
            if not rs:
                line.append(f"{CIRCLED[p - 1]} —")
            else:
                line.append(f"{CIRCLED[p - 1]} {ok(rs)}/{len(rs)}" + (f" 误报{fp(rs)}" if fp(rs) else ""))
        tab.append(line)
    out.append(_table(tab, [10, 16, 16, 16]))

    def rate(ps):
        rs = [r for p in ps for r in by[p]]
        return (ok(rs) / len(rs), len(rs)) if rs else (None, 0)
    rows_rate = {rn: rate([ri * 3 + c + 1 for c in range(3)]) for ri, rn in enumerate(ROW_DEPTH_MM)}
    cols_rate = {cn: rate([r * 3 + ci + 1 for r in range(3)]) for ci, cn in enumerate(COL_SIDE_DEG)}
    out.append("  按深度：" + "  ".join(f"{k} {100 * v[0]:.0f}%({v[1]})" for k, v in rows_rate.items() if v[1])
               + "    按左右：" + "  ".join(f"{k} {100 * v[0]:.0f}%({v[1]})" for k, v in cols_rate.items() if v[1]))

    # 偏差
    pm = {p: _means(by[p]) for p in POSITIONS}
    out.append("\n按位置的平均偏差（爪子落点相对物块中心，mm；右/左、前=落得远/后=落得近、高/低；括号里是有偏差记录的次数）：")
    tab = [["", "左", "中", "右"]]
    for ri, rn in enumerate(ROW_DEPTH_MM):
        line = [rn]
        for ci in range(3):
            p = ri * 3 + ci + 1
            m, c = pm[p]
            line.append(f"{CIRCLED[p - 1]} " + (f"{_fmt_dev(m)}({max(c.values())})" if c else "—"))
        tab.append(line)
    out.append(_table(tab, [6, 18, 18, 18]))

    # -------- 建议：左右
    lat = {p: pm[p][0]["lat"] for p in POSITIONS if "lat" in pm[p][0]}
    depth = [_f(r["d"]) for r in rows if _f(r["d"])]
    dmean = sum(depth) / len(depth) if depth else 180.0
    if len(lat) >= 3:
        right = [p for p, v in lat.items() if v >= BIAS_MM]
        left = [p for p, v in lat.items() if v <= -BIAS_MM]
        # 用各位置平均偏差的中位数：多数位置一致时它就是多数位置的偏差，不会被少数"居中"的位置拉小
        mean_all = statistics.median(lat.values())
        col_mean = {}
        for ci, cn in enumerate(COL_SIDE_DEG):
            vs = [lat[p] for p in lat if (p - 1) % 3 == ci]
            if vs:
                col_mean[cn] = sum(vs) / len(vs)
        trim_now = cfg.GRASP_BASE_TRIM_DEG
        for side, ps, sgn in (("右", right, 1), ("左", left, -1)):
            if len(ps) * 2 > len(lat) and abs(mean_all) >= BIAS_MM and (mean_all > 0) == (sgn > 0):
                tips.append(f"多数位置（{len(ps)}/{len(lat)}）偏{side}，约 {abs(mean_all):.0f}mm（各位置中位数） → 运行 "
                            f"python3 calib.py trim {mean_all:.0f} {dmean:.0f}（偏右为正、可累加；"
                            f"当前微调 {trim_now:+.2f}°），然后再测一轮")
                break
        else:
            # 不是多数一致：看是不是只有一侧偏
            for cn in ("右", "左"):
                other = [v for k, v in col_mean.items() if k != cn]
                v = col_mean.get(cn)
                if v is not None and abs(v) >= BIAS_MM and other and all(abs(o) < BIAS_MM or (o > 0) != (v > 0)
                                                                          for o in other):
                    tips.append(f"只有{cn}侧位置偏{'右' if v > 0 else '左'}（约 {abs(v):.0f}mm），其它位置不偏 → "
                                f"和底座角度有关（回差 / 遮挡），先别用 trim；等任务 09 的底座消回差，或先只摆中间")
            if col_mean.get("右") is not None and col_mean.get("左") is not None and \
                    col_mean["右"] >= BIAS_MM and col_mean["左"] <= -BIAS_MM:
                tips.append("右侧偏右、左侧偏左（越往外偏得越多）→ 底座转多了：检查 BASE_US_PER_DEG / 底座灵敏度，"
                            "或回差（任务 09）")
            elif col_mean.get("右") is not None and col_mean.get("左") is not None and \
                    col_mean["右"] <= -BIAS_MM and col_mean["左"] >= BIAS_MM:
                tips.append("右侧偏左、左侧偏右 → 底座转少了：小角度不到位（死区 / 回差，任务 09）")

    # -------- 建议：前后、高低（按深度）
    for ax, words in (("dep", ("落得偏远", "落得偏近")), ("hgt", ("偏高（没够到）", "偏低（压到物块）"))):
        rm = {}
        for ri, rn in enumerate(ROW_DEPTH_MM):
            vs = [pm[ri * 3 + c + 1][0][ax] for c in range(3) if ax in pm[ri * 3 + c + 1][0]]
            if vs:
                rm[rn] = sum(vs) / len(vs)
        bad = {k: v for k, v in rm.items() if abs(v) >= BIAS_MM}
        if not bad:
            continue
        if len(bad) == len(rm) and len(rm) >= 2 and len({v > 0 for v in bad.values()}) == 1:
            v = sum(bad.values()) / len(bad)
            tips.append(f"各个深度都{words[0 if v > 0 else 1]}约 {abs(v):.0f}mm → 两个伸出姿态整体偏了："
                        + ("重标 REACH_NEAR_D / REACH_FAR_D（calib.py teach）" if ax == "dep"
                           else "两个姿态都重标（calib.py teach），或者调整平台高度"))
            continue
        for k, v in bad.items():
            w = words[0 if v > 0 else 1]
            if k == "远":
                tips.append(f"远处{w}约 {abs(v):.0f}mm → reach_far 标得不准：python3 calib.py teach far 重标")
            elif k == "近":
                tips.append(f"近处{w}约 {abs(v):.0f}mm → reach_near 标得不准：python3 calib.py teach near 重标")
            else:
                tips.append(f"只有中间深度{w}约 {abs(v):.0f}mm → 两个姿态在关节空间插值的误差："
                            "标一个 reach_mid 做三点插值（任务 09）")

    # -------- 建议：成功率的形状
    rr = {k: v for k, v in rows_rate.items() if v[1] >= 2}
    if all(k in rr for k in ROW_DEPTH_MM) and rr["中"][0] + 0.25 <= min(rr["近"][0], rr["远"][0]):
        tips.append("中间深度没抓到、两端抓到 → 插值误差：标 reach_mid（任务 09）")
    if n_fp:
        logs = sorted({r["log"] for r in rows if r["truth"] != "y" and r["prog_result"] == "success" and r["log"]})
        tips.append(f"程序说抓到、实际没抓到 {n_fp} 次 → 确认被骗：把当时日志里\"观察确认\"那几行发给我"
                    + (f"（{', '.join('logs/' + x for x in logs)}）" if logs else ""))
    res = {}
    for r in rows:
        res[r["prog_result"]] = res.get(r["prog_result"], 0) + 1
    if res.get("ghost"):
        tips.append(f"有 {res['ghost']} 次停在\"识别到机械臂自己\"（ghost）→ 跟踪器锁住爪子脚，等任务 10；"
                    "临时：把物块挪离画面左侧再试")
    if res.get("out_of_reach"):
        tips.append(f"有 {res['out_of_reach']} 次 out_of_reach → 物块放得太近/太远，或者重标近/远姿态扩大范围")
    if res.get("lift_limit"):
        tips.append(f"有 {res['lift_limit']} 次 lift_limit → 平台太低/太高，换平台高度或重标")
    if n >= 9 and n_ok / n >= 0.9 and n_fp == 0:
        tips.append("真实成功率 ≥ 90% 且没有误报：可以进入底盘联调（抓取与调试方案第 4 节）")

    # 混淆表
    out.append("\n程序结果 vs 真实结果：")
    tab = [["程序结果", "真实抓到", "真实没抓到"]]
    for k in sorted(res, key=lambda k: (k != "success", k)):
        rs = [r for r in rows if r["prog_result"] == k]
        tab.append([k, str(ok(rs)), str(len(rs) - ok(rs))])
    out.append(_table(tab, [16, 10, 10]))
    return out, tips


def report(path=None, which=None, all_rounds=False):
    rows = load_rows(path)
    if not rows:
        print("还没有记录（logs/bench.csv）。先运行 python3 bench.py")
        return None
    rounds = []
    for r in rows:
        if r["round"] not in rounds:
            rounds.append(r["round"])
    if all_rounds:
        sel, title = rows, f"全部 {len(rounds)} 轮"
    else:
        rnd = which or rounds[-1]
        if rnd not in rounds:
            print(f"没有第 {rnd} 轮。已有：{', '.join(rounds)}")
            return None
        sel, title = [r for r in rows if r["round"] == rnd], f"第 {rnd} 轮"
        if len(rounds) > 1:
            title += f"（共 {len(rounds)} 轮：{', '.join(rounds)}；--all 看全部，--round R 看指定一轮）"
    print(f"bench.py 真实结果统计：{title}")
    out, tips = analyze(sel)
    print("\n".join(out))
    print("\n建议：")
    for t in tips or ["暂时没有明显规律（偏差记录太少，或者各位置偏差方向不一致）"]:
        print("  → " + t)
    return tips


def main(argv=None):
    ap = argparse.ArgumentParser(description="真机试验记录工具（详细用法见文件开头）")
    ap.add_argument("cmd", nargs="?", default="run", choices=["run", "report"])
    ap.add_argument("--positions", default="1,2,3,4,5,6,7,8,9", help="要测的位置，例如 2,5,8")
    ap.add_argument("--repeat", type=int, default=2, help="每个位置测几次（默认 2）")
    ap.add_argument("--resume", action="store_true", help="接着上一轮没测完的继续")
    ap.add_argument("--chassis", action="store_true", help="用底盘（默认不用）")
    ap.add_argument("--step", action="store_true", help="单步模式：每个动作前按回车确认")
    ap.add_argument("--force", action="store_true", help="启动前检查没通过也继续")
    ap.add_argument("--sim", action="store_true", help="用仿真代替真机（练习用）")
    ap.add_argument("--seed", type=int, default=1, help="仿真随机种子")
    ap.add_argument("--csv", default=None, help="记录文件（默认 logs/bench.csv）")
    ap.add_argument("--all", action="store_true", help="report：统计全部轮次")
    ap.add_argument("--round", default=None, help="report：统计指定的一轮")
    a = ap.parse_args(argv)
    if a.cmd == "report":
        report(a.csv, a.round, a.all)
        return
    positions = parse_positions(a.positions)
    cfg.USE_CHASSIS = bool(a.chassis)
    if a.sim:
        rig = SimRig(a.seed)
        path = a.csv or os.path.join(HERE, cfg.LOG_DIR, "bench_sim.csv")     # 仿真不混进真机记录
    else:
        import main as grasp_main
        sys.stdout = grasp_main.Tee(os.path.join(grasp_main.LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.log")))
        print("启动：", " ".join(sys.argv), " calib.json 覆盖：", sorted(cfg.CALIB_LOADED))
        rig = RealRig(a.step, a.force)
        path = a.csv
        grasp_main.install_stop_signals()      # systemctl stop / SSH 断开也走下面的停车流程
    from devices import DeviceResetError
    try:
        if not rig.open():
            return
        run_bench(rig, positions, a.repeat, a.resume, input, path)
    except KeyboardInterrupt:
        print("\n退出。")
    except DeviceResetError as e:
        print("设备复位：", e)
        print(grasp_main.RESET_HINT)
    finally:
        rig.close()
    if a.sim:
        rel = os.path.relpath(path, HERE)
        rel = path if rel.startswith("..") else rel
        print(f"（仿真记录在 {rel}；看统计：python3 bench.py report --csv {rel}）")


if __name__ == "__main__":
    main()
