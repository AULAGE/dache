"""
K230 存图 ↔ 树莓派动作记录 离线对齐（任务 12，论文数据集）。只用标准库，在电脑或树莓派上跑。

输入
  树莓派：logs/record_*.jsonl（main.py --record，且 config.K230_SYNC_PORT 配了同步串口）
          里面的 {"ev":"sync","t":发出时刻,"seq":序号,"pi_t":"时间原文"} 是发给 K230 的同步标记
  K230：  SD 卡 /sdcard/datalog/sNNNN/ 整个目录拷出来（frames.csv、sync.csv、info.txt、img/）

做法
  1. 按（序号, 时间原文）把两边的同步标记配对（树莓派程序重启后序号会从 1 重来，时间原文不会重复）
  2. 线性拟合两边时钟：pi_t = a + b × k230_s（偏移 + 漂移），去掉离群点后再拟合一次
     - K230 那一侧的时刻取 sync.csv 里 (k230_ms_prev, k230_ms] 区间的中点（K230 每帧只查几次串口）
     - 树莓派那一侧加上串口发送这一行的时间（字节数 × 10 / 波特率），因为 K230 收齐的是最后一个字节
  3. 每张图（frames.csv 的拍照时刻）换算成树莓派时间，找之前 / 之后最近的动作命令、当时的机械臂各舵机命令脉宽、
     升降轴高度、属于第几次抓取以及这次的结果 → dataset.csv
  4. 自检：K230 发出去的位置帧（frames.csv 的 sent=pos，带发送时刻和 X/Y/D）在树莓派 jsonl 里也有一条
     （k230 事件，带接收时刻和同样的 X/Y/D）。用拟合出的时钟预测接收时刻，和实际接收时刻比 → 对齐误差的上界
     （里面还包含串口传输约 1ms 和树莓派读串口的等待 0~50ms）

用法
  python3 research/align_k230_log.py --record logs/record_*.jsonl --k230 D:/datalog/s0003 -o dataset.csv
  python3 research/align_k230_log.py --record logs/record_x.jsonl --k230 s0003 s0004 --all-frames -o all.csv
"""
import argparse
import bisect
import csv
import glob
import json
import os
import statistics
import sys

DEFAULT_BAUD = 115200

DATASET_COLS = [
    "record", "session", "image", "frame", "k230_ms", "pi_t", "episode", "result", "phase",
    "in_cmd", "prev_cmd", "prev_cmd_t", "prev_cmd_args", "dt_prev_end", "next_cmd", "next_cmd_t", "next_cmd_args",
    "s1", "s2", "s3", "s4", "s5", "s6", "lift_cm", "lift_busy", "sent", "sx", "sy", "sd", "trk", "n_det",
]


# ============================================================ 读文件
def load_jsonl(path):
    evs = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                evs.append(json.loads(line))
            except ValueError:
                pass                    # 断电时最后一行可能写了一半
    return evs


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def load_session(d):
    """K230 会话目录 -> {"dir", "name", "frames": [...], "syncs": [...], "info": {...}}"""
    info = {}
    p = os.path.join(d, "info.txt")
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            for line in f:
                if "=" in line:
                    k, v = line.rstrip("\n").split("=", 1)
                    info[k.strip()] = v.strip()
    frames = []
    for r in _read_csv(os.path.join(d, "frames.csv")):
        try:
            r["frame"] = int(r["frame"])
            r["t_ms"] = float(r["t_ms"])
        except (KeyError, ValueError, TypeError):
            continue                    # 断电写了一半的行
        frames.append(r)
    syncs = []
    for r in _read_csv(os.path.join(d, "sync.csv")):
        try:
            syncs.append({"seq": int(r["seq"]), "pi_t": r["pi_t"].strip(),
                          "k0": float(r["k230_ms_prev"]), "k1": float(r["k230_ms"])})
        except (KeyError, ValueError, TypeError, AttributeError):
            continue
    return {"dir": d, "name": os.path.basename(os.path.normpath(d)), "frames": frames, "syncs": syncs, "info": info}


# ============================================================ 时钟对齐
def match_syncs(evs, k_syncs, baud=DEFAULT_BAUD):
    """
    配对两边的同步标记 -> [(pi_arrival_s, k230_mid_s, k230_halfwidth_s, seq)]
    pi_arrival = 发出时刻 + 这一行的发送时间（K230 收齐最后一个字节的时刻）
    """
    pi = {}
    for e in evs:
        if e.get("ev") == "sync" and not e.get("err") and "seq" in e and "pi_t" in e:
            pi[(int(e["seq"]), str(e["pi_t"]))] = float(e["t"])
    out = []
    for s in k_syncs:
        t = pi.get((s["seq"], s["pi_t"]))
        if t is None:
            continue
        nbytes = len("SYNC {} {}\n".format(s["seq"], s["pi_t"]))
        mid = (s["k0"] + s["k1"]) / 2000.0
        out.append((t + nbytes * 10.0 / baud, mid, max(0.0, s["k1"] - s["k0"]) / 2000.0, s["seq"]))
    out.sort(key=lambda p: p[1])
    return out


def _linfit(xs, ys, ws=None):
    """加权最小二乘 y = a + b x，返回 (a, b)"""
    n = len(xs)
    ws = ws or [1.0] * n
    sw = sum(ws)
    mx = sum(w * x for w, x in zip(ws, xs)) / sw
    my = sum(w * y for w, y in zip(ws, ys)) / sw
    sxx = sum(w * (x - mx) ** 2 for w, x in zip(ws, xs))
    if n < 2 or sxx <= 1e-12:
        return my - mx, 1.0              # 只有一个点（或都在同一时刻）：只估偏移，漂移按 0
    b = sum(w * (x - mx) * (y - my) for w, x, y in zip(ws, xs, ys)) / sxx
    return my - b * mx, b


def fit_clock(pairs, min_tol_s=0.002):
    """
    pairs = match_syncs 的结果。返回拟合 dict：pi_t = a + b × k230_s
      a, b, drift_ppm = (b − 1) × 10⁶（负数 = K230 的时钟走得比树莓派快）, n, used, rms_ms, max_ms, halfwidth_ms
    离群点：残差超过 max(4 × 1.4826 × MAD, 半区间宽度 + min_tol) 的去掉（比如 K230 那一帧卡住了很久）
    """
    if not pairs:
        return None
    xs = [p[1] for p in pairs]
    ys = [p[0] for p in pairs]
    # 权重：K230 查串口的区间越宽（那一帧卡得越久）越不准，按 1 / (半宽 + 1ms)² 加权
    wt = [1.0 / (p[2] + 0.001) ** 2 for p in pairs]
    a, b = _linfit(xs, ys, wt)
    keep = list(range(len(pairs)))
    for _ in range(3):
        res = [ys[i] - (a + b * xs[i]) for i in keep]
        if len(res) < 3:
            break
        med = statistics.median(res)
        mad = statistics.median(abs(r - med) for r in res)
        new = [i for i in keep
               if abs(ys[i] - (a + b * xs[i]) - med) <= max(4 * 1.4826 * mad, pairs[i][2] + min_tol_s)]
        if len(new) == len(keep) or len(new) < 2:
            break
        keep = new
        a, b = _linfit([xs[i] for i in keep], [ys[i] for i in keep], [wt[i] for i in keep])
    res = [ys[i] - (a + b * xs[i]) for i in keep]
    return {"a": a, "b": b, "drift_ppm": (b - 1.0) * 1e6, "n": len(pairs), "used": len(keep),
            "rms_ms": (sum(r * r for r in res) / len(res)) ** 0.5 * 1000.0,
            "max_ms": max(abs(r) for r in res) * 1000.0,
            "halfwidth_ms": statistics.median(p[2] for p in pairs) * 1000.0,
            "k_range_s": (min(xs), max(xs))}


def to_pi(fit, k230_ms):
    return fit["a"] + fit["b"] * k230_ms / 1000.0


# ============================================================ 动作时间线
class Timeline:
    """树莓派 jsonl 里的命令 / 抓取 / 阶段，按时间查询"""

    def __init__(self, evs):
        self.cmds = sorted((e for e in evs if e.get("ev") == "cmd" and "t" in e), key=lambda e: e["t"])
        self.cmd_t = [e["t"] for e in self.cmds]
        arm = [e for e in self.cmds if e.get("dev") == "arm" and isinstance(e.get("state"), dict)]
        self.arm_t = [e["t"] for e in arm]
        self.arm_state = [{int(k): v for k, v in e["state"].items()} for e in arm]
        lift_done = sorted((e for e in self.cmds if e.get("dev") == "lift" and e.get("state") is not None),
                           key=lambda e: e.get("t_end", e["t"]))
        self.lift_t = [e.get("t_end", e["t"]) for e in lift_done]
        self.lift_pos = [e["state"] for e in lift_done]
        # 升降轴忙的区间：move_to / move_by 本身；start_move_by 到之后第一个 wait_move 结束
        self.lift_busy = []
        pending = None
        for e in self.cmds:
            if e.get("dev") != "lift":
                continue
            if e.get("fn") in ("move_to", "move_by"):
                self.lift_busy.append((e["t"], e.get("t_end", e["t"])))
            elif e.get("fn") == "start_move_by":
                pending = e["t"]
            elif e.get("fn") == "wait_move" and pending is not None:
                self.lift_busy.append((pending, e.get("t_end", e["t"])))
                pending = None
        self.episodes = []
        start = None
        for e in sorted((e for e in evs if e.get("ev") in ("episode_start", "episode") and "t" in e),
                        key=lambda e: e["t"]):
            if e["ev"] == "episode_start":
                start = e
            elif start is not None:
                row = e.get("row") or {}
                self.episodes.append((start["t"], e["t"], row.get("episode", start.get("episode")), row.get("result")))
                start = None
        says = sorted((e for e in evs if e.get("ev") == "say" and "t" in e), key=lambda e: e["t"])
        self.say_t = [e["t"] for e in says]
        self.say_msg = [e.get("msg", "") for e in says]
        ts = [e["t"] for e in evs if isinstance(e.get("t"), (int, float))]
        self.t_range = (min(ts), max(ts)) if ts else (0.0, -1.0)

    @staticmethod
    def _last_le(ts, t):
        i = bisect.bisect_right(ts, t) - 1
        return i if i >= 0 else None

    def at(self, t):
        out = {}
        i = self._last_le(self.cmd_t, t)
        prev = self.cmds[i] if i is not None else None
        nxt = self.cmds[i + 1] if (i is not None and i + 1 < len(self.cmds)) else (
            self.cmds[0] if i is None and self.cmds else None)
        out["in_cmd"] = int(any(c["t"] <= t <= c.get("t_end", c["t"])
                                for c in self.cmds[max(0, (i or 0) - 3):(i or 0) + 1]))
        out["prev_cmd"] = _cmd_name(prev)
        out["prev_cmd_t"] = _r(prev["t"]) if prev else ""
        out["prev_cmd_args"] = _args(prev)
        out["dt_prev_end"] = _r(t - prev.get("t_end", prev["t"])) if prev else ""
        out["next_cmd"] = _cmd_name(nxt)
        out["next_cmd_t"] = _r(nxt["t"]) if nxt else ""
        out["next_cmd_args"] = _args(nxt)
        j = self._last_le(self.arm_t, t)
        st = self.arm_state[j] if j is not None else {}
        for sid in range(1, 7):
            out[f"s{sid}"] = st.get(sid, "")
        k = self._last_le(self.lift_t, t)
        out["lift_cm"] = self.lift_pos[k] if k is not None else ""
        out["lift_busy"] = int(any(a <= t <= b for a, b in self.lift_busy))
        out["episode"], out["result"] = "", ""
        for (a, b, ep, res) in self.episodes:
            if a <= t <= b:
                out["episode"], out["result"] = ep, res
                break
        s = self._last_le(self.say_t, t)
        out["phase"] = self.say_msg[s][:40] if (s is not None and out["episode"] != "") else ""
        return out


def _r(v):
    return round(v, 4)


def _cmd_name(e):
    return f'{e.get("dev")}.{e.get("fn")}' if e else ""


def _args(e):
    if not e:
        return ""
    return json.dumps({"args": e.get("args", []), "kw": e.get("kw", {})}, ensure_ascii=False, separators=(",", ":"))


# ============================================================ 自检：位置帧配对
def frame_match_check(frames, evs, fit, window_s=0.3):
    """K230 发出的位置帧 ↔ 树莓派收到的 k230 事件（X/Y/D 完全相同、时间在预测值 ±window 内）。
    返回 {"sent", "matched", "median_ms", "p95_ms", "residuals_ms"}；残差 = 实际接收 − 预测接收"""
    recv = sorted((e for e in evs if e.get("ev") == "k230"), key=lambda e: e["t"])
    rt = [e["t"] for e in recv]
    res = []
    sent = 0
    for f in frames:
        if f.get("sent") != "pos" or not f.get("t_send_ms"):
            continue
        try:
            x, y, d, ts = int(f["sx"]), int(f["sy"]), int(f["sd"]), float(f["t_send_ms"])
        except (TypeError, ValueError):
            continue
        sent += 1
        pred = to_pi(fit, ts)
        i0 = bisect.bisect_left(rt, pred - window_s)
        i1 = bisect.bisect_right(rt, pred + window_s)
        best = None
        for e in recv[i0:i1]:
            if int(round(e["x"])) == x and int(round(e["y"])) == y and int(round(e["d"])) == d:
                if best is None or abs(e["t"] - pred) < abs(best - pred):
                    best = e["t"]
        if best is not None:
            res.append((best - pred) * 1000.0)
    out = {"sent": sent, "matched": len(res), "residuals_ms": res}
    if res:
        s = sorted(abs(r) for r in res)
        out["median_ms"] = statistics.median(res)
        out["p95_ms"] = s[min(len(s) - 1, int(0.95 * len(s)))]
    return out


# ============================================================ 生成数据集
def build_rows(evs, sess, fit, record_name="", all_frames=False):
    tl = Timeline(evs)
    lo, hi = tl.t_range
    rows = []
    for f in sess["frames"]:
        if not all_frames and not f.get("img"):
            continue
        t = to_pi(fit, f["t_ms"])
        if t < lo - 1.0 or t > hi + 1.0:
            continue                      # 这张图不在这次树莓派记录的时间范围内
        r = {"record": record_name, "session": sess["name"],
             "image": os.path.join(sess["dir"], f["img"]) if f.get("img") else "",
             "frame": f["frame"], "k230_ms": f["t_ms"], "pi_t": _r(t),
             "sent": f.get("sent", ""), "sx": f.get("sx", ""), "sy": f.get("sy", ""), "sd": f.get("sd", ""),
             "trk": f.get("trk", ""), "n_det": f.get("n_det", "")}
        r.update(tl.at(t))
        rows.append(r)
    return rows


def align(record_paths, session_dirs, baud=DEFAULT_BAUD, all_frames=False, log=print):
    """返回 (rows, reports)。每个树莓派记录文件单独拟合，配对最多的那个 K230 会话就是它对应的会话"""
    sessions = [load_session(d) for d in session_dirs]
    rows, reports = [], []
    for rp in record_paths:
        evs = load_jsonl(rp)
        best = None
        for s in sessions:
            pairs = match_syncs(evs, s["syncs"], baud)
            if best is None or len(pairs) > len(best[1]):
                best = (s, pairs)
        name = os.path.basename(rp)
        n_pi = sum(1 for e in evs if e.get("ev") == "sync")
        if best is None or not best[1]:
            log(f"{name}: 树莓派同步标记 {n_pi} 个，没有和任何 K230 会话配上（同步线没接 / K230 没开 IMAGE_LOG_ENABLE？）")
            reports.append({"record": name, "fit": None})
            continue
        sess, pairs = best
        fit = fit_clock(pairs)
        chk = frame_match_check(sess["frames"], evs, fit)
        rep = {"record": name, "session": sess["name"], "pi_syncs": n_pi, "k230_syncs": len(sess["syncs"]),
               "fit": fit, "check": chk}
        reports.append(rep)
        log(format_report(rep))
        if fit["used"] < 2:
            log("  ⚠ 只有 1 个同步标记配上，只能估偏移，不能估漂移")
        rows.extend(build_rows(evs, sess, fit, name, all_frames))
    return rows, reports


def format_report(rep):
    f, c = rep["fit"], rep.get("check") or {}
    out = [f'{rep["record"]} ↔ K230 会话 {rep["session"]}：同步标记 树莓派 {rep["pi_syncs"]} / K230 {rep["k230_syncs"]} / '
           f'配上 {f["n"]}（用 {f["used"]}）',
           f'  时钟：pi_t = {f["a"]:.4f} + {f["b"]:.9f} × k230_s   漂移 {f["drift_ppm"]:+.1f} ppm（负 = K230 走得快）   '
           f'残差 RMS {f["rms_ms"]:.2f}ms 最大 {f["max_ms"]:.2f}ms   K230 查串口区间半宽中位数 {f["halfwidth_ms"]:.1f}ms']
    if c.get("sent"):
        if c.get("matched"):
            out.append(f'  自检：K230 发出位置帧 {c["sent"]}，树莓派对上 {c["matched"]}；'
                       f'接收 − 预测 中位数 {c["median_ms"]:+.1f}ms，|残差| 95% 分位 {c["p95_ms"]:.1f}ms'
                       f'（含串口传输约 1ms + 树莓派读串口等待 0~50ms）')
        else:
            out.append(f'  自检：K230 发出位置帧 {c["sent"]}，一个都没对上（K230_ROTATE 不是 0？或者时钟拟合错了）')
    return "\n".join(out)


def write_dataset(rows, path):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=DATASET_COLS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def main(argv):
    ap = argparse.ArgumentParser(description="K230 存图 ↔ 树莓派动作记录 离线对齐 → dataset.csv")
    ap.add_argument("--record", nargs="+", required=True, help="树莓派 record_*.jsonl（可以用通配符）")
    ap.add_argument("--k230", nargs="+", required=True, help="K230 会话目录（datalog/sNNNN），可以多个")
    ap.add_argument("-o", "--out", default="dataset.csv")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="同步串口波特率（算发送一行要多久）")
    ap.add_argument("--all-frames", action="store_true", help="没存图的帧也输出（默认只输出有图的）")
    a = ap.parse_args(argv)
    recs = sorted({p for pat in a.record for p in (glob.glob(pat) or [pat])})
    sess = sorted({p for pat in a.k230 for p in (glob.glob(pat) or [pat])})
    rows, _ = align(recs, sess, a.baud, a.all_frames)
    write_dataset(rows, a.out)
    print(f"写出 {len(rows)} 行 → {a.out}")
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
