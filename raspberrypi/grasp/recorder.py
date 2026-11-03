"""
真机动作记录器（论文数据集的第一步，见 docs/论文研究方案.md 第 3.1、6 节）

把每一条动作命令（开始/结束时间、参数、返回值、执行后状态）和 K230 的每一帧（位置帧 + 心跳）
带时间戳记到 logs/record_*.jsonl，一行一个事件。只记录，不改任何动作。

用法
  python3 main.py --once --no-chassis --record     # 跑的同时记录（bench.py 用环境变量 GRASP_RECORD=1）
  python3 recorder.py summary logs/record_xxx.jsonl # 看记录了什么
  python3 recorder.py pairs   logs/record_xxx.jsonl > pairs.csv
        # 抽出"动作 → 画面响应"样本对：每次只转底座 / 只动升降轴，前后各取 K230 中位数
        # 用来分析底座回差、死区、灵敏度（不需要图像）
  python3 recorder.py backlash logs/record_*.jsonl  # 按转动方向统计底座"命令角 vs 画面实际变化"

事件格式（JSON）
  {"ev":"header", "t":单调时钟, "wall":Unix 时间, "argv":[...], "calib":[...]}
  {"ev":"k230",   "t":..., "x":mm, "y":mm, "d":mm}           位置帧（已按 ROTATE_DEG 旋转，和 task 看到的一致）
  {"ev":"hb",     "t":..., "n":这段时间新增的心跳数}
  {"ev":"cmd",    "t":开始, "t_end":结束, "dev":"arm/lift/chassis", "fn":"move", "args":[...], "kw":{...},
                  "ret":返回值, "state":执行后状态（机械臂各舵机脉宽 / 升降轴高度）, "err":异常（有的话）}
  {"ev":"say",    "t":..., "msg":任务打印的一行}             用来切分阶段（对准 / 抓取 / 确认）
  {"ev":"episode","t":..., "row":{...}}                       一次抓取的结果（同 episodes.csv）
"""
import csv
import json
import math
import os
import statistics
import sys
import threading
import time

RECORDED = {"move_to", "move_by", "start_move_by", "wait_move", "stop", "drive", "forward", "turn", "move"}


def _jsonable(v):
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, float):
        return round(v, 4)
    if v is None or isinstance(v, (int, str, bool)):
        return v
    return str(v)


class Recorder:
    def __init__(self, path, clock, cam=None, poll_s=0.05):
        self.path = path
        self.clock = clock
        self.cam = cam
        self.poll_s = poll_s
        self._lock = threading.Lock()
        self._poll_lock = threading.Lock()
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._f = open(path, "a", encoding="utf-8")
        self._run = True
        self._last_frame_t = -1e18
        self._last_hb = None
        self.events = 0
        self.write("header", wall=time.time(), argv=sys.argv)
        self._thread = None
        if cam is not None and poll_s:
            self._thread = threading.Thread(target=self._poll, daemon=True)
            self._thread.start()

    def write(self, ev, t=None, **data):
        rec = {"ev": ev, "t": round(self.clock.now() if t is None else t, 4), **_jsonable(data)}
        line = json.dumps(rec, ensure_ascii=False)
        with self._lock:
            if self._f is None:
                return
            self._f.write(line + "\n")
            self.events += 1

    def set_cam(self, cam):
        """设备重开后换新的 K230 对象（帧时间戳是同一个时钟，不会重复记）"""
        self.cam = cam
        self._last_hb = None
        if self._thread is None and cam is not None and self.poll_s:
            self._thread = threading.Thread(target=self._poll, daemon=True)
            self._thread.start()

    def poll_once(self):
        with self._poll_lock:
            self._poll_once()

    def _poll_once(self):
        cam = self.cam
        if cam is None:
            return
        lock = getattr(cam, "_lock", None)
        frames = getattr(cam, "frames", None)
        if frames is None:
            frames = getattr(cam, "sent", None)       # 仿真相机（sim.SimCam）
        if frames is None:
            return
        if lock is not None:
            with lock:
                new = [f for f in frames if f[0] > self._last_frame_t]
                hb = getattr(cam, "heartbeats", None)
        else:
            new = [f for f in list(frames) if f[0] > self._last_frame_t]
            hb = getattr(cam, "heartbeats", None)
        for t, x, y, d in new:
            self.write("k230", t=t, x=x, y=y, d=d)
        if new:
            self._last_frame_t = new[-1][0]
        if hb is not None:
            if self._last_hb is not None and hb > self._last_hb:
                self.write("hb", n=hb - self._last_hb)
            self._last_hb = hb

    def _poll(self):
        while self._run:
            try:
                self.poll_once()
            except Exception:
                pass
            time.sleep(self.poll_s)

    def flush(self):
        with self._lock:
            if self._f:
                self._f.flush()

    def close(self):
        self._run = False
        try:
            self.poll_once()
        except Exception:
            pass
        with self._lock:
            if self._f:
                self._f.close()
                self._f = None


class RecordProxy:
    """包住一个设备：动作类调用照常执行，同时记一条 cmd 事件"""

    def __init__(self, dev, name, rec):
        self._dev, self._name, self._rec = dev, name, rec
        self._pulses = {}                # 机械臂：各舵机最后一次的命令脉宽（设备没有 state 时用）

    def __getattr__(self, k):
        attr = getattr(self._dev, k)
        if k not in RECORDED or not callable(attr):
            return attr

        def wrapped(*a, **kw):
            self._rec.poll_once()            # 先把动作之前收到的帧写进去，保证事件按时间顺序
            t0 = self._rec.clock.now()
            err = None
            ret = None
            try:
                ret = attr(*a, **kw)
                return ret
            except BaseException as e:
                err = f"{type(e).__name__}: {e}"
                raise
            finally:
                state = None
                if self._name == "arm":
                    if k == "move" and a and isinstance(a[0], dict) and err is None:
                        self._pulses.update({int(sid): int(max(500, min(2500, v))) for sid, v in a[0].items()})
                    state = dict(getattr(self._dev, "state", None) or self._pulses)
                elif self._name == "lift":
                    state = getattr(self._dev, "pos", None)
                data = {"dev": self._name, "fn": k, "args": list(a), "kw": kw, "ret": ret,
                        "t_end": self._rec.clock.now(), "state": state}
                if err:
                    data["err"] = err
                self._rec.poll_once()
                self._rec.write("cmd", t=t0, **data)
                self._rec.flush()
        return wrapped


def hook_task(task, rec):
    """任务打印的每一行、每次抓取的结果也记下来（用来切分阶段、打奖励标签）；不改变任务行为"""
    if getattr(task, "_recorder", None) is rec:
        return
    task._recorder = rec
    orig_say, orig_run = task.say, task.run_once

    def say(*a):
        rec.write("say", msg=" ".join(str(x) for x in a))
        return orig_say(*a)

    def run_once():
        rec.write("episode_start", episode=task.episode + 1)
        row = orig_run()
        rec.poll_once()
        rec.write("episode", row=row)
        rec.flush()
        return row

    task.say, task.run_once = say, run_once


# ============================================================ 离线分析
def load(paths):
    evs = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        evs.append(json.loads(line))
                    except ValueError:
                        pass          # 断电时最后一行可能写了一半
    return evs


def summary(evs):
    from collections import Counter
    c = Counter(e["ev"] for e in evs)
    cmds = Counter(f'{e["dev"]}.{e["fn"]}' for e in evs if e["ev"] == "cmd")
    ts = [e["t"] for e in evs if "t" in e]
    out = [f"事件 {len(evs)} 条，时长 {max(ts) - min(ts):.0f}s" if ts else "没有事件",
           "  " + "  ".join(f"{k}={v}" for k, v in sorted(c.items())),
           "  命令：" + "  ".join(f"{k}×{v}" for k, v in sorted(cmds.items()))]
    eps = [e["row"] for e in evs if e["ev"] == "episode"]
    if eps:
        res = Counter(r.get("result") for r in eps)
        out.append("  抓取：" + "  ".join(f"{k}×{v}" for k, v in res.items()))
    return "\n".join(out)


def _median_around(frames, t0, t1, n=5, last=True):
    fr = [f for f in frames if t0 <= f["t"] <= t1]
    if len(fr) < 2:
        return None
    fr = fr[-n:] if last else fr[:n]
    x = statistics.median(f["x"] for f in fr)
    y = statistics.median(f["y"] for f in fr)
    d = statistics.median(f["d"] for f in fr)
    return {"x": x, "y": y, "d": d, "x_deg": math.degrees(math.atan2(x, max(d, 1.0))), "n": len(fr)}


def pairs(evs, base_id=6, base_center=1500, us_per_deg=11.1, base_dir=-1, before_s=1.5, settle_s=0.25, after_s=1.5):
    """
    "动作 → 画面响应"样本对。只取单一动作（只转底座，或只动升降轴）且前后都看得到物块的。
    底座角度 = (脉宽 − 中位) / us_per_deg × base_dir（和 task.base_pulse 相反的换算；右转为正）
    """
    frames = [e for e in evs if e["ev"] == "k230"]
    cmds = sorted((e for e in evs if e["ev"] == "cmd" and not e.get("err")), key=lambda e: e["t"])
    rows = []
    base_prev = None
    last_dir = 0
    pending_lift = None
    for i, e in enumerate(cmds):
        nxt = cmds[i + 1]["t"] if i + 1 < len(cmds) else e["t_end"] + after_s + settle_s
        if e["dev"] == "arm" and e["fn"] == "move":
            pulses = {int(k): v for k, v in (e["args"][0] if e["args"] else {}).items()}
            st = {int(k): v for k, v in (e.get("state") or {}).items()}
            if set(pulses) == {base_id} and base_prev is not None:
                cmd_deg = (pulses[base_id] - base_prev) / us_per_deg * base_dir
                if abs(cmd_deg) > 1e-6:
                    a = _median_around(frames, e["t"] - before_s, e["t"])
                    b = _median_around(frames, e["t_end"] + settle_s, min(nxt, e["t_end"] + settle_s + after_s),
                                       last=False)
                    d = 1 if cmd_deg > 0 else -1
                    if a and b:
                        rows.append({"t": e["t"], "axis": "base", "cmd": round(cmd_deg, 3),
                                     "reversed": int(last_dir != 0 and d != last_dir),
                                     "base_deg_before": round((base_prev - base_center) / us_per_deg * base_dir, 2),
                                     "dx_deg": round(b["x_deg"] - a["x_deg"], 3), "dy": round(b["y"] - a["y"], 2),
                                     "dd": round(b["d"] - a["d"], 1),
                                     "x_deg_before": round(a["x_deg"], 2), "y_before": a["y"], "d_before": a["d"]})
                    last_dir = d
            if base_id in st:
                base_prev = st[base_id]
        elif e["dev"] == "lift" and e["fn"] in ("move_by", "move_to"):
            pending_lift = None
            _lift_row(rows, frames, e, e, nxt, before_s, settle_s, after_s)
        elif e["dev"] == "lift" and e["fn"] == "start_move_by":
            pending_lift = e
        elif e["dev"] == "lift" and e["fn"] == "wait_move" and pending_lift is not None:
            # 中间夹着别的动作（比如同时转底座）就不算单一动作
            between = [c for c in cmds if pending_lift["t"] < c["t"] < e["t"] and c["dev"] != "lift"]
            if not between:
                _lift_row(rows, frames, pending_lift, e, nxt, before_s, settle_s, after_s)
            pending_lift = None
    return rows


def _lift_row(rows, frames, start, end, nxt, before_s, settle_s, after_s):
    a = _median_around(frames, start["t"] - before_s, start["t"])
    b = _median_around(frames, end["t_end"] + settle_s, min(nxt, end["t_end"] + settle_s + after_s), last=False)
    if not (a and b):
        return
    cmd = start["args"][0] if start["args"] else None
    rows.append({"t": start["t"], "axis": "lift", "cmd": cmd, "reversed": "",
                 "base_deg_before": "", "dx_deg": round(b["x_deg"] - a["x_deg"], 3), "dy": round(b["y"] - a["y"], 2),
                 "dd": round(b["d"] - a["d"], 1), "x_deg_before": round(a["x_deg"], 2), "y_before": a["y"],
                 "d_before": a["d"]})


def backlash_report(rows):
    """底座：画面变化 / 命令角，按"同向继续"和"反向"分开统计。反向时比值明显小 = 有回差"""
    base = [r for r in rows if r["axis"] == "base" and abs(r["cmd"]) >= 0.5]
    if not base:
        return "没有可用的底座单独转动样本（需要物块一直在画面里）"
    out = ["底座 画面方位变化 / 命令角（理想值 ≈ 灵敏度，真机标定约 −0.6）"]
    for name, sel in (("同向继续", [r for r in base if r["reversed"] == 0]),
                      ("反向", [r for r in base if r["reversed"] == 1])):
        for lo, hi in ((0.5, 3), (3, 6), (6, 90)):
            g = [r["dx_deg"] / r["cmd"] for r in sel if lo <= abs(r["cmd"]) < hi]
            if g:
                out.append(f"  {name:4s} |命令| {lo:>3}~{hi:<3}°  {len(g):3d} 次  比值中位数 {statistics.median(g):+.2f}")
    out.append("  看法：小角度、反向时比值的绝对值明显变小 → 死区 / 回差；差多少就是要补的量")
    return "\n".join(out)


def main(argv):
    if len(argv) < 2 or argv[0] not in ("summary", "pairs", "backlash"):
        print(__doc__)
        return 2
    evs = load(argv[1:])
    if argv[0] == "summary":
        print(summary(evs))
        return 0
    try:
        import config as cfg
        kw = dict(base_id=cfg.BASE_ID, base_center=cfg.BASE_CENTER, us_per_deg=cfg.BASE_US_PER_DEG,
                  base_dir=cfg.BASE_DIR)
    except Exception:
        kw = {}
    rows = pairs(evs, **kw)
    if argv[0] == "backlash":
        print(f"样本对 {len(rows)} 个")
        print(backlash_report(rows))
        return 0
    if rows:
        w = csv.DictWriter(sys.stdout, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    else:
        print("没有样本对（物块要在动作前后都被看到）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
