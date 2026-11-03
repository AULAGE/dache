"""
recorder.py 的离线测试（独立文件，不依赖硬件）：python3 test_recorder.py
1. 记录不改变任务行为：同一个仿真种子，记录 / 不记录的抓取结果逐项相同
2. 记录内容齐全：K230 帧、命令、阶段、抓取结果；断电写一半的行能跳过
3. 样本对 + 回差统计：人为构造有回差的数据，统计能看出"反向时比值变小"
4. 同步标记（任务 12）：默认不发；打开后每条命令前后各发一条、格式 K230 能解析，且不改变抓取结果
5. 时钟对齐（任务 12）：合成的 K230 会话（偏移 + 漂移 + 每帧只查几次串口）→ align_k230_log 拟合误差、
   数据集里每张图对应的命令 / 舵机脉宽 / 升降轴 / 抓取结果都对
"""
import copy
import json
import os
import random
import sys
import tempfile

os.environ.setdefault("GRASP_IGNORE_CALIB", "1")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg                         # noqa: E402
import recorder                              # noqa: E402
from sim import SIM_POSES, make_car, make_sim   # noqa: E402
from task import GraspTask                   # noqa: E402

PASS, FAIL = [], []


def check(name, ok, info=""):
    (PASS if ok else FAIL).append(name)
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   {info}" if info else ""))


def run_episodes(record_path, n=4, seed=7, sync_out=None):
    """无底盘、逐帧 K230 模型，跑 n 次；record_path=None 不记录；sync_out 是 list 时打开同步标记（发到这个 list）"""
    saved = {k: getattr(cfg, k) for k in ("POSES", "USE_CHASSIS")}
    saved_trk = getattr(cfg, "SIM_K230_TRACKER", None)
    cfg.POSES, cfg.USE_CHASSIS, cfg.SIM_K230_TRACKER = SIM_POSES, False, True
    try:
        car = make_car(cfg, seed)
        rows, task, rec = [], None, None
        for i in range(n):
            world, clock, cam, lift, arm, chassis = make_sim(cfg, seed * 1000 + i, 1.0, cfg.R_CAM_MM, car)
            if record_path:
                if rec is None:
                    sync = None
                    if sync_out is not None:
                        sync = recorder.SyncSender(sync_out.append, clock, 1.0)
                    rec = recorder.Recorder(record_path, clock, cam, poll_s=None, sync=sync)
                else:
                    rec.clock = clock
                    if rec.sync is not None:
                        rec.sync.clock = clock
                        rec.sync.last_t = -1e18
                    rec.set_cam(cam)
                    rec._last_frame_t = -1e18     # 新场景的仿真时钟从 0 开始
                lift, arm, chassis = (recorder.RecordProxy(lift, "lift", rec), recorder.RecordProxy(arm, "arm", rec),
                                      recorder.RecordProxy(chassis, "chassis", rec))
            if task is None:
                task = GraspTask(cfg, clock, cam, lift, arm, chassis, log_dir=None, learn=False, verbose=False)
            else:
                task.clock, task.cam, task.lift, task.arm, task.chassis = clock, cam, lift, arm, chassis
                task.base_deg = 0.0
            if rec is not None:
                recorder.hook_task(task, rec)
            rows.append(task.run_once())
        if rec is not None:
            rec.close()
        return rows
    finally:
        for k, v in saved.items():
            setattr(cfg, k, v)
        if saved_trk is None:
            delattr(cfg, "SIM_K230_TRACKER")
        else:
            cfg.SIM_K230_TRACKER = saved_trk


def test_no_behaviour_change(tmp):
    print("1. 记录不改变行为")
    a = run_episodes(None)
    path = os.path.join(tmp, "rec.jsonl")
    b = run_episodes(path)
    check("记录 / 不记录的抓取结果逐项相同", a == b, f"{[r['result'] for r in a]}")
    return path


def test_contents(path):
    print("2. 记录内容")
    evs = recorder.load([path])
    kinds = {e["ev"] for e in evs}
    for k in ("header", "k230", "cmd", "say", "episode", "episode_start"):
        check(f"有 {k} 事件", k in kinds)
    cmds = [e for e in evs if e["ev"] == "cmd"]
    check("命令记录了设备、函数、开始/结束时间", all({"dev", "fn", "t", "t_end"} <= set(e) for e in cmds)
          and all(e["t_end"] >= e["t"] for e in cmds))
    arm_cmds = [e for e in cmds if e["dev"] == "arm"]
    check("机械臂命令带执行后的舵机状态", arm_cmds and all(isinstance(e["state"], dict) and e["state"] for e in arm_cmds))
    eps = [e for e in evs if e["ev"] == "episode"]
    check("每次抓取一条结果", len(eps) == 4, f"{len(eps)} 条")
    with open(path, "a", encoding="utf-8") as f:
        f.write('{"ev": "k230", "t": 1')                 # 模拟断电写了一半
    check("写了一半的行能跳过", len(recorder.load([path])) == len(evs))
    print("   " + recorder.summary(evs).replace("\n", "\n   "))
    rows = recorder.pairs(evs, base_id=cfg.BASE_ID, base_center=cfg.BASE_CENTER,
                          us_per_deg=cfg.BASE_US_PER_DEG, base_dir=cfg.BASE_DIR)
    check("能从仿真记录里抽出动作-响应样本对", len(rows) > 0, f"{len(rows)} 对")


def test_backlash_stats(tmp):
    print("3. 回差统计（人为构造：真实灵敏度 −0.6，回差 2°，死区 0.8°）")
    rng = random.Random(3)
    evs, t, pulse, last = [], 0.0, 1500, 0
    x_deg = 0.0
    evs.append({"ev": "cmd", "t": t, "t_end": t + 0.2, "dev": "arm", "fn": "move",
                "args": [{"6": pulse}], "state": {"6": pulse}})
    t += 1
    for _ in range(300):
        cmd = rng.choice([-1, 1]) * rng.uniform(0.6, 8.0)
        reversed_ = last != 0 and (cmd > 0) != (last > 0)
        eff = abs(cmd) - (2.0 if reversed_ else 0.0)
        eff = max(0.0, eff - 0.8 if eff < 3 else eff)
        real = eff * (1 if cmd > 0 else -1)
        for k in range(5):
            evs.append({"ev": "k230", "t": t - 0.5 + 0.1 * k, "x": 150 * __import__("math").tan(
                __import__("math").radians(x_deg)) + rng.gauss(0, 0.3), "y": 50, "d": 150})
        new_pulse = int(round(pulse + cmd * cfg.BASE_US_PER_DEG * cfg.BASE_DIR))
        evs.append({"ev": "cmd", "t": t, "t_end": t + 0.3, "dev": "arm", "fn": "move",
                    "args": [{"6": new_pulse}], "state": {"6": new_pulse}})
        x_deg += -0.6 * real
        for k in range(5):
            evs.append({"ev": "k230", "t": t + 0.6 + 0.1 * k, "x": 150 * __import__("math").tan(
                __import__("math").radians(x_deg)) + rng.gauss(0, 0.3), "y": 50, "d": 150})
        pulse, last, t = new_pulse, cmd, t + 2.0
    evs.sort(key=lambda e: e["t"])
    rows = recorder.pairs(evs, base_id=6, base_center=cfg.BASE_CENTER, us_per_deg=cfg.BASE_US_PER_DEG,
                          base_dir=cfg.BASE_DIR)
    check("样本对数量正确", len(rows) == 300, f"{len(rows)}")
    fwd = sorted(r["dx_deg"] / r["cmd"] for r in rows if not r["reversed"] and abs(r["cmd"]) >= 3)
    rev = sorted(r["dx_deg"] / r["cmd"] for r in rows if r["reversed"] and abs(r["cmd"]) >= 3)
    mf, mr = fwd[len(fwd) // 2], rev[len(rev) // 2]
    check("同向时比值 ≈ −0.6", abs(mf + 0.6) < 0.05, f"{mf:+.2f}")
    check("反向时比值的绝对值明显更小（看得出回差）", abs(mr) < abs(mf) - 0.1, f"{mr:+.2f}")
    rep = recorder.backlash_report(rows)
    check("backlash 报告能生成", "反向" in rep and "同向继续" in rep)
    print("   " + rep.replace("\n", "\n   "))


def test_sync_marks(tmp, path_plain):
    print("4. 同步标记（任务 12）")
    check("默认不发同步标记（config.K230_SYNC_PORT = None）", cfg.K230_SYNC_PORT is None)
    evs = recorder.load([path_plain])
    check("默认记录里没有 sync 事件、命令里没有 sync 字段",
          not any(e["ev"] == "sync" for e in evs) and not any("sync" in e for e in evs if e["ev"] == "cmd"))
    check("K230_SYNC_PORT / K230_SYNC_PERIOD_S 在 CALIB_KEYS 白名单里",
          {"K230_SYNC_PORT", "K230_SYNC_PERIOD_S"} <= cfg.CALIB_KEYS)
    a = run_episodes(None)
    lines = []
    path = os.path.join(tmp, "rec_sync.jsonl")
    b = run_episodes(path, sync_out=lines)
    check("打开同步标记后抓取结果逐项相同", a == b, f"{[r['result'] for r in b]}")
    evs = recorder.load([path])
    syn = [e for e in evs if e["ev"] == "sync"]
    cmds = [e for e in evs if e["ev"] == "cmd"]
    check("发出去的行数 = 记录的 sync 事件数", len(lines) == len(syn) and len(syn) > 0, f"{len(syn)} 条")
    ok = all(ln == f'SYNC {e["seq"]} {e["pi_t"]}\n'.encode() for ln, e in zip(lines, syn))
    check("每行是 SYNC <序号> <时间>，和 jsonl 里记的一致", ok, lines[0].decode().strip() if lines else "")
    check("每条命令前后各一条（sync = [前, 后]，后 = 前 + 1）",
          cmds and all(e.get("sync") and e["sync"][1] == e["sync"][0] + 1 for e in cmds))
    check("空闲时每秒也发（tick）", any(e["why"] == "tick" for e in syn))
    check("序号从 1 连续递增", [e["seq"] for e in syn] == list(range(1, len(syn) + 1)))
    # 用 K230 程序里的解析函数验证格式（需要 numpy 才能加载 K230 程序的假环境；没有就跳过）
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "k230_sdcard", "tests"))
        import k230_stub
        m = k230_stub.load_main()
        got = [m.parse_sync_line(ln) for ln in lines[:50]]
        check("K230 端 parse_sync_line 能解析树莓派发的每一行",
              all(g == (e["seq"], e["pi_t"]) for g, e in zip(got, syn[:50])))
    except ImportError as e:
        print(f"  （跳过 K230 端解析检查：{e}）")
    # 写失败不影响抓取
    def bad(_):
        raise OSError("串口断了")
    clk = type("C", (), {"now": lambda self: 1.0})()
    snd = recorder.SyncSender(bad, clk, 1.0)
    rec = recorder.Recorder(os.path.join(tmp, "bad.jsonl"), clk, None, poll_s=None, sync=snd)
    seq = rec.sync_mark("pre")
    rec.close()
    e = [x for x in recorder.load([os.path.join(tmp, "bad.jsonl")]) if x["ev"] == "sync"]
    check("同步串口写失败只记 err、不抛异常", seq is None and e and "err" in e[0] and snd.errors == 1)
    # 复用 K230 已经打开的串口
    sent = []
    cam = type("Cam", (), {})()
    cam.ser = type("S", (), {"write": lambda self, b: sent.append(b)})()
    w = recorder.open_sync_writer("/dev/ttyAMA3", 115200, get_cam=lambda: cam, k230_port="/dev/ttyAMA3")
    w(b"SYNC 1 2.0000\n")
    check("同步口 = K230 口时复用 K230 的 serial 对象", sent == [b"SYNC 1 2.0000\n"])


# ------------------------------------------------------------ 5. 时钟对齐
def _write_csv(path, cols, rows):
    import csv
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)


def make_synthetic(tmp, rng, offset_s=-4321.5, drift_ppm=60.0, minutes=20, bad_syncs=3):
    """
    合成一次真机记录：树莓派 jsonl（命令、抓取、同步标记、收到的 K230 帧）+ K230 会话目录。
    K230 时钟 k = (pi − offset) × (1 + drift)；K230 每帧（约 140ms）只在帧内 3 个时刻查串口。
    返回 (jsonl 路径, 会话目录, 真值函数 pi_of_k(ms), 每帧真实树莓派时间 dict)
    """
    import math
    k_of_pi = lambda t: (t - offset_s) * (1 + drift_ppm * 1e-6) * 1000.0        # noqa: E731
    pi_of_k = lambda k: k / 1000.0 / (1 + drift_ppm * 1e-6) + offset_s          # noqa: E731
    T0, T1 = 5000.0, 5000.0 + minutes * 60
    evs = [{"ev": "header", "t": T0, "wall": 0, "argv": []}]
    # 命令：每 2 秒一条，轮流 底座 / 升降轴；每 20 秒一次抓取
    t, pulse, lift, ep = T0 + 1.0, 1500, 10.0, 0
    truth_cmds = []
    while t < T1 - 3:
        if int((t - T0) // 20) != ep:
            ep = int((t - T0) // 20)
            evs.append({"ev": "episode_start", "t": t - 0.01, "episode": ep})
        if rng.random() < 0.5:
            pulse += rng.choice([-30, 30])
            e = {"ev": "cmd", "t": t, "t_end": t + 0.3, "dev": "arm", "fn": "move", "args": [{"6": pulse}],
                 "kw": {}, "ret": None, "state": {str(i): (pulse if i == 6 else 1500 + i) for i in range(1, 7)}}
        else:
            lift = round(lift + rng.choice([-1, 1]), 1)
            e = {"ev": "cmd", "t": t, "t_end": t + 0.8, "dev": "lift", "fn": "move_to", "args": [lift],
                 "kw": {}, "ret": None, "state": lift}
        evs.append(e)
        truth_cmds.append(e)
        if int((t + 2.0 - T0) // 20) != ep:
            evs.append({"ev": "episode", "t": t + 1.5, "row": {"episode": ep, "result": "ok" if ep % 2 else "miss"}})
        t += 2.0
    # 同步标记：每秒一条 + 每条命令前后
    seq = 0
    sync_t = sorted([T0 + 0.5 + k for k in range(int(T1 - T0) - 1)] +
                    [c["t"] - 0.001 for c in truth_cmds] + [c["t_end"] + 0.001 for c in truth_cmds])
    pi_syncs = []
    for st in sync_t:
        seq += 1
        txt = f"{st:.4f}"
        evs.append({"ev": "sync", "t": round(st, 4), "seq": seq, "pi_t": txt, "why": "tick"})
        pi_syncs.append((seq, txt, st))
    # K230 帧循环：拍照 -> 查串口(拍照后) -> KPU -> 查串口 -> ... -> 查串口(帧尾)
    polls, frames, frame_pi = [], [], {}
    t = T0 + 0.05
    fno = 0
    while t < T1:
        dur = 0.14 + rng.uniform(-0.02, 0.02)
        if rng.random() < 0.01:
            dur += 0.4                                  # 偶尔一帧卡很久（比如 gc）
        cap = t
        polls += [cap, cap + 0.45 * dur, cap + 0.85 * dur]
        frame_pi[fno] = cap
        row = {"frame": fno, "t_ms": f"{k_of_pi(cap):.1f}", "img": f"img/{fno // 1000:04d}/{fno:06d}.jpg",
               "n_det": 1, "dets": "", "trk": "lock", "trk_cls": 0, "trk_score": 0.9, "trk_box": "",
               "sent": "", "sx": "", "sy": "", "sd": "", "t_send_ms": ""}
        if fno % 2 == 0:
            ts = cap + 0.8 * dur
            x, y, d = rng.randint(-60, 60), rng.randint(-60, 60), rng.randint(120, 260)
            row.update(sent="pos", sx=x, sy=y, sd=d, t_send_ms=f"{k_of_pi(ts):.1f}")
            # 树莓派收到：传输约 0.9ms + reader 读串口等待 0~50ms
            evs.append({"ev": "k230", "t": round(ts + 0.00087 + rng.uniform(0, 0.05), 4), "x": x, "y": y, "d": d})
        frames.append(row)
        t += dur
        fno += 1
    import bisect
    srows = []
    for (sq, txt, st) in pi_syncs:
        arr = st + len(f"SYNC {sq} {txt}\n") * 10 / 115200
        i = bisect.bisect_left(polls, arr)
        if i == 0 or i >= len(polls):
            continue
        srows.append({"seq": sq, "pi_t": txt, "k230_ms_prev": f"{k_of_pi(polls[i - 1]):.1f}",
                      "k230_ms": f"{k_of_pi(polls[i]):.1f}", "frame": 0})
    for k in range(bad_syncs):                             # 几条坏数据：K230 那边时刻错了 0.5 秒
        srows[10 + 37 * k]["k230_ms"] = f'{float(srows[10 + 37 * k]["k230_ms"]) + 500:.1f}'
        srows[10 + 37 * k]["k230_ms_prev"] = f'{float(srows[10 + 37 * k]["k230_ms_prev"]) + 500:.1f}'
    evs.append({"ev": "k230", "t": T0 + 3.0, "x": 999, "y": 999, "d": 999})
    evs.sort(key=lambda e: e["t"])
    rec_path = os.path.join(tmp, "record_synth.jsonl")
    with open(rec_path, "w", encoding="utf-8") as f:
        for e in evs:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
    sess = os.path.join(tmp, "datalog", "s0007")
    os.makedirs(sess)
    _write_csv(os.path.join(sess, "frames.csv"), list(frames[0].keys()), frames)
    _write_csv(os.path.join(sess, "sync.csv"), ["seq", "pi_t", "k230_ms_prev", "k230_ms", "frame"], srows)
    with open(os.path.join(sess, "info.txt"), "w", encoding="utf-8") as f:
        f.write("session=s0007\nimg_w=320\nimg_h=180\n")
    # 另一个不相干的会话（序号相同、时间不同）：不能配上
    other = os.path.join(tmp, "datalog", "s0008")
    os.makedirs(other)
    _write_csv(os.path.join(other, "sync.csv"), ["seq", "pi_t", "k230_ms_prev", "k230_ms", "frame"],
               [{"seq": r["seq"], "pi_t": "1.0000", "k230_ms_prev": 0, "k230_ms": 1, "frame": 0} for r in srows[:5]])
    _write_csv(os.path.join(other, "frames.csv"), list(frames[0].keys()), frames[:3])
    return rec_path, sess, other, pi_of_k, frame_pi, truth_cmds


def test_alignment(tmp):
    print("5. 时钟对齐（任务 12，合成数据：偏移 −4321.5s、漂移 +60ppm、每帧查 3 次串口、3 条坏同步）")
    import bisect
    from research import align_k230_log as al
    rng = random.Random(12)
    rec_path, sess, other, pi_of_k, frame_pi, truth_cmds = make_synthetic(tmp, rng)
    logs = []
    rows, reps = al.align([rec_path], [other, sess], log=logs.append)
    for ln in logs:
        print("   " + ln.replace("\n", "\n   "))
    rep = reps[0]
    f = rep["fit"]
    check("配对到正确的 K230 会话", rep["session"] == "s0007")
    check("坏同步标记被去掉", f["n"] - f["used"] >= 3, f'{f["n"]} → {f["used"]}')
    check("漂移估计误差 < 5ppm", abs(f["drift_ppm"] - (1 / (1 + 60e-6) - 1) * 1e6) < 5, f'{f["drift_ppm"]:+.1f}ppm')
    errs = [abs(r["pi_t"] - frame_pi[r["frame"]]) * 1000 for r in rows]
    errs.sort()
    check("每张图的树莓派时间误差 < 3ms（最大）", errs and errs[-1] < 3.0,
          f"中位数 {errs[len(errs) // 2]:.2f}ms 最大 {errs[-1]:.2f}ms，{len(rows)} 张")
    c = rep["check"]
    check("自检：位置帧几乎全部对上", c["matched"] >= 0.98 * c["sent"], f'{c["matched"]}/{c["sent"]}')
    check("自检残差 ≈ 传输 + 读串口等待（中位数 0~50ms）", 0 < c["median_ms"] < 50, f'{c["median_ms"]:+.1f}ms')
    # 数据集内容：拿真实时间直接查，和工具给出的比
    ct = [e["t"] for e in truth_cmds]
    bad = 0
    for r in rows[::37]:
        t = frame_pi[r["frame"]]
        i = bisect.bisect_right(ct, t) - 1
        if i < 0:
            continue
        prev = truth_cmds[i]
        arm = [e for e in truth_cmds[:i + 1] if e["dev"] == "arm"]
        lift = [e for e in truth_cmds[:i + 1] if e["dev"] == "lift" and e["t_end"] <= t]
        exp_s6 = arm[-1]["state"]["6"] if arm else ""
        exp_lift = lift[-1]["state"] if lift else ""
        near = abs(t - prev["t"]) < 0.005 or abs(t - prev["t_end"]) < 0.005
        if not near and (r["prev_cmd"] != f'{prev["dev"]}.{prev["fn"]}' or r["s6"] != exp_s6
                         or r["lift_cm"] != exp_lift or r["in_cmd"] != int(t <= prev["t_end"])):
            bad += 1
    check("每张图之前的命令、舵机脉宽、升降轴高度、是否在动作中都对", bad == 0, f"错 {bad} 行")
    res = {r["result"] for r in rows if r["episode"] != ""}
    check("每张图标了第几次抓取和结果", res == {"ok", "miss"}, str(res))
    out = os.path.join(tmp, "dataset.csv")
    rc = al.main(["--record", rec_path, "--k230", sess, "-o", out])
    import csv
    with open(out, encoding="utf-8") as fh:
        got = list(csv.DictReader(fh))
    check("命令行能写出 dataset.csv，字段齐全", rc == 0 and len(got) == len(rows)
          and set(al.DATASET_COLS) == set(got[0].keys()))
    # 只有一个同步标记：只估偏移
    one = al.fit_clock([(100.0, 50.0, 0.01, 1)])
    check("只有一个同步标记时只估偏移（漂移 0）", one["b"] == 1.0 and abs(one["a"] - 50.0) < 1e-9)
    # 没配上：不崩，报原因
    lg = []
    rows2, reps2 = al.align([os.path.join(tmp, "rec.jsonl")], [sess], log=lg.append)
    check("没有同步标记的记录：不崩，说明原因", rows2 == [] and reps2[0]["fit"] is None and "没有和任何" in lg[0])


def main():
    tmp = tempfile.mkdtemp(prefix="rec_test_")
    path = test_no_behaviour_change(tmp)
    test_contents(path)
    test_backlash_stats(tmp)
    test_sync_marks(tmp, path)
    test_alignment(tmp)
    print(f"通过 {len(PASS)} 项，失败 {len(FAIL)} 项")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
