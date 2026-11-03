"""
recorder.py 的离线测试（独立文件，不依赖硬件）：python3 test_recorder.py
1. 记录不改变任务行为：同一个仿真种子，记录 / 不记录的抓取结果逐项相同
2. 记录内容齐全：K230 帧、命令、阶段、抓取结果；断电写一半的行能跳过
3. 样本对 + 回差统计：人为构造有回差的数据，统计能看出"反向时比值变小"
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


def run_episodes(record_path, n=4, seed=7):
    """无底盘、逐帧 K230 模型，跑 n 次；record_path=None 不记录"""
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
                    rec = recorder.Recorder(record_path, clock, cam, poll_s=None)
                else:
                    rec.clock = clock
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


def main():
    tmp = tempfile.mkdtemp(prefix="rec_test_")
    path = test_no_behaviour_change(tmp)
    test_contents(path)
    test_backlash_stats(tmp)
    print(f"通过 {len(PASS)} 项，失败 {len(FAIL)} 项")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
