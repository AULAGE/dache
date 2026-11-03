#!/usr/bin/env python3
"""
任务 11 的全部仿真实验，输出 Markdown 表格（所有结论都是仿真，未在真机验证）。

  python3 research/run_experiments.py --seed 0              # 全部实验（约 1~2 分钟）
  python3 research/run_experiments.py --seed 0 --quick      # 小规模（测试用）
  python3 research/run_experiments.py --only assoc,ident    # 只跑某几项：assoc ident comp outcome
  python3 research/run_experiments.py --out 结果.md         # 同时写到文件
  python3 research/run_experiments.py --png 目录            # 有 matplotlib 时顺便画收敛曲线（没有就跳过）
"""
import argparse
import math
import os
import random
import statistics
import sys
import time
from collections import Counter

_HERE = os.path.dirname(os.path.abspath(__file__))
_GRASP = os.path.dirname(_HERE)
if _GRASP not in sys.path:
    sys.path.insert(0, _GRASP)

from research.sim_world import Car, World, CAR_RANGES  # noqa: E402
from research.association import CarState, run_episode  # noqa: E402
from research.hysteresis_id import HysteresisBank, GRID  # noqa: E402
from research import outcome_calib as oc  # noqa: E402
import config as cfg  # noqa: E402
from servo import build_axes  # noqa: E402

OUT = []


def emit(s=""):
    print(s)
    OUT.append(s)


def wilson(k, n, z=1.96):
    """Wilson 95% 置信区间"""
    if n == 0:
        return 0.0, 0.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def pct(k, n, ci=True):
    if n == 0:
        return "—"
    s = f"{100.0 * k / n:.1f}%"
    if ci:
        lo, hi = wilson(k, n)
        s += f" [{100 * lo:.0f}, {100 * hi:.0f}]"
    return s


def mean(v):
    return statistics.mean(v) if v else float("nan")


def table(head, rows):
    emit("| " + " | ".join(head) + " |")
    emit("|" + "|".join("---" for _ in head) + "|")
    for r in rows:
        emit("| " + " | ".join(str(x) for x in r) + " |")
    emit()


# ====================================================================== 1 关联
ASSOC_METHODS = (("ghost", "现有 ghost 规则（K230 单目标）"),
                 ("p_k230", "概率模型（K230 单目标，只能停）"),
                 ("p_raw", "概率模型（多框 + 换目标 + 主动探测）"),
                 ("p_raw_nh", "消融：同上但去掉迟滞模型（b=δ=0，只学 J）"))


def exp_association(seed, cars, episodes, hyst_scale=1.0):
    if hyst_scale == 1.0:
        emit("## 1 数据关联：ghost 规则 vs 概率模型（仿真）")
    else:
        emit(f"### 1b 底座回差、死区放大 {hyst_scale:g} 倍（H3 的\"人为加大回差\"）")
    emit()
    emit(f"{cars} 台随机车 × {episodes} 回合；三种方法在**同样的场景**上配对比较（场景、帧噪声按回合播种）。"
         "每回合：原地看一眼（K230 选目标）→ 对准（升降 + 底座，同一个 task.py 控制律）。"
         "场景：真目标 + 爪子脚（每台车 10~50% 回合处于可见状态）+ 0~2 个干扰物（静止方块或乱动的手）。")
    emit()
    stats = {}
    t0 = time.time()
    for m, _ in ASSOC_METHODS:
        C, steps, firsts, times = Counter(), [], [], []
        by_self = Counter()
        for ci in range(cars):
            car = Car(seed * 1000 + ci, hyst_scale)
            w = World(car, seed * 1000 + ci)
            w.preload()
            cs = CarState(no_hyst=m.endswith("_nh"))
            for ep in range(episodes):
                r = run_episode(w, m.replace("_nh", ""), cs, episode_seed=(seed * 1000 + ci) * 100003 + ep)
                k = r["kind"]
                good = k in ("target", "block")
                if r["result"] == "commit":
                    cat = "ok" if good else "wrong"
                elif r["result"] == "abort":
                    cat = "abort_ok" if not good else "abort_false"
                else:
                    cat = "stuck_wrong" if (k is not None and not good) else "lost"
                C[cat] += 1
                if cat == "ok" and k == "block":
                    C["ok_block"] += 1
                self_on = w.objs[1].active
                by_self[(self_on, cat == "ok")] += 1
                steps.append(r["steps"])
                if r["first"] is not None:
                    firsts.append(r["first"])
                times.append(r["time"])
        stats[m] = (C, steps, firsts, by_self)
    n = cars * episodes
    rows = []
    for m, name in ASSOC_METHODS:
        C, steps, firsts, by_self = stats[m]
        rows.append([name, pct(C["ok"], n), pct(C["wrong"] + C["stuck_wrong"], n, False),
                     pct(C["abort_ok"], n, False), pct(C["abort_false"], n, False), pct(C["lost"], n, False),
                     pct(C["ok_block"], n, False), f"{mean(firsts):.2f}", f"{mean(steps):.2f}"])
    table(["方法", "对准到方块并确认 [95%CI]", "误锁（自身/手）", "正确中止", "误报中止（其实是方块）",
           "丢失/超步", "其中是另一方块", "首次正确锁定步数", "平均步数"], rows)
    rows = []
    for m, name in ASSOC_METHODS:
        bs = stats[m][3]
        on = bs[(True, True)] + bs[(True, False)]
        off = bs[(False, True)] + bs[(False, False)]
        rows.append([name, pct(bs[(True, True)], on), pct(bs[(False, True)], off)])
    table(["方法", "成功率：爪子脚可见的回合", "成功率：爪子脚不可见的回合"], rows)
    emit("说明：误锁 = 最后确认 / 一直跟着的是爪子脚或手；正确中止 = 停下时锁的确实不是方块"
         "（task.py 里整回合放弃）；首次正确锁定步数 = 成功回合里从第几步起一直锁在方块上。"
         f"（用时 {time.time() - t0:.0f}s）")
    emit()
    return stats


# ====================================================================== 2 辨识
def _measure_target(w, n=3):
    _, win = w.window(n)
    xs = [d for f in win for d in f.dets if d.kind == "target"]
    if len(xs) < 2:
        return None
    return statistics.median(d.x_deg for d in xs), statistics.median(d.y for d in xs)


def _excite(r, axis):
    if axis == "x":
        return r.choice((-1, 1)) * r.uniform(0.3, 6.0)
    return r.choice((-1, 1)) * r.uniform(0.03, 1.0)


def exp_identification(seed, cars, reps, checkpoints=(5, 10, 20, 40, 80)):
    emit("## 2 迟滞辨识：(J, b, δ) 估计误差随步数收敛（仿真）")
    emit()
    emit(f"{cars} 台车 × 每台 {reps} 段随机激励（每步随机方向、底座 0.3~6°、升降 0.03~1cm），"
         "只用真目标的检测（假设关联已解决），每步取 3 帧中值。网格：底座 b 0~6° × δ 0~4°（61×41），"
         "升降 b 0~0.5cm × δ 0~0.25cm（26×26）。真实参数范围（估计）：底座 b %.1f~%.1f°、δ %.1f~%.1f°；"
         "升降 b %.2f~%.2fcm、δ %.2f~%.2fcm。" % (
             *CAR_RANGES["x"]["b"], *CAR_RANGES["x"]["delta"], *CAR_RANGES["y"]["b"], *CAR_RANGES["y"]["delta"]))
    emit()
    curves = {}
    for axis in ("x", "y"):
        acc = {k: {"b": [], "delta": [], "J": [], "Jad": [], "o": [], "o_open": [], "bsd": []} for k in checkpoints}
        for ci in range(cars):
            for rep in range(reps):
                s = seed * 1000 + ci
                car = Car(s)
                w = World(car, s * 31 + rep)
                w.preload()
                w.new_episode(n_dist=0, self_active=False, target_view=(0.0, 0.0, 180.0), episode_seed=s * 7 + rep)
                bank = (HysteresisBank.for_axis("x", 61, 41) if axis == "x" else HysteresisBank.for_axis("y", 26, 26))
                ad = build_axes(cfg, ["x_deg" if axis == "x" else "y"])["x_deg" if axis == "x" else "y"]
                r = random.Random(s * 17 + rep)
                z = _measure_target(w)
                k_ax = 0 if axis == "x" else 1
                for k in range(1, max(checkpoints) + 1):
                    u = _excite(r, axis)
                    c = w.act[axis].cmd + u
                    if abs(c) > (30 if axis == "x" else 3):        # 别走出范围：往回走
                        u, c = -u, w.act[axis].cmd - u
                    q = GRID[axis]["quant"]
                    c = round(c / q) * q
                    u = c - w.act[axis].cmd
                    w.command({axis: [c]})
                    z2 = _measure_target(w)
                    dz = None if (z is None or z2 is None) else z2[k_ax] - z[k_ax]
                    bank.apply([c], dz)
                    if dz is not None:
                        ad.learn(u, z[k_ax], z2[k_ax])
                    z = z2
                    if k in checkpoints:
                        e = bank.estimate()
                        a = acc[k]
                        a["b"].append(abs(e["b"][0] - car.b[axis]))
                        a["bsd"].append(e["b"][1])
                        a["delta"].append(abs(e["delta"][0] - car.delta[axis]))
                        a["J"].append(abs(e["J"][0] - car.J[axis]) / abs(car.J[axis]))
                        a["Jad"].append(abs(ad.j - car.J[axis]) / abs(car.J[axis]))
                        wts = bank.weights()
                        o_hat = sum(wi * oi for wi, oi in zip(wts, bank.o))
                        true_o = w.act[axis].base_out
                        a["o"].append(abs(o_hat - true_o))
                        a["o_open"].append(abs(w.act[axis].cmd - true_o))
        u = "°" if axis == "x" else "cm"
        emit(f"### {'底座' if axis == 'x' else '升降轴'}（{u}）")
        emit()
        rows = []
        for k in checkpoints:
            a = acc[k]
            rows.append([k, f"{mean(a['b']):.3f}", f"{mean(a['bsd']):.3f}", f"{mean(a['delta']):.3f}",
                         f"{100 * mean(a['J']):.1f}%", f"{100 * mean(a['Jad']):.1f}%",
                         f"{mean(a['o']):.3f}", f"{mean(a['o_open']):.3f}"])
        table(["步数", f"b 误差（{u}）", f"b 后验 sd", f"δ 误差（{u}）", "J 相对误差：假设库",
               "J 相对误差：task.py 自适应", f"输出位置估计误差（{u}）", f"开环：认为输出=命令（{u}）"], rows)
        curves[axis] = acc
    emit("\"输出位置估计误差\"就是 H1 的雏形：不装编码器，只靠命令历史 + 画面位移，后验对舵机真实输出角的估计"
         "（对比开环把命令当成实际位置）。")
    emit()
    return curves


# ====================================================================== 3 补偿
COMP_METHODS = (("base", "基线：task.py 自适应 P 控制"),
                ("online", "逆迟滞补偿（在线辨识，从先验开始）"),
                ("pre", "逆迟滞补偿（先激励辨识 40 步）"),
                ("oracle", "逆迟滞补偿（真实参数，上限）"))


def comp_episode(w, mode, st, tol, max_steps, ep_seed, rng):
    if w.act["x"].cmd != 0.0:                     # 和 task.py 一样每回合先回正（回正这一步也会改变间隙状态）
        w.command({"x": [0.0]})
        if "bank" in st:
            st["bank"].apply([0.0], None)
    x0 = rng.choice((-1, 1)) * rng.uniform(2.0, 20.0)
    w.new_episode(n_dist=0, self_active=False, target_view=(x0, 0.0, rng.uniform(140, 230)), episode_seed=ep_seed)
    q = GRID["x"]["quant"]
    z = _measure_target(w)
    steps = 0
    while z is not None and steps < max_steps:
        e = z[0]
        if abs(e) <= tol:
            break
        if mode == "base":
            u = st["axis"].command(e)
            if not u:
                break
            cmds = [round((w.act["x"].cmd + u) / q) * q]
        else:
            cmds = st["bank"].plan(-e, quant=q)
            if not cmds:
                break
        c_prev = w.act["x"].cmd
        w.command({"x": cmds})
        steps += 1
        z2 = _measure_target(w)
        if mode == "base":
            if z2 is not None:
                st["axis"].learn(cmds[-1] - c_prev, e, z2[0])
        else:
            dz = None if (z2 is None or mode == "oracle") else z2[0] - z[0]
            st["bank"].apply(cmds, dz)
        z = z2
    true_x = w.view(w.objs[0], w.t)[0]
    return steps, abs(true_x), z is not None and abs(z[0]) <= tol


def exp_compensation(seed, cars, episodes, tols=(1.5, 0.5), max_steps=25):
    emit("## 3 逆迟滞补偿：底座对准（仿真）")
    emit()
    emit(f"{cars} 台车 × {episodes} 回合，只看底座（方位角）对准；初始误差 ±2~20°，"
         f"容差 {tols[0]}°（现在 config.TOL）和 {tols[1]}°（更严）。执行器状态跨回合延续（历史有影响）。"
         "最终误差 = 对准结束时物块真实方位角（不含测量噪声）。")
    emit()
    rows = []
    for tol in tols:
        for mode, name in COMP_METHODS:
            steps, errs, ok = [], [], 0
            for ci in range(cars):
                s = seed * 1000 + ci
                car = Car(s)
                w = World(car, s * 13 + 5)
                w.preload()
                st = {}
                if mode == "base":
                    st["axis"] = build_axes(cfg, ["x_deg"])["x_deg"]
                elif mode == "oracle":
                    a = w.act["x"]
                    st["bank"] = HysteresisBank.oracle(car.b["x"], car.delta["x"], car.J["x"],
                                                       a.base_motor, a.base_out, GRID["x"]["sigma"])
                else:
                    st["bank"] = HysteresisBank.for_axis("x")
                    if mode == "pre":
                        r = random.Random(s * 3 + 1)
                        w.new_episode(n_dist=0, self_active=False, target_view=(0.0, 0.0, 180.0))
                        z = _measure_target(w)
                        for _ in range(40):
                            u = _excite(r, "x")
                            c = w.act["x"].cmd + u
                            if abs(c) > 30:
                                c = w.act["x"].cmd - u
                            c = round(c / GRID["x"]["quant"]) * GRID["x"]["quant"]
                            w.command({"x": [c]})
                            z2 = _measure_target(w)
                            st["bank"].apply([c], None if (z is None or z2 is None) else z2[0] - z[0])
                            z = z2
                rng = random.Random(s * 5 + 2)
                for ep in range(episodes):
                    n, err, good = comp_episode(w, mode, st, tol, max_steps, s * 100003 + ep, rng)
                    steps.append(n)
                    errs.append(err)
                    ok += good
            errs_s = sorted(errs)
            rows.append([f"{tol}°", name, f"{mean(steps):.2f}", f"{mean(errs):.2f}°",
                         f"{errs_s[int(0.95 * (len(errs_s) - 1))]:.2f}°", pct(ok, len(errs), False)])
    table(["容差", "方法", "平均对准步数", "最终角度误差 均值", "95 分位", "进入容差的比例"], rows)


# ====================================================================== 4 结果监督标定
def exp_outcome(seed, cars, reps, n=40, checkpoints=(5, 10, 20, 30, 40)):
    emit("## 4 结果监督的残差标定（仿真）")
    emit()
    emit(f"{cars} 台车（真实残差 r ~ U(−6, 6) mm）× 每台 {reps} 次独立标定，每次 {n} 次抓取。"
         "成功概率 Φ((w/2 − |e − r|)/σ)，模型取 w/2 = 8mm、σ = 3mm（估计）；先验 r ~ N(0, 6²)。"
         "最后一列：标定结束后瞄准 r̂ 的真实成功率。")
    emit()
    settings = (("模型正确（w/2=8, σ=3）", None, None),
                ("模型设错：真实 w/2=6.5, σ=4.5", 6.5, 4.5))
    curves = {}
    for title, thw, ts in settings:
        emit(f"### {title}")
        emit()
        rows = []
        for strat in oc.STRATEGIES:
            acc = {k: [] for k in checkpoints}
            succ, fin = [], []
            for ci in range(cars):
                rr = random.Random(seed * 7919 + ci)
                r_true = rr.uniform(-6, 6)
                for rep in range(reps):
                    rng = random.Random((seed * 1000 + ci) * 1009 + rep)
                    res = oc.run_calibration(r_true, strat, n, rng, true_half_w=thw, true_sigma=ts,
                                             checkpoints=checkpoints)
                    for k in checkpoints:
                        acc[k].append(res["err"][k])
                    succ.append(res["succ"] / n)
                    fin.append(res["final_p"])
            curves[(title, strat)] = acc
            rows.append([oc.STRATEGY_NAME[strat]] + [f"{mean(acc[k]):.2f}" for k in checkpoints]
                        + [f"{100 * mean(succ):.0f}%", f"{100 * mean(fin):.1f}%"])
        table(["策略"] + [f"{k} 次后 r 误差 mm" for k in checkpoints] + ["标定期间成功率", "标定后成功率"], rows)
    return curves


# ====================================================================== 图（可选）
def save_png(path, ident, outcome):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        emit("（没有 matplotlib，跳过画图）")
        return
    os.makedirs(path, exist_ok=True)
    if ident:
        fig, ax = plt.subplots(1, 2, figsize=(9, 3.5))
        for i, axis in enumerate(("x", "y")):
            acc = ident[axis]
            ks = sorted(acc)
            for key, lab in (("b", "b error"), ("delta", "delta error"), ("o", "output pos")):
                ax[i].plot(ks, [mean(acc[k][key]) for k in ks], marker="o", label=lab)
            ax[i].set_xscale("log")
            ax[i].set_title("底座 (°)" if axis == "x" else "升降轴 (cm)")
            ax[i].legend()
        fig.tight_layout()
        fig.savefig(os.path.join(path, "ident.png"), dpi=120)
    if outcome:
        fig, ax = plt.subplots(figsize=(5, 3.5))
        for (title, strat), acc in outcome.items():
            if "正确" not in title:
                continue
            ks = sorted(acc)
            ax.plot(ks, [mean(acc[k]) for k in ks], marker="o", label=oc.STRATEGY_NAME[strat])
        ax.set_xlabel("抓取次数")
        ax.set_ylabel("|r̂−r| mm")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(path, "outcome.png"), dpi=120)
    emit(f"（图已保存到 {path}）")


def main(argv=None):
    ap = argparse.ArgumentParser(description="任务 11 仿真实验")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cars", type=int, default=10)
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--quick", action="store_true", help="小规模（2 台车、少量回合），测试用")
    ap.add_argument("--only", default="assoc,ident,comp,outcome")
    ap.add_argument("--out", default=None)
    ap.add_argument("--png", default=None)
    a = ap.parse_args(argv)
    OUT.clear()
    only = set(a.only.split(","))
    cars, eps = (2, 12) if a.quick else (a.cars, a.episodes)
    t0 = time.time()
    emit(f"# 任务 11 仿真实验结果（seed={a.seed}，**仿真，未在真机验证**）")
    emit()
    if "assoc" in only:
        exp_association(a.seed, cars, eps)
        exp_association(a.seed, cars, max(eps // 2, 5), hyst_scale=2.0)
    ident = outcome = None
    if "ident" in only:
        ident = exp_identification(a.seed, cars, 1 if a.quick else 3,
                                   (5, 10) if a.quick else (5, 10, 20, 40, 80))
    if "comp" in only:
        exp_compensation(a.seed, cars, eps // 2 if not a.quick else eps)
    if "outcome" in only:
        outcome = exp_outcome(a.seed, cars, 2 if a.quick else 20, 10 if a.quick else 40,
                              (5, 10) if a.quick else (5, 10, 20, 30, 40))
    if a.png:
        save_png(a.png, ident, outcome)
    emit(f"总用时 {time.time() - t0:.0f}s")
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write("\n".join(OUT) + "\n")
    return OUT


if __name__ == "__main__":
    main()
