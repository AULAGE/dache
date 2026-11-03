#!/usr/bin/env python3
"""
训练 + 评估（任务 14，合成数据）。CPU 能跑；有 GPU 自动用。

  python3 research/worldmodel/train.py --quick        10 分钟内：小数据、每个变体训练约 2 分钟，输出指标
  python3 research/worldmodel/train.py                完整：约 1~1.5 小时（4 核 CPU）
  python3 research/worldmodel/train.py --variants full,cmdonly --minutes 20

输出（research/worldmodel/results/）：
  train_<模式>.log      训练日志
  results_<模式>.md     H1 / H2 / 消融表
  results_<模式>.json   同上（机器可读）
  <模式>_<变体>.pt      模型权重（每个约 0.2MB）
  slots_<模式>_<变体>.png  3 条测试序列：输入 / 真实掩膜 / 槽位分割（越暗 = 越像"自身"）
  数据集按种子生成，不存盘（几秒钟就能重新生成）
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
_GRASP = os.path.dirname(os.path.dirname(_HERE))
if _GRASP not in sys.path:
    sys.path.insert(0, _GRASP)

from research.worldmodel import eval as ev  # noqa: E402
from research.worldmodel.dataset import make_dataset  # noqa: E402
from research.worldmodel.model import VARIANTS, WorldModel  # noqa: E402

RESULTS = os.path.join(_HERE, "results")
NAMES = {"full": "完整模型（GRU 命令历史 + 画面，分离先验）", "noprior": "无分离先验（槽位自由移动）",
         "nogru": "无 GRU（只用当前命令 + 当前画面）", "cmdonly": "GRU 只看命令历史（不看画面）"}


def log(f, *a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    f.write(s + "\n")
    f.flush()


def train_one(variant, data, tr, args, device, f):
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    model = WorldModel(variant, k=args.slots, d=args.dim, hdim=args.hdim).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    n_par = sum(p.numel() for p in model.parameters())
    log(f, f"--- {variant}：{NAMES[variant]}，参数 {n_par / 1e3:.0f}k")
    rng = np.random.default_rng(args.seed)
    t0 = time.time()
    step = 0
    hist = []
    while time.time() - t0 < args.minutes * 60 and step < args.max_steps:
        j = rng.choice(tr, args.batch, replace=False)
        s0 = rng.integers(0, data["u"].shape[1] - args.T + 1)
        obs = torch.from_numpy(data["obs"][j, s0:s0 + args.T + 1]).float().div(255).permute(0, 1, 4, 2, 3).to(device)
        u = torch.from_numpy(data["u"][j, s0:s0 + args.T]).to(device)
        lr = args.lr * min(1.0, (step + 1) / args.warmup)
        for g in opt.param_groups:
            g["lr"] = lr
        out = model(obs, u)
        loss = out["rec"] + args.pred_w * out["pred"] + args.sep_w * out["sep"]
        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        opt.step()
        hist.append([out["rec"].item(), out["pred"].item(), out["sep"].item() if torch.is_tensor(out["sep"]) else 0.0])
        step += 1
        if step % args.log_every == 0:
            m = np.mean(hist[-args.log_every:], 0)
            log(f, f"  {variant} 步 {step:5d}  {time.time() - t0:6.0f}s  重建 {m[0]:.5f}  预测 {m[1]:.5f}  分离 {m[2]:.5f}")
    log(f, f"  {variant} 共 {step} 步，{time.time() - t0:.0f}s")
    return model, step


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="10 分钟内跑完（小数据、短训练）")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-train", type=int, default=None)
    ap.add_argument("--n-test", type=int, default=None)
    ap.add_argument("--seq-len", type=int, default=24, help="每条序列的动作数")
    ap.add_argument("--T", type=int, default=None, help="训练时截取的动作数")
    ap.add_argument("--minutes", type=float, default=None, help="每个变体的训练时间（分钟）")
    ap.add_argument("--max-steps", type=int, default=10 ** 9)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--slots", type=int, default=4)
    ap.add_argument("--dim", type=int, default=32)
    ap.add_argument("--hdim", type=int, default=64)
    ap.add_argument("--lr", type=float, default=4e-4)
    ap.add_argument("--warmup", type=int, default=200)
    ap.add_argument("--pred-w", type=float, default=1.0)
    ap.add_argument("--sep-w", type=float, default=0.1)
    ap.add_argument("--log-every", type=int, default=50)
    ap.add_argument("--threads", type=int, default=0)
    a = ap.parse_args()
    mode = "quick" if a.quick else "full"
    a.n_train = a.n_train or (160 if a.quick else 1200)
    a.n_test = a.n_test or (40 if a.quick else 200)
    a.T = a.T or (8 if a.quick else 12)
    if a.quick:
        a.seq_len = min(a.seq_len, 16)
    a.minutes = a.minutes if a.minutes is not None else (1.8 if a.quick else 15.0)
    if a.threads:
        torch.set_num_threads(a.threads)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    os.makedirs(RESULTS, exist_ok=True)
    f = open(os.path.join(RESULTS, f"train_{mode}.log"), "w", encoding="utf-8")
    log(f, f"任务 14 世界模型（合成数据）  模式 {mode}  设备 {device}  torch {torch.__version__}  线程 {torch.get_num_threads()}")
    log(f, "参数：", json.dumps({k: v for k, v in vars(a).items()}, ensure_ascii=False))
    t0 = time.time()
    data = make_dataset(a.n_train + a.n_test, a.seq_len, a.seed)
    tr = np.arange(a.n_train)
    te = np.arange(a.n_train, a.n_train + a.n_test)
    log(f, f"数据：训练 {a.n_train} 条 × {a.seq_len} 步，测试 {a.n_test} 条（{time.time() - t0:.0f}s）；"
           f"底座回差 {data['b'][:, 0].min():.2f}~{data['b'][:, 0].max():.2f}°")

    probe_tr = tr[: min(len(tr), 4 * a.n_test)]
    feats_tr, feats_te, h2s, steps = {}, {}, {}, {}
    if "full" in a.variants.split(","):
        # 对照：和完整模型同样的初始权重、不训练。随机 GRU 本身就能"记住"命令历史（储备池），
        # 探针在它上面能解出多少，就是"训练学到的"要减掉的部分
        torch.manual_seed(a.seed)
        m0 = WorldModel("full", k=a.slots, d=a.dim, hdim=a.hdim).to(device)
        name0 = "对照：未训练的同结构模型（随机权重）"
        feats_tr[name0] = ev.collect(m0, data, probe_tr, device=device)["h"]
        feats_te[name0] = ev.collect(m0, data, te, device=device)["h"]
    for v in a.variants.split(","):
        model, steps[v] = train_one(v, data, tr, a, device, f)
        torch.save(model.state_dict(), os.path.join(RESULTS, f"{mode}_{v}.pt"))
        o_tr = ev.collect(model, data, probe_tr, device=device)
        o_te = ev.collect(model, data, te, device=device)
        feats_tr[NAMES[v]], feats_te[NAMES[v]] = o_tr["h"], o_te["h"]
        h2s[v] = ev.h2(o_te, data, te, v)
        ev.save_slots(os.path.join(RESULTS, f"slots_{mode}_{v}.png"), model, data, te[:3], device=device)
        log(f, f"  H2 {v}：", json.dumps({k: round(x, 3) for k, x in h2s[v].items()}, ensure_ascii=False))
    h1 = ev.h1(feats_tr, feats_te, data, probe_tr, te)

    lines = [f"# 任务 14 结果（{mode}，**合成数据**，种子 {a.seed}）", "",
             f"训练 {a.n_train} 条序列 × {a.seq_len} 步，测试 {a.n_test} 条；每个变体训练 {a.minutes} 分钟"
             f"（{', '.join(f'{v} {s} 步' for v, s in steps.items())}），设备 {device}。", "",
             "## H1 视觉本体感知：线性探针从隐状态解码真实舵机角（测试集平均绝对误差）", "",
             "| 方法 | 底座 ° | 升降 cm | 底座：回差 " + " | ".join(f"{lo:g}~{hi:g}°" if hi < 50 else f"≥{lo:g}°"
                                                       for lo, hi in ev.B_BINS) + " | 回差方向识别率 |",
             "|---|---|---|" + "---|" * len(ev.B_BINS) + "---|"]
    for name, r in h1.items():
        lines.append(f"| {name} | {r['x']:.3f} | {r['y']:.4f} | " + " | ".join(f"{x:.3f}" for x in r["x_bins"])
                     + f" | {r['dir_acc']:.1%} |")
    lines += ["", "## H2 自身 / 世界分离：槽位掩膜和真实掩膜的 IoU（测试集）", "",
              "| 变体 | 爪子最佳 IoU | 物块最佳 IoU | 自动判自身（按 w）的爪子 IoU | 判别正确率（按 w） | "
              "自动判自身（按位移）的爪子 IoU | 判别正确率（按位移） |", "|---|---|---|---|---|---|---|"]
    for v, r in h2s.items():
        lines.append(f"| {NAMES[v]} | {r['claw_best']:.3f} | {r['target_best']:.3f} | "
                     f"{r.get('claw_auto_w', float('nan')):.3f} | {r.get('acc_w', float('nan')):.1%} | "
                     f"{r['claw_auto_disp']:.3f} | {r['acc_disp']:.1%} |")
    lines += ["", f"总用时 {time.time() - t0:.0f}s"]
    txt = "\n".join(lines)
    log(f, "\n" + txt)
    with open(os.path.join(RESULTS, f"results_{mode}.md"), "w", encoding="utf-8") as g:
        g.write(txt + "\n")
    with open(os.path.join(RESULTS, f"results_{mode}.json"), "w", encoding="utf-8") as g:
        json.dump({"args": vars(a), "steps": steps, "h1": h1, "h2": h2s}, g, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
