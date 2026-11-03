"""
评估（任务 14，合成数据）：

H1 视觉本体感知：冻结模型，取迟滞记忆的隐状态 h_t，线性探针（岭回归，只在训练序列上拟合）解码
   "真实输出 − 命令"（命令是机器人自己知道的；不知道的只是回差 / 死区造成的残差），
   估计值 θ̂ = 命令 + 探针输出，和真实舵机角比。按底座回差大小分组。
   基线：命令积分（θ̂ = 命令）、方向记忆规则（θ̂ = 命令 − k·上一次转动方向，k 用训练集拟合）。
H2 自身 / 世界分离：解码器的槽位掩膜（argmax）和真实"爪子 / 目标物块"掩膜的 IoU。
   最佳匹配 IoU（槽位里和爪子重合最好的那个）+ 自动判定（不看真值）：
     按 w：w 最小的槽位 = 自身（只有带分离先验的变体有 w）
     按位移：整段序列里质心几乎不动、又不是铺满全图的背景的槽位 = 自身
   以及"判别正确率"：和爪子最匹配的槽位被判为自身、和物块最匹配的槽位被判为世界的帧的比例。
"""
import numpy as np
import torch

B_BINS = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.5), (3.5, 99.0)]


def collect(model, data, idx, batch=16, device="cpu"):
    """在数据子集 idx 上跑模型，收集 h、w、掩膜、位置"""
    model.eval()
    hs, ws, ms, ps = [], [], [], []
    with torch.no_grad():
        for i in range(0, len(idx), batch):
            j = idx[i:i + batch]
            obs = torch.from_numpy(data["obs"][j]).float().div(255).permute(0, 1, 4, 2, 3).to(device)
            u = torch.from_numpy(data["u"][j]).to(device)
            out = model(obs, u)
            hs.append(out["h"].cpu().numpy())
            ws.append(out["w"].cpu().numpy())
            ms.append(out["masks"].cpu().numpy())
            ps.append(out["pos"].cpu().numpy())
    model.train()
    return {"h": np.concatenate(hs), "w": np.concatenate(ws), "masks": np.concatenate(ms),
            "pos": np.concatenate(ps)}


def ridge(X, Y, lam=1e-2):
    X1 = np.concatenate([X, np.ones((len(X), 1))], 1)
    A = X1.T @ X1 + lam * np.eye(X1.shape[1])
    return np.linalg.solve(A, X1.T @ Y)


def apply(Wt, X):
    return np.concatenate([X, np.ones((len(X), 1))], 1) @ Wt


def _resid(data, idx):
    """t ≥ 1 的帧：残差（真实 − 命令），命令，真实值，回差。形状 [N, 2]"""
    r = (data["true"][idx] - data["cmd"][idx])[:, 1:]
    cmd = data["cmd"][idx][:, 1:]
    b = np.repeat(data["b"][idx][:, None, 0], r.shape[1], 1)
    return r.reshape(-1, 2), cmd.reshape(-1, 2), b.reshape(-1)


def _last_dir(data, idx):
    """方向记忆规则的特征：每个轴上一次非零命令变化的方向（−1 / 0 / +1）"""
    du = data["u"][idx][:, :, 2:4]                    # [n, T, 2] Δ命令（归一化）
    out = np.zeros(du.shape, np.float32)
    last = np.zeros((du.shape[0], 2), np.float32)
    for t in range(du.shape[1]):
        nz = np.abs(du[:, t]) > 1e-6
        last = np.where(nz, np.sign(du[:, t]), last)
        out[:, t] = last
    return out.reshape(-1, 2)


def h1(feat_tr, feat_te, data, tr, te):
    """H1：返回每种方法的底座 / 升降误差，以及按回差分组的底座误差"""
    r_tr, _, _ = _resid(data, tr)
    r_te, _, b_te = _resid(data, te)
    res = {}

    def score(name, rhat):
        e = np.abs(r_te - rhat)
        row = {"x": float(e[:, 0].mean()), "y": float(e[:, 1].mean()),
               "x_bins": [float(e[(b_te >= lo) & (b_te < hi), 0].mean()) if np.any((b_te >= lo) & (b_te < hi))
                          else float("nan") for lo, hi in B_BINS]}
        big = np.abs(r_te[:, 0]) > 0.25
        row["dir_acc"] = float((np.sign(rhat[big, 0]) == np.sign(r_te[big, 0])).mean()) if big.any() else float("nan")
        res[name] = row

    score("命令积分（开环）", np.zeros_like(r_te))
    d_tr, d_te = _last_dir(data, tr), _last_dir(data, te)
    score("方向记忆规则（上一次转动方向 × 常数）", apply(ridge(d_tr, r_tr), d_te))
    for name, Xtr in feat_tr.items():
        Xtr = Xtr[:, 1:].reshape(-1, Xtr.shape[-1])
        Xte = feat_te[name][:, 1:].reshape(-1, Xtr.shape[-1])
        score(name, apply(ridge(Xtr, r_tr), Xte))
    return res


def _down(mask, size):
    """真实标签 [.., H, W] → [.., size, size]（每块取多数）"""
    f = mask.shape[-1] // size
    m = mask.reshape(*mask.shape[:-2], size, f, size, f)
    oh = np.stack([(m == k).mean(axis=(-1, -3)) for k in range(5)], -1)
    return oh.argmax(-1)


def h2(out, data, idx, variant):
    masks = out["masks"]                              # [n, T+1, K, s, s]
    n, T1, K, s, _ = masks.shape
    lab = _down(data["mask"][idx], s)                 # [n, T+1, s, s]
    am = masks.argmax(2)                              # [n, T+1, s, s]
    claw = lab == 1
    tgt = lab == 2

    def iou(a, b):
        inter = (a & b).sum(axis=(-1, -2))
        uni = (a | b).sum(axis=(-1, -2))
        return np.where(uni > 0, inter / np.maximum(uni, 1), np.nan)

    ious_c = np.stack([iou(am == k, claw) for k in range(K)], -1)    # [n, T+1, K]
    ious_t = np.stack([iou(am == k, tgt) for k in range(K)], -1)
    best_c, best_t = ious_c.argmax(-1), ious_t.argmax(-1)
    res = {"claw_best": float(np.nanmean(ious_c.max(-1))), "target_best": float(np.nanmean(ious_t.max(-1)))}
    # 按位移：每个序列里各槽位质心的平均帧间位移；面积超过 40% 的（背景）不算
    pos = out["pos"]
    disp = np.abs(np.diff(pos, axis=1)).sum(-1).mean(1)               # [n, K]
    area = np.stack([(am == k).mean(axis=(-1, -2)) for k in range(K)], -1).mean(1)   # [n, K]
    disp = np.where(area > 0.4, np.inf, disp)
    self_d = disp.argmin(-1)                                           # [n]
    sel = np.take_along_axis(ious_c, np.repeat(self_d[:, None, None], T1, 1), 2)[..., 0]
    res["claw_auto_disp"] = float(np.nanmean(sel))
    res["acc_disp"] = float(np.mean(best_c == self_d[:, None]))
    if variant != "noprior":
        w = out["w"]                                                   # [n, T+1, K]
        wm = np.where(area[:, None, :] > 0.4, np.inf, w)
        self_w = wm.argmin(-1)                                         # [n, T+1]
        sel = np.take_along_axis(ious_c, self_w[..., None], 2)[..., 0]
        res["claw_auto_w"] = float(np.nanmean(sel))
        wc = np.take_along_axis(w, best_c[..., None], 2)[..., 0]
        wt = np.take_along_axis(w, best_t[..., None], 2)[..., 0]
        has_t = tgt.any(axis=(-1, -2))
        res["acc_w"] = float(np.mean((wc < 0.5) & ((wt > 0.5) | ~has_t)))
    return res


SLOT_COLORS = np.array([[230, 60, 60], [60, 200, 60], [70, 110, 240], [230, 200, 50], [200, 80, 220], [60, 210, 210]],
                       np.uint8)


def save_slots(path, model, data, idx, steps=8, device="cpu"):
    """每条序列三行：输入画面 / 真实掩膜 / 槽位分割（颜色 = 槽位，越暗 = w 越小 = 越像"自身"）"""
    from research.worldmodel.dataset import write_png
    out = collect(model, data, idx, device=device)
    rows = []
    for i in range(len(idx)):
        a, b, c = [], [], []
        for t in range(min(steps, data["obs"].shape[1])):
            a.append(data["obs"][idx[i], t])
            b.append(np.stack([data["mask"][idx[i], t] * 60] * 3, -1).astype(np.uint8))
            am = out["masks"][i, t].argmax(0)
            col = SLOT_COLORS[am].astype(np.float32) * (0.3 + 0.7 * out["w"][i, t][am][..., None])
            f = a[-1].shape[0] // am.shape[0]
            c.append(np.repeat(np.repeat(col.astype(np.uint8), f, 0), f, 1))
        rows += [np.concatenate(a, 1), np.concatenate(b, 1), np.concatenate(c, 1)]
    write_png(path, np.concatenate(rows, 0))
