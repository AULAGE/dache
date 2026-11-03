"""
动作条件的物体中心世界模型（任务 14，论文方案 v2 第 3.2 节的最小实现，合成数据）。

  o_t ──CNN 编码器──► 特征图 ──Slot Attention（K 个槽位，初值 = 上一步的预测）──► 槽位 (z_k, p_k)
                                                                    p_k = 注意力图的质心（画面坐标 −1~1）
  [u_t, 槽位位置] ──迟滞记忆 GRU──► h_{t+1} ──线性──► c_{t+1}（相机位姿，画面坐标）
  动力学（分离先验）：p̂_{t+1,k} = p_{t,k} + w_k · (c_{t+1} − c_t)，  ẑ_{t+1,k} = z_{t,k} + MLP(z, u)
                       w_k = sigmoid(MLP(z_k)) ∈ [0, 1]：0 = 自身（和相机固连，不动），1 = 世界（随相机动）
  解码器：每个槽位用"相对自己位置"的坐标网格做空间广播 → RGB + alpha，softmax 合成
  损失：重建 o_t（看到之后的槽位）+ 预测 o_{t+1}（只用 ẑ、p̂，不看 o_{t+1}）+ 自身槽位位移 ≈ 0 的正则

h_t 就是学出来的"本体感知"：训练时没有任何角度标注，c 只能通过"世界槽位怎么移动"学出来。
评估时冻结模型，用线性探针从 h 解码真实舵机角（eval.py）。

对照组（--variant）：
  full       上面的全部
  noprior    无分离先验：p̂ = p + MLP(z_k, h_t, h_{t+1})，每个槽位自由移动（没有 w、没有 c）
  nogru      无 GRU：h_{t+1} = MLP(u_t, 槽位位置)，只用当前命令（没有命令历史）
  cmdonly    GRU 只看命令、不看画面（纯命令历史的迟滞记忆）
"""
import math

import torch
import torch.nn as nn
import torch.nn.functional as F

VARIANTS = ("full", "noprior", "nogru", "cmdonly")


def grid(h, w, device=None):
    ys = torch.linspace(-1, 1, h, device=device)
    xs = torch.linspace(-1, 1, w, device=device)
    yy, xx = torch.meshgrid(ys, xs, indexing="ij")
    return torch.stack([xx, yy], -1)             # [h, w, 2]，(x, y)


class Encoder(nn.Module):
    def __init__(self, d=32, res=64):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(3, d, 5, 2, 2), nn.ReLU(),       # 32×32
            nn.Conv2d(d, d, 3, 1, 1), nn.ReLU())
        self.fh = res // 2
        self.pos = nn.Linear(4, d)
        self.mlp = nn.Sequential(nn.LayerNorm(d), nn.Linear(d, d), nn.ReLU(), nn.Linear(d, d))

    def forward(self, x):                          # x [B, 3, H, W]
        f = self.conv(x).flatten(2).transpose(1, 2)    # [B, N, d]
        g = grid(self.fh, self.fh, x.device).reshape(-1, 2)
        f = f + self.pos(torch.cat([g, -g], -1))
        return self.mlp(f)


class SlotAttention(nn.Module):
    def __init__(self, k, d, iters=3):
        super().__init__()
        self.k, self.d, self.iters = k, d, iters
        self.norm_in = nn.LayerNorm(d)
        self.norm_s = nn.LayerNorm(d)
        self.norm_m = nn.LayerNorm(d)
        self.q, self.kk, self.v = nn.Linear(d, d, bias=False), nn.Linear(d, d, bias=False), nn.Linear(d, d, bias=False)
        self.gru = nn.GRUCell(d, d)
        self.mlp = nn.Sequential(nn.Linear(d, 2 * d), nn.ReLU(), nn.Linear(2 * d, d))

    def forward(self, feats, slots):
        """feats [B, N, d]，slots [B, K, d] → (slots, attn [B, K, N])"""
        x = self.norm_in(feats)
        k, v = self.kk(x), self.v(x)
        B = feats.shape[0]
        for _ in range(self.iters):
            q = self.q(self.norm_s(slots))
            logits = torch.einsum("bkd,bnd->bkn", q, k) / self.d ** 0.5
            attn = logits.softmax(1) + 1e-8             # 槽位之间竞争
            w = attn / attn.sum(-1, keepdim=True)
            upd = torch.einsum("bkn,bnd->bkd", w, v)
            slots = self.gru(upd.reshape(-1, self.d), slots.reshape(-1, self.d)).reshape(B, self.k, self.d)
            slots = slots + self.mlp(self.norm_m(slots))
        return slots, attn


class Decoder(nn.Module):
    """
    空间广播解码器（逐像素 MLP，CPU 上比卷积解码器便宜约 10 倍）：每个像素的输入 =
    槽位特征 z + "像素位置 − 槽位位置"的傅里叶特征，所以改 p 就能把这个槽位整体画到别处
    """

    def __init__(self, d=32, out=32, freqs=4, hidden=48):
        super().__init__()
        self.out, self.freqs = out, freqs
        fdim = 2 + 4 * freqs
        # 第一层拆成 W_z·z + W_r·位置特征（z 部分每个槽位只算一次）
        self.lz = nn.Linear(d, hidden)
        self.lr = nn.Linear(fdim, hidden, bias=False)
        self.net = nn.Sequential(nn.ReLU(), nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, 4))

    def fourier(self, rel):
        fs = [rel]
        for i in range(self.freqs):
            a = rel * (2.0 ** i) * math.pi
            fs += [torch.sin(a), torch.cos(a)]
        return torch.cat(fs, -1)

    def forward(self, z, p):
        """z [B, K, d]，p [B, K, 2] → rgb [B, 3, H, W]，masks [B, K, H, W]"""
        n = self.out
        g = grid(n, n, z.device)                                    # [n, n, 2]
        rel = self.fourier(g[None, None] - p[:, :, None, None, :])  # [B, K, n, n, F]
        y = self.net(self.lr(rel) + self.lz(z)[:, :, None, None, :])   # [B, K, n, n, 4]
        rgb, alpha = torch.sigmoid(y[..., :3]), y[..., 3]
        m = alpha.softmax(1)                                        # [B, K, n, n]
        return (rgb * m[..., None]).sum(1).permute(0, 3, 1, 2), m


class WorldModel(nn.Module):
    def __init__(self, variant="full", k=4, d=32, hdim=64, u_dim=5, res=64, dec_res=32):
        super().__init__()
        assert variant in VARIANTS
        self.variant, self.k, self.d, self.hdim, self.res, self.dec_res = variant, k, d, hdim, res, dec_res
        self.enc = Encoder(d, res)
        self.sa = SlotAttention(k, d)
        self.slot_init = nn.Parameter(torch.randn(1, k, d) * 0.5)
        self.dec = Decoder(d, dec_res)
        mem_in = u_dim + (0 if variant == "cmdonly" else 2 * k)
        if variant == "nogru":
            self.mem = nn.Sequential(nn.Linear(mem_in, hdim), nn.Tanh(), nn.Linear(hdim, hdim), nn.Tanh())
        else:
            self.mem = nn.GRUCell(mem_in, hdim)
        self.cam = nn.Linear(hdim, 2)                     # 相机位姿（画面坐标）= h 的线性函数
        self.world = nn.Sequential(nn.Linear(d, d), nn.ReLU(), nn.Linear(d, 1))
        self.zdyn = nn.Sequential(nn.Linear(d + u_dim, d), nn.ReLU(), nn.Linear(d, d))
        if variant == "noprior":
            self.pdyn = nn.Sequential(nn.Linear(d + 2 * hdim + u_dim, d), nn.ReLU(), nn.Linear(d, 2))

    def centroid(self, attn):
        """注意力图 → 每个槽位的质心（−1~1）"""
        fh = self.res // 2
        g = grid(fh, fh, attn.device).reshape(-1, 2)
        w = attn / attn.sum(-1, keepdim=True)
        return torch.einsum("bkn,nc->bkc", w, g)

    def compact(self, attn, p, sigma2=0.1):
        """槽位的紧凑度 exp(−注意力的空间方差 / σ²)：物块、爪子约 0.8，铺满全图的背景约 0"""
        fh = self.res // 2
        g = grid(fh, fh, attn.device).reshape(-1, 2)
        w = attn / attn.sum(-1, keepdim=True)
        var = torch.einsum("bkn,bkn->bk", w, ((g[None, None] - p[:, :, None]) ** 2).sum(-1))
        return torch.exp(-var / sigma2)

    def memory(self, h, u, p):
        x = u if self.variant == "cmdonly" else torch.cat([u, p.flatten(1)], -1)
        if self.variant == "nogru":
            return self.mem(x)
        return self.mem(x, h)

    def forward(self, obs, u):
        """
        obs [B, T+1, 3, H, W]（0~1），u [B, T, U]。
        返回 dict：损失项 + 评估用的中间量（h、w、掩膜、位置）
        """
        B, T1 = obs.shape[:2]
        T = T1 - 1
        tgt = F.avg_pool2d(obs.flatten(0, 1), self.res // self.dec_res).reshape(B, T1, 3, self.dec_res, self.dec_res) \
            if self.dec_res != self.res else obs
        h = torch.zeros(B, self.hdim, device=obs.device)
        slots = self.slot_init.expand(B, -1, -1)
        rec_l, pred_l, self_l = 0.0, 0.0, 0.0
        hs, ws, masks, ps, cs = [h], [], [], [], []
        for t in range(T1):
            feats = self.enc(obs[:, t])
            slots, attn = self.sa(feats, slots)
            p = self.centroid(attn)
            cs.append(self.compact(attn, p))
            rgb, m = self.dec(slots, p)
            rec_l = rec_l + F.mse_loss(rgb, tgt[:, t])
            w = torch.sigmoid(self.world(slots)).squeeze(-1)          # [B, K]
            ws.append(w)
            masks.append(m)
            ps.append(p)
            if t == T:
                break
            ut = u[:, t]
            h_next = self.memory(h, ut, p)
            if self.variant == "noprior":
                hh = torch.cat([h, h_next], -1)[:, None].expand(-1, self.k, -1)
                uu = ut[:, None].expand(-1, self.k, -1)
                p_hat = p + self.pdyn(torch.cat([slots, hh, uu], -1))
            else:
                c_t, c_n = self.cam(h), self.cam(h_next)
                dc = (c_n - c_t)[:, None]                              # [B, 1, 2]
                p_hat = p + w[..., None] * dc
            z_hat = slots + self.zdyn(torch.cat([slots, ut[:, None].expand(-1, self.k, -1)], -1))
            rgb_hat, _ = self.dec(z_hat, p_hat)
            pred_l = pred_l + F.mse_loss(rgb_hat, tgt[:, t + 1])
            # 下一帧的槽位初值 = 预测（SAVi 式）：槽位身份跨帧保持
            slots = z_hat
            h = h_next
            hs.append(h)
        # 分离先验的正则：观测到的位移 = w·Δc（世界）或 0（自身）。用相邻两帧的质心
        if self.variant != "noprior":
            P = torch.stack(ps, 1)                                     # [B, T+1, K, 2]
            W = torch.stack(ws, 1)[:, :-1]                             # [B, T, K]
            H = torch.stack(hs, 1)                                     # [B, T+1, hdim]
            C = self.cam(H)                                            # [B, T+1, 2]
            dP = P[:, 1:] - P[:, :-1]
            dC = (C[:, 1:] - C[:, :-1])[:, :, None]
            # 只对"紧凑"的槽位（物块、爪子）算：铺满全图的背景槽位质心不随相机动，不能用质心检验
            Kc = torch.stack(cs, 1)[:, 1:].detach()                    # [B, T, K]
            self_l = (Kc * ((dP - W[..., None] * dC) ** 2).sum(-1)).sum() / Kc.sum().clamp(min=1e-6)
        return {"rec": rec_l / T1, "pred": pred_l / T, "sep": self_l,
                "h": torch.stack(hs, 1), "w": torch.stack(ws, 1), "masks": torch.stack(masks, 1),
                "pos": torch.stack(ps, 1)}
