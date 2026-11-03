"""
迟滞辨识与逆迟滞补偿（任务 11，第 3 项）

执行器模型（每个轴一样；底座单位 °，升降轴单位 cm）：
    电机位置 m、输出轴位置 o、命令 c
    死区：|c − m| > δ 时 m ← c，否则 m 不动（命令被"吃掉"，但下一条命令还是按绝对值比较）
    回差：o ← clip(o, m − b/2, m + b/2)  —— 输出轴在宽 b 的间隙里被推着走
    画面位移：Δz = J · Δo + ε，ε ~ N(0, σ²)
记作 Δo = g(u; b, δ, 历史)。同样的命令，结果取决于历史（换向时先走完间隙）。

辨识（"假设库"贝叶斯）：
    θ = (b, δ) 取网格；每个网格点 h 下 g 是确定的，用真实发过的命令序列推出 Δo_h,k。
    J 对每个 h 用共轭高斯先验 N(J0, sJ²)，观测 Δz_k = J·Δo_h,k + ε 是线性高斯，边缘似然有闭式：
        P_h   = 1/sJ² + Sxx/σ²                 Sxx = Σ Δo²,  Sxy = Σ Δo·Δz,  Syy = Σ Δz²
        Ĵ_h   = (J0/sJ² + Sxy/σ²) / P_h
        log p(Δz_1..k | h) = −½ (Syy/σ² + J0²/sJ² − Ĵ_h² P_h) − ½ log(P_h sJ²) + 常数
    后验 w_h ∝ p(h) · p(Δz | h)；只需维护每个 h 的 (m, o, Sxx, Sxy, Syy)，在线 O(网格大小)。
    初始状态：先做一次"预紧"（往负方向走一大步再回来），输出轴就贴在间隙负侧：o = m − b/2。

逆迟滞补偿：
    想让画面移动 Δz*，对后验最大的 K 个假设，各自解析求出命令序列：
        Δo* = Δz*/Ĵ_h，o* = o + Δo*，s = sign(Δo*)
        单步：c = o* + s·b/2（推着间隙一侧走到位）；若 |c − m| ≤ δ（会被死区吃掉）
        两步：先反向退 r = c − s·(δ + b + 余量)，再从同一侧逼近 c（"单向逼近"）
    把这些候选在全部 K 个假设下前向推演，选期望平方误差最小的：
        E[(Ĵ_h·Δo_h(c) − Δz*)² + Var(J_h)·Δo_h(c)²]
"""
import math

from .sim_world import play

GRID = {
    # 辨识网格（比仿真里真实参数的范围宽，不偷看）
    "x": {"b": (0.0, 6.0), "delta": (0.0, 4.0), "J0": -0.8, "sJ": 0.4, "sigma": 0.3, "quant": 1 / 11.1},
    "y": {"b": (0.0, 0.5), "delta": (0.0, 0.25), "J0": 6.0, "sJ": 2.5, "sigma": 1.5, "quant": 0.01},
}


def _frange(lo, hi, n):
    return [lo + (hi - lo) * i / (n - 1) for i in range(n)]


class HysteresisBank:
    def __init__(self, bs, deltas, J0, sJ, sigma, m0=0.0, o0=None):
        self.b, self.dl = [], []
        for b in bs:
            for d in deltas:
                self.b.append(b)
                self.dl.append(d)
        n = len(self.b)
        self.m = [m0] * n
        # 预紧后输出轴贴在间隙负侧；o0 给定时（真值假设）直接用
        self.o = [m0 - b / 2 for b in self.b] if o0 is None else [o0] * n
        self.Sxx = [0.0] * n
        self.Sxy = [0.0] * n
        self.Syy = [0.0] * n
        self.J0, self.sJ, self.sigma = J0, sJ, sigma
        self.nobs = 0
        self._w = None

    @classmethod
    def for_axis(cls, axis, n_b=None, n_d=None):
        """默认网格：底座 25×17（步长 0.25°），升降 17×11"""
        n_b = n_b or (25 if axis == "x" else 17)
        n_d = n_d or (17 if axis == "x" else 11)
        g = GRID[axis]
        return cls(_frange(*g["b"], n_b), _frange(*g["delta"], n_d), g["J0"], g["sJ"], g["sigma"])

    @classmethod
    def oracle(cls, b, delta, J, m, o, sigma):
        """真值假设（补偿的上限）：一个假设，J 几乎确定"""
        return cls([b], [delta], J, 1e-4, sigma, m0=m, o0=o)

    def __len__(self):
        return len(self.b)

    # ------------------------------------------------ 推演
    def preview(self, cmds, idx=None):
        """执行命令序列后每个假设的输出位移 Δo（不改变状态）"""
        out = []
        for i in (range(len(self.b)) if idx is None else idx):
            m, o = self.m[i], self.o[i]
            o_start = o
            b, d = self.b[i], self.dl[i]
            for c in cmds:
                m, o = play(m, o, c, b, d)
            out.append(o - o_start)
        return out

    def apply(self, cmds, dz=None):
        """真的发了 cmds；dz 是确认目标的画面位移（None = 没有可信观测，只推进状态）"""
        if not cmds and dz is None:
            return
        Sxx, Sxy, Syy = self.Sxx, self.Sxy, self.Syy
        for i in range(len(self.b)):
            m, o = self.m[i], self.o[i]
            o_start = o
            b, d = self.b[i], self.dl[i]
            for c in cmds:
                m, o = play(m, o, c, b, d)
            self.m[i], self.o[i] = m, o
            if dz is not None:
                do = o - o_start
                Sxx[i] += do * do
                Sxy[i] += do * dz
                Syy[i] += dz * dz
        if dz is not None:
            self.nobs += 1
            self._w = None

    # ------------------------------------------------ 后验
    def _J(self, i):
        s2, p0 = self.sigma ** 2, 1.0 / self.sJ ** 2
        P = p0 + self.Sxx[i] / s2
        return (self.J0 * p0 + self.Sxy[i] / s2) / P, 1.0 / P

    def weights(self):
        if self._w is None:
            s2, p0 = self.sigma ** 2, 1.0 / self.sJ ** 2
            lls = []
            for i in range(len(self.b)):
                P = p0 + self.Sxx[i] / s2
                mu = (self.J0 * p0 + self.Sxy[i] / s2) / P
                lls.append(-0.5 * (self.Syy[i] / s2 + self.J0 ** 2 * p0 - mu * mu * P) - 0.5 * math.log(P / p0))
            mx = max(lls)
            w = [math.exp(l - mx) for l in lls]
            z = sum(w)
            self._w = [x / z for x in w]
        return self._w

    def estimate(self):
        """后验均值和标准差：{'b': (均值, sd), 'delta': ..., 'J': ...}"""
        w = self.weights()
        res = {}
        for name, vals in (("b", self.b), ("delta", self.dl)):
            mu = sum(wi * v for wi, v in zip(w, vals))
            sd = math.sqrt(max(0.0, sum(wi * (v - mu) ** 2 for wi, v in zip(w, vals))))
            res[name] = (mu, sd)
        js = [self._J(i) for i in range(len(self.b))]
        mu = sum(wi * j[0] for wi, j in zip(w, js))
        var = sum(wi * (j[1] + (j[0] - mu) ** 2) for wi, j in zip(w, js))
        res["J"] = (mu, math.sqrt(max(0.0, var)))
        return res

    def topk(self, k):
        w = self.weights()
        idx = sorted(range(len(w)), key=lambda i: -w[i])[:k]
        z = sum(w[i] for i in idx)
        return idx, [w[i] / z for i in idx]

    def predictive(self, cmds, k=None):
        """执行 cmds 后确认目标画面位移的预测分布：[(权重, 均值, 方差)]（方差含 J 的不确定性，不含测量噪声）"""
        if k:
            idx, w = self.topk(k)
        else:
            idx, w = list(range(len(self.b))), self.weights()
        dos = self.preview(cmds, idx)
        out = []
        for wi, i, do in zip(w, idx, dos):
            mu, var = self._J(i)
            out.append((wi, mu * do, var * do * do))
        return out

    # ------------------------------------------------ 逆迟滞补偿
    def plan(self, dz_want, k=12, quant=None, margin=0.3):
        """想让画面移动 dz_want：返回要依次发出的绝对命令（[] = 不动）"""
        if abs(dz_want) < 1e-9:
            return []
        idx, w = self.topk(k)
        q = (lambda c: round(c / quant) * quant) if quant else (lambda c: c)
        cands = set()
        for i in idx:
            mu, _ = self._J(i)
            if abs(mu) < 1e-6:
                continue
            m, o, b, d = self.m[i], self.o[i], self.b[i], self.dl[i]
            do = dz_want / mu
            s = 1.0 if do > 0 else -1.0
            c = o + do + s * b / 2
            if abs(c - m) > d and s * (c - m) > 0:
                cands.add((q(c),))
            # 两步：单向逼近（先反向退出死区 + 间隙，再往同一方向走到位）
            r = (min(m, c) - d - b - margin) if s > 0 else (max(m, c) + d + b + margin)
            cands.add((q(r), q(c)))
        best, best_cost = [], None
        js = [self._J(i) for i in idx]
        for cmds in cands:
            dos = self.preview(list(cmds), idx)
            cost = sum(wi * ((j[0] * do - dz_want) ** 2 + j[1] * do * do) for wi, j, do in zip(w, js, dos))
            cost *= 1.0 + 0.02 * (len(cmds) - 1)          # 同样准时优先一步到位（省时间）
            if best_cost is None or cost < best_cost:
                best, best_cost = list(cmds), cost
        return best
