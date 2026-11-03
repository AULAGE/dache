"""
结果监督的残差标定（任务 11，第 4 项）

问题：对准后爪子落点和物块之间还有一个看不见的系统残差 r（真机：偏右约 4mm，现在用 GRASP_BASE_TRIM_DEG 手调）。
唯一的监督是每次抓取的成败（伯努利）。抓取时故意瞄准偏移 e（相对"对准后的位置"，通过底座微调实现）：
    P(成功 | e, r) = Φ((w/2 − |e − r|) / σ)
        w/2：爪子能兜住的左右余量（mm），σ：落点随机误差（mm）—— 都是估计值，要在台架上测
后验（网格，r ∈ [−15, 15] mm）：
    p(r | 数据) ∝ N(r; 0, 6²) · Π_k P(s_k | e_k, r)
下一次的瞄准点：
    纯利用    e = r̂（后验均值）——成功率高，但成功几乎不带信息（落在平台区，看不出偏哪边）
    固定 ±p   e = r̂ ± p 交替 —— 在爪子边缘附近试探，成败对 r 敏感
    信息增益  e = argmax_e I(s; r) = H(p̄(e)) − E_r[H(P(s|e,r))]   （H 为二元熵）
    信息增益（成功率约束）  只在预测成功率 ≥ 0.6 的 e 里挑信息增益最大的（标定期间也要尽量抓到）
"""
import math

GRID_STEP = 0.25
GRID = [-15.0 + GRID_STEP * i for i in range(int(30 / GRID_STEP) + 1)]
PRIOR_SD = 6.0
CANDS = [-12.0 + 0.5 * i for i in range(49)]     # 候选瞄准偏移（mm，对齐网格）


def phi(x):
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def p_success(e, r, half_w, sigma):
    return phi((half_w - abs(e - r)) / sigma)


def _h2(p):
    if p <= 0.0 or p >= 1.0:
        return 0.0
    return -(p * math.log(p) + (1 - p) * math.log(1 - p))


class ResidualPosterior:
    def __init__(self, half_w=8.0, sigma=3.0, prior_sd=PRIOR_SD):
        self.half_w, self.sigma = half_w, sigma
        w = [math.exp(-0.5 * (r / prior_sd) ** 2) for r in GRID]
        z = sum(w)
        self.w = [x / z for x in w]
        # P(成功) 只和 e − r 有关：按网格步长预先算好查表
        n = int(round(30 / GRID_STEP)) + int(round(24 / GRID_STEP)) + 2
        self._tab = {}
        for k in range(-n, n + 1):
            self._tab[k] = p_success(k * GRID_STEP, 0.0, half_w, sigma)
        self.n = 0

    def _p(self, e, i):
        return self._tab[int(round((e - GRID[i]) / GRID_STEP))]

    def update(self, e, success):
        w = self.w
        for i in range(len(GRID)):
            p = self._p(e, i)
            w[i] *= p if success else (1.0 - p)
        z = sum(w)
        if z > 0:
            self.w = [x / z for x in w]
        self.n += 1

    def mean(self):
        return sum(wi * r for wi, r in zip(self.w, GRID))

    def sd(self):
        mu = self.mean()
        return math.sqrt(sum(wi * (r - mu) ** 2 for wi, r in zip(self.w, GRID)))

    def pred_success(self, e):
        return sum(wi * self._p(e, i) for i, wi in enumerate(self.w) if wi > 1e-12)

    def info_gain(self, e):
        pb, hc = 0.0, 0.0
        for i, wi in enumerate(self.w):
            if wi < 1e-9:
                continue
            p = self._p(e, i)
            pb += wi * p
            hc += wi * _h2(p)
        return _h2(pb) - hc, pb

    def best_ig(self, min_success=None):
        best, best_v = None, -1.0
        for e in CANDS:
            ig, ps = self.info_gain(e)
            if min_success is not None and ps < min_success:
                continue
            if ig > best_v:
                best, best_v = e, ig
        if best is None:                     # 没有满足成功率约束的：退回纯利用
            return self.snap(self.mean())
        return best

    @staticmethod
    def snap(e):
        e = max(CANDS[0], min(CANDS[-1], e))
        return round(e * 2) / 2.0


STRATEGIES = ("exploit", "fixed4", "fixed8", "ig", "ig_safe")
STRATEGY_NAME = {"exploit": "纯利用 e=r̂", "fixed4": "固定探测 ±4mm", "fixed8": "固定探测 ±8mm",
                 "ig": "信息增益最大", "ig_safe": "信息增益（预测成功率≥0.6）"}


def next_aim(post, k, strategy):
    if strategy == "exploit":
        return post.snap(post.mean())
    if strategy in ("fixed4", "fixed8"):
        p = 4.0 if strategy == "fixed4" else 8.0
        return post.snap(post.mean() + (p if k % 2 == 0 else -p))
    if strategy == "ig":
        return post.best_ig()
    if strategy == "ig_safe":
        return post.best_ig(0.6)
    raise ValueError(strategy)


def run_calibration(r_true, strategy, n, rng, half_w=8.0, sigma=3.0, true_half_w=None, true_sigma=None,
                    checkpoints=(5, 10, 20, 30, 40)):
    """
    模拟 n 次标定抓取。真实世界的 w/2、σ 可以和模型不同（检验模型设错时的稳健性）。
    返回 {'err': {k: |r̂ − r|}, 'succ': 标定期间成功次数, 'final_p': 标定后瞄准 r̂ 的真实成功率}
    """
    thw = half_w if true_half_w is None else true_half_w
    ts = sigma if true_sigma is None else true_sigma
    post = ResidualPosterior(half_w, sigma)
    err, succ = {}, 0
    for k in range(n):
        e = next_aim(post, k, strategy)
        s = rng.random() < p_success(e, r_true, thw, ts)
        succ += s
        post.update(e, s)
        if k + 1 in checkpoints:
            err[k + 1] = abs(post.mean() - r_true)
    return {"err": err, "succ": succ, "final_p": p_success(post.mean(), r_true, thw, ts)}
