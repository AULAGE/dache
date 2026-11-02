"""
自适应视觉伺服（每个误差轴一个）

模型：执行器动 u，误差变化 Δe ≈ J·s·u
        J = 灵敏度（"权重"，要学的部分）
        s = 已知的几何系数（例如底座转动时 s = 1 + 相机半径/深度；没有就是 1）
控制：u = -α·e / (J·s)      一步就把误差消到 (1-α)
学习：每走一步，用实测的 Δe/(s·u) 修正 J（指数平均，符号必须与初值一致，幅度限制在 1/4~4 倍）

J 估得越准，越少步数就能对准；学到的 J 会存到文件，下次启动直接用。
"""
import json
import os


class AdaptiveAxis:
    def __init__(self, name, j0, max_step, min_step, alpha, lr, noise, j=None):
        self.name = name
        self.j0 = j0
        self.j = j if (j is not None and j * j0 > 0) else j0
        self.max_step = max_step
        self.min_step = min_step
        self.alpha = alpha
        self.lr = lr
        self.noise = noise       # 误差测量噪声量级，变化比这小的不用来学习

    def command(self, err, s=1.0):
        u = -self.alpha * err / (self.j * s)
        u = max(-self.max_step, min(self.max_step, u))
        return 0.0 if abs(u) < self.min_step else u

    def learn(self, u, err_before, err_after, s=1.0):
        if abs(u) < 2 * self.min_step:
            return
        de = err_after - err_before
        if abs(de) < 2 * self.noise:
            return
        jm = de / (u * s)
        if jm * self.j0 <= 0:          # 方向反了：多半是噪声 / 打滑 / 目标换了，不学
            return
        lo, hi = abs(self.j0) / 4, abs(self.j0) * 4
        mag = max(lo, min(hi, abs(jm)))
        jm = mag if self.j0 > 0 else -mag
        self.j = (1 - self.lr) * self.j + self.lr * jm


# 各轴的测量噪声量级（用于判断一步的变化是否可信）
NOISE = {"y": 1.5, "x_deg": 0.4, "d": 4.0}


def build_axes(cfg, names, learn_path=None):
    learned = {}
    if learn_path and os.path.exists(learn_path):
        try:
            with open(learn_path, encoding="utf-8") as f:
                learned = json.load(f)
        except (OSError, ValueError):
            learned = {}
    axes = {}
    for n in names:
        a = cfg.AXES[n]
        axes[n] = AdaptiveAxis(n, a["j0"], a["max"], a["min"], cfg.STEP_ALPHA,
                               cfg.LEARN_RATE, NOISE[n], learned.get(n))
    return axes


def save_axes(axes, learn_path):
    data = {n: a.j for n, a in axes.items()}
    tmp = learn_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, learn_path)
