"""
合成腕部相机画面（任务 14，合成数据）：numpy 实现的简单 2D 渲染器。

几何（和 research/sim_world.py 一样的线性近似，"眼在手上"）：
  物块的画面方位角 ax = x0 + J_x · θ        θ = 底座舵机真实输出角（°），J_x 画面° / 底座°
  物块的画面俯仰角 ay = atan((y0 + J_y · h) / d)   h = 升降轴真实输出高度（cm），J_y mm / cm
  背景（平台、墙面）是世界固定的纹理，跟物块一样随相机移动（按更远的距离 D_BG 算俯仰）
  爪子脚和相机固连：永远在画面左侧中间，只有张爪 / 合爪两种形态，画在最上层（会遮挡物块）

画面：IMG×IMG RGB，半视场角 HALF_FOV 度（比真机 K230 的 ±35° 窄：64 像素下真机视场每度不到 1 像素，
回差 1° 只有半个像素，合成实验先把它放大到每度 1.6 像素；见报告"和真机的差别"）。
掩膜标签：0 背景、1 爪子（自身）、2 目标物块、3 另一个物块（世界里的干扰）、4 手（独立运动的干扰）
"""
import math

import numpy as np

IMG = 64
HALF_FOV = 20.0                       # 度
PX_PER_DEG = (IMG / 2.0) / HALF_FOV
D_BG = 400.0                          # 背景平面的距离（mm），升降时背景比物块移动得少
BLOCK_MM = 24.0

LABELS = {"bg": 0, "self": 1, "target": 2, "block": 3, "hand": 4}

# 爪子脚：和相机固连，画面左侧中间（真机约占画面 1/60；这里约 1/35，小图里太小学不出来）
# 每个形态是几个轴对齐矩形 (x0, y0, x1, y1)（像素，按 64×64 定义，其他分辨率等比缩放）
CLAW = {
    "open":  [(0, 26, 9, 31), (0, 31, 5, 39), (5, 35, 11, 39)],
    "close": [(0, 27, 6, 37), (6, 30, 8, 34)],
}
CLAW_RGB = np.array([0.32, 0.32, 0.36])


def _cover_1d(lo, hi, n):
    """像素 [i, i+1) 被区间 [lo, hi) 覆盖的比例（解析抗锯齿，物块位置有亚像素精度）"""
    edges = np.arange(n, dtype=np.float64)
    return np.clip(np.minimum(edges + 1, hi) - np.maximum(edges, lo), 0.0, 1.0)


def rect_cover(x0, y0, x1, y1, n=IMG):
    return np.outer(_cover_1d(y0, y1, n), _cover_1d(x0, x1, n))       # [y, x]


def ellipse_cover(cx, cy, rx, ry, n=IMG, ss=2):
    """椭圆的覆盖率（ss×ss 超采样）"""
    g = (np.arange(n * ss) + 0.5) / ss
    yy, xx = np.meshgrid(g, g, indexing="ij")
    m = (((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0).astype(np.float64)
    return m.reshape(n, ss, n, ss).mean(axis=(1, 3))


def make_texture(rng, size=256):
    """世界固定的背景纹理：几组随机低频正弦叠加（彩色、对比度低），覆盖 ±80° 的视角"""
    yy, xx = np.meshgrid(np.arange(size), np.arange(size), indexing="ij")
    base = rng.uniform(0.35, 0.6, size=3)
    tex = np.zeros((size, size, 3))
    for _ in range(6):
        f = rng.uniform(0.02, 0.12)
        a = rng.uniform(0, 2 * math.pi)
        ph = rng.uniform(0, 2 * math.pi)
        w = np.sin(f * (xx * math.cos(a) + yy * math.sin(a)) + ph)
        tex += w[..., None] * rng.uniform(-0.025, 0.025, size=3)
    # 平台边缘：一条水平分界（上面墙、下面台面），让纵向移动也有明显的特征
    edge = size // 2 + int(rng.integers(-20, 20))
    tex[edge:] += rng.uniform(-0.06, 0.06, size=3)
    return np.clip(tex + base, 0.0, 1.0)


def sample_texture(tex, sx, sy, n=IMG):
    """从纹理里取 n×n 的窗口，窗口中心相对纹理中心偏 (sx, sy) 像素（双线性，亚像素）"""
    size = tex.shape[0]
    c = size / 2.0 - n / 2.0
    x0, y0 = c + sx, c + sy
    ix, iy = int(math.floor(x0)), int(math.floor(y0))
    fx, fy = x0 - ix, y0 - iy
    ix = max(0, min(size - n - 1, ix))
    iy = max(0, min(size - n - 1, iy))
    a = tex[iy:iy + n, ix:ix + n]
    b = tex[iy:iy + n, ix + 1:ix + n + 1]
    cc = tex[iy + 1:iy + n + 1, ix:ix + n]
    d = tex[iy + 1:iy + n + 1, ix + 1:ix + n + 1]
    return (a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + cc * (1 - fx) * fy + d * fx * fy)


class Scene:
    """一个序列的固定内容（一台车、一个场景）：物块位置和颜色、背景纹理、干扰物、光照色调"""

    def __init__(self, rng, J_x=-1.0, J_y=6.0, n=IMG, p_block2=0.3, p_hand=0.3):
        self.rng, self.n = rng, n
        self.J_x, self.J_y = J_x, J_y
        self.s = n / 64.0
        self.tex = make_texture(rng)
        self.tint = rng.uniform(0.9, 1.1, size=3)
        self.target = self._block(rng)
        self.block2 = self._block(rng) if rng.random() < p_block2 else None
        if self.block2 is not None:
            while abs(self.block2["x0"] - self.target["x0"]) < 6:
                self.block2["x0"] = rng.uniform(-14, 14)
        self.hand = None
        if rng.random() < p_hand:
            self.hand = {"x": rng.uniform(10, 54), "y": rng.uniform(10, 54), "rx": rng.uniform(5, 8),
                         "ry": rng.uniform(7, 11), "rgb": np.array([0.85, 0.62, 0.5]) * rng.uniform(0.9, 1.1)}

    def _block(self, rng):
        hue = rng.uniform(0, 1)
        rgb = np.array([abs(math.sin(math.pi * (hue + k / 3.0))) for k in range(3)])
        rgb = 0.15 + 0.8 * rgb / max(rgb.max(), 1e-6)
        return {"x0": rng.uniform(-6, 6), "y0": rng.uniform(-12, 12), "d": rng.uniform(110, 160), "rgb": rgb}

    def step_distractors(self):
        """手（独立运动的干扰物）每一步随机移动，和动作无关"""
        if self.hand is not None:
            self.hand["x"] = float(np.clip(self.hand["x"] + self.rng.normal(0, 4), 4, 60))
            self.hand["y"] = float(np.clip(self.hand["y"] + self.rng.normal(0, 4), 4, 60))

    def block_px(self, blk, theta, h):
        """物块中心的像素坐标和边长（像素）"""
        ax = blk["x0"] + self.J_x * theta
        ay = math.degrees(math.atan2(blk["y0"] + self.J_y * h, blk["d"]))
        size = 2 * math.degrees(math.atan2(BLOCK_MM / 2, blk["d"])) * PX_PER_DEG * self.s
        c = self.n / 2.0
        return c + ax * PX_PER_DEG * self.s, c + ay * PX_PER_DEG * self.s, size

    def render(self, theta, h, claw, gain=1.0, noise=0.02):
        """
        返回 (图像 float32 [n, n, 3] 0~1, 掩膜 uint8 [n, n])。
        theta：底座真实输出角（°）；h：升降轴真实高度（cm）；claw："open" / "close"
        """
        n, s = self.n, self.s
        # 背景：世界固定 → 随相机移动
        sx = self.J_x * theta * PX_PER_DEG * s
        sy = math.degrees(math.atan2(self.J_y * h, D_BG)) * PX_PER_DEG * s
        img = sample_texture(self.tex, sx, sy, n)
        lab = np.zeros((n, n), dtype=np.uint8)
        cov_best = np.zeros((n, n))

        def paint(cov, rgb, label):
            nonlocal img
            img = img * (1 - cov[..., None]) + cov[..., None] * rgb
            m = cov > 0.5
            lab[m] = label
            cov_best[m] = cov[m]

        for blk, label in ((self.block2, LABELS["block"]), (self.target, LABELS["target"])):
            if blk is None:
                continue
            cx, cy, size = self.block_px(blk, theta, h)
            cov = rect_cover(cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2, n)
            # 顶面亮一点、侧面暗一点：给物块一点内部结构
            shade = np.ones((n, n, 1)) * blk["rgb"]
            paint(cov, shade, label)
        if self.hand is not None:
            hd = self.hand
            paint(ellipse_cover(hd["x"] * s, hd["y"] * s, hd["rx"] * s, hd["ry"] * s, n), hd["rgb"], LABELS["hand"])
        claw_cov = np.zeros((n, n))
        for (x0, y0, x1, y1) in CLAW[claw]:
            claw_cov = np.maximum(claw_cov, rect_cover(x0 * s, y0 * s, x1 * s, y1 * s, n))
        paint(claw_cov, CLAW_RGB, LABELS["self"])
        img = img * self.tint * gain + self.rng.normal(0, noise, size=img.shape)
        return np.clip(img, 0, 1).astype(np.float32), lab
