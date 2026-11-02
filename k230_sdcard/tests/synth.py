# -*- coding: utf-8 -*-
"""合成测试图：背景上画一个（可旋转、可抗锯齿的）纯色正方形，输出 (3, H, W) uint8，和 K230 的 RGBP888 一样"""
import math

import numpy as np

FW, FH = 1280, 720


def render(cx, cy, side, angle_deg=0.0, color=(200, 40, 40), bg=(90, 90, 90),
           fw=FW, fh=FH, aa=4, side_b=None, extra=()):
    """extra: 额外的 (cx, cy, side, angle, color) 物块"""
    img = np.zeros((3, fh, fw), dtype=np.float32)
    for c in range(3):
        img[c, :, :] = bg[c]
    for (ox, oy, s, a, col, sb) in [(cx, cy, side, angle_deg, color, side_b)] + \
            [tuple(e) + ((None,) if len(e) == 5 else ()) for e in extra]:
        sb = s if sb is None else sb
        r = math.hypot(s, sb) / 2 + 2
        x0, x1 = max(0, int(ox - r)), min(fw, int(ox + r) + 1)
        y0, y1 = max(0, int(oy - r)), min(fh, int(oy + r) + 1)
        if x1 <= x0 or y1 <= y0:
            continue
        # 超采样求覆盖率
        sub = (np.arange(aa) + 0.5) / aa
        xs = (np.arange(x0, x1)[None, :, None, None] + sub[None, None, None, :])
        ys = (np.arange(y0, y1)[:, None, None, None] + sub[None, None, :, None])
        t = math.radians(a)
        c, s_ = math.cos(t), math.sin(t)
        dx, dy = xs - ox, ys - oy
        u = dx * c + dy * s_
        v = -dx * s_ + dy * c
        inside = (np.abs(u) <= s / 2) & (np.abs(v) <= sb / 2)
        cov = inside.mean(axis=(2, 3))
        for ch in range(3):
            reg = img[ch, y0:y1, x0:x1]
            img[ch, y0:y1, x0:x1] = reg * (1 - cov) + col[ch] * cov
    return np.clip(np.round(img), 0, 255).astype(np.uint8)


def det_box(cx, cy, side, angle_deg=0.0, pad=0.0, fw=FW, fh=FH, side_b=None):
    """模拟检测框：旋转物块的外接正框（可加松紧），并裁到画面内"""
    sb = side if side_b is None else side_b
    t = math.radians(angle_deg)
    hw = (abs(side * math.cos(t)) + abs(sb * math.sin(t))) / 2 * (1 + pad)
    hh = (abs(side * math.sin(t)) + abs(sb * math.cos(t))) / 2 * (1 + pad)
    return [max(0.0, cx - hw), max(0.0, cy - hh), min(fw - 1.0, cx + hw), min(fh - 1.0, cy + hh)]
