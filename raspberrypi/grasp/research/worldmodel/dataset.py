"""
合成数据集（任务 14，合成数据）：自主探索策略 → (o_t, u_t, o_{t+1}, 真实舵机角, 真实掩膜) 序列。

执行器：复用 research/sim_world.py 的 play()（死区 δ + 换向回差 b，和 sim.SimArm._servo_base 同一规则），
每个序列一台"车"：J 固定（一台车的灵敏度不变，见报告），回差 b、死区 δ 按 sim_world.CAR_RANGES 抽，
底座再乘一个 0.5~2 的放大倍数（覆盖"人为加大回差"），用来按回差大小分组评估。

探索策略（不看画面、不用真值）：底座随机游走（35% 概率换向，步长 0.3~4°，限制 ±12°），
升降轴一半的步动一下（0.05~0.6cm，有惯性地换向，限制 0~6cm），爪子 10% 概率张 / 合切换。

真值（真实输出角、高度、掩膜、b、δ）只用于评估，训练不用。
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_GRASP = os.path.dirname(os.path.dirname(_HERE))
if _GRASP not in sys.path:
    sys.path.insert(0, _GRASP)

from research.sim_world import CAR_RANGES, EXEC_SIGMA, play  # noqa: E402
from research.worldmodel.render import IMG, Scene  # noqa: E402

J_X, J_Y = -1.0, 6.0        # 画面° / 底座°、mm / cm（真机标定约 −0.6 和 6；底座取 −1 让小图里位移大一点）
BASE_LIM = 12.0
LIFT_LIM = (0.0, 6.0)
U_DIM = 5                   # 动作特征：[底座命令/10, 升降命令/3, Δ底座命令/3, Δ升降命令/0.5, 爪子(合=1)]


def make_sequence(rng, T, n=IMG, hyst_scale=None):
    sc = Scene(rng, J_X, J_Y, n)
    hs = rng.uniform(0.5, 2.0) if hyst_scale is None else hyst_scale
    b = {"x": rng.uniform(*CAR_RANGES["x"]["b"]) * hs, "y": rng.uniform(*CAR_RANGES["y"]["b"])}
    dl = {"x": rng.uniform(*CAR_RANGES["x"]["delta"]) * hs, "y": rng.uniform(*CAR_RANGES["y"]["delta"])}
    # 上电时输出轴在间隙里的位置未知
    lift0 = rng.uniform(1.0, 5.0)
    m = {"x": 0.0, "y": lift0}
    o = {"x": rng.uniform(-b["x"] / 2, b["x"] / 2), "y": lift0 + rng.uniform(-b["y"] / 2, b["y"] / 2)}
    cmd = {"x": 0.0, "y": lift0}
    claw = "open" if rng.random() < 0.5 else "close"
    dir_x = 1.0 if rng.random() < 0.5 else -1.0
    dir_y = 1.0 if rng.random() < 0.5 else -1.0

    obs = np.zeros((T + 1, n, n, 3), np.uint8)
    mask = np.zeros((T + 1, n, n), np.uint8)
    u = np.zeros((T, U_DIM), np.float32)
    cmd_hist = np.zeros((T + 1, 2), np.float32)       # 第 t 帧拍下时的命令（底座°、升降 cm）
    true = np.zeros((T + 1, 2), np.float32)           # 第 t 帧拍下时的真实输出
    clawv = np.zeros(T + 1, np.float32)

    def shoot(t):
        ex = rng.normal(0, EXEC_SIGMA["x"])
        ey = rng.normal(0, EXEC_SIGMA["y"])
        th, hh = o["x"] + ex, o["y"] + ey
        img, lab = sc.render(th, hh, claw, gain=rng.uniform(0.85, 1.15))
        obs[t] = (img * 255 + 0.5).astype(np.uint8)
        mask[t] = lab
        cmd_hist[t] = (cmd["x"], cmd["y"])
        true[t] = (th, hh)
        clawv[t] = 1.0 if claw == "close" else 0.0

    shoot(0)
    for t in range(T):
        if rng.random() < 0.35:
            dir_x = -dir_x
        cx = cmd["x"] + dir_x * rng.uniform(0.3, 4.0)
        if abs(cx) > BASE_LIM:
            dir_x = -math.copysign(1.0, cx)
            cx = max(-BASE_LIM, min(BASE_LIM, cx))
        cy = cmd["y"]
        if rng.random() < 0.5:
            if rng.random() < 0.3:
                dir_y = -dir_y
            cy = cmd["y"] + dir_y * rng.uniform(0.05, 0.6)
            if not LIFT_LIM[0] <= cy <= LIFT_LIM[1]:
                dir_y = -dir_y
                cy = max(LIFT_LIM[0], min(LIFT_LIM[1], cy))
        if rng.random() < 0.1:
            claw = "close" if claw == "open" else "open"
        u[t] = (cx / 10.0, cy / 3.0, (cx - cmd["x"]) / 3.0, (cy - cmd["y"]) / 0.5, 1.0 if claw == "close" else 0.0)
        cmd["x"], cmd["y"] = cx, cy
        for a in ("x", "y"):
            m[a], o[a] = play(m[a], o[a], cmd[a], b[a], dl[a])
        sc.step_distractors()
        shoot(t + 1)
    return {"obs": obs, "mask": mask, "u": u, "cmd": cmd_hist, "true": true, "claw": clawv,
            "b": np.array([b["x"], b["y"]], np.float32), "delta": np.array([dl["x"], dl["y"]], np.float32),
            "hand": sc.hand is not None, "block2": sc.block2 is not None}


def make_dataset(n_seq, T, seed, n=IMG):
    rng = np.random.default_rng(seed)
    seqs = [make_sequence(rng, T, n) for _ in range(n_seq)]
    out = {}
    for k in ("obs", "mask", "u", "cmd", "true", "claw", "b", "delta"):
        out[k] = np.stack([s[k] for s in seqs])
    out["hand"] = np.array([s["hand"] for s in seqs])
    out["block2"] = np.array([s["block2"] for s in seqs])
    return out


def write_png(path, img):
    """uint8 [H, W, 3] → PNG（只用标准库 zlib，不依赖图像库）"""
    import struct
    import zlib
    h, w = img.shape[:2]

    def chunk(tag, body):
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + np.ascontiguousarray(img[y]).tobytes() for y in range(h))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def save_preview(path, data, n_seq=4, steps=8):
    """把几条序列（上：画面，下：真实掩膜）拼成一张 PNG，方便肉眼检查渲染"""
    rows = []
    for i in range(min(n_seq, len(data["obs"]))):
        frames = [data["obs"][i, t] for t in range(min(steps, data["obs"].shape[1]))]
        masks = [np.stack([data["mask"][i, t] * 60] * 3, -1).astype(np.uint8) for t in range(len(frames))]
        rows.append(np.concatenate(frames, 1))
        rows.append(np.concatenate(masks, 1))
    write_png(path, np.concatenate(rows, 0))


if __name__ == "__main__":
    import time
    t0 = time.time()
    d = make_dataset(8, 16, 0)
    print("8 条 × 17 帧", {k: v.shape for k, v in d.items()}, f"{time.time() - t0:.2f}s")
    r = d["true"][..., 0] - d["cmd"][..., 0]
    print("底座残差（真实 − 命令）均值 %.2f°，|残差| 均值 %.2f°，最大 %.2f°" % (r.mean(), np.abs(r).mean(), np.abs(r).max()))
    os.makedirs(os.path.join(_HERE, "results"), exist_ok=True)
    save_preview(os.path.join(_HERE, "results", "preview.png"), d)
