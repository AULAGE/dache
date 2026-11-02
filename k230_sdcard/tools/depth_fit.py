# -*- coding: utf-8 -*-
"""
K230 尺寸法测深标定：用几组"真实距离 vs K230 读数"拟合 FOCAL_PIX / DEPTH_SCALE / DEPTH_OFFSET。
在电脑或树莓派上用 CPython 跑（只用标准库），不是在 K230 上跑。

数据文件（CSV，# 开头是注释），每行一个距离，读数取 calib.py watch 或 K230 终端 "Object:" 行的中位数：
    真实距离mm, D读数mm[, S像素]
    # 例：
    100, 97.5, 226.4
    150, 146.0, 151.2
    ...
S 是 K230 终端 "Object: ... S:xxx.xpx" 里的像素边长（打开新版程序才有）。

用法：
    python3 depth_fit.py data.csv                       # 只有 D：拟合 DEPTH_SCALE / DEPTH_OFFSET
    python3 depth_fit.py data.csv --scale 1.0 --offset 0  # 采数据时板子上的 DEPTH_SCALE / DEPTH_OFFSET
    python3 depth_fit.py data.csv --size 24             # 有 S 列：直接拟合 FOCAL_PIX 和 DEPTH_OFFSET（推荐）

模型：
    D = 物块边长 × FOCAL_PIX / S × DEPTH_SCALE + DEPTH_OFFSET
    有 S 时拟合 Z = A / S + B，得到 FOCAL_PIX = A / 边长（DEPTH_SCALE 设 1），DEPTH_OFFSET = B
    只有 D 时拟合 Z = a × D + b，换算回新的 DEPTH_SCALE / DEPTH_OFFSET
"""
import argparse
import sys


def linfit(xs, ys):
    """最小二乘 y = a x + b，返回 (a, b)"""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        raise ValueError("读数都一样，没法拟合：至少要 2 个不同的距离")
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return a, my - a * mx


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [p for p in line.replace("，", ",").replace("\t", ",").replace(" ", ",").split(",") if p]
            rows.append([float(p) for p in parts])
    return rows


def fit(rows, size_mm=24.0, scale=1.0, offset=0.0):
    """返回 dict：建议值 + 每个点的残差"""
    if len(rows) < 2:
        raise ValueError("至少要 2 个距离（建议 5 个以上，覆盖实际抓取用到的范围）")
    z = [r[0] for r in rows]
    out = {}
    if all(len(r) >= 3 and r[2] > 0 for r in rows):
        inv = [1.0 / r[2] for r in rows]
        A, B = linfit(inv, z)
        out["mode"] = "size"
        out["FOCAL_PIX"] = A / size_mm
        out["DEPTH_SCALE"] = 1.0
        out["DEPTH_OFFSET"] = B
        pred = [A * v + B for v in inv]
    else:
        d = [r[1] for r in rows]
        a, b = linfit(d, z)
        out["mode"] = "depth"
        out["DEPTH_SCALE"] = a * scale
        out["DEPTH_OFFSET"] = a * offset + b
        pred = [a * v + b for v in d]
    out["residual"] = [p - t for p, t in zip(pred, z)]
    out["before"] = [r[1] - r[0] for r in rows]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="K230 尺寸法测深标定")
    ap.add_argument("csv")
    ap.add_argument("--size", type=float, default=24.0, help="物块边长 mm（BLOCK_REAL_SIZE）")
    ap.add_argument("--scale", type=float, default=1.0, help="采数据时板子上的 DEPTH_SCALE")
    ap.add_argument("--offset", type=float, default=0.0, help="采数据时板子上的 DEPTH_OFFSET")
    args = ap.parse_args(argv)
    rows = load(args.csv)
    r = fit(rows, args.size, args.scale, args.offset)
    print("共 %d 个点，拟合方式：%s" % (len(rows), "像素边长 S（推荐）" if r["mode"] == "size" else "深度读数 D"))
    print("  真实mm   读数mm   原误差   拟合后误差")
    for row, e0, e1 in zip(rows, r["before"], r["residual"]):
        print("  %6.1f   %6.1f   %+6.1f   %+6.1f" % (row[0], row[1], e0, e1))
    print("\n建议写进 K230 main.py 配置区：")
    if r["mode"] == "size":
        print("  FOCAL_PIX = %.1f" % r["FOCAL_PIX"])
    print("  DEPTH_SCALE = %.4f" % r["DEPTH_SCALE"])
    print("  DEPTH_OFFSET = %.1f" % r["DEPTH_OFFSET"])
    worst = max(abs(e) for e in r["residual"])
    print("\n拟合后最大误差 %.1f mm%s" % (worst, "（偏大：检查读数是否稳定、物块是否正对镜头）" if worst > 5 else ""))
    if r["mode"] == "size":
        print("注意：改了 FOCAL_PIX 后 X/Y 的比例也跟着变（X = 像素偏移 × 深度 / FOCAL_PIX），"
              "树莓派 calib.json 里按 K230 读数标的 GOAL 要重新标一次。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
