# -*- coding: utf-8 -*-
"""
把 K230 存图（任务 12）退回原始格式时存的 .rgb 文件转成 PNG。在电脑 / 树莓派上用 CPython 跑，只用标准库。

.rgb 文件 = 平面 RGB：先 w*h 字节 R，再 w*h 字节 G，再 w*h 字节 B（和 K230 送进 AI 的 RGBP888 一样，旋转前）。
宽高从会话目录里的 info.txt 读（img_w / img_h），也可以用 --size 指定。

用法：
    python3 raw2png.py /path/to/datalog/s0001            # 把会话里所有 .rgb 转成同名 .png（原文件保留）
    python3 raw2png.py a.rgb b.rgb --size 320x180
"""
import argparse
import os
import struct
import sys
import zlib


def read_info(session_dir):
    """info.txt -> dict（key=value 每行一个）"""
    info = {}
    p = os.path.join(session_dir, "info.txt")
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            for line in f:
                if "=" in line:
                    k, v = line.rstrip("\n").split("=", 1)
                    info[k.strip()] = v.strip()
    return info


def planar_to_png_bytes(raw, w, h):
    """平面 RGB 字节 -> PNG 文件内容（8 位 RGB，无滤波）"""
    n = w * h
    if len(raw) != 3 * n:
        raise ValueError("大小不对：%d 字节，%dx%d 应为 %d" % (len(raw), w, h, 3 * n))
    r, g, b = raw[:n], raw[n:2 * n], raw[2 * n:]
    rows = bytearray()
    for y in range(h):
        rows.append(0)
        o = y * w
        line = bytearray(3 * w)
        line[0::3] = r[o:o + w]
        line[1::3] = g[o:o + w]
        line[2::3] = b[o:o + w]
        rows += line

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(bytes(rows), 6)) + chunk(b"IEND", b""))


def convert(path, w, h):
    with open(path, "rb") as f:
        raw = f.read()
    out = os.path.splitext(path)[0] + ".png"
    with open(out, "wb") as f:
        f.write(planar_to_png_bytes(raw, w, h))
    return out


def main(argv):
    ap = argparse.ArgumentParser(description="K230 .rgb（平面 RGB）转 PNG")
    ap.add_argument("paths", nargs="+", help="会话目录或 .rgb 文件")
    ap.add_argument("--size", help="宽x高，例如 320x180（默认读会话目录的 info.txt）")
    a = ap.parse_args(argv)
    size = tuple(int(v) for v in a.size.lower().split("x")) if a.size else None
    n = 0
    for p in a.paths:
        if os.path.isdir(p):
            info = read_info(p)
            wh = size or (int(info.get("img_w", 320)), int(info.get("img_h", 180)))
            for root, _, files in os.walk(p):
                for fn in sorted(files):
                    if fn.endswith(".rgb"):
                        convert(os.path.join(root, fn), *wh)
                        n += 1
        else:
            if size is None:
                info = read_info(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(p)))))
                wh = (int(info.get("img_w", 320)), int(info.get("img_h", 180)))
            else:
                wh = size
            convert(p, *wh)
            n += 1
    print("转换了 %d 个文件" % n)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
