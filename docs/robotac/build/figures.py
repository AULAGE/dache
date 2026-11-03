# -*- coding: utf-8 -*-
"""
画报告和 PPT 用的两张图（PNG）：
  figures/系统总体架构.png        树莓派居中，四路串口，各下位机
  figures/自适应视觉伺服流程.png  测量 → 误差 → u = −α·e/(J·s) → 执行 → 再测 → 学习 J

单独运行：  python figures.py
build_materials.py 在图片不存在或加 --figures 时会调用这里。

中文字体：按顺序找 微软雅黑 / 黑体（Windows）、Noto Sans CJK / 思源黑体、文泉驿；都没有会提示。
图里的内容全部来自代码和 CLAUDE.md（串口号、波特率、帧格式、控制律），不含任何实测数据。
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(HERE, "figures")

# 旧 PPT 主题色（theme1.xml：Mobile Manipulator）
NAVY = "#0F1A26"
ORANGE = "#F07D1A"
BLUE = "#2E6FD8"
GREEN = "#16A37F"
PURPLE = "#8A5CB8"
RED = "#D64545"
GRAY = "#8B97A5"
DGRAY = "#4A5868"
LIGHT = "#F5F7FA"
WHITE = "#FFFFFF"

CJK_CANDIDATES = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Noto Sans SC", "Source Han Sans SC",
                  "Source Han Sans CN", "WenQuanYi Zen Hei", "PingFang SC", "Heiti SC"]


def setup_font():
    names = {f.name for f in font_manager.fontManager.ttflist}
    for n in CJK_CANDIDATES:
        if n in names:
            plt.rcParams["font.sans-serif"] = [n, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return n
    print("⚠️ 没找到中文字体，图里的中文会显示成方框。Windows 自带微软雅黑；Linux 请装 fonts-noto-cjk。")
    return None


def box(ax, x, y, w, h, title, lines=(), fc=WHITE, ec=NAVY, tc=NAVY, lc=DGRAY, title_size=13, line_size=9.5,
        lw=1.6, ls="-", radius=0.02, bold=True):
    p = FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={radius}", fc=fc, ec=ec, lw=lw,
                       ls=ls, zorder=2)
    ax.add_patch(p)
    n = len(lines)
    if n == 0:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=title_size, color=tc,
                weight="bold" if bold else "normal", zorder=3)
        return p
    gap = 0.034
    block = 0.045 + (n - 1) * gap          # 标题到最后一行的高度
    top = y + h / 2 + block / 2
    ax.text(x + w / 2, top, title, ha="center", va="center", fontsize=title_size, color=tc,
            weight="bold" if bold else "normal", zorder=3)
    for i, s in enumerate(lines):
        ax.text(x + w / 2, top - 0.045 - i * gap, s, ha="center", va="center", fontsize=line_size, color=lc,
                zorder=3)
    return p


def arrow(ax, p0, p1, color=NAVY, lw=1.8, style="-|>", rad=0.0, ls="-", both=False):
    a = FancyArrowPatch(p0, p1, arrowstyle=("<|-|>" if both else style), mutation_scale=14, color=color, lw=lw,
                        connectionstyle=f"arc3,rad={rad}", linestyle=ls, zorder=1, shrinkA=2, shrinkB=2)
    ax.add_patch(a)


def label(ax, x, y, s, color=DGRAY, size=8.8, ha="center", va="center", bold=False, bg=None, rot=0):
    kw = {}
    if bg:
        kw["bbox"] = dict(boxstyle="round,pad=0.25", fc=bg, ec="none")
    ax.text(x, y, s, ha=ha, va=va, fontsize=size, color=color, weight="bold" if bold else "normal", zorder=4,
            rotation=rot, **kw)


def new_canvas(w=13.33, h=7.5):
    fig = plt.figure(figsize=(w, h), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    fig.patch.set_facecolor(WHITE)
    return fig, ax


# ------------------------------------------------------------------ 图：系统总体架构
def draw_architecture(path):
    fig, ax = new_canvas()
    ax.text(0.5, 0.955, "系统总体架构：树莓派总控 + 四路串口 + 各下位机", ha="center", va="center", fontsize=17,
            weight="bold", color=NAVY)

    # 中央：树莓派
    cx, cy, cw, ch = 0.335, 0.385, 0.33, 0.30
    p = FancyBboxPatch((cx, cy), cw, ch, boxstyle="round,pad=0,rounding_size=0.02", fc=NAVY, ec=NAVY, lw=2,
                       zorder=2)
    ax.add_patch(p)
    ax.text(cx + cw / 2, cy + ch - 0.035, "树莓派 4  总控（决策层）", ha="center", va="center", fontsize=14.5,
            weight="bold", color=WHITE, zorder=3)
    ax.text(cx + cw / 2, cy + ch - 0.072, "Ubuntu 24.04 · Python 3.12 · 仅依赖标准库 + pyserial", ha="center",
            va="center", fontsize=8.6, color="#C9D1DA", zorder=3)
    layers = [("任务状态机", "搜索 → 对准 → 伸出 → 确认 → 放置 / 重试", ORANGE),
              ("自适应视觉伺服", "u = −α·e/(J·s)，在线学习灵敏度 J", ORANGE),
              ("设备驱动层", "K230 收帧 · 升降轴文本协议 · 机械臂帧 · 底盘 50 Hz", "#2B3A4C"),
              ("安全与记录", "自检 · 停车信号 · 日志 CSV · 网页看板", "#2B3A4C")]
    for i, (t, s, c) in enumerate(layers):
        yy = cy + ch - 0.125 - i * 0.043
        q = FancyBboxPatch((cx + 0.015, yy - 0.017), cw - 0.03, 0.036, boxstyle="round,pad=0,rounding_size=0.008",
                           fc=c, ec="none", zorder=3)
        ax.add_patch(q)
        ax.text(cx + 0.025, yy, t, ha="left", va="center", fontsize=9.5, weight="bold", color=WHITE, zorder=4)
        ax.text(cx + cw - 0.02, yy, s, ha="right", va="center", fontsize=7.8, color=WHITE, zorder=4)

    # 四个下位机
    W, H = 0.25, 0.165
    nodes = {
        "k230": (0.03, 0.70, "K230 视觉模块（腕部）", ["CanMV · 目标检测 + 单目测距", "眼在手上：随机械臂一起运动",
                                                  "无目标时发心跳帧 D = 0"], BLUE),
        "lift": (0.72, 0.70, "升降轴  STM32F103C8T6", ["伺服驱动器 + 丝杠，0–50 cm", "软件限位 44 cm · 最高 80 mm/s",
                                                     "运动中可改目标，停下回 DONE"], GREEN),
        "arm": (0.03, 0.08, "机械臂  Lobot 控制板", ["6 路 PWM 舵机（1 爪 · 2 腕 · 3/4/5 臂 · 6 底座）",
                                                 "脉宽 500–2500 us", "控制板不回传位置"], PURPLE),
        "chassis": (0.72, 0.08, "底盘  WHEELTEC STM32F407", ["阿克曼转向 · 编码器 + IMU", "电机速度闭环（厂家固件）",
                                                           "固件无串口超时 → 树莓派兜底"], RED),
    }
    for k, (x, y, t, ls_, c) in nodes.items():
        box(ax, x, y, W, H, t, ls_, ec=c, tc=c, title_size=11.5, line_size=8.6, lw=2.0)

    # 连线 + 串口标签
    # 箭头从下位机框边缘指向树莓派；串口标签放在框外侧的空白处
    links = [
        ("k230", (0.03 + W * 0.7, 0.70), (cx, cy + ch - 0.06), BLUE, (0.03, 0.655, "left"),
         "uart3（GPIO5）· 115200 · 只收", "AA 55 | X Y D | CRC8 | 0D（10 字节）"),
        ("lift", (0.72 + W * 0.3, 0.70), (cx + cw, cy + ch - 0.06), GREEN, (0.97, 0.655, "right"),
         "/dev/ttyUSB0 · 115200", "文本：A/R/P/E/Z/V → OK/ERR/POS/DONE/READY"),
        ("arm", (0.03 + W * 0.7, 0.08 + H), (cx, cy + 0.06), PURPLE, (0.03, 0.31, "left"),
         "J-Link 虚拟串口 · 9600 · 只发", "55 55 | 长度 | 03 | n | 时间 | (id, 脉宽)×n"),
        ("chassis", (0.72 + W * 0.3, 0.08 + H), (cx + cw, cy + 0.06), RED, (0.97, 0.31, "right"),
         "/dev/ttyACM0 · 115200 · 双向", "下发 11 字节 50 Hz；回传 24 字节约 20 Hz"),
    ]
    for k, p_node, p_pi, c, (lx, ly, ha), l1, l2 in links:
        if k == "k230":
            arrow(ax, p_node, p_pi, color=c, lw=2.2)
        elif k == "chassis":
            arrow(ax, p_pi, p_node, color=c, lw=2.2, both=True)
        else:
            arrow(ax, p_pi, p_node, color=c, lw=2.2)
        label(ax, lx, ly, l1, color=c, size=9.2, ha=ha, bold=True)
        label(ax, lx, ly - 0.03, l2, color=DGRAY, size=8.3, ha=ha)

    # 机构链说明（中间下方）
    ax.text(0.5, 0.245, "机械链：底盘 → 升降平台 → 机械臂 → K230 装在腕部（2 号舵机那一节）",
            ha="center", va="center", fontsize=9.5, color=NAVY, weight="bold")
    ax.text(0.5, 0.21, "误差分工：竖直 y → 升降轴　方位 φ → 机械臂底座　深度 d → 底盘 / 伸出姿态插值",
            ha="center", va="center", fontsize=9, color=DGRAY)
    ax.text(0.5, 0.175, "各下位机只做本机构的实时控制（脉冲、插补、速度闭环），树莓派只发“去哪里、多快”",
            ha="center", va="center", fontsize=9, color=DGRAY)

    # 扩展（虚线）
    box(ax, 0.335, 0.04, 0.33, 0.09, "扩展功能（独立运行，不在抓取闭环内）",
        ["RDK X5：AI 语音交互 / 红外家电 / 视频通话　·　自研环境感知板"], fc=LIGHT, ec=GRAY, tc=DGRAY,
        title_size=9.5, line_size=8.2, ls="--", lw=1.3)
    fig.savefig(path, dpi=200, facecolor=WHITE)
    plt.close(fig)


# ------------------------------------------------------------------ 图：自适应视觉伺服流程
def draw_servo(path):
    fig, ax = new_canvas()
    ax.text(0.5, 0.955, "眼在手上的自适应视觉伺服：边对准，边学习执行机构的灵敏度", ha="center", va="center",
            fontsize=17, weight="bold", color=NAVY)

    bw, bh = 0.205, 0.17
    # 环形布局：上排 测量 → 误差 → 判断；下排 学习 ← 执行 ← 计算动作
    P = {
        "meas": (0.05, 0.68),
        "err": (0.31, 0.68),
        "tol": (0.57, 0.68),
        "cmd": (0.57, 0.37),
        "act": (0.31, 0.37),
        "learn": (0.05, 0.37),
    }
    box(ax, *P["meas"], bw, bh, "① 测量", ["动作后等 0.25 s 画面稳定", "取 5 帧中位数（远时 3 帧）",
                                          "K230：方位 φ、竖直 y、深度 d"], ec=BLUE, tc=BLUE)
    box(ax, *P["err"], bw, bh, "② 误差 e", ["e = 读数 − 目标", "目标随深度插值", "（物块越远在画面里越靠上）"],
        ec=NAVY, tc=NAVY)
    box(ax, *P["tol"], bw, bh, "③ 到位了吗？", ["方位 1.5° · 竖直 4 mm · 深度 10 mm", "在容差内 → 伸出抓取",
                                              "否则 → 计算下一步"], ec=NAVY, tc=NAVY)
    # 计算动作（橙色强调）
    p = FancyBboxPatch((P["cmd"][0], P["cmd"][1]), bw, bh, boxstyle="round,pad=0,rounding_size=0.02", fc=ORANGE,
                       ec=ORANGE, lw=2, zorder=2)
    ax.add_patch(p)
    x0, y0 = P["cmd"]
    ax.text(x0 + bw / 2, y0 + bh - 0.03, "④ 计算动作", ha="center", va="center", fontsize=13, weight="bold",
            color=WHITE, zorder=3)
    ax.text(x0 + bw / 2, y0 + bh / 2 - 0.005, "u = −α · e / (J · s)", ha="center", va="center", fontsize=16,
            weight="bold", color=WHITE, zorder=3, family="DejaVu Sans")
    ax.text(x0 + bw / 2, y0 + 0.028, "α = 0.85：只走预测量的 85%", ha="center", va="center", fontsize=9,
            color=WHITE, zorder=3)
    box(ax, *P["act"], bw, bh, "⑤ 执行", ["y → 升降轴", "φ → 机械臂底座舵机", "d → 底盘（无底盘时伸出姿态插值）"],
        ec=GREEN, tc=GREEN)
    # 学习（橙色描边）
    box(ax, *P["learn"], bw, bh, "⑥ 再测并学习 J", ["Ĵ = Δe / (u · s)", "J ← (1 − β)·J + β·Ĵ，β = 0.3",
                                                  "存文件，下次开机直接用"], ec=ORANGE, tc=ORANGE, lw=2.4)

    def mid(k, side):
        x, y = P[k]
        return {"r": (x + bw, y + bh / 2), "l": (x, y + bh / 2), "t": (x + bw / 2, y + bh),
                "b": (x + bw / 2, y)}[side]

    arrow(ax, mid("meas", "r"), mid("err", "l"), lw=2.2)
    arrow(ax, mid("err", "r"), mid("tol", "l"), lw=2.2)
    arrow(ax, mid("tol", "b"), mid("cmd", "t"), lw=2.2)
    label(ax, P["tol"][0] + bw / 2 + 0.012, (P["tol"][1] + P["cmd"][1] + bh) / 2, "否", size=10, ha="left",
          bold=True, color=NAVY)
    arrow(ax, mid("cmd", "l"), mid("act", "r"), lw=2.2)
    arrow(ax, mid("act", "l"), mid("learn", "r"), lw=2.2)
    arrow(ax, mid("learn", "t"), mid("meas", "b"), lw=2.2, color=ORANGE)
    label(ax, P["learn"][0] + bw / 2 + 0.012, (P["learn"][1] + bh + P["meas"][1]) / 2, "闭环", size=10,
          ha="left", bold=True, color=ORANGE)

    # 是 → 抓取
    gx, gy = 0.835, 0.68
    box(ax, gx, gy, 0.135, bh, "抓取", ["张爪 → 伸出", "合爪 → 观察确认", "没抓到 → 重试"], fc=NAVY, ec=NAVY,
        tc=WHITE, lc="#C9D1DA", line_size=9)
    arrow(ax, mid("tol", "r"), (gx, gy + bh / 2), lw=2.2)
    label(ax, (P["tol"][0] + bw + gx) / 2, gy + bh / 2 + 0.022, "是", size=10, bold=True, color=NAVY)

    # 右下：符号说明 / 学习保护
    nx, ny, nw, nh = 0.80, 0.33, 0.185, 0.25
    box(ax, nx, ny, nw, nh, "", [], fc=LIGHT, ec=GRAY, lw=1.0)
    rows = [("J", "灵敏度：单位动作引起的读数变化"), ("s", "已知几何系数，如 s = 1 + R/d"),
            ("α", "步长系数 0.85，宁可多一步"), ("β", "学习率 0.3")]
    ax.text(nx + 0.012, ny + nh - 0.03, "符号", fontsize=10, weight="bold", color=NAVY, va="center")
    for i, (a, b) in enumerate(rows):
        yy = ny + nh - 0.07 - i * 0.045
        ax.text(nx + 0.015, yy, a, fontsize=11, weight="bold", color=ORANGE, va="center", family="DejaVu Sans")
        ax.text(nx + 0.04, yy, b, fontsize=8.3, color=DGRAY, va="center")

    # 底部：学习保护 + 自身干扰保护
    box(ax, 0.05, 0.05, 0.43, 0.2, "学习的三道保护", ["动作太小 / 读数变化 < 2 倍噪声 → 不学",
                                                     "Ĵ 与初值符号相反（噪声、打滑、换了目标）→ 不学",
                                                     "|Ĵ| 限制在初值的 1/4 ~ 4 倍"], fc=LIGHT, ec=ORANGE, tc=ORANGE,
        title_size=11, line_size=9, lw=1.3)
    box(ax, 0.52, 0.05, 0.43, 0.2, "测量层的保护", ["连续两次动作后读数几乎不变 → 判为“看到了自己的爪子”，停止",
                                                    "连心跳都收不到 → 判为断线，报错停止",
                                                    "升降到最低 / 最高仍差 → 判为够不着，中止"], fc=LIGHT, ec=NAVY,
        tc=NAVY, title_size=11, line_size=9, lw=1.3)
    fig.savefig(path, dpi=200, facecolor=WHITE)
    plt.close(fig)


FIGURES = {
    "系统总体架构.png": draw_architecture,
    "自适应视觉伺服流程.png": draw_servo,
}


def build_all(force=True, out_dir=FIG_DIR):
    os.makedirs(out_dir, exist_ok=True)
    font = setup_font()
    out = {}
    for name, fn in FIGURES.items():
        path = os.path.join(out_dir, name)
        if force or not os.path.exists(path):
            fn(path)
            print(f"  画图：{name}（字体 {font}）")
        out[name] = path
    return out


if __name__ == "__main__":
    build_all(force=True)
