# =====================================================================
#  vision_locator.py —— K230 目标识别与空间定位
#  当前目标：带颜色的正方形小物块（可扩展到其他物体，见 OBJECT_SIZES_MM）
#  流程：检测 -> 颜色边缘精修 -> 跟踪/平滑 -> 畸变校正 -> 测距定位 -> 串口发送
# =====================================================================
import os
import ujson
import aicube
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import gc
import math
import time
from machine import UART
from machine import FPIOA

# ============================ 画面旋转 ============================
# ROTATE_DEG: 0 = 不旋转, 90 = 顺时针 90°, 270 = 逆时针 90°
# 只旋转视频层，OSD 文字不旋转。画面转反了就把 VO_FLAG_INVERT 改成 True。
ROTATE_DEG = 90
VO_FLAG_INVERT = False
POSITION_IN_ROTATED_FRAME = True   # 发给 STM32 的 X/Y 使用正向画面坐标系（X 右，Y 下）

# ============================ 物理参数 ============================
BLOCK_REAL_SIZE = 24.0    # 默认物体边长（毫米）
# 按类别名单独设置尺寸，以后识别多种物体时在这里加，例如 {"red": 24.0, "ball": 40.0}
OBJECT_SIZES_MM = {}
FOCAL_PIX = 944.0         # 焦距（像素，1280x720 AI 分辨率），用标定模式测出
dist_coeffs = [-0.2, 0.1, 0.0, 0.0, 0.0]   # [k1,k2,p1,p2,k3]，未标定请关掉 USE_UNDISTORT
USE_UNDISTORT = True
DEPTH_SCALE = 1.0         # 实测深度 = 计算深度 × DEPTH_SCALE + DEPTH_OFFSET
DEPTH_OFFSET = 0.0
CALIB_DISTANCE_MM = 0     # 焦距标定：>0 时进入标定模式（物块正对镜头放在画面中央）

# ============================ 识别稳定性 ============================
START_SCORE_EXTRA = 0.10  # 新目标置信度 >= 配置阈值 + 该值
KEEP_SCORE_RATIO = 0.75   # 已锁定目标置信度 >= 配置阈值 × 该值
ASPECT_MIN = 0.55         # 宽高比范围（正方形物块）
ASPECT_MAX = 1.80
MIN_BOX_PIX = 10

CONFIRM_FRAMES = 2
LOST_TOLERANCE = 6
TRACK_IOU = 0.15
TRACK_DIST = 0.8

SMOOTH_WINDOW = 3         # 中值滤波窗口（帧）。帧率低时窗口越大越滞后
REF_FRAME_MS = 33.0       # 下面的平滑系数按约 30fps 调定，实际帧率不同时自动换算
ALPHA_MIN = 0.25
ALPHA_MAX = 0.85
MOTION_REF = 0.12
DEADBAND_PIX = 0.6
CLASS_VOTE_WINDOW = 7

# ============================ 颜色辅助（纯色物体） ============================
# 颜色边缘精修：用物块自身颜色在检测框附近找真实边缘，框更贴合，测距更准
ENABLE_COLOR_REFINE = True
REFINE_EXPAND = 0.25      # 搜索范围：检测框向外扩 25%
REFINE_EXPAND_TIGHT = 0.06  # 第一次失败后收紧搜索范围重试（分开紧挨着的同色物块）
REFINE_MIN_CONTRAST = 45  # 物块与背景颜色差太小时不精修
REFINED_ASPECT_MAX = 1.70 # 精修结果的宽高比上限（被部分遮挡时可见部分会变成长方形，
                          # 此时取长边测距仍然正确，所以不能卡得太死）
REFINE_MAX_OUTSIDE = 0.12 # 精修框任一边超出检测框不能超过检测框尺寸的该比例（防止连上旁边的同色物体）
REFINE_GRID = 48          # 搜索区域降采样到约 48x48（精度由亚像素精修补回）
REFINE_MIN_CHROMA = 25    # 颜色不够鲜艳（灰/白/黑）时不精修
REFINE_MIN_INSIDE = 0.80  # 精修框至少有这么多面积落在（外扩后的）检测框内
REFINE_AREA_RANGE = (0.30, 1.60)   # 精修框面积 / 检测框面积 的合理范围（检测框偏松或偏紧都能接受）
REFINE_MIN_FILL = 0.55    # 精修框内同色像素占比下限
# 颜色一致性：已锁定目标只和颜色相近的检测框关联，防止跳到别的颜色的物块
SHADE_TOLERANT = True     # 颜色比较以色度为主：立方体亮面/暗面、阴影里的同色部分都算同色
SHADE_BRIGHT_WEIGHT = 0.35  # 亮度差的权重（越小越容忍明暗变化）
SHADE_DARK_RATIO = 0.40   # 比物块颜色暗到该比例以下（接近黑）的像素不算同色
ENABLE_COLOR_GATE = True
COLOR_GATE = 90           # RGB 各通道差的和超过该值视为不同物体
# 旋转框（最小面积外接矩形）：物块在平面内转动时，量的是真实边长而不是外接正框，
# 测距不再偏近，同时输出物块转角
ENABLE_OBB = True
OBB_MIN_FILL = 0.70       # 旋转框内同色像素占比下限（正方形贴合时接近 1）
OBB_MIN_POINTS = 20       # 同色像素点太少不拟合
ANGLE_PERIOD = 90.0       # 正方形旋转 90° 后外观相同，角度范围 [0, 90)
OBB_HINT_RANGE = 6.0      # 已锁定目标只在上一帧角度 ±该值 内搜索（大幅减少计算）
# 亚像素边缘精修：沿旋转框四条边的法线在全分辨率图像上找颜色跳变点，
# 阈值取“内侧颜色”和“外侧背景”的中点（自适应光照），线性插值到亚像素
ENABLE_SUBPIXEL = True
SUBPIX_PROFILES = (-0.28, 0.0, 0.28)   # 每条边上的采样线位置（相对边长，避开角点）
SUBPIX_WHEN_MOVING = False  # 物块移动中跳过亚像素精修（省时间、更跟手），停下后自动恢复
SUBPIX_MIN_CONTRAST = 30  # 内外颜色差太小（边缘不清）的采样线不用
SUBPIX_MAX_SHIFT = 0.15   # 单边修正量超过边长的该比例视为异常，放弃精修
ENABLE_ANGLE_REFINE = True  # 用亚像素边缘点拟合各边斜率，修正转角（突破 0.5° 搜索步长）
ANGLE_REFINE_MAX_DEG = 2.0  # 单次修正量上限，超过视为异常不修正
ANGLE_REFINE_MIN_SIDE = 60  # 边长（像素）小于该值时边太短、斜率噪声大，不做角度修正

# ============================ 通信 ============================
EDGE_MARGIN = 4
UART_INTERVAL_MS = 100

# ============================ 调试 ============================
# 分阶段耗时统计：每 PROFILE_EVERY 帧在终端打印一次各阶段平均耗时，用来找性能瓶颈
DEBUG_TIMING = True
PROFILE_EVERY = 30
GC_EVERY = 3              # 每几帧做一次内存回收（每帧都做比较耗时）
# 目标丢失时是否发送 X=0 Y=0 D=0 的“丢失帧”，让 STM32 知道没有目标（需 STM32 端配合）
SEND_LOST_FRAME = False
# 心跳：超过 HEARTBEAT_MS 没发过位置帧（没目标 / 目标贴边 / 跟踪预测中）就发一帧 X=Y=D=0。
# 树莓派据此区分"没看到物块"和"串口线断了 / K230 程序停了"
SEND_HEARTBEAT = True
HEARTBEAT_MS = 200

# ============================ 审查修复 / 提速开关（云端任务 04） ============================
# 说明见 docs/cloud_tasks/results/04_报告.md。每项都能单独开关，方便在板子上逐项对比。
# --- bug 修复：默认 False = 保持原来的行为；真机确认后再逐项打开 ---
FIX_OBB_ANGLE_WRAP = False  # 旋转框角度跨过 0°/90° 边界时宽高互换（原来只取模不互换，长方形目标会被转 90°，测深可差 30%）
FIX_STALE_OBB = False       # 某帧没拟合出旋转框时，沿用的旧旋转框跟着正框平移/缩放（原来原地不动，最多发 3 帧旧位置）
OBB_HOLD_FRAMES = 3         # 连续几帧拟合不到旋转框后退回正框（原来写死 3）
FIX_DEPTH_NONZERO = False   # 算出的深度 < 1mm 时不发位置帧（原来会被截成 D=0，树莓派当成心跳）
# --- 提速：结果完全不变的默认打开；会改变结果的默认保持原值 ---
FAST_OBB_POINTS = True      # fit_obb 先用布尔下标挑出掩膜内的点再搜角度（结果不变；ulab 不支持布尔下标时自动退回原算法）
CACHE_CENTER_COLOR = True   # 同一帧里同一个框的 center_color 只算一次（select 颜色门限和颜色精修共用，结果不变）
OSD_EVERY = 1               # 每几帧画一次 OSD（1 = 每帧画，原行为）。车上没人看屏幕时可设 3~5，只影响显示，不影响串口
PRINT_OBJECT = True         # 每次发位置帧时在终端打印 "Object: ..."（False 省一点时间，不影响串口）
GC_MIN_FREE_KB = 0          # >0 时改为"空闲内存低于该值才 gc"（替代 GC_EVERY），0 = 原行为
REFINE_GRID_CEIL = False    # 颜色精修降采样步长向上取整，保证网格不超过 REFINE_GRID（原来向下取整，小目标网格可达 95x95）
PROFILE_DETAIL = False      # Profiler 额外统计颜色精修内部各步（center_color / mask / fit_obb / subpixel）耗时

# ============================ 忽略区域（爪子脚，云端任务 10） ============================
# 相机和爪子装在同一节上，爪子在画面里的位置永远不变。中心落在这些区域里的检测框在进跟踪器之前直接丢掉；
# 跟踪器已经锁定的目标一旦进入区域也立即放弃 —— 跟踪器就不会再锁住爪子脚。
# 坐标：0~1 归一化，基于 IDE 里看到的（旋转后的）画面：左上角 (0, 0)，右下角 (1, 1)，
#       每项 (左, 上, 右, 下)。OSD 上画的红色虚线框就是这里的区域，和 IDE 画面对照着调。
# 说明见 docs/cloud_tasks/results/10_报告.md
IGNORE_REGIONS_ENABLE = True
IGNORE_REGIONS = [
    (0.00, 0.40, 0.12, 0.60),   # 爪子脚：画面左侧中间（2026-11-03 在 IDE 里目测约 1/60 面积，区域取得略大，约 1/40）
]
IGNORE_DRAW = True              # 在 OSD 上用红色虚线框画出忽略区域
IGNORE_CALIB_MODE = False       # True：终端持续打印每个检测框的归一化中心和大小（不放物块时看爪子脚被识别成什么样）
IGNORE_CALIB_EVERY_MS = 200     # 标定打印的最小间隔（毫秒）

# ============================ 显示 ============================
display_mode = "hdmi"

# ---------------------------------------------------------------------
def ALIGN_UP(x, align):
    return (x + align - 1) & ~(align - 1)

def ALIGN_DOWN(x, align):
    return x & ~(align - 1)

if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1280, 16)
OUT_RGB888P_HEIGHT = 720

ROTATED = ROTATE_DEG in (90, 270)
if ROTATED:
    CHN0_W = ALIGN_DOWN(DISPLAY_HEIGHT, 16)
    CHN0_H = ALIGN_UP(CHN0_W * OUT_RGB888P_HEIGHT // OUT_RGB888P_WIDTH, 8)
    VID_W, VID_H = CHN0_H, CHN0_W
else:
    CHN0_W, CHN0_H = DISPLAY_WIDTH, DISPLAY_HEIGHT
    VID_W, VID_H = CHN0_W, CHN0_H
VID_X = (DISPLAY_WIDTH - VID_W) // 2
VID_Y = (DISPLAY_HEIGHT - VID_H) // 2
# OSD 图层只覆盖画面区域（旋转后两侧是黑边，不需要绘制），每帧清空/显示的数据量减少约 2/3
OSD_X, OSD_Y, OSD_W, OSD_H = VID_X, VID_Y, VID_W, VID_H

root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"

NP_FLOAT = getattr(np, "float", None) or getattr(np, "float32")

fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)  # ⚠️ 请确认引脚
uart = UART(UART.UART1, 115200, 8, 0, 1, timeout=1000)

# ------------------------------ 通用工具 ------------------------------
def two_side_pad_param(input_size, output_size):
    ratio = min(output_size[0] / input_size[0], output_size[1] / input_size[1])
    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])
    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2
    return (int(round(dh - 0.1)), int(round(dh + 0.1)),
            int(round(dw - 0.1)), int(round(dw + 0.1)), ratio)

def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
            crc &= 0xFF
    return crc

def is_stop(e):
    """是否为“停止运行”信号：IDE 停止按钮会抛出 Exception('IDE interrupt')，
    Ctrl+C 是 KeyboardInterrupt。所有 except 都必须放行它，否则程序停不下来"""
    return isinstance(e, KeyboardInterrupt) or "IDE interrupt" in str(e)

def _exitpoint(flag=None):
    """CanMV 的 IDE 停止检查点（老固件没有该接口时忽略）"""
    try:
        if flag is None:
            os.exitpoint()
        else:
            os.exitpoint(flag)
    except AttributeError:
        pass

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def median(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n//2 - 1] + s[n//2]) / 2.0

def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = ix2 - ix1, iy2 - iy1
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0

def refine_box_ok(nb, det_box):
    """精修框是否可信：大部分落在检测框（外扩 10%）内，且面积比例合理"""
    dw, dh = det_box[2] - det_box[0], det_box[3] - det_box[1]
    ex = [det_box[0] - 0.1 * dw, det_box[1] - 0.1 * dh, det_box[2] + 0.1 * dw, det_box[3] + 0.1 * dh]
    ix = min(nb[2], ex[2]) - max(nb[0], ex[0])
    iy = min(nb[3], ex[3]) - max(nb[1], ex[1])
    na = (nb[2] - nb[0]) * (nb[3] - nb[1])
    da = dw * dh
    if ix <= 0 or iy <= 0 or na <= 0 or da <= 0:
        return False
    if ix * iy / na < REFINE_MIN_INSIDE:
        return False
    if (det_box[0] - nb[0] > REFINE_MAX_OUTSIDE * dw or nb[2] - det_box[2] > REFINE_MAX_OUTSIDE * dw or
            det_box[1] - nb[1] > REFINE_MAX_OUTSIDE * dh or nb[3] - det_box[3] > REFINE_MAX_OUTSIDE * dh):
        return False
    return REFINE_AREA_RANGE[0] <= na / da <= REFINE_AREA_RANGE[1]

def color_dist(a, b):
    """
    颜色距离（单位与 RGB 各通道差之和相当）。
    SHADE_TOLERANT 时以色度为主、亮度为辅：立方体亮面/暗面、阴影中的同色物块仍判为同色；
    但比参考色暗很多（接近黑）的像素额外加罚，避免阴影被当成物块
    """
    if not SHADE_TOLERANT:
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])
    sa = a[0] + a[1] + a[2] + 1.0
    sb = b[0] + b[1] + b[2] + 1.0
    chroma = (abs(a[0] / sa - b[0] / sb) + abs(a[1] / sa - b[1] / sb) +
              abs(a[2] / sa - b[2] / sb)) * sb
    bright = SHADE_BRIGHT_WEIGHT * abs(sa - sb) / 3.0
    dark = max(0.0, SHADE_DARK_RATIO * sb - sa) * 2.0
    return chroma + bright + dark

def color_dist_map(patch, color):
    """color_dist 的数组版本：patch 为 (3, h, w) 浮点数组，返回 (h, w) 距离图"""
    if not SHADE_TOLERANT:
        return abs(patch[0] - color[0]) + abs(patch[1] - color[1]) + abs(patch[2] - color[2])
    sb = color[0] + color[1] + color[2] + 1.0
    s = patch[0] + patch[1] + patch[2] + 1.0
    chroma = (abs(patch[0] / s - color[0] / sb) + abs(patch[1] / s - color[1] / sb) +
              abs(patch[2] / s - color[2] / sb)) * sb
    bright = abs(s - sb) * (SHADE_BRIGHT_WEIGHT / 3.0)
    dark = np.clip((SHADE_DARK_RATIO * sb - s) * 2.0, 0.0, 1e6)
    return chroma + bright + dark

# ------------------------------ 坐标旋转 ------------------------------
def rotate_box(x1, y1, x2, y2, frame_w, frame_h):
    if ROTATE_DEG == 90:      # 顺时针: (x, y) -> (H - y, x)
        return frame_h - y2, x1, frame_h - y1, x2
    elif ROTATE_DEG == 270:   # 逆时针: (x, y) -> (y, W - x)
        return y1, frame_w - x2, y2, frame_w - x1
    return x1, y1, x2, y2

def rotate_point(x, y, frame_w, frame_h):
    if ROTATE_DEG == 90:
        return frame_h - y, x
    elif ROTATE_DEG == 270:
        return y, frame_w - x
    return x, y

def unrotate_point(px, py, frame_w, frame_h):
    """rotate_point 的逆变换：显示（旋转后）画面坐标 -> 原始 AI 画面坐标（frame_w x frame_h）"""
    if ROTATE_DEG == 90:      # (x, y) -> (H - y, x)
        return py, frame_h - px
    elif ROTATE_DEG == 270:   # (x, y) -> (y, W - x)
        return frame_w - py, px
    return px, py

def disp_size_of(frame_w, frame_h):
    """显示（旋转后）画面的宽高"""
    return (frame_h, frame_w) if ROTATED else (frame_w, frame_h)

def raw_to_disp_norm(x, y, frame_w, frame_h):
    """原始 AI 画面像素 -> 显示画面的 0~1 归一化坐标（IDE 里看到的方向）"""
    dw, dh = disp_size_of(frame_w, frame_h)
    px, py = rotate_point(x, y, frame_w, frame_h)
    return px / dw, py / dh

def ignore_region_to_raw(region, frame_w, frame_h):
    """忽略区域（显示画面归一化 (左, 上, 右, 下)）-> 原始 AI 画面像素矩形 (x1, y1, x2, y2)"""
    dw, dh = disp_size_of(frame_w, frame_h)
    u1, v1, u2, v2 = region
    a = unrotate_point(min(u1, u2) * dw, min(v1, v2) * dh, frame_w, frame_h)
    b = unrotate_point(max(u1, u2) * dw, max(v1, v2) * dh, frame_w, frame_h)
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]))

def ignore_rects_raw(frame_w, frame_h):
    """当前配置下要忽略的原始像素矩形列表（关掉开关时为空）"""
    if not IGNORE_REGIONS_ENABLE:
        return []
    return [ignore_region_to_raw(r, frame_w, frame_h) for r in IGNORE_REGIONS]

def in_ignore(cx, cy, rects):
    """原始像素点是否落在任一忽略矩形里（含边界）"""
    for r in rects:
        if r[0] <= cx <= r[2] and r[1] <= cy <= r[3]:
            return True
    return False

def filter_ignored(det_boxes, rects):
    """丢掉中心落在忽略区域里的检测框。det = [cls, score, x1, y1, x2, y2]（原始像素）。返回 (保留的, 丢掉几个)"""
    if not rects or not det_boxes:
        return det_boxes, 0
    keep = [b for b in det_boxes if not in_ignore((b[2] + b[4]) / 2.0, (b[3] + b[5]) / 2.0, rects)]
    return keep, len(det_boxes) - len(keep)

def tracker_in_ignore(tracker, rects):
    """跟踪器当前锁定的目标是否进入了忽略区域（看平滑后的框中心；丢帧保持中看预测框中心）"""
    if not rects or tracker.smooth is None:
        return False
    s = tracker.predicted() if tracker.miss > 0 else tracker.smooth
    return in_ignore((s[0] + s[2]) / 2.0, (s[1] + s[3]) / 2.0, rects)

def dashed_rect_segments(x1, y1, x2, y2, dash=12, gap=8):
    """虚线矩形的线段列表 [(xa, ya, xb, yb), ...]（OSD 画忽略区域用，启动时算一次）"""
    segs = []
    def side(ax, ay, bx, by):
        length = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
        if length <= 0:
            return
        ux, uy = (bx - ax) / length, (by - ay) / length
        t = 0.0
        while t < length:
            e = min(length, t + dash)
            segs.append((int(ax + ux * t), int(ay + uy * t), int(ax + ux * e), int(ay + uy * e)))
            t = e + gap
    side(x1, y1, x2, y1)
    side(x2, y1, x2, y2)
    side(x2, y2, x1, y2)
    side(x1, y2, x1, y1)
    return segs

def ignore_osd_segments(osd_w, osd_h):
    """所有忽略区域在 OSD（与显示画面重合，osd_w x osd_h）上的虚线段，和每个区域标签的位置"""
    segs, labels = [], []
    for (u1, v1, u2, v2) in IGNORE_REGIONS:
        x1 = int(clamp(min(u1, u2) * osd_w, 0, osd_w - 1))
        x2 = int(clamp(max(u1, u2) * osd_w, 0, osd_w - 1))
        y1 = int(clamp(min(v1, v2) * osd_h, 0, osd_h - 1))
        y2 = int(clamp(max(v1, v2) * osd_h, 0, osd_h - 1))
        segs.extend(dashed_rect_segments(x1, y1, x2, y2))
        labels.append((x1 + 2, y1 + 2))
    return segs, labels

class IgnoreCalib:
    """IGNORE_CALIB_MODE：打印每个检测框在显示画面里的归一化中心 / 大小，并累计所有中心的范围，给出建议区域"""
    def __init__(self, frame_w, frame_h, every_ms):
        self.fw, self.fh = frame_w, frame_h
        self.every_ms = every_ms
        self.last = None
        self.rng = None          # [umin, vmin, umax, vmax]
        self.count = 0

    def describe(self, b, rects):
        """一个检测框 -> (u, v, w, h, 是否在忽略区域)，都是显示画面归一化值"""
        dw, dh = disp_size_of(self.fw, self.fh)
        u, v = raw_to_disp_norm((b[2] + b[4]) / 2.0, (b[3] + b[5]) / 2.0, self.fw, self.fh)
        bx = rotate_box(b[2], b[3], b[4], b[5], self.fw, self.fh)
        w, h = (bx[2] - bx[0]) / dw, (bx[3] - bx[1]) / dh
        ign = in_ignore((b[2] + b[4]) / 2.0, (b[3] + b[5]) / 2.0, rects)
        return u, v, w, h, ign

    def feed(self, det_boxes, rects, labels, now_ms):
        """每帧调用（det_boxes 是过滤前的全部检测框）。到了打印间隔返回要打印的文字，否则返回 None"""
        items = [self.describe(b, rects) for b in det_boxes]
        for (u, v, w, h, ign) in items:
            self.count += 1
            if self.rng is None:
                self.rng = [u, v, u, v]
            else:
                r = self.rng
                r[0], r[1] = min(r[0], u), min(r[1], v)
                r[2], r[3] = max(r[2], u), max(r[3], v)
        if self.last is not None and now_ms is not None and self.every_ms > 0 \
                and time.ticks_diff(now_ms, self.last) < self.every_ms:
            return None
        self.last = now_ms
        if not items:
            return "[IGN-CAL] 没有检测框"
        parts = []
        for b, (u, v, w, h, ign) in zip(det_boxes, items):
            cls = int(b[0])
            name = labels[cls] if labels and 0 <= cls < len(labels) else str(cls)
            area = w * h
            parts.append("{} {:.2f} 中心({:.3f},{:.3f}) 宽高({:.3f},{:.3f}) 面积1/{:.0f}{}{}".format(
                name, b[1], u, v, w, h, 1.0 / area if area > 0 else 0,
                "" if box_shape_ok(b) else " 形状不符", " [忽略]" if ign else ""))
        return "[IGN-CAL] " + " | ".join(parts)

    def suggestion(self, pad=0.02):
        """到目前为止所有检测框中心的范围外扩 pad，作为 IGNORE_REGIONS 的建议值（没放物块时才有意义）"""
        if self.rng is None:
            return None
        r = self.rng
        return (round(max(0.0, r[0] - pad), 3), round(max(0.0, r[1] - pad), 3),
                round(min(1.0, r[2] + pad), 3), round(min(1.0, r[3] + pad), 3))

def obb_corners(obb):
    """obb = [cx, cy, w, h, angle_deg] -> 4 个角点（顺时针）"""
    cx, cy, w, h, a = obb
    r = math.radians(a)
    c, s = math.cos(r), math.sin(r)
    hw, hh = w / 2.0, h / 2.0
    return [(cx + dx * c - dy * s, cy + dx * s + dy * c)
            for (dx, dy) in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))]

def wrap_angle_diff(d):
    half = ANGLE_PERIOD / 2.0
    return ((d + half) % ANGLE_PERIOD) - half

def norm_obb_angle(w, h, a):
    """
    把旋转框角度归一到 [0, ANGLE_PERIOD)，同时保持是"同一个矩形"：
    (w, h, a) 和 (h, w, a ± 90°) 是同一个矩形，所以每跨过奇数个 90° 宽高要互换。
    （只做 a % 90 而不互换，会把长方形转 90°；正方形 w≈h 看不出来）
    """
    k = math.floor(a / ANGLE_PERIOD)
    a = a - k * ANGLE_PERIOD
    swap = int(round(k * ANGLE_PERIOD / 90.0)) % 2 == 1
    if a >= ANGLE_PERIOD:           # 浮点误差：-1e-17 这类会得到正好 ANGLE_PERIOD
        a -= ANGLE_PERIOD
        swap = swap != (int(round(ANGLE_PERIOD / 90.0)) % 2 == 1)
    if swap:
        w, h = h, w
    return w, h, a

def vo_rotation_flag():
    deg = ROTATE_DEG
    if VO_FLAG_INVERT and deg in (90, 270):
        deg = 360 - deg
    if deg == 90:
        return Display.FLAG_ROTATION_90
    if deg == 270:
        return Display.FLAG_ROTATION_270
    return Display.FLAG_ROTATION_0

# ------------------------------ 畸变校正 ------------------------------
def undistort_point(x_dist, y_dist, K, dist, cx, cy):
    xn = (x_dist - cx) / K[0]
    yn = (y_dist - cy) / K[1]
    x_u, y_u = xn, yn
    for i in range(10):
        r2 = x_u * x_u + y_u * y_u
        radial = 1 + dist[0]*r2 + dist[1]*r2*r2 + dist[4]*r2*r2*r2
        if abs(x_u) > 10 or abs(y_u) > 10 or radial <= 0:
            x_u, y_u = xn, yn
            break
        xp, yp = x_u, y_u
        x_u = (xn - (2*dist[2]*x_u*y_u + dist[3]*(r2 + 2*x_u*x_u))) / radial
        y_u = (yn - (dist[2]*(r2 + 2*y_u*y_u) + 2*dist[3]*x_u*y_u)) / radial
        if abs(x_u - xp) < 1e-6 and abs(y_u - yp) < 1e-6:
            break
    return x_u * K[0] + cx, y_u * K[1] + cy

def undistort_box(x1, y1, x2, y2, frame_w, frame_h):
    if not USE_UNDISTORT:
        return x1, y1, x2, y2
    K = [FOCAL_PIX, FOCAL_PIX]
    cx, cy = frame_w / 2.0, frame_h / 2.0
    pts = [undistort_point(px, py, K, dist_coeffs, cx, cy)
           for (px, py) in ((x1, y1), (x2, y1), (x1, y2), (x2, y2))]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)

# ------------------------------ 颜色分析 ------------------------------
def _patch_mean(np_img, x0, y0, x1, y1, fw, fh, n=6):
    x0 = int(clamp(x0, 0, fw - 1)); x1 = int(clamp(x1, x0 + 1, fw))
    y0 = int(clamp(y0, 0, fh - 1)); y1 = int(clamp(y1, y0 + 1, fh))
    step = max(1, max(x1 - x0, y1 - y0) // n)
    p = np.array(np_img[:, y0:y1:step, x0:x1:step], dtype=NP_FLOAT)
    return [np.mean(p[i].flatten()) for i in range(3)]

def center_color(np_img, box, fw, fh):
    """
    物块颜色：在框内取 5 个小块（中心 + 上下左右偏 22%），
    以最鲜艳的一块为准，再平均与它相近的块。中心被遮挡/反光时仍能取到物块本身的颜色
    """
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    r = 0.09
    samples = []
    for ox, oy in ((0, 0), (-0.22, 0), (0.22, 0), (0, -0.22), (0, 0.22)):
        px, py = cx + ox * w, cy + oy * h
        samples.append(_patch_mean(np_img, px - r * w, py - r * h, px + r * w, py + r * h, fw, fh))
    best = max(samples, key=lambda c: max(c) - min(c))
    close = [c for c in samples if color_dist(c, best) < 45]
    return [sum(c[i] for c in close) / len(close) for i in range(3)]

def border_color(patch):
    """搜索区域四周一圈的颜色中值，作为背景颜色"""
    out = []
    for i in range(3):
        ch = patch[i]
        vals = ch[0, :].tolist() + ch[-1, :].tolist() + ch[:, 0].tolist() + ch[:, -1].tolist()
        out.append(median(vals))
    return out

def longest_run(vals, thresh, center):
    """从 center 出发向两边扩展，找到连续超过阈值的区间（允许 1 个格子的缺口）"""
    n = len(vals)
    if n == 0:
        return None
    c = int(clamp(center, 0, n - 1))
    if vals[c] < thresh:
        found = -1
        for d in range(1, n):
            for k in (c - d, c + d):
                if 0 <= k < n and vals[k] >= thresh:
                    found = k
                    break
            if found >= 0:
                break
        if found < 0:
            return None
        c = found
    lo = c
    while lo - 1 >= 0 and (vals[lo - 1] >= thresh or (lo - 2 >= 0 and vals[lo - 2] >= thresh)):
        lo -= 1
    hi = c
    while hi + 1 < n and (vals[hi + 1] >= thresh or (hi + 2 < n and vals[hi + 2] >= thresh)):
        hi += 1
    while lo < hi and vals[lo] < thresh:
        lo += 1
    while hi > lo and vals[hi] < thresh:
        hi -= 1
    return lo, hi

_coord_cache = {}

# PROFILE_DETAIL：颜色精修内部各步的累计耗时（微秒），由 Profiler.end_frame 打印后清零
_detail = {}
_detail_order = []

def _dt_add(name, t0):
    """把 t0（ticks_us）到现在的耗时累加到 name，返回现在的时间"""
    now = time.ticks_us()
    if name not in _detail:
        _detail[name] = 0
        _detail_order.append(name)
    _detail[name] += time.ticks_diff(now, t0)
    return now

def _t_now():
    return time.ticks_us() if PROFILE_DETAIL else 0

# FAST_OBB_POINTS：ulab 是否支持布尔下标（第一次用时实测一次；不支持就退回原算法）
_bool_index = None

def _bool_index_ok():
    global _bool_index
    if _bool_index is None:
        try:
            a = np.array([1.0, 2.0, 3.0, 4.0, 5.0], dtype=NP_FLOAT)
            r = a[a > 2.5]
            _bool_index = (len(r) == 3 and float(r[0]) == 3.0 and float(r[2]) == 5.0)
        except Exception as e:
            if is_stop(e):
                raise
            _bool_index = False
        print("FAST_OBB_POINTS:", "on" if _bool_index else "off (ulab 不支持布尔下标，用原算法)")
    return _bool_index

def _grid_coords(eh, ew):
    """腐蚀后网格的坐标数组（按尺寸缓存，避免每帧用 Python 循环重新生成几千个元素）"""
    key = (eh, ew)
    v = _coord_cache.get(key)
    if v is None:
        if len(_coord_cache) > 24:
            _coord_cache.clear()
        xs = np.array(list(range(1, ew + 1)) * eh, dtype=NP_FLOAT)
        ys = np.array([i + 1 for i in range(eh) for _ in range(ew)], dtype=NP_FLOAT)
        v = (xs, ys)
        _coord_cache[key] = v
    return v

def fit_obb(mask, step, rx0, ry0, angle_hint=None):
    """
    在同色掩膜上搜索最小面积外接矩形（旋转框）。
    先 2x2 腐蚀去掉孤立噪点，再搜索角度：
      有上一帧角度时只在其 ±OBB_HINT_RANGE 内搜索（快），否则在 [0, 90) 内全范围粗搜
    返回 [cx, cy, w, h, angle_deg]（原始图像像素坐标），失败返回 None
    """
    ph, pw = mask.shape
    if ph < 4 or pw < 4:
        return None
    total = np.sum(mask.flatten())
    er = mask[:-1, :-1] * mask[1:, :-1] * mask[:-1, 1:] * mask[1:, 1:]
    eh, ew = er.shape
    ef = er.flatten()
    if np.sum(ef) < OBB_MIN_POINTS:
        return None
    # 腐蚀后格子 (i, j) 对应 2x2 块，中心位于格点坐标 (j + 1, i + 1)
    xs, ys = _grid_coords(eh, ew)
    px = py = None
    if FAST_OBB_POINTS and _bool_index_ok():
        # 只保留掩膜内的点（通常只占搜索区域的 30~50%），每个角度少做 4 次整图运算
        sel = ef > 0.5
        px, py = xs[sel], ys[sel]
        if len(px) < OBB_MIN_POINTS:
            px = py = None
    if px is None:
        pen = (ef - 1.0) * 1e6       # 掩膜内 0，掩膜外 -1e6

    def extents(a):
        r = math.radians(a)
        c, s = math.cos(r), math.sin(r)
        if px is not None:
            u = px * c + py * s
            v = py * c - px * s
            return (np.min(u), np.max(u), np.min(v), np.max(v))
        u = xs * c + ys * s
        v = ys * c - xs * s
        return (np.min(u - pen), np.max(u + pen), np.min(v - pen), np.max(v + pen))

    # N 个同色采样点经 2x2 腐蚀后，中心跨度为 N-2，真实宽度 = 跨度 + 2 个采样格
    def area(e):
        return (e[1] - e[0] + 2) * (e[3] - e[2] + 2)

    def search(angles, best_a, best_e):
        for a in angles:
            e = extents(a)
            if best_e is None or area(e) < area(best_e):
                best_a, best_e = a, e
        return best_a, best_e

    best_a, best_e = None, None
    if angle_hint is not None:
        # 快速路径：在上一帧角度附近搜索（1.5° 步长），最优解落在边界说明转得快，改走全范围搜索
        n = int(OBB_HINT_RANGE / 1.5)
        best_a, best_e = search([angle_hint + k * 1.5 for k in range(-n, n + 1)], None, None)
        if abs(best_a - angle_hint) >= n * 1.5 - 1e-6:
            best_a, best_e = None, None
        else:
            best_a, best_e = search([best_a - 0.5, best_a + 0.5], best_a, best_e)
    if best_a is None:
        best_a, best_e = search([k * 3.0 for k in range(30)], None, None)
        best_a, best_e = search([best_a + k * 0.5 for k in (-3, -2, -1, 1, 2, 3)], best_a, best_e)

    umin, umax, vmin, vmax = best_e
    du, dv = umax - umin + 2, vmax - vmin + 2
    if total / (du * dv) < OBB_MIN_FILL:
        return None
    r = math.radians(best_a)
    c, s = math.cos(r), math.sin(r)
    uc, vc = (umin + umax) / 2.0, (vmin + vmax) / 2.0
    cx = rx0 + (uc * c - vc * s) * step
    cy = ry0 + (uc * s + vc * c) * step
    w, h = du * step, dv * step
    if FIX_OBB_ANGLE_WRAP:
        w, h, ang = norm_obb_angle(w, h, best_a)
    else:
        ang = best_a % ANGLE_PERIOD
    return [cx, cy, w, h, ang]

def _pix_rgb(np_img, x, y, fw, fh):
    if x < 0 or y < 0:
        return None
    xi, yi = int(x + 0.5), int(y + 0.5)
    if xi >= fw or yi >= fh:
        return None
    return (int(np_img[0, yi, xi]), int(np_img[1, yi, xi]), int(np_img[2, yi, xi]))

def subpixel_refine(np_img, obb, color, fw, fh, step, bg):
    """
    亚像素修正旋转框的四条边。返回 (新 obb, 成功的边数)；失败时返回原 obb
    - 每条采样线从物块内部向外扫描，取第一次越过阈值的位置，
      旁边紧挨着同色物块时也能停在本物块的真实边缘
    - 边缘定位用“物块色 -> 该处背景色”方向上的线性投影（0=物块，1=背景），
      投影值与边缘像素的混合比例成正比，阈值 0.5 处就是真实边缘，且对物块明暗变化不敏感
    """
    cx, cy, w, h, a = obb
    r = math.radians(a)
    c, s = math.cos(r), math.sin(r)
    R = int(clamp(1.5 * step + 2, 3, 0.6 * min(w, h) / 2))
    if R < 2:
        return obb, 0
    # (外法线, 切线, 半宽, 边长)
    edges = (((c, s), (-s, c), w / 2.0, h),
             ((-c, -s), (-s, c), w / 2.0, h),
             ((-s, c), (c, s), h / 2.0, w),
             ((s, -c), (c, s), h / 2.0, w))
    offs = []
    good = 0
    # 角度修正：若真实转角比当前多 δ（弧度），四条边的偏移量沿切线方向的斜率分别为 -δ, +δ, +δ, -δ
    slope_sign = (-1.0, 1.0, 1.0, -1.0)
    deltas = []
    for ei, (n, t, half, length) in enumerate(edges):
        ex, ey = cx + n[0] * half, cy + n[1] * half
        # 采样位置（相对当前边，负数在物块内侧）：内侧粗采样 + 边缘附近逐像素采样
        start = -int(half * 0.75)       # 从靠近中心处开始（粘连时框中心会偏，留足余量）
        coarse = max(1, (-R - start) // 4)
        positions = list(range(start, -R, coarse)) + list(range(-R, R + 1))
        vals = []
        tpos = []
        for f in SUBPIX_PROFILES:
            bx, by = ex + t[0] * f * length, ey + t[1] * f * length
            rgbs = []
            for i in positions:
                p = _pix_rgb(np_img, bx + n[0] * i, by + n[1] * i, fw, fh)
                if p is None:
                    rgbs = None
                    break
                rgbs.append(p)
            if rgbs is None:
                continue
            # 该采样线外侧的背景色；外侧若仍是同色（粘连），改用整体背景色
            outer = [(rgbs[-1][j] + rgbs[-2][j]) / 2.0 for j in range(3)]
            if color_dist(outer, color) < SUBPIX_MIN_CONTRAST:
                outer = bg
            dv0, dv1, dv2 = outer[0] - color[0], outer[1] - color[1], outer[2] - color[2]
            den = dv0 * dv0 + dv1 * dv1 + dv2 * dv2
            if den < SUBPIX_MIN_CONTRAST * SUBPIX_MIN_CONTRAST:
                continue
            prof = [((q[0] - color[0]) * dv0 + (q[1] - color[1]) * dv1 + (q[2] - color[2]) * dv2) / den
                    for q in rgbs]
            din = (prof[0] + prof[1]) / 2.0
            dout = max(1.0, (prof[-1] + prof[-2]) / 2.0)
            if dout - din < 0.3:
                continue
            thr = (din + dout) / 2.0
            for k in range(1, len(prof)):
                # 第一次由内向外越过阈值，且后一个点仍在阈值以上（排除单点噪声/反光）
                if prof[k - 1] < thr <= prof[k] and (k + 1 >= len(prof) or prof[k + 1] >= thr):
                    frac = (thr - prof[k - 1]) / (prof[k] - prof[k - 1])
                    vals.append(positions[k - 1] + frac * (positions[k] - positions[k - 1]))
                    tpos.append(f * length)
                    break
        if len(vals) >= max(2, len(SUBPIX_PROFILES) // 2 + 1):
            off = median(vals)
            # 向外扩张受限（避免吃进背景）；向内收缩允许更大（分开粘连的同色物块）
            if off > SUBPIX_MAX_SHIFT * length or off < -0.45 * length:
                return obb, 0
            offs.append(off)
            good += 1
            # Theil-Sen 斜率（两两斜率取中值，抗个别采样线出错）
            sl = []
            for i in range(len(vals)):
                for j in range(i + 1, len(vals)):
                    dt = tpos[j] - tpos[i]
                    if abs(dt) > 1e-6:
                        sl.append((vals[j] - vals[i]) / dt)
            if sl:
                deltas.append(slope_sign[ei] * math.atan(median(sl)))
        else:
            offs.append(0.0)
    if good < 2:
        return obb, 0
    if ENABLE_ANGLE_REFINE and len(deltas) >= 2 and min(w, h) >= ANGLE_REFINE_MIN_SIDE:
        d = median(deltas)
        if abs(math.degrees(d)) <= ANGLE_REFINE_MAX_DEG:
            if FIX_OBB_ANGLE_WRAP:
                a = a + math.degrees(d)          # 最后和宽高一起归一化
            else:
                a = (a + math.degrees(d)) % ANGLE_PERIOD
    ou_p, ou_m, ov_p, ov_m = offs
    du = (ou_p - ou_m) / 2.0
    dv = (ov_p - ov_m) / 2.0
    ncx = cx + c * du - s * dv
    ncy = cy + s * du + c * dv
    nw = w + ou_p + ou_m
    nh = h + ov_p + ov_m
    if nw < MIN_BOX_PIX or nh < MIN_BOX_PIX:
        return obb, 0
    if FIX_OBB_ANGLE_WRAP:
        nw, nh, a = norm_obb_angle(nw, nh, a)
    return [ncx, ncy, nw, nh, a], good

def center_color_cached(np_img, box, fw, fh, cache):
    """CACHE_CENTER_COLOR：同一帧里同一个框只算一次 center_color（cache 为每帧新建的 dict）"""
    if cache is None or not CACHE_CENTER_COLOR:
        return center_color(np_img, box, fw, fh)
    key = (box[0], box[1], box[2], box[3])
    c = cache.get(key)
    if c is None:
        c = center_color(np_img, box, fw, fh)
        cache[key] = c
    return c

def color_refine(np_img, box, fw, fh, angle_hint=None, fast=False, cache=None):
    """
    用物块颜色在检测框附近重新找边缘。
    返回 (refined_box, color, obb)：
      refined_box 精修后的正框（失败为 None）；color 物块颜色；obb 旋转框（失败为 None）
    """
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    if w < 12 or h < 12:
        return None, None, None
    t0 = _t_now()
    color = center_color_cached(np_img, box, fw, fh, cache)
    if PROFILE_DETAIL:
        _dt_add("r.color", t0)
    chroma = max(color) - min(color)
    if not ENABLE_COLOR_REFINE or chroma < REFINE_MIN_CHROMA:
        return None, color, None
    # 先用较大的搜索范围（应对检测框偏紧）；失败（例如旁边紧挨着同色物块被连成一片）
    # 再收紧到检测框附近重试，借助检测器把两个物块分开
    for expand in (REFINE_EXPAND, REFINE_EXPAND_TIGHT):
        res = _refine_once(np_img, box, color, chroma, expand, fw, fh, angle_hint, fast)
        if res is not None:
            return res[0], color, res[1]
    return None, color, None

def _refine_once(np_img, box, color, chroma, expand, fw, fh, angle_hint=None, fast=False):
    """单次精修，成功返回 (refined_box, obb_or_None)，失败返回 None"""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    rx0 = int(clamp(x1 - w * expand, 0, fw - 2))
    ry0 = int(clamp(y1 - h * expand, 0, fh - 2))
    rx1 = int(clamp(x2 + w * expand, rx0 + 2, fw))
    ry1 = int(clamp(y2 + h * expand, ry0 + 2, fh))
    if REFINE_GRID_CEIL:
        step = max(1, (max(rx1 - rx0, ry1 - ry0) + REFINE_GRID - 1) // REFINE_GRID)
    else:
        step = max(1, max(rx1 - rx0, ry1 - ry0) // REFINE_GRID)
    t0 = _t_now()
    patch = np.array(np_img[:, ry0:ry1:step, rx0:rx1:step], dtype=NP_FLOAT)

    # 自适应容差：既看物块颜色鲜艳度，也看与背景的颜色差（背景接近物块颜色时收紧）
    bg = border_color(patch)
    contrast = color_dist(color, bg)
    if contrast < REFINE_MIN_CONTRAST:
        return None
    tol = clamp(min(chroma * 0.6, contrast * 0.5), 20, 90)
    dist = color_dist_map(patch, color)
    mask = np.clip((tol - dist) * 1000.0, 0.0, 1.0)     # 同色=1，异色=0
    if PROFILE_DETAIL:
        t0 = _dt_add("r.mask", t0)

    # 优先：旋转框
    if ENABLE_OBB:
        obb = fit_obb(mask, step, rx0, ry0, angle_hint)
        if PROFILE_DETAIL:
            t0 = _dt_add("r.obb", t0)
        if obb is not None and ENABLE_SUBPIXEL and not fast:
            obb, _ = subpixel_refine(np_img, obb, color, fw, fh, step, bg)
            if PROFILE_DETAIL:
                t0 = _dt_add("r.subpix", t0)
        if obb is not None and 1.0 / REFINED_ASPECT_MAX <= obb[2] / obb[3] <= REFINED_ASPECT_MAX:
            cs = obb_corners(obb)
            px = [p[0] for p in cs]
            py = [p[1] for p in cs]
            nb = [clamp(min(px), 0, fw), clamp(min(py), 0, fh),
                  clamp(max(px), 0, fw), clamp(max(py), 0, fh)]
            if nb[2] - nb[0] >= MIN_BOX_PIX and nb[3] - nb[1] >= MIN_BOX_PIX \
                    and refine_box_ok(nb, box):
                return nb, obb

    # 退回：轴对齐边缘精修
    cols = np.sum(mask, axis=0).tolist()
    rows = np.sum(mask, axis=1).tolist()
    if not cols or not rows:
        return None
    cx_i = ((x1 + x2) / 2 - rx0) / step
    cy_i = ((y1 + y2) / 2 - ry0) / step
    rc = longest_run(cols, 0.4 * max(cols), cx_i)
    rr = longest_run(rows, 0.4 * max(rows), cy_i)
    if rc is None or rr is None:
        return None

    nb = [rx0 + rc[0] * step, ry0 + rr[0] * step,
          rx0 + (rc[1] + 1) * step, ry0 + (rr[1] + 1) * step]
    nb = [min(nb[0], fw), min(nb[1], fh), min(nb[2], fw), min(nb[3], fh)]
    nw, nh = nb[2] - nb[0], nb[3] - nb[1]
    if nw < MIN_BOX_PIX or nh < MIN_BOX_PIX:
        return None
    if not (1.0 / REFINED_ASPECT_MAX <= nw / nh <= REFINED_ASPECT_MAX):
        return None
    if not refine_box_ok(nb, box):
        return None
    sub = mask[rr[0]:rr[1] + 1, rc[0]:rc[1] + 1]
    fill = np.sum(sub.flatten()) / ((rr[1] - rr[0] + 1) * (rc[1] - rc[0] + 1))
    if fill < REFINE_MIN_FILL:
        return None
    return nb, None

# ------------------------------ 目标跟踪与平滑 ------------------------------
def box_shape_ok(b):
    w, h = b[4] - b[2], b[5] - b[3]
    if w < MIN_BOX_PIX or h < MIN_BOX_PIX:
        return False
    return ASPECT_MIN <= w / h <= ASPECT_MAX

class Tracker:
    """
    单目标跟踪：置信度双阈值、IoU+中心距离+颜色关联、速度预测、
    中值滤波 + 自适应 EMA + 死区、类别多数投票
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.history = []
        self.smooth = None
        self.vel = [0.0, 0.0]
        self.color = None
        self.cls_hist = []
        self.cls = -1
        self.score = 0.0
        self.hits = 0
        self.miss = 0
        self.refined = False
        self.motion = 0.0         # 最近一帧的位移（相对框边长），用于判断是否在移动
        self.obb = None           # 平滑后的旋转框 [cx, cy, w, h, angle]
        self.obb_miss = 0

    def confirmed(self):
        return self.smooth is not None and self.hits >= CONFIRM_FRAMES

    def predicted(self):
        if self.smooth is None:
            return None
        k = self.miss + 1
        dx, dy = self.vel[0] * k, self.vel[1] * k
        s = self.smooth
        return [s[0] + dx, s[1] + dy, s[2] + dx, s[3] + dy]

    def predicted_obb(self):
        if self.obb is None:
            return None
        k = self.miss
        o = self.obb
        return [o[0] + self.vel[0] * k, o[1] + self.vel[1] * k, o[2], o[3], o[4]]

    def _update_obb(self, obb, alpha, follow=None):
        """follow = (dx, dy, scale)：正框本帧的平移和尺寸比例，FIX_STALE_OBB 时让沿用的旧旋转框跟着走"""
        if obb is None:
            self.obb_miss += 1
            if self.obb_miss > OBB_HOLD_FRAMES:
                self.obb = None   # 连续几帧拟合不到旋转框，退回正框
            elif FIX_STALE_OBB and follow is not None and self.obb is not None:
                o = self.obb
                o[0] += follow[0]
                o[1] += follow[1]
                o[2] *= follow[2]
                o[3] *= follow[2]
            return
        self.obb_miss = 0
        n = list(obb)
        if self.obb is None:
            self.obb = n
            return
        o = self.obb
        d_raw = n[4] - o[4]
        if abs(d_raw) > ANGLE_PERIOD / 2.0:
            n[2], n[3] = n[3], n[2]   # 角度跨越周期边界时，宽高轴互换
        d_ang = wrap_angle_diff(d_raw)
        for i in range(4):
            d = n[i] - o[i]
            if abs(d) > DEADBAND_PIX:
                o[i] += alpha * d
        if abs(d_ang) > 0.3:          # 角度死区 0.3°
            if FIX_OBB_ANGLE_WRAP:
                o[2], o[3], o[4] = norm_obb_angle(o[2], o[3], o[4] + alpha * d_ang)
            else:
                o[4] = (o[4] + alpha * d_ang) % ANGLE_PERIOD

    def select(self, det_boxes, start_score, keep_score, color_of):
        """返回 (det, matched)。color_of(box) 返回该框中心颜色，用于颜色一致性校验"""
        cands = [b for b in det_boxes if box_shape_ok(b)]
        pred = self.predicted()
        if pred is not None:
            pcx, pcy = (pred[0] + pred[2]) / 2, (pred[1] + pred[3]) / 2
            psize = max(pred[2] - pred[0], pred[3] - pred[1], 1)
            best, best_v = None, 0.0
            for b in cands:
                if b[1] < keep_score:
                    continue
                bb = (b[2], b[3], b[4], b[5])
                v = iou(pred, bb)
                if v >= TRACK_IOU:
                    v = 0.5 + v
                else:
                    d = math.sqrt(((b[2] + b[4]) / 2 - pcx) ** 2 + ((b[3] + b[5]) / 2 - pcy) ** 2)
                    v = 0.5 * (1 - d / (TRACK_DIST * psize)) if d < TRACK_DIST * psize else 0.0
                if v > best_v and ENABLE_COLOR_GATE and self.color is not None:
                    try:
                        c = color_of(bb)
                        if c is not None and color_dist(c, self.color) > COLOR_GATE:
                            continue
                    except Exception as e:
                        if is_stop(e):
                            raise
                if v > best_v:
                    best, best_v = b, v
            if best is not None:
                return best, True
            if self.confirmed() and self.miss < 2:
                return None, False
        best, best_v = None, -1
        for b in cands:
            if b[1] < start_score:
                continue
            v = b[1] * math.sqrt((b[4] - b[2]) * (b[5] - b[3]))
            if v > best_v:
                best, best_v = b, v
        return best, False

    def update(self, det, matched, box, color, refined, obb=None, dt_ms=None):
        if not matched:
            self.reset()
        self.refined = refined

        self.cls_hist.append(det[0])
        if len(self.cls_hist) > CLASS_VOTE_WINDOW:
            self.cls_hist.pop(0)
        counts = {}
        for c in self.cls_hist:
            counts[c] = counts.get(c, 0) + 1
        self.cls = max(counts, key=lambda c: counts[c])
        self.score = det[1] if self.smooth is None else 0.6 * self.score + 0.4 * det[1]
        if color is not None:
            self.color = list(color) if self.color is None else \
                [0.8 * self.color[i] + 0.2 * color[i] for i in range(3)]

        if self.smooth is None:
            self.history = [list(box)]
            self.smooth = list(box)
            self.hits, self.miss = 1, 0
            self._update_obb(obb, 1.0)
            return

        size = max(self.smooth[2] - self.smooth[0], self.smooth[3] - self.smooth[1], 1)
        motion = max(abs(box[i] - self.smooth[i]) for i in range(4)) / size
        self.motion = motion

        self.history.append(list(box))
        if motion > MOTION_REF:
            self.history = self.history[-2:]
        elif len(self.history) > SMOOTH_WINDOW:
            self.history.pop(0)
        med = [median([h[i] for h in self.history]) for i in range(4)]

        alpha = ALPHA_MIN + (ALPHA_MAX - ALPHA_MIN) * clamp(motion / MOTION_REF, 0.0, 1.0)
        # 按实际帧间隔换算平滑系数：ALPHA_* 是按 REF_FRAME_MS（约 30fps）调的，
        # 帧率低时每帧间隔长，相应减弱平滑，保证“按时间算”的跟随速度不变
        if dt_ms is not None and dt_ms > 0:
            alpha = 1.0 - (1.0 - alpha) ** (dt_ms / REF_FRAME_MS)
        old_cx = (self.smooth[0] + self.smooth[2]) / 2
        old_cy = (self.smooth[1] + self.smooth[3]) / 2
        old_size = size
        for i in range(4):
            d = med[i] - self.smooth[i]
            if abs(d) > DEADBAND_PIX:
                self.smooth[i] += alpha * d
        steps = self.miss + 1
        new_cx = (self.smooth[0] + self.smooth[2]) / 2
        new_cy = (self.smooth[1] + self.smooth[3]) / 2
        self.vel[0] = 0.5 * self.vel[0] + 0.5 * (new_cx - old_cx) / steps
        self.vel[1] = 0.5 * self.vel[1] + 0.5 * (new_cy - old_cy) / steps
        follow = None
        if FIX_STALE_OBB:
            new_size = max(self.smooth[2] - self.smooth[0], self.smooth[3] - self.smooth[1], 1)
            follow = (new_cx - old_cx, new_cy - old_cy, new_size / old_size)
        self._update_obb(obb, alpha, follow)

        self.hits += 1
        self.miss = 0

    def mark_missed(self):
        """返回 True 表示本次刚好判定为丢失"""
        if self.smooth is None:
            return False
        self.miss += 1
        if self.miss > LOST_TOLERANCE:
            self.reset()
            return True
        return False

# ------------------------------ 深度与位置计算 ------------------------------
def compute_position(raw_box, frame_w, frame_h, real_size, obb=None):
    """
    统一用 4 个角点计算：有旋转框用旋转框角点，否则用正框角点。
    顺序：畸变校正（原始坐标系）-> 边长/中心 -> 深度 -> 旋转到显示方向
    """
    if obb is not None:
        corners = obb_corners(obb)
    else:
        x1, y1, x2, y2 = raw_box
        corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]

    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    touches_edge = (min(xs) <= EDGE_MARGIN or min(ys) <= EDGE_MARGIN or
                    max(xs) >= frame_w - EDGE_MARGIN or max(ys) >= frame_h - EDGE_MARGIN)

    if USE_UNDISTORT:
        K = [FOCAL_PIX, FOCAL_PIX]
        cx0, cy0 = frame_w / 2.0, frame_h / 2.0
        ucs = [undistort_point(px, py, K, dist_coeffs, cx0, cy0) for (px, py) in corners]
    else:
        ucs = corners

    def dist(p, q):
        return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)
    side_a = (dist(ucs[0], ucs[1]) + dist(ucs[2], ucs[3])) / 2.0
    side_b = (dist(ucs[1], ucs[2]) + dist(ucs[3], ucs[0])) / 2.0
    if side_a <= 2 or side_b <= 2:
        return None
    ux = [p[0] for p in ucs]
    uy = [p[1] for p in ucs]
    if max(ux) - min(ux) >= frame_w * 0.95 or max(uy) - min(uy) >= frame_h * 0.95:
        return None

    size_pix = max(side_a, side_b)
    depth_mm = (real_size * FOCAL_PIX) / size_pix * DEPTH_SCALE + DEPTH_OFFSET

    ucx = sum(ux) / 4.0
    ucy = sum(uy) / 4.0
    if ROTATED and POSITION_IN_ROTATED_FRAME:
        pcx, pcy = rotate_point(ucx, ucy, frame_w, frame_h)
        pw, ph = frame_h, frame_w
    else:
        pcx, pcy = ucx, ucy
        pw, ph = frame_w, frame_h
    X_mm = (pcx - pw / 2.0) * depth_mm / FOCAL_PIX
    Y_mm = (pcy - ph / 2.0) * depth_mm / FOCAL_PIX

    # 显示用：未校正的角点（与画面对齐），旋转到显示方向
    disp_poly = [rotate_point(px, py, frame_w, frame_h) for (px, py) in corners]
    disp_size = (frame_h, frame_w) if ROTATED else (frame_w, frame_h)

    # 显示坐标系下的转角（y 向下，顺时针为正），范围 [0, ANGLE_PERIOD)
    angle = None
    if obb is not None:
        p0, p1 = disp_poly[0], disp_poly[1]
        angle = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])) % ANGLE_PERIOD

    return {
        'x_mm': X_mm, 'y_mm': Y_mm, 'depth_mm': depth_mm, 'size_pix': size_pix,
        'disp_poly': disp_poly, 'disp_size': disp_size, 'angle': angle,
        'touches_edge': touches_edge,
    }

# ------------------------------ 串口发送 ------------------------------
def send_position_via_uart(x_mm, y_mm, depth_mm, verbose=True):
    """格式: 0xAA 0x55 | X_H X_L | Y_H Y_L | D_H D_L | CRC8 | 0x0D，共10字节"""
    try:
        x_int = max(-32768, min(32767, int(round(x_mm))))
        y_int = max(-32768, min(32767, int(round(y_mm))))
        d_int = max(0, min(65535, int(round(depth_mm))))
        x_u16, y_u16, d_u16 = x_int & 0xFFFF, y_int & 0xFFFF, d_int & 0xFFFF
        frame = bytearray([0xAA, 0x55,
                           (x_u16 >> 8) & 0xFF, x_u16 & 0xFF,
                           (y_u16 >> 8) & 0xFF, y_u16 & 0xFF,
                           (d_u16 >> 8) & 0xFF, d_u16 & 0xFF])
        checksum = crc8(frame)
        uart.write(bytes(frame) + bytes([checksum, 0x0D]))
        if verbose:
            print("UART sent: X={} Y={} D={} | CRC=0x{:02X}".format(x_int, y_int, d_int, checksum))
    except Exception as e:
        if is_stop(e):
            raise
        print("UART send error:", e)

class Profiler:
    """分阶段计时：start() 开始一帧，mark(name) 记录上一个点到现在的耗时"""
    def __init__(self, enabled, every):
        self.enabled = enabled
        self.every = every
        self.acc = {}
        self.order = []
        self.frames = 0
        self.t = 0

    def start(self):
        if self.enabled:
            self.t = time.ticks_us()

    def mark(self, name):
        if not self.enabled:
            return
        now = time.ticks_us()
        if name not in self.acc:
            self.acc[name] = 0
            self.order.append(name)
        self.acc[name] += time.ticks_diff(now, self.t)
        self.t = now

    def end_frame(self):
        if not self.enabled:
            return
        self.frames += 1
        if self.frames >= self.every:
            total = sum(self.acc.values())
            parts = ["{} {:.1f}".format(k, self.acc[k] / self.frames / 1000.0) for k in self.order]
            print("[timing ms/frame] " + " | ".join(parts) +
                  " | total {:.1f} ({:.1f} fps)".format(total / self.frames / 1000.0,
                                                       self.frames * 1e6 / total if total else 0))
            if PROFILE_DETAIL and _detail_order:
                print("[timing detail ms/frame] " + " | ".join(
                    "{} {:.1f}".format(k, _detail[k] / self.frames / 1000.0) for k in _detail_order))
                _detail.clear()
                _detail_order.clear()
            self.acc = {}
            self.order = []
            self.frames = 0

class _NullOSD:
    """OSD_EVERY > 1 时不画的那几帧用它代替 osd_img：所有绘图调用都什么也不做"""
    def clear(self):
        pass

    def draw_line(self, *a, **k):
        pass

    def draw_cross(self, *a, **k):
        pass

    def draw_string_advanced(self, *a, **k):
        pass

_NULL_OSD = _NullOSD()

def pick_color(color_four, cls):
    try:
        color = color_four[cls]
        if isinstance(color, (list, tuple)):
            if len(color) == 4:
                return tuple(color[1:])
            if len(color) == 3:
                return tuple(color)
    except (IndexError, TypeError):
        pass
    return (0, 255, 0)

# ------------------------------ 主流程 ------------------------------
# 已初始化的硬件资源，退出时由 cleanup() 统一释放（无论是正常结束、出错还是 IDE 停止）
_res = {"sensor": None, "display": False, "media": False}

def cleanup():
    print("Shutting down...")
    s = _res["sensor"]
    if s is not None:
        try:
            s.stop()
        except Exception as e:
            print("sensor.stop error:", e)
        _res["sensor"] = None
    if _res["display"]:
        try:
            Display.deinit()
        except Exception as e:
            print("Display.deinit error:", e)
        _res["display"] = False
    if hasattr(os, "EXITPOINT_ENABLE_SLEEP"):
        _exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    if _res["media"]:
        try:
            MediaManager.deinit()
        except Exception as e:
            print("MediaManager.deinit error:", e)
        _res["media"] = False
    gc.collect()
    try:
        nn.shrink_memory_pool()
    except Exception:
        pass
    print("vision_locator end")

def detection():
    print("vision_locator start")
    if hasattr(os, "EXITPOINT_ENABLE"):
        _exitpoint(os.EXITPOINT_ENABLE)   # 允许 IDE 停止按钮中断

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            deploy_conf = ujson.load(f)
        print("Config loaded.")
    except Exception as e:
        print("Fatal Error: cannot read config", config_path, e)
        return -1

    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    color_four = get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]

    if model_type != "AnchorBaseDet":
        print("Fatal Error: only AnchorBaseDet is supported, got", model_type)
        return -1
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

    keep_score = confidence_threshold * KEEP_SCORE_RATIO
    start_score = min(0.95, confidence_threshold + START_SCORE_EXTRA)
    print("score thresholds: start={:.2f} keep={:.2f}".format(start_score, keep_score))

    kmodel_frame_size = img_size
    FW, FH = OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGHT
    frame_size = [FW, FH]
    strides = [8, 16, 32]
    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

    kpu = nn.kpu()
    try:
        kpu.load_kmodel(root_path + kmodel_name)
        print("KPU model loaded:", kmodel_name)
    except Exception as e:
        print("Failed to load kmodel:", e)
        return -1

    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    try:
        ai2d_builder = ai2d.build([1, 3, FH, FW], [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]])
    except Exception as e:
        print("Failed to build AI2D:", e)
        return -1

    try:
        sensor = Sensor()
        _res["sensor"] = sensor
        sensor.reset()
        sensor.set_hmirror(False)
        sensor.set_vflip(False)
        sensor.set_framesize(width=CHN0_W, height=CHN0_H)
        sensor.set_framesize(width=FW, height=FH, chn=CAM_CHN_ID_2)
        sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

        bind_info = sensor.bind_info(x=VID_X, y=VID_Y, chn=CAM_CHN_ID_0)
        if ROTATED:
            Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1, flag=vo_rotation_flag())
        else:
            Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

        if display_mode == "lcd":
            Display.init(Display.ST7701, to_ide=True)
        else:
            Display.init(Display.LT9611, to_ide=True)
        _res["display"] = True

        osd_img = image.Image(OSD_W, OSD_H, image.ARGB8888)
        MediaManager.init()
        _res["media"] = True
        sensor.run()
    except Exception as e:
        if is_stop(e):
            raise
        print("Sensor/Display init failed:", e)
        return -1

    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)

    tracker = Tracker()
    last_send_time = 0
    calib_sizes = []
    calib_text = ""
    color_ok = ENABLE_COLOR_REFINE or ENABLE_COLOR_GATE
    color_fail = 0
    err_count = 0
    fps = 0.0
    t_last = time.ticks_ms()
    prof = Profiler(DEBUG_TIMING, PROFILE_EVERY)
    frame_dt = REF_FRAME_MS
    frame_no = 0
    # 忽略区域（任务 10）：启动时换算成原始像素矩形和 OSD 虚线段，主循环里只做比较
    ign_rects = ignore_rects_raw(FW, FH)
    ign_segs, ign_labels = ignore_osd_segments(OSD_W, OSD_H) if (IGNORE_REGIONS_ENABLE and IGNORE_DRAW) else ([], [])
    ign_calib = IgnoreCalib(FW, FH, IGNORE_CALIB_EVERY_MS) if IGNORE_CALIB_MODE else None
    ign_tip_ms = time.ticks_ms()
    if ign_rects:
        print("ignore regions (display 0~1):", IGNORE_REGIONS)
        print("ignore regions (raw {}x{} px):".format(FW, FH),
              [tuple(int(round(v)) for v in r) for r in ign_rects])
    print("Init done. rotate={} video={}x{} at ({},{})".format(ROTATE_DEG, VID_W, VID_H, VID_X, VID_Y))

    while True:
        try:
            _exitpoint()                  # 响应 IDE 停止按钮
            with ScopedTiming("total", 0):
                prof.start()
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
                if rgb888p_img.format() != image.RGBP888:
                    continue
                prof.mark("capture")

                np_img = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(np_img)
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                del ai2d_input_tensor

                kpu.set_input_tensor(0, ai2d_output_tensor)
                kpu.run()
                prof.mark("ai2d+kpu")

                results = []
                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()
                    results.append(result.reshape((result.shape[0] * result.shape[1] *
                                                   result.shape[2] * result.shape[3])))
                    del out_data

                det_boxes = aicube.anchorbasedet_post_process(
                    results[0], results[1], results[2],
                    kmodel_frame_size, frame_size, strides,
                    num_classes, keep_score, nms_threshold,
                    anchors, nms_option)
                del results
                prof.mark("post")

                # ---------- 忽略区域（爪子脚）：中心落在区域里的检测框不进跟踪器 ----------
                if ign_calib is not None:
                    txt = ign_calib.feed(det_boxes or [], ign_rects, labels, time.ticks_ms())
                    if txt:
                        print(txt)
                    if time.ticks_diff(time.ticks_ms(), ign_tip_ms) > 5000:
                        ign_tip_ms = time.ticks_ms()
                        print("[IGN-CAL] 累计 {} 个检测框，中心范围外扩 0.02 = 建议区域 {}".format(
                            ign_calib.count, ign_calib.suggestion()))
                if ign_rects:
                    det_boxes, _ = filter_ignored(det_boxes or [], ign_rects)

                # ---------- 选择目标 ----------
                cc_cache = {} if CACHE_CENTER_COLOR else None
                if color_ok:
                    color_of = lambda bb: center_color_cached(np_img, bb, FW, FH, cc_cache)
                else:
                    color_of = lambda bb: None
                det, matched = tracker.select(det_boxes or [], start_score, keep_score, color_of)
                prof.mark("select")

                if det is not None:
                    raw = [det[2], det[3], det[4], det[5]]
                    box, color, refined, obb = raw, None, False, None
                    if color_ok:
                        try:
                            hint = tracker.obb[4] if (matched and tracker.obb is not None) else None
                            moving = matched and tracker.motion > MOTION_REF
                            nb, color, obb = color_refine(np_img, raw, FW, FH, hint,
                                                          fast=moving and not SUBPIX_WHEN_MOVING,
                                                          cache=cc_cache)
                            if nb is not None:
                                box, refined = nb, True
                            color_fail = 0
                        except Exception as e:
                            if is_stop(e):
                                raise
                            color_fail += 1
                            print("color refine error:", e)
                            if color_fail >= 5:
                                color_ok = False
                                print("color features disabled after repeated errors")
                    tracker.update(det, matched, box, color, refined, obb, frame_dt)
                else:
                    if tracker.mark_missed():
                        print("Target lost.")
                        if SEND_LOST_FRAME:
                            send_position_via_uart(0, 0, 0)
                if ign_rects and tracker_in_ignore(tracker, ign_rects):
                    print("Target entered ignore region, dropped.")
                    tracker.reset()
                prof.mark("refine")

                draw_osd = OSD_EVERY <= 1 or frame_no % OSD_EVERY == 0
                osd = osd_img if draw_osd else _NULL_OSD
                osd.clear()
                for (xa, ya, xb, yb) in ign_segs:
                    osd.draw_line(xa, ya, xb, yb, color=(255, 60, 60), thickness=2)
                for (lx, ly) in ign_labels:
                    osd.draw_string_advanced(lx, ly, 20, "IGN", color=(255, 60, 60))

                if tracker.confirmed():
                    holding = tracker.miss > 0
                    box = tracker.predicted() if holding else tracker.smooth
                    label = labels[tracker.cls] if 0 <= tracker.cls < len(labels) else str(tracker.cls)
                    real_size = OBJECT_SIZES_MM.get(label, BLOCK_REAL_SIZE)
                    obb_now = tracker.predicted_obb() if ENABLE_OBB else None
                    pos = compute_position(box, FW, FH, real_size, obb_now)
                    if pos:
                        cw, ch = pos['disp_size']
                        sx, sy = VID_W / cw, VID_H / ch
                        # OSD 图层与画面区域重合，直接用画面内坐标
                        poly = [(int(clamp(px * sx, 0, OSD_W - 1)),
                                 int(clamp(py * sy, 0, OSD_H - 1)))
                                for (px, py) in pos['disp_poly']]
                        draw_x = min(min(p[0] for p in poly), OSD_W - 230)   # 文字别超出右边界
                        draw_y = min(p[1] for p in poly)
                        draw_h = max(p[1] for p in poly) - draw_y

                        color = (128, 128, 128) if holding else pick_color(color_four, tracker.cls)
                        for k in range(4):
                            p, q = poly[k], poly[(k + 1) % 4]
                            osd.draw_line(p[0], p[1], q[0], q[1], color=color, thickness=3)
                        # 中心十字
                        mcx = sum(p[0] for p in poly) // 4
                        mcy = sum(p[1] for p in poly) // 4
                        osd.draw_cross(mcx, mcy, color=color, size=8, thickness=2)

                        tag = " R" if tracker.refined and not holding else ""
                        label_y = draw_y - 40 if draw_y >= 40 else draw_y + draw_h + 5
                        osd.draw_string_advanced(draw_x, label_y, 32,
                            "{} {:.2f}{}".format(label, tracker.score, tag), color=color)

                        info_y = draw_y + draw_h + 5
                        if info_y + 90 > OSD_H:
                            info_y = max(0, draw_y - 90)
                        osd.draw_string_advanced(draw_x, info_y, 24,
                            "Depth: {:.1f}mm".format(pos['depth_mm']), color=(255, 255, 0))
                        osd.draw_string_advanced(draw_x, info_y + 30, 24,
                            "X:{:.1f} Y:{:.1f}mm".format(pos['x_mm'], pos['y_mm']), color=(0, 255, 255))
                        extra_y = info_y + 60
                        if pos['angle'] is not None:
                            osd.draw_string_advanced(draw_x, extra_y, 24,
                                "Angle: {:.1f}deg".format(pos['angle']), color=(255, 160, 0))
                            extra_y += 30
                        if pos['touches_edge']:
                            osd.draw_string_advanced(draw_x, extra_y, 24, "EDGE", color=(255, 80, 80))

                        if CALIB_DISTANCE_MM > 0 and not holding and not pos['touches_edge']:
                            calib_sizes.append(pos['size_pix'])
                            if len(calib_sizes) >= 30:
                                f_est = median(calib_sizes) * CALIB_DISTANCE_MM / real_size
                                calib_text = "CALIB FOCAL_PIX = {:.1f}".format(f_est)
                                print(calib_text)
                                calib_sizes = []

                        now = time.ticks_ms()
                        depth_ok = not FIX_DEPTH_NONZERO or pos['depth_mm'] >= 1.0
                        if (not holding and not pos['touches_edge'] and depth_ok
                                and time.ticks_diff(now, last_send_time) > UART_INTERVAL_MS):
                            if PRINT_OBJECT:
                                ang_s = "" if pos['angle'] is None else " A:{:.1f}".format(pos['angle'])
                                print("Object: {} | Conf: {:.2f}{} | X:{:.1f} Y:{:.1f} D:{:.1f}mm S:{:.1f}px{}".format(
                                    label, tracker.score, tag, pos['x_mm'], pos['y_mm'], pos['depth_mm'],
                                    pos['size_pix'], ang_s))
                            send_position_via_uart(pos['x_mm'], pos['y_mm'], pos['depth_mm'], verbose=False)
                            last_send_time = now

                if SEND_HEARTBEAT:
                    now = time.ticks_ms()
                    if time.ticks_diff(now, last_send_time) > HEARTBEAT_MS:
                        send_position_via_uart(0, 0, 0, verbose=False)
                        last_send_time = now

                now = time.ticks_ms()
                dt = time.ticks_diff(now, t_last)
                t_last = now
                if dt > 0:
                    frame_dt = dt
                    fps = 0.9 * fps + 0.1 * (1000.0 / dt) if fps > 0 else 1000.0 / dt
                osd.draw_string_advanced(10, OSD_H - 36, 24,
                    "FPS {:.1f}".format(fps), color=(200, 200, 200))
                if calib_text:
                    osd.draw_string_advanced(10, 10, 28, calib_text, color=(255, 0, 255))
                prof.mark("pos+draw")

                if draw_osd:
                    Display.show_image(osd_img, OSD_X, OSD_Y, Display.LAYER_OSD3)
                del np_img
                frame_no += 1
                if GC_MIN_FREE_KB > 0:
                    if gc.mem_free() < GC_MIN_FREE_KB * 1024:
                        gc.collect()
                elif frame_no % GC_EVERY == 0:
                    gc.collect()
                prof.mark("show+gc")
                prof.end_frame()
                err_count = 0

        except KeyboardInterrupt:
            print("Stopped by user.")
            break
        except Exception as e:
            if is_stop(e):
                print("Stopped by IDE.")
                break
            err_count += 1
            if err_count <= 3 or err_count % 50 == 0:
                print("Loop error ({}):".format(err_count), e)
            time.sleep(0.05)
    return 0

if __name__ == "__main__":
    try:
        detection()
    except BaseException as e:
        if is_stop(e):
            print("Stopped.")
        else:
            print("Fatal error:", e)
            raise
    finally:
        cleanup()   # 无论怎样退出都释放摄像头/显示/媒体资源，下次运行才不会报 snapshot failed
