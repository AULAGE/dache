# -*- coding: utf-8 -*-
"""
K230 程序（k230_sdcard/main.py）纯函数的离线测试，CPython + numpy 就能跑：

    cd k230_sdcard/tests
    python3 test_k230.py          # 全部
    python3 test_k230.py -v       # 显示每个用例

ulab.numpy 用 numpy 代替（见 k230_stub.py），所以这里只能验证"算得对不对"，
不能代表板子上的耗时；耗时要在板子上看 Profiler 输出。
"""
import math
import os
import random
import re
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import k230_stub  # noqa: E402
import synth  # noqa: E402

FW, FH = synth.FW, synth.FH
REPO = os.path.dirname(os.path.dirname(HERE))


def F(v):
    return float(v)


def run_refine(m, cx, cy, side, ang, hint=None, side_b=None, pad=0.05, color=(200, 40, 40)):
    img = synth.render(cx, cy, side, ang, side_b=side_b, color=color)
    box = synth.det_box(cx, cy, side, ang, pad=pad, side_b=side_b)
    return img, box, m.color_refine(img, box, FW, FH, angle_hint=hint)


def rect_corner_set(obb):
    """矩形的 4 个角点（四舍五入到 0.01，排序），用来判断两个 obb 是不是同一个矩形"""
    import k230_stub as _s  # noqa
    cx, cy, w, h, a = [F(v) for v in obb]
    r = math.radians(a)
    c, s = math.cos(r), math.sin(r)
    pts = [(cx + dx * c - dy * s, cy + dx * s + dy * c)
           for (dx, dy) in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]
    return sorted((round(x, 2), round(y, 2)) for x, y in pts)


# ====================================================================== 协议
class TestProtocol(unittest.TestCase):
    def setUp(self):
        self.m = k230_stub.load_main()

    def test_crc_and_frame_match_raspberrypi(self):
        """K230 发的帧要能被树莓派 devices.K230 的解析逻辑接受（同样的 CRC8、大端、0x0D 结尾）"""
        sys.path.insert(0, os.path.join(REPO, "raspberrypi", "grasp"))
        try:
            import devices
            pi_crc8 = devices.crc8
        except Exception:      # 没有 pyserial 等依赖时，按协议自己算
            def pi_crc8(data):
                crc = 0
                for b in data:
                    crc ^= b
                    for _ in range(8):
                        crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
                return crc
        self.m.uart.sent.clear()
        self.m.send_position_via_uart(-12.6, 345.4, 151.5, verbose=False)
        f = self.m.uart.sent[-1]
        self.assertEqual(len(f), 10)
        self.assertEqual(f[:2], b"\xAA\x55")
        self.assertEqual(f[9], 0x0D)
        self.assertEqual(pi_crc8(f[:8]), f[8])
        self.assertEqual(int.from_bytes(f[2:4], "big", signed=True), -13)
        self.assertEqual(int.from_bytes(f[4:6], "big", signed=True), 345)
        self.assertEqual(int.from_bytes(f[6:8], "big", signed=False), 152)

    def test_heartbeat_frame(self):
        self.m.uart.sent.clear()
        self.m.send_position_via_uart(0, 0, 0, verbose=False)
        f = self.m.uart.sent[-1]
        self.assertEqual(f[2:8], b"\x00" * 6)

    def test_bug_tiny_depth_looks_like_heartbeat(self):
        """[bug 记录] 深度 < 0.5mm 会被截成 D=0，树莓派会当成心跳（FIX_DEPTH_NONZERO 在主循环里拦掉）"""
        self.m.uart.sent.clear()
        self.m.send_position_via_uart(10, 10, 0.4, verbose=False)
        f = self.m.uart.sent[-1]
        self.assertEqual(int.from_bytes(f[6:8], "big"), 0)


# ====================================================================== 坐标
class TestGeometry(unittest.TestCase):
    def setUp(self):
        self.m = k230_stub.load_main()

    def test_rotate_point_is_clockwise_90(self):
        m = self.m
        # 原始画面左上角 -> 旋转后（宽 = FH）右上角；原始右上角 -> 右下角
        self.assertEqual(m.rotate_point(0, 0, FW, FH), (FH, 0))
        self.assertEqual(m.rotate_point(FW, 0, FW, FH), (FH, FW))
        self.assertEqual(m.rotate_point(FW / 2, FH / 2, FW, FH), (FH / 2, FW / 2))

    def test_rotate_box_matches_rotate_point(self):
        m = self.m
        for _ in range(50):
            x1, x2 = sorted(random.uniform(0, FW) for _ in range(2))
            y1, y2 = sorted(random.uniform(0, FH) for _ in range(2))
            pts = [m.rotate_point(x, y, FW, FH) for x, y in ((x1, y1), (x2, y1), (x1, y2), (x2, y2))]
            bx = m.rotate_box(x1, y1, x2, y2, FW, FH)
            self.assertAlmostEqual(bx[0], min(p[0] for p in pts))
            self.assertAlmostEqual(bx[1], min(p[1] for p in pts))
            self.assertAlmostEqual(bx[2], max(p[0] for p in pts))
            self.assertAlmostEqual(bx[3], max(p[1] for p in pts))

    def test_compute_position_signs_in_rotated_frame(self):
        """ROTATE_DEG=90 + POSITION_IN_ROTATED_FRAME：原始画面里往上（y 小）= 正向画面里往右 = X 为正"""
        m = k230_stub.load_main(USE_UNDISTORT=False)
        side = 100

        def pos(cx, cy):
            return m.compute_position([cx - side / 2, cy - side / 2, cx + side / 2, cy + side / 2],
                                      FW, FH, 24.0, None)
        c = pos(FW / 2, FH / 2)
        self.assertAlmostEqual(c['x_mm'], 0.0, places=6)
        self.assertAlmostEqual(c['y_mm'], 0.0, places=6)
        up = pos(FW / 2, FH / 2 - 100)        # 原始画面往上
        self.assertGreater(up['x_mm'], 0)
        self.assertAlmostEqual(up['y_mm'], 0.0, places=6)
        right = pos(FW / 2 + 100, FH / 2)     # 原始画面往右 = 正向画面往下
        self.assertGreater(right['y_mm'], 0)
        # 针孔模型：X = 偏移像素 * 深度 / 焦距
        self.assertAlmostEqual(up['x_mm'], 100 * up['depth_mm'] / m.FOCAL_PIX, places=6)

    def test_depth_formula_no_undistort(self):
        m = k230_stub.load_main(USE_UNDISTORT=False)
        for side in (40, 80, 150, 300):
            p = m.compute_position([600, 300, 600 + side, 300 + side], FW, FH, 24.0, None)
            self.assertAlmostEqual(p['depth_mm'], 24.0 * m.FOCAL_PIX / side, places=6)

    def test_undistort_point_inverts_distortion(self):
        """undistort_point 是 OpenCV 畸变模型 (k1,k2,p1,p2,k3) 的反函数"""
        m = self.m
        K = [m.FOCAL_PIX, m.FOCAL_PIX]
        cx, cy = FW / 2.0, FH / 2.0
        for dist in ([-0.2, 0.1, 0.0, 0.0, 0.0], [-0.1, 0.02, 0.001, -0.002, 0.0], [0.05, 0, 0, 0, 0]):
            for (xu, yu) in ((640, 360), (900, 500), (100, 50), (1250, 700), (300, 650)):
                xn, yn = (xu - cx) / K[0], (yu - cy) / K[1]
                r2 = xn * xn + yn * yn
                rad = 1 + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 ** 3
                xd = xn * rad + 2 * dist[2] * xn * yn + dist[3] * (r2 + 2 * xn * xn)
                yd = yn * rad + dist[2] * (r2 + 2 * yn * yn) + 2 * dist[3] * xn * yn
                bx, by = m.undistort_point(xd * K[0] + cx, yd * K[1] + cy, K, dist, cx, cy)
                self.assertAlmostEqual(bx, xu, delta=0.05)
                self.assertAlmostEqual(by, yu, delta=0.05)

    def test_default_dist_coeffs_bias_on_ideal_lens(self):
        """[深度误差来源] 默认 dist_coeffs 是没标定的占位值：如果镜头其实没那么大畸变，
        画面边上的深度会偏小最多约 16%（这里只记录数值，见报告"深度精度"）"""
        m = self.m
        side = 100
        errs = []
        for px in (640, 960, 1200):
            b = [px - side / 2, 360 - side / 2, px + side / 2, 360 + side / 2]
            p = m.compute_position(b, FW, FH, 24.0, None)
            errs.append(p['depth_mm'] / (24.0 * m.FOCAL_PIX / side) - 1)
        self.assertLess(abs(errs[0]), 0.01)
        self.assertLess(errs[2], -0.10)      # 画面边上偏小 >10%

    def test_two_side_pad(self):
        top, bottom, left, right, ratio = self.m.two_side_pad_param([1280, 720], [640, 640])
        self.assertEqual((top, bottom, left, right), (140, 140, 0, 0))
        self.assertAlmostEqual(ratio, 0.5)


# ====================================================================== 颜色精修 + 旋转框
class TestRefine(unittest.TestCase):
    def test_refine_accuracy_squares(self):
        """理想合成图上：各种角度和大小，中心误差 < 1px、深度误差 < 2.5%"""
        for fix in (False, True):
            m = k230_stub.load_main(USE_UNDISTORT=False, FIX_OBB_ANGLE_WRAP=fix)
            for ang in (0, 7, 22, 45, 63, 80, 89):
                for side in (60, 120, 220):
                    img, box, (nb, color, obb) = run_refine(m, 640.3, 360.7, side, ang)
                    self.assertIsNotNone(obb, (ang, side))
                    p = m.compute_position(nb, FW, FH, 24.0, obb)
                    true_d = 24.0 * m.FOCAL_PIX / side
                    self.assertLess(abs(p['depth_mm'] / true_d - 1), 0.025, (fix, ang, side, p['depth_mm']))
                    # 合成图里像素 i 覆盖 [i, i+1)，程序的像素坐标以像素中心为整数，所以差 0.5
                    self.assertLess(abs(F(obb[0]) - (640.3 - 0.5)), 1.0, (ang, side, obb))
                    self.assertLess(abs(F(obb[1]) - (360.7 - 0.5)), 1.0, (ang, side, obb))

    def test_bug_obb_angle_wrap_rectangle(self):
        """
        [bug] 长方形目标（被爪子挡住一部分的物块）角度在 90° 附近、带上一帧角度提示时：
        fit_obb 搜到 90° 后只取模成 0°、宽高没互换 -> 框被转了 90°，
        亚像素精修再把它"修"成错的尺寸，测深偏 20~30%。FIX_OBB_ANGLE_WRAP 修好。
        """
        long_side, short_side = 150, 110
        true_d = 24.0 * 944.0 / long_side
        res = {}
        for fix in (False, True):
            m = k230_stub.load_main(USE_UNDISTORT=False, FIX_OBB_ANGLE_WRAP=fix)
            img, box, (nb, color, obb) = run_refine(m, 640, 360, long_side, 88, hint=88.0, side_b=short_side)
            self.assertIsNotNone(obb)
            p = m.compute_position(nb, FW, FH, 24.0, obb)
            res[fix] = p['depth_mm'] / true_d - 1
        self.assertGreater(res[False], 0.15)          # 原版：深度偏远 >15%
        self.assertLess(abs(res[True]), 0.02)         # 修复后 < 2%

    def test_obb_wrap_fix_many_angles(self):
        """修复后：长方形在 0°/90° 边界两侧、有/没有角度提示，量到的长边都对"""
        m = k230_stub.load_main(USE_UNDISTORT=False, FIX_OBB_ANGLE_WRAP=True)
        for ang in (0.5, 1.5, 88, 88.8, 89.5):
            for hint in (None, ang, (ang + 1.0) % 90, (ang - 1.5) % 90):
                img, box, (nb, color, obb) = run_refine(m, 640, 360, 150, ang, hint=hint, side_b=110)
                if obb is None:
                    continue      # 退回正框也可以接受（不会算错）
                long_side = max(F(obb[2]), F(obb[3]))
                self.assertLess(abs(long_side - 150), 3.0, (ang, hint, obb))
                self.assertTrue(0 <= F(obb[4]) < 90)

    def test_norm_obb_angle_same_rectangle(self):
        m = k230_stub.load_main()
        for a in (-95.0, -4.5, -1e-17, 0.0, 45.0, 89.9, 90.0, 93.0, 181.0, 275.5):
            w, h, a2 = m.norm_obb_angle(30.0, 20.0, a)
            self.assertTrue(0 <= a2 < 90, a2)
            self.assertEqual(rect_corner_set([5, 7, 30.0, 20.0, a]), rect_corner_set([5, 7, w, h, a2]), a)

    def test_fast_obb_points_identical(self):
        """FAST_OBB_POINTS（布尔下标挑点）和原算法结果完全一样"""
        rng = random.Random(1)
        for i in range(40):
            side = rng.uniform(40, 260)
            ang = rng.uniform(0, 90)
            sb = side * rng.uniform(0.7, 1.0)
            cx, cy = rng.uniform(300, 980), rng.uniform(200, 520)
            hint = None if i % 2 else (ang + rng.uniform(-3, 3)) % 90
            outs = []
            for fast in (False, True):
                m = k230_stub.load_main(FAST_OBB_POINTS=fast)
                img, box, r = run_refine(m, cx, cy, side, ang, hint=hint, side_b=sb)
                outs.append(r)
            a, b = outs
            if a[2] is None or b[2] is None:
                self.assertEqual(a[2] is None, b[2] is None)
            else:
                for u, v in zip(a[2], b[2]):
                    self.assertAlmostEqual(F(u), F(v), places=4)
            if a[0] is not None:
                for u, v in zip(a[0], b[0]):
                    self.assertAlmostEqual(F(u), F(v), places=4)

    def test_fast_obb_points_fallback_when_no_bool_index(self):
        m = k230_stub.load_main(FAST_OBB_POINTS=True)
        m._bool_index = False                      # 模拟 ulab 不支持布尔下标
        img, box, (nb, color, obb) = run_refine(m, 640, 360, 120, 30)
        m2 = k230_stub.load_main(FAST_OBB_POINTS=False)
        img, box, (nb2, color2, obb2) = run_refine(m2, 640, 360, 120, 30)
        for u, v in zip(obb, obb2):
            self.assertAlmostEqual(F(u), F(v), places=5)

    def test_cache_center_color_identical(self):
        m = k230_stub.load_main()
        img = synth.render(640, 360, 120, 20)
        box = synth.det_box(640, 360, 120, 20, pad=0.05)
        cache = {}
        c1 = m.center_color_cached(img, tuple(box), FW, FH, cache)
        c2 = m.center_color_cached(img, list(box), FW, FH, cache)   # 列表和元组同一个键
        self.assertEqual(len(cache), 1)
        self.assertEqual(c1, c2)
        self.assertEqual([F(v) for v in c1], [F(v) for v in m.center_color(img, box, FW, FH)])
        r1 = m.color_refine(img, box, FW, FH, cache=cache)
        r2 = m.color_refine(img, box, FW, FH)
        for u, v in zip(r1[2], r2[2]):
            self.assertEqual(F(u), F(v))

    def test_refine_grid_ceil_accuracy(self):
        """REFINE_GRID_CEIL：小目标网格从最多 95x95 降到 48x48，精度仍然 < 2%（亚像素补回来）"""
        m0 = k230_stub.load_main(USE_UNDISTORT=False)
        m1 = k230_stub.load_main(USE_UNDISTORT=False, REFINE_GRID_CEIL=True)
        for side in (40, 55, 70, 90, 130):
            for ang in (0, 25, 60):
                for m in (m0, m1):
                    img, box, (nb, color, obb) = run_refine(m, 640, 360, side, ang)
                    self.assertIsNotNone(nb)
                    p = m.compute_position(nb, FW, FH, 24.0, obb)
                    self.assertLess(abs(p['depth_mm'] / (24.0 * 944 / side) - 1), 0.025, (side, ang))

    def test_touches_edge_when_cut_by_frame(self):
        """物块伸出画面右边界：不管角度、伸出多少，都要判为 touches_edge（不发位置帧）"""
        m = k230_stub.load_main()
        side = 150
        for ang in (0, 20, 40):
            for cut in (3, 20, 70):
                cx = FW - side / 2 + cut
                img, box, (nb, color, obb) = run_refine(m, cx, 360, side, ang, pad=0.03)
                p = m.compute_position(nb or box, FW, FH, 24.0, obb)
                self.assertTrue(p['touches_edge'], (ang, cut))

    def test_longest_run(self):
        m = k230_stub.load_main()
        self.assertEqual(m.longest_run([0, 0, 5, 5, 0, 5, 5, 0, 0], 1, 3), (2, 6))   # 允许 1 格缺口
        self.assertEqual(m.longest_run([0, 5, 5, 0, 0, 5, 5], 1, 1), (1, 2))
        self.assertEqual(m.longest_run([0, 0, 0], 1, 1), None)
        self.assertEqual(m.longest_run([0, 0, 0, 9], 1, 0), (3, 3))


# ====================================================================== 跟踪器
def det(box, score=0.8, cls=0):
    return [cls, score, box[0], box[1], box[2], box[3]]


class TestTracker(unittest.TestCase):
    def test_bug_stale_obb_sent_after_move(self):
        """
        [bug] 旋转框拟合偶尔失败（爪子挡住、和别的物块粘连）时，tracker.obb 原地保留最多 3 帧，
        但 compute_position 优先用 obb -> 物块（或相机）已经移动了，发出去的还是旧位置。
        FIX_STALE_OBB：沿用的旧框跟着正框一起平移/缩放。
        """
        out = {}
        for fix in (False, True):
            m = k230_stub.load_main(USE_UNDISTORT=False, FIX_STALE_OBB=fix)
            t = m.Tracker()
            side = 120.0
            b0 = [600, 300, 600 + side, 300 + side]
            obb0 = [600 + side / 2, 300 + side / 2, side, side, 0.0]
            t.update(det(b0), False, b0, None, True, obb0, 140)
            for _ in range(4):
                t.update(det(b0), True, b0, None, True, list(obb0), 140)
            # 相机移动了 60px，这几帧旋转框拟合失败（obb=None），只有正框
            b1 = [b0[0] + 60, b0[1], b0[2] + 60, b0[3]]
            xs = []
            for _ in range(3):
                t.update(det(b1), True, b1, None, True, None, 140)
                p = m.compute_position(t.smooth, FW, FH, 24.0, t.predicted_obb())
                xs.append(p['y_mm'])          # 原始画面 x 方向 = 正向画面 Y
            box_p = m.compute_position(t.smooth, FW, FH, 24.0, None)
            out[fix] = (xs, box_p['y_mm'])
        stale, box_y = out[False]
        fixed, _ = out[True]
        self.assertTrue(all(abs(v - stale[0]) < 1e-6 for v in stale))   # 原版：位置一动不动
        self.assertLess(abs(fixed[-1] - box_y), 0.5)                       # 修复后：跟上正框

    def test_bug_obb_smoothing_wrap_keeps_long_side(self):
        """
        [bug] 长方形（150x110）转角在 0°/90° 边界附近来回时，Tracker._update_obb 平滑后的角度取模、
        宽高不互换：跨界那一帧框被转 90°，之后宽高互相"平滑"，最长边短暂变短 -> 深度偏远 ~9%。
        输入用正确的表示（fit_obb 修好后的输出），只测跟踪器本身。FIX_OBB_ANGLE_WRAP 修好。
        """
        res = {}
        for fix in (False, True):
            m = k230_stub.load_main(FIX_OBB_ANGLE_WRAP=fix)
            t = m.Tracker()
            box = [580, 300, 700, 420]
            longest = []
            for i, a in enumerate([-1.0, -0.8, -0.5, 0.6, 0.9, 1.2, 0.8, 1.0, 1.1, 0.9]):
                w, h, na = m.norm_obb_angle(150.0, 110.0, a)
                t.update(det(box), i > 0, box, None, True, [640, 360, w, h, na], 140)
                longest.append(max(F(t.obb[2]), F(t.obb[3])))
            res[fix] = min(longest)
        self.assertLess(res[False], 140.0)
        self.assertGreater(res[True], 149.0)

    def test_predict_and_lost(self):
        m = k230_stub.load_main()
        t = m.Tracker()
        b = [100, 100, 200, 200]
        t.update(det(b), False, b, None, False, None, 140)
        for k in range(1, 4):
            nb = [100 + 10 * k, 100, 200 + 10 * k, 200]
            t.update(det(nb), True, nb, None, False, None, 140)
        self.assertTrue(t.confirmed())
        self.assertGreater(t.vel[0], 0)
        for k in range(m.LOST_TOLERANCE):
            self.assertFalse(t.mark_missed())
        self.assertTrue(t.mark_missed())
        self.assertIsNone(t.smooth)

    def test_settle_frames_after_step(self):
        """
        [延迟分析] 相机动一下（目标在画面里跳 X px）后，平滑输出要几帧才落到 1px 以内。
        7fps（dt=140ms）下应该 <= 3 帧；这是树莓派每步要等 3~5 帧的原因之一（另一部分是相机延迟）。
        """
        m = k230_stub.load_main()
        for jump in (5, 15, 40):
            t = m.Tracker()
            b = [600, 300, 720, 420]
            t.update(det(b), False, b, None, False, None, 140)
            for _ in range(5):
                t.update(det(b), True, b, None, False, None, 140)
            nb = [b[0] + jump, b[1], b[2] + jump, b[3]]
            n = 0
            while n < 20:
                t.update(det(nb), True, nb, None, False, None, 140)
                n += 1
                if abs(t.smooth[0] - nb[0]) < 1.0:
                    break
            self.assertLessEqual(n, 3, (jump, n))

    def test_select_color_gate(self):
        m = k230_stub.load_main()
        t = m.Tracker()
        b = [100, 100, 200, 200]
        t.update(det(b), False, b, [200, 40, 40], False, None, 140)
        t.update(det(b), True, b, [200, 40, 40], False, None, 140)
        cand = [det([102, 100, 202, 200], 0.9)]
        # 颜色不同 -> 不关联（已确认目标 miss<2 时返回 None 保持）
        d, matched = t.select(cand, 0.35, 0.19, lambda bb: [40, 40, 200])
        self.assertIsNone(d)
        d, matched = t.select(cand, 0.35, 0.19, lambda bb: [195, 45, 40])
        self.assertTrue(matched)


# ====================================================================== 其它
class TestMisc(unittest.TestCase):
    def test_null_osd_has_all_draw_methods(self):
        """OSD_EVERY>1 时不画的帧用 _NullOSD 代替：主循环里用到的 osd.xxx 方法它都要有"""
        m = k230_stub.load_main()
        with open(k230_stub.MAIN_PATH, encoding="utf-8") as f:
            src = f.read()
        names = set(re.findall(r"\bosd\.(\w+)\(", src))
        self.assertTrue(names)
        for n in names:
            self.assertTrue(callable(getattr(m._NULL_OSD, n, None)), n)

    def test_switch_defaults_keep_old_behaviour(self):
        """会改变结果的开关默认都关；只影响速度、结果不变的默认开"""
        m = k230_stub.load_main()
        self.assertFalse(m.FIX_OBB_ANGLE_WRAP)
        self.assertFalse(m.FIX_STALE_OBB)
        self.assertFalse(m.FIX_DEPTH_NONZERO)
        self.assertEqual(m.OBB_HOLD_FRAMES, 3)
        self.assertFalse(m.REFINE_GRID_CEIL)
        self.assertEqual(m.OSD_EVERY, 1)
        self.assertEqual(m.GC_MIN_FREE_KB, 0)
        self.assertTrue(m.PRINT_OBJECT)
        # 协议和坐标约定不能动
        self.assertEqual(m.ROTATE_DEG, 90)
        self.assertTrue(m.POSITION_IN_ROTATED_FRAME)

    def test_main_differs_from_original_only_by_switches(self):
        """所有开关都设成"原行为"时，几个核心函数的输出和板子原版程序完全一致"""
        orig_path = os.path.join(REPO, "k230_sdcard", "main_板子原版_20261102.py")
        saved = k230_stub.MAIN_PATH
        try:
            k230_stub.MAIN_PATH = orig_path
            mo = k230_stub.load_main()
        finally:
            k230_stub.MAIN_PATH = saved
        mn = k230_stub.load_main(FAST_OBB_POINTS=True, CACHE_CENTER_COLOR=True)   # 提速项开着也要一致
        rng = random.Random(7)
        for i in range(25):
            side = rng.uniform(40, 250)
            ang = rng.uniform(0, 90)
            hint = None if i % 3 == 0 else (ang + rng.uniform(-4, 4)) % 90
            sb = side * rng.uniform(0.75, 1.0)
            img = synth.render(640, 360, side, ang, side_b=sb)
            box = synth.det_box(640, 360, side, ang, pad=0.05, side_b=sb)
            a = mo.color_refine(img, box, FW, FH, angle_hint=hint)
            b = mn.color_refine(img, box, FW, FH, angle_hint=hint, cache={})
            self.assertEqual(a[2] is None, b[2] is None)
            if a[2] is not None:
                for u, v in zip(a[2], b[2]):
                    self.assertAlmostEqual(F(u), F(v), places=4)
                pa = mo.compute_position(a[0], FW, FH, 24.0, a[2])
                pb = mn.compute_position(b[0], FW, FH, 24.0, b[2])
                for k in ('x_mm', 'y_mm', 'depth_mm'):
                    self.assertAlmostEqual(pa[k], pb[k], places=3)
        # 跟踪器
        to, tn = mo.Tracker(), mn.Tracker()
        for i in range(30):
            bx = [500 + 3 * i + rng.uniform(-1, 1), 300, 620 + 3 * i, 420 + rng.uniform(-1, 1)]
            ob = None if i % 7 == 3 else [560 + 3 * i, 360, 120, 118, (i * 4.0) % 90]
            for t in (to, tn):
                t.update(det(bx), i > 0, bx, None, True, None if ob is None else list(ob), 140)
            self.assertEqual(to.smooth, tn.smooth)
            self.assertEqual(to.obb, tn.obb)

    def test_profiler_detail(self):
        m = k230_stub.load_main(PROFILE_DETAIL=True)
        p = m.Profiler(True, 2)
        img = synth.render(640, 360, 120, 20)
        box = synth.det_box(640, 360, 120, 20, pad=0.05)
        for _ in range(2):
            p.start()
            m.color_refine(img, box, FW, FH)
            p.mark("refine")
            p.end_frame()
        self.assertEqual(m._detail_order, [])      # 打印后清零

    def test_fast_obb_speed_numpy(self):
        """（只供参考）numpy 下 FAST_OBB_POINTS 的 fit_obb 提速倍数；ulab 上要以板子 Profiler 为准"""
        res = {}
        img = synth.render(640, 360, 90, 33)
        box = synth.det_box(640, 360, 90, 33, pad=0.05)
        for fast in (False, True):
            m = k230_stub.load_main(FAST_OBB_POINTS=fast)
            x1, y1, x2, y2 = box
            rx0, ry0 = int(x1 - 0.25 * (x2 - x1)), int(y1 - 0.25 * (y2 - y1))
            rx1, ry1 = int(x2 + 0.25 * (x2 - x1)), int(y2 + 0.25 * (y2 - y1))
            step = max(1, max(rx1 - rx0, ry1 - ry0) // m.REFINE_GRID)
            patch = np.array(img[:, ry0:ry1:step, rx0:rx1:step], dtype=m.NP_FLOAT)
            color = m.center_color(img, box, FW, FH)
            mask = np.clip((40 - m.color_dist_map(patch, color)) * 1000.0, 0.0, 1.0)
            t0 = time.perf_counter()
            for _ in range(20):
                m.fit_obb(mask, step, rx0, ry0, None)
            res[fast] = (time.perf_counter() - t0) / 20
        print("\n    [参考] numpy 下 fit_obb 全范围搜索：原算法 %.2fms，FAST_OBB_POINTS %.2fms（%.1f 倍）"
              % (res[False] * 1e3, res[True] * 1e3, res[False] / res[True]), end=" ")


# ====================================================================== 忽略区域（任务 10）
class _RecOSD:
    def __init__(self):
        self.lines = []

    def draw_line(self, x1, y1, x2, y2, **k):
        self.lines.append((x1, y1, x2, y2))


def run_frames(m, frames, rects):
    """按主循环的顺序（过滤 -> select -> update -> 区域检查）跑一串帧，返回每帧确认后跟踪框的中心（没有为 None）"""
    t = m.Tracker()
    out = []
    for dets in frames:
        dets, _ = m.filter_ignored(list(dets), rects)
        d, matched = t.select(dets, 0.35, 0.19, lambda bb: None)
        if d is not None:
            box = [d[2], d[3], d[4], d[5]]
            t.update(d, matched, box, None, False, None, 140)
        else:
            t.mark_missed()
        if rects and m.tracker_in_ignore(t, rects):
            t.reset()
        if t.confirmed():
            s = t.smooth
            out.append(((s[0] + s[2]) / 2, (s[1] + s[3]) / 2))
        else:
            out.append(None)
    return out


def box_at(cx, cy, side):
    return [cx - side / 2, cy - side / 2, cx + side / 2, cy + side / 2]


class TestIgnoreRegions(unittest.TestCase):
    def test_defaults(self):
        """默认打开、标定模式关；默认区域在画面左侧中间，面积约 1/60 ~ 1/30"""
        m = k230_stub.load_main()
        self.assertTrue(m.IGNORE_REGIONS_ENABLE)
        self.assertFalse(m.IGNORE_CALIB_MODE)
        self.assertTrue(m.IGNORE_REGIONS)
        for (u1, v1, u2, v2) in m.IGNORE_REGIONS:
            self.assertTrue(0 <= u1 < u2 <= 1 and 0 <= v1 < v2 <= 1)
        u1, v1, u2, v2 = m.IGNORE_REGIONS[0]
        self.assertLessEqual(u2, 0.25)                      # 左侧
        self.assertLess(v1, 0.5)                            # 竖直方向包住中间
        self.assertGreater(v2, 0.5)
        area = (u2 - u1) * (v2 - v1)
        self.assertTrue(1 / 60.0 <= area <= 1 / 30.0, area)

    def test_unrotate_is_inverse_of_rotate(self):
        for deg in (0, 90, 270):
            m = k230_stub.load_main(ROTATE_DEG=deg)
            m.ROTATED = deg in (90, 270)
            for _ in range(30):
                x, y = random.uniform(0, FW), random.uniform(0, FH)
                px, py = m.rotate_point(x, y, FW, FH)
                bx, by = m.unrotate_point(px, py, FW, FH)
                self.assertAlmostEqual(bx, x)
                self.assertAlmostEqual(by, y)

    def test_region_to_raw_pixels_rot90(self):
        """固定换算：ROTATE_DEG=90 时，显示画面 (左, 上, 右, 下) = (u1, v1, u2, v2) 对应
        原始 1280x720 的 x ∈ [v1·1280, v2·1280]、y ∈ [(1-u2)·720, (1-u1)·720]
        —— 也就是说显示画面的"左边"是原始画面的"下边"（y 大）"""
        m = k230_stub.load_main()
        r = m.ignore_region_to_raw((0.00, 0.40, 0.12, 0.60), FW, FH)
        for a, b in zip(r, (512.0, 633.6, 768.0, 720.0)):
            self.assertAlmostEqual(a, b, places=6)
        # 反过来：原始画面底边中间的点，在显示画面里是左侧中间
        u, v = m.raw_to_disp_norm(640, 700, FW, FH)
        self.assertAlmostEqual(u, 20 / 720.0)
        self.assertAlmostEqual(v, 0.5)
        # 显示画面右上角 (1, 0) -> 原始左上角 (0, 0)
        r = m.ignore_region_to_raw((0.9, 0.0, 1.0, 0.1), FW, FH)
        for a, b in zip(r, (0.0, 0.0, 128.0, 72.0)):
            self.assertAlmostEqual(a, b, places=6)

    def test_region_roundtrip_all_rotations(self):
        """任意区域：换算到原始像素后，区域内（外）的随机点换回显示归一化坐标仍在区域内（外）"""
        rng = random.Random(3)
        for deg in (0, 90, 270):
            m = k230_stub.load_main(ROTATE_DEG=deg)
            m.ROTATED = deg in (90, 270)
            for _ in range(10):
                u1, u2 = sorted(rng.uniform(0, 1) for _ in range(2))
                v1, v2 = sorted(rng.uniform(0, 1) for _ in range(2))
                rect = m.ignore_region_to_raw((u1, v1, u2, v2), FW, FH)
                for _ in range(40):
                    x, y = rng.uniform(0, FW), rng.uniform(0, FH)
                    u, v = m.raw_to_disp_norm(x, y, FW, FH)
                    inside = u1 + 1e-9 < u < u2 - 1e-9 and v1 + 1e-9 < v < v2 - 1e-9
                    outside = u < u1 - 1e-9 or u > u2 + 1e-9 or v < v1 - 1e-9 or v > v2 + 1e-9
                    if inside:
                        self.assertTrue(m.in_ignore(x, y, [rect]), (deg, u, v))
                    elif outside:
                        self.assertFalse(m.in_ignore(x, y, [rect]), (deg, u, v))

    def test_left_middle_box_reads_negative_x(self):
        """显示画面左侧中间的框，发出去的 X 为负（左）、Y 约为 0 —— 用来对照真机 ghost 读数（见报告）"""
        m = k230_stub.load_main(USE_UNDISTORT=False)
        cx, cy = m.unrotate_point(0.06 * FH, 0.5 * FW, FW, FH)
        p = m.compute_position(box_at(cx, cy, 60), FW, FH, 24.0)
        self.assertLess(p['x_mm'], 0)
        self.assertLess(abs(p['y_mm']), 1.0)

    def test_filter_drops_only_centers_inside(self):
        m = k230_stub.load_main()
        rects = m.ignore_rects_raw(FW, FH)
        self.assertEqual(len(rects), 1)
        foot = det(box_at(640, 690, 50))                  # 中心在区域里（框本身伸出区域也算）
        edge = det(box_at(640, 600, 80))                  # 框碰到区域、中心在外面 -> 保留
        block = det(box_at(640, 300, 90))
        kept, n = m.filter_ignored([foot, edge, block], rects)
        self.assertEqual(n, 1)
        self.assertEqual(kept, [edge, block])
        # 关掉开关 -> 没有忽略矩形，什么都不丢
        m2 = k230_stub.load_main(IGNORE_REGIONS_ENABLE=False)
        self.assertEqual(m2.ignore_rects_raw(FW, FH), [])
        kept, n = m2.filter_ignored([foot, block], m2.ignore_rects_raw(FW, FH))
        self.assertEqual((kept, n), ([foot, block], 0))
        self.assertEqual(m.filter_ignored([], rects), ([], 0))

    def test_tracker_never_locks_claw_foot(self):
        """
        只有爪子脚（分数还更高）+ 一个物块：不忽略时跟踪器锁住爪子脚；忽略后锁住物块。
        物块从画面里消失时，不忽略会转去锁爪子脚，忽略后就是"没有目标"（只发心跳）
        """
        m = k230_stub.load_main()
        rects = m.ignore_rects_raw(FW, FH)
        foot = det(box_at(640, 690, 50), score=0.9)
        block = det(box_at(500, 300, 70), score=0.6)
        frames = [[foot, block]] * 6 + [[foot]] * 12
        on = run_frames(m, frames, rects)
        off = run_frames(m, frames, [])
        self.assertTrue(all(c is not None and m.in_ignore(c[0], c[1], rects) for c in off[1:6]))
        self.assertTrue(all(c is not None and abs(c[0] - 500) < 1 for c in on[1:6]))
        self.assertTrue(all(c is None for c in on[-4:]))
        self.assertTrue(all(c is not None and m.in_ignore(c[0], c[1], rects) for c in off[-4:]))

    def test_locked_target_dropped_when_entering_region(self):
        """已经锁定的目标一步步移进区域：中心一进区域立即放弃，之后不会再锁上"""
        m = k230_stub.load_main()
        rects = m.ignore_rects_raw(FW, FH)
        frames = [[det(box_at(640, 500 + 10 * k, 50))] for k in range(20)]   # y 从 500 走到 690（区域 y>=633.6）
        out = run_frames(m, frames, rects)
        locked = [c for c in out if c is not None]
        self.assertTrue(locked)
        self.assertTrue(all(not m.in_ignore(c[0], c[1], rects) for c in locked))
        self.assertTrue(all(c is None for c in out[-5:]))
        # 丢帧保持中：预测框（按速度外推）进入区域也立即放弃，不用等 LOST_TOLERANCE 帧
        t = m.Tracker()
        for k in range(8):
            b = box_at(640, 560 + 8 * k, 50)
            t.update(det(b), k > 0, b, None, False, None, 140)
        self.assertFalse(m.tracker_in_ignore(t, rects))
        n = 0
        while not m.tracker_in_ignore(t, rects) and n < m.LOST_TOLERANCE:
            t.mark_missed()
            n += 1
        self.assertTrue(m.tracker_in_ignore(t, rects))
        self.assertLess(n, m.LOST_TOLERANCE)

    def test_tracker_in_ignore(self):
        m = k230_stub.load_main()
        rects = m.ignore_rects_raw(FW, FH)
        t = m.Tracker()
        self.assertFalse(m.tracker_in_ignore(t, rects))
        b = box_at(640, 690, 50)
        t.update(det(b), False, b, None, False, None, 140)
        self.assertTrue(m.tracker_in_ignore(t, rects))
        self.assertFalse(m.tracker_in_ignore(t, []))

    def test_osd_dashed_region(self):
        """OSD 虚线框：线段都在区域边上、在 OSD 范围内，并覆盖一半以上的周长"""
        m = k230_stub.load_main()
        segs, labels = m.ignore_osd_segments(m.OSD_W, m.OSD_H)
        self.assertEqual(len(labels), len(m.IGNORE_REGIONS))
        u1, v1, u2, v2 = m.IGNORE_REGIONS[0]
        x1, y1 = int(u1 * m.OSD_W), int(v1 * m.OSD_H)
        x2, y2 = int(u2 * m.OSD_W), int(v2 * m.OSD_H)
        total = 0.0
        for (xa, ya, xb, yb) in segs:
            for x, y in ((xa, ya), (xb, yb)):
                self.assertTrue(0 <= x < m.OSD_W and 0 <= y < m.OSD_H)
                self.assertTrue(x1 - 1 <= x <= x2 + 1 and y1 - 1 <= y <= y2 + 1)
                self.assertTrue(abs(x - x1) <= 1 or abs(x - x2) <= 1 or abs(y - y1) <= 1 or abs(y - y2) <= 1)
            total += math.hypot(xb - xa, yb - ya)
        perim = 2 * ((x2 - x1) + (y2 - y1))
        self.assertGreater(total, 0.5 * perim)
        self.assertLess(total, perim + 1)
        # 主循环画虚线用的 osd 方法 _NullOSD 也要有
        rec = _RecOSD()
        for s in segs:
            rec.draw_line(*s, color=(255, 0, 0), thickness=2)
            m._NULL_OSD.draw_line(*s, color=(255, 0, 0), thickness=2)
        self.assertEqual(len(rec.lines), len(segs))

    def test_calib_mode_output(self):
        m = k230_stub.load_main()
        rects = m.ignore_rects_raw(FW, FH)
        cal = m.IgnoreCalib(FW, FH, 200)
        foot = det(box_at(640, 690, 50), score=0.71)
        block = det(box_at(640, 300, 90), score=0.8)
        txt = cal.feed([foot, block], rects, ["red"], 1000)
        self.assertIn("[IGN-CAL]", txt)
        self.assertEqual(txt.count("[忽略]"), 1)
        self.assertIn("red 0.71", txt)
        u, v, w, h, ign = cal.describe(foot, rects)
        self.assertTrue(ign)
        self.assertAlmostEqual(u, 30 / 720.0)
        self.assertAlmostEqual(v, 0.5)
        self.assertAlmostEqual(w, 50 / 720.0)
        self.assertAlmostEqual(h, 50 / 1280.0)
        self.assertIsNone(cal.feed([foot], rects, ["red"], 1100))    # 没到打印间隔
        self.assertIsNotNone(cal.feed([], rects, ["red"], 1300))
        self.assertEqual(cal.count, 3)
        # 只喂爪子脚 -> 建议区域包住它的中心
        cal2 = m.IgnoreCalib(FW, FH, 0)
        for dy in (-5, 0, 5):
            cal2.feed([det(box_at(640 + dy, 690, 50))], rects, [], 0)
        sug = cal2.suggestion()
        self.assertTrue(sug[0] <= u <= sug[2] and sug[1] <= 0.5 <= sug[3], sug)
        self.assertIsNone(m.IgnoreCalib(FW, FH, 0).suggestion())


# ====================================================================== 标定工具
class TestDepthFit(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
        import depth_fit
        self.df = depth_fit

    def test_fit_size_recovers_focal_and_offset(self):
        f_true, off_true, real = 910.0, 12.0, 24.0
        rows = []
        for z in (90, 120, 160, 220, 300):
            s = real * f_true / (z - off_true)
            rows.append([z, real * 944.0 / s, s])
        r = self.df.fit(rows, real)
        self.assertAlmostEqual(r["FOCAL_PIX"], f_true, places=3)
        self.assertAlmostEqual(r["DEPTH_OFFSET"], off_true, places=3)

    def test_fit_depth_only(self):
        # 板子上当时 DEPTH_SCALE=1.1、DEPTH_OFFSET=-5；真实 Z = 0.95 * 读数 + 8
        rows = [[0.95 * d + 8, d] for d in (100, 150, 200, 260)]
        r = self.df.fit(rows, 24.0, 1.1, -5.0)
        # 新参数下的读数应该等于真实值：raw = (d + 5) / 1.1
        for z, d in rows:
            raw = (d + 5.0) / 1.1
            self.assertAlmostEqual(raw * r["DEPTH_SCALE"] + r["DEPTH_OFFSET"], z, places=6)


if __name__ == "__main__":
    unittest.main()
