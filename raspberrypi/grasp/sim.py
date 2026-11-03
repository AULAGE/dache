"""
仿真环境（眼在手上）：不接任何硬件，在电脑上验证任务逻辑和参数。

几何：
  小车中心 → 前方 PIVOT_FWD 处是机械臂底座转轴（底座装在升降台上）
  observe 姿态下相机在转轴外 r_cam_true 处，朝向 = 车头朝向 - 底座角度（底座右转为正）
  ready / reach 姿态相机往前伸 CAM_ADV[姿态] mm
  相机高度 = CAM_Z0 + 升降轴高度

原有的误差来源：
  - K230 尺寸法测深有固定的系统误差（每台车不同），x/y 按同样比例偏；另有随机噪声和偶尔丢帧
  - 真实相机半径和 config.R_CAM_MM 不一样（量得不准）
  - 底盘转向、直行 ±5% 误差，直行会轻微跑偏
  - 机械爪：观察姿态下真实误差在抓取容差内才抓得住，且有 10% 随机失败

任务 02 新增的真实因素（每项都由 config.py 的 SIM_* 开关控制，默认开启）：
  1 SIM_K230_TRACKER   逐帧模拟 K230：约 7 帧/秒，每帧过一遍和板子程序同样的 Tracker
                       （中值 + 自适应 EMA + 死区、2 帧确认、丢帧时"预测/holding"不发帧、
                       跳变太大要等 2 帧才换目标），运动过程中的帧带运动模糊
  2 SIM_CHASSIS_DYNAMICS 底盘用真机同一套 WheeltecChassis.drive()（剩余距离降速 + 提前松油门 + 学 τ），
                       下面是一阶滞后的假控制器 + 20Hz 假回传帧，松油门后会滑行
  3 SIM_SERVO_DEADBAND_US / SIM_SERVO_BACKLASH_US  底座舵机死区和换向回差
  4 SIM_PUSH_P         机械臂伸出时和物块有小偏差 → 有概率把物块推开 10~30mm
  5 SIM_DEPTH_NONLINEAR 尺寸法测深：像素噪声 + 每台车固定的框边长偏差 → 误差随距离平方增长
  6 SIM_ARM_USB_DROP_P 机械臂 CH340 偶尔掉线 0.5~2s，走真机同一套 Arm.move 重连逻辑
  7 SIM_FALSE_TARGET_P 偶尔出现假目标，持续几帧
新因素都用独立的随机数流，关掉某一项时其余随机序列基本不变，方便逐项对比。

任务 09 新增（默认全关，原来的仿真结果不变；python3 sim_accuracy.py 打开它们对比）：
  8 SIM_GRASP_BIAS_MM  每台车固定的爪子落点横向偏差（偏右为正），SIM_GRASP_BIAS_VAR_MM 让它随深度变；
                       SIM_GRIP_TOL_MM 横向抓取容差（mm，代替原来的 3°）
  9 SIM_REACH_SHIFT    伸出时机械臂重心外移，把底座推到回差间隙的一侧（每台车固定哪一侧）
  10 SIM_REACH_SAG_MM  两点关节插值在中间深度时爪子高度偏（抛物线，t=0.5 处最大）
"""
import math
import random

from devices import Arm, Measurement, WheeltecChassis, _median_measure

# 仿真用的机械臂姿态：3 号舵机取不同的值，用来区分当前是哪个姿态
SIM_POSES = {
    "observe":    {2: 1500, 3: 1500, 4: 1500, 5: 1500},
    "ready":      {2: 1500, 3: 1450, 4: 1500, 5: 1500},
    "reach_near": {2: 1500, 3: 1300, 4: 1500, 5: 1500},
    "reach_far":  {2: 1500, 3: 1200, 4: 1500, 5: 1500},
    "carry":      {2: 1500, 3: 1600, 4: 1500, 5: 1500},
    "drop":       {2: 1500, 3: 1700, 4: 1500, 5: 1500, 6: 1500},
}
CAM_ADV = {"observe": 0.0, "ready": 40.0, "reach": 100.0, "carry": 0.0, "drop": 0.0}

# K230 板子程序（k230_sdcard/main.py）里的常数
K_FOCAL_PIX = 944.0        # FOCAL_PIX
K_BLOCK_MM = 24.0          # BLOCK_REAL_SIZE
K_CONFIRM_FRAMES = 2       # CONFIRM_FRAMES
K_LOST_TOLERANCE = 6       # LOST_TOLERANCE
K_TRACK_DIST = 0.8         # TRACK_DIST
K_SMOOTH_WINDOW = 3        # SMOOTH_WINDOW
K_REF_FRAME_MS = 33.0      # REF_FRAME_MS
K_ALPHA_MIN, K_ALPHA_MAX = 0.25, 0.85
K_MOTION_REF = 0.12        # MOTION_REF
K_DEADBAND_PIX = 0.6       # DEADBAND_PIX
K_EDGE_MARGIN = 4          # EDGE_MARGIN


def pose_of(p3):
    if p3 == 1500:
        return "observe"
    if p3 == 1450:
        return "ready"
    if 1200 <= p3 <= 1300:
        return "reach"
    if p3 == 1600:
        return "carry"
    return "drop"


def _c(cfg, name, default):
    return getattr(cfg, name, default)


class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        if s > 0:
            self.t += s


def make_car(cfg, seed):
    """每台车固定的未知量（新因素用），和每次抓取的场景无关"""
    r = random.Random(seed * 7919 + 17)
    car = {
        "box_bias_px": r.uniform(-1, 1) * _c(cfg, "SIM_BOX_BIAS_PX", 1.0),   # 框边长固定偏差（颜色精修边界）
        "coast_tau": _c(cfg, "SIM_CHASSIS_TAU_S", 0.45) * r.uniform(0.9, 1.1),
    }
    # 任务 09（排在后面抽，前面两项的值不变）
    b = _c(cfg, "SIM_GRASP_BIAS_MM", 0.0)
    lo, hi = b if isinstance(b, (tuple, list)) else (b, b)
    car["grasp_bias_mm"] = r.uniform(lo, hi)                             # 落点偏差（偏右为正）
    car["grasp_bias_var_mm"] = r.uniform(-1, 1) * _c(cfg, "SIM_GRASP_BIAS_VAR_MM", 0.0)   # 近→远变化量
    car["reach_side"] = 1 if r.random() < 0.5 else -1                    # 伸出时底座被推向回差间隙的哪一侧
    return car


class World:
    CAM_Z0 = 100.0     # 升降轴在 0 时相机离地高度 mm
    PIVOT_FWD = 80.0   # 底座转轴在小车中心前方多远

    def __init__(self, cfg, rng, depth_bias, r_cam_true, car=None):
        self.cfg = cfg
        self.rng = rng
        self.depth_bias = depth_bias
        self.r_cam = r_cam_true
        self.car = car or {"box_bias_px": 0.0, "coast_tau": 0.45}
        r = rng
        dist = r.uniform(300, 1200)
        if cfg.USE_CHASSIS and not cfg.CHASSIS_ACKERMANN and r.random() < 0.15:
            ang = math.radians(r.uniform(120, 240))     # 有时物块在车后面
        else:
            ang = math.radians(r.uniform(-60, 60))
        self.block = [dist * math.cos(ang), dist * math.sin(ang),
                      self.CAM_Z0 + 10 * r.uniform(5, 40)]   # x 前, y 左, z 高
        if not cfg.USE_CHASSIS:       # 以底座转轴为圆心放，保证底座转过去之后够得着
            reach = r_cam_true + r.uniform(cfg.REACH_NEAR_D, cfg.REACH_FAR_D) / depth_bias
            self.block[0] = self.PIVOT_FWD + reach * math.cos(ang)
            self.block[1] = reach * math.sin(ang)
        self.robot = [0.0, 0.0, 0.0]                         # x, y, 朝向(rad, 左转为正)
        self.lift_cm = 0.0
        self.base_deg = 0.0
        self.pose = "observe"
        self.held = False
        self.held_view = None
        self.picked = False
        self.removed = False
        self.pushes = 0              # 统计：物块被机械臂推开的次数
        # 运动窗口：[开始, 结束, 开始前的真实视图 or None, 是否插值]
        self.clock = None
        self.motion = None
        self.listeners = []          # 状态改变前先让相机把之前的帧处理完

    def camera(self, pose=None):
        rx, ry, th = self.robot
        px = rx + self.PIVOT_FWD * math.cos(th)
        py = ry + self.PIVOT_FWD * math.sin(th)
        h = th - math.radians(self.base_deg)
        r = self.r_cam + CAM_ADV[pose or self.pose]
        return px + r * math.cos(h), py + r * math.sin(h), self.CAM_Z0 + 10 * self.lift_cm, h

    def true_view(self, pose=None, fov=True):
        """相机看到的真实值 (x右, y下, d)；看不到返回 None"""
        if self.removed:
            return None
        if self.held:      # 物块在爪子里：只在 reach 姿态下看得到，且相对相机不动
            return self.held_view if (pose or self.pose) == "reach" else None
        cx, cy, cz, h = self.camera(pose)
        dx, dy = self.block[0] - cx, self.block[1] - cy
        fwd = dx * math.cos(h) + dy * math.sin(h)
        left = -dx * math.sin(h) + dy * math.cos(h)
        down = cz - self.block[2]
        if fwd < 40:
            return None
        if fov:
            # 和板子上的 K230 程序一致：物块碰到画面边缘（touches_edge）就不发位置帧，
            # 所以要把物块本身的大小（24mm）算进去；太远也识别不出来
            half = math.degrees(math.atan2(12.0, fwd))
            if (abs(math.degrees(math.atan2(left, fwd))) + half > self.cfg.FOV_X_DEG
                    or abs(math.degrees(math.atan2(down, fwd))) + half > self.cfg.FOV_Y_DEG
                    or fwd > self.cfg.SIM_MAX_DETECT_MM):
                return None
        return -left, down, fwd

    # ---------------- 新因素用的工具
    def nonlinear(self):
        return self.cfg.SIM_DEPTH_NONLINEAR if hasattr(self.cfg, "SIM_DEPTH_NONLINEAR") else False

    def measured_clean(self, v):
        """真实视图 → K230 无噪声时的读数（含每台车的系统误差）。用户按 K230 读数标定 GOAL"""
        k = self.depth_bias
        if not self.nonlinear():
            return v[0] * k, v[1] * k, v[2] * k
        fs = K_FOCAL_PIX * K_BLOCK_MM
        dm = k * fs / (fs / v[2] + self.car["box_bias_px"])
        return v[0] * dm / v[2], v[1] * dm / v[2], dm

    def touch(self, dur, interp=True):
        """执行器要改变状态了：先让相机处理到现在为止的帧，再记录运动窗口（窗口内的帧有运动模糊）"""
        if self.clock is None:
            return
        now = self.clock.now()
        for f in self.listeners:
            f(now)
        if self.motion and now < self.motion[1]:
            self.motion[1] = max(self.motion[1], now + dur)
        else:
            self.motion = [now, now + dur, self.true_view(fov=False) if interp else None, interp]

    def view_at(self, t):
        """返回 (视图, 是否在运动中)"""
        cur = self.true_view(fov=False)
        m = self.motion
        if not m or not m[0] <= t < m[1]:
            return cur, False
        if not m[3]:
            return cur, True
        old = m[2]
        if old is None or cur is None:
            return (old if t - m[0] < (m[1] - m[0]) / 2 else cur), True
        a = (t - m[0]) / max(m[1] - m[0], 1e-6)
        return tuple(o + a * (c - o) for o, c in zip(old, cur)), True


# ============================================================ K230（逐帧 + 跟踪器）
def _median(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class _Tracker:
    """和 k230_sdcard/main.py 的 Tracker 同样的关联 / 平滑 / 预测逻辑（去掉了颜色和旋转框）"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.history = []
        self.smooth = None
        self.vel = [0.0, 0.0]
        self.hits = 0
        self.miss = 0
        self.kind = None

    def confirmed(self):
        return self.smooth is not None and self.hits >= K_CONFIRM_FRAMES

    def predicted(self):
        k = self.miss + 1
        dx, dy = self.vel[0] * k, self.vel[1] * k
        s = self.smooth
        return [s[0] + dx, s[1] + dy, s[2] + dx, s[3] + dy]

    def select(self, cands):
        """cands: [(kind, box, score)]，返回 (cand, matched)"""
        if self.smooth is not None:
            p = self.predicted()
            pcx, pcy = (p[0] + p[2]) / 2, (p[1] + p[3]) / 2
            psize = max(p[2] - p[0], p[3] - p[1], 1)
            best, best_v = None, 0.0
            for c in cands:
                b = c[1]
                d = math.hypot((b[0] + b[2]) / 2 - pcx, (b[1] + b[3]) / 2 - pcy)
                v = 0.5 * (1 - d / (K_TRACK_DIST * psize)) if d < K_TRACK_DIST * psize else 0.0
                if v > best_v:
                    best, best_v = c, v
            if best is not None:
                return best, True
            if self.confirmed() and self.miss < 2:
                return None, False
        best, best_v = None, -1
        for c in cands:
            b = c[1]
            v = c[2] * math.sqrt(max(b[2] - b[0], 0) * max(b[3] - b[1], 0))
            if v > best_v:
                best, best_v = c, v
        return best, False

    def update(self, cand, matched, dt_ms):
        if not matched:
            self.reset()
        box = list(cand[1])
        self.kind = cand[0]
        if self.smooth is None:
            self.history = [box]
            self.smooth = list(box)
            self.hits, self.miss = 1, 0
            return
        size = max(self.smooth[2] - self.smooth[0], self.smooth[3] - self.smooth[1], 1)
        motion = max(abs(box[i] - self.smooth[i]) for i in range(4)) / size
        self.history.append(box)
        if motion > K_MOTION_REF:
            self.history = self.history[-2:]
        elif len(self.history) > K_SMOOTH_WINDOW:
            self.history.pop(0)
        med = [_median([h[i] for h in self.history]) for i in range(4)]
        alpha = K_ALPHA_MIN + (K_ALPHA_MAX - K_ALPHA_MIN) * max(0.0, min(1.0, motion / K_MOTION_REF))
        alpha = 1.0 - (1.0 - alpha) ** (dt_ms / K_REF_FRAME_MS)
        ocx = (self.smooth[0] + self.smooth[2]) / 2
        ocy = (self.smooth[1] + self.smooth[3]) / 2
        for i in range(4):
            d = med[i] - self.smooth[i]
            if abs(d) > K_DEADBAND_PIX:
                self.smooth[i] += alpha * d
        steps = self.miss + 1
        self.vel[0] = 0.5 * self.vel[0] + 0.5 * ((self.smooth[0] + self.smooth[2]) / 2 - ocx) / steps
        self.vel[1] = 0.5 * self.vel[1] + 0.5 * ((self.smooth[1] + self.smooth[3]) / 2 - ocy) / steps
        self.hits += 1
        self.miss = 0

    def mark_missed(self):
        if self.smooth is None:
            return
        self.miss += 1
        if self.miss > K_LOST_TOLERANCE:
            self.reset()


class SimCam:
    def __init__(self, world, clock, rng, rng_f=None):
        self.w, self.clock, self.rng = world, clock, rng
        self.rf = rng_f or random.Random(0)
        cfg = world.cfg
        self.frame_model = _c(cfg, "SIM_K230_TRACKER", False)
        self.trk = _Tracker()
        self.sent = []                       # (t, x, y, d)：K230 真正发出来的位置帧
        self.t_next = clock.now() + self.rf.uniform(0, 1.0 / cfg.K230_FPS)
        self.fake_left = 0
        self.fake_box = None
        self.half_w = K_FOCAL_PIX * math.tan(math.radians(cfg.FOV_X_DEG))
        self.half_h = K_FOCAL_PIX * math.tan(math.radians(cfg.FOV_Y_DEG))
        self.frames_total = 0
        self.fake_sent = 0
        world.listeners.append(self._advance)

    # ---------------- 原版（一次测量只算一次噪声）
    def _measure_legacy(self, n):
        v = self.w.true_view()
        self.clock.sleep(n / float(self.w.cfg.K230_FPS))   # 实测约 7 帧/秒
        if v is None or self.rng.random() < 0.03:
            return None
        x, y, d = v
        g = self.rng.gauss
        if self.w.nonlinear():
            xm, ym, dm = self.w.measured_clean(v)
            sp = _c(self.w.cfg, "SIM_PIX_NOISE", 0.5)
            sd = dm * dm / (K_FOCAL_PIX * K_BLOCK_MM) * sp * math.sqrt(2) / math.sqrt(n)
            sx = sp / math.sqrt(2) * dm / K_FOCAL_PIX / math.sqrt(n)
            return Measurement(xm + g(0, sx + 0.3), ym + g(0, sx + 0.3), dm + g(0, sd + 0.5), n)
        k = self.w.depth_bias
        return Measurement(x * k + g(0, 1.0), y * k + g(0, 1.0), d * k + g(0, 0.01 * d + 2), n)

    # ---------------- 逐帧
    def _box_of(self, v):
        """真实视图 → 带噪声的检测框（像素，画面中心为原点）"""
        x, y, d = v
        f = K_FOCAL_PIX
        cx, cy, s = f * x / d, f * y / d, f * K_BLOCK_MM / d
        g = self.rf.gauss
        if self.w.nonlinear():
            sp = _c(self.w.cfg, "SIM_PIX_NOISE", 0.5)
            s += self.w.car["box_bias_px"]
            e = [g(0, sp) for _ in range(4)]
        else:
            # 和原版仿真的噪声量级相当（原版是对整次测量加噪声，逐帧要大一些，中值/EMA 后差不多）
            sc = 1.5 * 1.0 * f / d
            ss = 1.5 * s * (0.01 * d + 2) / d
            ex, ey, es = g(0, sc), g(0, sc), g(0, ss)
            e = [ex - es / 2, ey - es / 2, ex + es / 2, ey + es / 2]
        return [cx - s / 2 + e[0], cy - s / 2 + e[1], cx + s / 2 + e[2], cy + s / 2 + e[3]]

    def _frame(self, t, dt_ms):
        cfg, rf = self.w.cfg, self.rf
        self.frames_total += 1
        v, moving = self.w.view_at(t)
        cands = []
        if v is not None and v[2] <= cfg.SIM_MAX_DETECT_MM:
            b = self._box_of(v)
            cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
            seen = abs(cx) < self.half_w and abs(cy) < self.half_h     # 中心在画面里就能检出（部分可见）
            if seen and rf.random() >= 0.03 and (not moving or rf.random() < _c(cfg, "SIM_BLUR_DETECT_P", 0.5)):
                cands.append(("real", b, 0.8))
        # 7 误识别：别的东西被认成物块，持续几帧
        p_fake = _c(cfg, "SIM_FALSE_TARGET_P", 0.0)
        if self.fake_left <= 0 and p_fake > 0 and rf.random() < p_fake:
            lo, hi = _c(cfg, "SIM_FALSE_TARGET_FRAMES", (2, 5))
            self.fake_left = rf.randint(lo, hi)
            d = rf.uniform(150, 800)
            s = K_FOCAL_PIX * K_BLOCK_MM / d * rf.uniform(0.8, 1.3)
            cx = rf.uniform(-0.8, 0.8) * self.half_w
            cy = rf.uniform(-0.8, 0.8) * self.half_h
            self.fake_box = [cx - s / 2, cy - s / 2, cx + s / 2, cy + s / 2]
            self.fake_score = rf.uniform(0.5, 0.9)
        if self.fake_left > 0:
            self.fake_left -= 1
            b = [q + rf.gauss(0, 0.5) for q in self.fake_box]
            cands.append(("fake", b, self.fake_score))

        if _c(cfg, "SIM_K230_TRACKER_FILTER", True):
            cand, matched = self.trk.select(cands)
            if cand is not None:
                self.trk.update(cand, matched, dt_ms)
            else:
                self.trk.mark_missed()
            if not self.trk.confirmed() or self.trk.miss > 0:
                return
            box = self.trk.smooth
        else:
            if not cands:
                return
            box = max(cands, key=lambda c: c[2])[1]
        # K230 compute_position + touches_edge：碰边缘不发
        hw, hh = self.half_w - K_EDGE_MARGIN, self.half_h - K_EDGE_MARGIN
        if box[0] <= -hw or box[2] >= hw or box[1] <= -hh or box[3] >= hh:
            return
        s = max(box[2] - box[0], box[3] - box[1])
        if s <= 2:
            return
        dm = self.w.depth_bias * K_FOCAL_PIX * K_BLOCK_MM / s
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        self.sent.append((t, cx * dm / K_FOCAL_PIX, cy * dm / K_FOCAL_PIX, dm))
        if self.trk.kind == "fake":
            self.fake_sent += 1
        if len(self.sent) > 300:
            del self.sent[:100]

    def _advance(self, now):
        if not self.frame_model:
            return
        fps = float(self.w.cfg.K230_FPS)
        while self.t_next <= now:
            dt = 1.0 / fps + self.rf.uniform(-0.01, 0.01)
            self._frame(self.t_next, dt * 1000.0)
            self.t_next += dt

    def measure(self, since, n, timeout):
        if not self.frame_model:
            return self._measure_legacy(n)
        deadline = self.clock.now() + timeout
        while True:
            self._advance(self.clock.now())
            fr = [f for f in self.sent if f[0] >= since]
            if len(fr) >= n or self.clock.now() > deadline:
                return _median_measure(fr, n)
            # 真机每 20ms 查一次；这里直接跳到下一帧（或超时）
            self.clock.sleep(max(0.02, min(self.t_next, deadline + 0.001) - self.clock.now()))


# ============================================================ 升降轴
class SimLift:
    MAX = 50.0

    def __init__(self, world, clock):
        self.w, self.clock = world, clock
        self._done_at = 0.0

    def position(self):
        return self.w.lift_cm

    def move_to(self, cm, timeout=30):
        cm = max(0.0, min(self.MAX, cm))
        dur = abs(cm - self.w.lift_cm) / 8.0 + 0.2
        self.w.touch(dur)
        self.w.lift_cm = cm
        self.clock.sleep(dur)
        return cm

    def move_by(self, cm, timeout=30):
        return self.move_to(self.w.lift_cm + cm)

    def start_move_by(self, cm):
        target = max(0.0, min(self.MAX, self.w.lift_cm + cm))
        dur = abs(target - self.w.lift_cm) / 8.0 + 0.2
        self.w.touch(dur)
        self._done_at = self.clock.now() + dur
        self.w.lift_cm = target      # 仿真简化：位置立即生效，时间在 wait_move 里补齐（运动中的帧按插值）

    def wait_move(self, timeout=30):
        self.clock.sleep(self._done_at - self.clock.now())
        return self.w.lift_cm

    def stop(self):
        pass


# ============================================================ 底盘
class SimChassis:
    """原版：走多少就是多少 ±5%。ackermann=True 时不能原地转，drive 的转角受最小转弯半径限制"""

    def __init__(self, world, clock, rng, ackermann=True, min_radius=600.0):
        self.w, self.clock, self.rng = world, clock, rng
        self.ackermann = ackermann
        self.min_radius = min_radius

    def turn(self, deg, timeout=15):
        if self.ackermann:
            raise RuntimeError("阿克曼底盘不能原地转向")
        self.w.touch(abs(deg) / 60.0 + 0.3)
        self.w.robot[2] -= math.radians(deg * self.rng.uniform(0.95, 1.05))   # 右转 = 朝向减小
        self.clock.sleep(abs(deg) / 60.0 + 0.3)

    def drive(self, mm, dpsi_deg=0.0, timeout=15):
        if abs(mm) < 1:
            return 0.0, 0.0
        if self.ackermann:
            lim = math.degrees(abs(mm) / self.min_radius)
            dpsi_deg = max(-lim, min(lim, dpsi_deg))
        s = mm * self.rng.uniform(0.95, 1.05)
        dpsi = math.radians(dpsi_deg * self.rng.uniform(0.9, 1.1) + self.rng.gauss(0, 0.5))
        self.w.touch(abs(mm) / 100.0 + 0.3)
        th = self.w.robot[2]
        mid = th - dpsi / 2                  # 圆弧近似：按弧中点的朝向走直线
        self.w.robot[0] += s * math.cos(mid)
        self.w.robot[1] += s * math.sin(mid)
        self.w.robot[2] = th - dpsi
        self.clock.sleep(abs(mm) / 100.0 + 0.3)
        # 真机里程计（编码器 + 陀螺仪）也有误差
        return s * self.rng.uniform(0.98, 1.02), math.degrees(dpsi) * self.rng.uniform(0.95, 1.05)

    def forward(self, mm, timeout=15):
        return self.drive(mm, 0.0)[0]

    def stop(self):
        pass


class _NoLock:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class SimWheeltec(WheeltecChassis):
    """
    真机同一个 WheeltecChassis.drive()（降速、提前松油门、学滑行 τ），只把串口换成假控制器：
      速度一阶滞后（τ = 每台车的 coast_tau，实测 100mm/s 时滑约 45mm → τ≈0.45s）
      转向：阿克曼前轮转角 = wz/vx 对应的曲率（vx=0 时回正，和固件 Vz_to_Akm_Angle 一致），舵机有 0.1s 滞后
      回传 20Hz：编码器速度 vx、陀螺仪 wz（各有误差），驱动里按回传积分里程
    """
    FB_DT = 0.05

    def __init__(self, world, clock, rng, cfg):
        self.w, self.clock, self.rng = world, clock, rng
        self.speed = cfg.CHASSIS_SPEED_MM_S
        self.coast_tau = 0.5           # 驱动的初值（和真机一样），会在 drive 里自动修正
        self.tau_file = None
        self.min_radius = cfg.ACK_MIN_RADIUS_MM
        self.ackermann = cfg.CHASSIS_ACKERMANN
        self.cmd = (0, 0)
        self.dist = 0.0
        self.yaw = 0.0
        self.fb_vx = 0
        self.voltage_mv = 25000
        self.last_fb = clock.now()
        self._lock = _NoLock()
        self._run = False
        # 任务 03 给真机驱动新增的属性（这里不调用父类 __init__，所以要自己补齐，和 config 一致）
        self.stop_model = cfg.CHASSIS_STOP_MODEL
        self.cmd_latency = cfg.CHASSIS_CMD_LATENCY_S
        self.pi_tau = cfg.CHASSIS_PI_TAU_S
        self.ref_voltage = cfg.CHASSIS_REF_VOLTAGE_V
        self.cmd_ttl = cfg.CHASSIS_CMD_TTL_S
        self.yaw_trim = math.radians(cfg.CHASSIS_YAW_TRIM_DEG_PER_M) / 1000.0
        self.gyro_lsb = 65.5
        self.cmd_t = clock.now()
        self.ttl_tripped = 0
        self.yaw_gyro = 0.0
        self.fb_wz = 0
        self.fb_flag = 0
        self.gyro_z_raw = 0
        self.last_release = None
        self.tau_true = world.car["coast_tau"]
        self.v = 0.0                   # 真实车轮速度 mm/s
        self.kappa = 0.0               # 真实曲率 rad/mm（左转为正）
        self._fb_t = 0.0
        self._scale = 1.0
        self._steer = 1.0
        self._drift = 0.0
        self._gyro = 1.0
        self._fb_wz = 0

    def _now(self):
        return self.clock.now()

    def has_feedback(self):
        return True                    # 真机回传线程一直在跑

    def _plant(self, h):
        vx, wz = self.cmd
        self.v += (vx - self.v) * min(1.0, h / self.tau_true)
        k_cmd = (wz / 1000.0) / vx if vx else 0.0
        k_cmd *= self._steer
        kmax = 1.0 / self.min_radius
        k_cmd = max(-kmax, min(kmax, k_cmd))
        self.kappa += (k_cmd - self.kappa) * min(1.0, h / 0.1)
        ground = self.v * self._scale                      # 打滑 / 轮径误差：编码器和地面不一致
        yaw_rate = ground * self.kappa + abs(ground) * self._drift
        r = self.w.robot
        r[2] += yaw_rate * h / 2
        r[0] += ground * math.cos(r[2]) * h
        r[1] += ground * math.sin(r[2]) * h
        r[2] += yaw_rate * h / 2
        self._fb_t += h
        if self._fb_t >= self.FB_DT:                       # 20Hz 回传，和驱动的 _rx 一样按"本帧速度 × 间隔"积分
            self.fb_vx = int(self.v)
            self._fb_wz = int(yaw_rate * self._gyro * 1000.0)
            self.dist += self.fb_vx * self._fb_t
            self.yaw += self._fb_wz / 1000.0 * self._fb_t
            self._fb_t = 0.0

    def _sleep(self, s):
        if abs(self.v) > 5 or self.cmd[0]:
            self.w.touch(s + 0.05, interp=False)            # 车在动：这段时间相机的帧有运动模糊
        n = max(1, int(round(s / 0.01)))
        for _ in range(n):
            self._plant(s / n)
        self.clock.sleep(s)

    def drive(self, mm, dpsi_deg=0.0, timeout=None):
        if abs(mm) < 1:
            return 0.0, 0.0
        g = self.rng
        self._scale = g.uniform(0.98, 1.02)                 # 实测里程计误差约 2%
        self._steer = g.uniform(0.9, 1.1)
        self._drift = math.radians(g.gauss(0, 0.5)) / max(abs(mm), 50.0)
        self._gyro = g.uniform(0.95, 1.05)
        self._fb_wz = 0
        self.fb_vx = int(self.v)
        out = super().drive(mm, dpsi_deg, timeout)
        # 驱动返回后车还会以 <5mm/s 蠕动一点：直接滑完
        while abs(self.v) > 0.5:
            self._plant(0.02)
            self.clock.sleep(0.02)
        self.v = 0.0
        return out

    def turn(self, deg, timeout=None):
        raise RuntimeError("阿克曼底盘不能原地转向" if self.ackermann else "差速原地转向尚未实现")

    def stop(self):
        self.cmd = (0, 0)


# ============================================================ 机械臂
class SimArmSerial:
    """
    假的机械臂 CH340：当作 serial.Serial 工厂用，每次打开返回一个新句柄。可以注入 USB 掉线：
      掉线后设备消失 0.5~2s，期间打不开；旧句柄永远写不进去（真机 USB 重新枚举后旧 fd 失效）
      掉线那一刻的帧有一部分是"写进缓冲区没报错、但没发出去"（静默丢失）
    """

    def __init__(self, arm):
        self.arm = arm

    def __call__(self, port, baud, timeout=0.1):
        a = self.arm
        if a.clock.now() < a.down_until:
            raise OSError("could not open port: No such file or directory")
        return _SimArmHandle(a, a.usb_gen)


class _SimArmHandle:
    def __init__(self, arm, gen):
        self.arm, self.gen = arm, gen

    def write(self, b):
        a = self.arm
        now = a.clock.now()
        if self.gen != a.usb_gen:
            raise OSError("write failed: device disconnected")
        p = _c(a.cfg, "SIM_ARM_USB_DROP_P", 0.0)
        if p > 0 and a.rng_u.random() < p:
            lo, hi = _c(a.cfg, "SIM_ARM_USB_DROP_S", (0.5, 2.0))
            a.down_until = now + a.rng_u.uniform(lo, hi)
            a.usb_gen += 1
            a.drops += 1
            if a.rng_u.random() < _c(a.cfg, "SIM_ARM_USB_SILENT_P", 0.0):
                a.silent_lost += 1
                return len(b)               # 已经写进缓冲区但没发出去：不报错，这一帧丢了
            raise OSError("write failed: device disconnected")
        b = bytes(b)
        n = b[4]
        ms = b[5] | (b[6] << 8)
        pulses = {b[7 + 3 * i]: b[8 + 3 * i] | (b[9 + 3 * i] << 8) for i in range(n)}
        a.apply(pulses, ms)
        return len(b)

    def flush(self):
        pass

    def close(self):
        pass


class SimArm:
    # 机械爪实际能抓住的范围（观察姿态下的测量值，比对准容差宽一些）
    GRIP_TOL = {"x_deg": 3.0, "y": 8.0, "d": 18.0}

    def __init__(self, world, clock, cfg, rng, rng_u=None, rng_s=None):
        self.w, self.clock, self.cfg, self.rng = world, clock, cfg, rng
        self.rng_u = rng_u or random.Random(1)      # USB 掉线
        self.rng_s = rng_s or random.Random(2)      # 推开物块
        self.down_until = -1.0
        self.usb_gen = 0
        self.drops = 0
        self.silent_lost = 0
        self.base_motor = self.base_out = float(cfg.BASE_CENTER)
        # 走真机同一个 Arm（Lobot 帧打包 + USB 掉线重连），只把串口换成假的
        self.dev = Arm("sim", 9600, clock, ser_factory=SimArmSerial(self),
                       robust=_c(cfg, "ARM_USB_ROBUST", False))

    @property
    def reconnects(self):
        return self.dev.reconnects

    def move(self, pulses, ms):
        self.dev.move(pulses, ms)

    def _servo_base(self, p):
        """3 舵机死区 + 换向回差：输出轴在电机位置 ±回差/2 的间隙里"""
        db = _c(self.cfg, "SIM_SERVO_DEADBAND_US", 0)
        bl = _c(self.cfg, "SIM_SERVO_BACKLASH_US", 0)
        if abs(p - self.base_motor) > db:
            self.base_motor = float(p)
        lo, hi = self.base_motor - bl / 2.0, self.base_motor + bl / 2.0
        self.base_out = min(max(self.base_out, lo), hi)
        return self.base_out

    def _misalign(self):
        """观察姿态下物块相对 GOAL 的偏差（按 K230 读数），各轴除以抓取容差，取最大"""
        c, w = self.cfg, self.w
        v = w.true_view("observe", fov=False)
        if v is None:
            return None, None
        x, y, d = w.measured_clean(v)
        g = c.GOAL
        ex = math.degrees(math.atan2(x, d)) - g["x_deg"]
        if self._lateral_mode():
            lat, tol = self._lateral(ex, d)
            en = max(abs(lat) / tol, abs(y + self._height_err(d) - g["y"]) / self.GRIP_TOL["y"],
                     abs(d - g["d"]) / self.GRIP_TOL["d"] if c.USE_CHASSIS else 0.0)
            return en, lat
        en = max(abs(ex) / self.GRIP_TOL["x_deg"], abs(y - g["y"]) / self.GRIP_TOL["y"],
                 abs(d - g["d"]) / self.GRIP_TOL["d"] if c.USE_CHASSIS else 0.0)
        return en, ex

    # ---------------- 任务 09：落点偏差 / 横向容差 / 中间深度高度偏差
    def _lateral_mode(self):
        car = self.w.car
        return (_c(self.cfg, "SIM_GRIP_TOL_MM", 0.0) > 0 or car.get("grasp_bias_mm", 0.0) != 0.0
                or car.get("grasp_bias_var_mm", 0.0) != 0.0 or _c(self.cfg, "SIM_REACH_SAG_MM", 0.0) > 0)

    def grasp_bias(self, d):
        """这台车在深度 d（K230 读数）时爪子落点相对"对准位置"偏右多少 mm"""
        c, car = self.cfg, self.w.car
        mid = (c.REACH_NEAR_D + c.REACH_FAR_D) / 2.0
        half = max(abs(c.REACH_FAR_D - c.REACH_NEAR_D) / 2.0, 1.0)
        k = max(-1.0, min(1.0, (d - mid) / half))
        return car.get("grasp_bias_mm", 0.0) + car.get("grasp_bias_var_mm", 0.0) * k

    def _lateral(self, ex_deg, d):
        """物块中心相对爪子落点的横向偏差（mm，物块偏右为正）和横向容差"""
        tol = _c(self.cfg, "SIM_GRIP_TOL_MM", 0.0) or d * math.tan(math.radians(self.GRIP_TOL["x_deg"]))
        return d * math.tan(math.radians(ex_deg)) - self.grasp_bias(d), tol

    def _height_err(self, d):
        """
        中间深度爪子高度偏差（mm）：假设"刚好抓到"需要的 3 号舵机脉宽随深度是一条抛物线
        p(t) = near + (far − near)·t − sag·4t(1−t)（1us ≈ 1mm 高度），程序发的是插值出来的脉宽。
        两点插值 → 中间偏 sag；标了 reach_mid（= p(t_mid)）后三点分段插值，偏差变小
        """
        sag = _c(self.cfg, "SIM_REACH_SAG_MM", 0.0)
        p3 = getattr(self, "reach_p3", None)
        if sag <= 0 or p3 is None or self.w.pose != "reach":
            return 0.0
        c = self.cfg
        near, far = c.POSES["reach_near"][3], c.POSES["reach_far"][3]
        t = (d - c.REACH_NEAR_D) / (c.REACH_FAR_D - c.REACH_NEAR_D)
        t = max(0.0, min(1.0, t))
        p_true = near + (far - near) * t - sag * 4 * t * (1 - t) * (1 if far < near else -1)
        return abs(p3 - p_true)

    def _maybe_push(self):
        """4 机械臂伸出时爪子蹭到物块：有小偏差时有概率把物块推开"""
        p = _c(self.cfg, "SIM_PUSH_P", 0.0)
        if p <= 0 or self.w.held:
            return
        en, ex = self._misalign()
        lo, hi = _c(self.cfg, "SIM_PUSH_RANGE", (0.3, 2.0))
        if en is None or not lo <= en <= hi or self.rng_s.random() >= p:
            return
        r = self.rng_s
        a, b = _c(self.cfg, "SIM_PUSH_MM", (10, 30))
        dist = r.uniform(a, b)
        _, _, _, h = self.w.camera("observe")
        phi = h - math.copysign(r.uniform(0, math.radians(60)), ex)    # 物块偏右 → 往右推
        self.w.block[0] += dist * math.cos(phi)
        self.w.block[1] += dist * math.sin(phi)
        self.w.pushes += 1

    def apply(self, pulses, ms):
        """控制板收到一帧：舵机开始动（状态立即生效，相机在动作时间内的帧按插值 + 模糊）"""
        c, w = self.cfg, self.w
        w.touch(ms / 1000.0 + 0.15)
        entering_reach = False
        if 3 in pulses:
            new_pose = pose_of(pulses[3])
            if new_pose == "reach" and w.pose != "reach":
                self._maybe_push()
                entering_reach = True
            if new_pose == "reach":
                self.reach_p3 = pulses[3]
            w.pose = new_pose
        if c.BASE_ID in pulses:
            out = self._servo_base(pulses[c.BASE_ID])
            w.base_deg = (out - c.BASE_CENTER) / (c.BASE_US_PER_DEG * c.BASE_DIR)
        if entering_reach and _c(c, "SIM_REACH_SHIFT", False):
            # 9 机械臂伸出去、重心外移：底座输出轴被推到回差间隙的一侧
            bl = _c(c, "SIM_SERVO_BACKLASH_US", 0)
            self.base_out = self.base_motor + w.car.get("reach_side", 1) * bl / 2.0
            w.base_deg = (self.base_out - c.BASE_CENTER) / (c.BASE_US_PER_DEG * c.BASE_DIR)
        claw = pulses.get(c.CLAW_ID)
        if claw == c.CLAW_CLOSE and not w.held:
            v = w.true_view("observe", fov=False)
            if v is None:
                return
            x, y, d = w.measured_clean(v)   # 用户是按 K230 读数标定 GOAL 的，所以按"测量值"比较
            g = c.GOAL
            ok_d = abs(d - g["d"]) <= self.GRIP_TOL["d"] if c.USE_CHASSIS else \
                c.REACH_NEAR_D - 10 <= d <= c.REACH_FAR_D + 10
            if self._lateral_mode():        # 任务 09：按毫米算横向偏差（含落点偏差），高度加上插值误差
                lat, tol = self._lateral(math.degrees(math.atan2(x, d)) - g["x_deg"], d)
                self.last_lateral_mm = lat
                ok = (abs(lat) <= tol and abs(y + self._height_err(d) - g["y"]) <= self.GRIP_TOL["y"]
                      and ok_d)
            else:
                ok = (abs(math.degrees(math.atan2(x, d)) - g["x_deg"]) <= self.GRIP_TOL["x_deg"]
                      and abs(y - g["y"]) <= self.GRIP_TOL["y"] and ok_d)
            if ok and self.rng.random() < 0.9:
                w.held_view = w.true_view("reach", fov=False)
                w.held = True
                w.picked = True
        elif claw == c.CLAW_OPEN and w.held:
            w.held = False
            if w.pose == "drop":
                w.removed = True          # 放进收纳筐了


def make_sim(cfg, seed, depth_bias, r_cam_true, car=None):
    rng = random.Random(seed)
    clock = SimClock()
    world = World(cfg, rng, depth_bias, r_cam_true, car or make_car(cfg, 0))
    world.clock = clock
    cam = SimCam(world, clock, rng, random.Random(seed * 31 + 1))
    lift = SimLift(world, clock)
    arm = SimArm(world, clock, cfg, rng, random.Random(seed * 31 + 2), random.Random(seed * 31 + 3))
    if _c(cfg, "SIM_CHASSIS_DYNAMICS", False) and cfg.CHASSIS_ACKERMANN:
        chassis = SimWheeltec(world, clock, random.Random(seed * 31 + 4), cfg)
    else:
        chassis = SimChassis(world, clock, rng, cfg.CHASSIS_ACKERMANN, cfg.ACK_MIN_RADIUS_MM)
    return world, clock, cam, lift, arm, chassis
