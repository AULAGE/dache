"""
研究用仿真世界（任务 11）：可复现（--seed），只为验证动作-响应模型，不替代 sim.py。

和 sim.py 的关系：**复用**而不是复制
  - 检测框噪声：直接调用 sim.SimCam._box_of（框边像素噪声 + 每台车固定的框边长偏差）
  - K230 跟踪器：直接用 sim._Tracker（中值 + EMA、2 帧确认、丢帧预测不发帧、跳变要等 2 帧）
  - 舵机死区 + 换向回差：直接调用 sim.SimArm._servo_base（单位不限：底座用度，升降轴用 cm）
  - 帧率 config.K230_FPS、动作中检出概率 config.SIM_BLUR_DETECT_P、K230 常数 sim.K_*
  只有"框 → 位置帧"那几行（碰边缘不发、尺寸法测深）是 sim.SimCam._frame 里内联的，这里照抄了 6 行。

简化（写在报告里）：只看"观察姿态下对准"这一段；画面量用 (x_deg, y) 两个轴：
  x_deg = x0 + J_x · θ（θ = 底座舵机真实输出角，度）
  y     = y0 + J_y · h（h = 升降轴真实输出高度，cm）
  深度不随动作变（无底盘）。J、回差 b、死区 δ 每台车不同，都是**估计值**（见 CAR_RANGES）。

画面里的东西（每个都有 kind，算法看不到，只用于评估）：
  target  真目标：位移 = J · g(u; b, δ) + 噪声
  block   另一个静止方块：位移规律和目标一样，位置不同
  self    爪子脚：画面左侧中间固定位置，与相机固连，偶尔被检测到
  hand    人手：随机运动，和动作无关
"""
import math
import os
import random
import sys
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_GRASP = os.path.dirname(_HERE)
if _GRASP not in sys.path:
    sys.path.insert(0, _GRASP)
os.environ.setdefault("GRASP_IGNORE_CALIB", "1")     # 研究仿真按 config.py 默认值，不读这台车的 calib.json

import config as cfg  # noqa: E402
import sim as _sim  # noqa: E402

AXES = ("x", "y")
UNIT = {"x": "°", "y": "cm"}
OBS_UNIT = {"x": "°", "y": "mm"}

# ---------------------------------------------------------------- 参数（全部是估计值，真机测法见报告）
CAR_RANGES = {
    # 底座：命令 +4.8° 实际只修正约 1.6°（真机现象）→ 回差/死区在 1~3° 量级
    "x": {"J": (-1.1, -0.45),       # 画面方位角 / 底座输出角（真机标定约 −0.6）
          "b": (0.5, 3.0),          # 换向回差（°）
          "delta": (0.5, 2.0)},     # 死区（°）：命令和电机当前位置差不超过它就不动
    # 升降轴：丝杠 + 伺服，回差应该小得多（估计偏大，宁可难一点）
    "y": {"J": (4.0, 8.0),          # 画面 y（mm）/ 升降轴高度（cm），真机标定约 6
          "b": (0.05, 0.3),         # cm
          "delta": (0.02, 0.15)},   # cm
}
EXEC_SIGMA = {"x": 0.15, "y": 0.01}   # 每次动作落点的随机误差（° / cm，估计）
PIX_NOISE = 2.0          # 检测框每条边的像素噪声（px）；sim.py 默认 0.5 偏乐观，这里取 2（估计）
BLOCK_DET_P = 0.97       # 静止时每帧检出方块的概率（sim.py 同样是 3% 丢帧）
SELF_DET_P = 0.9         # 爪子脚"处于可见状态"时每帧被检测到的概率（估计）
HAND_DET_P = 0.8
SCORE = {"target": 0.8, "block": 0.8, "self": 0.7, "hand": 0.6}   # 检测置信度（估计：爪子脚被当成方块）
HAND_STEP = (3.0, 15.0)  # 每次动作期间手移动的标准差（°, mm）
HAND_JITTER = (0.3, 1.5)  # 手每帧的抖动

HALF_W = _sim.K_FOCAL_PIX * math.tan(math.radians(cfg.FOV_X_DEG))
HALF_H = _sim.K_FOCAL_PIX * math.tan(math.radians(cfg.FOV_Y_DEG))


def play(m, o, c, b, delta):
    """
    死区 + 换向回差（和 sim.SimArm._servo_base 完全相同的规则，辨识器里用这个快速版本）：
      |c − m| > δ 时电机走到 c，否则不动；输出轴在 [m − b/2, m + b/2] 的间隙里被推着走
    返回新的 (m, o)
    """
    if abs(c - m) > delta:
        m = c
    lo, hi = m - b / 2.0, m + b / 2.0
    return m, (lo if o < lo else hi if o > hi else o)


class Actuator:
    """真执行器：调用 sim.SimArm._servo_base（复用），加每次动作的随机落点误差"""

    def __init__(self, b, delta, exec_sigma, rng):
        self.cfg = SimpleNamespace(SIM_SERVO_DEADBAND_US=delta, SIM_SERVO_BACKLASH_US=b)
        self.b, self.delta = b, delta
        self.rng, self.exec_sigma = rng, exec_sigma
        self.base_motor = 0.0
        self.base_out = rng.uniform(-b / 2, b / 2)      # 上电时输出轴在间隙里的位置未知
        self.cmd = 0.0
        self.noise = 0.0

    def command(self, c):
        self.cmd = c
        _sim.SimArm._servo_base(self, c)
        self.noise = self.rng.gauss(0, self.exec_sigma)

    @property
    def out(self):
        return self.base_out + self.noise


class Car:
    """每台车固定的未知量"""

    def __init__(self, seed, hyst_scale=1.0):
        """hyst_scale：底座回差 / 死区放大倍数（论文 H3 "人为加大回差" 的仿真版）"""
        r = random.Random(seed * 104729 + 11)
        self.seed = seed
        self.J = {a: r.uniform(*CAR_RANGES[a]["J"]) for a in AXES}
        self.b = {a: r.uniform(*CAR_RANGES[a]["b"]) for a in AXES}
        self.delta = {a: r.uniform(*CAR_RANGES[a]["delta"]) for a in AXES}
        self.b["x"] *= hyst_scale
        self.delta["x"] *= hyst_scale
        self.p_self = r.uniform(0.1, 0.5)            # 每回合爪子脚处于可见状态的概率（估计）
        self.self_view = (r.uniform(-27, -22), r.uniform(-5, 10), r.uniform(85, 100))   # x_deg, y mm, d mm
        self.box_bias_px = r.uniform(-1, 1) * cfg.SIM_BOX_BIAS_PX
        self.depth_bias = r.uniform(0.95, 1.05)

    def describe(self):
        return ("J_x=%.2f b_x=%.2f° δ_x=%.2f°  J_y=%.1f b_y=%.2fcm δ_y=%.2fcm  p_self=%.2f" % (
            self.J["x"], self.b["x"], self.delta["x"], self.J["y"], self.b["y"], self.delta["y"], self.p_self))


class Obj:
    def __init__(self, oid, kind, x_deg, y, d, active=True):
        self.oid, self.kind = oid, kind
        self.x0, self.y0, self.d = x_deg, y, d     # target/block：执行器输出为 0 时的画面位置；其他：当前位置
        self.active = active
        self.move_from = None                      # hand：动作期间从哪移到哪

    @property
    def is_block(self):
        return self.kind in ("target", "block")


class Det:
    """一条检测（K230 的一个位置帧）。kind / oid 是真值，算法不许用"""
    __slots__ = ("t", "x_deg", "y", "d", "oid", "kind")

    def __init__(self, t, x_deg, y, d, oid, kind):
        self.t, self.x_deg, self.y, self.d, self.oid, self.kind = t, x_deg, y, d, oid, kind


class Frame:
    __slots__ = ("t", "dets", "k230", "moving")

    def __init__(self, t, dets, k230, moving):
        self.t, self.dets, self.k230, self.moving = t, dets, k230, moving


def k230_readout(box, depth_bias):
    """检测框 → K230 位置帧 (x, y, d)；碰画面边缘或太小返回 None（同 sim.SimCam._frame 末尾）"""
    hw, hh = HALF_W - _sim.K_EDGE_MARGIN, HALF_H - _sim.K_EDGE_MARGIN
    if box[0] <= -hw or box[2] >= hw or box[1] <= -hh or box[3] >= hh:
        return None
    s = max(box[2] - box[0], box[3] - box[1])
    if s <= 2:
        return None
    dm = depth_bias * _sim.K_FOCAL_PIX * _sim.K_BLOCK_MM / s
    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    return cx * dm / _sim.K_FOCAL_PIX, cy * dm / _sim.K_FOCAL_PIX, dm


class World:
    """
    一台车 + 一个场景。时间是仿真时间（秒）。
    每一帧同时给出两种观测（同一次随机实现）：
      dets  原始检测（K230 把所有框都发出来时能看到的；要改 K230 程序 / 存 SD 卡，现有协议做不到）
      k230  现有 K230 程序的单目标输出（sim._Tracker 选一个、确认后才发）
    """

    def __init__(self, car, seed, fps=None):
        self.car = car
        self.rng = random.Random(seed * 7 + 1)          # 场景
        self.rng_f = random.Random(seed * 7 + 2)        # 帧（检测、框噪声）
        self.rng_a = random.Random(seed * 7 + 3)        # 执行器落点误差
        self.act = {a: Actuator(car.b[a], car.delta[a], EXEC_SIGMA[a], self.rng_a) for a in AXES}
        self.fps = float(fps or cfg.K230_FPS)
        self.t = 0.0
        self.t_next = self.rng_f.uniform(0, 1.0 / self.fps)
        self.objs = []
        self.trk = _sim._Tracker()
        # sim.SimCam._box_of 需要的最小接口（复用它的框噪声模型）
        self._cam = SimpleNamespace(
            w=SimpleNamespace(cfg=SimpleNamespace(SIM_PIX_NOISE=PIX_NOISE), nonlinear=lambda: True,
                              car={"box_bias_px": car.box_bias_px}),
            rf=self.rng_f)
        self.motion = None             # (t0, t1, 旧输出 {axis: o})
        self.n_actions = 0

    # ------------------------------------------------ 执行器
    def out(self, axis, t=None):
        """真实输出（含落点误差）；t 落在动作窗口里时按线性插值"""
        o = self.act[axis].out
        m = self.motion
        if t is not None and m and m[0] <= t < m[1]:
            a = (t - m[0]) / (m[1] - m[0])
            return m[2][axis] + a * (o - m[2][axis])
        return o

    def command(self, cmds):
        """
        cmds: {"x": [绝对命令...], "y": [...]}（一个轴可以连发两条：先退再进）。
        各轴同时动；返回动作期间产生的帧。
        """
        dur = 0.0
        old = {a: self.out(a) for a in AXES}
        for a, seq in cmds.items():
            prev = self.act[a].cmd
            dur_a = 0.0                     # 同一轴的几条命令依次执行，不同轴同时动
            for c in seq:
                if a == "x":
                    dur_a += max(0.15, abs(c - prev) * cfg.BASE_MS_PER_DEG / 1000.0) + 0.15
                else:
                    dur_a += abs(c - prev) / 8.0 + 0.2
                self.act[a].command(c)
                prev = c
            dur = max(dur, dur_a)
        for o in self.objs:
            if o.kind == "hand" and o.active:
                o.move_from = (o.x0, o.y0)
                o.x0 += self.rng.gauss(0, HAND_STEP[0])
                o.y0 += self.rng.gauss(0, HAND_STEP[1])
        if dur <= 0:
            return []
        self.n_actions += 1
        self.motion = (self.t, self.t + dur, old)
        return self.run(dur)

    def preload(self):
        """预紧：各轴先往负方向走一大步再回 0 → 输出轴贴在间隙的负侧（辨识器的已知初始状态）"""
        self.command({"x": [-6.0], "y": [-0.6]})
        self.command({"x": [0.0], "y": [0.0]})
        self.run(0.5)

    # ------------------------------------------------ 场景
    def new_episode(self, n_dist=None, self_active=None, target_view=None, episode_seed=None):
        """episode_seed：给定时场景和帧噪声按它重新播种 → 不同方法可以在同样的场景上配对比较"""
        if episode_seed is not None:
            self.rng.seed(episode_seed * 2 + 1)
            self.rng_f.seed(episode_seed * 2 + 2)
        r = self.rng
        tv = target_view or (r.uniform(-15, 15), r.uniform(-25, 25), r.uniform(140, 230))
        self.objs = [self._block(0, "target", *tv)]
        if self_active is None:
            self_active = r.random() < self.car.p_self
        sv = self.car.self_view
        self.objs.append(Obj(1, "self", sv[0], sv[1], sv[2], active=self_active))
        n = r.randint(0, 2) if n_dist is None else n_dist
        for i in range(n):
            if r.random() < 0.5:
                while True:
                    x = r.uniform(-25, 25)
                    if abs(x - tv[0]) >= 8:
                        break
                self.objs.append(self._block(2 + i, "block", x, r.uniform(-30, 30), r.uniform(130, 260)))
            else:
                self.objs.append(Obj(2 + i, "hand", r.uniform(-25, 25), r.uniform(-40, 40), r.uniform(100, 200)))
        self.trk.reset()                 # 新的一次搜索：K230 跟踪器重新选目标
        self.motion = None
        return self.objs

    def _block(self, oid, kind, x_deg, y, d):
        # 场景给的是"现在"的画面位置，换算成执行器输出为 0 时的位置
        return Obj(oid, kind, x_deg - self.car.J["x"] * self.out("x"), y - self.car.J["y"] * self.out("y"), d)

    def view(self, o, t):
        """物体在 t 时刻的真实画面位置 (x_deg, y, d)"""
        if o.is_block:
            return (o.x0 + self.car.J["x"] * self.out("x", t), o.y0 + self.car.J["y"] * self.out("y", t), o.d)
        if o.kind == "hand":
            x, y = o.x0, o.y0
            m = self.motion
            if o.move_from and m and m[0] <= t < m[1]:
                a = (t - m[0]) / (m[1] - m[0])
                x = o.move_from[0] + a * (x - o.move_from[0])
                y = o.move_from[1] + a * (y - o.move_from[1])
            g = self.rng_f.gauss
            return x + g(0, HAND_JITTER[0]), y + g(0, HAND_JITTER[1]), o.d
        return o.x0, o.y0, o.d             # self：与相机固连

    def target(self):
        return self.objs[0]

    def true_err(self, oid=0):
        """物体当前的真实画面位置（无测量噪声），评估用"""
        o = self.objs[oid]
        v = self.view(o, self.t) if o.kind != "hand" else (o.x0, o.y0, o.d)
        return v

    # ------------------------------------------------ 帧
    def run(self, seconds):
        """让时间走 seconds 秒，返回这段时间里的帧"""
        t_end = self.t + seconds
        out = []
        while self.t_next <= t_end:
            out.append(self._frame(self.t_next))
            self.t_next += 1.0 / self.fps + self.rng_f.uniform(-0.01, 0.01)
        self.t = t_end
        return out

    def window(self, n, settle=None):
        """动作结束后：等 settle 秒，再收 n 帧。返回 (settle 期间的帧, 测量窗口的帧)"""
        s = self.run(cfg.SETTLE_S if settle is None else settle)
        w = []
        while len(w) < n:
            w.extend(self.run(1.0 / self.fps))
        return s, w

    def _frame(self, t):
        rf = self.rng_f
        m = self.motion
        moving = bool(m and m[0] <= t < m[1])
        dets, cands = [], []
        for o in self.objs:
            if not o.active:
                continue
            if o.kind == "self":
                p = SELF_DET_P                      # 和相机固连：运动时也不模糊
            elif o.kind == "hand":
                p = HAND_DET_P
            else:
                p = BLOCK_DET_P * (cfg.SIM_BLUR_DETECT_P if moving else 1.0)
            if rf.random() >= p:
                continue
            x_deg, y, d = self.view(o, t)
            if abs(x_deg) >= 85:
                continue
            v = (d * math.tan(math.radians(x_deg)), y, d)
            box = _sim.SimCam._box_of(self._cam, v)          # 复用 sim.py 的框噪声
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            if abs(cx) >= HALF_W or abs(cy) >= HALF_H:
                continue
            cands.append(((o.oid, o.kind), box, SCORE[o.kind]))
            r = k230_readout(box, self.car.depth_bias)
            if r is not None:
                dets.append(Det(t, math.degrees(math.atan2(r[0], r[2])), r[1], r[2], o.oid, o.kind))
        # 现有 K230 程序：单目标跟踪器（复用 sim._Tracker）
        trk = self.trk
        cand, matched = trk.select(cands)
        if cand is not None:
            trk.update(cand, matched, 1000.0 / self.fps)
        else:
            trk.mark_missed()
        k = None
        if trk.confirmed() and trk.miss == 0:
            r = k230_readout(trk.smooth, self.car.depth_bias)
            if r is not None:
                k = Det(t, math.degrees(math.atan2(r[0], r[2])), r[1], r[2], trk.kind[0], trk.kind[1])
        return Frame(t, dets, k, moving)
