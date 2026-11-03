"""
数据关联：画面里每条检测轨迹是 目标(T) / 自身(S，爪子脚) / 干扰(D，手等) 的后验递推（任务 11，第 2 项）

核心想法：动作是唯一可信的信号。每走一步（命令 u 已知），比较这条轨迹在动作前后两个测量窗口的位移 Δz：
    T：Δz ~ Σ_h w_h · N(Ĵ_h·Δo_h(u), Var(J_h)·Δo_h² + σ²)   —— 执行器假设库的预测（含迟滞！见 hysteresis_id）
    S：Δz ~ N(0, σ²)                                           —— 与相机固连，不随动作动
    D：Δz ~ N(0, σ_D² + σ²)                                     —— 自己乱动，和 u 无关
    log P_k(c) = log P_{k−1}(c) + log p(Δz_k | c)，归一化后再和先验混一点（ε，允许轨迹身份偶尔出错）
当预测的位移 ≈ 0（命令被死区 / 回差吃掉）时，T 和 S 的似然几乎一样 → 这一步不提供证据，
不会像"两步无响应即停"那样把真目标误判成幽灵。

跨动作的轨迹关联本身也用动作：一条轨迹在下一个窗口里要么"没动"（S/D 假设），要么"按预测动了"（T 假设），
取两者中马氏距离小的那个做门限，所以底座转 20° 后也能接上，不需要像素级的连续跟踪。

对比的基线：task.py 现有的 ghost 规则（GhostRule，照 task.py align() 的阈值复现）。
"""
import itertools
import math
import statistics
from collections import Counter

from . import sim_world  # noqa: F401  （先导入：把 grasp/ 加进 sys.path）
from .hysteresis_id import HysteresisBank

import config as cfg  # noqa: E402
from devices import _median_measure  # noqa: E402
from servo import NOISE, build_axes  # noqa: E402

SIGMA = {"x": 0.3, "y": 1.5}          # 一次测量（窗口中值）的噪声量级（°, mm）—— 和 hysteresis_id.GRID 一致
SIGMA_D = {"x": 3.0, "y": 15.0}       # 干扰物（手）每步自己的位移（估计，和 sim_world.HAND_STEP 一致）
PRIOR = {"T": 0.5, "S": 0.25, "D": 0.25}
FORGET = 0.02
OUTLIER = 0.05                        # 真目标某一步"不合模型"的概率（稳健化）
GOAL = {"x": 0.0, "y": 0.0}           # 对准目标（画面中心；config.GOAL 的 x_deg / y）
TOL = {"x": cfg.TOL["x_deg"], "y": cfg.TOL["y"]}


def _npdf(x, mu, var):
    return math.exp(-0.5 * (x - mu) ** 2 / var) / math.sqrt(2 * math.pi * var)


# ================================================================ 观测：窗口聚类
class WinObs:
    """一个测量窗口里同一个东西的若干次检测，取中值"""
    __slots__ = ("x", "y", "d", "n", "kinds")

    def __init__(self, dets):
        self.x = statistics.median(d.x_deg for d in dets)
        self.y = statistics.median(d.y for d in dets)
        self.d = statistics.median(d.d for d in dets)
        self.n = len(dets)
        self.kinds = Counter((d.oid, d.kind) for d in dets)    # 真值，只用于评估

    @property
    def truth(self):
        return self.kinds.most_common(1)[0][0]


def cluster_window(det_lists, min_n=2):
    """det_lists：每帧一个检测列表 → 窗口内的观测（同一物体在窗口内基本不动，按位置就近聚类）"""
    groups = []
    for dets in det_lists:
        for d in dets:
            for g in groups:
                if abs(d.x_deg - g[0].x_deg) < 2.5 and abs(d.y - g[0].y) < 12 and abs(d.d - g[0].d) < 0.3 * g[0].d:
                    g.append(d)
                    break
            else:
                groups.append([d])
    return [WinObs(g) for g in groups if len(g) >= min_n]


# ================================================================ 轨迹 + 分类
class Track:
    _ids = itertools.count(1)

    def __init__(self, obs):
        self.id = next(Track._ids)
        self.obs = obs
        self.p = dict(PRIOR)
        self.miss = 0
        self.n_upd = 0

    @property
    def x(self):
        return self.obs.x

    @property
    def y(self):
        return self.obs.y


# K230 单目标输出有跟踪器的 EMA 滞后：动作后测到的位移常常只有真实位移的 60~100%（仿真里看到的现象）。
# 对 K230 流把"测到的比例" λ 当成未知量做混合；原始检测框没有滤波，λ = 1
LAGS_K230 = (0.55, 0.7, 0.85, 1.0)
LAGS_RAW = (1.0,)


def like_T(dz, pred, lags=LAGS_RAW):
    """pred：{'x': [(w, mu, var)], 'y': [...]}；两个轴共用同一个 λ"""
    tot = 0.0
    for lam in lags:
        L = 1.0
        for a in ("x", "y"):
            s2 = SIGMA[a] ** 2
            L *= sum(w * _npdf(dz[a], lam * mu, lam * lam * var + s2) for w, mu, var in pred[a])
        tot += L
    return tot / len(lags)


def like_S(dz):
    return _npdf(dz["x"], 0.0, SIGMA["x"] ** 2) * _npdf(dz["y"], 0.0, SIGMA["y"] ** 2)


def like_D(dz):
    return (_npdf(dz["x"], 0.0, SIGMA_D["x"] ** 2 + SIGMA["x"] ** 2)
            * _npdf(dz["y"], 0.0, SIGMA_D["y"] ** 2 + SIGMA["y"] ** 2))


def pred_summary(pred):
    """混合预测 → (均值, 标准差)，用于关联门限"""
    out = {}
    for a in ("x", "y"):
        mu = sum(w * m for w, m, v in pred[a])
        var = sum(w * (v + (m - mu) ** 2) for w, m, v in pred[a])
        out[a] = (mu, math.sqrt(var + SIGMA[a] ** 2))
    return out


class TrackManager:
    """跨动作的轨迹关联 + 每条轨迹的 P(T/S/D)"""

    def __init__(self, lags=LAGS_RAW):
        self.tracks = []
        self.lags = lags

    def update(self, obs_list, pred):
        """
        obs_list：这个窗口的观测；pred：上一步动作下真目标的位移预测（None = 没动过）。
        返回 [(track, dz or None)]（这一步看到的轨迹）
        """
        ps = pred_summary(pred) if pred else {"x": (0.0, SIGMA["x"]), "y": (0.0, SIGMA["y"])}
        pairs = []
        for ti, t in enumerate(self.tracks):
            for oi, o in enumerate(obs_list):
                dx, dy = o.x - t.x, o.y - t.y
                if abs(o.d - t.obs.d) > 0.3 * t.obs.d + 15:
                    continue
                cS = (dx / (2 * SIGMA["x"])) ** 2 + (dy / (2 * SIGMA["y"])) ** 2
                cT = min(((dx - lam * ps["x"][0]) / (2 * ps["x"][1])) ** 2
                         + ((dy - lam * ps["y"][0]) / (2 * ps["y"][1])) ** 2 for lam in self.lags)
                c = min(cS, cT)
                if t.p["D"] > 0.2:
                    # 手的门限放宽，但优先级低；已经确认是目标 / 自身的轨迹不走这条（跳变更可能是 K230 换了目标）
                    c = min(c, (dx / SIGMA_D["x"]) ** 2 + (dy / SIGMA_D["y"]) ** 2 + 4.0)
                if c < 9.0:
                    pairs.append((c, ti, oi))
        pairs.sort()
        used_t, used_o, out = set(), set(), []
        for c, ti, oi in pairs:
            if ti in used_t or oi in used_o:
                continue
            used_t.add(ti)
            used_o.add(oi)
            t, o = self.tracks[ti], obs_list[oi]
            dz = {"x": o.x - t.x, "y": o.y - t.y}
            if pred:
                self._classify(t, dz, pred, self.lags)
            t.obs, t.miss = o, 0
            out.append((t, dz))
        keep = []
        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t.miss += 1
            if t.miss <= 2:
                keep.append(t)
        for oi, o in enumerate(obs_list):
            if oi not in used_o:
                t = Track(o)
                keep.append(t)
                out.append((t, None))
        self.tracks = keep
        return out

    @staticmethod
    def _classify(t, dz, pred, lags=LAGS_RAW):
        lD = like_D(dz)
        # 稳健化：真目标也有小概率出现"模型解释不了"的一步（执行器模型还没学好、K230 一帧换了目标……），
        # 混一个宽分布，单独一步再离谱也最多把 T:D 的比值压低 1/OUTLIER 倍，不会一步翻盘
        lT = (1 - OUTLIER) * like_T(dz, pred, lags) + OUTLIER * lD
        L = {"T": lT, "S": like_S(dz), "D": lD}
        post = {c: t.p[c] * max(L[c], 1e-300) for c in L}
        z = sum(post.values())
        if z <= 0:
            return
        for c in post:
            t.p[c] = (1 - FORGET) * post[c] / z + FORGET * PRIOR[c]
        t.n_upd += 1


# ================================================================ 基线：task.py 的 ghost 规则
class GhostRule:
    """
    照 task.py GraspTask.align() 复现：动了某个轴、预测位移 |J·u| 够大（升降 ≥ max(6, 3·NOISE_y) mm，
    底座 ≥ 1.5°），实测变化 < 30% 预测 → 记一次；有响应就清零；任一轴连续 2 次 → 判定是机械臂自己
    """

    def __init__(self):
        self.strikes = {"y": 0, "x_deg": 0}

    def update(self, checks):
        """checks：[(轴, 预测位移, 实测位移, 阈值)]；返回是否判定 ghost"""
        for ax, pred, got, thresh in checks:
            if abs(pred) >= thresh:
                self.strikes[ax] = self.strikes[ax] + 1 if abs(got) < 0.3 * abs(pred) else 0
        return max(self.strikes.values()) >= 2


# ================================================================ 对准代理（三种方法共用同一个控制律）
class CarState:
    """每台车跨回合保存的东西：自适应灵敏度（task.py 同款）、执行器假设库"""

    def __init__(self, no_hyst=False):
        """no_hyst=True：消融——执行器假设库只有 b = δ = 0 一个假设（认为命令多少就动多少，只学 J）"""
        self.axes = build_axes(cfg, ["y", "x_deg"])          # 复用 servo.AdaptiveAxis，初值来自 config.AXES
        if no_hyst:
            from .hysteresis_id import GRID
            self.bank = {a: HysteresisBank([0.0], [0.0], GRID[a]["J0"], GRID[a]["sJ"], GRID[a]["sigma"])
                         for a in ("x", "y")}
        else:
            self.bank = {"x": HysteresisBank.for_axis("x"), "y": HysteresisBank.for_axis("y")}


class Agent:
    """
    method:
      "ghost"   现有系统：K230 单目标输出 + task.py ghost 规则
      "p_k230"  同样的 K230 单目标输出，用 P(T/S/D) 代替 ghost 规则（只能停，不能换目标）
      "p_raw"   K230 发出所有检测框（要改 K230 程序），多轨迹 P(T/S/D)，可以换目标、可以主动探测
    """
    PRED_K = 40

    def __init__(self, method, car_state, world):
        self.method, self.cs, self.w = method, car_state, world
        self.tm = TrackManager(LAGS_RAW if method == "p_raw" else LAGS_K230)
        self.ghost = GhostRule()
        self.prev = None              # 上一步：(e, u_x, u_y)
        self.pred = None
        self.lock = None
        self.probes = 0
        self.lock_hist = []
        self.pending = None
        self._dz = None

    # -------------------------------------------------- 测量
    def measure(self, n):
        """收 n 帧（不够就再等，最多 MEASURE_TIMEOUT_S）；返回窗口里的帧"""
        w = self.w
        _, frames = w.window(n)
        t_end = w.t + cfg.MEASURE_TIMEOUT_S - n / w.fps
        pick = (lambda f: [f.k230] if f.k230 else []) if self.method != "p_raw" else (lambda f: f.dets)
        while sum(1 for f in frames[-n:] if pick(f)) < n and w.t < t_end:
            frames.extend(w.run(1.0 / w.fps))
        return frames, pick

    # -------------------------------------------------- 一步
    def step(self, n):
        """返回 ("commit"/"abort"/"lost", 锁定的真值 (oid, kind) or None) 或 ("move", cmds)"""
        self._dz = None
        act = self._step(n)
        self._apply_pending()                 # 提前结束的分支也要把上一条命令推进到假设库
        if act[0] == "move":
            self.pending = act[1]
        return act

    def _apply_pending(self):
        """上一步发出的命令推进假设库状态；有已确认目标的位移时顺便学习（自监督）"""
        if self.pending is None or self.method == "ghost":
            self.pending = None
            return
        for a in ("x", "y"):
            dz = self._dz[a] if self._dz is not None else None
            self.cs.bank[a].apply(self.pending.get(a, []), dz)
        self.pending = None

    def finish(self):
        self._dz = None
        self._apply_pending()

    def _step(self, n):
        frames, pick = self.measure(n)
        cs = self.cs
        if self.method == "p_raw":
            obs_list = cluster_window([pick(f) for f in frames[-max(n, 3):]])
            seen = self.tm.update(obs_list, self.pred)
            self._learn_bank(seen)
            vis = [t for t, _ in seen]
            if not vis:
                return ("lost", None)
            if self.lock not in vis or self.lock.p["T"] < 0.5:
                # 选 P(T) 最大的；差不多时选看起来最大（最近）的 —— 和 K230 选框的习惯一样
                best = max(vis, key=lambda t: (round(t.p["T"], 1), -t.obs.d))
                if self.lock not in vis or best.p["T"] > self.lock.p["T"] + 0.1:
                    self.lock = best
            lock = self.lock
            if lock.p["T"] < 0.05 and lock.n_upd >= 2 and all(t.p["T"] < 0.05 for t in vis):
                return ("abort", lock.obs.truth)
            m = lock.obs
            e = {"x": m.x - GOAL["x"], "y": m.y - GOAL["y"]}
        else:
            dets = [d for f in frames for d in pick(f)]
            fr = [(d.t, d.d * math.tan(math.radians(d.x_deg)), d.y, d.d) for d in dets]
            meas = _median_measure(fr, n)               # 复用 devices._median_measure（task.py 同款）
            if meas is None:
                return ("lost", None)
            ks = Counter((d.oid, d.kind) for d in dets[-n:])
            truth = ks.most_common(1)[0][0]
            e = {"x": meas.x_deg - GOAL["x"], "y": meas.y - GOAL["y"]}
            if self.method == "p_k230":
                obs_list = cluster_window([[d] for d in dets[-n:]])
                obs_list = sorted(obs_list, key=lambda o: -o.n)[:1]       # K230 窗口内换过目标时取多数
                seen = self.tm.update(obs_list, self.pred)
                self._learn_bank(seen)
                if seen:
                    self.lock = seen[0][0]
                    if self.lock.p["T"] < 0.05 and self.lock.n_upd >= 2:
                        return ("abort", truth)
            elif self.prev is not None:
                pe, u_x, u_y = self.prev
                checks = []
                if u_y:
                    checks.append(("y", cs.axes["y"].j * u_y, e["y"] - pe["y"], max(6.0, 3 * NOISE["y"])))
                if u_x:
                    checks.append(("x_deg", cs.axes["x_deg"].j * u_x, e["x"] - pe["x"], 1.5))
                if self.ghost.update(checks):
                    return ("abort", truth)
            m = None
        truth = self.lock.obs.truth if self.method == "p_raw" else truth
        self.lock_hist.append(truth)

        # 自适应灵敏度学习（task.py 同款）
        if self.prev is not None:
            pe, u_x, u_y = self.prev
            if u_y:
                cs.axes["y"].learn(u_y, pe["y"], e["y"])
            if u_x:
                cs.axes["x_deg"].learn(u_x, pe["x"], e["x"])

        in_tol = abs(e["x"]) <= TOL["x"] and abs(e["y"]) <= TOL["y"]
        u_y = cs.axes["y"].command(e["y"]) if abs(e["y"]) > TOL["y"] else 0.0
        u_x = cs.axes["x_deg"].command(e["x"]) if abs(e["x"]) > TOL["x"] else 0.0
        if in_tol or not (u_x or u_y):
            pT = self.lock.p["T"] if self.lock is not None else 1.0
            if self.method == "ghost" or pT >= 0.9:
                return ("commit", truth)
            if self.method == "p_k230":
                return ("commit", truth) if pT >= 0.3 else ("abort", truth)
            # p_raw：还不确定 → 用升降轴（迟滞小）主动探测一下
            if self.probes >= 3:
                return ("commit", truth) if pT >= 0.5 else ("abort", truth)
            self.probes += 1
            u_x, u_y = 0.0, (0.8 if self.probes % 2 else -0.8)
        cmds = {}
        if u_x:
            cmds["x"] = [self.w.act["x"].cmd + u_x]
        if u_y:
            cmds["y"] = [self.w.act["y"].cmd + u_y]
        self.prev = (e, u_x, u_y)
        self._apply_pending()
        if self.method != "ghost":
            self.pred = {a: (cs.bank[a].predictive(cmds.get(a, []), self.PRED_K) if a in cmds
                             else [(1.0, 0.0, 0.0)]) for a in ("x", "y")}
        return ("move", cmds)

    def _learn_bank(self, seen):
        """只用"已确认是真目标"的轨迹的位移更新假设库（自监督：P(T) ≥ 0.95）"""
        self._dz = None
        for t, dz in seen:
            if dz is not None and t.p["T"] >= 0.95 and t.n_upd >= 2:
                self._dz = dz
                break


def run_episode(world, method, car_state, max_steps=None, n_dist=None, self_active=None, episode_seed=None):
    """
    跑一回合"原地看一眼 → 对准"。返回 dict：
      result  commit / abort / lost / timeout
      kind    结束时锁定的东西的真实类别（target/block/self/hand）
      steps   动作步数
      first   从第几步起一直锁在方块上（只对 commit 到方块的回合有意义）
    """
    max_steps = max_steps or cfg.MAX_ALIGN_STEPS
    if world.act["x"].cmd != 0.0:                 # task.py run_once 开头 go_observe(0)：底座先回正
        world.command({"x": [0.0]})
        if method != "ghost":
            car_state.bank["x"].apply([0.0], None)
    world.new_episode(n_dist=n_dist, self_active=self_active, episode_seed=episode_seed)
    world.run(1.0)                                 # 原地看一眼（K230 选定目标）
    ag = Agent(method, car_state, world)
    n = cfg.SAMPLE_N
    res = {"result": "timeout", "kind": None, "steps": max_steps}
    for step in range(max_steps):
        act = ag.step(n)
        if act[0] != "move":
            res = {"result": act[0], "kind": act[1][1] if act[1] else None, "steps": step}
            break
        world.command(act[1])
        e = ag.prev[0]
        n = cfg.SAMPLE_N_FAR if (abs(e["x"]) > 4 * TOL["x"] or abs(e["y"]) > 4 * TOL["y"]) else cfg.SAMPLE_N
    else:
        ag.finish()
        res["kind"] = ag.lock_hist[-1][1] if ag.lock_hist else None
    hist = ag.lock_hist
    first = None
    if res["result"] == "commit" and res["kind"] in ("target", "block"):
        first = len(hist) - 1
        while first > 0 and hist[first - 1][1] in ("target", "block"):
            first -= 1
    res["first"] = first
    res["time"] = world.t
    return res
