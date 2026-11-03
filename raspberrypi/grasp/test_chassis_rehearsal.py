#!/usr/bin/env python3
"""
任务 18：底盘联调预演（离线，仿真，**不是真机**）

  python3 test_chassis_rehearsal.py          全部：危险动作检查 + 有底盘场景统计（约 1~2 分钟）
  python3 test_chassis_rehearsal.py quick    只跑危险动作检查和 chassis_test 拟合（selftest.py 会调用这部分）
  python3 test_chassis_rehearsal.py stats 60 只跑场景统计，每种设置 60 次

场景：物块在车前 300~700mm（从车的转动中心量）、左右 ±30°，高度 5~40cm（升降轴量程内）。
底盘用 sim.SimWheeltec（真机同一套 WheeltecChassis.drive + 假控制器，任务 03）。
sim.py 只读：需要的仿真设置（物块位置、真实最小转弯半径、真实滑行 τ、K230 认错成爪子）都在这个文件里做。

统计每次抓取：真实抓到没有、对准步数、用时、底盘总里程、倒车总量、离起点最远后退了多少、
转向打满（请求的转角被最小转弯半径截断）连续几次、车有没有开到物块跟前（撞平台）。
"""
import importlib
import math
import os
import random
import statistics
import sys
import tempfile

os.environ.setdefault("GRASP_IGNORE_CALIB", "1")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config as cfg                          # noqa: E402
import sim                                    # noqa: E402
from devices import DeviceError, Measurement  # noqa: E402
from task import GraspTask                    # noqa: E402

PASS, FAIL = [], []

# 危险动作的判定线（估计值，联调手册里用同样的数）
DANGER_REVERSE_MM = 600      # 一次抓取里离起点往后退超过这么多 = 倒车过远（车后面一般没人看着）
DANGER_SAT_DEG = 90          # 连续几次 drive 转向都被截到最小半径、车头累计转过这么多度 = 转向打满持续（在兜圈）
DANGER_NEAR_MM = 60          # 底座转轴离物块的水平距离比"对准时应有的距离"（相机半径 + GOAL 深度）和起始距离都还近
                             # 这么多 = 车主动开到了物块跟前（会撞平台）
DANGER_ODO_MM = 3000         # 一次抓取底盘总里程超过 3m = 来回乱跑


def check(name, ok, info=""):
    (PASS if ok else FAIL).append(name)
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   {info}" if info else ""))


# ------------------------------------------------------------ 仿真设置（不改 sim.py）
class _Probe:
    """包住 SimWheeltec.drive：记录每次底盘动作和车的真实位姿"""

    def __init__(self, chassis, world, true_radius=None):
        self.c, self.w = chassis, world
        self.calls = []
        self.start = list(world.robot)
        self.min_back = 0.0          # 沿起始车头方向，离起点最远后退了多少（负数）
        self.max_fwd = 0.0           # 沿起始车头方向，最远前进了多少
        self.min_gap = float("inf")  # 底座转轴到物块的最小水平距离
        self.init_gap = self._gap()
        self.true_radius = true_radius
        orig_drive, orig_plant = chassis.drive, chassis._plant
        probe = self

        def plant(h):
            if probe.true_radius:                      # 真实最小转弯半径和 config 不一样
                r = chassis.min_radius
                chassis.min_radius = probe.true_radius
                try:
                    orig_plant(h)
                finally:
                    chassis.min_radius = r
            else:
                orig_plant(h)
            probe._track()

        def drive(mm, dpsi_deg=0.0, timeout=None):
            lim = math.degrees(abs(mm) / chassis.min_radius) if chassis.ackermann else 180.0
            sat = abs(mm) >= 1 and abs(dpsi_deg) > lim + 1e-6
            out = orig_drive(mm, dpsi_deg, timeout)
            probe.calls.append({"mm": mm, "dpsi_req": dpsi_deg, "sat": sat, "moved": out[0], "dpsi": out[1]})
            return out

        chassis._plant = plant
        chassis.drive = drive

    def _track(self):
        r, s = self.w.robot, self.start
        along = (r[0] - s[0]) * math.cos(s[2]) + (r[1] - s[1]) * math.sin(s[2])
        self.min_back = min(self.min_back, along)
        self.max_fwd = max(self.max_fwd, along)
        if not self.w.held and not self.w.removed:
            self.min_gap = min(self.min_gap, self._gap())

    def _gap(self):
        r = self.w.robot
        px = r[0] + self.w.PIVOT_FWD * math.cos(r[2])
        py = r[1] + self.w.PIVOT_FWD * math.sin(r[2])
        return math.hypot(self.w.block[0] - px, self.w.block[1] - py)

    def summary(self):
        odo = sum(abs(c["moved"]) for c in self.calls)
        rev = sum(-c["moved"] for c in self.calls if c["moved"] < 0)
        run = best = 0
        turn = best_turn = 0.0
        for c in self.calls:
            run = run + 1 if c["sat"] else 0
            turn = turn + abs(c["dpsi"]) if c["sat"] else 0.0
            best, best_turn = max(best, run), max(best_turn, turn)
        return {"init_gap": self.init_gap, "odo": odo, "reverse": rev, "back": -self.min_back, "sat_run": best, "sat_turn": best_turn,
                "n_drive": len(self.calls), "min_gap": self.min_gap,
                "max_cmd": max([abs(c["mm"]) for c in self.calls] or [0.0])}


def place_block(world, rng, dist=(300, 700), ang=30.0):
    d = rng.uniform(*dist)
    a = math.radians(rng.uniform(-ang, ang))
    world.block = [d * math.cos(a), d * math.sin(a), world.CAM_Z0 + 10 * rng.uniform(5, 40)]
    return d, math.degrees(a)


def real_like(c):
    """
    真机标定的近似（CLAUDE.md "真机现状"）：相机几乎在底座转轴上、底座方向 −1、
    近 156mm→Y+70、远 216mm→Y+43。有底盘时对准目标就是 GOAL（近姿态那一组），底盘把深度对到 156mm
    """
    c.R_CAM_MM = 0.0
    c.BASE_DIR = -1
    c.REACH_NEAR_D, c.REACH_FAR_D = 156.0, 216.0
    c.GOAL = {"x_deg": 0.0, "y": 70.0, "d": 156.0}
    c.GOAL_FAR = {"x_deg": 0.0, "y": 43.0, "d": 216.0}
    # K230 画面转了 90°（k230_sdcard/main.py ROTATE_DEG=90，1280×720，FOCAL_PIX=944）：
    # 半视场 水平 atan(360/944)≈21°、竖直 atan(640/944)≈34°。config.py 的 35°/25° 是没转之前的，
    # 只有仿真用它；不改的话 GOAL 的 Y+70（向下 24°）在仿真里出了画面，有底盘时一次都抓不到
    c.FOV_X_DEG, c.FOV_Y_DEG = 21.0, 34.0
    c.VERIFY = "observe"         # 真机现在用的确认方式（抬升确认会被爪子骗）


PROFILES = {
    # 名字: (说明, 改 config 的函数, 相机真实半径, 真实最小转弯半径 / None = 和 config 一样, 真实滑行 τ)
    "default": ("config 默认值", None, None, None, None),
    "real": ("真机标定近似（R=0、底座方向 −1、GOAL d=156 y=+70）", real_like, 0.0, None, None),
    "real_r900": ("真机近似 + 真实最小半径 900mm（config 仍是 600）", real_like, 0.0, 900.0, None),
    "real_tau06": ("真机近似 + 真实滑行 τ=0.6s（100mm/s 滑 60mm）", real_like, 0.0, None, 0.6),
    "real_search": ("真机近似 + SEARCH_BASE_DEG=[0,±35,±70]（按转 90° 后的水平视场 ±21° 加密）",
                    lambda c: (real_like(c), setattr(c, "SEARCH_BASE_DEG", [0, -35, 35, -70, 70])), 0.0, None, None),
    "real_backup": ("真机近似 + BASE_LIMIT_BACKUP_MM=150", lambda c: (real_like(c),
                                                                      setattr(c, "BASE_LIMIT_BACKUP_MM", 150)),
                    0.0, None, None),
}


def rehearse(n, seed, profile="default", verbose=False):
    desc, tweak, r_cam, true_radius, tau = PROFILES[profile]
    importlib.reload(cfg)
    cfg.POSES = sim.SIM_POSES
    cfg.USE_CHASSIS = True
    if tweak:
        tweak(cfg)
    rng = random.Random(seed)
    bias = rng.uniform(0.92, 1.08)
    r_true = cfg.R_CAM_MM * rng.uniform(0.8, 1.25) if r_cam is None else r_cam
    car = sim.make_car(cfg, seed)
    if tau:
        car["coast_tau"] = tau
    place_rng = random.Random(seed * 13 + 5)
    log_dir = tempfile.mkdtemp(prefix="grasp_reh_")
    task, rows = None, []
    for i in range(n):
        world, clock, cam, lift, arm, ch = sim.make_sim(cfg, seed * 1000 + i, bias, r_true, car)
        dist, ang = place_block(world, place_rng)
        if not isinstance(ch, sim.SimWheeltec):
            raise RuntimeError("需要 SIM_CHASSIS_DYNAMICS = True 和阿克曼底盘")
        if task is not None:
            ch.coast_tau, ch.pi_tau = task.chassis.coast_tau, task.chassis.pi_tau   # 真机会存文件接着用
        probe = _Probe(ch, world, true_radius)
        if task is None:
            task = GraspTask(cfg, clock, cam, lift, arm, ch, log_dir=log_dir, verbose=verbose)
        else:
            task.clock, task.cam, task.lift, task.arm, task.chassis = clock, cam, lift, arm, ch
            task.base_deg = 0.0
        err = None
        try:
            r = task.run_once()
        except Exception as e:       # 和 main.run_sim 一样：设备错误 / 崩溃都算这次失败（机械臂 USB 掉线等）
            err = repr(e)
            r = {"result": "device_error" if isinstance(e, DeviceError) else "crash", "steps": 0,
                 "time_s": round(clock.now(), 1)}
            task._lifted = False
        s = probe.summary()
        s.update(result=r["result"], ok=(world.picked if err is None else world.removed), steps=r["steps"], time=r["time_s"],
                 dist=dist, ang=ang, err=err, profile=profile, goal_gap=r_true + cfg.GOAL["d"])
        rows.append(s)
    importlib.reload(cfg)
    return rows


def dangers(r):
    out = []
    if r["back"] > DANGER_REVERSE_MM:
        out.append(f"倒车 {r['back']:.0f}mm")
    if r["sat_turn"] >= DANGER_SAT_DEG:
        out.append(f"转向打满连续 {r['sat_run']} 次、转过 {r['sat_turn']:.0f}°")
    if r["min_gap"] < min(r["goal_gap"], r["init_gap"]) - DANGER_NEAR_MM:
        out.append(f"离物块只剩 {r['min_gap']:.0f}mm")
    if r["odo"] > DANGER_ODO_MM:
        out.append(f"总里程 {r['odo']:.0f}mm")
    return out


def summarize(rows, title):
    n = len(rows)
    ok = sum(r["ok"] for r in rows)
    bad = [(i, dangers(r)) for i, r in enumerate(rows) if dangers(r)]
    fails = {}
    for r in rows:
        if not r["ok"]:
            fails[r["result"]] = fails.get(r["result"], 0) + 1
    p90 = sorted(r["back"] for r in rows)[int(0.9 * (n - 1))]
    print(f"  {title}：抓到 {ok}/{n} = {100 * ok / n:.0f}%  步数 {statistics.mean(r['steps'] for r in rows):.1f}  "
          f"用时 {statistics.mean(r['time'] for r in rows):.0f}s（最长 {max(r['time'] for r in rows):.0f}s）  "
          f"里程 平均 {statistics.mean(r['odo'] for r in rows):.0f} / 最大 {max(r['odo'] for r in rows):.0f}mm  "
          f"后退 p90 {p90:.0f} / 最大 {max(r['back'] for r in rows):.0f}mm  "
          f"转向打满最长连续 {max(r['sat_run'] for r in rows)} 次 / {max(r['sat_turn'] for r in rows):.0f}°  "
          f"最多比对准距离多靠近 {max(0.0, -min(r['min_gap'] - min(r['goal_gap'], r['init_gap']) for r in rows)):.0f}mm")
    if fails:
        print(f"      没抓到：{fails}")
    for i, d in bad[:5]:
        r = rows[i]
        print(f"      ⚠ 第 {i + 1} 次（物块 {r['dist']:.0f}mm {r['ang']:+.0f}°，{r['result']}）：{'，'.join(d)}")
    return ok / n, len(bad)


# ------------------------------------------------------------ 危险动作：K230 把爪子当成物块
class _GhostCam:
    """K230 锁住了爪子的一只脚：读数恒定，不随底盘 / 底座 / 升降轴变化（真机已知问题 1）"""

    def __init__(self, x, y, d):
        self.m = Measurement(x, y, d, 5)

    def measure(self, since, n, timeout):
        return Measurement(self.m.x, self.m.y, self.m.d, n)


def ghost_drive(x, y, d, seed=5):
    """对准时 K230 一直报同一个读数：底盘一共会走多远"""
    importlib.reload(cfg)
    cfg.POSES = sim.SIM_POSES
    cfg.USE_CHASSIS = True
    world, clock, cam, lift, arm, ch = sim.make_sim(cfg, seed, 1.0, cfg.R_CAM_MM, sim.make_car(cfg, 0))
    probe = _Probe(ch, world)
    task = GraspTask(cfg, clock, _GhostCam(x, y, d), lift, arm, ch, log_dir=None, verbose=False)
    task.go_observe(0)
    lift.move_to(20)
    ok, _, steps = task.align(task.measure())
    s = probe.summary()
    importlib.reload(cfg)
    return ok, steps, s["odo"], getattr(task, "_abort", None)


def test_ghost_chassis():
    print("危险动作：K230 把爪子当成物块时底盘会不会一直开（读数不随底盘动作变化）")
    # 爪子脚在画面左侧中间：x 偏左、y≈0。尺寸法测深：脚比物块大、离得近 → d 偏小；也测两种偏大的情况
    for x, y, d in ((-60.0, 0.0, 90.0), (-60.0, 2.0, 400.0), (0.0, 0.0, 300.0)):
        ok, steps, odo, why = ghost_drive(x, y, d)
        check(f"假目标 x={x:+.0f} y={y:+.0f} d={d:.0f}：底盘总共走 ≤ 2 步的量就停（{why or '没停'}）",
              (not ok) and why == "ghost" and odo <= 2 * cfg.AXES["d"]["max"] and steps <= 3,
              f"{steps} 步，里程 {odo:.0f}mm")


class _JumpCam:
    """物块在正前方，深度按车的真实位置算；底盘第一次走完后插一次"深度跳大"的误读（K230 跟踪器换了目标）"""

    def __init__(self, world, probe, d0, jump_to):
        self.w, self.p, self.d0, self.jump_to = world, probe, d0, jump_to
        self.x0 = world.robot[0]
        self.jumped = False

    def measure(self, since, n, timeout):
        d = self.d0 - (self.w.robot[0] - self.x0)
        if len(self.p.calls) == 1 and not self.jumped:
            self.jumped = True
            d = self.jump_to
        return Measurement(0.0, 0.0, d, n)


def jump_drive(check_on):
    importlib.reload(cfg)
    cfg.POSES = sim.SIM_POSES
    cfg.USE_CHASSIS = True
    cfg.CHASSIS_JUMP_CHECK = check_on
    world, clock, cam, lift, arm, ch = sim.make_sim(cfg, 5, 1.0, cfg.R_CAM_MM, sim.make_car(cfg, 0))
    probe = _Probe(ch, world)
    d0 = cfg.GOAL["d"] + 300
    task = GraspTask(cfg, clock, _JumpCam(world, probe, d0, d0 + 150), lift, arm, ch, log_dir=None, verbose=False)
    task.go_observe(0)
    ok, m, steps = task.align(task.measure())
    over = probe.max_fwd - (d0 - cfg.GOAL["d"])      # 中途最多比该走的多往前开了多少
    importlib.reload(cfg)
    return ok, over, [round(c["mm"]) for c in probe.calls]


def test_jump_check():
    print("危险动作：底盘走完一步后 K230 误读（深度跳大 150mm），会不会照着误读再开一大步")
    ok0, over0, calls0 = jump_drive(False)
    check("原行为（CHASSIS_JUMP_CHECK=False）：照着误读多开了一大步（复现原问题）", over0 > 150,
          f"中途开过头 {over0:.0f}mm，每步 {calls0}")
    ok1, over1, calls1 = jump_drive(True)
    check("深度变化和里程对不上 → 再确认一次，不照着误读开车", ok1 and over1 < 30,
          f"中途开过头 {over1:+.0f}mm，每步 {calls1}")


class _LostFeedback(sim.SimWheeltec):
    """走到一半回传断了（串口只剩下发）：里程不再增加，has_feedback() 变 False"""
    t_lost = float("inf")

    def has_feedback(self):
        return self.clock.now() - self.t_lost < 0.5

    def _plant(self, h):
        frozen = self.clock.now() >= self.t_lost
        d, y, v = self.dist, self.yaw, self.fb_vx
        super()._plant(h)
        if frozen:
            self.dist, self.yaw, self.fb_vx = d, y, v


def test_feedback_loss():
    print("危险动作：底盘走到一半回传断了")
    importlib.reload(cfg)
    world, clock, cam, lift, arm, ch = sim.make_sim(cfg, 3, 1.0, cfg.R_CAM_MM, sim.make_car(cfg, 0))
    ch.__class__ = _LostFeedback
    ch.t_lost = clock.now() + 1.0
    x0 = world.robot[0]
    err = None
    try:
        ch.drive(400)
    except DeviceError as e:
        err = str(e)
    ground = world.robot[0] - x0
    check("回传断了 0.5 秒内停车并报设备错误（原来会按时间一直开到超时）",
          err is not None and "回传" in err and ground < 250 and ch.cmd == (0, 0), f"实际走了 {ground:.0f}mm")
    importlib.reload(cfg)


# ------------------------------------------------------------ chassis_test.py 的拟合和写 calib
def test_chassis_test_fit():
    print("chassis_test.py：尺子数据拟合滑行 τ / 最小转弯半径（纯计算，不碰串口）")
    import chassis_test as ct
    # 构造：真实滑行 τ=0.45s，驱动以为 0.30s → 每次多走 (0.45-0.30)·v
    rows = [{"cmd": s * 100, "v": 80.0, "done": 100 - 0.30 * 80.0, "ruler": 100 - 0.30 * 80 + 0.45 * 80 + e}
            for s, e in ((1, 1.0), (1, -1.0), (1, 0.5), (-1, -0.5), (-1, 1.5), (-1, -1.5))]
    tau, spread = ct.fit_coast_tau(rows)
    check("6 次尺子读数拟合出滑行 τ≈0.45s", tau is not None and abs(tau - 0.45) < 0.02, f"{tau:.3f}s ±{spread:.3f}")
    check("读数不够（< 3 次）时不拟合", ct.fit_coast_tau(rows[:2])[0] is None)
    # 打满方向走一段圆弧：终点 (R sinθ, R(1−cosθ))
    R, th = 800.0, 0.75
    h = R * (1 - math.cos(th))
    chord = math.hypot(R * math.sin(th), h)
    check("弦长 + 侧移 → 转弯半径", abs(ct.radius_from_chord(chord, h) - R) < 1.0,
          f"{ct.radius_from_chord(chord, h):.1f}mm")
    rows2 = [{"ruler": 1.03 * o, "odo": o} for o in (100, -100, 200, -200)]
    check("尺子 / 里程 比例（里程计偏 3%）", abs(ct.odom_scale(rows2) - 1.03) < 1e-9)
    check("滑行 τ → τ_PI（扣延迟、折算电压）",
          abs(ct.pi_tau_from_coast(0.47, 0.02, 25.8, 25.8) - 0.45) < 1e-9
          and ct.pi_tau_from_coast(0.47, 0.02, 22.5, 25.8) < 0.45)
    check("航向变化 + 里程 → 转弯半径", abs(ct.radius_from_heading(R * th, math.degrees(th)) - R) < 1e-6)
    d = tempfile.mkdtemp(prefix="grasp_ct_")
    path = os.path.join(d, "calib.json")
    with open(path, "w", encoding="utf-8") as f:
        f.write('{"GOAL": {"x_deg": 0, "y": 0, "d": 156}, "R_CAM_MM": 0}')
    ct.write_calib({"ACK_MIN_RADIUS_MM": 812.0}, path)
    import json
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    check("写 calib.json 只改这一项、保留原有标定", data.get("ACK_MIN_RADIUS_MM") == 812.0
          and data.get("R_CAM_MM") == 0 and data["GOAL"]["d"] == 156)
    check("写入的键都在 CALIB_KEYS 白名单里", all(k in cfg.CALIB_KEYS for k in ct.CALIB_WRITES))


_SIGTERM_CHILD = r"""
import os, sys, time, types
sys.path.insert(0, sys.argv[1])
os.environ["GRASP_IGNORE_CALIB"] = "1"
import devices, main
out = open(sys.argv[2], "ab", buffering=0)


class FakeSerial:
    def __init__(self, *a, **k):
        pass

    def write(self, b):
        out.write(b)

    def read(self, n):
        time.sleep(0.01)
        return b""

    def flush(self):
        pass

    def close(self):
        pass


devices.serial = types.SimpleNamespace(Serial=FakeSerial, SerialException=OSError)
if sys.argv[3] == "1":
    main.install_stop_signals()
c = devices.WheeltecChassis("fake", 115200, cmd_ttl_s=0.0)    # TTL 关掉：只看信号处理
try:
    while True:
        c._set(80, 0)
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    c.close()
"""


def _sigterm_last_frame(install):
    import signal
    import subprocess
    d = tempfile.mkdtemp(prefix="grasp_sig_")
    out, script = os.path.join(d, "frames.bin"), os.path.join(d, "child.py")
    with open(script, "w", encoding="utf-8") as f:
        f.write(_SIGTERM_CHILD)
    p = subprocess.Popen([sys.executable, script, HERE, out, "1" if install else "0"],
                         stderr=subprocess.DEVNULL)
    import time
    time.sleep(1.5)
    p.send_signal(signal.SIGTERM)
    p.wait(10)
    with open(out, "rb") as f:
        data = f.read()
    return data[-11:] if len(data) >= 11 else b""


def test_sigterm():
    print("危险动作：systemctl stop / kill 时底盘停不停（固件没有串口超时）")
    if sys.platform.startswith("win"):
        # Windows 上 send_signal(SIGTERM) 等于 TerminateProcess，进程收不到信号，测不了；树莓派（Linux）上照常测
        print("  （Windows 上跳过：SIGTERM 在 Windows 上无法被程序捕获）")
        return
    from devices import WheeltecChassis
    stop = WheeltecChassis.frame(0, 0)
    check("没有信号处理时：SIGTERM 后最后一帧是行驶帧（复现原问题：车会按最后速度一直走）",
          _sigterm_last_frame(False) != stop)
    check("main.install_stop_signals()：SIGTERM 后最后发出的是停车帧", _sigterm_last_frame(True) == stop)


def _empty_backoff(cap, n=6):
    """周围没有物块（都抓完了），main.py 连续运行 n 次：车一共往后退了多少"""
    importlib.reload(cfg)
    cfg.POSES = sim.SIM_POSES
    cfg.USE_CHASSIS = True
    cfg.SEARCH_BACKOFF_MAX_MM = cap
    world, clock, cam, lift, arm, ch = sim.make_sim(cfg, 1, 1.0, cfg.R_CAM_MM, sim.make_car(cfg, 0))
    world.removed = True
    x0 = world.robot[0]
    task = GraspTask(cfg, clock, cam, lift, arm, ch, log_dir=None, verbose=False)
    res = [task.run_once()["result"] for _ in range(n)]
    importlib.reload(cfg)
    return x0 - world.robot[0], res


def test_backoff_cap():
    print("危险动作：周围没有物块时 main.py 连续运行，搜索后退会不会一直倒下去")
    back0, _ = _empty_backoff(0)
    check("原行为（SEARCH_BACKOFF_MAX_MM=0）：6 次一共退了 1.5m（复现原问题）", back0 > 1400, f"{back0:.0f}mm")
    back, res = _empty_backoff(500)
    check("累计后退上限 500mm：6 次以后最多退 500mm 左右，结果仍是 not_found",
          back < 550 and all(r == "not_found" for r in res), f"{back:.0f}mm")


def quick():
    test_ghost_chassis()
    test_jump_check()
    test_feedback_loss()
    test_sigterm()
    test_backoff_cap()
    test_chassis_test_fit()


def stats(n=40):
    print(f"有底盘场景统计（仿真 SimWheeltec，每种设置 {n} 次，物块在车前 300~700mm、±30°）")
    out = {}
    for name in PROFILES:
        rows = rehearse(n, 18, name)
        rate, nbad = summarize(rows, f"{name}：{PROFILES[name][0]}")
        out[name] = (rate, nbad)
    rate, nbad = out["default"]
    check("config 默认值：真实抓到 ≥ 80%", rate >= 0.80, f"{rate:.0%}")
    check("所有设置都没有危险动作", all(v[1] == 0 for v in out.values()),
          " ".join(f"{k}:{v[1]}" for k, v in out.items()))
    return out


if __name__ == "__main__":
    args = sys.argv[1:]
    if args[:1] == ["stats"]:
        stats(int(args[1]) if len(args) > 1 else 40)
    else:
        quick()
        if args[:1] != ["quick"]:
            stats()
    print(f"\n通过 {len(PASS)} 项，失败 {len(FAIL)} 项" + ("" if not FAIL else "：" + "；".join(FAIL)))
    sys.exit(1 if FAIL else 0)
