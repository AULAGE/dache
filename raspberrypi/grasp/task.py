"""
全自动抓取的任务流程（状态机），按"眼在手上"设计：相机装在机械臂腕部

  观察姿态 → 搜索 → 对准（闭环，自适应）→ 抓取 → 抬升确认 → 放置
                ↑__________ 丢失目标 / 抓取失败（最多 MAX_ATTEMPTS 次）

对准时机械臂保持 observe 姿态，只动三样东西：
  y      升降轴（整条机械臂连相机一起上下）
  x_deg  机械臂底座（相机跟着转）；底座转出去太多时，底盘转过去、底座回正
  d      底盘前进 / 后退
"""
import csv
import math
import os
import time

from devices import DeviceError, ManualOverride, lerp_pose
from servo import NOISE, build_axes, save_axes


class GraspTask:
    def __init__(self, cfg, clock, cam, lift, arm, chassis, log_dir=None, learn=True, verbose=True,
                 guard=None):
        self.cfg = cfg
        self.guard = guard            # 手柄接管检测（ManualGuard），None = 不检测
        self.can_turn = cfg.USE_CHASSIS and not cfg.CHASSIS_ACKERMANN   # 能不能原地转
        self.clock = clock
        self.cam = cam
        self.lift = lift
        self.arm = arm
        self.chassis = chassis
        self.verbose = verbose
        self.learn = learn
        self.axis_names = ["y", "x_deg", "d"] if cfg.USE_CHASSIS else ["y", "x_deg"]
        self.learn_path = os.path.join(log_dir, cfg.LEARN_FILE) if log_dir else None
        self.axes = build_axes(cfg, self.axis_names, self.learn_path if learn else None)
        self.log_dir = log_dir
        self.episode = 0
        self.base_deg = 0.0
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

    # ------------------------------------------------ 工具
    def say(self, *a):
        if self.verbose:
            print(f"[{self.clock.now():8.2f}]", *a)

    def measure(self, n=None):
        """等画面稳定后测量一次（每一步都会经过这里，顺便检查有没有被手柄接管）"""
        if self.guard is not None and self.guard.active:
            raise ManualOverride("手柄已连接，自动抓取暂停")
        self.clock.sleep(self.cfg.SETTLE_S)
        m = self.cam.measure(self.clock.now(), n or self.cfg.SAMPLE_N, self.cfg.MEASURE_TIMEOUT_S)
        # 连心跳都没有 = 串口线断了 / K230 程序停了：不能当成"没看到物块"继续开车乱找
        link_ok = getattr(self.cam, "link_ok", None)
        if m is None and link_ok is not None and not link_ok(2.0):
            raise DeviceError("K230 没有任何数据（连心跳都没有）：检查 K230 串口线和程序")
        return m

    def in_tol(self, e):
        return all(abs(e[n]) <= self.cfg.TOL[n] for n in self.axis_names)

    def _append_csv(self, name, row):
        if not self.log_dir:
            return
        path = os.path.join(self.log_dir, name)
        new = not os.path.exists(path)
        if not new:
            # 列变了（程序升级加了新列）：旧文件改名留档，开一个新文件，否则新行会和旧表头错位
            with open(path, newline="", encoding="utf-8") as f:
                header = next(csv.reader(f), [])
            if header != list(row.keys()):
                base, ext = os.path.splitext(path)
                os.replace(path, f"{base}_{time.strftime('%Y%m%d_%H%M%S')}{ext}")
                new = True
        with open(path, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if new:
                w.writeheader()
            w.writerow(row)

    # ------------------------------------------------ 机械臂
    def base_pulse(self, deg=None):
        c = self.cfg
        deg = self.base_deg if deg is None else deg
        return int(round(c.BASE_CENTER + c.BASE_DIR * deg * c.BASE_US_PER_DEG))

    def set_base(self, deg):
        c = self.cfg
        deg = max(-c.BASE_LIMIT_DEG, min(c.BASE_LIMIT_DEG, deg))
        ms = int(max(150, abs(deg - self.base_deg) * c.BASE_MS_PER_DEG))
        self.arm.move({c.BASE_ID: self.base_pulse(deg)}, ms)
        self.base_deg = deg
        return deg

    def base_after_move(self, m, base_deg, s_mm, dpsi_right_deg):
        """
        几何前馈：车走了 s_mm、车头右转 dpsi_right_deg 之后，底座要转到多少度，相机才正对物块。
        坐标（车体系）：x 向前、y 向左；底座转轴在车的转动中心前方 PIVOT_FWD_MM，
        相机在转轴外 R_CAM_MM，朝向沿半径向外（底座右转 = 朝向角减小）。
        """
        c = self.cfg
        R, pv = c.R_CAM_MM, c.PIVOT_FWD_MM
        phi = -math.radians(base_deg)
        ux, uy = math.cos(phi), math.sin(phi)            # 相机朝向
        cx, cy = pv + R * ux, R * uy                     # 相机位置
        tx = cx + m.d * ux + m.x * uy                    # 物块位置 = 相机 + 深度·朝向 − x右·左方向
        ty = cy + m.d * uy - m.x * ux
        psi = -math.radians(dpsi_right_deg)              # 车头左转为正
        px, py = s_mm * math.cos(psi / 2), s_mm * math.sin(psi / 2)   # 沿圆弧走：近似按弧中点朝向
        dx, dy = tx - px, ty - py
        qx = dx * math.cos(psi) + dy * math.sin(psi) - pv               # 物块在新车体系里、相对底座转轴
        qy = -dx * math.sin(psi) + dy * math.cos(psi)
        dist = math.hypot(qx, qy)
        # 让物块出现在 GOAL 的方位角上：迭代修正（相机在半径上，偏 1° 方位角 ≈ 1 + R/深度 度）
        want = -math.radians(c.GOAL["x_deg"])            # 左为正
        ph = math.atan2(qy, qx)
        for _ in range(5):
            ex, ey = qx - R * math.cos(ph), qy - R * math.sin(ph)
            bearing = math.atan2(-ex * math.sin(ph) + ey * math.cos(ph), ex * math.cos(ph) + ey * math.sin(ph))
            ph -= (bearing - want) / -(1.0 + R / max(dist - R, 50.0))
        return -math.degrees(ph)

    def go_observe(self, base_deg=None):
        c = self.cfg
        if base_deg is not None:
            self.base_deg = max(-c.BASE_LIMIT_DEG, min(c.BASE_LIMIT_DEG, base_deg))
        self.arm.move({**c.POSES["observe"], c.BASE_ID: self.base_pulse(), c.CLAW_ID: c.observe_claw()},
                      c.ARM_MOVE_MS)

    # ------------------------------------------------ 搜索
    def search(self):
        """
        每个高度：先只转底座扫前方（最快）→ 底盘掉头扫后方（仅差速底盘）；没找到再换高度
        """
        c = self.cfg
        if c.SEARCH_LOOK_FIRST:
            # 先原地看一眼（不动升降轴、不转底座）：物块往往就在标定时的位置附近
            m = self.measure()
            if m:                        # 原地这一眼一律复核：避免把一闪而过的误识别当真
                m = self.confirm_target(m)
            if m:
                self.say("搜索：原地就看到了目标")
                return m
        turns = [0] + (list(c.SEARCH_CHASSIS_TURNS) if self.can_turn else [])
        bases = list(c.SEARCH_BASE_DEG if c.USE_CHASSIS else c.SEARCH_BASE_DEG_NEAR)
        heights = c.SEARCH_LIFT_CM if c.USE_CHASSIS else c.SEARCH_LIFT_CM_NEAR
        rounds = 2 if (c.USE_CHASSIS and c.SEARCH_BACKOFF_MM) else 1
        turned = 0.0
        for r in range(rounds):
            if r:
                self.say(f"搜索：没找到，后退 {c.SEARCH_BACKOFF_MM}mm 再搜")
                self.set_base(0)
                back = (turned + 180.0) % 360.0 - 180.0      # 转回原方向，走最短的一边
                if abs(back) > 1 and self.can_turn:
                    self.chassis.turn(-back)
                turned = 0.0
                self.chassis.forward(-c.SEARCH_BACKOFF_MM)
            for h in heights:
                self.lift.move_to(h)
                for t in turns:
                    if t:
                        self.set_base(0)
                        self.chassis.turn(t)
                        turned += t
                    for b in bases:
                        self.set_base(b)
                        m = self.measure()
                        if m and c.SEARCH_CONFIRM:
                            m = self.confirm_target(m)
                        if m:
                            self.say(f"搜索：找到目标（高度 {h}cm，底座 {b:+.0f}°）")
                            return m
                    bases.reverse()          # 下一轮反向扫，少转一大圈
        return None

    def confirm_target(self, m):
        """
        误识别过滤（SEARCH_CONFIRM）：搜索时第一次看到目标，原地再测 1~2 次，位置一致才算真物块。
        K230 偶尔把别的东西认成物块，但一般只持续几帧；真物块不动，第二次测的位置应该差不多。
        """
        c = self.cfg
        for _ in range(2):
            m2 = self.measure()
            if m2 is None:
                continue
            same = (abs(m2.d - m.d) <= max(c.CONFIRM_D_MM, c.CONFIRM_D_FRAC * m.d)
                    and abs(m2.x_deg - m.x_deg) <= c.CONFIRM_X_DEG
                    and abs(m2.y - m.y) <= max(c.CONFIRM_D_MM, c.CONFIRM_D_FRAC * m.d))
            if same:
                return m2
            m = m2                       # 换成新看到的，再确认一次
        self.say("搜索：目标没有复现（可能是误识别），继续搜")
        return None

    # ------------------------------------------------ 对准
    def align(self, m):
        """闭环对准。返回 (是否对准, 最后一次测量, 用了几步)"""
        c = self.cfg
        self._abort = None
        strikes = {"y": 0, "x_deg": 0}          # 连续几次"动了但读数几乎没变"
        for step in range(c.MAX_ALIGN_STEPS):
            if m is None:
                self.say("对准：目标丢失")
                return False, None, step
            goal = self.goal_for(m)
            e = m.err(goal)
            # 升降轴已经在最低（或最高）点、还想往外走一点：误差在爪子能兜住的范围内就算对准了高度
            # （地上的物块只有升降轴在 0cm 时才够得着，读数稍有波动就会想"再往下一点"）
            pos0 = getattr(self.lift, "pos", None)
            if pos0 is not None and abs(e["y"]) <= c.LIFT_LIMIT_SLACK_MM and (
                    (pos0 <= 0.05 and e["y"] > 0) or (pos0 >= c.LIFT_MAX_CM - 0.05 and e["y"] < 0)):
                if abs(e["y"]) > c.TOL["y"]:
                    self.say(f"对准：升降轴已到{'最低' if e['y'] > 0 else '最高'}，剩余 y={e['y']:+.0f}mm 在容差"
                             f" {c.LIFT_LIMIT_SLACK_MM:.0f}mm 内，按已对准处理")
                e = dict(e, y=0.0)
            if self.in_tol(e):
                return True, m, step

            # 已知的几何系数：底座转动时相机绕转轴走一段弧，偏角变化比转角大
            s_x = 1.0 + c.R_CAM_MM / max(m.d, 50.0)
            s_d = max(0.2, math.cos(math.radians(self.base_deg)))
            u_y = self.axes["y"].command(e["y"]) if abs(e["y"]) > c.TOL["y"] else 0.0
            u_x = self.axes["x_deg"].command(e["x_deg"], s_x) if abs(e["x_deg"]) > c.TOL["x_deg"] else 0.0
            u_d = 0.0
            if c.USE_CHASSIS and abs(e["d"]) > c.TOL["d"]:
                u_d = self.axes["d"].command(e["d"], s_d)

            base_old = self.base_deg
            base_want = base_old + u_x
            base_new = max(-c.BASE_LIMIT_DEG, min(c.BASE_LIMIT_DEG, base_want))
            u_x = base_new - base_old
            recenter = c.USE_CHASSIS and abs(base_new) > c.BASE_RECENTER_DEG
            if self.can_turn:
                # 差速底盘：底座偏得多就原地转过去、底座回正；偏的时候不前进（否则斜着走）
                if recenter or abs(e["x_deg"]) > 10:
                    u_d = 0.0
            else:
                # 阿克曼底盘：不能原地转，只能在前进的同时把车头往底座的方向拐（见下面 drive）
                if (c.BASE_LIMIT_BACKUP_MM > 0 and c.USE_CHASSIS and not u_d
                        and abs(base_want) > c.BASE_LIMIT_DEG and abs(e["x_deg"]) > c.TOL["x_deg"]):
                    # 底座已经转到头、深度又刚好：原来会"没法更好"直接去抓（必然抓空）。
                    # 改成后退一段、同时把车头往物块方向拐，下一步再往前对深度
                    u_d = -c.BASE_LIMIT_BACKUP_MM
                    self.say(f"对准：底座到极限 {base_new:+.0f}° 还差 {e['x_deg']:+.1f}°，后退并转车头")
                recenter = recenter and bool(u_d)

            if not (u_y or u_x or u_d or recenter):
                return True, m, step       # 剩下的误差小于最小动作，没法更好了

            self.say("对准 #%d  误差 x=%+.1f° y=%+.1fmm d=%+.0fmm  底座 %+.1f° → %s" % (
                step + 1, e["x_deg"], e["y"], e.get("d", 0.0), base_old,
                " ".join(s for s in [
                    f"升降{u_y:+.2f}cm" if u_y else "",
                    f"底座{u_x:+.1f}°" if u_x else "",
                    f"前进{u_d:+.0f}mm" if u_d else "",
                    f"车头往底座方向拐" if recenter else ""] if s)))

            # 升降轴已经到头、还要往外走：再怎么调也够不着这个高度，别在原地来回试
            pos = getattr(self.lift, "pos", None)
            if u_y and pos is not None and abs(e["y"]) > 2 * c.TOL["y"] and (
                    (u_y < 0 and pos <= 0.05) or (u_y > 0 and pos >= c.LIFT_MAX_CM - 0.05)):
                self.say(f"对准：升降轴已经到{'最低' if u_y < 0 else '最高'}（{pos:.1f}cm），"
                         f"还差 y={e['y']:+.0f}mm，这个高度够不着")
                self._abort = "lift_limit"
                return False, m, step

            # 执行：升降轴和底盘 / 底座同时动
            if u_y:
                self.lift.start_move_by(u_y)
            if recenter and self.can_turn:
                self.chassis.turn(base_new)
                self.set_base(0)
            elif u_d:
                # 边走边转；走完按实际里程 + 转角做几何前馈，直接算出底座该转到哪（含平移视差）
                moved, dpsi = self.chassis.drive(u_d, base_new if recenter else 0.0)
                if c.GEOM_FEEDFORWARD:
                    target = self.base_after_move(m, base_old, moved, dpsi)
                    target = max(-c.BASE_LIMIT_DEG, min(c.BASE_LIMIT_DEG, target))
                else:
                    target = base_new - dpsi
                self.set_base(target)
                u_x = target - base_old
                recenter = abs(dpsi) > 0.5
            elif u_x:
                self.set_base(base_new)
            if u_y:
                self.lift.wait_move()

            # 离目标还远时少取几帧（快），快对准了再取足帧数（准）
            far = any(abs(e[k]) > 4 * c.TOL[k] for k in self.axis_names)
            m2 = self.measure(c.SAMPLE_N_FAR if far else None)

            if m2 is not None and c.GHOST_CHECK:
                e2 = m2.err(goal)
                checks = []
                if u_y:
                    checks.append(("y", self.axes["y"].j * u_y, e2["y"] - e["y"], max(6.0, 3 * NOISE["y"])))
                if u_x and not recenter and not u_d:
                    checks.append(("x_deg", self.axes["x_deg"].j * s_x * u_x, e2["x_deg"] - e["x_deg"], 1.5))
                for ax, pred, got, thresh in checks:
                    if abs(pred) >= thresh:
                        strikes[ax] = strikes[ax] + 1 if abs(got) < 0.3 * abs(pred) else 0
                if max(strikes.values()) >= 2:
                    what = "升降轴" if strikes["y"] >= 2 else "底座"
                    self.say(f"对准：{what}动了两次，物块的读数几乎没变 → K230 识别到的很可能是机械臂自己"
                             f"（例如爪子），停止。请在 IDE 画面里看识别框框在哪")
                    self._abort = "ghost"
                    return False, m2, step + 1

            if m2 is not None and self.learn:
                e2 = m2.err(goal)
                # 只学习没有被别的动作干扰的轴
                if u_y:
                    self.axes["y"].learn(u_y, e["y"], e2["y"])
                if u_x and not recenter and not u_d:
                    self.axes["x_deg"].learn(u_x, e["x_deg"], e2["x_deg"], s_x)
                if u_d and not u_x and not recenter:
                    self.axes["d"].learn(u_d, e["d"], e2["d"], s_d)

            self._append_csv("steps.csv", {
                "episode": self.episode, "step": step + 1, "base_deg": round(base_old, 1),
                "lift_cm": None if pos is None else round(pos, 2),
                **{f"e_{n}": round(e[n], 2) for n in self.axis_names},
                "u_y": round(u_y, 3), "u_x": round(u_x, 2), "u_d": round(u_d, 1), "recenter": int(recenter),
                **{f"J_{n}": round(self.axes[n].j, 4) for n in self.axis_names}})
            m = m2
        return False, m, c.MAX_ALIGN_STEPS

    # ------------------------------------------------ 抓取
    def grasp(self, m):
        """伸出 → 合爪 → 抬升；返回是否确认抓到"""
        c = self.cfg
        self._grasp_ref = m
        t = (m.d - c.REACH_NEAR_D) / (c.REACH_FAR_D - c.REACH_NEAR_D)
        if not 0 <= t <= 1 and not c.REACH_RANGE_OK:
            raise DeviceError(f"物块深度 {m.d:.0f}mm 超出机械臂范围")
        reach = lerp_pose(c.POSES["reach_near"], c.POSES["reach_far"], t)
        # 左右微调：对准后爪子系统性偏左/偏右时，伸出前让底座多转一点（calib.py trim 设置）
        trim = c.GRASP_BASE_TRIM_DEG
        bp = self.base_pulse(max(-c.BASE_LIMIT_DEG, min(c.BASE_LIMIT_DEG, self.base_deg + trim)))

        # 给 bench.py 记录用（不影响流程）：最后一次伸爪时对准测到的物块位置和当时的姿态
        lift_pos = getattr(self.lift, "pos", None)
        if lift_pos is None and hasattr(self.lift, "position"):     # 仿真升降轴没有 pos（真机 preflight 后总有）
            try:
                lift_pos = self.lift.position()
            except DeviceError:
                pass
        self.last_grasp = {"x_deg": round(m.x_deg, 2), "y": round(m.y, 1), "d": round(m.d, 1),
                           "base_deg": round(self.base_deg, 2),
                           "lift_cm": None if lift_pos is None else round(lift_pos, 2),
                           "t": round(t, 3), "trim_deg": trim}
        self.say(f"抓取：深度 {m.d:.0f}mm，插值 t={max(0.0, min(1.0, t)):.2f}，底座 {self.base_deg:+.1f}°"
                 + (f"（微调 {trim:+.1f}°）" if trim else ""))
        self.arm.move({c.CLAW_ID: c.CLAW_OPEN}, 300)
        self.arm.move({**c.POSES["ready"], c.BASE_ID: bp}, c.ARM_MOVE_MS)
        self.arm.move({**reach, c.BASE_ID: bp}, c.ARM_SLOW_MS)
        self.arm.move({c.CLAW_ID: c.CLAW_CLOSE}, 500)

        verify = c.VERIFY
        m_before = self.measure() if verify in ("lift_check", "auto") else None
        self.lift.move_by(c.LIFT_AFTER_GRASP_CM)
        self._lifted = True
        m_after = self.measure() if verify in ("lift_check", "auto") else None
        self.arm.move({**c.POSES["carry"], c.BASE_ID: bp}, c.ARM_MOVE_MS)

        if verify == "none":
            return True
        if verify == "observe":                  # 只做观察确认（爪子在画面里时抬升确认会被骗）
            return self._observe_check(bp)
        ok = self._lift_check(m_before, m_after)
        if ok is None and verify == "auto":
            ok = self._observe_check(bp)
        if ok is None:
            self.say("确认：无法判断，按成功处理")
            return True
        return ok

    def goal_for(self, m):
        """
        对准目标。没有底盘时深度没法对准，物块在近/远两个标定姿态之间时，
        目标的 Y 和方位角也要按深度插值（物块越远，在画面里越靠上），否则升降轴会对到错误的高度。
        有底盘时底盘会把深度对到 GOAL，用固定的 GOAL。
        """
        c = self.cfg
        g, gf = c.GOAL, getattr(c, "GOAL_FAR", None)
        if c.USE_CHASSIS or not gf or m is None or gf["d"] == g["d"]:
            return g
        t = max(0.0, min(1.0, (m.d - g["d"]) / (gf["d"] - g["d"])))
        return {"x_deg": g["x_deg"] + (gf["x_deg"] - g["x_deg"]) * t,
                "y": g["y"] + (gf["y"] - g["y"]) * t,
                "d": m.d}                         # 深度不对准（够不够得着由 reachable 判断）

    def reach_range(self):
        """机械臂（不靠底盘）能抓到的物块深度范围 (近, 远)，单位 mm"""
        c = self.cfg
        if c.POSES.get("reach_near") == c.POSES.get("reach_far"):
            lo = hi = c.GOAL["d"]                    # 只标定了一个伸出姿态：只能抓 GOAL 那个深度
        else:
            lo, hi = sorted((c.REACH_NEAR_D, c.REACH_FAR_D))
        return lo - c.REACH_MARGIN_MM, hi + c.REACH_MARGIN_MM

    def reachable(self, m):
        """没有底盘时深度不会被对准：物块超出臂展就不去抓（抓了也是空抓）"""
        if self.cfg.USE_CHASSIS:
            return True
        lo, hi = self.reach_range()
        if lo <= m.d <= hi:
            return True
        hint = "往前推近一点" if m.d > hi else "往后挪远一点"
        self.say(f"够不着：物块深度 {m.d:.0f}mm，机械臂只能抓 {lo:.0f}~{hi:.0f}mm。请把物块{hint}，"
                 f"或者去掉 --no-chassis 让底盘去对准")
        return False

    def _lift_check(self, m_before, m_after):
        """抬升确认：抓到了物块跟着相机走（y 不变）；没抓到物块留在原地（y 变大或看不见）。None = 判断不了"""
        c = self.cfg
        if m_before is None:
            self.say("抬升确认：伸出后看不到物块（离得太近、碰到画面边缘），判断不了")
            return None
        if m_after is None:
            self.say("抬升确认：抬起后物块不见了（留在原地）→ 没抓到")
            return False
        dy = m_after.y - m_before.y
        ok = abs(dy) < c.VERIFY_DY_MM
        self.say(f"抬升确认：抬升后物块在画面里移动 {dy:+.1f}mm → {'抓到' if ok else '没抓到'}")
        return ok

    def _observe_check(self, bp):
        """
        观察确认：爪子保持闭合，退回观察姿态、升降轴降回抓取时的高度，看原来的位置还有没有物块。
        还在（和 GOAL 差不多的位置）= 没抓到；不在 = 抓到了。
        """
        c = self.cfg
        ref = getattr(self, "_grasp_ref", None)    # 对准时实际看到的物块位置（没有就用 GOAL）
        g = {"x_deg": ref.x_deg, "y": ref.y, "d": ref.d} if ref is not None else c.GOAL
        self.arm.move({**c.POSES["observe"], c.BASE_ID: bp}, c.ARM_MOVE_MS)     # 爪子不动
        self.lift.move_by(-c.LIFT_AFTER_GRASP_CM)
        self._lifted = False
        m = self.measure()
        still = (m is not None and abs(m.d - g["d"]) < c.VERIFY_SAME_D_MM
                 and abs(m.x_deg - g["x_deg"]) < 6 and abs(m.y - g["y"]) < 30)
        self.say("观察确认：" + ("原位置还能看到物块 → 没抓到" if still else "原位置没有物块了 → 抓到"))
        if not still:                       # 抓到了：重新抬起、收回，准备放置
            self.lift.move_by(c.LIFT_AFTER_GRASP_CM)
            self._lifted = True
            self.arm.move({**c.POSES["carry"], c.BASE_ID: bp}, c.ARM_MOVE_MS)
        return not still

    def _lower_back(self):
        """升降轴回到抓取前的高度（只在抬起过时）"""
        if getattr(self, "_lifted", False):
            self.lift.move_by(-self.cfg.LIFT_AFTER_GRASP_CM)
            self._lifted = False

    def place(self):
        c = self.cfg
        drop = c.POSES.get("drop")
        if drop:
            self.arm.move({c.BASE_ID: self.base_pulse(), **drop}, c.ARM_MOVE_MS)
            self.arm.move({c.CLAW_ID: c.CLAW_OPEN}, 400)
        self.go_observe(0)
        self._lower_back()

    def release_and_back_off(self):
        c = self.cfg
        self.arm.move({c.CLAW_ID: c.CLAW_OPEN}, 400)
        self.go_observe()
        self._lower_back()
        if c.USE_CHASSIS:
            self.chassis.forward(-60)        # 退一点再重新看

    # ------------------------------------------------ 一次完整抓取
    def run_once(self):
        c = self.cfg
        self.episode += 1
        self.last_grasp = None          # 这次有没有伸过爪、伸爪时的位置（bench.py 用）
        t0 = self.clock.now()
        steps = 0
        attempts = 0
        result = "not_found"
        last_err = None

        self.go_observe(0)
        m = self.search()
        while m is not None and attempts < c.MAX_ATTEMPTS:
            attempts += 1
            ok, m_al, n = self.align(m)
            steps += n
            if m_al is not None:
                last_err = m_al.err(self.goal_for(m_al))
            if not ok and getattr(self, "_abort", None):
                result = self._abort
                self.go_observe()
                break
            if not ok:
                result = "lost" if m_al is None else "align_failed"
                self.go_observe()
                m = self.search()
                continue
            if not self.reachable(m_al):
                result = "out_of_reach"
                break
            steps += 1
            if self.grasp(m_al):
                result = "success"
                self.place()
                break
            result = "grasp_failed"
            self.say("抓取失败，松开、退后重试")
            self.release_and_back_off()
            m = self.measure() or self.search()

        dt = self.clock.now() - t0
        err_norm = 0.0
        if last_err:
            err_norm = sum(abs(last_err[n]) / c.TOL[n] for n in self.axis_names) / len(self.axis_names)
        score = (100.0 if result == "success" else 0.0) - c.SCORE_STEP_COST * steps \
            - c.SCORE_ERR_COST * err_norm - c.SCORE_TIME_COST * dt

        if self.learn and self.learn_path:
            save_axes(self.axes, self.learn_path)
        row = {"episode": self.episode, "result": result, "attempts": attempts, "steps": steps,
               "time_s": round(dt, 1), "err_norm": round(err_norm, 2), "score": round(score, 1),
               **{f"J_{n}": round(a.j, 4) for n, a in self.axes.items()}}
        self._append_csv("episodes.csv", row)
        self.say(f"结果：{result}  步数 {steps}  用时 {dt:.1f}s  评分 {score:.1f}")
        return row
