"""
仿真环境（眼在手上）：不接任何硬件，在电脑上验证任务逻辑和参数。

几何：
  小车中心 → 前方 PIVOT_FWD 处是机械臂底座转轴（底座装在升降台上）
  observe 姿态下相机在转轴外 r_cam_true 处，朝向 = 车头朝向 - 底座角度（底座右转为正）
  ready / reach 姿态相机往前伸 CAM_ADV[姿态] mm
  相机高度 = CAM_Z0 + 升降轴高度

误差来源：
  - K230 尺寸法测深有固定的系统误差（每台车不同），x/y 按同样比例偏；另有随机噪声和偶尔丢帧
  - 真实相机半径和 config.R_CAM_MM 不一样（量得不准）
  - 底盘转向、直行 ±5% 误差，直行会轻微跑偏
  - 机械爪：观察姿态下真实误差在抓取容差内才抓得住，且有 10% 随机失败
"""
import math
import random

from devices import Measurement

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


class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        if s > 0:
            self.t += s


class World:
    CAM_Z0 = 100.0     # 升降轴在 0 时相机离地高度 mm
    PIVOT_FWD = 80.0   # 底座转轴在小车中心前方多远

    def __init__(self, cfg, rng, depth_bias, r_cam_true):
        self.cfg = cfg
        self.rng = rng
        self.depth_bias = depth_bias
        self.r_cam = r_cam_true
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


class SimCam:
    def __init__(self, world, clock, rng):
        self.w, self.clock, self.rng = world, clock, rng

    def measure(self, since, n, timeout):
        v = self.w.true_view()
        self.clock.sleep(n / float(self.w.cfg.K230_FPS))   # 实测约 7 帧/秒
        if v is None or self.rng.random() < 0.03:
            return None
        x, y, d = v
        k = self.w.depth_bias
        g = self.rng.gauss
        return Measurement(x * k + g(0, 1.0), y * k + g(0, 1.0), d * k + g(0, 0.01 * d + 2), n)


class SimLift:
    MAX = 50.0

    def __init__(self, world, clock):
        self.w, self.clock = world, clock
        self._done_at = 0.0

    def position(self):
        return self.w.lift_cm

    def move_to(self, cm, timeout=30):
        cm = max(0.0, min(self.MAX, cm))
        self.clock.sleep(abs(cm - self.w.lift_cm) / 8.0 + 0.2)
        self.w.lift_cm = cm
        return cm

    def move_by(self, cm, timeout=30):
        return self.move_to(self.w.lift_cm + cm)

    def start_move_by(self, cm):
        target = max(0.0, min(self.MAX, self.w.lift_cm + cm))
        self._done_at = self.clock.now() + abs(target - self.w.lift_cm) / 8.0 + 0.2
        self.w.lift_cm = target      # 仿真简化：位置立即生效，时间在 wait_move 里补齐

    def wait_move(self, timeout=30):
        self.clock.sleep(self._done_at - self.clock.now())
        return self.w.lift_cm

    def stop(self):
        pass


class SimChassis:
    """ackermann=True 时不能原地转，drive 的转角受最小转弯半径限制（和真机驱动一致）"""

    def __init__(self, world, clock, rng, ackermann=True, min_radius=600.0):
        self.w, self.clock, self.rng = world, clock, rng
        self.ackermann = ackermann
        self.min_radius = min_radius

    def turn(self, deg, timeout=15):
        if self.ackermann:
            raise RuntimeError("阿克曼底盘不能原地转向")
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


class SimArm:
    # 机械爪实际能抓住的范围（观察姿态下的测量值，比对准容差宽一些）
    GRIP_TOL = {"x_deg": 3.0, "y": 8.0, "d": 18.0}

    def __init__(self, world, clock, cfg, rng):
        self.w, self.clock, self.cfg, self.rng = world, clock, cfg, rng

    def move(self, pulses, ms):
        c, w = self.cfg, self.w
        self.clock.sleep(ms / 1000.0 + 0.15)
        if 3 in pulses:
            w.pose = pose_of(pulses[3])
        if c.BASE_ID in pulses:
            w.base_deg = (pulses[c.BASE_ID] - c.BASE_CENTER) / (c.BASE_US_PER_DEG * c.BASE_DIR)
        claw = pulses.get(c.CLAW_ID)
        if claw == c.CLAW_CLOSE and not w.held:
            v = w.true_view("observe", fov=False)
            if v is None:
                return
            k = w.depth_bias      # 用户是按 K230 读数标定 GOAL 的，所以按"测量值"比较
            x, y, d = v[0] * k, v[1] * k, v[2] * k
            g = c.GOAL
            ok_d = abs(d - g["d"]) <= self.GRIP_TOL["d"] if c.USE_CHASSIS else \
                c.REACH_NEAR_D - 10 <= d <= c.REACH_FAR_D + 10
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


def make_sim(cfg, seed, depth_bias, r_cam_true):
    rng = random.Random(seed)
    clock = SimClock()
    world = World(cfg, rng, depth_bias, r_cam_true)
    return world, clock, SimCam(world, clock, rng), SimLift(world, clock), \
        SimArm(world, clock, cfg, rng),         SimChassis(world, clock, rng, cfg.CHASSIS_ACKERMANN, cfg.ACK_MIN_RADIUS_MM)
