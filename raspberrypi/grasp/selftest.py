#!/usr/bin/env python3
"""
离线自测：不接任何硬件，用假的下位机把每个驱动和整套流程测一遍。
改完代码先跑它，全部 ✅ 再上真机。

  python3 selftest.py          全部测试（约 1 分钟）
  python3 selftest.py quick    跳过耗时的仿真回归
"""
import importlib
import json
import math
import os
import sys
import tempfile
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["GRASP_IGNORE_CALIB"] = "1"     # 测试按 config.py 默认值写，忽略这台车的 calib.json

import devices  # noqa: E402

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   {detail}" if detail else ""))


class FakeSerialException(OSError):
    pass


def install_fake(factory):
    devices.serial = types.SimpleNamespace(Serial=factory, SerialException=FakeSerialException)


# ------------------------------------------------------------ 机械臂：和 arm.txt 逐字节比对
def test_arm():
    print("机械臂协议（和 ~/arm.txt 里 Lobot 上位机录的帧比对）")
    sent = []

    class S:
        def __init__(self, *a, **k):
            pass

        def write(self, b):
            sent.append(bytes(b))

        def flush(self):
            pass

        def close(self):
            pass

    install_fake(S)
    clock = types.SimpleNamespace(sleep=lambda s: None, now=time.monotonic)
    arm = devices.Arm("x", 9600, clock)
    # arm.txt 第 1 行和第 8 行
    arm.move({6: 1500, 1: 1500, 2: 1500, 3: 758, 4: 601, 5: 1185}, 1000)
    arm.move({1: 500, 2: 1500, 3: 758, 4: 714, 5: 1724, 6: 2287}, 500)
    want1 = bytes.fromhex("55 55 17 03 06 E8 03 01 DC 05 02 DC 05 03 F6 02 04 59 02 05 A1 04 06 DC 05")
    want8 = bytes.fromhex("55 55 17 03 06 F4 01 01 F4 01 02 DC 05 03 F6 02 04 CA 02 05 BC 06 06 EF 08")
    check("初始姿态帧与 arm.txt 第 1 行一致", sent[0] == want1, sent[0].hex(" "))
    check("松爪帧与 arm.txt 第 8 行一致", sent[1] == want8)

    # USB 掉线后自动重连
    state = {"fail": 1, "opens": 0}

    class Flaky(S):
        def __init__(self, *a, **k):
            state["opens"] += 1

        def write(self, b):
            if state["fail"]:
                state["fail"] -= 1
                raise FakeSerialException("device disconnected")
            sent.append(bytes(b))

    install_fake(Flaky)
    arm = devices.Arm("x", 9600, clock)
    n0 = len(sent)
    arm.move({1: 500}, 300)
    check("USB 掉线后自动重开串口并重发", len(sent) == n0 + 1 and arm.reconnects == 1 and state["opens"] == 2)

    # 重连后马上又掉一次（接触不良时会这样）：原行为抛串口异常；robust 模式反复重连并整帧重发所有舵机
    state.update(fail=2, opens=0)
    arm = devices.Arm("x", 9600, clock)
    arm.state = {6: 1600, 3: 758}
    try:
        arm.move({1: 500}, 300)
        check("连续掉两次：原行为抛异常（ARM_USB_ROBUST=False）", False)
    except FakeSerialException:
        check("连续掉两次：原行为抛异常（ARM_USB_ROBUST=False）", True)
    state.update(fail=2, opens=0)
    arm = devices.Arm("x", 9600, clock, robust=True)
    arm.state = {6: 1600, 3: 758}
    n0 = len(sent)
    arm.move({1: 500}, 300)
    want = bytes.fromhex("55 55 0E 03 03 2C 01 01 F4 01 03 F6 02 06 40 06")
    check("连续掉两次：robust 模式重连成功，整帧重发所有舵机", len(sent) == n0 + 1 and sent[-1] == want
          and arm.reconnects == 2, sent[-1].hex(" "))


# ------------------------------------------------------------ K230 帧解析
def test_k230():
    print("K230 帧解析")

    def fr(x, y, d):
        f = bytearray([0xAA, 0x55]) + (x & 0xFFFF).to_bytes(2, "big") + (y & 0xFFFF).to_bytes(2, "big") \
            + d.to_bytes(2, "big")
        return bytes(f) + bytes([devices.crc8(f), 0x0D])

    chunks = [b"\x12\xaa", fr(-47, 97, 382)[:4], fr(-47, 97, 382)[4:], fr(0, 0, 0), b"\xaa\x55\x00",
              fr(10, -5, 200)]
    bad = bytearray(fr(1, 2, 3))
    bad[8] ^= 0xFF
    chunks.append(bytes(bad))

    class S:
        def __init__(self, *a, **k):
            pass

        def read(self, n):
            time.sleep(0.005)
            return chunks.pop(0) if chunks else b""

    install_fake(S)
    cam = devices.K230("x", 115200, devices.RealClock(), 0.4)
    time.sleep(0.2)
    got = [f[1:] for f in cam.frames]
    check("位置帧（含被拆成两半的帧）", got == [(-47, 97, 382), (10, -5, 200)], str(got))
    check("心跳帧计数、不当成物块", cam.heartbeats == 1)
    check("CRC 错的帧被丢弃", cam.bad_frames >= 1)
    check("link_ok / visible", cam.link_ok() and cam.visible())
    m = cam.measure(0, 2, 0.1)
    check("measure 取中位数", m is not None and m.n == 2)
    cam.close()
    check("坐标旋转 90°", devices.rotate_xy(10, 20, 90) == (20, -10))


# ------------------------------------------------------------ 升降轴：假 STM32
class FakeLiftMCU:
    """按 伺服电机控制 固件的协议回话"""

    def __init__(self, homed=True):
        self.pos = 0.0
        self.homed = homed
        self.out = []
        self.rx = b""
        self.cmds = []

    def __call__(self, *a, **k):
        return self

    def write(self, b):
        self.rx += b
        while b"\n" in self.rx:
            line, self.rx = self.rx.split(b"\n", 1)
            self.handle(line.decode().strip())

    def handle(self, line):
        if not line:
            return
        self.cmds.append(line)
        c = line[0].upper()
        if c == "P":
            self.out.append(f"POS {self.pos:.2f} {'IDLE' if self.homed else 'UNHOMED'}")
        elif c == "A":
            t = float(line[1:])
            if not 0 <= t <= 50:
                self.out.append("ERR RANGE 0.00-50.00")
                return
            self.out.append(f"OK GO {t:.2f}")
            self.pos = t
            self.out.append(f"DONE {t:.2f}")
        elif c == "E":
            self.out.append("OK E")
        else:
            self.out.append("ERR UNKNOWN")

    def readline(self):
        return (self.out.pop(0) + "\r\n").encode() if self.out else b""

    def reset_input_buffer(self):
        self.out.clear()

    def close(self):
        pass


def test_lift():
    print("升降轴协议（假 STM32）")
    mcu = FakeLiftMCU()
    install_fake(mcu)
    lift = devices.Lift("x", 115200, max_cm=44)
    check("打开串口先发空行清缓冲", mcu.rx == b"" and not mcu.cmds)
    check("position", lift.position() == 0.0)
    check("move_to 10", lift.move_to(10) == 10.0)
    check("软件上限：move_to(60) 截断到 44", lift.move_to(60) == 44.0, mcu.cmds[-1])
    lift.move_to(40)
    check("move_by(+10) 从 40 截断到 44", lift.move_by(10) == 44.0)
    check("move_by(-50) 截断到 0", lift.move_by(-50) == 0.0)
    lift.start_move_by(5)
    check("start_move_by + wait_move", lift.wait_move() == 5.0)
    mcu2 = FakeLiftMCU(homed=False)
    install_fake(mcu2)
    try:
        devices.Lift("x", 115200).position()
        check("UNHOMED 报错", False)
    except devices.DeviceError as e:
        check("UNHOMED 报错", "UNHOMED" in str(e))


# ------------------------------------------------------------ 底盘：带平滑减速的假控制器
def test_chassis(tau_true=0.45):
    print(f"底盘里程控制（假控制器：滑行时间常数 {tau_true}s）")

    class S:
        def __init__(self, *a, **k):
            self.cmd_v = 0.0
            self.v = 0.0
            self.t = time.monotonic()

        def write(self, b):
            if len(b) == 11 and b[0] == 0x7B:
                self.cmd_v = int.from_bytes(b[3:5], "big", signed=True)

        def read(self, n):
            time.sleep(0.05)
            now = time.monotonic()
            dt, self.t = now - self.t, now
            self.v += (self.cmd_v - self.v) * min(1, dt / tau_true)
            vx = int(self.v) & 0xFFFF
            f = bytearray([0x7B, 0, vx >> 8, vx & 0xFF] + [0] * 16 + [0x64, 0xF6])
            b = 0
            for x in f:
                b ^= x
            return bytes(f + bytes([b, 0x7D]))

        def close(self):
            pass

    install_fake(S)
    tau_file = os.path.join(tempfile.mkdtemp(), "tau.json")
    c = devices.WheeltecChassis("x", 115200, speed_mm_s=100, tau_file=tau_file)
    check("收到回传、电压解析", c.has_feedback() and abs(c.voltage_mv - 25846) < 1, f"{c.voltage_mv}mV")
    errs = []
    for mm in (100, 100, -100, 150):
        moved, _ = c.drive(mm)
        errs.append(abs(moved) - abs(mm))
    c.close()
    check("走 2 次后停车误差 ≤ 8mm", all(abs(e) <= 8 for e in errs[2:]), " ".join(f"{e:+.1f}" for e in errs))
    check("学到的 τ 存进文件", os.path.exists(tau_file))
    c2 = devices.WheeltecChassis("x", 115200, tau_file=tau_file)
    check("下次启动读回 τ", abs(c2.coast_tau - json.load(open(tau_file))["coast_tau"]) < 1e-9)
    c2.close()
    try:
        c2.turn(30)
        check("阿克曼不能原地转（报错）", False)
    except devices.DeviceError:
        check("阿克曼不能原地转（报错）", True)


# ------------------------------------------------------------ 底盘：按固件源码复现的假控制器
class FwChassisModel:
    """
    按 底盘代码/大车底盘 源码（阿克曼模式）逐步复现的假底盘控制器，100Hz 一拍：
      串口收帧 usartx.c:645-705 → XYZ_Target_Speed_transition usartx.c:829-840
      → Vz_to_Akm_Angle usartx.c:784-820 → Drive_Motor 阿克曼分支 balance.c:70-107（阿克曼不做平滑，balance.c:24）
      → Incremental_PI balance.c:378-397（KP=KI=700 system.c:49，限幅 ±16800）→ 回传 usartx.c:40-148（20Hz）
      固件没有串口超时：Move_X 一直保持到下一帧。
    电机（固件之外，假设的）：一阶惯性 + 库仑摩擦，稳态速度 = KV × 电压/参考电压 × (PWM − 摩擦)。
      KV 取得让 τ_PI = 0.01/(KI·KV) = 0.45s，对上真机"100mm/s 松油门滑约 45mm"。
    """
    DT = 0.01
    KP = KI = 700.0
    W, L = 0.162, 0.144          # 轮距、轴距（m），robot_select_init.h:37/45（Mini 车的值，假控制器够用）
    MIN_R = 0.350                # robot_select_init.h:31
    KV = 0.01 / (700.0 * 0.45)
    V_REF = 25.8
    FRICTION = 500.0
    TM = 0.06

    def __init__(self, voltage=25.8, steer_offset_rad=0.0):
        self.voltage = voltage
        self.steer_offset = steer_offset_rad
        self.move_x = self.move_z = 0.0
        self.va = self.vb = 0.0                 # 左后轮 A、右后轮 B 实际速度 m/s
        self.pwm = [0.0, 0.0]
        self.last_bias = [0.0, 0.0]
        self.yaw_rate = 0.0
        self.flag_stop = 0
        self.rxbuf = bytearray()

    @staticmethod
    def xyz(h, l):
        t = (h << 8) + l
        if t >= 0x8000:
            t -= 0x10000
        q = int(t / 1000)                       # C 的整数除法向 0 截断
        return q + (t - q * 1000) * 0.001

    def vz_to_akm_angle(self, vx, vz):
        if vz != 0 and vx != 0:
            if abs(vx / vz) <= self.MIN_R:
                vz = abs(vx) / self.MIN_R if vz > 0 else -abs(vx) / self.MIN_R
            return math.atan(self.L / (vx / vz + 0.5 * self.W))
        return 0.0

    def feed(self, data):
        for byte in data:
            if byte == 0x7B or self.rxbuf:
                self.rxbuf.append(byte)
            if len(self.rxbuf) == 11:
                b = bytes(self.rxbuf)
                self.rxbuf.clear()
                x = 0
                for v in b[:9]:
                    x ^= v
                if b[10] == 0x7D and x == b[9]:
                    self.move_x = self.xyz(b[3], b[4])
                    self.move_z = self.vz_to_akm_angle(self.move_x, self.xyz(b[7], b[8]))

    def _pi(self, i, enc, target):
        bias = target - enc
        self.pwm[i] += self.KP * (bias - self.last_bias[i]) + self.KI * bias
        self.pwm[i] = max(-16800.0, min(16800.0, self.pwm[i]))
        self.last_bias[i] = bias
        return self.pwm[i]

    def _plant(self, v, pwm):
        eff = 0.0 if abs(pwm) <= self.FRICTION else pwm - math.copysign(self.FRICTION, pwm)
        return v + (self.KV * self.voltage / self.V_REF * eff - v) * self.DT / self.TM

    def tick(self):
        vx, ang = self.move_x, self.move_z
        r = self.L / math.tan(ang) - 0.5 * self.W if ang != 0 else 0.0
        ang = max(-0.49, min(0.32, ang))
        ta, tb = (vx * (r - 0.5 * self.W) / r, vx * (r + 0.5 * self.W) / r) if ang != 0 else (vx, vx)
        pa = self._pi(0, self.va, max(-3.5, min(3.5, ta)))
        pb = self._pi(1, self.vb, max(-3.5, min(3.5, tb)))
        self.va, self.vb = self._plant(self.va, pa), self._plant(self.vb, pb)
        actual = ang + self.steer_offset        # 转向没回正 = 实际转角多一个偏置（编码器看不出来，陀螺仪看得出）
        self.yaw_rate = (self.va + self.vb) / 2 * math.tan(actual) / self.L if actual else 0.0

    def feedback(self):
        def s16(x):
            return int(x) & 0xFFFF
        vx = s16((self.va + self.vb) / 2 * 1000)
        wz = s16((self.vb - self.va) / self.W * 1000)
        gz = s16(math.degrees(self.yaw_rate) * 65.5)
        mv = int(self.voltage * 1000)
        f = bytearray([0x7B, self.flag_stop, vx >> 8, vx & 0xFF, 0, 0, wz >> 8, wz & 0xFF]
                      + [0] * 10 + [gz >> 8, gz & 0xFF, (mv >> 8) & 0xFF, mv & 0xFF])
        b = 0
        for x in f:
            b ^= x
        return bytes(f + bytes([b, 0x7D]))


class FwSerial:
    """把 FwChassisModel 包成假串口：按墙上时钟推进 100Hz 控制，每 50ms 回一帧"""
    model = None

    def __init__(self, *a, **k):
        self.m = FwSerial.model
        self.lock = threading.Lock()
        self.t = time.monotonic()
        self.writes = []

    def write(self, b):
        with self.lock:
            self.writes.append(bytes(b))
            self.m.feed(b)

    def flush(self):
        pass

    def read(self, n):
        time.sleep(0.05)
        with self.lock:
            k = int((time.monotonic() - self.t) / self.m.DT)
            for _ in range(min(k, 50)):
                self.m.tick()
            self.t += k * self.m.DT
            return self.m.feedback()

    def close(self):
        pass


def test_chassis_fw():
    print("底盘（假控制器按固件源码复现：阿克曼不平滑 + 增量 PI + 无串口超时）")
    M = FwChassisModel

    def coast(v0, volt=25.8):
        m = M(voltage=volt)
        m.move_x = v0
        for _ in range(400):
            m.tick()
        vs, d = (m.va + m.vb) / 2, 0.0
        m.move_x = 0.0
        for _ in range(300):
            m.tick()
            d += (m.va + m.vb) / 2 * m.DT
        return d / vs
    taus = [coast(v) for v in (0.05, 0.10, 0.15)]
    check("PI 积分恒等式：滑行/速度 和速度无关（τ 模型形式正确）", max(taus) - min(taus) < 0.02,
          " ".join(f"{t:.3f}s" for t in taus))
    t_low = coast(0.10, 22.5)
    check("τ_PI 和电压成反比", abs(t_low / taus[1] - 25.8 / 22.5) < 0.05, f"22.5V: {t_low:.3f}s")
    m = M()
    m.feed(devices.WheeltecChassis.frame(-100, 0))
    check("驱动下发帧被固件解析（−100mm/s）", abs(m.move_x + 0.1) < 1e-9, f"{m.move_x}")
    m.feed(devices.WheeltecChassis.frame(0, 300))
    check("vx=0 时固件把转角置 0（阿克曼不能原地转）", m.move_z == 0.0)
    m.feed(devices.WheeltecChassis.frame(100, 1000))
    check("转弯半径小于固件最小半径会被截到 350mm", abs(m.move_z - math.atan(0.144 / (0.35 + 0.081))) < 1e-6)

    install_fake(FwSerial)

    def runs(mode, seq, volt=25.8, **kw):
        FwSerial.model = M(voltage=volt)
        c = devices.WheeltecChassis("x", 115200, speed_mm_s=100, stop_model=mode, **kw)
        errs = [abs(c.drive(mm)[0]) - abs(mm) for mm in seq]
        return c, errs

    c, errs = runs("tau", (100, 100, -100))
    check("原做法（tau）在固件假控制器上 2 次后误差 ≤ 8mm", all(abs(e) <= 8 for e in errs[1:]),
          " ".join(f"{e:+.1f}" for e in errs))
    learned = (c.coast_tau, c.pi_tau)
    c.close()
    c, errs = runs("model", (100, 100, -100))
    # 第 1 次偏短：RAMP_K 在终点前已经在减速，PI 积分低于稳态，实际滑得比 τ·v 少；学 1 次就对上
    check("模型预测（model）2 次后误差 ≤ 8mm", all(abs(e) <= 8 for e in errs[1:]), " ".join(f"{e:+.1f}" for e in errs))
    c.close()
    FwSerial.model = M(voltage=22.5)
    ct = devices.WheeltecChassis("x", 115200, stop_model="tau")
    ct.coast_tau = learned[0]
    et = abs(ct.drive(100)[0]) - 100
    ct.close()
    FwSerial.model = M(voltage=22.5)
    cm = devices.WheeltecChassis("x", 115200, stop_model="model")
    cm.pi_tau = learned[1]
    em = abs(cm.drive(100)[0]) - 100
    cm.close()
    check("电池掉到 22.5V：model 按电压修正，误差 ≤ 6mm", abs(em) <= 6, f"model {em:+.1f}mm / tau {et:+.1f}mm")

    FwSerial.model = M()
    c = devices.WheeltecChassis("x", 115200, speed_mm_s=100, min_radius_mm=600)
    _, d1 = c.drive(200, 10.0)
    _, d2 = c.drive(-200, 10.0)
    check("边走边转：前进/倒车都右转约 10°（wz 符号和固件一致）", abs(d1 - 10) < 2 and abs(d2 - 10) < 2,
          f"{d1:+.1f}° {d2:+.1f}°")
    c._set(100, 0)                       # 模拟调用方卡死：只设了一次速度就不管了
    time.sleep(0.8)
    check("指令过期自动停车（固件没有超时，驱动兜底）", c.ttl_tripped >= 1 and FwSerial.model.move_x == 0.0)
    c.close()
    check("close() 最后写的是停车帧", c.ser.writes[-1] == devices.WheeltecChassis.frame(0, 0))

    FwSerial.model = M(steer_offset_rad=math.radians(-1.5))   # 转向没回正，直走往右偏
    c = devices.WheeltecChassis("x", 115200)
    c.drive(200)
    drift = c.last_release["dyaw_gyro_deg"]
    trim = drift / (abs(c.last_release["moved"]) / 1000.0)
    c.yaw_trim = math.radians(trim) / 1000.0
    c.drive(200)
    f1 = c.last_release["dyaw_gyro_deg"]
    c.drive(-200)
    b1 = c.last_release["dyaw_gyro_deg"]
    c.close()
    check("航向补偿 yaw_trim：直走偏航（陀螺仪）明显减小，倒车也不反向", abs(f1) < 0.35 * abs(drift) and abs(b1) < 0.35 * abs(drift),
          f"补偿前 {drift:+.2f}° → 前进 {f1:+.2f}° / 倒车 {b1:+.2f}°（trim={trim:+.2f}°/m）")


# ------------------------------------------------------------ calib.json 覆盖
def test_calib_override():
    print("calib.json 覆盖 config")
    import config
    # 用临时文件，不碰这台车真实的 calib.json；测试期间打开覆盖功能
    path = os.path.join(tempfile.mkdtemp(), "calib.json")
    os.environ["GRASP_CALIB_FILE"] = path
    os.environ.pop("GRASP_IGNORE_CALIB", None)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"GOAL": {"x_deg": 1, "y": 2, "d": 150}, "POSES": {"observe": {"2": 1400}},
                       "AXES_J0": {"y": 9.5}, "BASE_DIR": -1, "NOT_ALLOWED": 1}, f)
        importlib.reload(config)
        check("白名单参数被覆盖", config.GOAL["d"] == 150 and config.BASE_DIR == -1 and config.AXES["y"]["j0"] == 9.5)
        check("姿态只覆盖写了的舵机字典", config.POSES["observe"] == {2: 1400})
        check("不在白名单的忽略", not hasattr(config, "NOT_ALLOWED"))
        os.remove(path)
        importlib.reload(config)
        check("删掉 calib.json 恢复默认", config.GOAL["d"] == 160.0)
    finally:
        if os.path.exists(path):
            os.remove(path)
        os.environ.pop("GRASP_CALIB_FILE", None)
        os.environ["GRASP_IGNORE_CALIB"] = "1"
        importlib.reload(config)


# ------------------------------------------------------------ 单步模式 / 手柄接管
def test_step_and_guard():
    print("单步模式、手柄接管")
    import builtins
    import main
    calls = []
    dev = types.SimpleNamespace(move_to=lambda cm: calls.append(cm) or cm, position=lambda: 1.0)
    p = main.StepProxy(dev, "升降轴")
    old = builtins.input
    try:
        builtins.input = lambda prompt="": ""
        check("回车 → 执行", p.move_to(3.0) == 3.0 and calls == [3.0])
        check("非运动方法不询问", p.position() == 1.0)
        builtins.input = lambda prompt="": "q"
        try:
            p.move_to(4.0)
            check("q → 停止", False)
        except KeyboardInterrupt:
            check("q → 停止", calls == [3.0])
    finally:
        builtins.input = old

    import config as cfg
    from sim import SIM_POSES, make_sim
    from task import GraspTask
    saved = cfg.POSES
    cfg.POSES = SIM_POSES
    try:
        world, clock, cam, lift, arm, ch = make_sim(cfg, 1, 1.0, cfg.R_CAM_MM)
        guard = types.SimpleNamespace(active=True)
        t = GraspTask(cfg, clock, cam, lift, arm, ch, verbose=False, guard=guard)
        try:
            t.run_once()
            check("手柄接管时立即停止任务", False)
        except devices.ManualOverride:
            check("手柄接管时立即停止任务", True)
    finally:
        cfg.POSES = saved


# ------------------------------------------------------------ 几何前馈
def test_geometry():
    print("几何前馈（底座角度）")
    import config as cfg
    from task import GraspTask
    t = GraspTask.__new__(GraspTask)
    t.cfg = cfg
    g0 = dict(cfg.GOAL)
    cfg.GOAL["x_deg"] = 0.0
    try:
        m = devices.Measurement(0, 0, 300, 5)
        check("物块正前方、车不动 → 底座不动", abs(t.base_after_move(m, 0, 0, 0)) < 1e-6)
        check("物块在右 → 底座往右（正）", t.base_after_move(devices.Measurement(50, 0, 300, 5), 0, 0, 0) > 0)
        # 车头左转 10°（右转 -10）、不平移：底座要往右补约 10°
        b = t.base_after_move(m, 0, 0, -10)
        check("车头左转 10° → 底座右补约 10°", 8 < b < 12, f"{b:.2f}")
        # 物块在左前方，车往前走：物块相对车更偏左 → 底座要往左多转
        m2 = devices.Measurement(0, 0, 300, 5)
        b2 = t.base_after_move(m2, -40, 150, 0)
        check("侧方物块 + 前进 → 底座往同侧多转（平移视差）", b2 < -40, f"{b2:.2f}")
    finally:
        cfg.GOAL.update(g0)

    # 没有底盘时目标按深度插值（2026-11-03 真机标定：近 156mm Y=70，远 216mm Y=43）
    saved = (cfg.USE_CHASSIS, getattr(cfg, "GOAL_FAR", None), dict(cfg.GOAL))
    try:
        cfg.USE_CHASSIS = False
        cfg.GOAL.update({"x_deg": -7.33, "y": 70.0, "d": 155.5})
        cfg.GOAL_FAR = {"x_deg": -7.4, "y": 43.0, "d": 215.5}
        g = t.goal_for(devices.Measurement(0, 0, 185.5, 5))
        check("无底盘：目标 Y 按深度插值（186mm → Y≈56.5）", abs(g["y"] - 56.5) < 0.01 and abs(g["d"] - 185.5) < 1e-9,
              f"{g}")
        g = t.goal_for(devices.Measurement(0, 0, 300, 5))
        check("无底盘：超出远端时按远端目标", abs(g["y"] - 43.0) < 1e-9)
        cfg.USE_CHASSIS = True
        check("有底盘：用固定 GOAL", t.goal_for(devices.Measurement(0, 0, 185.5, 5)) is cfg.GOAL)
    finally:
        cfg.USE_CHASSIS, cfg.GOAL_FAR = saved[0], saved[1]
        cfg.GOAL.clear()
        cfg.GOAL.update(saved[2])


# ------------------------------------------------------------ 仿真真实度（任务 02 新增的因素）
def test_sim_factors():
    print("仿真真实度因素（任务 02）")
    import config as cfg
    import sim
    from task import GraspTask
    importlib.reload(cfg)
    cfg.POSES = sim.SIM_POSES
    car = {"box_bias_px": 0.0, "coast_tau": 0.45}
    try:
        # 2 底盘：一阶滞后假控制器 + 真机 drive 逻辑
        w, clock, cam, lift, arm, ch = sim.make_sim(cfg, 3, 1.0, 150, dict(car))
        check("仿真底盘走真机 WheeltecChassis.drive", isinstance(ch, devices.WheeltecChassis))
        ch.cmd = (100, 0)
        for _ in range(300):
            ch._plant(0.01)
        x0 = w.robot[0]
        ch.cmd = (0, 0)
        for _ in range(500):
            ch._plant(0.01)
        coast = w.robot[0] - x0
        check("100mm/s 松油门滑行约 45mm（README 实测）", 40 < coast < 50, f"{coast:.1f}mm")
        errs = [ch.drive(mm)[0] - mm for mm in (100, 100, -100, 150)]
        check("仿真底盘：走 2 次后里程停车误差 ≤ 8mm", all(abs(e) <= 8 for e in errs[2:]),
              " ".join(f"{e:+.1f}" for e in errs))

        # 1 K230 跟踪器：小动作后第一帧带滞后，之后收敛
        cfg.USE_CHASSIS = False
        w, clock, cam, lift, arm, ch = sim.make_sim(cfg, 3, 1.0, 150, dict(car))
        w.block = [80 + 150 + 300, 0, w.CAM_Z0]
        arm.move({**cfg.POSES["observe"], 6: 1500, 1: 500}, 1000)
        clock.sleep(2)
        cam._advance(clock.now())
        n0 = len(cam.sent)
        arm.move({6: 1533}, 150)              # 底座右转约 3°
        clock.sleep(1.5)
        cam._advance(clock.now())
        xt = w.true_view()[0]
        xs = [f[1] for f in cam.sent[n0:]]
        check("跟踪器：动作后第一帧是滞后的旧位置", bool(xs) and abs(xs[0] - xt) > 0.3 * abs(xt),
              f"第一帧 {xs[0]:.1f}，真实 {xt:.1f}" if xs else "没有帧")
        check("跟踪器：1 秒后收敛", bool(xs) and abs(xs[-1] - xt) < 1.5, f"{xs[-1]:.1f}" if xs else "")

        # 3 舵机死区 / 回差
        b0 = w.base_deg
        arm.move({6: 1536}, 150)              # 3us：在死区里
        check("舵机死区：3us 的指令不动", w.base_deg == b0)
        arm.move({6: 1600}, 300)
        up = arm.base_out
        arm.move({6: 1700}, 300)
        arm.move({6: 1600}, 300)
        check("舵机回差：从两边到同一位置差约 10us",
              abs(abs(arm.base_out - up) - cfg.SIM_SERVO_BACKLASH_US) < 1e-6, f"{up:.0f} / {arm.base_out:.0f}")

        # 4 伸出时碰歪物块
        cfg.SIM_PUSH_P = 1.0
        w, clock, cam, lift, arm, ch = sim.make_sim(cfg, 3, 1.0, 150, dict(car))
        w.block = [80 + 150 + 170, -170 * math.tan(math.radians(1.5)), w.CAM_Z0]   # 偏右 1.5°（抓取容差的一半）
        b = list(w.block)
        arm.move(cfg.POSES["reach_near"], 1000)
        moved = math.hypot(w.block[0] - b[0], w.block[1] - b[1])
        check("小偏差伸出：物块被推开 10~30mm", w.pushes == 1 and 10 <= moved <= 30, f"{moved:.1f}mm")
        cfg.SIM_PUSH_P = 0.3

        # 5 深度非线性：固定的框边长偏差 → 相对误差随距离线性增长（绝对误差 ∝ 距离²）
        w.car = {"box_bias_px": 1.0, "coast_tau": 0.45}
        e160 = w.measured_clean((0, 0, 160))[2] - 160
        e900 = w.measured_clean((0, 0, 900))[2] - 900
        check("尺寸法测深：900mm 处误差约是 160mm 处的 (900/160)² 倍", 25 < e900 / e160 < 35,
              f"{e160:+.2f} / {e900:+.1f}mm")

        # 6 USB 掉线：走真机 Arm.move 的重连逻辑
        w, clock, cam, lift, arm, ch = sim.make_sim(cfg, 3, 1.0, 150, dict(car))
        arm.down_until = clock.now() + 1.0
        arm.usb_gen += 1
        t0 = clock.now()
        arm.move({6: 1700}, 300)
        check("USB 掉线 1s：Arm 等它回来、重连后指令生效",
              arm.reconnects == 1 and abs(w.base_deg - 200 / 11.1) < 1 and clock.now() - t0 >= 1.0,
              f"用时 {clock.now() - t0:.1f}s")

        # 7 误识别：SEARCH_CONFIRM 能过滤只出现一次的假目标
        seq = [None, None]
        fake_cam = types.SimpleNamespace(measure=lambda *a: seq.pop(0) if seq else None)
        t = GraspTask(cfg, clock, fake_cam, lift, arm, ch, verbose=False)
        check("SEARCH_CONFIRM：复测看不到的目标不采信", t.confirm_target(devices.Measurement(50, 0, 400, 5)) is None)
        seq[:] = [devices.Measurement(52, 1, 405, 5)]
        check("SEARCH_CONFIRM：复测一致的目标采信", t.confirm_target(devices.Measurement(50, 0, 400, 5)) is not None)
        seq[:] = [devices.Measurement(-80, 30, 250, 5), None]
        check("SEARCH_CONFIRM：复测位置对不上的不采信", t.confirm_target(devices.Measurement(50, 0, 400, 5)) is None)
    finally:
        importlib.reload(cfg)


# ------------------------------------------------------------ 仿真回归
def test_sim():
    print("仿真回归（整套流程）")
    import io
    import contextlib
    import config as cfg
    import main
    saved = (cfg.POSES, cfg.USE_CHASSIS)
    try:
        fixes = {"ARM_USB_ROBUST": True, "SEARCH_CONFIRM": True, "BASE_LIMIT_BACKUP_MM": 150}
        for n, seed, nc, need, extra in ((40, 31, False, 0.85, {}), (30, 33, True, 0.85, {}),
                                         (40, 31, False, 0.85, fixes)):
            importlib.reload(cfg)
            for k, v in extra.items():
                setattr(cfg, k, v)
            main.cfg = cfg
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rate = main.run_sim(n, seed, True, nc)
            line = [l for l in buf.getvalue().splitlines() if l.startswith("全部")][0]
            check(f"{'无' if nc else '有'}底盘 {n} 次{'（打开任务 02 的改进开关）' if extra else ''}，"
                  f"真实抓到 ≥ {need:.0%}", rate >= need, line.split(":", 1)[1].strip())
    finally:
        importlib.reload(cfg)
        main.cfg = cfg


if __name__ == "__main__":
    quick = len(sys.argv) > 1 and sys.argv[1] == "quick"
    real_serial = devices.serial
    for t in (test_arm, test_k230, test_lift, test_chassis, test_chassis_fw, test_calib_override,
              test_step_and_guard, test_geometry, test_sim_factors):
        try:
            t()
        except Exception as e:
            check(f"{t.__name__} 崩溃", False, repr(e))
        finally:
            devices.serial = real_serial
    if not quick:
        test_sim()
    print(f"\n通过 {len(PASS)} 项，失败 {len(FAIL)} 项" + ("" if not FAIL else "：" + "；".join(FAIL)))
    sys.exit(1 if FAIL else 0)
