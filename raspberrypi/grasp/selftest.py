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


# ------------------------------------------------------------ 假目标（机械臂自己）/ 升降轴到头
def test_ghost():
    print("识别到机械臂自己 / 升降轴到头")
    import config as cfg
    from task import GraspTask

    class Clock:
        t = 0.0
        def now(self): return self.t
        def sleep(self, s): self.t += max(0.0, s)

    class GhostCam:                     # 读数永远不变：K230 盯着爪子
        def __init__(self, x, y, d): self.m = devices.Measurement(x, y, d, 5)
        def measure(self, since, n, timeout): return self.m

    class Lift:
        def __init__(self, pos): self.pos = pos; self.moves = []
        def start_move_by(self, cm): self.moves.append(cm); self.pos = max(0.0, min(44.0, self.pos + cm))
        def wait_move(self, timeout=30): return self.pos
        def move_by(self, cm, timeout=30): self.start_move_by(cm); return self.pos
        def move_to(self, cm, timeout=30): self.pos = cm; return cm

    class Arm:
        def move(self, pulses, ms): pass

    saved = cfg.USE_CHASSIS
    try:
        cfg.USE_CHASSIS = False
        g = cfg.GOAL
        # y 偏 +30mm、x 偏 +6°，读数不变 → 应该两三步内判"ghost"，升降轴不会一路降到底
        lift = Lift(20.0)
        t = GraspTask(cfg, Clock(), GhostCam(math.tan(math.radians(g["x_deg"] + 6)) * g["d"], g["y"] + 30, g["d"]),
                      lift, Arm(), None, verbose=False, learn=False)
        ok, m, n = t.align(t.cam.m)
        check("读数不随动作变化 → 判定识别到机械臂自己并停止", not ok and t._abort == "ghost" and n <= 3,
              f"步数 {n}，升降轴共动了 {sum(lift.moves):+.1f}cm")
        check("停下前升降轴没有一路降到底", lift.pos > 10, f"{lift.pos:.1f}cm")
        # 升降轴已经在最低点、还要往下 → 判"lift_limit"
        lift = Lift(0.0)
        t = GraspTask(cfg, Clock(), GhostCam(0, g["y"] + 30, g["d"]), lift, Arm(), None, verbose=False, learn=False)
        ok, m, n = t.align(t.cam.m)
        check("升降轴到最低点还要往下 → 判定够不着", not ok and t._abort == "lift_limit" and not lift.moves)
        # 升降轴在最低点、剩余误差在容差内（地上的物块）→ 按已对准处理，去抓
        lift = Lift(0.0)
        t = GraspTask(cfg, Clock(), GhostCam(math.tan(math.radians(g["x_deg"])) * g["d"], g["y"] + 10, g["d"]),
                      lift, Arm(), None, verbose=False, learn=False)
        ok, m, n = t.align(t.cam.m)
        check("升降轴在最低点、剩余 10mm（容差 15mm 内）→ 按已对准处理", ok and not lift.moves, f"ok={ok} abort={t._abort}")
    finally:
        cfg.USE_CHASSIS = saved


# ------------------------------------------------------------ ghost 指纹（任务 10）
def test_ghost_fingerprint():
    print("ghost 指纹（GHOST_FINGERPRINT）")
    import json
    import shutil
    import tempfile
    import config as cfg
    from task import GraspTask

    class Clock:
        t = 0.0
        def now(self): return self.t
        def sleep(self, s): self.t += max(0.0, s)

    class Cam:                          # 依次返回给定的读数；用完了一直返回最后一个
        def __init__(self, *ms): self.ms = list(ms)
        def measure(self, since, n, timeout):
            return self.ms.pop(0) if len(self.ms) > 1 else self.ms[0]

    class Lift:
        pos = 20.0
        def start_move_by(self, cm): self.pos += cm
        def wait_move(self, timeout=30): return self.pos
        def move_to(self, cm, timeout=30): self.pos = cm; return cm

    class Arm:
        def move(self, pulses, ms): pass

    saved = (cfg.USE_CHASSIS, cfg.GHOST_FINGERPRINT)
    tmp = tempfile.mkdtemp(prefix="ghost_fp_")
    try:
        cfg.USE_CHASSIS = False
        g = cfg.GOAL
        x_of = lambda deg, d: math.tan(math.radians(deg)) * d
        ghost = devices.Measurement(x_of(g["x_deg"] + 6, g["d"]), g["y"] + 30, g["d"], 5)
        path = os.path.join(tmp, cfg.GHOST_FP_FILE)

        def task(*ms):
            return GraspTask(cfg, Clock(), Cam(*ms), Lift(), Arm(), None, log_dir=tmp, verbose=False, learn=False)

        # 关着（默认）：判定 ghost 也不写文件、不过滤
        cfg.GHOST_FINGERPRINT = False
        check("默认关闭", saved[1] is False)
        t = task(ghost)
        ok, m, n = t.align(ghost)
        check("关闭时：判 ghost 但不记指纹", t._abort == "ghost" and not os.path.exists(path))
        check("关闭时：ghost 读数照常返回", t.measure() is ghost)

        # 打开：判定 ghost -> 记到 ghosts.json
        cfg.GHOST_FINGERPRINT = True
        t = task(ghost)
        ok, m, n = t.align(ghost)
        data = json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}
        gs = data.get("ghosts", [])
        check("打开时：判 ghost 后记下指纹（x_deg、y、d）", t._abort == "ghost" and len(gs) == 1
              and abs(gs[0]["x_deg"] - ghost.x_deg) < 0.05 and abs(gs[0]["y"] - ghost.y) < 0.1
              and abs(gs[0]["d"] - ghost.d) < 0.1, str(gs))

        # 新的一次运行（重新加载文件）：接近指纹的读数当作没看到，并提示
        near = devices.Measurement(x_of(ghost.x_deg + 1.0, ghost.d + 10), ghost.y - 4, ghost.d + 10, 5)
        far = devices.Measurement(x_of(g["x_deg"], g["d"]), g["y"], g["d"], 5)
        buf = []
        t = task(ghost, near, far)
        t.verbose = True
        import builtins
        real_print = builtins.print
        builtins.print = lambda *a, **k: buf.append(" ".join(str(x) for x in a))
        try:
            r1, r2, r3 = t.measure(), t.measure(), t.measure()
        finally:
            builtins.print = real_print
        check("重新加载后：同样的读数当作没看到", r1 is None)
        check("容差内的读数也当作没看到", r2 is None)
        check("离指纹远的读数（真物块）照常返回", r3 is far)
        check("提示“看到的是之前记录的假目标”", any("之前记录的假目标" in b for b in buf), buf[0] if buf else "")
        # 只有 ghost 时：搜索找不到（不会再锁着假目标去对准）
        t = task(ghost)
        check("只看得到假目标时搜索返回 None", t.search() is None)

        # 再判一次接近的 ghost：合并到同一个指纹、命中次数 +1
        t = task(near)
        t.record_ghost(near)
        gs = json.load(open(path, encoding="utf-8"))["ghosts"]
        check("接近的 ghost 合并到已有指纹", len(gs) == 1 and gs[0]["hits"] == 2, str(gs))
        t.record_ghost(far)
        gs = json.load(open(path, encoding="utf-8"))["ghosts"]
        check("不同位置的 ghost 另记一个", len(gs) == 2)

        # GHOST_FP_MIN_HITS：命中次数不够时不过滤
        saved_min = cfg.GHOST_FP_MIN_HITS
        try:
            cfg.GHOST_FP_MIN_HITS = 3
            t = task(ghost)
            check("命中次数 < GHOST_FP_MIN_HITS 时不过滤", t.measure() is ghost)
        finally:
            cfg.GHOST_FP_MIN_HITS = saved_min

        # 文件坏了：不崩溃，当作没有指纹
        with open(path, "w", encoding="utf-8") as f:
            f.write("{坏了")
        with contextlib_quiet():
            t = task(ghost)
        check("ghosts.json 损坏时不崩溃、不过滤", t.ghosts == [] and t.measure() is ghost)
        check("新参数在 calib.json 白名单里", {"GHOST_FINGERPRINT", "GHOST_FP_TOL", "GHOST_FP_MIN_HITS",
                                               "GHOST_FP_MAX"} <= cfg.CALIB_KEYS)
    finally:
        cfg.USE_CHASSIS, cfg.GHOST_FINGERPRINT = saved
        shutil.rmtree(tmp, ignore_errors=True)


def contextlib_quiet():
    import contextlib
    import io
    return contextlib.redirect_stdout(io.StringIO())


# ------------------------------------------------------------ 试验记录工具 bench.py
def test_bench():
    print("试验记录工具 bench.py（仿真设备 + 假的键盘输入）")
    import io
    import contextlib
    import config as cfg
    import bench
    saved = (cfg.POSES, cfg.USE_CHASSIS)
    tmp = tempfile.mkdtemp(prefix="bench_")
    path = os.path.join(tmp, "bench.csv")
    try:
        cfg.USE_CHASSIS = False
        check("位置说明：④ 在左边、底座约 −10°（右转为正）",
              "往左" in bench.pos_text(4) and "-10°" in bench.pos_text(4) and "18cm" in bench.pos_text(4),
              bench.pos_text(4))
        check("--positions 解析（数字 / 圆圈编号）", bench.parse_positions("2,5,8") == [2, 5, 8]
              and bench.parse_positions("⑤ 9") == [5, 9])
        check("偏差方向解析", bench.parse_dir("右前") == "右前" and bench.parse_dir("rf") == "右前"
              and bench.parse_dir("") == "" and bench.parse_dir("左右") is None and bench.parse_dir("x9") is None)
        check("计划：先每个位置一遍再第二遍；--resume 跳过已测的",
              bench.make_plan([1, 2], 2) == [(1, 1), (2, 1), (1, 2), (2, 2)]
              and bench.make_plan([1, 2], 2, {1: 2, 2: 1}) == [(2, 2)])

        rig = bench.SimRig(seed=3)
        rig.open()
        state = {"redo": 1, "asked": []}

        def ask(prompt):
            state["asked"].append(prompt)
            if "回车开始" in prompt:
                return ""
            if "真的抓到" in prompt:
                if state["redo"]:              # 第一次回答"这次不算，重测"
                    state["redo"] -= 1
                    return "r"
                return "y" if rig.truth else "n"
            if "偏哪边" in prompt or "居中" in prompt:
                return "r"
            if "几毫米" in prompt:
                return "4mm"
            return "测试备注"

        with contextlib.redirect_stdout(io.StringIO()):
            n = bench.run_bench(rig, [1, 9], 1, ask=ask, path=path)
        rows = bench.load_rows(path)
        check("跑 2 个位置（其中 1 次选了重测）→ 记 2 条", n == 2 and [r["pos"] for r in rows] == ["1", "9"]
              and rig.n == 3, f"运行 {rig.n} 次")
        ok_cols = all(r["base_deg"] and r["d"] and r["t"] and r["trim_deg"] and r["steps"] for r in rows
                      if r["prog_result"] == "success")
        check("记录里有对准位置、底座角度、深度插值 t、微调、步数", ok_cols, str({k: rows[0][k] for k in
                                                                ("prog_result", "x_deg", "d", "base_deg", "t")}))
        bd = [float(r["base_deg"]) for r in rows if r["base_deg"]]
        check("左边位置底座角度为负、右边为正（和 config 约定一致）", len(bd) == 2 and bd[0] < -5 and bd[1] > 5, str(bd))
        check("偏差和备注记下来了", rows[0]["dev_dir"] == "右" and float(rows[0]["dev_mm"]) == 4.0
              and rows[0]["note"] == "测试备注")

        # 中途 Ctrl+C：急停、不记这一次；--resume 从没测的接着测，并且是同一轮
        class Interrupt(bench.SimRig):
            def run_once(self, p=None):
                if self.n >= 1:
                    self.n += 1
                    raise KeyboardInterrupt
                return super().run_once(p)
        os.remove(path)
        rig2 = Interrupt(seed=4)
        rig2.open()
        rig = rig2
        state["redo"] = 0
        with contextlib.redirect_stdout(io.StringIO()):
            n = bench.run_bench(rig2, [2, 5, 8], 1, ask=ask, path=path)
        check("运行中 Ctrl+C → 升降轴急停、已完成的已记录、这一次不记", n == 1 and rig2.emergencies == 1
              and len(bench.load_rows(path)) == 1)
        rig = bench.SimRig(seed=5)
        rig.open()
        with contextlib.redirect_stdout(io.StringIO()):
            n = bench.run_bench(rig, [2, 5, 8], 1, resume=True, ask=ask, path=path)
        rows = bench.load_rows(path)
        check("--resume 只测剩下的位置、同一轮", n == 2 and [r["pos"] for r in rows] == ["2", "5", "8"]
              and len({r["round"] for r in rows}) == 1)

        def boom(prompt):
            raise KeyboardInterrupt
        with contextlib.redirect_stdout(io.StringIO()):
            n = bench.run_bench(bench.SimRig(seed=6), [1], 1, ask=boom, path=path)
        check("等待输入时 Ctrl+C → 正常退出", n == 0)

        # 统计和建议：用构造的记录
        def rows_of(spec, rnd="R1"):
            out = []
            for p, prog, truth, dd, mm in spec:
                out.append({"round": rnd, "pos": str(p), "prog_result": prog, "truth": truth, "dev_dir": dd,
                            "dev_mm": "" if mm is None else str(mm), "d": "180", "time_s": "20", "trim_deg": "0.0",
                            "log": "run_x.log"})
            return out

        def tips_of(rs):
            with contextlib.redirect_stdout(io.StringIO()):
                p2 = os.path.join(tmp, "r.csv")
                if os.path.exists(p2):
                    os.remove(p2)
                for r in rs:
                    bench.append_row(r, p2)
                buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                t = bench.report(p2)
            return t, buf.getvalue()

        spec = [(p, "grasp_failed", "n", "右", 4) for p in (1, 2, 4, 5, 6, 8)] + \
               [(p, "success", "y", "中", 0) for p in (3, 7, 9)] + [(5, "success", "n", "右", 5)]
        t, txt = tips_of(rows_of(spec))
        check("多数位置偏右约 4mm → 建议 calib.py trim 4", any("calib.py trim 4" in x for x in t), "；".join(t))
        check("误报（程序说抓到、实际没有）计数 + 提示看日志", "误报（程序说抓到、实际没有）1 次" in txt
              and any("run_x.log" in x for x in t))
        check("混淆表", "程序结果 vs 真实结果" in txt and "grasp_failed" in txt)
        spec = [(p, "grasp_failed", "n", "右", 5) for p in (3, 6, 9)] + \
               [(p, "success", "y", "中", 0) for p in (1, 2, 4, 5, 7, 8)]
        t, _ = tips_of(rows_of(spec))
        check("只有右侧位置偏右 → 提示底座回差、任务 09，不建议 trim",
              any("只有右侧" in x and "任务 09" in x for x in t) and not any("calib.py trim" in x for x in t),
              "；".join(t))
        spec = [(p, "grasp_failed", "n", "高", 6) for p in (7, 8, 9)] + \
               [(p, "success", "y", "中", 0) for p in range(1, 7)]
        t, _ = tips_of(rows_of(spec))
        check("只有远处偏高 → 建议 teach far 重标", any("teach far" in x for x in t), "；".join(t))
        spec = [(p, "grasp_failed", "n", "", None) for p in (4, 5, 6) for _ in range(2)] + \
               [(p, "success", "y", "中", 0) for p in (1, 2, 3, 7, 8, 9) for _ in range(2)]
        t, _ = tips_of(rows_of(spec))
        check("中间深度没抓到、两端抓到 → 建议 reach_mid", any("reach_mid" in x for x in t), "；".join(t))
        rs = rows_of([(5, "success", "y", "中", 0)], "R1") + rows_of([(5, "grasp_failed", "n", "", None)], "R2")
        _, txt = tips_of(rs)
        check("report 默认只统计最新一轮", "第 R2 轮" in txt and "共 1 次：真实抓到 0 次" in txt)
    finally:
        cfg.POSES, cfg.USE_CHASSIS = saved


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
              test_step_and_guard, test_geometry, test_ghost, test_ghost_fingerprint, test_sim_factors,
              test_bench):
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
