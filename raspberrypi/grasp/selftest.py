#!/usr/bin/env python3
"""
离线自测：不接任何硬件，用假的下位机把每个驱动和整套流程测一遍。
改完代码先跑它，全部 ✅ 再上真机。

  python3 selftest.py          全部测试（约 1 分钟）
  python3 selftest.py quick    跳过耗时的仿真回归（任务 18 的底盘危险动作检查照样跑）
  python3 selftest.py research 全部测试 + 研究原型（research/，任务 11）的单元测试
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


# ------------------------------------------------------------ 任务 09：抓取精度
def test_accuracy():
    print("抓取精度（任务 09：消回差、最小步长、左右试探、学习微调、三点插值）")
    import config as cfg
    import calib
    import sim
    from task import GraspTask

    class Rec:                                  # 记录发给机械臂的每一帧
        def __init__(self): self.sent = []
        def move(self, pulses, ms): self.sent.append((dict(pulses), ms))

    class Clock:
        t = 0.0
        def now(self): return self.t
        def sleep(self, s): self.t += max(0.0, s)

    keys = ("BASE_BACKLASH_DEG", "BASE_APPROACH_DIR", "BASE_MIN_STEP_DEG", "GRASP_PROBE_MM", "GRASP_PROBE_FIRST",
            "TRIM_LEARN", "TRIM_LEARN_RATE", "TRIM_MAX_DEG", "TRIM_LEARN_EDGES", "REACH_MID_D", "GOAL_MID")
    saved = {k: getattr(cfg, k) for k in keys}
    saved_poses, saved_goal = dict(cfg.POSES), dict(cfg.GOAL)
    tmp = tempfile.mkdtemp(prefix="acc_")
    try:
        check("新参数都在 calib.json 白名单里", all(k in cfg.CALIB_KEYS for k in keys))
        check("默认全部关闭", cfg.BASE_BACKLASH_DEG == 0 and cfg.BASE_MIN_STEP_DEG == 0 and cfg.GRASP_PROBE_MM == 0
              and not cfg.TRIM_LEARN and cfg.REACH_MID_D is None and cfg.GOAL_MID is None
              and "reach_mid" not in cfg.POSES)

        # ---- 消回差
        arm = Rec()
        t = GraspTask(cfg, Clock(), None, None, arm, None, verbose=False, learn=False)
        for deg in (5, 2, 2):
            t.set_base(deg)
        check("关闭时：每次转底座只发一帧（和原来一样）", len(arm.sent) == 3)
        cfg.BASE_BACKLASH_DEG = 2.0
        arm = Rec()
        t = GraspTask(cfg, Clock(), None, None, arm, None, verbose=False, learn=False)
        t.set_base(0)                                   # 不知道之前从哪边来：先过头再回来
        n0 = len(arm.sent)
        t.set_base(5)                                   # 右转 = 逼近方向：直接走
        n1 = len(arm.sent)
        t.set_base(2)                                   # 左转：先转到 0°（多 2°）再右转回 2°
        last = [p[6] for p, _ in arm.sent[n1:]]
        n2 = len(arm.sent)
        t.set_base(2)                                   # 原地、上一段是右转：不动
        check("消回差：位置未知时先过头再回来", n0 == 2 and arm.sent[0][0][6] < arm.sent[1][0][6])
        check("消回差：顺着逼近方向转只发一帧", n1 - n0 == 1)
        p2 = t.base_pulse(2)
        check("消回差：反方向转先多转 2° 再回来", last == [p2 - 22, p2], str(last))
        check("消回差：只动底座时回程用时按 2° 算", arm.sent[n1 + 1][1] == 150, str(arm.sent[n1 + 1][1]))
        check("消回差：原地再发不多转", len(arm.sent) - n2 == 1)
        cfg.BASE_APPROACH_DIR = -1
        arm = Rec()
        t = GraspTask(cfg, Clock(), None, None, arm, None, verbose=False, learn=False)
        t.set_base(0)
        t.set_base(5)
        last = [p[6] for p, _ in arm.sent[2:]]
        check("BASE_APPROACH_DIR=-1：右转时先多转再左转回来", last == [t.base_pulse(5) + 22, t.base_pulse(5)], str(last))
        cfg.BASE_APPROACH_DIR, cfg.BASE_BACKLASH_DEG = 1, 0.0

        # ---- 最小步长
        cfg.BASE_MIN_STEP_DEG = 2.5
        t = GraspTask(cfg, Clock(), None, None, Rec(), None, verbose=False, learn=False)
        check("最小步长：误差 +1.6°、算出 +1.4° → 放大到 +2.5°", abs(t._min_base_step(1.36, 1.6, 1.0) - 2.5) < 1e-9)
        t.axes["x_deg"].j = -0.6
        u = t._min_base_step(-2.0, -1.6, 1.0)
        check("最小步长：方向跟误差走（误差为负 → 左转）", abs(u + 2.5) < 1e-9, f"{u:.2f}")
        cfg.BASE_MIN_STEP_DEG = 5.0
        t.axes["x_deg"].j = -1.0
        u = t._min_base_step(1.36, 1.6, 1.0)
        check("最小步长：不会冲出容差另一边（最多到 −0.9×容差）", abs(u - (1.6 + 0.9 * cfg.TOL["x_deg"])) < 1e-9,
              f"{u:.2f}")
        check("最小步长：大步子不变", t._min_base_step(8.0, 9.0, 1.0) == 8.0)
        cfg.BASE_MIN_STEP_DEG = 0.0

        # ---- 左右试探
        cfg.GRASP_PROBE_MM, cfg.GRASP_PROBE_FIRST = 3.0, "left"
        t = GraspTask(cfg, Clock(), None, None, Rec(), None, verbose=False, learn=False)
        check("试探：第 1 次不试探，失败后依次 −p、+p、−2p", [t._probe_for(k) for k in range(4)] == [0, -3, 3, -6])
        cfg.GRASP_PROBE_FIRST = "alternate"
        t = GraspTask(cfg, Clock(), None, None, Rec(), None, verbose=False, learn=False)
        a = t._probe_for(1)
        t._flip_probe_first()
        check("试探：alternate 每个物块换一边先试", a == -3 and t._probe_for(1) == 3)

        # ---- 伸出时的底座 = 对准角度 + 固定微调 + 学到的 + 试探
        class Lift:
            pos = 10.0
            def move_by(self, cm, timeout=30): pass
        arm = Rec()
        cfg.TRIM_LEARN = True
        t = GraspTask(cfg, Clock(), None, Lift(), arm, None, log_dir=tmp, verbose=False)
        t.trim_state["segments"][t._trim_seg(180)]["trim_deg"] = -1.0
        t._probe_mm = 3.0
        cfg.VERIFY, v0 = "none", cfg.VERIFY
        t.grasp(devices.Measurement(0, 0, 180, 5))
        cfg.VERIFY = v0
        want = t.base_pulse(-1.0 + math.degrees(math.atan2(3.0, 180 + cfg.R_CAM_MM)))
        got = [p[6] for p, _ in arm.sent if 6 in p]
        check("伸出时底座 = 学到的 −1° + 试探 +3mm", got and all(g == want for g in got), f"{got} / {want}")
        check("last_grasp 记录了学到的微调和试探量", t.last_grasp.get("probe_mm") == 3.0
              and t.last_grasp.get("learned_trim_deg") == -1.0 and t.last_grasp["trim_deg"] == cfg.GRASP_BASE_TRIM_DEG)

        # ---- 学习
        cfg.TRIM_LEARN_RATE, cfg.TRIM_MAX_DEG = 0.5, 3.0
        t = GraspTask(cfg, Clock(), None, None, Rec(), None, log_dir=tmp, verbose=False)
        t.trim_state = t._new_trim_state()
        lo, hi = sorted((cfg.REACH_NEAR_D, cfg.REACH_FAR_D))
        check("默认分段：把近~远三等分", t._trim_edges() == [round(lo + (hi - lo) / 3, 1), round(lo + 2 * (hi - lo) / 3, 1)]
              and t._trim_seg(lo) == 0 and t._trim_seg(hi) == 2, str(t._trim_edges()))
        d = hi
        t._learn_trim(d, -3.0, -1.0, True)
        seg = t.trim_state["segments"][2]
        check("试探 −3mm 抓到 → 这一段微调往左挪 0.5×", abs(seg["trim_deg"] + 0.5) < 1e-9 and seg["probe_ok_left"] == 1)
        t._learn_trim(d, 0.0, 0.0, True)
        t._learn_trim(d, 0.0, 0.0, False)
        check("直接抓到 / 都没抓到 → 微调不变，只计数", abs(seg["trim_deg"] + 0.5) < 1e-9 and seg["direct_ok"] == 1
              and seg["failed"] == 1)
        check("别的深度段不受影响", t.trim_state["segments"][0]["trim_deg"] == 0.0)
        for _ in range(20):
            t._learn_trim(d, -3.0, -1.0, True)
        check("学到的微调限制在 ±TRIM_MAX_DEG", seg["trim_deg"] == -3.0)
        t2 = GraspTask(cfg, Clock(), None, None, Rec(), None, log_dir=tmp, verbose=False)
        check("学到的微调存文件、下次启动接着用", t2.learned_trim(d) == -3.0)
        t3 = GraspTask(cfg, Clock(), None, None, Rec(), None, log_dir=tmp, verbose=False, learn=False)
        t3._learn_trim(d, 3.0, 1.0, True)
        check("--no-learn：用学到的值但不再改", t3.learned_trim(d) == -3.0)
        cfg.TRIM_LEARN_EDGES = [170, 190]
        t4 = GraspTask(cfg, Clock(), None, None, Rec(), None, log_dir=tmp, verbose=False)
        check("深度分段改了 → 旧的学习结果作废、重新学", t4.learned_trim(d) == 0.0 and t4._trim_edges() == [170, 190])
        cfg.TRIM_LEARN_EDGES = None
        cfg.TRIM_LEARN = False
        check("TRIM_LEARN 关闭：学到的微调为 0",
              GraspTask(cfg, Clock(), None, None, Rec(), None, log_dir=tmp, verbose=False).learned_trim(d) == 0.0)
        show = io_capture(lambda: calib.show_learned_trim(os.path.join(tmp, cfg.TRIM_LEARN_FILE)))
        check("calib.py trim 显示学到的分段微调", "-3.00°" in show and "直接抓到" in show, show.splitlines()[-1])

        # ---- 三点插值
        cfg.POSES = dict(saved_poses)
        cfg.POSES["reach_near"] = {2: 1500, 3: 700, 4: 700, 5: 1900}
        cfg.POSES["reach_far"] = {2: 1500, 3: 800, 4: 800, 5: 1700}
        t = GraspTask(cfg, Clock(), None, None, Rec(), None, verbose=False, learn=False)
        dm = (cfg.REACH_NEAR_D + cfg.REACH_FAR_D) / 2
        two, tt = t.reach_pose(dm)
        check("没标 reach_mid：两点插值（原行为）", two == devices.lerp_pose(cfg.POSES["reach_near"],
                                                             cfg.POSES["reach_far"], 0.5) and abs(tt - 0.5) < 1e-9)
        cfg.POSES["reach_mid"] = {2: 1500, 3: 760, 4: 740, 5: 1850}
        cfg.REACH_MID_D = dm
        mid, _ = t.reach_pose(dm)
        check("标了 reach_mid：中间深度正好用 reach_mid", all(abs(mid[k] - cfg.POSES["reach_mid"][k]) < 1e-9 for k in mid))
        q, _ = t.reach_pose(cfg.REACH_NEAR_D + (dm - cfg.REACH_NEAR_D) / 2)
        check("近和中之间按近-中插值", abs(q[3] - 730) < 1e-9 and abs(q[5] - 1875) < 1e-9, str(q))
        q, _ = t.reach_pose(cfg.REACH_FAR_D + 50)
        check("超出远端：用 reach_far", q == {k: float(v) for k, v in cfg.POSES["reach_far"].items()} or
              all(abs(q[k] - cfg.POSES["reach_far"][k]) < 1e-9 for k in q))
        cfg.REACH_MID_D = cfg.REACH_FAR_D + 10
        check("REACH_MID_D 不在近远之间 → 不用它", t.reach_pose(dm)[0] == two)
        cfg.REACH_MID_D = None

        cfg.USE_CHASSIS, uc = False, cfg.USE_CHASSIS
        gf0 = cfg.GOAL_FAR
        cfg.GOAL.update({"x_deg": -7.0, "y": 70.0, "d": 156.0})
        cfg.GOAL_FAR = {"x_deg": -7.4, "y": 43.0, "d": 216.0}
        g_two = t.goal_for(devices.Measurement(0, 0, 186, 5))
        cfg.GOAL_MID = {"x_deg": -7.2, "y": 60.0, "d": 186.0}
        g_mid = t.goal_for(devices.Measurement(0, 0, 186, 5))
        g_q = t.goal_for(devices.Measurement(0, 0, 201, 5))
        cfg.GOAL_MID = None
        check("GOAL_MID：中间深度的目标 Y 用标定值（两点插值是 56.5）", abs(g_two["y"] - 56.5) < 1e-9
              and abs(g_mid["y"] - 60.0) < 1e-9 and abs(g_q["y"] - 51.5) < 1e-9, f"{g_mid['y']} / {g_q['y']}")
        cfg.USE_CHASSIS, cfg.GOAL_FAR = uc, gf0
        cfg.GOAL.clear()
        cfg.GOAL.update(saved_goal)

        # ---- calib.py teach mid 的初值
        cfg.POSES.pop("reach_mid", None)
        calib.cfg = cfg
        m0 = calib.ensure_mid_pose()
        check("teach mid：没标过时用近、远的中点当初值", m0 == {2: 1500, 3: 750, 4: 750, 5: 1800}, str(m0))

        # ---- 仿真：落点偏差模型
        cfg.POSES = dict(sim.SIM_POSES)
        cfg.USE_CHASSIS = False
        cfg.SIM_GRASP_BIAS_MM, cfg.SIM_GRIP_TOL_MM = (4.0, 4.0), 5.0
        car = sim.make_car(cfg, 3)
        cfg.SIM_GRASP_BIAS_MM, cfg.SIM_GRIP_TOL_MM = 0.0, 0.0
        car0 = sim.make_car(cfg, 3)
        check("仿真新因素不改变原来每台车的随机量", car["box_bias_px"] == car0["box_bias_px"]
              and car["coast_tau"] == car0["coast_tau"] and car["grasp_bias_mm"] == 4.0)
        w, clock, cam, lift, arm2, ch = sim.make_sim(cfg, 3, 1.0, 0.0, car0)
        check("仿真新因素默认关（按原来的 3° 判断）", not arm2._lateral_mode())
        cfg.SIM_GRIP_TOL_MM = 5.0
        w, clock, cam, lift, arm2, ch = sim.make_sim(cfg, 3, 1.0, 0.0, car)
        dd = 180.0
        w.block = [80 + dd, 0.0, w.CAM_Z0 + 10 * w.lift_cm + cfg.GOAL["y"]]
        lat, tol = arm2._lateral(0.0, dd)
        check("落点偏右 4mm：正对的物块在爪子左边 4mm", abs(lat + 4.0) < 1e-9 and tol == 5.0)
        arm2.move({**cfg.POSES["observe"], 6: 1500, 1: 500}, 500)
        arm2.move(cfg.POSES["reach_near"], 500)
        arm2.move({1: cfg.CLAW_CLOSE}, 300)
        check("落点偏差在容差内 → 抓得到（90% 概率那一档除外）", arm2.last_lateral_mm is not None
              and abs(arm2.last_lateral_mm + 4.0) < 0.5, f"{arm2.last_lateral_mm:.2f}")
        trim = -math.degrees(math.atan2(4.0, dd))
        cfg.SIM_SERVO_BACKLASH_US = 0              # 只验证偏差模型（回差会吃掉一部分补偿，见上面消回差）
        w, clock, cam, lift, arm2, ch = sim.make_sim(cfg, 3, 1.0, 0.0, car)
        w.block = [80 + dd, 0.0, w.CAM_Z0 + cfg.GOAL["y"]]
        arm2.move({**cfg.POSES["observe"], 6: int(round(1500 + trim * cfg.BASE_US_PER_DEG)), 1: 500}, 500)
        arm2.move(cfg.POSES["reach_near"], 500)
        arm2.move({1: cfg.CLAW_CLOSE}, 300)
        check("底座往左补 atan(4/180) → 落点偏差基本消掉", abs(arm2.last_lateral_mm) < 0.6, f"{arm2.last_lateral_mm:.2f}")
    finally:
        for k, v in saved.items():
            setattr(cfg, k, v)
        cfg.POSES = saved_poses
        cfg.GOAL.clear()
        cfg.GOAL.update(saved_goal)
        importlib.reload(cfg)


# ------------------------------------------------------------ 任务 13：提速 + 确认防误报 + 学习去偏
def test_speed_verify():
    print("提速与确认防误报（任务 13：整画面确认、去掉确认前 carry、动作时间缩放、同向才学 J）")
    import config as cfg
    import sim
    from task import GraspTask
    M = devices.Measurement

    class Clock:
        t = 0.0
        def now(self): return self.t
        def sleep(self, s): self.t += max(0.0, s)

    class Rec:
        def __init__(self): self.sent = []
        def move(self, pulses, ms): self.sent.append((dict(pulses), ms))

    class Lift:
        pos = 10.0
        def move_by(self, cm, timeout=30): pass

    class Cam:                                  # 依次返回 seq 里的测量（用完了就一直 None）
        def __init__(self, seq): self.seq = list(seq)
        def measure(self, since, n, timeout): return self.seq.pop(0) if self.seq else None

    keys = ("VERIFY_WHOLE_FRAME", "VERIFY_WHOLE_MIN_D_MM", "VERIFY_SKIP_CARRY", "ARM_MOVE_SCALE", "ARM_MS_PER_US",
            "ARM_MIN_MS", "LEARN_SAME_DIR_ONLY")
    saved = {k: getattr(cfg, k) for k in keys}
    saved2 = {k: getattr(cfg, k) for k in ("VERIFY", "R_CAM_MM", "USE_CHASSIS")}
    try:
        check("新参数都在 calib.json 白名单里", all(k in cfg.CALIB_KEYS for k in keys))
        check("默认全部关闭", not cfg.VERIFY_WHOLE_FRAME and not cfg.VERIFY_SKIP_CARRY and not cfg.ARM_MOVE_SCALE
              and not cfg.LEARN_SAME_DIR_ONLY and cfg.VERIFY_WHOLE_MIN_D_MM == 0 and cfg.SIM_CLAW_PUSH_P == 0)

        # ---- 动作时间缩放
        def grasp_frames(over, see=()):
            for k, v in over.items():
                setattr(cfg, k, v)
            arm = Rec()
            t = GraspTask(cfg, Clock(), Cam(see), Lift(), arm, None, verbose=False, learn=False)
            t.go_observe(0)
            ok = t.grasp(M(0.0, 0.0, 180.0, 5))
            for k in over:
                setattr(cfg, k, saved.get(k, saved2.get(k)))
            return arm.sent, ok, t

        base, _, _ = grasp_frames({"VERIFY": "observe"})
        check("关闭时：动作时间和原来一样（张爪 300、ready 1000、伸出 1200、合爪 500、carry 1000）",
              [ms for _, ms in base[:6]] == [1000, 300, 1000, 1200, 500, 1000], str([ms for _, ms in base]))
        fast, _, _ = grasp_frames({"VERIFY": "observe", "ARM_MOVE_SCALE": True})
        p = cfg.POSES
        want_carry = int(min(1000, max(cfg.ARM_MIN_MS, max(abs(p["carry"][k] - p["reach_near"][k]) for k in (4, 5))
                                       * cfg.ARM_MS_PER_US)))
        check("打开后：第一次动作不知道舵机在哪 → 用原来的时间", fast[0][1] == 1000)
        check("打开后：张爪（已经张着，差 0）→ 最短 ARM_MIN_MS 和原值取小", fast[1][1] == min(300, cfg.ARM_MIN_MS))
        check("打开后：伸向物块那一步保持慢速", fast[3][1] == cfg.ARM_SLOW_MS)
        check("打开后：carry 按最大脉宽差缩放", fast[5][1] == want_carry, f"{fast[5][1]} / {want_carry}")
        check("打开后：任何动作都不会比原来慢", all(f[1] <= b[1] for f, b in zip(fast, base)))
        check("只改时间、不改脉宽（协议不变）", [f[0] for f in fast] == [b[0] for b in base])
        cfg.ARM_MOVE_SCALE = True
        arm = Rec()
        t = GraspTask(cfg, Clock(), None, Lift(), arm, None, verbose=False, learn=False)
        t.go_observe(0)
        t.arm = Rec()                                      # 换了设备对象（重新打开串口）
        t.go_observe(0)
        check("换了机械臂设备后不再相信记住的位置", t.arm.sent[0][1] == cfg.ARM_MOVE_MS)
        cfg.ARM_MOVE_SCALE = False

        # ---- 去掉确认前的 carry
        skip, ok, _ = grasp_frames({"VERIFY": "observe", "VERIFY_SKIP_CARRY": True})
        carry = {**cfg.POSES["carry"], cfg.BASE_ID: 1500}
        n_carry = lambda fr: sum(1 for pz, _ in fr if pz == carry)      # noqa: E731
        check("observe 确认：原来抓到时 carry 两次，打开后只在放置前一次", n_carry(base) == 2 and n_carry(skip) == 1
              and ok, f"{n_carry(base)} → {n_carry(skip)}")
        auto, _, _ = grasp_frames({"VERIFY": "auto", "VERIFY_SKIP_CARRY": True}, see=[M(0, 0, 180, 5)] * 2)
        check("VERIFY_SKIP_CARRY 只影响 observe 模式", n_carry(auto) == 1 and auto[5][0] == carry)

        # ---- 整画面确认
        far = M(30.0, 5.0, 205.0, 5)                       # 原位置 (0,0,180)：右 30mm、远 25mm
        _, ok, _ = grasp_frames({"VERIFY": "observe"}, see=[far])
        check("关闭时：物块被推到别处 → 原位置空了 → 判成抓到（误报，原行为）", ok)
        _, ok, t = grasp_frames({"VERIFY": "observe", "VERIFY_WHOLE_FRAME": True}, see=[far, far])
        check("整画面确认：画面别处还有物块 → 没抓到，记下推开了多远", not ok and t._pushed == (30.0, 5.0, 25.0),
              str(t._pushed))
        _, ok, t = grasp_frames({"VERIFY": "observe", "VERIFY_WHOLE_FRAME": True}, see=[far])
        check("整画面确认：一闪而过（再测一次没有了）→ 按抓到处理", ok and t._pushed is None)
        _, ok, t = grasp_frames({"VERIFY": "observe", "VERIFY_WHOLE_FRAME": True, "VERIFY_WHOLE_MIN_D_MM": 100.0},
                                see=[M(0.0, 40.0, 70.0, 5)])
        check("整画面确认：比 VERIFY_WHOLE_MIN_D_MM 还近的（爪子里的）不算", ok)
        _, ok, _ = grasp_frames({"VERIFY": "observe", "VERIFY_WHOLE_FRAME": True}, see=[M(1.0, 1.0, 182.0, 5)])
        check("整画面确认：原位置还在 → 没抓到（和原来一样）", not ok)

        # ---- 同向才学 J（假底座：J = −1，回差 2°）
        class GapArm:
            def __init__(self): self.out, self.sent = 0.0, []
            def move(self, pulses, ms):
                self.sent.append((dict(pulses), ms))
                if 6 in pulses:
                    cmd = (pulses[6] - cfg.BASE_CENTER) / (cfg.BASE_US_PER_DEG * cfg.BASE_DIR)
                    self.out = min(max(self.out, cmd - 1.0), cmd + 1.0)

        class GapCam:
            def __init__(self, arm): self.arm = arm
            def measure(self, since, n, timeout):
                xd = 5.0 - self.arm.out                    # 物块在底座 +5° 方向
                return M(160.0 * math.tan(math.radians(xd)), 0.0, 160.0, n)

        def learned_j(same):
            cfg.LEARN_SAME_DIR_ONLY, cfg.R_CAM_MM, cfg.USE_CHASSIS = same, 0.0, False
            arm = GapArm()
            t = GraspTask(cfg, Clock(), GapCam(arm), Lift(), arm, None, verbose=False, learn=True)
            t.set_base(20.0)                               # 搜索时一路右转过去
            ok, _, _ = t.align(t.measure())                # 对准要左转回来：第一步换向，被回差吃掉 2°
            return t.axes["x_deg"].j, t.learn_skipped_reversal, ok

        j_all, sk0, ok0 = learned_j(False)
        j_same, sk1, ok1 = learned_j(True)
        check("每步都学：换向那一步被回差吃掉 → J 学小（原行为）", ok0 and j_all > -0.99 and sk0 == 0, f"J={j_all:.3f}")
        check("同向才学：跳过换向步，J 仍是真值 −1", ok1 and abs(j_same + 1.0) < 0.01 and sk1 == 1, f"J={j_same:.3f}")

        # ---- 仿真：合爪推开物块模型 + 整画面确认消除误报
        cfg.POSES = dict(sim.SIM_POSES)
        w, clock, cam, lift, arm2, ch = sim.make_sim(cfg, 3, 1.0, 0.0, sim.make_car(cfg, 3))
        check("仿真推开模型默认关", cfg.SIM_CLAW_PUSH_P == 0 and w.claw_pushes == 0)
        import sim_speed_verify as sv
        res = {}
        for name, over in (("off", {}), ("whole", sv.WHOLE), ("all", {**sv.ALL13, **sv.LEARN})):
            res[name] = io_capture(lambda: res.__setitem__(name + "_r", sv.summarize(name, over, 2, 25)))
        r0, r1, r2 = res["off_r"], res["whole_r"], res["all_r"]
        check("仿真（推开概率 0.5）：现状确实有误报", r0["false_ok"] > 0, f"误报 {r0['false_ok']}")
        check("仿真：整画面确认后误报 = 0，真实抓到率不降", r1["false_ok"] == 0 and r1["ok"] >= r0["ok"] - 0.02,
              f"{r0['ok']:.0%} → {r1['ok']:.0%}")
        check("仿真：任务 13 全开 + 任务 09 学习：误报 = 0、更快", r2["false_ok"] == 0 and r2["t"] < r0["t"],
              f"{r0['t']:.1f}s → {r2['t']:.1f}s")
    finally:
        for k, v in {**saved, **saved2}.items():
            setattr(cfg, k, v)
        importlib.reload(cfg)


def io_capture(fn):
    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fn()
    return buf.getvalue()


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
        # 任务 09 全开（只测无底盘：任务 09 针对无底盘；有底盘 6 台车×100 次对比见 09_报告.md，差别在噪声内）
        acc = {"BASE_BACKLASH_DEG": 2.0, "BASE_MIN_STEP_DEG": 1.0, "GRASP_PROBE_MM": 3.0, "TRIM_LEARN": True}
        # 任务 13 全开（observe 确认 + 整画面 + 去掉 carry + 动作时间缩放 + 同向才学 J）
        sp = {"VERIFY": "observe", "VERIFY_WHOLE_FRAME": True, "VERIFY_SKIP_CARRY": True, "ARM_MOVE_SCALE": True,
              "LEARN_SAME_DIR_ONLY": True}
        for n, seed, nc, need, extra in ((40, 31, False, 0.85, {}), (30, 33, True, 0.85, {}),
                                         (40, 31, False, 0.85, fixes), (30, 33, True, 0.85, acc),
                                         (30, 33, True, 0.85, sp)):
            importlib.reload(cfg)
            for k, v in extra.items():
                setattr(cfg, k, v)
            main.cfg = cfg
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rate = main.run_sim(n, seed, True, nc)
            line = [l for l in buf.getvalue().splitlines() if l.startswith("全部")][0]
            tag = ("（打开任务 13 的开关）" if "ARM_MOVE_SCALE" in extra else
                   "（打开任务 09 的开关）" if "TRIM_LEARN" in extra else "（打开任务 02 的改进开关）") if extra else ""
            check(f"{'无' if nc else '有'}底盘 {n} 次{tag}，"
                  f"真实抓到 ≥ {need:.0%}", rate >= need, line.split(":", 1)[1].strip())
    finally:
        importlib.reload(cfg)
        main.cfg = cfg


# ------------------------------------------------------------ 底盘联调预演（任务 18，test_chassis_rehearsal.py）
def test_chassis_rehearsal(full=False):
    import test_chassis_rehearsal as tcr
    tcr.PASS, tcr.FAIL = PASS, FAIL          # 结果计入 selftest 的统计
    tcr.quick()
    if full:
        tcr.stats(40)


# ------------------------------------------------------------ 研究原型（可选，任务 11）
def test_research():
    print("研究原型 research/（任务 11，仿真）")
    import io
    import unittest
    from research import test_research as tr
    buf = io.StringIO()
    res = unittest.TextTestRunner(stream=buf, verbosity=1).run(unittest.defaultTestLoader.loadTestsFromModule(tr))
    bad = [str(t) for t, _ in res.failures + res.errors]
    check(f"research 单元测试 {res.testsRun} 项全部通过", res.wasSuccessful(), "；".join(bad))


if __name__ == "__main__":
    quick = len(sys.argv) > 1 and sys.argv[1] == "quick"
    research = "research" in sys.argv[1:]
    real_serial = devices.serial
    for t in (test_arm, test_k230, test_lift, test_chassis, test_chassis_fw, test_calib_override,
              test_step_and_guard, test_geometry, test_ghost, test_ghost_fingerprint, test_sim_factors,
              test_bench, test_accuracy, test_speed_verify):
        try:
            t()
        except Exception as e:
            check(f"{t.__name__} 崩溃", False, repr(e))
        finally:
            devices.serial = real_serial
    try:
        test_chassis_rehearsal(full=not quick)
    except Exception as e:
        check("test_chassis_rehearsal 崩溃", False, repr(e))
    finally:
        devices.serial = real_serial
    if not quick:
        test_sim()
    if research:
        try:
            test_research()
        except Exception as e:
            check("test_research 崩溃", False, repr(e))
    print(f"\n通过 {len(PASS)} 项，失败 {len(FAIL)} 项" + ("" if not FAIL else "：" + "；".join(FAIL)))
    sys.exit(1 if FAIL else 0)
