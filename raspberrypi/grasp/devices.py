"""
各下位机驱动。真机和仿真（sim.py）实现同一套接口，任务逻辑不用区分。
"""
import atexit
import json
import math
import os
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass

try:
    import serial   # pip install pyserial
except ImportError:  # 仿真模式不需要
    serial = None


class DeviceError(RuntimeError):
    pass


class ManualOverride(DeviceError):
    """手柄接管：不是故障，等手柄断开后继续"""


class DeviceResetError(DeviceError):
    """下位机复位了（升降轴发了 READY）：它记的位置可能不对，不能自动接着动"""


class RealClock:
    def now(self):
        return time.monotonic()

    def sleep(self, s):
        if s > 0:
            time.sleep(s)


@dataclass
class Measurement:
    x: float          # mm，右为正
    y: float          # mm，下为正
    d: float          # mm，深度
    n: int            # 用了几帧

    @property
    def x_deg(self):
        return math.degrees(math.atan2(self.x, max(self.d, 1.0)))

    def err(self, goal):
        """相对目标的误差（各轴）"""
        return {"x_deg": self.x_deg - goal["x_deg"],
                "y": self.y - goal["y"],
                "d": self.d - goal["d"]}


def _median_measure(frames, n):
    if len(frames) < max(min(3, n), n // 2 + 1):     # 要 n 帧时至少有 min(3,n) 帧、且超过一半
        return None
    frames = frames[-n:]
    return Measurement(statistics.median(f[1] for f in frames),
                       statistics.median(f[2] for f in frames),
                       statistics.median(f[3] for f in frames),
                       len(frames))


# ============================================================ K230
def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if crc & 0x80 else (crc << 1)
            crc &= 0xFF
    return crc


def rotate_xy(x, y, rot):
    """相机顺时针转了 rot 度安装时，把画面坐标转回 x 右、y 下"""
    if rot == 90:
        return y, -x
    if rot == -90:
        return -y, x
    if rot == 180:
        return -x, -y
    return x, y


class K230:
    """
    接收 K230 的 10 字节帧：AA 55 | X(int16) | Y(int16) | D(uint16) | CRC8 | 0D
    大端，CRC8 多项式 0x07、初值 0，校验前 8 字节。
    深度 = 0 的帧是心跳：链路正常、但 K230 没看到物块（K230 端每 200ms 发一次）
    """
    FRAME_LEN = 10

    def __init__(self, port, baud, clock, timeout_s, rotate=0):
        if serial is None:
            raise DeviceError("缺少 pyserial：pip install pyserial")
        self.clock = clock
        self.timeout_s = timeout_s
        self.rotate = rotate      # 相机装歪了 90°/180° 时把坐标转回"x 右、y 下"
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.frames = deque(maxlen=300)    # (t, x, y, d)，只存看到物块的帧
        self.bad_frames = 0
        self.heartbeats = 0
        self.last_rx = -1e9                # 最后一次收到任何有效帧（含心跳）的时间
        self._lock = threading.Lock()
        self._run = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = bytearray()
        while self._run:
            try:
                buf += self.ser.read(64)
            except Exception:
                time.sleep(0.2)
                continue
            while True:
                i = buf.find(b"\xAA\x55")
                if i < 0:
                    del buf[:-1]       # 保留最后一个字节，可能是 AA
                    break
                del buf[:i]
                if len(buf) < self.FRAME_LEN:
                    break
                f = bytes(buf[:self.FRAME_LEN])
                if f[9] == 0x0D and crc8(f[:8]) == f[8]:
                    x = int.from_bytes(f[2:4], "big", signed=True)
                    y = int.from_bytes(f[4:6], "big", signed=True)
                    d = int.from_bytes(f[6:8], "big", signed=False)
                    x, y = rotate_xy(x, y, self.rotate)
                    with self._lock:
                        self.last_rx = self.clock.now()
                        if d == 0:
                            self.heartbeats += 1
                        else:
                            self.frames.append((self.last_rx, x, y, d))
                    del buf[:self.FRAME_LEN]
                else:
                    self.bad_frames += 1
                    del buf[:1]        # 错位，往后找下一个帧头

    def link_ok(self, within_s=1.0):
        """1 秒内收到过帧（位置帧或心跳）= 串口和 K230 程序都正常"""
        return self.clock.now() - self.last_rx < within_s

    def visible(self):
        with self._lock:
            return bool(self.frames) and self.clock.now() - self.frames[-1][0] < self.timeout_s

    def measure(self, since, n, timeout):
        """收集 since 之后的 n 帧取中位数；目标丢失或帧不够返回 None"""
        deadline = self.clock.now() + timeout
        while True:
            with self._lock:
                fr = [f for f in self.frames if f[0] >= since]
            if len(fr) >= n:
                return _median_measure(fr, n)
            if self.clock.now() > deadline:
                return _median_measure(fr, n)
            self.clock.sleep(0.02)

    def close(self):
        self._run = False


# ============================================================ 文本协议下位机（升降轴 / 底盘）
class _LineDevice:
    """一行一条指令，回复 OK/ERR/POS，运动结束回 DONE；收到 READY 说明下位机复位了"""

    def __init__(self, name, port, baud):
        if serial is None:
            raise DeviceError("缺少 pyserial：pip install pyserial")
        self.name = name
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.1)
        # 先发一个空行：清掉下位机行缓冲里残留的半行（否则第一条指令会被当成 ERR UNKNOWN）
        self.ser.write(b"\n")
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def _readline(self, deadline):
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            if line.startswith("READY"):
                raise DeviceResetError(f"{self.name} 复位了：{line}")
            return line
        raise DeviceError(f"{self.name} 回复超时")

    def cmd(self, text, timeout=1.0):
        self.ser.write((text + "\n").encode("ascii"))
        deadline = time.monotonic() + timeout
        while True:
            line = self._readline(deadline)
            if line.startswith("ERR"):
                raise DeviceError(f"{self.name}: {text} -> {line}")
            if line.startswith(("OK", "POS")):
                return line

    def wait_done(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            line = self._readline(deadline)
            if line.startswith("DONE"):
                return line


class Lift(_LineDevice):
    """
    升降轴（伺服电机 STM32），协议见 伺服电机控制/readme.md

    软件限位：所有移动都换算成绝对目标再截断到 [0, max_cm]，比固件的 50cm 更保守。
    （不用固件的 R 相对指令，免得软件这边不知道实际目标在哪）
    """

    def __init__(self, port, baud, max_cm=50.0):
        super().__init__("升降轴", port, baud)
        self.max_cm = max_cm
        self.pos = None             # 最近一次已知位置（cm）

    def position(self):
        parts = self.cmd("P").split()
        if parts[2] == "UNHOMED":
            raise DeviceError("升降轴位置未知（UNHOMED）：把它放到最下端后发 Z，或断电重启")
        self.pos = float(parts[1])
        return self.pos

    def _target(self, cm):
        return max(0.0, min(self.max_cm, cm))

    def move_to(self, cm, timeout=30.0):
        self.cmd(f"A {self._target(cm):.2f}")
        return self.wait_move(timeout)

    def move_by(self, cm, timeout=30.0):
        self.start_move_by(cm)
        return self.wait_move(timeout)

    def start_move_by(self, cm):
        """只发指令不等待，之后调用 wait_move()，期间可以去动别的设备"""
        base = self.pos if self.pos is not None else self.position()
        self.cmd(f"A {self._target(base + cm):.2f}")

    def wait_move(self, timeout=30.0):
        self.pos = float(self.wait_done(timeout).split()[1])
        return self.pos

    def stop(self):
        self.cmd("E")


class WheeltecChassis:
    """
    WHEELTEC 底盘控制器（/dev/ttyACM0，115200），和 pad_drive3.py / ack_forward_cm.py 同一个协议：
      下发 11 字节：7B 00 00 | vx(int16, mm/s) | vy(int16) | wz(int16, 0.001rad/s，左转为正) | BCC | 7D
      回传 24 字节：7B flag | vx vy wz | 加速度x3 | 角速度x3 | 电压(mV) | BCC | 7D （约 20Hz）
    用回传的实际速度积分里程，走多远按里程判断，不按时间猜。

    ackermann=True：阿克曼转向，不能原地转；转向只能边走边转（drive 的 dpsi_deg）

    固件分析结论（底盘代码/大车底盘，详见 docs/cloud_tasks/results/03_报告.md；仓库里的源码和真机
    刷的固件版本可能不同，以下都"未在真机验证"）：
      - 固件【没有】串口超时停车：最后一帧的速度会一直保持（usartx.c:645-705 只在收到帧时改 Move_X）。
        所以这里 50Hz 持续发送 + 指令过期（cmd_ttl_s）自动发 0 + 退出时直接写停车帧。
      - 阿克曼车型【不做】速度平滑（balance.c:24 只对麦轮/全向轮调用 Smooth_control），
        松油门后的滑行来自增量式 PI（balance.c:378-397，KP=KI=700，100Hz）：
        积分恒等式给出 滑行距离 ≈ v × τ_PI，τ_PI 和速度无关、和电池电压成反比。
      - 回传 flag 字节 = Flag_Stop（usartx.c:109），1 = 电机失能（开机前 10 秒都是 1）。
      - 回传角速度 Z = ICM20948 原始值，量程 ±500°/s → 65.5 LSB/(°/s)（ICM20948.c:85）。
    """
    TX_HZ = 50

    def __init__(self, port, baud, speed_mm_s=100, min_radius_mm=600, ackermann=True, tau_file=None,
                 stop_model="tau", cmd_latency_s=0.02, pi_tau_s=0.45, ref_voltage_v=25.8,
                 cmd_ttl_s=0.5, yaw_trim_deg_per_m=0.0, gyro_lsb_per_dps=65.5):
        if serial is None:
            raise DeviceError("缺少 pyserial：pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.speed = speed_mm_s
        self.coast_tau = 0.5     # 松油门后的滑行时间常数（s）：滑行距离 ≈ τ × 速度；会自动修正
        # 模型预测（stop_model="model"）：滑行 = v ×（指令延迟 + τ_PI × 参考电压/当前电压），
        # 并用回传帧的"年龄"外推里程。τ_PI 同样自适应学习。"tau" = 原来的做法（默认）
        self.stop_model = stop_model
        self.cmd_latency = cmd_latency_s
        self.pi_tau = pi_tau_s
        self.ref_voltage = ref_voltage_v
        self.tau_file = tau_file  # 学到的 τ 存这里，下次启动直接用
        if tau_file:
            try:
                with open(tau_file, encoding="utf-8") as f:
                    saved = json.load(f)
                self.coast_tau = float(saved["coast_tau"])
                self.pi_tau = float(saved.get("pi_tau", self.pi_tau))
            except (OSError, ValueError, KeyError, TypeError, AttributeError):
                pass
        self.min_radius = min_radius_mm
        self.ackermann = ackermann
        self.cmd_ttl = cmd_ttl_s          # 指令多久没刷新就自动改发 0（固件自己不会超时停车）；0 = 关
        self.yaw_trim = math.radians(yaw_trim_deg_per_m) / 1000.0   # rad/mm：直走时车头每 mm 右偏多少
        self.gyro_lsb = gyro_lsb_per_dps
        self.cmd = (0, 0)
        self.cmd_t = time.monotonic()
        self.ttl_tripped = 0       # 指令过期自动停车的次数（调试用）
        self.dist = 0.0            # 里程（mm，前进为正）
        self.yaw = 0.0             # 航向（rad，左转为正），编码器差速算的（回传 wz）
        self.yaw_gyro = 0.0        # 航向（rad，左转为正），陀螺仪 Z 积分（只做对比，未在真机验证量程）
        self.fb_vx = 0
        self.fb_wz = 0
        self.fb_flag = 0           # 固件 Flag_Stop：1 = 电机失能
        self.gyro_z_raw = 0
        self.voltage_mv = 0
        self.last_fb = 0.0
        self.last_release = None   # 最近一次松油门的记录（调试/测试脚本用）
        self._lock = threading.Lock()
        self._run = True
        threading.Thread(target=self._tx, daemon=True).start()
        threading.Thread(target=self._rx, daemon=True).start()
        atexit.register(self._emergency_stop)
        time.sleep(0.3)            # 等第一批回传帧

    @staticmethod
    def frame(vx, wz):
        vx = int(vx) & 0xFFFF
        wz = int(wz) & 0xFFFF
        fr = bytearray([0x7B, 0, 0, vx >> 8, vx & 0xFF, 0, 0, wz >> 8, wz & 0xFF])
        b = 0
        for x in fr:
            b ^= x
        return bytes(fr + bytes([b, 0x7D]))

    def _tx(self):
        while self._run:
            with self._lock:
                vx, wz = self.cmd
                if (vx or wz) and self.cmd_ttl > 0 and self._now() - self.cmd_t > self.cmd_ttl:
                    # 调用方卡住/崩了没再刷新指令：固件不会自己停，这里替它停
                    self.cmd = (0, 0)
                    vx = wz = 0
                    self.ttl_tripped += 1
            try:
                self.ser.write(self.frame(vx, wz))
            except Exception:
                pass
            time.sleep(1.0 / self.TX_HZ)

    def _rx(self):
        buf = bytearray()
        last = None
        while self._run:
            try:
                buf += self.ser.read(128)
            except Exception:
                time.sleep(0.2)
                continue
            while True:
                i = buf.find(b"\x7B")
                if i < 0:
                    buf.clear()
                    break
                del buf[:i]
                if len(buf) < 24:
                    break
                f = bytes(buf[:24])
                b = 0
                for x in f[:22]:
                    b ^= x
                if f[23] != 0x7D or b != f[22]:
                    del buf[:1]
                    continue
                del buf[:24]
                now = time.monotonic()
                vx = int.from_bytes(f[2:4], "big", signed=True)
                wz = int.from_bytes(f[6:8], "big", signed=True)
                gz = int.from_bytes(f[18:20], "big", signed=True)
                with self._lock:
                    if last is not None and now - last < 0.5:
                        dt = now - last
                        self.dist += vx * dt
                        self.yaw += wz / 1000.0 * dt
                        if self.gyro_lsb:
                            self.yaw_gyro += math.radians(gz / self.gyro_lsb) * dt
                    self.fb_vx = vx
                    self.fb_wz = wz
                    self.fb_flag = f[1]
                    self.gyro_z_raw = gz
                    self.voltage_mv = int.from_bytes(f[20:22], "big")
                    self.last_fb = now
                last = now

    def _set(self, vx, wz):
        with self._lock:
            self.cmd = (int(vx), int(wz))
            self.cmd_t = self._now()

    # 时间函数单独拿出来：仿真（sim.SimWheeltec）用仿真时钟替换，真机行为不变
    def _now(self):
        return time.monotonic()

    def _sleep(self, s):
        time.sleep(s)

    def has_feedback(self):
        return self._now() - self.last_fb < 0.5

    def _voltage_v(self):
        v = self.voltage_mv / 1000.0
        return v if 5.0 < v < 60.0 else self.ref_voltage

    def predict_coast(self, v_act):
        """模型预测：以实际速度 v_act（mm/s）松油门后还会走多远（mm）"""
        tau = self.cmd_latency + self.pi_tau * self.ref_voltage / self._voltage_v()
        if not (0.0 < tau < 3.0):          # 模型参数坏了：退回自适应 τ
            tau = self.coast_tau
        return abs(v_act) * tau

    MIN_SPEED = 30          # mm/s，快到终点时最低指令速度（太低控制器可能不动）
    RAMP_K = 1.5            # 1/s：剩余距离 × RAMP_K = 指令速度上限（越靠近终点越慢）

    def drive(self, mm, dpsi_deg=0.0, timeout=None):
        """
        走 mm（负数后退），同时让车头右转 dpsi_deg（负数左转）。
        阿克曼底盘转角受最小转弯半径限制，会自动截断。返回实际 (走了多少 mm, 车头右转了多少度)

        松油门后车还会滑一段（固件 PI 速度环要把积分量放掉）。所以：
          1. 快到终点按剩余距离降低指令速度
          2. 剩余距离 ≤ 预计滑行距离 时松油门
             stop_model="tau"  ：预计滑行 = τ × 回传的实际速度（原做法）
             stop_model="model"：预计滑行 = predict_coast(v)，里程按回传帧年龄外推
          3. 停稳后用"滑了多远 / 松油门时的速度"修正 τ（两种参数都学，越走越准）
        """
        if abs(mm) < 1:
            return 0.0, 0.0
        max_dpsi = math.degrees(abs(mm) / self.min_radius) if self.ackermann else 180.0
        dpsi_deg = max(-max_dpsi, min(max_dpsi, dpsi_deg))
        sign = 1 if mm > 0 else -1
        target = abs(mm)
        curv = -math.radians(dpsi_deg) / target      # 每 mm 的航向变化（rad/mm，ROS 左转为正）
        timeout = timeout or (target / self.MIN_SPEED * 1.5 + 3.0)
        model = self.stop_model == "model"

        with self._lock:
            d0, y0, g0 = self.dist, self.yaw, self.yaw_gyro
        fb = self.has_feedback()
        t0 = self._now()
        done_rel = v_rel = done_est_rel = None
        try:
            while True:
                el = self._now() - t0
                if fb:
                    with self._lock:
                        done = abs(self.dist - d0)
                        v_act = abs(self.fb_vx)
                        age = self._now() - self.last_fb
                    if model:
                        done_est = done + v_act * min(max(age, 0.0), 0.1)   # 回传帧之后又走了这么多
                        coast = self.predict_coast(v_act)
                    else:
                        done_est = done
                        coast = self.coast_tau * v_act
                    remaining = target - done_est
                    if remaining <= coast or remaining <= 0:
                        done_rel, v_rel, done_est_rel = done, v_act, done_est
                        break
                    v = max(self.MIN_SPEED, min(self.speed, remaining * self.RAMP_K))
                else:
                    if el >= target / self.speed:            # 没有回传时按时间走
                        break
                    v = self.speed
                # wz 跟着速度变，保证走的是同一条弧线；yaw_trim 抵消转向没回正（倒车时舵角要同向，所以乘 sign）
                self._set(sign * v, (curv + sign * self.yaw_trim) * v * 1000.0)
                if el > timeout:
                    flag = f"，回传 flag={self.fb_flag}（1 = 固件电机失能：开机不到 10 秒？）" if self.fb_flag else ""
                    raise DeviceError("底盘移动超时（被卡住？）" + flag)
                self._sleep(0.02)
        finally:
            self._set(0, 0)
        t1 = self._now()                                # 等车停稳
        while fb and abs(self.fb_vx) > 5 and self._now() - t1 < 1.5:
            self._sleep(0.02)
        if not fb:
            return float(mm), dpsi_deg
        with self._lock:
            moved, dyaw, dgyro = self.dist - d0, self.yaw - y0, self.yaw_gyro - g0
        self.last_release = {"done": done_rel, "done_est": done_est_rel, "v": v_rel, "moved": moved,
                             "dyaw_gyro_deg": -math.degrees(dgyro), "voltage_v": self._voltage_v()}
        if v_rel and v_rel > 20:
            tau_meas = (abs(moved) - done_rel) / v_rel
            tau_meas = max(0.05, min(1.5, tau_meas))
            self.coast_tau += 0.6 * (tau_meas - self.coast_tau)
            # 模型参数：扣掉指令延迟，再换算到参考电压
            pi_meas = ((abs(moved) - done_est_rel) / v_rel - self.cmd_latency) * self._voltage_v() / self.ref_voltage
            pi_meas = max(0.02, min(1.5, pi_meas))
            self.pi_tau += 0.6 * (pi_meas - self.pi_tau)
            if self.tau_file:
                try:
                    os.makedirs(os.path.dirname(self.tau_file) or ".", exist_ok=True)
                    with open(self.tau_file, "w", encoding="utf-8") as f:
                        json.dump({"coast_tau": self.coast_tau, "pi_tau": self.pi_tau}, f)
                except OSError:
                    pass
        return moved, -math.degrees(dyaw)

    def forward(self, mm, timeout=None):
        return self.drive(mm, 0.0, timeout)[0]

    def turn(self, deg, timeout=None):
        raise DeviceError("阿克曼底盘不能原地转向" if self.ackermann else "差速原地转向尚未实现")

    def stop(self):
        self._set(0, 0)

    def _write_stop_frames(self, n=3):
        """不经过发送线程，直接往串口写停车帧（退出时线程可能已经不在了）"""
        for _ in range(n):
            try:
                self.ser.write(self.frame(0, 0))
                self.ser.flush()
            except Exception:
                return

    def _emergency_stop(self):
        if self._run:
            self._set(0, 0)
            self._write_stop_frames()

    def close(self):
        self._set(0, 0)
        time.sleep(0.1)
        self._write_stop_frames()
        self._run = False


class ManualGuard:
    """手柄控制程序（pad_drive3.py）在运行 = 人工接管，自动抓取必须让出串口"""

    def __init__(self, pattern="pad_drive3.py"):
        self.pattern = pattern
        self.active = False
        threading.Thread(target=self._watch, daemon=True).start()

    def _watch(self):
        import subprocess
        while True:
            try:
                r = subprocess.run(["pgrep", "-f", self.pattern], capture_output=True)
                self.active = (r.returncode == 0)
            except Exception:
                self.active = False
            time.sleep(1.0)


class NullChassis:
    def turn(self, deg, timeout=0):
        raise DeviceError("没有底盘（USE_CHASSIS=False）")

    forward = turn

    def drive(self, mm, dpsi_deg=0.0, timeout=None):
        raise DeviceError("没有底盘（USE_CHASSIS=False）")

    def stop(self):
        pass


# ============================================================ 机械臂（Lobot 控制板）
class Arm:
    """
    Lobot 协议 CMD_MULT_SERVO_MOVE：
      55 55 | len=3n+5 | 03 | n | time_L time_H | (id pos_L pos_H) * n
    控制板不回传位置，所以按动作时间等待。

    USB 掉线保护：机械臂板的 CH340 偶尔会断开重连（实测舵机供电不稳或线接触不良时会这样），
    发送失败时最多等 RECONNECT_S 秒，设备重新出现就重开串口、把这一帧重发。
    """
    CMD_MULT_SERVO_MOVE = 3
    RECONNECT_S = 5.0

    def __init__(self, port, baud, clock, ser_factory=None, robust=False):
        # ser_factory：仿真注入假串口（sim.SimArmSerial）；真机用 pyserial
        # robust（config.ARM_USB_ROBUST）：掉线后在 RECONNECT_S 内反复重连；重连后把所有舵机的
        #   最新位置整帧重发（掉线前那一帧可能写进缓冲区没报错却丢了）；最终失败报 DeviceError
        self.robust = robust
        if ser_factory is None and serial is None:
            raise DeviceError("缺少 pyserial：pip install pyserial")
        self._factory = ser_factory or serial.Serial
        self._errs = (OSError,) + ((serial.SerialException,) if serial is not None else ())
        self.port, self.baud = port, baud
        self.clock = clock
        self.state = {}
        self.reconnects = 0
        self.ser = None
        self._open(wait_s=2.0)

    def _open(self, wait_s):
        deadline = self.clock.now() + wait_s
        while True:
            try:
                self.ser = self._factory(self.port, self.baud, timeout=0.1)
                return
            except self._errs:
                if self.clock.now() > deadline:
                    raise DeviceError(f"机械臂串口打不开：{self.port}（USB 线松了 / 机械臂板没上电？"
                                      f"用 `ls -l /dev/serial/by-path/` 看看还在不在）")
                self.clock.sleep(0.2)

    def move(self, pulses, ms):
        pulses = {int(k): int(max(500, min(2500, v))) for k, v in sorted(pulses.items())}
        if not pulses:
            return
        ms = int(ms)
        n = len(pulses)
        frame = bytearray([0x55, 0x55, 3 * n + 5, self.CMD_MULT_SERVO_MOVE, n, ms & 0xFF, (ms >> 8) & 0xFF])
        for sid, p in pulses.items():
            frame += bytes([sid, p & 0xFF, (p >> 8) & 0xFF])
        try:
            self.ser.write(frame)
            self.ser.flush()
        except self._errs:
            if self.robust:
                self._reconnect_send(pulses, ms)
            else:
                # USB 掉了：等它回来，重开串口再发一次
                try:
                    self.ser.close()
                except Exception:
                    pass
                self._open(wait_s=self.RECONNECT_S)
                self.reconnects += 1
                self.ser.write(frame)
                self.ser.flush()
        self.state.update(pulses)
        self.clock.sleep(ms / 1000.0 + 0.15)


    def _frame(self, pulses, ms):
        n = len(pulses)
        frame = bytearray([0x55, 0x55, 3 * n + 5, self.CMD_MULT_SERVO_MOVE, n, ms & 0xFF, (ms >> 8) & 0xFF])
        for sid, p in pulses.items():
            frame += bytes([sid, p & 0xFF, (p >> 8) & 0xFF])
        return frame

    def _reconnect_send(self, pulses, ms):
        """robust 模式：RECONNECT_S 内反复重开串口，成功后发 {之前所有舵机的位置 + 这一帧}"""
        deadline = self.clock.now() + self.RECONNECT_S
        full = {**self.state, **pulses}
        frame = self._frame(dict(sorted(full.items())), ms)
        while True:
            try:
                self.ser.close()
            except Exception:
                pass
            self._open(wait_s=max(0.2, deadline - self.clock.now()))   # 超时会报 DeviceError
            self.reconnects += 1
            try:
                self.ser.write(frame)
                self.ser.flush()
                return
            except self._errs:
                if self.clock.now() > deadline:
                    raise DeviceError("机械臂 USB 反复掉线，重连失败（检查 USB 线和舵机供电）")
                self.clock.sleep(0.2)


def lerp_pose(a, b, t):
    t = max(0.0, min(1.0, t))
    return {k: a[k] + (b[k] - a[k]) * t for k in a if k in b}
