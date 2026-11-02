#!/usr/bin/env python3
"""
树莓派控制升降轴（STM32 下位机）

依赖：pip install pyserial
串口：树莓派自带 UART 用 /dev/serial0；USB 转 TTL 模块一般是 /dev/ttyUSB0
"""
import time
import serial


class LiftError(RuntimeError):
    pass


class Lift:
    def __init__(self, port="/dev/serial0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    # ---------- 底层 ----------
    def _readline(self, deadline):
        while time.time() < deadline:
            line = self.ser.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            if line.startswith("READY"):
                # 下位机复位了（断电、看门狗……），位置可能不可信
                raise LiftError("下位机复位: " + line)
            return line
        raise TimeoutError("等待下位机回复超时")

    def cmd(self, text, timeout=1.0):
        """发一条指令，返回 OK/POS 那一行；收到 ERR 抛异常"""
        self.ser.write((text + "\n").encode("ascii"))
        deadline = time.time() + timeout
        while True:
            line = self._readline(deadline)
            if line.startswith("ERR"):
                raise LiftError(f"{text} -> {line}")
            if line.startswith(("OK", "POS")):
                return line
            # 其他行（比如之前没等的 DONE）忽略

    def wait_done(self, timeout=60.0):
        """等电机最终停下，返回 (位置cm, 是否被 S/E 打断)。
        运动中改过几次目标也只会有一条 DONE"""
        deadline = time.time() + timeout
        while True:
            line = self._readline(deadline)
            if line.startswith("DONE"):
                parts = line.split()
                return float(parts[1]), ("STOP" in parts)

    # ---------- 常用操作 ----------
    def move_to(self, cm, wait=True, timeout=60.0):
        """走到绝对位置（0 = 最下端）"""
        self.cmd(f"A {cm:.2f}")
        return self.wait_done(timeout) if wait else None

    def move_by(self, cm, wait=True, timeout=60.0):
        """相对移动，正数向上；超出行程会自动截断。
        运动中调用会在当前目标上累加（配合 wait=False 使用）"""
        self.cmd(f"R {cm:+.2f}")
        return self.wait_done(timeout) if wait else None

    def stop(self):
        """减速停"""
        self.cmd("S")

    def halt(self):
        """立即停"""
        self.cmd("E")

    def position(self):
        """返回 (位置cm, 状态 IDLE/MOVING/UNHOMED)"""
        parts = self.cmd("P").split()
        return float(parts[1]), parts[2]

    def set_position(self, cm=0.0):
        """告诉下位机当前在哪（默认 0 = 最下端），复位后用"""
        self.cmd(f"Z {cm:.2f}")

    def speed(self, cm_s=None):
        """设置/查询速度 cm/s，下次运动生效"""
        line = self.cmd("V" if cm_s is None else f"V {cm_s:.2f}")
        return float(line.split()[2])


if __name__ == "__main__":
    lift = Lift("/dev/serial0")
    print("当前:", lift.position())
    lift.speed(8)
    print("到 20cm:", lift.move_to(20))
    print("再上 5cm:", lift.move_by(5))
    print("回 0:", lift.move_to(0))
