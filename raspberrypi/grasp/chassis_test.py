#!/usr/bin/env python3
"""
底盘停车精度真机测试（任务 03 的验证脚本）：走 +100mm ×3、−100mm ×3，打印每次误差。

  python3 chassis_test.py                 用 config.py 里的设置（默认 tau 做法）
  python3 chassis_test.py --model         用模型预测松油门（CHASSIS_STOP_MODEL="model"）
  python3 chassis_test.py --ruler         每次走完让你输入尺子量的距离，一起算误差
  python3 chassis_test.py --dist 200 --n 5 --speed 100
  python3 chassis_test.py --timeout-check 【先把车架空、轮子离地！】验证"固件没有串口超时"

说明：
  - 学到的 τ 存在 logs/chassis_tau_test.json，不影响 main.py 用的 logs/chassis_tau.json
  - 开机后 10 秒内固件不接收串口指令（源码 usartx.c:653 CONTROL_DELAY），脚本会自己等
  - 每次走完打印两种航向：编码器差速（回传 wz）和陀螺仪（回传角速度 Z，按 ±500°/s 量程换算，未验证）
    直走时如果陀螺仪航向总往一边偏，把打印的建议值填进 config.py 的 CHASSIS_YAW_TRIM_DEG_PER_M
"""
import argparse
import math
import os
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config as cfg  # noqa: E402
from devices import DeviceError, WheeltecChassis  # noqa: E402
from main import chassis_options  # noqa: E402


def manual_running():
    try:
        return subprocess.run(["pgrep", "-f", cfg.MANUAL_PROCESS], capture_output=True).returncode == 0
    except OSError:
        return False


def wait_ready(c, wait_s=15.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        if c.has_feedback() and c.fb_flag == 0:
            return True
        time.sleep(0.2)
    return c.has_feedback()


def timeout_check(port, baud):
    """不经过驱动，直接发帧：先以 50mm/s 发 1 秒，然后停发 1.5 秒，看回传速度是不是还在"""
    import serial
    print("⚠ 这个测试会让车轮在没有指令的情况下继续转！必须先把车架空、轮子离地。")
    if input("轮子已经离地？输入 yes 继续：").strip() != "yes":
        return
    ser = serial.Serial(port, baud, timeout=0.02)
    buf = bytearray()

    def read_vx():
        nonlocal buf
        buf += ser.read(256)
        vx = None
        while True:
            i = buf.find(b"\x7B")
            if i < 0 or len(buf) - i < 24:
                del buf[:max(0, i)]
                return vx
            f = bytes(buf[i:i + 24])
            b = 0
            for x in f[:22]:
                b ^= x
            if f[23] == 0x7D and b == f[22]:
                vx = int.from_bytes(f[2:4], "big", signed=True)
                del buf[:i + 24]
            else:
                del buf[:i + 1]

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            ser.write(WheeltecChassis.frame(50, 0))
            read_vx()
            time.sleep(0.02)
        print("停止发送，观察 1.5 秒内回传速度（mm/s）：")
        t1 = time.monotonic()
        seen = []
        while time.monotonic() - t1 < 1.5:
            v = read_vx()
            if v is not None:
                seen.append((time.monotonic() - t1, v))
            time.sleep(0.02)
        for t, v in seen:
            print(f"  {t:4.2f}s  vx={v}")
        tail = [v for t, v in seen if t > 1.0]
        if tail and min(abs(v) for v in tail) > 20:
            print("结论：停发 1 秒后车轮仍在转 → 固件没有串口超时（和源码分析一致），驱动的指令过期保护是必要的")
        elif tail:
            print("结论：停发后车轮停了 → 真机固件有超时停车（和仓库源码不一致，请记到报告里）")
        else:
            print("没收到回传，无法判断")
    finally:
        for _ in range(5):
            ser.write(WheeltecChassis.frame(0, 0))
            time.sleep(0.02)
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="底盘停车精度测试")
    ap.add_argument("--dist", type=float, default=100.0, help="每次走多远（mm），默认 100")
    ap.add_argument("--n", type=int, default=3, help="前进、后退各几次，默认 3")
    ap.add_argument("--speed", type=float, default=cfg.CHASSIS_SPEED_MM_S, help="速度 mm/s")
    ap.add_argument("--model", action="store_true", help="用模型预测松油门（默认按 config.py）")
    ap.add_argument("--tau", action="store_true", help="用原来的自适应 τ（默认按 config.py）")
    ap.add_argument("--ruler", action="store_true", help="每次走完输入尺子量的距离")
    ap.add_argument("--fresh", action="store_true", help="不读上次学到的 τ，从默认值开始学")
    ap.add_argument("--timeout-check", action="store_true", help="验证固件有没有串口超时（轮子必须离地）")
    a = ap.parse_args()

    if manual_running():
        print(f"手柄程序 {cfg.MANUAL_PROCESS} 在运行，会抢底盘串口。先停掉它。")
        return 1
    if a.timeout_check:
        timeout_check(cfg.PORTS["chassis"], cfg.BAUD["chassis"])
        return 0

    opts = chassis_options(cfg)
    if a.model:
        opts["stop_model"] = "model"
    if a.tau:
        opts["stop_model"] = "tau"
    tau_file = os.path.join(HERE, cfg.LOG_DIR, "chassis_tau_test.json")
    if a.fresh and os.path.exists(tau_file):
        os.remove(tau_file)
    c = WheeltecChassis(cfg.PORTS["chassis"], cfg.BAUD["chassis"], a.speed, cfg.ACK_MIN_RADIUS_MM,
                        cfg.CHASSIS_ACKERMANN, tau_file, **opts)
    try:
        if not wait_ready(c):
            print("底盘没有回传：检查 USB 线、底盘电源开关")
            return 1
        print(f"底盘电压 {c.voltage_mv / 1000:.2f}V  flag={c.fb_flag}  松油门方式={c.stop_model}  "
              f"τ={c.coast_tau:.3f}s  τ_PI={c.pi_tau:.3f}s  yaw_trim={cfg.CHASSIS_YAW_TRIM_DEG_PER_M}°/m")
        if c.fb_flag:
            print("⚠ flag=1（源码里表示电机失能），车可能不会动")
        input(f"车前后各留 {a.dist + 150:.0f}mm 空地，回车开始（Ctrl+C 随时停车）")
        rows = []
        for mm in [a.dist] * a.n + [-a.dist] * a.n:
            moved, dpsi = c.drive(mm)
            r = c.last_release or {}
            row = {"cmd": mm, "odo": moved, "err": abs(moved) - abs(mm), "dpsi": dpsi,
                   "gyro": r.get("dyaw_gyro_deg", float("nan")), "v": r.get("v") or 0,
                   "coast": abs(moved) - (r.get("done") or 0)}
            if a.ruler:
                s = input("  尺子量的距离（mm，直接回车跳过）：").strip()
                row["ruler"] = float(s) if s else None
            rows.append(row)
            ruler = row.get("ruler")
            print(f"  指令 {mm:+6.0f}mm  里程 {moved:+7.1f}mm  误差 {row['err']:+6.1f}mm"
                  + (f"  尺子 {ruler:.1f}mm（误差 {ruler - abs(mm):+.1f}）" if ruler else "")
                  + f"  松油门速度 {row['v']:4.0f}mm/s  滑行 {row['coast']:5.1f}mm"
                  + f"  航向(右正) 编码器 {dpsi:+5.2f}° 陀螺仪 {row['gyro']:+5.2f}°"
                  + f"  → τ={c.coast_tau:.3f}s τ_PI={c.pi_tau:.3f}s  {c.voltage_mv / 1000:.2f}V")
            time.sleep(0.5)

        print("\n汇总：")
        for name, sel in (("前进", [r for r in rows if r["cmd"] > 0]), ("后退", [r for r in rows if r["cmd"] < 0])):
            errs = [r["err"] for r in sel]
            print(f"  {name}：里程误差 {' '.join(f'{e:+.1f}' for e in errs)}  "
                  f"平均 {statistics.mean(errs):+.1f}mm  最大 {max(abs(e) for e in errs):.1f}mm")
            rul = [r["ruler"] - abs(r["cmd"]) for r in sel if r.get("ruler")]
            if rul:
                print(f"        尺子误差 {' '.join(f'{e:+.1f}' for e in rul)}  平均 {statistics.mean(rul):+.1f}mm")
        fwd = [r for r in rows if r["cmd"] > 0]
        if fwd and not any(math.isnan(r["gyro"]) for r in fwd):
            per_m = statistics.mean(r["gyro"] / (abs(r["odo"]) / 1000.0) for r in fwd if abs(r["odo"]) > 20)
            per_m_enc = statistics.mean(r["dpsi"] / (abs(r["odo"]) / 1000.0) for r in fwd if abs(r["odo"]) > 20)
            print(f"  直走航向漂移：陀螺仪 {per_m:+.1f}°/m，编码器 {per_m_enc:+.1f}°/m（右转为正）")
            print(f"  → 如果多次测试陀螺仪漂移方向一致，可以试 CHASSIS_YAW_TRIM_DEG_PER_M = "
                  f"{cfg.CHASSIS_YAW_TRIM_DEG_PER_M + per_m:.1f}（在当前值上叠加）")
        print(f"  学到的 τ={c.coast_tau:.3f}s  τ_PI={c.pi_tau:.3f}s（存在 {tau_file}）")
        print("  把这段输出贴回 docs/cloud_tasks/results/03_报告.md 的真机验证一节")
    except KeyboardInterrupt:
        print("\n中断，停车")
    except DeviceError as e:
        print(f"出错：{e}")
        return 1
    finally:
        c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
