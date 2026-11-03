#!/usr/bin/env python3
"""
底盘真机测试（任务 03 停车精度 + 任务 18 联调预演补全）。分步操作见 docs/底盘联调手册.md

  python3 chassis_test.py --voltage            只读：电压、flag、陀螺仪读数（车不会动）
  python3 chassis_test.py --hang               【车架空、轮子离地】确认三种"停"：指令过期、close()、收到 SIGTERM
  python3 chassis_test.py --timeout-check      【车架空、轮子离地】不经过驱动，验证固件有没有串口超时
  python3 chassis_test.py --ruler              直走精度：±100mm、±200mm 各 3 次，每次输入尺子量的距离
  python3 chassis_test.py --ruler --write      同上，拟合滑行 τ 写进 calib.json（CHASSIS_PI_TAU_S）和 logs/chassis_tau.json
  python3 chassis_test.py --radius             打满方向左、右各走一段弧，输入量的弦长和侧移 → 最小转弯半径
  python3 chassis_test.py --radius --write     同上，写进 calib.json（ACK_MIN_RADIUS_MM）

  其他：--dist 100,200 --n 3 --speed 100 --model / --tau（松油门方式）--fresh（从默认 τ 开始学）

说明：
  - 精度测试学到的 τ 存在 logs/chassis_tau_test.json，不影响 main.py 用的 logs/chassis_tau.json（--write 才写）
  - 开机后 10 秒内固件不接收串口指令（源码 usartx.c:653 CONTROL_DELAY），脚本会自己等
  - 每次走完打印两种航向：编码器差速（回传 wz）和陀螺仪（回传角速度 Z，按 ±500°/s 量程换算，未验证）
    直走时如果陀螺仪航向总往一边偏，把打印的建议值填进 config.py 的 CHASSIS_YAW_TRIM_DEG_PER_M
  - 随时 Ctrl+C 停车；SIGTERM / SSH 断开也会先停车（kill -9 和断电拦不住：手放在底盘电源开关旁）
"""
import argparse
import json
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
from main import chassis_options, install_stop_signals  # noqa: E402

CALIB_WRITES = ("CHASSIS_PI_TAU_S", "ACK_MIN_RADIUS_MM")   # 这个脚本会写进 calib.json 的参数（都在 CALIB_KEYS 里）
ODOM_SCALE_TOL = 0.03       # 尺子 / 里程 的比例偏离 1 超过 3%：里程计本身不准，τ 拟合不可信


# ------------------------------------------------------------ 纯计算（selftest / test_chassis_rehearsal 会测）
def fit_coast_tau(rows):
    """
    用尺子读数拟合滑行时间常数：松油门后真实滑行 = 尺子 − 松油门时的里程，τ = 滑行 / 松油门速度。
    rows 每项要有 ruler、done、v（松油门速度 mm/s）。返回 (τ 平均, 标准差)；有效数据 < 3 次返回 (None, None)
    """
    t = [(abs(r["ruler"]) - r["done"]) / r["v"] for r in rows
         if r.get("ruler") is not None and r.get("done") is not None and (r.get("v") or 0) > 20]
    if len(t) < 3:
        return None, None
    return statistics.mean(t), statistics.stdev(t)


def odom_scale(rows):
    """尺子 / 里程 的最小二乘比例（1 = 里程计准）；没有尺子数据返回 None"""
    p = [(abs(r["ruler"]), abs(r["odo"])) for r in rows if r.get("ruler") is not None and abs(r.get("odo", 0)) > 20]
    if not p:
        return None
    return sum(a * b for a, b in p) / sum(b * b for _, b in p)


def radius_from_chord(chord_mm, lateral_mm):
    """
    后轴中点走了一段圆弧：起点到终点的直线距离（弦长）c、终点偏离原来直线的侧移 h → R = c² / (2h)
    （起点切线方向为 x：终点 (R sinθ, R(1−cosθ))，c² = 2R·h）
    """
    h = abs(lateral_mm)
    return chord_mm * chord_mm / (2.0 * h) if h > 1 else float("inf")


def radius_from_heading(arc_mm, dpsi_deg):
    """里程（弧长）+ 航向变化 → 半径"""
    a = math.radians(abs(dpsi_deg))
    return abs(arc_mm) / a if a > 1e-6 else float("inf")


def pi_tau_from_coast(tau, latency_s, voltage_v, ref_voltage_v):
    """把实测的总滑行 τ 换算成 model 模式的 τ_PI（扣掉指令延迟，再折算到参考电压）"""
    return max(0.02, (tau - latency_s) * voltage_v / ref_voltage_v)


def write_calib(updates, path=None):
    """只改这几项，保留 calib.json 里原有的标定"""
    path = path or cfg.CALIB_FILE
    bad = [k for k in updates if k not in cfg.CALIB_KEYS]
    if bad:
        raise ValueError(f"{bad} 不在 CALIB_KEYS 白名单里")
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        data = {}
    data.update(updates)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)
    print("已写入", path, "：", json.dumps(updates, ensure_ascii=False))


# ------------------------------------------------------------ 串口工具
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


class RawReader:
    """不经过驱动，直接从串口读回传帧的 vx（驱动 close() 之后它的接收线程已经停了）"""

    def __init__(self, ser):
        self.ser, self.buf = ser, bytearray()

    def vx(self):
        self.buf += self.ser.read(256)
        vx = None
        while True:
            i = self.buf.find(b"\x7B")
            if i < 0 or len(self.buf) - i < 24:
                del self.buf[:max(0, i)]
                return vx
            f = bytes(self.buf[i:i + 24])
            b = 0
            for x in f[:22]:
                b ^= x
            if f[23] == 0x7D and b == f[22]:
                vx = int.from_bytes(f[2:4], "big", signed=True)
                del self.buf[:i + 24]
            else:
                del self.buf[:i + 1]

    def watch(self, seconds):
        t0, seen = time.monotonic(), []
        while time.monotonic() - t0 < seconds:
            v = self.vx()
            if v is not None:
                seen.append((time.monotonic() - t0, v))
            time.sleep(0.02)
        return seen


def wheels_off_ground():
    print("⚠ 这个测试会让车轮转起来！必须先把车架空（垫块垫在底盘下面）、四个轮子都离地，手放在底盘电源开关旁。")
    return input("轮子已经离地？输入 yes 继续：").strip() == "yes"


def timeout_check(port, baud):
    """不经过驱动，直接发帧：先以 50mm/s 发 1 秒，然后停发 1.5 秒，看回传速度是不是还在"""
    import serial
    print("（这个测试会让车轮在没有指令的情况下继续转）")
    if not wheels_off_ground():
        return 1
    ser = serial.Serial(port, baud, timeout=0.02)
    rd = RawReader(ser)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            ser.write(WheeltecChassis.frame(50, 0))
            rd.vx()
            time.sleep(0.02)
        print("停止发送，观察 1.5 秒内回传速度（mm/s）：")
        seen = rd.watch(1.5)
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
    return 0


def open_chassis(speed, tau_file, opts=None):
    return WheeltecChassis(cfg.PORTS["chassis"], cfg.BAUD["chassis"], speed, cfg.ACK_MIN_RADIUS_MM,
                           cfg.CHASSIS_ACKERMANN, tau_file, **(opts or chassis_options(cfg)))


def print_status(c):
    print(f"底盘电压 {c.voltage_mv / 1000:.2f}V（下限 {cfg.BATTERY_MIN_V}V）  flag={c.fb_flag}  "
          f"松油门方式={c.stop_model}  τ={c.coast_tau:.3f}s  τ_PI={c.pi_tau:.3f}s  "
          f"最小半径(config)={cfg.ACK_MIN_RADIUS_MM:.0f}mm  yaw_trim={cfg.CHASSIS_YAW_TRIM_DEG_PER_M}°/m")
    if c.fb_flag:
        print("⚠ flag=1（源码里表示电机失能，开机 10 秒内是这样），车可能不会动")
    if c.voltage_mv and c.voltage_mv / 1000 < cfg.BATTERY_MIN_V:
        print(f"⚠ 电压低于 {cfg.BATTERY_MIN_V}V：先充电（固件对 24V 电池没有有效的低压保护）")


# ------------------------------------------------------------ 只读：电压
def voltage(seconds=3.0):
    c = open_chassis(cfg.CHASSIS_SPEED_MM_S, None)
    try:
        if not wait_ready(c, 12.0):
            print("底盘没有回传：检查 USB 线（/dev/ttyACM0）、底盘电源开关")
            return 1
        print_status(c)
        t0 = time.monotonic()
        vs = []
        while time.monotonic() - t0 < seconds:
            vs.append(c.voltage_mv / 1000)
            print(f"  {time.monotonic() - t0:4.1f}s  电压 {vs[-1]:.2f}V  vx={c.fb_vx}  wz={c.fb_wz}  "
                  f"陀螺仪Z原始={c.gyro_z_raw}  flag={c.fb_flag}")
            time.sleep(0.5)
        print(f"电压 {min(vs):.2f}~{max(vs):.2f}V；静止时 vx、wz 应为 0，陀螺仪Z 应在 ±30 以内（零偏）")
    finally:
        c.close()
    return 0


# ------------------------------------------------------------ 架空测试
def hang_test():
    """
    车架空：确认驱动的三种"停"都管用（固件自己不会停）：
      A 指令过期：只发一次速度、不再刷新 → CHASSIS_CMD_TTL_S 后驱动自己改发 0
      B close()：程序正常退出时直接写停车帧
      C SIGTERM：systemctl stop / kill（不带 -9）时也先停车（任务 18 加的信号处理）
    """
    if not wheels_off_ground():
        return 1
    results = []
    c = open_chassis(cfg.CHASSIS_SPEED_MM_S, None)
    try:
        if not wait_ready(c):
            print("底盘没有回传：检查 USB 线、底盘电源开关")
            return 1
        print_status(c)
        print(f"\nA 指令过期：发一次 80mm/s 后不再刷新，{c.cmd_ttl:.1f}s 后驱动应自动停")
        c._set(80, 0)
        t0, peak, end = time.monotonic(), 0, None
        while time.monotonic() - t0 < c.cmd_ttl + 1.5:
            peak = max(peak, abs(c.fb_vx))
            end = c.fb_vx
            time.sleep(0.05)
        ok = peak > 20 and abs(end) < 10
        print(f"  最高 {peak}mm/s，{c.cmd_ttl + 1.5:.1f}s 后 {end}mm/s，过期停车 {c.ttl_tripped} 次 → {'✅' if ok else '❌'}"
              + ("" if peak > 20 else "（轮子没转起来：flag？使能开关？）"))
        results.append(("指令过期自动停", ok))

        print("\nB close()：以 80mm/s 持续发 1 秒，然后 close()，看 1.5 秒内回传速度")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            c._set(80, 0)
            time.sleep(0.02)
        c.close()
        tail = [v for t, v in RawReader(c.ser).watch(1.5) if t > 1.0]
        ok = bool(tail) and max(abs(v) for v in tail) < 10
        print(f"  close() 1 秒后回传速度 {tail[-3:] if tail else '没收到'} → {'✅' if ok else '❌'}")
        results.append(("close() 停车", ok))
    finally:
        c.close()
        try:
            c.ser.close()
        except Exception:
            pass

    print("\nC SIGTERM：子进程以 80mm/s 持续发送，1.5 秒后对它发 SIGTERM，看车轮停不停")
    code = ("import sys,time; sys.path.insert(0,%r); import main, chassis_test as ct; main.install_stop_signals(); "
            "c=ct.open_chassis(100,None)\ntry:\n  t0=time.monotonic()\n  while time.monotonic()-t0<10: "
            "c._set(80,0); time.sleep(0.02)\nexcept KeyboardInterrupt:\n  pass\nfinally:\n  c.close()\n" % HERE)
    p = subprocess.Popen([sys.executable, "-c", code])
    time.sleep(1.8)
    p.terminate()
    p.wait(5)
    import serial
    ser = serial.Serial(cfg.PORTS["chassis"], cfg.BAUD["chassis"], timeout=0.02)
    try:
        seen = RawReader(ser).watch(1.5)
    finally:
        for _ in range(3):
            ser.write(WheeltecChassis.frame(0, 0))
            time.sleep(0.02)
        ser.close()
    tail = [v for t, v in seen if t > 0.8]
    ok = bool(tail) and max(abs(v) for v in tail) < 10
    print(f"  SIGTERM 后回传速度 {tail[-3:] if tail else '没收到'} → {'✅' if ok else '❌'}")
    results.append(("SIGTERM 停车", ok))

    print("\n架空测试：" + "  ".join(f"{'✅' if ok else '❌'} {n}" for n, ok in results))
    if not all(ok for _, ok in results):
        print("有一项没通过：不要让车下地自动跑，把输出贴给云端分析")
        return 1
    print("全部通过。再试一次 kill -9（可选）：车轮会一直转，只能拨使能开关 / 断电 —— 确认你知道开关在哪")
    return 0


# ------------------------------------------------------------ 直走精度
def accuracy(a, opts):
    tau_file = os.path.join(HERE, cfg.LOG_DIR, "chassis_tau_test.json")
    if a.fresh and os.path.exists(tau_file):
        os.remove(tau_file)
    dists = [float(x) for x in str(a.dist).split(",") if x.strip()]
    c = open_chassis(a.speed, tau_file, opts)
    rows = []
    try:
        if not wait_ready(c):
            print("底盘没有回传：检查 USB 线、底盘电源开关")
            return 1
        print_status(c)
        if a.ruler:
            print("量法：地上贴一条胶带当起点线，在车身上选一个固定点（例如后轴中点正下方）对准它；"
                  "每次走完量这个点到起点线的距离，然后把车推回起点（或者以新位置为起点）")
        input(f"车前后各留 {max(dists) + 200:.0f}mm 空地，回车开始（Ctrl+C 随时停车）")
        seq = []
        for d in dists:
            seq += [d] * a.n + [-d] * a.n
        for mm in seq:
            moved, dpsi = c.drive(mm)
            r = c.last_release or {}
            row = {"cmd": mm, "odo": moved, "err": abs(moved) - abs(mm), "dpsi": dpsi,
                   "gyro": r.get("dyaw_gyro_deg", float("nan")), "v": r.get("v") or 0, "done": r.get("done"),
                   "coast": abs(moved) - (r.get("done") or 0), "volt": c.voltage_mv / 1000}
            if a.ruler:
                s = input("  尺子量的距离（mm，正数；直接回车跳过）：").strip()
                row["ruler"] = abs(float(s)) if s else None
            rows.append(row)
            ruler = row.get("ruler")
            print(f"  指令 {mm:+6.0f}mm  里程 {moved:+7.1f}mm  误差 {row['err']:+6.1f}mm"
                  + (f"  尺子 {ruler:.1f}mm（误差 {ruler - abs(mm):+.1f}）" if ruler else "")
                  + f"  松油门速度 {row['v']:4.0f}mm/s  滑行 {row['coast']:5.1f}mm"
                  + f"  航向(右正) 编码器 {dpsi:+5.2f}° 陀螺仪 {row['gyro']:+5.2f}°"
                  + f"  → τ={c.coast_tau:.3f}s τ_PI={c.pi_tau:.3f}s  {c.voltage_mv / 1000:.2f}V")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n中断，停车")
    except DeviceError as e:
        print(f"出错：{e}")
        return 1
    finally:
        c.close()
    if not rows:
        return 1

    print("\n汇总：")
    for d in dists:
        for name, sel in (("前进", [r for r in rows if r["cmd"] == d]), ("后退", [r for r in rows if r["cmd"] == -d])):
            if not sel:
                continue
            errs = [r["err"] for r in sel]
            line = (f"  {name} {d:.0f}mm：里程误差 {' '.join(f'{e:+.1f}' for e in errs)}  "
                    f"平均 {statistics.mean(errs):+.1f}  最大 {max(abs(e) for e in errs):.1f}mm")
            rul = [r["ruler"] - abs(r["cmd"]) for r in sel if r.get("ruler")]
            if rul:
                line += f"   尺子误差 {' '.join(f'{e:+.1f}' for e in rul)}  平均 {statistics.mean(rul):+.1f}mm"
            print(line)
    fwd = [r for r in rows if r["cmd"] > 0 and abs(r["odo"]) > 20]
    if fwd and not any(math.isnan(r["gyro"]) for r in fwd):
        per_m = statistics.mean(r["gyro"] / (abs(r["odo"]) / 1000.0) for r in fwd)
        per_m_enc = statistics.mean(r["dpsi"] / (abs(r["odo"]) / 1000.0) for r in fwd)
        print(f"  直走航向漂移：陀螺仪 {per_m:+.1f}°/m，编码器 {per_m_enc:+.1f}°/m（右转为正）")
        print(f"  → 如果多次测试陀螺仪漂移方向一致，可以试 CHASSIS_YAW_TRIM_DEG_PER_M = "
              f"{cfg.CHASSIS_YAW_TRIM_DEG_PER_M + per_m:.1f}（在当前值上叠加）")
    print(f"  驱动自己按里程学到的 τ={c.coast_tau:.3f}s  τ_PI={c.pi_tau:.3f}s（存在 {tau_file}）")

    k = odom_scale(rows)
    tau, sd = fit_coast_tau(rows)
    if k is None:
        print("  没有尺子读数：只能看里程误差（加 --ruler 才能拟合）")
        return 0
    print(f"  尺子 / 里程 = {k:.3f}" + ("（里程计准）" if abs(k - 1) <= ODOM_SCALE_TOL else
                                     f"（偏 {100 * (k - 1):+.1f}%：固件轮径/编码器参数和真车不一致。对准是视觉闭环，"
                                     f"只会多走一两步；几何前馈会按这个比例偏）"))
    if tau is None:
        print("  尺子读数少于 3 次，不拟合 τ")
        return 0
    volt = statistics.mean(r["volt"] for r in rows if r["volt"] > 5) if any(r["volt"] > 5 for r in rows) \
        else cfg.CHASSIS_REF_VOLTAGE_V
    pi = pi_tau_from_coast(tau, cfg.CHASSIS_CMD_LATENCY_S, volt, cfg.CHASSIS_REF_VOLTAGE_V)
    print(f"  尺子拟合：滑行 τ = {tau:.3f}s ±{sd:.3f}（{volt:.1f}V）→ τ_PI = {pi:.3f}s（参考 {cfg.CHASSIS_REF_VOLTAGE_V}V）")
    trusted = abs(k - 1) <= ODOM_SCALE_TOL and sd < 0.1
    if a.write:
        if not trusted:
            print("  ⚠ 里程计比例偏离 1 超过 3% 或 τ 离散太大：τ 用驱动按里程学到的值写入（尺子拟合不可信）")
            tau, pi = c.coast_tau, c.pi_tau
        write_calib({"CHASSIS_PI_TAU_S": round(pi, 3)})
        main_tau = os.path.join(HERE, cfg.LOG_DIR, "chassis_tau.json")
        os.makedirs(os.path.dirname(main_tau), exist_ok=True)
        with open(main_tau, "w", encoding="utf-8") as f:     # main.py 启动时读这个文件（会覆盖 calib 里的初值）
            json.dump({"coast_tau": round(tau, 3), "pi_tau": round(pi, 3)}, f)
        print(f"  已写入 {main_tau}：coast_tau={tau:.3f} pi_tau={pi:.3f}")
    else:
        print("  加 --write 把这个结果写进 calib.json 和 logs/chassis_tau.json")
    print("  把这段输出贴回 docs/底盘联调手册.md 第 3 步的记录表")
    return 0


# ------------------------------------------------------------ 最小转弯半径
def radius_test(a, opts):
    L = a.arc
    c = open_chassis(a.speed, None, opts)
    res, by_ruler = {}, True
    try:
        if not wait_ready(c):
            print("底盘没有回传：检查 USB 线、底盘电源开关")
            return 1
        print_status(c)
        print(f"打满方向走一段 {L:.0f}mm 的弧（固件会把转角截到它自己的最小半径）。\n"
              "量法：在后轴中点正下方的地面上做记号（或者两个后轮着地点各做记号，取中点）。走完以后量：\n"
              "  弦长 = 起点记号到终点记号的直线距离；侧移 = 终点记号离'起点处车头方向那条直线'的垂直距离")
        input(f"车左右各留 {L:.0f}mm、前方留 {L + 200:.0f}mm 空地，回车开始（Ctrl+C 随时停车）")
        c.min_radius = 150.0          # 只在这个测试里放开驱动的截断，让固件按自己的最大转角打满
        for side, sgn in (("左", -1), ("右", 1)):
            input(f"\n把车摆回起点，{side}转打满，回车开始")
            moved, dpsi = c.drive(L, sgn * math.degrees(L / c.min_radius))
            g = (c.last_release or {}).get("dyaw_gyro_deg", float("nan"))
            r_gyro = radius_from_heading(moved, g) if not math.isnan(g) else float("nan")
            print(f"  里程 {moved:.0f}mm  航向 编码器 {dpsi:+.1f}° 陀螺仪 {g:+.1f}° → 陀螺仪推算半径 {r_gyro:.0f}mm")
            s = input("  量的弦长和侧移（mm，空格分开；直接回车 = 用陀螺仪推算）：").split()
            if len(s) >= 2:
                r = radius_from_chord(float(s[0]), float(s[1]))
                print(f"  → {side}转最小半径 {r:.0f}mm（尺子）")
            else:
                r, by_ruler = r_gyro, False
                print(f"  → {side}转最小半径 {r:.0f}mm（陀螺仪，未验证量程，仅参考）")
            res[side] = r
    except KeyboardInterrupt:
        print("\n中断，停车")
    except DeviceError as e:
        print(f"出错：{e}")
        return 1
    finally:
        c.close()
    vals = [v for v in res.values() if v and not math.isnan(v) and v != float("inf")]
    if not vals:
        return 1
    rec = math.ceil(max(vals) * 1.05 / 10.0) * 10
    print(f"\n左 {res.get('左', float('nan')):.0f}mm / 右 {res.get('右', float('nan')):.0f}mm；"
          f"建议 ACK_MIN_RADIUS_MM = {rec}（取大的一边 +5%，现在是 {cfg.ACK_MIN_RADIUS_MM:.0f}）")
    print("  比真实值大是安全的（驱动不会要求固件做不到的弯）；比真实值小，几何前馈会按实测转角补，也不危险，只是多一步")
    if a.write and not by_ruler:
        print("  ⚠ 有一边是陀螺仪推算的（量程未验证），不写 calib.json；用尺子量了再 --write")
    elif a.write:
        write_calib({"ACK_MIN_RADIUS_MM": float(rec)})
    else:
        print("  加 --write 写进 calib.json")
    return 0


def main():
    ap = argparse.ArgumentParser(description="底盘真机测试（分步说明见 docs/底盘联调手册.md）")
    ap.add_argument("--dist", default="100,200", help="直走距离（mm），逗号分开，默认 100,200")
    ap.add_argument("--n", type=int, default=3, help="每个距离前进、后退各几次，默认 3")
    ap.add_argument("--speed", type=float, default=cfg.CHASSIS_SPEED_MM_S, help="速度 mm/s")
    ap.add_argument("--model", action="store_true", help="用模型预测松油门（默认按 config.py）")
    ap.add_argument("--tau", action="store_true", help="用原来的自适应 τ（默认按 config.py）")
    ap.add_argument("--ruler", action="store_true", help="每次走完输入尺子量的距离")
    ap.add_argument("--write", action="store_true", help="把拟合结果写进 calib.json")
    ap.add_argument("--fresh", action="store_true", help="不读上次学到的 τ，从默认值开始学")
    ap.add_argument("--voltage", action="store_true", help="只读：电压 / flag / 陀螺仪")
    ap.add_argument("--hang", action="store_true", help="架空测试：三种停车方式（轮子必须离地）")
    ap.add_argument("--timeout-check", action="store_true", help="验证固件有没有串口超时（轮子必须离地）")
    ap.add_argument("--radius", action="store_true", help="打满方向测最小转弯半径")
    ap.add_argument("--arc", type=float, default=600.0, help="--radius 每边走多远（mm），默认 600")
    a = ap.parse_args()

    if manual_running():
        print(f"手柄程序 {cfg.MANUAL_PROCESS} 在运行，会抢底盘串口。先停掉它。")
        return 1
    install_stop_signals()
    if a.timeout_check:
        return timeout_check(cfg.PORTS["chassis"], cfg.BAUD["chassis"])
    if a.voltage:
        return voltage()
    if a.hang:
        return hang_test()
    opts = chassis_options(cfg)
    if a.model:
        opts["stop_model"] = "model"
    if a.tau:
        opts["stop_model"] = "tau"
    if a.radius:
        return radius_test(a, opts)
    return accuracy(a, opts)


if __name__ == "__main__":
    sys.exit(main())
