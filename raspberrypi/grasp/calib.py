#!/usr/bin/env python3
"""
标定小工具（在树莓派上运行，标定结果自动存进 calib.json，config.py 启动时自动加载）

只读：
  python3 calib.py watch               实时显示 K230 读数（X、Y、深度、方位角），Ctrl+C 退出
  python3 calib.py goal                取 30 帧中位数，打印 GOAL（问你要不要保存）
  python3 calib.py show                查看 calib.json 和最后一次发给机械臂的脉宽

会动：
  python3 calib.py pose observe        机械臂摆到某个姿态（底座回中；observe / ready 时爪子张开）
  python3 calib.py servo 4 714 [5 1900 ...]   直接设舵机脉宽
  python3 calib.py claw open|close     张爪 / 合爪
  python3 calib.py lift 15             升降轴走到 15cm
  python3 calib.py jog observe         键盘点动微调某个姿态，调好按 s 保存
  python3 calib.py auto                半自动：测升降轴方向/灵敏度、底座方向、相机半径（需要物块在画面里）
  python3 calib.py teach               引导式标定抓取位置：GOAL、REACH_NEAR_D（倒推法）
  python3 calib.py done                全部标定完：把 POSES_CALIBRATED 设为 true
  python3 calib.py shake [姿态]        摆到某个姿态（不写 = 原地不动）静止 6 秒，用 K230 读数量机械臂抖多厉害
"""
import json
import math
import os
import statistics
import sys
import time

import config as cfg
from devices import Arm, K230, Lift, RealClock

HERE = os.path.dirname(os.path.abspath(__file__))
STATE = os.path.join(HERE, cfg.LOG_DIR, "arm_state.json")


# ------------------------------------------------ calib.json / 机械臂状态
def load_calib():
    try:
        with open(cfg.CALIB_FILE, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_calib(updates):
    data = load_calib()
    for k, v in updates.items():
        if k in ("POSES", "AXES_J0") and isinstance(v, dict):
            data.setdefault(k, {}).update(v)
        else:
            data[k] = v
    tmp = cfg.CALIB_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, cfg.CALIB_FILE)
    print("已保存到 calib.json：", json.dumps(updates, ensure_ascii=False))


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return ""


def load_state():
    try:
        with open(STATE, encoding="utf-8") as f:
            return {int(k): v for k, v in json.load(f).items()}
    except (OSError, ValueError):
        return {}


def save_state(st):
    os.makedirs(os.path.dirname(STATE), exist_ok=True)
    with open(STATE, "w", encoding="utf-8") as f:
        json.dump({str(k): v for k, v in sorted(st.items())}, f)


_arm = None


def arm_move(pulses, ms=1000, quiet=False):
    global _arm
    if _arm is None:
        _arm = Arm(cfg.PORTS["arm"], cfg.BAUD["arm"], RealClock())
    st = load_state()
    _arm.move(pulses, ms)
    st.update({int(k): int(v) for k, v in pulses.items()})
    save_state(st)
    if not quiet:
        print("已发送：", {k: int(v) for k, v in sorted(pulses.items())})
    return st


def pose_pulses(name, base_pulse=None):
    pulses = {cfg.BASE_ID: base_pulse if base_pulse is not None else cfg.BASE_CENTER, **cfg.POSES[name]}
    if name == "observe":
        pulses[cfg.CLAW_ID] = cfg.observe_claw()
    elif name == "ready":
        pulses[cfg.CLAW_ID] = cfg.CLAW_OPEN
    return pulses


# ------------------------------------------------ K230
def open_cam():
    return K230(cfg.PORTS["k230"], cfg.BAUD["k230"], RealClock(), cfg.TARGET_TIMEOUT_S, cfg.K230_ROTATE)


def measure(cam, n=10, settle=0.6, timeout=4.0):
    """等 settle 秒画面稳定，再取 n 帧中位数；返回 (x, y, d, x_deg, 抖动) 或 None"""
    time.sleep(settle)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        fr = [f for f in cam.frames if f[0] >= t0]
        if len(fr) >= n:
            break
        time.sleep(0.05)
    fr = [f for f in cam.frames if f[0] >= t0][-n:]
    if len(fr) < max(3, n // 2):
        return None
    x = statistics.median(f[1] for f in fr)
    y = statistics.median(f[2] for f in fr)
    d = statistics.median(f[3] for f in fr)
    jit = statistics.pstdev(f[3] for f in fr)
    return x, y, d, math.degrees(math.atan2(x, max(d, 1))), jit


def fmt(m):
    return "看不到物块" if m is None else f"X={m[0]:+.1f}mm Y={m[1]:+.1f}mm 深度={m[2]:.1f}mm 方位角={m[3]:+.2f}°"


# ------------------------------------------------ 命令
def watch():
    cam = open_cam()
    print("把物块放到画面里，Ctrl+C 退出")
    try:
        while True:
            time.sleep(0.15)
            if not cam.link_ok():
                msg = "没收到任何数据：检查 K230 串口线"
            elif cam.visible():
                _, x, y, d = cam.frames[-1]
                xd = math.degrees(math.atan2(x, max(d, 1)))
                msg = f"X={x:+5d}mm  Y={y:+5d}mm  深度={d:4d}mm  方位角={xd:+5.1f}°"
            else:
                msg = "链路正常（心跳），没看到物块（或物块碰到画面边缘）"
            print("\r" + msg + " " * 10, end="", flush=True)
    except KeyboardInterrupt:
        print()


def goal(cam=None, offer_save=True):
    cam = cam or open_cam()
    print("保持物块不动，采集 30 帧……")
    m = measure(cam, n=30, settle=0.8, timeout=8)
    if m is None:
        print("帧数不够：物块没被识别到，或者碰到了画面边缘（太近）")
        return None
    x, y, d, xd, jit = m
    print(f"30 帧中位数：{fmt(m)}（深度抖动 ±{jit:.1f}mm）")
    g = {"x_deg": round(xd, 2), "y": round(y, 1), "d": round(d, 1)}
    print("GOAL =", g)
    if offer_save and ask("保存为 GOAL？[y/N] ") == "y":
        save_calib({"GOAL": g})
    return m


def jog(name):
    """键盘点动：<id> <+/-步长>、<id> =<脉宽>、o/c 张合爪、b <+/-度> 转底座、s 保存、q 退出"""
    if name not in cfg.POSES:
        sys.exit(f"没有这个姿态：{name}，可选 {list(cfg.POSES)}")
    cur = dict(pose_pulses(name))
    if name == "drop" and cfg.BASE_ID in cfg.POSES["drop"]:
        cur[cfg.BASE_ID] = cfg.POSES["drop"][cfg.BASE_ID]
    arm_move(cur, 1500)
    try:
        cam = open_cam()                   # 每动一下显示一次 K230 读数，边调边看
    except Exception as e:
        cam = None
        print("（K230 打不开，不显示读数：", e, "）")
    print("点动命令：'4 +20' 4 号舵机 +20us；'5 =1900' 直接设；'b +10' 底座转 10°；"
          "'o' 张爪；'c' 合爪；'s' 保存；'q' 退出；直接回车 = 只看读数")
    while True:
        if cam is not None:
            print("   K230：", fmt(measure(cam, n=4, settle=0.5, timeout=2.0)))
        cmd = ask(f"[{name}] {dict(sorted(cur.items()))} > ")
        if cmd == "":
            continue
        if cmd in ("q", "quit", "exit"):
            break
        if cmd == "s":
            keep = {k: v for k, v in cur.items()
                    if k != cfg.CLAW_ID and (k != cfg.BASE_ID or name == "drop")}
            save_calib({"POSES": {name: keep}})
            continue
        if cmd in ("o", "c"):
            cur[cfg.CLAW_ID] = cfg.CLAW_OPEN if cmd == "o" else cfg.CLAW_CLOSE
            arm_move({cfg.CLAW_ID: cur[cfg.CLAW_ID]}, 500, quiet=True)
            continue
        parts = cmd.split()
        try:
            if parts[0] == "b":
                sid, newp = cfg.BASE_ID, cur[cfg.BASE_ID] + float(parts[1]) * cfg.BASE_US_PER_DEG * cfg.BASE_DIR
            else:
                sid = int(parts[0])
                newp = float(parts[1][1:]) if parts[1].startswith("=") else cur.get(sid, 1500) + float(parts[1])
        except (IndexError, ValueError, KeyError):
            print("看不懂，例子：'4 +20'、'5 =1900'、'b -10'、'o'、'c'、's'、'q'")
            continue
        newp = int(max(500, min(2500, newp)))
        if not 1 <= sid <= 6:
            print("舵机 id 只能是 1~6")
            continue
        cur[sid] = newp
        arm_move({sid: newp}, 400, quiet=True)


def auto():
    """
    半自动标定（机械臂会动一点、升降轴会动 3cm）：
      1. 升降轴上/下 3cm，看 Y 怎么变 → AXES_J0["y"]（方向和灵敏度）
      2. 底座转 +10°（脉宽 +111），看方位角怎么变 → BASE_DIR 和 R_CAM_MM
    """
    print("把物块放在机械臂正前方 20~40cm、画面中间附近，机械臂摆到 observe 姿态")
    if ask("准备好了按回车（q 退出）") == "q":
        return
    cam = open_cam()
    lift = Lift(cfg.PORTS["lift"], cfg.BAUD["lift"], cfg.LIFT_MAX_CM)
    arm_move(pose_pulses("observe"), 1500)
    m0 = measure(cam)
    print("起始：", fmt(m0))
    if m0 is None:
        sys.exit("看不到物块，调整物块位置后重来")
    result = {}

    # ---- 升降轴
    h0 = lift.position()
    dh = 3.0 if h0 + 3.0 <= cfg.LIFT_MAX_CM - 1 else -3.0
    lift.move_to(h0 + dh)
    m1 = measure(cam)
    lift.move_to(h0)
    print(f"升降轴 {dh:+.0f}cm 后：", fmt(m1))
    if m1 is not None:
        jy = (m1[1] - m0[1]) / dh
        print(f"  → 升降轴每上升 1cm，Y 变化 {jy:+.2f}mm（预期约 +10，相机跟着升降台走）")
        if abs(jy) < 2:
            print("  ⚠ 变化太小：相机可能没跟着升降轴动，或者 Y 被别的因素掩盖，先不保存")
        else:
            result["AXES_J0"] = {"y": round(jy, 2)}
    else:
        print("  ⚠ 移动后看不到物块了，跳过升降轴这一项")

    # ---- 底座
    step_us = int(round(10 * cfg.BASE_US_PER_DEG))
    arm_move({cfg.BASE_ID: cfg.BASE_CENTER + step_us}, 600)
    m2 = measure(cam)
    if m2 is None:                                   # 转出画面了，换小一点的角度
        step_us //= 2
        arm_move({cfg.BASE_ID: cfg.BASE_CENTER + step_us}, 400)
        m2 = measure(cam)
    arm_move({cfg.BASE_ID: cfg.BASE_CENTER}, 600)
    deg = step_us / cfg.BASE_US_PER_DEG
    print(f"底座脉宽 +{step_us}us（约 {deg:.0f}°）后：", fmt(m2))
    if m2 is not None:
        dx = m2[3] - m0[3]
        base_dir = 1 if dx < 0 else -1               # 底座右转 → 物块方位角减小
        ratio = abs(dx) / deg                        # = 1 + R/d
        r_est = max(0.0, min(400.0, (ratio - 1.0) * m0[2]))
        print(f"  → 方位角变化 {dx:+.2f}°：脉宽增大是{'右' if base_dir == 1 else '左'}转，BASE_DIR = {base_dir}")
        print(f"  → 方位角/转角 = {ratio:.2f}，估算相机到底座转轴 R_CAM_MM ≈ {r_est:.0f}mm（配置里是 {cfg.R_CAM_MM:.0f}）")
        result["BASE_DIR"] = base_dir
        if 1.05 <= ratio <= 4:
            result["R_CAM_MM"] = round(r_est)
        elif 0.4 <= ratio < 1.05:
            # 比值 ≤ 1：这个姿态下相机几乎在底座转轴上（甚至稍靠后，机械臂收拢时常见）。
            # 半径按 0 算，转 1° 物块偏 ratio 度 → 直接存成底座的灵敏度初值
            print(f"  → 相机基本在底座转轴上：R_CAM_MM 记为 0，底座灵敏度 j0 = {-ratio:.2f}")
            result["R_CAM_MM"] = 0
            result.setdefault("AXES_J0", {})["x_deg"] = round(-ratio, 3)
        else:
            print("  ⚠ 比值不合理（底座可能没转到位 / 物块出了画面），R_CAM_MM 先不保存")
    else:
        print("  ⚠ 转动后看不到物块了，跳过底座这一项")

    if result and ask(f"保存这些结果 {result}？[y/N] ") == "y":
        save_calib(result)


def teach():
    """倒推法标定抓取位置：先把物块放进爪子，再退回观察姿态看相机读数"""
    cam = open_cam()
    for which in ("reach_near", "reach_far"):
        if which == "reach_far" and ask("要不要再标定一个更远的抓取姿态 reach_far（用于插值）？[y/N] ") != "y":
            near = load_calib().get("REACH_NEAR_D", cfg.REACH_NEAR_D)
            save_calib({"POSES": {"reach_far": cfg.POSES["reach_near"]}, "REACH_FAR_D": near + 70})
            print("reach_far 设成和 reach_near 一样（不插值）")
            break
        print(f"\n===== 标定 {which} =====")
        print("1) 机械臂伸到抓取姿态、爪子张开")
        arm_move({**pose_pulses(which), cfg.CLAW_ID: cfg.CLAW_OPEN}, 1500)
        while True:
            a = ask("2) 把物块放在爪子正中间（放在桌面上）。回车=合爪试夹，j=先微调姿态，q=退出 ")
            if a == "q":
                return
            if a == "j":
                jog(which)
                arm_move({**pose_pulses(which), cfg.CLAW_ID: cfg.CLAW_OPEN}, 800)
                continue
            arm_move({cfg.CLAW_ID: cfg.CLAW_CLOSE}, 600)
            if ask("3) 夹稳了吗？[y=夹稳了 / n=没夹住，调整后重来] ") == "y":
                break
            arm_move({cfg.CLAW_ID: cfg.CLAW_OPEN}, 600)
        print("4) 松爪，物块留在原地不要动；机械臂退回 observe 姿态")
        arm_move({cfg.CLAW_ID: cfg.CLAW_OPEN}, 600)
        arm_move(pose_pulses("ready"), 1000)
        arm_move(pose_pulses("observe"), 1200)
        m = goal(cam, offer_save=False)
        if m is None:
            print("相机看不到物块：observe 姿态离物块太近/角度不对。用 `calib.py jog observe` 把相机调得更往后、往上，再重来")
            return
        x, y, d, xd, _ = m
        upd = {"REACH_NEAR_D" if which == "reach_near" else "REACH_FAR_D": round(d)}
        g_here = {"x_deg": round(xd, 2), "y": round(y, 1), "d": round(d, 1)}
        upd["GOAL" if which == "reach_near" else "GOAL_FAR"] = g_here
        if ask(f"保存 {upd}？[y/N] ") == "y":
            save_calib(upd)


def shake(name, seconds=6.0):
    """
    抖动测量：相机在腕部，机械臂一抖，静止物块的读数就跟着抖。
    摆到某个姿态、静止 seconds 秒，统计 K230 读数的波动（物块要放着别动）。
    """
    cam = open_cam()
    if name != "now":
        arm_move(pose_pulses(name), 1500)
        time.sleep(1.0)                      # 等动作完全结束
    print(f"保持 {seconds:.0f} 秒不动，测量中……")
    t0 = time.monotonic()
    time.sleep(seconds)
    fr = [f for f in cam.frames if f[0] >= t0]
    if len(fr) < 5:
        print(f"只收到 {len(fr)} 帧：物块没被识别到 / 碰到画面边缘，或者抖得太厉害一直丢目标")
        return None
    xs = [f[1] for f in fr]
    ys = [f[2] for f in fr]
    ds = [f[3] for f in fr]
    sx, sy, sd = statistics.pstdev(xs), statistics.pstdev(ys), statistics.pstdev(ds)
    print(f"{len(fr)} 帧：X 标准差 {sx:.1f}mm（峰峰 {max(xs) - min(xs)}）  "
          f"Y 标准差 {sy:.1f}mm（峰峰 {max(ys) - min(ys)}）  深度标准差 {sd:.1f}mm")
    worst = max(sx, sy)
    if worst < 1.5:
        print("→ 基本不抖（这个量级是 K230 自身的测量噪声）")
    elif worst < 4:
        print("→ 轻微抖动")
    else:
        print("→ 明显抖动：先查舵机供电（电压、电流够不够、地线），再看这个姿态是不是顶到了机械限位")
    return worst


def main(argv):
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return
    cmd, args = argv[0], argv[1:]
    if cmd == "watch":
        watch()
    elif cmd == "goal":
        goal()
    elif cmd == "show":
        print("calib.json：", json.dumps(load_calib(), ensure_ascii=False, indent=2))
        print("最后一次发给机械臂的脉宽：", dict(sorted(load_state().items())))
    elif cmd == "pose":
        arm_move(pose_pulses(args[0]), int(args[1]) if len(args) > 1 else 1500)
    elif cmd == "servo":
        if len(args) % 2 or not args:
            sys.exit("用法：calib.py servo <id> <脉宽> [<id> <脉宽> ...]")
        pulses = {int(args[i]): int(args[i + 1]) for i in range(0, len(args), 2)}
        for sid, p in pulses.items():
            if not 1 <= sid <= 6 or not 500 <= p <= 2500:
                sys.exit(f"舵机 id 要在 1~6、脉宽要在 500~2500：{sid} {p}")
        arm_move(pulses, 600)
    elif cmd == "claw":
        arm_move({cfg.CLAW_ID: cfg.CLAW_OPEN if args[0] == "open" else cfg.CLAW_CLOSE}, 500)
    elif cmd == "lift":
        lift = Lift(cfg.PORTS["lift"], cfg.BAUD["lift"], cfg.LIFT_MAX_CM)
        print("升降轴到达：", lift.move_to(float(args[0])), "cm")
    elif cmd == "jog":
        jog(args[0] if args else "observe")
    elif cmd == "auto":
        auto()
    elif cmd == "teach":
        teach()
    elif cmd == "done":
        save_calib({"POSES_CALIBRATED": True})
    elif cmd == "shake":
        shake(args[0] if args else "now", float(args[1]) if len(args) > 1 else 6.0)
    else:
        print(__doc__)


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print("\n已退出")
    except Exception as e:        # 设备问题给一句人话，不甩一大段堆栈
        if type(e).__name__ in ("DeviceError", "SerialException"):
            sys.exit(f"设备错误：{e}")
        raise
