#!/usr/bin/env python3
"""
比赛演示模式（任务 16）：连续抓取，终端输出适合拍视频的大号状态行，结束时打印统计

  python3 demo.py --no-chassis --rounds 5               # 真机：每轮之间等操作员把物块放回去、按回车
  python3 demo.py --no-chassis --rounds 10 --auto 3     # 真机：多个物块依次抓，每轮之间自动等 3 秒
  python3 demo.py --no-chassis --dashboard --record     # 同时开网页看板、记录数据
  python3 demo.py --sim 3                               # 仿真跑 3 轮（不需要硬件）
  python3 demo.py --sim 3 --dashboard --realtime 1      # 电脑上按真实速度仿真，浏览器看看板效果

  任何时候 Ctrl+C：升降轴急停、底盘停车，然后打印本次统计。
  语音播报：config.DEMO_TTS_CMD（默认 None 不播报），例如 "espeak-ng -v zh {text}"

抓取逻辑全部来自 main.py（open_devices / preflight / attach_task）和 task.GraspTask.run_once，
这里只负责"一轮接一轮"、显示和播报。详细日志写到 logs/demo_日期_时间.log
"""
import argparse
import os
import queue
import shlex
import statistics
import subprocess
import sys
import threading
import time

import config as cfg
import dashboard
import main
from devices import DeviceError, ManualGuard, ManualOverride, RealClock

SAY = {"found": "发现物块", "grasp_ok": "抓到了", "grasp_fail": "没抓到，重试"}


# ============================================================ 语音播报
class Speaker:
    """后台线程按顺序执行 TTS 命令（不阻塞抓取）；cmd 为 None 时什么都不做"""

    def __init__(self, cmd, max_queue=2):
        self.cmd = cmd
        self.max_queue = max_queue
        self.spoken = []
        self.failed = None
        self._q = queue.Queue()
        if cmd:
            threading.Thread(target=self._run, daemon=True).start()

    def argv(self, text):
        parts = shlex.split(self.cmd) if isinstance(self.cmd, str) else [str(p) for p in self.cmd]
        if any("{text}" in p for p in parts):
            return [p.replace("{text}", text) for p in parts]
        return parts + [text]

    def say(self, text):
        if not self.cmd or self.failed:
            return
        if self._q.qsize() >= self.max_queue:        # 播报跟不上就丢掉，别越积越晚
            return
        self._q.put(text)

    def _run(self):
        while True:
            text = self._q.get()
            try:
                subprocess.run(self.argv(text), timeout=15, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.spoken.append(text)
            except Exception as e:                    # 没装 TTS / 命令写错：提示一次，不再播报
                self.failed = f"{type(e).__name__}: {e}"
                print(f"  ⚠ 语音播报命令执行失败（{self.failed}），本次不再播报。检查 config.DEMO_TTS_CMD")
            finally:
                self._q.task_done()

    def wait(self):
        if self.cmd:
            self._q.join()


# ============================================================ 终端显示
class DemoUI:
    """接收 TaskMonitor 的事件：每个阶段结束打印一行（图标 + 用时 + 说明），每轮结束打印结果"""

    def __init__(self, mon, speaker, color=True, out=None, logfile=None):
        self.mon = mon
        self.speaker = speaker
        self.color = color
        self.out = out or sys.stdout
        self.logfile = logfile
        self.note = ""
        self.align_steps = 0
        self.phase_times = {}             # 阶段 → [用时, ...]
        self.rows = []
        self.round_t0 = None
        mon.listeners.append(self.on_event)

    def _c(self, code, s):
        return f"\033[{code}m{s}\033[0m" if self.color else s

    def p(self, s=""):
        print(s, file=self.out, flush=True)

    def _log(self, s):
        if self.logfile:
            try:
                self.logfile.write(time.strftime("%H:%M:%S ") + s + "\n")
                self.logfile.flush()
            except (OSError, ValueError):
                pass

    def banner(self, i, n):
        title = f" 第 {i} / {n} 轮 " if n else f" 第 {i} 轮 "
        self.p()
        self.p(self._c("1;36", "━" * 14 + title + "━" * 14))
        self._log(title.strip())
        self.round_t0 = time.monotonic()

    def on_event(self, kind, d):
        if kind == "log":
            self._log("  " + d["msg"])
            if self.mon.phase == "align" and d["msg"].startswith("对准 #"):
                self.align_steps += 1
            return
        if kind == "phase":
            self._phase_line(d["old"], d["dt"])
            self.note = ""
            if d["new"] == "align":
                self.align_steps = 0
            elif d["new"] == "reach" and self.mon.last_m is not None:
                self.note = f"深度 {self.mon.last_m.d:.0f}mm"
            return
        if kind == "found":
            self.note = f"发现物块  深度 {d['d']:.0f}mm  方位 {d['x_deg']:+.1f}°"
        elif kind == "grasp_ok":
            self.note = "✅ 抓到了"
        elif kind == "grasp_fail":
            self.note = "❌ 没抓到"
        elif kind == "episode":
            self._result(d["row"])
        if kind in SAY:
            self.speaker.say(SAY[kind])

    def _phase_line(self, key, dt):
        if key in ("idle", "done", "stopped"):
            return
        name, icon = dashboard.PHASES.get(key, (key, "•"))
        self.phase_times.setdefault(key, []).append(dt)
        note = self.note
        if key == "align":
            ok = (self.mon.align_err is not None and self.mon.task is not None
                  and self.mon.task.in_tol(self.mon.align_err))
            note = f"{self.align_steps} 步" + ("，误差进入容差" if ok else "")
        if key == "search" and not note:
            note = "没找到"
        line = f"  {icon}  {self._c('1', name)}   {dt:5.1f}s   {note}"
        self.p(line)
        self._log(f"[{name}] {dt:.1f}s {note}")

    def _result(self, row):
        self.rows.append(row)
        res = row.get("result")
        c = self.mon.counts
        text = dashboard.RESULT_TEXT.get(res, res)
        ok = res == "success"
        head = self._c("1;32", f"  ✅  {text}！") if ok else self._c("1;31", f"  ❌  {text}")
        self.p(f"{head}   本轮用时 {row.get('time_s', 0):.1f}s   "
               f"累计 成功 {c['success']} / {c['total']}")
        self._log(f"结果 {res} {row}")

    def summary(self, extra=None):
        rows = self.rows
        self.p()
        self.p(self._c("1;36", "━" * 16 + " 本次统计 " + "━" * 16))
        if not rows:
            self.p("  一轮都没有完成")
            return
        ok = sum(1 for r in rows if r.get("result") == "success")
        self.p(self._c("1", f"  共 {len(rows)} 轮   成功 {ok}   失败 {len(rows) - ok}   "
                            f"成功率 {100.0 * ok / len(rows):.0f}%"))
        times = [r["time_s"] for r in rows if "time_s" in r]
        ok_times = [r["time_s"] for r in rows if r.get("result") == "success"]
        if times:
            self.p(f"  平均每轮 {statistics.mean(times):.1f}s" +
                   (f"   成功的平均 {statistics.mean(ok_times):.1f}s（最快 {min(ok_times):.1f}s）" if ok_times else ""))
        fails = {}
        for r in rows:
            if r.get("result") != "success":
                k = dashboard.RESULT_TEXT.get(r.get("result"), r.get("result"))
                fails[k] = fails.get(k, 0) + 1
        if fails:
            self.p("  没成功的原因：" + "  ".join(f"{k}×{v}" for k, v in fails.items()))
        if self.phase_times:
            parts = []
            for k in ("search", "align", "reach", "verify", "place", "retry"):
                if k in self.phase_times:
                    parts.append(f"{dashboard.PHASES[k][0]} {statistics.mean(self.phase_times[k]):.1f}s")
            self.p("  各阶段平均用时：" + "  ".join(parts))
        task = self.mon.task
        if task is not None:
            self.p("  学到的灵敏度 J：" + "  ".join(f"{n}={a.j:.3f}" for n, a in task.axes.items()))
        for s in extra or []:
            self.p("  " + s)
        self._log(f"统计 共 {len(rows)} 轮 成功 {ok}")


# ============================================================ 运行
def wait_next(auto_s, sleep=time.sleep):
    """两轮之间：auto_s 为 None 等操作员回车（q 退出）；否则自动等 auto_s 秒。返回 False = 退出"""
    if auto_s is None:
        try:
            ans = input("  ▶ 把物块放好后按回车开始下一轮（q 退出）：").strip().lower()
        except EOFError:
            return False
        return ans != "q"
    if auto_s > 0:
        print(f"  ⏳ {auto_s:g} 秒后开始下一轮（Ctrl+C 停止）", flush=True)
        sleep(auto_s)
    return True


def safe_stop(devs):
    """Ctrl+C：先让会动的东西停下（升降轴急停 E、底盘停车），再关串口"""
    if not devs:
        return
    cam, lift, arm, chassis = devs
    for d in (lift, chassis):
        try:
            d.stop()
        except Exception:
            pass
    main.close_devices(devs)


def make_ui(a, mon, logfile=None):
    color = not a.plain and sys.stdout.isatty()
    return DemoUI(mon, Speaker(cfg.DEMO_TTS_CMD), color=color, logfile=logfile)


def start_dashboard(a, mon):
    if not a.dashboard:
        return None
    return dashboard.start(mon, a.port if a.port is not None else cfg.DASHBOARD_PORT)


def run_real(a, clock=None, use_guard=True, logfile=None):
    """真机演示。clock / use_guard 只给离线测试用（换成仿真时钟、不检测手柄）"""
    clock = clock or RealClock()
    guard = ManualGuard(cfg.MANUAL_PROCESS) if use_guard else None
    mon = dashboard.TaskMonitor(cfg, clock, None, mode="real")
    ui = make_ui(a, mon, logfile)
    srv = start_dashboard(a, mon)
    auto = a.auto if a.auto is not None else cfg.DEMO_AUTO_INTERVAL_S
    if use_guard:
        time.sleep(1.2)
    task, devs, checked, i, dev_errors = None, None, False, 0, 0
    try:
        while a.rounds <= 0 or i < a.rounds:
            if guard is not None and guard.active:
                if devs:
                    main.close_devices(devs)
                    devs = None
                    print("手柄接管中，串口已释放，等待手柄断开……")
                time.sleep(1.0)
                continue
            if devs is None:
                devs = main.open_devices(clock)
                if not checked:
                    problems = main.preflight(*devs)
                    checked = True
                    if problems and not a.force:
                        print(f"有 {len(problems)} 项检查没通过，不启动（确认安全的话可以加 --force 跳过）")
                        return ui
                task = main.attach_task(task, devs, clock, a.step, guard, a.record, monitor=mon)
                task.verbose = a.verbose       # 默认只显示大号状态行，详细过程在日志文件和看板里
            mon.round = {"i": i + 1, "n": a.rounds if a.rounds > 0 else None}
            ui.banner(i + 1, a.rounds if a.rounds > 0 else None)
            try:
                task.run_once()
            except ManualOverride as e:
                print(e)
                continue
            except DeviceError as e:
                print("  ⚠ 设备错误：", e)
                main.close_devices(devs)
                devs = None
                dev_errors += 1
                if dev_errors >= 3:
                    print("  连续 3 次设备错误，停止演示")
                    break
                time.sleep(3)
                continue
            dev_errors = 0
            i += 1
            if (a.rounds <= 0 or i < a.rounds) and not wait_next(auto):
                break
    except KeyboardInterrupt:
        print("\n手动停止：升降轴急停、底盘停车")
        mon.set_phase("stopped")
        safe_stop(devs)
        devs = None
    finally:
        if devs:
            main.close_devices(devs)
        ui.speaker.wait()
        ui.summary()
        if srv is not None:
            srv.close()
    return ui


def run_sim(a, logfile=None):
    """仿真演示：每轮一个新场景（同 main.run_sim），学到的灵敏度接着用"""
    import random
    from sim import SIM_POSES, make_car, make_sim
    from task import GraspTask

    cfg.POSES = SIM_POSES
    if a.no_chassis:
        cfg.USE_CHASSIS = False
    rng = random.Random(a.seed)
    bias = rng.uniform(0.88, 1.12)
    r_cam = cfg.R_CAM_MM * rng.uniform(0.8, 1.25)
    car = make_car(cfg, a.seed)
    mon = dashboard.TaskMonitor(cfg, None, None, mode="sim")
    ui = make_ui(a, mon, logfile)
    srv = start_dashboard(a, mon)
    print(f"仿真演示：{a.sim} 轮，{'无底盘' if not cfg.USE_CHASSIS else '有底盘'}，种子 {a.seed}" +
          (f"，按 {a.realtime:g} 倍真实速度播放" if a.realtime else ""))
    task, truth = None, []
    auto = a.auto if a.auto is not None else 0
    try:
        for i in range(a.sim):
            world, clock, cam, lift, arm, chassis = make_sim(cfg, a.seed * 1000 + i, bias, r_cam, car)
            if a.realtime:
                _pace(clock, a.realtime)
            if task is None:
                task = GraspTask(cfg, clock, cam, lift, arm, chassis, log_dir=None, learn=True, verbose=a.verbose)
            else:
                task.clock = clock
                task.base_deg = 0.0
            main.attach_task(task, (cam, lift, arm, chassis), clock, monitor=mon)
            mon.round = {"i": i + 1, "n": a.sim}
            ui.banner(i + 1, a.sim)
            try:
                task.run_once()
            except DeviceError as e:
                print("  ⚠ 设备错误：", e)
                ui.rows.append({"result": "device_error", "time_s": round(clock.now(), 1)})
                task._lifted = False
            truth.append(world.picked)
            if i + 1 < a.sim and auto:
                time.sleep(auto if a.realtime else 0)
    except KeyboardInterrupt:
        print("\n手动停止")
        mon.set_phase("stopped")
    finally:
        ui.speaker.wait()
        ui.summary([f"仿真真实抓到 {sum(truth)} / {len(truth)}"] if truth else None)
        if srv is not None and a.realtime:
            print("看板还开着，按 Ctrl+C 退出")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
        if srv is not None:
            srv.close()
    return ui


def _pace(clock, factor):
    """仿真时钟每走 s 秒，真实时间睡 s / factor 秒（只为了看看板，不影响仿真结果）"""
    orig = clock.sleep

    def sleep(s):
        orig(s)
        if s > 0:
            time.sleep(s / factor)
    clock.sleep = sleep


def parse(argv=None):
    ap = argparse.ArgumentParser(description="比赛演示模式：连续抓取 + 大号状态行 + 统计")
    ap.add_argument("--rounds", type=int, default=5, help="真机抓几轮（0 = 一直抓到 Ctrl+C），默认 5")
    ap.add_argument("--auto", type=float, metavar="秒", default=None,
                    help="两轮之间自动等几秒（不加 = 等操作员按回车；默认值见 config.DEMO_AUTO_INTERVAL_S）")
    ap.add_argument("--no-chassis", action="store_true", help="不用底盘")
    ap.add_argument("--dashboard", action="store_true", help="开网页看板（http://<树莓派IP>:8080）")
    ap.add_argument("--port", type=int, default=None, help="看板端口（默认 config.DASHBOARD_PORT）")
    ap.add_argument("--record", action="store_true", help="同时记录动作和 K230 数据（同 main.py --record）")
    ap.add_argument("--step", action="store_true", help="单步模式：每个动作前按回车确认")
    ap.add_argument("--force", action="store_true", help="启动前检查没通过也继续（自己确认安全）")
    ap.add_argument("--verbose", action="store_true", help="终端里也打印任务的详细过程（默认只写日志文件）")
    ap.add_argument("--plain", action="store_true", help="不用颜色")
    ap.add_argument("--sim", type=int, metavar="N", help="仿真演示 N 轮（不需要硬件）")
    ap.add_argument("--seed", type=int, default=1, help="仿真种子")
    ap.add_argument("--realtime", type=float, metavar="倍速", default=0,
                    help="仿真按真实速度播放（1 = 原速，2 = 两倍速），配合 --dashboard 预览看板")
    return ap.parse_args(argv)


def open_log():
    os.makedirs(main.LOG_DIR, exist_ok=True)
    path = os.path.join(main.LOG_DIR, time.strftime("demo_%Y%m%d_%H%M%S.log"))
    f = open(path, "a", encoding="utf-8")
    f.write("启动：" + " ".join(sys.argv) + "  calib.json 覆盖：" + str(sorted(cfg.CALIB_LOADED)) + "\n")
    print("详细日志写到", path)
    return f


if __name__ == "__main__":
    args = parse()
    if args.sim:
        run_sim(args)
    else:
        if args.no_chassis:
            cfg.USE_CHASSIS = False
        log = open_log()
        try:
            run_real(args, logfile=log)
        finally:
            log.close()
