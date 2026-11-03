#!/usr/bin/env python3
"""
任务 17（上车前代码审查）的复现测试。不接硬件，用假串口 / 仿真设备构造审查里发现的场景。

  python3 test_review.py          全部
  python3 test_review.py -v       显示每个用例

每个用例的说明里写了对应报告 docs/cloud_tasks/results/17_报告.md 的哪一条：
  "修复"  = 这次改了代码，测试验证修好了（改之前这个测试会失败 / 卡住）
  "复现"  = 只报告没修，测试验证问题确实存在（问题修好以后要把断言反过来）
"""
import contextlib
import io
import os
import sys
import tempfile
import threading
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["GRASP_IGNORE_CALIB"] = "1"     # 按 config.py 默认值测，不受这台车 calib.json 影响
os.environ.pop("GRASP_RECORD", None)

import config as cfg  # noqa: E402
import devices  # noqa: E402


class FakeSerialException(OSError):
    pass


def install_fake(factory):
    devices.serial = types.SimpleNamespace(Serial=factory, SerialException=FakeSerialException)


class LineSerial:
    """升降轴假串口：按脚本一行一行回复"""

    def __init__(self, replies):
        self.replies = list(replies)
        self.written = []

    def __call__(self, *a, **k):
        return self

    def write(self, b):
        self.written.append(bytes(b))

    def readline(self):
        time.sleep(0.001)
        return (self.replies.pop(0) + "\n").encode() if self.replies else b""

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


def _no_guard(*a, **k):
    return types.SimpleNamespace(active=False)


class RunRealHarness:
    """把 main.run_real 里的真机部分换成假的：open_devices / preflight / ManualGuard / sleep / attach_task"""

    def __init__(self, main, run_once_seq, open_fail=()):
        self.main = main
        self.seq = list(run_once_seq)         # 每次 run_once 做什么：异常实例 / "ok"
        self.open_fail = list(open_fail)      # 第几次打开设备时抛什么（None = 正常）
        self.opens = 0
        self.closed = []
        self.saved = {}

    def __enter__(self):
        m = self.main
        h = self

        class Dev:
            def __init__(self, name):
                self.name = name

            def stop(self):
                h.closed.append(self.name + ".stop")

            def close(self):
                h.closed.append(self.name + ".close")

        def open_devices(clock):
            h.opens += 1
            if h.opens > 20:
                raise KeyboardInterrupt        # 防止测试卡死：重开太多次就当成 Ctrl+C
            if h.open_fail:
                e = h.open_fail.pop(0)
                if e is not None:
                    raise e
            return (Dev("cam"), Dev("lift"), Dev("arm"), Dev("chassis"))

        def run_once():
            if not h.seq:
                raise KeyboardInterrupt
            e = h.seq.pop(0)
            if isinstance(e, BaseException):
                raise e
            return {"result": "success"}

        task = types.SimpleNamespace(run_once=run_once)
        for k, v in dict(open_devices=open_devices, preflight=lambda *a: [], ManualGuard=_no_guard,
                         attach_task=lambda *a, **k: task).items():
            self.saved[k] = getattr(m, k)
            setattr(m, k, v)
        self.saved_sleep = m.time.sleep
        m.time.sleep = lambda s: None
        return self

    def __exit__(self, *a):
        for k, v in self.saved.items():
            setattr(self.main, k, v)
        self.main.time.sleep = self.saved_sleep


class TestLiftReset(unittest.TestCase):
    """报告 [伤硬件-1]（修复）：升降轴复位（READY）后不能自动接着跑"""

    def test_ready_raises_reset_error(self):
        fake = LineSerial(["OK GO 30.00", "READY MAX 50.00 RST POR"])
        install_fake(fake)
        lift = devices.Lift("x", 115200, 44)
        with self.assertRaises(devices.DeviceResetError) as cm:
            lift.move_to(30, timeout=1.0)
        self.assertIsInstance(cm.exception, devices.DeviceError)     # 老代码 except DeviceError 仍能接住

    def test_run_real_stops_after_reset(self):
        import main
        reset = devices.DeviceResetError("升降轴 复位了：READY MAX 50.00 RST POR")
        with RunRealHarness(main, [reset, "ok", "ok"]) as h:
            main.run_real(once=False, step=False, force=False)
        # 改之前：DeviceError 分支关掉设备、3 秒后重开接着抓（opens = 2，第二次 run_once 会让升降轴按错的零点走）
        self.assertEqual(h.opens, 1, "复位后不应该重开设备继续")
        self.assertIn("lift.stop", h.closed)
        self.assertIn("chassis.stop", h.closed)

    def test_bench_stops_after_reset(self):
        import bench
        rig = bench.RealRig()
        closed = []
        rig.main = types.SimpleNamespace(close_devices=lambda d: closed.append(d))
        rig.clock = devices.RealClock()
        rig.devs = ("cam", "lift", "arm", "chassis")

        def boom():
            raise devices.DeviceResetError("升降轴 复位了：READY MAX 50.00 RST POR")
        rig.task = types.SimpleNamespace(run_once=boom)
        # 改之前：当成普通设备错误记一行 device_error，下一个位置自动重开设备接着测
        with self.assertRaises(devices.DeviceResetError):
            rig.run_once(1)
        self.assertIsNone(rig.devs)
        self.assertEqual(len(closed), 1)

    def test_other_device_error_still_retries(self):
        import main
        with RunRealHarness(main, [devices.DeviceError("K230 没有任何数据"), "ok"]) as h:
            main.run_real(once=False, step=False, force=False)
        self.assertEqual(h.opens, 2, "普通设备错误仍然重开设备再来（原行为）")


CHASSIS_SCRIPT = r'''
import os, sys, time, types
sys.path.insert(0, {here!r})
os.environ["GRASP_IGNORE_CALIB"] = "1"
import devices, main
out = open({out!r}, "ab", buffering=0)

class S:
    def __init__(self, *a, **k): pass
    def write(self, b): out.write(bytes(b))
    def flush(self): pass
    def read(self, n):
        time.sleep(0.02)
        return b""
    def close(self): pass

devices.serial = types.SimpleNamespace(Serial=S, SerialException=OSError)
ch = devices.WheeltecChassis("x", 115200, 100, 600, True, None)
if {install}:
    main.install_stop_signals()
print("ready", flush=True)
try:
    ch.drive(3000)            # 没有回传：按时间走 30 秒
finally:
    main.close_devices([ch])
'''


def last_chassis_frame(path):
    with open(path, "rb") as fh:
        data = fh.read()
    i = data.rfind(b"\x7B")
    f = data[i:i + 11]
    return int.from_bytes(f[3:5], "big", signed=True), int.from_bytes(f[7:9], "big", signed=True)


class TestStopSignals(unittest.TestCase):
    """报告 [伤硬件-2]（修复）：systemctl stop / SSH 断开时底盘最后一帧不是停车帧"""

    def _run(self, sig, install):
        import signal
        import subprocess
        out = tempfile.mktemp(suffix=".bin")
        code = CHASSIS_SCRIPT.format(here=HERE, out=out, install=install)
        p = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True)
        try:
            self.assertEqual(p.stdout.readline().strip(), "ready")
            time.sleep(0.4)                     # 让它开起来（发送线程在发 vx=100）
            p.send_signal(getattr(signal, sig))
            p.wait(timeout=10)
        finally:
            if p.poll() is None:
                p.kill()
            p.stdout.close()
            p.stderr.close()
        try:
            return last_chassis_frame(out)
        finally:
            os.remove(out)

    def test_sigterm_sends_stop_frame(self):
        self.assertEqual(self._run("SIGTERM", True), (0, 0))

    def test_sighup_sends_stop_frame(self):
        self.assertEqual(self._run("SIGHUP", True), (0, 0))

    def test_without_handler_last_frame_is_moving(self):
        # 复现原来的危险：不装信号处理时进程被直接杀掉，最后发出去的是 vx=100 的帧，固件会一直按它跑
        vx, wz = self._run("SIGTERM", False)
        self.assertGreater(vx, 0)


class TestReopenAndSerialErrors(unittest.TestCase):
    """报告 [报错退出-1]（修复）：一直抓模式下串口异常 / 重开失败直接带堆栈退出"""

    def test_serial_exception_mid_run_reopens(self):
        import main
        # pyserial 的 SerialException 是 OSError，不是 DeviceError：改之前直接冲出 run_real，程序退出
        with RunRealHarness(main, [FakeSerialException("device reports readiness to read but returned no data"),
                                   "ok"]) as h:
            main.run_real(once=False, step=False, force=False)
        self.assertEqual(h.opens, 2)
        self.assertIn("lift.stop", h.closed)

    def test_reopen_failure_retries(self):
        import main
        with RunRealHarness(main, [devices.DeviceError("机械臂 USB 掉线"), "ok"],
                            open_fail=[None, FakeSerialException("No such file"),
                                       devices.DeviceError("机械臂串口打不开"), None]) as h:
            main.run_real(once=False, step=False, force=False)
        self.assertEqual(h.opens, 4, "重开失败两次后第三次成功，接着抓")

    def test_first_open_failure_still_raises(self):
        import main
        with RunRealHarness(main, ["ok"], open_fail=[devices.DeviceError("机械臂串口打不开")]):
            with self.assertRaises(devices.DeviceError):
                main.run_real(once=False, step=False, force=False)

    def test_open_devices_closes_partial(self):
        import main
        log = []

        class Cam:
            def __init__(self, *a, **k):
                log.append("cam.open")
                self.ser = types.SimpleNamespace(close=lambda: log.append("cam.ser.close"))

            def close(self):
                log.append("cam.close")

        class LiftF(Cam):
            def __init__(self, *a, **k):
                log.append("lift.open")
                self.ser = types.SimpleNamespace(close=lambda: log.append("lift.ser.close"))

            def stop(self):
                log.append("lift.stop")

        def arm_fail(*a, **k):
            raise devices.DeviceError("机械臂串口打不开")
        saved = main.K230, main.Lift, main.Arm
        main.K230, main.Lift, main.Arm = Cam, LiftF, arm_fail
        try:
            with self.assertRaises(devices.DeviceError):
                main.open_devices(devices.RealClock())
        finally:
            main.K230, main.Lift, main.Arm = saved
        self.assertIn("cam.close", log)            # K230 读线程停掉，重开时不会两个线程抢一个串口
        self.assertIn("cam.ser.close", log)
        self.assertIn("lift.ser.close", log)


class Spy:
    """包住仿真设备：记录调用；第 stop_at 个动作调用之前抛 KeyboardInterrupt（模拟这时按 Ctrl+C）"""
    MOTION = {"move_to", "move_by", "start_move_by", "wait_move", "drive", "forward", "turn", "move"}

    def __init__(self, dev, name, ctr, log):
        self._dev, self._name, self._ctr, self._log = dev, name, ctr, log

    def __getattr__(self, k):
        attr = getattr(self._dev, k)
        if not callable(attr):
            return attr
        if k in ("stop", "close"):
            def w(*a, **kw):
                self._log.append(f"{self._name}.{k}")
                return attr(*a, **kw)
            return w
        if k not in self.MOTION:
            return attr

        def wrapped(*a, **kw):
            self._ctr["n"] += 1
            self._ctr["calls"].append(f"{self._name}.{k}")
            if self._ctr["n"] == self._ctr["stop_at"]:
                if k != "drive" or not hasattr(self._dev, "_sleep"):
                    raise KeyboardInterrupt
                # 底盘：在行驶途中（第 5 个控制周期）按 Ctrl+C
                sl, cnt = self._dev._sleep, {"n": 0}

                def sleep_then_ctrl_c(s):
                    cnt["n"] += 1
                    if cnt["n"] == 5:
                        raise KeyboardInterrupt
                    sl(s)
                self._dev._sleep = sleep_then_ctrl_c
            return attr(*a, **kw)
        return wrapped


class TestCtrlCEveryStage(unittest.TestCase):
    """报告"异常路径"一节：每个阶段按 Ctrl+C，最后升降轴都发了急停、底盘都停了"""

    def _run(self, use_chassis, stop_at, seed=3):
        import main
        import sim
        old = cfg.POSES, cfg.USE_CHASSIS, cfg.LEARN_FILE
        cfg.POSES, cfg.USE_CHASSIS = dict(sim.SIM_POSES), use_chassis
        world, clock, cam, lift, arm, chassis = sim.make_sim(cfg, seed, 1.0, cfg.R_CAM_MM)
        ctr, log = {"n": 0, "stop_at": stop_at, "calls": []}, []
        devs = (cam, Spy(lift, "lift", ctr, log), Spy(arm, "arm", ctr, log),
                Spy(chassis if use_chassis else devices.NullChassis(), "chassis", ctr, log))
        saved = {k: getattr(main, k) for k in ("open_devices", "preflight", "ManualGuard", "RealClock", "LOG_DIR")}
        main.open_devices = lambda c: devs
        main.preflight = lambda *a: []
        main.ManualGuard = _no_guard
        main.RealClock = lambda: clock
        main.LOG_DIR = tempfile.mkdtemp(prefix="review_")
        sleep, main.time.sleep = main.time.sleep, (lambda s: None)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                main.run_real(once=True, step=False, force=False)
        finally:
            for k, v in saved.items():
                setattr(main, k, v)
            main.time.sleep = sleep
            cfg.POSES, cfg.USE_CHASSIS, cfg.LEARN_FILE = old
        return ctr, log, chassis

    def test_every_stage(self):
        for use_chassis in (False, True):
            total = self._run(use_chassis, stop_at=10 ** 9)[0]["n"]   # 先跑一遍完整流程数一下有几个动作
            self.assertGreater(total, 5)
            for k in range(1, total + 1):
                ctr, log, chassis = self._run(use_chassis, stop_at=k)
                where = f"{'有' if use_chassis else '无'}底盘，第 {k} 个动作 {ctr['calls'][k - 1]} 时按 Ctrl+C"
                self.assertIn("lift.stop", log, where)
                self.assertIn("chassis.stop", log, where)
                if use_chassis and hasattr(chassis, "cmd"):
                    self.assertEqual(tuple(chassis.cmd), (0, 0), where)


class TestReportedNotFixed(unittest.TestCase):
    """只报告、没有修的问题（断言的是"问题存在"；修好以后把断言反过来）"""

    def test_ghost_fingerprint_learns_real_block_when_base_has_deadband(self):
        """[会抓错-1]：底座死区大时 GHOST_CHECK 把真物块判成 ghost，GHOST_FINGERPRINT 再把它记成指纹、以后一直过滤"""
        import sim
        from task import GraspTask
        names = ("POSES", "USE_CHASSIS", "R_CAM_MM", "BASE_DIR", "SIM_SERVO_DEADBAND_US", "GHOST_FINGERPRINT")
        old = {k: getattr(cfg, k) for k in names}
        old_j0 = cfg.AXES["x_deg"]["j0"]
        try:
            # 真机标定值：相机在底座转轴上、底座灵敏度 −0.6、底座方向 −1；死区 50us ≈ 4.5°（"< 5° 转动不到位"）
            cfg.POSES, cfg.USE_CHASSIS, cfg.R_CAM_MM, cfg.BASE_DIR = dict(sim.SIM_POSES), False, 0.0, -1
            cfg.AXES["x_deg"]["j0"] = -0.6
            cfg.SIM_SERVO_DEADBAND_US, cfg.GHOST_FINGERPRINT = 50, True
            d = tempfile.mkdtemp(prefix="review_ghost_")
            car = sim.make_car(cfg, 1)
            task, res = None, []
            for i in range(20):
                world, clock, cam, lift, arm, ch = sim.make_sim(cfg, 1000 + i, 1.0, 0.0, car)
                if task is None:
                    task = GraspTask(cfg, clock, cam, lift, arm, ch, log_dir=d, verbose=False)
                else:
                    task.clock, task.cam, task.lift, task.arm, task.chassis = clock, cam, lift, arm, ch
                    task.base_deg, task._base_sent, task._base_dir = 0.0, None, 0
                res.append(task.run_once()["result"])
        finally:
            for k, v in old.items():
                setattr(cfg, k, v)
            cfg.AXES["x_deg"]["j0"] = old_j0
        # 仿真里根本没有"爪子被识别成物块"，所有 ghost 都是误判；记下的指纹全是真物块的位置
        self.assertGreater(res.count("ghost"), 0, res)
        self.assertGreater(len(task.ghosts), 0)
        self.assertGreater(sum(g["seen"] for g in task.ghosts), 0, "真物块被指纹过滤掉了")

    def test_reach_pose_divides_by_zero_when_near_equals_far(self):
        """[报错退出-2]（概率很小，没修）：REACH_NEAR_D == REACH_FAR_D 时 reach_pose 除零"""
        from task import GraspTask
        t = GraspTask.__new__(GraspTask)
        t.cfg = types.SimpleNamespace(POSES={"reach_near": {3: 1}, "reach_far": {3: 2}},
                                      REACH_NEAR_D=180.0, REACH_FAR_D=180.0)
        with self.assertRaises(ZeroDivisionError):
            t.reach_pose(180.0)

    def test_partial_tol_in_calib_json_crashes(self):
        """[报错退出-3]（没修）：calib.json 里 TOL 只写一部分轴会整体替换，缺的轴 KeyError"""
        import json
        import subprocess
        path = tempfile.mktemp(suffix=".json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"TOL": {"x_deg": 2.0}}, f)
        env = dict(os.environ, GRASP_CALIB_FILE=path)
        env.pop("GRASP_IGNORE_CALIB", None)
        code = "import config as c; print(sorted(c.TOL))"
        out = subprocess.run([sys.executable, "-c", code], cwd=HERE, env=env, capture_output=True, text=True)
        os.remove(path)
        self.assertEqual(out.stdout.strip(), "['x_deg']")       # y / d 没了 → in_tol() 里 KeyError

    def test_lift_clamp_makes_observe_check_lower(self):
        """[会抓错-3]（没修）：升降轴离上限不到 5cm 时，抬升被截断，观察确认却按满 5cm 往下降"""

        class S(LineSerial):
            def write(self, b):
                t = b.decode().split()
                if t and t[0] == "A":
                    self.replies += [f"OK GO {t[1]}", f"DONE {t[1]}"]

        install_fake(S([]))
        lift = devices.Lift("x", 115200, 44.0)
        lift.pos = 42.0                                     # 抓取时的高度
        up = lift.move_by(5.0)                              # grasp()：合爪后上抬 → 截到 44
        down = lift.move_by(-5.0)                           # _observe_check()：降回"抓取时的高度"
        self.assertEqual((up, down), (44.0, 39.0))          # 比抓取时低 3cm（≈ 物块在画面里偏 18mm）


def wheeltec_fb(vx=0, volt_mv=25000, flag=0):
    f = bytearray(24)
    f[0], f[1] = 0x7B, flag
    f[2:4] = (vx & 0xFFFF).to_bytes(2, "big")
    f[20:22] = volt_mv.to_bytes(2, "big")
    b = 0
    for x in f[:22]:
        b ^= x
    f[22], f[23] = b, 0x7D
    return bytes(f)


class TestChassisFeedbackLoss(unittest.TestCase):
    """报告 [伤硬件-3]（修复）：底盘行驶中回传断了，原来会一直全速开到超时"""

    def test_drive_stops_when_feedback_stops(self):
        written = []
        t_start = time.monotonic()

        class S:
            def __init__(self, *a, **k):
                pass

            def write(self, b):
                written.append(bytes(b))

            def flush(self):
                pass

            def read(self, n):
                time.sleep(0.05)               # 20Hz 回传
                # 前 0.6 秒正常回传（车速回传 0：轮子打滑 / 编码器线松），之后回传断了
                return wheeltec_fb() if time.monotonic() - t_start < 0.6 else b""

            def close(self):
                pass

        install_fake(S)
        ch = devices.WheeltecChassis("x", 115200, 100, 600, True, None, cmd_ttl_s=0)
        try:
            self.assertTrue(ch.has_feedback())
            t0 = time.monotonic()
            with self.assertRaises(devices.DeviceError) as cm:
                ch.drive(400, timeout=8.0)
            dt = time.monotonic() - t0
            # 改之前：一直以 100mm/s 发指令，8 秒后才报"底盘移动超时"
            self.assertLess(dt, 2.0, f"{dt:.1f}s 后才停：{cm.exception}")
            self.assertIn("回传中断", str(cm.exception))
            self.assertEqual(ch.cmd, (0, 0))
        finally:
            ch.close()
        self.assertEqual(written[-1], devices.WheeltecChassis.frame(0, 0))

    def test_normal_drive_unaffected(self):
        """回传一直在：照常走完（假控制器按指令速度回传）"""
        st = {"vx": 0}

        class S:
            def __init__(self, *a, **k):
                pass

            def write(self, b):
                st["vx"] = int.from_bytes(b[3:5], "big", signed=True)

            def flush(self):
                pass

            def read(self, n):
                time.sleep(0.05)
                return wheeltec_fb(st["vx"])

            def close(self):
                pass

        install_fake(S)
        ch = devices.WheeltecChassis("x", 115200, 100, 600, True, None)
        try:
            moved, _ = ch.drive(60, timeout=8.0)
            self.assertGreater(moved, 20)
        finally:
            ch.close()


if __name__ == "__main__":
    unittest.main()
