#!/usr/bin/env python3
"""
任务 17（上车前代码审查）的复现测试。不接硬件，用假串口 / 仿真设备构造审查里发现的场景。

  python3 test_review.py          全部
  python3 test_review.py -v       显示每个用例

每个用例的说明里写了对应报告 docs/cloud_tasks/results/17_报告.md 的哪一条：
  "修复"  = 这次改了代码，测试验证修好了（改之前这个测试会失败 / 卡住）
  "复现"  = 只报告没修，测试验证问题确实存在（问题修好以后要把断言反过来）
"""
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


if __name__ == "__main__":
    unittest.main()
