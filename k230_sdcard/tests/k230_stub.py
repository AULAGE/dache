# -*- coding: utf-8 -*-
"""
在 CPython 里加载 K230 程序（k230_sdcard/main.py）的纯函数部分。

做法：把 CanMV 专有模块（media、nncase_runtime、aicube、machine、ulab ...）换成假模块，
ulab.numpy 用真 numpy 代替，然后按普通模块导入 main.py（不会进入 detection() 主循环）。

注意 ulab.numpy 和 numpy 的差异：
- ulab 的浮点是 float32（K230 固件），这里 NP_FLOAT 在 numpy 2.x 下也会落到 float32，基本一致
- ulab 不一定支持布尔下标，main.py 里用到时有 try/except 回退；测试里两条路径都测
- 标量下标 np_img[c, y, x] 在 ulab 里很慢（每次几微秒），numpy 里快，所以耗时测不准，只测结果
"""
import importlib.util
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN_PATH = os.path.join(os.path.dirname(HERE), "main.py")


def _install_fakes():
    import numpy

    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Any:
        """什么属性都有、怎么调用都行的占位对象（只在主循环里用到，测试不会走到）"""
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            return _Any()

        def __call__(self, *a, **k):
            return _Any()

    class FakeUART:
        UART1 = 1

        def __init__(self, *a, **k):
            self.sent = []

        def write(self, b):
            self.sent.append(bytes(b))
            return len(b)

    class FakeFPIOA:
        UART1_TXD = 0
        UART1_RXD = 1

        def set_function(self, *a):
            pass

    mod("ujson", load=lambda f: {}, loads=lambda s: {})
    mod("aicube", anchorbasedet_post_process=lambda *a, **k: [])
    mod("libs")
    mod("libs.PipeLine", ScopedTiming=_Any)
    mod("libs.Utils", get_colors=lambda n: [(255, 0, 255, 0)] * n)
    mod("media")
    mod("media.sensor", Sensor=_Any)
    mod("media.display", Display=_Any())
    mod("media.media", MediaManager=_Any())
    mod("nncase_runtime", kpu=_Any, ai2d=_Any)
    ulab = mod("ulab")
    ulab.numpy = numpy
    sys.modules["ulab.numpy"] = numpy
    mod("image", Image=_Any, ARGB8888=0, RGBP888=1)
    mod("machine", UART=FakeUART, FPIOA=FakeFPIOA)
    # MicroPython 的 time 扩展：ticks 按 2^30 回绕
    if not hasattr(time, "ticks_ms"):
        period = 1 << 30
        time.ticks_ms = lambda: int(time.monotonic() * 1000) & (period - 1)
        time.ticks_us = lambda: int(time.monotonic() * 1e6) & (period - 1)
        time.ticks_add = lambda t, d: (t + d) & (period - 1)

        def ticks_diff(a, b):
            return ((a - b + period // 2) & (period - 1)) - period // 2
        time.ticks_diff = ticks_diff
        time.sleep_ms = lambda ms: time.sleep(ms / 1000.0)


def load_main(quiet=True, **overrides):
    """导入一份全新的 main.py 模块对象；overrides 在导入后覆盖配置常量。
    quiet=True 时屏蔽 main.py 里的 print（测试输出干净一些）"""
    _install_fakes()
    name = "k230_main_%d" % load_main.counter
    load_main.counter += 1
    spec = importlib.util.spec_from_file_location(name, MAIN_PATH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    if quiet:
        m.print = lambda *a, **k: None
    for k, v in overrides.items():
        if not hasattr(m, k):
            raise AttributeError("main.py 里没有配置项 " + k)
        setattr(m, k, v)
    return m


load_main.counter = 0
