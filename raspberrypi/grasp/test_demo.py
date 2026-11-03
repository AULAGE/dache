"""
demo.py / dashboard.py 的离线测试（独立文件，不依赖硬件）：python3 test_demo.py
1. 看板不改变任务行为：同一个仿真种子，挂 / 不挂看板的抓取结果逐项相同；默认（不加 --dashboard）什么都不挂
2. 仿真演示跑 3 轮：统计、阶段用时都有
3. 看板接口：/api/state 返回正确的 JSON，/ 是中文页面，/events 是 SSE
4. 语音播报：配置 DEMO_TTS_CMD 后在"发现物块 / 抓到了"时执行命令
5. 真机流程（open_devices / preflight 换成仿真设备）：连续抓 2 轮；中途 Ctrl+C 时升降轴急停、底盘停车
"""
import io
import json
import os
import sys
import tempfile
import urllib.request

os.environ.setdefault("GRASP_IGNORE_CALIB", "1")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg                         # noqa: E402
import dashboard                             # noqa: E402
import demo                                  # noqa: E402
import main                                  # noqa: E402
from sim import SIM_POSES, make_car, make_sim   # noqa: E402
from task import GraspTask                   # noqa: E402

PASS, FAIL = [], []
SAVED = {k: getattr(cfg, k) for k in ("POSES", "USE_CHASSIS", "DEMO_TTS_CMD")}


def check(name, ok, info=""):
    (PASS if ok else FAIL).append(name)
    print(("  ✅ " if ok else "  ❌ ") + name + (f"   {info}" if info else ""))


def restore():
    for k, v in SAVED.items():
        setattr(cfg, k, v)


def run_episodes(mon=None, n=4, seed=7, chassis=False):
    """仿真跑 n 次；mon 不为 None 时经 main.attach_task 挂上看板"""
    cfg.POSES, cfg.USE_CHASSIS = SIM_POSES, chassis
    try:
        car = make_car(cfg, seed)
        rows, task = [], None
        for i in range(n):
            world, clock, cam, lift, arm, ch = make_sim(cfg, seed * 1000 + i, 1.0, cfg.R_CAM_MM, car)
            if task is None:
                task = GraspTask(cfg, clock, cam, lift, arm, ch, log_dir=None, learn=True, verbose=False)
            else:
                task.clock, task.base_deg = clock, 0.0
                task.cam, task.lift, task.arm, task.chassis = cam, lift, arm, ch
            if mon is not None:
                main.attach_task(task, (cam, lift, arm, ch), clock, monitor=mon)
            rows.append(task.run_once())
        return rows, task
    finally:
        restore()


def test_no_behaviour_change():
    print("1. 看板不改变行为")
    for chassis in (False, True):
        a, _ = run_episodes(None, chassis=chassis)
        mon = dashboard.TaskMonitor(cfg, mode="sim")
        b, task = run_episodes(mon, chassis=chassis)
        check(f"{'有' if chassis else '无'}底盘：挂 / 不挂看板的抓取结果逐项相同", a == b, f"{[r['result'] for r in a]}")
        check(f"{'有' if chassis else '无'}底盘：看板计数和结果一致", mon.counts["total"] == len(b)
              and mon.counts["success"] == sum(r["result"] == "success" for r in b), str(mon.counts))
    # 默认不挂
    cfg.POSES, cfg.USE_CHASSIS = SIM_POSES, False
    try:
        world, clock, cam, lift, arm, ch = make_sim(cfg, 1, 1.0, cfg.R_CAM_MM)
        old = main.LOG_DIR
        main.LOG_DIR = tempfile.mkdtemp(prefix="demo_test_log_")
        try:
            t = main.attach_task(None, (cam, lift, arm, ch), clock)
        finally:
            main.LOG_DIR = old
        check("默认（不加 --dashboard）不挂看板、不包设备", getattr(t, "_monitor", None) is None
              and "say" not in t.__dict__ and "run_once" not in t.__dict__ and t.lift is lift)
        check("get_dashboard 不开时返回 None", main.get_dashboard(clock, cam, False) is None)
    finally:
        restore()
    check("新参数默认关闭", cfg.DEMO_TTS_CMD is None and cfg.DEMO_AUTO_INTERVAL_S is None
          and cfg.DASHBOARD_PORT == 8080)
    check("新参数在 calib.json 白名单里",
          {"DASHBOARD_PORT", "DEMO_AUTO_INTERVAL_S", "DEMO_TTS_CMD"} <= cfg.CALIB_KEYS)


def test_demo_sim():
    print("2. 仿真演示 3 轮")
    out = io.StringIO()
    old = sys.stdout
    sys.stdout = out
    try:
        ui = demo.run_sim(demo.parse(["--sim", "3", "--plain", "--no-chassis"]))
    finally:
        sys.stdout = old
        restore()
    text = out.getvalue()
    check("跑了 3 轮", len(ui.rows) == 3 and ui.mon.counts["total"] == 3, str(ui.mon.counts))
    check("每轮有标题行", all(f"第 {i} / 3 轮" in text for i in (1, 2, 3)))
    check("有阶段行（带图标和用时）", "🔍  搜索" in text and "🎯  对准" in text and "s   " in text)
    check("结束时打印统计", "本次统计" in text and "成功率" in text and "各阶段平均用时" in text)
    check("没有颜色控制符（--plain）", "\033[" not in text)
    print("   " + "\n   ".join(text.strip().splitlines()[-6:]))


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


def test_dashboard_api():
    print("3. 看板接口")
    mon = dashboard.TaskMonitor(cfg, mode="sim")
    rows, task = run_episodes(mon, n=3, chassis=True)
    srv = dashboard.DashboardServer(mon, port=0, host="127.0.0.1")
    base = f"http://127.0.0.1:{srv.port}"
    try:
        code, ctype, body = _get(base + "/api/state")
        s = json.loads(body.decode("utf-8"))
        check("/api/state 返回 JSON", code == 200 and "application/json" in ctype)
        keys = {"phase", "phase_name", "phase_icon", "phase_s", "k230", "align", "lift_cm", "base_deg",
                "counts", "logs", "J", "J0", "episode", "last_result"}
        check("字段齐全", keys <= set(s), str(sorted(keys - set(s))))
        check("阶段 = 完成", s["phase"] == "done" and s["phase_name"] == "完成")
        ok = sum(r["result"] == "success" for r in rows)
        check("计数正确", s["counts"]["total"] == 3 and s["counts"]["success"] == ok
              and s["counts"]["fail"] == 3 - ok, str(s["counts"]))
        check("最近日志最多 10 条", 0 < len(s["logs"]) <= 10 and all("msg" in l for l in s["logs"]))
        check("最后一条日志是结果", s["logs"][-1]["msg"].startswith("结果："), s["logs"][-1]["msg"])
        check("学到的灵敏度", set(s["J"]) == {"y", "x_deg", "d"}
              and abs(s["J"]["y"] - round(task.axes["y"].j, 3)) < 1e-6, str(s["J"]))
        k = s["k230"]
        check("K230 读数有状态和数值", k["status"] in ("seen", "lost", "heartbeat", "none", "unknown")
              and k["status_text"] and isinstance(k["d"], (int, float)), str(k))
        al = s["align"]
        check("对准误差：每轴有误差、容差、是否在容差内", al and {a["name"] for a in al["axes"]} == {"y", "x_deg", "d"}
              and all({"err", "tol", "ok", "label", "unit"} <= set(a) for a in al["axes"])
              and all(a["ok"] == (abs(a["err"]) <= a["tol"]) for a in al["axes"]), str(al))
        check("底座角度、升降轴高度", isinstance(s["base_deg"], (int, float)) and isinstance(s["lift_cm"], (int, float)),
              f"{s['base_deg']} / {s['lift_cm']}")
        code, ctype, body = _get(base + "/")
        html = body.decode("utf-8")
        check("/ 是中文页面", code == 200 and "text/html" in ctype and "抓取看板" in html and "对准误差" in html)
        with urllib.request.urlopen(base + "/events", timeout=5) as r:
            ctype = r.headers.get("Content-Type", "")
            line = r.readline().decode("utf-8")
        check("/events 是 SSE、推送 JSON", "text/event-stream" in ctype and line.startswith("data: ")
              and json.loads(line[6:])["counts"]["total"] == 3)
        try:
            _get(base + "/nope")
            check("未知路径 404", False)
        except urllib.error.HTTPError as e:
            check("未知路径 404", e.code == 404)
    finally:
        srv.close()


def test_tts(tmp):
    print("4. 语音播报")
    path = os.path.join(tmp, "tts.txt")
    cfg.DEMO_TTS_CMD = [sys.executable, "-c",
                        "import sys; open(sys.argv[1], 'a', encoding='utf-8').write(sys.argv[2] + '\\n')",
                        path, "{text}"]
    out, old = io.StringIO(), sys.stdout
    sys.stdout = out
    try:
        ui = demo.run_sim(demo.parse(["--sim", "1", "--plain", "--no-chassis", "--seed", "3"]))
    finally:
        sys.stdout = old
        restore()
    said = open(path, encoding="utf-8").read().split() if os.path.exists(path) else []
    check("播报了 发现物块", "发现物块" in said, str(said))
    check("抓到时播报 抓到了", ui.rows[0]["result"] != "success" or "抓到了" in said, f"{ui.rows[0]['result']} {said}")
    check("字符串命令按 {text} 替换", demo.Speaker("espeak-ng -v zh {text}").argv("你好") == ["espeak-ng", "-v", "zh", "你好"]
          and demo.Speaker("say").argv("你好") == ["say", "你好"])
    sp = demo.Speaker(None)
    sp.say("不会执行")
    check("默认 None 不播报", sp.spoken == [] and sp._q.qsize() == 0)


class _Stops:
    """包住仿真设备：记录 stop() 调用；lift 第 kill_at 次动作时模拟 Ctrl+C"""

    def __init__(self, dev, log, name, kill_at=None):
        self._dev, self._log, self._name, self._kill_at, self._n = dev, log, name, kill_at, 0

    def __getattr__(self, k):
        attr = getattr(self._dev, k)
        if k == "stop":
            def stop():
                self._log.append(self._name)
                return attr()
            return stop
        if self._kill_at and k in ("move_to", "move_by", "start_move_by"):
            def mv(*a, **kw):
                self._n += 1
                if self._n >= self._kill_at:
                    raise KeyboardInterrupt
                return attr(*a, **kw)
            return mv
        return attr


def run_fake_real(argv, kill_at=None, seed=11):
    cfg.POSES, cfg.USE_CHASSIS = SIM_POSES, False
    stops = []
    # 仿真设备用自己的时钟（传给 run_real 代替 RealClock）
    world, clk, cam, lift, arm, ch = make_sim(cfg, seed * 1000, 1.0, cfg.R_CAM_MM)
    saved = (main.open_devices, main.preflight, main.LOG_DIR)
    main.open_devices = lambda clock: (cam, _Stops(lift, stops, "lift", kill_at), arm, _Stops(ch, stops, "chassis"))
    main.preflight = lambda *d: []
    main.LOG_DIR = tempfile.mkdtemp(prefix="demo_test_log_")
    out, old = io.StringIO(), sys.stdout
    sys.stdout = out
    try:
        ui = demo.run_real(demo.parse(argv), clock=clk, use_guard=False)
    finally:
        sys.stdout = old
        main.open_devices, main.preflight, main.LOG_DIR = saved
        restore()
    return ui, stops, out.getvalue()


def test_real_flow():
    print("5. 真机流程（仿真设备代替）")
    ui, stops, text = run_fake_real(["--rounds", "2", "--auto", "0", "--plain", "--no-chassis"])
    check("连续抓 2 轮（自动间隔）", len(ui.rows) == 2 and "第 2 / 2 轮" in text, str([r["result"] for r in ui.rows]))
    check("复用 main.attach_task 挂看板", getattr(ui.mon.task, "_monitor", None) is ui.mon)
    check("正常结束也关设备（stop）", "lift" in stops)
    ui, stops, text = run_fake_real(["--rounds", "3", "--auto", "0", "--plain"], kill_at=3)
    check("Ctrl+C：打印停止提示", "手动停止" in text)
    check("Ctrl+C：升降轴急停、底盘停车", "lift" in stops and "chassis" in stops, str(stops))
    check("Ctrl+C：阶段变成 已停止，仍打印统计", ui.mon.phase == "stopped" and "本次统计" in text)


def main_():
    tmp = tempfile.mkdtemp(prefix="demo_test_")
    test_no_behaviour_change()
    test_demo_sim()
    test_dashboard_api()
    test_tts(tmp)
    test_real_flow()
    print(f"通过 {len(PASS)} 项，失败 {len(FAIL)} 项")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main_())
