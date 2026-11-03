"""
实时网页看板（任务 16）：手机 / 电脑浏览器打开 http://<树莓派IP>:8080 看抓取过程，适合录屏做画中画

  python3 main.py --once --no-chassis --dashboard     # 跑的同时开看板（默认关）
  python3 demo.py --no-chassis --dashboard            # 演示模式
  python3 demo.py --sim 3 --dashboard --realtime 1    # 电脑上用仿真预览看板（按真实速度播放）

只用标准库：http.server + Server-Sent Events（/events），另有 /api/state 返回一次 JSON。
接入方式和 recorder.py 一样：在 main.attach_task 里用代理 / 钩子挂到任务上，**只看不改**——
包住 task 的几个方法（搜索 / 对准 / 抓取 / 确认 / 放置 / measure / say / run_once），原样调用、原样返回。

页面参数：?pip=1 只显示阶段、K230、对准误差和计数（画中画用的小窗口）
"""
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 阶段：key → (中文名, 图标)
PHASES = {
    "idle": ("待机", "⏸"),
    "search": ("搜索", "🔍"),
    "align": ("对准", "🎯"),
    "reach": ("伸出", "🦾"),
    "verify": ("确认", "👀"),
    "place": ("放置", "📦"),
    "retry": ("重试", "🔁"),
    "done": ("完成", "🏁"),
    "stopped": ("已停止", "⛔"),
}
RESULT_TEXT = {
    "success": "抓到了", "not_found": "没找到物块", "lost": "目标丢失", "align_failed": "对准失败",
    "grasp_failed": "没抓到", "out_of_reach": "够不着", "ghost": "识别到机械臂自己（ghost）",
    "lift_limit": "升降轴到头", "device_error": "设备错误", "crash": "程序异常", "manual": "手柄接管",
}
AXIS_INFO = {"x_deg": ("方位角", "°"), "y": ("高度 Y", "mm"), "d": ("深度", "mm")}
J_UNIT = {"y": "mm/cm", "x_deg": "°/°", "d": "mm/mm"}


def _r(v, n=1):
    return None if v is None else round(float(v), n)


class TaskMonitor:
    """
    挂在 GraspTask 上，收集看板 / 演示模式要显示的状态。线程安全：任务线程写，网页线程读。
    listeners：fn(kind, data) —— kind 为 phase（阶段切换，data 有 old/new/dt）、found、grasp_ok、
    grasp_fail、episode_start、episode、log
    """

    def __init__(self, cfg, clock=None, cam=None, log_n=10, mode="real"):
        self.cfg = cfg
        self.clock = clock
        self.cam = cam
        self.mode = mode
        self._lock = threading.RLock()
        self.listeners = []
        self.task = None
        self.phase = "idle"
        self.phase_t0 = self._now()
        self.episode = 0
        self.round = None                 # 演示模式：{"i": 第几轮, "n": 共几轮}
        self.counts = {"total": 0, "success": 0, "fail": 0}
        self.results = {}
        self.last_result = None
        self.last_m = None                # 最近一次 measure 的结果（Measurement）
        self.last_m_t = None
        self.align_err = None             # 最近一次对准误差 {轴: 误差}
        self.lift_cm = None
        self.logs = deque(maxlen=log_n)
        self.seq = 0                      # 每次状态变化 +1，SSE 用来判断要不要推送

    # ------------------------------------------------ 基础
    def _now(self):
        try:
            return self.clock.now() if self.clock is not None else time.monotonic()
        except Exception:
            return time.monotonic()

    def set_devices(self, clock, cam):
        """设备重开 / 仿真换场景后换新的时钟和 K230 对象"""
        with self._lock:
            if clock is not None and clock is not self.clock:
                self.clock = clock
                self.phase_t0 = self._now()
            self.cam = cam

    def _emit(self, kind, **data):
        for fn in list(self.listeners):
            try:
                fn(kind, data)
            except Exception:
                pass                       # 显示 / 播报出错不能影响抓取

    def _touch(self):
        self.seq += 1

    def set_phase(self, key):
        with self._lock:
            if key == self.phase:
                return
            now = self._now()
            old, dt = self.phase, now - self.phase_t0
            self.phase, self.phase_t0 = key, now
            self._touch()
        self._emit("phase", old=old, new=key, dt=dt)

    def log(self, msg):
        with self._lock:
            self.logs.append({"t": round(self._now(), 2), "wall": time.strftime("%H:%M:%S"), "msg": msg})
            self._touch()
        self._emit("log", msg=msg)

    def on_measure(self, m):
        with self._lock:
            self.last_m, self.last_m_t = m, self._now()
            task = self.task
            if m is not None and task is not None and self.phase in ("search", "align", "retry", "idle"):
                try:
                    e = m.err(task.goal_for(m))
                    self.align_err = {n: e[n] for n in task.axis_names}
                except Exception:
                    pass
            self._touch()

    def on_lift(self, pos):
        if pos is None:
            return
        try:
            pos = float(pos)
        except (TypeError, ValueError):
            return
        with self._lock:
            self.lift_cm = pos
            self._touch()

    def episode_start(self, ep):
        with self._lock:
            self.episode = ep
            self.align_err = None
            self._touch()
        self._emit("episode_start", episode=ep)

    def episode_end(self, row):
        res = row.get("result")
        with self._lock:
            self.counts["total"] += 1
            self.counts["success" if res == "success" else "fail"] += 1
            self.results[res] = self.results.get(res, 0) + 1
            self.last_result = res
            self._touch()
        self.set_phase("done")
        self._emit("episode", row=row)

    # ------------------------------------------------ K230 实时读数
    def _k230(self):
        """直接看 K230 对象（真机 K230.frames / 仿真 SimCam.sent），没有就退回最近一次 measure"""
        cam, now = self.cam, self._now()
        out = {"status": "unknown", "status_text": "未知", "x_deg": None, "y": None, "d": None, "age_s": None}
        last = None
        if cam is not None:
            frames = getattr(cam, "frames", None)
            if frames is None:
                frames = getattr(cam, "sent", None)
            lock = getattr(cam, "_lock", None)
            try:
                if lock is not None:
                    with lock:
                        last = frames[-1] if frames else None
                else:
                    last = frames[-1] if frames else None
            except Exception:
                last = None
        timeout = getattr(self.cfg, "TARGET_TIMEOUT_S", 0.4)
        if last is not None:
            t, x, y, d = last[:4]
            out.update(x_deg=_r(math.degrees(math.atan2(x, max(d, 1.0)))), y=_r(y), d=_r(d, 0), age_s=_r(now - t, 2))
            if now - t <= max(timeout, 0.6):
                out.update(status="seen", status_text="看到物块")
                return out
        elif self.last_m is not None:
            m = self.last_m
            out.update(x_deg=_r(m.x_deg), y=_r(m.y), d=_r(m.d, 0),
                       age_s=None if self.last_m_t is None else _r(now - self.last_m_t, 2))
        link_ok = getattr(cam, "link_ok", None) if cam is not None else None
        if link_ok is not None:
            try:
                ok = link_ok(1.0)
            except Exception:
                ok = None
            if ok:
                out.update(status="heartbeat", status_text="心跳（没看到物块）")
            elif ok is False:
                out.update(status="none", status_text="没有数据（检查 K230）")
        elif cam is not None:
            out.update(status="lost", status_text="没看到物块")
        return out

    # ------------------------------------------------ 给网页的 JSON
    def snapshot(self):
        with self._lock:
            c = self.cfg
            task = self.task
            name, icon = PHASES.get(self.phase, (self.phase, ""))
            tol = getattr(c, "TOL", {})
            align = None
            if self.align_err:
                axes = []
                for n, e in self.align_err.items():
                    label, unit = AXIS_INFO.get(n, (n, ""))
                    t = float(tol.get(n, 1.0))
                    axes.append({"name": n, "label": label, "unit": unit, "err": _r(e, 2), "tol": t,
                                 "ok": abs(e) <= t})
                align = {"axes": axes, "ok": all(a["ok"] for a in axes)}
            lift = self.lift_cm
            if task is not None:
                p = getattr(task.lift, "pos", None)       # 真机 Lift / 代理都有 pos（最近一次已知位置）
                if isinstance(p, (int, float)):
                    lift = p
            j = j0 = {}
            if task is not None:
                j = {n: _r(a.j, 3) for n, a in task.axes.items()}
                j0 = {n: c.AXES[n]["j0"] for n in task.axes if n in getattr(c, "AXES", {})}
            n_ok, n_all = self.counts["success"], self.counts["total"]
            return {
                "seq": self.seq, "mode": self.mode, "wall": time.strftime("%H:%M:%S"),
                "phase": self.phase, "phase_name": name, "phase_icon": icon,
                "phase_s": _r(self._now() - self.phase_t0),
                "episode": self.episode, "round": self.round,
                "k230": self._k230(),
                "align": align,
                "lift_cm": _r(lift, 2),
                "base_deg": None if task is None else _r(task.base_deg),
                "counts": dict(self.counts, rate=None if not n_all else round(100.0 * n_ok / n_all)),
                "results": dict(self.results),
                "last_result": self.last_result,
                "last_result_text": RESULT_TEXT.get(self.last_result, self.last_result),
                "logs": list(self.logs),
                "J": j, "J0": j0, "J_unit": {n: J_UNIT.get(n, "") for n in j},
            }


class _LiftProxy:
    """包住升降轴：调用照常执行，记下返回的高度（真机 Lift 本身有 pos，仿真 SimLift 没有）"""
    WATCH = {"move_to", "move_by", "wait_move", "position"}

    def __init__(self, dev, mon):
        self._dev, self._mon = dev, mon

    def __getattr__(self, k):
        attr = getattr(self._dev, k)
        if k not in self.WATCH or not callable(attr):
            return attr

        def wrapped(*a, **kw):
            ret = attr(*a, **kw)
            self._mon.on_lift(ret)
            return ret
        return wrapped


def wrap_lift(lift, mon):
    return _LiftProxy(lift, mon)


def hook_task(task, mon):
    """把看板挂到任务上；同一个 task 重复调用（设备重开）不会挂两次。不改变任务行为"""
    if getattr(task, "_monitor", None) is mon:
        return
    task._monitor = mon
    mon.task = task

    def wrap(name, before=None, after=None):
        orig = getattr(task, name)

        def f(*a, **kw):
            if before:
                before(*a)
            ret = orig(*a, **kw)
            if after:
                after(ret)
            return ret
        setattr(task, name, f)

    wrap("say", before=lambda *a: mon.log(" ".join(str(x) for x in a)))
    # 伸出合爪之后的第一次测量就是"确认"（VERIFY = lift_check / auto）；observe 模式由 _observe_check 切换
    wrap("measure", before=lambda *a: mon.phase == "reach" and mon.set_phase("verify"), after=mon.on_measure)

    def search_done(m):
        if m is not None:
            mon._emit("found", d=m.d, x_deg=m.x_deg, y=m.y)
    wrap("search", before=lambda *a: mon.set_phase("search"), after=search_done)
    wrap("align", before=lambda *a: mon.set_phase("align"))
    wrap("grasp", before=lambda *a: mon.set_phase("reach"),
         after=lambda ok: mon._emit("grasp_ok" if ok else "grasp_fail"))
    for name in ("_observe_check", "_lift_check"):
        if hasattr(task, name):
            wrap(name, before=lambda *a: mon.set_phase("verify"))
    wrap("place", before=lambda *a: mon.set_phase("place"))
    wrap("release_and_back_off", before=lambda *a: mon.set_phase("retry"))

    orig_run = task.run_once

    def run_once():
        mon.episode_start(task.episode + 1)
        try:
            row = orig_run()
        except BaseException:
            mon.set_phase("stopped")
            raise
        mon.episode_end(row)
        return row
    task.run_once = run_once


# ============================================================ 网页
PAGE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>抓取看板</title>
<style>
:root{--bg:#0b0f14;--card:#151c24;--fg:#e8eef5;--dim:#8a99a8;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--acc:#38bdf8}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font-family:"Noto Sans CJK SC","PingFang SC","Microsoft YaHei",sans-serif;
 font-size:20px;padding:12px}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border-radius:14px;padding:14px 16px}
.t{color:var(--dim);font-size:16px;margin-bottom:6px}
.phase{display:flex;align-items:center;gap:16px;flex-wrap:wrap}
.phase .ic{font-size:64px}
.phase .nm{font-size:60px;font-weight:800;letter-spacing:4px;white-space:nowrap}
.phase .s{font-size:24px;color:var(--dim)}
.big{font-size:40px;font-weight:700;font-variant-numeric:tabular-nums}
.kv{display:flex;justify-content:space-between;gap:10px;font-variant-numeric:tabular-nums}
.kv b{font-size:34px}
.badge{display:inline-block;padding:4px 14px;border-radius:999px;font-weight:700;font-size:22px}
.seen{background:var(--ok);color:#03210f}.heartbeat,.lost{background:var(--warn);color:#2a1a00}.none{background:var(--bad)}
.unknown{background:#334155}
.bar{position:relative;height:30px;background:#0f172a;border-radius:8px;margin:4px 0 12px;overflow:hidden}
.bar .tol{position:absolute;top:0;bottom:0;background:#1e3a5f}
.bar .zero{position:absolute;top:0;bottom:0;width:2px;left:50%;background:#64748b}
.bar .m{position:absolute;top:3px;bottom:3px;width:14px;margin-left:-7px;border-radius:4px;background:var(--warn);transition:left .25s}
.bar.ok .tol{background:#14532d}.bar.ok .m{background:var(--ok)}
.al{display:flex;justify-content:space-between;font-size:20px;font-variant-numeric:tabular-nums}
.cnt{display:grid;grid-template-columns:repeat(4,1fr);text-align:center}
.cnt b{display:block;font-size:42px;white-space:nowrap}
.ok{color:var(--ok)}.bad{color:var(--bad)}
.logs{font-family:ui-monospace,Consolas,monospace;font-size:16px;line-height:1.45;white-space:pre-wrap;word-break:break-all}
.logs div{border-bottom:1px solid #1f2933;padding:2px 0}
.res{font-size:26px;font-weight:700;margin-top:6px}
#conn{position:fixed;right:10px;bottom:6px;font-size:14px;color:var(--dim)}
body.pip .hide-pip{display:none}
body.pip{font-size:18px}
body.pip .phase .nm{font-size:48px}
</style></head><body>
<div id="conn">连接中…</div>
<div class="grid">
 <div class="card"><div class="t">当前阶段 <span id="round"></span></div>
  <div class="phase"><span class="ic" id="pic">⏸</span><span class="nm" id="pnm">待机</span><span class="s" id="ps"></span></div>
  <div class="res" id="res"></div></div>
 <div class="card"><div class="t">K230 实时读数</div>
  <div><span class="badge unknown" id="kst">未知</span></div>
  <div class="kv" style="margin-top:8px"><span>方位角</span><b id="kx">—</b></div>
  <div class="kv"><span>Y</span><b id="ky">—</b></div>
  <div class="kv"><span>深度</span><b id="kd">—</b></div></div>
 <div class="card"><div class="t">对准误差（深色带 = 容差，进入容差变绿）</div><div id="align"><div class="t">还没有对准数据</div></div></div>
 <div class="card"><div class="t">本轮统计</div>
  <div class="cnt"><div>已抓<b id="ct">0</b></div><div>成功<b class="ok" id="cs">0</b></div>
  <div>失败<b class="bad" id="cf">0</b></div><div>成功率<b id="cr">—</b></div></div></div>
 <div class="card hide-pip"><div class="t">执行器</div>
  <div class="kv"><span>升降轴高度</span><b id="lift">—</b></div>
  <div class="kv"><span>底座角度</span><b id="base">—</b></div></div>
 <div class="card hide-pip"><div class="t">学到的灵敏度 J（括号内为初值）</div><div id="jj"></div></div>
 <div class="card hide-pip" style="grid-column:1/-1"><div class="t">最近 10 条日志</div><div class="logs" id="logs"></div></div>
</div>
<script>
const $=id=>document.getElementById(id);
if(new URLSearchParams(location.search).get("pip"))document.body.classList.add("pip");
const f=(v,n,u)=>v===null||v===undefined?"—":(n===0?Math.round(v):(+v).toFixed(n))+(u||"");
const sg=(v,n,u)=>v===null||v===undefined?"—":((v>0?"+":"")+(+v).toFixed(n))+(u||"");
const esc=s=>String(s).replace(/[&<>]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;"}[c]));
function render(s){
 $("pic").textContent=s.phase_icon;$("pnm").textContent=s.phase_name;$("ps").textContent=f(s.phase_s,1,"s");
 $("round").textContent=s.round?`（第 ${s.round.i}${s.round.n?" / "+s.round.n:""} 轮）`:(s.episode?`（第 ${s.episode} 次）`:"");
 $("res").innerHTML=s.last_result?("上一次："+`<span class="${s.last_result==="success"?"ok":"bad"}">${esc(s.last_result_text)}</span>`):"";
 const k=s.k230;$("kst").className="badge "+k.status;$("kst").textContent=k.status_text;
 $("kx").textContent=sg(k.x_deg,1,"°");$("ky").textContent=sg(k.y,0," mm");$("kd").textContent=f(k.d,0," mm");
 if(s.align){let h="";for(const a of s.align.axes){const span=4*a.tol,cl=x=>Math.max(0,Math.min(100,50+50*x/span));
  h+=`<div class="al"><span>${a.label}</span><span class="${a.ok?"ok":""}">${sg(a.err,a.unit==="°"?1:0,a.unit)} / ±${a.tol}${a.unit}</span></div>`+
  `<div class="bar ${a.ok?"ok":""}"><div class="tol" style="left:${cl(-a.tol)}%;right:${100-cl(a.tol)}%"></div><div class="zero"></div>`+
  `<div class="m" style="left:${cl(a.err)}%"></div></div>`;}$("align").innerHTML=h;}
 const c=s.counts;$("ct").textContent=c.total;$("cs").textContent=c.success;$("cf").textContent=c.fail;
 $("cr").textContent=c.rate===null?"—":c.rate+"%";
 $("lift").textContent=f(s.lift_cm,1," cm");$("base").textContent=sg(s.base_deg,1,"°");
 $("jj").innerHTML=Object.keys(s.J).map(n=>`<div class="kv"><span>${n}</span><b>${s.J[n]} <small style="font-size:16px;color:var(--dim)">${s.J_unit[n]||""}（${s.J0[n]}）</small></b></div>`).join("");
 $("logs").innerHTML=s.logs.slice().reverse().map(l=>`<div>${l.wall} ${esc(l.msg)}</div>`).join("");
}
let poll=null;
function startPoll(){if(poll)return;poll=setInterval(()=>fetch("/api/state").then(r=>r.json()).then(s=>{render(s);$("conn").textContent="轮询 "+s.wall}).catch(()=>$("conn").textContent="断开"),700)}
if(window.EventSource){const es=new EventSource("/events");
 es.onmessage=e=>{const s=JSON.parse(e.data);render(s);$("conn").textContent=(s.mode==="sim"?"仿真 ":"实时 ")+s.wall};
 es.onerror=()=>{$("conn").textContent="重连中…"};}else startPoll();
</script></body></html>
"""


class DashboardServer:
    """后台线程跑 HTTP 服务；port=0 自动选一个空闲端口（测试用）"""

    def __init__(self, mon, port=8080, host="0.0.0.0", interval_s=0.3):
        self.mon = mon
        self.interval_s = interval_s
        self._stop = threading.Event()
        srv = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass                         # 不往终端打印每次请求

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?")[0]
                if path in ("/", "/index.html"):
                    self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
                elif path == "/api/state":
                    body = json.dumps(srv.mon.snapshot(), ensure_ascii=False).encode("utf-8")
                    self._send(200, "application/json; charset=utf-8", body)
                elif path == "/events":
                    self._sse()
                else:
                    self._send(404, "text/plain; charset=utf-8", "没有这个页面".encode("utf-8"))

            def _sse(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                last, beat = None, 0.0
                try:
                    while not srv._stop.is_set():
                        s = srv.mon.snapshot()
                        # 状态没变也每秒推一次（阶段用时、K230 读数一直在变）
                        if s["seq"] != last or time.monotonic() - beat > 1.0:
                            data = json.dumps(s, ensure_ascii=False)
                            self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last, beat = s["seq"], time.monotonic()
                        srv._stop.wait(srv.interval_s)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass                     # 浏览器关了

        self.httpd = ThreadingHTTPServer((host, port), H)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass


def local_ips():
    """树莓派的局域网 IP（打印给操作员看手机该打开哪个地址）"""
    import socket
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))          # 不会真的发包，只是让系统选出口网卡
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return [ip for ip in ips if not ip.startswith("127.")] or ["<树莓派IP>"]


def start(mon, port):
    srv = DashboardServer(mon, port)
    for ip in local_ips():
        print(f"网页看板：手机 / 电脑浏览器打开 http://{ip}:{srv.port}   （画中画小窗口加 ?pip=1）")
    return srv
