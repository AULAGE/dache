# -*- coding: utf-8 -*-
"""
ROBOTAC 比赛材料一键生成：设计报告（docx）+ 答辩 PPT（pptx）+ 作品信息概要表（docx）

    python build_materials.py                 用 data.json 生成三份文件到 out/
    python build_materials.py --pdf           另外用 LibreOffice 转 PDF（可选；Windows 上建议用 Office 自己导出）
    python build_materials.py --figures       重新画架构图和伺服流程图
    python build_materials.py --only ppt      只生成某一份：report / ppt / summary
    python build_materials.py --data 某.json --out 某目录

输入：
    ../设计报告_v2_草稿.md      正文（写法见文件开头的说明）
    data.json                   真机数据、队员、照片视频文件名等（没填的显示黄色占位）
    media/                      照片、截图、视频、bench.csv
    ../templates/               旧版报告 / 概要表 / PPT（样式、封面、页脚、母版、配色都从这里来）
输出：
    out/设计报告_<标签>.docx、out/答辩PPT_<标签>.pptx、out/作品信息概要表_<标签>.docx、out/缺项清单.txt

依赖：python-docx、python-pptx、matplotlib（见 requirements.txt）。Windows / Linux 都能运行。
原则：不编造实测数据——data.json 没填的一律显示【待真机：…】黄色占位；仿真数字一律注明“仿真”。
"""
import argparse
import copy
import csv
import datetime
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOTAC = os.path.dirname(HERE)
TEMPLATES = os.path.join(ROBOTAC, "templates")
DRAFT_MD = os.path.join(ROBOTAC, "设计报告_v2_草稿.md")
T_REPORT = os.path.join(TEMPLATES, "设计报告_旧版.docx")
T_SUMMARY = os.path.join(TEMPLATES, "作品信息概要表_旧版.docx")
T_PPT = os.path.join(TEMPLATES, "答辩PPT_旧版_视频已去除.pptx")
T_IMG = os.path.join(TEMPLATES, "图片")

try:  # Windows 控制台默认 GBK，打印 ✅ 之类会报错
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

try:
    import docx  # noqa: F401
    import pptx  # noqa: F401
except ImportError:
    print("缺少依赖：请先运行  pip install -r requirements.txt")
    raise

from docx import Document
from docx.enum.text import WD_COLOR_INDEX
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt
from lxml import etree
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn as pqn
from pptx.util import Cm as PCm
from pptx.util import Emu
from pptx.util import Pt as PPt

PH_RE = re.compile(r"【[^】]*】")


# =====================================================================================  数据与占位
class Ctx:
    """data.json + 缺项登记"""

    def __init__(self, data, data_dir):
        self.d = data
        self.data_dir = data_dir
        self.missing = []          # (文件, 说明)
        self.where = ""
        self.bench = load_bench(self)

    # ---------- 取值
    def get(self, path, default=None):
        cur = self.d
        for k in path.split("."):
            if isinstance(cur, dict) and k in cur:
                cur = cur[k]
            elif isinstance(cur, list) and k.isdigit() and int(k) < len(cur):
                cur = cur[int(k)]
            else:
                return default
        return cur

    @staticmethod
    def empty(v):
        return v is None or (isinstance(v, str) and v.strip() == "") or v == [] or v == {}

    def need(self, label, kind="待真机"):
        """登记缺项并返回占位文字"""
        self.missing.append((self.where, f"{kind}：{label}"))
        return f"【{kind}：{label}】"

    def val(self, path, label=None, fmt=None):
        v = self.get(path)
        if self.empty(v):
            return self.need(label or path.split(".")[-1])
        if fmt and isinstance(v, (int, float)):
            return fmt.format(v)
        return str(v)

    def fact(self, name):
        f = self.get("核对事实." + name) or {}
        v = f.get("值")
        if self.empty(v):
            return self.need(name, "请核对")
        if f.get("已核对"):
            return str(v)
        self.missing.append((self.where, f"请核对：{name}（当前写的是：{v}）"))
        return f"{v}【请核对】"

    def switch(self, name):
        return self.get("开关." + name)

    def media(self, name):
        """照片/视频文件名 → 绝对路径；不存在返回 None"""
        if self.empty(name):
            return None
        cands = [name] if os.path.isabs(name) else [os.path.join(HERE, "media", name), os.path.join(HERE, name),
                                                       os.path.join(self.data_dir, name)]
        for c in cands:
            if os.path.isfile(c):
                return os.path.abspath(c)
        self.missing.append((self.where, f"文件不存在：{name}（放到 build/media/ 里）"))
        return None

    def substitute(self, s):
        """把 {{…}} 换成值或占位文字（占位文字带【】，之后统一黄色高亮）"""
        def rep(m):
            body = m.group(1).strip()
            if body.startswith("核对:"):
                return self.fact(body[3:])
            if body.startswith("图号:") or body.startswith("表号:"):
                kind, key = body[:2], body[3:]
                n = NUMBERS.get((kind[0], key))
                return str(n) if n else self.need(f"{kind}{key}", "请核对")
            path, _, label = body.partition("|")
            return self.val(path.strip(), label.strip() or None)
        return re.sub(r"\{\{(.+?)\}\}", rep, s)


NUMBERS = {}       # ("图"/"表", 键) → 编号，正文生成前预先算好


def pct(a, n):
    return f"{100.0 * a / n:.0f}%" if n else "—"


def load_bench(ctx):
    """bench：优先读 csv（树莓派 logs/bench.csv），否则用 data.json 手填的格子。
    返回 {pos: {"次数","抓到","误报","平均用时_s"}} 和来源说明；没有任何数据返回 None"""
    b = ctx.get("bench") or {}
    cells = {}
    src = ""
    path = b.get("csv文件")
    if not Ctx.empty(path):
        full = ctx.media(path)
        if full:
            with open(full, encoding="utf-8-sig", newline="") as f:
                rows = list(csv.DictReader(f))
            if b.get("只算无底盘", True):
                rows = [r for r in rows if str(r.get("chassis", "")).strip().lower() in ("", "0", "false", "n", "no")]
            rnd = (b.get("轮次") or "").strip()
            rounds = [r.get("round", "") for r in rows]
            if rnd == "":
                rnd = rounds[-1] if rounds else ""
            if rnd != "all":
                rows = [r for r in rows if r.get("round", "") == rnd]
            for r in rows:
                try:
                    p = int(r["pos"])
                except (KeyError, ValueError):
                    continue
                c = cells.setdefault(p, {"次数": 0, "抓到": 0, "误报": 0, "_t": []})
                c["次数"] += 1
                ok = r.get("truth") == "y"
                c["抓到"] += ok
                c["误报"] += (not ok) and r.get("prog_result") == "success"
                try:
                    c["_t"].append(float(r.get("time_s") or "nan"))
                except ValueError:
                    pass
            for c in cells.values():
                ts = [t for t in c.pop("_t") if t == t]
                c["平均用时_s"] = round(sum(ts) / len(ts), 1) if ts else None
            src = f"bench.csv（{'全部轮次' if rnd == 'all' else '第 ' + str(rnd) + ' 轮'}，共 {len(rows)} 次）"
    if not cells:
        for k, c in (b.get("格子") or {}).items():
            if k.isdigit() and not Ctx.empty(c.get("次数")):
                cells[int(k)] = {"次数": c.get("次数"), "抓到": c.get("抓到") or 0, "误报": c.get("误报"),
                                 "平均用时_s": c.get("平均用时_s")}
        if cells:
            src = "data.json 手填"
    if not cells:
        return None
    tot_n = sum(c["次数"] for c in cells.values())
    tot_ok = sum(c["抓到"] for c in cells.values())
    tot_fp = sum((c.get("误报") or 0) for c in cells.values())
    ts = [(c["平均用时_s"], c["次数"]) for c in cells.values() if c.get("平均用时_s") is not None]
    avg_t = round(sum(t * n for t, n in ts) / sum(n for _, n in ts), 1) if ts else None
    return {"cells": cells, "src": src, "n": tot_n, "ok": tot_ok, "fp": tot_fp, "t": avg_t}


def headline(ctx):
    """低平台无底盘总成功率：bench 优先，否则 data.json 真机.低平台无底盘"""
    if ctx.bench:
        return ctx.bench["ok"], ctx.bench["n"], ctx.bench["t"]
    h = ctx.get("真机.低平台无底盘") or {}
    if not Ctx.empty(h.get("总次数")):
        return h.get("抓到") or 0, h["总次数"], h.get("平均用时_s")
    return None, None, None


# =====================================================================================  Markdown 解析
def apply_conditionals(text, ctx):
    pat = re.compile(r"\[\[如果:(.+?)\]\](.*?)(?:\[\[否则\]\](.*?))?\[\[结束\]\]", re.S)

    def rep(m):
        name, a, b = m.group(1).strip(), m.group(2), m.group(3) or ""
        v = ctx.switch(name)
        if v is True:
            return a
        if v is False:
            return b
        keep = a if a.strip() else b
        ctx.missing.append(("开关", f"请核对：开关“{name}”还没定（data.json 开关）"))
        keep = keep.rstrip("\n")
        return keep + f"【请核对：{name}？】" + ("\n" if a.endswith("\n") else "")
    return pat.sub(rep, text)


def parse_md(text):
    """→ 块列表：(类型, ...)"""
    blocks = []
    lines = text.splitlines()
    i = 0
    pending_caption = None
    while i < len(lines):
        ln = lines[i].rstrip()
        s = ln.strip()
        i += 1
        if not s or s.startswith(">") or s == "---" or s.startswith("# "):
            continue
        m = re.match(r"^(#{2,4})\s+(.*)$", s)
        if m:
            level = len(m.group(1)) - 1
            t = m.group(2)
            t = re.sub(r"^第\s*\d+\s*章\s*", "", t) if level == 1 else re.sub(r"^\d+(\.\d+)+\s*", "", t)
            blocks.append(("h", level, t))
            continue
        m = re.match(r"^表\{(.+?)\}[：:]\s*(.*)$", s)
        if m:
            pending_caption = (m.group(1), m.group(2))
            continue
        if s.startswith("|"):
            rows = []
            i -= 1
            while i < len(lines) and lines[i].strip().startswith("|"):
                r = lines[i].strip()
                i += 1
                if re.match(r"^\|[\s:\-|]+\|$", r):
                    continue
                cells = [c.strip().replace("\\|", "|") for c in re.split(r"(?<!\\)\|", r.strip("|"))]
                rows.append(cells)
            key, cap = pending_caption or (None, "")
            blocks.append(("table", key, cap, rows))
            pending_caption = None
            continue
        m = re.match(r"^!\[(.*?)\]\((\w+):(.+?)\)$", s)
        if m:
            blocks.append(("img", m.group(1), m.group(2), m.group(3)))
            continue
        m = re.match(r"^\[\[表:(.+?)\]\]$", s)
        if m:
            blocks.append(("special", m.group(1)))
            continue
        if s.startswith("$$"):
            blocks.append(("eq", s[2:].strip()))
            continue
        if s.startswith("- "):
            blocks.append(("li", s[2:]))
            continue
        blocks.append(("p", s))
    return blocks


FIG_ALIAS = {"系统总体架构.png": "架构", "自适应视觉伺服流程.png": "伺服", "大车三维建模分解图.png": "三维",
             "环境感知板PCB实物.png": "感知板", "抓取序列": "序列"}

SPECIAL_CAPTIONS = {"仿真": "离线仿真统计（仿真结果，不代表真机）", "bench": "真机测试矩阵结果（人工判定真实结果）",
                    "改进对比": "改进前后对比（真机，同一测试矩阵）", "阶段用时": "一次抓取各阶段平均用时（真机）"}


def number_blocks(blocks):
    NUMBERS.clear()
    nf = nt = 0
    for b in blocks:
        if b[0] == "img":
            nf += 1
            key = FIG_ALIAS.get(b[3], b[3])
            NUMBERS[("图", key)] = nf
            NUMBERS[("图", b[3])] = nf
        elif b[0] == "table":
            nt += 1
            if b[1]:
                NUMBERS[("表", b[1])] = nt
        elif b[0] == "special":
            nt += 1
            NUMBERS[("表", b[1])] = nt


def segments(s):
    """'**粗**【占位】普通' → [(文字, 粗体?, 高亮?)]"""
    out = []
    bold = False
    for part in re.split(r"(\*\*)", s):
        if part == "**":
            bold = not bold
            continue
        pos = 0
        for m in PH_RE.finditer(part):
            if m.start() > pos:
                out.append((part[pos:m.start()], bold, False))
            out.append((m.group(0), bold, True))
            pos = m.end()
        if pos < len(part):
            out.append((part[pos:], bold, False))
    return [x for x in out if x[0]]


def strip_md(s):
    return s.replace("**", "").replace("`", "")


# =====================================================================================  图片处理
def prepared_image(path, out_dir, max_px=1800):
    """大照片缩到 max_px 并转 JPEG，避免文件太大；PNG 图表保持原样"""
    try:
        from PIL import Image
    except ImportError:
        return path
    try:
        im = Image.open(path)
        w, h = im.size
        big = os.path.getsize(path) > 1_500_000 or max(w, h) > max_px
        if not big:
            return path
        cache = os.path.join(out_dir, "_cache")
        os.makedirs(cache, exist_ok=True)
        name = os.path.splitext(os.path.basename(path))[0] + f"_{max_px}.jpg"
        dst = os.path.join(cache, name)
        if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(path):
            im = im.convert("RGB")
            im.thumbnail((max_px, max_px))
            im.save(dst, "JPEG", quality=85)
        return dst
    except Exception:
        return path


def image_size(path):
    try:
        from PIL import Image
        with Image.open(path) as im:
            return im.size
    except Exception:
        return (16, 9)


# =====================================================================================  设计报告 docx
W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"


def _el(tag, **attrs):
    e = OxmlElement(tag)
    for k, v in attrs.items():
        e.set(qn(k), str(v))
    return e


class ReportBuilder:
    def __init__(self, ctx, out_dir):
        self.ctx = ctx
        self.out_dir = out_dir
        self.doc = Document(T_REPORT)
        self.body = self.doc.element.body
        self.headings = []      # (级别, 编号文字, 标题)
        self.h_count = [0, 0, 0]
        self.fig_n = 0
        self.tab_n = 0
        self._grab_exemplars()
        self._clear_old_body()

    # ---------- 模板
    def _grab_exemplars(self):
        ex = {}
        for p in self.doc.paragraphs:
            sid = p.style.style_id if p.style is not None else None
            key = {"1": "h1", "2": "h2", "3": "h3", "a0": "body", "a4": "cap", "af0": "fig"}.get(sid)
            if key and key not in ex and p._p.pPr is not None and p.text.strip() or (key == "fig" and key not in ex):
                if key is None or p._p.pPr is None:
                    continue
                pPr = copy.deepcopy(p._p.pPr)
                for x in pPr.findall(qn("w:sectPr")):
                    pPr.remove(x)
                for x in pPr.findall(qn("w:rPr")):
                    pPr.remove(x)
                if key == "h1" and "h1" not in ex:
                    sp = pPr.find(qn("w:spacing"))
                    if sp is not None:          # 第一章前有分节，模板里加大了段前；正文其余各章用正常段前
                        sp.set(qn("w:before"), "468")
                ex.setdefault(key, pPr)
        t0 = self.doc.tables[0]._tbl
        ex["tblPr"] = copy.deepcopy(t0.tblPr)
        cellp = t0.find(".//" + qn("w:p"))
        ex["cell"] = copy.deepcopy(cellp.pPr)
        if "h3" not in ex:
            ex["h3"] = copy.deepcopy(ex["h2"])
            ex["h3"].find(qn("w:pStyle")).set(qn("w:val"), "3")
        self.ex = ex

    def _clear_old_body(self):
        children = list(self.body)
        # 第一节结束的段落（pPr 里带 sectPr）之后全部删掉，只留最后的 sectPr
        cut = None
        for idx, el in enumerate(children):
            if el.tag == qn("w:p") and el.find(qn("w:pPr") + "/" + qn("w:sectPr")) is not None:
                cut = idx
                break
        for el in children[cut + 1:]:
            if el.tag != qn("w:sectPr"):
                self.body.remove(el)

    def _new_p(self, key, align=None):
        p = self.doc.add_paragraph()
        p._p.insert(0, copy.deepcopy(self.ex[key]))
        if align:
            jc = p._p.pPr.find(qn("w:jc"))
            if jc is None:
                jc = _el("w:jc")
                p._p.pPr.append(jc)
            jc.set(qn("w:val"), align)
        return p

    def _runs(self, p, text, size=None, force_bold=None):
        for t, b, hl in segments(text):
            r = p.add_run(t)
            if b or force_bold:
                r.bold = True
            if hl:
                r.font.highlight_color = WD_COLOR_INDEX.YELLOW
            if size:
                r.font.size = Pt(size)

    def text(self, s):
        return self.ctx.substitute(s)

    # ---------- 块
    def heading(self, level, title):
        self.h_count[level - 1] += 1
        for j in range(level, 3):
            self.h_count[j] = 0
        num = (f"第{self.h_count[0]}章" if level == 1 else ".".join(str(x) for x in self.h_count[:level]))
        self.headings.append((level, num, title))
        p = self._new_p({1: "h1", 2: "h2", 3: "h3"}[level])
        self._runs(p, title)
        self.ctx.where = f"报告 {num} {title}"

    def para(self, s, key="body", align=None):
        p = self._new_p(key, align)
        self._runs(p, self.text(s))
        return p

    def caption(self, kind, n, title):
        p = self._new_p("cap")
        p.add_run(f"{kind} ")
        r = p.add_run()
        r._r.append(_el("w:fldChar", **{"w:fldCharType": "begin"}))
        r = p.add_run()
        it = _el("w:instrText")
        it.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
        it.text = f" SEQ {kind} \\* ARABIC "
        r._r.append(it)
        r = p.add_run()
        r._r.append(_el("w:fldChar", **{"w:fldCharType": "separate"}))
        p.add_run(str(n))
        r = p.add_run()
        r._r.append(_el("w:fldChar", **{"w:fldCharType": "end"}))
        p.add_run("  ")
        self._runs(p, self.text(title))

    def table(self, rows, caption=None, widths=None, header=True, highlight_cells=()):
        self.tab_n += 1
        if caption is not None:
            self.caption("表", self.tab_n, caption)
        rows = [[self.text(c) for c in r] for r in rows]
        ncol = max(len(r) for r in rows)
        rows = [r + [""] * (ncol - len(r)) for r in rows]
        t = self.doc.add_table(rows=len(rows), cols=ncol)
        tbl = t._tbl
        tbl.remove(tbl.tblPr)
        tbl.insert(0, copy.deepcopy(self.ex["tblPr"]))
        total = 8306
        if widths is None:
            lens = [max(min(len(strip_md(r[c])), 30) for r in rows) + 2 for c in range(ncol)]
            s = sum(lens)
            widths = [max(900, int(total * x / s)) for x in lens]
            k = total / sum(widths)
            widths = [int(w * k) for w in widths]
        grid = tbl.tblGrid
        for gc, w in zip(grid.findall(qn("w:gridCol")), widths):
            gc.set(qn("w:w"), str(w))
        for ri, (row, r) in enumerate(zip(t.rows, rows)):
            trPr = row._tr.get_or_add_trPr()
            trPr.append(_el("w:jc", **{"w:val": "center"}))
            for ci, (cell, txt) in enumerate(zip(row.cells, r)):
                tcPr = cell._tc.get_or_add_tcPr()
                tcW = tcPr.find(qn("w:tcW"))
                if tcW is None:
                    tcW = _el("w:tcW")
                    tcPr.append(tcW)
                tcW.set(qn("w:w"), str(widths[ci]))
                tcW.set(qn("w:type"), "dxa")
                tcPr.append(_el("w:vAlign", **{"w:val": "center"}))
                if ri == 0 and header:
                    b = _el("w:tcBorders")
                    b.append(_el("w:bottom", **{"w:val": "single", "w:sz": "6", "w:space": "0", "w:color": "000000"}))
                    tcPr.insert(1, b)
                p = cell.paragraphs[0]
                p._p.insert(0, copy.deepcopy(self.ex["cell"]))
                for t_, bld, hl in segments(txt):
                    run = p.add_run(t_)
                    run.font.size = Pt(10.5)
                    if bld or (ri == 0 and header):
                        run.bold = True
                    if hl or (ri, ci) in highlight_cells:
                        run.font.highlight_color = WD_COLOR_INDEX.YELLOW
        return t

    def picture(self, path, caption, width_cm=14.5, max_h_cm=11.0):
        self.fig_n += 1
        p = self._new_p("fig")
        w, h = image_size(path)
        width = width_cm
        if h / w * width > max_h_cm:
            width = max_h_cm * w / h
        p.add_run().add_picture(prepared_image(path, self.out_dir), width=Cm(width))
        self.caption("图", self.fig_n, caption)

    def placeholder_fig(self, caption, hint):
        self.fig_n += 1
        p = self._new_p("fig")
        for _ in range(2):
            p.add_run().add_break()
        r = p.add_run(self.ctx.need(f"{strip_md(caption)}（{hint}）", "待图"))
        r.font.highlight_color = WD_COLOR_INDEX.YELLOW
        for _ in range(2):
            p.add_run().add_break()
        self.caption("图", self.fig_n, caption)

    def image_block(self, caption, kind, ref):
        caption_t = caption
        if kind == "figure":
            path = os.path.join(HERE, "figures", ref)
            return self.picture(path, caption_t, width_cm=15.5)
        if kind == "template":
            return self.picture(os.path.join(T_IMG, ref), caption_t)
        # photo
        if ref == "抓取序列":
            return self.sequence(caption_t)
        f = self.ctx.get("照片." + ref)
        path = self.ctx.media(f)
        if path:
            return self.picture(path, caption_t)
        return self.placeholder_fig(caption_t, f"data.json 照片.{ref}")

    def sequence(self, caption):
        names = ["搜索", "对准", "伸出", "合爪", "确认", "放置"]
        files = (self.ctx.get("照片.抓取序列") or []) + [""] * 6
        paths = [self.ctx.media(f) for f in files[:6]]
        if not any(paths):
            return self.placeholder_fig(caption, "data.json 照片.抓取序列，6 张：" + "、".join(names))
        self.fig_n += 1
        t = self.doc.add_table(rows=4, cols=3)
        for i, (nm, pth) in enumerate(zip(names, paths)):
            cell = t.cell((i // 3) * 2, i % 3)
            p = cell.paragraphs[0]
            p._p.insert(0, copy.deepcopy(self.ex["fig"]))
            if pth:
                p.add_run().add_picture(prepared_image(pth, self.out_dir, 1200), width=Cm(4.9))
            else:
                r = p.add_run(self.ctx.need(f"抓取序列第 {i + 1} 张（{nm}）", "待图"))
                r.font.highlight_color = WD_COLOR_INDEX.YELLOW
            c2 = t.cell((i // 3) * 2 + 1, i % 3).paragraphs[0]
            c2._p.insert(0, copy.deepcopy(self.ex["fig"]))
            r = c2.add_run(f"（{chr(ord('a') + i)}）{nm}")
            r.font.size = Pt(9)
        self.caption("图", self.fig_n, caption)

    # ---------- 程序生成的数据表
    def special(self, name):
        ctx = self.ctx
        cap = SPECIAL_CAPTIONS.get(name, name)
        if name == "仿真":
            rows = [["场景（仿真）", "命令", "真实抓到 / 次数", "成功率", "平均对准步数", "平均用时（仿真）"]]
            for r in ctx.get("仿真.行") or []:
                rows.append([r["场景"], r["命令"].replace("python3 ", ""), f"{r['抓到']} / {r['次数']}",
                             pct(r["抓到"], r["次数"]), str(r.get("平均对准步数", "—")),
                             f"{r['平均用时_s']} s" if r.get("平均用时_s") is not None else "—"])
            if len(rows) == 1:
                rows.append([ctx.need("仿真结果", "待补")] + [""] * 5)
            self.table(rows, cap)
            note = ctx.get("仿真.备注")
            if note:
                self.para(f"注：以上均为仿真结果。{note}。")
        elif name == "bench":
            self.bench_table(cap)
        elif name == "改进对比":
            rows = [["配置", "真实抓到 / 次数", "成功率"]]
            for r in ctx.get("改进对比.行") or []:
                if Ctx.empty(r.get("次数")):
                    rows.append([r["配置"], ctx.need(f"{r['配置']} 的次数和抓到次数"), ""])
                else:
                    rows.append([r["配置"], f"{r.get('抓到') or 0} / {r['次数']}", pct(r.get("抓到") or 0, r["次数"])])
            self.table(rows, cap)
        elif name == "阶段用时":
            keys = ["搜索", "对准", "伸出夹取", "确认", "放置"]
            vals = [ctx.get("各阶段用时_s." + k) for k in keys]
            row = [ctx.need(f"{k}平均用时") if Ctx.empty(v) else f"{v} s" for k, v in zip(keys, vals)]
            src = ctx.get("各阶段用时_s.来源")
            self.table([keys, row], cap + ("" if Ctx.empty(src) else f"（{src}）"))

    def bench_table(self, cap):
        ctx = self.ctx
        b = ctx.bench
        hdr = ["深度 \\ 方位", "左（约 −10°）", "中（0°）", "右（约 +10°）", "合计"]
        rows = [hdr]
        names = {1: "①", 2: "②", 3: "③", 4: "④", 5: "⑤", 6: "⑥", 7: "⑦", 8: "⑧", 9: "⑨"}
        hl = set()
        for ri, rn in enumerate(["近（约 15 cm）", "中（约 18 cm）", "远（约 21 cm）"]):
            row = [rn]
            rn_ok = rn_n = 0
            for ci in range(3):
                p = ri * 3 + ci + 1
                c = (b or {}).get("cells", {}).get(p) if b else None
                if c:
                    row.append(f"{names[p]} {c['抓到']}/{c['次数']}（{pct(c['抓到'], c['次数'])}）")
                    rn_ok += c["抓到"]
                    rn_n += c["次数"]
                else:
                    row.append(f"{names[p]} 【待真机】")
                    hl.add((ri + 1, ci + 1))
            row.append(f"{rn_ok}/{rn_n}（{pct(rn_ok, rn_n)}）" if rn_n else "【待真机】")
            rows.append(row)
        if b:
            rows.append(["合计", "", "", "", f"{b['ok']}/{b['n']}（{pct(b['ok'], b['n'])}）"])
        else:
            ctx.need("bench 测试矩阵 9 个位置的结果（data.json bench）")
            rows.append(["合计", "", "", "", "【待真机】"])
        self.table(rows, cap + (f"，数据来源：{b['src']}" if b else ""), highlight_cells=hl)
        if b:
            t = f"{b['t']} s" if b.get("t") is not None else "【待真机：平均用时】"
            self.para(f"表中每格为“真实抓到 / 次数（成功率）”。误报（程序判定抓到、实际没抓到）共 {b['fp']} 次；"
                      f"平均每次用时 {t}。")
        else:
            self.para("表中每格为“真实抓到 / 次数（成功率）”，同时统计误报（程序判定抓到、实际没抓到）次数和平均用时。"
                      "【待真机：bench 测试完成后填 data.json 的 bench 一节】")

    # ---------- 封面、目录、收尾
    def cover(self):
        title = self.ctx.get("作品.名称")
        for p in self.doc.paragraphs[:8]:
            if p.text.startswith("作品名称"):
                for r in p.runs[1:]:
                    r._r.getparent().remove(r._r)
                p.runs[0].text = f"作品名称：{title}"

    def toc(self):
        sdt = self.body.find(qn("w:sdt"))
        content = sdt.find(qn("w:sdtContent"))
        paras = content.findall(qn("w:p"))
        for p in paras[2:]:
            content.remove(p)

        def mk(style, runs):
            p = _el("w:p")
            pPr = _el("w:pPr")
            pPr.append(_el("w:pStyle", **{"w:val": style}))
            p.append(pPr)
            for r in runs:
                p.append(r)
            content.append(p)

        def run_text(t, hl=False):
            r = _el("w:r")
            if hl:
                rPr = _el("w:rPr")
                rPr.append(_el("w:highlight", **{"w:val": "yellow"}))
                r.append(rPr)
            tt = _el("w:t")
            tt.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
            tt.text = t
            r.append(tt)
            return r

        def run_fld(kind):
            r = _el("w:r")
            r.append(_el("w:fldChar", **{"w:fldCharType": kind}))
            return r

        instr = _el("w:r")
        it = _el("w:instrText")
        it.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
        it.text = ' TOC \\o "1-3" \\h \\z \\u '
        instr.append(it)
        mk("TOC1", [run_fld("begin"), instr, run_fld("separate"),
                    run_text("【目录待更新：在 Word 里右键目录 → 更新域 → 更新整个目录（打开时如果提示更新域，选“是”）】",
                             True)])
        for lvl, num, t in self.headings:
            if lvl <= 2:
                mk("TOC1" if lvl == 1 else "TOC2", [run_text(f"{num} {strip_md(t)}")])
        mk("TOC2", [run_fld("end")])
        # 打开文档时让 Word 提示更新域（目录、图表编号）
        settings = self.doc.settings.element
        if settings.find(qn("w:updateFields")) is None:
            settings.append(_el("w:updateFields", **{"w:val": "true"}))

    def drop_unused_images(self):
        part = self.doc.part
        xml = etree.tostring(self.doc.element).decode("utf-8")
        used = set(re.findall(r'r:embed="(rId\d+)"', xml))
        for rId, rel in list(part.rels.items()):
            if rel.reltype.endswith("/image") and rId not in used:
                del part.rels[rId]

    # ---------- 主流程
    def build(self, blocks, path):
        self.cover()
        for b in blocks:
            kind = b[0]
            if kind == "h":
                self.heading(b[1], b[2])
            elif kind == "p":
                self.para(b[1])
            elif kind == "li":
                self.para("• " + b[1])
            elif kind == "eq":
                p = self._new_p("body", "center")
                p._p.pPr.append(_el("w:ind", **{"w:firstLine": "0", "w:firstLineChars": "0"}))
                self._runs(p, self.text(b[1]))
            elif kind == "table":
                self.table(b[3], b[2] if b[2] else None)
            elif kind == "img":
                self.image_block(b[1], b[2], b[3])
            elif kind == "special":
                self.special(b[1])
        self.toc()
        self.drop_unused_images()
        self.doc.core_properties.title = f"设计报告 - {self.ctx.get('作品.名称')}"
        self.doc.save(path)


# =====================================================================================  答辩 PPT
NAVY = RGBColor(0x0F, 0x1A, 0x26)
ORANGE = RGBColor(0xF0, 0x7D, 0x1A)
DGRAY = RGBColor(0x4A, 0x58, 0x68)
GRAY = RGBColor(0x8B, 0x97, 0xA5)
LIGHT = RGBColor(0xF5, 0xF7, 0xFA)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
SOFT = RGBColor(0xC9, 0xD1, 0xDA)
WM = RGBColor(0xE9, 0xED, 0xF2)
CARD_DARK = RGBColor(0x16, 0x20, 0x2B)
YELLOW_HL = "FFFF00"
CN_FONT = "Microsoft YaHei"
NUM_FONT = "Bahnschrift"


def cm(x):
    return PCm(x)


class PPTBuilder:
    def __init__(self, ctx, out_dir):
        self.ctx = ctx
        self.out_dir = out_dir
        self.prs = Presentation(T_PPT)
        self.layouts = {l.name: l for l in self.prs.slide_layouts}
        self.bg_cover = None
        self.bg_divider = None
        self._grab_backgrounds()
        self._delete_all_slides()
        self.total = 0
        self.page = 0
        self.notes_missing = []

    def _grab_backgrounds(self):
        slides = list(self.prs.slides)
        for idx, attr in ((0, "bg_cover"), (1, "bg_divider")):
            for sh in slides[idx].shapes:
                if sh.shape_type == 13:  # PICTURE
                    p = os.path.join(self.out_dir, "_cache", f"ppt_{attr}.{sh.image.ext}")
                    os.makedirs(os.path.dirname(p), exist_ok=True)
                    with open(p, "wb") as f:
                        f.write(sh.image.blob)
                    setattr(self, attr, p)
                    break

    def _delete_all_slides(self):
        lst = self.prs.slides._sldIdLst
        for sld in list(lst):
            self.prs.part.drop_rel(sld.rId)
            lst.remove(sld)

    # ---------- 文本
    def _run(self, p, text, size, color=None, bold=False, font=CN_FONT, hl=False, spc=None):
        r = p.add_run()
        r.text = text
        rPr = r._r.get_or_add_rPr()
        rPr.set("lang", "zh-CN")
        rPr.set("sz", str(int(size * 100)))
        if bold:
            rPr.set("b", "1")
        if spc:
            rPr.set("spc", str(spc))
        # 子元素顺序：solidFill → highlight → latin → ea → cs
        if color is not None:
            sf = etree.SubElement(rPr, pqn("a:solidFill"))
            c = etree.SubElement(sf, pqn("a:srgbClr"))
            c.set("val", str(color))
        if hl:
            h = etree.SubElement(rPr, pqn("a:highlight"))
            c = etree.SubElement(h, pqn("a:srgbClr"))
            c.set("val", YELLOW_HL)
        for tag in ("a:latin", "a:ea", "a:cs"):
            e = etree.SubElement(rPr, pqn(tag))
            e.set("typeface", font)
        return r

    def text(self, slide, x, y, w, h, content, size=12, color=NAVY, bold=False, font=CN_FONT, align=PP_ALIGN.LEFT,
             anchor=MSO_ANCHOR.TOP, spc=None, line_spacing=None):
        """content: 字符串（\\n 分段）或段落列表；【…】自动黄色高亮，**粗体**"""
        tb = slide.shapes.add_textbox(cm(x), cm(y), cm(w), cm(h))
        tf = tb.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
        tf.vertical_anchor = anchor
        paras = content if isinstance(content, list) else str(content).split("\n")
        for i, ptxt in enumerate(paras):
            p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
            p.alignment = align
            if line_spacing:
                p.line_spacing = line_spacing
            ptxt = self.ctx.substitute(ptxt)
            for seg, b, hl in segments(ptxt):
                self._run(p, seg, size, NAVY if hl else color, bold or b, font, hl, spc)
        return tb

    # ---------- 形状
    def rect(self, slide, x, y, w, h, fill=None, line=None, dash=False, round_=False, shadow=False, lw=1.0):
        shp = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE if round_ else MSO_SHAPE.RECTANGLE,
                                     cm(x), cm(y), cm(w), cm(h))
        if round_:
            shp.adjustments[0] = 0.07
        if fill is None:
            shp.fill.background()
        else:
            shp.fill.solid()
            shp.fill.fore_color.rgb = fill
        if line is None:
            shp.line.fill.background()
        else:
            shp.line.color.rgb = line
            shp.line.width = PPt(lw)
            if dash:
                ln = shp.line._get_or_add_ln()
                d = etree.SubElement(ln, pqn("a:prstDash"))
                d.set("val", "dash")
        if shadow:
            spPr = shp._element.spPr
            eff = etree.SubElement(spPr, pqn("a:effectLst"))
            sh = etree.SubElement(eff, pqn("a:outerShdw"))
            for k, v in dict(blurRad="101600", dist="25400", dir="5400000", algn="bl", rotWithShape="0").items():
                sh.set(k, v)
            c = etree.SubElement(sh, pqn("a:srgbClr"))
            c.set("val", "000000")
            a = etree.SubElement(c, pqn("a:alpha"))
            a.set("val", "8000")
        shp.text_frame.text = ""
        shp.shadow.inherit = False if not shadow else shp.shadow.inherit
        return shp

    def line(self, slide, x1, y1, x2, y2, color=ORANGE, width=1.75):
        c = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, cm(x1), cm(y1), cm(x2), cm(y2))
        c.line.color.rgb = color
        c.line.width = PPt(width)
        return c

    def card(self, slide, x, y, w, h, num, title, desc, dark=False, title_size=13, desc_size=10.5):
        self.rect(slide, x, y, w, h, fill=CARD_DARK if dark else WHITE, round_=True, shadow=not dark,
                  line=None)
        tx = x + 0.45
        if num:
            self.text(slide, x + 0.4, y + 0.38, 1.2, 0.9, num, 18, ORANGE, True, NUM_FONT)
            tx = x + 1.6
        self.text(slide, tx, y + 0.35, w - (tx - x) - 0.3, 0.7, title, title_size, WHITE if dark else NAVY, True)
        self.text(slide, tx, y + 0.35 + title_size * 0.055, w - (tx - x) - 0.3, h - 0.9, desc, desc_size,
                  SOFT if dark else DGRAY)

    # ---------- 页
    def slide(self, kind, kicker=None, title=None, watermark=None, notes=""):
        layout = {"light": "LIGHT", "dark": "DARK", "blank": "BLANK_DARK"}[kind]
        s = self.prs.slides.add_slide(self.layouts[layout])
        self.page += 1
        self.ctx.where = f"PPT 第 {self.page} 页"
        if watermark and kind == "light":
            self.text(s, 7.62, 11.56, 17.53, 2.67, watermark, 72, WM, True, NUM_FONT, PP_ALIGN.RIGHT,
                      MSO_ANCHOR.BOTTOM)
        if title is not None:
            ph = None
            for p in s.placeholders:
                ph = p
            tf = ph.text_frame
            tf.text = ""
            p = tf.paragraphs[0]
            for seg, b, hl in segments(self.ctx.substitute(title)):
                self._run(p, seg, 24, NAVY if (kind == "light" or hl) else WHITE, True, CN_FONT, hl)
        if kicker:
            self.text(s, 1.397, 0.813, 15.24, 0.56, kicker, 10, ORANGE, True, NUM_FONT, spc=200)
        if kind != "blank":
            self._page_no = self.text(s, 21.08, 13.26, 3.05, 0.56, "", 9, GRAY, False, NUM_FONT, PP_ALIGN.RIGHT)
            self._page_no.name = "PageNo"
        s.notes_slide.notes_text_frame.text = self.ctx.substitute(notes)
        return s

    def background(self, s, override_key="照片.封面背景"):
        f = self.ctx.get(override_key)
        p = self.ctx.media(f) if not Ctx.empty(f) else None
        if p:
            p = prepared_image(p, self.out_dir, 2000)
            pic = s.shapes.add_picture(p, 0, 0, self.prs.slide_width, self.prs.slide_height)
            ov = self.rect(s, 0, 0, 25.4, 14.29, fill=NAVY)
            sf = ov.fill._xPr.find(pqn("a:solidFill"))
            clr = sf[0]
            a = etree.SubElement(clr, pqn("a:alpha"))
            a.set("val", "55000")
            return pic
        return s.shapes.add_picture(self.bg_cover, 0, 0, self.prs.slide_width, self.prs.slide_height)

    def picture_fit(self, s, path, x, y, w, h):
        pw, ph = image_size(path)
        r = min(w / pw, h / ph)
        ww, hh = pw * r, ph * r
        return s.shapes.add_picture(prepared_image(path, self.out_dir), cm(x + (w - ww) / 2), cm(y + (h - hh) / 2),
                                    cm(ww), cm(hh))

    def photo_slot(self, s, key, x, y, w, h, label, fallback=None, dark=False):
        f = self.ctx.get("照片." + key)
        p = self.ctx.media(f)
        if p:
            self.rect(s, x, y, w, h, fill=CARD_DARK if dark else WHITE, round_=True, shadow=not dark)
            self.picture_fit(s, p, x + 0.2, y + 0.2, w - 0.4, h - 0.4)
            return True
        if fallback:
            self.rect(s, x, y, w, h, fill=WHITE, round_=True, shadow=True)
            self.picture_fit(s, fallback[0], x + 0.25, y + 0.25, w - 0.5, h - 0.5)
            self.text(s, x + 0.2, y + h - 0.75, w - 0.4, 0.6, f"【待图：{fallback[1]}】", 9, NAVY, align=PP_ALIGN.CENTER)
            self.ctx.missing.append((self.ctx.where, f"待图：{label}（data.json 照片.{key}，现在用的是旧图）"))
            return False
        self.rect(s, x, y, w, h, fill=CARD_DARK if dark else WHITE, line=ORANGE, dash=True)
        self.text(s, x, y + h / 2 - 0.5, w, 1.0, [self.ctx.need(label, "待图"), f"data.json 照片.{key}"], 11,
                  ORANGE, True, align=PP_ALIGN.CENTER)
        return False

    def video_slot(self, s, vid, x, y, w, h, show_caption=True):
        v = self.ctx.get("视频." + vid) or {}
        title = v.get("标题", vid)
        fname = v.get("文件")
        path = self.ctx.media(fname) if not Ctx.empty(fname) else None
        if path:
            poster = make_poster(path, title, self.out_dir, vid)
            s.shapes.add_movie(path, cm(x), cm(y), cm(w), cm(h), poster_frame_image=poster,
                               mime_type="video/mp4" if path.lower().endswith(".mp4") else "video/unknown")
        else:
            self.ctx.missing.append((self.ctx.where, f"待视频：{vid} {title}（data.json 视频.{vid}.文件）"))
            self.rect(s, x, y, w, h, fill=CARD_DARK, line=ORANGE, dash=True)
            cy = y + h / 2
            self.text(s, x, cy - 1.2, w, 0.8, "待补视频", 16, ORANGE, True, align=PP_ALIGN.CENTER)
            self.text(s, x, cy - 0.35, w, 0.6, f"拍摄清单 {vid}  {title}", 12, WHITE, True, align=PP_ALIGN.CENTER)
            self.text(s, x + 0.8, cy + 0.4, w - 1.6, 1.6, v.get("说明", ""), 10, SOFT, align=PP_ALIGN.CENTER)
        # 四角标记
        L = 0.56
        for cx, cy_, dx, dy in ((x - 0.2, y - 0.2, 1, 1), (x + w + 0.2, y - 0.2, -1, 1),
                                (x - 0.2, y + h + 0.2, 1, -1), (x + w + 0.2, y + h + 0.2, -1, -1)):
            self.line(s, cx, cy_, cx + dx * L, cy_)
            self.line(s, cx, cy_, cx, cy_ + dy * L)
        if show_caption:
            self.text(s, x, y + h + 0.3, w, 0.6, f"▶  {vid} {title}", 10, WHITE, True)

    def steps_list(self, s, x, y, items, dark=True, gap=1.95, w=6.4):
        for i, (t, d) in enumerate(items):
            yy = y + i * gap
            self.text(s, x, yy, 1.2, 0.9, f"{i + 1:02d}", 17, ORANGE, True, NUM_FONT)
            self.text(s, x + 1.35, yy + 0.02, w, 0.7, t, 13, WHITE if dark else NAVY, True)
            self.text(s, x + 1.35, yy + 0.78, w, 1.0, d, 10, SOFT if dark else DGRAY)

    def ptable(self, s, rows, x, y, w, col_w=None, font=10, row_h=0.72, hl_cells=()):
        rows = [[self.ctx.substitute(c) for c in r] for r in rows]
        nr, nc = len(rows), len(rows[0])
        gt = s.shapes.add_table(nr, nc, cm(x), cm(y), cm(w), cm(row_h * nr)).table
        tblPr = gt._tbl.tblPr
        tblPr.set("firstRow", "1")
        tblPr.set("bandRow", "0")
        sid = tblPr.find(pqn("a:tableStyleId"))
        if sid is not None:
            sid.text = "{5940675A-B579-460E-94D1-54222C63F5DA}"   # 无样式、细网格
        if col_w:
            for i, cw in enumerate(col_w):
                gt.columns[i].width = cm(cw)
        for ri, r in enumerate(rows):
            gt.rows[ri].height = cm(row_h)
            for ci, txt in enumerate(r):
                cell = gt.cell(ri, ci)
                cell.margin_left = cell.margin_right = cm(0.15)
                cell.margin_top = cell.margin_bottom = cm(0.05)
                cell.vertical_anchor = MSO_ANCHOR.MIDDLE
                cell.fill.solid()
                cell.fill.fore_color.rgb = NAVY if ri == 0 else (WHITE if ri % 2 else LIGHT)
                tf = cell.text_frame
                tf.word_wrap = True
                p = tf.paragraphs[0]
                p.alignment = PP_ALIGN.CENTER if ci > 0 or ri == 0 else PP_ALIGN.LEFT
                for seg, b, hl in segments(txt):
                    hl = hl or (ri, ci) in hl_cells
                    self._run(p, seg, font, WHITE if ri == 0 and not hl else NAVY, b or ri == 0 or ci == 0, CN_FONT,
                              hl)
        return gt

    def finish(self, path):
        n = len(self.prs.slides)
        for i, s in enumerate(self.prs.slides, 1):
            for sh in s.shapes:
                if sh.name == "PageNo":
                    p = sh.text_frame.paragraphs[0]
                    self._run(p, f"{i:02d} / {n:02d}", 9, GRAY, False, NUM_FONT)
        self.prs.core_properties.title = f"答辩 - {self.ctx.get('作品.名称')}"
        self.prs.save(path)

    # ---------- 内容
    def build(self, path):
        c = self.ctx
        names = c.get("队员.名单") or []
        team = "　".join(names) + ("" if c.get("队员.已核对") else "【请核对】")
        if not c.get("队员.已核对"):
            c.missing.append(("PPT 封面", "请核对：队员名单（从旧版照搬）"))
        ok, n, t_avg = headline(c)
        sims = c.get("仿真.行") or []
        sim_txt = "；".join(f"{r['场景'].split('（')[0]} {r['抓到']}/{r['次数']}" for r in sims) or "【待补：仿真】"
        selftest = c.val("自测.selftest项数", "自测项数")
        learn_sw = c.switch("比赛版启用失败试探与学习")
        chassis_sw = c.switch("底盘参与抓取已联调")

        # 1 封面
        s = self.slide("blank", notes=(
            "各位评委老师好，我们的作品是“移动机械臂感知与作业一体化平台”。它由阿克曼底盘、伺服丝杠升降轴和六自由度机械臂组成，"
            "K230 相机装在机械臂的手腕上。树莓派作为总控，同时指挥升降轴、机械臂和底盘，看到物块后自己对准、自己夹取、"
            "自己确认有没有抓到，没抓到会自动重试。接下来我从问题、方案、核心算法、测试结果四个方面介绍。"))
        self.background(s)
        self.text(s, 1.52, 1.40, 12.7, 0.6, "ROBOTAC 2026  ·  GANSU", 10, ORANGE, True, NUM_FONT, spc=200)
        self.text(s, 1.52, 2.16, 14.0, 1.3, [c.get("作品.赛事"), c.get("作品.赛项")], 10, SOFT)
        self.text(s, 1.52, 4.19, 14.2, 3.8, "移动机械臂感知与\n作业一体化平台", 34, WHITE, True, line_spacing=1.05)
        self.text(s, 1.52, 8.26, 14.0, 1.0, c.get("作品.口号"), 15, ORANGE, True)
        self.rect(s, 1.52, 10.67, 11.0, 1.6, fill=None, line=SOFT, lw=0.75)
        self.text(s, 2.03, 10.67, 10.4, 1.6, "参赛队员　" + team, 12, WHITE, anchor=MSO_ANCHOR.MIDDLE)

        # 2 痛点
        s = self.slide("light", "01  PROBLEM", "从“能动”到“能可靠地抓”：低成本平台的四个痛点", "PROBLEM", notes=(
            "上一版我们实现了识别、接近和夹取，但调试中发现四个问题。第一，底盘、升降轴、机械臂都能改变夹爪位置，"
            "同时修正会互相干扰。第二，舵机没有位置反馈、有死区和回差，手工调的增益换一次装配就不准。第三，相机装在手腕上，"
            "爪子自己会出现在画面里，会被当成物块。第四，没有力传感器，爪子合上不代表抓到了。"
            "所以这一版的目标是：全自动、不用手眼标定、每一步都能验证。"))
        items = [("01", "多执行机构协同", "底盘、升降轴、机械臂都能改变夹爪位置，同时修正互相干扰"),
                 ("02", "低成本执行器不准", "舵机无反馈、有死区回差；手调增益换一次装配就不准"),
                 ("03", "眼在手上的自身干扰", "腕部相机能看到自己的爪子，可能被当成物块"),
                 ("04", "“抓没抓到”不可靠", "没有力传感器，爪子合上不代表夹住了")]
        for i, (num, t, d) in enumerate(items):
            self.card(s, 1.40, 3.30 + i * 2.05, 12.9, 1.8, num, t, d)
        self.rect(s, 14.86, 3.30, 9.14, 7.95, fill=NAVY, round_=True)
        self.text(s, 15.62, 3.95, 7.6, 0.6, "OUR GOAL", 9, ORANGE, True, NUM_FONT, spc=200)
        self.text(s, 15.62, 4.85, 7.6, 3.9, "全自动抓取\n免手眼标定\n每一步可验证", 22, WHITE, True, line_spacing=1.15)
        self.text(s, 15.62, 9.10, 7.9, 1.8, "在低成本硬件上，可靠地把台面上的物块拿起来", 11, SOFT)

        # 3 方案总览
        s = self.slide("light", "01  OVERVIEW", "方案总览：树莓派统一调度三套执行机构", notes=(
            "这是系统总体架构。中间是树莓派总控，通过四路串口分别连接：手腕上的 K230 发来物块的位置和深度；"
            "升降轴用文本指令控制高度；机械臂控制板接收舵机帧；底盘以每秒 50 次的频率接收速度指令。"
            "各下位机只负责自己机构的实时控制，树莓派只决定“去哪里、多快”。语音交互和环境感知是独立运行的扩展功能，"
            "不在抓取闭环里。"))
        self.rect(s, 1.40, 3.20, 22.6, 9.75, fill=WHITE, round_=True, shadow=True)
        self.picture_fit(s, os.path.join(HERE, "figures", "系统总体架构.png"), 1.6, 3.3, 22.2, 9.55)
        self.text(s, 1.40, 13.15, 19.0, 0.6, "通信协议沿用实测确认过的帧格式：K230 帧带 CRC8，底盘帧带 BCC 异或校验", 10,
                  DGRAY)

        # 4 硬件
        s = self.slide("light", "01  HARDWARE", "三段式结构：底盘 + 伺服丝杠升降轴 + 机械臂", "DESIGN", notes=(
            "硬件分三段。底盘是 WHEELTEC 的阿克曼底盘，负责接近目标；中间是伺服电机驱动的丝杠升降轴，行程 0 到 50 厘米，"
            "软件限位 44 厘米；上面是六自由度舵机机械臂，K230 相机固定在手腕那一节，跟着机械臂一起动，这就是“眼在手上”。"
            "升降轴固件我们重写了，最高速度从上一版的约每秒 6.8 毫米提高到每秒 80 毫米，运动中还能随时改目标。"))
        self.photo_slot(s, "整机", 1.40, 3.20, 10.9, 9.2, "整机实物照片（K230 在腕部、树莓派、升降轴）",
                        fallback=(os.path.join(T_IMG, "大车三维建模分解图.png"), "换成新整机照片；这是旧版三维模型"))
        hw = [("01", "K230 视觉（腕部）", "检测 + 单目测距，随机械臂运动"),
              ("02", "6 自由度机械臂", "Lobot PWM 舵机，底座负责方位对准"),
              ("03", "伺服丝杠升降轴", "0–50 cm，软件限位 44 cm"),
              ("04", "阿克曼底盘", "WHEELTEC STM32F407，编码器 + IMU")]
        for i, (num, t, d) in enumerate(hw):
            yy = 3.25 + i * 1.62
            self.text(s, 13.2, yy, 1.3, 1.0, num, 18, ORANGE, True, NUM_FONT)
            self.text(s, 14.6, yy + 0.02, 9.2, 0.7, t, 14, NAVY, True)
            self.text(s, 14.6, yy + 0.78, 9.2, 0.7, d, 11, DGRAY)
        self.rect(s, 13.2, 9.90, 10.8, 2.5, fill=NAVY, round_=True)
        self.text(s, 13.8, 10.05, 4.6, 2.2, "80", 40, ORANGE, True, NUM_FONT, anchor=MSO_ANCHOR.MIDDLE)
        self.text(s, 17.0, 10.30, 6.8, 2.0, "mm/s 升降最高速度（固件默认值）\n上一版约 6.8 mm/s；运动中可随时改目标", 11, WHITE,
                  anchor=MSO_ANCHOR.MIDDLE)

        # 5 视觉
        s = self.slide("light", "02  VISION", "K230 单目视觉：检测、跟踪、测距、心跳", "VISION", notes=(
            "视觉部分不用深度相机。物块边长已知，用检测框的大小就能算出距离，公式是 D 等于 L 乘 f 除以根号 w 乘 h。"
            "这一版在 K230 端做了四个改进：没看到物块时也发心跳帧，树莓派就能区分“没看到”和“串口断了”；"
            "画面随手腕旋转了 90 度，坐标在 K230 端统一；爪子在画面里的区域直接屏蔽；物块碰到画面边缘时框被截断，就不发位置。"))
        self.rect(s, 1.40, 3.30, 11.2, 5.1, fill=NAVY, round_=True)
        self.text(s, 2.16, 3.80, 9.7, 0.6, "已知尺寸测距", 10, ORANGE, True)
        self.text(s, 2.16, 4.70, 9.9, 1.6, "D = L·f / √(w·h)", 30, WHITE, True, "Times New Roman")
        self.text(s, 2.16, 6.70, 9.9, 1.3, "L 物块边长 24 mm　f 焦距 944 px　w·h 检测框面积\nX = (u − cₓ)·D / f，Y = (v − c_y)·D / f",
                  10, SOFT)
        self.text(s, 1.40, 9.00, 11.2, 1.3, "**跟踪**：多目标时按 IOU × 0.7 + 置信度 × 0.3 锁定同一物块", 11, DGRAY)
        self.text(s, 1.40, 10.30, 11.2, 1.6, "**输出**：约 7 帧/s（自检实测 2 s 收到 14 帧），10 字节帧 + CRC8 校验",
                  11, DGRAY)
        vis = [("心跳帧", "没看到物块也发 D = 0，树莓派区分“没目标”和“断线”"),
               ("画面旋转", "相机随腕部转了 90°，K230 端统一成 X 右正、Y 下正"),
               ("夹爪屏蔽区", "中心落在区域内的框进跟踪器前就丢掉，OSD 画红色虚线"),
               ("边缘不发", "物块碰到画面边缘时框被截断，测距不可信，不发位置")]
        self.text(s, 13.59, 3.30, 10.2, 0.8, "本版本在 K230 端的改进", 14, NAVY, True)
        for i, (t, d) in enumerate(vis):
            yy = 4.50 + i * 1.95
            self.text(s, 13.59, yy, 1.3, 1.0, f"{i + 1:02d}", 18, ORANGE, True, NUM_FONT)
            self.text(s, 14.99, yy + 0.02, 8.9, 0.7, t, 13, NAVY, True)
            self.text(s, 14.99, yy + 0.76, 8.9, 1.0, d, 10.5, DGRAY)

        # 6 核心 1
        s = self.slide("light", "02  CORE · 1 / 2", "核心①：三个误差方向，各交给最合适的执行机构", "CONTROL", notes=(
            "这是作品的核心。机械臂保持观察姿态，K230 测出物块的方位角、竖直偏移和深度三个误差。"
            "竖直误差交给升降轴，整条手臂连相机一起上下；方位误差交给机械臂底座舵机，相机绕底座转；深度误差交给底盘前后走，"
            "没有底盘时就按深度在两个标定的伸出姿态之间插值。每一步的动作量都用同一个公式算：u 等于负 α 乘误差，"
            "再除以灵敏度 J 和几何系数 s。α 取 0.85，每步只走预测量的 85%，宁可多走一步也不冲过头。"))
        cols = [("竖直 y", "升降轴", "整条机械臂连同相机上下平移"),
                ("方位 φ", "机械臂底座舵机", "相机绕底座转轴转动，几何系数 s = 1 + R/d"),
                ("深度 d", "底盘前进 / 后退", "无底盘时按深度在近、远两个伸出姿态之间插值")]
        for i, (e, a, d) in enumerate(cols):
            x = 1.40 + i * 7.67
            self.rect(s, x, 3.30, 7.2, 5.1, fill=WHITE, round_=True, shadow=True)
            self.text(s, x + 0.63, 3.80, 6.1, 0.6, e, 11, ORANGE, True)
            self.text(s, x + 0.63, 4.60, 6.1, 1.1, a, 19, NAVY, True)
            self.text(s, x + 0.63, 6.20, 6.1, 1.9, d, 11, DGRAY)
            if i < 2:
                self.text(s, x + 7.2, 5.3, 0.5, 1.3, "›", 26, GRAY, True, align=PP_ALIGN.CENTER)
        self.rect(s, 1.40, 9.00, 22.6, 3.6, fill=NAVY, round_=True)
        self.text(s, 2.16, 9.40, 9.0, 0.6, "ADAPTIVE CONTROL LAW", 9, ORANGE, True, NUM_FONT, spc=200)
        self.text(s, 2.16, 10.10, 9.5, 1.6, "u = −α · e / (J · s)", 28, WHITE, True, "Times New Roman")
        self.text(s, 12.2, 9.50, 11.3, 2.9, "e 误差　J 灵敏度（单位动作引起的读数变化，在线学习）\ns 已知几何系数　α = 0.85，每步只走预测量的 85%\n"
                  "对准容差：方位 1.5°、竖直 4 mm、深度 10 mm", 11, SOFT, line_spacing=1.2)

        # 7 核心 2
        s = self.slide("light", "02  CORE · 2 / 2", "核心②：边对准边学习灵敏度 J，不需要手眼标定", notes=(
            "每走一步，我们都再测一次，用实际读数变化除以动作量，得到这一步实测的灵敏度，再按 0.3 的学习率更新 J。"
            "学习有三道保护：动作太小或读数变化小于噪声的两倍不学；方向和初值相反，多半是噪声或打滑，不学；"
            "学到的值限制在初值的四分之一到四倍。学到的 J 存在文件里，下次开机直接用，所以换车、换装配后不用重新标定。"
            "底盘转弯后我们还用几何前馈，直接算出底座该转多少，相机始终对着物块。"))
        self.rect(s, 1.40, 3.20, 14.6, 8.4, fill=WHITE, round_=True, shadow=True)
        self.picture_fit(s, os.path.join(HERE, "figures", "自适应视觉伺服流程.png"), 1.55, 3.35, 14.3, 8.1)
        pts = [("在线学习", "Ĵ = Δe / (u·s)，J ← 0.7·J + 0.3·Ĵ\n学到的值存文件，下次开机直接用"),
               ("三道保护", "动作太小 / 变化小于噪声不学；方向反了不学；限制在初值 1/4~4 倍"),
               ("几何前馈", "阿克曼底盘走完一段弧，按里程和转角直接算底座角度，相机不丢目标"),
               ("稳健测量", "动作后等 0.25 s，取 5 帧中位数（远时 3 帧）")]
        for i, (t, d) in enumerate(pts):
            yy = 3.20 + i * 2.15
            self.text(s, 16.6, yy, 7.4, 0.7, t, 13, ORANGE, True)
            self.text(s, 16.6, yy + 0.75, 7.4, 1.4, d, 10, DGRAY)
        self.text(s, 1.40, 12.0, 22.0, 0.8, "不做手眼标定，也不需要事先知道相机安装位置和执行机构的精确参数", 11, NAVY, True)

        # 8 安全
        s = self.slide("light", "02  SAFETY", "安全与容错：每一条都来自真机调试或上车前审查", "SAFETY", notes=(
            "安全方面分两类。感知异常：连心跳都收不到就判断为断线停下；动了两步读数几乎不变，说明看到的是自己的爪子，立即停；"
            "有底盘时看到目标先复核一次再开车，深度读数突变先确认。运动安全：底盘固件没有串口超时，我们每秒发 50 次，"
            "0.5 秒没更新自动归零；程序被 kill 或 SSH 断开也会先停车；底盘回传断了马上停；升降轴意外复位后不自动接着跑；"
            "搜索后退累计不超过 50 厘米。右边这几项是上车前代码审查和底盘预演时在仿真里复现、修复的，都有回归测试。"))
        left = [("断线 ≠ 没看到", "连 K230 心跳都收不到 → 报错停止"),
                ("看到自己的爪子", "动了两步读数几乎不变 → 判为自身干扰，立即停"),
                ("误识别复核", "有底盘时看到目标先复核再开车"),
                ("深度跳变确认", "底盘走完一步读数突变 → 先确认，不照着误读开")]
        right = [("底盘兜底", "50 Hz 持续发送，0.5 s 不更新自动归零"),
                 ("进程终止也停车", "SIGTERM / SIGHUP 同样走停车流程"),
                 ("回传中断停车", "里程不更新 0.5 s 内停车报错"),
                 ("升降复位保护", "下位机意外复位后不自动接着跑"),
                 ("后退上限", "搜不到时累计后退 ≤ 500 mm")]
        self.text(s, 1.40, 3.10, 10.9, 0.7, "感知异常", 13, NAVY, True)
        for i, (t, d) in enumerate(left):
            self.card(s, 1.40, 3.90 + i * 2.05, 10.9, 1.8, None, t, d, title_size=12, desc_size=10)
        self.text(s, 12.9, 3.10, 11.1, 0.7, "运动安全（后四项来自代码审查与底盘预演）", 13, NAVY, True)
        for i, (t, d) in enumerate(right):
            self.card(s, 12.9, 3.90 + i * 1.66, 11.1, 1.45, None, t, d, dark=i >= 1, title_size=12, desc_size=10)
        self.text(s, 1.40, 12.30, 11.0, 0.9, "审查发现的问题均在仿真 / 假串口中复现并修复，有回归测试；未在真机上触发验证", 9.5,
                  DGRAY)

        # 9 抓取确认与学习
        s = self.slide("light", "02  GRASP", "抓没抓到？退回观察姿态，再看一眼原位置", notes=(
            "抓取分三步：张爪伸出、合爪、确认。确认这一步我们走过弯路：一开始用“抬起来看物块跟不跟着动”，"
            "但腕部相机能看到爪子，爪子本身就跟着动，会把没抓到判成抓到。现在改成爪子保持闭合、退回观察姿态，"
            "看原来的位置还有没有物块，有就说明没抓到，松开重试。第一天联调还发现落点系统性偏右约 4 毫米，"
            "我们先用固定微调补偿。"
            + ("另外实现了失败时左右试探、从抓取结果学习落点偏差的功能，仿真里有小幅提升。" if learn_sw is not False else "")))
        stepsx = [("伸出", "张爪 → 预备姿态 → 按深度在近、远两个标定姿态之间插值 → 合爪"),
                  ("观察确认", "爪子保持闭合，退回观察姿态、升降轴降回原高度"),
                  ("判断与重试", "原位置还看得到物块 = 没抓到 → 松开、后退、自动重试")]
        for i, (t, d) in enumerate(stepsx):
            x = 1.40 + i * 7.67
            self.card(s, x, 3.30, 7.2, 3.6, f"{i + 1:02d}", t, d, title_size=14)
            if i < 2:
                self.text(s, x + 7.2, 4.4, 0.5, 1.3, "›", 26, GRAY, True, align=PP_ALIGN.CENTER)
        self.rect(s, 1.40, 7.50, 10.9, 4.9, fill=NAVY, round_=True)
        self.text(s, 2.0, 7.90, 9.8, 0.6, "为什么不用“抬起来看”", 13, ORANGE, True)
        self.text(s, 2.0, 8.80, 9.8, 3.4, "腕部相机能看到夹爪，夹爪本身就跟着相机动，“物块跟着动”会被夹爪骗过，"
                  "没抓到也报抓到。改成观察确认后，判断只看原位置。", 11, SOFT)
        self.rect(s, 12.9, 7.50, 11.1, 4.9, fill=WHITE, round_=True, shadow=True)
        self.text(s, 13.5, 7.90, 10.0, 0.6, "落点偏差补偿", 13, NAVY, True)
        trim_txt = ["第一天联调：落点系统性偏右约 4 mm → 标定固定微调（底座约 −1.3°）"]
        if learn_sw is not False:
            trim_txt.append("失败试探 + 从结果学习：没抓到时重试落点左右交替偏移几毫米，哪边成功就把该深度段的微调往那边挪"
                            + ("" if learn_sw else "【请核对：比赛版是否启用】"))
            trim_txt.append("仿真（10 台车 × 100 次，无底盘）：固定微调 86.1% → 加试探与学习 87.9%（仿真，未在真机验证）")
        self.text(s, 13.5, 8.70, 10.0, 3.6, trim_txt, 10, DGRAY, line_spacing=1.1)

        # 10 开发方法
        s = self.slide("light", "03  METHOD", "开发方法：假下位机 → 仿真回归 → 真机测试矩阵", "TESTING", notes=(
            "开发方法上，我们不直接上车试。第一层是离线自测：给每个下位机写了假设备，底盘的假控制器按厂家固件源码复现，"
            f"自测脚本共 {selftest} 项。第二层是仿真回归：仿真器模拟相机噪声、帧率、舵机回差、底盘滑行，"
            f"默认配置下仿真成功率为：{sim_txt}，这是仿真数字，不代表真机。第三层是真机测试矩阵：近中远乘左中右九个位置，"
            "每次由人确认真实结果，程序判定不算。每次修改都要先过前两层，成功率不下降才上车。"))
        layers = [("离线自测", f"{selftest} 项", "假下位机覆盖全部驱动；底盘假控制器按厂家固件源码复现"),
                  ("仿真回归", "仿真", "模拟相机噪声、帧率、跟踪器、舵机回差、底盘滑行；改动前后对比成功率"),
                  ("真机测试矩阵", "3 × 3", "近 / 中 / 远 × 左 / 中 / 右，人工确认真实结果，程序判定不算")]
        for i, (t, big, d) in enumerate(layers):
            x = 1.40 + i * 7.67
            self.rect(s, x, 3.30, 7.2, 6.0, fill=NAVY if i == 1 else WHITE, round_=True, shadow=i != 1)
            dark = i == 1
            self.text(s, x + 0.6, 3.75, 6.0, 0.6, f"{i + 1:02d}  {t}", 12, ORANGE, True)
            self.text(s, x + 0.6, 4.60, 6.0, 1.5, big, 30, WHITE if dark else NAVY, True, NUM_FONT if i != 1 else CN_FONT)
            self.text(s, x + 0.6, 6.40, 6.0, 2.6, d, 10.5, SOFT if dark else DGRAY)
        simrows = [["仿真场景（默认配置，100 次）", "真实抓到", "平均对准步数"]]
        for r in sims:
            simrows.append([r["场景"] + "（仿真）", f"{r['抓到']} / {r['次数']}", str(r.get("平均对准步数", "—"))])
        if len(simrows) > 1:
            self.ptable(s, simrows, 1.40, 9.85, 15.0, [8.6, 3.2, 3.2], font=9.5, row_h=0.72)
        self.text(s, 16.9, 9.95, 7.1, 2.4, "仿真数字只用于比较改动前后是否变差，不代表真机成功率；真机结果见测试结果页", 9.5, DGRAY)

        # 11 问题与解决
        s = self.slide("light", "03  CHALLENGES", "开发难点与解决方法", notes=(
            "这一页是调试中遇到的主要问题。升降轴连续发指令不动，是旧固件运动中拒绝新指令，我们重写了运动规划；"
            "机械臂控制板 USB 识别失败，就改用 J-Link 的虚拟串口连控制板串口；底盘走 100 毫米停在约 145 毫米，"
            "是电机滑行，我们学习滑行时间常数提前停车；跟踪器把爪子当成物块，用 K230 屏蔽区和树莓派自身干扰检测两道防线；"
            "没抓到也报抓到，改成观察确认。"))
        probs = [("连续发“A 10”升降轴不动", "重写运动规划，运动中可改目标"),
                 ("升降轴第一条指令回 ERR", "打开串口先发空行清掉残留半行"),
                 ("机械臂控制板 USB 识别失败", "改用 J-Link 虚拟串口连控制板串口"),
                 ("底盘走 100 mm 停在约 145 mm", "学习滑行时间常数，提前停车"),
                 ("跟踪器把爪子当物块，底座转到极限", "K230 屏蔽区 + 树莓派自身干扰检测"),
                 ("没抓到也报“抓到”", "抬升确认改为观察确认")]
        for i, (a, b) in enumerate(probs):
            yy = 3.20 + i * 1.55
            self.rect(s, 1.40, yy, 22.6, 1.30, fill=WHITE, round_=True, shadow=True)
            self.text(s, 2.0, yy, 1.2, 1.3, f"{i + 1:02d}", 15, ORANGE, True, NUM_FONT, anchor=MSO_ANCHOR.MIDDLE)
            self.text(s, 3.4, yy, 10.6, 1.3, a, 11.5, NAVY, anchor=MSO_ANCHOR.MIDDLE)
            self.text(s, 14.0, yy, 1.0, 1.3, "→", 14, ORANGE, True, anchor=MSO_ANCHOR.MIDDLE)
            self.text(s, 15.0, yy, 8.8, 1.3, b, 11.5, NAVY, True, anchor=MSO_ANCHOR.MIDDLE)

        # 12 版本迭代
        s = self.slide("light", "03  ITERATION", "版本迭代：从“分级规则”到“自适应 + 可验证”", notes=(
            "和上一版比较：总控从 STM32 改成树莓派，偏差分级的手调规则换成了自适应视觉伺服；相机从固定位置改到手腕；"
            "伺服参数从手工设定变成在线学习；升降轴快了十倍以上；新增了抓取确认、自身干扰识别和多重停车保护；"
            "测试方法从直接上车试，变成离线自测、仿真回归加真机测试矩阵。"))
        rows = [["模块", "上一版", "本版本"],
                ["总控", "STM32F103，偏差分级分配", "树莓派，统一状态机 + 自适应视觉伺服"],
                ["相机", "{{核对:上一版相机位置}}", "腕部，眼在手上"],
                ["伺服参数", "手工设定阈值、步长", "在线学习灵敏度，结果保存复用"],
                ["升降轴", "约 6.8 mm/s，运动中不接受新指令", "80 mm/s（固件默认值），运动中可改目标"],
                ["抓取确认", "无", "观察确认，失败自动重试"],
                ["自身干扰", "无", "K230 屏蔽 + 树莓派检测"],
                ["安全停车", "看门狗 + 视觉丢失停车", "再加进程终止、回传中断停车、复位保护"],
                ["测试方法", "上车试", "离线自测 + 仿真回归 + 真机测试矩阵"]]
        self.ptable(s, rows, 1.40, 3.20, 22.6, [4.0, 8.3, 10.3], font=10.5, row_h=1.0)

        # 13 测试结果
        s = self.slide("light", "04  RESULTS", "真机测试结果：低平台、无底盘", "RESULTS", notes=self._results_notes(ok, n, t_avg))
        self.text(s, 1.40, 3.05, 12.0, 0.7, "测试矩阵（真实抓到 / 次数，人工判定）", 12, NAVY, True)
        grid = [["", "左 −10°", "中 0°", "右 +10°"]]
        hl = set()
        cells = (c.bench or {}).get("cells", {}) if c.bench else {}
        for ri, rn in enumerate(["近 15 cm", "中 18 cm", "远 21 cm"]):
            row = [rn]
            for ci in range(3):
                cc = cells.get(ri * 3 + ci + 1)
                if cc:
                    row.append(f"{cc['抓到']}/{cc['次数']}\n{pct(cc['抓到'], cc['次数'])}")
                else:
                    row.append("【待真机】")
                    hl.add((ri + 1, ci + 1))
            grid.append(row)
        self.ptable(s, grid, 1.40, 3.90, 12.0, [3.0, 3.0, 3.0, 3.0], font=12, row_h=1.75, hl_cells=hl)
        if not c.bench:
            c.missing.append((c.where, "待真机：bench 测试矩阵 9 个位置的结果"))
        stats = []
        stats.append(((f"{pct(ok, n)}" if n else "【待真机】"), "总成功率" + (f"（{ok}/{n}）" if n else "")))
        stats.append(((f"{t_avg} s" if t_avg is not None else "【待真机】"), "平均每次用时"))
        if not n:
            c.missing.append((c.where, "待真机：低平台无底盘总成功率、平均用时"))
        fd = c.get("真机.第一天联调") or {}
        stats.append((f"{fd.get('抓到')}/{fd.get('总次数')}", "第一天联调记录（11-03，未分位置）"))
        if chassis_sw is True:
            ch = c.get("真机.有底盘") or {}
            stats.append((f"{c.val('真机.有底盘.抓到', '有底盘抓到')}/{c.val('真机.有底盘.总次数', '有底盘次数')}",
                          "有底盘全自动抓取"))
        elif chassis_sw is None:
            stats.append(("【待联调】", "有底盘全自动抓取【请核对】"))
        for i, (big, lab) in enumerate(stats[:4]):
            x = 14.0 + (i % 2) * 5.1
            y = 3.90 + (i // 2) * 3.55
            self.rect(s, x, y, 4.85, 3.25, fill=NAVY if i == 0 else WHITE, round_=True, shadow=i != 0)
            self.text(s, x + 0.35, y + 0.35, 4.3, 1.5, big, 24, ORANGE if i == 0 else NAVY, True, NUM_FONT)
            self.text(s, x + 0.35, y + 1.95, 4.3, 1.2, lab, 10, SOFT if i == 0 else DGRAY)
        self.text(s, 1.40, 11.35, 22.6, 1.2, "判定：每次抓取后由人确认是否真的抓到（程序判定不算）；误报 = 程序报抓到、实际没抓到。"
                  + (f" 数据来源：{c.bench['src']}" if c.bench else ""), 9.5, DGRAY)

        # 14 视频 ①
        s = self.slide("dark", "04  DEMO", "全自动抓取一镜到底", notes=(
            "这段视频是一镜到底、中间没有剪辑的全自动抓取。可以看到：先搜索，看到物块后对准，机械臂伸出、合爪，"
            "然后退回观察姿态确认原位置已经没有物块，最后放进筐里。整个过程没有人干预。"
            "【视频时长和拍摄条件按实际情况补充】"))
        self.video_slot(s, "①", 1.40, 3.30, 14.2, 8.0)
        self.steps_list(s, 16.6, 3.40, [("搜索", "升降轴换高度、底座左右扫"), ("对准", "三个方向同时收敛到容差内"),
                                         ("伸出夹取", "按深度插值伸出，合爪"), ("观察确认", "原位置没有物块 = 抓到"),
                                         ("放置", "放进筐里，回到观察姿态")], gap=1.75)

        # 15 视频 ③
        s = self.slide("dark", "04  DEMO", "对准过程：每一步都在测量、修正、学习", notes=(
            "这一段是对准过程的特写，右边是终端日志。每一行是一次对准：显示三个方向的误差、这一步各执行机构走了多少。"
            "可以看到误差一步步变小，一般几步就进入容差，然后开始抓取。"))
        self.video_slot(s, "③", 1.40, 3.30, 14.2, 8.0)
        self.steps_list(s, 16.6, 3.40, [("左：机械臂特写", "底座和升降轴的每一步动作"),
                                         ("右：终端日志", "“对准 #n 误差 …”逐步变小"),
                                         ("几步收敛", "进入方位 1.5°、竖直 4 mm、深度 10 mm 容差")], gap=2.2)

        # 16 视频 ④⑤
        s = self.slide("dark", "04  DEMO", "连续抓取与失败自动重试", notes=(
            "左边是在测试矩阵不同位置上的连续抓取，加速播放，字幕标了第几次。右边是一次没抓到后，程序通过观察确认发现物块还在，"
            "自动松开、后退、再抓一次成功的片段。这个片段不是刻意制造的，是测试中实际出现的。"))
        self.video_slot(s, "④", 1.40, 3.30, 10.9, 6.6)
        self.video_slot(s, "⑤", 13.08, 3.30, 10.9, 6.6)
        self.text(s, 1.40, 11.2, 22.6, 1.0, "连续抓取片段与测试矩阵同时记录（bench.py），结果见测试结果页；重试片段为测试中实际出现", 10.5,
                  SOFT)

        # 17 视频 ⑦⑧
        s = self.slide("dark", "04  DEMO", "升降轴速度与底盘参与抓取", notes=(
            "左边是升降轴从 0 升到 30 厘米再降回来，重写固件后最高速度每秒 80 毫米。"
            + ("右边是底盘参与的全自动抓取：车从四五十厘米外开过去，边走边对准。" if chassis_sw is not False else
               "底盘参与的全自动抓取还没有完成联调，这次演示以无底盘模式为主。")))
        self.video_slot(s, "⑦", 1.40, 3.30, 10.9, 6.6)
        if chassis_sw is False:
            self.rect(s, 13.08, 3.30, 10.9, 6.6, fill=CARD_DARK, line=GRAY, dash=True)
            self.text(s, 13.08, 5.8, 10.9, 1.6, "底盘参与的全自动抓取尚未完成联调\n本次演示以无底盘模式为主", 13, SOFT,
                      align=PP_ALIGN.CENTER)
        else:
            self.video_slot(s, "⑧", 13.08, 3.30, 10.9, 6.6)
        self.text(s, 1.40, 11.2, 22.6, 1.0, "升降轴：伺服电机 + 丝杠，0–50 cm，软件限位 44 cm；运动中可随时改目标", 10.5, SOFT)

        # 18 创新点
        s = self.slide("light", "05  INNOVATION", "作品特色与创新点", "INNOVATION", notes=(
            "总结四个创新点。第一，眼在手上的在线自适应视觉伺服，不用手眼标定，边运行边学习。第二，三套执行机构按误差方向分工，"
            "阿克曼底盘用几何前馈保持对准。第三，面向低成本硬件的可靠性设计，每一条保护都来自真实问题。"
            "第四，工程化的开发方法，假下位机、仿真回归、真机测试矩阵，让每次改动都可验证、可对比。"))
        inn = [("01", "在线自适应视觉伺服", "眼在手上，免手眼标定；边运行边学习执行机构灵敏度，换车、换装配自动适应"),
               ("02", "三执行机构按误差分工", "升降轴管竖直、底座管方位、底盘管深度；阿克曼底盘几何前馈保持对准"),
               ("03", "面向低成本硬件的可靠性", "自身干扰识别、心跳区分断线、观察确认、多重停车保护"),
               ("04", "可验证的开发方法", "假下位机 + 仿真回归 + 真机测试矩阵，每次改动可验证、可对比")]
        for i, (num, t, d) in enumerate(inn):
            x = 1.40 + (i % 2) * 11.5
            y = 3.30 + (i // 2) * 3.9
            self.card(s, x, y, 11.1, 3.4, num, t, d, title_size=15, desc_size=11)

        # 19 不足与展望
        s = self.slide("light", "05  OUTLOOK", "不足与展望", notes=(
            "我们也如实说明不足：现在的机械结构够不到地面上的物块，只能抓台面上的；爪子不能绕竖直轴转，歪放的物块成功率低；"
            + ("底盘参与的全自动抓取还在联调；" if chassis_sw is not True else "")
            + "K230 每秒约 7 帧，限制了对准速度。下一步：改机械结构实现地面取物，增加腕部旋转，"
            "用记录的动作和画面数据让机器人自监督地学习自己的执行误差，并结合语音实现“说一句话就把东西拿来”。"))
        lack = ["够不到地面上的物块，目前只抓台面上的", "夹爪不能绕竖直轴旋转，歪放的物块成功率低",
                "K230 约 7 帧/s，限制对准速度", "近、远两个伸出姿态线性插值，中间深度高度可能偏"]
        if chassis_sw is not True:
            lack.insert(2, "底盘参与的全自动抓取" + ("尚未完成联调" if chassis_sw is False else "【待联调结果】"))
        self.rect(s, 1.40, 3.30, 10.9, 8.9, fill=WHITE, round_=True, shadow=True)
        self.text(s, 2.0, 3.70, 9.8, 0.7, "如实说明的不足", 14, NAVY, True)
        self.text(s, 2.0, 4.70, 9.8, 7.2, ["· " + x for x in lack], 11, DGRAY, line_spacing=1.35)
        self.rect(s, 12.9, 3.30, 11.1, 8.9, fill=NAVY, round_=True)
        self.text(s, 13.5, 3.70, 10.0, 0.7, "下一步", 14, ORANGE, True)
        nxt = ["改进机械结构（下移安装位置或加长夹爪），实现地面取物", "增加腕部旋转自由度，抓取歪放物体",
               "用记录的动作与画面数据，自监督学习执行误差", "语音指令选择目标：“说一句话就把东西拿来”", "结合激光雷达建图，跨区域取物"]
        self.text(s, 13.5, 4.70, 10.0, 7.2, ["· " + x for x in nxt], 11, WHITE, line_spacing=1.35)

        # 20 致谢
        s = self.slide("blank", notes="我们的汇报就到这里，谢谢各位老师！请各位老师批评指正。")
        self.background(s)
        self.text(s, 1.52, 3.20, 12.0, 0.6, "THANK YOU", 10, ORANGE, True, NUM_FONT, spc=200)
        self.text(s, 1.52, 4.20, 16.0, 3.0, "谢谢！\n敬请各位老师批评指正", 32, WHITE, True, line_spacing=1.1)
        self.text(s, 1.52, 8.40, 14.0, 1.0, c.get("作品.名称"), 14, ORANGE, True)
        self.text(s, 1.52, 9.40, 14.0, 1.0, "参赛队员　" + team, 12, SOFT)
        self.finish(path)

    def _results_notes(self, ok, n, t_avg):
        if n:
            s = (f"这是低平台、无底盘的真机测试结果。测试矩阵是近中远乘左中右九个位置，每次由人确认是否真的抓到。"
                 f"总共 {n} 次，真实抓到 {ok} 次，成功率 {pct(ok, n)}")
            if t_avg is not None:
                s += f"，平均每次 {t_avg} 秒"
            return s + "。【请按表里的具体数字补充：哪些位置好、哪些位置差、主要失败原因】"
        return ("【待真机数据：bench 测试完成后重新生成，讲稿会自动带上总次数和成功率】"
                "这一页是真机测试结果，测试矩阵是近中远乘左中右九个位置，每次抓取后由人确认是否真的抓到，程序判定不算。"
                "第一天联调时，在低平台、无底盘的条件下，15 次中抓到 11 次，每次 15 到 40 秒。")


def make_poster(video, title, out_dir, vid):
    """视频封面：优先用 ffmpeg 截第 1 秒，否则画一张深色占位封面"""
    cache = os.path.join(out_dir, "_cache")
    os.makedirs(cache, exist_ok=True)
    dst = os.path.join(cache, f"poster_{ord(vid[0])}.png")
    ff = shutil.which("ffmpeg")
    if ff:
        try:
            subprocess.run([ff, "-y", "-loglevel", "error", "-ss", "1", "-i", video, "-frames:v", "1",
                            "-vf", "scale=1280:-2", dst], check=True, timeout=60)
            if os.path.exists(dst):
                return dst
        except Exception:
            pass
    try:
        from PIL import Image, ImageDraw
        im = Image.new("RGB", (1280, 720), (22, 32, 43))
        d = ImageDraw.Draw(im)
        d.polygon([(590, 300), (590, 420), (700, 360)], fill=(240, 125, 26))
        im.save(dst)
        return dst
    except Exception:
        return None


# =====================================================================================  作品信息概要表
class SummaryBuilder:
    def __init__(self, ctx):
        self.ctx = ctx
        self.doc = Document(T_SUMMARY)
        self.t = self.doc.tables[0]

    def cells(self, ri):
        out, prev = [], None
        for c in self.t.rows[ri].cells:
            if c._tc is not prev:
                out.append(c)
            prev = c._tc
        return out

    @staticmethod
    def set_par(p, text):
        """保留第一个 run 的格式，换掉段落文字；【…】黄色高亮"""
        runs = p.runs
        rPr = copy.deepcopy(runs[0]._r.rPr) if runs and runs[0]._r.rPr is not None else None
        for r in runs:
            r._r.getparent().remove(r._r)
        for seg, b, hl in segments(text):
            r = p.add_run(seg)
            if rPr is not None:
                r._r.insert(0, copy.deepcopy(rPr))
            if hl:
                r.font.highlight_color = WD_COLOR_INDEX.YELLOW

    def set_cell(self, cell, lines, keep_first=0):
        ps = cell.paragraphs
        start = int(keep_first)
        tmpl = ps[start] if len(ps) > start else ps[-1]
        for p in ps[start + 1:]:
            p._p.getparent().remove(p._p)
        if len(ps) <= start:
            new = copy.deepcopy(tmpl._p)
            tmpl._p.addnext(new)
        target = cell.paragraphs[start]
        self.set_par(target, lines[0])
        last = target._p
        for ln in lines[1:]:
            new = copy.deepcopy(target._p)
            last.addnext(new)
            last = new
            from docx.text.paragraph import Paragraph
            self.set_par(Paragraph(new, target._parent), ln)

    def check_len(self, label, s, limit=100):
        n = len(re.sub(r"\s", "", s))
        if n > limit:
            self.ctx.missing.append(("概要表", f"超字数：{label} {n} 字（限 {limit} 字）"))
            return s + f"【超字数：{n}/{limit}】"
        return s

    def build(self, path):
        c = self.ctx
        c.where = "概要表"
        g = lambda k: c.get("概要表." + k)  # noqa: E731
        self.set_cell(self.cells(0)[1], [c.get("作品.名称")])
        self.set_cell(self.cells(1)[0], [self.check_len("作品简介", g("作品简介"))], keep_first=True)
        self.set_cell(self.cells(2)[0], [self.check_len("创新描述", g("创新描述"))], keep_first=True)
        self.set_cell(self.cells(3)[0], [g("特别说明")], keep_first=2)
        names = c.get("队员.名单") or []
        mark = "" if c.get("队员.已核对") else "【请核对】"
        hdr = self.cells(5)
        for i in range(1, len(hdr)):
            self.set_cell(hdr[i], [(names[i - 1] + mark) if i - 1 < len(names) else ""])
        if not c.get("队员.已核对"):
            c.missing.append(("概要表", "请核对：队员名单"))
        work = (c.get("分工.表") or {})
        wmark = "" if c.get("分工.已核对") else "【请核对】"
        if not c.get("分工.已核对"):
            c.missing.append(("概要表", "请核对：分工比例（从旧表照搬）"))
        for ri in range(6, 14):
            row = self.cells(ri)
            item = row[0].text.strip()
            if item in work:
                for i, v in enumerate(work[item]):
                    if i + 1 < len(row):
                        self.set_cell(row[i + 1], [v])
        self.set_cell(self.cells(5)[0], ["项目" + (wmark and "【请核对：分工比例】")])
        self.set_cell(self.cells(15)[1], [g("开发制作平台")])
        self.set_cell(self.cells(16)[1], [g("运行展示平台")])
        self.set_cell(self.cells(17)[1], [g("开发制作工具")])
        self.set_cell(self.cells(18)[1], g("参考文献"))
        files = [
            ("设计报告_移动机械臂感知与作业一体化平台.pdf", "作品设计报告", "■自制□未知版权", "□开源□获得授权"),
            ("答辩PPT（.pptx 及 .pdf）", "答辩演示文稿（含讲稿）", "■自制□未知版权", "□开源□获得授权"),
            ("演示视频（全自动抓取一镜到底、对准特写、连续抓取等）", "整机全自动抓取演示", "■自制□未知版权", "□开源□获得授权"),
            ("raspberrypi/grasp", "树莓派总控：任务状态机、自适应视觉伺服、设备驱动、标定、仿真与离线自测", "■自制□未知版权",
             "□开源□获得授权"),
            ("k230_sdcard", "K230 目标检测、单目测距、心跳帧、夹爪屏蔽区【请核对：检测部署部分是否基于嘉楠例程】", "■自制□未知版权",
             "□开源□获得授权"),
            ("伺服电机控制", "升降轴 STM32F103 固件（重写运动规划与串口协议）", "■自制□未知版权", "□开源□获得授权"),
            ("PWM机械臂", "机械臂控制板固件，在 Lobot 开源例程基础上二次开发", "□自制□未知版权", "■开源□获得授权"),
            ("环境感知板固件", "火焰、烟雾、光照、温湿度采集与报警（沿用上一版）", "■自制□未知版权", "□开源□获得授权"),
            ("底盘代码", "WHEELTEC 厂家底盘固件，未修改，仅作协议参考【请核对：版权状态】", "□自制□未知版权", "□开源■获得授权"),
        ]
        c.missing.append(("概要表", "请核对：相关文件列表与版权状态（k230_sdcard、底盘代码两行）"))
        base_ri = 22
        tmpl_tr = self.t.rows[28]._tr
        # 行不够就复制最后一行
        while len(self.t.rows) - 1 - base_ri < len(files):     # 最后一行是“特别申明”
            new = copy.deepcopy(tmpl_tr)
            self.t.rows[-2]._tr.addnext(new)
        for i, (fn, desc, cp1, cp2) in enumerate(files):
            row = self.cells(base_ri + i)
            self.set_cell(row[0], [str(i + 1)])
            self.set_cell(row[1], [f"文件名：{fn}", f"描述：{desc}"])
            self.set_cell(row[-1], [cp1, cp2])
        self.doc.save(path)


# =====================================================================================  主程序
def to_pdf(paths, out_dir):
    cands = [shutil.which("soffice"), shutil.which("libreoffice"),
             r"C:\Program Files\LibreOffice\program\soffice.exe",
             r"C:\Program Files (x86)\LibreOffice\program\soffice.exe"]
    exe = next((x for x in cands if x and os.path.exists(x)), None)
    if not exe:
        print("⚠️ 没找到 LibreOffice，跳过 PDF。Windows 上请用 Word / PowerPoint：文件 → 另存为 → PDF")
        return
    for p in paths:
        print(f"  转 PDF：{os.path.basename(p)}")
        subprocess.run([exe, "--headless", "--convert-to", "pdf", "--outdir", out_dir, p],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=300)
    print("  注意：LibreOffice 不会刷新 Word 目录的页码，正式提交请用 Office 更新目录后导出 PDF")


def main():
    ap = argparse.ArgumentParser(description="生成 ROBOTAC 比赛材料（设计报告、答辩 PPT、作品信息概要表）")
    ap.add_argument("--data", default=os.path.join(HERE, "data.json"), help="数据文件（默认 build/data.json）")
    ap.add_argument("--out", default=os.path.join(HERE, "out"), help="输出目录（默认 build/out）")
    ap.add_argument("--only", choices=["report", "ppt", "summary"], action="append", help="只生成某一份（可重复）")
    ap.add_argument("--figures", action="store_true", help="重新画架构图和伺服流程图")
    ap.add_argument("--pdf", action="store_true", help="用 LibreOffice 转 PDF（可选）")
    ap.add_argument("--tag", default=None, help="输出文件名后缀（默认 data.json 的 版本标签，再没有用日期）")
    args = ap.parse_args()

    with open(args.data, encoding="utf-8") as f:
        data = json.load(f)
    os.makedirs(args.out, exist_ok=True)
    ctx = Ctx(data, os.path.dirname(os.path.abspath(args.data)))
    tag = args.tag or data.get("版本标签") or datetime.date.today().strftime("%m%d")

    fig_names = ["系统总体架构.png", "自适应视觉伺服流程.png"]
    if args.figures or any(not os.path.exists(os.path.join(HERE, "figures", n)) for n in fig_names):
        import figures          # 需要 matplotlib；图已经在仓库里时不用装
        figures.build_all(force=True)

    only = set(args.only or ["report", "ppt", "summary"])
    outs = []
    if "report" in only:
        with open(DRAFT_MD, encoding="utf-8") as f:
            md = f.read()
        md = "\n".join(ln for ln in md.splitlines() if not ln.lstrip().startswith(">"))
        md = apply_conditionals(md, ctx)
        blocks = parse_md(md)
        number_blocks(blocks)
        p = os.path.join(args.out, f"设计报告_{tag}.docx")
        ReportBuilder(ctx, args.out).build(blocks, p)
        outs.append(p)
        print(f"✅ 设计报告：{p}")
    if "ppt" in only:
        p = os.path.join(args.out, f"答辩PPT_{tag}.pptx")
        PPTBuilder(ctx, args.out).build(p)
        outs.append(p)
        print(f"✅ 答辩 PPT：{p}")
    if "summary" in only:
        p = os.path.join(args.out, f"作品信息概要表_{tag}.docx")
        SummaryBuilder(ctx).build(p)
        outs.append(p)
        print(f"✅ 作品信息概要表：{p}")

    # 缺项清单（去重，保持顺序）
    seen, items = set(), []
    for where, what in ctx.missing:
        if (where, what) not in seen:
            seen.add((where, what))
            items.append((where, what))
    lst = os.path.join(args.out, "缺项清单.txt")
    with open(lst, "w", encoding="utf-8") as f:
        f.write(f"生成时间 {datetime.datetime.now():%Y-%m-%d %H:%M}，共 {len(items)} 项（文档里都是黄色高亮）\n\n")
        for where, what in items:
            f.write(f"[{where}] {what}\n")
    print(f"📋 还缺 {len(items)} 项（文档里黄色高亮），清单：{lst}")
    if args.pdf:
        to_pdf(outs, args.out)
    print("下一步：Word 里右键目录 → 更新域 → 更新整个目录；检查黄色高亮；导出 PDF。")


if __name__ == "__main__":
    main()
