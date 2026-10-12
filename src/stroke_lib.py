"""汉字笔画库 —— 给机器人写毛笔字用的"笔画折线"。

两条获取路径，按可靠性排序：
  1) 【首选】汉字笔顺数据集 hanzi-writer-data（Make Me a Hanzi / 文鼎楷体派生）
     —— 每条笔画自带中心线（medians）与真实笔顺，一=1 笔、去=5 笔、家=10 笔，
     和楷书字帖完全一致。首次取某字需要联网，取到后落盘缓存，之后永久离线可用。
  2) 【兜底】本地字体自动提骨架（fontTools + 光栅化 + Zhang-Suen 细化）
     —— 数据集里没有的生僻字可以用它硬凑；质量不如路径 1，复杂字（家/座）会散架。

产出格式：strokes(ch) -> [[(x0,y0), (x1,y1), ...], ...]
    坐标 0~1，(0,0)=田字格左上角，(1,1)=右下角，y 轴向下
    每条折线 = 一"笔"（落笔→走完→抬笔），笔与笔之间由执行层抬笔搬运
    列表顺序 = 书写顺序（数据集路径下就是标准笔顺）

★ 比赛现场不要依赖网络：先用 `python src/stroke_lib.py --fetch 全诗文本` 把要用到的字
  全部取下来存进 data/strokes_cache.json，比赛时只读缓存。

用法：
    python src/stroke_lib.py 一去二三里          # 看笔画数/笔长（缺字会尝试联网抓取）
    python src/stroke_lib.py --fetch 一去二三里  # 只抓取并落盘，不打印
    python src/stroke_lib.py --offline 一去二三里 # 禁止联网，只读缓存
    python src/stroke_lib.py --rebuild 一去二三里 # 忽略缓存重算（骨架兜底路径会重跑）
"""
from __future__ import annotations

import glob
import json
import math
import os
import socket
import sys
import time
import urllib.parse
import urllib.request

import numpy as np

# macOS 上 jsdelivr 的 IPv6 路由经常不通：connect 挂 SYN_SENT 直到超时（每字白等 20s）。
# 强制 urllib 只走 IPv4（10-12 赛场实测：IPv4 1-2s/字，IPv6 必卡死）
_orig_getaddrinfo = socket.getaddrinfo
def _ipv4_getaddrinfo(*a, **kw):
    return [r for r in _orig_getaddrinfo(*a, **kw) if r[0] == socket.AF_INET]
socket.getaddrinfo = _ipv4_getaddrinfo

# ---------------- 坐标基准（把数据集坐标搬进田字格） ----------------
# 数据集坐标：y 轴向上、基线在 y=0；楷体设计里"满格字"大致落在 x∈[60,960]、y∈[-40,860]。
# 我们取一个 900x900 的正方形当基准框，等比映射到田字格中间，四周留 (1-GLYPH_FILL)/2 的边。
FRAME_X0, FRAME_X1 = 62.0, 962.0        # 基准框左右（数据集单位）
FRAME_TOP_UP, FRAME_BOT_UP = 860.0, -40.0   # 基准框上下（数据集单位，y 向上）
GLYPH_FILL = 0.80                       # 字占田字格比例。规则：字迹超出田字格边框记 0 分 → 宁可小
                                        #   0.80 实测：字的中心线距格线还有 7mm，加笔宽也在格内
CACHE_VERSION = 5                       # 算法/基准框/GLYPH_FILL 改过就 +1，旧缓存自动失效

# 骨架兜底路径的可调常量
RASTER_PX = 320        # 细化分辨率，字形越复杂越要调高
SIMPLIFY_EPS = 0.008   # RDP 简化容差（归一化单位，1 = 整格）
MIN_STROKE = 0.035     # 短于此长度的碎笔画丢弃
MERGE_TOL = 0.045      # 端点接近且方向连续的碎段合并成一笔
MERGE_MAX_TURN = 105   # 合并处折角上限（度）

DATASET_MIRRORS = [
    "https://cdn.jsdelivr.net/npm/hanzi-writer-data@2.0.1/{ch}.json",
    "https://unpkg.com/hanzi-writer-data@2.0.1/{ch}.json",
    "https://registry.npmmirror.com/hanzi-writer-data/2.0.1/files/{ch}.json",
]
HTTP_TIMEOUT = 20
_HERE = os.path.dirname(os.path.abspath(__file__))
CACHE_FILE = os.path.normpath(os.path.join(_HERE, "..", "data", "strokes_cache.json"))

# 骨架兜底用的字体候选（优先楷体）
FONT_CANDIDATES = [
    "~/robot-contest-2026/assets/*.tt[fc]",
    "/System/Library/Fonts/STKaiti.ttc",
    "/System/Library/Fonts/Supplemental/Songti.ttc",
    "/System/Library/Fonts/PingFang.ttc",
    "C:/Windows/Fonts/simkai.ttf", "C:/Windows/Fonts/STKAITI.TTF",
    "C:/Windows/Fonts/simsun.ttc", "C:/Windows/Fonts/simhei.ttf",
    "/usr/share/fonts/**/STKaiti.ttf", "/usr/share/fonts/**/KaiTi.ttf",
    "/usr/share/fonts/**/simkai.ttf",
    "/usr/share/fonts/**/NotoSerifCJK*.ttc", "/usr/share/fonts/**/NotoSerifCJK*.otf",
    "/usr/share/fonts/**/wqy-zenhei.ttc",
]


# ============================================================================
# 数据集路径
# ============================================================================
def fetch_char(ch):
    """从镜像下载单字数据，返回归一化笔画；失败返回 None（不抛异常，保持调用方简单）。"""
    last = None
    for tpl in DATASET_MIRRORS:
        url = tpl.format(ch=urllib.parse.quote(ch))
        try:
            with urllib.request.urlopen(url, timeout=HTTP_TIMEOUT) as r:
                data = json.loads(r.read().decode("utf-8"))
            medians = data.get("medians") or []
            return [normalize_dataset(m) for m in medians if len(m) >= 2]
        except Exception as e:                       # noqa: BLE001
            last = e
            continue
    print(f"[笔画库] 取字失败 {ch!r}: {last}", file=sys.stderr)
    return None


def normalize_dataset(points):
    """数据集坐标 → 田字格 0~1（y 向下）。"""
    x0, x1 = FRAME_X0, FRAME_X1
    top, bot = FRAME_TOP_UP, FRAME_BOT_UP
    w = x1 - x0
    h = top - bot
    pad = (1.0 - GLYPH_FILL) / 2.0
    out = []
    for x, y in points:
        nx = pad + (x - x0) / w * GLYPH_FILL
        ny = pad + (top - y) / h * GLYPH_FILL
        out.append((round(float(nx), 5), round(float(ny), 5)))
    return out


def simplify(points, eps=0.004):
    """RDP 简化，把数据集的密点压成机器人好插补的稀疏折线（0.004 ≈ 0.3mm/格 77mm）。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        return [tuple(p) for p in pts]
    keep = np.zeros(len(pts), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        seg = pts[j] - pts[i]
        nrm = float(np.hypot(*seg))
        mid = pts[i + 1:j]
        if nrm < 1e-12:
            d = np.hypot(*(mid - pts[i]).T)
        else:
            d = np.abs((mid[:, 0] - pts[i][0]) * seg[1] - (mid[:, 1] - pts[i][1]) * seg[0]) / nrm
        if d.size == 0:
            continue
        k = int(np.argmax(d)) + i + 1
        if d.max() > eps:
            keep[k] = True
            stack.append((i, k))
            stack.append((k, j))
    return [tuple(p) for p in pts[keep]]


# ============================================================================
# 兜底路径：本地字库 → 骨架
# ============================================================================
def _expand(path):
    return sorted(glob.glob(os.path.expanduser(path.replace("\\", "/")), recursive=True))


def find_font(preferred=None):
    for cand in ([preferred] if preferred else []) + ([os.environ["BRUSH_FONT"]]
                                                      if os.environ.get("BRUSH_FONT") else []) \
            + FONT_CANDIDATES:
        hits = _expand(cand)
        if hits:
            return hits[0]
    raise FileNotFoundError("找不到中文字体：放一个 .ttf/.ttc 到 assets/ 或设 BRUSH_FONT")


def _quad(p0, p1, p2, n=8):
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 2 * np.asarray(p0) + 2 * (1 - t) * t * np.asarray(p1) + t ** 2 * np.asarray(p2)


def _cubic(p0, p1, p2, p3, n=10):
    t = np.linspace(0, 1, n)[:, None]
    m = 1 - t
    return (m ** 3 * np.asarray(p0) + 3 * m ** 2 * t * np.asarray(p1)
            + 3 * m * t ** 2 * np.asarray(p2) + t ** 3 * np.asarray(p3))


def _flatten(commands):
    """RecordingPen 指令 → 折线多边形。"""
    polys, cur = [], []
    for op, args in commands:
        if op == "moveTo":
            if len(cur) > 1:
                polys.append(cur)
            cur = [tuple(args[0])]
        elif op == "lineTo":
            cur.append(tuple(args[0]))
        elif op in ("qCurveTo", "quadTo"):
            ctrl, on = list(args[:-1]), tuple(args[-1])
            if len(ctrl) == 1:
                cur.extend(tuple(p) for p in _quad(cur[-1], ctrl[0], on)[1:])
            else:
                p = cur[-1]
                for i, c in enumerate(ctrl):
                    nxt = on if i == len(ctrl) - 1 else ((ctrl[i][0] + ctrl[i + 1][0]) / 2.0,
                                                         (ctrl[i][1] + ctrl[i + 1][1]) / 2.0)
                    cur.extend(tuple(q) for q in _quad(p, c, nxt)[1:])
                    p = nxt
        elif op == "curveTo":
            pts = list(args)
            if len(pts) >= 3:
                cur.extend(tuple(p) for p in _cubic(cur[-1], pts[0], pts[1], pts[2])[1:])
        elif op == "closePath":
            if len(cur) > 2:
                polys.append(cur + [cur[0]])
            cur = []
        elif op == "endPath":
            if len(cur) > 2:
                polys.append(cur)
            cur = []
    if len(cur) > 1:
        polys.append(cur)
    return polys


def _rasterize(polys, size):
    """扫描线 + 非零环绕填充（TrueType 规则，交叠笔画不会掏洞）。"""
    img = np.zeros((size, size), dtype=bool)
    xs, ys, ws = [], [], []
    for poly in polys:
        p = np.asarray(poly, dtype=float)
        if len(p) < 3:
            continue
        a, b = p[:-1], p[1:]
        keep = a[:, 1] != b[:, 1]
        a, b = a[keep], b[keep]
        if len(a) == 0:
            continue
        swap = a[:, 1] > b[:, 1]
        lo = np.where(swap[:, None], b, a)
        hi = np.where(swap[:, None], a, b)
        xs.append(np.stack([lo[:, 0], hi[:, 0]], 1))
        ys.append(np.stack([lo[:, 1], hi[:, 1]], 1))
        ws.append(np.where(swap, -1.0, 1.0))
    if not xs:
        return img
    ex, ey, ew = np.vstack(xs), np.vstack(ys), np.concatenate(ws)
    for row in range(size):
        yc = row + 0.5
        hit = (ey[:, 0] <= yc) & (yc < ey[:, 1])
        if not hit.any():
            continue
        x0, x1 = ex[hit, 0], ex[hit, 1]
        y0, y1 = ey[hit, 0], ey[hit, 1]
        cross = x0 + (x1 - x0) * (yc - y0) / (y1 - y0)
        idx = np.clip(np.floor(cross + 0.5).astype(int), 0, size)
        diff = np.zeros(size + 1, dtype=np.int32)
        np.add.at(diff, idx, ew[hit].astype(np.int32))
        img[row] = np.cumsum(diff[:-1]) != 0
    return img


def _thin(img, max_iter=500):
    """Zhang-Suen 细化 → 1 像素宽骨架。"""
    I = img.astype(np.uint8)
    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            P = np.pad(I, 1)
            p2 = P[0:-2, 1:-1]; p3 = P[0:-2, 2:]; p4 = P[1:-1, 2:]
            p5 = P[2:, 2:];    p6 = P[2:, 1:-1]; p7 = P[2:, 0:-2]
            p8 = P[1:-1, 0:-2]; p9 = P[0:-2, 0:-2]
            B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            A = _step(p9, p2) + _step(p2, p3) + _step(p3, p4) + _step(p4, p5) \
                + _step(p5, p6) + _step(p6, p7) + _step(p7, p8) + _step(p8, p9)
            if step == 0:
                m = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                m = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            mask = (I == 1) & (B >= 2) & (B <= 6) & (A == 1) & m
            if mask.any():
                I[mask] = 0
                changed = True
        if not changed:
            break
    return I.astype(bool)


def _step(a, b):                                   # 0→1 跳变计数
    return ((a == 0) & (b == 1)).astype(np.uint8)


_NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _nb(p, pix):
    return [(p[0] + dy, p[1] + dx) for dy, dx in _NB8 if (p[0] + dy, p[1] + dx) in pix]


def _angdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return abs(d)


def _trace(skel):
    ys, xs = np.nonzero(skel)
    pix = set(zip(ys.tolist(), xs.tolist()))
    if not pix:
        return []
    deg = {p: len(_nb(p, pix)) for p in pix}
    visited, strokes = set(), []

    def walk(start):
        poly = [start]
        visited.add(start)
        prev = cur = start
        step_dir = None
        while True:
            cands = [q for q in _nb(cur, pix) if q != prev and (q not in visited or deg.get(q, 1) >= 3)]
            if not cands:
                break
            if len(cands) == 1:
                nxt = cands[0]
            else:
                d = math.atan2(cur[1] - prev[1], cur[0] - prev[0]) if step_dir is None else step_dir
                nxt = min(cands, key=lambda q: _angdiff(math.atan2(q[1] - cur[1], q[0] - cur[0]), d))
            step_dir = math.atan2(nxt[1] - cur[1], nxt[0] - cur[0])
            visited.add(nxt)
            poly.append(nxt)
            prev, cur = cur, nxt
            if deg.get(cur, 1) == 1 or len(poly) > 4000:
                break
        return poly

    for p in [p for p, d in deg.items() if d == 1]:
        if p not in visited:
            strokes.append(walk(p))
    for p in sorted(pix):
        if p not in visited:
            strokes.append(walk(p))
    return strokes


def _pt_seg_dist(pts, poly):
    a = np.asarray(poly, dtype=float)
    if len(a) == 1:
        return np.hypot(*(pts - a[0]).T)
    p0, p1 = a[:-1], a[1:]
    d = p1 - p0
    L2 = (d ** 2).sum(1)
    L2[L2 == 0] = 1e-12
    t = np.clip(((pts[:, None, :] - p0[None]) * d[None]).sum(-1) / L2[None], 0, 1)
    proj = p0[None] + t[..., None] * d[None]
    return np.hypot(*(pts[:, None, :] - proj).transpose(2, 0, 1)).min(1)


def _poly_len(poly):
    a = np.asarray(poly, dtype=float)
    return float(np.hypot(*np.diff(a, axis=0).T).sum()) if len(a) > 1 else 0.0


def _dedupe(strokes, tol_px=2.6, max_ratio=1.35):
    """去掉细化留下的并行双线（短的那条完全贴在长的上面就丢掉）。"""
    tol = tol_px / RASTER_PX
    segs = [[tuple(p) for p in s] for s in strokes if len(s) >= 2]
    keep = [True] * len(segs)
    order = sorted(range(len(segs)), key=lambda i: -_poly_len(segs[i]))
    for oi, i in enumerate(order):
        if not keep[i]:
            continue
        for j in order[oi + 1:]:
            if not keep[j] or _poly_len(segs[i]) > max_ratio * _poly_len(segs[j]) + 1e-9:
                continue
            if float(_pt_seg_dist(np.asarray(segs[j], dtype=float), segs[i]).max()) <= tol:
                keep[j] = False
    return [s for s, k in zip(segs, keep) if k]


def _merge(strokes, tol=MERGE_TOL, max_turn_deg=MERGE_MAX_TURN):
    """端点足够近 + 折角不过分的碎段拼成一笔（减少无谓抬笔）。"""
    segs = [[tuple(p) for p in s] for s in strokes if len(s) >= 2]
    changed = True
    while changed and len(segs) > 1:
        changed = False
        best = None
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                for fa in (False, True):
                    A = list(reversed(segs[i])) if fa else segs[i]
                    for fb in (False, True):
                        B = list(reversed(segs[j])) if fb else segs[j]
                        d = math.hypot(A[-1][0] - B[0][0], A[-1][1] - B[0][1])
                        if d > tol or (best and d >= best[0]):
                            continue
                        ta = math.atan2(A[-1][1] - A[-2][1], A[-1][0] - A[-2][0])
                        tb = math.atan2(B[1][1] - B[0][1], B[1][0] - B[0][0])
                        if math.degrees(_angdiff(ta, tb)) > max_turn_deg:
                            continue
                        best = (d, i, j, fa, fb)
        if best:
            _, i, j, fa, fb = best
            A = list(reversed(segs[i])) if fa else segs[i]
            B = list(reversed(segs[j])) if fb else segs[j]
            segs = [s for k, s in enumerate(segs) if k not in (i, j)] + [A + B[1:]]
            changed = True
    return segs


def _stroke_key(poly):
    p = np.asarray(poly, dtype=float)
    s = p[np.argmin(p[:, 0])] if float(np.ptp(p[:, 0])) > float(np.ptp(p[:, 1])) else p[np.argmin(p[:, 1])]
    return (round(float(s[1]), 3), round(float(s[0]), 3))


class _Font:
    """本地字体（兜底路径用）。.ttf/.ttc 通吃，按全局基准框统一缩放。"""

    def __init__(self, path):
        from fontTools.ttLib import TTFont, TTCollection
        if path.lower().endswith((".ttc", ".otc")):
            self.fonts = TTCollection(path).fonts
        else:
            self.fonts = [TTFont(path)]
        self.path = path
        self.upem = self.fonts[0]["head"].unitsPerEm
        self._cmaps = [(f, f.getBestCmap()) for f in self.fonts]
        self._gs = {id(f): f.getGlyphSet() for f in self.fonts}
        self._frame_box = None

    def _glyph(self, ch):
        for f, cmap in self._cmaps:
            if ord(ch) in cmap:
                return self._gs[id(f)], cmap[ord(ch)]
        raise KeyError(f"字体没有字符 {ch!r}")

    def _raw(self, ch):
        from fontTools.pens.recordingPen import RecordingPen
        gs, gname = self._glyph(ch)
        pen = RecordingPen()
        gs[gname].draw(pen)
        return _flatten(pen.value)

    def _frame(self):
        """全局基准框：用"国"的外框当参考。

        ★ 一个字一个字地按各自外框缩放是错的："一"的外框只有一条线那么高，
        会把细横放大成巨粗条。所有字共用同一基准框，才保得住相对大小。"""
        if self._frame_box is None:
            for probe in ("国", "圖", "田", "口"):
                try:
                    polys = self._raw(probe)
                except KeyError:
                    continue
                if polys:
                    pts = np.vstack([np.asarray(p) for p in polys])
                    self._frame_box = (pts[:, 0].min(), pts[:, 1].min(),
                                       pts[:, 0].max(), pts[:, 1].max())
                    break
            if self._frame_box is None:
                self._frame_box = (0.0, -0.14 * self.upem, self.upem, 0.86 * self.upem)
        return self._frame_box

    def contours(self, ch, size_px):
        from fontTools.pens.recordingPen import RecordingPen
        from fontTools.pens.transformPen import TransformPen
        gs, gname = self._glyph(ch)
        x0, y0, x1, y1 = self._frame()
        bw, bh = max(x1 - x0, 1e-6), max(y1 - y0, 1e-6)
        s = (size_px * GLYPH_FILL) / max(bw, bh)
        tx = (size_px - bw * s) / 2.0 - x0 * s
        ty = (size_px - bh * s) / 2.0 + y1 * s
        pen = RecordingPen()
        gs[gname].draw(TransformPen(pen, (s, 0, 0, -s, tx, ty)))
        return _flatten(pen.value)


def skeleton_strokes(font: _Font, ch):
    """兜底：字体轮廓 → 骨架 → 归一化笔画。"""
    polys = font.contours(ch, RASTER_PX)
    if not polys:
        return []
    img = _rasterize(polys, RASTER_PX)
    if not img.any():
        return []
    out = []
    for poly in _trace(_thin(img)):
        if len(poly) < 4:
            continue
        a = np.asarray([(float(p[1]), float(p[0])) for p in poly])       # (row,col) → (x,y)
        if float(np.hypot(*np.diff(a, axis=0).T).sum()) / RASTER_PX < MIN_STROKE:
            continue
        s = simplify(a / RASTER_PX, eps=SIMPLIFY_EPS)
        if len(s) >= 2:
            out.append([tuple(p) for p in s])
    merged = []
    for poly in _dedupe(_merge(_dedupe(out))):
        a = np.asarray(poly)
        poly = simplify(a, eps=SIMPLIFY_EPS) if len(a) > 2 else a
        if len(poly) >= 2:
            merged.append([tuple(p) for p in poly])
    merged.sort(key=_stroke_key)
    return merged


# ============================================================================
# 对外接口
# ============================================================================
class StrokeLib:
    """笔画库：内存 + 磁盘缓存，数据集优先、字体骨架兜底。

    参数
        cache_file : 缓存 JSON 路径（默认 data/strokes_cache.json）
        font_path  : 兜底字体路径（默认自动搜索）
        offline    : True = 完全不联网，只读缓存 + 字体兜底（比赛现场用这个）
        verbose    : 打印取字过程
    """

    def __init__(self, cache_file=None, font_path=None, offline=False, verbose=True):
        self.cache_file = cache_file or CACHE_FILE
        self.font_path = font_path
        self.offline = offline
        self.verbose = verbose
        self.font = None
        self._cache = self._load_cache()
        self._miss = set()

    def _load_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, encoding="utf-8") as f:
                data = json.load(f)
            if data.get("version") == CACHE_VERSION:
                return dict(data.get("chars", {}))
            print(f"[笔画库] 缓存版本不符（{data.get('version')} != {CACHE_VERSION}），忽略并重建",
                  file=sys.stderr)
        return {}

    def save(self):
        os.makedirs(os.path.dirname(self.cache_file), exist_ok=True)
        with open(self.cache_file, "w", encoding="utf-8") as f:
            json.dump({"version": CACHE_VERSION, "glyph_fill": GLYPH_FILL,
                       "frame": [FRAME_X0, FRAME_X1, FRAME_TOP_UP, FRAME_BOT_UP],
                       "source": "hanzi-writer-data@2.0.1 + 字体骨架兜底",
                       "chars": self._cache}, f, ensure_ascii=False)

    def has(self, ch):
        return ch in self._cache

    @staticmethod
    def _clean(strokes):
        """统一成纯 float 元组（numpy 标量直接落盘/比较都不好看）。"""
        return [[(float(p[0]), float(p[1])) for p in s] for s in strokes]

    def strokes(self, ch, rebuild=False):
        """取一个字：[[(x,y),...], ...]，0~1 田字格坐标，y 向下，顺序 = 书写顺序。"""
        if ch in self._cache and not rebuild:
            return self._clean(self._cache[ch])

        st = None
        if not self.offline:
            st = fetch_char(ch)                      # 路径 1：数据集（笔顺最准）
        if not st:                                   # 路径 2：本地字体骨架
            if self.font is None and not getattr(self, "_font_failed", False):
                try:
                    self.font = _Font(self.font_path or find_font())
                except Exception as e:
                    self._font_failed = True
                    print(f"[笔画库] 字体骨架路径不可用（{e}），只能靠缓存/数据集里的字",
                          file=sys.stderr)
            try:
                st = skeleton_strokes(self.font, ch) if self.font else []
                if st and self.verbose:
                    print(f"[笔画库] {ch} 用字体骨架兜底（{len(st)} 笔，字形不如数据集准）",
                          file=sys.stderr)
            except Exception:
                st = []
        if not st:
            self._miss.add(ch)
            print(f"[笔画库] ★ 拿不到 {ch} 的笔画：离线且缓存没有它，本地字体也没有这个字",
                  file=sys.stderr)
            st = []
        self._cache[ch] = self._clean(st)
        self.save()
        return self._clean(self._cache[ch])

    def prefetch(self, text, retry=3, sleep=1.0):
        """把文本里所有字抓齐（比赛前跑一次，之后可离线）。返回缺字列表。"""
        todo = [c for c in dict.fromkeys(text) if c.strip() and not self.has(c)]
        for ch in todo:
            for k in range(retry):
                if self.strokes(ch):
                    break
                time.sleep(sleep * (k + 1))
        cache_now = self._load_cache()
        miss = [c for c in dict.fromkeys(text) if c.strip() and not cache_now.get(c)]
        return miss

    def stats(self, ch):
        st = self.strokes(ch)
        length = sum(_poly_len(s) for s in st)
        return {"strokes": len(st), "points": sum(len(s) for s in st), "ink_len": round(length, 3)}

    def text_stats(self, text):
        """整段文本的工时估算输入：总笔数、总笔长（格子数）。"""
        strokes = 0
        ink = 0.0
        miss = []
        for ch in text:
            if not ch.strip():
                continue
            if not self.has(ch):
                miss.append(ch)
            s = self.stats(ch)
            strokes += s["strokes"]
            ink += s["ink_len"]
        return {"chars": len([c for c in text if c.strip()]), "strokes": strokes,
                "ink_len": round(ink, 2), "missing": miss}


def main():
    args = list(sys.argv[1:])
    flags = {a for a in args if a.startswith("--")}
    rebuild = "--rebuild" in flags
    offline = "--offline" in flags
    args = [a for a in args if not a.startswith("--")]
    font = None
    if "--font" in sys.argv:
        i = sys.argv.index("--font")
        font = sys.argv[i + 1]
        if font in args:
            args.remove(font)
    text = "".join(args) or "一去二三里"

    lib = StrokeLib(font_path=font, offline=offline)
    print("缓存:", lib.cache_file)
    if "--fetch" in flags:
        miss = lib.prefetch(text)
        print(f"抓取完成，仍缺字：{miss or '无'}")
    for ch in text:
        if not ch.strip():
            continue
        s = lib.stats(ch)
        print(f"{ch}: {s['strokes']:>2} 笔 / {s['points']:>3} 点 / 累计笔长 {s['ink_len']:.2f} 格")
    print("合计:", lib.text_stats(text))


if __name__ == "__main__":
    main()
