"""Draws one HUD frame with QPainter (works offscreen, no display needed)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QLinearGradient, QPainter, QPainterPath, QPen, QPolygonF

from hud_core import BLOCK_S, GRID, Analysis, Event, direction_name, wrap

W, H = 1920, 1080
CAM = QRectF(24, 86, 1120, 840)
TIMELINE = QRectF(24, 944, 1120, 112)
RADAR_BOX = QRectF(1170, 86, 726, 600)
METER_BOX = QRectF(1170, 702, 726, 150)
EVENT_BOX = QRectF(1170, 868, 726, 188)
RADAR_CY, RADAR_R = 86 + 292, 222
BEAM_HALF_WIDTH = 6.0  # drawn half-width of the beam marker, degrees

INK = QColor("#e6edf3")
DIM = QColor("#8b949e")
PANEL = QColor("#161b22")
EDGE = QColor("#30363d")
BEAM = QColor("#58a6ff")
ALERT = QColor("#ff7b72")
OK = QColor("#3fb950")
SIM_BADGE = QColor("#d29922")

_CMAP = np.array([[0, 0, 4], [40, 11, 84], [101, 21, 110], [159, 42, 99], [212, 72, 66],
                  [245, 125, 21], [250, 193, 39], [252, 255, 164]], float)


def heat(v: float) -> QColor:
    v = float(np.clip(v, 0, 1)) * (len(_CMAP) - 1)
    i = min(int(v), len(_CMAP) - 2)
    c = _CMAP[i] + (_CMAP[i + 1] - _CMAP[i]) * (v - i)
    return QColor(int(c[0]), int(c[1]), int(c[2]))


def font(size: float, bold: bool = False, mono: bool = False) -> QFont:
    f = QFont("Menlo" if mono else "Helvetica")
    f.setPixelSize(int(size))
    f.setBold(bold)
    return f


def text(p: QPainter, x, y, s, size=18, color=INK, bold=False, align="left", mono=False):
    p.setFont(font(size, bold, mono))
    p.setPen(color)
    fm = p.fontMetrics()
    w = fm.horizontalAdvance(s)
    dx = {"left": 0, "center": -w / 2, "right": -w}[align]
    p.drawText(QPointF(x + dx, y), s)
    return w


def panel(p: QPainter, r: QRectF, title: str | None = None):
    p.setPen(QPen(EDGE, 1.5))
    p.setBrush(PANEL)
    p.drawRoundedRect(r, 10, 10)
    if title:
        text(p, r.x() + 16, r.y() + 28, title, 17, DIM, True)


def sample_map(m: np.ndarray, az: float) -> float:
    pos = (wrap(az) + 180.0) / 2.5
    i = int(math.floor(pos)) % len(GRID)
    frac = pos - math.floor(pos)
    return float(m[i] * (1 - frac) + m[(i + 1) % len(GRID)] * frac)


@dataclass
class Camera:
    hfov: float = 66.0
    yaw: float = 0.0  # where the camera points, in array angles

    @property
    def f(self) -> float:
        return (CAM.width() / 2) / math.tan(math.radians(self.hfov / 2))

    def x_of(self, az: float) -> float | None:
        rel = wrap(az - self.yaw)
        if abs(rel) >= 89:
            return None
        return CAM.center().x() - self.f * math.tan(math.radians(rel))

    def az_of(self, x: float) -> float:
        return float(wrap(self.yaw + math.degrees(math.atan(-(x - CAM.center().x()) / self.f))))

    def visible(self, az: float) -> bool:
        return abs(wrap(az - self.yaw)) <= self.hfov / 2


@dataclass
class HudData:
    an: Analysis
    track: np.ndarray  # beam angle per block
    beam_db: np.ndarray  # beam output level per block
    events: list[Event]
    cam: Camera
    beam_label: str
    scene: dict | None = None  # ground truth for simulated recordings
    simulated: bool = False
    locks: list[tuple[float, float]] = field(default_factory=list)  # (time, angle) beam re-aims
    timeline_img: QImage | None = None
    disp: np.ndarray | None = None  # map for display: peak-hold so speech pauses don't flicker

    def __post_init__(self):
        decay = 0.7  # per 0.1 s frame
        m = self.an.srp
        self.disp = np.empty_like(m)
        self.disp[0] = m[0]
        for b in range(1, len(m)):
            self.disp[b] = np.maximum(m[b], decay * self.disp[b - 1])


def beam_locks(track: np.ndarray, min_jump: float = 10.0) -> list[tuple[float, float]]:
    locks = []
    for b in range(1, len(track)):
        if abs(wrap(track[b] - track[b - 1])) >= min_jump:
            locks.append((b * BLOCK_S, float(track[b])))
    return locks


# ------------------------------------------------------------------ camera view

def draw_synthetic_scene(p: QPainter, d: HudData, t: float):
    r = CAM
    g = QLinearGradient(0, r.top(), 0, r.bottom())
    g.setColorAt(0, QColor("#20262e"))
    g.setColorAt(0.5, QColor("#2b323b"))
    g.setColorAt(0.5001, QColor("#3a4048"))
    g.setColorAt(1, QColor("#22272e"))
    p.fillRect(r, g)
    cx, cy = r.center().x(), r.center().y()
    # server racks down both sides of the aisle, in perspective
    for side in (-1, 1):
        for k in range(6):
            near, far = 1.0 / (1 + k * 0.55), 1.0 / (1 + (k + 1) * 0.55)
            x0 = cx + side * (r.width() * 0.62) * near
            x1 = cx + side * (r.width() * 0.62) * far
            top0, top1 = cy - 330 * near, cy - 330 * far
            bot0, bot1 = cy + 300 * near, cy + 300 * far
            poly = QPolygonF([QPointF(x0, top0), QPointF(x1, top1), QPointF(x1, bot1), QPointF(x0, bot0)])
            p.setPen(QPen(QColor("#11151a"), 2))
            p.setBrush(QColor("#2d343d") if k % 2 == 0 else QColor("#283038"))
            p.drawPolygon(poly)
            for j in range(5):
                yy = top0 + (bot0 - top0) * (0.15 + 0.17 * j)
                xx = x0 + (x1 - x0) * 0.3
                p.setPen(Qt.NoPen)
                p.setBrush(QColor("#3fb950") if (j + k) % 3 else QColor("#58a6ff"))
                p.drawEllipse(QPointF(xx, yy), 3 * near + 1, 3 * near + 1)
    if not d.scene:
        return
    for s in d.scene["sources"]:
        if s["kind"] != "talker":
            continue
        x = d.cam.x_of(s["az"])
        if x is None:
            continue
        depth = s["dist"] * math.cos(math.radians(wrap(s["az"] - d.cam.yaw)))
        f = d.cam.f
        head_y = cy - f * (s["height"] - 1.3) / depth
        head_r = f * 0.1 / depth
        shoulder = f * 0.2 / depth
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#4d5866"))
        body = QPainterPath()
        body.moveTo(x - shoulder, r.bottom())
        body.lineTo(x - shoulder, head_y + head_r * 2.2)
        body.quadTo(x - shoulder, head_y + head_r * 1.4, x - shoulder * 0.5, head_y + head_r * 1.4)
        body.lineTo(x + shoulder * 0.5, head_y + head_r * 1.4)
        body.quadTo(x + shoulder, head_y + head_r * 1.4, x + shoulder, head_y + head_r * 2.2)
        body.lineTo(x + shoulder, r.bottom())
        body.closeSubpath()
        p.drawPath(body.intersected(_rect_path(r)))
        p.setBrush(QColor("#6e7a88"))
        p.drawEllipse(QPointF(x, head_y), head_r * 0.85, head_r)
        speaking = any(a <= t <= b for a, b in s["active"])
        tag = s["name"] + ("  (talking)" if speaking else "")
        text(p, x, head_y - head_r - 16, tag, 17, INK if speaking else DIM, speaking, "center")


def _rect_path(r: QRectF) -> QPainterPath:
    path = QPainterPath()
    path.addRect(r)
    return path


def draw_camera(p: QPainter, d: HudData, b: int, frame: QImage | None):
    t = b * BLOCK_S
    p.save()
    p.setClipRect(CAM)
    if frame is not None:
        p.drawImage(CAM, frame)
    else:
        draw_synthetic_scene(p, d, t)
    # sound heat across the view: brighter = more sound from that direction
    m = d.disp[b]
    top, bot = CAM.top() + 60, CAM.bottom() - 120
    step = 4
    x = CAM.left()
    p.setRenderHint(QPainter.Antialiasing, False)
    while x < CAM.right():
        v = sample_map(m, d.cam.az_of(x + step / 2))
        if v > 0.2:
            c = heat(0.45 + 0.55 * v)
            g = QLinearGradient(0, top, 0, bot)
            a = int(200 * ((v - 0.2) / 0.8) ** 1.2)
            for pos, k in ((0, 0), (0.25, 1), (0.6, 0.8), (1, 0)):
                cc = QColor(c)
                cc.setAlpha(int(a * k))
                g.setColorAt(pos, cc)
            p.fillRect(QRectF(x, top, step, bot - top), g)
        x += step
    p.setRenderHint(QPainter.Antialiasing, True)
    # compass tape: angle ticks and the same heat as a strip
    tape = QRectF(CAM.left(), CAM.top(), CAM.width(), 44)
    p.fillRect(tape, QColor(13, 17, 23, 200))
    x = CAM.left()
    while x < CAM.right():
        v = sample_map(m, d.cam.az_of(x + 2))
        p.fillRect(QRectF(x, tape.bottom() - 10, 4.5, 10), heat(v))
        x += 4
    for a in range(-90, 91, 10):
        tx_ = d.cam.x_of(a + 0.0)
        if tx_ is None or not (CAM.left() + 10 < tx_ < CAM.right() - 10):
            continue
        p.setPen(QPen(DIM, 1.5))
        p.drawLine(QPointF(tx_, tape.bottom() - 10), QPointF(tx_, tape.bottom() - (18 if a % 30 == 0 else 14)))
        if a % 30 == 0:
            text(p, tx_, tape.top() + 20, f"{a:+d}°" if a else "0°", 14, DIM, False, "center")
    # beam marker
    az = float(d.track[b])
    bx = d.cam.x_of(az)
    if bx is not None and d.cam.visible(az):
        xl = d.cam.x_of(az + BEAM_HALF_WIDTH) or CAM.left()
        xr = d.cam.x_of(az - BEAM_HALF_WIDTH) or CAM.right()
        band = QColor(BEAM)
        band.setAlpha(45)
        p.fillRect(QRectF(xl, CAM.top(), xr - xl, CAM.height()), band)
        p.setPen(QPen(BEAM, 3))
        y0, y1 = CAM.top() + 150, CAM.top() + 470
        for xx, sgn in ((xl, 1), (xr, -1)):
            p.drawLine(QPointF(xx, y0), QPointF(xx + sgn * 24, y0))
            p.drawLine(QPointF(xx, y0), QPointF(xx, y0 + 40))
            p.drawLine(QPointF(xx, y1), QPointF(xx + sgn * 24, y1))
            p.drawLine(QPointF(xx, y1), QPointF(xx, y1 - 40))
        tag = f"BEAM  {az:+.0f}°"
        bx = min(max(bx, CAM.left() + 80), CAM.right() - 80)
        p.setBrush(QColor(13, 17, 23, 210))
        p.setPen(QPen(BEAM, 1.5))
        p.drawRoundedRect(QRectF(bx - 70, CAM.top() + 54, 140, 32), 6, 6)
        text(p, bx, CAM.top() + 77, tag, 18, BEAM, True, "center")
    # sounds outside the view: arrows on the edges
    edge_marks = []
    if d.an.activity[b] > 0:
        for a, h in d.an.peaks(b, 0.8, max_peaks=3):
            if not d.cam.visible(a):
                edge_marks.append((a, DIM, f"{direction_name(a)} {a:+.0f}°", False))
    for e in d.events:
        if 0 <= t - e.t <= 3.0 and not d.cam.visible(e.az):
            edge_marks.append((e.az, ALERT, f"{e.label} · {direction_name(e.az)}", True))
    used = []
    for a, col, label, strong in sorted(edge_marks, key=lambda m: not m[3]):
        rel = wrap(a - d.cam.yaw)
        left = rel > 0
        y = CAM.top() + 140 + (abs(rel) - d.cam.hfov / 2) / (180 - d.cam.hfov / 2) * (CAM.height() - 300)
        if any(abs(y - uy) < 46 and ul == left for uy, ul in used):
            continue
        used.append((y, left))
        x = CAM.left() + 18 if left else CAM.right() - 18
        sgn = -1 if left else 1
        size = 22 if strong else 15
        p.setPen(Qt.NoPen)
        p.setBrush(col)
        p.drawPolygon(QPolygonF([QPointF(x, y), QPointF(x - sgn * size * 1.6, y - size),
                                 QPointF(x - sgn * size * 1.6, y + size)]))
        tx = x - sgn * (size * 1.6 + 10)
        w = len(label) * (10.5 if strong else 9)
        p.setBrush(QColor(13, 17, 23, 200))
        p.drawRoundedRect(QRectF(tx - (0 if left else w) - 6, y - 15, w + 12, 30), 6, 6)
        text(p, tx, y + 7, label, 19 if strong else 16, col, strong, "left" if left else "right")
    p.restore()
    p.setPen(QPen(EDGE, 2))
    p.setBrush(Qt.NoBrush)
    p.drawRoundedRect(CAM, 8, 8)
    src = "camera" if frame is not None else "simulated camera view"
    text(p, CAM.left() + 14, CAM.bottom() - 14, f"{src} · {d.cam.hfov:.0f}° wide", 15, DIM)


# ------------------------------------------------------------------ radar

def radar_point(cx, cy, r, az) -> QPointF:
    a = math.radians(az)
    return QPointF(cx - r * math.sin(a), cy - r * math.cos(a))


def draw_radar(p: QPainter, d: HudData, b: int):
    panel(p, RADAR_BOX, "All around the wearer (top view)")
    cx, cy, R = RADAR_BOX.center().x(), RADAR_CY, RADAR_R
    t = b * BLOCK_S
    # camera field of view
    fov = QPainterPath()
    fov.moveTo(cx, cy)
    for a in np.linspace(d.cam.yaw - d.cam.hfov / 2, d.cam.yaw + d.cam.hfov / 2, 30):
        fov.lineTo(radar_point(cx, cy, R, a))
    fov.closeSubpath()
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(255, 255, 255, 18))
    p.drawPath(fov)
    p.setBrush(Qt.NoBrush)
    for rr in (R / 3, 2 * R / 3, R):
        p.setPen(QPen(EDGE, 1))
        p.drawEllipse(QPointF(cx, cy), rr, rr)
    for a, lab in ((0, "front"), (90, "left"), (-90, "right"), (180, "behind")):
        q = radar_point(cx, cy, R + 22, a)
        text(p, q.x(), q.y() + 6, lab, 15, DIM, False, "center")
    # steady noise (fans) as a dashed outline
    steady = QPolygonF([radar_point(cx, cy, R * (0.15 + 0.6 * sample_map(d.an.steady, a)), a)
                        for a in np.arange(-180, 180, 5)])
    p.setPen(QPen(QColor("#8b949e"), 1.5, Qt.DashLine))
    p.drawPolygon(steady)
    # live map of new sounds
    m = d.disp[b]
    live = QPolygonF([radar_point(cx, cy, R * (0.1 + 0.85 * sample_map(m, a) ** 2), a)
                      for a in np.arange(-180, 180, 2.5)])
    fill = heat(0.75)
    fill.setAlpha(110)
    p.setPen(QPen(heat(0.85), 2))
    p.setBrush(fill)
    p.drawPolygon(live)
    # beam
    az = float(d.track[b])
    wedge = QPainterPath()
    wedge.moveTo(cx, cy)
    for a in np.linspace(az - BEAM_HALF_WIDTH, az + BEAM_HALF_WIDTH, 8):
        wedge.lineTo(radar_point(cx, cy, R, a))
    wedge.closeSubpath()
    bc = QColor(BEAM)
    bc.setAlpha(90)
    p.setPen(QPen(BEAM, 2))
    p.setBrush(bc)
    p.drawPath(wedge)
    # ground truth for simulated scenes
    if d.scene:
        for s in d.scene["sources"]:
            q = radar_point(cx, cy, R * min(1.0, 0.35 + s["dist"] / 6), s["az"])
            active = any(a <= t <= bb for a, bb in s["active"])
            col = INK if (active and s["kind"] != "noise") else DIM
            p.setPen(QPen(col, 1.5))
            p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(q.x() - 6, q.y() - 6, 12, 12))
            name = s["name"].replace("Rack fan", "fan")
            text(p, q.x() + 10, q.y() - 8, name, 13, col)
    # recent events
    for e in d.events:
        if 0 <= t - e.t <= 3.0:
            q = radar_point(cx, cy, R * 0.92, e.az)
            p.setPen(Qt.NoPen)
            p.setBrush(ALERT)
            p.drawEllipse(q, 9, 9)
    # wearer
    p.setPen(QPen(INK, 2))
    p.setBrush(QColor("#30363d"))
    p.drawEllipse(QPointF(cx, cy), 34, 20)
    p.drawLine(QPointF(cx, cy - 20), QPointF(cx, cy - 30))
    leg_y = RADAR_BOX.bottom() - 14
    x = RADAR_BOX.left() + 18
    for col, lab, dash in ((heat(0.85), "new sounds", False), (QColor("#8b949e"), "steady noise", True),
                           (BEAM, "beam", False), (ALERT, "event", False)):
        p.setPen(QPen(col, 3, Qt.DashLine if dash else Qt.SolidLine))
        p.drawLine(QPointF(x, leg_y - 6), QPointF(x + 22, leg_y - 6))
        x += 30 + text(p, x + 30, leg_y, lab, 14, DIM) + 18
    if d.scene:
        p.setPen(QPen(DIM, 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(x, leg_y - 12, 12, 12))
        text(p, x + 18, leg_y, "true position (sim)", 14, DIM)


# ------------------------------------------------------------------ meters, events, timeline

def draw_meters(p: QPainter, d: HudData, b: int):
    panel(p, METER_BOX, "Level")
    ref_name = d.an.geom.names[d.an.geom.ref]
    rows = [(f"one mic ({ref_name})", d.an.level_db[b], DIM), ("beam output", d.beam_db[b], BEAM)]
    for i, (lab, db, col) in enumerate(rows):
        y = METER_BOX.top() + 62 + i * 44
        text(p, METER_BOX.left() + 18, y + 6, lab, 16, INK)
        x0, w = METER_BOX.left() + 190, 400
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#21262d"))
        p.drawRoundedRect(QRectF(x0, y - 12, w, 22), 4, 4)
        frac = np.clip((db + 60) / 60, 0, 1)
        p.setBrush(col)
        p.drawRoundedRect(QRectF(x0, y - 12, w * frac, 22), 4, 4)
        text(p, x0 + w + 12, y + 6, f"{db:5.1f} dB", 16, INK, mono=True)
    lo, hi = max(0, b - 10), b + 1
    diff = np.mean(d.beam_db[lo:hi] - d.an.level_db[lo:hi])
    text(p, METER_BOX.right() - 18, METER_BOX.top() + 28, f"beam vs one mic: {diff:+.1f} dB", 16, BEAM, True,
         "right")


def draw_events(p: QPainter, d: HudData, b: int):
    panel(p, EVENT_BOX, "Event log")
    t = b * BLOCK_S
    rows = [(e.t, f"{e.label}", f"{direction_name(e.az)} {e.az:+.0f}°", ALERT) for e in d.events if e.t <= t]
    rows += [(lt, "beam re-aimed", f"{direction_name(la)} {la:+.0f}°", BEAM) for lt, la in d.locks if lt <= t]
    rows.sort(key=lambda r: r[0])
    for i, (et, lab, where, col) in enumerate(rows[-4:]):
        y = EVENT_BOX.top() + 62 + i * 31
        text(p, EVENT_BOX.left() + 18, y, f"{et:5.1f} s", 16, DIM, mono=True)
        text(p, EVENT_BOX.left() + 120, y, lab, 17, col, True)
        text(p, EVENT_BOX.left() + 360, y, where, 17, INK)
    if not rows:
        text(p, EVENT_BOX.left() + 18, EVENT_BOX.top() + 62, "nothing yet", 16, DIM)


def build_timeline(d: HudData) -> QImage:
    """Direction (up = behind-left, middle = front, down = behind-right) vs time."""
    m = d.an.srp  # (blocks, angles), GRID from -180 upward
    n_b, n_a = m.shape
    img = QImage(n_b, n_a, QImage.Format_RGB32)
    lut = [heat(v / 255 * 0.95).rgb() for v in range(256)]
    for b in range(n_b):
        for a in range(n_a):
            img.setPixel(b, n_a - 1 - a, lut[int(min(1.0, m[b, a]) ** 1.3 * 255)])
    return img


def draw_timeline(p: QPainter, d: HudData, b: int):
    r = TIMELINE
    if d.timeline_img is None:
        d.timeline_img = build_timeline(d)
    p.setPen(QPen(EDGE, 1.5))
    p.setBrush(PANEL)
    p.drawRoundedRect(r.adjusted(-2, -2, 2, 2), 6, 6)
    inner = QRectF(r.left() + 70, r.top() + 6, r.width() - 80, r.height() - 12)
    p.drawImage(inner, d.timeline_img)
    dur = d.an.duration

    def ty(az):
        return inner.top() + (180 - az) / 360 * inner.height()

    def tx(t):
        return inner.left() + t / dur * inner.width()

    p.setPen(QPen(QColor(255, 255, 255, 70), 1, Qt.DashLine))
    for a in (d.cam.yaw - d.cam.hfov / 2, d.cam.yaw + d.cam.hfov / 2):
        p.drawLine(QPointF(inner.left(), ty(a)), QPointF(inner.right(), ty(a)))
    path = QPainterPath()
    for i, a in enumerate(d.track):
        q = QPointF(tx(i * BLOCK_S), ty(a))
        path.moveTo(q) if i == 0 else path.lineTo(q)
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(QColor(13, 17, 23), 5))
    p.drawPath(path)
    p.setPen(QPen(QColor("#a5d6ff"), 2.5))
    p.drawPath(path)
    for e in d.events:
        p.setPen(Qt.NoPen)
        p.setBrush(ALERT)
        p.drawEllipse(QPointF(tx(e.t), ty(e.az)), 5, 5)
    p.setPen(QPen(INK, 2))
    p.drawLine(QPointF(tx(b * BLOCK_S), inner.top()), QPointF(tx(b * BLOCK_S), inner.bottom()))
    text(p, r.left() + 8, ty(150) + 5, "left", 13, DIM)
    text(p, r.left() + 8, ty(0) + 5, "front", 13, DIM)
    text(p, r.left() + 8, ty(-150) + 5, "right", 13, DIM)


def render_frame(d: HudData, b: int, frame: QImage | None = None) -> QImage:
    img = QImage(W, H, QImage.Format_RGB32)
    img.fill(QColor("#0d1117"))
    p = QPainter(img)
    p.setRenderHint(QPainter.Antialiasing)
    p.setRenderHint(QPainter.TextAntialiasing)
    text(p, 24, 52, "Mic-array HUD", 30, INK, True)
    x = 260
    if d.simulated:
        p.setPen(Qt.NoPen)
        p.setBrush(SIM_BADGE)
        p.drawRoundedRect(QRectF(x, 26, 230, 34), 8, 8)
        text(p, x + 115, 50, "SIMULATED RECORDING", 16, QColor("#0d1117"), True, "center")
        x += 250
    text(p, x, 50, d.beam_label, 18, DIM)
    text(p, W - 24, 52, f"t = {b * BLOCK_S:5.1f} s / {d.an.duration:.1f} s", 22, INK, True, "right", mono=True)
    draw_camera(p, d, b, frame)
    draw_timeline(p, d, b)
    draw_radar(p, d, b)
    draw_meters(p, d, b)
    draw_events(p, d, b)
    p.end()
    return img
