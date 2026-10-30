"""Top-down breadboard diagram: 6-mic test array (4 front, 2 back), shoulders added later."""
import sys

P = 21
X0 = 120
NCOL = 63
ROWS_TOP, ROWS_BOT = "abcde", "fghij"
Y_A = 330


def hx(c):
    return X0 + (c - 1) * P


def hy(r):
    if r in ROWS_TOP:
        return Y_A + ROWS_TOP.index(r) * P
    return Y_A + 7 * P + ROWS_BOT.index(r) * P


def hole(s):
    return hx(int(s[1:])), hy(s[0])


W, H = 1600, 1250
out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
       'font-family="Helvetica, Arial, sans-serif">', f'<rect width="{W}" height="{H}" fill="#fff"/>',
       '<text x="40" y="50" font-size="30" font-weight="bold" fill="#111">6-mic test array on one breadboard '
       '(top view): 4 front + 2 back, shoulders later</text>',
       '<text x="40" y="82" font-size="17" fill="#555">Full-size breadboard (63 columns) · XIAO ESP32-S3 Sense '
       '· firmware eight_mic_stream · viewer tools/mic_test_ui.py · top of page = front of the wearer</text>']

bx0, bx1 = hx(1) - 36, hx(NCOL) + 36
by0, by1 = Y_A - 95, hy("j") + 95
out.append(f'<rect x="{bx0}" y="{by0}" width="{bx1 - bx0}" height="{by1 - by0}" rx="14" fill="#f3efe6" stroke="#c9c2b2" stroke-width="2"/>')
out.append(f'<rect x="{bx0 + 8}" y="{hy("e") + P * 0.6}" width="{bx1 - bx0 - 16}" height="{hy("f") - hy("e") - P * 1.2}" fill="#e2dccd"/>')
for ry in (Y_A - 70, hy("j") + 45):
    for k in range(2):
        yy = ry + k * P
        for c in range(1, NCOL + 1):
            out.append(f'<rect x="{hx(c) - 3}" y="{yy - 3}" width="6" height="6" fill="#c9c2b2"/>')
out.append(f'<text x="{bx1 - 14}" y="{by1 + 20}" font-size="13" fill="#888" text-anchor="end"></text>')
for c in range(1, NCOL + 1):
    for r in ROWS_TOP + ROWS_BOT:
        out.append(f'<rect x="{hx(c) - 3}" y="{hy(r) - 3}" width="6" height="6" fill="#9c9585"/>')
    if c == 1 or c % 5 == 0:
        out.append(f'<text x="{hx(c)}" y="{hy("e") + 22}" font-size="10" fill="#888" text-anchor="middle">{c}</text>')
for r in ROWS_TOP + ROWS_BOT:
    out.append(f'<text x="{hx(1) - 20}" y="{hy(r) + 4}" font-size="11" fill="#888" text-anchor="middle">{r}</text>')

# XIAO, USB-C left, top pins row c, bottom pins row g, columns 29-35
XC = 29
top_pins = ["5V", "GND", "3V3", "D10", "D9", "D8", "D7"]
bot_pins = ["D0", "D1", "D2", "D3", "D4", "D5", "D6"]
xl, xr = hx(XC) - 16, hx(XC + 6) + 16
yt, yb = hy("c") - 18, hy("g") + 18
out.append(f'<rect x="{xl - 26}" y="{(yt + yb) / 2 - 20}" width="30" height="40" rx="6" fill="#b8b8b8" stroke="#666" stroke-width="2"/>')
out.append(f'<rect x="{xl}" y="{yt}" width="{xr - xl}" height="{yb - yt}" rx="9" fill="#1f5d3a" stroke="#0e3320" stroke-width="2"/>')
out.append(f'<text x="{(xl + xr) / 2}" y="{(yt + yb) / 2 + 5}" font-size="12" fill="#cfe" text-anchor="middle">◀ USB-C   XIAO</text>')
for i, n in enumerate(top_pins):
    x, y = hx(XC + i), hy("c")
    out.append(f'<circle cx="{x}" cy="{y}" r="6" fill="#e8c35a" stroke="#8a6d1a"/>')
    out.append(f'<text x="{x}" y="{y + 20}" font-size="10" fill="#fff" text-anchor="middle" font-weight="bold">{n}</text>')
for i, n in enumerate(bot_pins):
    x, y = hx(XC + i), hy("g")
    out.append(f'<circle cx="{x}" cy="{y}" r="6" fill="#e8c35a" stroke="#8a6d1a"/>')
    out.append(f'<text x="{x}" y="{y - 12}" font-size="10" fill="#fff" text-anchor="middle" font-weight="bold">{n}</text>')

FRONT_ORDER = ["3V", "GND", "SEL", "CLK", "DAT"]


def mic(name, col, front, ch, sel, dashed=False):
    """Header along row a (front, board overhangs top edge) or row j (back, overhangs bottom edge)."""
    pins = FRONT_ORDER if front else FRONT_ORDER[::-1]
    x0, x1 = hx(col) - 15, hx(col + 4) + 15
    if front:
        y1 = hy("a") + 13
        y0 = y1 - 125
    else:
        y0 = hy("j") - 13
        y1 = y0 + 125
    style = 'fill="#2a2f7a" stroke="#141850" stroke-width="2"' if not dashed else \
        'fill="none" stroke="#7a7fc0" stroke-width="2" stroke-dasharray="7 5"'
    out.append(f'<rect x="{x0}" y="{y0}" width="{x1 - x0}" height="{y1 - y0}" rx="9" {style}/>')
    ink = "#fff" if not dashed else "#5a5fa0"
    cx = (x0 + x1) / 2
    port_y = y0 + 22 if front else y1 - 22
    if not dashed:
        out.append(f'<rect x="{cx - 12}" y="{port_y - 9}" width="24" height="18" rx="3" fill="#cfcfcf" stroke="#777"/>')
        out.append(f'<circle cx="{cx}" cy="{port_y}" r="3" fill="#333"/>')
    title_y = (y0 + 54) if front else (y1 - 44)
    out.append(f'<text x="{cx}" y="{title_y}" font-size="17" fill="{ink}" text-anchor="middle" font-weight="bold">{name}</text>')
    out.append(f'<text x="{cx}" y="{title_y + 17}" font-size="11" fill="{ink}" text-anchor="middle">ch{ch} · SEL {sel}</text>')
    row = "a" if front else "j"
    for i, pn in enumerate(pins):
        x, y = hx(col + i), hy(row)
        ty = y - 17 if front else y + 24
        out.append(f'<text x="{x}" y="{ty}" font-size="9" fill="{ink}" text-anchor="middle" font-weight="bold">{pn}</text>')
        if not dashed:
            out.append(f'<circle cx="{x}" cy="{y}" r="5.5" fill="#e8c35a" stroke="#8a6d1a"/>')
    arrow_y = y0 - 14 if front else y1 + 14
    out.append(f'<text x="{cx}" y="{arrow_y + (0 if front else 8)}" font-size="13" fill="#2a2f7a" text-anchor="middle">'
               f'{"▲ faces front" if front else "▼ faces back"}</text>')


# front (wearer's right is page right): F5 F4 F3 F2, 18 holes = 4.6 cm apart
FRONT = [3, 21, 39, 57]
FRONT_SEL = ["GND", "3V3", "3V3", "GND"]
BACK = [3, 57]
mic("F5", 3, True, 3, "GND")
mic("F4", 21, True, 2, "3V3")
mic("F3", 39, True, 0, "3V3")
mic("F2", 57, True, 1, "GND")
mic("B4", 3, False, 7, "GND")
mic("B1", 57, False, 5, "GND")
mic("S_L (later)", 14, False, 6, "3V3", dashed=True)
mic("S_R (later)", 45, False, 4, "3V3", dashed=True)

RED, BLK, CLK, SELC = "#d22", "#222", "#e0a800", "#3b6fd8"
L0, L1, L2, L3 = "#1a9e3a", "#0aa3a3", "#9b4dca", "#e8590c"
RAIL_Y = {"T+": Y_A - 70, "T-": Y_A - 49, "B-": hy("j") + 45, "B+": hy("j") + 66}


def pt(s):
    if s[:2] in RAIL_Y:
        return hx(int(s[2:])), RAIL_Y[s[:2]]
    return hole(s)


def wire(a, b, color, bow, w=4.5, low=None):
    """Quadratic arc bowed by `bow`, or with `low` a U that dips to that y between the ends."""
    (x1, y1), (x2, y2) = pt(a), pt(b)
    if low is None:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + bow
        d = f"M{x1},{y1} Q{mx},{my} {x2},{y2}"
    else:
        d = f"M{x1},{y1} C{x1},{low} {x2},{low} {x2},{y2}"
    out.append(f'<path d="{d}" fill="none" stroke="#00000030" stroke-width="{w + 3}" stroke-linecap="round"/>')
    out.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}" stroke-linecap="round"/>')
    for x, y in ((x1, y1), (x2, y2)):
        out.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="{color}" stroke="#000" stroke-width="0.8"/>')


# rail colour bands
for key, col in (("T+", RED), ("T-", "#33d"), ("B-", "#33d"), ("B+", RED)):
    out.append(f'<line x1="{bx0 + 24}" y1="{RAIL_Y[key]}" x2="{bx1 - 24}" y2="{RAIL_Y[key]}" stroke="{col}" stroke-width="1.5" opacity="0.35"/>')
    out.append(f'<text x="{bx0 + 12}" y="{RAIL_Y[key] + 5}" font-size="14" fill="{col}" font-weight="bold" text-anchor="middle">{"+" if "+" in key else "−"}</text>')

# XIAO onto the top rails, top rails bridged to the bottom rails
wire("a31", "T+31", RED, 0)
wire("a30", "T-30", BLK, 0)
wire("T+11", "B+11", RED, 0)
wire("T-12", "B-12", BLK, 0)
# each mic: two short jumpers to the rails beside its board
for m in FRONT:
    wire(f"b{m}", f"T+{m - 2}", RED, 0)
    wire(f"b{m + 1}", f"T-{m - 1}", BLK, 0)
for m in BACK:
    wire(f"i{m + 4}", f"B+{m + 6}", RED, 0)
    wire(f"i{m + 3}", f"B-{m + 5}", BLK, 0)
# SEL: short jumper to the GND strip next door, or two over to the 3V strip
for m, sel in zip(FRONT, FRONT_SEL):
    wire(f"e{m + 2}", f"e{m + 1}" if sel == "GND" else f"e{m}", SELC, -9, 4)
for m in BACK:
    wire(f"g{m + 2}", f"g{m + 3}", SELC, 9, 4)
F5, F4, F3, F2 = FRONT
B4, B1 = BACK
# clock: D0 to F4 and F3, chained out to F5 and F2, then down to B4 and B1
wire("h29", f"e{F4 + 3}", CLK, 0, low=hy("i") + 4)
wire("i29", f"e{F3 + 3}", CLK, 0, low=hy("j") + 10)
wire(f"b{F4 + 3}", f"b{F5 + 3}", CLK, 40)
wire(f"b{F3 + 3}", f"b{F2 + 3}", CLK, 40)
wire(f"d{F5 + 3}", f"f{B4 + 1}", CLK, 0)
wire(f"d{F2 + 3}", f"f{B1 + 1}", CLK, 0)
# data lines
wire("h30", f"e{F3 + 4}", L0, 0, low=hy("j") - 4)
wire(f"d{F3 + 4}", f"d{F2 + 4}", L0, 30)
wire("h32", f"e{F4 + 4}", L1, 0, low=hy("j") + 2)
wire(f"d{F4 + 4}", f"d{F5 + 4}", L1, 30)
wire("h33", f"h{B1}", L2, 0, low=hy("j") + 6)
wire("h34", f"h{B4}", L3, 0, low=hy("j") + 14)

# spacing marks
yy = Y_A - 205
for a, b in ((5, 23), (23, 41), (41, 59)):
    out.append(f'<line x1="{hx(a)}" y1="{yy}" x2="{hx(b)}" y2="{yy}" stroke="#888" stroke-width="1.2"/>')
    out.append(f'<text x="{(hx(a) + hx(b)) / 2}" y="{yy - 6}" font-size="12" fill="#666" text-anchor="middle">4.6 cm (18 holes)</text>')
    for x in (hx(a), hx(b)):
        out.append(f'<line x1="{x}" y1="{yy - 5}" x2="{x}" y2="{yy + 5}" stroke="#888" stroke-width="1.2"/>')
yb2 = hy("j") + 215
out.append(f'<line x1="{hx(5)}" y1="{yb2}" x2="{hx(59)}" y2="{yb2}" stroke="#888" stroke-width="1.2"/>')
out.append(f'<text x="{(hx(5) + hx(59)) / 2}" y="{yb2 + 16}" font-size="12" fill="#666" text-anchor="middle">B4 to B1: 13.7 cm (54 holes)</text>')
for x, anchor in ((hx(1) - 30, "end"), (hx(NCOL) + 30, "start")):
    side = "LEFT" if anchor == "end" else "RIGHT"
    out.append(f'<text x="{x}" y="{Y_A - 120}" font-size="13" fill="#666" text-anchor="{anchor}">wearer\'s</text>')
    out.append(f'<text x="{x}" y="{Y_A - 104}" font-size="13" fill="#666" text-anchor="{anchor}">{side}</text>')

# legend
lx, ly = 40, yb2 + 50
out.append(f'<text x="{lx}" y="{ly}" font-size="17" font-weight="bold" fill="#111">Wires (each column of 5 holes is one strip; any free hole in the strip works)</text>')
legend = [
    (RED, "3V3", "XIAO 3V3 → top + rail → bridged to bottom + rail; each mic's 3V → nearest + rail"),
    (BLK, "GND", "XIAO GND → top − rail → bridged to bottom − rail; each mic's GND → nearest − rail"),
    (CLK, "CLK", "XIAO D0 → F4, F3 → chained to F5, F2 → down to B4, B1 (all 6 mics)"),
    (L0, "D1", "data line 0: F3 (ch0) + F2 (ch1)"),
    (L1, "D3", "data line 1: F4 (ch2) + F5 (ch3)"),
    (L2, "D4", "data line 2: B1 (ch5), S_R joins later (ch4)"),
    (L3, "D5", "data line 3: B4 (ch7), S_L joins later (ch6)"),
    (SELC, "SEL", "short jumper: to the GND strip next door (ch odd) or 2 over to 3V (ch even)"),
]
for i, (col, tag, txt) in enumerate(legend):
    y = ly + 28 + i * 24
    out.append(f'<rect x="{lx}" y="{y - 11}" width="30" height="9" rx="4" fill="{col}"/>')
    out.append(f'<text x="{lx + 40}" y="{y - 2}" font-size="14" fill="#111" font-weight="bold">{tag}</text>')
    out.append(f'<text x="{lx + 85}" y="{y - 2}" font-size="14" fill="#333">{txt}</text>')

nx = 900
notes = [
    ("Before you power it", True),
    ("• Back mics are turned 180°, so their pin order is reversed:", False),
    ("  DAT CLK SEL GND 3V from left to right. Follow the printed labels.", False),
    ("• Each SEL goes to exactly one rail. Never join two SEL pins.", False),
    ("• Add one mic at a time and check it in mic_test_ui before the next.", False),
    ("", False),
    ("Adding the shoulders later (no other rewiring)", True),
    ("• S_R: 3V, GND, CLK like B1; DAT into B1's DAT strip (col 57);", False),
    ("  SEL to 3V3 → ch4.", False),
    ("• S_L: same, DAT into B4's DAT strip (col 3); SEL to 3V3 → ch6.", False),
    ("", False),
    ("Good to know", True),
    ("• Long breadboards often split the rails in the middle: bridge them.", False),
    ("• MEMS mics hear all around; on a bare breadboard, front vs back", False),
    ("  differ by spacing, not by facing. A card or foam wall between", False),
    ("  the rows mimics the body shadow of the shirt.", False),
    ("• Spacing is shorter than on the shirt (front 5 → 4.6 cm,", False),
    ("  back 30 → 13.7 cm). Measure port-to-port for the software.", False),
]
for i, (t, bold) in enumerate(notes):
    out.append(f'<text x="{nx}" y="{ly + i * 21}" font-size="14" fill="{"#111" if bold else "#444"}"'
               f'{" font-weight=\"bold\"" if bold else ""}>{t}</text>')

out.append("</svg>")
open(sys.argv[1], "w").write("\n".join(out))
