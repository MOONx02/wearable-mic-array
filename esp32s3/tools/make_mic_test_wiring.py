"""Top-down breadboard diagram for the one-mic test (eight_mic_stream, DAT on D1)."""
import sys

P = 36                      # hole pitch, px
X0 = 150                    # x of column 1
NCOL = 30
ROWS_TOP = "abcde"
ROWS_BOT = "fghij"
Y_A = 250                   # y of row a
GAP = 2 * P                 # e -> f distance is 3 pitches (0.3")


def hx(col):
    return X0 + (col - 1) * P


def hy(row):
    if row in ROWS_TOP:
        return Y_A + ROWS_TOP.index(row) * P
    return Y_A + 4 * P + 3 * P + ROWS_BOT.index(row) * P


def hole(spec):
    return hx(int(spec[1:])), hy(spec[0])


out = []
W, H = 1300, 1030
out.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
           'font-family="Helvetica, Arial, sans-serif">')
out.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
out.append('<text x="40" y="52" font-size="30" font-weight="bold" fill="#111">'
           'One-mic test wiring (top view): XIAO ESP32-S3 Sense + 1 PDM mic</text>')
out.append('<text x="40" y="84" font-size="18" fill="#555">Firmware: eight_mic_stream · '
           'Viewer: tools/mic_test_ui.py · this wiring shows up as ch1 (DAT on D1, SEL on GND)</text>')

# breadboard body
bx0, bx1 = hx(1) - 40, hx(NCOL) + 40
by0, by1 = Y_A - 110, hy("j") + 110
out.append(f'<rect x="{bx0}" y="{by0}" width="{bx1 - bx0}" height="{by1 - by0}" rx="14" fill="#f3efe6" stroke="#c9c2b2" stroke-width="2"/>')
# centre gap
gy0, gy1 = hy("e") + P * 0.6, hy("f") - P * 0.6
out.append(f'<rect x="{bx0 + 10}" y="{gy0}" width="{bx1 - bx0 - 20}" height="{gy1 - gy0}" fill="#e2dccd"/>')
# power rails (unused)
for ry, label in ((Y_A - 80, "top"), (hy("j") + 55, "bottom")):
    for k, (col, sign) in enumerate((("#d33", "+"), ("#33d", "−"))):
        yy = ry + k * P
        for c in range(2, NCOL, 1):
            if (c - 2) % 6 == 5:
                continue
            out.append(f'<rect x="{hx(c) - 4}" y="{yy - 4}" width="8" height="8" fill="#bdb6a6"/>')
        out.append(f'<text x="{bx0 + 14}" y="{yy + 6}" font-size="16" fill="{col}" font-weight="bold">{sign}</text>')
out.append(f'<text x="{bx1 - 20}" y="{by1 + 20}" font-size="13" fill="#888" text-anchor="end">power rails: not used</text>')

# holes and labels
for c in range(1, NCOL + 1):
    for r in ROWS_TOP + ROWS_BOT:
        x, y = hx(c), hy(r)
        out.append(f'<rect x="{x - 4}" y="{y - 4}" width="8" height="8" fill="#9c9585"/>')
    if c in (1, 5, 10, 15, 20, 25, 30) or c in (3, 4, 5, 20, 21, 22, 23, 24):
        out.append(f'<text x="{hx(c)}" y="{hy("a") - 14}" font-size="11" fill="#777" text-anchor="middle">{c}</text>')
        out.append(f'<text x="{hx(c)}" y="{hy("j") + 24}" font-size="11" fill="#777" text-anchor="middle">{c}</text>')
for r in ROWS_TOP + ROWS_BOT:
    out.append(f'<text x="{hx(1) - 24}" y="{hy(r) + 4}" font-size="12" fill="#777" text-anchor="middle">{r}</text>')

# highlight the strips we use (each column of 5 holes is one connected strip)
used = {("top", 4): "#222", ("top", 5): "#d22", ("bot", 3): "#e0a800", ("bot", 4): "#1a9e3a",
        ("top", 20): "#d22", ("top", 21): "#222", ("top", 22): "#3b6fd8", ("top", 23): "#e0a800", ("top", 24): "#1a9e3a"}
for (half, c), col in used.items():
    r0, r1 = ("a", "e") if half == "top" else ("f", "j")
    out.append(f'<rect x="{hx(c) - 9}" y="{hy(r0) - 9}" width="18" height="{hy(r1) - hy(r0) + 18}" rx="6" '
               f'fill="{col}" opacity="0.13" stroke="{col}" stroke-opacity="0.5"/>')

# XIAO: USB-C left, pins top row c (5V GND 3V3 D10 D9 D8 D7), bottom row g (D0..D6)
top_pins = ["5V", "GND", "3V3", "D10", "D9", "D8", "D7"]
bot_pins = ["D0", "D1", "D2", "D3", "D4", "D5", "D6"]
xl, xr = hx(3) - 20, hx(9) + 20
yt, yb = hy("c") - 22, hy("g") + 22
out.append(f'<rect x="{xl - 34}" y="{(yt + yb) / 2 - 26}" width="40" height="52" rx="8" fill="#b8b8b8" stroke="#666" stroke-width="2"/>')
out.append(f'<text x="{xl - 40}" y="{(yt + yb) / 2 + 5}" font-size="13" fill="#444" text-anchor="end">USB-C</text>')
out.append(f'<rect x="{xl}" y="{yt}" width="{xr - xl}" height="{yb - yt}" rx="10" fill="#1f5d3a" stroke="#0e3320" stroke-width="2"/>')
out.append(f'<rect x="{xl + 40}" y="{(yt + yb) / 2 - 22}" width="70" height="44" rx="4" fill="#c8c8c8" stroke="#888"/>')
out.append(f'<text x="{xl + 75}" y="{(yt + yb) / 2 + 5}" font-size="12" fill="#333" text-anchor="middle">ESP32-S3</text>')
out.append(f'<text x="{xl + 130}" y="{(yt + yb) / 2 + 5}" font-size="13" fill="#cfe">XIAO, parts side up</text>')
for i, name in enumerate(top_pins):
    x, y = hx(3 + i), hy("c")
    out.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#e8c35a" stroke="#8a6d1a" stroke-width="1.5"/>')
    out.append(f'<text x="{x}" y="{y + 24}" font-size="12" fill="#fff" text-anchor="middle" font-weight="bold">{name}</text>')
for i, name in enumerate(bot_pins):
    x, y = hx(3 + i), hy("g")
    out.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#e8c35a" stroke="#8a6d1a" stroke-width="1.5"/>')
    out.append(f'<text x="{x}" y="{y - 14}" font-size="12" fill="#fff" text-anchor="middle" font-weight="bold">{name}</text>')

# mic breakout: header in row a cols 20-24, board overhangs the top edge
mic_pins = ["3V", "GND", "SEL", "CLK", "DAT"]
mx0, mx1 = hx(20) - 22, hx(24) + 22
my1 = hy("a") + 16
my0 = my1 - 150
out.append(f'<rect x="{mx0}" y="{my0}" width="{mx1 - mx0}" height="{my1 - my0}" rx="10" fill="#2a2f7a" stroke="#141850" stroke-width="2"/>')
out.append(f'<rect x="{(mx0 + mx1) / 2 - 16}" y="{my0 + 22}" width="32" height="24" rx="3" fill="#cfcfcf" stroke="#777"/>')
out.append(f'<circle cx="{(mx0 + mx1) / 2}" cy="{my0 + 34}" r="4" fill="#333"/>')
out.append(f'<text x="{(mx0 + mx1) / 2}" y="{my0 + 66}" font-size="12" fill="#dde" text-anchor="middle">PDM mic (MP34DT01)</text>')
for i, name in enumerate(mic_pins):
    x, y = hx(20 + i), hy("a")
    out.append(f'<text x="{x}" y="{y - 22}" font-size="12" fill="#fff" text-anchor="middle" font-weight="bold">{name}</text>')
    out.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#e8c35a" stroke="#8a6d1a" stroke-width="1.5"/>')


# jumpers: curved wires from hole to hole
def wire(a, b, color, bow, label=None, lx=None, ly=None):
    (x1, y1), (x2, y2) = hole(a), hole(b)
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + bow
    out.append(f'<path d="M{x1},{y1} Q{mx},{my} {x2},{y2}" fill="none" stroke="#00000033" stroke-width="9" stroke-linecap="round"/>')
    out.append(f'<path d="M{x1},{y1} Q{mx},{my} {x2},{y2}" fill="none" stroke="{color}" stroke-width="6" stroke-linecap="round"/>')
    for x, y in ((x1, y1), (x2, y2)):
        out.append(f'<circle cx="{x}" cy="{y}" r="6" fill="{color}" stroke="#000" stroke-width="1"/>')


wire("b5", "c20", "#d22", -40)
wire("a4", "d21", "#222", -110)
wire("e21", "e22", "#3b6fd8", -14)
wire("h3", "e23", "#e0a800", 170)
wire("h4", "e24", "#1a9e3a", 120)
out.append(f'<text x="{hx(25) + 4}" y="{hy("e") + 30}" font-size="13" fill="#3b6fd8" font-weight="bold">SEL → GND</text>')
out.append(f'<text x="{hx(25) + 4}" y="{hy("e") + 46}" font-size="12" fill="#3b6fd8">(short jumper e21–e22)</text>')

# legend / steps
lx, ly = 40, by1 + 50
rows = [
    ("#d22", "XIAO 3V3 (c5)", "mic 3V (col 20)", "power"),
    ("#222", "XIAO GND (c4)", "mic GND (col 21)", "ground"),
    ("#3b6fd8", "mic GND (col 21)", "mic SEL (col 22)", "SEL on GND = ch1 (to col 20 = ch0)"),
    ("#e0a800", "XIAO D0 (g3)", "mic CLK (col 23)", "clock, 2.048 MHz"),
    ("#1a9e3a", "XIAO D1 (g4)", "mic DAT (col 24)", "data line 0"),
]
out.append(f'<text x="{lx}" y="{ly}" font-size="18" font-weight="bold" fill="#111">Wires</text>')
for i, (col, a, b, note) in enumerate(rows):
    y = ly + 30 + i * 28
    out.append(f'<rect x="{lx}" y="{y - 13}" width="34" height="10" rx="4" fill="{col}"/>')
    out.append(f'<text x="{lx + 46}" y="{y - 3}" font-size="15" fill="#222">{a}  →  {b}</text>')
    out.append(f'<text x="{lx + 380}" y="{y - 3}" font-size="15" fill="#666">{note}</text>')

nx = 720
notes = [
    "Each numbered column of 5 holes (a–e, or f–j) is one",
    "connected strip; the gap in the middle splits them.",
    "• XIAO straddles the gap, so every pin gets its own strip.",
    "• Mic header runs along a row (a20–a24), never down a column.",
    "• Wire into any free hole of the right strip; the hole letter",
    "  doesn't matter, the column does.",
    "• Header pins must be soldered to the XIAO.",
    "• Follow the mic's printed labels if your board differs.",
    "Other data pins: DAT to D3 (g6) → ch2/ch3, D4 (g7) → ch4/ch5,",
    "D5 (g8) → ch6/ch7 (SEL 3V3 / SEL GND).",
]
for i, t in enumerate(notes):
    out.append(f'<text x="{nx}" y="{ly + i * 22}" font-size="14" fill="{"#111" if i < 2 else "#444"}"'
               f'{" font-weight=\"bold\"" if i < 2 else ""}>{t}</text>')

out.append("</svg>")
open(sys.argv[1], "w").write("\n".join(s for s in out if s))
