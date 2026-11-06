#!/usr/bin/env python3
"""
make_figures.py  --  generates every figure used in the documentation.

    python tools/make_figures.py        # writes docs/fig_*.png at 200 dpi

Kept as code (not hand-drawn) so a design change regenerates consistent art.
"""
from __future__ import annotations

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Circle
import matplotlib.patheffects as pe

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "docs")

ACC = "#B71234"      # Half Cat crimson
INK = "#1b1f23"
GREY = "#5b6770"
LIGHT = "#eef1f4"
BLUE = "#1f6feb"
GREEN = "#1d9a4e"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 8,
    "axes.linewidth": 0,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
})


def box(ax, x, y, w, h, text, fc=LIGHT, ec=INK, fs=8, weight="normal", tc=INK, lw=1.0):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.02",
                                fc=fc, ec=ec, lw=lw, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            color=tc, weight=weight, zorder=3, linespacing=1.35)


def arrow(ax, p0, p1, text=None, style="-|>", ls="-", color=INK, lw=1.0, fs=7,
          toff=(0, 0.02), ha="center", rad=0.0):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=9,
                                 lw=lw, color=color, linestyle=ls, zorder=1,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((p0[0] + p1[0]) / 2 + toff[0], (p0[1] + p1[1]) / 2 + toff[1], text,
                fontsize=fs, color=color, ha=ha, va="bottom")


def newfig(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off")
    return fig, ax


# ------------------------------------------------------------ drawing helper
class Grid:
    """Tiny orthogonal drawing helper on a 0..100 x 0..100 canvas.

    Every box is registered so overlaps are caught at generation time instead
    of shipping a figure with text on top of text."""

    def __init__(self, w, h, title=None, sub=None):
        self.fig, self.ax = plt.subplots(figsize=(w, h))
        self.ax.set_xlim(0, 100); self.ax.set_ylim(0, 100); self.ax.axis("off")
        self.boxes = []
        if title:
            self.ax.text(50, 97.5, title, ha="center", fontsize=9.5, weight="bold")
        if sub:
            self.ax.text(1, 93, sub, fontsize=6.3, color=GREY, va="top")

    def box(self, x, y, w, h, text, fc=LIGHT, fs=6.8, weight="normal", ec=INK, lw=1.0, tc=INK):
        box = (x, y, x + w, y + h)
        for b in self.boxes:
            if not (box[2] <= b[0] or box[0] >= b[2] or box[3] <= b[1] or box[1] >= b[3]):
                raise AssertionError(f"overlap: {text!r} {box} vs {b}")
        self.boxes.append(box)
        self.ax.add_patch(FancyBboxPatch((x, y), w, h,
                                         boxstyle="round,pad=0,rounding_size=1.2",
                                         fc=fc, ec=ec, lw=lw, zorder=2))
        self.ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
                     weight=weight, color=tc, zorder=3, linespacing=1.4)

    def wire(self, pts, color=INK, lw=1.0, ls="-", z=1):
        self.ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, lw=lw, ls=ls,
                     zorder=z, solid_capstyle="round")

    def node(self, x, y):
        self.ax.add_patch(Circle((x, y), 0.65, color=INK, zorder=4))

    def lbl(self, x, y, t, fs=6.2, color=INK, ha="left", va="center", weight="normal", rot=0):
        self.ax.text(x, y, t, fontsize=fs, color=color, ha=ha, va=va, weight=weight,
                     rotation=rot, zorder=5, linespacing=1.35)

    def arrowhead(self, x, y, direction="right", color=INK):
        dx, dy = {"right": (2, 0), "left": (-2, 0), "up": (0, 2), "down": (0, -2)}[direction]
        self.ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                         arrowprops=dict(arrowstyle="-|>", lw=1.0, color=color), zorder=4)

    def gnd(self, x, y):
        self.wire([(x, y), (x, y - 2)])
        for i, w in enumerate((2.0, 1.3, 0.6)):
            self.ax.plot([x - w, x + w], [y - 2 - i * 1.0] * 2, color=INK, lw=1.1, zorder=2)

    def save(self, name):
        self.fig.savefig(os.path.join(OUT, name), facecolor="white")
        plt.close(self.fig)


# ---------------------------------------------------------------- fig 1: block
def fig_system_block():
    g = Grid(7.6, 4.0,
             "GSE control architecture  (replaces the FS-i6X / FS-iA10B radio pair)")
    for x, t in ((11, "OPERATOR"), (38, "GROUND SUPPORT BOX"), (86, "LAUNCH PAD")):
        g.lbl(x, 88, t, fs=7, color=GREY, weight="bold", ha="center")
    g.wire([(24, 12), (24, 84)], color="#c9d2d9", ls="--")
    g.wire([(66, 12), (66, 84)], color="#c9d2d9", ls="--")

    g.box(2, 64, 16, 18, "Laptop\nPython GUI\nUSB-C CDC", fc="#e8f0fe", fs=7)
    g.box(27, 64, 16, 18, "ESP32-S3\nTX node", fc="#fff4e5", fs=7)
    g.box(46, 64, 14, 18, "DX-LR41\n2.4 GHz", fc="#fdecef", fs=7)
    g.box(70, 64, 14, 18, "DX-LR41\n2.4 GHz", fc="#fdecef", fs=7)
    g.box(87, 64, 11, 18, "ESP32-S3\npad node", fc="#fff4e5", fs=7)

    for (x0, x1, t) in ((18, 27, "frames"), (43, 46, "UART"), (60, 70, "2.4 GHz LoRa"),
                        (84, 87, "UART")):
        g.wire([(x0, 73), (x1, 73)])
        g.lbl((x0 + x1) / 2, 75.5, t, fs=6.0, color=GREY, ha="center")

    # power chain
    g.box(2, 34, 13, 12, "12 V AGM\n160 CCA", fc="#f0f0f0")
    g.box(18, 34, 13, 12, "fuse +\npower switch", fc="#f0f0f0")
    g.box(34, 34, 12, 12, "12 → 5 V\nbuck", fc="#f0f0f0")
    g.wire([(15, 40), (18, 40)]); g.wire([(31, 40), (34, 40)])
    g.wire([(40, 46), (40, 64)], color=BLUE)
    g.lbl(41, 55, "5 V", color=BLUE)

    # output bus
    g.wire([(92, 64), (92, 30)]); g.wire([(54, 30), (97, 30)])
    for x in (55, 70, 85, 95):
        g.wire([(x, 30), (x, 26)])
    g.lbl(49, 33.5, "3x servo PWM 50 Hz  ·  igniter relay  ·  25 Hz hardware keep-alive",
          fs=6.0, color=GREY)

    for t, x, c, fs in (("FILL valve\nservo", 49, LIGHT, 6.4),
                        ("VEHICLE valves\nservo (Y)", 64, LIGHT, 6.4),
                        ("PURGE valve\nservo", 79, LIGHT, 6.4),
                        ("IGNITER\nrelay K1", 92, "#ffe3e3", 6.4)):
        g.box(x, 12, 11, 14, t, fc=c, fs=fs)

    g.lbl(1, 8, "19 B CMD at 20 Hz up-link  ·  31 B STATE at 10 Hz down-link  ·  watchdog 1.0 s → "
                "FILL/PURGE closed, VEHICLE open, IGN off.\n"
                "The pad node re-derives every output from the last frame plus its own clock, so a "
                "hung laptop or a dead link cannot hold a valve open.",
          fs=6.1, color=GREY, va="top")
    g.save("fig_system_block.png")


# ------------------------------------------------------------ fig 2: schematic
def fig_schematic():
    fig = plt.figure(figsize=(8.2, 5.6))
    fig.suptitle("Figure 6.1 (revised): GSE electrical schematic", fontsize=9.5,
                 weight="bold", y=0.985)
    ax1 = fig.add_axes([0.0, 0.42, 1.0, 0.50]); ax1.axis("off")
    ax1.set_xlim(0, 100); ax1.set_ylim(0, 100)
    ax2 = fig.add_axes([0.0, 0.02, 1.0, 0.38]); ax2.axis("off")
    ax2.set_xlim(0, 100); ax2.set_ylim(0, 100)

    g = Grid.__new__(Grid)
    g.ax, g.boxes = ax1, []
    ax1.text(1, 99, "A.  power distribution and node I/O", fontsize=7, weight="bold",
             color=GREY, va="top")

    # battery -> switch -> fuse -> rail
    g.box(2, 66, 16, 13, "12 V AGM\n(5-15 side terminals)", fc="#f0f0f0", fs=6.2)
    g.gnd(5, 66)
    g.wire([(18, 72.5), (23, 72.5)])
    g.box(23, 67, 11, 11, "SW1\npower", fc=LIGHT, fs=6.4)
    g.wire([(34, 72.5), (38, 72.5)])
    g.box(38, 68, 8, 9, "F1\n5 A", fc="#ffe3e3", fs=6.2)
    g.wire([(46, 72.5), (98, 72.5)])
    g.lbl(60, 75, "+12 V rail", fs=6.6, weight="bold")
    g.node(52, 72.5)
    # buck
    g.wire([(52, 72.5), (52, 60)])
    g.box(42, 47, 20, 13, "DROK 12 → 5 V, 3 A", fc=LIGHT, fs=6.4)
    g.wire([(52, 47), (52, 44)])
    # node
    g.box(30, 14, 46, 30, "", fc="#fff4e5", lw=1.4)
    g.lbl(53, 41, "ESP32-S3  pad node", fs=8, weight="bold", ha="center")
    g.lbl(53, 37.4, "5 V in   •   3.3 V logic   •   USB-C (service only)",
          fs=6.0, color=GREY, ha="center")
    g.lbl(32, 33.5, "5V", fs=6.0, color=BLUE); g.lbl(32, 31, "GND", fs=6.0, color=GREY)
    for i, nm in enumerate(["D4  FILL servo", "D5  VEHICLE servos", "D6  PURGE servo"]):
        g.lbl(32, 27.5 - i * 3.2, nm, fs=6.0, color=GREY)
    for i, nm in enumerate(["D7  ign coil", "D8  heartbeat", "D9  AUX"]):
        g.lbl(74, 27.5 - i * 3.2, nm, fs=6.0, color=GREY, ha="right")
    # servos
    for i, (t, y) in enumerate((("FILL valve\nservo", 31), ("VEHICLE valves\nservo (Y)", 21),
                                ("PURGE valve\nservo", 11))):
        g.box(81, y, 17, 8, t, fc=LIGHT, fs=6.2)
        g.wire([(76, y + 4), (81, y + 4)])
    # radio
    g.box(30, 2, 46, 8, "DX-LR41-24T12S   UART 9600 8N1, 2.4 GHz LoRa  →  laptop-side TX node",
        fc="#fdecef", fs=6.3)
    g.wire([(53, 14), (53, 10)])
    # ignition output leaves the node area through the interlock chain in B
    g.box(40, 84, 30, 8, "IGN outlet → igniter leads (via K1, panel B)", fc="#ffe3e3", fs=6.3)

    # ---------------- panel B: the enable ladder --------------------------
    g2 = Grid.__new__(Grid)
    g2.ax, g2.boxes = ax2, []
    ax2.text(1, 99, "B.  igniter enable chain  (every condition must be true)",
             fontsize=7, weight="bold", color=GREY, va="top")

    g2.box(2, 66, 12, 13, "+12 V\nrail", fc="#f0f0f0", fs=6.4)
    g2.wire([(14, 72.5), (19, 72.5)])
    g2.box(19, 66, 15, 13, "SW2\nARM key", fc="#e8f0fe", fs=6.4)
    g2.wire([(34, 72.5), (39, 72.5)])
    g2.box(39, 66, 16, 13, "K1 contact\nNO, 10 A", fc="#ffe3e3", fs=6.4)
    g2.wire([(55, 72.5), (61, 72.5)])
    g2.box(61, 66, 19, 13, "IGN outlet\n→ igniter leads", fc="#ffe3e3", fs=6.4)

    g2.node(47, 72.5)
    g2.wire([(47, 72.5), (47, 62)])
    g2.box(37, 52, 20, 10, "K1 coil  (12 V)", fc="#ffe3e3", fs=6.4)
    g2.wire([(47, 52), (47, 47)])
    g2.box(37, 37, 20, 10, "Q1  keep-alive", fc="#e8f0fe", fs=6.4)
    g2.wire([(47, 37), (47, 32)])
    g2.box(37, 22, 20, 10, "Q2  coil driver", fc="#ffe3e3", fs=6.4)
    g2.wire([(47, 22), (47, 18)])
    g2.gnd(47, 18)

    # gate inputs, drawn as labelled arrows rather than boxes
    ax2.annotate("", xy=(37, 42), xytext=(24, 42),
                 arrowprops=dict(arrowstyle="-|>", lw=1.0, ls="--", color=BLUE))
    ax2.text(23, 42, "D8  25 Hz heartbeat", fontsize=6.2, color=BLUE, ha="right",
             va="center")
    ax2.annotate("", xy=(37, 27), xytext=(24, 27),
                 arrowprops=dict(arrowstyle="-|>", lw=1.0, ls="--", color=ACC))
    ax2.text(23, 27, "D7  igniter command", fontsize=6.2, color=ACC, ha="right",
             va="center")
    ax2.text(62, 42, "coil can only conduct while\nthe firmware is still running",
             fontsize=6.0, color=BLUE, va="center")
    ax2.text(62, 27, "and only while the operator\nis commanding ignition",
             fontsize=6.0, color=ACC, va="center")
    ax2.text(1, 8, "Q1 and Q2 sit in series with the K1 coil; SW2 is a physical key in the 12 V "
                   "branch.  Opening any one of the three drops the relay,\n"
                   "whatever the firmware or the laptop is doing.  The contact side is the same "
                   "IGN outlet and wiring as the stock GSE.",
           fontsize=6.1, color=GREY, va="top")
    fig.savefig(os.path.join(OUT, "fig_gse_schematic.png"), facecolor="white")
    plt.close(fig)


# --------------------------------------------------------- fig 3: state machine
def fig_state_machine():
    fig, ax = newfig(6.8, 3.0)
    ax.text(0.5, 0.97, "Pad-node state machine", ha="center", fontsize=9.5, weight="bold")
    y = 0.55
    box(ax, 0.02, y, 0.17, 0.22, "BOOT\noutputs forced\nsafe", fc="#f0f0f0")
    box(ax, 0.26, y, 0.18, 0.22, "WAIT_HELLO\nradio configured,\nno CMD yet", fc="#e8f0fe")
    box(ax, 0.50, y, 0.18, 0.22, "RUN\nCMD <= watchdog,\noutputs follow CMD",
        fc="#e6f6ec")
    box(ax, 0.76, y, 0.22, 0.22, "LATCHED / COMMS LOSS\nFILL,PURGE closed\nVEHICLE open, IGN off",
        fc="#ffe3e3")
    arrow(ax, (0.19, y + 0.11), (0.26, y + 0.11), "self test ok", fs=6.4)
    arrow(ax, (0.44, y + 0.11), (0.50, y + 0.11), "first valid CMD\nor HELLO", fs=6.4)
    arrow(ax, (0.68, y + 0.15), (0.76, y + 0.15), "age > watchdog", fs=6.4, color=ACC)
    arrow(ax, (0.76, y + 0.06), (0.68, y + 0.06), "fresh CMD", fs=6.4, color=GREEN, rad=0.0)
    ax.text(0.5, 0.22, "Arming is orthogonal to the state: the igniter additionally requires\n"
                       "arm_code = 0xA5 in every CMD, a fresh link, SW2 (key) closed and the\n"
                       "hardware keep-alive running.  A 2 s task watchdog reboots a wedged loop\n"
                       "back to BOOT, which re-asserts the safe outputs before the radio is probed.",
            ha="center", fontsize=6.8, color=GREY, linespacing=1.5)
    fig.savefig(os.path.join(OUT, "fig_state_machine.png"), facecolor="white")
    plt.close(fig)


# ------------------------------------------------------------ fig 4: timing
def fig_timing():
    fig, ax = plt.subplots(figsize=(7.4, 3.1))
    ax.set_xlim(0, 3.2); ax.set_ylim(-0.2, 3.6)
    ax.axis("off")
    ax.set_title("Failsafe timing: command staleness -> safe outputs", fontsize=9.5,
                 weight="bold")

    def band(y, label, color=LIGHT):
        ax.add_patch(Rectangle((0, y - 0.12), 3.2, 0.26, fc=color, ec="none", zorder=0))
        ax.text(-0.06, y, label, ha="right", va="center", fontsize=7)

    band(3.0, "laptop CMD\n(20 Hz)")
    band(2.2, "pad receives")
    band(1.4, "link state")
    band(0.6, "FILL valve")
    band(-0.0, "IGNITER")

    import random
    random.seed(4)
    for i in range(38):
        t = 0.05 + i * 0.05
        if 0.85 < t < 1.95:            # blackout window
            continue
        ax.plot([t, t], [2.85, 3.15], color=BLUE, lw=2, solid_capstyle="butt")
        ax.plot([t + 0.004, t + 0.016], [2.05, 2.35], color=GREY, lw=2)

    ax.plot([0, 0.85], [1.4, 1.4], color=GREEN, lw=2.2)
    ax.plot([0.95, 1.95], [1.4, 1.4], color=ACC, lw=2.2, ls="--")
    ax.plot([1.95, 3.2], [1.4, 1.4], color=GREEN, lw=2.2)
    ax.text(0.5, 1.62, "OK", fontsize=7, color=GREEN)
    ax.text(1.30, 1.62, "stale (> watchdog)", fontsize=7, color=ACC)
    ax.text(2.45, 1.62, "OK", fontsize=7, color=GREEN)

    ax.plot([0, 0.75], [0.6, 0.6], color=ACC, lw=2.2)
    ax.plot([0.75, 1.95], [0.0, 0.0], color=ACC, lw=2.2)
    ax.plot([1.95, 3.2], [0.6, 0.6], color=ACC, lw=2.2)
    ax.vlines([0.75, 1.95], -0.12, 0.7, color=GREY, lw=0.8, ls=":")

    ax.plot([0.6, 0.9], [0.6, 0.6], color=PURPLE if False else "#7b2fbf", lw=4)
    ax.annotate("igniter on-time limit (0.5 s default)", xy=(0.75, 0.02),
                xytext=(1.0, -0.14), fontsize=6.6, color="#7b2fbf",
                arrowprops=dict(arrowstyle="->", lw=0.7, color="#7b2fbf"))

    ax.annotate("", xy=(0.95, 1.05), xytext=(1.95, 1.05),
                arrowprops=dict(arrowstyle="<|-|>", lw=0.8))
    ax.text(1.45, 1.10, "watchdog 1.0 s", ha="center", fontsize=6.8)

    ax.text(0.05, -0.14, "Outputs on loss: FILL closed, PURGE closed, VEHICLE open (dump), IGN off.\n"
                          "Recovery is automatic when a fresh CMD arrives; arming must be re-asserted.",
            fontsize=6.6, color=GREY, va="top")
    fig.savefig(os.path.join(OUT, "fig_failsafe_timing.png"), facecolor="white")
    plt.close(fig)


# --------------------------------------------------------- fig 5: airtime chart
def fig_airtime():
    levels = ["3\nSF8/BW406\n10 kb/s", "5\nSF6/BW806\n50 kb/s", "7\nSF5/BW1625\n150 kb/s",
              "8\nFLRC\n1040 kb/s"]
    airtime_ms = [27, 7, 3, 0.3]
    fig, ax = plt.subplots(figsize=(6.4, 2.5))
    bars = ax.bar(levels, airtime_ms, color=[ACC, "#d4576b", "#e08d9b", BLUE])
    ax.set_ylabel("time on air per 22 B frame (ms)")
    ax.set_title("DX-LR41 preset trade: airtime vs range (vendor figures)", fontsize=9,
                 weight="bold")
    for b, v in zip(bars, airtime_ms):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.7, f"{v:g} ms",
                ha="center", fontsize=7.5)
    ax.set_ylim(0, 32)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(labelsize=7)
    ax.axhline(50, color=GREY, ls=":", lw=0.8)
    ax.text(3.4, 51, "20 Hz stream budget (50 ms)", ha="right", fontsize=6.5, color=GREY)
    fig.savefig(os.path.join(OUT, "fig_airtime.png"), facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig_system_block()
    fig_schematic()
    fig_state_machine()
    fig_timing()
    fig_airtime()
    print("figures written to", os.path.abspath(OUT))
