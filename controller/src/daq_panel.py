"""DAQ panel for the ground controller: live readouts, a live chart, the
telemetry-format toggle and the channel-ranges dialog.  Logic lives in daq.py."""
from __future__ import annotations

import json
import math
import time

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import daq as D

TRACE = ["#4da3ff", "#ff7b54", "#5fd38d", "#d9a6ff"]
LEVEL = {"ok": "#5fd38d", "warn": "#f1c40f", "bad": "#ff6b6b", "none": "#8a9299"}
GRID = "#1e2830"
DIM = "#8a9299"
NOW_CSS = "font: bold 15px monospace;"    # the app stylesheet would override setFont()
SETTINGS_KEY = "daq/settings"
FORMAT_KEY = "daq/format"


def load_settings(store) -> tuple[D.DaqSettings, str]:
    """(channel settings, format) from a QSettings store; defaults if absent."""
    try:
        s = D.DaqSettings.from_dict(json.loads(store.value(SETTINGS_KEY, "") or "null"))
    except (TypeError, ValueError):
        s = D.DaqSettings()
    fmt = str(store.value(FORMAT_KEY, "combined"))
    return s, fmt if fmt in D.FORMATS else "combined"


class ChartWidget(QWidget):
    """Pressure (top) and temperature (bottom) over the last HISTORY_S seconds."""

    def __init__(self, state: D.DaqState, parent=None):
        super().__init__(parent)
        self.state = state
        self.setMinimumSize(380, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#050708"))
        st = self.state
        now = st.clock()
        w, h = self.width(), self.height()
        split = int(h * 0.6)
        max_pts = max(50, (w - 60) // 2)             # at most one point per 2 pixel columns
        conv = D.c_to_f if st.settings.temp_unit == "F" else (lambda x: x)
        pres = [(t, [st.psi(i, u) for i, u in enumerate(v)])
                for t, v in self._window(st.p_hist, now, max_pts)]
        temps = [(t, [None if x is None else conv(x) for x in v])
                 for t, v in self._window(st.t_hist, now, max_pts)]
        self._plot(p, QRectF(0, 0, w, split), pres, now, "psi", floor0=True, min_span=50)
        self._plot(p, QRectF(0, split, w, h - split), temps, now,
                   f"°{st.settings.temp_unit}", floor0=False, min_span=10)
        p.end()

    @staticmethod
    def _window(hist, now, max_pts):
        """The samples inside the chart window, thinned to at most max_pts."""
        t_from = now - D.HISTORY_S
        recent = []
        for entry in reversed(hist):                 # newest first; stop at the window edge
            if entry[0] < t_from:
                break
            recent.append(entry)
        recent.reverse()
        stride = max(1, -(-len(recent) // max_pts))
        return recent[::stride] if stride > 1 else recent

    @staticmethod
    def _nice_step(span, n=4):
        raw = span / n
        mag = 10 ** math.floor(math.log10(raw))
        for m in (1, 2, 2.5, 5, 10):
            if raw <= m * mag:
                return m * mag
        return 10 * mag

    def _plot(self, p, area, series, now, unit, floor0, min_span):
        left, right, top, bottom = 52, 10, 8, 18
        x0, x1 = area.left() + left, area.right() - right
        y0, y1 = area.top() + top, area.bottom() - bottom
        t_from = now - D.HISTORY_S
        pts = [(t, v) for t, v in series if t >= t_from]
        p.setPen(QPen(QColor(GRID)))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(x0, y0, x1 - x0, y1 - y0))
        small = QFont("Monospace", 7)
        p.setFont(small)
        vals = [x for _, v in pts for x in v if x is not None]
        if not vals:
            p.setPen(QColor(DIM))
            p.drawText(QRectF(x0, y0, x1 - x0, y1 - y0), Qt.AlignCenter, f"no data ({unit})")
            return
        lo, hi = min(vals), max(vals)
        if floor0:
            lo = 0.0 if lo > -0.05 * max(hi, min_span) else lo
        if hi - lo < min_span:
            mid = (hi + lo) / 2
            lo, hi = (lo, lo + min_span) if floor0 else (mid - min_span / 2, mid + min_span / 2)
        step = self._nice_step(hi - lo)
        lo = step * math.floor(lo / step)
        hi = step * (math.floor(hi / step) + 1)

        def ys(v):
            return y1 - (v - lo) / (hi - lo) * (y1 - y0)

        def xs(t):
            return x0 + (t - t_from) / D.HISTORY_S * (x1 - x0)

        v = lo
        while v <= hi + 1e-9:
            y = ys(v)
            p.setPen(QPen(QColor(GRID)))
            p.drawLine(QPointF(x0, y), QPointF(x1, y))
            p.setPen(QColor(DIM))
            p.drawText(QRectF(area.left(), y - 7, left - 4, 14), Qt.AlignRight | Qt.AlignVCenter, f"{v:g}")
            v += step
        for s_ago, align in ((D.HISTORY_S, Qt.AlignLeft), (D.HISTORY_S / 2, Qt.AlignHCenter), (0, Qt.AlignRight)):
            x = xs(now - s_ago)
            rect = QRectF(x - (0 if align == Qt.AlignLeft else 60 if align == Qt.AlignRight else 30),
                          y1 + 2, 60, 14)
            p.drawText(rect, align | Qt.AlignTop, "now" if s_ago == 0 else f"-{s_ago:g} s")
        p.drawText(QRectF(area.left() + 4, y0, left, 14), Qt.AlignLeft | Qt.AlignTop, unit)
        n_ch = len(pts[0][1])
        sparse = len(pts) < 200                      # 2 Hz: mark each sample with a dot
        xscale = (x1 - x0) / D.HISTORY_S
        yscale = (y1 - y0) / (hi - lo)
        xpos = [x0 + (t - t_from) * xscale for t, _ in pts]
        for i in range(n_ch):
            pen = QPen(QColor(TRACE[i % 4]))
            pen.setWidthF(1.6)
            p.setPen(pen)
            p.setBrush(QColor(TRACE[i % 4]))
            seg: list = []
            for x, (_, vals_t) in zip(xpos, pts):
                val = vals_t[i]
                if val is None:                      # gap: no reading
                    if len(seg) > 1:
                        p.drawPolyline(QPolygonF(seg))
                    seg = []
                    continue
                seg.append(QPointF(x, y1 - (val - lo) * yscale))
            if len(seg) > 1:
                p.drawPolyline(QPolygonF(seg))
            if sparse:
                for t, vals_t in pts:
                    if vals_t[i] is not None:
                        p.drawEllipse(QPointF(xs(t), ys(vals_t[i])), 2, 2)
        p.setBrush(Qt.NoBrush)


class DaqSettingsDialog(QDialog):
    """Channel names, pressure ranges (psi at 4 mA / 20 mA) and temperature unit."""

    def __init__(self, current: D.DaqSettings, parent=None):
        super().__init__(parent)
        self.setWindowTitle("DAQ channels & ranges")
        self.result_settings: D.DaqSettings | None = None
        lay = QVBoxLayout(self)
        intro = QLabel("Pressure transducers (4-20 mA): set each range to the transducer "
                       "actually fitted - psi at 4 mA and at 20 mA.")
        intro.setWordWrap(True)
        lay.addWidget(intro)
        grid = QGridLayout()
        for col, text in enumerate(["", "name", "psi at 4 mA", "psi at 20 mA"]):
            grid.addWidget(QLabel(text), 0, col)
        self.p_rows = []
        for i, ch in enumerate(current.pressure):
            tag = QLabel(f"P{i}")
            tag.setStyleSheet(f"color: {TRACE[i]}; font-weight: bold;")
            name = QLineEdit(ch.name)
            lo, hi = QDoubleSpinBox(), QDoubleSpinBox()
            for box, val in ((lo, ch.min_psi), (hi, ch.max_psi)):
                box.setRange(-1000.0, 20000.0)
                box.setDecimals(1)
                box.setValue(val)
            for col, wdg in enumerate((tag, name, lo, hi)):
                grid.addWidget(wdg, 1 + i, col)
            self.p_rows.append((name, lo, hi))
        r = 1 + D.N_P
        grid.addWidget(QLabel("Thermocouples (type K)"), r, 0, 1, 4)
        self.t_rows = []
        for j, n in enumerate(current.tc_names):
            tag = QLabel(f"TC{j + 1}")
            tag.setStyleSheet(f"color: {TRACE[j]}; font-weight: bold;")
            name = QLineEdit(n)
            grid.addWidget(tag, r + 1 + j, 0)
            grid.addWidget(name, r + 1 + j, 1)
            self.t_rows.append(name)
        lay.addLayout(grid)
        unit_row = QHBoxLayout()
        unit_row.addWidget(QLabel("Show temperatures in"))
        self.unit = QComboBox()
        self.unit.addItems(["C", "F"])
        self.unit.setCurrentText(current.temp_unit)
        unit_row.addWidget(self.unit)
        unit_row.addStretch()
        lay.addLayout(unit_row)
        note = QLabel("Raw loop current (uA) is always logged, so a range change only changes "
                      "the psi shown and the psi columns from now on.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {DIM};")
        lay.addWidget(note)
        self.error = QLabel("")
        self.error.setStyleSheet(f"color: {LEVEL['bad']};")
        self.error.setWordWrap(True)
        lay.addWidget(self.error)
        buttons = QHBoxLayout()
        buttons.addStretch()
        defaults = QPushButton("Defaults")
        defaults.clicked.connect(self.set_defaults)
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        self.save_button = QPushButton("Save")
        self.save_button.setObjectName("go")
        self.save_button.clicked.connect(self.save)
        for b in (defaults, cancel, self.save_button):
            buttons.addWidget(b)
        lay.addLayout(buttons)

    def set_defaults(self) -> None:
        d = D.DaqSettings()
        for (name, lo, hi), ch in zip(self.p_rows, d.pressure):
            name.setText(ch.name)
            lo.setValue(ch.min_psi)
            hi.setValue(ch.max_psi)
        for name, n in zip(self.t_rows, d.tc_names):
            name.setText(n)
        self.unit.setCurrentText(d.temp_unit)

    def collect(self) -> D.DaqSettings:
        return D.DaqSettings(
            pressure=[D.PressureChannel(n.text().strip(), lo.value(), hi.value())
                      for n, lo, hi in self.p_rows],
            tc_names=[n.text().strip() for n in self.t_rows],
            temp_unit=self.unit.currentText())

    def save(self) -> bool:
        s = self.collect()
        errs = s.validate()
        if errs:
            self.error.setText("\n".join(errs))
            return False
        self.result_settings = s
        self.accept()
        return True


class DaqPanel(QGroupBox):
    def __init__(self, state: D.DaqState, store, log=None, on_format_changed=None, parent=None):
        super().__init__("DAQ  /  HOTFIRE-BOARD VIA VEHICLE TELEMETRY", parent)
        self.state = state
        self.store = store
        self.log = log or (lambda msg: None)
        self.on_format_changed = on_format_changed
        self._styles: dict[int, str] = {}
        self._chart_frames = -1               # frame count at the last chart redraw
        self._chart_at = 0.0
        lay = QVBoxLayout(self)

        head = QHBoxLayout()
        head.addWidget(QLabel("DAQ line format"))
        self.format_combo = QComboBox()
        for fmt in D.FORMATS:
            self.format_combo.addItem(D.FORMAT_LABELS[fmt], fmt)
        self.format_combo.setCurrentIndex(D.FORMATS.index(state.fmt))
        self.format_combo.setToolTip(
            "Must match what the DAQ firmware sends. The vehicle forwards only the newest "
            "UART line every 500 ms: with separate P / T lines, thermocouple lines are "
            "rarely the newest one.")
        self.format_combo.currentIndexChanged.connect(self._format_changed)
        head.addWidget(self.format_combo)
        head.addStretch()
        self.ranges_button = QPushButton("Channels & ranges...")
        self.ranges_button.clicked.connect(self.edit_settings)
        head.addWidget(self.ranges_button)
        clear = QPushButton("Clear peaks / chart")
        clear.clicked.connect(self.clear)
        head.addWidget(clear)
        lay.addLayout(head)

        self.link_label = QLabel("")
        self.link_label.setWordWrap(True)
        lay.addWidget(self.link_label)

        body = QVBoxLayout()                         # readouts on top, chart below
        table = QGridLayout()
        table.setHorizontalSpacing(12)
        for col, text in enumerate(["", "channel", "now", "loop", "peak", "range", "status"]):
            hdr = QLabel(text)
            hdr.setStyleSheet(f"color: {DIM}; font-weight: bold;")
            table.addWidget(hdr, 0, col)
        self.rows: list[dict] = []
        for i in range(D.N_P + D.N_T):
            r = i + 1 + (1 if i >= D.N_P else 0)
            cells = {}
            sw = QLabel("  ")
            sw.setStyleSheet(f"background: {TRACE[i % 4]};")
            sw.setFixedWidth(14)
            table.addWidget(sw, r, 0)
            for col, key in enumerate(("name", "now", "loop", "peak", "range", "status"), start=1):
                lab = QLabel("-")
                if key == "now":
                    lab.setMinimumWidth(110)
                if key == "status":
                    lab.setMinimumWidth(190)
                table.addWidget(lab, r, col)
                cells[key] = lab
            self.rows.append(cells)
        table.setRowMinimumHeight(D.N_P + 1, 10)
        table.setRowStretch(D.N_P + D.N_T + 2, 1)
        body.addLayout(table, 0)
        self.chart = ChartWidget(state)
        body.addWidget(self.chart, 1)
        lay.addLayout(body, 1)
        self.refresh()

    # ------------------------------------------------------------ actions
    def _format_changed(self, _index) -> None:
        fmt = self.format_combo.currentData()
        if fmt == self.state.fmt:
            return
        self.state.set_format(fmt)
        self.store.setValue(FORMAT_KEY, fmt)
        self.log(f"DAQ line format: {D.FORMAT_LABELS[fmt]}")
        if self.on_format_changed:
            self.on_format_changed(fmt)
        self.refresh()

    def edit_settings(self) -> None:
        dlg = DaqSettingsDialog(self.state.settings, self)
        if dlg.exec() == QDialog.Accepted and dlg.result_settings is not None:
            self.apply_settings(dlg.result_settings)

    def apply_settings(self, new: D.DaqSettings) -> list[str]:
        errs = new.validate()
        if errs:
            return errs
        old = self.state.settings
        for i, (a, b) in enumerate(zip(old.pressure, new.pressure)):
            if (a.name, a.min_psi, a.max_psi) != (b.name, b.min_psi, b.max_psi):
                self.log(f"DAQ P{i}: '{b.name}' {b.min_psi:g}-{b.max_psi:g} psi "
                         f"(was '{a.name}' {a.min_psi:g}-{a.max_psi:g})")
        for i, (a, b) in enumerate(zip(old.tc_names, new.tc_names)):
            if a != b:
                self.log(f"DAQ TC{i + 1}: '{b}' (was '{a}')")
        self.state.set_settings(new)
        self.store.setValue(SETTINGS_KEY, json.dumps(new.to_dict()))
        self.refresh()
        return []

    def clear(self) -> None:
        self.state.reset()
        self.log("DAQ peaks, counters and chart cleared (new CSV with the next frame)")
        self.refresh()

    # ------------------------------------------------------------ display
    def _temp(self, c):
        if c is None:
            return "-"
        unit = self.state.settings.temp_unit
        return f"{(D.c_to_f(c) if unit == 'F' else c):.1f} °{unit}"

    def _css(self, label, css: str) -> None:
        """Set a stylesheet only when it changes (re-styling every 100 ms is wasted work)."""
        if self._styles.get(id(label)) != css:
            self._styles[id(label)] = css
            label.setStyleSheet(css)

    def refresh(self) -> None:
        st = self.state
        text, level = st.link_text()
        if st.csv_path:
            text += f"  ·  CSV {st.csv_path.name}"
        self.link_label.setText(text)
        self._css(self.link_label, f"color: {LEVEL[level]};")
        lp, lt = st.latest_p, st.latest_t
        for i in range(D.N_P):
            row, ch = self.rows[i], st.settings.pressure[i]
            ua = lp.ua[i] if lp else None
            psi, peak = st.psi(i, ua), st.psi(i, st.p_peak[i])
            status, lvl = st.pressure_status(i)
            row["name"].setText(f"P{i} {ch.name}")
            row["now"].setText("-" if psi is None else f"{psi:.1f} psi")
            self._css(row["now"], NOW_CSS + (f"color: {LEVEL[lvl]};" if lvl in ("warn", "bad") else ""))
            row["loop"].setText("-" if ua is None else f"{ua / 1000:.2f} mA")
            row["peak"].setText("-" if peak is None else f"{peak:.1f}")
            row["range"].setText(f"{ch.min_psi:g}-{ch.max_psi:g} psi")
            row["status"].setText(status)
            self._css(row["status"], f"color: {LEVEL[lvl]};")
        for j in range(D.N_T):
            row = self.rows[D.N_P + j]
            c = lt.temp_c[j] if lt else None
            status, lvl = st.tc_status(j)
            row["name"].setText(f"TC{j + 1} {st.settings.tc_names[j]}")
            row["now"].setText(self._temp(c))
            self._css(row["now"], NOW_CSS + (f"color: {LEVEL['bad']};" if lvl == "bad" else ""))
            row["loop"].setText("type K")
            row["peak"].setText(self._temp(st.t_peak[j]))
            row["range"].setText("")
            row["status"].setText(status)
            self._css(row["status"], f"color: {LEVEL[lvl]};")
        # Redraw the chart when a frame arrives, else every 0.5 s to scroll the
        # time axis (data comes at 2 Hz; redrawing at the 10 Hz tick is wasted).
        frames = st.p.frames + st.t.frames
        now = time.monotonic()
        if frames != self._chart_frames or now - self._chart_at >= 0.5:
            self._chart_frames, self._chart_at = frames, now
            self.chart.update()
