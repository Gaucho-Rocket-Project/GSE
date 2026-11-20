#!/usr/bin/env python3
"""Native Qt controller for the two-ESP32 ESP-NOW servo / ignition link.

The binary talks to the ground ESP32 over USB serial. The firmware owns the
firmware-side interlocks; this application adds the operator-facing TEST and
LIVE modes, countdowns and disabled (greyed-out) buttons.

Live-mode policy:
  * FILL timeout notifies but leaves FILL OPEN; operator closes early or
    extends the deadline.
  * VEH and IGN controls are disabled while FILL is open.
  * IGN ON is locked out until the arming countdown finishes.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

from PySide6.QtCore import QIODevice, Qt, QTimer, QTime
from PySide6.QtGui import QColor, QFont
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from telemetry import TelemetryLog, parse_telemetry
from live_state import ARMED, ARMING, FILL_TIME_UP, IGN_ARMED, OFF, ON, REJECT, LiveState

BAUD = 115200


def mmss(seconds: float | None) -> str:
    if seconds is None:
        return "--:--"
    total = max(0, int(math.ceil(seconds)))
    return f"{total // 60:02d}:{total % 60:02d}"


class MainWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.state = LiveState()
        self.mode = "TEST"
        self.rx_buffer = ""
        # Frozen builds write to the launch working directory; source builds to
        # the repository root. Override for packaged deployments as needed.
        default_root = Path.cwd() if getattr(sys, "frozen", False) else Path(__file__).resolve().parents[2]
        self.telemetry_log = TelemetryLog(Path(os.environ.get("GSE_DATA_DIR", default_root / "data")) / "telemetry.csv")

        self.serial = QSerialPort(self)
        self.serial.setBaudRate(BAUD)
        self.serial.readyRead.connect(self.on_ready_read)
        self.serial.errorOccurred.connect(self.on_serial_error)

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start()

        self.build_ui()
        self.refresh_ports()
        self.update_view()

    # ------------------------------------------------------------------- UI
    def build_ui(self) -> None:
        self.setWindowTitle("Rocket Ground Controller - ESP-NOW")
        self.setMinimumWidth(720)
        self.setStyleSheet(
            """
            QWidget { background: #080b0e; color: #d8dce0; font-family: monospace; font-size: 12px; }
            QGroupBox { border: 1px solid #41474d; margin-top: 15px; padding-top: 8px; color: #ff9b26; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 5px; }
            QLabel#big { font: bold 26px monospace; color: #ffae36; }
            QPushButton { padding: 7px 8px; border: 1px solid #626a71; background: #20262c; color: #e6e9eb; }
            QPushButton:hover { border-color: #ff9b26; background: #303840; }
            QPushButton:disabled { color: #62666a; background: #111417; border-color: #292d30; }
            QPushButton#go { background: #153c2e; border-color: #378a60; }
            QPushButton#warn { background: #4b3215; border-color: #b87926; }
            QPushButton#danger { background: #59211f; border-color: #b74139; }
            QComboBox, QSpinBox { background: #151a1e; border: 1px solid #525b62; padding: 4px; color: #ffaf36; }
            QPlainTextEdit { background: #050708; border: 1px solid #41474d; color: #d4dadc; font: 12px monospace; }
            """
        )

        root = QVBoxLayout(self)

        header = QLabel("GSE  /  GROUND CONTROL                                      ESP-NOW   •   OPERATIONS")
        header.setStyleSheet("background: #da7819; color: #080b0e; font-weight: bold; padding: 7px;")
        root.addWidget(header)

        # Top: mode + serial link.
        top = QHBoxLayout()
        top.addWidget(QLabel("Mode"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["TEST", "LIVE"])
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        top.addWidget(self.mode_combo)

        top.addWidget(QLabel("Port"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        top.addWidget(self.port_combo)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        top.addWidget(self.refresh_button)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_serial)
        top.addWidget(self.connect_button)

        self.status_label = QLabel("Disconnected")
        self.status_label.setMinimumWidth(220)
        top.addWidget(self.status_label)
        root.addLayout(top)

        # Countdowns.
        timers = QGroupBox("Countdowns")
        tl = QHBoxLayout(timers)
        self.fill_time_label = QLabel("--:--")
        self.fill_time_label.setObjectName("big")
        self.fill_note_label = QLabel("FILL (servo0)")
        self.ign_time_label = QLabel("OFF")
        self.ign_time_label.setObjectName("big")
        self.ign_note_label = QLabel("IGN (GPIO32)")
        tl.addWidget(self.fill_note_label)
        tl.addWidget(self.fill_time_label)
        tl.addStretch()
        tl.addWidget(self.ign_note_label)
        tl.addWidget(self.ign_time_label)
        root.addWidget(timers)

        # Configurable durations.
        cfg = QHBoxLayout()
        cfg.addWidget(QLabel("FILL countdown (s)"))
        self.fill_seconds = QSpinBox()
        self.fill_seconds.setRange(1, 3600)
        self.fill_seconds.setValue(60)
        self.fill_seconds.valueChanged.connect(self.on_durations_changed)
        cfg.addWidget(self.fill_seconds)

        cfg.addWidget(QLabel("FILL extend (s)"))
        self.extend_seconds = QSpinBox()
        self.extend_seconds.setRange(1, 600)
        self.extend_seconds.setValue(10)
        self.extend_seconds.valueChanged.connect(self.on_durations_changed)
        cfg.addWidget(self.extend_seconds)

        cfg.addWidget(QLabel("IGN countdown (s)"))
        self.ign_seconds = QSpinBox()
        self.ign_seconds.setRange(1, 60)
        self.ign_seconds.setValue(5)
        self.ign_seconds.valueChanged.connect(self.on_durations_changed)
        cfg.addWidget(self.ign_seconds)
        cfg.addStretch()
        root.addLayout(cfg)
        targets = QHBoxLayout()
        targets.addWidget(QLabel("SERVO TARGETS  /  µs"))
        self.fill_target = QSpinBox()
        self.fill_target.setRange(500, 2500)
        self.fill_target.setValue(1860)
        targets.addWidget(QLabel("FILL"))
        targets.addWidget(self.fill_target)
        self.veh_target = QSpinBox()
        self.veh_target.setRange(500, 2500)
        self.veh_target.setValue(1860)
        targets.addWidget(QLabel("VEH"))
        targets.addWidget(self.veh_target)
        targets.addWidget(QLabel("Applied on next actuation"))
        targets.addStretch()
        root.addLayout(targets)

        # Output controls.
        outputs = QGroupBox("Outputs")
        grid = QGridLayout(outputs)

        self.btn_fill_on = QPushButton("FILL OPEN")
        self.btn_fill_on.setObjectName("go")
        self.btn_fill_on.clicked.connect(self.fill_on_clicked)
        self.btn_fill_close = QPushButton("FILL CLOSE")
        self.btn_fill_close.setObjectName("warn")
        self.btn_fill_close.clicked.connect(self.fill_close_clicked)
        self.btn_fill_extend = QPushButton("EXTEND")
        self.btn_fill_extend.setObjectName("warn")
        self.btn_fill_extend.clicked.connect(self.fill_extend_clicked)

        self.btn_veh_on = QPushButton("VEH OPEN")
        self.btn_veh_on.setObjectName("go")
        self.btn_veh_on.clicked.connect(self.veh_on_clicked)
        self.btn_veh_off = QPushButton("VEH CLOSE")
        self.btn_veh_off.setObjectName("warn")
        self.btn_veh_off.clicked.connect(self.veh_off_clicked)

        self.btn_both_on = QPushButton("BOTH OPEN")
        self.btn_both_on.setObjectName("go")
        self.btn_both_on.setToolTip("Live mode uses individual controls: FILL must be closed for VEH")
        self.btn_both_on.clicked.connect(self.both_on_clicked)
        self.btn_both_pulse = QPushButton("BOTH PULSE")
        self.btn_both_pulse.clicked.connect(self.both_pulse_clicked)
        self.btn_both_off = QPushButton("BOTH CLOSE")
        self.btn_both_off.setObjectName("warn")
        self.btn_both_off.clicked.connect(self.both_off_clicked)

        self.btn_ign_arm = QPushButton("IGN ARM")
        self.btn_ign_arm.setObjectName("warn")
        self.btn_ign_arm.clicked.connect(self.ign_arm_clicked)
        self.btn_ign_on = QPushButton("IGN ON")
        self.btn_ign_on.setObjectName("danger")
        self.btn_ign_on.clicked.connect(self.ign_on_clicked)
        self.btn_ign_pulse = QPushButton("IGN PULSE (TEST)")
        self.btn_ign_pulse.setObjectName("danger")
        self.btn_ign_pulse.clicked.connect(lambda: self.send("4"))
        self.btn_ign_off = QPushButton("IGN OFF / RESET")
        self.btn_ign_off.clicked.connect(self.ign_off_clicked)

        self.btn_all_off = QPushButton("ALL OFF")
        self.btn_all_off.setObjectName("danger")
        self.btn_all_off.clicked.connect(self.all_off_clicked)

        grid.addWidget(QLabel("FILL  servo0 / GPIO18"), 0, 0)
        grid.addWidget(self.btn_fill_on, 1, 0)
        grid.addWidget(self.btn_fill_close, 1, 1)
        grid.addWidget(self.btn_fill_extend, 1, 2)

        grid.addWidget(QLabel("VEH  servo1 / GPIO5"), 2, 0)
        grid.addWidget(self.btn_veh_on, 3, 0)
        grid.addWidget(self.btn_veh_off, 3, 1)

        self.both_label = QLabel("BOTH servos  /  TEST only")
        grid.addWidget(self.both_label, 4, 0)
        grid.addWidget(self.btn_both_on, 5, 0)
        grid.addWidget(self.btn_both_pulse, 5, 1)
        grid.addWidget(self.btn_both_off, 5, 2)

        grid.addWidget(QLabel("IGN  GPIO32 active-high"), 6, 0)
        grid.addWidget(self.btn_ign_pulse, 7, 0)
        grid.addWidget(self.btn_ign_arm, 7, 0)
        grid.addWidget(self.btn_ign_on, 7, 1)
        grid.addWidget(self.btn_ign_off, 7, 2)

        grid.addWidget(self.btn_all_off, 8, 0, 1, 3)
        root.addWidget(outputs)

        self.telemetry_toggle = QPushButton("TELEMETRY  ▾  /  SHOW")
        self.telemetry_toggle.setCheckable(True)
        self.telemetry_toggle.toggled.connect(self.toggle_telemetry)
        root.addWidget(self.telemetry_toggle)
        self.telemetry_panel = QGroupBox("VEHICLE TELEMETRY  /  LIVE")
        telemetry_layout = QVBoxLayout(self.telemetry_panel)
        self.telemetry_info = QLabel(f"Waiting for data  •  CSV: {self.telemetry_log.path}")
        self.telemetry_info.setWordWrap(True)
        telemetry_layout.addWidget(self.telemetry_info)
        self.telemetry_view = QPlainTextEdit()
        self.telemetry_view.setReadOnly(True)
        self.telemetry_view.setMaximumHeight(160)
        self.telemetry_view.document().setMaximumBlockCount(200)
        telemetry_layout.addWidget(self.telemetry_view)
        self.telemetry_panel.hide()
        root.addWidget(self.telemetry_panel)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(170)
        root.addWidget(QLabel("Link log"))
        root.addWidget(self.log_view)

        self.log("TEST mode selected. Connect to the ground ESP32.")

    def toggle_telemetry(self, visible: bool) -> None:
        self.telemetry_panel.setVisible(visible)
        self.telemetry_toggle.setText("TELEMETRY  ▴  /  HIDE" if visible else "TELEMETRY  ▾  /  SHOW")

    # --------------------------------------------------------------- serial
    def refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for info in QSerialPortInfo.availablePorts():
            self.port_combo.addItem(f"{info.portName()} - {info.description()}", info.systemLocation())
        if current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def toggle_serial(self) -> None:
        if self.serial.isOpen():
            self.serial.close()
            self.log("Serial port closed")
            self.flash_status("Disconnected", "#95a5a6")
        else:
            port = self.port_combo.currentData()
            if not port:
                self.log("No serial port selected - click Refresh")
                return
            self.serial.setPortName(port)
            if not self.serial.open(QIODevice.ReadWrite):
                self.log(f"Could not open {port}: {self.serial.errorString()}")
                return
            self.log(f"Connected {port} @ {BAUD}")
            self.flash_status("Connected", "#2ecc71")
            self.send("PING")
        self.update_view()

    def send(self, command: str) -> bool:
        if not self.serial.isOpen():
            self.log(f"NO LINK - {command} NOT SENT")
            self.flash_status("NO LINK - command not sent", "#e67e22")
            return False
        if command in ("1", "2", "3", "SV0_ON", "SV1_ON", "IGN_ON"):
            command = f"{command}:{self.fill_target.value()},{self.veh_target.value()}"
        self.serial.write((command + "\n").encode("ascii"))
        self.log(f"TX  {command}")
        return True

    def on_ready_read(self) -> None:
        self.rx_buffer += bytes(self.serial.readAll()).decode("utf-8", "replace")
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            self.parse_line(line.rstrip("\r"))

    def parse_line(self, line: str) -> None:
        if not line:
            return
        if line.startswith("TLM "):
            sample = parse_telemetry(line)
            if sample is None:
                self.log("Invalid telemetry record discarded")
                return
            try:
                timestamp = self.telemetry_log.append(sample)
            except OSError as exc:
                self.log(f"Telemetry CSV write failed: {exc}")
                self.flash_status("TELEMETRY CSV ERROR", "#e74c3c")
                timestamp = "NOT SAVED"
            self.telemetry_info.setText(f"Latest #{sample[0]}  |  vehicle uptime {sample[1]} ms  |  {timestamp}  |  CSV: {self.telemetry_log.path}")
            self.telemetry_view.appendPlainText(f"{timestamp}  #{sample[0]}  {sample[2]}")
        elif line.startswith("ACK "):
            parts = line.split()
            if len(parts) >= 4 and parts[3] == "0":
                self.log(f"REJECTED by actuator: cmd {parts[2]} (interlock/countdown)")
                self.flash_status("ACTUATOR REJECTED COMMAND", "#e74c3c")
            else:
                self.log(f"ACK cmd {parts[2] if len(parts) > 2 else '?'} accepted")
        elif line == "PONG":
            self.log("Link OK (PONG)")
        elif line.startswith("SENT"):
            pass  # local echo from ground
        elif line.startswith("ERR"):
            self.log(line)
            self.flash_status("Ground error", "#e74c3c")
        else:
            self.log(f"RX  {line}")

    def on_serial_error(self, error) -> None:
        if self.serial.isOpen():
            self.log(f"Serial error: {self.serial.errorString()}")

    # ------------------------------------------------------------- logging
    def log(self, message: str) -> None:
        self.log_view.appendPlainText(f"{QTime.currentTime().toString('HH:mm:ss')}  {message}")

    def flash_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    # --------------------------------------------------------------- events
    def process_events(self, events) -> None:
        for code, message in events:
            self.log(message)
            if code in (FILL_TIME_UP, IGN_ARMED):
                QApplication.beep()
                self.flash_status(message, "#f1c40f")
            elif code == REJECT:
                self.flash_status("INTERLOCK", "#e74c3c")
            else:
                self.flash_status(message.replace(" (latched - reset with IGN OFF)", ""), "#2ecc71")

    def flush_state_events(self) -> None:
        self.process_events(self.state.drain_events())

    def on_tick(self) -> None:
        if self.mode == "LIVE":
            self.process_events(self.state.tick())
        self.update_view()

    def on_durations_changed(self, _value=None) -> None:
        self.state.set_durations(
            fill_seconds=self.fill_seconds.value(),
            ign_seconds=self.ign_seconds.value(),
            fill_extend_seconds=self.extend_seconds.value(),
        )

    def on_mode_changed(self, _index=None) -> None:
        new_mode = self.mode_combo.currentText()
        if new_mode == self.mode:
            return
        if self.mode == "LIVE" and (
            self.state.fill_open or self.state.veh_open or self.state.ign_state != OFF
        ):
            answer = QMessageBox.question(
                self,
                "Leave LIVE mode",
                "Leaving LIVE mode sends ALL OFF to the actuator. Continue?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                self.mode_combo.blockSignals(True)
                self.mode_combo.setCurrentText(self.mode)
                self.mode_combo.blockSignals(False)
                return
        self.state.all_off()
        self.flush_state_events()
        self.send("ALL_OFF")
        self.mode = new_mode
        self.log(f"MODE {new_mode} (ALL OFF sent to actuator)")
        self.update_view()

    # --------------------------------------------------------- click handlers
    def fill_on_clicked(self) -> None:
        if self.mode == "TEST":
            self.send("SV0_ON")
            self.log("TEST: FILL latched ON")
        elif self.state.fill_on():
            self.send("SV0_ON")
        self.flush_state_events()

    def fill_close_clicked(self) -> None:
        if self.mode == "TEST":
            self.send("SV0_OFF")
            self.log("TEST: FILL latched OFF")
        elif self.state.fill_close():
            self.send("SV0_OFF")
        self.flush_state_events()

    def fill_extend_clicked(self) -> None:
        if self.mode == "LIVE" and self.state.fill_extend():
            self.log("FILL close deadline extended (no radio command needed)")
        self.flush_state_events()

    def veh_on_clicked(self) -> None:
        if self.mode == "TEST":
            self.send("SV1_ON")
            self.log("TEST: VEH latched ON")
        elif self.state.veh_on():
            self.send("SV1_ON")
        self.flush_state_events()

    def veh_off_clicked(self) -> None:
        if self.mode == "TEST":
            self.send("SV1_OFF")
            self.log("TEST: VEH latched OFF")
        elif self.state.veh_off():
            self.send("SV1_OFF")
        self.flush_state_events()

    def both_on_clicked(self) -> None:
        # Disabled in LIVE mode: FILL must be closed before VEH opens.
        if self.mode == "TEST":
            self.send("3")  # timed both-servo pulse
            self.log("TEST: both-servo pulse")
        self.flush_state_events()

    def both_pulse_clicked(self) -> None:
        self.send("3")
        self.log("TEST: both-servo pulse")
        self.flush_state_events()

    def both_off_clicked(self) -> None:
        self.send("BOTH_OFF")
        if self.mode == "LIVE":
            self.state.fill_open = False
            self.state.veh_open = False
            self.state.fill_deadline = None
        self.flush_state_events()

    def ign_arm_clicked(self) -> None:
        if self.mode == "LIVE" and self.state.ign_arm():
            self.send(f"IGN_ARM:{int(self.state.ign_seconds * 1000)}")
        self.flush_state_events()

    def ign_on_clicked(self) -> None:
        if self.mode == "LIVE" and self.state.ign_on():
            self.send("IGN_ON")
        self.flush_state_events()

    def ign_off_clicked(self) -> None:
        if self.mode == "LIVE":
            self.state.ign_off()
        self.send("IGN_OFF")
        self.flush_state_events()

    def all_off_clicked(self) -> None:
        self.state.all_off()
        self.flush_state_events()
        self.send("ALL_OFF")
        self.log("ALL OFF requested")

    # ------------------------------------------------------------- rendering
    def update_view(self) -> None:
        connected = self.serial.isOpen()
        live = self.mode == "LIVE"
        state = self.state

        fill_remaining, ign_remaining = state.remaining()
        if live and state.fill_open:
            text = mmss(fill_remaining) + ("  DUE" if state.fill_notified else "")
            self.fill_time_label.setText(text)
            self.fill_time_label.setStyleSheet(
                f"color: {'#f1c40f' if state.fill_notified else '#2ecc71'};"
            )
            self.fill_note_label.setText("FILL OPEN (servo0)")
        else:
            self.fill_time_label.setText("--:--")
            self.fill_time_label.setStyleSheet("color: #5f6878;")
            self.fill_note_label.setText("FILL CLOSED (servo0)")

        if not live:
            self.ign_time_label.setText("TEST")
            self.ign_time_label.setStyleSheet("color: #5f6878;")
        elif state.ign_state == ARMING:
            self.ign_time_label.setText(mmss(ign_remaining))
            self.ign_time_label.setStyleSheet("color: #f39c12;")
        elif state.ign_state == ARMED:
            self.ign_time_label.setText("ARMED")
            self.ign_time_label.setStyleSheet("color: #f1c40f;")
        elif state.ign_state == ON:
            self.ign_time_label.setText("ON")
            self.ign_time_label.setStyleSheet("color: #e74c3c;")
        else:
            self.ign_time_label.setText("OFF")
            self.ign_time_label.setStyleSheet("color: #2ecc71;")

        self.btn_fill_on.setEnabled(connected and (state.can_fill_on if live else True))
        self.btn_fill_close.setEnabled(connected and (state.can_fill_close if live else True))
        self.btn_fill_extend.setEnabled(connected and (state.can_fill_extend if live else False))
        self.btn_veh_on.setEnabled(connected and (state.can_veh_on if live else True))
        self.btn_veh_off.setEnabled(connected and (state.can_veh_off if live else True))
        for widget in (self.both_label, self.btn_both_on, self.btn_both_pulse, self.btn_both_off):
            widget.setVisible(not live)
        self.btn_both_on.setEnabled(connected and not live)
        self.btn_both_pulse.setEnabled(connected and not live)
        self.btn_both_off.setEnabled(connected and not live)
        self.btn_ign_pulse.setVisible(not live)
        self.btn_ign_pulse.setEnabled(connected and not live)
        self.btn_ign_arm.setVisible(live)
        self.btn_ign_arm.setEnabled(connected and (state.can_ign_arm if live else False))
        self.btn_ign_on.setEnabled(connected and (state.can_ign_on if live else False))
        self.btn_ign_off.setEnabled(connected and (state.can_ign_off if live else True))
        self.btn_all_off.setEnabled(True)


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Monospace", 10))
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
