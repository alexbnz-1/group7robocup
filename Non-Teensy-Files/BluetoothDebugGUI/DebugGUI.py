"""
DebugGUI.py

Live robot debug console with:
- COM port selection
- dynamic telemetry
- dashboard commands
- live plots
- dynamic parameters and commands
- logs/raw serial
- full session recording to .rdbg files

Dependencies:
    pip install PyQt6 pyserial pyqtgraph
"""

from __future__ import annotations

import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

from PyQt6.QtCore import Qt, QSettings, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from BluetoothSerial import BluetoothSerial
from DataRecorder import DataRecorder


class ValueEditor(QWidget):
    def __init__(self, definition: dict, parent=None):
        super().__init__(parent)
        self.definition = definition
        datatype = str(
            definition.get("datatype", definition.get("type", "float"))
        ).lower()
        self.datatype = datatype

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if datatype in ("bool", "boolean"):
            self.editor = QCheckBox()
            self.editor.setChecked(bool(definition.get("value", definition.get("default", False))))

        elif datatype in ("int", "integer"):
            self.editor = QSpinBox()
            self.editor.setRange(
                int(definition.get("min", -1_000_000_000)),
                int(definition.get("max", 1_000_000_000)),
            )
            self.editor.setSingleStep(max(int(definition.get("step", 1)), 1))
            self.editor.setValue(int(definition.get("value", definition.get("default", 0))))

        elif datatype in ("enum", "choice", "select"):
            self.editor = QComboBox()
            for option in definition.get("options", []):
                if isinstance(option, dict):
                    label = str(option.get("label", option.get("value", "")))
                    value = option.get("value", label)
                    self.editor.addItem(label, value)
                else:
                    self.editor.addItem(str(option), option)

            current = definition.get("value", definition.get("default"))
            for index in range(self.editor.count()):
                if self.editor.itemData(index) == current:
                    self.editor.setCurrentIndex(index)
                    break

        elif datatype in ("str", "string", "text"):
            self.editor = QLineEdit()
            self.editor.setText(str(definition.get("value", definition.get("default", ""))))

        else:
            self.editor = QDoubleSpinBox()
            self.editor.setDecimals(int(definition.get("decimals", 4)))
            self.editor.setRange(
                float(definition.get("min", -1e12)),
                float(definition.get("max", 1e12)),
            )
            self.editor.setSingleStep(float(definition.get("step", 0.01)))
            self.editor.setValue(float(definition.get("value", definition.get("default", 0.0))))

        layout.addWidget(self.editor)

    def value(self):
        if isinstance(self.editor, QCheckBox):
            return self.editor.isChecked()
        if isinstance(self.editor, QComboBox):
            return self.editor.currentData()
        if isinstance(self.editor, QLineEdit):
            return self.editor.text()
        return self.editor.value()

    def set_value(self, value: Any):
        try:
            if isinstance(self.editor, QCheckBox):
                self.editor.setChecked(bool(value))
            elif isinstance(self.editor, QComboBox):
                for index in range(self.editor.count()):
                    if self.editor.itemData(index) == value:
                        self.editor.setCurrentIndex(index)
                        break
            elif isinstance(self.editor, QLineEdit):
                self.editor.setText(str(value))
            else:
                self.editor.setValue(value)
        except Exception:
            pass


class CommandWidget(QGroupBox):
    def __init__(self, definition: dict, send_callback, parent=None):
        name = str(definition.get("name", "command"))
        label = str(definition.get("label", name))
        super().__init__(label, parent)

        self.name = name
        self.send_callback = send_callback
        self.argument_editors = {}

        layout = QFormLayout(self)

        description = definition.get("description")
        if description:
            lbl = QLabel(str(description))
            lbl.setWordWrap(True)
            layout.addRow(lbl)

        arguments = definition.get("args", definition.get("arguments", []))
        if isinstance(arguments, dict):
            converted = []
            for arg_name, arg_definition in arguments.items():
                if isinstance(arg_definition, dict):
                    item = dict(arg_definition)
                    item["name"] = arg_name
                else:
                    item = {"name": arg_name, "type": str(arg_definition)}
                converted.append(item)
            arguments = converted

        for argument in arguments:
            arg_name = str(argument.get("name", "value"))
            arg_label = str(argument.get("label", arg_name))
            editor = ValueEditor(argument)
            self.argument_editors[arg_name] = editor
            layout.addRow(arg_label, editor)

        run_button = QPushButton(f"Run {label}")
        run_button.clicked.connect(self._execute)
        layout.addRow(run_button)

    def _execute(self):
        arguments = {
            name: editor.value()
            for name, editor in self.argument_editors.items()
        }
        self.send_callback(self.name, arguments)


class RobotDebugGUI(QMainWindow):
    MAX_HISTORY_POINTS = 10000

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Debug Console")
        self.resize(1500, 900)

        self.settings = QSettings("RobotProject", "DebugGUI")
        self.bluetooth = BluetoothSerial()
        self.recorder = DataRecorder()

        self.telemetry = {}
        self.telemetry_rows = {}
        self.telemetry_history = {}
        self.parameter_editors = {}
        self.command_widgets = {}
        self.dashboard_command_widgets = {}
        self.plot_curves = {}
        self.start_time = time.monotonic()

        self._build_ui()
        self._connect_signals()

        self.port_refresh_timer = QTimer(self)
        self.port_refresh_timer.timeout.connect(self.refresh_ports)
        self.port_refresh_timer.start(2000)

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(50)

        self.recording_timer = QTimer(self)
        self.recording_timer.timeout.connect(self.update_recording_clock)
        self.recording_timer.start(250)

        self.refresh_ports()

    # ------------------------------------------------------------------
    # Paths / recording
    # ------------------------------------------------------------------

    def default_data_directory(self) -> Path:
        debug_dir = Path(__file__).resolve().parent
        data_dir = debug_dir.parent / "DataVisualisationAfter" / "Data"
        data_dir.mkdir(parents=True, exist_ok=True)
        return data_dir

    def toggle_recording(self):
        if self.recorder.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        if not self.bluetooth.is_connected():
            QMessageBox.warning(
                self,
                "Robot not connected",
                "Connect to the robot before starting a recording.",
            )
            return

        now = datetime.now()
        suggested = f"RobotRun_{now:%Y-%m-%d_%H-%M-%S}.rdbg"

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Robot Recording As",
            str(self.default_data_directory() / suggested),
            "Robot Debug Recording (*.rdbg)",
        )
        if not filename:
            return

        path = Path(filename)
        if path.suffix.lower() != ".rdbg":
            path = path.with_suffix(".rdbg")

        self.recorder.start(
            path,
            session_name=path.stem,
            port=str(self.port_combo.currentData() or ""),
            baudrate=int(self.baud_combo.currentData()),
        )

        self.record_button.setText("Stop Recording")
        self.recording_status.setText(f"● RECORDING — {path.name}")
        self.add_log("SYSTEM", f"Recording started: {path.name}")
        self.statusBar().showMessage(f"Recording to {path}")

    def stop_recording(self):
        if not self.recorder.is_recording:
            return

        path = self.recorder.path
        self.recorder.record_log("SYSTEM", "Recording stopped")
        self.recorder.stop()

        self.record_button.setText("Start Recording")
        self.recording_status.setText("○ NOT RECORDING")
        self.recording_time_label.setText("00:00:00")

        if path is not None:
            self.add_log("SYSTEM", f"Recording saved: {path.name}")
            self.statusBar().showMessage(f"Recording saved: {path}")

    def update_recording_clock(self):
        if not self.recorder.is_recording:
            return
        seconds = int(self.recorder.elapsed)
        h, rem = divmod(seconds, 3600)
        m, s = divmod(rem, 60)
        self.recording_time_label.setText(f"{h:02d}:{m:02d}:{s:02d}")

    # ------------------------------------------------------------------
    # Main UI
    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        connection_group = QGroupBox("Robot Connection")
        connection_layout = QHBoxLayout(connection_group)

        connection_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(260)
        connection_layout.addWidget(self.port_combo)

        self.refresh_button = QPushButton("Refresh")
        connection_layout.addWidget(self.refresh_button)

        connection_layout.addSpacing(10)
        connection_layout.addWidget(QLabel("Baud:"))

        self.baud_combo = QComboBox()
        for baud in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600):
            self.baud_combo.addItem(str(baud), baud)

        saved_baud = int(self.settings.value("baud", 115200))
        baud_index = self.baud_combo.findData(saved_baud)
        if baud_index >= 0:
            self.baud_combo.setCurrentIndex(baud_index)

        connection_layout.addWidget(self.baud_combo)

        self.connect_button = QPushButton("Connect")
        self.connect_button.setMinimumWidth(100)
        connection_layout.addWidget(self.connect_button)

        self.connection_status = QLabel("● DISCONNECTED")
        self.connection_status.setMinimumWidth(150)
        connection_layout.addWidget(self.connection_status)

        connection_layout.addSpacing(12)

        # Recording controls: visible at all times for testing.
        self.record_button = QPushButton("Start Recording")
        self.record_button.setMinimumWidth(125)
        self.record_button.setEnabled(False)
        connection_layout.addWidget(self.record_button)

        self.recording_status = QLabel("○ NOT RECORDING")
        self.recording_status.setMinimumWidth(180)
        connection_layout.addWidget(self.recording_status)

        self.recording_time_label = QLabel("00:00:00")
        self.recording_time_label.setMinimumWidth(65)
        connection_layout.addWidget(self.recording_time_label)

        connection_layout.addStretch()

        self.enter_debug_button = QPushButton("Enter Debug Mode")
        self.enter_debug_button.setEnabled(False)
        connection_layout.addWidget(self.enter_debug_button)

        self.exit_debug_button = QPushButton("Exit Debug Mode")
        self.exit_debug_button.setEnabled(False)
        connection_layout.addWidget(self.exit_debug_button)

        main_layout.addWidget(connection_group)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs, 1)

        self._build_dashboard_tab()
        self._build_plot_tab()
        self._build_parameter_tab()
        self._build_command_tab()
        self._build_log_tab()
        self._build_raw_tab()

        self.statusBar().showMessage("Ready")

    def _build_dashboard_tab(self):
        page = QWidget()
        main_layout = QVBoxLayout(page)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter, 1)

        telemetry_panel = QWidget()
        telemetry_layout = QVBoxLayout(telemetry_panel)

        top_layout = QHBoxLayout()
        title = QLabel("Live Telemetry")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        top_layout.addWidget(title)
        top_layout.addStretch()

        self.clear_telemetry_button = QPushButton("Clear")
        top_layout.addWidget(self.clear_telemetry_button)
        telemetry_layout.addLayout(top_layout)

        self.telemetry_table = QTableWidget(0, 3)
        self.telemetry_table.setHorizontalHeaderLabels(["Signal", "Value", "Last Update"])
        self.telemetry_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.telemetry_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)

        header = self.telemetry_table.horizontalHeader()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(0, header.ResizeMode.Stretch)
        header.setSectionResizeMode(1, header.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, header.ResizeMode.ResizeToContents)

        telemetry_layout.addWidget(self.telemetry_table, 1)
        splitter.addWidget(telemetry_panel)

        command_panel = QWidget()
        command_panel_layout = QVBoxLayout(command_panel)

        command_header = QHBoxLayout()
        command_title = QLabel("Commands")
        command_title.setFont(title_font)
        command_header.addWidget(command_title)
        command_header.addStretch()
        command_panel_layout.addLayout(command_header)

        stop_group = QGroupBox("Emergency / General")
        stop_layout = QVBoxLayout(stop_group)
        self.dashboard_stop_button = QPushButton("STOP ROBOT")
        self.dashboard_stop_button.setMinimumHeight(45)
        self.dashboard_stop_button.setEnabled(False)
        stop_layout.addWidget(self.dashboard_stop_button)
        command_panel_layout.addWidget(stop_group)

        self.dashboard_command_scroll = QScrollArea()
        self.dashboard_command_scroll.setWidgetResizable(True)
        self.dashboard_command_container = QWidget()
        self.dashboard_command_layout = QVBoxLayout(self.dashboard_command_container)
        self.dashboard_command_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.dashboard_command_scroll.setWidget(self.dashboard_command_container)
        command_panel_layout.addWidget(self.dashboard_command_scroll, 1)

        splitter.addWidget(command_panel)
        splitter.setSizes([950, 500])
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        self.tabs.addTab(page, "Dashboard")

    def _build_plot_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Signal:"))

        self.plot_signal_combo = QComboBox()
        self.plot_signal_combo.setMinimumWidth(300)
        controls.addWidget(self.plot_signal_combo)

        self.add_plot_button = QPushButton("Add to Plot")
        self.remove_plot_button = QPushButton("Remove Selected")
        self.clear_plot_button = QPushButton("Clear Plot")

        controls.addWidget(self.add_plot_button)
        controls.addWidget(self.remove_plot_button)
        controls.addWidget(self.clear_plot_button)

        controls.addSpacing(20)
        controls.addWidget(QLabel("Time Window:"))

        self.time_window_combo = QComboBox()
        for text, seconds in (
            ("5 seconds", 5),
            ("10 seconds", 10),
            ("30 seconds", 30),
            ("1 minute", 60),
            ("2 minutes", 120),
            ("5 minutes", 300),
        ):
            self.time_window_combo.addItem(text, seconds)

        self.time_window_combo.setCurrentIndex(1)
        controls.addWidget(self.time_window_combo)
        controls.addStretch()
        layout.addLayout(controls)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        signal_container = QWidget()
        signal_layout = QVBoxLayout(signal_container)
        signal_layout.addWidget(QLabel("Currently plotted:"))
        self.active_plot_list = QListWidget()
        signal_layout.addWidget(self.active_plot_list)
        signal_container.setMaximumWidth(280)
        splitter.addWidget(signal_container)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.addLegend()
        splitter.addWidget(self.plot_widget)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter, 1)
        self.tabs.addTab(page, "Plots")

    def _build_parameter_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        top = QHBoxLayout()
        title = QLabel("Robot Parameters")
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        title.setFont(font)
        top.addWidget(title)
        top.addStretch()

        self.refresh_definitions_button = QPushButton("Refresh From Robot")
        top.addWidget(self.refresh_definitions_button)
        layout.addLayout(top)

        info = QLabel("Parameters advertised by the robot appear here automatically.")
        layout.addWidget(info)

        self.parameter_scroll = QScrollArea()
        self.parameter_scroll.setWidgetResizable(True)
        self.parameter_container = QWidget()
        self.parameter_layout = QVBoxLayout(self.parameter_container)
        self.parameter_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.parameter_scroll.setWidget(self.parameter_container)
        layout.addWidget(self.parameter_scroll)

        self.tabs.addTab(page, "Parameters")

    def _build_command_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        top = QHBoxLayout()
        title = QLabel("Robot Commands")
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        title.setFont(font)
        top.addWidget(title)
        top.addStretch()

        self.stop_button = QPushButton("STOP ROBOT")
        self.stop_button.setEnabled(False)
        top.addWidget(self.stop_button)
        layout.addLayout(top)

        info = QLabel("Commands advertised by the robot are generated here automatically.")
        layout.addWidget(info)

        self.command_scroll = QScrollArea()
        self.command_scroll.setWidgetResizable(True)
        self.command_container = QWidget()
        self.command_layout = QVBoxLayout(self.command_container)
        self.command_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.command_scroll.setWidget(self.command_container)
        layout.addWidget(self.command_scroll)

        self.tabs.addTab(page, "Commands")

    def _build_log_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        controls.addStretch()
        self.clear_log_button = QPushButton("Clear")
        controls.addWidget(self.clear_log_button)
        layout.addLayout(controls)

        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        font = QFont("Consolas")
        font.setStyleHint(QFont.StyleHint.Monospace)
        self.log_console.setFont(font)
        layout.addWidget(self.log_console)

        self.tabs.addTab(page, "Logs")

    def _build_raw_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        self.raw_pause_checkbox = QCheckBox("Pause display")
        controls.addWidget(self.raw_pause_checkbox)
        controls.addStretch()
        self.clear_raw_button = QPushButton("Clear")
        controls.addWidget(self.clear_raw_button)
        layout.addLayout(controls)

        self.raw_console = QTextEdit()
        self.raw_console.setReadOnly(True)
        font = QFont("Consolas")
        font.setStyleHint(QFont.StyleHint.Monospace)
        self.raw_console.setFont(font)
        layout.addWidget(self.raw_console)

        self.tabs.addTab(page, "Raw Serial")

    # ------------------------------------------------------------------
    # Signals
    # ------------------------------------------------------------------

    def _connect_signals(self):
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.record_button.clicked.connect(self.toggle_recording)

        self.enter_debug_button.clicked.connect(
            lambda: self.execute_command("set_debug_mode", {"enabled": True})
        )
        self.exit_debug_button.clicked.connect(
            lambda: self.execute_command("set_debug_mode", {"enabled": False})
        )

        self.stop_button.clicked.connect(lambda: self.execute_command("stop", {}))
        self.dashboard_stop_button.clicked.connect(lambda: self.execute_command("stop", {}))

        self.refresh_definitions_button.clicked.connect(self.bluetooth.request_definitions)
        self.clear_telemetry_button.clicked.connect(self.clear_telemetry)
        self.add_plot_button.clicked.connect(self.add_selected_plot)
        self.remove_plot_button.clicked.connect(self.remove_selected_plot)
        self.clear_plot_button.clicked.connect(self.clear_plots)
        self.clear_log_button.clicked.connect(self.log_console.clear)
        self.clear_raw_button.clicked.connect(self.raw_console.clear)

        self.bluetooth.connection_changed.connect(self.on_connection_changed)
        self.bluetooth.telemetry_received.connect(self.on_telemetry)
        self.bluetooth.parameter_definition_received.connect(self.on_parameter_definition)
        self.bluetooth.parameter_value_received.connect(self.on_parameter_value)
        self.bluetooth.command_definition_received.connect(self.on_command_definition)
        self.bluetooth.log_received.connect(self.on_log)
        self.bluetooth.raw_received.connect(self.on_raw)
        self.bluetooth.error_received.connect(self.on_error)
        self.bluetooth.robot_state_received.connect(self.on_robot_state)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def refresh_ports(self):
        ports = self.bluetooth.available_ports()
        current_device = self.port_combo.currentData()
        saved_device = self.settings.value("port", "")
        target_device = current_device or saved_device

        existing_devices = {
            self.port_combo.itemData(i)
            for i in range(self.port_combo.count())
        }
        new_devices = {port["device"] for port in ports}

        if existing_devices == new_devices:
            return

        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        for port in ports:
            device = port["device"]
            description = port["description"]
            label = f"{device} — {description}" if description else device
            self.port_combo.addItem(label, device)

        if target_device:
            index = self.port_combo.findData(target_device)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

        self.port_combo.blockSignals(False)

    def toggle_connection(self):
        if self.bluetooth.is_connected():
            self.bluetooth.disconnect_port()
            return

        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(
                self,
                "No Serial Port",
                "No serial port is available.\n\nConnect the adapter and press Refresh.",
            )
            return

        baudrate = int(self.baud_combo.currentData())
        self.settings.setValue("port", port)
        self.settings.setValue("baud", baudrate)

        self.statusBar().showMessage(f"Connecting to {port}...")
        self.connect_button.setEnabled(False)
        self.bluetooth.connect_port(port, baudrate)

    def on_connection_changed(self, connected: bool, port: str):
        self.connect_button.setEnabled(True)

        if connected:
            self.connect_button.setText("Disconnect")
            self.connection_status.setText(f"● CONNECTED — {port}")
            self.port_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
            self.refresh_button.setEnabled(False)

            self.enter_debug_button.setEnabled(True)
            self.exit_debug_button.setEnabled(True)
            self.stop_button.setEnabled(True)
            self.dashboard_stop_button.setEnabled(True)
            self.record_button.setEnabled(True)

            self.statusBar().showMessage(f"Connected to {port}")
            self.add_log("SYSTEM", f"Connected to {port}")

        else:
            if self.recorder.is_recording:
                self.stop_recording()

            self.connect_button.setText("Connect")
            self.connection_status.setText("● DISCONNECTED")
            self.port_combo.setEnabled(True)
            self.baud_combo.setEnabled(True)
            self.refresh_button.setEnabled(True)

            self.enter_debug_button.setEnabled(False)
            self.exit_debug_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            self.dashboard_stop_button.setEnabled(False)
            self.record_button.setEnabled(False)

            self.statusBar().showMessage("Disconnected")
            self.add_log("SYSTEM", "Disconnected")

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def on_telemetry(self, name: str, value: Any, robot_timestamp: Any):
        self.recorder.record_telemetry(name, value, robot_timestamp)

        now = time.monotonic()
        self.telemetry[name] = value

        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if name not in self.telemetry_history:
                self.telemetry_history[name] = deque(maxlen=self.MAX_HISTORY_POINTS)

            elapsed = now - self.start_time
            self.telemetry_history[name].append((elapsed, float(value)))

            if self.plot_signal_combo.findText(name) < 0:
                self.plot_signal_combo.addItem(name)

        if name not in self.telemetry_rows:
            row = self.telemetry_table.rowCount()
            self.telemetry_table.insertRow(row)
            self.telemetry_rows[name] = row
            self.telemetry_table.setItem(row, 0, QTableWidgetItem(name))

        row = self.telemetry_rows[name]
        self.telemetry_table.setItem(row, 1, QTableWidgetItem(self.format_value(value)))
        self.telemetry_table.setItem(row, 2, QTableWidgetItem(time.strftime("%H:%M:%S")))

    @staticmethod
    def format_value(value: Any) -> str:
        if isinstance(value, float):
            return f"{value:.6g}"
        if isinstance(value, bool):
            return "TRUE" if value else "FALSE"
        return str(value)

    def clear_telemetry(self):
        self.telemetry.clear()
        self.telemetry_rows.clear()
        self.telemetry_history.clear()
        self.telemetry_table.setRowCount(0)
        self.plot_signal_combo.clear()
        self.clear_plots()
        self.start_time = time.monotonic()

    # ------------------------------------------------------------------
    # Live plotting
    # ------------------------------------------------------------------

    def add_selected_plot(self):
        name = self.plot_signal_combo.currentText()
        if not name or name in self.plot_curves:
            return

        curve = self.plot_widget.plot([], [], name=name)
        self.plot_curves[name] = curve
        self.active_plot_list.addItem(name)

    def remove_selected_plot(self):
        for item in self.active_plot_list.selectedItems():
            name = item.text()
            curve = self.plot_curves.pop(name, None)
            if curve is not None:
                self.plot_widget.removeItem(curve)
            self.active_plot_list.takeItem(self.active_plot_list.row(item))

    def clear_plots(self):
        for curve in self.plot_curves.values():
            self.plot_widget.removeItem(curve)
        self.plot_curves.clear()
        self.active_plot_list.clear()

    def update_plot(self):
        if not self.plot_curves:
            return

        now = time.monotonic() - self.start_time
        window = float(self.time_window_combo.currentData())
        minimum_time = now - window

        for name, curve in self.plot_curves.items():
            history = self.telemetry_history.get(name)
            if not history:
                continue

            x = []
            y = []
            for timestamp, value in history:
                if timestamp >= minimum_time:
                    x.append(timestamp - now)
                    y.append(value)

            curve.setData(x, y)

        self.plot_widget.setXRange(-window, 0, padding=0)

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def on_parameter_definition(self, definition: dict):
        name = str(definition.get("name", ""))
        if not name:
            return

        if name in self.parameter_editors:
            if "value" in definition:
                self.parameter_editors[name].set_value(definition["value"])
            return

        group = QGroupBox(str(definition.get("label", name)))
        layout = QVBoxLayout(group)

        description = definition.get("description")
        if description:
            text = QLabel(str(description))
            text.setWordWrap(True)
            layout.addWidget(text)

        editor = ValueEditor(definition)
        self.parameter_editors[name] = editor
        layout.addWidget(editor)

        unit = definition.get("unit")
        if unit:
            layout.addWidget(QLabel(f"Unit: {unit}"))

        apply_button = QPushButton("Apply")
        apply_button.clicked.connect(
            lambda checked=False, parameter_name=name, parameter_editor=editor:
            self.apply_parameter(parameter_name, parameter_editor)
        )
        layout.addWidget(apply_button)
        self.parameter_layout.addWidget(group)

    def apply_parameter(self, name: str, editor: ValueEditor):
        value = editor.value()
        self.recorder.record_parameter(name, value)
        self.bluetooth.set_parameter(name, value)
        self.add_log("TX", f"{name} = {value}")

    def on_parameter_value(self, name: str, value: Any):
        editor = self.parameter_editors.get(name)
        if editor is not None:
            editor.set_value(value)

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def on_command_definition(self, definition: dict):
        name = str(definition.get("name", ""))
        if not name:
            return

        if name not in self.command_widgets:
            widget = CommandWidget(definition, self.execute_command)
            self.command_widgets[name] = widget
            self.command_layout.addWidget(widget)

        if name not in self.dashboard_command_widgets:
            widget = CommandWidget(definition, self.execute_command)
            self.dashboard_command_widgets[name] = widget
            self.dashboard_command_layout.addWidget(widget)

    def execute_command(self, name: str, arguments: dict):
        self.recorder.record_command(name, arguments)
        self.bluetooth.send_command(name, **arguments)

        if arguments:
            argument_text = ", ".join(f"{key}={value}" for key, value in arguments.items())
            text = f"{name}({argument_text})"
        else:
            text = f"{name}()"

        self.add_log("TX", text)

    # ------------------------------------------------------------------
    # Logs / states / raw
    # ------------------------------------------------------------------

    def on_log(self, level: str, message: str):
        self.recorder.record_log(level, message)
        self.add_log(level, message)

    def add_log(self, level: str, message: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_console.append(f"[{timestamp}] [{level}] {message}")

    def on_raw(self, text: str):
        # Raw data is recorded even if display is paused.
        self.recorder.record_raw(text)

        if self.raw_pause_checkbox.isChecked():
            return

        self.raw_console.append(text)

    def on_robot_state(self, state: dict):
        self.recorder.record_state(state)

        debug_enabled = state.get("debug_mode")
        if debug_enabled is True:
            self.statusBar().showMessage("Robot is in DEBUG mode")
        elif debug_enabled is False:
            self.statusBar().showMessage("Robot is in NORMAL mode")

    def on_error(self, message: str):
        self.add_log("ERROR", message)
        self.statusBar().showMessage(message)

    def closeEvent(self, event):
        if self.recorder.is_recording:
            self.stop_recording()

        self.bluetooth.disconnect_port()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Debug Console")
    window = RobotDebugGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
