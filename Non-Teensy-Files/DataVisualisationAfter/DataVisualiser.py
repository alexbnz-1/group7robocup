"""
DataVisualiser.py

Offline robot recording visualiser for *.rdbg files.

Features:
- searchable telemetry plotting
- safe autoscale, including sensible all-zero handling
- temporary blue highlight for selected plotted signal
- click-to-inspect cursor with values for every plotted signal
- draggable measurement region with min/max/mean/std/delta/sample count
- fault / command / parameter / state event markers
- double-click an event to jump the graph to it
- derived signals using expressions
- compare a second recording, including align-to-first-fault
- basic anomaly detection overlays
- raw serial, metadata, parameter snapshots, CSV export

Dependencies:
    pip install PyQt6 pyqtgraph numpy
"""

from __future__ import annotations

import csv
import json
import math
import sqlite3
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QEvent, Qt
from PyQt6.QtGui import QBrush, QColor, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QInputDialog,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / "Data"
DATA_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class SignalSeries:
    elapsed: np.ndarray
    robot: np.ndarray
    values: np.ndarray


class DataVisualiser(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Data Visualiser")
        self.resize(1600, 950)

        self.db_path: Path | None = None
        self.conn: sqlite3.Connection | None = None

        self.compare_path: Path | None = None
        self.compare_conn: sqlite3.Connection | None = None
        self.compare_offset = 0.0

        self.plot_curves: dict[str, pg.PlotDataItem] = {}
        self.compare_curves: dict[str, pg.PlotDataItem] = {}
        self.signal_cache: dict[tuple[str, str], SignalSeries] = {}
        self.derived: dict[str, str] = {}

        self.highlighted_signal: str | None = None
        self.highlighted_original_pen = None
        self.highlighted_original_visible = None
        self.highlighted_original_z = None

        self.marker_items = []
        self.anomaly_items = []

        self.cursor_line = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen(width=1))
        self.cursor_line.setZValue(1000)
        self.cursor_line.hide()

        self.measure_region = pg.LinearRegionItem(values=(0, 1), movable=True)
        self.measure_region.setZValue(900)
        self.measure_region.hide()

        self._build_ui()
        QApplication.instance().installEventFilter(self)
        self.refresh_recordings()

    # ------------------------------------------------------------------
    # DB helpers
    # ------------------------------------------------------------------

    @staticmethod
    def table_exists(conn: sqlite3.Connection | None, name: str) -> bool:
        if conn is None:
            return False
        return conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
            (name,),
        ).fetchone() is not None

    @staticmethod
    def get_metadata(conn: sqlite3.Connection | None) -> dict[str, str]:
        if conn is None or not DataVisualiser.table_exists(conn, "metadata"):
            return {}
        return {str(k): str(v) for k, v in conn.execute("SELECT key,value FROM metadata")}

    def _series(self, signal: str, source: str = "main") -> SignalSeries:
        key = (source, signal)
        if key in self.signal_cache:
            return self.signal_cache[key]

        if source == "main" and signal in self.derived:
            series = self._evaluate_derived(self.derived[signal])
            self.signal_cache[key] = series
            return series

        conn = self.conn if source == "main" else self.compare_conn
        if conn is None:
            return SignalSeries(np.array([]), np.array([]), np.array([]))

        rows = conn.execute(
            """SELECT elapsed_s, robot_time, value_num
               FROM telemetry
               WHERE signal=? AND value_num IS NOT NULL
               ORDER BY elapsed_s""",
            (signal,),
        ).fetchall()

        if not rows:
            result = SignalSeries(np.array([]), np.array([]), np.array([]))
        else:
            e = np.asarray([r[0] for r in rows], dtype=float)
            r = np.asarray([np.nan if r[1] is None else r[1] for r in rows], dtype=float)
            v = np.asarray([r[2] for r in rows], dtype=float)
            result = SignalSeries(e, r, v)

        self.signal_cache[key] = result
        return result

    def _evaluate_derived(self, expression: str) -> SignalSeries:
        # Expression syntax: sig("drive.left_rpm") - sig("drive.right_rpm")
        import re

        names = re.findall(r"""sig\(\s*["']([^"']+)["']\s*\)""", expression)
        if not names:
            raise ValueError('Expression must contain at least one sig("name") reference.')

        base = self._series(names[0], "main")
        if len(base.elapsed) == 0:
            raise ValueError(f"No numeric data for {names[0]}")

        x = base.elapsed.copy()

        def sig(name: str):
            s = self._series(name, "main")
            if len(s.elapsed) == 0:
                raise ValueError(f"No numeric data for {name}")
            return np.interp(x, s.elapsed, s.values, left=np.nan, right=np.nan)

        safe = {
            "sig": sig,
            "np": np,
            "abs": np.abs,
            "sqrt": np.sqrt,
            "sin": np.sin,
            "cos": np.cos,
            "tan": np.tan,
            "clip": np.clip,
            "minimum": np.minimum,
            "maximum": np.maximum,
        }
        values = eval(expression, {"__builtins__": {}}, safe)
        values = np.asarray(values, dtype=float)
        return SignalSeries(x, np.full_like(x, np.nan), values)

    def _x_values(self, s: SignalSeries, source: str = "main") -> np.ndarray:
        if self.x_axis_combo.currentData() == "robot":
            robot = s.robot.copy()
            finite = np.isfinite(robot)
            if finite.any():
                first = robot[finite][0]
                robot[finite] -= first
                if np.nanmax(np.abs(robot[finite])) > 1000:
                    robot[finite] /= 1000.0
                return robot
        x = s.elapsed.copy()
        if source == "compare":
            x = x + self.compare_offset
        return x

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top = QHBoxLayout()
        top.addWidget(QLabel("Recording:"))
        self.file_combo = QComboBox()
        self.file_combo.setMinimumWidth(380)
        top.addWidget(self.file_combo)

        self.refresh_button = QPushButton("Refresh")
        self.open_button = QPushButton("Open")
        self.browse_button = QPushButton("Browse...")
        self.compare_button = QPushButton("Compare...")
        self.align_fault_button = QPushButton("Align Compare to First Fault")
        self.clear_compare_button = QPushButton("Clear Compare")
        self.export_button = QPushButton("Export Selected CSV")

        for w in (
            self.refresh_button, self.open_button, self.browse_button,
            self.compare_button, self.align_fault_button, self.clear_compare_button,
            self.export_button,
        ):
            top.addWidget(w)
        top.addStretch()
        root.addLayout(top)

        self.summary_label = QLabel("No recording loaded")
        self.summary_label.setWordWrap(True)
        root.addWidget(self.summary_label)

        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)

        self._build_plot_tab()
        self._build_events_tab()
        self._build_raw_tab()
        self._build_metadata_tab()

        self.refresh_button.clicked.connect(self.refresh_recordings)
        self.open_button.clicked.connect(self.open_selected_recording)
        self.browse_button.clicked.connect(self.browse_recording)
        self.compare_button.clicked.connect(self.browse_compare_recording)
        self.align_fault_button.clicked.connect(self.align_compare_to_first_fault)
        self.clear_compare_button.clicked.connect(self.clear_compare)
        self.export_button.clicked.connect(self.export_selected_csv)

    def _build_plot_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Search:"))
        self.signal_filter = QLineEdit()
        self.signal_filter.setPlaceholderText("drive, imu, battery...")
        controls.addWidget(self.signal_filter)

        self.add_button = QPushButton("Add Selected")
        self.remove_button = QPushButton("Remove Plot")
        self.clear_button = QPushButton("Clear Plots")
        self.autoscale_button = QPushButton("Auto Scale")
        self.derived_button = QPushButton("Add Derived Signal")
        self.measure_button = QPushButton("Measure Region")
        self.anomaly_button = QPushButton("Find Anomalies")
        self.clear_anomaly_button = QPushButton("Clear Anomalies")

        for w in (
            self.add_button, self.remove_button, self.clear_button, self.autoscale_button,
            self.derived_button, self.measure_button, self.anomaly_button, self.clear_anomaly_button,
        ):
            controls.addWidget(w)

        self.show_faults = QCheckBox("Faults")
        self.show_faults.setChecked(True)
        self.show_commands = QCheckBox("Commands")
        self.show_parameters = QCheckBox("Parameters")
        self.show_states = QCheckBox("States")
        for w in (self.show_faults, self.show_commands, self.show_parameters, self.show_states):
            controls.addWidget(w)

        controls.addWidget(QLabel("X:"))
        self.x_axis_combo = QComboBox()
        self.x_axis_combo.addItem("Elapsed time", "elapsed")
        self.x_axis_combo.addItem("Robot time", "robot")
        controls.addWidget(self.x_axis_combo)
        layout.addLayout(controls)

        split = QSplitter(Qt.Orientation.Horizontal)

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.addWidget(QLabel("Available numeric signals"))
        self.signal_list = QListWidget()
        self.signal_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        ll.addWidget(self.signal_list)

        ll.addWidget(QLabel("Currently plotted"))
        self.plotted_list = QListWidget()
        ll.addWidget(self.plotted_list)

        ll.addWidget(QLabel("Cursor values"))
        self.cursor_table = QTableWidget(0, 2)
        self.cursor_table.setHorizontalHeaderLabels(["Signal", "Value"])
        self.cursor_table.horizontalHeader().setStretchLastSection(True)
        self.cursor_table.setMaximumHeight(220)
        ll.addWidget(self.cursor_table)

        self.measure_label = QLabel("Measurement region: off")
        self.measure_label.setWordWrap(True)
        ll.addWidget(self.measure_label)
        left.setMaximumWidth(410)
        split.addWidget(left)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "Elapsed time", units="s")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.addLegend()
        self.plot_widget.addItem(self.cursor_line)
        self.plot_widget.addItem(self.measure_region)
        split.addWidget(self.plot_widget)
        split.setStretchFactor(1, 1)

        layout.addWidget(split, 1)
        self.tabs.addTab(page, "Plots")

        self.signal_filter.textChanged.connect(self.apply_signal_filter)
        self.add_button.clicked.connect(self.add_selected_signals)
        self.remove_button.clicked.connect(self.remove_selected_plot)
        self.clear_button.clicked.connect(self.clear_plots)
        self.autoscale_button.clicked.connect(self.auto_scale_plots)
        self.derived_button.clicked.connect(self.add_derived_signal)
        self.measure_button.clicked.connect(self.toggle_measure_region)
        self.anomaly_button.clicked.connect(self.find_anomalies)
        self.clear_anomaly_button.clicked.connect(self.clear_anomalies)
        self.x_axis_combo.currentIndexChanged.connect(self.reload_plots)
        self.plotted_list.itemSelectionChanged.connect(self.highlight_selected_curve)
        self.show_faults.stateChanged.connect(self.refresh_markers)
        self.show_commands.stateChanged.connect(self.refresh_markers)
        self.show_parameters.stateChanged.connect(self.refresh_markers)
        self.show_states.stateChanged.connect(self.refresh_markers)
        self.measure_region.sigRegionChanged.connect(self.update_measurement)
        self.plot_widget.scene().sigMouseClicked.connect(self.plot_clicked)

    def _build_events_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        self.ev_fault = QCheckBox("Faults"); self.ev_fault.setChecked(True)
        self.ev_logs = QCheckBox("Logs"); self.ev_logs.setChecked(True)
        self.ev_commands = QCheckBox("Commands"); self.ev_commands.setChecked(True)
        self.ev_parameters = QCheckBox("Parameters"); self.ev_parameters.setChecked(True)
        self.ev_states = QCheckBox("States"); self.ev_states.setChecked(True)
        self.ev_annotations = QCheckBox("Annotations"); self.ev_annotations.setChecked(True)
        for w in (self.ev_fault, self.ev_logs, self.ev_commands, self.ev_parameters, self.ev_states, self.ev_annotations):
            controls.addWidget(w)
            w.stateChanged.connect(self.load_events)
        controls.addStretch()
        layout.addLayout(controls)

        self.events_table = QTableWidget(0, 4)
        self.events_table.setHorizontalHeaderLabels(["Elapsed (s)", "Type", "Name / Level", "Details"])
        self.events_table.horizontalHeader().setStretchLastSection(True)
        self.events_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.events_table.cellDoubleClicked.connect(self.jump_to_event)
        layout.addWidget(self.events_table)

        hint = QLabel("Tip: double-click any event to centre the plot around it.")
        layout.addWidget(hint)
        self.tabs.addTab(page, "Events")

    def _build_raw_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["Elapsed (s)", "Wall Time", "Serial Line"])
        self.raw_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.raw_table)
        self.tabs.addTab(page, "Raw Serial")

    def _build_metadata_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.metadata_text = QTextEdit()
        self.metadata_text.setReadOnly(True)
        layout.addWidget(self.metadata_text)
        self.tabs.addTab(page, "Session Info")

    # ------------------------------------------------------------------
    # Recording loading
    # ------------------------------------------------------------------

    def refresh_recordings(self):
        current = self.file_combo.currentData()
        self.file_combo.clear()
        files = sorted(DATA_DIR.glob("*.rdbg"), key=lambda p: p.stat().st_mtime, reverse=True)
        for f in files:
            self.file_combo.addItem(f.name, str(f))
        if current:
            idx = self.file_combo.findData(current)
            if idx >= 0:
                self.file_combo.setCurrentIndex(idx)

    def open_selected_recording(self):
        path = self.file_combo.currentData()
        if path:
            self.load_recording(Path(path))

    def browse_recording(self):
        name, _ = QFileDialog.getOpenFileName(self, "Open Robot Recording", str(DATA_DIR), "Robot Debug Recording (*.rdbg)")
        if name:
            self.load_recording(Path(name))

    def load_recording(self, path: Path):
        try:
            if self.conn is not None:
                self.conn.close()
            self.conn = sqlite3.connect(path)
            self.db_path = path
            self.signal_cache.clear()
            self.derived.clear()
            self.clear_plots()
            self.load_signal_names()
            self.load_events()
            self.load_raw()
            self.load_metadata()
            self.update_summary()
            self.refresh_markers()
            self.statusBar().showMessage(f"Loaded {path}")
        except Exception as exc:
            QMessageBox.critical(self, "Could not open recording", str(exc))

    def browse_compare_recording(self):
        name, _ = QFileDialog.getOpenFileName(self, "Open comparison recording", str(DATA_DIR), "Robot Debug Recording (*.rdbg)")
        if not name:
            return
        try:
            if self.compare_conn is not None:
                self.compare_conn.close()
            self.compare_path = Path(name)
            self.compare_conn = sqlite3.connect(self.compare_path)
            self.compare_offset = 0.0
            self.signal_cache = {k: v for k, v in self.signal_cache.items() if k[0] != "compare"}
            self.reload_plots()
            self.statusBar().showMessage(f"Comparison loaded: {self.compare_path.name}")
        except Exception as exc:
            QMessageBox.critical(self, "Could not load comparison", str(exc))

    def clear_compare(self):
        if self.compare_conn is not None:
            self.compare_conn.close()
        self.compare_conn = None
        self.compare_path = None
        self.compare_offset = 0.0
        for curve in self.compare_curves.values():
            self.plot_widget.removeItem(curve)
        self.compare_curves.clear()
        self.statusBar().showMessage("Comparison cleared")

    def _first_fault(self, conn):
        if conn is None or not self.table_exists(conn, "faults"):
            return None
        row = conn.execute("SELECT elapsed_s FROM faults ORDER BY elapsed_s LIMIT 1").fetchone()
        return None if row is None else float(row[0])

    def align_compare_to_first_fault(self):
        if self.conn is None or self.compare_conn is None:
            QMessageBox.information(self, "Compare", "Load both a main and comparison recording first.")
            return
        a = self._first_fault(self.conn)
        b = self._first_fault(self.compare_conn)
        if a is None or b is None:
            QMessageBox.information(self, "Compare", "Both recordings need at least one fault marker.")
            return
        self.compare_offset = a - b
        self.reload_plots()
        self.statusBar().showMessage(f"Comparison aligned: first fault at {a:.3f} s")

    def update_summary(self):
        if self.conn is None:
            return
        meta = self.get_metadata(self.conn)
        sigs = self.conn.execute("SELECT COUNT(DISTINCT signal) FROM telemetry").fetchone()[0]
        samples = self.conn.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        duration = self.conn.execute("SELECT COALESCE(MAX(elapsed_s),0) FROM telemetry").fetchone()[0]
        faults = 0
        if self.table_exists(self.conn, "faults"):
            faults = self.conn.execute("SELECT COUNT(*) FROM faults").fetchone()[0]
        compare = f" | Compare: {self.compare_path.name}" if self.compare_path else ""
        self.summary_label.setText(
            f"{self.db_path.name} | {duration:.2f} s | {sigs} signals | {samples} telemetry rows | "
            f"{faults} faults | Test: {meta.get('test_name','—')}{compare}"
        )

    def load_signal_names(self):
        self.signal_list.clear()
        if self.conn is None:
            return
        rows = self.conn.execute(
            "SELECT DISTINCT signal FROM telemetry WHERE value_num IS NOT NULL ORDER BY signal"
        )
        for (name,) in rows:
            self.signal_list.addItem(str(name))

    def apply_signal_filter(self):
        needle = self.signal_filter.text().strip().lower()
        for i in range(self.signal_list.count()):
            item = self.signal_list.item(i)
            item.setHidden(needle not in item.text().lower())

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------

    def add_selected_signals(self):
        for item in self.signal_list.selectedItems():
            self._add_signal(item.text())
        self.ensure_sensible_zero_only_view()
        self.refresh_markers()

    def _add_signal(self, name: str):
        if name in self.plot_curves:
            return
        s = self._series(name, "main")
        x = self._x_values(s, "main")
        finite = np.isfinite(x) & np.isfinite(s.values)
        curve = self.plot_widget.plot(x[finite], s.values[finite], name=name)
        self.plot_curves[name] = curve
        self.plotted_list.addItem(name)

        if self.compare_conn is not None:
            cs = self._series(name, "compare")
            cx = self._x_values(cs, "compare")
            cfinite = np.isfinite(cx) & np.isfinite(cs.values)
            if cfinite.any():
                pen = pg.mkPen(style=Qt.PenStyle.DashLine, width=2)
                ccurve = self.plot_widget.plot(cx[cfinite], cs.values[cfinite], pen=pen, name=f"{name} [compare]")
                self.compare_curves[name] = ccurve

    def reload_plots(self):
        names = list(self.plot_curves)
        self.clear_plot_items_only()
        self.plotted_list.clear()
        for name in names:
            self._add_signal(name)
        self.refresh_markers()
        self.update_cursor_values()
        self.update_measurement()

    def clear_plot_items_only(self):
        self.restore_highlight()
        for curve in list(self.plot_curves.values()) + list(self.compare_curves.values()):
            self.plot_widget.removeItem(curve)
        self.plot_curves.clear()
        self.compare_curves.clear()

    def remove_selected_plot(self):
        for item in list(self.plotted_list.selectedItems()):
            name = item.text()
            self.restore_highlight()
            curve = self.plot_curves.pop(name, None)
            if curve is not None:
                self.plot_widget.removeItem(curve)
            ccurve = self.compare_curves.pop(name, None)
            if ccurve is not None:
                self.plot_widget.removeItem(ccurve)
            self.plotted_list.takeItem(self.plotted_list.row(item))
        self.update_cursor_values()

    def clear_plots(self):
        self.clear_plot_items_only()
        self.plotted_list.clear()
        self.cursor_table.setRowCount(0)
        self.cursor_line.hide()
        self.refresh_markers()

    def auto_scale_plots(self):
        visible = []
        for name, curve in self.plot_curves.items():
            if not curve.isVisible():
                continue
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if mask.any():
                visible.append((x[mask], s.values[mask]))

        for name, curve in self.compare_curves.items():
            if not curve.isVisible():
                continue
            s = self._series(name, "compare")
            x = self._x_values(s, "compare")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if mask.any():
                visible.append((x[mask], s.values[mask]))

        if not visible:
            return

        all_x = np.concatenate([x for x, _ in visible])
        all_y = np.concatenate([y for _, y in visible])

        if np.all(all_y == 0):
            xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
            if xmin == xmax:
                xmin, xmax = xmin - 0.5, xmax + 0.5
            else:
                pad = max((xmax - xmin) * 0.05, 0.05)
                xmin, xmax = xmin - pad, xmax + pad
            self.plot_widget.setXRange(xmin, xmax, padding=0)
            self.plot_widget.setYRange(-1.0, 1.0, padding=0)
            return

        xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
        ymin, ymax = float(np.min(all_y)), float(np.max(all_y))

        if xmin == xmax:
            xmin, xmax = xmin - 0.5, xmax + 0.5
        else:
            px = max((xmax - xmin) * 0.05, 0.05)
            xmin, xmax = xmin - px, xmax + px

        if ymin == ymax:
            py = max(abs(ymin) * 0.1, 1.0)
        else:
            py = (ymax - ymin) * 0.08

        self.plot_widget.setXRange(xmin, xmax, padding=0)
        self.plot_widget.setYRange(ymin - py, ymax + py, padding=0)

    def ensure_sensible_zero_only_view(self):
        ys = []
        xs = []
        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if mask.any():
                xs.append(x[mask]); ys.append(s.values[mask])
        if ys and all(np.all(y == 0) for y in ys):
            x = np.concatenate(xs)
            xmin, xmax = float(np.min(x)), float(np.max(x))
            if xmin == xmax:
                xmin, xmax = xmin - 0.5, xmax + 0.5
            self.plot_widget.setXRange(xmin, xmax, padding=0)
            self.plot_widget.setYRange(-1, 1, padding=0)
            return True
        return False

    # ------------------------------------------------------------------
    # Temporary blue highlight
    # ------------------------------------------------------------------

    def highlight_selected_curve(self):
        self.restore_highlight()
        items = self.plotted_list.selectedItems()
        if not items:
            return
        name = items[0].text()
        curve = self.plot_curves.get(name)
        if curve is None:
            return
        self.highlighted_signal = name
        self.highlighted_original_pen = curve.opts.get("pen")
        self.highlighted_original_visible = curve.isVisible()
        self.highlighted_original_z = curve.zValue()
        curve.setVisible(True)
        curve.setPen(pg.mkPen((0, 120, 255), width=4))
        curve.setZValue(500)

    def restore_highlight(self):
        if self.highlighted_signal is None:
            return
        curve = self.plot_curves.get(self.highlighted_signal)
        if curve is not None:
            if self.highlighted_original_pen is not None:
                curve.setPen(self.highlighted_original_pen)
            curve.setVisible(bool(self.highlighted_original_visible))
            curve.setZValue(self.highlighted_original_z or 0)
        self.highlighted_signal = None

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonPress and self.highlighted_signal is not None:
            pos = event.globalPosition().toPoint() if hasattr(event, "globalPosition") else None
            if pos is not None:
                local = self.plotted_list.mapFromGlobal(pos)
                if not self.plotted_list.rect().contains(local):
                    self.restore_highlight()
                    self.plotted_list.clearSelection()
        return super().eventFilter(obj, event)

    # ------------------------------------------------------------------
    # Cursor
    # ------------------------------------------------------------------

    def plot_clicked(self, ev):
        if ev.button() != Qt.MouseButton.LeftButton:
            return
        if not self.plot_widget.getPlotItem().sceneBoundingRect().contains(ev.scenePos()):
            return
        mouse = self.plot_widget.getPlotItem().vb.mapSceneToView(ev.scenePos())
        self.cursor_line.setPos(mouse.x())
        self.cursor_line.show()
        self.update_cursor_values()

    def update_cursor_values(self):
        if not self.cursor_line.isVisible():
            return
        x0 = float(self.cursor_line.value())
        rows = []

        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if not mask.any():
                continue
            xf, yf = x[mask], s.values[mask]
            idx = int(np.argmin(np.abs(xf - x0)))
            rows.append((name, yf[idx]))

        for name in self.compare_curves:
            s = self._series(name, "compare")
            x = self._x_values(s, "compare")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if not mask.any():
                continue
            xf, yf = x[mask], s.values[mask]
            idx = int(np.argmin(np.abs(xf - x0)))
            rows.append((f"{name} [compare]", yf[idx]))

        self.cursor_table.setRowCount(len(rows))
        for r, (name, value) in enumerate(rows):
            self.cursor_table.setItem(r, 0, QTableWidgetItem(name))
            self.cursor_table.setItem(r, 1, QTableWidgetItem(f"{value:.8g}"))
        self.statusBar().showMessage(f"Cursor: {x0:.4f} s")

    # ------------------------------------------------------------------
    # Measurement region
    # ------------------------------------------------------------------

    def toggle_measure_region(self):
        if self.measure_region.isVisible():
            self.measure_region.hide()
            self.measure_label.setText("Measurement region: off")
            self.measure_button.setText("Measure Region")
            return

        xr = self.plot_widget.getPlotItem().vb.viewRange()[0]
        a = xr[0] + 0.35 * (xr[1] - xr[0])
        b = xr[0] + 0.65 * (xr[1] - xr[0])
        self.measure_region.setRegion((a, b))
        self.measure_region.show()
        self.measure_button.setText("Hide Measure Region")
        self.update_measurement()

    def update_measurement(self):
        if not self.measure_region.isVisible():
            return
        a, b = sorted(self.measure_region.getRegion())
        lines = [f"{a:.3f}–{b:.3f} s  (Δt={b-a:.3f}s)"]
        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values) & (x >= a) & (x <= b)
            vals = s.values[mask]
            if len(vals):
                std = float(np.std(vals)) if len(vals) > 1 else 0.0
                lines.append(
                    f"{name}: n={len(vals)} min={np.min(vals):.5g} max={np.max(vals):.5g} "
                    f"mean={np.mean(vals):.5g} std={std:.5g} Δ={vals[-1]-vals[0]:.5g}"
                )
        self.measure_label.setText("\n".join(lines))

    # ------------------------------------------------------------------
    # Derived signals
    # ------------------------------------------------------------------

    def add_derived_signal(self):
        name, ok = QInputDialog.getText(self, "Derived signal", "New signal name:")
        if not ok or not name.strip():
            return
        expr, ok = QInputDialog.getText(
            self,
            "Derived signal expression",
            'Expression, e.g. sig("drive.left_rpm") - sig("drive.right_rpm"):',
        )
        if not ok or not expr.strip():
            return

        name = name.strip()
        try:
            self.derived[name] = expr.strip()
            self.signal_cache.pop(("main", name), None)
            self._series(name, "main")  # validate now
        except Exception as exc:
            self.derived.pop(name, None)
            QMessageBox.warning(self, "Derived signal error", str(exc))
            return

        self.signal_list.addItem(name)
        self._add_signal(name)
        self.statusBar().showMessage(f"Derived signal added: {name}")

    # ------------------------------------------------------------------
    # Event markers
    # ------------------------------------------------------------------

    def _remove_markers(self):
        for item in self.marker_items:
            try:
                self.plot_widget.removeItem(item)
            except Exception:
                pass
        self.marker_items.clear()

    def _add_marker(self, x: float, label: str, color, width=2, style=Qt.PenStyle.SolidLine):
        line = pg.InfiniteLine(pos=x, angle=90, movable=False, pen=pg.mkPen(color=color, width=width, style=style))
        line.setZValue(800)
        text = pg.InfLineLabel(line, text=label, position=0.92, rotateAxis=(1, 0), anchor=(1, 1))
        self.plot_widget.addItem(line)
        self.marker_items.extend([line, text])

    def refresh_markers(self):
        self._remove_markers()
        if self.conn is None or self.x_axis_combo.currentData() != "elapsed":
            return

        if self.show_faults.isChecked() and self.table_exists(self.conn, "faults"):
            for elapsed, label in self.conn.execute("SELECT elapsed_s,label FROM faults ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), str(label), (255, 0, 0), width=3)

        if self.show_commands.isChecked() and self.table_exists(self.conn, "commands"):
            for elapsed, name in self.conn.execute("SELECT elapsed_s,command FROM commands ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), f"CMD {name}", (0, 170, 255), style=Qt.PenStyle.DashLine)

        if self.show_parameters.isChecked() and self.table_exists(self.conn, "parameters"):
            for elapsed, name in self.conn.execute("SELECT elapsed_s,name FROM parameters ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), f"PARAM {name}", (255, 170, 0), style=Qt.PenStyle.DotLine)

        if self.show_states.isChecked() and self.table_exists(self.conn, "states"):
            for elapsed, in self.conn.execute("SELECT elapsed_s FROM states ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), "STATE", (170, 0, 255), style=Qt.PenStyle.DashDotLine)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def load_events(self):
        self.events_table.setRowCount(0)
        if self.conn is None:
            return
        events = []

        if self.ev_fault.isChecked() and self.table_exists(self.conn, "faults"):
            for t, label in self.conn.execute("SELECT elapsed_s,label FROM faults"):
                events.append((t, "FAULT", label, "Manual fault marker"))

        if self.ev_logs.isChecked() and self.table_exists(self.conn, "logs"):
            for t, level, msg in self.conn.execute("SELECT elapsed_s,level,message FROM logs"):
                events.append((t, "LOG", level, msg))

        if self.ev_commands.isChecked() and self.table_exists(self.conn, "commands"):
            for t, cmd, args in self.conn.execute("SELECT elapsed_s,command,arguments_json FROM commands"):
                events.append((t, "COMMAND", cmd, args))

        if self.ev_parameters.isChecked() and self.table_exists(self.conn, "parameters"):
            for t, name, value in self.conn.execute("SELECT elapsed_s,name,value_json FROM parameters"):
                events.append((t, "PARAMETER", name, value))

        if self.ev_states.isChecked() and self.table_exists(self.conn, "states"):
            for t, state in self.conn.execute("SELECT elapsed_s,state_json FROM states"):
                events.append((t, "STATE", "Robot state", state))

        if self.ev_annotations.isChecked() and self.table_exists(self.conn, "annotations"):
            for t, label, note in self.conn.execute("SELECT elapsed_s,label,note FROM annotations"):
                events.append((t, "ANNOTATION", label, note))

        events.sort(key=lambda x: x[0])
        self.events_table.setRowCount(len(events))
        for row, event in enumerate(events):
            for col, value in enumerate(event):
                self.events_table.setItem(row, col, QTableWidgetItem(f"{value:.3f}" if col == 0 else str(value)))

    def jump_to_event(self, row: int, _col: int):
        item = self.events_table.item(row, 0)
        if item is None:
            return
        try:
            t = float(item.text())
        except ValueError:
            return
        self.tabs.setCurrentIndex(0)
        self.plot_widget.setXRange(t - 3.0, t + 3.0, padding=0)
        self.cursor_line.setPos(t)
        self.cursor_line.show()
        self.update_cursor_values()

    # ------------------------------------------------------------------
    # Anomalies
    # ------------------------------------------------------------------

    def clear_anomalies(self):
        for item in self.anomaly_items:
            try:
                self.plot_widget.removeItem(item)
            except Exception:
                pass
        self.anomaly_items.clear()

    def find_anomalies(self):
        self.clear_anomalies()
        if not self.plot_curves:
            return

        total = 0
        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            x, y = x[mask], s.values[mask]
            if len(y) < 8:
                continue

            median = np.median(y)
            mad = np.median(np.abs(y - median))
            if mad > 0:
                robust_z = 0.6745 * (y - median) / mad
                idx = np.where(np.abs(robust_z) > 6.0)[0]
            else:
                idx = np.array([], dtype=int)

            dy = np.diff(y)
            if len(dy) >= 5:
                dmed = np.median(dy)
                dmad = np.median(np.abs(dy - dmed))
                didx = np.where(np.abs(0.6745 * (dy - dmed) / dmad) > 8.0)[0] + 1 if dmad > 0 else np.array([], dtype=int)
                idx = np.unique(np.concatenate([idx, didx]))

            # Collapse markers closer than 50 ms.
            last = -1e99
            for i in idx:
                if x[i] - last < 0.05:
                    continue
                line = pg.InfiniteLine(pos=float(x[i]), angle=90, movable=False, pen=pg.mkPen((255, 0, 255), width=1))
                line.setZValue(700)
                self.plot_widget.addItem(line)
                self.anomaly_items.append(line)
                last = x[i]
                total += 1

        self.statusBar().showMessage(f"Anomaly scan: {total} candidate points marked in magenta")

    # ------------------------------------------------------------------
    # Raw / metadata
    # ------------------------------------------------------------------

    def load_raw(self):
        self.raw_table.setRowCount(0)
        if self.conn is None or not self.table_exists(self.conn, "raw_serial"):
            return
        rows = self.conn.execute("SELECT elapsed_s,wall_time,line FROM raw_serial ORDER BY elapsed_s").fetchall()
        self.raw_table.setRowCount(len(rows))
        for r, values in enumerate(rows):
            for c, value in enumerate(values):
                self.raw_table.setItem(r, c, QTableWidgetItem(f"{value:.3f}" if c == 0 else str(value)))

    def load_metadata(self):
        if self.conn is None:
            self.metadata_text.clear()
            return
        meta = self.get_metadata(self.conn)
        lines = ["SESSION METADATA", "================"]
        for key in sorted(meta):
            lines.append(f"{key}: {meta[key]}")

        if self.table_exists(self.conn, "parameter_snapshots"):
            row = self.conn.execute(
                "SELECT elapsed_s,snapshot_json FROM parameter_snapshots ORDER BY elapsed_s LIMIT 1"
            ).fetchone()
            if row:
                lines.extend(["", "PARAMETER SNAPSHOT", "==================", f"at {row[0]:.3f} s"])
                try:
                    snap = json.loads(row[1])
                    for key in sorted(snap):
                        lines.append(f"{key}: {snap[key]}")
                except Exception:
                    lines.append(str(row[1]))

        self.metadata_text.setPlainText("\n".join(lines))

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def export_selected_csv(self):
        if self.conn is None:
            return
        names = [i.text() for i in self.plotted_list.selectedItems()]
        if not names:
            names = list(self.plot_curves)
        if not names:
            QMessageBox.information(self, "Export", "Plot or select at least one signal first.")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Export telemetry CSV", str(DATA_DIR / "telemetry_export.csv"), "CSV (*.csv)"
        )
        if not filename:
            return

        rows = []
        for name in names:
            s = self._series(name, "main")
            for t, v in zip(s.elapsed, s.values):
                if np.isfinite(t) and np.isfinite(v):
                    rows.append((float(t), name, float(v)))
        rows.sort()

        with open(filename, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["elapsed_s", "signal", "value"])
            w.writerows(rows)

        self.statusBar().showMessage(f"Exported {len(rows)} rows to {filename}")

    def closeEvent(self, event):
        if self.conn is not None:
            self.conn.close()
        if self.compare_conn is not None:
            self.compare_conn.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = DataVisualiser()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
