"""
DataVisualiser.py

Offline analysis suite for Robot Debug Console recording files (*.rdbg).

Install:
    pip install PyQt6 pyqtgraph

Run:
    python DataVisualiser.py
"""

from __future__ import annotations

import csv
import json
import sqlite3
import sys
from pathlib import Path

import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / "Data"
DATA_DIR.mkdir(parents=True, exist_ok=True)


class DataVisualiser(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Data Visualiser")
        self.resize(1500, 900)

        self.db_path: Path | None = None
        self.conn: sqlite3.Connection | None = None
        self.plot_curves = {}
        self.signal_data = {}

        self._build_ui()
        self.refresh_recordings()

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
        self.file_combo.setMinimumWidth(420)
        top.addWidget(self.file_combo)

        self.refresh_button = QPushButton("Refresh")
        self.open_button = QPushButton("Open")
        self.browse_button = QPushButton("Browse...")
        self.export_button = QPushButton("Export Selected CSV")

        top.addWidget(self.refresh_button)
        top.addWidget(self.open_button)
        top.addWidget(self.browse_button)
        top.addStretch()
        top.addWidget(self.export_button)

        root.addLayout(top)

        self.summary_label = QLabel("No recording loaded.")
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
        self.export_button.clicked.connect(self.export_selected_csv)

    def _build_plot_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Search signals:"))
        self.signal_filter = QLineEdit()
        self.signal_filter.setPlaceholderText("e.g. drive, imu, battery")
        controls.addWidget(self.signal_filter)

        self.add_selected_button = QPushButton("Add Selected")
        self.remove_selected_button = QPushButton("Remove Plot")
        self.clear_plots_button = QPushButton("Clear Plots")

        controls.addWidget(self.add_selected_button)
        controls.addWidget(self.remove_selected_button)
        controls.addWidget(self.clear_plots_button)

        controls.addWidget(QLabel("X axis:"))
        self.x_axis_combo = QComboBox()
        self.x_axis_combo.addItem("Elapsed time (s)", "elapsed")
        self.x_axis_combo.addItem("Robot time", "robot")
        controls.addWidget(self.x_axis_combo)

        layout.addLayout(controls)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(QLabel("Available numeric signals"))
        self.signal_list = QListWidget()
        self.signal_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        left_layout.addWidget(self.signal_list)

        left_layout.addWidget(QLabel("Currently plotted"))
        self.plotted_list = QListWidget()
        left_layout.addWidget(self.plotted_list)
        left.setMaximumWidth(360)

        splitter.addWidget(left)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "Elapsed time", units="s")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.addLegend()

        splitter.addWidget(self.plot_widget)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, 1)

        self.tabs.addTab(page, "Plots")

        self.signal_filter.textChanged.connect(self.apply_signal_filter)
        self.add_selected_button.clicked.connect(self.add_selected_signals)
        self.remove_selected_button.clicked.connect(self.remove_selected_plot)
        self.clear_plots_button.clicked.connect(self.clear_plots)
        self.x_axis_combo.currentIndexChanged.connect(self.reload_plots)

    def _build_events_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Show:"))

        self.show_logs = QCheckBox("Logs")
        self.show_logs.setChecked(True)
        self.show_commands = QCheckBox("Commands")
        self.show_commands.setChecked(True)
        self.show_parameters = QCheckBox("Parameter changes")
        self.show_parameters.setChecked(True)
        self.show_states = QCheckBox("Robot states")
        self.show_states.setChecked(True)

        for w in (
            self.show_logs,
            self.show_commands,
            self.show_parameters,
            self.show_states,
        ):
            controls.addWidget(w)
            w.stateChanged.connect(self.load_events)

        controls.addStretch()
        layout.addLayout(controls)

        self.events_table = QTableWidget(0, 4)
        self.events_table.setHorizontalHeaderLabels(
            ["Elapsed (s)", "Type", "Name / Level", "Details"]
        )
        self.events_table.horizontalHeader().setStretchLastSection(True)
        self.events_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        layout.addWidget(self.events_table)

        self.tabs.addTab(page, "Events")

    def _build_raw_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["Elapsed (s)", "Wall Time", "Serial Line"])
        self.raw_table.horizontalHeader().setStretchLastSection(True)
        self.raw_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        layout.addWidget(self.raw_table)

        self.tabs.addTab(page, "Raw Serial")

    def _build_metadata_tab(self):
        page = QWidget()
        layout = QFormLayout(page)
        self.metadata_layout = layout
        self.metadata_widgets = []
        self.tabs.addTab(page, "Session Info")

    # ------------------------------------------------------------------
    # File handling
    # ------------------------------------------------------------------

    def refresh_recordings(self):
        current = self.file_combo.currentData()
        self.file_combo.clear()

        files = sorted(
            DATA_DIR.glob("*.rdbg"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )

        for path in files:
            self.file_combo.addItem(path.name, str(path))

        if current:
            index = self.file_combo.findData(current)
            if index >= 0:
                self.file_combo.setCurrentIndex(index)

    def open_selected_recording(self):
        path = self.file_combo.currentData()
        if path:
            self.load_recording(Path(path))

    def browse_recording(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open Robot Recording",
            str(DATA_DIR),
            "Robot Debug Recording (*.rdbg);;SQLite Database (*.sqlite *.db);;All Files (*)",
        )
        if filename:
            self.load_recording(Path(filename))

    def load_recording(self, path: Path):
        try:
            if self.conn is not None:
                self.conn.close()

            self.conn = sqlite3.connect(path)
            self.db_path = path

            self.clear_plots()
            self.signal_data.clear()

            self.load_metadata()
            self.load_signal_names()
            self.load_events()
            self.load_raw_serial()
            self.update_summary()

            self.statusBar().showMessage(f"Loaded {path.name}")

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Could not open recording",
                f"{path}\n\n{exc}",
            )

    # ------------------------------------------------------------------
    # Metadata / summary
    # ------------------------------------------------------------------

    def metadata(self) -> dict:
        if self.conn is None:
            return {}
        return dict(self.conn.execute("SELECT key, value FROM metadata").fetchall())

    def load_metadata(self):
        for label, widget in self.metadata_widgets:
            self.metadata_layout.removeWidget(label)
            self.metadata_layout.removeWidget(widget)
            label.deleteLater()
            widget.deleteLater()
        self.metadata_widgets.clear()

        for key, value in sorted(self.metadata().items()):
            label = QLabel(key)
            value_label = QLabel(str(value))
            value_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            self.metadata_layout.addRow(label, value_label)
            self.metadata_widgets.append((label, value_label))

    def update_summary(self):
        if self.conn is None:
            self.summary_label.setText("No recording loaded.")
            return

        metadata = self.metadata()
        samples = self.conn.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        signals = self.conn.execute(
            "SELECT COUNT(DISTINCT signal) FROM telemetry"
        ).fetchone()[0]
        duration = self.conn.execute(
            "SELECT COALESCE(MAX(elapsed_s), 0) FROM telemetry"
        ).fetchone()[0]

        session_name = metadata.get("session_name") or self.db_path.stem

        self.summary_label.setText(
            f"{session_name}    |    "
            f"{duration:.2f} s    |    "
            f"{signals} signals    |    "
            f"{samples:,} telemetry samples"
        )

    # ------------------------------------------------------------------
    # Signals / plotting
    # ------------------------------------------------------------------

    def load_signal_names(self):
        self.signal_list.clear()

        if self.conn is None:
            return

        rows = self.conn.execute(
            """
            SELECT DISTINCT signal
            FROM telemetry
            WHERE value_num IS NOT NULL
            ORDER BY signal
            """
        ).fetchall()

        for (signal,) in rows:
            self.signal_list.addItem(signal)

        self.apply_signal_filter()

    def apply_signal_filter(self):
        text = self.signal_filter.text().strip().lower()
        for i in range(self.signal_list.count()):
            item = self.signal_list.item(i)
            item.setHidden(text not in item.text().lower())

    def get_signal_data(self, signal: str):
        if self.conn is None:
            return [], [], []

        rows = self.conn.execute(
            """
            SELECT elapsed_s, robot_time, value_num
            FROM telemetry
            WHERE signal = ? AND value_num IS NOT NULL
            ORDER BY elapsed_s
            """,
            (signal,),
        ).fetchall()

        elapsed = [r[0] for r in rows]
        robot = [r[1] for r in rows]
        values = [r[2] for r in rows]
        return elapsed, robot, values

    def add_selected_signals(self):
        for item in self.signal_list.selectedItems():
            signal = item.text()
            if signal in self.plot_curves:
                continue

            elapsed, robot, values = self.get_signal_data(signal)
            self.signal_data[signal] = (elapsed, robot, values)

            x = self._choose_x(elapsed, robot)
            curve = self.plot_widget.plot(x, values, name=signal)
            self.plot_curves[signal] = curve
            self.plotted_list.addItem(signal)

        self._update_x_label()

    def _choose_x(self, elapsed, robot):
        if self.x_axis_combo.currentData() != "robot":
            return elapsed

        if robot and all(v is not None for v in robot):
            first = robot[0]
            # Robot timestamps are often milliseconds. Display relative robot time in seconds.
            span = max(robot) - min(robot) if len(robot) > 1 else 0
            if abs(span) > 1000:
                return [(v - first) / 1000.0 for v in robot]
            return [v - first for v in robot]

        return elapsed

    def _update_x_label(self):
        if self.x_axis_combo.currentData() == "robot":
            self.plot_widget.setLabel("bottom", "Relative robot time", units="s")
        else:
            self.plot_widget.setLabel("bottom", "Elapsed time", units="s")

    def reload_plots(self):
        for signal, curve in self.plot_curves.items():
            elapsed, robot, values = self.signal_data[signal]
            curve.setData(self._choose_x(elapsed, robot), values)
        self._update_x_label()

    def remove_selected_plot(self):
        for item in self.plotted_list.selectedItems():
            signal = item.text()
            curve = self.plot_curves.pop(signal, None)
            if curve is not None:
                self.plot_widget.removeItem(curve)
            self.signal_data.pop(signal, None)
            self.plotted_list.takeItem(self.plotted_list.row(item))

    def clear_plots(self):
        for curve in self.plot_curves.values():
            self.plot_widget.removeItem(curve)
        self.plot_curves.clear()
        self.signal_data.clear()
        self.plotted_list.clear()

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def load_events(self):
        self.events_table.setRowCount(0)
        if self.conn is None:
            return

        events = []

        if self.show_logs.isChecked():
            for elapsed, level, message in self.conn.execute(
                "SELECT elapsed_s, level, message FROM logs"
            ):
                events.append((elapsed, "Log", level, message))

        if self.show_commands.isChecked():
            for elapsed, command, args in self.conn.execute(
                "SELECT elapsed_s, command, arguments_json FROM commands"
            ):
                events.append((elapsed, "Command", command, args))

        if self.show_parameters.isChecked():
            for elapsed, name, value in self.conn.execute(
                "SELECT elapsed_s, name, value_json FROM parameters"
            ):
                events.append((elapsed, "Parameter", name, value))

        if self.show_states.isChecked():
            for elapsed, state_json in self.conn.execute(
                "SELECT elapsed_s, state_json FROM states"
            ):
                events.append((elapsed, "State", "", state_json))

        events.sort(key=lambda x: x[0])

        self.events_table.setRowCount(len(events))
        for row, event in enumerate(events):
            for col, value in enumerate(
                (f"{event[0]:.3f}", event[1], event[2], event[3])
            ):
                self.events_table.setItem(row, col, QTableWidgetItem(str(value)))

    def load_raw_serial(self):
        self.raw_table.setRowCount(0)
        if self.conn is None:
            return

        rows = self.conn.execute(
            "SELECT elapsed_s, wall_time, line FROM raw_serial ORDER BY elapsed_s"
        ).fetchall()

        self.raw_table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            display = (f"{values[0]:.3f}", values[1], values[2])
            for col, value in enumerate(display):
                self.raw_table.setItem(row, col, QTableWidgetItem(str(value)))

    # ------------------------------------------------------------------
    # CSV export
    # ------------------------------------------------------------------

    def export_selected_csv(self):
        if self.conn is None:
            QMessageBox.information(self, "No recording", "Open a recording first.")
            return

        selected = [item.text() for item in self.signal_list.selectedItems()]
        if not selected:
            selected = list(self.plot_curves.keys())

        if not selected:
            QMessageBox.information(
                self,
                "No signals selected",
                "Select one or more signals in the signal list first.",
            )
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export Telemetry CSV",
            str(self.db_path.with_suffix(".csv")),
            "CSV Files (*.csv)",
        )
        if not filename:
            return

        placeholders = ",".join("?" for _ in selected)
        rows = self.conn.execute(
            f"""
            SELECT elapsed_s, wall_time, robot_time, signal,
                   value_num, value_text, value_type
            FROM telemetry
            WHERE signal IN ({placeholders})
            ORDER BY elapsed_s, signal
            """,
            selected,
        ).fetchall()

        with open(filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "elapsed_s",
                    "wall_time",
                    "robot_time",
                    "signal",
                    "value_num",
                    "value_text",
                    "value_type",
                ]
            )
            writer.writerows(rows)

        self.statusBar().showMessage(f"Exported {len(rows):,} rows to {filename}")

    def closeEvent(self, event):
        if self.conn is not None:
            self.conn.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Data Visualiser")
    window = DataVisualiser()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
