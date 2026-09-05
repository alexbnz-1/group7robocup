"""
DataVisualiser.py

Offline analysis GUI for Robot Debug Console recordings (*.rdbg).

Features:
- recording picker
- searchable numeric signal list
- multi-signal plotting
- bright red manual fault markers as vertical lines
- chronological events table
- raw serial inspection
- session metadata
- CSV export

Dependencies:
    pip install PyQt6 pyqtgraph
"""

from __future__ import annotations

import csv
import json
import sqlite3
import sys
from pathlib import Path

import pyqtgraph as pg
from PyQt6.QtCore import Qt
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

        self.setWindowTitle(
            "Robot Data Visualiser"
        )

        self.resize(
            1500,
            900,
        )

        self.db_path: Path | None = None
        self.conn: sqlite3.Connection | None = None

        self.plot_curves = {}
        self.signal_data = {}

        # Manual fault markers shown on the plot.
        self.fault_lines = []
        self.fault_labels = []

        self._build_ui()
        self.refresh_recordings()

    # =================================================================
    # Helpers
    # =================================================================

    def table_exists(
        self,
        name: str,
    ) -> bool:
        if self.conn is None:
            return False

        row = self.conn.execute(
            """
            SELECT name
            FROM sqlite_master
            WHERE type = 'table'
              AND name = ?
            """,
            (name,),
        ).fetchone()

        return row is not None

    # =================================================================
    # UI
    # =================================================================

    def _build_ui(self):
        central = QWidget()

        self.setCentralWidget(
            central
        )

        root = QVBoxLayout(
            central
        )

        top = QHBoxLayout()

        top.addWidget(
            QLabel("Recording:")
        )

        self.file_combo = QComboBox()
        self.file_combo.setMinimumWidth(
            420
        )

        top.addWidget(
            self.file_combo
        )

        self.refresh_button = QPushButton(
            "Refresh"
        )

        self.open_button = QPushButton(
            "Open"
        )

        self.browse_button = QPushButton(
            "Browse..."
        )

        self.export_button = QPushButton(
            "Export Selected CSV"
        )

        top.addWidget(
            self.refresh_button
        )

        top.addWidget(
            self.open_button
        )

        top.addWidget(
            self.browse_button
        )

        top.addStretch()

        top.addWidget(
            self.export_button
        )

        root.addLayout(
            top
        )

        self.summary_label = QLabel(
            "No recording loaded."
        )

        root.addWidget(
            self.summary_label
        )

        self.tabs = QTabWidget()

        root.addWidget(
            self.tabs,
            1,
        )

        self._build_plot_tab()
        self._build_events_tab()
        self._build_raw_tab()
        self._build_metadata_tab()

        self.refresh_button.clicked.connect(
            self.refresh_recordings
        )

        self.open_button.clicked.connect(
            self.open_selected_recording
        )

        self.browse_button.clicked.connect(
            self.browse_recording
        )

        self.export_button.clicked.connect(
            self.export_selected_csv
        )

    # =================================================================
    # Plot tab
    # =================================================================

    def _build_plot_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        controls.addWidget(
            QLabel("Search signals:")
        )

        self.signal_filter = QLineEdit()

        self.signal_filter.setPlaceholderText(
            "e.g. drive, imu, battery"
        )

        controls.addWidget(
            self.signal_filter
        )

        self.add_selected_button = QPushButton(
            "Add Selected"
        )

        self.remove_selected_button = QPushButton(
            "Remove Plot"
        )

        self.clear_plots_button = QPushButton(
            "Clear Plots"
        )

        self.auto_scale_button = QPushButton(
            "Auto Scale"
        )

        controls.addWidget(
            self.add_selected_button
        )

        controls.addWidget(
            self.remove_selected_button
        )

        controls.addWidget(
            self.clear_plots_button
        )

        controls.addWidget(
            self.auto_scale_button
        )

        controls.addSpacing(
            15
        )

        self.show_faults_checkbox = QCheckBox(
            "Show fault markers"
        )

        self.show_faults_checkbox.setChecked(
            True
        )

        controls.addWidget(
            self.show_faults_checkbox
        )

        controls.addWidget(
            QLabel("X axis:")
        )

        self.x_axis_combo = QComboBox()

        self.x_axis_combo.addItem(
            "Elapsed time (s)",
            "elapsed",
        )

        self.x_axis_combo.addItem(
            "Robot time",
            "robot",
        )

        controls.addWidget(
            self.x_axis_combo
        )

        layout.addLayout(
            controls
        )

        splitter = QSplitter(
            Qt.Orientation.Horizontal
        )

        left = QWidget()

        left_layout = QVBoxLayout(
            left
        )

        left_layout.addWidget(
            QLabel(
                "Available numeric signals"
            )
        )

        self.signal_list = QListWidget()

        self.signal_list.setSelectionMode(
            QListWidget.SelectionMode.ExtendedSelection
        )

        left_layout.addWidget(
            self.signal_list
        )

        left_layout.addWidget(
            QLabel(
                "Currently plotted"
            )
        )

        self.plotted_list = QListWidget()

        left_layout.addWidget(
            self.plotted_list
        )

        left.setMaximumWidth(
            360
        )

        splitter.addWidget(
            left
        )

        self.plot_widget = pg.PlotWidget()

        self.plot_widget.showGrid(
            x=True,
            y=True,
            alpha=0.3,
        )

        self.plot_widget.setLabel(
            "bottom",
            "Elapsed time",
            units="s",
        )

        self.plot_widget.setLabel(
            "left",
            "Value",
        )

        self.plot_widget.addLegend()

        splitter.addWidget(
            self.plot_widget
        )

        splitter.setStretchFactor(
            1,
            1,
        )

        layout.addWidget(
            splitter,
            1,
        )

        self.tabs.addTab(
            page,
            "Plots",
        )

        self.signal_filter.textChanged.connect(
            self.apply_signal_filter
        )

        self.add_selected_button.clicked.connect(
            self.add_selected_signals
        )

        self.remove_selected_button.clicked.connect(
            self.remove_selected_plot
        )

        self.clear_plots_button.clicked.connect(
            self.clear_plots
        )

        self.auto_scale_button.clicked.connect(
            self.auto_scale_plots
        )

        self.x_axis_combo.currentIndexChanged.connect(
            self.reload_plots
        )

        self.show_faults_checkbox.stateChanged.connect(
            self.refresh_fault_markers
        )

    # =================================================================
    # Events tab
    # =================================================================

    def _build_events_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        controls.addWidget(
            QLabel("Show:")
        )

        self.show_fault_events = QCheckBox(
            "Fault markers"
        )

        self.show_fault_events.setChecked(
            True
        )

        self.show_logs = QCheckBox(
            "Logs"
        )

        self.show_logs.setChecked(
            True
        )

        self.show_commands = QCheckBox(
            "Commands"
        )

        self.show_commands.setChecked(
            True
        )

        self.show_parameters = QCheckBox(
            "Parameter changes"
        )

        self.show_parameters.setChecked(
            True
        )

        self.show_states = QCheckBox(
            "Robot states"
        )

        self.show_states.setChecked(
            True
        )

        for widget in (
            self.show_fault_events,
            self.show_logs,
            self.show_commands,
            self.show_parameters,
            self.show_states,
        ):
            controls.addWidget(
                widget
            )

            widget.stateChanged.connect(
                self.load_events
            )

        controls.addStretch()

        layout.addLayout(
            controls
        )

        self.events_table = QTableWidget(
            0,
            4,
        )

        self.events_table.setHorizontalHeaderLabels(
            [
                "Elapsed (s)",
                "Type",
                "Name / Level",
                "Details",
            ]
        )

        self.events_table.horizontalHeader().setStretchLastSection(
            True
        )

        self.events_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )

        layout.addWidget(
            self.events_table
        )

        self.tabs.addTab(
            page,
            "Events",
        )

    # =================================================================
    # Raw serial
    # =================================================================

    def _build_raw_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        self.raw_table = QTableWidget(
            0,
            3,
        )

        self.raw_table.setHorizontalHeaderLabels(
            [
                "Elapsed (s)",
                "Wall Time",
                "Serial Line",
            ]
        )

        self.raw_table.horizontalHeader().setStretchLastSection(
            True
        )

        self.raw_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )

        layout.addWidget(
            self.raw_table
        )

        self.tabs.addTab(
            page,
            "Raw Serial",
        )

    # =================================================================
    # Metadata
    # =================================================================

    def _build_metadata_tab(self):
        page = QWidget()

        layout = QFormLayout(
            page
        )

        self.metadata_layout = layout
        self.metadata_widgets = []

        self.tabs.addTab(
            page,
            "Session Info",
        )

    # =================================================================
    # Files
    # =================================================================

    def refresh_recordings(self):
        current = (
            self.file_combo.currentData()
        )

        self.file_combo.clear()

        files = sorted(
            DATA_DIR.glob("*.rdbg"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )

        for path in files:
            self.file_combo.addItem(
                path.name,
                str(path),
            )

        if current:
            index = (
                self.file_combo.findData(
                    current
                )
            )

            if index >= 0:
                self.file_combo.setCurrentIndex(
                    index
                )

    def open_selected_recording(self):
        path = (
            self.file_combo.currentData()
        )

        if path:
            self.load_recording(
                Path(path)
            )

    def browse_recording(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open Robot Recording",
            str(DATA_DIR),
            (
                "Robot Debug Recording (*.rdbg);;"
                "SQLite Database (*.sqlite *.db);;"
                "All Files (*)"
            ),
        )

        if filename:
            self.load_recording(
                Path(filename)
            )

    def load_recording(
        self,
        path: Path,
    ):
        try:
            if self.conn is not None:
                self.conn.close()

            self.conn = sqlite3.connect(
                path
            )

            self.db_path = path

            self.clear_plots()
            self.clear_fault_markers()
            self.signal_data.clear()

            self.load_metadata()
            self.load_signal_names()
            self.load_events()
            self.load_raw_serial()
            self.update_summary()
            self.refresh_fault_markers()

            self.statusBar().showMessage(
                f"Loaded {path.name}"
            )

        except Exception as exc:
            QMessageBox.critical(
                self,
                "Could not open recording",
                f"{path}\n\n{exc}",
            )

    # =================================================================
    # Metadata / summary
    # =================================================================

    def metadata(self) -> dict:
        if self.conn is None:
            return {}

        return dict(
            self.conn.execute(
                "SELECT key, value FROM metadata"
            ).fetchall()
        )

    def load_metadata(self):
        for label, widget in self.metadata_widgets:
            self.metadata_layout.removeWidget(
                label
            )

            self.metadata_layout.removeWidget(
                widget
            )

            label.deleteLater()
            widget.deleteLater()

        self.metadata_widgets.clear()

        for key, value in sorted(
            self.metadata().items()
        ):
            label = QLabel(
                key
            )

            value_label = QLabel(
                str(value)
            )

            value_label.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextSelectableByMouse
            )

            self.metadata_layout.addRow(
                label,
                value_label,
            )

            self.metadata_widgets.append(
                (
                    label,
                    value_label,
                )
            )

    def update_summary(self):
        if self.conn is None:
            self.summary_label.setText(
                "No recording loaded."
            )
            return

        metadata = self.metadata()

        samples = self.conn.execute(
            """
            SELECT COUNT(*)
            FROM telemetry
            """
        ).fetchone()[0]

        signals = self.conn.execute(
            """
            SELECT COUNT(DISTINCT signal)
            FROM telemetry
            """
        ).fetchone()[0]

        duration = self.conn.execute(
            """
            SELECT COALESCE(
                MAX(elapsed_s),
                0
            )
            FROM telemetry
            """
        ).fetchone()[0]

        faults = 0

        if self.table_exists(
            "faults"
        ):
            faults = self.conn.execute(
                """
                SELECT COUNT(*)
                FROM faults
                """
            ).fetchone()[0]

        session_name = (
            metadata.get(
                "session_name"
            )
            or self.db_path.stem
        )

        self.summary_label.setText(
            f"{session_name}"
            f"    |    {duration:.2f} s"
            f"    |    {signals} signals"
            f"    |    {samples:,} telemetry samples"
            f"    |    {faults} fault markers"
        )

    # =================================================================
    # Signals / plotting
    # =================================================================

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

        for signal, in rows:
            self.signal_list.addItem(
                signal
            )

        self.apply_signal_filter()

    def apply_signal_filter(self):
        text = (
            self.signal_filter.text()
            .strip()
            .lower()
        )

        for index in range(
            self.signal_list.count()
        ):
            item = (
                self.signal_list.item(
                    index
                )
            )

            item.setHidden(
                text
                not in item.text().lower()
            )

    def get_signal_data(
        self,
        signal: str,
    ):
        if self.conn is None:
            return [], [], []

        rows = self.conn.execute(
            """
            SELECT
                elapsed_s,
                robot_time,
                value_num
            FROM telemetry
            WHERE signal = ?
              AND value_num IS NOT NULL
            ORDER BY elapsed_s
            """,
            (signal,),
        ).fetchall()

        elapsed = [
            row[0]
            for row in rows
        ]

        robot = [
            row[1]
            for row in rows
        ]

        values = [
            row[2]
            for row in rows
        ]

        return (
            elapsed,
            robot,
            values,
        )

    def add_selected_signals(self):
        for item in (
            self.signal_list.selectedItems()
        ):
            signal = item.text()

            if signal in self.plot_curves:
                continue

            elapsed, robot, values = (
                self.get_signal_data(
                    signal
                )
            )

            self.signal_data[
                signal
            ] = (
                elapsed,
                robot,
                values,
            )

            x = self._choose_x(
                elapsed,
                robot,
            )

            curve = self.plot_widget.plot(
                x,
                values,
                name=signal,
            )

            self.plot_curves[
                signal
            ] = curve

            self.plotted_list.addItem(
                signal
            )

        self._update_x_label()
        self.refresh_fault_markers()

    def _choose_x(
        self,
        elapsed,
        robot,
    ):
        if (
            self.x_axis_combo.currentData()
            != "robot"
        ):
            return elapsed

        if (
            robot
            and all(
                value is not None
                for value in robot
            )
        ):
            first = robot[0]

            span = (
                max(robot)
                - min(robot)
                if len(robot) > 1
                else 0
            )

            if abs(span) > 1000:
                return [
                    (value - first)
                    / 1000.0
                    for value in robot
                ]

            return [
                value
                - first
                for value in robot
            ]

        return elapsed

    def _update_x_label(self):
        if (
            self.x_axis_combo.currentData()
            == "robot"
        ):
            self.plot_widget.setLabel(
                "bottom",
                "Relative robot time",
                units="s",
            )

        else:
            self.plot_widget.setLabel(
                "bottom",
                "Elapsed time",
                units="s",
            )

    def reload_plots(self):
        for signal, curve in (
            self.plot_curves.items()
        ):
            elapsed, robot, values = (
                self.signal_data[
                    signal
                ]
            )

            curve.setData(
                self._choose_x(
                    elapsed,
                    robot,
                ),
                values,
            )

        self._update_x_label()
        self.refresh_fault_markers()


    def auto_scale_plots(self):
        """
        Auto-range the plot so every currently visible signal fits on screen.

        This uses PyQtGraph's built-in auto-range behaviour and includes a
        small padding around the data. Fault marker InfiniteLines do not
        determine the Y range.
        """
        if not self.plot_curves:
            return

        self.plot_widget.enableAutoRange(
            axis=pg.ViewBox.XYAxes,
            enable=True,
        )

        self.plot_widget.autoRange(
            padding=0.05
        )

    def remove_selected_plot(self):
        for item in (
            self.plotted_list.selectedItems()
        ):
            signal = item.text()

            curve = self.plot_curves.pop(
                signal,
                None,
            )

            if curve is not None:
                self.plot_widget.removeItem(
                    curve
                )

            self.signal_data.pop(
                signal,
                None,
            )

            self.plotted_list.takeItem(
                self.plotted_list.row(
                    item
                )
            )

    def clear_plots(self):
        for curve in (
            self.plot_curves.values()
        ):
            self.plot_widget.removeItem(
                curve
            )

        self.plot_curves.clear()
        self.signal_data.clear()
        self.plotted_list.clear()

        # Fault lines remain conceptually independent of selected signals.
        self.refresh_fault_markers()

    # =================================================================
    # Fault markers
    # =================================================================

    def get_faults(self):
        if (
            self.conn is None
            or not self.table_exists(
                "faults"
            )
        ):
            return []

        return self.conn.execute(
            """
            SELECT
                elapsed_s,
                wall_time,
                label
            FROM faults
            ORDER BY elapsed_s
            """
        ).fetchall()

    def clear_fault_markers(self):
        for line in self.fault_lines:
            try:
                self.plot_widget.removeItem(
                    line
                )
            except Exception:
                pass

        for label in self.fault_labels:
            try:
                self.plot_widget.removeItem(
                    label
                )
            except Exception:
                pass

        self.fault_lines.clear()
        self.fault_labels.clear()

    def refresh_fault_markers(self):
        self.clear_fault_markers()

        if (
            self.conn is None
            or not self.show_faults_checkbox.isChecked()
        ):
            return

        # Fault positions are recorded in elapsed PC time.
        # If robot-time mode is selected, convert elapsed fault times to the
        # closest available relative robot time so the marker still aligns.
        for fault_number, (
            elapsed_s,
            wall_time,
            label_text,
        ) in enumerate(
            self.get_faults(),
            start=1,
        ):
            x_position = self.fault_x_position(
                elapsed_s
            )

            # Explicit bright red requested for manual fault markers.
            line = pg.InfiniteLine(
                pos=x_position,
                angle=90,
                movable=False,
                pen=pg.mkPen(
                    color=(255, 0, 0),
                    width=3,
                ),
            )

            self.plot_widget.addItem(
                line
            )

            self.fault_lines.append(
                line
            )

            label = pg.TextItem(
                text=(
                    f"FAULT {fault_number}"
                ),
                color=(255, 0, 0),
                anchor=(0, 1),
            )

            label.setPos(
                x_position,
                0,
            )

            self.plot_widget.addItem(
                label
            )

            self.fault_labels.append(
                label
            )

    def fault_x_position(
        self,
        elapsed_s: float,
    ) -> float:
        if (
            self.x_axis_combo.currentData()
            != "robot"
        ):
            return float(
                elapsed_s
            )

        if self.conn is None:
            return float(
                elapsed_s
            )

        # Find the telemetry sample nearest to the manual fault.
        row = self.conn.execute(
            """
            SELECT
                robot_time
            FROM telemetry
            WHERE robot_time IS NOT NULL
            ORDER BY ABS(
                elapsed_s - ?
            )
            LIMIT 1
            """,
            (elapsed_s,),
        ).fetchone()

        first = self.conn.execute(
            """
            SELECT
                robot_time
            FROM telemetry
            WHERE robot_time IS NOT NULL
            ORDER BY elapsed_s
            LIMIT 1
            """
        ).fetchone()

        last = self.conn.execute(
            """
            SELECT
                robot_time
            FROM telemetry
            WHERE robot_time IS NOT NULL
            ORDER BY elapsed_s DESC
            LIMIT 1
            """
        ).fetchone()

        if (
            row is None
            or first is None
            or row[0] is None
            or first[0] is None
        ):
            return float(
                elapsed_s
            )

        robot_time = float(
            row[0]
        )

        first_robot_time = float(
            first[0]
        )

        span = 0.0

        if (
            last is not None
            and last[0] is not None
        ):
            span = (
                float(last[0])
                - first_robot_time
            )

        if abs(span) > 1000:
            return (
                robot_time
                - first_robot_time
            ) / 1000.0

        return (
            robot_time
            - first_robot_time
        )

    # =================================================================
    # Events
    # =================================================================

    def load_events(self):
        self.events_table.setRowCount(
            0
        )

        if self.conn is None:
            return

        events = []

        if (
            self.show_fault_events.isChecked()
            and self.table_exists(
                "faults"
            )
        ):
            for (
                elapsed,
                label,
            ) in self.conn.execute(
                """
                SELECT
                    elapsed_s,
                    label
                FROM faults
                """
            ):
                events.append(
                    (
                        elapsed,
                        "FAULT",
                        label,
                        "Manual fault marker",
                    )
                )

        if self.show_logs.isChecked():
            for (
                elapsed,
                level,
                message,
            ) in self.conn.execute(
                """
                SELECT
                    elapsed_s,
                    level,
                    message
                FROM logs
                """
            ):
                events.append(
                    (
                        elapsed,
                        "Log",
                        level,
                        message,
                    )
                )

        if self.show_commands.isChecked():
            for (
                elapsed,
                command,
                arguments,
            ) in self.conn.execute(
                """
                SELECT
                    elapsed_s,
                    command,
                    arguments_json
                FROM commands
                """
            ):
                events.append(
                    (
                        elapsed,
                        "Command",
                        command,
                        arguments,
                    )
                )

        if self.show_parameters.isChecked():
            for (
                elapsed,
                name,
                value,
            ) in self.conn.execute(
                """
                SELECT
                    elapsed_s,
                    name,
                    value_json
                FROM parameters
                """
            ):
                events.append(
                    (
                        elapsed,
                        "Parameter",
                        name,
                        value,
                    )
                )

        if self.show_states.isChecked():
            for (
                elapsed,
                state_json,
            ) in self.conn.execute(
                """
                SELECT
                    elapsed_s,
                    state_json
                FROM states
                """
            ):
                events.append(
                    (
                        elapsed,
                        "State",
                        "",
                        state_json,
                    )
                )

        events.sort(
            key=lambda item: item[0]
        )

        self.events_table.setRowCount(
            len(events)
        )

        for row, event in enumerate(
            events
        ):
            display = (
                f"{event[0]:.3f}",
                event[1],
                event[2],
                event[3],
            )

            for column, value in enumerate(
                display
            ):
                self.events_table.setItem(
                    row,
                    column,
                    QTableWidgetItem(
                        str(value)
                    ),
                )

    # =================================================================
    # Raw serial
    # =================================================================

    def load_raw_serial(self):
        self.raw_table.setRowCount(
            0
        )

        if self.conn is None:
            return

        rows = self.conn.execute(
            """
            SELECT
                elapsed_s,
                wall_time,
                line
            FROM raw_serial
            ORDER BY elapsed_s
            """
        ).fetchall()

        self.raw_table.setRowCount(
            len(rows)
        )

        for row, values in enumerate(
            rows
        ):
            display = (
                f"{values[0]:.3f}",
                values[1],
                values[2],
            )

            for column, value in enumerate(
                display
            ):
                self.raw_table.setItem(
                    row,
                    column,
                    QTableWidgetItem(
                        str(value)
                    ),
                )

    # =================================================================
    # Export
    # =================================================================

    def export_selected_csv(self):
        if self.conn is None:
            QMessageBox.information(
                self,
                "No recording",
                "Open a recording first.",
            )
            return

        selected = [
            item.text()
            for item in (
                self.signal_list.selectedItems()
            )
        ]

        if not selected:
            selected = list(
                self.plot_curves.keys()
            )

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
            str(
                self.db_path.with_suffix(
                    ".csv"
                )
            ),
            "CSV Files (*.csv)",
        )

        if not filename:
            return

        placeholders = ",".join(
            "?"
            for _ in selected
        )

        rows = self.conn.execute(
            f"""
            SELECT
                elapsed_s,
                wall_time,
                robot_time,
                signal,
                value_num,
                value_text,
                value_type
            FROM telemetry
            WHERE signal IN (
                {placeholders}
            )
            ORDER BY
                elapsed_s,
                signal
            """,
            selected,
        ).fetchall()

        with open(
            filename,
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.writer(
                file
            )

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

            writer.writerows(
                rows
            )

        self.statusBar().showMessage(
            f"Exported {len(rows):,} rows to {filename}"
        )

    # =================================================================
    # Shutdown
    # =================================================================

    def closeEvent(
        self,
        event,
    ):
        if self.conn is not None:
            self.conn.close()

        event.accept()


def main():
    app = QApplication(
        sys.argv
    )

    app.setApplicationName(
        "Robot Data Visualiser"
    )

    window = DataVisualiser()

    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()
