"""Editable, locally persisted CPU-side wiring guide for the debug console.

This is a documentation view. Editing it never changes Teensy pin assignments.
"""

from __future__ import annotations

import json

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QPen
from PyQt6.QtWidgets import (
    QComboBox,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


DEFAULT_ENTRIES = [
    {"kind": "Comms", "device": "Bluetooth CH9143", "connector": "SERIAL1", "pins": "RX1 D0 / TX1 D1", "notes": "Wireless debug link"},
    {"kind": "Servo", "device": "Herkulex bus", "connector": "SERIAL2", "pins": "Serial2 TX/RX", "notes": "Smart servos by ID"},
    {"kind": "Motor driver", "device": "203 driver 1 — A", "connector": "Digital", "pins": "D27", "notes": "Servo pulse; 1500 µs neutral"},
    {"kind": "Motor driver", "device": "203 driver 1 — B", "connector": "Digital", "pins": "D26", "notes": "Servo pulse; 1500 µs neutral"},
    {"kind": "Motor driver", "device": "203 driver 2 — A", "connector": "Digital", "pins": "D25", "notes": "Servo pulse; 1500 µs neutral"},
    {"kind": "Motor driver", "device": "203 driver 2 — B", "connector": "Digital", "pins": "D15 / A1", "notes": "Conflicts with planned IR_RIGHT input"},
    {"kind": "Servo", "device": "HX12K A", "connector": "Digital Raw 1 / level shift", "pins": "D33", "notes": "Level-shifted servo output"},
    {"kind": "Servo", "device": "HX12K B", "connector": "Digital Raw 1 / level shift", "pins": "D32", "notes": "Level-shifted servo output"},
    {"kind": "Servo", "device": "HX12K C", "connector": "Digital Raw 1 / level shift", "pins": "D31", "notes": "Level-shifted servo output"},
    {"kind": "Servo", "device": "HX12K D", "connector": "Digital Raw 1 / level shift", "pins": "D30", "notes": "Level-shifted servo output"},
    {"kind": "Sensor", "device": "Dual encoder board", "connector": "Digital Raw 2", "pins": "E1 A:D2 B:D3 / E2 A:D4 B:D5", "notes": "Quadrature; 3.3 V signals only"},
    {"kind": "Sensor", "device": "TOF Long Top Mid Left", "connector": "XSHUT1", "pins": "I2C0 / XSHUT1", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Long Top Mid Right", "connector": "XSHUT2", "pins": "I2C0 / XSHUT2", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Mid Left", "connector": "XSHUT5", "pins": "I2C0 / XSHUT5", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Mid Right", "connector": "XSHUT0", "pins": "I2C0 / XSHUT0", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Right Right", "connector": "XSHUT3", "pins": "I2C0 / XSHUT3", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "SEN0628 8×8 TOF", "connector": "Raw I2C1", "pins": "3V / G / SC / SD", "notes": "Wire1, I2C address 0x33"},
    {"kind": "Sensor", "device": "BNO055 IMU", "connector": "Raw I2C1", "pins": "3V / G / SC / SD", "notes": "Wire1, I2C address 0x28/0x29"},
]

COLUMNS = ("Type", "Device", "CPU connector", "CPU pin(s)", "Notes")
FIELDS = ("kind", "device", "connector", "pins", "notes")
COLORS = {
    "Comms": QColor("#67b7f7"),
    "Motor": QColor("#f6a768"),
    "Motor driver": QColor("#f6a768"),
    "Servo": QColor("#b39df5"),
    "Sensor": QColor("#7dd6ab"),
}


class WiringGuide(QWidget):
    SETTINGS_KEY = "wiring_guide/entries_v1"
    entries_changed = pyqtSignal()

    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.entries = self._load_entries()
        self._populating = False

        layout = QVBoxLayout(self)
        heading = QLabel("CPU wiring guide")
        heading.setStyleSheet("font-size: 20px; font-weight: 700;")
        layout.addWidget(heading)
        layout.addWidget(QLabel(
            "Edit the table or add a device below. Changes save automatically on this computer. "
            "This guide does not change Teensy firmware pin assignments."
        ))

        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter, 1)

        self.scene = QGraphicsScene(self)
        self.diagram = QGraphicsView(self.scene)
        self.diagram.setRenderHints(self.diagram.renderHints())
        self.diagram.setMinimumHeight(250)
        splitter.addWidget(self.diagram)

        table_panel = QWidget()
        table_layout = QVBoxLayout(table_panel)
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.itemChanged.connect(self._table_changed)
        table_layout.addWidget(self.table, 1)

        form = QHBoxLayout()
        self.kind = QComboBox()
        self.kind.setEditable(True)
        self.kind.addItems(("Sensor", "Motor", "Motor driver", "Servo", "Comms", "Other"))
        self.device = QLineEdit()
        self.device.setPlaceholderText("Device name")
        self.connector = QLineEdit()
        self.connector.setPlaceholderText("CPU connector")
        self.pins = QLineEdit()
        self.pins.setPlaceholderText("CPU pin(s)")
        self.notes = QLineEdit()
        self.notes.setPlaceholderText("Notes / address")
        for widget in (self.kind, self.device, self.connector, self.pins, self.notes):
            form.addWidget(widget)
        table_layout.addLayout(form)

        buttons = QHBoxLayout()
        self.add_button = QPushButton("Add device")
        self.remove_button = QPushButton("Remove selected")
        self.add_button.clicked.connect(self.add_entry)
        self.remove_button.clicked.connect(self.remove_selected)
        buttons.addWidget(self.add_button)
        buttons.addWidget(self.remove_button)
        buttons.addStretch()
        table_layout.addLayout(buttons)
        splitter.addWidget(table_panel)
        splitter.setSizes((420, 360))

        self._refresh()

    def _load_entries(self):
        raw = self.settings.value(self.SETTINGS_KEY)
        if raw is None:
            return [entry.copy() for entry in DEFAULT_ENTRIES]
        try:
            entries = json.loads(str(raw))
            if not isinstance(entries, list):
                raise ValueError("Not a list")
            cleaned = [
                {field: str(entry.get(field, "")) for field in FIELDS}
                for entry in entries if isinstance(entry, dict)
            ]
            # Existing installations already have a saved guide; add the newly
            # confirmed encoder connection once, without replacing user edits.
            if not self.settings.value("wiring_guide/encoder_raw2_migrated", False, type=bool):
                if not any("Digital Raw 2" in entry["connector"] or
                           "encoder" in entry["device"].lower() for entry in cleaned):
                    cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                        if entry["device"] == "Dual encoder board"))
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/encoder_raw2_migrated", True)
                self.settings.sync()
            return cleaned
        except (TypeError, ValueError):
            return [entry.copy() for entry in DEFAULT_ENTRIES]

    def _save(self):
        self.settings.setValue(self.SETTINGS_KEY, json.dumps(self.entries, ensure_ascii=False))
        self.settings.sync()
        self.entries_changed.emit()

    def _refresh(self):
        self._populating = True
        self.table.setRowCount(len(self.entries))
        for row, entry in enumerate(self.entries):
            for column, field in enumerate(FIELDS):
                self.table.setItem(row, column, QTableWidgetItem(entry[field]))
        self._populating = False
        self.table.resizeColumnsToContents()
        self._draw_diagram()

    def _table_changed(self, item):
        if self._populating or item.row() >= len(self.entries):
            return
        self.entries[item.row()][FIELDS[item.column()]] = item.text().strip()
        self._save()
        self._draw_diagram()

    def add_entry(self):
        entry = {
            "kind": self.kind.currentText().strip() or "Other",
            "device": self.device.text().strip(),
            "connector": self.connector.text().strip(),
            "pins": self.pins.text().strip(),
            "notes": self.notes.text().strip(),
        }
        if not entry["device"] or not entry["pins"]:
            self.device.setFocus() if not entry["device"] else self.pins.setFocus()
            return
        self.entries.append(entry)
        self._save()
        self._refresh()
        self.table.selectRow(len(self.entries) - 1)
        for edit in (self.device, self.connector, self.pins, self.notes):
            edit.clear()

    def remove_selected(self):
        row = self.table.currentRow()
        if row < 0 or row >= len(self.entries):
            return
        del self.entries[row]
        self._save()
        self._refresh()

    def _draw_diagram(self):
        self.scene.clear()
        row_height = 72
        top = 80
        height = max(280, top + len(self.entries) * row_height + 30)
        self.scene.setSceneRect(0, 0, 1100, height)
        self.scene.setBackgroundBrush(QBrush(QColor("#202327")))

        cpu_brush = QBrush(QColor("#35465c"))
        outline = QPen(QColor("#8093a7"), 2)
        self.scene.addRect(35, 24, 360, 48, outline, cpu_brush)
        self._label("TEENSY / CPU BOARD", 52, 36, QColor("#ffffff"), bold=True)
        self._label("CPU CONNECTOR / PIN", 52, 84, QColor("#bfcbd8"), bold=True)
        self._label("CONNECTED DEVICE", 595, 84, QColor("#bfcbd8"), bold=True)

        if not self.entries:
            self._label("No wiring entries yet — add one below.", 52, 145, QColor("#bfcbd8"))
            return

        for index, entry in enumerate(self.entries):
            y = top + 30 + index * row_height
            color = COLORS.get(entry["kind"], QColor("#d6dce4"))
            connector = entry["connector"] or "Unspecified connector"
            pins = entry["pins"] or "Unspecified pins"
            device = entry["device"] or "Unnamed device"
            self.scene.addRect(35, y, 360, 54, outline, cpu_brush)
            self._label(connector, 50, y + 3, QColor("#ffffff"), bold=True)
            self._label(pins, 50, y + 27, QColor("#d5e2f0"))
            self.scene.addLine(395, y + 27, 585, y + 27, QPen(color, 3))
            self.scene.addEllipse(486, y + 22, 10, 10, QPen(color), QBrush(color))
            self.scene.addRect(585, y, 480, 54, QPen(color, 2), QBrush(QColor("#2d3238")))
            self._label(f"{device}  ·  {entry['kind']}", 600, y + 3, color, bold=True)
            self._label(entry["notes"], 600, y + 27, QColor("#d6dce4"))

    def _label(self, text, x, y, color, bold=False):
        item = self.scene.addText(text)
        item.setDefaultTextColor(color)
        font = item.font()
        font.setBold(bold)
        item.setFont(font)
        item.setPos(x, y)
