"""Headless checks for the editable CPU wiring guide."""

import os
import json
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtCore import QSettings
from PyQt6.QtWidgets import QApplication

from DebugGUI import RobotDebugGUI
from WiringGuide import DEFAULT_ENTRIES, WiringGuide


class WiringGuideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_app_contains_wiring_tab(self):
        window = RobotDebugGUI()
        labels = [window.tabs.tabText(i) for i in range(window.tabs.count())]
        self.assertIn("Wiring Guide", labels)
        self.assertGreater(len(window.wiring_guide.entries), 0)
        window.close()

    def test_changes_persist_across_new_widget(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings_file = str(Path(temporary) / "wiring.ini")
            settings = QSettings(settings_file, QSettings.Format.IniFormat)
            first = WiringGuide(settings)
            first.kind.setCurrentText("Sensor")
            first.device.setText("Test encoder")
            first.connector.setText("Digital Raw 2")
            first.pins.setText("D40")
            first.notes.setText("Test note")
            first.add_entry()
            self.assertEqual(first.entries[-1]["device"], "Test encoder")

            first.table.item(0, 3).setText("D99")
            first.table.selectRow(len(first.entries) - 1)
            first.remove_selected()
            first.close()

            second = WiringGuide(QSettings(settings_file, QSettings.Format.IniFormat))
            self.assertEqual(second.entries[0]["pins"], "D99")
            self.assertEqual(len(second.entries), len(DEFAULT_ENTRIES))
            self.assertNotIn("Test encoder", [entry["device"] for entry in second.entries])
            second.close()

    def test_existing_guide_gets_encoder_and_xshut4_without_losing_edits(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings_file = str(Path(temporary) / "existing.ini")
            settings = QSettings(settings_file, QSettings.Format.IniFormat)
            settings.setValue(WiringGuide.SETTINGS_KEY, json.dumps([
                {"kind": "Other", "device": "Custom device", "connector": "Custom port",
                 "pins": "D99", "notes": "Keep this"}
            ]))
            guide = WiringGuide(settings)
            self.assertEqual(guide.entries[0]["device"], "Custom device")
            self.assertEqual(guide.entries[1]["device"], "Dual encoder board")
            self.assertEqual(guide.entries[2]["connector"], "XSHUT4")
            guide.close()


if __name__ == "__main__":
    unittest.main()
