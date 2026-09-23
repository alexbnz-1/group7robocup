"""Focused, hardware-free checks for pre-laid mission upload and detours."""

import json
import tempfile
import unittest
from pathlib import Path

from PyQt6.QtCore import QSettings
from PyQt6.QtWidgets import QApplication

from ArenaView import ArenaView, MissionLayout


class MissionUploadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_map_contains_forbidden_home_and_physical_landmarks(self):
        layout = MissionLayout()
        layout.add_obstacle("wall", 1800, 1000)
        layout.add_weight(2600, 1200, dummy=True)
        features = layout.map_payload()["features"]
        self.assertEqual(len(features), 15)
        self.assertEqual(features[:5], [0, 1750, 650, 2400, 0])
        self.assertEqual(features[9], 1)
        self.assertEqual(features[14], 1)

    def test_run_uploads_map_then_route_then_start(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.navigation_strategy.setCurrentIndex(3)
            view.mission_layout.add_weight(1500, 900)
            self.assertTrue(view.mission_layout.plan())
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view._run_navigation_with_tuning()
            self.assertEqual([name for name, _ in sent],
                             ["mission_map_set", "mission_plan_set",
                              "autonomous_navigation"])
            for name, data in sent[:2]:
                encoded = json.dumps({"type": "command", "command": name, **data},
                                     separators=(",", ":")).encode("utf-8")
                self.assertLess(len(encoded), 760)
            view.close()

    def test_wall_consensus_tuning_is_sent_to_firmware(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            requested = []
            view.parameter_requested.connect(
                lambda name, value: requested.append((name, value)))
            view.navigation_tuning_controls["matrix_wall_mm"].setValue(375)
            view._apply_navigation_tuning()
            self.assertIn(("navigation.matrix_wall_mm", 375), requested)
            view.close()

    def test_return_home_sends_clear_route_and_stop_at_home_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest.update({
                "mission.pose_x_mm": 1000.0,
                "mission.pose_y_mm": 1000.0,
                "mission.heading_deg": 0.0,
                "imu.valid": True,
                "system.stopped": False,
            })
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view._return_home()
            self.assertEqual([name for name, _ in sent], [
                "autonomous_navigation", "mission_map_set",
                "mission_plan_set", "autonomous_navigation"])
            self.assertFalse(sent[0][1]["enabled"])
            self.assertTrue(sent[2][1]["return_home"])
            self.assertTrue(sent[-1][1]["enabled"])
            self.assertEqual(sent[2][1]["points"][-1], 1)
            self.assertEqual(len(sent[1][1]["bottom_sensors"]), 12)
            self.assertEqual(len(sent[1][1]["ultrasound_offsets"]), 4)
            view.close()

    def test_oversized_map_does_not_start_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.navigation_strategy.setCurrentIndex(3)
            view.mission_layout.add_weight(1500, 900)
            self.assertTrue(view.mission_layout.plan())
            for index in range(24):
                view.mission_layout.add_obstacle("tube", 800 + index * 140, 1800)
            # Keep the already-planned route to exercise upload validation.
            view.mission_layout.route = [{"x": 1500, "y": 900, "target": True}]
            sent = []
            view.command_requested.connect(lambda name, data: sent.append(name))
            view._run_navigation_with_tuning()
            self.assertNotIn("autonomous_navigation", sent)
            self.assertIn("exceeds", view.mission_status.text())
            view.close()

    def test_weight_visibility_distinguishes_target_and_height_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest.update({
                "weight.detected": False,
                "weight.geometry_valid": True,
                "weight.valid_mask": 1,
                "weight.gap_mask": 1,
                "weight.target_mask": 0,
                "weight.map_wall_mask": 0,
            })
            view._refresh_mission_weight_visibility()
            self.assertIn("does not land near", view.mission_weight_visibility.text())
            view.model.latest.update({"weight.gap_mask": 0, "weight.target_mask": 1})
            view._refresh_mission_weight_visibility()
            self.assertIn("height gap", view.mission_weight_visibility.text())
            view.close()


if __name__ == "__main__":
    unittest.main()
