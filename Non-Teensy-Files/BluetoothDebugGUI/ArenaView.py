"""Live, local robot view from encoders, BNO055, TOF and ultrasound telemetry.

This is a visualisation, not a navigation or collision-avoidance controller.
The first received pose defines the origin; no absolute arena localisation exists.
"""

from __future__ import annotations

import math
import json
import re
from collections import deque
from pathlib import Path

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget,
)


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class ArenaModel:
    """Local-frame odometry and an expanding occupancy-evidence map."""

    def __init__(self):
        self.mm_per_count = 0.0
        self.encoder_1_mm_per_count = 0.0
        self.encoder_2_mm_per_count = 0.0
        self.track_width_mm = 0.0
        self.invert_left = False
        self.invert_right = False
        self.front_angle_deg = 0.0
        self.left_angle_deg = 90.0
        self.matrix_fov_deg = 60.0  # user-adjustable estimate, not a sensor spec
        self.matrix_mirrored = True
        self.sensor_specs = [
            {"name": "front", "angle": 0.0, "x": 0.0, "y": 0.0, "port": "XSHUT1"},
            {"name": "left", "angle": 90.0, "x": 0.0, "y": 0.0, "port": "XSHUT2"},
        ]
        self.ultrasound_specs = []
        self.matrix_spec = {"name": "matrix", "port": "I2C1", "label": "8x8 TOF",
                            "angle": 0.0, "x": 0.0, "y": 150.0, "layer": "top"}
        self.cell_size_mm = 10.0
        self.weight_gap_mm = 150.0
        self.min_range_mm = 10
        self.max_range_mm = 3500.0
        self.reset()

    def reset(self):
        self.grid_generation = getattr(self, "grid_generation", 0) + 1
        self.x = 0.0
        self.y = 0.0
        self.theta = math.pi / 2  # forward is up in the initial local frame
        self._last_counts = None
        self._last_heading = None
        self._frame_time = None
        self._frame = {}
        self.latest = {}
        self.points = deque(maxlen=5000)
        self.cells = {}  # (column,row) -> log-odds-like evidence, negative free
        self.weight_votes = {}  # purple only after repeated paired readings
        self.trail = deque([(0.0, 0.0)], maxlen=1500)
        self.current_rays = []
        self.last_matrix_frame = None
        self.last_matrix_available = None
        self.last_matrix_valid = None
        self.matrix_top_samples = []
        self.matrix_sample_time = None
        self.last_source = "Awaiting telemetry"
        self.last_imu_calibration = None
        self.last_imu_fusion_running = None
        self.last_imu_system_status = None
        self.last_imu_system_error = None
        self.last_imu_system_error_active = None
        self.last_imu_self_test_passed = None
        self.distance_travelled_mm = 0.0
        self.linear_speed_mm_s = 0.0
        self.range_filter_state = {}

    def _filtered_distance(self, name, distance, confirm_initial=False):
        """Reject one-frame range jumps, then lightly median-filter accepted data."""
        state = self.range_filter_state.setdefault(name, {
            "accepted": None, "candidate": None, "candidate_count": 0,
            "recent": deque(maxlen=3),
        })
        accepted = state["accepted"]
        if accepted is None and confirm_initial:
            candidate = state["candidate"]
            candidate_limit = max(100.0, abs(distance) * 0.20)
            if candidate is not None and abs(distance - candidate) <= candidate_limit:
                state["candidate_count"] += 1
            else:
                state["candidate"] = distance
                state["candidate_count"] = 1
            if state["candidate_count"] < 2:
                return None
        if accepted is not None:
            jump_limit = max(150.0, abs(accepted) * 0.35)
            if abs(distance - accepted) > jump_limit:
                candidate = state["candidate"]
                candidate_limit = max(100.0, abs(distance) * 0.20)
                if candidate is not None and abs(distance - candidate) <= candidate_limit:
                    state["candidate_count"] += 1
                else:
                    state["candidate"] = distance
                    state["candidate_count"] = 1
                if state["candidate_count"] < 2:
                    return None
                state["recent"].clear()
            else:
                state["candidate"] = None
                state["candidate_count"] = 0
        state["accepted"] = distance
        state["candidate"] = None
        state["candidate_count"] = 0
        state["recent"].append(distance)
        ordered = sorted(state["recent"])
        return ordered[len(ordered) // 2]

    def receive_telemetry(self, name, value, timestamp):
        if timestamp is None:
            return False
        if self._frame_time is not None and timestamp != self._frame_time:
            self.finish_frame()
        self._frame_time = timestamp
        self._frame[name] = value
        return True

    def finish_frame(self):
        if not self._frame:
            return
        frame = self._frame
        self._frame = {}
        self.latest = frame.copy()
        self._integrate_pose(frame)
        self.current_rays = []
        valid_distances = {}
        for spec in self.sensor_specs:
            label = spec["name"]
            if frame.get(f"tof.{label}.available") is not True or frame.get(f"tof.{label}.timed_out") is True:
                continue
            distance = _number(frame.get(f"tof.{label}.distance_mm"))
            # Zero and 8191 are common invalid/sentinel returns from these
            # modules. Reject every out-of-map-range value before it can
            # become the range filter's accepted baseline or a weight vote.
            if (distance is not None and
                    self.min_range_mm <= distance <= self.max_range_mm):
                filtered = self._filtered_distance(label, distance)
                if filtered is not None:
                    valid_distances[label] = filtered
                    self._add_observation(filtered, spec["angle"], label, spec["x"], spec["y"])

        # Ultrasound contributes ordinary free/obstacle centreline evidence.
        # Keep it out of the point-TOF top/bottom weight comparison because
        # its acoustic beam is broad and has different geometry.
        for spec in self.ultrasound_specs:
            telemetry_name = spec["telemetry_name"]
            if (frame.get(f"ultrasound.{telemetry_name}.valid") is not True or
                    frame.get(f"ultrasound.{telemetry_name}.timed_out") is True):
                continue
            distance = _number(frame.get(f"ultrasound.{telemetry_name}.distance_mm"))
            if (distance is not None and
                    self.min_range_mm <= distance <= self.max_range_mm):
                filtered = self._filtered_distance(
                    f"ultrasound.{telemetry_name}", distance, confirm_initial=True
                )
                if filtered is not None:
                    self._add_observation(
                        filtered,
                        spec["angle"],
                        f"ultrasound.{telemetry_name}",
                        spec["x"],
                        spec["y"],
                    )

        for bottom in self.sensor_specs:
            if bottom.get("layer") != "bottom" or bottom["name"] not in valid_distances:
                continue
            possible_tops = [top for top in self.sensor_specs
                             if top.get("layer") == "top" and top["name"] in valid_distances
                             and abs(top["x"] - bottom["x"]) <= 150
                             and abs(top["y"] - bottom["y"]) <= 150
                             and abs((top["angle"] - bottom["angle"] + 180) % 360 - 180) <= 15]
            bottom_distance = valid_distances[bottom["name"]]
            if not self.min_range_mm <= bottom_distance <= self.max_range_mm:
                continue
            top_distance = None
            if possible_tops:
                top = min(possible_tops, key=lambda item: abs(item["x"] - bottom["x"]) +
                          abs(item["y"] - bottom["y"]))
                top_distance = valid_distances[top["name"]]
            elif (self.matrix_spec.get("layer") == "top" and
                  abs(self.matrix_spec["x"] - bottom["x"]) <= 0 and
                  self.matrix_top_samples and
                  _number(self._frame_time) is not None and
                  _number(self.matrix_sample_time) is not None and
                  abs(float(self._frame_time) - float(self.matrix_sample_time)) <= 750):
                matching = [(abs((angle - bottom["angle"] + 180) % 360 - 180), distance)
                            for angle, distance in self.matrix_top_samples]
                matching = [item for item in matching if item[0] <= 10]
                if matching:
                    top_distance = min(matching)[1]
            if top_distance is None:
                continue
            endpoint = self._endpoint(bottom_distance, bottom["angle"], bottom["x"], bottom["y"])
            cell = self._cell(*endpoint)
            if top_distance >= bottom_distance + self.weight_gap_mm:
                self.weight_votes[cell] = min(5, self.weight_votes.get(cell, 0) + 1)
            else:
                self.weight_votes.pop(cell, None)

    def _integrate_pose(self, frame):
        left = _number(frame.get("encoder.1.count"))
        right = _number(frame.get("encoder.2.count"))
        if left is None or right is None:
            return
        counts = (left, right)
        if self._last_counts is None:
            self._last_counts = counts
            self.last_source = "Encoder baseline acquired"
            return
        dleft = left - self._last_counts[0]
        dright = right - self._last_counts[1]
        self._last_counts = counts
        # An encoder-zero command can cause a huge count discontinuity.
        if abs(dleft) > 100000 or abs(dright) > 100000:
            self.last_source = "Encoder reset detected; pose held"
            return
        if self.invert_left:
            dleft = -dleft
        if self.invert_right:
            dright = -dright
        left_scale = self.encoder_1_mm_per_count or self.mm_per_count
        right_scale = self.encoder_2_mm_per_count or self.mm_per_count
        distance_scale_ready = left_scale > 0 and right_scale > 0
        dleft_mm = dleft * left_scale
        dright_mm = dright * right_scale
        ds = (dleft_mm + dright_mm) * 0.5

        valid_imu = frame.get("imu.available") is True and frame.get("imu.valid") is True
        heading = _number(frame.get("imu.heading_deg")) if valid_imu else None
        cal = _number(frame.get("imu.calibration.system"))
        self.last_imu_calibration = int(cal) if cal is not None else None
        self.last_imu_fusion_running = frame.get("imu.fusion_running")
        system_status = _number(frame.get("imu.system_status"))
        system_error = _number(frame.get("imu.system_error"))
        self.last_imu_system_status = int(system_status) if system_status is not None else None
        self.last_imu_system_error = int(system_error) if system_error is not None else None
        self.last_imu_system_error_active = frame.get("imu.system_error_active")
        self.last_imu_self_test_passed = frame.get("imu.self_test_passed")
        fusion_field_present = "imu.fusion_running" in frame
        heading_ready = (
            valid_imu and heading is not None and
            (self.last_imu_fusion_running is True or
             (not fusion_field_present and (cal is None or cal >= 1)))
        )
        previous_theta = self.theta
        if heading_ready:
            if self._last_heading is not None:
                # Wrap through 0/360 without a spurious full revolution.
                clockwise_delta = (heading - self._last_heading + 180) % 360 - 180
                # The BNO055 magnetometer drifts around the powered chassis.
                # If neither wheel moved, update its baseline but hold map yaw.
                if abs(dleft) + abs(dright) >= 4:
                    self.theta -= math.radians(clockwise_delta)
            self._last_heading = heading
            self.last_source = (
                "IMU heading + encoder distance" if abs(dleft) + abs(dright) >= 4
                else "Stationary: encoder lock suppressing IMU yaw drift"
            )
        elif self.track_width_mm > 0 and distance_scale_ready:
            self._last_heading = None
            self.theta += (dright_mm - dleft_mm) / self.track_width_mm
            self.last_source = "Encoder-only heading (IMU uncalibrated/unavailable)"
        else:
            self._last_heading = None
            self.last_source = "IMU fusion unavailable; turn mapping paused"

        if distance_scale_ready:
            self.distance_travelled_mm += abs(ds)
            left_rate = _number(frame.get("encoder.1.counts_per_s"))
            right_rate = _number(frame.get("encoder.2.counts_per_s"))
            if left_rate is not None and right_rate is not None:
                if self.invert_left:
                    left_rate = -left_rate
                if self.invert_right:
                    right_rate = -right_rate
                self.linear_speed_mm_s = (
                    left_rate * left_scale + right_rate * right_scale
                ) * 0.5
            # Integrate the exact constant-curvature arc between telemetry
            # frames. The old midpoint approximation badly displaced the map
            # during turns when Bluetooth delivered sparse pose samples.
            dtheta = self.theta - previous_theta
            if abs(dtheta) < 1e-6:
                self.x += ds * math.cos(self.theta)
                self.y += ds * math.sin(self.theta)
            else:
                radius = ds / dtheta
                self.x += radius * (math.sin(self.theta) - math.sin(previous_theta))
                self.y -= radius * (math.cos(self.theta) - math.cos(previous_theta))
            if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) >= 15:
                self.trail.append((self.x, self.y))

    def _cell(self, x, y):
        return math.floor(x / self.cell_size_mm), math.floor(y / self.cell_size_mm)

    def _endpoint(self, distance_mm, angle_deg, lateral_mm=0.0, forward_mm=0.0):
        angle = self.theta + math.radians(angle_deg)
        origin_x = self.x + forward_mm * math.cos(self.theta) + lateral_mm * math.sin(self.theta)
        origin_y = self.y + forward_mm * math.sin(self.theta) - lateral_mm * math.cos(self.theta)
        target = (origin_x + distance_mm * math.cos(angle),
                  origin_y + distance_mm * math.sin(angle))
        return target

    def _add_observation(self, distance_mm, angle_deg, source, lateral_mm=0.0, forward_mm=0.0):
        if not self.min_range_mm <= distance_mm <= self.max_range_mm:
            return
        origin_x = self.x + forward_mm * math.cos(self.theta) + lateral_mm * math.sin(self.theta)
        origin_y = self.y + forward_mm * math.sin(self.theta) - lateral_mm * math.cos(self.theta)
        target = self._endpoint(distance_mm, angle_deg, lateral_mm, forward_mm)
        self.current_rays.append((origin_x, origin_y, *target, source))
        # Without a distance scale, previous readings cannot be registered.
        if self.mm_per_count > 0:
            self.points.append((*target, source))
            endpoint = self._cell(*target)
            traversed = set()
            count = max(1, math.ceil(distance_mm / (self.cell_size_mm / 3)))
            for step in range(count):
                fraction = step / count
                cell = self._cell(origin_x + fraction * (target[0] - origin_x),
                                  origin_y + fraction * (target[1] - origin_y))
                if cell == endpoint or cell in traversed:
                    continue
                traversed.add(cell)
                self.cells[cell] = max(-8, self.cells.get(cell, 0) - 1)
            self.cells[endpoint] = min(8, self.cells.get(endpoint, 0) + 3)

    def _add_wedge_observation(self, distance_mm, centre_angle_deg, width_deg,
                               source, lateral_mm=0.0, forward_mm=0.0,
                               reference_angle_deg=0.0):
        """Paint a matrix sector whose distance is forward depth, not radius."""
        if not self.min_range_mm <= distance_mm <= self.max_range_mm:
            return
        origin_x = self.x + forward_mm * math.cos(self.theta) + lateral_mm * math.sin(self.theta)
        origin_y = self.y + forward_mm * math.sin(self.theta) - lateral_mm * math.cos(self.theta)
        half_width = width_deg * 0.5

        def plane_endpoint(angle_deg):
            # Equal matrix depths lie on a plane perpendicular to the module's
            # centreline. Dividing by cos converts axial depth to ray length.
            relative = math.radians(angle_deg - reference_angle_deg)
            cosine = max(0.05, math.cos(relative))
            return self._endpoint(distance_mm / cosine, angle_deg,
                                  lateral_mm, forward_mm)

        for offset in (-half_width, 0.0, half_width):
            target = plane_endpoint(centre_angle_deg + offset)
            self.current_rays.append((origin_x, origin_y, *target, source))
        if self.mm_per_count <= 0:
            return

        centre_target = plane_endpoint(centre_angle_deg)
        self.points.append((*centre_target, source))
        free_cells = set()
        left_relative = math.radians(centre_angle_deg - half_width - reference_angle_deg)
        right_relative = math.radians(centre_angle_deg + half_width - reference_angle_deg)
        cap_width = abs(distance_mm * (math.tan(right_relative) - math.tan(left_relative)))
        angular_samples = max(2, math.ceil(cap_width / self.cell_size_mm))
        endpoint_cells = set()
        for index in range(angular_samples + 1):
            fraction = index / angular_samples
            angle_deg = centre_angle_deg - half_width + width_deg * fraction
            relative = math.radians(angle_deg - reference_angle_deg)
            ray_length = distance_mm / max(0.05, math.cos(relative))
            world_angle = self.theta + math.radians(angle_deg)
            free_limit = max(0.0, ray_length - self.cell_size_mm * 1.5)
            radius = self.cell_size_mm * 0.5
            while radius <= free_limit:
                free_cells.add(self._cell(origin_x + radius * math.cos(world_angle),
                                          origin_y + radius * math.sin(world_angle)))
                radius += self.cell_size_mm
            endpoint_cells.add(self._cell(
                origin_x + ray_length * math.cos(world_angle),
                origin_y + ray_length * math.sin(world_angle)
            ))
        for cell in free_cells:
            self.cells[cell] = max(-8, self.cells.get(cell, 0) - 1)
        for cell in endpoint_cells:
            self.cells[cell] = min(8, self.cells.get(cell, 0) + 3)

    def receive_matrix(self, message):
        self.last_matrix_available = bool(message.get("available"))
        self.last_matrix_valid = bool(message.get("valid"))
        if not message.get("available") or not message.get("valid"):
            self.matrix_top_samples = []
            return
        frame_id = message.get("frame")
        if frame_id == self.last_matrix_frame:
            return
        values = message.get("data")
        if not isinstance(values, list) or len(values) != 64:
            return
        self.last_matrix_frame = frame_id
        self.matrix_sample_time = message.get("time")
        self.matrix_top_samples = []
        # A floor plan has no vertical axis. Use all eight vertical samples in
        # each real horizontal column to obtain a robust column distance, then
        # paint that column's full angular sector. This preserves the useful
        # 8x8 information without inventing horizontal bearings for Y rows.
        side = -1 if self.matrix_mirrored else 1
        sector_width = self.matrix_fov_deg / 8.0
        for col in range(8):
            valid = sorted(
                distance for row in range(8)
                if (distance := _number(values[row * 8 + col])) is not None
                and self.min_range_mm <= distance <= self.max_range_mm
            )
            # Do not let one or two stray pixels define an entire sector.
            if len(valid) < 3:
                continue
            distance = valid[len(valid) // 2]
            distance = self._filtered_distance(
                f"matrix.column.{col}", distance, confirm_initial=True
            )
            if distance is None:
                continue
            angle = (self.matrix_spec["angle"] +
                     side * ((3.5 - col) / 8.0) * self.matrix_fov_deg)
            self.matrix_top_samples.append((angle, distance))
            self._add_wedge_observation(
                distance, angle, sector_width, "8x8",
                self.matrix_spec["x"], self.matrix_spec["y"],
                self.matrix_spec["angle"]
            )


class ArenaCanvas(QWidget):
    def __init__(self, model, owner):
        super().__init__(owner)
        self.model = model
        self.owner = owner
        self.setMinimumSize(550, 450)
        self.zoom = 1.0
        self.pan = QPointF(0, 0)
        self._drag_at = None
        self._grid_key = None
        self._grid_image = None
        self._painted = {}

    def wheelEvent(self, event):
        old = self.zoom
        self.zoom = min(30.0, max(1.0, old * (1.25 if event.angleDelta().y() > 0 else 0.8)))
        ratio = self.zoom / old
        cursor = event.position() - QPointF(self.width() / 2, self.height() / 2)
        self.pan = cursor - (cursor - self.pan) * ratio
        self.update()

    def mousePressEvent(self, event):
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.MiddleButton):
            self._drag_at = event.position()

    def mouseMoveEvent(self, event):
        if self._drag_at is not None:
            delta = event.position() - self._drag_at
            self.pan += delta
            self._drag_at = event.position()
            self.update()

    def mouseReleaseEvent(self, _event):
        self._drag_at = None

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#151b25"))

        # The map has no assumed arena rectangle. Fit the view around the robot
        # and accumulated evidence; detected obstacle cells eventually form the
        # wall boundary themselves.
        cell_size = self.model.cell_size_mm
        xs = [self.model.x]
        ys = [self.model.y]
        if self.model.trail:
            xs.extend(point[0] for point in self.model.trail)
            ys.extend(point[1] for point in self.model.trail)
        if self.model.cells:
            xs.extend(column * cell_size for column, _row in self.model.cells)
            ys.extend(row * cell_size for _column, row in self.model.cells)
        for x0, y0, x1, y1, _source in self.model.current_rays:
            xs.extend((x0, x1))
            ys.extend((y0, y1))
        padding = 350.0
        min_x, max_x = min(xs) - padding, max(xs) + padding
        min_y, max_y = min(ys) - padding, max(ys) + padding
        if max_x - min_x < 1500.0:
            extra = (1500.0 - (max_x - min_x)) * 0.5
            min_x -= extra
            max_x += extra
        if max_y - min_y < 1500.0:
            extra = (1500.0 - (max_y - min_y)) * 0.5
            min_y -= extra
            max_y += extra
        world_width = max_x - min_x
        world_height = max_y - min_y
        scale = self.zoom * min((self.width() - 60) / world_width,
                                (self.height() - 60) / world_height)
        scale = max(scale, 0.01)

        def screen(x, y):
            return QPointF(30 + (x - min_x) * scale + self.pan.x(),
                           self.height() - 30 - (y - min_y) * scale + self.pan.y())

        bounds = QRectF(screen(min_x, max_y), screen(max_x, min_y)).normalized()
        col_min = math.floor(min_x / cell_size)
        col_max = math.ceil(max_x / cell_size)
        row_min = math.floor(min_y / cell_size)
        row_max = math.ceil(max_y / cell_size)
        cols = col_max - col_min
        rows = row_max - row_min
        p.setClipRect(bounds)
        key = (self.model.grid_generation, col_min, col_max, row_min, row_max)
        if cols * rows <= 1_000_000:
            if self._grid_key != key:
                self._grid_key = key
                self._grid_image = QImage(cols, rows, QImage.Format.Format_ARGB32)
                self._grid_image.fill(QColor(0, 0, 0, 0))
                self._painted = {}
            for cell, evidence in self.model.cells.items():
                votes = self.model.weight_votes.get(cell, 0)
                colour = "#a855f7" if votes >= 2 else (
                    "#dc2626" if evidence > 0 else "#16a34a" if evidence < 0 else "#374151")
                if self._painted.get(cell) == colour:
                    continue
                self._painted[cell] = colour
                col, row = cell
                if col_min <= col < col_max and row_min <= row < row_max:
                    self._grid_image.setPixelColor(col - col_min, row_max - 1 - row, QColor(colour))
            image_bounds = QRectF(screen(col_min * cell_size, row_max * cell_size),
                                  screen(col_max * cell_size, row_min * cell_size)).normalized()
            p.drawImage(image_bounds, self._grid_image)
        else:
            p.setPen(Qt.PenStyle.NoPen)
            for (column, row), evidence in self.model.cells.items():
                colour = "#a855f7" if self.model.weight_votes.get((column, row), 0) >= 2 else (
                    "#dc2626" if evidence > 0 else "#16a34a" if evidence < 0 else "#374151")
                p.setBrush(QColor(colour))
                x = column * cell_size
                y = row * cell_size
                p.drawRect(QRectF(screen(x, y + cell_size), screen(x + cell_size, y)).normalized())
        p.setClipping(False)
        p.setPen(QColor("#667489"))
        p.drawText(20, 23, "DISCOVERED MAP — red sensor returns form the arena boundary")

        if len(self.model.trail) > 1:
            p.setPen(QPen(QColor("#60a5fa"), 2))
            trail = QPolygonF([screen(x, y) for x, y in self.model.trail])
            p.drawPolyline(trail)
        p.setPen(QPen(QColor("#6b7280"), 1))
        for x0, y0, x1, y1, _source in self.model.current_rays:
            p.drawLine(screen(x0, y0), screen(x1, y1))

        robot = screen(self.model.x, self.model.y)
        theta = self.model.theta
        size = 19.0
        shape = QPolygonF([
            QPointF(robot.x() + size * math.cos(theta), robot.y() - size * math.sin(theta)),
            QPointF(robot.x() + size * 0.7 * math.cos(theta + 2.45),
                    robot.y() - size * 0.7 * math.sin(theta + 2.45)),
            QPointF(robot.x() + size * 0.7 * math.cos(theta - 2.45),
                    robot.y() - size * 0.7 * math.sin(theta - 2.45)),
        ])
        p.setPen(QPen(QColor("#e0f2fe"), 2))
        p.setBrush(QColor("#0ea5e9"))
        p.drawPolygon(shape)
        p.end()


class SensorLayoutCanvas(QWidget):
    """Drag sensor markers in robot coordinates; drag arrow tips to rotate."""

    selected = pyqtSignal(str)
    geometry_changed = pyqtSignal(str, str, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.specs = []
        self.aux_specs = []
        self.selected_name = None
        self.drag_mode = None

    def _all_specs(self):
        return self.specs + self.aux_specs

    def set_specs(self, specs):
        self.specs = specs
        all_specs = self._all_specs()
        if self.selected_name not in {item["name"] for item in all_specs}:
            self.selected_name = all_specs[0]["name"] if all_specs else None
        self.update()

    def set_aux_specs(self, specs):
        """Add extra draggable sensors without changing the primary TOF spec list."""
        self.aux_specs = specs
        all_specs = self._all_specs()
        if self.selected_name not in {item["name"] for item in all_specs}:
            self.selected_name = all_specs[0]["name"] if all_specs else None
        self.update()

    def _scale(self):
        return min((self.width() - 55) / 700.0, (self.height() - 60) / 800.0)

    def _marker(self, spec):
        if spec.get("kind") == "ultrasound":
            layer_shift = 0
        else:
            layer_shift = -6 if spec.get("layer") == "top" else 6
        return QPointF(self.width() / 2 + spec["x"] * self._scale() + layer_shift,
                       self.height() / 2 - spec["y"] * self._scale())

    def _tip(self, spec):
        point = self._marker(spec)
        radians = math.radians(spec["angle"])
        return QPointF(point.x() - 42 * math.sin(radians),
                       point.y() - 42 * math.cos(radians))

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#17202b"))
        centre = QPointF(self.width() / 2, self.height() / 2)
        scale = self._scale()
        body = QRectF(centre.x() - 190 * scale, centre.y() - 250 * scale,
                      380 * scale, 500 * scale)
        p.setPen(QPen(QColor("#94a3b8"), 2))
        p.setBrush(QColor("#253244"))
        p.drawRoundedRect(body, 15, 15)
        p.setPen(QPen(QColor("#93c5fd"), 2))
        p.drawLine(QPointF(centre.x(), body.top() + 24),
                   QPointF(centre.x(), body.top() - 14))
        p.drawText(int(centre.x() - 24), int(body.top() - 19), "FRONT")
        p.setPen(QColor("#94a3b8"))
        p.drawText(8, self.height() - 8, "Robot left ←                         → Robot right")
        for spec in self._all_specs():
            marker = self._marker(spec)
            tip = self._tip(spec)
            is_ultrasound = spec.get("kind") == "ultrasound"
            colour = QColor(
                "#facc15" if is_ultrasound
                else "#38bdf8" if spec.get("layer") == "top"
                else "#fb923c"
            )
            selected = spec["name"] == self.selected_name
            p.setPen(QPen(colour, 3 if selected else 2))
            p.drawLine(marker, tip)
            p.setBrush(colour)
            if is_ultrasound:
                size = 9 if selected else 7
                diamond = QPolygonF([
                    QPointF(marker.x(), marker.y() - size),
                    QPointF(marker.x() + size, marker.y()),
                    QPointF(marker.x(), marker.y() + size),
                    QPointF(marker.x() - size, marker.y()),
                ])
                p.drawPolygon(diamond)
            elif spec.get("layer") == "top":
                p.drawEllipse(marker, 8 if selected else 6, 8 if selected else 6)
            else:
                size = 8 if selected else 6
                p.drawRect(QRectF(marker.x() - size, marker.y() - size, size * 2, size * 2))
            if selected:
                p.setBrush(QColor("#f8fafc"))
                p.drawEllipse(tip, 5, 5)
            p.setPen(QColor("#e2e8f0"))
            p.drawText(int(marker.x() + 9), int(marker.y() - 10), spec["port"])
        p.end()

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        position = event.position()
        current = next((s for s in self._all_specs() if s["name"] == self.selected_name), None)
        if current and (position - self._tip(current)).manhattanLength() <= 17:
            self.drag_mode = "angle"
            return
        nearest = min(self._all_specs(),
                      key=lambda s: (position - self._marker(s)).manhattanLength(),
                      default=None)
        if nearest and (position - self._marker(nearest)).manhattanLength() <= 24:
            self.selected_name = nearest["name"]
            self.selected.emit(self.selected_name)
            self.drag_mode = "position"
            self.update()

    def mouseMoveEvent(self, event):
        if not self.drag_mode:
            return
        spec = next((s for s in self._all_specs() if s["name"] == self.selected_name), None)
        if spec is None:
            return
        position = event.position()
        if self.drag_mode == "angle":
            marker = self._marker(spec)
            angle = math.degrees(math.atan2(-(position.x() - marker.x()),
                                            -(position.y() - marker.y())))
            self.geometry_changed.emit(spec["name"], "angle", max(-180.0, min(180.0, angle)))
        else:
            if spec.get("kind") == "ultrasound":
                shift = 0
            else:
                shift = -6 if spec.get("layer") == "top" else 6
            lateral = (position.x() - self.width() / 2 - shift) / self._scale()
            forward = (self.height() / 2 - position.y()) / self._scale()
            self.geometry_changed.emit(spec["name"], "x", max(-500.0, min(500.0, lateral)))
            self.geometry_changed.emit(spec["name"], "y", max(-500.0, min(500.0, forward)))

    def mouseReleaseEvent(self, _event):
        self.drag_mode = None


class ArenaView(QWidget):
    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.model = ArenaModel()
        self.sensor_unmatched = []
        self.model.matrix_mirrored = str(settings.value("sensor/mirror_8x8", "true")).lower() == "true"
        layout = QHBoxLayout(self)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.canvas = ArenaCanvas(self.model, self)
        left_layout.addWidget(self.canvas, 1)
        self._build_raw_tof_panel()
        left_layout.addWidget(self.raw_tof_group)
        layout.addWidget(left, 1)
        side = QWidget()
        side.setMaximumWidth(330)
        form = QFormLayout(side)
        self.status = QLabel("Waiting for encoder, IMU, TOF and ultrasound telemetry")
        self.status.setWordWrap(True)
        form.addRow(self.status)

        def spin(key, label, default, maximum, decimals=0):
            control = QDoubleSpinBox()
            control.setRange(0, maximum)
            control.setDecimals(decimals)
            control.setSuffix(" mm" if "deg" not in key and "count" not in key else "")
            try:
                control.setValue(float(settings.value("arena/" + key, default)))
            except (TypeError, ValueError):
                control.setValue(default)
            control.valueChanged.connect(lambda value, k=key: self._changed(k, value))
            form.addRow(label, control)
            return control

        self.arena_width = spin("width", "Arena width", 2400, 50000)
        self.arena_height = spin("height", "Arena length", 4900, 50000)
        # Retain these values for old recording compatibility, but they no
        # longer constrain or draw the live discovered map.
        for control in (self.arena_width, self.arena_height):
            label = form.labelForField(control)
            if label is not None:
                label.hide()
            control.hide()
        # Powered straight-run calibration. The first two consistent trials
        # covered 775 mm over 8522/8080 counts. A third trial was rejected as
        # an approximately 12% distance outlier.
        if not settings.value("arena/dual_encoder_calibration_20260920", False, type=bool):
            settings.setValue("arena/encoder_1_mm_per_count", 775.0 / 8522.0)
            settings.setValue("arena/encoder_2_mm_per_count", 775.0 / 8080.0)
            settings.setValue("arena/invert_left", True)
            settings.setValue("arena/invert_right", False)
            settings.setValue("arena/dual_encoder_calibration_20260920", True)
        self.encoder_1_scale = spin(
            "encoder_1_mm_per_count", "Encoder 1 mm / count", 775.0 / 8522.0, 100, 5)
        self.encoder_2_scale = spin(
            "encoder_2_mm_per_count", "Encoder 2 mm / count", 775.0 / 8080.0, 100, 5)
        self.track_width = spin("track_width", "Wheel track", 0, 2000)
        if not settings.value("arena/wheel_track_40mm_migrated", False, type=bool):
            if self.track_width.value() == 0:
                self.track_width.blockSignals(True)
                self.track_width.setValue(40)
                self.track_width.blockSignals(False)
                settings.setValue("arena/track_width", 40)
            settings.setValue("arena/wheel_track_40mm_migrated", True)
        if not settings.value("arena/centimetre_grid_migrated", False, type=bool):
            old_size = _number(settings.value("arena/cell_size_mm"))
            if old_size is None or old_size == 100:
                settings.setValue("arena/cell_size_mm", 10)
            settings.setValue("arena/centimetre_grid_migrated", True)
        self.grid_size = spin("cell_size_mm", "Grid square size", 10, 500)
        self.grid_size.setMinimum(5)
        self.weight_gap = spin("weight_gap_mm", "Purple distance gap", 150, 1000)
        self.weight_gap.setMinimum(20)
        self.matrix_fov = spin("matrix_fov_deg", "8x8 horizontal FOV (°)", 60, 180, 1)
        self.sensor_canvas = SensorLayoutCanvas()
        self.sensor_canvas.geometry_changed.connect(self._sensor_canvas_changed)
        form.addRow(self.sensor_canvas)
        self.sensor_controls = {}
        self.sensor_group = None
        self.sensor_form_parent = form
        self.reload_sensor_layout()
        reload_button = QPushButton("Reload sensor ports from Wiring Guide")
        reload_button.clicked.connect(self.reload_sensor_layout)
        form.addRow(reload_button)
        self.invert_left = QCheckBox("Invert encoder 1")
        self.invert_right = QCheckBox("Invert encoder 2")
        for key, check in (("invert_left", self.invert_left), ("invert_right", self.invert_right)):
            check.setChecked(str(settings.value("arena/" + key, "false")).lower() == "true")
            check.toggled.connect(lambda value, k=key: self._changed(k, value))
            form.addRow(check)
        reset = QPushButton("Reset local map / set current pose as origin")
        reset.clicked.connect(self.reset_map)
        form.addRow(reset)
        reset_view = QPushButton("Reset map zoom / pan")
        reset_view.clicked.connect(self.reset_view)
        form.addRow(reset_view)
        note = QLabel("Grey: unknown · green: measured free · red: obstacle · purple: possible weight (bottom return, paired top sees farther). "
                      "Wheel over map to zoom; drag map to pan. In the robot diagram, drag a sensor to place it and drag its white arrow tip to aim it. "
                      "Cyan circles are top point TOFs; orange squares are bottom point TOFs; yellow diamonds are ultrasound. "
                      "All sensor markers can be dragged to place them and their arrow tips can be dragged to aim them. "
                      "No arena outline is assumed; detected red wall cells build the boundary. "
                      "Initial wheel scale assumes ~80 mm diameter, 663 PPR and 4× decoding; verify with a measured push. "
                      "Blue is the robot trail. Free/obstacle colours are sensor evidence, not confirmed walls. "
                      "Set wheel scale and directions before trusting the history. "
                      "The robot starts at the map centre, not a surveyed arena location.")
        note.setWordWrap(True)
        form.addRow(note)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(side)
        scroll.setMaximumWidth(410)
        layout.addWidget(scroll)
        self._sync()

    def _build_raw_tof_panel(self):
        """Build the always-visible raw range-sensor readout."""
        self.raw_tof_group = QGroupBox("Raw range readings — values before mapping filters")
        self.raw_tof_group.setMaximumHeight(285)
        panel = QHBoxLayout(self.raw_tof_group)

        point_box = QGroupBox("Point TOF sensors")
        self.raw_point_grid = QGridLayout(point_box)
        self.raw_point_grid.setColumnStretch(0, 1)
        self.raw_point_labels = {}
        self.raw_point_state = {}
        panel.addWidget(point_box, 2)

        ultrasound_box = QGroupBox("Ultrasound sensors")
        self.raw_ultrasound_grid = QGridLayout(ultrasound_box)
        self.raw_ultrasound_grid.setColumnStretch(0, 1)
        self.raw_ultrasound_labels = {}
        self.raw_ultrasound_state = {}
        panel.addWidget(ultrasound_box, 2)

        matrix_box = QGroupBox("8×8 TOF — raw millimetres")
        matrix_layout = QVBoxLayout(matrix_box)
        self.raw_matrix_status = QLabel("Waiting for an 8×8 frame")
        matrix_layout.addWidget(self.raw_matrix_status)
        matrix_grid = QGridLayout()
        matrix_grid.setSpacing(2)
        corner = QLabel("Y\\X")
        corner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        matrix_grid.addWidget(corner, 0, 0)
        for col in range(8):
            header = QLabel(f"X{col}")
            header.setAlignment(Qt.AlignmentFlag.AlignCenter)
            matrix_grid.addWidget(header, 0, col + 1)
        self.raw_matrix_cells = []
        for row in range(8):
            header = QLabel(f"Y{row}")
            header.setAlignment(Qt.AlignmentFlag.AlignCenter)
            matrix_grid.addWidget(header, row + 1, 0)
            row_cells = []
            for col in range(8):
                cell = QLabel("—")
                cell.setAlignment(Qt.AlignmentFlag.AlignCenter)
                cell.setMinimumWidth(37)
                cell.setStyleSheet(
                    "QLabel { background: #374151; color: white; "
                    "border: 1px solid #111827; padding: 1px; }"
                )
                matrix_grid.addWidget(cell, row + 1, col + 1)
                row_cells.append(cell)
            self.raw_matrix_cells.append(row_cells)
        matrix_layout.addLayout(matrix_grid)
        panel.addWidget(matrix_box, 3)

    def _rebuild_raw_point_rows(self):
        while self.raw_point_grid.count():
            item = self.raw_point_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self.raw_point_labels = {}
        for row, spec in enumerate(self.model.sensor_specs):
            name = spec["name"]
            title = QLabel(f"{spec.get('label', name)} ({spec.get('port', '?')})")
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.raw_point_grid.addWidget(title, row, 0)
            self.raw_point_grid.addWidget(value, row, 1)
            self.raw_point_labels[name] = value
            self._refresh_raw_point(name)

    def _refresh_raw_point(self, name):
        label = self.raw_point_labels.get(name)
        if label is None:
            return
        state = self.raw_point_state.get(name, {})
        available = state.get("available")
        timed_out = state.get("timed_out")
        distance = _number(state.get("distance_mm"))
        if available is False:
            text, colour = "OFFLINE", "#ef4444"
        elif timed_out is True:
            text, colour = "TIMEOUT", "#f59e0b"
        elif distance is None:
            text, colour = "—", "#9ca3af"
        elif not self.model.min_range_mm <= distance <= self.model.max_range_mm:
            text, colour = f"{distance:g} mm · invalid", "#f59e0b"
        else:
            text, colour = f"{distance:g} mm", "#22c55e"
        label.setText(text)
        label.setStyleSheet(f"QLabel {{ color: {colour}; font-weight: bold; }}")

    def _rebuild_raw_ultrasound_rows(self):
        while self.raw_ultrasound_grid.count():
            item = self.raw_ultrasound_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self.raw_ultrasound_labels = {}
        for row, spec in enumerate(self.model.ultrasound_specs):
            telemetry_name = spec["telemetry_name"]
            title = QLabel(f"{spec.get('label', telemetry_name)} ({spec.get('port', '?')})")
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.raw_ultrasound_grid.addWidget(title, row, 0)
            self.raw_ultrasound_grid.addWidget(value, row, 1)
            self.raw_ultrasound_labels[telemetry_name] = value
            self._refresh_raw_ultrasound(telemetry_name)

    def _refresh_raw_ultrasound(self, name):
        label = self.raw_ultrasound_labels.get(name)
        if label is None:
            return
        state = self.raw_ultrasound_state.get(name, {})
        valid = state.get("valid")
        timed_out = state.get("timed_out")
        distance = _number(state.get("distance_mm"))
        echo_us = _number(state.get("echo_us"))
        rise_count = _number(state.get("rise_count"))
        fall_count = _number(state.get("fall_count"))

        if timed_out is True:
            text, colour = "TIMEOUT", "#f59e0b"
        elif valid is True and distance is not None:
            details = [f"{distance:g} mm"]
            if echo_us is not None:
                details.append(f"{echo_us:g} µs")
            if rise_count is not None and fall_count is not None:
                details.append(f"edges {int(rise_count)}/{int(fall_count)}")
            if not self.model.min_range_mm <= distance <= self.model.max_range_mm:
                details.append("outside map")
                colour = "#f59e0b"
            else:
                colour = "#22c55e"
            text = " · ".join(details)
        elif valid is False:
            text, colour = "NO VALID RANGE", "#9ca3af"
        else:
            text, colour = "—", "#9ca3af"

        label.setText(text)
        label.setStyleSheet(f"QLabel {{ color: {colour}; font-weight: bold; }}")

    def _refresh_raw_matrix(self, message):
        available = bool(message.get("available"))
        valid = bool(message.get("valid"))
        values = message.get("data")
        if not available or not valid or not isinstance(values, list) or len(values) != 64:
            self.raw_matrix_status.setText(
                "Unavailable" if not available else "Latest frame is invalid"
            )
            for row_cells in self.raw_matrix_cells:
                for cell in row_cells:
                    cell.setText("—")
            return
        address = _number(message.get("address"))
        address_text = f"0x{int(address):02X}" if address is not None else "?"
        self.raw_matrix_status.setText(
            f"Frame {message.get('frame', '?')} · {message.get('bus', 'I2C')} · "
            f"address {address_text}"
        )
        for row in range(8):
            for col in range(8):
                value = _number(values[row * 8 + col])
                self.raw_matrix_cells[row][col].setText("—" if value is None else f"{value:g}")

    def reload_sensor_layout(self):
        """Join firmware telemetry names to the user's persisted port guide."""
        config_path = Path(__file__).resolve().parents[2] / "PlatformIO" / "debug_config.json"
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            config = {}
        configured = config.get("tof_sensors", [])
        configured_ultrasound = config.get("ultrasound_sensors", [])
        try:
            guide = json.loads(str(self.settings.value("wiring_guide/entries_v1", "[]")))
        except (TypeError, ValueError):
            guide = []
        by_port = {}
        ultrasound_labels = {}
        for entry in guide if isinstance(guide, list) else []:
            if not isinstance(entry, dict) or str(entry.get("kind", "")).lower() != "sensor":
                continue
            port = str(entry.get("connector", "")).strip().upper()
            device = str(entry.get("device", "")).strip()
            pins = str(entry.get("pins", "")).upper()
            if re.fullmatch(r"XSHUT[0-7]", port) and "tof" in device.lower():
                by_port[port] = device
            if "ultrasound" in device.lower():
                lower_device = device.lower()
                display_device = device
                if lower_device == "ultrasound a":
                    display_device = "Ultrasound Left (A)"
                elif lower_device == "ultrasound b":
                    display_device = "Ultrasound Right (B)"
                if ("(a)" in lower_device or lower_device.endswith(" a") or
                        ("D14" in pins and "D24" in pins)):
                    ultrasound_labels["a"] = display_device
                elif ("(b)" in lower_device or lower_device.endswith(" b") or
                      ("D22" in pins and "D20" in pins)):
                    ultrasound_labels["b"] = display_device

        if self.sensor_group is not None:
            self.sensor_form_parent.removeRow(self.sensor_group)
        self.sensor_group = QGroupBox("Range sensors — robot point of view")
        sensor_form = QFormLayout(self.sensor_group)
        self.sensor_controls = {}
        specs = []
        configured_ports = set()
        for sensor in configured if isinstance(configured, list) else []:
            if not isinstance(sensor, dict):
                continue
            port = str(sensor.get("port", "")).strip().upper()
            name = str(sensor.get("name", "")).strip()
            if not name or not re.fullmatch(r"XSHUT[0-7]", port):
                continue
            configured_ports.add(port)
            label = by_port.get(port, str(sensor.get("label", name)))
            lower = label.lower()
            lateral = -100 if "left" in lower else (180 if "right right" in lower else 100 if "right" in lower else 0)
            forward = 120  # top/bottom describes height, not forward/back position
            layer_default = "bottom" if "bottom" in lower else "top"
            layer = str(self.settings.value(f"arena/sensors/{name}/layer", layer_default)).lower()
            if layer not in ("top", "bottom"):
                layer = layer_default
            spec = {"name": name, "port": port, "label": label,
                    "angle": 0.0, "x": float(lateral), "y": float(forward), "layer": layer}
            sensor_form.addRow(QLabel(f"{label} ({port}, {sensor.get('type', '?')})"))
            controls = {}
            for field, caption, default in (
                ("angle", "Angle (°)", 0),
                ("x", "Right offset (mm)", lateral),
                ("y", "Forward offset (mm)", forward),
            ):
                control = QDoubleSpinBox()
                control.setRange(-180 if field == "angle" else -1000,
                                 180 if field == "angle" else 1000)
                control.setDecimals(1)
                key = f"arena/sensors/{name}/{field}"
                try:
                    value = float(self.settings.value(key, default))
                except (TypeError, ValueError):
                    value = float(default)
                control.setValue(value)
                spec[field] = control.value()
                control.valueChanged.connect(
                    lambda new_value, s=spec, f=field, k=key: self._sensor_geometry_changed(s, f, k, new_value)
                )
                sensor_form.addRow(caption, control)
                controls[field] = control
            layer_combo = QComboBox()
            layer_combo.addItems(["top", "bottom"])
            layer_combo.setCurrentText(layer)
            layer_combo.currentTextChanged.connect(
                lambda value, s=spec: self._sensor_layer_changed(s, value)
            )
            sensor_form.addRow("Height layer", layer_combo)
            controls["layer"] = layer_combo
            self.sensor_controls[name] = controls
            specs.append(spec)

        # Ultrasound channels are ordinary draggable range sensors in the same
        # robot-layout canvas. They retain their own telemetry names (a/b), while
        # the user-facing labels come from the Wiring Guide when available.
        ultrasound_specs = []
        for index, sensor in enumerate(
                configured_ultrasound if isinstance(configured_ultrasound, list) else []):
            if not isinstance(sensor, dict):
                continue
            telemetry_name = str(sensor.get("name", "")).strip()
            if not telemetry_name:
                continue
            control_name = f"ultrasound.{telemetry_name}"
            trigger_pin = sensor.get("trigger_pin", "?")
            echo_pin = sensor.get("echo_pin", "?")
            default_label = (
                "Ultrasound Left (A)" if telemetry_name.lower() == "a"
                else "Ultrasound Right (B)" if telemetry_name.lower() == "b"
                else str(sensor.get("label", f"Ultrasound {telemetry_name.upper()}"))
            )
            label = ultrasound_labels.get(telemetry_name.lower(), default_label)
            lateral_default = -90 if index == 0 else 90
            forward_default = 180
            spec = {
                "name": control_name,
                "telemetry_name": telemetry_name,
                "kind": "ultrasound",
                "port": f"US-{telemetry_name.upper()}",
                "label": label,
                "angle": 0.0,
                "x": float(lateral_default),
                "y": float(forward_default),
                "layer": "top",
            }
            sensor_form.addRow(QLabel(
                f"{label} (TRIG D{trigger_pin}, ECHO D{echo_pin})"
            ))
            controls = {}
            for field, caption, default in (
                ("angle", "Angle (°)", 0),
                ("x", "Right offset (mm)", lateral_default),
                ("y", "Forward offset (mm)", forward_default),
            ):
                control = QDoubleSpinBox()
                control.setRange(-180 if field == "angle" else -1000,
                                 180 if field == "angle" else 1000)
                control.setDecimals(1)
                key = f"arena/sensors/{control_name}/{field}"
                try:
                    value = float(self.settings.value(key, default))
                except (TypeError, ValueError):
                    value = float(default)
                control.setValue(value)
                spec[field] = control.value()
                control.valueChanged.connect(
                    lambda new_value, s=spec, f=field, k=key:
                    self._sensor_geometry_changed(s, f, k, new_value)
                )
                sensor_form.addRow(caption, control)
                controls[field] = control
            self.sensor_controls[control_name] = controls
            ultrasound_specs.append(spec)

        matrix_entry = next((entry for entry in guide if isinstance(entry, dict) and
                             "8x8" in str(entry.get("device", "")).lower()), None)
        matrix_label = str(matrix_entry.get("device", "SEN0628 8x8 TOF")) if matrix_entry else "SEN0628 8x8 TOF"
        matrix = {"name": "matrix", "port": "I2C1", "label": matrix_label,
                  "angle": 0.0, "x": 0.0, "y": 150.0, "layer": "top"}
        sensor_form.addRow(QLabel(f"{matrix_label} (I2C1)"))
        matrix_controls = {}
        for field, caption, default in (("angle", "Angle (°)", 0),
                                        ("x", "Right offset (mm)", 0),
                                        ("y", "Forward offset (mm)", 150)):
            control = QDoubleSpinBox()
            control.setRange(-180 if field == "angle" else -1000,
                             180 if field == "angle" else 1000)
            control.setDecimals(1)
            key = f"arena/sensors/matrix/{field}"
            try:
                control.setValue(float(self.settings.value(key, default)))
            except (TypeError, ValueError):
                control.setValue(default)
            matrix[field] = control.value()
            control.valueChanged.connect(
                lambda value, s=matrix, f=field, k=key: self._sensor_geometry_changed(s, f, k, value)
            )
            sensor_form.addRow(caption, control)
            matrix_controls[field] = control
        matrix_layer = QComboBox()
        matrix_layer.addItems(["top", "bottom"])
        matrix_layer.setCurrentText(str(self.settings.value("arena/sensors/matrix/layer", "top")))
        matrix["layer"] = matrix_layer.currentText()
        matrix_layer.currentTextChanged.connect(lambda value, s=matrix: self._sensor_layer_changed(s, value))
        sensor_form.addRow("Height layer", matrix_layer)
        matrix_controls["layer"] = matrix_layer
        self.sensor_controls["matrix"] = matrix_controls
        self.sensor_form_parent.addRow(self.sensor_group)
        self.model.sensor_specs = specs
        self.model.ultrasound_specs = ultrasound_specs
        self.model.matrix_spec = matrix
        self.sensor_canvas.set_specs(specs + [matrix])
        self.sensor_canvas.set_aux_specs(ultrasound_specs)
        self._rebuild_raw_point_rows()
        self._rebuild_raw_ultrasound_rows()
        self.sensor_unmatched = sorted(set(by_port) - configured_ports)
        self.reset_map()

    def _sensor_geometry_changed(self, spec, field, key, value):
        spec[field] = value
        self.settings.setValue(key, value)
        self.sensor_canvas.update()
        self.reset_map()

    def _sensor_layer_changed(self, spec, layer):
        spec["layer"] = layer
        self.settings.setValue(f"arena/sensors/{spec['name']}/layer", layer)
        self.sensor_canvas.update()
        self.reset_map()

    def _sensor_canvas_changed(self, name, field, value):
        control = self.sensor_controls.get(name, {}).get(field)
        if control is not None:
            control.setValue(value)

    def reset_view(self):
        self.canvas.zoom = 1.0
        self.canvas.pan = QPointF(0, 0)
        self.canvas.update()

    def _sync(self):
        m = self.model
        m.encoder_1_mm_per_count = self.encoder_1_scale.value()
        m.encoder_2_mm_per_count = self.encoder_2_scale.value()
        m.mm_per_count = (
            m.encoder_1_mm_per_count + m.encoder_2_mm_per_count
        ) * 0.5
        m.track_width_mm = self.track_width.value()
        m.cell_size_mm = self.grid_size.value()
        m.weight_gap_mm = self.weight_gap.value()
        m.matrix_fov_deg = self.matrix_fov.value()
        m.invert_left = self.invert_left.isChecked()
        m.invert_right = self.invert_right.isChecked()
        self._refresh_status()

    def _changed(self, key, value):
        self.settings.setValue("arena/" + key, value)
        # Changing geometric calibration invalidates registration of old points.
        if key not in ("width", "height"):
            self.reset_map()
        self._sync()
        self.canvas.update()

    def reset_map(self):
        self.model.reset()
        self.canvas.update()
        self._refresh_status()

    def receive_telemetry(self, name, value, timestamp):
        match = re.fullmatch(r"tof\.([^.]+)\.(available|timed_out|distance_mm)", name)
        if match:
            sensor_name, field = match.groups()
            self.raw_point_state.setdefault(sensor_name, {})[field] = value
            self._refresh_raw_point(sensor_name)

        ultrasound_match = re.fullmatch(
            r"ultrasound\.([^.]+)\.(valid|timed_out|distance_mm|echo_us|"
            r"trigger_count|rise_count|fall_count|echo_high)",
            name,
        )
        if ultrasound_match:
            sensor_name, field = ultrasound_match.groups()
            self.raw_ultrasound_state.setdefault(sensor_name, {})[field] = value
            self._refresh_raw_ultrasound(sensor_name)

        old_time = self.model._frame_time
        self.model.receive_telemetry(name, value, timestamp)
        if old_time is not None and old_time != timestamp:
            self._refresh_status()
            self.canvas.update()

    def receive_matrix(self, message):
        self._refresh_raw_matrix(message)
        self.model.receive_matrix(message)
        self._refresh_status()
        self.canvas.update()

    def set_matrix_mirrored(self, mirrored):
        if self.model.matrix_mirrored != mirrored:
            self.model.matrix_mirrored = mirrored
            self.reset_map()

    def _refresh_status(self):
        m = self.model
        calibration = "?" if m.last_imu_calibration is None else str(m.last_imu_calibration)
        accuracy = ("History disabled until both encoder scales are set. "
                    if m.encoder_1_mm_per_count <= 0 or m.encoder_2_mm_per_count <= 0 else "")
        old_firmware = (any(key.startswith("tof.front.") or key.startswith("tof.left.")
                            for key in m.latest) and "imu.available" not in m.latest)
        firmware_warning = (
            "Firmware mismatch: robot still sends old front/left TOF names and no IMU; "
            "upload the current PlatformIO firmware. " if old_firmware else ""
        )
        online = sum(m.latest.get(f"tof.{spec['name']}.available") is True
                     for spec in m.sensor_specs)
        ultrasound_valid = sum(
            m.latest.get(f"ultrasound.{spec['telemetry_name']}.valid") is True
            for spec in m.ultrasound_specs
        )
        missing = (f" Wiring Guide ports not active in firmware: {', '.join(self.sensor_unmatched)}."
                   if self.sensor_unmatched else "")
        self.status.setText(
            f"{firmware_warning}{accuracy}Pose: {m.x:.0f}, {m.y:.0f} mm; heading {math.degrees(m.theta):.1f}°. "
            f"Travel: {m.distance_travelled_mm:.0f} mm; speed: {m.linear_speed_mm_s:.0f} mm/s. "
            f"IMU fusion {'running' if m.last_imu_fusion_running is True else 'not ready'}; "
            f"system calibration {calibration}/3; status/error "
            f"{m.last_imu_system_status if m.last_imu_system_status is not None else '?'}/"
            f"{m.last_imu_system_error if m.last_imu_system_error_active is True else 'inactive'}; "
            f"self-test {'pass' if m.last_imu_self_test_passed is True else 'not confirmed'}. "
            f"{m.last_source}. "
            f"Point TOF online: {online}/{len(m.sensor_specs)}; "
            f"ultrasound valid: {ultrasound_valid}/{len(m.ultrasound_specs)}; "
            f"8x8: {'ready' if m.last_matrix_available and m.last_matrix_valid else 'waiting/invalid'}. "
            f"Known grid squares: {len(m.cells)}; possible-weight squares: "
            f"{sum(v >= 2 for v in m.weight_votes.values())}.{missing}"
        )
