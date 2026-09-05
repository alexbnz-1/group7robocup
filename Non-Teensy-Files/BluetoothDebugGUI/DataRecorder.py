"""
DataRecorder.py

SQLite-backed session recorder for the robot debug GUI.

A recording contains:
- session metadata
- all telemetry samples
- structured robot logs
- commands sent from the GUI
- parameter changes sent from the GUI
- robot state messages
- raw serial lines
- manually logged fault markers

The database uses the .rdbg extension but is a normal SQLite database.
"""

from __future__ import annotations

import json
import sqlite3
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any


class DataRecorder:
    def __init__(self):
        self._conn: sqlite3.Connection | None = None
        self._lock = threading.RLock()
        self._path: Path | None = None
        self._session_start_monotonic: float | None = None

    @property
    def is_recording(self) -> bool:
        return self._conn is not None

    @property
    def path(self) -> Path | None:
        return self._path

    @property
    def elapsed(self) -> float:
        if self._session_start_monotonic is None:
            return 0.0
        return time.monotonic() - self._session_start_monotonic

    def start(
        self,
        path: str | Path,
        *,
        session_name: str = "",
        port: str = "",
        baudrate: int = 0,
    ) -> Path:
        with self._lock:
            if self._conn is not None:
                self.stop()

            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)

            conn = sqlite3.connect(path, check_same_thread=False)
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("PRAGMA synchronous=NORMAL")

            self._create_schema(conn)

            self._conn = conn
            self._path = path
            self._session_start_monotonic = time.monotonic()

            metadata = {
                "session_name": session_name,
                "created_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "port": port,
                "baudrate": str(baudrate),
                "format_version": "2",
                "application": "Robot Debug Console",
            }

            conn.executemany(
                "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
                metadata.items(),
            )
            conn.commit()

            return path

    def stop(self):
        with self._lock:
            if self._conn is None:
                return

            try:
                self._conn.execute(
                    "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
                    (
                        "closed_at",
                        datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    ),
                )
                self._conn.commit()
            finally:
                self._conn.close()
                self._conn = None
                self._session_start_monotonic = None

    def _create_schema(self, conn: sqlite3.Connection):
        conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS metadata (
                key TEXT PRIMARY KEY,
                value TEXT
            );

            CREATE TABLE IF NOT EXISTS telemetry (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                robot_time REAL,
                signal TEXT NOT NULL,
                value_num REAL,
                value_text TEXT,
                value_type TEXT NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_telemetry_signal_time
                ON telemetry(signal, elapsed_s);

            CREATE TABLE IF NOT EXISTS logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                level TEXT NOT NULL,
                message TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS commands (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                command TEXT NOT NULL,
                arguments_json TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS parameters (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                name TEXT NOT NULL,
                value_json TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS states (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                state_json TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS raw_serial (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                line TEXT NOT NULL
            );

            CREATE TABLE IF NOT EXISTS faults (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                elapsed_s REAL NOT NULL,
                wall_time TEXT NOT NULL,
                label TEXT NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_faults_time
                ON faults(elapsed_s);
            """
        )
        conn.commit()

    def _times(self) -> tuple[float, str]:
        return (
            self.elapsed,
            datetime.now().astimezone().isoformat(timespec="milliseconds"),
        )

    @staticmethod
    def _json(value: Any) -> str:
        return json.dumps(value, separators=(",", ":"), ensure_ascii=False)

    def record_telemetry(self, signal: str, value: Any, robot_time: Any = None):
        with self._lock:
            if self._conn is None:
                return

            elapsed, wall = self._times()

            if isinstance(value, bool):
                value_num = 1.0 if value else 0.0
                value_text = "true" if value else "false"
                value_type = "bool"
            elif isinstance(value, (int, float)):
                value_num = float(value)
                value_text = None
                value_type = "number"
            elif value is None:
                value_num = None
                value_text = None
                value_type = "null"
            else:
                value_num = None
                value_text = str(value)
                value_type = type(value).__name__

            try:
                robot_num = float(robot_time) if robot_time is not None else None
            except (TypeError, ValueError):
                robot_num = None

            self._conn.execute(
                """
                INSERT INTO telemetry(
                    elapsed_s, wall_time, robot_time, signal,
                    value_num, value_text, value_type
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    elapsed,
                    wall,
                    robot_num,
                    signal,
                    value_num,
                    value_text,
                    value_type,
                ),
            )
            self._conn.commit()

    def record_log(self, level: str, message: str):
        with self._lock:
            if self._conn is None:
                return

            elapsed, wall = self._times()

            self._conn.execute(
                """
                INSERT INTO logs(elapsed_s, wall_time, level, message)
                VALUES (?, ?, ?, ?)
                """,
                (elapsed, wall, level, message),
            )
            self._conn.commit()

    def record_command(self, command: str, arguments: dict):
        with self._lock:
            if self._conn is None:
                return

            elapsed, wall = self._times()

            self._conn.execute(
                """
                INSERT INTO commands(
                    elapsed_s, wall_time, command, arguments_json
                ) VALUES (?, ?, ?, ?)
                """,
                (elapsed, wall, command, self._json(arguments)),
            )
            self._conn.commit()

    def record_parameter(self, name: str, value: Any):
        with self._lock:
            if self._conn is None:
                return

            elapsed, wall = self._times()

            self._conn.execute(
                """
                INSERT INTO parameters(
                    elapsed_s, wall_time, name, value_json
                ) VALUES (?, ?, ?, ?)
                """,
                (elapsed, wall, name, self._json(value)),
            )
            self._conn.commit()

    def record_state(self, state: dict):
        with self._lock:
            if self._conn is None:
                return

            elapsed, wall = self._times()

            self._conn.execute(
                """
                INSERT INTO states(elapsed_s, wall_time, state_json)
                VALUES (?, ?, ?)
                """,
                (elapsed, wall, self._json(state)),
            )
            self._conn.commit()

    def record_raw(self, line: str):
        with self._lock:
            if self._conn is None:
                return

            elapsed, wall = self._times()

            self._conn.execute(
                """
                INSERT INTO raw_serial(elapsed_s, wall_time, line)
                VALUES (?, ?, ?)
                """,
                (elapsed, wall, line),
            )
            self._conn.commit()

    def record_fault(self, label: str = "MANUAL FAULT MARKER") -> float | None:
        """
        Record an instantaneous manual fault marker.

        Returns the elapsed recording time of the marker, or None if no
        recording is active.
        """
        with self._lock:
            if self._conn is None:
                return None

            elapsed, wall = self._times()

            self._conn.execute(
                """
                INSERT INTO faults(elapsed_s, wall_time, label)
                VALUES (?, ?, ?)
                """,
                (elapsed, wall, label),
            )
            self._conn.commit()

            return elapsed
