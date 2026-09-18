import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "Non-Teensy-Files" / "BluetoothDebugGUI"))

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication
from DebugGUI import RobotDebugGUI

app = QApplication([])
window = RobotDebugGUI()
start = time.monotonic()
heartbeats = 0
last_telemetry = None


def note(text):
    print(f"{time.monotonic()-start:8.3f} {text}", flush=True)


def heartbeat():
    global heartbeats, last_telemetry
    heartbeats += 1
    last_telemetry = window.last_telemetry_monotonic
    if heartbeats % 10 == 0:
        age = None if last_telemetry is None else time.monotonic() - last_telemetry
        note(f"GUI heartbeat={heartbeats} telemetry_age={age} errors={window.protocol_error_count}")


def command(name, **args):
    note(f"COMMAND {name} {args}")
    window.execute_command(name, args)


def finish():
    age = None if window.last_telemetry_monotonic is None else time.monotonic() - window.last_telemetry_monotonic
    note(f"RESULT heartbeats={heartbeats} telemetry_age={age} errors={window.protocol_error_count}")
    window.bluetooth.disconnect_port()
    app.quit()


window.bluetooth.connection_changed.connect(lambda connected, port: note(f"CONNECTED={connected} {port}"))
window.bluetooth.error_received.connect(lambda message: note(f"ERROR {message}"))
window.bluetooth.log_received.connect(lambda level, message: note(f"LOG {level}: {message}"))
window.bluetooth.connect_port("COM10", 115200)

timer = QTimer()
timer.timeout.connect(heartbeat)
timer.start(100)

QTimer.singleShot(5000, window.bluetooth.enter_debug_mode)
QTimer.singleShot(6000, lambda: command("run"))
QTimer.singleShot(7000, lambda: command("zero_herkulex_here", id=4))
QTimer.singleShot(8000, lambda: command("read_herkulex_angle", id=4))
QTimer.singleShot(9000, lambda: command("set_herkulex_angle", id=4, angle_deg=-30, move_time_ms=500))
QTimer.singleShot(39000, lambda: command("set_herkulex_angle", id=4, angle_deg=30, move_time_ms=500))
QTimer.singleShot(69000, lambda: command("set_herkulex_angle", id=4, angle_deg=-30, move_time_ms=500))
QTimer.singleShot(99000, lambda: command("set_herkulex_angle", id=4, angle_deg=30, move_time_ms=500))
QTimer.singleShot(129000, lambda: command("read_herkulex_angle", id=4))
QTimer.singleShot(132000, finish)

raise SystemExit(app.exec())
