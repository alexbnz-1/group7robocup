import json
import serial
import time
from pathlib import Path

PORT = "COM10"
BAUD = 115200
LOG = Path("tmp/herkulex_live_repro.log")

port = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=1.0)
start = time.monotonic()
rx = bytearray()
lines = []
json_errors = 0
last_telemetry = None


def stamp():
    return time.monotonic() - start


def note(text):
    line = f"{stamp():8.3f} {text}"
    print(line, flush=True)
    lines.append(line)


def drain(seconds):
    global rx, json_errors, last_telemetry
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        chunk = port.read(waiting if waiting else 1)
        if chunk:
            rx.extend(chunk)
        while b"\n" in rx:
            raw, _, remainder = rx.partition(b"\n")
            rx = bytearray(remainder)
            text = raw.decode("utf-8", errors="replace").strip("\r ")
            if not text:
                continue
            try:
                message = json.loads(text)
                kind = message.get("type", "?") if isinstance(message, dict) else "non-object"
                if kind == "telemetry":
                    last_telemetry = stamp()
                if kind in ("error", "log", "state"):
                    note(f"RX {kind}: {text}")
            except json.JSONDecodeError as error:
                json_errors += 1
                note(f"RX INVALID JSON ({error}): {text!r}")
        time.sleep(0.002)


def send(command, **arguments):
    message = {"type": "command", "command": command, **arguments}
    payload = (json.dumps(message, separators=(",", ":")) + "\n").encode()
    written = port.write(payload)
    port.flush()
    note(f"TX {written}/{len(payload)}: {payload.decode().strip()}")
    drain(1.0)


try:
    port.reset_input_buffer()
    port.reset_output_buffer()
    port.write(b'{"type":"command","command":"set_debug_mode","enabled":true}\n')
    port.flush()
    note("TX enter debug mode")
    drain(1.0)
    send("run")
    send("zero_herkulex_here", id=4)
    send("read_herkulex_angle", id=4)

    for cycle in range(1, 3):
        note(f"CYCLE {cycle} START")
        send("set_herkulex_angle", id=4, angle_deg=-30, move_time_ms=500)
        note("HOLD -30 for 30 seconds")
        drain(30.0)
        send("set_herkulex_angle", id=4, angle_deg=30, move_time_ms=500)
        note("HOLD +30 for 30 seconds")
        drain(30.0)

    send("read_herkulex_angle", id=4)
    telemetry_age = None if last_telemetry is None else stamp() - last_telemetry
    note(f"RESULT invalid_json={json_errors} buffered_bytes={len(rx)} telemetry_age_s={telemetry_age}")
finally:
    port.close()
    LOG.write_text("\n".join(lines) + "\n", encoding="utf-8")
