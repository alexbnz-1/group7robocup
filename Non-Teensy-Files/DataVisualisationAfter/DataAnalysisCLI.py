"""
DataAnalysisCLI.py

Headless analysis interface for .rdbg robot recordings.

Designed for:
- Claude Code
- ChatGPT/Codex-style agents
- shell scripts
- CI pipelines
- Python automation

No PyQt dependency is required.

Examples:
    python DataAnalysisCLI.py summary Data/Test.rdbg
    python DataAnalysisCLI.py signals Data/Test.rdbg
    python DataAnalysisCLI.py stats Data/Test.rdbg --signal drive.left_rpm
    python DataAnalysisCLI.py range Data/Test.rdbg --signal drive.left_rpm --start 10 --end 20
    python DataAnalysisCLI.py events Data/Test.rdbg --start 10 --end 20
    python DataAnalysisCLI.py correlate Data/Test.rdbg --signals drive.left_rpm drive.right_rpm drive.left_current
    python DataAnalysisCLI.py context Data/Test.rdbg --time 18.5 --window 2.0
    python DataAnalysisCLI.py export-json Data/Test.rdbg --output analysis.json
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path
from typing import Any


def connect(path: str | Path) -> sqlite3.Connection:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    return conn


def metadata(conn):
    return dict(conn.execute("SELECT key, value FROM metadata").fetchall())


def signal_names(conn, numeric_only=False):
    if numeric_only:
        rows = conn.execute(
            "SELECT DISTINCT signal FROM telemetry WHERE value_num IS NOT NULL ORDER BY signal"
        ).fetchall()
    else:
        rows = conn.execute(
            "SELECT DISTINCT signal FROM telemetry ORDER BY signal"
        ).fetchall()
    return [r[0] for r in rows]


def series(conn, signal, start=None, end=None):
    sql = """
        SELECT elapsed_s, wall_time, robot_time, value_num, value_text, value_type
        FROM telemetry
        WHERE signal = ?
    """
    args = [signal]
    if start is not None:
        sql += " AND elapsed_s >= ?"
        args.append(start)
    if end is not None:
        sql += " AND elapsed_s <= ?"
        args.append(end)
    sql += " ORDER BY elapsed_s"
    return [dict(r) for r in conn.execute(sql, args)]


def numeric_values(rows):
    return [r["value_num"] for r in rows if r["value_num"] is not None]


def percentile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * p
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[lo]
    frac = index - lo
    return ordered[lo] * (1 - frac) + ordered[hi] * frac


def stats_for_signal(conn, signal, start=None, end=None):
    rows = series(conn, signal, start, end)
    vals = numeric_values(rows)
    result = {
        "signal": signal,
        "start_s": start,
        "end_s": end,
        "samples": len(rows),
        "numeric_samples": len(vals),
    }
    if not vals:
        return result

    result.update(
        {
            "min": min(vals),
            "max": max(vals),
            "mean": statistics.fmean(vals),
            "median": statistics.median(vals),
            "stdev": statistics.stdev(vals) if len(vals) > 1 else 0.0,
            "p05": percentile(vals, 0.05),
            "p25": percentile(vals, 0.25),
            "p75": percentile(vals, 0.75),
            "p95": percentile(vals, 0.95),
            "first": vals[0],
            "last": vals[-1],
            "delta": vals[-1] - vals[0],
        }
    )

    if len(rows) > 1:
        duration = rows[-1]["elapsed_s"] - rows[0]["elapsed_s"]
        result["duration_s"] = duration
        result["sample_rate_hz"] = ((len(rows) - 1) / duration) if duration > 0 else None

    return result


def events(conn, start=None, end=None):
    output = []

    def add_time_filter(base, col="elapsed_s"):
        args = []
        clauses = []
        if start is not None:
            clauses.append(f"{col} >= ?")
            args.append(start)
        if end is not None:
            clauses.append(f"{col} <= ?")
            args.append(end)
        if clauses:
            base += " WHERE " + " AND ".join(clauses)
        return base, args

    sql, args = add_time_filter("SELECT elapsed_s, wall_time, level, message FROM logs")
    for r in conn.execute(sql, args):
        output.append({
            "elapsed_s": r["elapsed_s"], "wall_time": r["wall_time"],
            "type": "log", "name": r["level"], "details": r["message"]
        })

    sql, args = add_time_filter("SELECT elapsed_s, wall_time, command, arguments_json FROM commands")
    for r in conn.execute(sql, args):
        output.append({
            "elapsed_s": r["elapsed_s"], "wall_time": r["wall_time"],
            "type": "command", "name": r["command"],
            "details": json.loads(r["arguments_json"])
        })

    sql, args = add_time_filter("SELECT elapsed_s, wall_time, name, value_json FROM parameters")
    for r in conn.execute(sql, args):
        output.append({
            "elapsed_s": r["elapsed_s"], "wall_time": r["wall_time"],
            "type": "parameter", "name": r["name"],
            "details": json.loads(r["value_json"])
        })

    sql, args = add_time_filter("SELECT elapsed_s, wall_time, state_json FROM states")
    for r in conn.execute(sql, args):
        output.append({
            "elapsed_s": r["elapsed_s"], "wall_time": r["wall_time"],
            "type": "state", "name": "",
            "details": json.loads(r["state_json"])
        })

    output.sort(key=lambda e: e["elapsed_s"])
    return output


def summary(conn):
    md = metadata(conn)
    count = conn.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
    sigs = conn.execute("SELECT COUNT(DISTINCT signal) FROM telemetry").fetchone()[0]
    duration = conn.execute("SELECT COALESCE(MAX(elapsed_s), 0) FROM telemetry").fetchone()[0]
    log_count = conn.execute("SELECT COUNT(*) FROM logs").fetchone()[0]
    cmd_count = conn.execute("SELECT COUNT(*) FROM commands").fetchone()[0]
    param_count = conn.execute("SELECT COUNT(*) FROM parameters").fetchone()[0]
    state_count = conn.execute("SELECT COUNT(*) FROM states").fetchone()[0]

    return {
        "metadata": md,
        "duration_s": duration,
        "telemetry_samples": count,
        "signal_count": sigs,
        "log_count": log_count,
        "command_count": cmd_count,
        "parameter_change_count": param_count,
        "state_count": state_count,
        "signals": signal_names(conn),
    }


def value_near(conn, signal, t):
    row = conn.execute(
        """
        SELECT elapsed_s, value_num, value_text, value_type
        FROM telemetry
        WHERE signal = ?
        ORDER BY ABS(elapsed_s - ?)
        LIMIT 1
        """,
        (signal, t),
    ).fetchone()
    return dict(row) if row else None


def context_at_time(conn, t, window):
    start = max(0.0, t - window)
    end = t + window
    return {
        "target_time_s": t,
        "window_s": window,
        "events": events(conn, start, end),
        "signals_at_target": {
            sig: value_near(conn, sig, t)
            for sig in signal_names(conn)
        },
        "stats_in_window": {
            sig: stats_for_signal(conn, sig, start, end)
            for sig in signal_names(conn, numeric_only=True)
        },
    }


def interpolate_series(rows, times):
    pts = [(r["elapsed_s"], r["value_num"]) for r in rows if r["value_num"] is not None]
    if not pts:
        return [None] * len(times)

    out = []
    j = 0
    for t in times:
        while j + 1 < len(pts) and pts[j + 1][0] < t:
            j += 1

        if j + 1 >= len(pts):
            out.append(pts[-1][1] if abs(pts[-1][0] - t) < 1e-9 else None)
            continue

        t0, v0 = pts[j]
        t1, v1 = pts[j + 1]

        if t < t0 or t > t1:
            out.append(None)
        elif t1 == t0:
            out.append(v0)
        else:
            f = (t - t0) / (t1 - t0)
            out.append(v0 + f * (v1 - v0))
    return out


def correlation_matrix(conn, signals, start=None, end=None):
    raw = {s: series(conn, s, start, end) for s in signals}
    all_times = sorted({
        r["elapsed_s"]
        for s in signals
        for r in raw[s]
        if r["value_num"] is not None
    })

    # Downsample union of timestamps to keep analysis manageable.
    if len(all_times) > 5000:
        step = max(1, len(all_times) // 5000)
        all_times = all_times[::step]

    aligned = {s: interpolate_series(raw[s], all_times) for s in signals}

    def corr(a, b):
        pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
        if len(pairs) < 3:
            return None
        xs, ys = zip(*pairs)
        mx = statistics.fmean(xs)
        my = statistics.fmean(ys)
        sx = sum((x - mx) ** 2 for x in xs)
        sy = sum((y - my) ** 2 for y in ys)
        if sx == 0 or sy == 0:
            return None
        cov = sum((x - mx) * (y - my) for x, y in pairs)
        return cov / math.sqrt(sx * sy)

    matrix = {}
    for a in signals:
        matrix[a] = {}
        for b in signals:
            matrix[a][b] = corr(aligned[a], aligned[b])

    return {
        "signals": signals,
        "start_s": start,
        "end_s": end,
        "aligned_points": len(all_times),
        "correlation": matrix,
    }


def full_export(conn):
    result = summary(conn)
    result["statistics"] = {
        sig: stats_for_signal(conn, sig)
        for sig in signal_names(conn, numeric_only=True)
    }
    result["events"] = events(conn)
    return result


def dump(data, pretty=True):
    print(json.dumps(data, indent=2 if pretty else None, ensure_ascii=False))


def build_parser():
    parser = argparse.ArgumentParser(
        description="Headless analyzer for Robot Debug .rdbg recordings"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("summary")
    p.add_argument("file")

    p = sub.add_parser("signals")
    p.add_argument("file")
    p.add_argument("--numeric-only", action="store_true")

    p = sub.add_parser("stats")
    p.add_argument("file")
    p.add_argument("--signal", required=True)
    p.add_argument("--start", type=float)
    p.add_argument("--end", type=float)

    p = sub.add_parser("range")
    p.add_argument("file")
    p.add_argument("--signal", required=True)
    p.add_argument("--start", type=float)
    p.add_argument("--end", type=float)

    p = sub.add_parser("events")
    p.add_argument("file")
    p.add_argument("--start", type=float)
    p.add_argument("--end", type=float)

    p = sub.add_parser("context")
    p.add_argument("file")
    p.add_argument("--time", type=float, required=True)
    p.add_argument("--window", type=float, default=2.0)

    p = sub.add_parser("correlate")
    p.add_argument("file")
    p.add_argument("--signals", nargs="+", required=True)
    p.add_argument("--start", type=float)
    p.add_argument("--end", type=float)

    p = sub.add_parser("export-json")
    p.add_argument("file")
    p.add_argument("--output", required=True)

    return parser


def main():
    args = build_parser().parse_args()
    conn = connect(args.file)

    try:
        if args.command == "summary":
            dump(summary(conn))

        elif args.command == "signals":
            dump(signal_names(conn, numeric_only=args.numeric_only))

        elif args.command == "stats":
            dump(stats_for_signal(conn, args.signal, args.start, args.end))

        elif args.command == "range":
            dump(series(conn, args.signal, args.start, args.end))

        elif args.command == "events":
            dump(events(conn, args.start, args.end))

        elif args.command == "context":
            dump(context_at_time(conn, args.time, args.window))

        elif args.command == "correlate":
            dump(correlation_matrix(conn, args.signals, args.start, args.end))

        elif args.command == "export-json":
            output = Path(args.output)
            output.write_text(
                json.dumps(full_export(conn), indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            dump({"written": str(output)})

    finally:
        conn.close()


if __name__ == "__main__":
    main()
