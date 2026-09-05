"""
DataAnalysisCLI.py

Headless analysis interface for .rdbg robot recordings.

Designed for:
- Claude Code
- shell scripts
- automated analysis
- AI/code-agent workflows

No PyQt dependency is required.

Examples:
    python DataAnalysisCLI.py summary Data/Test.rdbg
    python DataAnalysisCLI.py signals Data/Test.rdbg
    python DataAnalysisCLI.py faults Data/Test.rdbg
    python DataAnalysisCLI.py stats Data/Test.rdbg --signal drive.left_rpm
    python DataAnalysisCLI.py context Data/Test.rdbg --time 18.5 --window 2
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
from pathlib import Path


def connect(path: str | Path) -> sqlite3.Connection:
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(path)

    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row

    return conn


def table_exists(
    conn: sqlite3.Connection,
    table_name: str,
) -> bool:
    row = conn.execute(
        """
        SELECT name
        FROM sqlite_master
        WHERE type = 'table'
          AND name = ?
        """,
        (table_name,),
    ).fetchone()

    return row is not None


def metadata(conn):
    return dict(
        conn.execute(
            """
            SELECT key, value
            FROM metadata
            """
        ).fetchall()
    )


def signal_names(
    conn,
    numeric_only=False,
):
    if numeric_only:
        rows = conn.execute(
            """
            SELECT DISTINCT signal
            FROM telemetry
            WHERE value_num IS NOT NULL
            ORDER BY signal
            """
        ).fetchall()

    else:
        rows = conn.execute(
            """
            SELECT DISTINCT signal
            FROM telemetry
            ORDER BY signal
            """
        ).fetchall()

    return [
        row[0]
        for row in rows
    ]


def series(
    conn,
    signal,
    start=None,
    end=None,
):
    sql = """
        SELECT
            elapsed_s,
            wall_time,
            robot_time,
            value_num,
            value_text,
            value_type
        FROM telemetry
        WHERE signal = ?
    """

    arguments = [
        signal
    ]

    if start is not None:
        sql += """
            AND elapsed_s >= ?
        """

        arguments.append(
            start
        )

    if end is not None:
        sql += """
            AND elapsed_s <= ?
        """

        arguments.append(
            end
        )

    sql += """
        ORDER BY elapsed_s
    """

    return [
        dict(row)
        for row in conn.execute(
            sql,
            arguments,
        )
    ]


def numeric_values(rows):
    return [
        row["value_num"]
        for row in rows
        if row["value_num"]
        is not None
    ]


def percentile(
    values,
    fraction,
):
    if not values:
        return None

    ordered = sorted(
        values
    )

    if len(ordered) == 1:
        return ordered[0]

    index = (
        len(ordered)
        - 1
    ) * fraction

    low = math.floor(
        index
    )

    high = math.ceil(
        index
    )

    if low == high:
        return ordered[low]

    interpolation = (
        index
        - low
    )

    return (
        ordered[low]
        * (1 - interpolation)
        + ordered[high]
        * interpolation
    )


def stats_for_signal(
    conn,
    signal,
    start=None,
    end=None,
):
    rows = series(
        conn,
        signal,
        start,
        end,
    )

    values = numeric_values(
        rows
    )

    result = {
        "signal": signal,
        "start_s": start,
        "end_s": end,
        "samples": len(rows),
        "numeric_samples": len(values),
    }

    if not values:
        return result

    result.update(
        {
            "min": min(values),
            "max": max(values),
            "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "stdev": (
                statistics.stdev(values)
                if len(values) > 1
                else 0.0
            ),
            "p05": percentile(values, 0.05),
            "p25": percentile(values, 0.25),
            "p75": percentile(values, 0.75),
            "p95": percentile(values, 0.95),
            "first": values[0],
            "last": values[-1],
            "delta": (
                values[-1]
                - values[0]
            ),
        }
    )

    if len(rows) > 1:
        duration = (
            rows[-1]["elapsed_s"]
            - rows[0]["elapsed_s"]
        )

        result[
            "duration_s"
        ] = duration

        result[
            "sample_rate_hz"
        ] = (
            (len(rows) - 1)
            / duration
            if duration > 0
            else None
        )

    return result


def faults(
    conn,
    start=None,
    end=None,
):
    if not table_exists(
        conn,
        "faults",
    ):
        return []

    sql = """
        SELECT
            elapsed_s,
            wall_time,
            label
        FROM faults
        WHERE 1 = 1
    """

    arguments = []

    if start is not None:
        sql += """
            AND elapsed_s >= ?
        """

        arguments.append(
            start
        )

    if end is not None:
        sql += """
            AND elapsed_s <= ?
        """

        arguments.append(
            end
        )

    sql += """
        ORDER BY elapsed_s
    """

    return [
        dict(row)
        for row in conn.execute(
            sql,
            arguments,
        )
    ]


def events(
    conn,
    start=None,
    end=None,
):
    output = []

    for fault in faults(
        conn,
        start,
        end,
    ):
        output.append(
            {
                "elapsed_s": fault[
                    "elapsed_s"
                ],
                "wall_time": fault[
                    "wall_time"
                ],
                "type": "fault",
                "name": fault[
                    "label"
                ],
                "details": (
                    "Manual fault marker"
                ),
            }
        )

    def add_time_filter(
        base,
        column="elapsed_s",
    ):
        arguments = []
        clauses = []

        if start is not None:
            clauses.append(
                f"{column} >= ?"
            )

            arguments.append(
                start
            )

        if end is not None:
            clauses.append(
                f"{column} <= ?"
            )

            arguments.append(
                end
            )

        if clauses:
            base += (
                " WHERE "
                + " AND ".join(
                    clauses
                )
            )

        return (
            base,
            arguments,
        )

    sql, arguments = add_time_filter(
        """
        SELECT
            elapsed_s,
            wall_time,
            level,
            message
        FROM logs
        """
    )

    for row in conn.execute(
        sql,
        arguments,
    ):
        output.append(
            {
                "elapsed_s": row[
                    "elapsed_s"
                ],
                "wall_time": row[
                    "wall_time"
                ],
                "type": "log",
                "name": row[
                    "level"
                ],
                "details": row[
                    "message"
                ],
            }
        )

    sql, arguments = add_time_filter(
        """
        SELECT
            elapsed_s,
            wall_time,
            command,
            arguments_json
        FROM commands
        """
    )

    for row in conn.execute(
        sql,
        arguments,
    ):
        output.append(
            {
                "elapsed_s": row[
                    "elapsed_s"
                ],
                "wall_time": row[
                    "wall_time"
                ],
                "type": "command",
                "name": row[
                    "command"
                ],
                "details": json.loads(
                    row[
                        "arguments_json"
                    ]
                ),
            }
        )

    sql, arguments = add_time_filter(
        """
        SELECT
            elapsed_s,
            wall_time,
            name,
            value_json
        FROM parameters
        """
    )

    for row in conn.execute(
        sql,
        arguments,
    ):
        output.append(
            {
                "elapsed_s": row[
                    "elapsed_s"
                ],
                "wall_time": row[
                    "wall_time"
                ],
                "type": "parameter",
                "name": row[
                    "name"
                ],
                "details": json.loads(
                    row[
                        "value_json"
                    ]
                ),
            }
        )

    sql, arguments = add_time_filter(
        """
        SELECT
            elapsed_s,
            wall_time,
            state_json
        FROM states
        """
    )

    for row in conn.execute(
        sql,
        arguments,
    ):
        output.append(
            {
                "elapsed_s": row[
                    "elapsed_s"
                ],
                "wall_time": row[
                    "wall_time"
                ],
                "type": "state",
                "name": "",
                "details": json.loads(
                    row[
                        "state_json"
                    ]
                ),
            }
        )

    output.sort(
        key=lambda event: event[
            "elapsed_s"
        ]
    )

    return output


def summary(conn):
    meta = metadata(
        conn
    )

    sample_count = conn.execute(
        """
        SELECT COUNT(*)
        FROM telemetry
        """
    ).fetchone()[0]

    signal_count = conn.execute(
        """
        SELECT COUNT(
            DISTINCT signal
        )
        FROM telemetry
        """
    ).fetchone()[0]

    duration = conn.execute(
        """
        SELECT COALESCE(
            MAX(elapsed_s),
            0
        )
        FROM telemetry
        """
    ).fetchone()[0]

    log_count = conn.execute(
        """
        SELECT COUNT(*)
        FROM logs
        """
    ).fetchone()[0]

    command_count = conn.execute(
        """
        SELECT COUNT(*)
        FROM commands
        """
    ).fetchone()[0]

    parameter_count = conn.execute(
        """
        SELECT COUNT(*)
        FROM parameters
        """
    ).fetchone()[0]

    state_count = conn.execute(
        """
        SELECT COUNT(*)
        FROM states
        """
    ).fetchone()[0]

    fault_count = len(
        faults(
            conn
        )
    )

    return {
        "metadata": meta,
        "duration_s": duration,
        "telemetry_samples": sample_count,
        "signal_count": signal_count,
        "log_count": log_count,
        "command_count": command_count,
        "parameter_change_count": parameter_count,
        "state_count": state_count,
        "fault_marker_count": fault_count,
        "faults": faults(
            conn
        ),
        "signals": signal_names(
            conn
        ),
    }


def value_near(
    conn,
    signal,
    target_time,
):
    row = conn.execute(
        """
        SELECT
            elapsed_s,
            value_num,
            value_text,
            value_type
        FROM telemetry
        WHERE signal = ?
        ORDER BY ABS(
            elapsed_s - ?
        )
        LIMIT 1
        """,
        (
            signal,
            target_time,
        ),
    ).fetchone()

    return (
        dict(row)
        if row
        else None
    )


def context_at_time(
    conn,
    target_time,
    window,
):
    start = max(
        0.0,
        target_time - window,
    )

    end = (
        target_time
        + window
    )

    return {
        "target_time_s": target_time,
        "window_s": window,
        "faults": faults(
            conn,
            start,
            end,
        ),
        "events": events(
            conn,
            start,
            end,
        ),
        "signals_at_target": {
            signal: value_near(
                conn,
                signal,
                target_time,
            )
            for signal in signal_names(
                conn
            )
        },
        "stats_in_window": {
            signal: stats_for_signal(
                conn,
                signal,
                start,
                end,
            )
            for signal in signal_names(
                conn,
                numeric_only=True,
            )
        },
    }


def full_export(conn):
    result = summary(
        conn
    )

    result[
        "statistics"
    ] = {
        signal: stats_for_signal(
            conn,
            signal,
        )
        for signal in signal_names(
            conn,
            numeric_only=True,
        )
    }

    result[
        "events"
    ] = events(
        conn
    )

    return result


def dump(data):
    print(
        json.dumps(
            data,
            indent=2,
            ensure_ascii=False,
        )
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Headless analyzer for "
            "Robot Debug .rdbg recordings"
        )
    )

    sub = parser.add_subparsers(
        dest="command",
        required=True,
    )

    parser_summary = sub.add_parser(
        "summary"
    )

    parser_summary.add_argument(
        "file"
    )

    parser_signals = sub.add_parser(
        "signals"
    )

    parser_signals.add_argument(
        "file"
    )

    parser_signals.add_argument(
        "--numeric-only",
        action="store_true",
    )

    parser_faults = sub.add_parser(
        "faults"
    )

    parser_faults.add_argument(
        "file"
    )

    parser_faults.add_argument(
        "--start",
        type=float,
    )

    parser_faults.add_argument(
        "--end",
        type=float,
    )

    parser_stats = sub.add_parser(
        "stats"
    )

    parser_stats.add_argument(
        "file"
    )

    parser_stats.add_argument(
        "--signal",
        required=True,
    )

    parser_stats.add_argument(
        "--start",
        type=float,
    )

    parser_stats.add_argument(
        "--end",
        type=float,
    )

    parser_range = sub.add_parser(
        "range"
    )

    parser_range.add_argument(
        "file"
    )

    parser_range.add_argument(
        "--signal",
        required=True,
    )

    parser_range.add_argument(
        "--start",
        type=float,
    )

    parser_range.add_argument(
        "--end",
        type=float,
    )

    parser_events = sub.add_parser(
        "events"
    )

    parser_events.add_argument(
        "file"
    )

    parser_events.add_argument(
        "--start",
        type=float,
    )

    parser_events.add_argument(
        "--end",
        type=float,
    )

    parser_context = sub.add_parser(
        "context"
    )

    parser_context.add_argument(
        "file"
    )

    parser_context.add_argument(
        "--time",
        type=float,
        required=True,
    )

    parser_context.add_argument(
        "--window",
        type=float,
        default=2.0,
    )

    parser_export = sub.add_parser(
        "export-json"
    )

    parser_export.add_argument(
        "file"
    )

    parser_export.add_argument(
        "--output",
        required=True,
    )

    return parser


def main():
    args = (
        build_parser()
        .parse_args()
    )

    conn = connect(
        args.file
    )

    try:
        if args.command == "summary":
            dump(
                summary(
                    conn
                )
            )

        elif args.command == "signals":
            dump(
                signal_names(
                    conn,
                    numeric_only=args.numeric_only,
                )
            )

        elif args.command == "faults":
            dump(
                faults(
                    conn,
                    args.start,
                    args.end,
                )
            )

        elif args.command == "stats":
            dump(
                stats_for_signal(
                    conn,
                    args.signal,
                    args.start,
                    args.end,
                )
            )

        elif args.command == "range":
            dump(
                series(
                    conn,
                    args.signal,
                    args.start,
                    args.end,
                )
            )

        elif args.command == "events":
            dump(
                events(
                    conn,
                    args.start,
                    args.end,
                )
            )

        elif args.command == "context":
            dump(
                context_at_time(
                    conn,
                    args.time,
                    args.window,
                )
            )

        elif args.command == "export-json":
            output = Path(
                args.output
            )

            output.write_text(
                json.dumps(
                    full_export(
                        conn
                    ),
                    indent=2,
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )

            dump(
                {
                    "written": str(
                        output
                    )
                }
            )

    finally:
        conn.close()


if __name__ == "__main__":
    main()
