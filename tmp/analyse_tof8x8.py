import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path

path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("Non-Teensy-Files/DataVisualisationAfter/Data/Weight8x8.rdbg")
conn = sqlite3.connect(f"file:{path.as_posix()}?mode=ro", uri=True)
rows = conn.execute(
    "SELECT elapsed_s, line FROM raw_serial WHERE line LIKE '%tof_8x8%' ORDER BY elapsed_s"
).fetchall()

frames = []
for elapsed, line in rows:
    start = line.find("{")
    if start < 0:
        continue
    try:
        message = json.loads(line[start:])
    except json.JSONDecodeError:
        continue
    if message.get("type") == "tof_8x8" and message.get("valid") and len(message.get("data", [])) == 64:
        frames.append((elapsed, message))

print("rows", len(rows), "valid_frames", len(frames))
if not frames:
    raise SystemExit

times = [f[0] for f in frames]
gaps = [b-a for a,b in zip(times,times[1:])]
data = [[float(v) for v in f[1]["data"]] for f in frames]
valid_counts = [sum(math.isfinite(v) and v > 0 for v in frame) for frame in data]
all_valid = [v for frame in data for v in frame if math.isfinite(v) and v > 0]
print("duration_s", times[-1]-times[0])
print("rate_hz", (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0)
print("gap_median_max", statistics.median(gaps) if gaps else 0, max(gaps) if gaps else 0)
print("valid_zones_min_med_max", min(valid_counts), statistics.median(valid_counts), max(valid_counts))
print("distance_mm_min_med_max", min(all_valid), statistics.median(all_valid), max(all_valid))

zone_medians=[]; zone_mads=[]; zone_ranges=[]
for i in range(64):
    vals=[f[i] for f in data if math.isfinite(f[i]) and f[i]>0]
    med=statistics.median(vals)
    zone_medians.append(med)
    zone_mads.append(statistics.median(abs(v-med) for v in vals))
    zone_ranges.append(max(vals)-min(vals))
print("zone_median_grid")
for y in range(8): print(" ".join(f"{zone_medians[y*8+x]:6.0f}" for x in range(8)))
print("zone_MAD_grid")
for y in range(8): print(" ".join(f"{zone_mads[y*8+x]:6.0f}" for x in range(8)))
worst=sorted(range(64), key=lambda i: zone_mads[i], reverse=True)[:10]
print("noisiest", [(i//8,i%8,zone_medians[i],zone_mads[i],zone_ranges[i]) for i in worst])

frame_medians=[statistics.median(v for v in f if v>0) for f in data]
print("frame_median_min_max",min(frame_medians),max(frame_medians))
print("first_last_frame", frames[0][1].get("frame"),frames[-1][1].get("frame"))

pair_diffs=[]
large_changes=0
valid_pairs=0
for a,b in zip(data,data[1:]):
    for x,y in zip(a,b):
        if x>0 and y>0:
            d=abs(y-x); pair_diffs.append(d); valid_pairs += 1
            large_changes += d > 250
print("successive_abs_change_med_p95_max", statistics.median(pair_diffs), sorted(pair_diffs)[int(.95*(len(pair_diffs)-1))], max(pair_diffs))
print("successive_changes_over_250_pct",100*large_changes/valid_pairs)
print("timeline")
for n in [0,32,65,98,131,164,197,230,262]:
    f=data[n]
    rowmed=[statistics.median(v for v in f[y*8:(y+1)*8] if v>0) for y in range(8)]
    print(round(times[n]-times[0],1), "frame", frames[n][1].get("frame"), "rows", [round(v) for v in rowmed], "min", min(v for v in f if v>0))
