#!/usr/bin/env python3
# Export stepper + encoder data for a given axis to CSV
# Usage: python3 export_axis_csv.py <log_prefix> <axis> [angle_chip] [duration_s] [skip_s]
#   axis       : x or y
#   angle_chip : name of [angle <chip>] section in printer.cfg, or omit if no encoder
#   duration_s : seconds to export (default 10)
#   skip_s     : seconds to skip from start (default 0)
# Example:
#   python3 export_axis_csv.py ~/y_capture y y_angle_sensor 10.0 0.0

import sys, os, csv
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import readlog

if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(1)

log_prefix  = sys.argv[1]
axis        = sys.argv[2].lower()
angle_chip  = sys.argv[3] if len(sys.argv) > 3 else None
duration    = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0
skip        = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
seg_time    = 0.0001  # 100 µs resolution

if axis not in ("x", "y", "z"):
    print("ERROR: axis must be x, y, or z")
    sys.exit(1)

stepper = "stepper_" + axis

lm = readlog.LogManager(log_prefix)
lm.setup_index()
lm.seek_time(skip)

step_h  = lm.setup_dataset("stepq(%s)" % stepper)
trapq_h = lm.setup_dataset("trapq(toolhead,%s_velocity)" % axis)

angle_h = None
if angle_chip:
    try:
        angle_h = lm.setup_dataset("angle(%s)" % angle_chip)
    except readlog.error as e:
        print("WARNING: encoder not available — %s" % e)
        print("  Available subscriptions in log:")
        for k in lm.log_subscriptions:
            print("    %s" % k)
        angle_h = None

n   = int(duration / seg_time)
t0  = lm.get_start_time()

columns = ["time_s", "stepper_%s_mm" % axis]
if angle_h:
    columns.append("encoder_mm")
columns.append("%s_velocity_mm_s" % axis)

out_file = "%s_%s_axis.csv" % (log_prefix, axis)
with open(out_file, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(columns)
    for i in range(n):
        t   = t0 + i * seg_time
        s   = step_h.pull_data(t)
        vel = trapq_h.pull_data(t)
        row = [
            "%.6f" % (i * seg_time),
            "%.6f" % (s   if s   is not None else 0.0),
        ]
        if angle_h:
            enc = angle_h.pull_data(t)
            row.append("%.6f" % (enc if enc is not None else 0.0))
        row.append("%.6f" % (vel if vel is not None else 0.0))
        w.writerow(row)

print("Exported %d rows → %s" % (n, out_file))
if angle_h is None and angle_chip:
    print("NOTE: encoder column was skipped — check data_logger --subscribe argument")
