#!/usr/bin/env python3
# Export stepper + encoder data for one or more axes to CSV
# Usage: python3 export_axis_csv.py <log_prefix> <axis[,axis...]> [chip[,chip...]] [duration_s] [skip_s]
#   axis    : x, y, or z — comma-separated for multiple (e.g. x,y)
#   chip    : [angle <name>] section(s) matching axis order; use - to skip encoder for an axis
#   duration_s : seconds to export (default 10)
#   skip_s     : seconds to skip from start (default 0)
# Examples:
#   python3 export_axis_csv.py ~/xy_capture x,y x_angle_sensor,y_angle_sensor 10.0 0.0
#   python3 export_axis_csv.py ~/y_capture y y_angle_sensor 10.0 0.0

import sys, os, csv
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import readlog

if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(1)

log_prefix = sys.argv[1]
axes       = [a.strip().lower() for a in sys.argv[2].split(",")]
chip_arg   = sys.argv[3] if len(sys.argv) > 3 else None
duration   = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0
skip       = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
seg_time   = 0.0001  # 100 µs resolution

# Build per-axis chip list (pad / trim to match axes)
if chip_arg:
    chips = [c.strip() if c.strip() != "-" else None
             for c in chip_arg.split(",")]
    while len(chips) < len(axes):
        chips.append(None)
    chips = chips[:len(axes)]
else:
    chips = [None] * len(axes)

for axis in axes:
    if axis not in ("x", "y", "z"):
        print("ERROR: axis must be x, y, or z (got %r)" % axis)
        sys.exit(1)

lm = readlog.LogManager(log_prefix)
lm.setup_index()
lm.seek_time(skip)

n  = int(duration / seg_time)
t0 = lm.get_start_time()

# Set up all dataset handles before iterating so the dispatcher is fully wired
axis_data = []
for axis, angle_chip in zip(axes, chips):
    step_h  = lm.setup_dataset("stepq(stepper_%s)" % axis)
    trapq_h = lm.setup_dataset("trapq(toolhead,%s_velocity)" % axis)
    angle_h = None
    if angle_chip:
        try:
            angle_h = lm.setup_dataset("angle(%s)" % angle_chip)
        except readlog.error as e:
            print("WARNING: encoder not available for %s — %s" % (axis, e))
            print("  Available subscriptions in log:")
            for k in lm.log_subscriptions:
                print("    %s" % k)
    axis_data.append((axis, step_h, trapq_h, angle_h, angle_chip))

# Write one CSV file per axis
for axis, step_h, trapq_h, angle_h, angle_chip in axis_data:
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
        print("NOTE: encoder column skipped for %s — check --subscribe argument" % axis)
