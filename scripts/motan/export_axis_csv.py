#!/usr/bin/env python3
# Export stepper + encoder + accelerometer data for one or more axes to CSV
# Usage: python3 export_axis_csv.py <log_prefix> <axes> [chips]
#                                   [--accel SENSOR] [--duration N] [--skip N]
#   axes    : x, y, or z — comma-separated (e.g. x,y)
#   chips   : angle sensor name(s) matching axes order, comma-separated;
#             use - to skip encoder for an axis
#   --accel : [adxl345] section name in printer.cfg (default: adxl345)
#   --duration : seconds to export (default 10)
#   --skip     : seconds to skip from start (default 0)
# Examples:
#   python3 export_axis_csv.py ~/xy_capture x,y x_angle_sensor,y_angle_sensor \
#       --accel adxl345 --duration 10 --skip 0
#   python3 export_axis_csv.py ~/y_capture y y_angle_sensor

import sys, os, csv, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import readlog

def parse_args():
    p = argparse.ArgumentParser(
        description="Export stepper / encoder / accelerometer data to CSV")
    p.add_argument("log_prefix")
    p.add_argument("axes",
                   help="Comma-separated axes to export: x,y or x,y,z")
    p.add_argument("chips", nargs="?", default=None,
                   help="Comma-separated angle sensor names (use - to omit "
                        "encoder for an axis)")
    p.add_argument("--accel", default=None, metavar="SENSOR",
                   help="Accelerometer sensor name from printer.cfg "
                        "(e.g. adxl345); omit to skip accel columns")
    p.add_argument("--duration", type=float, default=10.0,
                   help="Seconds to export (default 10)")
    p.add_argument("--skip", type=float, default=0.0,
                   help="Seconds to skip from start (default 0)")
    return p.parse_args()

def main():
    args     = parse_args()
    axes     = [a.strip().lower() for a in args.axes.split(",")]
    seg_time = 0.0001  # 100 µs resolution

    for axis in axes:
        if axis not in ("x", "y", "z"):
            print("ERROR: axis must be x, y, or z (got %r)" % axis)
            sys.exit(1)

    # Build per-axis chip list
    if args.chips:
        chips = [c.strip() if c.strip() != "-" else None
                 for c in args.chips.split(",")]
        while len(chips) < len(axes):
            chips.append(None)
        chips = chips[:len(axes)]
    else:
        chips = [None] * len(axes)

    lm = readlog.LogManager(args.log_prefix)
    lm.setup_index()
    lm.seek_time(args.skip)

    n  = int(args.duration / seg_time)
    t0 = lm.get_start_time()

    # --- set up all dataset handles upfront so jdispatch routes correctly ---

    # Per-axis stepper / trapq / angle handles
    axis_data = []
    for axis, chip in zip(axes, chips):
        step_h  = lm.setup_dataset("stepq(stepper_%s)" % axis)
        trapq_h = lm.setup_dataset("trapq(toolhead,%s_velocity)" % axis)
        angle_h = None
        if chip:
            try:
                angle_h = lm.setup_dataset("angle(%s)" % chip)
            except readlog.error as e:
                print("WARNING: encoder not available for %s — %s" % (axis, e))
                print("  Available subscriptions:")
                for k in lm.log_subscriptions:
                    print("    %s" % k)
        axis_data.append((axis, step_h, trapq_h, angle_h, chip))

    # Single shared accelerometer handles (all 3 spatial axes)
    accel_h = {}
    if args.accel:
        for ax in ("x", "y", "z"):
            try:
                accel_h[ax] = lm.setup_dataset(
                    "accelerometer(%s,%s)" % (args.accel, ax))
            except readlog.error as e:
                print("WARNING: accel %s axis not available — %s" % (ax, e))

    # --- write one CSV per stepper axis ---
    for axis, step_h, trapq_h, angle_h, chip in axis_data:
        columns = ["time_s", "stepper_%s_mm" % axis]
        if angle_h:
            columns.append("encoder_mm")
        columns.append("%s_velocity_mm_s" % axis)
        if accel_h:
            columns += ["accel_x_mm_s2", "accel_y_mm_s2", "accel_z_mm_s2"]

        out_file = "%s_%s_axis.csv" % (args.log_prefix, axis)
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
                if accel_h:
                    for ax in ("x", "y", "z"):
                        h   = accel_h.get(ax)
                        val = h.pull_data(t) if h else 0.0
                        row.append("%.4f" % (val if val is not None else 0.0))
                w.writerow(row)

        print("Exported %d rows → %s" % (n, out_file))
        if angle_h is None and chip:
            print("NOTE: encoder column skipped for %s" % axis)
    if args.accel and not accel_h:
        print("NOTE: no accelerometer columns written — check --subscribe arg")

if __name__ == "__main__":
    main()