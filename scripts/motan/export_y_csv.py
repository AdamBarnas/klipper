# Usage: python3 export_y_csv.py <log_prefix> <angle_chip_name> [duration_s] [skip_s]
import sys, csv
sys.path.insert(0, '/home/pi/klipper/scripts/motan')
import readlog

log_prefix   = sys.argv[1]
angle_chip   = sys.argv[2]
duration     = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
skip         = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
seg_time     = 0.0001  # 100 µs steps

lm = readlog.LogManager(log_prefix)
lm.setup_index()
lm.seek_time(skip)

step_h  = lm.setup_dataset("stepq(stepper_y)")
angle_h = lm.setup_dataset("angle(%s)" % angle_chip)
trapq_h = lm.setup_dataset("trapq(toolhead,y_velocity)")

n = int(duration / seg_time)
t0 = lm.get_start_time()

out_file = log_prefix + "_y_axis.csv"
with open(out_file, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_s", "stepper_y_mm", "encoder_mm", "y_velocity_mm_s"])
    for i in range(n):
        t = t0 + i * seg_time
        w.writerow([
            "%.6f" % (i * seg_time),
            "%.6f" % (step_h.pull_data(t) or 0.0),
            "%.6f" % (angle_h.pull_data(t) or 0.0),
            "%.6f" % (trapq_h.pull_data(t) or 0.0),
        ])

print("Exported %d rows → %s" % (n, out_file))
