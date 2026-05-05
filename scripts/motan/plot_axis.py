#!/usr/bin/env python3
# Visualize stepper vs encoder data from export_axis_csv.py output
# Usage: python3 plot_axis.py <csv_file> [output.png]
# Axis is detected automatically from CSV column names.
import sys
import csv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames

        # Detect axis from stepper_<axis>_mm column
        axis = "?"
        stepper_col = vel_col = None
        for h in headers:
            if h.startswith("stepper_") and h.endswith("_mm"):
                axis = h.split("_")[1]
                stepper_col = h
            if h.endswith("_velocity_mm_s"):
                vel_col = h
        has_encoder = "encoder_mm" in headers

        time, stepper, encoder, velocity = [], [], [], []
        for row in reader:
            time.append(float(row["time_s"]))
            stepper.append(float(row[stepper_col]))
            if has_encoder:
                encoder.append(float(row["encoder_mm"]))
            velocity.append(float(row[vel_col]))

    return axis, has_encoder, time, stepper, encoder, velocity

def main():
    if len(sys.argv) < 2:
        print("Usage: plot_axis.py <csv_file> [output.png]")
        sys.exit(1)

    csv_path = sys.argv[1]
    output   = sys.argv[2] if len(sys.argv) > 2 else None

    axis, has_encoder, time, stepper, encoder, velocity = load_csv(csv_path)
    AXIS = axis.upper()

    nrows         = 3 if has_encoder else 2
    height_ratios = [2, 1, 1] if has_encoder else [2, 1]

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("%s Axis — Stepper vs Encoder" % AXIS if has_encoder
                 else "%s Axis — Stepper position" % AXIS, fontsize=13)
    gs = gridspec.GridSpec(nrows, 1, height_ratios=height_ratios, hspace=0.08)

    # --- Position ---
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(time, stepper, label="Stepper (commanded)", color="royalblue",
             linewidth=0.8)
    if has_encoder:
        ax1.plot(time, encoder, label="Encoder (measured)", color="tomato",
                 linewidth=0.8, alpha=0.85)
    ax1.set_ylabel("Position (mm)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, linewidth=0.4, alpha=0.6)
    ax1.tick_params(labelbottom=False)

    # --- Following error (only when encoder present) ---
    if has_encoder:
        error = [s - e for s, e in zip(stepper, encoder)]
        ax2 = fig.add_subplot(gs[1], sharex=ax1)
        ax2.plot(time, error, color="darkorange", linewidth=0.7)
        ax2.axhline(0, color="black", linewidth=0.5, linestyle="--")
        ax2.set_ylabel("Error\n(stepper−enc, mm)")
        ax2.grid(True, linewidth=0.4, alpha=0.6)
        ax2.tick_params(labelbottom=False)

    # --- Velocity ---
    ax_vel = fig.add_subplot(gs[-1], sharex=ax1)
    ax_vel.plot(time, velocity, color="seagreen", linewidth=0.7)
    ax_vel.set_ylabel("Velocity (mm/s)")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.grid(True, linewidth=0.4, alpha=0.6)

    plt.tight_layout()
    if output:
        fig.savefig(output, dpi=150, bbox_inches="tight")
        print("Saved →", output)
    else:
        plt.show()

if __name__ == "__main__":
    main()
