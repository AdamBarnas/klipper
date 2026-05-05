#!/usr/bin/env python3
# Visualize Y axis step vs encoder data from export_y_csv.py output
# Usage: python3 plot_y_axis.py <csv_file> [output.png]
import sys
import csv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def load_csv(path):
    time, stepper, encoder, velocity = [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row["time_s"]))
            stepper.append(float(row["stepper_y_mm"]))
            encoder.append(float(row["encoder_mm"]))
            velocity.append(float(row["y_velocity_mm_s"]))
    return time, stepper, encoder, velocity

def main():
    if len(sys.argv) < 2:
        print("Usage: plot_y_axis.py <csv_file> [output.png]")
        sys.exit(1)

    csv_path = sys.argv[1]
    output   = sys.argv[2] if len(sys.argv) > 2 else None

    time, stepper, encoder, velocity = load_csv(csv_path)
    error = [s - e for s, e in zip(stepper, encoder)]

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Y Axis — Stepper vs Encoder", fontsize=13)
    gs = gridspec.GridSpec(3, 1, height_ratios=[2, 1, 1], hspace=0.08)

    # --- Position ---
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(time, stepper, label="Stepper (commanded)", color="royalblue",
             linewidth=0.8)
    ax1.plot(time, encoder, label="Encoder (measured)",  color="tomato",
             linewidth=0.8, alpha=0.85)
    ax1.set_ylabel("Position (mm)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, linewidth=0.4, alpha=0.6)
    ax1.tick_params(labelbottom=False)

    # --- Following error ---
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(time, error, color="darkorange", linewidth=0.7)
    ax2.axhline(0, color="black", linewidth=0.5, linestyle="--")
    ax2.set_ylabel("Error (mm)\nstepper − encoder")
    ax2.grid(True, linewidth=0.4, alpha=0.6)
    ax2.tick_params(labelbottom=False)

    # --- Velocity ---
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(time, velocity, color="seagreen", linewidth=0.7)
    ax3.set_ylabel("Velocity (mm/s)")
    ax3.set_xlabel("Time (s)")
    ax3.grid(True, linewidth=0.4, alpha=0.6)

    if output:
        fig.savefig(output, dpi=150, bbox_inches="tight")
        print("Saved →", output)
    else:
        plt.show()

if __name__ == "__main__":
    main()
