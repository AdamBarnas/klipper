#!/usr/bin/env python3
# Visualize stepper / encoder / accelerometer data from export_axis_csv.py
# Usage: python3 plot_axis.py <csv_file> [csv_file2 ...] [output.png]
# Axis and available channels are detected automatically from column names.
# Multiple CSV files are plotted side-by-side in one figure.
import sys
import csv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

ACCEL_COLORS = {"x": "crimson", "y": "darkorange", "z": "mediumpurple"}

def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames

        axis = "?"
        stepper_col = vel_col = None
        for h in headers:
            if h.startswith("stepper_") and h.endswith("_mm"):
                axis = h.split("_")[1]
                stepper_col = h
            if h.endswith("_velocity_mm_s"):
                vel_col = h
        has_encoder = "encoder_mm" in headers
        has_accel   = "accel_x_mm_s2" in headers

        time, stepper, encoder, velocity = [], [], [], []
        accel = {"x": [], "y": [], "z": []}
        for row in reader:
            time.append(float(row["time_s"]))
            stepper.append(float(row[stepper_col]))
            if has_encoder:
                encoder.append(float(row["encoder_mm"]))
            velocity.append(float(row[vel_col]))
            if has_accel:
                for ax in ("x", "y", "z"):
                    accel[ax].append(float(row["accel_%s_mm_s2" % ax]))

    return axis, has_encoder, has_accel, time, stepper, encoder, velocity, accel

def _row_layout(has_encoder, has_accel):
    # Returns (nrows, height_ratios, row_indices) where row_indices is a dict
    rows   = ["pos"]
    if has_encoder:
        rows.append("err")
    rows.append("vel")
    if has_accel:
        rows.append("acc")
    ratios = {
        "pos": 3, "err": 1, "vel": 1, "acc": 2,
    }
    return len(rows), [ratios[r] for r in rows], {r: i for i, r in enumerate(rows)}

def _fill_column(grid, col, dataset, row_idx, share_axes):
    axis, has_encoder, has_accel, time, stepper, encoder, velocity, accel = dataset
    AXIS = axis.upper()

    # --- position ---
    ax_pos = grid[row_idx["pos"]][col]
    ax_pos.set_title("%s Axis" % AXIS, fontsize=12)
    ax_pos.plot(time, stepper, label="Stepper (commanded)",
                color="royalblue", linewidth=0.8)
    if has_encoder:
        ax_pos.plot(time, encoder, label="Encoder (measured)",
                    color="tomato", linewidth=0.8, alpha=0.85)
    ax_pos.set_ylabel("Position (mm)")
    ax_pos.legend(loc="upper right", fontsize=8)
    ax_pos.grid(True, linewidth=0.4, alpha=0.6)
    ax_pos.tick_params(labelbottom=False)
    share_axes["pos"].append(ax_pos)

    # --- following error ---
    if "err" in row_idx:
        ax_err = grid[row_idx["err"]][col]
        if has_encoder:
            error = [s - e for s, e in zip(stepper, encoder)]
            ax_err.plot(time, error, color="darkorange", linewidth=0.7)
            ax_err.axhline(0, color="black", linewidth=0.5, linestyle="--")
            ax_err.set_ylabel("Error\n(step−enc, mm)")
            ax_err.grid(True, linewidth=0.4, alpha=0.6)
        else:
            ax_err.set_visible(False)
        ax_err.tick_params(labelbottom=False)

    # --- velocity ---
    ax_vel = grid[row_idx["vel"]][col]
    ax_vel.plot(time, velocity, color="seagreen", linewidth=0.7)
    ax_vel.set_ylabel("Velocity (mm/s)")
    ax_vel.grid(True, linewidth=0.4, alpha=0.6)
    if "acc" not in row_idx:
        ax_vel.set_xlabel("Time (s)")
    else:
        ax_vel.tick_params(labelbottom=False)
    share_axes["vel"].append(ax_vel)

    # --- accelerometer ---
    if "acc" in row_idx:
        ax_acc = grid[row_idx["acc"]][col]
        if has_accel:
            for ax in ("x", "y", "z"):
                ax_acc.plot(time, accel[ax],
                            label="acc_%s" % ax,
                            color=ACCEL_COLORS[ax],
                            linewidth=0.6, alpha=0.85)
            ax_acc.axhline(0, color="black", linewidth=0.4, linestyle="--")
            ax_acc.set_ylabel("Accel (mm/s²)")
            ax_acc.legend(loc="upper right", fontsize=7)
            ax_acc.grid(True, linewidth=0.4, alpha=0.6)
        else:
            ax_acc.set_visible(False)
        ax_acc.set_xlabel("Time (s)")
        share_axes["acc"].append(ax_acc)

def plot_single(dataset, output):
    axis, has_encoder, has_accel, time, stepper, encoder, velocity, accel = dataset
    AXIS = axis.upper()

    nrows, ratios, row_idx = _row_layout(has_encoder, has_accel)
    fig = plt.figure(figsize=(14, 3 * nrows))
    title = "%s Axis" % AXIS
    if has_encoder and has_accel:
        title += " — Stepper / Encoder / Accelerometer"
    elif has_encoder:
        title += " — Stepper vs Encoder"
    elif has_accel:
        title += " — Stepper / Accelerometer"
    else:
        title += " — Stepper position"
    fig.suptitle(title, fontsize=13)

    gs   = gridspec.GridSpec(nrows, 1, height_ratios=ratios, hspace=0.08)
    grid = [[fig.add_subplot(gs[r])] for r in range(nrows)]
    share = {"pos": [], "vel": [], "acc": []}
    _fill_column(grid, 0, dataset, row_idx, share)

    plt.tight_layout()
    _save_or_show(fig, output)

def plot_multi(datasets, output):
    ncols = len(datasets)
    has_any_encoder = any(d[1] for d in datasets)
    has_any_accel   = any(d[2] for d in datasets)
    nrows, ratios, row_idx = _row_layout(has_any_encoder, has_any_accel)

    fig, grid = plt.subplots(
        nrows, ncols,
        figsize=(8 * ncols, 3 * nrows),
        gridspec_kw={"height_ratios": ratios, "hspace": 0.08},
        squeeze=False,
    )
    axes_labels = [d[0].upper() for d in datasets]
    fig.suptitle("%s Axis comparison" % " / ".join(axes_labels), fontsize=13)

    share = {"pos": [], "vel": [], "acc": []}
    for col, dataset in enumerate(datasets):
        _fill_column(grid, col, dataset, row_idx, share)

    plt.tight_layout()
    _save_or_show(fig, output)

def _save_or_show(fig, output):
    if output:
        fig.savefig(output, dpi=150, bbox_inches="tight")
        print("Saved →", output)
    else:
        plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: plot_axis.py <csv_file> [csv_file2 ...] [output.png]")
        sys.exit(1)

    args = sys.argv[1:]
    if args[-1].lower().endswith((".png", ".pdf", ".svg")):
        output    = args[-1]
        csv_paths = args[:-1]
    else:
        output    = None
        csv_paths = args

    if not csv_paths:
        print("ERROR: no CSV files provided")
        sys.exit(1)

    datasets = [load_csv(p) for p in csv_paths]

    if len(datasets) == 1:
        plot_single(datasets[0], output)
    else:
        plot_multi(datasets, output)

if __name__ == "__main__":
    main()