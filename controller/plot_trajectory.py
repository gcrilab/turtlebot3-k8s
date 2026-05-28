#!/usr/bin/env python3
"""Plot reference vs actual trajectory and tracking error from controller logs."""

import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def load_log(path=None):
    """Load the most recent log CSV, or a specific one if path is given."""
    if path is None:
        files = sorted(glob.glob("enhanced_log_figure8_*.csv"))
        if not files:
            print("No log files found. Run the controller first.")
            sys.exit(1)
        path = files[-1]
        print(f"Loading most recent log: {path}")
    df = pd.read_csv(path)
    return df, path


def plot(df, filename):
    t = df["Time"].values
    x_act = df["X"].values
    y_act = df["Y"].values
    ex = df["error_x"].values
    ey = df["error_y"].values
    err_norm = df["tracking_error"].values

    # Reconstruct reference: error = actual - reference  =>  reference = actual - error
    x_ref = x_act - ex
    y_ref = y_act - ey

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"Controller Performance — {filename}", fontsize=14, fontweight="bold")
    gs = GridSpec(2, 3, figure=fig, hspace=0.35, wspace=0.35)

    # --- 1. XY trajectory ---
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(x_ref, y_ref, "b-", linewidth=1.5, label="Reference")
    ax1.plot(x_act, y_act, "r--", linewidth=1.2, alpha=0.8, label="Actual")
    ax1.plot(x_act[0], y_act[0], "go", markersize=8, label="Start")
    ax1.plot(x_act[-1], y_act[-1], "ks", markersize=8, label="End")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("XY Trajectory")
    ax1.legend(fontsize=8)
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3)

    # --- 2. Tracking error norm over time ---
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, err_norm * 100, "m-", linewidth=0.8)
    ax2.axhline(np.mean(err_norm) * 100, color="k", linestyle="--", linewidth=0.8,
                label=f"Mean = {np.mean(err_norm)*100:.2f} cm")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error (cm)")
    ax2.set_title("Tracking Error ‖e‖")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # --- 3. X and Y error components over time ---
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(t, ex * 100, "r-", linewidth=0.8, label="eₓ")
    ax3.plot(t, ey * 100, "b-", linewidth=0.8, label="eᵧ")
    ax3.axhline(0, color="k", linewidth=0.5)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Error (cm)")
    ax3.set_title("Error Components")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # --- 4. X ref vs actual over time ---
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(t, x_ref, "b-", linewidth=1.2, label="X ref")
    ax4.plot(t, x_act, "r--", linewidth=1.0, alpha=0.8, label="X actual")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("X (m)")
    ax4.set_title("X Position vs Time")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # --- 5. Y ref vs actual over time ---
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(t, y_ref, "b-", linewidth=1.2, label="Y ref")
    ax5.plot(t, y_act, "r--", linewidth=1.0, alpha=0.8, label="Y actual")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Y (m)")
    ax5.set_title("Y Position vs Time")
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    # --- 6. Summary statistics ---
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis("off")
    stats = (
        f"Duration:      {t[-1]:.1f} s\n"
        f"Samples:       {len(t)}\n"
        f"Rate:          {len(t)/t[-1]:.0f} Hz\n"
        f"\n"
        f"Tracking Error (cm)\n"
        f"  Mean:        {np.mean(err_norm)*100:.2f}\n"
        f"  Std:         {np.std(err_norm)*100:.2f}\n"
        f"  Max:         {np.max(err_norm)*100:.2f}\n"
        f"  RMSE:        {np.sqrt(np.mean(err_norm**2))*100:.2f}\n"
        f"\n"
        f"Steady-state (last 25%)\n"
        f"  Mean:        {np.mean(err_norm[len(err_norm)//4*3:])*100:.2f} cm\n"
        f"  Max:         {np.max(err_norm[len(err_norm)//4*3:])*100:.2f} cm\n"
        f"\n"
        f"Commanded velocity\n"
        f"  v mean:      {np.mean(df['vCmd']):.3f} m/s\n"
        f"  ω mean:      {np.mean(np.abs(df['wCmd'])):.3f} rad/s"
    )
    ax6.text(0.05, 0.95, stats, transform=ax6.transAxes, fontsize=9,
             verticalalignment="top", fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow", alpha=0.8))
    ax6.set_title("Summary Statistics")

    plt.savefig(filename.replace(".csv", "_performance.png"), dpi=150, bbox_inches="tight")
    print(f"Saved plot to {filename.replace('.csv', '_performance.png')}")
    plt.show()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else None
    df, fname = load_log(path)
    plot(df, fname)
