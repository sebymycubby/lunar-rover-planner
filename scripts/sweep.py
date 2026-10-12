#!/usr/bin/env python3
"""
Parameter sweep: driving drain rate vs. peak solar gain.

For every (drain, gain) pair, both planners run on the SAME set of maps
(common random numbers), so the difference between cells and between
planners comes from the parameters and the planner, not from terrain luck.

    python scripts/sweep.py [--seeds 30] [--steps 7] [--out results/sweep.png]
"""

import argparse
import json
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

SURFACE, TEXT, TEXT_2, GRID = "#1a1a19", "#f0efec", "#c3c2b7", "#383835"
# Sequential single-hue ramp (dark -> light = low -> high success)
CMAP = LinearSegmentedColormap.from_list(
    "success", ["#0d366b", "#184f95", "#256abf", "#3987e5", "#6da7ec", "#9ec5f4", "#cde2fb"])


def run(exe, seed, drain, gain, blind):
    cmd = [exe, "--seed", str(seed), "--drain", f"{drain:.6f}", "--gain", f"{gain:.6f}",
           "--summary", "--no-output", "--quiet"] + (["--blind"] if blind else [])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    return json.loads(r.stdout.strip().splitlines()[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default="build/lunar_rover")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--steps", type=int, default=7)
    ap.add_argument("--drain-min", type=float, default=0.004)
    ap.add_argument("--drain-max", type=float, default=0.012)
    ap.add_argument("--gain-min", type=float, default=0.002)
    ap.add_argument("--gain-max", type=float, default=0.010)
    ap.add_argument("--out", default="results/sweep.png")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)

    drains = np.linspace(a.drain_min, a.drain_max, a.steps)
    gains = np.linspace(a.gain_min, a.gain_max, a.steps)
    seeds = range(1, a.seeds + 1)  # identical maps in every cell

    jobs = [(gi, di, blind, s) for gi in range(len(gains)) for di in range(len(drains))
            for blind in (True, False) for s in seeds]
    with ThreadPoolExecutor(a.jobs) as pool:
        res = list(pool.map(lambda j: (j, run(a.exe, j[3], drains[j[1]], gains[j[0]], j[2])), jobs))

    rate = {True: np.zeros((len(gains), len(drains))), False: np.zeros((len(gains), len(drains)))}
    for (gi, di, blind, _), r in res:
        rate[blind][gi, di] += r["success"] / a.seeds

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.4), facecolor=SURFACE)
    fig.suptitle(f"Share of missions that reach the goal ({a.seeds} maps per cell, "
                 f"same maps in every cell, fog of war)",
                 color=TEXT, fontsize=12.5, x=0.01, ha="left")
    for ax, blind, title in [(axes[0], True, "Time-only planner"),
                             (axes[1], False, "Battery-aware planner")]:
        m = rate[blind]
        im = ax.imshow(m, origin="lower", cmap=CMAP, vmin=0, vmax=1, aspect="auto")
        for gi in range(len(gains)):
            for di in range(len(drains)):
                v = m[gi, di]
                ax.text(di, gi, f"{100 * v:.0f}", ha="center", va="center", fontsize=8.5,
                        color="#0b0b0b" if v > 0.6 else TEXT)
        ax.set_xticks(range(len(drains)), [f"{d * 1000:.1f}" for d in drains])
        ax.set_yticks(range(len(gains)), [f"{g * 1000:.1f}" for g in gains])
        ax.set_xlabel("drain while driving (‰ of capacity per s)", color=TEXT_2)
        ax.set_ylabel("solar gain in full sun (‰ per s)", color=TEXT_2)
        ax.set_title(title, color=TEXT_2, fontsize=10.5, loc="left")
        ax.tick_params(colors=TEXT_2, length=0)
        for sp in ax.spines.values():
            sp.set_visible(False)
    cb = fig.colorbar(im, ax=axes, shrink=0.85, pad=0.02)
    cb.set_label("% of missions reaching the goal", color=TEXT_2)
    cb.ax.yaxis.set_tick_params(color=TEXT_2, labelcolor=TEXT_2)
    cb.ax.set_yticks([0, 0.25, 0.5, 0.75, 1], ["0", "25", "50", "75", "100"])
    cb.outline.set_visible(False)
    fig.savefig(a.out, dpi=130, bbox_inches="tight", facecolor=SURFACE)
    print("wrote", a.out)
    os.makedirs("results", exist_ok=True)
    with open("results/sweep.csv", "w") as f:
        f.write("drain_per_sec,gain_per_sec,success_time_only,success_battery_aware\n")
        for gi, g in enumerate(gains):
            for di, d in enumerate(drains):
                f.write(f"{d:.6f},{g:.6f},{rate[True][gi, di]:.4f},{rate[False][gi, di]:.4f}\n")
    print("wrote results/sweep.csv")


if __name__ == "__main__":
    main()
