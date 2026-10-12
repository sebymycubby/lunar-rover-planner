#!/usr/bin/env python3
"""
Parameter sweep: drain_per_sec vs max_gain_per_sec
Runs the C++ planner across a grid of (drain, gain) values on randomly seeded
maps, then plots a heatmap of outcomes.

Usage:
    python sweep.py [--runs-per-cell N] [--drain-steps M] [--gain-steps M]
                    [--drain-min D] [--drain-max D]
                    [--gain-min G]  [--gain-max G]
                    [--seeds-file FILE]   # optional: one seed per line
"""

import subprocess, os, json, sys, argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# ═══════════════════════════════════════════════════════════════════
#  SWEEP CONFIGURATION — edit these
# ═══════════════════════════════════════════════════════════════════

PLANNER_EXE   = "./build/lunar_rover"   # path to compiled binary
MAP_WIDTH     = 60
MAP_HEIGHT    = 60
RUNS_PER_CELL = 5          # how many random seeds per (drain, gain) pair
DRAIN_STEPS   = 10         # number of drain values to test
GAIN_STEPS    = 10         # number of gain values to test
DRAIN_MIN     = 0.002
DRAIN_MAX     = 0.012
GAIN_MIN      = 0.002
GAIN_MAX      = 0.014

# ═══════════════════════════════════════════════════════════════════

def run_planner(seed, drain, gain, tmp_path="output/_sweep_tmp.json"):
    cmd = [
        PLANNER_EXE,
        "--width",  str(MAP_WIDTH),
        "--height", str(MAP_HEIGHT),
        "--seed",   str(seed),
        "--drain",  f"{drain:.6f}",
        "--gain",   f"{gain:.6f}",
        "--output", tmp_path,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if r.returncode == 0:
            with open(tmp_path) as f:
                data = json.load(f)
            res = data['result']
            return {
                'success':       res['success'],
                'total_time':    res['total_time'],
                'final_battery': res['final_battery'],
            }
        else:
            return {'success': False, 'total_time': 0, 'final_battery': 0}
    except Exception as e:
        print(f"  [warn] planner error: {e}")
        return {'success': False, 'total_time': 0, 'final_battery': 0}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs-per-cell', type=int,   default=RUNS_PER_CELL)
    parser.add_argument('--drain-steps',   type=int,   default=DRAIN_STEPS)
    parser.add_argument('--gain-steps',    type=int,   default=GAIN_STEPS)
    parser.add_argument('--drain-min',     type=float, default=DRAIN_MIN)
    parser.add_argument('--drain-max',     type=float, default=DRAIN_MAX)
    parser.add_argument('--gain-min',      type=float, default=GAIN_MIN)
    parser.add_argument('--gain-max',      type=float, default=GAIN_MAX)
    args = parser.parse_args()

    os.makedirs("output", exist_ok=True)

    drains = np.linspace(args.drain_min, args.drain_max, args.drain_steps)
    gains  = np.linspace(args.gain_min,  args.gain_max,  args.gain_steps)

    # Result matrices (gain on Y axis, drain on X axis)
    success_rate  = np.zeros((args.gain_steps, args.drain_steps))
    avg_battery   = np.zeros((args.gain_steps, args.drain_steps))
    avg_time      = np.zeros((args.gain_steps, args.drain_steps))

    total_runs = args.drain_steps * args.gain_steps * args.runs_per_cell
    run_num    = 0

    # Use consecutive seeds so each cell sees different terrain
    base_seed = 1000

    for gi, gain in enumerate(gains):
        for di, drain in enumerate(drains):
            successes = []
            batteries = []
            times     = []
            for ri in range(args.runs_per_cell):
                seed = base_seed + gi * args.drain_steps * args.runs_per_cell \
                                 + di * args.runs_per_cell + ri
                run_num += 1
                print(f"[{run_num:4d}/{total_runs}] "
                      f"drain={drain:.4f} gain={gain:.4f} seed={seed}", end='  ')
                res = run_planner(seed, drain, gain)
                s = res['success']
                successes.append(1 if s else 0)
                batteries.append(res['final_battery'] if s else 0)
                times.append(res['total_time']        if s else 0)
                print("OK" if s else "FAIL")

            success_rate[gi, di] = np.mean(successes)
            avg_battery[gi, di]  = np.mean(batteries)
            avg_time[gi, di]     = np.mean(times)

    # ─── Plot ───────────────────────────────────────────────────────

    fig, axes = plt.subplots(1, 3, figsize=(20, 6), facecolor='#080810')
    fig.suptitle('Parameter Sweep: Drain Rate vs Gain Rate',
                 fontsize=14, color='#e0d8c8', fontweight='bold',
                 fontfamily='monospace', y=1.01)

    drain_labels = [f"{d:.4f}" for d in drains]
    gain_labels  = [f"{g:.4f}" for g in gains]

    def styled_heatmap(ax, data, title, cmap, vmin=None, vmax=None, fmt='.2f'):
        ax.set_facecolor('#080810')
        im = ax.imshow(data, cmap=cmap, aspect='auto',
                       origin='lower', vmin=vmin, vmax=vmax,
                       extent=[-0.5, args.drain_steps-0.5,
                                -0.5, args.gain_steps-0.5])
        # Annotate cells
        for gi2 in range(args.gain_steps):
            for di2 in range(args.drain_steps):
                val = data[gi2, di2]
                ax.text(di2, gi2, f'{val:{fmt}}',
                        ha='center', va='center', fontsize=6,
                        color='white' if val < (vmax or 1)*0.6 else '#111',
                        fontfamily='monospace')
        ax.set_xticks(range(args.drain_steps))
        ax.set_xticklabels(drain_labels, rotation=45, ha='right',
                           fontsize=7, color='#aaa', fontfamily='monospace')
        ax.set_yticks(range(args.gain_steps))
        ax.set_yticklabels(gain_labels, fontsize=7, color='#aaa',
                           fontfamily='monospace')
        ax.set_xlabel('Drain rate (per sec)', color='#aaa', fontsize=9,
                      fontfamily='monospace')
        ax.set_ylabel('Gain rate (per sec)', color='#aaa', fontsize=9,
                      fontfamily='monospace')
        ax.set_title(title, color='#ccccee', fontsize=10,
                     fontfamily='monospace', pad=8)
        for sp in ax.spines.values(): sp.set_color('#2a2a3a')
        cb = plt.colorbar(im, ax=ax, shrink=0.75, pad=0.02)
        cb.ax.tick_params(colors='#888', labelsize=7)
        return im

    # 1. Success rate
    success_cmap = LinearSegmentedColormap.from_list(
        'sr', ['#330000','#883300','#aaaa00','#44ff44'], N=256)
    styled_heatmap(axes[0], success_rate,
                   f'Success Rate\n({args.runs_per_cell} runs per cell)',
                   success_cmap, vmin=0, vmax=1, fmt='.2f')

    # Draw the approximate success boundary
    # Find gain/drain ratio = 1 line
    ratio_drains = np.linspace(0, args.drain_steps-1, 200)
    ratio_gains  = ratio_drains  # ratio=1 line (gain=drain)
    axes[0].plot(ratio_drains, ratio_gains, 'w--', lw=1.0, alpha=0.5,
                 label='gain=drain')

    # 2. Avg final battery (successful runs only)
    battery_cmap = LinearSegmentedColormap.from_list(
        'bat', ['#1a0000','#882200','#ddaa00','#00ff88'], N=256)
    styled_heatmap(axes[1], avg_battery*100,
                   f'Avg Final Battery %\n(successful runs)',
                   battery_cmap, vmin=0, vmax=100, fmt='.0f')

    # 3. Gain/drain ratio overlaid with success contour
    ratio_matrix = np.where(drains[np.newaxis,:] > 0,
                            gains[:,np.newaxis] / drains[np.newaxis,:], 0)
    ratio_cmap = plt.cm.coolwarm
    styled_heatmap(axes[2], ratio_matrix,
                   'Gain / Drain Ratio\n(>1 = net positive when fully lit)',
                   ratio_cmap, vmin=0, vmax=3, fmt='.2f')
    # Overlay success rate as contour
    axes[2].contour(success_rate, levels=[0.5, 0.8], colors=['yellow','lime'],
                    linewidths=[1.0, 1.5], origin='lower',
                    extent=[-0.5, args.drain_steps-0.5,
                            -0.5, args.gain_steps-0.5])

    plt.tight_layout()
    out = "output/sweep_heatmap.png"
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='#080810')
    plt.close(fig)
    print(f"\nHeatmap saved → {out}")

    # Save raw results as JSON for future use
    sweep_data = {
        'drains': drains.tolist(),
        'gains':  gains.tolist(),
        'success_rate':  success_rate.tolist(),
        'avg_battery':   avg_battery.tolist(),
        'avg_time':      avg_time.tolist(),
        'runs_per_cell': args.runs_per_cell,
    }
    with open("output/sweep_results.json", 'w') as f:
        json.dump(sweep_data, f, indent=2)
    print("Raw results → output/sweep_results.json")

if __name__ == '__main__':
    main()
