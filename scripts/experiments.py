#!/usr/bin/env python3
"""
Benchmark the planners on many random maps.

Every configuration runs on the SAME set of seeds, so differences come from
the planner, not from the terrain.

    python scripts/experiments.py [--seeds 200] [--exe build/lunar_rover]

Writes results/experiments.csv and results/experiments.md.
"""

import argparse
import csv
import json
import os
import statistics as st
import subprocess
from concurrent.futures import ThreadPoolExecutor

CONFIGS = [
    # name,                        extra CLI flags
    ("Time-only (baseline)",         ["--blind"]),
    ("Battery-aware, no parking",    ["--no-wait"]),
    ("Battery-aware",                []),
    ("Battery-aware, full map",      ["--full-map"]),
]


def run(exe, seed, flags):
    r = subprocess.run([exe, "--seed", str(seed), "--summary", "--no-output", "--quiet", *flags],
                       capture_output=True, text=True, timeout=300)
    return json.loads(r.stdout.strip().splitlines()[-1])


def median_or_dash(xs, fmt="{:.0f}"):
    return fmt.format(st.median(xs)) if xs else "–"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=200)
    ap.add_argument("--exe", default="build/lunar_rover")
    ap.add_argument("--outdir", default="results")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    seeds = list(range(1, args.seeds + 1))

    jobs = [(name, flags, s) for name, flags in CONFIGS for s in seeds]
    with ThreadPoolExecutor(args.jobs) as pool:
        out = list(pool.map(lambda j: (j[0], run(args.exe, j[2], j[1])), jobs))

    rows = []
    for name, r in out:
        r["config"] = name
        rows.append(r)
    with open(os.path.join(args.outdir, "experiments.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    by = {name: {r["seed"]: r for r in rows if r["config"] == name} for name, _ in CONFIGS}
    n = len(seeds)
    lines = [f"Results on {n} random 60×60 maps (seeds 1–{n}), default parameters.",
             "", "| Planner | Reached goal | Median time (s) | Mean recharge stops | Mean planning per replan (ms) |",
             "|---|---|---|---|---|"]
    for name, _ in CONFIGS:
        rs = list(by[name].values())
        ok = [r for r in rs if r["success"]]
        per_replan = [r["planning_ms"] / max(r["steps"], 1) for r in rs
                      if "full map" not in name]
        plan_ms = (f"{st.mean(per_replan):.2f}" if per_replan
                   else f"{st.mean(r['planning_ms'] for r in rs):.1f} (one plan)")
        lines.append(f"| {name} | {len(ok)}/{n} ({100 * len(ok) / n:.0f}%) | "
                     f"{median_or_dash([r['total_time'] for r in ok])} | "
                     f"{st.mean(r['waits'] for r in ok) if ok else 0:.1f} | {plan_ms} |")

    base, aware = by[CONFIGS[0][0]], by["Battery-aware"]
    nowait, full = by["Battery-aware, no parking"], by["Battery-aware, full map"]
    died = [s for s in seeds if not base[s]["success"]]
    rescued = [s for s in died if aware[s]["success"]]
    rescued_detour = [s for s in died if nowait[s]["success"]]
    both = [s for s in seeds if base[s]["success"] and aware[s]["success"]]
    same_time = sum(abs(aware[s]["total_time"] - base[s]["total_time"]) < 1e-6 for s in both)
    fog_cost = [aware[s]["total_time"] / full[s]["total_time"] for s in seeds
                if aware[s]["success"] and full[s]["success"]]

    lines += ["",
              f"- The time-only planner lost the rover on **{len(died)}/{n}** maps. "
              f"The battery-aware planner reached the goal on **{len(rescued)}** of those "
              f"({len(rescued_detour)} by route choice alone, the rest also needed recharge stops).",
              f"- Where the baseline survived ({len(both)} maps), the battery-aware planner was "
              f"exactly as fast on {same_time} of them: it only gives up time when it has to.",
              f"- With the whole map known, the battery-aware rover needed "
              f"{st.mean(r['waits'] for r in full.values()):.1f} recharge stops per mission on "
              f"average, versus {st.mean(r['waits'] for r in aware.values()):.1f} under fog of war: "
              f"seeing the terrain early lets it choose a sunlit route instead of parking.",
              f"- Exploring under fog of war took a median {100 * (st.median(fog_cost) - 1):.0f}% "
              f"longer than planning with the full map known."]
    md = "\n".join(l for l in lines if l is not None) + "\n"
    with open(os.path.join(args.outdir, "experiments.md"), "w") as f:
        f.write(md)
    print(md)


if __name__ == "__main__":
    main()
