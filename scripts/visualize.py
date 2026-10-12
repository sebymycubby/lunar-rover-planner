#!/usr/bin/env python3
"""
Visualise lunar_rover output.

    python scripts/visualize.py overview output/plan.json  [--out FILE]
    python scripts/visualize.py compare  blind.json aware.json [--out FILE]
    python scripts/visualize.py animate  output/plan.json  [--out FILE] [--fps N]
    python scripts/visualize.py animate-compare blind.json aware.json [--out FILE]

Lighting comes from the light maps the C++ planner writes into the JSON, so
the pictures always use exactly the planner's shadow model.
"""

import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D

# ─── Style ─────────────────────────────────────────────────────────────────
SURFACE   = "#1a1a19"
TEXT      = "#f0efec"
TEXT_2    = "#c3c2b7"
GRID      = "#383835"
AWARE     = "#3987e5"   # battery-aware planner (categorical slot 1)
BLIND     = "#d95926"   # time-only baseline    (categorical slot 2)
CRITICAL  = "#e66767"   # minimum-charge line / failure marker
SUN       = "#c98500"

TERRAIN_BASE = {           # 0 flat, 1 rocky, 2 dune, 3 crater rim, 4 crater floor
    0: np.array([0.60, 0.58, 0.55]),
    1: np.array([0.50, 0.47, 0.42]),
    2: np.array([0.66, 0.60, 0.47]),
    3: np.array([0.40, 0.38, 0.36]),
    4: np.array([0.26, 0.24, 0.23]),
}
SHADOW = np.array([0.05, 0.06, 0.13])
FOG    = np.array([0.07, 0.07, 0.07])
COST_CMAP = LinearSegmentedColormap.from_list("cost", ["#0d366b", "#3987e5", "#cde2fb"])

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "text.color": TEXT,
    "axes.labelcolor": TEXT_2,
    "axes.edgecolor": GRID,
    "xtick.color": TEXT_2,
    "ytick.color": TEXT_2,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
})


# ─── Data helpers ──────────────────────────────────────────────────────────
def load(path):
    with open(path) as f:
        d = json.load(f)
    d["terrain_type"] = np.array(d["terrain_type"])
    d["cost"] = np.array(d["cost"])
    d["light"] = np.array(d["light"])
    d["reveal_step"] = np.array(d["reveal_step"])
    return d


def light_at(d, t):
    k = int(round(t / d["light_dt"]))
    return d["light"][min(max(k, 0), len(d["light"]) - 1)]


def terrain_rgb(d, t, known_mask=None):
    base = np.zeros(d["terrain_type"].shape + (3,))
    for k, col in TERRAIN_BASE.items():
        base[d["terrain_type"] == k] = col
    light = light_at(d, t)[..., None]
    rgb = base * light + SHADOW * (1 - light)
    if known_mask is not None:
        rgb = np.where(known_mask[..., None], rgb, rgb * 0.25 + FOG * 0.75)
    return rgb


def xy(steps):
    return [s["x"] for s in steps], [s["y"] for s in steps]


def style_map_axis(ax):
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color(GRID)


def style_chart_axis(ax):
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for name, sp in ax.spines.items():
        sp.set_visible(name in ("left", "bottom"))
        sp.set_color(GRID)


def mark_start_goal(ax, d):
    sx, sy = d["config"]["start"]; gx, gy = d["config"]["goal"]
    ax.plot(sx, sy, "o", ms=9, color=TEXT, mec=SURFACE, mew=2, zorder=10)
    ax.plot(gx, gy, "*", ms=15, color=TEXT, mec=SURFACE, mew=1.5, zorder=10)
    ax.annotate("start", (sx, sy), xytext=(0, -14), textcoords="offset points",
                ha="center", color=TEXT, fontsize=9, zorder=11)
    ax.annotate("goal", (gx, gy), xytext=(0, -16), textcoords="offset points",
                ha="center", color=TEXT, fontsize=9, zorder=11)


def draw_route(ax, d, color, ls="-", label=None):
    ex = d["executed"]
    px, py = xy(ex)
    ax.plot(px, py, color=SURFACE, lw=4.5, ls="-", zorder=5, solid_capstyle="round")
    ax.plot(px, py, color=color, lw=2.2, ls=ls, zorder=6, label=label,
            solid_capstyle="round")
    waits = [s for s in ex if s["wait"]]
    if waits:
        wx, wy = xy(waits)
        ax.scatter(wx, wy, s=90, facecolor="none", edgecolor=SUN, lw=2, zorder=7)
    if not d["result"]["success"]:
        ax.plot(px[-1], py[-1], "X", ms=13, color=CRITICAL, mec=SURFACE, mew=1.5, zorder=9)


def battery_chart(ax, runs):
    """runs: list of (data, color, linestyle, label)."""
    cfg = runs[0][0]["config"]
    tmax = max(r[0]["executed"][-1]["t"] for r in runs)
    for d, color, ls, label in runs:
        ex = d["executed"]
        t = [s["t"] for s in ex]
        b = [100 * s["bat"] for s in ex]
        ax.plot(t, b, color=color, lw=2, ls=ls, zorder=3 if ls != "-" else 2)
        for s0, s1 in zip(ex, ex[1:]):          # shade time spent parked
            if s1["wait"]:
                ax.axvspan(s0["t"], s1["t"], color=SUN, alpha=0.15, lw=0)
        end = "  (lost)" if not d["result"]["success"] else ""
        ax.annotate(label + end, (t[-1], b[-1]),
                    xytext=(8, 10) if end else (6, 0), textcoords="offset points",
                    va="center", color=TEXT, fontsize=9)
        if not d["result"]["success"]:
            ax.plot(t[-1], b[-1], "X", ms=10, color=CRITICAL, mec=SURFACE, mew=1.2)
    ax.axhline(100 * cfg["battery_min"], color=CRITICAL, lw=1.2, ls="--")
    ax.annotate(f"minimum charge {100 * cfg['battery_min']:.0f}%",
                (0, 100 * cfg["battery_min"]), xytext=(2, -12),
                textcoords="offset points", color=TEXT_2, fontsize=8.5)
    lo = min(min(100 * s["bat"] for s in r[0]["executed"]) for r in runs)
    hi = max(max(100 * s["bat"] for s in r[0]["executed"]) for r in runs)
    lo = min(lo, 100 * cfg["battery_min"])
    pad = 0.12 * (hi - lo)
    ax.set_ylim(lo - pad, hi + pad * 0.5)
    ax.set_xlim(0, tmax * 1.32)
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("battery charge (%)")
    style_chart_axis(ax)


# ─── Figures ───────────────────────────────────────────────────────────────
def render_overview(d, out):
    ex, res, cfg = d["executed"], d["result"], d["config"]
    t_end = ex[-1]["t"]
    final_step = len(ex) - 1
    known = (d["reveal_step"] >= 0) & (d["reveal_step"] <= final_step)

    fig = plt.figure(figsize=(17, 6.4))
    sf_map, sf_cost, sf_chart = fig.subfigures(1, 3, width_ratios=[1, 1.12, 1.15], wspace=0.02)
    status = "reached goal" if res["success"] else f"FAILED: {res['failure_reason']}"
    planner = "battery-aware" if cfg["battery_aware"] else "time-only"
    fig.suptitle(f"Seed {d['seed']} · {planner} planner · {status} · "
                 f"{res['total_time']:.0f} s, {res['waits']} recharge stops",
                 color=TEXT, fontsize=12.5, x=0.01, ha="left", y=1.02)

    ax = sf_map.subplots()
    ax.imshow(terrain_rgb(d, 0.0, None if cfg["full_map"] else known),
              interpolation="nearest")
    draw_route(ax, d, AWARE if cfg["battery_aware"] else BLIND)
    mark_start_goal(ax, d)
    ax.set_title("Route (shading = shadow; dark = never observed)"
                 if not cfg["full_map"] else "Route over terrain (shading = shadow at t=0)",
                 color=TEXT_2, fontsize=9.5, loc="left")
    style_map_axis(ax)

    ax = sf_cost.subplots()
    im = ax.imshow(d["cost"], cmap=COST_CMAP, interpolation="nearest")
    px, py = xy(ex)
    ax.plot(px, py, color=SURFACE, lw=4); ax.plot(px, py, color=TEXT, lw=1.6)
    mark_start_goal(ax, d)
    from mpl_toolkits.axes_grid1 import make_axes_locatable
    cax = make_axes_locatable(ax).append_axes("right", size="4%", pad=0.08)
    cb = sf_cost.colorbar(im, cax=cax)
    cb.set_label("seconds to cross cell", color=TEXT_2)
    cb.outline.set_edgecolor(GRID)
    ax.set_title("True traverse cost", color=TEXT_2, fontsize=9.5, loc="left")
    style_map_axis(ax)

    ax_b, ax_l = sf_chart.subplots(2, 1, gridspec_kw={"height_ratios": [1.4, 1], "hspace": 0.5})
    ax = ax_b
    battery_chart(ax, [(d, AWARE if cfg["battery_aware"] else BLIND, "-", "")])
    ax.set_title("Battery", color=TEXT_2, fontsize=9.5, loc="left")

    ax = ax_l
    t = [s["t"] for s in ex]; l = [s["light"] for s in ex]
    ax.fill_between(t, l, color=SUN, alpha=0.25, lw=0, step="pre")
    ax.plot(t, l, color=SUN, lw=1.6, drawstyle="steps-pre")
    ax.set_ylim(0, 1.05); ax.set_xlim(0, t_end * 1.32)
    ax.set_xlabel("mission time (s)"); ax.set_ylabel("light (0–1)")
    ax.set_title("Sunlight on the rover", color=TEXT_2, fontsize=9.5, loc="left")
    style_chart_axis(ax)

    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)


def render_compare(blind, aware, out):
    fig = plt.figure(figsize=(14, 6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.15], wspace=0.12)
    rb, ra = blind["result"], aware["result"]
    fig.suptitle(f"Seed {aware['seed']} · same map, same {100 * aware['config']['battery_start']:.0f}% "
                 f"starting charge: time-only planner vs battery-aware planner",
                 color=TEXT, fontsize=12.5, x=0.02, ha="left", y=0.995)

    ax = fig.add_subplot(gs[0])
    ax.imshow(terrain_rgb(aware, 0.0), interpolation="nearest")
    draw_route(ax, blind, BLIND, ls=(0, (3, 1.6)))
    draw_route(ax, aware, AWARE)
    mark_start_goal(ax, aware)
    handles = [Line2D([], [], color=BLIND, lw=2.2, ls=(0, (3, 1.6)),
                      label=f"time-only: lost at t={rb['total_time']:.0f} s"),
               Line2D([], [], color=AWARE, lw=2.2,
                      label=f"battery-aware: goal at t={ra['total_time']:.0f} s")]
    if ra["waits"]:
        handles.append(Line2D([], [], marker="o", ls="", mfc="none", mec=SUN, mew=2, ms=9,
                              label="parked to recharge"))
    ax.legend(handles=handles, loc="lower left", fontsize=9, facecolor=SURFACE,
              edgecolor=GRID, labelcolor=TEXT, framealpha=0.92)
    ax.set_title("Terrain and shadow (darker = less sunlight)", color=TEXT_2,
                 fontsize=9.5, loc="left")
    style_map_axis(ax)

    ax = fig.add_subplot(gs[1])
    battery_chart(ax, [(blind, BLIND, (0, (3, 1.6)), "time-only"),
                       (aware, AWARE, "-", "battery-aware")])
    ax.set_title("Battery charge during the traverse", color=TEXT_2, fontsize=9.5, loc="left")

    fig.subplots_adjust(left=0.02, right=0.98)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)


def render_animation(d, out, fps=8):
    ex, cfg = d["executed"], d["config"]
    plans = d["plans"]
    n = len(ex)
    color = AWARE if cfg["battery_aware"] else BLIND

    fig = plt.figure(figsize=(10.5, 5.2))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.05], wspace=0.15)
    ax_map = fig.add_subplot(gs[0]); ax_bat = fig.add_subplot(gs[1])
    style_map_axis(ax_map)

    img = ax_map.imshow(terrain_rgb(d, 0, d["reveal_step"] == 0), interpolation="nearest")
    plan_line, = ax_map.plot([], [], color=TEXT, lw=1.3, ls=(0, (1, 1.5)), zorder=4)
    trail_bg, = ax_map.plot([], [], color=SURFACE, lw=4.5, zorder=5)
    trail, = ax_map.plot([], [], color=color, lw=2.2, zorder=6)
    rover, = ax_map.plot([], [], "o", ms=10, color=color, mec=TEXT, mew=1.6, zorder=8)
    waits_sc = ax_map.scatter([], [], s=90, facecolor="none", edgecolor=SUN, lw=2, zorder=7)
    mark_start_goal(ax_map, d)
    ax_map.legend(handles=[Line2D([], [], color=color, lw=2.2, label="driven"),
                           Line2D([], [], color=TEXT, lw=1.3, ls=(0, (1, 1.5)),
                                  label="current plan")],
                  loc="lower left", fontsize=8.5, facecolor=SURFACE, edgecolor=GRID,
                  labelcolor=TEXT, framealpha=0.9)
    title = ax_map.set_title("", color=TEXT, fontsize=10, loc="left")

    t_all = [s["t"] for s in ex]; b_all = [100 * s["bat"] for s in ex]
    ax_bat.set_xlim(0, t_all[-1] * 1.05)
    ax_bat.set_ylim(0, 100)
    ax_bat.axhline(100 * cfg["battery_min"], color=CRITICAL, lw=1.2, ls="--")
    ax_bat.annotate(f"minimum {100 * cfg['battery_min']:.0f}%", (0, 100 * cfg["battery_min"]),
                    xytext=(2, -12), textcoords="offset points", color=TEXT_2, fontsize=8.5)
    ax_bat.set_xlabel("mission time (s)"); ax_bat.set_ylabel("battery charge (%)")
    ax_bat.set_title("Battery", color=TEXT_2, fontsize=9.5, loc="left")
    style_chart_axis(ax_bat)
    bat_line, = ax_bat.plot([], [], color=color, lw=2)
    bat_dot, = ax_bat.plot([], [], "o", ms=7, color=color, mec=TEXT, mew=1.2)

    planner = "Battery-aware" if cfg["battery_aware"] else "Time-only"
    hold = fps * 2  # hold the last frame for two seconds

    def update(fi):
        k = min(fi, n - 1)
        s = ex[k]
        known = (d["reveal_step"] >= 0) & (d["reveal_step"] <= k)
        img.set_data(terrain_rgb(d, s["t"], None if cfg["full_map"] else known))
        if k < len(plans):
            plan_line.set_data([p[0] for p in plans[k]], [p[1] for p in plans[k]])
        else:
            plan_line.set_data([], [])
        px, py = xy(ex[:k + 1])
        trail.set_data(px, py); trail_bg.set_data(px, py)
        rover.set_data([s["x"]], [s["y"]])
        w = [e for e in ex[:k + 1] if e["wait"]]
        waits_sc.set_offsets(np.c_[[e["x"] for e in w], [e["y"] for e in w]] if w
                             else np.empty((0, 2)))
        bat_line.set_data(t_all[:k + 1], b_all[:k + 1])
        bat_dot.set_data([t_all[k]], [b_all[k]])
        state = "parked, recharging" if s["wait"] else "driving"
        if k == n - 1:
            state = "goal reached" if d["result"]["success"] else d["result"]["failure_reason"]
        title.set_text(f"{planner} · t = {s['t']:5.1f} s · {state}")
        return []

    anim = FuncAnimation(fig, update, frames=n + hold, blit=False)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=72)
    plt.close(fig)
    print("wrote", out)


def render_animation_compare(blind, aware, out, fps=10, dt=1.0):
    """Both rovers side by side on a shared clock."""
    runs = [(blind, BLIND, (0, (3, 1.6)), "Time-only"), (aware, AWARE, "-", "Battery-aware")]
    t_end = max(r[0]["executed"][-1]["t"] for r in runs)
    times = list(np.arange(0, t_end + dt, dt))
    times += [times[-1]] * (fps * 3)          # hold the final frame

    fig = plt.figure(figsize=(12.5, 4.7))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.15], wspace=0.06,
                          left=0.01, right=0.94, top=0.86, bottom=0.12)
    cfg = aware["config"]
    fig.suptitle(f"Seed {aware['seed']} · fog of war · both rovers start with "
                 f"{100 * cfg['battery_start']:.0f}% charge",
                 color=TEXT, fontsize=12, x=0.01, ha="left", y=0.995)

    panes = []
    for i, (d, color, ls, name) in enumerate(runs):
        ax = fig.add_subplot(gs[i]); style_map_axis(ax)
        img = ax.imshow(terrain_rgb(d, 0, d["reveal_step"] == 0), interpolation="nearest")
        plan, = ax.plot([], [], color=TEXT, lw=1.6, ls=(0, (1, 1.2)), zorder=4)
        bg, = ax.plot([], [], color=SURFACE, lw=4.5, zorder=5)
        trail, = ax.plot([], [], color=color, lw=2.4, zorder=6)
        rover, = ax.plot([], [], "o", ms=10, color=color, mec=TEXT, mew=1.6, zorder=8)
        dead, = ax.plot([], [], "X", ms=14, color=CRITICAL, mec=SURFACE, mew=1.5, zorder=9)
        waits = ax.scatter([], [], s=90, facecolor="none", edgecolor=SUN, lw=2, zorder=7)
        mark_start_goal(ax, d)
        title = ax.set_title("", color=TEXT, fontsize=10, loc="left")
        panes.append(dict(d=d, img=img, plan=plan, bg=bg, trail=trail, rover=rover,
                          dead=dead, waits=waits, title=title, name=name))
    panes[0]["ax_legend"] = fig.axes[0].legend(
        handles=[Line2D([], [], color=TEXT, lw=1.6, ls=(0, (1, 1.2)), label="current plan")],
        loc="lower left", fontsize=8.5, facecolor=SURFACE, edgecolor=GRID,
        labelcolor=TEXT, framealpha=0.9)

    axb = fig.add_subplot(gs[2])
    all_b = [100 * s["bat"] for r in runs for s in r[0]["executed"]]
    lo = min(min(all_b), 100 * cfg["battery_min"]); hi = max(all_b)
    axb.set_xlim(0, t_end * 1.05); axb.set_ylim(lo - 0.12 * (hi - lo), hi + 0.06 * (hi - lo))
    axb.axhline(100 * cfg["battery_min"], color=CRITICAL, lw=1.2, ls="--")
    axb.annotate(f"minimum {100 * cfg['battery_min']:.0f}%", (0, 100 * cfg["battery_min"]),
                 xytext=(2, -12), textcoords="offset points", color=TEXT_2, fontsize=8.5)
    axb.set_xlabel("mission time (s)"); axb.set_ylabel("battery charge (%)")
    axb.yaxis.set_label_position("right"); axb.yaxis.tick_right()
    axb.set_title("Battery", color=TEXT_2, fontsize=10, loc="left")
    style_chart_axis(axb)
    axb.spines["left"].set_visible(False); axb.spines["right"].set_visible(True)
    lines = []
    for d, color, ls, name in runs:
        ln, = axb.plot([], [], color=color, lw=2, ls=ls, label=name)
        dot, = axb.plot([], [], "o", ms=6, color=color, mec=TEXT, mew=1)
        lines.append((ln, dot))
    axb.legend(loc="upper right", fontsize=8.5, facecolor=SURFACE, edgecolor=GRID,
               labelcolor=TEXT, framealpha=0.9)

    def step_at(ex, t):
        k = 0
        while k + 1 < len(ex) and ex[k + 1]["t"] <= t + 1e-9:
            k += 1
        return k

    def update(fi):
        t = times[fi]
        for p, (ln, dot) in zip(panes, lines):
            d = p["d"]; ex = d["executed"]; k = step_at(ex, t)
            s = ex[k]; finished = k == len(ex) - 1
            known = (d["reveal_step"] >= 0) & (d["reveal_step"] <= k)
            p["img"].set_data(terrain_rgb(d, min(t, ex[-1]["t"]), known))
            if k < len(d["plans"]) and not finished:
                pl = d["plans"][k]
                p["plan"].set_data([q[0] for q in pl], [q[1] for q in pl])
            else:
                p["plan"].set_data([], [])
            px, py = xy(ex[:k + 1])
            p["trail"].set_data(px, py); p["bg"].set_data(px, py)
            p["rover"].set_data([s["x"]], [s["y"]])
            w = [e for e in ex[:k + 1] if e["wait"]]
            p["waits"].set_offsets(np.c_[[e["x"] for e in w], [e["y"] for e in w]]
                                   if w else np.empty((0, 2)))
            if finished and not d["result"]["success"]:
                p["dead"].set_data([s["x"]], [s["y"]]); state = "battery depleted"
            elif finished:
                state = f"goal reached at {ex[-1]['t']:.0f} s"
            else:
                state = "parked, recharging" if (k + 1 < len(ex) and ex[k + 1]["wait"]) else "driving"
            p["title"].set_text(f"{p['name']} · {state}")
            tt = [e["t"] for e in ex[:k + 1]]; bb = [100 * e["bat"] for e in ex[:k + 1]]
            ln.set_data(tt, bb); dot.set_data([tt[-1]], [bb[-1]])
        return []

    anim = FuncAnimation(fig, update, frames=len(times), blit=False)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("overview"); p.add_argument("plan"); p.add_argument("--out")
    p = sub.add_parser("compare"); p.add_argument("blind"); p.add_argument("aware")
    p.add_argument("--out")
    p = sub.add_parser("animate"); p.add_argument("plan"); p.add_argument("--out")
    p.add_argument("--fps", type=int, default=8)
    p = sub.add_parser("animate-compare"); p.add_argument("blind"); p.add_argument("aware")
    p.add_argument("--out"); p.add_argument("--fps", type=int, default=10)
    a = ap.parse_args()

    if a.cmd == "overview":
        out = a.out or os.path.join(os.path.dirname(a.plan) or ".", "overview.png")
        render_overview(load(a.plan), out)
    elif a.cmd == "compare":
        out = a.out or os.path.join(os.path.dirname(a.aware) or ".", "compare.png")
        render_compare(load(a.blind), load(a.aware), out)
    elif a.cmd == "animate-compare":
        out = a.out or os.path.join(os.path.dirname(a.aware) or ".", "compare.gif")
        render_animation_compare(load(a.blind), load(a.aware), out, a.fps)
    else:
        out = a.out or os.path.join(os.path.dirname(a.plan) or ".", "animation.gif")
        render_animation(load(a.plan), out, a.fps)


if __name__ == "__main__":
    main()
