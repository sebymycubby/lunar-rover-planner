#!/usr/bin/env python3
"""
Lunar Rover Planner — Visualizer
Produces: overview.png (static) and animation.gif (animated traversal)
"""

import json, math, sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.animation import FuncAnimation, PillowWriter

# ─── Terrain palette ───────────────────────────────────────────────────────
# 0=FLAT, 1=ROCKY_PATCH, 2=DUNE, 3=CRATER, 4=CRATER_FLOOR
TERRAIN_BASE = {
    0: np.array([0.55, 0.53, 0.50]),   # flat — mid grey regolith
    1: np.array([0.48, 0.45, 0.40]),   # rocky patch — darker grey-brown
    2: np.array([0.62, 0.57, 0.45]),   # dune — dusty tan
    3: np.array([0.38, 0.36, 0.34]),   # crater rim — dark grey
    4: np.array([0.22, 0.20, 0.19]),   # crater floor — very dark
}
SHADOW_COLOR = np.array([0.06, 0.07, 0.14])

def load_plan(path):
    with open(path) as f:
        return json.load(f)

# ─── Light / shadow grid ───────────────────────────────────────────────────

def compute_light_grid(W, H, craters, dunes, sun_angle_deg):
    sun_rad = math.radians(sun_angle_deg)
    sx, sy  = -math.cos(sun_rad), -math.sin(sun_rad)
    xs, ys  = np.arange(W, dtype=float), np.arange(H, dtype=float)
    XX, YY  = np.meshgrid(xs, ys)
    light   = np.ones((H, W))

    for c in craters:
        cx, cy, r, depth = c['cx'], c['cy'], c['r'], c['d']
        DX, DY = XX - cx, YY - cy
        dist   = np.sqrt(DX**2 + DY**2)
        inside = dist < r
        b_arr  = 2.0*(DX*sx + DY*sy)
        cv_arr = dist**2 - r**2
        disc   = np.maximum(b_arr**2 - 4.0*cv_arr, 0)
        t_rim  = (-b_arr + np.sqrt(disc)) / 2.0
        t_rim  = np.where(t_rim > 0, t_rim, np.inf)
        elev   = np.arctan2(depth, t_rim + 1e-9)
        sun_elev = abs(math.sin(sun_rad))*0.5 + 0.1
        shadow = np.clip((elev - math.radians(sun_elev*90)) / math.radians(30), 0, 1)
        cell_depth   = np.where(inside, depth*(1.0 - dist/(r+1e-9)), 0)
        depth_factor = np.where(depth > 0, cell_depth/depth, 0)
        light = np.minimum(light, 1.0 - np.clip(shadow*depth_factor, 0, 1))

    for d in dunes:
        cx, cy  = d['cx'], d['cy']
        length, width, height = d['l'], d['w'], d['h']
        arad    = math.radians(d['a'])
        DX, DY  = XX - cx, YY - cy
        lx      =  DX*math.cos(arad) + DY*math.sin(arad)
        ly      = -DX*math.sin(arad) + DY*math.cos(arad)
        on_dune = (np.abs(lx) <= length/2) & (np.abs(ly) <= width*3)
        sun_perp = math.cos(sun_angle_deg*math.pi/180 - arad)
        shadow_len = height * 2.5
        if sun_perp >= 0:
            shadow = np.where(ly < 0, np.clip(1.0 - np.abs(ly)/shadow_len, 0, 1)*0.7, 0)
        else:
            shadow = np.where(ly > 0, np.clip(1.0 - np.abs(ly)/shadow_len, 0, 1)*0.7, 0)
        light = np.minimum(light, 1.0 - np.clip(np.where(on_dune, shadow, 0), 0, 1))

    return np.clip(light, 0, 1)

def build_terrain_rgb(data, sun_angle_deg):
    W, H = data['width'], data['height']
    terrain = data['terrain']
    light   = compute_light_grid(W, H, data['craters'], data['dunes'], sun_angle_deg)
    rgb     = np.zeros((H, W, 3))
    for y in range(H):
        for x in range(W):
            t = terrain[y][x]['t']
            base = TERRAIN_BASE.get(t, TERRAIN_BASE[0])
            l    = light[y, x]
            rgb[y, x] = base * l + SHADOW_COLOR * (1.0 - l)
    return rgb, light

def build_cost_rgb(data):
    """Visualize traverse cost per cell."""
    W, H    = data['width'], data['height']
    terrain = data['terrain']
    costs   = np.array([[terrain[y][x]['c'] for x in range(W)] for y in range(H)])
    # Normalize to [0,1] with log scale to handle wide range
    log_costs = np.log1p(costs)
    norm      = (log_costs - log_costs.min()) / (log_costs.max() - log_costs.min() + 1e-9)
    cmap      = plt.cm.plasma
    return cmap(norm)[:,:,:3], costs

# ─── Static Overview ────────────────────────────────────────────────────────

def render_static(data, out_path="output/overview.png"):
    cfg    = data['config']
    result = data['result']
    path   = result['path']
    W, H   = data['width'], data['height']

    mid_sun = cfg['sun_angle_start'] + cfg['sun_deg_per_sec'] * result['total_time'] / 2
    rgb, light_grid = build_terrain_rgb(data, mid_sun)
    cost_rgb, costs = build_cost_rgb(data)

    fig, axes = plt.subplots(1, 3, figsize=(21, 7), facecolor='#080810')
    fig.suptitle('Lunar Rover — Time-Aware A* Planner', fontsize=15,
                 color='#e0d8c8', fontweight='bold', fontfamily='monospace', y=0.98)

    # ── Axes shared setup ──
    for ax in axes:
        ax.set_facecolor('#080810')
        for sp in ax.spines.values(): sp.set_color('#2a2a3a')
        ax.tick_params(colors='#556')

    # ── Left: Terrain + path ──
    ax = axes[0]
    ax.imshow(rgb, origin='upper', interpolation='nearest', aspect='equal')
    for c in data['craters']:
        ax.add_patch(plt.Circle((c['cx'], c['cy']), c['r'],
                                fill=False, edgecolor='#5577aa', lw=0.8, alpha=0.5))
    if path:
        px = [s['x'] for s in path]; py = [s['y'] for s in path]
        bats = [s['bat'] for s in path]
        ax.plot(px, py, color='#ffcc44', lw=1.4, alpha=0.7, zorder=5)
        sc = ax.scatter(px, py, c=bats, cmap='RdYlGn', vmin=0, vmax=1,
                        s=14, zorder=6, alpha=0.9)
        cb = plt.colorbar(sc, ax=ax, shrink=0.55, pad=0.02)
        cb.set_label('Battery', color='#aaa', fontsize=8, fontfamily='monospace')
        plt.setp(cb.ax.yaxis.get_ticklabels(), color='#888', fontsize=7)
    ax.plot(cfg['start'][0], cfg['start'][1], 's', color='#44aaff', ms=9, zorder=10,
            markeredgecolor='white', markeredgewidth=1.2)
    ax.plot(cfg['goal'][0],  cfg['goal'][1],  '*', color='#ff6644', ms=13, zorder=10,
            markeredgecolor='white', markeredgewidth=0.8)
    ax.set_title(f'Terrain + Path  (sun≈{mid_sun:.1f}°)',
                 color='#9999bb', fontsize=10, fontfamily='monospace')
    ax.set_xlim(0,W); ax.set_ylim(H,0)

    # Legend patches
    legend_items = [
        mpatches.Patch(color=TERRAIN_BASE[0], label='Flat'),
        mpatches.Patch(color=TERRAIN_BASE[1], label='Rocky'),
        mpatches.Patch(color=TERRAIN_BASE[2], label='Dune'),
        mpatches.Patch(color=TERRAIN_BASE[3], label='Crater rim'),
        mpatches.Patch(color=TERRAIN_BASE[4], label='Crater floor'),
    ]
    ax.legend(handles=legend_items, loc='lower right',
              facecolor='#1a1a2a', edgecolor='#444',
              labelcolor='#c8c0b0', fontsize=7)

    # ── Middle: Traverse cost map ──
    ax2 = axes[1]
    im = ax2.imshow(costs, origin='upper', cmap='plasma',
                    interpolation='nearest', aspect='equal')
    if path:
        ax2.plot([s['x'] for s in path], [s['y'] for s in path],
                 color='#ffffff', lw=1.2, alpha=0.6, zorder=5)
    ax2.plot(cfg['start'][0], cfg['start'][1], 's', color='#44aaff', ms=8, zorder=10,
             markeredgecolor='white', markeredgewidth=1.2)
    ax2.plot(cfg['goal'][0],  cfg['goal'][1],  '*', color='#ff6644', ms=12, zorder=10)
    cb2 = plt.colorbar(im, ax=ax2, shrink=0.55, pad=0.02)
    cb2.set_label('Traverse cost (s)', color='#aaa', fontsize=8, fontfamily='monospace')
    plt.setp(cb2.ax.yaxis.get_ticklabels(), color='#888', fontsize=7)
    ax2.set_title('Traverse Cost per Cell', color='#9999bb', fontsize=10,
                  fontfamily='monospace')
    ax2.set_xlim(0,W); ax2.set_ylim(H,0)

    # ── Right: Battery + light charts + stats ──
    ax3 = axes[2]
    ax3.set_facecolor('#080810'); ax3.axis('off')

    if path:
        times  = [s['t']     for s in path]
        bats   = [s['bat']   for s in path]
        lights = [s['light'] for s in path]
        dts    = [s['dt']    for s in path]

        # Battery chart
        inner1 = ax3.inset_axes([0.05, 0.55, 0.90, 0.38])
        inner1.set_facecolor('#0f0f1a')
        inner1.plot(times, [b*100 for b in bats], color='#44ff88', lw=1.8)
        inner1.axhline(cfg['battery_min']*100, color='#ff4444', ls='--', lw=1.0,
                       label=f'Min ({cfg["battery_min"]*100:.0f}%)')
        inner1.set_xlabel('Time (s)', color='#888', fontsize=8, fontfamily='monospace')
        inner1.set_ylabel('Battery %', color='#888', fontsize=8, fontfamily='monospace')
        inner1.tick_params(colors='#666', labelsize=7)
        for sp in inner1.spines.values(): sp.set_color('#333')
        inner1.legend(loc='lower right', facecolor='#1a1a2a',
                      edgecolor='#333', labelcolor='#c8c0b0', fontsize=7)
        inner1.set_title('Battery over Time', color='#9999bb', fontsize=9,
                         fontfamily='monospace')

        # Light level chart
        inner2 = ax3.inset_axes([0.05, 0.08, 0.90, 0.38])
        inner2.set_facecolor('#0f0f1a')
        inner2.fill_between(times, lights, alpha=0.35, color='#ffdd88')
        inner2.plot(times, lights, color='#ffdd88', lw=1.5)
        inner2.set_xlabel('Time (s)', color='#888', fontsize=8, fontfamily='monospace')
        inner2.set_ylabel('Light level', color='#888', fontsize=8, fontfamily='monospace')
        inner2.tick_params(colors='#666', labelsize=7)
        for sp in inner2.spines.values(): sp.set_color('#333')
        inner2.set_title('Light Level along Path', color='#9999bb', fontsize=9,
                         fontfamily='monospace')

    # Summary
    status = "✓ SUCCESS" if result['success'] else "✗ FAILED"
    color  = '#44ff88' if result['success'] else '#ff4444'
    ax3.text(0.5, 0.97, status, ha='center', va='top', color=color,
             fontsize=13, fontweight='bold', fontfamily='monospace',
             transform=ax3.transAxes)
    info = [
        f"Steps      : {len(path)}",
        f"Total time : {result['total_time']:.1f} s",
        f"Final batt : {result['final_battery']*100:.1f} %",
        f"Drain/sec  : {cfg['drain_per_sec']:.4f}",
        f"Gain/sec   : {cfg['max_gain_per_sec']:.4f}",
        f"Ratio G/D  : {cfg['max_gain_per_sec']/max(cfg['drain_per_sec'],1e-9):.3f}",
        f"Grid       : {W}×{H}",
        f"Craters    : {len(data['craters'])}",
        f"Dunes      : {len(data['dunes'])}",
    ]
    for i, line in enumerate(info):
        ax3.text(0.08, 0.93 - i*0.043, line, ha='left', va='top',
                 color='#b0a090', fontsize=9, fontfamily='monospace',
                 transform=ax3.transAxes)

    plt.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='#080810')
    plt.close(fig)
    print(f"Static overview → {out_path}")

# ─── Animation ──────────────────────────────────────────────────────────────

def render_animation(data, out_path="output/animation.gif", fps=6):
    cfg        = data['config']
    result     = data['result']
    path_steps = result['path']
    vis_snaps  = result['visibility']
    W, H       = data['width'], data['height']
    if not path_steps:
        print("No path to animate."); return

    N = len(path_steps)
    fig = plt.figure(figsize=(14, 7), facecolor='#080810')
    ax_map  = fig.add_axes([0.02, 0.10, 0.55, 0.84])
    ax_bat  = fig.add_axes([0.62, 0.55, 0.35, 0.36])
    ax_cost = fig.add_axes([0.62, 0.10, 0.35, 0.36])

    for ax in [ax_map, ax_bat, ax_cost]:
        ax.set_facecolor('#0a0a12')
        for sp in ax.spines.values(): sp.set_color('#2a2a3a')
        ax.tick_params(colors='#556', labelsize=7)
        ax.grid(True, color='#111122', lw=0.4)

    fig.text(0.5, 0.975, 'LUNAR ROVER — REAL-TIME TRAVERSAL',
             ha='center', va='top', fontsize=12,
             color='#d0c8b8', fontweight='bold', fontfamily='monospace')

    # Initial frame
    ps0   = path_steps[0]
    rgb0, _ = build_terrain_rgb(data, ps0['sun'])
    fog0  = np.array(vis_snaps[0]).reshape(H, W).astype(float)
    disp0 = rgb0.copy()
    disp0[fog0 == 0] *= 0.10
    img_h = ax_map.imshow(disp0, origin='upper', interpolation='nearest',
                           aspect='equal', extent=[0,W,H,0])

    for c in data['craters']:
        ax_map.add_patch(plt.Circle((c['cx'],c['cy']),c['r'],
                                     fill=False, edgecolor='#5577aa', lw=0.7, alpha=0.4))

    sx, sy = cfg['start']
    gx, gy = cfg['goal']
    ax_map.plot(sx, sy, 's', color='#44aaff', ms=8, zorder=10,
                markeredgecolor='white', markeredgewidth=1.0)
    ax_map.plot(gx, gy, '*', color='#ff6644', ms=12, zorder=10,
                markeredgecolor='white', markeredgewidth=0.8)

    obs_circ = plt.Circle((sx,sy), cfg['obs_radius'], fill=False,
                            edgecolor='#88bbff', lw=0.7, ls='--', alpha=0.5, zorder=9)
    ax_map.add_patch(obs_circ)
    rover_dot, = ax_map.plot([sx],[sy], 'o', color='#ffee44', ms=7, zorder=12,
                              markeredgecolor='#ff8800', markeredgewidth=1.3)
    trail_line, = ax_map.plot([],[],  color='#ffcc44', lw=1.0, alpha=0.55, zorder=8)
    time_txt = ax_map.text(0.02, 0.98, '', transform=ax_map.transAxes,
                            color='#88aacc', fontsize=8, va='top', fontfamily='monospace')

    # Sun quiver
    aqx, aqy = W*0.88, H*0.09
    sun_q = [ax_map.quiver(aqx, aqy, 1, 0, color='#ffee55',
                            scale=1, scale_units='xy', angles='xy',
                            width=0.006, zorder=20)]
    ax_map.text(aqx, aqy-3.5, '☀', ha='center', color='#ffee55',
                fontsize=9, zorder=21)
    ax_map.set_xlim(0,W); ax_map.set_ylim(H,0)

    # Battery chart
    bat_line, = ax_bat.plot([],[],  color='#44ff88', lw=1.6)
    ax_bat.axhline(cfg['battery_min']*100, color='#ff4444', ls='--', lw=1.0)
    ax_bat.set_xlim(0, max(result['total_time'], 1))
    ax_bat.set_ylim(0, 105)
    ax_bat.set_xlabel('Time (s)', color='#778', fontsize=7, fontfamily='monospace')
    ax_bat.set_ylabel('Battery %', color='#778', fontsize=7, fontfamily='monospace')
    ax_bat.set_title('Battery', color='#9999bb', fontsize=8, fontfamily='monospace')

    # Per-step cost chart
    step_costs = [s['dt'] for s in path_steps]
    cost_bar_colors = ['#ff6644' if c > 5 else '#ffaa44' if c > 2 else '#44aaff'
                       for c in step_costs]
    cost_line, = ax_cost.plot([],[], color='#ff9944', lw=1.4)
    ax_cost.set_xlim(0, N)
    ax_cost.set_ylim(0, max(step_costs)*1.15)
    ax_cost.set_xlabel('Step', color='#778', fontsize=7, fontfamily='monospace')
    ax_cost.set_ylabel('Traverse cost (s)', color='#778', fontsize=7, fontfamily='monospace')
    ax_cost.set_title('Cell Traverse Cost', color='#9999bb', fontsize=8, fontfamily='monospace')
    ax_cost.axhline(np.mean(step_costs), color='#aaaaff', ls=':', lw=0.8, alpha=0.6)

    times_all = [s['t']   for s in path_steps]
    bats_all  = [s['bat'] for s in path_steps]

    def update(fi):
        ps   = path_steps[fi]
        fog  = np.array(vis_snaps[min(fi, len(vis_snaps)-1)]).reshape(H,W).astype(float)
        rgb, _ = build_terrain_rgb(data, ps['sun'])
        disp = rgb.copy(); disp[fog==0] *= 0.10
        img_h.set_data(disp)

        trail_line.set_data([path_steps[i]['x'] for i in range(fi+1)],
                            [path_steps[i]['y'] for i in range(fi+1)])
        rover_dot.set_data([ps['x']], [ps['y']])
        obs_circ.center = (ps['x'], ps['y'])

        # Sun arrow
        alen = W*0.07
        sr   = math.radians(ps['sun'])
        sun_q[0].set_UVC(math.cos(sr)*alen, math.sin(sr)*alen)
        sun_q[0].set_offsets([[aqx, aqy]])

        time_txt.set_text(f't={ps["t"]:.1f}s  sun={ps["sun"]:.1f}°  '
                          f'bat={ps["bat"]*100:.1f}%  light={ps["light"]:.2f}')

        bat_line.set_data(times_all[:fi+1], [b*100 for b in bats_all[:fi+1]])
        cost_line.set_data(range(fi+1), step_costs[:fi+1])

        return [img_h, trail_line, rover_dot, bat_line, cost_line, time_txt]

    print(f"Rendering {N} frames...")
    anim = FuncAnimation(fig, update, frames=N, interval=1000//fps, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps), dpi=100)
    plt.close(fig)
    print(f"Animation → {out_path}")

# ─── Entry Point ────────────────────────────────────────────────────────────

if __name__ == '__main__':
    json_path = sys.argv[1] if len(sys.argv) > 1 else "output/plan.json"
    print(f"Loading {json_path}...")
    data = load_plan(json_path)
    os.makedirs("output", exist_ok=True)
    render_static(data,    "output/overview.png")
    render_animation(data, "output/animation.gif", fps=6)
    print("Done.")
