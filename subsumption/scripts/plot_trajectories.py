#!/usr/bin/env python3
"""Plot 2D trajectories and print quantitative metrics for Subsumption levels.

Usage:
    python3 plot_trajectories.py \
        --level0 results/level0.csv \
        --level1 results/level1.csv \
        --level2 results/level2.csv \
        --out results/trajectory_comparison.png
"""

import argparse
import csv
import math
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


LAYER_COLORS = {
    'IDLE': '#7f8c8d',
    'L0_AVOID': '#e74c3c',
    'L1_WANDER': '#f39c12',
    'L2_EXPLORE': '#2980b9',
}


def load_csv(path: str):
    rows = []
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                't': float(r['time_s']),
                'x': float(r['x']),
                'y': float(r['y']),
                'v': float(r['linear_x']),
                'w': float(r['angular_z']),
                'min_scan': float(r['min_scan']),
                'layer': r['active_layer'],
            })
    return rows


def compute_metrics(rows):
    if not rows:
        return {}
    dist = 0.0
    for i in range(1, len(rows)):
        dist += math.hypot(rows[i]['x'] - rows[i - 1]['x'], rows[i]['y'] - rows[i - 1]['y'])
    xs = [r['x'] for r in rows]
    ys = [r['y'] for r in rows]
    bbox_area = (max(xs) - min(xs)) * (max(ys) - min(ys))
    valid_scans = [r['min_scan'] for r in rows if math.isfinite(r['min_scan'])]
    min_clear = min(valid_scans) if valid_scans else float('nan')

    counts = {'IDLE': 0, 'L0_AVOID': 0, 'L1_WANDER': 0, 'L2_EXPLORE': 0}
    for r in rows:
        counts[r['layer']] = counts.get(r['layer'], 0) + 1
    n = len(rows)
    return {
        'duration_s': rows[-1]['t'] - rows[0]['t'],
        'path_len_m': dist,
        'bbox_area_m2': bbox_area,
        'min_clearance_m': min_clear,
        'pct_idle': 100.0 * counts['IDLE'] / n,
        'pct_l0': 100.0 * counts['L0_AVOID'] / n,
        'pct_l1': 100.0 * counts['L1_WANDER'] / n,
        'pct_l2': 100.0 * counts['L2_EXPLORE'] / n,
    }


def plot_panel(ax, rows, title: str):
    for i in range(1, len(rows)):
        c = LAYER_COLORS.get(rows[i]['layer'], '#34495e')
        ax.plot(
            [rows[i - 1]['x'], rows[i]['x']],
            [rows[i - 1]['y'], rows[i]['y']],
            color=c, linewidth=2.0, alpha=0.9
        )
    if rows:
        ax.scatter(rows[0]['x'], rows[0]['y'], c='#2ecc71', s=70, zorder=5, edgecolors='black', label='Start')
        ax.scatter(rows[-1]['x'], rows[-1]['y'], c='#8e44ad', marker='X', s=80, zorder=5, edgecolors='black', label='End')
    ax.set_title(title, fontsize=11, fontweight='bold')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, linestyle='--', alpha=0.4)


def main():
    parser = argparse.ArgumentParser(description='Plot Subsumption trajectory comparison')
    parser.add_argument('--level0', type=str, default=None)
    parser.add_argument('--level1', type=str, default=None)
    parser.add_argument('--level2', type=str, default=None)
    parser.add_argument('--out', type=str, required=True)
    args = parser.parse_args()

    entries = []
    if args.level0 and os.path.exists(args.level0):
        entries.append(('Level 0: Avoid Only', load_csv(args.level0)))
    if args.level1 and os.path.exists(args.level1):
        entries.append(('Level 0 + 1: Avoid + Wander', load_csv(args.level1)))
    if args.level2 and os.path.exists(args.level2):
        entries.append(('Level 0 + 1 + 2: Full Subsumption', load_csv(args.level2)))

    if not entries:
        raise SystemExit('No valid CSV files found to plot.')

    fig, axes = plt.subplots(1, len(entries), figsize=(5.5 * len(entries), 5.0), squeeze=False)
    print('\n| Run | Duration (s) | Path (m) | BBox Area (m^2) | Min Clearance (m) | L0 Avoid (%) | L1 Wander (%) | L2 Explore (%) | Idle (%) |')
    print('| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |')

    for idx, (title, rows) in enumerate(entries):
        m = compute_metrics(rows)
        plot_panel(axes[0][idx], rows, title)
        print(
            f"| {title} | {m['duration_s']:.1f} | {m['path_len_m']:.2f} | "
            f"{m['bbox_area_m2']:.2f} | {m['min_clearance_m']:.3f} | "
            f"{m['pct_l0']:.1f} | {m['pct_l1']:.1f} | {m['pct_l2']:.1f} | {m['pct_idle']:.1f} |"
        )

    legend_handles = [
        Line2D([0], [0], color=LAYER_COLORS['L0_AVOID'], lw=2.5, label='Layer 0: Avoid'),
        Line2D([0], [0], color=LAYER_COLORS['L1_WANDER'], lw=2.5, label='Layer 1: Wander'),
        Line2D([0], [0], color=LAYER_COLORS['L2_EXPLORE'], lw=2.5, label='Layer 2: Explore'),
        Line2D([0], [0], color=LAYER_COLORS['IDLE'], lw=2.5, label='Idle'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#2ecc71', markeredgecolor='k', markersize=8, label='Start'),
        Line2D([0], [0], marker='X', color='w', markerfacecolor='#8e44ad', markeredgecolor='k', markersize=9, label='End'),
    ]
    fig.legend(handles=legend_handles, loc='lower center', ncol=6, frameon=True, fontsize=9)
    plt.tight_layout(rect=[0, 0.08, 1, 1])

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=180)
    print(f'\nSaved trajectory figure to: {args.out}')


if __name__ == '__main__':
    main()
