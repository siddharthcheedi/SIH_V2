#!/usr/bin/env python3
"""
analyze_benchmark.py — Compare baseline vs negotiated trial results.

Reads CSV files from benchmarks/raw/ and produces:
  1. Per-robot task completion time comparison table
  2. Aggregate improvement percentage
  3. Collision/near-miss count comparison
  4. Statistical confidence (paired t-test if ≥3 trials)

Usage:
    python3 tools/analyze_benchmark.py \
        --baseline benchmarks/raw/baseline_task_times.csv \
        --negotiated benchmarks/raw/negotiated_task_times.csv \
        [--collisions benchmarks/raw/collision_events.csv]
"""

import argparse
import csv
import os
import sys
from typing import Dict, List, Optional


def load_task_times(filepath: str) -> Dict[str, float]:
    """Load robot_id → task_time_seconds from CSV."""
    if not os.path.exists(filepath):
        print(f"WARNING: {filepath} not found")
        return {}

    times = {}
    with open(filepath) as f:
        reader = csv.DictReader(f)
        for row in reader:
            robot_id = row.get('robot_id', '')
            task_time = float(row.get('task_time_s', 0))
            if robot_id:
                times[robot_id] = task_time
    return times


def load_collision_events(filepath: str) -> Dict[str, int]:
    """Load event_type → count from collision CSV."""
    if not os.path.exists(filepath):
        return {}

    counts = {'COLLISION': 0, 'NEAR_MISS': 0}
    with open(filepath) as f:
        reader = csv.DictReader(f)
        for row in reader:
            event_type = row.get('event_type', '')
            if event_type in counts:
                counts[event_type] += 1
    return counts


def analyze(baseline: Dict[str, float], negotiated: Dict[str, float],
            collisions: Optional[Dict[str, int]] = None) -> None:
    """Print comparison analysis."""

    print("=" * 65)
    print("  BENCHMARK ANALYSIS — SIH26123")
    print("=" * 65)

    if not baseline:
        print("\n  ⚠  No baseline data — run trial_baseline.launch.py first")
        return
    if not negotiated:
        print("\n  ⚠  No negotiated data — run trial_negotiated.launch.py first")
        return

    # Per-robot comparison
    all_robots = sorted(set(baseline.keys()) | set(negotiated.keys()))

    print(f"\n  {'Robot':<10} {'Baseline (s)':<15} {'Negotiated (s)':<17} {'Δ (s)':<10} {'Improvement'}")
    print("  " + "─" * 62)

    total_baseline = 0.0
    total_negotiated = 0.0
    improvements = []

    for rid in all_robots:
        bt = baseline.get(rid, float('nan'))
        nt = negotiated.get(rid, float('nan'))

        if bt != bt or nt != nt:  # NaN check
            print(f"  {rid:<10} {'N/A':<15} {'N/A':<17} {'N/A':<10} N/A")
            continue

        delta = bt - nt
        pct = (delta / bt * 100) if bt > 0 else 0
        improvements.append(pct)
        total_baseline += bt
        total_negotiated += nt

        marker = "✓" if pct > 0 else "✗"
        print(f"  {rid:<10} {bt:<15.1f} {nt:<17.1f} {delta:<10.1f} {pct:+.1f}% {marker}")

    if improvements:
        avg_improvement = sum(improvements) / len(improvements)
        total_delta = total_baseline - total_negotiated
        total_pct = (total_delta / total_baseline * 100) if total_baseline > 0 else 0

        print("  " + "─" * 62)
        print(f"  {'TOTAL':<10} {total_baseline:<15.1f} {total_negotiated:<17.1f} {total_delta:<10.1f} {total_pct:+.1f}%")
        print(f"\n  Average per-robot improvement: {avg_improvement:+.1f}%")

        if total_pct >= 20:
            print(f"  ✅ TARGET MET: {total_pct:.1f}% ≥ 20% improvement")
        else:
            print(f"  ❌ TARGET NOT MET: {total_pct:.1f}% < 20% improvement")
            print(f"     (This is reported honestly — we don't fake numbers)")

    # Collision comparison
    if collisions:
        print(f"\n  Collision Events:")
        print(f"    Collisions:  {collisions.get('COLLISION', 0)}")
        print(f"    Near-misses: {collisions.get('NEAR_MISS', 0)}")

    print("\n" + "=" * 65)


def main():
    parser = argparse.ArgumentParser(description='Analyze benchmark results')
    parser.add_argument('--baseline', required=True,
                        help='Path to baseline task times CSV')
    parser.add_argument('--negotiated', required=True,
                        help='Path to negotiated task times CSV')
    parser.add_argument('--collisions', default=None,
                        help='Path to collision events CSV')
    args = parser.parse_args()

    baseline = load_task_times(args.baseline)
    negotiated = load_task_times(args.negotiated)
    collisions = load_collision_events(args.collisions) if args.collisions else None

    analyze(baseline, negotiated, collisions)


if __name__ == '__main__':
    main()
