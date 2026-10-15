#!/usr/bin/env python3
"""Analyze the documented G3 raw-odometry yaw bias from a recorded phase3 report.

Read-only: computes per-turn raw/GT yaw ratios and the wheel-separation a rigid
no-slip model would need to explain the measured ground-truth yaw from the raw
joint-integrated odometry. This is evidence analysis, not parameter tuning.
"""

import argparse
import json
from pathlib import Path
import sys

# Fortress 2WD model contract (src/linorobot2/.../2wd_fortress.urdf.xacro):
WHEEL_RADIUS_M = 0.03375
CONTROLLER_WHEEL_SEPARATION_M = 0.126  # wheel_pos_y * 2


def analyze_turns(report):
    """Return per-turn and aggregate bias metrics for segments of kind 'turn'."""
    turns = [s for s in report["square_segments"] if s["kind"] == "turn"]
    if not turns:
        raise ValueError("report contains no turn segments")
    rows = []
    for seg in turns:
        gt_yaw = float(seg["ground_truth_delta"])
        raw_yaw = float(seg["unfiltered_odom_delta"])
        imu_yaw = float(seg["imu"]["integrated_yaw_rad"])
        if gt_yaw <= 0 or raw_yaw <= 0:
            raise ValueError(f"{seg['name']}: expected positive yaw deltas")
        wheels = seg["mean_wheel_velocity_rad_s"]
        d_omega = wheels["right_wheel_joint"]-wheels["left_wheel_joint"]
        ratio = raw_yaw/gt_yaw
        effective_sep = ratio*CONTROLLER_WHEEL_SEPARATION_M
        rows.append({
            "name": seg["name"],
            "ground_truth_yaw_rad": gt_yaw,
            "raw_odom_yaw_rad": raw_yaw,
            "imu_integrated_yaw_rad": imu_yaw,
            "raw_over_gt": ratio,
            "raw_minus_gt_rad": raw_yaw-gt_yaw,
            "effective_no_slip_separation_m": effective_sep,
            "mean_wheel_delta_omega_rad_s": d_omega,
            "imu_matches_gt_within": abs(imu_yaw-gt_yaw) < 0.02,
        })
    ratios = [r["raw_over_gt"] for r in rows]
    spread = max(ratios)-min(ratios)
    return rows, {"ratio_min": min(ratios), "ratio_max": max(ratios),
                  "ratio_spread": spread, "turn_count": len(rows),
                  "imu_matches_gt_all": all(r["imu_matches_gt_within"] for r in rows),
                  "controller_wheel_separation_m": CONTROLLER_WHEEL_SEPARATION_M,
                  "wheel_radius_m": WHEEL_RADIUS_M}


def analyze_motion_report(report):
    """Return per-direction turn ratios from a G2-style motion report."""
    turns = [s for s in report["segments"] if s["name"].startswith("turn_")]
    if len(turns) < 2:
        raise ValueError("motion report must contain turn_positive and turn_negative")
    rows = []
    for seg in turns:
        gt_yaw = float(seg["signed_motion"])
        raw_yaw = float(seg["odom_motion"]["unfiltered"]["signed_yaw_delta_rad"])
        if gt_yaw == 0 or raw_yaw == 0 or (gt_yaw < 0) != (raw_yaw < 0):
            raise ValueError(f"{seg['name']}: sign or magnitude mismatch")
        rows.append({
            "name": seg["name"],
            "ground_truth_yaw_rad": gt_yaw,
            "raw_odom_yaw_rad": raw_yaw,
            "raw_over_gt": raw_yaw/gt_yaw,
            "commanded_angular_z_rps": seg["command"]["angular_z_rps"],
        })
    ratios = [r["raw_over_gt"] for r in rows]
    return rows, {"ratio_min": min(ratios), "ratio_max": max(ratios),
                  "ratio_spread": max(ratios)-min(ratios),
                  "turn_count": len(rows),
                  "directions": [r["name"] for r in rows]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path,
                        help="G3-style phase3.json with square_segments")
    parser.add_argument("--motion-report", type=Path,
                        help="optional G2-style motion.json with signed turn segments")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.report is None and args.motion_report is None:
        parser.error("at least one of --report or --motion-report is required")
    result = {"pass": False, "status": "FAIL", "errors": []}
    try:
        result["pass"] = True
        if args.report is not None:
            report = json.loads(args.report.read_text())
            rows, summary = analyze_turns(report)
            result.update({"turns": rows, "summary": summary})
        if args.motion_report is not None:
            motion_rows, motion_summary = analyze_motion_report(
                json.loads(args.motion_report.read_text()))
            result["motion_turns"] = motion_rows
            result["motion_summary"] = motion_summary
    except Exception as exc:
        result["errors"] = [f"{type(exc).__name__}: {exc}"]
        result["pass"] = False
    result["status"] = "PASS" if result["pass"] else "FAIL"
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    for row in result.get("turns", []):
        print(f"{row['name']}: raw/GT={row['raw_over_gt']:.6f} "
              f"raw-GT={row['raw_minus_gt_rad']:+.6f} rad "
              f"eff_sep={row['effective_no_slip_separation_m']:.4f} m "
              f"imu~gt={row['imu_matches_gt_within']}")
    if result["pass"] and "summary" in result:
        s = result["summary"]
        print(f"ratio range {s['ratio_min']:.4f}..{s['ratio_max']:.4f} "
              f"(spread {s['ratio_spread']:.4f}); imu_matches_gt_all={s['imu_matches_gt_all']}")
    for row in result.get("motion_turns", []):
        print(f"{row['name']}: raw/GT={row['raw_over_gt']:.6f} "
              f"cmd_w={row['commanded_angular_z_rps']:+.2f} rad/s")
    if "motion_summary" in result:
        m = result["motion_summary"]
        print(f"motion ratio range {m['ratio_min']:.4f}..{m['ratio_max']:.4f} "
              f"across {m['directions']}")
    print(f"{result['status']} [G3_YAW_BIAS_ANALYSIS]: {args.out}", flush=True)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
