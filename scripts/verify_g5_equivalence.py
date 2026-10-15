#!/usr/bin/env python3
"""Compare off/mock Nav2 outcomes on the same frozen SIL map and profiles."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

GOALS = (
    ("free", "reachable_goal"),
    ("avoidance", "obstacle_avoidance_goal"),
    ("narrow_passage", "narrow_passage_goal"),
)


def _read_json(path, field):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {field}: {exc}") from exc


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _summarize_goal(report, field, label):
    goal = report.get(field)
    if not isinstance(goal, dict):
        raise ValueError(f"{report.get('scenario')} report missing {field}")
    if goal.get("result") != "SUCCEEDED":
        raise ValueError(f"{label} goal was not SUCCEEDED: {goal.get('result')!r}")
    distance = goal.get("distance_to_goal_m")
    if isinstance(distance, bool) or not isinstance(distance, (float, int)):
        raise ValueError(f"{label} goal lacks numeric Ground-Truth distance")
    if not math.isfinite(float(distance)) or float(distance) > 0.20:
        raise ValueError(f"{label} goal exceeded frozen 0.20 m arrival criterion")
    zero = goal.get("zero_command")
    if (not isinstance(zero, dict) or zero.get("zero") is not True
            or tuple(zero.get("values", ())) != (0.0, 0.0, 0.0)):
        raise ValueError(f"{label} goal lacks its final zero command")
    return {"result": goal["result"], "ground_truth_distance_to_goal_m": float(distance),
            "final_zero_command": zero}


def compare_reports(off_path, mock_path, off_transport_path, mock_transport_path,
                    map_path, world_path, profile_path, common_path):
    off = _read_json(off_path, "off navigation report")
    mock = _read_json(mock_path, "mock navigation report")
    off_transport = _read_json(off_transport_path, "off transport report")
    mock_transport = _read_json(mock_transport_path, "mock transport report")
    artifacts = {"map": _sha256(map_path), "world": _sha256(world_path),
                 "nav2_profile": _sha256(profile_path), "common_profile": _sha256(common_path)}

    for name, report in (("off", off), ("mock", mock)):
        if report.get("pass") is not True or report.get("scenario") != "baseline":
            raise ValueError(f"{name} Nav2 report did not pass the baseline scenario")
        if report.get("hardware_topics_or_devices") != "none":
            raise ValueError(f"{name} run does not have the simulation-only scope")
    for name, transport, mode in (("off", off_transport, "off"),
                                  ("mock", mock_transport, "mock")):
        expected_state = "unavailable" if mode == "off" else "ok"
        expected_detection_count = 0 if mode == "off" else 1
        if (transport.get("pass") is not True or transport.get("mode") != mode
                or transport.get("result_messages", 0) < 3
                or transport.get("status_messages", 0) < 3
            or transport.get("expected_state") != expected_state
            or transport.get("detection_count") != expected_detection_count
                or transport.get("command_topics_published") != []):
            raise ValueError(f"{name} vision transport evidence is incomplete")

    goals = {}
    for label, key in GOALS:
        off_goal = _summarize_goal(off, key, f"off/{label}")
        mock_goal = _summarize_goal(mock, key, f"mock/{label}")
        goals[label] = {"off": off_goal, "mock": mock_goal,
                        "absolute_distance_delta_m": abs(
                            off_goal["ground_truth_distance_to_goal_m"]
                            -mock_goal["ground_truth_distance_to_goal_m"])}

    if off.get("map") != mock.get("map"):
        raise ValueError("off/mock runs did not use identical map metadata")
    if off.get("safety_limits") != mock.get("safety_limits"):
        raise ValueError("off/mock runs used different safety limits")

    return {
        "pass": True,
        "status": "PASS",
        "scope": "G5_NAV2_VISION_OFF_MOCK_EQUIVALENCE_SIL_ONLY",
        "vision_modes": ["off", "mock"],
        "goals": goals,
        "identical_artifact_sha256": artifacts,
        "safety_limits": off["safety_limits"],
        "navigation_acceptance_claim": "same frozen G4 reachable-goal criteria passed in both modes",
        "real_inference_used": False,
        "hardware_topics_or_devices": "none",
        "errors": [],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--off-report", type=Path, required=True)
    parser.add_argument("--mock-report", type=Path, required=True)
    parser.add_argument("--off-transport", type=Path, required=True)
    parser.add_argument("--mock-transport", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--common-profile", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    report = {"pass": False, "status": "FAIL",
              "scope": "G5_NAV2_VISION_OFF_MOCK_EQUIVALENCE_SIL_ONLY",
              "errors": []}
    try:
        report = compare_reports(args.off_report, args.mock_report,
                                 args.off_transport, args.mock_transport,
                                 args.map, args.world, args.profile, args.common_profile)
    except Exception as exc:
        report["errors"] = [f"{type(exc).__name__}: {exc}"]
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"{report['status']} [G5_NAV2_VISION_OFF_MOCK_EQUIVALENCE_SIL_ONLY]: "
          + (str(args.report) if report["pass"] else "; ".join(report["errors"])),
          flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
