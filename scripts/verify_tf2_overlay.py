#!/usr/bin/env python3
"""Read-only TF2 source and Nav2/RMW dependency preflight; no ROS nodes."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys


PINNED_COMMIT = "9997e969556f5fe5d32ad498d02b3b3473da70b6"


def default_overlay_root(home=None):
    home = Path.home() if home is None else Path(home)
    canonical = home/"linorobot2_tf2_fix"
    historical = home/"linobot2_tf2_fix"
    if canonical.exists() or not historical.exists():
        return canonical
    return historical


def library_paths(output):
    if re.search(r"=>\s+not found", output):
        raise RuntimeError("unresolved shared-library dependency: " + output)
    return dict(re.findall(r"^\s*(\S+)\s+=>\s+(\S+)\s+\(", output, re.MULTILINE))


def validate_tf2_libraries(output, root):
    paths = library_paths(output)
    prefix = Path(root).resolve()/"install/lib"
    selected = {}
    for name in ("libtf2.so", "libtf2_ros.so"):
        if name not in paths:
            raise RuntimeError(f"required Nav2 dependency absent: {name}")
        actual = Path(paths[name]).resolve()
        expected = (prefix/name).resolve()
        if not expected.is_relative_to(prefix) or actual != expected:
            raise RuntimeError(f"Nav2 does not select the pinned overlay for {name}: {actual}")
        selected[name] = str(actual)
    return selected


def inspect_overlay(root):
    commit = subprocess.check_output(
        ["git", "-C", str(root/"src/geometry2"), "rev-parse", "HEAD"],
        text=True, timeout=10).strip()
    if commit != PINNED_COMMIT:
        raise RuntimeError(f"unexpected TF2 source commit: {commit}")
    controller = Path("/opt/ros/humble/lib/libcontroller_server_core.so")
    controller_output = subprocess.check_output(["ldd", str(controller)], text=True, timeout=10)
    selected = validate_tf2_libraries(controller_output, root)
    rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    if not re.fullmatch(r"rmw_[A-Za-z0-9_]+", rmw):
        raise RuntimeError(f"invalid middleware implementation name: {rmw}")
    middleware = Path("/opt/ros/humble/lib")/f"lib{rmw}.so"
    middleware_output = subprocess.check_output(["ldd", str(middleware)], text=True, timeout=10)
    rmw_paths = library_paths(middleware_output)
    return {"source_commit": commit, "controller": str(controller),
            "tf2_libraries": selected,
            "tf2_library_sha256": {name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                                   for name, path in selected.items()},
            "middleware": rmw, "middleware_dependencies": rmw_paths}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=default_overlay_root())
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    report = {"pass": False, "scope": "TF2_OVERLAY_READ_ONLY", "errors": []}
    try:
        report.update(inspect_overlay(args.root.resolve()))
        report["pass"] = True
    except Exception as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    report["status"] = "PASS" if report["pass"] else "FAIL"
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+"\n")
    print(f"{report['status']} [TF2_OVERLAY_READ_ONLY]: "
          + (str(args.report) if report["pass"] else "; ".join(report["errors"])), flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())