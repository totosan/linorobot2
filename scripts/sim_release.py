#!/usr/bin/env python3
"""Create, validate, activate, and roll back immutable simulation releases."""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import platform
import subprocess
import tempfile


PROCESS_MARKERS = (
    "ros2 launch linorobot2_",
    "ign gazebo",
    "supervised_process.py",
)


def git_value(source, *args):
    return subprocess.check_output(
        ["git", "-C", str(source), *args], text=True, timeout=10
    ).strip()


def installed_version(command):
    try:
        return subprocess.check_output(command, text=True, timeout=10).strip()
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None


def dpkg_versions(packages):
    command = ["dpkg-query", "-W", "-f=${Package}=${Version}\\n", *packages]
    output = installed_version(command)
    return sorted(output.splitlines()) if output else []


def write_manifest(release, source, validation_status, validation_commands=None):
    release = Path(release).resolve()
    source = Path(source).resolve()
    if validation_status not in {"pending", "passed", "failed"}:
        raise ValueError("validation status must be pending, passed, or failed")
    dirty = git_value(source, "status", "--porcelain")
    manifest = {
        "schema_version": 1,
        "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "release_path": str(release),
        "source_path": str(source),
        "git": {
            "commit": git_value(source, "rev-parse", "HEAD"),
            "remote": git_value(source, "remote", "get-url", "origin"),
            "dirty": bool(dirty),
        },
        "platform": {
            "system": platform.system(),
            "machine": platform.machine(),
            "release": platform.release(),
        },
        "versions": {
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "python": platform.python_version(),
            "colcon": installed_version(["colcon", "version-check"]),
            "ign": installed_version(["ign", "gazebo", "--versions"]),
            "debian_packages": dpkg_versions([
                "ignition-fortress",
                "python3-colcon-common-extensions",
                "python3-zmq",
                "ros-humble-navigation2",
                "ros-humble-rmw-cyclonedds-cpp",
                "ros-humble-ros-base",
                "ros-humble-ros-ign-bridge",
            ]),
        },
        "validation": {
            "status": validation_status,
            "commands": validation_commands or [],
        },
    }
    evidence = release / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    target = evidence / "release-manifest.json"
    with tempfile.NamedTemporaryFile(
        "w", dir=evidence, prefix=".release-manifest-", delete=False
    ) as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(target)
    return target


def read_manifest(release):
    path = Path(release) / "evidence" / "release-manifest.json"
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as error:
        raise ValueError(f"release manifest is missing: {path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"release manifest is invalid: {path}") from error


def validate_release(release):
    release = Path(release).resolve()
    setup = release / "install" / "setup.bash"
    if not setup.is_file():
        raise ValueError(f"release overlay is missing: {setup}")
    manifest = read_manifest(release)
    if manifest.get("release_path") != str(release):
        raise ValueError("release manifest path does not match the selected release")
    if manifest.get("git", {}).get("dirty"):
        raise ValueError("release source was dirty when the manifest was written")
    if manifest.get("validation", {}).get("status") != "passed":
        raise ValueError("release has not passed validation")
    return release


def running_simulation_processes(rows=None):
    if rows is None:
        rows = subprocess.check_output(
            ["ps", "-eo", "pid=,args="], text=True, timeout=5
        )
    matches = []
    own_pid = os.getpid()
    for row in rows.splitlines():
        fields = row.strip().split(maxsplit=1)
        if len(fields) != 2 or not fields[0].isdigit():
            continue
        pid, command = int(fields[0]), fields[1]
        if pid == own_pid:
            continue
        if any(marker in command for marker in PROCESS_MARKERS):
            matches.append({"pid": pid, "command": command})
    return matches


def _atomic_symlink(link, target):
    link = Path(link)
    link.parent.mkdir(parents=True, exist_ok=True)
    temporary = link.with_name(f".{link.name}.tmp-{os.getpid()}")
    try:
        temporary.unlink(missing_ok=True)
        temporary.symlink_to(target)
        os.replace(temporary, link)
    finally:
        temporary.unlink(missing_ok=True)


def activate_release(sim_root, release, process_rows=None):
    running = running_simulation_processes(process_rows)
    if running:
        details = "; ".join(f"{row['pid']} {row['command']}" for row in running)
        raise RuntimeError(f"simulation processes are still active: {details}")
    sim_root = Path(sim_root).resolve()
    release = validate_release(release)
    active = sim_root / "active"
    previous = sim_root / "previous"
    old_target = active.resolve() if active.is_symlink() else None
    if old_target == release:
        return release
    if old_target is not None:
        validate_release(old_target)
        _atomic_symlink(previous, old_target)
    _atomic_symlink(active, release)
    return release


def rollback_release(sim_root, process_rows=None):
    sim_root = Path(sim_root).resolve()
    previous = sim_root / "previous"
    if not previous.is_symlink():
        raise ValueError(f"previous release is not recorded: {previous}")
    target = validate_release(previous.resolve())
    return activate_release(sim_root, target, process_rows)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    manifest = subparsers.add_parser("manifest")
    manifest.add_argument("--release", required=True)
    manifest.add_argument("--source", required=True)
    manifest.add_argument(
        "--validation-status",
        required=True,
        choices=("pending", "passed", "failed"),
    )
    manifest.add_argument("--validation-command", action="append", default=[])

    for name in ("activate", "rollback"):
        subparser = subparsers.add_parser(name)
        subparser.add_argument(
            "--sim-root", default=os.path.expanduser("~/linobot2_sim")
        )
        if name == "activate":
            subparser.add_argument("--release", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.command == "manifest":
        path = write_manifest(
            args.release,
            args.source,
            args.validation_status,
            args.validation_command,
        )
        print(path)
    elif args.command == "activate":
        print(activate_release(args.sim_root, args.release))
    else:
        print(rollback_release(args.sim_root))


if __name__ == "__main__":
    main()
