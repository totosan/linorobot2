#!/usr/bin/env python3
"""Bounded readiness check: require an actual Gazebo world-service response."""

import argparse
import os
import re
import subprocess
import threading
import time


def wait_for_world(world, pid, timeout):
    deadline = time.monotonic() + timeout
    tick = threading.Event()
    pattern = re.compile(r'\bdata:\s*"' + re.escape(world) + r'"')
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            print("FAIL: Simulator-Supervisor vor Readiness beendet.", flush=True)
            return False
        try:
            response = subprocess.run(
                ['ign', 'service', '-s', '/gazebo/worlds',
                 '--reqtype', 'ignition.msgs.Empty',
                 '--reptype', 'ignition.msgs.StringMsg_V',
                 '--timeout', '1000', '--req', ''],
                capture_output=True, text=True,
                timeout=min(3.0, max(0.01, deadline - time.monotonic())))
            if response.returncode == 0 and pattern.search(response.stdout):
                print(f"PASS: Fortress-World-Service antwortet mit {world}.", flush=True)
                return True
        except subprocess.TimeoutExpired:
            pass
        tick.wait(min(0.2, max(0, deadline - time.monotonic())))
    print(f"FAIL: keine World-Service-Antwort für {world} innerhalb {timeout:g}s.", flush=True)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--world', default='fortress_test')
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()
    if args.pid <= 0 or args.timeout <= 0:
        parser.error('PID und Timeout müssen positiv sein')
    return 0 if wait_for_world(args.world, args.pid, args.timeout) else 1


if __name__ == '__main__':
    raise SystemExit(main())