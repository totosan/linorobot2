#!/usr/bin/env python3
"""Wartet begrenzt auf sichere lokale RDP-Listener."""

import ipaddress
import subprocess
import threading
import time


def listener_state(rows):
    endpoints = set()
    for line in rows.splitlines():
        fields = line.split()
        if len(fields) < 5:
            raise ValueError(f"Ungültige ss-Ausgabe: {line}")
        host, port = fields[3].rsplit(":", 1)
        host = host.strip("[]")
        try:
            address = ipaddress.ip_address(host)
        except ValueError as error:
            raise ValueError(f"Unsicherer RDP-Listener: {fields[3]}") from error
        if not address.is_loopback or port not in ("3389", "3350"):
            raise ValueError(f"Unsicherer RDP-Listener: {fields[3]}")
        endpoints.add((host, port))
    # Der Client verwendet IPv4; sesman darf auf IPv4 ODER IPv6-Loopback lauschen.
    return ("127.0.0.1", "3389") in endpoints and any(port == "3350" for _, port in endpoints)


def read_listeners():
    return subprocess.check_output(
        ["ss", "-H", "-ltn", "( sport = :3389 or sport = :3350 )"],
        text=True, timeout=3,
    )


def wait_ready(timeout=15, read_rows=read_listeners, clock=time.monotonic,
               pause=None):
    pause = pause or threading.Event().wait
    deadline = clock() + timeout
    last_rows = ""
    while True:
        last_rows = read_rows()
        if listener_state(last_rows):
            return last_rows
        remaining = deadline - clock()
        if remaining <= 0:
            raise TimeoutError(f"RDP nach {timeout}s nicht bereit. Listener:\n{last_rows or '(keine)'}")
        pause(min(0.2, remaining))


if __name__ == "__main__":
    try:
        print(wait_ready(), end="")
        print("[OK] RDP erreichbar; RDP und sesman ausschließlich auf Loopback.")
    except (ValueError, TimeoutError, OSError, subprocess.SubprocessError) as error:
        raise SystemExit(f"[FEHLER] {error}") from error