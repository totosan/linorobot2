#!/usr/bin/env python3
"""Linux-only, stdlib-only supervisor: python3 supervised_process.py -- COMMAND...

Run as a dedicated process, not inside an application with other children.
stdio is inherited. Cleanup allows 5 seconds for TERM and 2 seconds for KILL
and reaping; unkillable descendants cause a nonzero result rather than a hang.
"""

import ctypes
from dataclasses import dataclass
import errno
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


POLL_INTERVAL = 0.05
TERM_GRACE = 5.0
KILL_GRACE = 2.0
PR_SET_CHILD_SUBREAPER = 36
PROC = Path("/proc")


@dataclass(frozen=True)
class Process:
    pid: int
    ppid: int
    state: str
    starttime: int

    @property
    def identity(self):
        return self.pid, self.starttime

    @property
    def alive(self):
        return self.state not in ("Z", "X", "x")


def parse_stat(text):
    """Parse /proc/PID/stat, including commands containing spaces or ')'."""
    pid_text, separator, rest = text.partition(" (")
    end = rest.rfind(")")
    if not separator or end < 0:
        raise ValueError("malformed /proc stat")
    fields = rest[end + 1:].split()
    # fields[0] is field 3 (state); starttime is field 22.
    return Process(int(pid_text), int(fields[1]), fields[0], int(fields[19]))


def read_process(pid):
    try:
        return parse_stat((PROC / str(pid) / "stat").read_text())
    except (FileNotFoundError, ProcessLookupError):
        return None


def process_table():
    result = {}
    for entry in PROC.iterdir():
        if entry.name.isdecimal():
            process = read_process(int(entry.name))
            if process is not None:
                result[process.pid] = process
    return result


def descendants(table, root):
    """Only PPid-connected descendants; sessions/process groups are irrelevant."""
    children = {}
    for process in table.values():
        children.setdefault(process.ppid, []).append(process)
    result = {}
    pending = [root]
    visited = {root}
    while pending:
        for process in children.get(pending.pop(), ()):
            if process.pid not in visited:
                visited.add(process.pid)
                result[process.pid] = process
                pending.append(process.pid)
    return result


def enable_subreaper():
    if sys.platform != "linux":
        raise RuntimeError("requires Linux and a readable /proc filesystem")
    try:
        own = read_process(os.getpid())
        if own is None or own.pid != os.getpid():
            raise RuntimeError("/proc does not describe this PID namespace")
        process_table()
    except (OSError, ValueError) as exc:
        raise RuntimeError("cannot inspect /proc: {}".format(exc)) from exc
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong,
                           ctypes.c_ulong, ctypes.c_ulong]
    libc.prctl.restype = ctypes.c_int
    if libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise RuntimeError("PR_SET_CHILD_SUBREAPER failed: {}".format(os.strerror(error)))


def signal_process(process, signum, owned=None):
    """Verify identity immediately before signaling; pidfds close the reuse race."""
    if process.pid == os.getpid() or not process.alive:
        return False
    fd = None
    try:
        if hasattr(os, "pidfd_open") and hasattr(signal, "pidfd_send_signal"):
            try:
                fd = os.pidfd_open(process.pid, 0)
            except OSError as exc:
                if exc.errno not in (errno.ENOSYS, errno.EINVAL):
                    raise
        current = read_process(process.pid)
        if current is None or current.identity != process.identity or not current.alive:
            return False
        if owned is not None:
            # /proc is not an atomic snapshot. Recheck the ancestry identities
            # so a recycled intermediate PID cannot attach an unrelated tree.
            ancestor = current
            seen = {current.pid}
            while ancestor.ppid != os.getpid():
                expected = owned.get(ancestor.ppid)
                if expected is None or expected.pid in seen:
                    return False
                ancestor = read_process(expected.pid)
                if ancestor is None or ancestor.identity != expected.identity:
                    return False
                seen.add(ancestor.pid)
        if fd is not None:
            signal.pidfd_send_signal(fd, signum)
        else:
            os.kill(process.pid, signum)
        return True
    except ProcessLookupError:
        return False
    finally:
        if fd is not None:
            os.close(fd)


def reap(child, processes):
    # Let Popen own its wait status; do not steal it with waitpid(-1).
    child.poll()
    for process in processes.values():
        if process.pid == child.pid or process.ppid != os.getpid():
            continue
        current = read_process(process.pid)
        if current is None or current.identity != process.identity:
            continue
        try:
            os.waitpid(process.pid, os.WNOHANG)
        except ChildProcessError:
            pass


def cleanup(child):
    """Rescan both stages to catch forks, adoption and delayed zombie reaping."""
    tick = threading.Event()
    empty_scans = 0
    for signum, grace in ((signal.SIGTERM, TERM_GRACE), (signal.SIGKILL, KILL_GRACE)):
        deadline = time.monotonic() + grace
        signaled = set()
        while True:
            owned = descendants(process_table(), os.getpid())
            reap(child, owned)
            for process in owned.values():
                if process.alive and process.identity not in signaled:
                    if signal_process(process, signum, owned):
                        signaled.add(process.identity)
            # Reaping may have removed entries or exposed newly adopted children.
            remaining = descendants(process_table(), os.getpid())
            empty_scans = empty_scans + 1 if not remaining else 0
            if empty_scans >= 2 and child.poll() is not None:
                return True
            left = deadline - time.monotonic()
            if left <= 0:
                break
            tick.wait(min(POLL_INTERVAL, left))
    remaining = descendants(process_table(), os.getpid())
    reap(child, remaining)
    remaining = descendants(process_table(), os.getpid())
    if remaining or child.poll() is None:
        print("supervised_process: cleanup deadline exceeded; remaining PIDs: {}".format(
            ", ".join(str(pid) for pid in sorted(remaining)) or str(child.pid)),
            file=sys.stderr)
        return False
    return True


def normalized_exitcode(returncode):
    return 128 - returncode if returncode < 0 else returncode


def supervise(command):
    """Return command status, 128+interrupt, or 1 on cleanup failure.

    This entry point changes subreaper state; use it only in a dedicated process.
    """
    enable_subreaper()
    wake = threading.Event()
    interrupted = []

    def interrupt(signum, frame):
        if not interrupted:
            interrupted.append(signum)
        # No locking in a signal handler: Event.set() could interrupt this
        # thread while Event.wait() holds the same non-reentrant lock.
        # Bounded polling observes this flag within POLL_INTERVAL instead.

    previous = {}
    child = None
    cleanup_started = False
    try:
        # Explicit installation is essential when a background shell inherited
        # SIG_IGN for SIGINT. The first interrupt determines the final status.
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        child = subprocess.Popen(command, start_new_session=True)
        while child.poll() is None and not interrupted:
            wake.wait(POLL_INTERVAL)
        status = child.returncode
        cleanup_started = True
        cleaned = cleanup(child)
        if not cleaned:
            return 1
        if interrupted:
            return 128 + interrupted[0]
        return normalized_exitcode(status)
    finally:
        # Clean up if monitoring failed, but never repeat an expired deadline.
        if child is not None and not cleanup_started:
            try:
                cleanup(child)
            except (OSError, ValueError) as exc:
                print("supervised_process: cleanup failed: {}".format(exc), file=sys.stderr)
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] != "--" or len(args) == 1:
        print("usage: python3 supervised_process.py -- COMMAND...", file=sys.stderr)
        return 2
    try:
        return supervise(args[1:])
    except (OSError, RuntimeError, ValueError) as exc:
        print("supervised_process: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())