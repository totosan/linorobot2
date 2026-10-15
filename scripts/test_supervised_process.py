"""Local: python3 -m unittest discover -s scripts -p test_supervised_process.py

Linux integration tests use real processes; on macOS only unit tests run.
No VM, ROS, dependencies, elevated privileges or external services required.
"""

import contextlib
import importlib.util
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


SCRIPT = Path(__file__).with_name("supervised_process.py")
SPEC = importlib.util.spec_from_file_location("supervised_process", SCRIPT)
supervisor = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = supervisor
SPEC.loader.exec_module(supervisor)

# Launch -> intermediate child -> detached grandchild. The intermediate exits,
# leaving the grandchild adopted by the supervisor even if launch stays alive.
TREE = r'''
import os, signal, sys, threading
path, mode = sys.argv[1:]
ready_read, ready_write = os.pipe()
middle = os.fork()
if middle == 0:
    os.close(ready_read)
    detached = os.fork()
    if detached == 0:
        os.setsid()
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        with open(path, "w") as stream:
            stream.write(str(os.getpid()))
        os.write(ready_write, b"1")
        os.close(ready_write)
        threading.Event().wait()
    os._exit(0)
os.close(ready_write)
assert os.read(ready_read, 1) == b"1"
os.close(ready_read)
os.waitpid(middle, 0)
if mode == "wait":
    threading.Event().wait()
if mode == "signal":
    os.kill(os.getpid(), signal.SIGKILL)
sys.exit(int(mode))
'''


class UnitTests(unittest.TestCase):
    def test_stat_with_tricky_command(self):
        fields = ["S", "10"] + ["0"] * 17 + ["12345"]
        process = supervisor.parse_stat("42 (name ) with spaces)) " + " ".join(fields))
        self.assertEqual(process, supervisor.Process(42, 10, "S", 12345))
        self.assertEqual(process.identity, (42, 12345))

    def test_zombies_not_alive(self):
        self.assertFalse(supervisor.Process(42, 10, "Z", 1).alive)
        self.assertTrue(supervisor.Process(42, 10, "D", 1).alive)

    def test_descendants_excludes_self_and_external_tree(self):
        table = {p.pid: p for p in (
            supervisor.Process(10, 1, "S", 1),
            supervisor.Process(11, 10, "S", 2),
            supervisor.Process(12, 11, "Z", 3),
            supervisor.Process(13, 10, "S", 4),
            supervisor.Process(20, 1, "S", 5),
            supervisor.Process(21, 20, "S", 6))}
        self.assertEqual(set(supervisor.descendants(table, 10)), {11, 12, 13})

    def test_exitcode_normalization(self):
        self.assertEqual(supervisor.normalized_exitcode(0), 0)
        self.assertEqual(supervisor.normalized_exitcode(7), 7)
        self.assertEqual(supervisor.normalized_exitcode(-15), 143)

    def test_cli_requires_separator_and_command(self):
        with contextlib.redirect_stderr(io.StringIO()):
            for args in ([], ["--"], ["echo"]):
                self.assertEqual(supervisor.main(args), 2)

    def test_nonlinux_fails_explicitly(self):
        with mock.patch.object(supervisor.sys, "platform", "darwin"):
            with self.assertRaisesRegex(RuntimeError, "Linux"):
                supervisor.enable_subreaper()

    def test_missing_proc_fails_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(supervisor.sys, "platform", "linux"), \
                    mock.patch.object(supervisor, "PROC", Path(directory) / "missing"):
                with self.assertRaisesRegex(RuntimeError, "/proc"):
                    supervisor.enable_subreaper()

    def test_pid_reuse_and_zombies_are_not_signaled(self):
        original = supervisor.Process(123456, os.getpid(), "S", 1)
        for current in (None, supervisor.Process(123456, os.getpid(), "S", 2),
                        supervisor.Process(123456, os.getpid(), "Z", 1)):
            with mock.patch.object(supervisor.os, "pidfd_open", return_value=99, create=True), \
                    mock.patch.object(supervisor.os, "close"), \
                    mock.patch.object(supervisor, "read_process", return_value=current), \
                    mock.patch.object(supervisor.os, "kill") as kill, \
                    mock.patch.object(supervisor.signal, "pidfd_send_signal", create=True) as send:
                supervisor.signal_process(original, signal.SIGTERM)
                kill.assert_not_called()
                send.assert_not_called()

    def test_self_is_never_signaled(self):
        with mock.patch.object(supervisor.os, "kill") as kill:
            supervisor.signal_process(supervisor.Process(os.getpid(), 1, "S", 1), signal.SIGKILL)
            kill.assert_not_called()

    def test_recycled_ancestor_cannot_attach_unrelated_child(self):
        parent = supervisor.Process(123455, os.getpid(), "S", 1)
        child = supervisor.Process(123456, parent.pid, "S", 2)
        recycled = supervisor.Process(parent.pid, 1, "S", 3)
        with mock.patch.object(supervisor.os, "pidfd_open", return_value=99, create=True), \
                mock.patch.object(supervisor.os, "close"), \
                mock.patch.object(supervisor, "read_process", side_effect=[child, recycled]), \
                mock.patch.object(supervisor.os, "kill") as kill, \
                mock.patch.object(supervisor.signal, "pidfd_send_signal", create=True) as send:
            self.assertFalse(supervisor.signal_process(
                child, signal.SIGTERM, {parent.pid: parent, child.pid: child}))
            kill.assert_not_called()
            send.assert_not_called()

    def test_reap_uses_nonblocking_wait_for_adopted_zombies(self):
        zombie = supervisor.Process(123456, os.getpid(), "Z", 1)
        child = mock.Mock(pid=123455)
        with mock.patch.object(supervisor, "read_process", return_value=zombie), \
                mock.patch.object(supervisor.os, "waitpid", return_value=(zombie.pid, 0)) as wait:
            supervisor.reap(child, {zombie.pid: zombie})
            wait.assert_called_once_with(zombie.pid, os.WNOHANG)
            child.poll.assert_called_once()

    def test_cleanup_rescans_forks_during_term_and_kill(self):
        root = os.getpid()
        first = supervisor.Process(123451, root, "S", 1)
        second = supervisor.Process(123452, root, "S", 2)
        third = supervisor.Process(123453, root, "S", 3)
        live = {first.pid: first}
        sent = []

        def send(process, signum, owned):
            sent.append((process.pid, signum))
            if signum == signal.SIGTERM and process.pid == first.pid:
                live[second.pid] = second
            if signum == signal.SIGKILL:
                live.pop(process.pid, None)
                if process.pid == second.pid:
                    live[third.pid] = third
            return True

        child = mock.Mock()
        child.poll.return_value = 0
        with mock.patch.object(supervisor, "process_table", side_effect=lambda: dict(live)), \
                mock.patch.object(supervisor, "reap"), \
                mock.patch.object(supervisor, "signal_process", side_effect=send), \
                mock.patch.object(supervisor.threading, "Event"), \
                mock.patch.object(supervisor.time, "monotonic", side_effect=range(100)), \
                mock.patch.object(supervisor, "KILL_GRACE", 10):
            self.assertTrue(supervisor.cleanup(child))
        self.assertIn((second.pid, signal.SIGTERM), sent)
        for process in (first, second, third):
            self.assertIn((process.pid, signal.SIGKILL), sent)
        self.assertFalse(live)

    def test_cleanup_deadline_reports_failure(self):
        stuck = supervisor.Process(123456, os.getpid(), "D", 1)
        child = mock.Mock(pid=stuck.pid)
        child.poll.return_value = None
        with mock.patch.object(supervisor, "process_table", return_value={stuck.pid: stuck}), \
                mock.patch.object(supervisor, "reap"), \
                mock.patch.object(supervisor, "signal_process", return_value=True) as send, \
                mock.patch.object(supervisor.threading, "Event"), \
                mock.patch.object(supervisor.time, "monotonic", side_effect=range(100)), \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertFalse(supervisor.cleanup(child))
        self.assertIn("deadline exceeded", errors.getvalue())
        self.assertEqual([call.args[1] for call in send.call_args_list],
                         [signal.SIGTERM, signal.SIGKILL])

    def test_failed_cleanup_is_not_retried(self):
        child = mock.Mock(returncode=None)
        child.poll.return_value = None

        def install(signum, handler):
            if callable(handler):
                handler(signum, None)
            return signal.SIG_DFL

        with mock.patch.object(supervisor, "enable_subreaper"), \
                mock.patch.object(supervisor.signal, "signal", side_effect=install), \
                mock.patch.object(supervisor.subprocess, "Popen", return_value=child) as launch, \
                mock.patch.object(supervisor, "cleanup", return_value=False) as clean:
            self.assertEqual(supervisor.supervise(["example"]), 1)
            launch.assert_called_once_with(["example"], start_new_session=True)
            clean.assert_called_once_with(child)


@unittest.skipUnless(sys.platform == "linux", "requires Linux /proc and subreaper")
class LinuxIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.pid_path = Path(self.directory.name) / "detached.pid"
        self.launched = []
        self.detached = None
        self.addCleanup(self.emergency_cleanup)

    def emergency_cleanup(self):
        # Test-failure containment: only exact processes created by this test.
        if self.detached is not None:
            supervisor.signal_process(self.detached, signal.SIGKILL)
        for process in self.launched:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=3)

    def launch(self, mode, inherited_ignore=False):
        command = [sys.executable, str(SCRIPT), "--", sys.executable,
                   "-c", TREE, str(self.pid_path), mode]
        if inherited_ignore:
            # Equivalent to a shell background job inheriting SIGINT=SIG_IGN.
            wrapper = "import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_IGN); os.execv(sys.argv[1],sys.argv[1:])"
            command = [sys.executable, "-c", wrapper] + command
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE, text=True)
        self.launched.append(process)
        deadline = time.monotonic() + 4
        tick = threading.Event()
        while time.monotonic() < deadline:
            if self.pid_path.exists():
                text = self.pid_path.read_text().strip()
                if text:
                    self.detached = supervisor.read_process(int(text))
                    if self.detached is not None:
                        return process
            if process.poll() is not None:
                self.fail("supervisor exited before readiness: " + process.communicate()[1])
            tick.wait(0.02)
        self.fail("detached grandchild readiness deadline exceeded")

    def assert_finished(self, process, expected):
        start = time.monotonic()
        _, errors = process.communicate(timeout=10)
        self.assertLess(time.monotonic() - start, 9)
        self.assertEqual(process.returncode, expected, errors)
        # Absence, not just zombie state: adoption and reaping must finish too.
        current = supervisor.read_process(self.detached.pid)
        self.assertTrue(current is None or current.identity != self.detached.identity,
                        "detached grandchild remains in /proc")

    def test_normal_success_cleans_detached_grandchild_and_preserves_sentinel(self):
        sentinel = subprocess.Popen([sys.executable, "-c",
                                     "import threading; threading.Event().wait()"],
                                    start_new_session=True)
        self.launched.append(sentinel)
        process = self.launch("0")
        self.assert_finished(process, 0)
        self.assertIsNone(sentinel.poll(), "unrelated sentinel was killed")

    def test_normal_failure_cleans_detached_grandchild(self):
        self.assert_finished(self.launch("7"), 7)

    def test_child_signal_exit_is_normalized(self):
        self.assert_finished(self.launch("signal"), 128 + signal.SIGKILL)

    def test_sigterm_cleans_detached_grandchild(self):
        process = self.launch("wait")
        process.send_signal(signal.SIGTERM)
        self.assert_finished(process, 128 + signal.SIGTERM)

    def test_sigint_overrides_inherited_ignore(self):
        process = self.launch("wait", inherited_ignore=True)
        process.send_signal(signal.SIGINT)
        self.assert_finished(process, 128 + signal.SIGINT)


if __name__ == "__main__":
    unittest.main()