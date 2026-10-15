"""Local regression tests: python3 scripts/test_fortress_readiness.py.

No Gazebo, processes, VM, or real waits: all readiness I/O is mocked.
"""

import contextlib
import importlib.util
import io
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    "fortress_readiness_under_test", Path(__file__).with_name("wait_for_fortress.py"))
readiness = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(readiness)


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.run = mock.Mock(return_value=SimpleNamespace(returncode=0, stdout=""))
        self.kill = mock.Mock()
        self.tick = mock.Mock()
        self.tick.wait.side_effect = self.advance
        patches = (
            mock.patch.object(readiness, "time", SimpleNamespace(monotonic=lambda: self.now)),
            mock.patch.object(readiness, "os", SimpleNamespace(kill=self.kill)),
            mock.patch.object(readiness, "subprocess", SimpleNamespace(
                run=self.run, TimeoutExpired=subprocess.TimeoutExpired)),
            mock.patch.object(readiness, "threading", SimpleNamespace(Event=lambda: self.tick)),
        )
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        self.output = io.StringIO()
        redirect = contextlib.redirect_stdout(self.output)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)

    def advance(self, seconds):
        self.now += seconds

    def test_matching_world_requires_actual_world_service_response(self):
        self.run.return_value = SimpleNamespace(
            returncode=0, stdout='data: "another_world"\ndata: "fortress_test"\n')
        self.assertTrue(readiness.wait_for_world("fortress_test", 123, 5))
        self.kill.assert_called_once_with(123, 0)
        self.run.assert_called_once_with(
            ['ign', 'service', '-s', '/gazebo/worlds',
             '--reqtype', 'ignition.msgs.Empty',
             '--reptype', 'ignition.msgs.StringMsg_V',
             '--timeout', '1000', '--req', ''],
            capture_output=True, text=True, timeout=3.0)
        self.tick.wait.assert_not_called()
        self.assertIn("PASS:", self.output.getvalue())

    def test_wrong_partial_and_unstructured_world_names_are_rejected(self):
        for stdout in ('data: "wrong_world"', 'data: "fortress_test_extra"',
                       'data: "prefix_fortress_test"', 'fortress_test',
                       'metadata: "fortress_test"', ''):
            with self.subTest(stdout=stdout):
                self.now = 0
                self.run.return_value = SimpleNamespace(returncode=0, stdout=stdout)
                self.assertFalse(readiness.wait_for_world("fortress_test", 123, 0.4))
                self.assertGreaterEqual(self.now, 0.4)

    def test_world_name_regex_metacharacters_are_literal(self):
        self.run.return_value = SimpleNamespace(returncode=0, stdout='data: "worldX1"')
        self.assertFalse(readiness.wait_for_world("world.1", 123, 0.2))
        self.now = 0
        self.run.return_value = SimpleNamespace(returncode=0, stdout='data:   "world.1"')
        self.assertTrue(readiness.wait_for_world("world.1", 123, 0.2))

    def test_nonzero_exit_rejected_even_with_matching_stdout(self):
        for code in (1, 7, -15):
            with self.subTest(returncode=code):
                self.now = 0
                self.run.return_value = SimpleNamespace(
                    returncode=code, stdout='data: "fortress_test"')
                self.assertFalse(readiness.wait_for_world("fortress_test", 123, 0.4))

    def test_wrong_world_is_retried_until_correct_response(self):
        self.run.side_effect = [
            SimpleNamespace(returncode=0, stdout='data: "wrong_world"'),
            SimpleNamespace(returncode=0, stdout='data: "fortress_test"'),
        ]
        self.assertTrue(readiness.wait_for_world("fortress_test", 123, 1))
        self.assertEqual(self.run.call_count, 2)
        self.tick.wait.assert_called_once_with(0.2)

    def test_command_timeout_is_bounded_by_remaining_deadline_and_cap(self):
        def expire(*args, **kwargs):
            self.advance(kwargs["timeout"])
            raise subprocess.TimeoutExpired(args[0], kwargs["timeout"])

        self.run.side_effect = expire
        for duration, expected in ((0.5, [0.5]), (4.0, [3.0, 0.8])):
            with self.subTest(duration=duration):
                self.now = 0
                self.run.reset_mock()
                self.assertFalse(readiness.wait_for_world("fortress_test", 123, duration))
                bounds = [call.kwargs["timeout"] for call in self.run.call_args_list]
                self.assertEqual(len(bounds), len(expected))
                for actual, wanted in zip(bounds, expected):
                    self.assertAlmostEqual(actual, wanted)
                    self.assertGreater(actual, 0)
                    self.assertLessEqual(actual, 3)
                self.assertAlmostEqual(self.now, duration)

    def test_timeout_expired_can_recover_on_next_response(self):
        self.run.side_effect = [
            subprocess.TimeoutExpired("ign", 0.1),
            SimpleNamespace(returncode=0, stdout='data: "fortress_test"'),
        ]
        self.assertTrue(readiness.wait_for_world("fortress_test", 123, 1))
        self.assertEqual(self.run.call_count, 2)

    def test_supervisor_exit_prevents_service_call(self):
        self.kill.side_effect = ProcessLookupError
        self.assertFalse(readiness.wait_for_world("fortress_test", 123, 1))
        self.run.assert_not_called()
        self.tick.wait.assert_not_called()
        self.assertIn("Supervisor", self.output.getvalue())

    def test_supervisor_exit_during_retry_stops_polling(self):
        self.kill.side_effect = [None, ProcessLookupError]
        self.assertFalse(readiness.wait_for_world("fortress_test", 123, 1))
        self.assertEqual(self.run.call_count, 1)


class ReadinessCliTests(unittest.TestCase):
    def test_invalid_pid_timeout_and_missing_pid_are_argparse_errors(self):
        for args in ([], ["--pid", "0"], ["--pid", "-1"],
                     ["--pid", "123", "--timeout", "0"],
                     ["--pid", "123", "--timeout", "-1"]):
            with self.subTest(args=args), mock.patch.object(sys, "argv", ["probe"] + args), \
                    mock.patch.object(readiness, "wait_for_world") as wait, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    readiness.main()
                self.assertEqual(error.exception.code, 2)
                wait.assert_not_called()

    def test_cli_defaults_custom_arguments_and_exit_status(self):
        for args, expected, ready in (
            (["--pid", "123"], ("fortress_test", 123, 45.0), True),
            (["--pid", "123", "--world", "custom", "--timeout", "2.5"],
             ("custom", 123, 2.5), False),
        ):
            with self.subTest(args=args), mock.patch.object(sys, "argv", ["probe"] + args), \
                    mock.patch.object(readiness, "wait_for_world", return_value=ready) as wait:
                self.assertEqual(readiness.main(), 0 if ready else 1)
                wait.assert_called_once_with(*expected)


if __name__ == "__main__":
    unittest.main()