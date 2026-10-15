#!/usr/bin/env python3
"""Regressionstests ohne sudo, Paketinstallation oder Desktopstart."""

from pathlib import Path
import os
import subprocess
import tempfile
import unittest

from configure_rdp import configure, update_section
from check_rdp_ready import listener_state, wait_ready


class RdpConfigTests(unittest.TestCase):
    def test_scope_comments_and_duplicate_keys(self):
        text = "[Globals]\n; port=9999\nport=3389\nPORT=3390\n[Xorg]\nport=-1\n"
        result = update_section(text, "Globals", {"port": "tcp://127.0.0.1:3389"})
        self.assertIn("; port=9999", result)
        self.assertEqual(result.count("port=tcp://127.0.0.1:3389"), 2)
        self.assertIn("[Xorg]\nport=-1", result)

    def test_missing_and_duplicate_sections_rejected(self):
        for text in ("[Other]\n", "[Globals]\n[globals]\n"):
            with self.assertRaises(ValueError):
                update_section(text, "Globals", {"port": "3389"})

    def test_safe_config_backups_and_idempotence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xrdp = "[Globals]\nport=3389\n[Xorg]\nport=-1\n"
            sesman = "[Globals]\nListenAddress=0.0.0.0\n[Security]\nAllowRootLogin=true\n[Xorg]\nparam=Xorg\nparam=-noreset\n"
            (root / "xrdp.ini").write_text(xrdp)
            (root / "sesman.ini").write_text(sesman)
            configure(root)
            result = [(root / f).read_text() for f in ("xrdp.ini", "sesman.ini")]
            self.assertIn("security_layer=tls", result[0])
            self.assertIn("port=tcp://127.0.0.1:3389", result[0])
            self.assertIn("ListenAddress=127.0.0.1", result[1])
            self.assertIn("AllowRootLogin=false", result[1])
            self.assertIn("AlwaysGroupCheck=true", result[1])
            self.assertIn("param=Xorg\nparam=-noreset", result[1])
            self.assertEqual((root / "xrdp.ini.linobot2-backup").read_text(), xrdp)
            configure(root)
            self.assertEqual(result, [(root / f).read_text() for f in ("xrdp.ini", "sesman.ini")])

    def test_validate_both_files_before_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            original = "[Globals]\nport=3389\n[Xorg]\nport=-1\n"
            (root / "xrdp.ini").write_text(original)
            (root / "sesman.ini").write_text("[Globals]\n")
            with self.assertRaises(ValueError):
                configure(root)
            self.assertEqual((root / "xrdp.ini").read_text(), original)
            self.assertFalse((root / "xrdp.ini.linobot2-backup").exists())


class RdpReadinessTests(unittest.TestCase):
    RDP = "LISTEN 0 2 127.0.0.1:3389 0.0.0.0:*\n"
    SESMAN = "LISTEN 0 2 [::1]:3350 [::]:*\n"

    def test_ipv4_and_ipv6_loopback(self):
        self.assertTrue(listener_state(self.RDP + self.SESMAN))
        self.assertTrue(listener_state(self.RDP + self.SESMAN.replace("[::1]", "127.0.0.1")))
        self.assertFalse(listener_state(self.SESMAN))
        self.assertFalse(listener_state(self.RDP))
        self.assertFalse(listener_state(""))

    def test_public_and_wildcard_bindings_rejected(self):
        for host in ("0.0.0.0", "[::]", "*", "192.168.139.46", "[fd07:b51a::1]"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                listener_state(self.RDP + self.SESMAN.replace("[::1]", host))
        with self.assertRaises(ValueError):
            listener_state("unparseable")

    def test_waits_for_delayed_listener(self):
        snapshots = iter([self.SESMAN, self.RDP + self.SESMAN])
        rows = wait_ready(read_rows=lambda: next(snapshots), pause=lambda _: None)
        self.assertEqual(rows, self.RDP + self.SESMAN)

    def test_bounded_timeout_without_rdp(self):
        ticks = iter([0, 1, 16])
        with self.assertRaises(TimeoutError):
            wait_ready(read_rows=lambda: self.SESMAN, clock=lambda: next(ticks),
                       pause=lambda _: None)

    def test_unsafe_listener_rejected_without_waiting(self):
        with self.assertRaises(ValueError):
            wait_ready(read_rows=lambda: self.RDP.replace("127.0.0.1", "0.0.0.0"),
                       pause=lambda _: self.fail("Unsicheren Listener nicht abwarten"))


class RdpSessionTests(unittest.TestCase):
    def test_local_display_and_render_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            commands = {
                "dbus-run-session": '#!/bin/sh\nshift\nexec "$@"\n',
                "xfconf-query": '#!/bin/sh\nprintf "xfconf %s\\n" "$*" >> "$RDP_TRACE"\n',
                "setxkbmap": '#!/bin/sh\nprintf "xkb %s\\n" "$*" >> "$RDP_TRACE"\n',
                "startxfce4": '#!/bin/sh\nprintf "%s|%s|%s|%s|%s\\n" "$DISPLAY" "$LIBGL_ALWAYS_SOFTWARE" "${LIBGL_ALWAYS_INDIRECT-unset}" "${MESA_GL_VERSION_OVERRIDE-unset}" "${DBUS_SESSION_BUS_ADDRESS-unset}"\n',
            }
            for name, content in commands.items():
                path = root / name
                path.write_text(content)
                path.chmod(0o755)
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ["PATH"],
                       DISPLAY=":10.0", LIBGL_ALWAYS_INDIRECT="1",
                       MESA_GL_VERSION_OVERRIDE="4.6", DBUS_SESSION_BUS_ADDRESS="stale",
                       RDP_TRACE=str(root / "commands.log"))
            result = subprocess.run(
                ["bash", str(Path(__file__).with_name("rdp_session.sh"))],
                env=env, text=True, capture_output=True, check=True,
            )
            self.assertEqual(result.stdout.strip(), ":10.0|1|unset|unset|unset")
            trace = (root / "commands.log").read_text()
            self.assertIn("/Default/XkbDisable --create -t bool -s false", trace)
            self.assertIn("/Default/XkbLayout --create -t string -s de", trace)
            self.assertIn("/Default/XkbModel --create -t string -s pc105", trace)
            self.assertIn("xkb -model pc105 -layout de -variant", trace)

    def test_mac_display_rejected_before_desktop_start(self):
        env = dict(os.environ, DISPLAY="/var/run/com.apple.launchd.test/org.xquartz:0")
        result = subprocess.run(
            ["bash", str(Path(__file__).with_name("rdp_session.sh"))],
            env=env, text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("Kein lokales RDP-X11-Display", result.stderr)


if __name__ == "__main__":
    unittest.main()