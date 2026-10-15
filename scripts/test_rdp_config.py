#!/usr/bin/env python3
"""Regressionstests ohne sudo, Paketinstallation oder Desktopstart."""

from pathlib import Path
import os
import platform
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


ON_DARWIN = platform.system() == "Darwin"
BRIDGE = str(Path(__file__).with_name("configure_rdp_bridge.sh"))
TUNNEL = str(Path(__file__).with_name("connect_rdp_mac.sh"))
MARKER = "# X-Linorobot2-Managed: true"


def _stub(root, name, body):
    path = root / name
    path.write_text(body)
    path.chmod(0o755)


class RdpBridgeUnitTests(unittest.TestCase):
    def _units(self):
        result = subprocess.run(
            ["bash", BRIDGE, "--print-units"], text=True, capture_output=True,
            check=True,
        )
        socket_unit, service_unit = result.stdout.split("### ")[1:]
        return socket_unit, service_unit

    def test_socket_has_no_device_dependency(self):
        socket_unit, _ = self._units()
        self.assertIn("ListenStream=0.0.0.0:13389", socket_unit)
        self.assertIn("ListenStream=[::]:13389", socket_unit)
        self.assertIn("WantedBy=sockets.target", socket_unit)
        self.assertIn(MARKER, socket_unit)
        # BindToDevice= erzeugt eine Abhaengigkeit auf ein .device-Unit. OrbStack
        # aktiviert solche Units nicht, der Socket scheiterte damit beim Boot.
        self.assertNotIn("BindToDevice", socket_unit)

    def test_service_proxies_to_loopback_xrdp_without_private_network(self):
        _, service_unit = self._units()
        self.assertIn("systemd-socket-proxyd 127.0.0.1:3389", service_unit)
        self.assertIn("After=linorobot2-rdp-bridge.socket xrdp.service", service_unit)
        self.assertIn(MARKER, service_unit)
        # PrivateNetwork wuerde das Loopback des Hosts verdecken; dann liefe die
        # Weiterleitung ins leere Namespace-Loopback statt zu xrdp.
        self.assertNotIn("PrivateNetwork", service_unit)
        self.assertIn("DynamicUser=yes", service_unit)
        self.assertIn("CapabilityBoundingSet=\n", service_unit)

    def test_bridge_never_exposes_xrdp_itself(self):
        socket_unit, service_unit = self._units()
        self.assertNotIn("0.0.0.0:3389", socket_unit + service_unit)

    def test_usage_rejects_unknown_and_multiple_arguments(self):
        for arguments in ([], ["--enable", "--disable"], ["--bogus"]):
            result = subprocess.run(["bash", BRIDGE, *arguments], text=True,
                                    capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("Usage:", result.stderr)

    @unittest.skipUnless(ON_DARWIN, "Der Mac-Guard ist nur dort pruefbar")
    def test_enable_refused_on_mac(self):
        result = subprocess.run(["bash", BRIDGE, "--enable"], text=True,
                                capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertIn("gehoert in die VM", result.stderr)


class RdpLanExposureGuardTests(unittest.TestCase):
    """OrbStack, nicht die Unit, setzt die Aussengrenze der Bruecke."""

    def _check(self, mac_body):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            if mac_body is not None:
                _stub(root, "mac", mac_body)
            environment = dict(os.environ, PATH=f"{root}:/usr/bin:/bin")
            return subprocess.run(
                ["bash", BRIDGE, "--check-exposure"], text=True,
                capture_output=True, env=environment,
            )

    def test_lan_exposed_host_blocks_the_bridge(self):
        result = self._check('#!/bin/sh\necho true\n')
        self.assertEqual(result.returncode, 1)
        self.assertIn("LAN-weit erreichbar", result.stderr)
        self.assertIn("orb config set machines.expose_ports_to_lan false",
                      result.stderr)

    def test_loopback_only_host_is_accepted(self):
        result = self._check('#!/bin/sh\necho false\n')
        self.assertEqual(result.returncode, 0)
        self.assertIn("bindet nur Loopback", result.stdout)

    def test_unreadable_host_setting_warns_without_claiming_safety(self):
        # Ein nicht ermittelbarer Hostzustand darf weder als sicher gelten noch
        # die Einrichtung blockieren; er muss sichtbar bleiben.
        for body in (None, '#!/bin/sh\nexit 1\n'):
            result = self._check(body)
            self.assertEqual(result.returncode, 0)
            self.assertIn("WARN", result.stderr)
            self.assertNotIn("bindet nur Loopback", result.stdout)

    def test_setup_enables_the_bridge_instead_of_only_printing_a_hint(self):
        setup = Path(__file__).with_name("setup_rdp_env.sh").read_text()
        self.assertIn("configure_rdp_bridge.sh\" --enable", setup)


class RdpTunnelFallbackTests(unittest.TestCase):
    @unittest.skipUnless(ON_DARWIN, "Der Tunnel laeuft nur auf dem Mac")
    def test_existing_listener_prevents_second_tunnel(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _stub(root, "lsof", '#!/bin/sh\necho 4242\n')
            _stub(root, "ssh", '#!/bin/sh\necho started > "$RDP_TRACE"\n')
            result = subprocess.run(
                ["bash", TUNNEL], text=True, capture_output=True,
                env=dict(os.environ, PATH=str(root) + os.pathsep + os.environ["PATH"],
                         RDP_TRACE=str(root / "ssh.log")),
            )
            self.assertEqual(result.returncode, 0)
            self.assertIn("läuft bereits (PID 4242)", result.stdout)
            self.assertFalse((root / "ssh.log").exists())

    @unittest.skipUnless(ON_DARWIN, "Der Tunnel laeuft nur auf dem Mac")
    def test_tunnel_forwards_loopback_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _stub(root, "lsof", '#!/bin/sh\nexit 1\n')
            _stub(root, "ssh", '#!/bin/sh\nprintf "%s\\n" "$*"\n')
            result = subprocess.run(
                ["bash", TUNNEL], text=True, capture_output=True, check=True,
                env=dict(os.environ, PATH=str(root) + os.pathsep + os.environ["PATH"],
                         RDP_TUNNEL_PORT="13390"),
            )
            self.assertIn("-L 127.0.0.1:13390:127.0.0.1:3389", result.stdout)
            self.assertIn("-L [::1]:13390:127.0.0.1:3389", result.stdout)
            self.assertIn("-o StrictHostKeyChecking=yes", result.stdout)
            self.assertIn("-o ExitOnForwardFailure=yes", result.stdout)
            self.assertIn("toto@ros-humble@orb", result.stdout)
            self.assertNotIn("0.0.0.0", result.stdout)


if __name__ == "__main__":
    unittest.main()