"""Local stdlib tests: python3 -m unittest discover -s scripts -p test_gui_x11_probe.py

All Xlib, /proc, clock and waiting operations are mocked. No DISPLAY server,
GUI, ROS, subreaper, VM command, subprocess or installed X11 tools are used.
"""

import ctypes as C
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import gui_x11_probe as probe
from supervised_process import Process


def write_pointer(pointer, ctype, value):
    C.cast(pointer, C.POINTER(ctype))[0] = value


class Clock:
    def __init__(self):
        self.now = 100.0
        self.waits = []

    def monotonic(self):
        self.now += 0.001
        return self.now

    def wait(self, duration):
        self.waits.append(duration)
        self.now += duration


class FakeXlib:
    """Exercise real ctypes output pointers and allocations without loading X11."""
    def __init__(self):
        self.display = 0x1234
        self.root = 100
        self.atoms = {}
        self.properties = {}
        self.attributes = {}
        self.allocations = {}
        self.freed = []
        self.error_handler = None
        self.pending_error = None
        self.queue = []
        self.reply_mutations = []
        self.auto_reply = True
        self.selected_mask = 0
        self.reply_mask = 1 << 17  # Qt 5 uses StructureNotify, NOT SubstructureNotify.
        self.after_send = None
        self.after_attributes = None
        self.sent = []
        self.lib = SimpleNamespace()
        methods = {
            "XOpenDisplay": lambda name: self.display,
            "XCloseDisplay": lambda display: 0,
            "XDefaultRootWindow": lambda display: self.root,
            "XInternAtom": self.intern,
            "XSelectInput": self.select_input,
            "XSync": self.sync,
            "XFlush": lambda display: 0,
            "XFree": self.free,
            "XSetErrorHandler": self.set_handler,
            "XGetWindowProperty": self.get_property,
            "XGetWindowAttributes": self.get_attributes,
            "XSendEvent": self.send,
            "XCheckTypedEvent": self.check,
        }
        for name, implementation in methods.items():
            setattr(self.lib, name, mock.Mock(side_effect=implementation))

    def intern(self, display, name, only_if_exists):
        return self.atoms.setdefault(name.decode("ascii"), len(self.atoms) + 1000)

    def select_input(self, display, window, mask):
        self.selected_mask = mask
        return 1

    def set_handler(self, callback):
        previous = self.error_handler
        self.error_handler = callback.value if isinstance(callback, C.c_void_p) else callback
        return previous

    def sync(self, display, discard):
        if self.pending_error:
            event = probe.XErrorEvent()
            event.display = display
            event.error_code, event.resourceid = self.pending_error
            event.request_code = 3
            self.pending_error = None
            C.cast(self.error_handler, probe.XErrorHandler)(display, C.byref(event))
        return 0

    def free(self, pointer):
        address = pointer.value
        self.freed.append(address)
        del self.allocations[address]
        return 0

    def put(self, window, name, kind, fmt, values):
        self.properties[window, self.atoms[name]] = (kind, fmt, values, 0)

    def get_property(self, display, window, atom, offset, length, delete,
                     requested, actual, fmt, count, remaining, output):
        spec = self.properties.get((window, atom))
        if spec is None:
            write_pointer(actual, C.c_ulong, 0)
            return 0
        kind, format_value, values, left = spec
        write_pointer(actual, C.c_ulong, kind)
        write_pointer(fmt, C.c_int, format_value)
        write_pointer(count, C.c_ulong, len(values))
        write_pointer(remaining, C.c_ulong, left)
        if format_value == 32:
            allocation = (C.c_ulong * len(values))(*values)
        else:
            allocation = C.create_string_buffer(values)
        address = C.addressof(allocation)
        self.allocations[address] = allocation
        write_pointer(output, C.POINTER(C.c_ubyte),
                      C.cast(allocation, C.POINTER(C.c_ubyte)))
        return 0

    def get_attributes(self, display, window, output):
        spec = self.attributes.get(window)
        if spec is None:
            self.pending_error = (3, window)  # BadWindow, delivered at XSync.
            return 0
        attributes = C.cast(output, C.POINTER(probe.XWindowAttributes)).contents
        attributes.map_state, attributes.width, attributes.height = spec
        attributes.x, attributes.y = 100000, -100000  # Deliberately offscreen.
        if self.after_attributes:
            self.after_attributes()
        return 1

    def send(self, display, window, propagate, event_mask, pointer):
        event = C.cast(pointer, C.POINTER(probe.XEvent)).contents
        copy = probe.XEvent.from_buffer_copy(event)
        self.sent.append((window, propagate, event_mask, copy))
        if self.auto_reply and self.selected_mask & self.reply_mask:
            for mutate in self.reply_mutations + [lambda event: None]:
                reply = probe.XEvent.from_buffer_copy(event)
                reply.xclient.window = self.root
                reply.xclient.send_event = 1
                mutate(reply)
                self.queue.append(reply)
        if self.after_send:
            self.after_send()
        return 1

    def check(self, display, event_type, output):
        if not self.queue:
            return 0
        event = self.queue.pop(0)
        C.memmove(output, C.byref(event), C.sizeof(event))
        return 1


class HelpersTests(unittest.TestCase):
    def test_lp64_abi_including_aarch64(self):
        if C.sizeof(C.c_long) != 8 or C.sizeof(C.c_void_p) != 8:
            self.skipTest("LP64 shape assertions")
        self.assertEqual(C.sizeof(probe.XEvent), 192)
        self.assertEqual(C.sizeof(probe.XClientMessageEvent), 96)
        self.assertEqual(C.sizeof(probe.XClientMessageData), 40)
        self.assertEqual(probe.XClientMessageEvent.data.offset, 56)
        self.assertEqual(probe.XClientMessageEvent.window.offset, 32)
        self.assertEqual(C.sizeof(probe.XWindowAttributes), 136)
        self.assertEqual(probe.XWindowAttributes.map_state.offset, 92)
        self.assertEqual(C.sizeof(probe.XErrorEvent), 40)
        self.assertEqual(probe.XErrorEvent.error_code.offset, 32)

    def test_property32_uses_unsigned_long_stride_and_wire_mask(self):
        values = [7, 0xffffffff, 13]
        if C.sizeof(C.c_ulong) == 8:
            values[1] |= 0x1234567800000000
        allocation = (C.c_ulong * 3)(*values)
        self.assertEqual(probe.property32(C.cast(allocation, C.POINTER(C.c_ubyte)), 3),
                         [7, 0xffffffff, 13])
        self.assertEqual(probe.property32(None, 0), [])
        with self.assertRaisesRegex(RuntimeError, "null"):
            probe.property32(None, 1)

    def test_cmdline_exact_gui_forms(self):
        for command in (b"ign\0gazebo\0gui\0-v\0" + b"4\0",
                        b"/usr/bin/ign\0gazebo\0-g\0world.sdf\0",
                        b"ign gazebo gui\0"):
            self.assertEqual(probe.gui_kind(command), "gazebo")
        self.assertEqual(probe.gui_kind(b"/opt/ros/humble/lib/rviz2/rviz2\0-d\0x"),
                         "rviz")
        for command in (b"", b"bash\0-c\0ign gazebo gui\0",
                        b"ign\0gazebo\0-s\0", b"ign\0gazebo\0-g\0-s\0",
                        b"ign\0gazebo\0gui-helper\0", b"notign\0gazebo\0gui\0",
                        b"/tmp/rviz2\0", b"/rviz2/rviz2-helper\0",
                        b"python3\0/rviz2/rviz2\0"):
            with self.subTest(command=command):
                self.assertIsNone(probe.gui_kind(command))

    def test_actual_gui_titles_with_nul_padding_and_original_ruby_argv(self):
        titles = [b"ign gazebo -g"] + [
            b"ign gazebo -g -v " + str(version).encode() for version in range(5)]
        commands = titles + [b"ruby\0/usr/bin/ign\0gazebo\0-g\0-v\0" + b"3"]
        for command in commands:
            for padding in (b"\0", b"\0" * 128):
                with self.subTest(command=command, padding=len(padding)):
                    self.assertEqual(probe.gui_kind(command + padding), "gazebo")

    def test_actual_gui_title_rejects_nonexact_and_malformed_forms(self):
        title = b"ign gazebo -g -v 3"
        commands = [b"sh -c " + title, b"sh\0-c\0" + title,
                    b"prefix " + title, title + b" suffix", title + b"*",
                    b"*" + title, b" " + title, title + b" ", title + b"\n",
                    title + b"\0-g", title + b"\0\0-s"]
        commands += [title + operator + b"echo" for operator in
                     (b";", b"&&", b"|", b"`", b"$", b"<", b">")]
        commands += [b"ign gazebo -g" + suffix for suffix in
                     (b" -s", b" --server-only", b" -g", b" -v", b" -v ",
                      b" -v -1", b" -v 5", b" -v 03", b" -v 3.0", b" -v x",
                      b" -v *", b" -v 3 -v 3")]
        commands += [b"ign gazebo " + mode + b" -v 3" for mode in
                     (b"-s", b"-gg", b"--gui-only", b"-G")]
        for command in commands:
            with self.subTest(command=command):
                self.assertIsNone(probe.gui_kind(command + b"\0" * 128))

    def test_rewritten_title_requires_exact_single_argv_not_shell_string(self):
        for command in (b"sh -c ign gazebo gui\0", b"sh\0-c\0ign gazebo gui\0",
                        b"prefix ign gazebo gui\0", b"ign gazebo gui-helper\0",
                        b"ign gazebo gui*\0", b"*ign gazebo gui\0",
                        b" ign gazebo gui\0", b"ign gazebo gui \0",
                        b"ign gazebo gui -g\0", b"ign gazebo gui\0-g\0",
                        b"ign gazebo gui\0-s\0", b"ign gazebo server\0"):
            with self.subTest(command=command):
                self.assertIsNone(probe.gui_kind(command))


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeXlib()
        self.clock = Clock()
        self.table = {p.pid: p for p in (
            Process(10, 1, "S", 100), Process(11, 10, "S", 110),
            Process(12, 11, "S", 120), Process(13, 11, "S", 130),
            Process(20, 1, "S", 200), Process(21, 20, "S", 210))}
        self.commands = {10: b"ign\0gazebo\0gui\0", 11: b"ros2\0launch\0",
                         12: b"ign\0gazebo\0gui\0",
                         13: b"/opt/ros/humble/lib/rviz2/rviz2\0",
                         21: b"ign\0gazebo\0gui\0"}
        patches = [mock.patch.dict(os.environ, {"DISPLAY": ":10.0"}),
                   mock.patch.object(probe, "find_library", return_value="fake-X11"),
                   mock.patch.object(probe.C, "CDLL", return_value=self.fake.lib),
                   mock.patch.object(probe, "process_table", side_effect=lambda: dict(self.table)),
                   mock.patch.object(probe, "read_process", side_effect=lambda pid: self.table.get(pid)),
                   mock.patch.object(probe.X11Probe, "_cmdline",
                                     side_effect=lambda pid: self.commands[pid]),
                   mock.patch.object(probe.time, "monotonic", side_effect=self.clock.monotonic),
                   mock.patch.object(probe.threading, "Event",
                                     return_value=SimpleNamespace(wait=self.clock.wait)),
                   mock.patch.object(probe.secrets, "randbits", return_value=0x80000000)]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        self.client = probe.X11Probe()
        self.addCleanup(self.client.close)
        self.fake.put(100, "_NET_CLIENT_LIST", probe.XA_WINDOW, 32, [101, 102, 103, 104, 105])
        for window, pid in ((101, 12), (102, 13), (103, 21), (104, 11), (105, 10)):
            self.window(window, pid)

    def window(self, window, pid):
        self.fake.put(window, "_NET_WM_PID", probe.XA_CARDINAL, 32, [pid])
        self.fake.put(window, "WM_PROTOCOLS", probe.XA_ATOM, 32,
                      [self.client.atoms["_NET_WM_PING"]])
        self.fake.put(window, "_NET_WM_NAME", self.client.atoms["UTF8_STRING"],
                      8, b"arbitrary title")
        self.fake.put(window, "WM_CLASS", probe.XA_STRING, 8, b"app\0Qt\0")
        self.fake.attributes[window] = (probe.IS_VIEWABLE, 800, 600)

    def discover(self):
        return self.client.find_owned_windows(10)

    def test_context_close_restores_handler_and_is_idempotent(self):
        self.fake.lib.XSelectInput.assert_called_once_with(
            self.fake.display, self.fake.root, (1 << 17) | (1 << 19))
        with self.client as same:
            self.assertIs(same, self.client)
        self.client.close()
        self.fake.lib.XCloseDisplay.assert_called_once_with(self.fake.display)
        self.assertIsNone(self.fake.error_handler)
        with self.assertRaisesRegex(RuntimeError, "closed"):
            self.client.find_owned_windows(10)

    def test_correct_function_signatures(self):
        self.assertEqual(self.fake.lib.XGetWindowProperty.argtypes[-1],
                         C.POINTER(C.POINTER(C.c_ubyte)))
        self.assertEqual(self.fake.lib.XCheckTypedEvent.argtypes[-1], C.POINTER(probe.XEvent))
        self.assertEqual(self.fake.lib.XGetWindowAttributes.argtypes[-1],
                         C.POINTER(probe.XWindowAttributes))
        self.assertIs(self.fake.lib.XSetErrorHandler.restype, C.c_void_p)
        self.assertIs(self.fake.lib.XDefaultRootWindow.restype, C.c_ulong)

    def test_owned_windows_excludes_root_parent_and_external_desktop(self):
        windows = self.discover()
        self.assertEqual([w["window_id"] for w in windows], [101, 102])
        self.assertEqual([w["kind"] for w in windows], ["gazebo", "rviz"])
        self.assertEqual(windows[0]["starttime"], 120)
        self.assertEqual(windows[0]["wm_class"], "app/Qt")
        self.assertTrue(all(w["protocol_ping"] for w in windows))
        self.assertFalse(self.fake.allocations)
        for window in (103, 104, 105):
            with self.assertRaisesRegex(RuntimeError, "not discovered"):
                self.client.ping(window)
        self.assertFalse(self.fake.sent)

    def test_original_gazebo_g_form(self):
        self.commands[12] = b"/usr/bin/ign\0gazebo\0-g\0world.sdf\0"
        self.assertEqual(self.discover()[0]["kind"], "gazebo")

    def test_rewritten_gui_title_owned_ancestry_and_external_pid(self):
        # Identical Ruby-rewritten titles never grant ownership by themselves.
        for pid in (10, 12, 21):
            self.commands[pid] = b"ign gazebo gui\0"
        windows = self.discover()
        self.assertEqual([(w["window_id"], w["pid"], w["kind"]) for w in windows],
                         [(101, 12, "gazebo"), (102, 13, "rviz")])
        for window in (103, 105):
            with self.assertRaisesRegex(RuntimeError, "not discovered"):
                self.client.ping(window)
        self.assertFalse(self.fake.sent)
        self.assertTrue(self.client.ping(101)["pass"])
        self.table[12] = Process(12, 20, "S", 120)
        with self.assertRaisesRegex(RuntimeError, "ancestry"):
            self.client.ping(101)
        self.assertEqual(len(self.fake.sent), 1)

    def test_actual_gui_title_and_ruby_argv_preserve_owned_window_scope(self):
        for command in (b"ign gazebo -g -v 3\0" + b"\0" * 128,
                        b"ruby\0/usr/bin/ign\0gazebo\0-g\0-v\0" + b"3\0"):
            with self.subTest(command=command):
                self.table[12] = Process(12, 11, "S", 120)
                for pid in (10, 12, 21):
                    self.commands[pid] = command
                self.fake.sent.clear()
                windows = self.discover()
                self.assertEqual([(w["window_id"], w["pid"], w["kind"]) for w in windows],
                                 [(101, 12, "gazebo"), (102, 13, "rviz")])
                for window in (103, 105):
                    with self.assertRaisesRegex(RuntimeError, "not discovered"):
                        self.client.ping(window)
                self.assertFalse(self.fake.sent)
                self.assertTrue(self.client.ping(101)["pass"])
                self.table[12] = Process(12, 20, "S", 120)
                with self.assertRaisesRegex(RuntimeError, "ancestry"):
                    self.client.ping(101)
                self.assertEqual(len(self.fake.sent), 1)

    def test_optional_names_and_class(self):
        for name in ("_NET_WM_NAME", "WM_CLASS"):
            self.fake.properties.pop((101, self.client.atoms[name]))
        self.fake.put(101, "WM_NAME", probe.XA_STRING, 8, b"fallback\xff")
        window = self.discover()[0]
        self.assertEqual(window["name"], "fallback\ufffd")
        self.assertEqual(window["wm_class"], "")
        self.fake.properties.pop((101, self.client.atoms["WM_NAME"]))
        self.assertEqual(self.discover()[0]["name"], "")

    def test_mapped_positive_size_required_not_screen_position(self):
        # A mapped window outside the viewport still satisfies this evidence;
        # no pixel visibility or occlusion claim is made by the probe.
        for attributes in ((0, 800, 600), (1, 800, 600), (2, 0, 600), (2, 800, -1)):
            self.fake.attributes[101] = attributes
            self.assertEqual([w["window_id"] for w in self.discover()], [102])
        self.fake.attributes[101] = (2, 800, 600)
        self.assertEqual(len(self.discover()), 2)

    def test_missing_invalid_pid_and_property_types_fail_closed(self):
        key = (101, self.client.atoms["_NET_WM_PID"])
        for spec in ((probe.XA_CARDINAL, 32, [], 0),
                     (probe.XA_CARDINAL, 32, [12, 13], 0),
                     (probe.XA_CARDINAL, 32, [0], 0),
                     (probe.XA_WINDOW, 32, [12], 0),
                     (probe.XA_CARDINAL, 8, b"12", 0),
                     (probe.XA_CARDINAL, 32, [12], 4)):
            self.fake.properties[key] = spec
            self.assertEqual([w["window_id"] for w in self.discover()], [102])
            self.assertFalse(self.fake.allocations)
        self.fake.properties.pop(key)
        self.assertEqual([w["window_id"] for w in self.discover()], [102])

    def test_property_freed_even_on_async_xerror(self):
        self.fake.pending_error = (3, 101)
        with self.assertRaisesRegex(RuntimeError, "X11.*error"):
            self.client._property(101, "_NET_WM_PID", probe.XA_CARDINAL, 32)
        self.assertFalse(self.fake.allocations)

    def test_badwindow_during_discovery_is_contained(self):
        del self.fake.attributes[101]
        self.assertEqual([w["window_id"] for w in self.discover()], [102])
        self.assertIsNotNone(self.fake.error_handler)

    def test_supervisor_missing_dead_and_proc_failure(self):
        with self.assertRaisesRegex(RuntimeError, "supervisor"):
            self.client.find_owned_windows(999)
        self.table[10] = Process(10, 1, "Z", 100)
        with self.assertRaisesRegex(RuntimeError, "supervisor"):
            self.discover()
        with mock.patch.object(probe, "process_table", side_effect=PermissionError):
            with self.assertRaisesRegex(RuntimeError, "/proc"):
                self.discover()

    def test_discovery_rechecks_identity_after_xlib_reads(self):
        def recycle():
            self.table[12] = Process(12, 11, "S", 999)
        self.fake.after_attributes = recycle
        self.assertEqual([w["window_id"] for w in self.discover()], [102])

    def test_ping_exact_message_and_evidence(self):
        window = self.discover()[0]
        # Modifying report fields must not grant or change ownership.
        window["pid"] = 21
        evidence = self.client.ping(window)
        self.assertTrue(evidence["pass"])
        self.assertEqual(evidence["pid"], 12)
        self.assertEqual(evidence["kind"], "gazebo")
        self.assertGreaterEqual(evidence["latency_seconds"], 0)
        self.assertLess(evidence["latency_seconds"], 2.0)
        target, propagate, mask, sent = self.fake.sent[0]
        self.assertEqual((target, propagate, mask), (101, 0, probe.NO_EVENT_MASK))
        self.assertEqual(sent.xclient.type, probe.CLIENT_MESSAGE)
        self.assertEqual(sent.xclient.message_type, self.client.atoms["WM_PROTOCOLS"])
        self.assertEqual(sent.xclient.format, 32)
        self.assertEqual(list(sent.xclient.data.l)[:3],
                         [self.client.atoms["_NET_WM_PING"], evidence["nonce"], 101])
        self.fake.lib.XFlush.assert_called_once()
        second = self.client.ping(102)
        self.assertEqual(second["kind"], "rviz")
        self.assertNotEqual(second["nonce"], evidence["nonce"])

    def test_spoofed_or_stale_replies_ignored(self):
        self.discover()
        mutations = [lambda e: setattr(e.xclient, "window", 101),
                     lambda e: setattr(e.xclient, "display", 999),
                     lambda e: setattr(e.xclient, "format", 8),
                     lambda e: setattr(e.xclient, "message_type", 999),
                     lambda e: setattr(e.xclient, "send_event", 0),
                     lambda e: setattr(e.xclient, "type", 2),
                     lambda e: e.xclient.data.l.__setitem__(0, 999),
                     lambda e: e.xclient.data.l.__setitem__(1, 999),
                     lambda e: e.xclient.data.l.__setitem__(2, 102)]
        self.fake.reply_mutations = mutations
        self.assertTrue(self.client.ping(101)["pass"])
        self.assertEqual(self.fake.lib.XCheckTypedEvent.call_count, len(mutations) + 1)

    def test_qt_structure_notify_reply_requires_matching_selection(self):
        self.discover()
        self.fake.selected_mask = 1 << 19  # Old probe cannot receive Qt echoes.
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.client.ping(101, 0.04)
        self.fake.selected_mask = (1 << 17) | (1 << 19)
        self.assertTrue(self.client.ping(101)["pass"])

    def test_substructure_notify_clients_also_supported_without_redirect(self):
        self.discover()
        self.fake.reply_mask = 1 << 19
        self.assertTrue(self.client.ping(101)["pass"])
        self.assertFalse(self.fake.selected_mask & (1 << 20))

    def test_only_spoofed_reply_times_out(self):
        self.discover()
        original = self.fake.send
        def send(*args):
            result = original(*args)
            self.fake.queue[0].xclient.data.l[2] = 103
            return result
        self.fake.lib.XSendEvent.side_effect = send
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.client.ping(101, 0.05)
        self.assertTrue(self.clock.waits)
        self.assertTrue(all(0 < wait <= 0.01 for wait in self.clock.waits))

    def test_timeout_without_reply_and_invalid_deadlines(self):
        self.discover()
        self.fake.auto_reply = False
        for timeout in (0, -1, float("inf"), float("nan")):
            with self.assertRaisesRegex(RuntimeError, "finite and positive"):
                self.client.ping(101, timeout)
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.client.ping(101, 0.04)
        self.assertLess(self.clock.now, 100.1)

    def test_reply_flood_still_bounded(self):
        self.discover()
        self.fake.auto_reply = False
        def flood(display, event_type, output):
            return 1  # Zeroed event is always rejected; clock still advances.
        self.fake.lib.XCheckTypedEvent.side_effect = flood
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.client.ping(101, 0.04)
        self.assertLess(self.fake.lib.XCheckTypedEvent.call_count, 50)

    def test_unsupported_ping_never_sends(self):
        self.fake.put(101, "WM_PROTOCOLS", probe.XA_ATOM, 32, [])
        windows = self.discover()
        self.assertFalse(windows[0]["protocol_ping"])
        with self.assertRaisesRegex(RuntimeError, "does not support"):
            self.client.ping(101)
        self.assertFalse(self.fake.sent)

    def test_identity_ancestry_cmdline_windowpid_and_unmapping_races(self):
        for race in ("pid", "ancestor", "root", "dead", "reparent", "command",
                     "windowpid", "unmapped", "destroyed"):
            with self.subTest(race=race):
                # Restore baseline for each independent ownership race.
                self.table[10] = Process(10, 1, "S", 100)
                self.table[11] = Process(11, 10, "S", 110)
                self.table[12] = Process(12, 11, "S", 120)
                self.commands[12] = b"ign\0gazebo\0gui\0"
                self.window(101, 12)
                self.discover()
                if race == "pid":
                    self.table[12] = Process(12, 11, "S", 999)
                elif race == "ancestor":
                    self.table[11] = Process(11, 10, "S", 999)
                elif race == "root":
                    self.table[10] = Process(10, 1, "S", 999)
                elif race == "dead":
                    self.table[12] = Process(12, 11, "Z", 120)
                elif race == "reparent":
                    self.table[12] = Process(12, 20, "S", 120)
                elif race == "command":
                    self.commands[12] = b"ign\0gazebo\0-s\0"
                elif race == "windowpid":
                    self.fake.put(101, "_NET_WM_PID", probe.XA_CARDINAL, 32, [21])
                elif race == "unmapped":
                    self.fake.attributes[101] = (0, 800, 600)
                else:
                    del self.fake.attributes[101]
                with self.assertRaises(RuntimeError):
                    self.client.ping(101)
        self.assertFalse(self.fake.sent)

    def test_ownership_race_after_echo_does_not_pass(self):
        self.discover()
        def recycle():
            self.table[12] = Process(12, 11, "S", 999)
        self.fake.after_send = recycle
        with self.assertRaisesRegex(RuntimeError, "identity"):
            self.client.ping(101)

    def test_windowpid_race_after_echo_does_not_pass(self):
        self.discover()
        self.fake.after_send = lambda: self.fake.put(
            101, "_NET_WM_PID", probe.XA_CARDINAL, 32, [21])
        with self.assertRaisesRegex(RuntimeError, "window PID"):
            self.client.ping(101)

    def test_windowpid_race_during_metadata_reads_never_sends(self):
        self.discover()
        self.fake.after_attributes = lambda: self.fake.put(
            101, "_NET_WM_PID", probe.XA_CARDINAL, 32, [21])
        with self.assertRaisesRegex(RuntimeError, "window PID"):
            self.client.ping(101)
        self.assertFalse(self.fake.sent)

    def test_dead_pid_and_unreadable_cmdline_raise(self):
        self.discover()
        process = self.table.pop(12)
        with self.assertRaisesRegex(RuntimeError, "PID is dead"):
            self.client.ping(101)
        self.table[12] = process
        with mock.patch.object(probe.X11Probe, "_cmdline", side_effect=PermissionError):
            with self.assertRaisesRegex(RuntimeError, "cannot be verified"):
                self.client.ping(101)
        self.assertFalse(self.fake.sent)

    def test_reply_arriving_after_deadline_does_not_pass(self):
        self.discover()
        original = self.fake.check
        def delayed_check(*args):
            self.clock.now += 1.0
            return original(*args)
        self.fake.lib.XCheckTypedEvent.side_effect = delayed_check
        with self.assertRaisesRegex(RuntimeError, "timeout"):
            self.client.ping(101, 0.05)

    def test_prior_nonnull_error_handler_restored(self):
        self.client.close()
        previous = probe.XErrorHandler(lambda display, event: 0)
        address = C.cast(previous, C.c_void_p).value
        self.fake.error_handler = address
        with probe.X11Probe():
            self.assertNotEqual(self.fake.error_handler, address)
        self.assertEqual(self.fake.error_handler, address)

    def test_protocol_removed_after_discovery(self):
        self.discover()
        self.fake.properties.pop((101, self.client.atoms["WM_PROTOCOLS"]))
        with self.assertRaisesRegex(RuntimeError, "does not support"):
            self.client.ping(101)

    def test_send_failure_and_async_badwindow_raise(self):
        self.discover()
        self.fake.lib.XSendEvent.side_effect = lambda *args: 0
        with self.assertRaisesRegex(RuntimeError, "XSendEvent failed"):
            self.client.ping(101)
        def badwindow(*args):
            self.fake.pending_error = (3, 101)
            return 1
        self.fake.lib.XSendEvent.side_effect = badwindow
        with self.assertRaisesRegex(RuntimeError, "X11 XSendEvent error"):
            self.client.ping(101)

    def test_rediscovery_revokes_old_allowlist(self):
        self.discover()
        self.fake.put(100, "_NET_CLIENT_LIST", probe.XA_WINDOW, 32, [102])
        self.discover()
        with self.assertRaisesRegex(RuntimeError, "not discovered"):
            self.client.ping(101)

    def test_nonce_wrap_skips_zero(self):
        self.discover()
        self.client._nonce = 0xffffffff
        self.assertEqual(self.client.ping(101)["nonce"], 1)

    def test_second_probe_rejected_without_opening_connection(self):
        with self.assertRaisesRegex(RuntimeError, "only one"):
            probe.X11Probe()
        self.fake.lib.XOpenDisplay.assert_called_once()

    def test_constructor_validation_and_failure_cleanup(self):
        self.client.close()
        for display in ("", "localhost:10.0", "host:0", "unix:0", ":0evil", ":-1", ":\uff11"):
            with mock.patch.dict(os.environ, {"DISPLAY": display}):
                with self.assertRaisesRegex(RuntimeError, "local"):
                    probe.X11Probe()
        with mock.patch.object(probe, "find_library", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "libX11"):
                probe.X11Probe()
        self.fake.lib.XOpenDisplay.side_effect = lambda name: None
        with self.assertRaisesRegex(RuntimeError, "cannot open"):
            probe.X11Probe()
        self.assertFalse(probe._XLIB_LOCK.locked())
        self.fake.lib.XOpenDisplay.side_effect = lambda name: self.fake.display
        self.fake.pending_error = (3, self.fake.root)
        with self.assertRaisesRegex(RuntimeError, "X11"):
            probe.X11Probe()
        self.assertIsNone(self.fake.error_handler)
        self.assertFalse(probe._XLIB_LOCK.locked())


if __name__ == "__main__":
    unittest.main()