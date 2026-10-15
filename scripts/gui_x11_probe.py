"""Stdlib-only evidence of owned Gazebo/RViz Qt event-loop responsiveness.

Use X11Probe() as a context manager, discover with find_owned_windows(root_pid),
then ping each returned dict (or its window_id). No ROS imports or subreaper
changes. Requires Linux /proc, local DISPLAY (e.g. :10.0), libX11 and EWMH.
Mapped positive-size windows are not proof of visible pixels or rendering.
WM_PING is not authentication against malicious clients on the same X server.

XSetErrorHandler is process-global: use in a dedicated, single-threaded probe
process without concurrent third-party Xlib calls. Only one probe may be open.
The response deadline bounds polling, not a stalled X server's synchronous I/O.
"""

import ctypes as C
from ctypes.util import find_library
import math
import os
import re
import secrets
import threading
import time

from supervised_process import PROC, descendants, process_table, read_process


CLIENT_MESSAGE = 33
IS_VIEWABLE = 2
STRUCTURE_NOTIFY_MASK = 1 << 17
SUBSTRUCTURE_NOTIFY_MASK = 1 << 19
NO_EVENT_MASK = 0
XA_ATOM = 4
XA_CARDINAL = 6
XA_STRING = 31
XA_WINDOW = 33
UINT32_MASK = 0xffffffff
_XLIB_LOCK = threading.Lock()


class XClientMessageData(C.Union):
    _fields_ = [("b", C.c_char * 20), ("s", C.c_short * 10),
                ("l", C.c_long * 5)]


class XClientMessageEvent(C.Structure):
    _fields_ = [("type", C.c_int), ("serial", C.c_ulong),
                ("send_event", C.c_int), ("display", C.c_void_p),
                ("window", C.c_ulong), ("message_type", C.c_ulong),
                ("format", C.c_int), ("data", XClientMessageData)]


class XEvent(C.Union):
    _fields_ = [("type", C.c_int), ("xclient", XClientMessageEvent),
                ("pad", C.c_long * 24)]


class XWindowAttributes(C.Structure):
    _fields_ = [("x", C.c_int), ("y", C.c_int), ("width", C.c_int),
                ("height", C.c_int), ("border_width", C.c_int),
                ("depth", C.c_int), ("visual", C.c_void_p),
                ("root", C.c_ulong), ("class_", C.c_int),
                ("bit_gravity", C.c_int), ("win_gravity", C.c_int),
                ("backing_store", C.c_int), ("backing_planes", C.c_ulong),
                ("backing_pixel", C.c_ulong), ("save_under", C.c_int),
                ("colormap", C.c_ulong), ("map_installed", C.c_int),
                ("map_state", C.c_int), ("all_event_masks", C.c_long),
                ("your_event_mask", C.c_long),
                ("do_not_propagate_mask", C.c_long),
                ("override_redirect", C.c_int), ("screen", C.c_void_p)]


class XErrorEvent(C.Structure):
    _fields_ = [("type", C.c_int), ("display", C.c_void_p),
                ("resourceid", C.c_ulong), ("serial", C.c_ulong),
                ("error_code", C.c_ubyte), ("request_code", C.c_ubyte),
                ("minor_code", C.c_ubyte)]


XErrorHandler = C.CFUNCTYPE(C.c_int, C.c_void_p, C.POINTER(XErrorEvent))


def property32(pointer, count):
    """Xlib expands wire-format 32-bit items to native unsigned longs."""
    if not count:
        return []
    if not pointer:
        raise RuntimeError("null X11 property data")
    longs = C.cast(pointer, C.POINTER(C.c_ulong))
    return [int(longs[i]) & UINT32_MASK for i in range(count)]


def gui_kind(cmdline):
    """Match argv or Fortress's exact rewritten argv[0], never shell strings."""
    argv = cmdline.rstrip(b"\0").split(b"\0")
    if not argv or not argv[0]:
        return None
    # Ruby's Fortress CLI uses $0 to rewrite /proc/PID/cmdline into one title.
    # The caller still verifies PID/starttime and the owned supervisor ancestry.
    if len(argv) == 1 and argv[0] == b"ign gazebo gui":
        return "gazebo"
    if len(argv) == 1 and re.fullmatch(rb"ign gazebo -g(?: -v [0-4])?", argv[0]):
        return "gazebo"
    if len(argv) >= 2 and os.path.basename(argv[0]) == b"ruby" and argv[1] == b"/usr/bin/ign":
        argv = argv[1:]
    if (os.path.basename(argv[0]) == b"ign" and len(argv) >= 3
            and argv[1] == b"gazebo" and argv[2] in (b"gui", b"-g")
            and b"-s" not in argv[2:] and b"--server-only" not in argv[2:]):
        return "gazebo"
    if argv[0].endswith(b"/rviz2/rviz2"):
        return "rviz"
    return None


def _bind_xlib(lib):
    display, window, atom = C.c_void_p, C.c_ulong, C.c_ulong
    signatures = {
        "XOpenDisplay": ([C.c_char_p], display),
        "XCloseDisplay": ([display], C.c_int),
        "XDefaultRootWindow": ([display], window),
        "XInternAtom": ([display, C.c_char_p, C.c_int], atom),
        "XSelectInput": ([display, window, C.c_long], C.c_int),
        "XSync": ([display, C.c_int], C.c_int),
        "XFlush": ([display], C.c_int),
        "XFree": ([C.c_void_p], C.c_int),
        "XSetErrorHandler": ([C.c_void_p], C.c_void_p),
        "XGetWindowProperty": ([display, window, atom, C.c_long, C.c_long,
                                C.c_int, atom, C.POINTER(atom),
                                C.POINTER(C.c_int), C.POINTER(C.c_ulong),
                                C.POINTER(C.c_ulong),
                                C.POINTER(C.POINTER(C.c_ubyte))], C.c_int),
        "XGetWindowAttributes": ([display, window,
                                  C.POINTER(XWindowAttributes)], C.c_int),
        "XSendEvent": ([display, window, C.c_int, C.c_long,
                        C.POINTER(XEvent)], C.c_int),
        "XCheckTypedEvent": ([display, C.c_int, C.POINTER(XEvent)], C.c_int),
    }
    for name, (args, result) in signatures.items():
        function = getattr(lib, name)
        function.argtypes = args
        function.restype = result


class X11Probe:
    def __init__(self):
        self.display = None
        self._locked = False
        self._handler_installed = False
        self._owned = {}
        self._errors = []
        self._nonce = secrets.randbits(32)
        display_name = os.environ.get("DISPLAY", "")
        if not re.fullmatch(r":[0-9]+(?:\.[0-9]+)?", display_name):
            raise RuntimeError("DISPLAY must be a local :N[.screen] display")
        library = find_library("X11")
        if not library:
            raise RuntimeError("libX11 not found")
        if not _XLIB_LOCK.acquire(blocking=False):
            raise RuntimeError("only one X11Probe may be open per process")
        self._locked = True
        try:
            self.lib = C.CDLL(library)
            _bind_xlib(self.lib)
            self.display = self.lib.XOpenDisplay(display_name.encode("ascii"))
            if not self.display:
                raise RuntimeError("cannot open DISPLAY " + display_name)
            # Retain callback for its entire installed lifetime.
            self._callback = XErrorHandler(self._on_error)
            self._previous_handler = self.lib.XSetErrorHandler(
                C.cast(self._callback, C.c_void_p))
            self._handler_installed = True
            self.root = self._call("XDefaultRootWindow", self.display)
            self.atoms = {name: self._call("XInternAtom", self.display,
                                         name.encode("ascii"), 0)
                          for name in ("_NET_CLIENT_LIST", "_NET_WM_PID",
                                       "WM_PROTOCOLS", "_NET_WM_PING",
                                       "_NET_WM_NAME", "UTF8_STRING",
                                       "WM_NAME", "WM_CLASS")}
            # Never SubstructureRedirectMask: we are not a window manager.
            # Qt 5 xcb sends ping echoes with StructureNotify | Redirect,
            # whereas other clients may use SubstructureNotify | Redirect.
            # Both notification masks are nonexclusive and safe to observe.
            self._call("XSelectInput", self.display, self.root,
                       STRUCTURE_NOTIFY_MASK | SUBSTRUCTURE_NOTIFY_MASK)
        except BaseException:
            self.close()
            raise

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        """Idempotently close our connection and restore the global handler."""
        try:
            if self.display:
                try:
                    self.lib.XCloseDisplay(self.display)
                finally:
                    self.display = None
        finally:
            try:
                if self._handler_installed:
                    self.lib.XSetErrorHandler(self._previous_handler)
                    self._handler_installed = False
            finally:
                self._owned.clear()
                if self._locked:
                    self._locked = False
                    _XLIB_LOCK.release()

    def _require_open(self):
        if not self.display:
            raise RuntimeError("X11Probe is closed")

    def _on_error(self, display, event):
        error = event.contents
        self._errors.append((error.error_code, error.resourceid,
                             error.request_code, error.minor_code))
        return 0

    def _call(self, name, *args):
        self._require_open()
        self._errors.clear()
        result = getattr(self.lib, name)(*args)
        self.lib.XSync(self.display, 0)
        if self._errors:
            raise RuntimeError("X11 {} error(s): {}".format(name, self._errors))
        return result

    def _property(self, window, name, expected_type, expected_format):
        actual_type, actual_format = C.c_ulong(), C.c_int()
        count, remaining = C.c_ulong(), C.c_ulong()
        pointer = C.POINTER(C.c_ubyte)()
        try:
            status = self._call(
                "XGetWindowProperty", self.display, window, self.atoms[name],
                0, 65536, 0, expected_type, C.byref(actual_type),
                C.byref(actual_format), C.byref(count), C.byref(remaining),
                C.byref(pointer))
            if status != 0:
                raise RuntimeError("cannot read X11 property " + name)
            if actual_type.value == 0:
                return [] if expected_format == 32 else b""
            if (actual_type.value != expected_type
                    or actual_format.value != expected_format or remaining.value):
                raise RuntimeError("invalid or oversized X11 property " + name)
            if expected_format == 32:
                return property32(pointer, count.value)
            if count.value and not pointer:
                raise RuntimeError("null X11 property data")
            return C.string_at(pointer, count.value) if count.value else b""
        finally:
            if pointer:
                self.lib.XFree(C.cast(pointer, C.c_void_p))

    def _pid(self, window):
        pids = self._property(window, "_NET_WM_PID", XA_CARDINAL, 32)
        if len(pids) != 1 or pids[0] <= 0:
            raise RuntimeError("missing/invalid window PID")
        return pids[0]

    def _inspect(self, window, process, kind):
        if self._pid(window) != process.pid:
            raise RuntimeError("window PID ownership changed")
        attributes = XWindowAttributes()
        if not self._call("XGetWindowAttributes", self.display, window,
                          C.byref(attributes)):
            raise RuntimeError("window no longer exists")
        if (attributes.map_state != IS_VIEWABLE or attributes.width <= 0
                or attributes.height <= 0):
            raise RuntimeError("window is not mapped/viewable with positive size")
        protocols = self._property(window, "WM_PROTOCOLS", XA_ATOM, 32)
        name = self._property(window, "_NET_WM_NAME", self.atoms["UTF8_STRING"], 8)
        if not name:
            name = self._property(window, "WM_NAME", XA_STRING, 8)
        wm_class = self._property(window, "WM_CLASS", XA_STRING, 8)
        # Metadata reads are separate requests; catch ownership changes between
        # them before registering a client, sending a ping, or reporting success.
        if self._pid(window) != process.pid:
            raise RuntimeError("window PID ownership changed")
        return {"kind": kind, "window_id": window, "pid": process.pid,
                "starttime": process.starttime,
                "name": name.decode("utf-8", errors="replace"),
                "width": attributes.width, "height": attributes.height,
                "protocol_ping": self.atoms["_NET_WM_PING"] in protocols,
                "wm_class": wm_class.rstrip(b"\0").replace(b"\0", b"/").decode(
                    "utf-8", errors="replace")}

    @staticmethod
    def _cmdline(pid):
        return (PROC / str(pid) / "cmdline").read_bytes()

    @staticmethod
    def _chain(table, process, root):
        chain, seen = [], set()
        while True:
            if process.pid in seen or not process.alive:
                raise RuntimeError("invalid/dead process ancestry")
            seen.add(process.pid)
            chain.append(process)
            if process.pid == root:
                return tuple(chain)
            process = table.get(process.ppid)
            if process is None:
                raise RuntimeError("incomplete supervisor ancestry")

    def _check_chain(self, chain, kind):
        for expected in chain:
            current = read_process(expected.pid)
            if (current is None or not current.alive
                    or current.identity != expected.identity
                    or current.ppid != expected.ppid):
                raise RuntimeError("process identity/ancestry changed or PID is dead")
        if gui_kind(self._cmdline(chain[0].pid)) != kind:
            raise RuntimeError("GUI process command changed")

    def find_owned_windows(self, supervisor_pid):
        """Return only EWMH clients belonging to this live supervisor's GUI tree.

        Each discovery replaces the ping allowlist. A caller must supply the
        dedicated test supervisor PID, not a user session/desktop root PID.
        """
        self._require_open()
        self._owned.clear()
        try:
            table = process_table()
            root = table.get(supervisor_pid)
            if root is None or not root.alive:
                raise RuntimeError("supervisor PID is missing or dead")
            candidates = {}
            for process in descendants(table, supervisor_pid).values():
                if not process.alive:
                    continue
                try:
                    kind = gui_kind(self._cmdline(process.pid))
                    if kind:
                        chain = self._chain(table, process, supervisor_pid)
                        self._check_chain(chain, kind)
                        candidates[process.pid] = (process, kind, chain)
                except (OSError, ValueError, RuntimeError):
                    continue
            windows = self._property(self.root, "_NET_CLIENT_LIST", XA_WINDOW, 32)
            result = []
            for window in dict.fromkeys(windows):
                try:
                    candidate = candidates.get(self._pid(window))
                    if candidate is None:
                        continue
                    process, kind, chain = candidate
                    self._check_chain(chain, kind)
                    evidence = self._inspect(window, process, kind)
                    self._check_chain(chain, kind)
                    self._owned[window] = (dict(evidence), chain)
                    result.append(evidence)
                except (OSError, ValueError, RuntimeError):
                    # Clients can disappear while enumerating; never broaden scope.
                    continue
            return result
        except (OSError, ValueError) as exc:
            raise RuntimeError("cannot inspect supervisor /proc tree") from exc

    def _validate(self, window):
        registered = self._owned.get(window)
        if registered is None:
            raise RuntimeError("window was not discovered under this supervisor")
        original, chain = registered
        try:
            self._check_chain(chain, original["kind"])
            evidence = self._inspect(window, chain[0], original["kind"])
            self._check_chain(chain, original["kind"])
        except (OSError, ValueError) as exc:
            raise RuntimeError("GUI PID is dead or cannot be verified") from exc
        if not evidence["protocol_ping"]:
            raise RuntimeError("window does not support _NET_WM_PING")
        return evidence

    def ping(self, window, timeout=2.0):
        """Send WM_PING to an allowlisted app and require its exact root echo.

        No focus, input, geometry or window-manager redirection changes. A
        passing result proves event-loop response, not screenshot pixel content.
        """
        self._require_open()
        if not math.isfinite(timeout) or timeout <= 0:
            raise RuntimeError("ping timeout must be finite and positive")
        window = window["window_id"] if isinstance(window, dict) else window
        evidence = self._validate(window)
        self._nonce = (self._nonce + 1) & UINT32_MASK
        if not self._nonce:
            self._nonce = 1
        nonce = self._nonce
        event = XEvent()
        event.xclient.type = CLIENT_MESSAGE
        event.xclient.display = self.display
        event.xclient.window = window
        event.xclient.message_type = self.atoms["WM_PROTOCOLS"]
        event.xclient.format = 32
        event.xclient.data.l[0] = self.atoms["_NET_WM_PING"]
        event.xclient.data.l[1] = nonce
        event.xclient.data.l[2] = window
        started = time.monotonic()
        deadline = started + timeout
        if not self._call("XSendEvent", self.display, window, 0,
                          NO_EVENT_MASK, C.byref(event)):
            raise RuntimeError("XSendEvent failed for GUI window")
        self._call("XFlush", self.display)
        wake = threading.Event()
        observed = []
        while time.monotonic() < deadline:
            reply = XEvent()
            if self._call("XCheckTypedEvent", self.display, CLIENT_MESSAGE,
                          C.byref(reply)):
                client = reply.xclient
                data = [int(value) & UINT32_MASK for value in client.data.l]
                if data[2] == window and len(observed) < 5:
                    observed.append({"type": client.type, "window": client.window,
                                     "root": self.root, "send_event": client.send_event,
                                     "format": client.format, "message_type": client.message_type,
                                     "expected_type": self.atoms["WM_PROTOCOLS"],
                                     "data": data, "nonce": nonce})
                if (client.type == CLIENT_MESSAGE and client.display == self.display
                        and client.window == self.root and client.send_event
                        and client.format == 32
                        and client.message_type == self.atoms["WM_PROTOCOLS"]
                        and data[:3] == [self.atoms["_NET_WM_PING"], nonce, window]):
                    elapsed = time.monotonic() - started
                    if elapsed >= timeout:
                        break
                    evidence = self._validate(window)
                    elapsed = time.monotonic() - started
                    if elapsed >= timeout:
                        break
                    return dict(evidence, **{"pass": True,
                                            "latency_seconds": elapsed,
                                            "nonce": nonce})
            else:
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    wake.wait(min(0.01, remaining))
        raise RuntimeError("_NET_WM_PING timeout for window {}; replies={}".format(window, observed))