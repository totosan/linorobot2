"""Native X11 window for offline vision-fixture replay; uses system libX11 only."""

import ctypes as C
from ctypes.util import find_library
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "linorobot2_vision"
sys.path.insert(0, str(PKG / "python"))
from linorobot2_vision import load_replay_case  # noqa: E402

FIXTURE_KEYS = {"1": "detection", "2": "empty", "3": "unavailable"}
EXPOSURE = 1 << 15
KEY_PRESS = 1
STRUCTURE_NOTIFY = 1 << 17
CLIENT_MESSAGE = 33


class XKeyEvent(C.Structure):
    _fields_ = [("type", C.c_int), ("serial", C.c_ulong), ("send_event", C.c_int),
                ("display", C.c_void_p), ("window", C.c_ulong),
                ("root", C.c_ulong), ("subwindow", C.c_ulong), ("time", C.c_ulong),
                ("x", C.c_int), ("y", C.c_int), ("x_root", C.c_int),
                ("y_root", C.c_int), ("state", C.c_uint), ("keycode", C.c_uint),
                ("same_screen", C.c_int)]


class XClientMessage(C.Structure):
    _fields_ = [("type", C.c_int), ("serial", C.c_ulong), ("send_event", C.c_int),
                ("display", C.c_void_p), ("window", C.c_ulong),
                ("message_type", C.c_ulong), ("format", C.c_int),
                ("data", C.c_long * 5)]


class XEvent(C.Union):
    _fields_ = [("type", C.c_int), ("key", XKeyEvent),
                ("client", XClientMessage), ("pad", C.c_long * 24)]


class XColor(C.Structure):
    _fields_ = [("pixel", C.c_ulong), ("red", C.c_ushort), ("green", C.c_ushort),
                ("blue", C.c_ushort), ("flags", C.c_char), ("pad", C.c_char)]


def bbox_rectangle(box, scale=1.0):
    x1, y1, x2, y2 = box
    return round(x1 * scale), round(y1 * scale), max(1, round((x2-x1)*scale)), max(1, round((y2-y1)*scale))


class Monitor:
    def __init__(self):
        lib = find_library("X11")
        if not lib:
            raise RuntimeError("libX11 not found")
        self.x = C.CDLL(lib)
        self.bind()
        self.dpy = self.x.XOpenDisplay(None)
        if not self.dpy:
            raise RuntimeError(f"cannot open DISPLAY={os.environ.get('DISPLAY')!r}")
        screen = self.x.XDefaultScreen(self.dpy)
        root = self.x.XRootWindow(self.dpy, screen)
        cmap = self.x.XDefaultColormap(self.dpy, screen)
        self.sw = self.x.XDisplayWidth(self.dpy, screen)
        self.sh = self.x.XDisplayHeight(self.dpy, screen)
        self.colors = {name: self.alloc(cmap, value) for name, value in {
            "bg": "#07111f", "panel": "#101d2e", "grid": "#243449",
            "text": "#e6edf3", "muted": "#93a4b7", "cyan": "#22d3ee",
            "green": "#4ade80", "amber": "#fbbf24", "red": "#fb7185",
            "box": "#facc15"}.items()}
        self.color_aliases = {
            "#07111f": "bg", "#101d2e": "panel", "#243449": "grid",
            "#e6edf3": "text", "#93a4b7": "muted", "#22d3ee": "cyan",
            "#4ade80": "green", "#fbbf24": "amber", "#fb7185": "red",
            "#facc15": "box",
        }
        self.w = min(1180, self.sw-40)
        self.h = min(850, self.sh-80)
        self.scale = min(1.0, (self.w-390)/640, (self.h-285)/480)
        if self.scale <= 0:
            raise RuntimeError("RDP display is too small")
        self.iw, self.ih = round(640*self.scale), round(480*self.scale)
        self.ix, self.iy = 32, 152
        win_x, win_y = max(0, (self.sw-self.w)//2), max(0, (self.sh-self.h)//2)
        self.win = self.x.XCreateSimpleWindow(self.dpy, root, win_x, win_y,
                                               self.w, self.h, 1,
                                               self.colors["cyan"], self.colors["bg"])
        self.x.XStoreName(self.dpy, self.win, b"Linorobot2 Vision Fixture Monitor | OFFLINE MOCK")
        self.delete_atom = self.x.XInternAtom(self.dpy, b"WM_DELETE_WINDOW", 0)
        protocols = (C.c_ulong * 1)(self.delete_atom)
        self.x.XSetWMProtocols(self.dpy, self.win, protocols, 1)
        self.x.XSelectInput(self.dpy, self.win, EXPOSURE | KEY_PRESS | STRUCTURE_NOTIFY)
        self.gc = self.x.XCreateGC(self.dpy, self.win, 0, None)
        self.x.XMapRaised(self.dpy, self.win)
        self.x.XSync(self.dpy, 0)
        self.x.XRaiseWindow(self.dpy, self.win)
        self.x.XFlush(self.dpy)
        print(f"Native X11 window 0x{self.win:x} mapped at {self.w}x{self.h} on {os.environ.get('DISPLAY')}", flush=True)

    def bind(self):
        d, w, gc = C.c_void_p, C.c_ulong, C.c_void_p
        sig = {
            "XOpenDisplay": ([C.c_char_p], d), "XDefaultScreen": ([d], C.c_int),
            "XRootWindow": ([d, C.c_int], w), "XDefaultColormap": ([d, C.c_int], C.c_ulong),
            "XDisplayWidth": ([d, C.c_int], C.c_int), "XDisplayHeight": ([d, C.c_int], C.c_int),
            "XAllocNamedColor": ([d, C.c_ulong, C.c_char_p, C.POINTER(XColor), C.POINTER(XColor)], C.c_int),
            "XCreateSimpleWindow": ([d, w, C.c_int, C.c_int, C.c_uint, C.c_uint, C.c_uint, C.c_ulong, C.c_ulong], w),
            "XStoreName": ([d, w, C.c_char_p], C.c_int), "XInternAtom": ([d, C.c_char_p, C.c_int], C.c_ulong),
            "XSetWMProtocols": ([d, w, C.POINTER(C.c_ulong), C.c_int], C.c_int),
            "XSelectInput": ([d, w, C.c_long], C.c_int), "XCreateGC": ([d, w, C.c_ulong, C.c_void_p], gc),
            "XMapRaised": ([d, w], C.c_int), "XSync": ([d, C.c_int], C.c_int),
            "XRaiseWindow": ([d, w], C.c_int), "XFlush": ([d], C.c_int),
            "XSetForeground": ([d, gc, C.c_ulong], C.c_int),
            "XFillRectangle": ([d, w, gc, C.c_int, C.c_int, C.c_uint, C.c_uint], C.c_int),
            "XDrawRectangle": ([d, w, gc, C.c_int, C.c_int, C.c_uint, C.c_uint], C.c_int),
            "XDrawLine": ([d, w, gc, C.c_int, C.c_int, C.c_int, C.c_int], C.c_int),
            "XDrawString": ([d, w, gc, C.c_int, C.c_int, C.c_char_p, C.c_int], C.c_int),
            "XNextEvent": ([d, C.POINTER(XEvent)], C.c_int), "XLookupKeysym": ([C.POINTER(XKeyEvent), C.c_int], C.c_ulong),
            "XFreeGC": ([d, gc], C.c_int), "XDestroyWindow": ([d, w], C.c_int), "XCloseDisplay": ([d], C.c_int)}
        for name, (args, ret) in sig.items():
            fn = getattr(self.x, name)
            fn.argtypes, fn.restype = args, ret

    def alloc(self, cmap, name):
        screen, exact = XColor(), XColor()
        if not self.x.XAllocNamedColor(self.dpy, cmap, name.encode(), C.byref(screen), C.byref(exact)):
            raise RuntimeError(f"cannot allocate color {name}")
        return int(screen.pixel)

    def color(self, name):
        self.x.XSetForeground(self.dpy, self.gc, self.colors[name])

    def text(self, px, py, value, color="text"):
        raw = str(value).encode("ascii", errors="replace")
        self.color(color)
        self.x.XDrawString(self.dpy, self.win, self.gc, int(px), int(py), raw, len(raw))

    def fill(self, px, py, width, height, color):
        self.color(color)
        self.x.XFillRectangle(self.dpy, self.win, self.gc, int(px), int(py), max(1, int(width)), max(1, int(height)))

    def rect(self, px, py, width, height, color):
        self.color(color)
        self.x.XDrawRectangle(self.dpy, self.win, self.gc, int(px), int(py), max(1, int(width)), max(1, int(height)))

    def draw_scene_shapes(self, shapes):
        for shape in shapes:
            kind = shape["kind"]
            if kind == "rect":
                px = self.ix+round(shape["x"]*self.scale)
                py = self.iy+round(shape["y"]*self.scale)
                width = max(1, round(shape["width"]*self.scale))
                height = max(1, round(shape["height"]*self.scale))
                if shape["fill"] != "none":
                    color = self.color_aliases[shape["fill"]]
                    self.fill(px, py, width, height, color)
                if shape["stroke"] != "none":
                    color = self.color_aliases[shape["stroke"]]
                    border = max(1, round(shape["stroke_width"]*self.scale))
                    for inset in range(border):
                        self.rect(px+inset, py+inset, width-2*inset,
                                  height-2*inset, color)
            elif kind == "line":
                self.color(self.color_aliases[shape["stroke"]])
                x1 = self.ix+round(shape["x1"]*self.scale)
                y1 = self.iy+round(shape["y1"]*self.scale)
                x2 = self.ix+round(shape["x2"]*self.scale)
                y2 = self.iy+round(shape["y2"]*self.scale)
                self.x.XDrawLine(self.dpy, self.win, self.gc, x1, y1, x2, y2)

    def draw(self, response, name, scene_shapes):
        self.fill(0, 0, self.w, self.h, "bg")
        self.fill(0, 0, self.w, 78, "panel")
        self.text(30, 36, "LINOROBOT2 / VISION FIXTURE REPLAY", "cyan")
        self.text(30, 59, "OFFLINE MOCK ONLY - NO CAMERA, ROS, NETWORK, OR INFERENCE", "muted")
        status = response["status"]
        self.text(30, 112, f"FIXTURE: {name.upper()}")
        self.text(350, 112, f"STATUS: {status.upper()}", {"ok": "green", "error": "red", "unavailable": "amber"}[status])
        self.text(610, 112, f"SCHEMA: {response['schema_version']}", "muted")
        x, y = self.ix, self.iy
        self.fill(x, y, self.iw, self.ih, "panel")
        self.draw_scene_shapes(scene_shapes)
        self.rect(x, y, self.iw, self.ih, "cyan")
        dx = x+self.iw+35
        if status == "ok":
            result, = (response["result"],)
            capture, model = result["capture"], result["model"]
            for i, line in enumerate(("CAPTURE", f"{capture['width']} x {capture['height']} px",
                                      f"frame: {capture['frame_id'][:24]}", f"time: {capture['stamp_ns']} ns",
                                      f"clock: {capture['clock_domain']}", "MODEL", model["name"][:28],
                                      f"version: {model['version'][:20]}", f"labels: {model['labels_version'][:20]}",
                                      f"DETECTIONS: {len(result['detections'])}")):
                self.text(dx, y+20+i*27, line, "cyan" if i in (0, 5) else "green" if i == 9 else "text")
            for i, det in enumerate(result["detections"][:4]):
                bx, by, bw, bh = bbox_rectangle(det["bbox_xyxy_px"], self.scale)
                self.rect(x+bx, y+by, bw, bh, "box")
                label = f"{det['label']} {det['confidence']:.2f}"
                self.fill(x+bx, max(y+2, y+by-22), len(label)*9+12, 19, "bg")
                self.text(x+bx+5, max(y+16, y+by-8), label, "box")
                self.text(dx, y+320+i*25, f"{det['label']} {det['confidence']:.3f}", "box")
            if not result["detections"]:
                self.text(x+20, y+self.ih//2, "VALID EMPTY RESULT", "green")
        else:
            self.text(x+24, y+self.ih//2, "VISION BACKEND UNAVAILABLE", "red")
            err = response["error"]
            self.text(x+24, y+self.ih//2+28, f"{err['code']}: {err['message']}"[:85])
            self.text(dx, y+30, "NO IMAGE DATA", "amber")
            self.text(dx, y+58, "Capture is not fabricated.", "muted")
        fy = y+self.ih+40
        for i, item in enumerate(("1 Detection", "2 Empty result", "3 Unavailable", "q Quit")):
            self.text(30+i*205, fy, item, "cyan" if i == 0 else "text")
        self.text(30, fy+28, "SYNTHETIC FIXTURE DATA - NOT A CAMERA PREVIEW", "amber")
        self.x.XFlush(self.dpy)

    def run(self):
        def load_case(case_id):
            case = load_replay_case(PKG/"fixtures"/"replay_manifest.json", case_id)
            return case.display_name, case.response, case.shapes

        name, response, shapes = load_case("detection")
        self.draw(response, name, shapes)
        keymap = {ord("1"): "1", ord("2"): "2", ord("3"): "3", ord("q"): "q", ord("Q"): "q"}
        try:
            while True:
                event = XEvent()
                self.x.XNextEvent(self.dpy, C.byref(event))
                if event.type == 12:
                    self.draw(response, name, shapes)
                elif event.type == 2:
                    key = keymap.get(int(self.x.XLookupKeysym(C.byref(event.key), 0)))
                    if key == "q":
                        break
                    if key in FIXTURE_KEYS:
                        name, response, shapes = load_case(FIXTURE_KEYS[key])
                        self.draw(response, name, shapes)
                elif event.type == CLIENT_MESSAGE and int(event.client.data[0]) == self.delete_atom:
                    break
        finally:
            self.x.XFreeGC(self.dpy, self.gc)
            self.x.XDestroyWindow(self.dpy, self.win)
            self.x.XCloseDisplay(self.dpy)


if __name__ == "__main__":
    Monitor().run()
