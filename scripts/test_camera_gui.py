#!/usr/bin/env python3
"""ROS-free tests: python3 -B scripts/test_camera_gui.py (no helper required)."""

import builtins
import contextlib
import copy
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock
import zlib


spec = importlib.util.spec_from_file_location(
    "camera_gui_under_test", Path(__file__).with_name("verify_camera_gui.py"))
probe = importlib.util.module_from_spec(spec)
with mock.patch.object(sys, "dont_write_bytecode", True):
    spec.loader.exec_module(probe)


def header(seconds=1, frame="camera_link"):
    return NS(frame_id=frame, stamp=NS(sec=seconds, nanosec=0))


def image(rgb=None, seconds=1, padding=0):
    row = (bytes((70, 90, 110))*640 if rgb is None else rgb[:640*3])
    data = (row + b"\xff"*padding)*480 if rgb is None else b"".join(
        rgb[y*640*3:(y+1)*640*3] + b"\xff"*padding for y in range(480))
    return NS(width=640, height=480, step=1920+padding, encoding="rgb8", data=data,
              header=header(seconds))


def info(seconds=1):
    return NS(width=640, height=480, header=header(seconds),
              k=[554.383, 0, 320, 0, 554.383, 240, 0, 0, 1],
              p=[554.383, 0, 320, 0, 0, 554.383, 240, 0, 0, 0, 1, 0],
              d=[0]*5, distortion_model="plumb_bob")


def frames(pixels=1000, red_pixels=100):
    # Both scenes have a nonuniform background; only the red rectangle changes.
    outside = bytes((70, 90, 110))*(pixels//2) + bytes((100, 120, 140))*(pixels-pixels//2)
    original = bytes((200, 20, 20))*red_pixels + outside[red_pixels*3:]
    return original, outside


def result(text="data: true\n", code=0):
    return NS(stdout=text, stderr="", returncode=code)


def chunk(kind, payload):
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind+payload)&0xffffffff)


def make_png(width=4, height=5, color=2, mode=0, solid=False, transparent=False):
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[color]
    previous = [0]*(width*channels)
    rows = bytearray()
    for y in range(height):
        row = []
        for x in range(width):
            level = 200 if solid else 20+(x+y)*20
            if color in (0, 3):
                row += [level if color == 0 else (0 if solid else (x+y)%2)]
            elif color == 4:
                row += [level, 0 if transparent else 255]
            else:
                row += [level, 30, 40]
                if color == 6:
                    row += [0 if transparent else 255]
        rows.append(mode)
        for i, value in enumerate(row):
            left, up = (row[i-channels] if i >= channels else 0), previous[i]
            corner = previous[i-channels] if i >= channels else 0
            if mode == 4:
                p = left+up-corner
                choices = sorted(enumerate((left, up, corner)), key=lambda pair: (abs(p-pair[1]), pair[0]))
                predictor = choices[0][1]
            else:
                predictor = (0, left, up, (left+up)//2)[mode]
            rows.append((value-predictor)&255)
        previous = row
    ihdr = struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0)
    palette = chunk(b"PLTE", bytes((200, 20, 20, 30, 40, 50))) if color == 3 else b""
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + palette + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b"")


class PixelTests(unittest.TestCase):
    def test_solid_red_is_blank_not_two_byte_values(self):
        for pixel in ((200, 20, 20), (0, 0, 0), (255, 255, 255)):
            with self.subTest(pixel=pixel), self.assertRaisesRegex(ValueError, "uniform RGB"):
                probe.rgb_statistics(bytes(pixel)*100)

    def test_padding_not_part_of_pixel_content_or_fingerprint(self):
        a, _ = frames(640*480)
        plain, padded = image(a), image(a, padding=7)
        self.assertEqual(probe.normalize_rgb(plain), a)
        self.assertEqual(probe.normalize_rgb(padded), a)
        self.assertEqual(probe.rgb_statistics(probe.normalize_rgb(padded))["sha256"],
                         probe.rgb_statistics(a)["sha256"])

    def test_bad_image_shape_encoding_step_length_frame(self):
        for key, value in (("width", 639), ("height", 481), ("encoding", "bgr8"),
                           ("step", 1919), ("data", b"")):
            msg = image()
            setattr(msg, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                probe.normalize_rgb(msg)
        msg = image()
        msg.header.frame_id = ""
        with self.assertRaises(ValueError):
            probe.normalize_rgb(msg)

    def test_aba_all_pairs_spatial_change_mean_red_drop_return(self):
        a, b = frames()
        evidence = probe.camera_evidence([a]*3, [b]*3, [a]*3)
        self.assertTrue(evidence["outward_pass"])
        self.assertTrue(evidence["return_pass"])
        self.assertEqual(len(evidence["outward"]), 9)
        self.assertAlmostEqual(evidence["outward"][0]["changed_ratio"], 0.1)
        self.assertEqual(evidence["red_decrease_ratio"], 1)

    def test_hash_difference_and_byte_diversity_never_suffice(self):
        a, _ = frames()
        noisy = bytes(min(v+1, 255) for v in a)
        evidence = probe.camera_evidence([a]*3, [noisy]*3)
        self.assertFalse(evidence["outward_pass"])
        self.assertNotEqual(probe.rgb_statistics(a)["sha256"], probe.rgb_statistics(noisy)["sha256"])
        self.assertEqual(evidence["outward"][0]["changed_ratio"], 0)

    def test_changed_ratio_and_mean_are_independent_requirements(self):
        a, _ = frames(red_pixels=10)
        b = bytes((a[i]+20 if i < 30 and i % 3 == 2 else a[i]) for i in range(len(a)))
        stats = probe.compare_rgb(a, b)
        self.assertEqual(stats["changed_ratio"], 0.01)
        self.assertLess(stats["mean_absolute_rgb_difference"], 1)
        a, b = frames(red_pixels=9)
        self.assertFalse(probe.camera_evidence([a]*3, [b]*3)["outward_pass"])

    def test_red_threshold_and_return_noise_margin(self):
        a, b = frames()
        still_red = bytes((180, 0, 0))*100 + b[300:]
        self.assertFalse(probe.camera_evidence([a]*3, [still_red]*3)["outward_pass"])
        for changed, expected in ((50, True), (51, False)):
            returned = bytes((0, 200, 0))*changed + a[changed*3:]
            self.assertEqual(probe.camera_evidence([a]*3, [b]*3, [returned]*3)["return_pass"], expected)

    def test_insufficient_frames_and_mismatched_sizes(self):
        a, b = frames()
        with self.assertRaisesRegex(ValueError, "three"):
            probe.camera_evidence([a]*2, [b]*3)
        with self.assertRaises(ValueError):
            probe.compare_rgb(a, b[:-3])

    def test_rgb_cpu_work_respects_stage_deadline(self):
        a, b = frames()
        with mock.patch.object(probe.time, "monotonic", return_value=31):
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                probe.camera_evidence([a]*3, [b]*3, deadline=30)
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                probe.compare_rgb(a, b, deadline=30)

    def test_info_matching_valid_and_invalid_intrinsics(self):
        self.assertEqual(probe.validate_info(info(), "camera_link")["width"], 640)
        for change in (lambda x: setattr(x, "width", 639), lambda x: setattr(x.header, "frame_id", "other"),
                       lambda x: x.k.__setitem__(0, 0), lambda x: x.k.__setitem__(4, math.nan),
                       lambda x: x.k.__setitem__(8, 0), lambda x: x.p.__setitem__(0, 1)):
            msg = info()
            change(msg)
            with self.assertRaises(ValueError):
                probe.validate_info(msg, "camera_link")


class PngTests(unittest.TestCase):
    def parse(self, content):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"native.png"
            path.write_bytes(content)
            return probe.inspect_png(path)

    def test_all_five_filters_and_8bit_color_types(self):
        for color in (0, 2, 3, 4, 6):
            fingerprints = []
            for mode in range(5):
                with self.subTest(color=color, mode=mode):
                    stats = self.parse(make_png(color=color, mode=mode))
                    self.assertEqual((stats["width"], stats["height"]), (4, 5))
                    self.assertGreaterEqual(stats["rgb_colors_capped_256"], 2)
                    fingerprints.append(stats["sha256"])
            self.assertEqual(len(set(fingerprints)), 1)

    def test_uniform_and_invisible_pixels_are_not_nonblank(self):
        for content in (make_png(solid=True), make_png(color=6, transparent=True)):
            with self.assertRaisesRegex(ValueError, "uniform"):
                self.parse(content)

    def test_png_deadline_and_symlinks_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"valid.png"
            path.write_bytes(make_png())
            with mock.patch.object(probe.time, "monotonic", return_value=31), \
                    self.assertRaisesRegex(RuntimeError, "deadline"):
                probe.inspect_png(path, 30)
            link = Path(tmp)/"not-new.png"
            link.symlink_to(path)
            with self.assertRaises(ValueError):
                probe.inspect_png(link)

    def test_truncated_crc_signature_trailing_and_missing_idat(self):
        png = make_png()
        badcrc = bytearray(png)
        badcrc[20] ^= 1
        for data in (b"", b"not png", png[:-1], bytes(badcrc), png+b"junk", png[:33]+chunk(b"IEND", b"")):
            with self.subTest(length=len(data)), self.assertRaises(ValueError):
                self.parse(data)

    def test_oversized_interlaced_wrong_depth_and_decompression_bomb(self):
        for width, height, depth, interlace in ((100000, 100000, 8, 0), (4, 5, 16, 0), (4, 5, 8, 1)):
            data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, depth, 2, 0, 0, interlace))
            data += chunk(b"IDAT", zlib.compress(b"x"*100000)) + chunk(b"IEND", b"")
            with self.assertRaises(ValueError):
                self.parse(data)
        data = make_png()[:33] + chunk(b"IDAT", zlib.compress(b"x"*100000)) + chunk(b"IEND", b"")
        with self.assertRaisesRegex(ValueError, "length/stream"):
            self.parse(data)


class CommandTests(unittest.TestCase):
    def test_boolean_exact_true_only_no_trick_or_nonzero(self):
        for text in ("data: true", "\ndata: true\n", " data : true  "):
            probe.require_boolean(result(text))
        for text in ("data: false", "", 'data: "true"', "data: trueish", "error data: true",
                     "data: false\ndata: true", "data: true\nfailed", '# data: true', 'data: "data: true"'):
            with self.subTest(text=text), self.assertRaises(RuntimeError):
                probe.require_boolean(result(text))
        with self.assertRaises(RuntimeError):
            probe.require_boolean(result(code=1))

    def test_process_timeout_and_pose_request_types(self):
        with mock.patch.object(probe.time, "monotonic", return_value=5), \
                mock.patch.object(probe.subprocess, "run", return_value=result()) as run:
            probe.service_call(probe.POSE_SERVICE, "ignition.msgs.Pose", 'name: "test_obstacle"', 20)
            command = run.call_args.args[0]
            self.assertEqual(command[command.index("--timeout")+1], "2000")
            self.assertEqual(command[command.index("--reptype")+1], "ignition.msgs.Boolean")
            self.assertEqual(run.call_args.kwargs["timeout"], 3)
            probe.bounded_command(["ign"], 6)
            self.assertEqual(run.call_args.kwargs["timeout"], 1)
            with self.assertRaises(RuntimeError):
                probe.bounded_command(["ign"], 5)
        with mock.patch.object(probe.subprocess, "run", side_effect=subprocess.TimeoutExpired("ign", 3)):
            with self.assertRaises(subprocess.TimeoutExpired):
                probe.bounded_command(["ign"], probe.time.monotonic()+10)

    def test_inspection_uses_actual_provider_request_response_order(self):
        for text, expected in (("Service providers [Address, Request Message Type, Response Message Type]:\n"
                                "tcp://127.0.0.1:4000, ignition.msgs.Pose, ignition.msgs.Boolean", True),
                               ("ignition.msgs.Boolean ignition.msgs.Pose", False),
                               ("ignition.msgs.StringMsg ignition.msgs.Boolean", False), ("", False)):
            report = {}
            with mock.patch.object(probe, "bounded_command", return_value=result(text)) as run:
                if expected:
                    probe.inspect_service(probe.POSE_SERVICE, "ignition.msgs.Pose", 30, report)
                else:
                    with self.assertRaises(RuntimeError):
                        probe.inspect_service(probe.POSE_SERVICE, "ignition.msgs.Pose", 30, report)
                self.assertEqual(run.call_args.args[0], ["ign", "service", "-i", "-s", probe.POSE_SERVICE])
                self.assertEqual(report["service_inspection"][0]["pass"], expected)

    def test_observed_static_pose_json_not_name_prefix_or_launch_fallback(self):
        entry = {"name": "test_obstacle", "position": {"x": 2, "z": 0.5}, "orientation": {"w": 1}}
        self.assertEqual(probe.parse_obstacle_pose(json.dumps({"pose": [entry]})), probe.ORIGINAL)
        moved = {"name": "test_obstacle", "position": {"x": 2, "y": 3, "z": 0.5},
                 "orientation": {"w": 1}}
        stream = json.dumps({"pose": [entry]}) + "\n" + json.dumps({"pose": [moved]})
        self.assertEqual(probe.parse_obstacle_pose(stream), probe.OUTSIDE)
        for entries in ([], [entry, entry], [{**entry, "name": "test_obstacle::box_link"}],
                        [{**entry, "orientation": {}}], [{**entry, "position": {"x": math.nan}}]):
            with self.assertRaises(ValueError):
                probe.parse_obstacle_pose(json.dumps({"pose": entries}))
        for stream in (json.dumps({"pose": [entry]}) + "\nnot-json",
                       json.dumps({"pose": [entry]}) + "\n" + json.dumps({"pose": []})):
            with self.subTest(stream=stream), self.assertRaises(ValueError):
                probe.parse_obstacle_pose(stream)
        with mock.patch.object(probe, "bounded_command", return_value=result(json.dumps({"pose": [entry]}))) as run:
            self.assertEqual(probe.read_obstacle_pose(30), probe.ORIGINAL)
            self.assertIn("--json-output", run.call_args.args[0])
            self.assertIn("/world/fortress_test/pose/info", run.call_args.args[0])


ENV = {"ROS_DOMAIN_ID": "42", "IGN_PARTITION": "linobot2_g1_test_123"}


def process(pid, ppid, argv, starttime=100):
    return {"pid": pid, "ppid": ppid, "argv": argv, "starttime": starttime,
            "alive": True, "uid": os.getuid(), "domain": "42", "partition": ENV["IGN_PARTITION"]}


def tree():
    return {10: process(10, 1, ["python3", "/scripts/supervised_process.py"]),
            11: process(11, 10, ["ros2", "launch"]),
            12: process(12, 11, ["ign", "gazebo", "server", "-s"]),
            13: process(13, 11, ["ign", "gazebo", "gui", "-g"]),
            14: process(14, 11, ["/opt/ros/humble/lib/rviz2/rviz2"]),
            15: process(15, 11, ["parameter_bridge"])}


def windows():
    return [{"kind": kind, "window_id": pid*100, "pid": pid, "starttime": 100,
             "width": 800, "height": 600, "name": kind, "protocol_ping": True}
            for kind, pid in (("gazebo", 13), ("rviz", 14))]


class ProcessRoleTests(unittest.TestCase):
    def test_exact_rewritten_titles_and_original_argv(self):
        for argv, expected in ((["ign gazebo server"], "server"),
                               (["ign gazebo gui"], "gazebo"),
                               (["ign", "gazebo", "server", "-s"], "server"),
                               (["ign", "gazebo", "gui", "-g"], "gazebo"),
                               (["/usr/bin/ign", "gazebo", "-s", "world.sdf"], "server"),
                               (["/usr/bin/ign", "gazebo", "-g", "world.sdf"], "gazebo"),
                               (["ignition-gazebo6", "--server-only"], "server"),
                               (["ignition-gazebo6", "--gui-only"], "gazebo")):
            with self.subTest(argv=argv):
                self.assertEqual(probe.process_role(process(12, 11, argv)), expected)

    def test_actual_mapped_titles_and_original_ruby_gui_argv(self):
        world = "/home/toto/linobot2_sim/install/linorobot2_gazebo/share/linorobot2_gazebo/worlds/fortress_test.sdf"
        cases = [(["ign gazebo -g"], "gazebo"),
                 (["ruby", "/usr/bin/ign", "gazebo", "-g", "-v", "3"], "gazebo")]
        for version in range(5):
            cases += [([f"ign gazebo -g -v {version}"], "gazebo"),
                      ([f"ign gazebo -s -r {world} -v {version}"], "server")]
        for argv, expected in cases:
            with self.subTest(argv=argv):
                self.assertEqual(probe.process_role(process(12, 11, argv)), expected)

    def test_actual_titles_reject_nonexact_and_malformed_forms(self):
        server = "ign gazebo -s -r /world.sdf -v 3"
        gui = "ign gazebo -g -v 3"
        for title in (server, gui):
            cases = [["sh -c " + title], ["sh", "-c", title],
                     ["prefix " + title], [title + " suffix"], [title + "*"],
                     ["*" + title], [" " + title], [title + " "], [title + "\n"],
                     [title, "-s"], [title, "-g"]]
            cases += [[title + operator + "echo"] for operator in
                      (";", "&&", "|", "`", "$", "<", ">")]
            for argv in cases:
                with self.subTest(argv=argv):
                    self.assertIsNone(probe.process_role(process(12, 11, argv)))
        titles = [prefix + version for prefix in
                  ("ign gazebo -g -v ", "ign gazebo -s -r /world.sdf -v ")
                  for version in ("", "-1", "5", "03", "3.0", "x", "*", "3 -v 3")]
        titles += ["ign gazebo -g -s", "ign gazebo -gg -v 3",
                   "ign gazebo --gui-only -v 3", "ign gazebo -G -v 3",
                   "ign gazebo -s -r /world.sdf", "ign gazebo -g -r /world.sdf -v 3",
                   "ign gazebo -s /world.sdf -v 3", "ign gazebo -s -r world.sdf -v 3",
                   "ign gazebo -s -r / -v 3", "ign gazebo -s -r /two words.sdf -v 3"]
        titles += [f"ign gazebo -s -r /world{operator}.sdf -v 3" for operator in
                   (";", "&", "|", "`", "$", "<", ">")]
        for title in titles:
            with self.subTest(title=title):
                self.assertIsNone(probe.process_role(process(12, 11, [title])))

    def test_rewritten_titles_reject_shell_prefix_suffix_wildcard_and_extra_argv(self):
        for title in ("ign gazebo server", "ign gazebo gui"):
            for argv in (["sh -c " + title], ["sh", "-c", title],
                         ["prefix " + title], [title + "-helper"],
                         [title + "*"], ["*" + title],
                         [" " + title], [title + " "], [title + " -g"],
                         [title, "-s"], [title, "-g"]):
                with self.subTest(argv=argv):
                    self.assertIsNone(probe.process_role(process(12, 11, argv)))


class OwnershipGuiTests(unittest.TestCase):
    def setUp(self):
        self.table = tree()
        for patch in (mock.patch.dict(os.environ, ENV, clear=True),
                      mock.patch.object(probe, "process_snapshot", side_effect=lambda: self.table)):
            patch.start()
            self.addCleanup(patch.stop)
        self.guard = probe.OwnershipGuard(10)

    def test_connected_descendants_and_server_are_owned(self):
        self.assertEqual(set(self.guard.check()), set(self.table))
        self.assertEqual(self.guard.server_identity, (12, 100))

    def test_rewritten_server_and_gui_titles_owned_inventory_and_ping(self):
        self.table[12]["argv"] = ["ign gazebo server"]
        self.table[13]["argv"] = ["ign gazebo gui"]
        self.assertEqual(set(self.guard.check()), set(self.table))
        self.assertEqual(self.guard.server_identity, (12, 100))
        gui = self.gui()
        evidence = gui.inventory(probe.time.monotonic()+10, ping=True)
        self.assertTrue(evidence["pass"])
        self.assertEqual(evidence["server_identity"], (12, 100))
        self.assertEqual([w["pid"] for w in evidence["windows"]], [13, 14])
        self.assertEqual(gui.probe.ping.call_count, 2)
        self.table[12]["ppid"] = 1
        with self.assertRaisesRegex(RuntimeError, "external"):
            self.guard.check()

    def test_external_rewritten_gui_title_never_grants_window_ownership(self):
        self.table[12]["argv"] = ["ign gazebo server"]
        self.table[13]["argv"] = ["ign gazebo gui"]
        self.table[30] = process(30, 1, ["ign gazebo gui"])
        with self.assertRaisesRegex(RuntimeError, "external"):
            self.guard.check()
        self.table[30]["partition"] = "someone_else"
        self.assertNotIn(30, self.guard.check())
        gui = self.gui()
        external_windows = windows()
        external_windows[0]["pid"] = 30
        gui.probe.find_owned_windows.side_effect = lambda pid: external_windows
        with self.assertRaisesRegex(RuntimeError, "ownership invalid"):
            gui.inventory(probe.time.monotonic()+10, ping=True)
        gui.probe.ping.assert_not_called()

    def test_actual_server_title_and_gui_forms_owned_inventory_and_ping(self):
        for argv in (["ign gazebo -g -v 3"],
                     ["ruby", "/usr/bin/ign", "gazebo", "-g", "-v", "3"]):
            with self.subTest(argv=argv):
                self.table = tree()
                self.table[12]["argv"] = ["ign gazebo -s -r /home/toto/linobot2_sim/install/linorobot2_gazebo/share/linorobot2_gazebo/worlds/fortress_test.sdf -v 3"]
                self.table[13]["argv"] = argv
                self.assertEqual(set(self.guard.check()), set(self.table))
                self.assertEqual(self.guard.server_identity, (12, 100))
                gui = self.gui()
                evidence = gui.inventory(probe.time.monotonic()+10, ping=True)
                self.assertTrue(evidence["pass"])
                self.assertEqual(evidence["server_identity"], (12, 100))
                self.assertEqual([w["pid"] for w in evidence["windows"]], [13, 14])
                self.assertEqual(gui.probe.ping.call_count, 2)
                self.table[12]["ppid"] = 1
                with self.assertRaisesRegex(RuntimeError, "external"):
                    self.guard.check()

    def test_actual_titles_never_grant_external_process_or_window_ownership(self):
        for argv in (["ign gazebo -g -v 3"],
                     ["ign gazebo -s -r /world.sdf -v 3"],
                     ["ruby", "/usr/bin/ign", "gazebo", "-g", "-v", "3"]):
            with self.subTest(argv=argv):
                self.table = tree()
                self.table[30] = process(30, 1, argv)
                with self.assertRaisesRegex(RuntimeError, "external"):
                    self.guard.check()
                self.table[30]["partition"] = None
                with self.assertRaisesRegex(RuntimeError, "unverifiable"):
                    self.guard.check()
                self.table[30]["partition"] = "someone_else"
                self.assertNotIn(30, self.guard.check())
                gui = self.gui()
                external_windows = windows()
                external_windows[0]["pid"] = 30
                gui.probe.find_owned_windows.side_effect = lambda pid: external_windows
                with self.assertRaisesRegex(RuntimeError, "ownership invalid"):
                    gui.inventory(probe.time.monotonic()+10, ping=True)
                gui.probe.ping.assert_not_called()

    def test_domain_partition_and_root_guards(self):
        for field, value in (("ROS_DOMAIN_ID", "44"), ("ROS_DOMAIN_ID", "042"),
                             ("IGN_PARTITION", ""), ("IGN_PARTITION", "linobot2_g1_"),
                             ("IGN_PARTITION", "external")):
            with mock.patch.dict(os.environ, {field: value}), self.assertRaises(ValueError):
                self.guard.check()
        for change in (lambda t: t.pop(10), lambda t: t[10].update(alive=False),
                       lambda t: t[10].update(uid=os.getuid()+1), lambda t: t[10].update(argv=["bash"]),
                       lambda t: t[12].update(partition="other"), lambda t: t[15].update(domain="44"),
                       lambda t: t[11].update(partition="other"),
                       lambda t: t[12].update(ppid=1)):
            self.table = tree()
            change(self.table)
            with self.assertRaises(RuntimeError):
                probe.OwnershipGuard(10).check()

    def test_partition_prefix_is_configurable_for_isolated_phase_harnesses(self):
        partition = "linobot2_g2_test_123"
        for entry in self.table.values():
            entry["partition"] = partition
        with mock.patch.dict(os.environ, {"IGN_PARTITION": partition}):
            guard = probe.OwnershipGuard(10, partition_prefix="linobot2_g2_")
            self.assertEqual(set(guard.check()), set(self.table))
            with self.assertRaisesRegex(ValueError, "linobot2_g1_"):
                probe.OwnershipGuard(10).check()
        for prefix in ("", "external_", "linobot2_G2_"):
            with self.subTest(prefix=prefix), self.assertRaisesRegex(ValueError, "prefix"):
                probe.OwnershipGuard(10, partition_prefix=prefix)
        with self.assertRaisesRegex(ValueError, "prefix"):
            probe.OwnershipGuard(10, partition_prefix="linorobot2_g4_")
        self.assertEqual(probe.OwnershipGuard(10, partition_prefix="linobot2_g4_").partition_prefix,
                         "linobot2_g4_")

    def test_external_same_partition_rejected_other_partition_not_attached(self):
        self.table[30] = process(30, 1, ["ign", "gazebo", "server"])
        with self.assertRaisesRegex(RuntimeError, "external"):
            self.guard.check()
        self.table[30]["partition"] = "someone_else"
        self.assertNotIn(30, self.guard.check())
        self.table[30]["partition"] = None
        with self.assertRaisesRegex(RuntimeError, "unverifiable"):
            self.guard.check()

    def test_server_and_supervisor_pid_reuse_or_restart_rejected(self):
        for pid in (10, 12):
            self.table = tree()
            guard = probe.OwnershipGuard(10)
            guard.check()
            self.table[pid]["starttime"] += 1
            with self.assertRaisesRegex(RuntimeError, "changed"):
                guard.check()

    def gui(self, report=None, directory=Path("/unused")):
        helper = NS(find_owned_windows=mock.Mock(side_effect=lambda pid: windows()),
                    ping=mock.Mock(return_value={"pass": True}))
        return probe.GuiChecks(helper, self.guard, report if report is not None else {}, directory, lambda: None)

    def test_inventory_ping_roles_identity_and_dead_windows(self):
        gui = self.gui()
        evidence = gui.inventory(probe.time.monotonic()+10, ping=True)
        self.assertTrue(evidence["pass"])
        self.assertEqual(gui.probe.ping.call_count, 2)
        for call in gui.probe.ping.call_args_list:
            self.assertLessEqual(call.kwargs["timeout"], 2)
        self.table[14]["alive"] = False
        with self.assertRaises(RuntimeError):
            gui.inventory(probe.time.monotonic()+10)

    def test_missing_role_mismatched_pid_no_ping_and_helper_failure(self):
        for change in (lambda w: w.pop(), lambda w: w[0].update(pid=12),
                       lambda w: w[0].update(starttime=200), lambda w: w[0].update(protocol_ping=False),
                       lambda w: w[0].update(width=0)):
            gui = self.gui()
            owned_windows = windows()
            change(owned_windows)
            gui.probe.find_owned_windows.side_effect = lambda pid: owned_windows
            with self.assertRaises(RuntimeError):
                gui.inventory(probe.time.monotonic()+10, ping=True)
        gui = self.gui()
        gui.probe.ping.side_effect = RuntimeError("X11 response timeout")
        with self.assertRaisesRegex(RuntimeError, "X11 response timeout"):
            gui.inventory(probe.time.monotonic()+10, ping=True)

    def test_fresh_native_screenshot_and_exact_world_reply(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = {}
            gui = self.gui(report, Path(tmp))

            def screenshot(service, reqtype, request, deadline):
                self.assertEqual(service, "/gui/screenshot")
                self.assertEqual(reqtype, "ignition.msgs.StringMsg")
                directory = Path(json.loads(request.split(":", 1)[1]))
                self.assertTrue(directory.is_dir())
                self.assertEqual(list(directory.iterdir()), [])
                path = directory / "2026-10-06T10:00:00.png"
                path.write_bytes(make_png())
                return {"reply": "data: true"}

            with mock.patch.object(probe, "bounded_command", return_value=result('data: "fortress_test"\n')), \
                    mock.patch.object(probe, "service_call", side_effect=screenshot):
                gui.full(probe.time.monotonic()+10)
                gui.full(probe.time.monotonic()+10)
            paths = [e["screenshot"]["path"] for e in report["gui_checks"]]
            self.assertEqual(len(set(paths)), 2)
            self.assertTrue(all(e["pass"] for e in report["gui_checks"]))
            for text in ('data: "external"', 'data: "fortress_test"\ndata: "another"'):
                with mock.patch.object(probe, "bounded_command", return_value=result(text)), \
                        self.assertRaisesRegex(RuntimeError, "exactly fortress_test"):
                    gui.full(probe.time.monotonic()+10)

    def test_multiple_screenshot_outputs_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            gui = self.gui(directory=Path(tmp))

            def screenshot(service, reqtype, request, deadline):
                directory = Path(json.loads(request.split(":", 1)[1]))
                for name in ("first.png", "second.png"):
                    (directory / name).write_bytes(make_png())
                return {"reply": "data: true"}

            with mock.patch.object(probe, "bounded_command", return_value=result('data: "fortress_test"\n')), \
                    mock.patch.object(probe, "service_call", side_effect=screenshot), \
                    self.assertRaisesRegex(RuntimeError, "multiple PNGs"):
                gui.full(probe.time.monotonic()+10)


class SensorTests(unittest.TestCase):
    def ready(self):
        state = probe.SensorState()
        for seconds in (1, 2):
            state.receive("clock", NS(clock=header(seconds).stamp))
            for key in ("scan", "imu"):
                state.receive(key, NS(header=header(seconds, key)))
            state.receive("image", image(seconds=seconds))
            state.receive("info", info(seconds))
        return state

    def test_clock_progress_all_five_fresh_and_metadata(self):
        state = self.ready()
        state.health()
        self.assertEqual(state.clock_advances, 1)
        self.assertEqual(set(state.received), set(probe.TOPICS))

    def test_rewind_repeated_image_and_old_timestamp_do_not_pass(self):
        for key, msg in (("clock", NS(clock=header(1).stamp)), ("image", image(seconds=2))):
            state = self.ready()
            state.receive(key, msg)
            with self.assertRaises(ValueError):
                state.health()
        state = self.ready()
        state.received["scan"] -= 11
        state.receive("scan", NS(header=header(2)))
        with self.assertRaisesRegex(ValueError, "freshness"):
            state.health()
        state = self.ready()
        state.clock_progress_at -= 11
        with self.assertRaisesRegex(ValueError, "freshness"):
            state.health()

    def test_cutoff_requires_strictly_later_than_point_two_sim_seconds(self):
        state = self.ready()
        state.cutoff = 2_000_000_000 + probe.SETTLE_NS
        a, _ = frames(640*480)
        for ns in (100_000_000, 200_000_000, 300_000_000, 400_000_000, 500_000_000, 600_000_000):
            msg = image(a, seconds=2)
            msg.header.stamp.nanosec = ns
            state.receive("image", msg)
        self.assertIsNone(state.error)
        self.assertEqual([x["stamp_ns"] for x in state.images], [2_300_000_000, 2_400_000_000, 2_500_000_000])

    def test_invalid_stamp_and_increasing(self):
        for seconds, ns in ((-1, 0), (0, 1_000_000_000)):
            with self.assertRaises(ValueError):
                probe.stamp_ns(NS(sec=seconds, nanosec=ns))
        for current in (1, 0):
            with self.assertRaises(ValueError):
                probe.require_increasing(1, current)


class AcquisitionTests(unittest.TestCase):
    def test_drains_queued_images_then_collects_three_after_observed_clock_plus_settle(self):
        now, ticks, drains, report = [0.0], [20], [0], {}
        a, _ = frames(640*480)
        with mock.patch.object(probe.time, "monotonic", side_effect=lambda: now[0]):
            state = SensorTests().ready()

            def feed():
                ticks[0] += 1
                sec, ns = divmod(ticks[0]*100_000_000, 1_000_000_000)
                stamp = NS(sec=sec, nanosec=ns)
                state.receive("clock", NS(clock=stamp))
                for key in ("scan", "imu"):
                    state.receive(key, NS(header=NS(stamp=stamp, frame_id=key)))
                meta, msg = info(sec), image(a, sec)
                meta.header.stamp = msg.header.stamp = stamp
                state.receive("info", meta)
                state.receive("image", msg)

            def spin(node, timeout_sec):
                if timeout_sec == 0:
                    drains[0] += 1
                    if drains[0] <= 5:
                        self.assertIsNone(state.cutoff)
                        feed()
                else:
                    now[0] += 0.1
                    feed()

            ros = NS(ok=lambda: True, spin_once=spin)
            runtime = probe.RosRuntime(ros, None, state, None, NS(inventory=mock.Mock()),
                                       report, lambda: None, 60)
            images = runtime.acquire("outside", 2_000_000_000)
        self.assertEqual(drains[0], 40)
        self.assertEqual(images, [a]*3)
        evidence = report["camera_states"][0]
        self.assertEqual(evidence["minimum_stamp_exclusive_ns"], 2_700_000_000)
        self.assertEqual([x["stamp_ns"] for x in evidence["samples"]],
                         [2_800_000_000, 2_900_000_000, 3_000_000_000])
        self.assertIsNone(state.cutoff)
        self.assertFalse(state.images)
        self.assertNotIn('"rgb":', json.dumps(report))

    def test_no_frames_times_out_within_wall_stage_and_clears_collection(self):
        now = [0.0]
        with mock.patch.object(probe.time, "monotonic", side_effect=lambda: now[0]):
            state = probe.SensorState()
            state.clock_ns = 2_000_000_000
            state.health = mock.Mock()
            ros = NS(ok=lambda: True, spin_once=lambda node, timeout_sec: now.__setitem__(
                0, now[0]+(1 if timeout_sec else 0)))
            runtime = probe.RosRuntime(ros, None, state, None, NS(inventory=mock.Mock()),
                                       {}, lambda: None, 60)
            with self.assertRaisesRegex(RuntimeError, "deadline"):
                runtime.acquire("no frames", 2_000_000_000)
        self.assertEqual(now[0], 30)
        self.assertIsNone(state.cutoff)
        self.assertFalse(state.images)

    def test_ongoing_inventory_every_five_seconds_rejects_dead_window(self):
        now, checks = [0.0], []
        with mock.patch.object(probe.time, "monotonic", side_effect=lambda: now[0]):
            ros = NS(ok=lambda: True, spin_once=lambda node, timeout_sec: now.__setitem__(0, now[0]+1))
            state = NS(error=None, health=lambda: None)
            gui = NS(inventory=lambda deadline: checks.append(now[0]))
            runtime = probe.RosRuntime(ros, None, state, None, gui, {}, lambda: None, 60)
            for _ in range(12):
                runtime.spin(60)
            self.assertEqual(checks, [5, 10])
            gui.inventory = mock.Mock(side_effect=RuntimeError("dead GUI window"))
            with self.assertRaisesRegex(RuntimeError, "dead GUI"):
                for _ in range(5):
                    runtime.spin(60)
        self.assertEqual(now[0], 15)


class OrchestrationTests(unittest.TestCase):
    def runtime(self):
        a, b = frames()
        return NS(observe=mock.Mock(return_value=copy.deepcopy(probe.ORIGINAL)),
                  state=NS(clock_ns=1_000_000_000),
                  acquire=mock.Mock(side_effect=[[a]*3, [b]*3, [a]*3]), set_pose=mock.Mock(),
                  stage_deadline=lambda: probe.time.monotonic()+30)

    def test_aba_restores_observed_original_before_return_images(self):
        runtime, report = self.runtime(), {}
        probe.run_camera_sequence(runtime, report, lambda: None)
        self.assertTrue(report["restore"]["pass"])
        self.assertTrue(report["camera_comparison"]["return_pass"])
        self.assertEqual(runtime.set_pose.call_args_list, [mock.call(probe.OUTSIDE),
                         mock.call(report["observed_original_pose"], restoring=True)])

    def test_failure_false_write_timeout_and_keyboard_interrupt_always_restore(self):
        for failure in (ValueError("bad images"), subprocess.TimeoutExpired("ign", 3), KeyboardInterrupt()):
            for where in ("write", "acquisition"):
                runtime, report = self.runtime(), {}
                if where == "write":
                    runtime.set_pose.side_effect = [failure, None]
                else:
                    runtime.acquire.side_effect = [runtime.acquire.side_effect.__next__(), failure]
                with self.subTest(failure=type(failure).__name__, where=where), self.assertRaises(type(failure)):
                    probe.run_camera_sequence(runtime, report, lambda: None)
                self.assertTrue(report["restore"]["pass"])
                self.assertEqual(runtime.set_pose.call_args_list[-1], mock.call(probe.ORIGINAL, restoring=True))

    def test_restore_failure_marks_partial_report_fail(self):
        runtime, report = self.runtime(), {}
        runtime.set_pose.side_effect = [None, RuntimeError("lost world")]
        with self.assertRaisesRegex(RuntimeError, "restoration failed"):
            probe.run_camera_sequence(runtime, report, lambda: None)
        self.assertFalse(report["restore"]["pass"])
        self.assertIn("lost world", report["restore"]["error"])

    def test_report_failure_cannot_block_restoration(self):
        runtime, report = self.runtime(), {}
        a, b = frames()
        runtime.acquire.side_effect = [[a]*3, [b]*3]
        persist = mock.Mock(side_effect=[None, OSError("report disk full"), OSError("report disk full")])
        with self.assertRaises(OSError):
            probe.run_camera_sequence(runtime, report, persist)
        self.assertEqual(runtime.set_pose.call_args_list[-1], mock.call(probe.ORIGINAL, restoring=True))
        self.assertTrue(report["restore"]["pass"])

        guard = NS(check=lambda: None)
        actual = probe.RosRuntime(None, None, None, guard, None, {},
                                  mock.Mock(side_effect=OSError("disk full")), probe.time.monotonic()+60)
        with mock.patch.object(probe, "service_call", return_value={"reply": "data: true"}) as call, \
                mock.patch.object(probe, "read_obstacle_pose", return_value=probe.ORIGINAL), \
                self.assertRaises(OSError):
            actual.set_pose(probe.ORIGINAL, restoring=True)
        call.assert_called_once()

    def test_wrong_original_refuses_any_pose_write(self):
        runtime, report = self.runtime(), {}
        runtime.observe.return_value["position"] = (2, 1, 0.5)
        with self.assertRaises(ValueError):
            probe.run_camera_sequence(runtime, report, lambda: None)
        runtime.set_pose.assert_not_called()

    def test_actual_set_pose_only_obstacle_and_failed_reply_not_passed(self):
        report = {}
        guard = NS(check=mock.Mock())
        runtime = probe.RosRuntime(None, None, None, guard, None, report, lambda: None,
                                   probe.time.monotonic()+60)
        with mock.patch.object(probe, "service_call", return_value={"reply": "data: true"}) as call, \
                mock.patch.object(probe, "read_obstacle_pose", return_value=probe.OUTSIDE):
            runtime.set_pose(probe.OUTSIDE)
            self.assertEqual(call.call_args.args[:2], (probe.POSE_SERVICE, "ignition.msgs.Pose"))
            self.assertIn('name: "test_obstacle"', call.call_args.args[2])
            self.assertIn("y: 3", call.call_args.args[2])
            self.assertIn("w: 1", call.call_args.args[2])
            self.assertTrue(report["pose_writes"][0]["pass"])
            guard.check.assert_called_once()

    def test_duration_begins_after_initial_gui_and_checks_each_thirty_seconds(self):
        for duration in (0, 900):
            now, checks, report = [100.0], [], {}

            def gui_full(deadline):
                checks.append(now[0])
                now[0] += 2

            def spin(deadline):
                now[0] += 1

            state = NS(health=mock.Mock(), counts={"image": 100}, clock_advances=50)
            runtime = NS(gui=NS(full=gui_full), spin=spin, stage_deadline=lambda: now[0]+30,
                         deadline=2000, state=state)
            with mock.patch.object(probe.time, "monotonic", side_effect=lambda: now[0]):
                probe.run_stability(runtime, duration, report, lambda: None)
            self.assertEqual(report["stability"]["started_monotonic_seconds"], 102)
            self.assertGreaterEqual(report["stability"]["elapsed_seconds"], duration)
            self.assertTrue(report["stability"]["pass"])
            if duration:
                self.assertGreaterEqual(len(checks), 31)
                self.assertTrue(all(b-a <= 32 for a, b in zip(checks, checks[1:])))
            else:
                self.assertEqual(len(checks), 1)


def import_overrides(stubs):
    original = builtins.__import__

    def imported(name, globals=None, locals=None, fromlist=(), level=0):
        if not level and name in stubs:
            return stubs[name]
        return original(name, globals, locals, fromlist, level)

    return mock.patch.object(builtins, "__import__", side_effect=imported)


class MainAndCleanupTests(unittest.TestCase):
    def test_module_import_does_not_import_ros_or_gui_helper(self):
        self.assertNotIn("rclpy", probe.__dict__)
        self.assertNotIn("gui_x11_probe", probe.__dict__)

    def test_process_snapshot_drops_raced_records_and_stores_no_secret_environment(self):
        unstable = NS(pid=10, ppid=1, starttime=100, identity=(10, 100), alive=True)
        stable = NS(pid=20, ppid=1, starttime=200, identity=(20, 200), alive=True)
        reused = NS(pid=10, ppid=1, starttime=101, identity=(10, 101), alive=True)
        helper = NS(process_table=lambda: {10: unstable, 20: stable},
                    read_process=lambda pid: reused if pid == 10 else stable)

        def read(path):
            if path.name == "cmdline":
                return b"python3\0supervised_process.py\0"
            return b"IGN_PARTITION=linobot2_g1_test_123\0ROS_DOMAIN_ID=42\0SECRET=not-for-report\0"

        with import_overrides({"supervised_process": helper}), \
                mock.patch.object(Path, "read_bytes", autospec=True, side_effect=read), \
                mock.patch.object(Path, "stat", autospec=True, return_value=NS(st_uid=os.getuid())):
            snapshot = probe.process_snapshot()
        self.assertEqual(set(snapshot), {20})
        self.assertEqual(snapshot[20]["partition"], ENV["IGN_PARTITION"])
        self.assertNotIn("SECRET", json.dumps(snapshot))
        self.assertNotIn("not-for-report", json.dumps(snapshot))

    def test_process_snapshot_parses_actual_titles_with_nul_padding_and_ruby_argv(self):
        entries = {pid: NS(pid=pid, ppid=11, starttime=100, identity=(pid, 100), alive=True)
                   for pid in (12, 13, 14)}
        server = b"ign gazebo -s -r /home/toto/linobot2_sim/install/linorobot2_gazebo/share/linorobot2_gazebo/worlds/fortress_test.sdf -v 3"
        commands = {12: server + b"\0" * 128,
                    13: b"ign gazebo -g -v 3" + b"\0" * 128,
                    14: b"ruby\0/usr/bin/ign\0gazebo\0-g\0-v\0" + b"3\0"}
        helper = NS(process_table=lambda: entries, read_process=lambda pid: entries[pid])

        def read(path):
            if path.name == "cmdline":
                return commands[int(path.parent.name)]
            return b"IGN_PARTITION=linobot2_g1_test_123\0ROS_DOMAIN_ID=42\0"

        with import_overrides({"supervised_process": helper}), \
                mock.patch.object(Path, "read_bytes", autospec=True, side_effect=read), \
                mock.patch.object(Path, "stat", autospec=True, return_value=NS(st_uid=os.getuid())):
            snapshot = probe.process_snapshot()
        self.assertEqual(set(snapshot), {12, 13, 14})
        self.assertEqual(snapshot[12]["argv"], [server.decode()])
        self.assertEqual(snapshot[13]["argv"], ["ign gazebo -g -v 3"])
        self.assertEqual(snapshot[14]["argv"], ["ruby", "/usr/bin/ign", "gazebo", "-g", "-v", "3"])
        self.assertEqual([probe.process_role(snapshot[pid]) for pid in (12, 13, 14)],
                         ["server", "gazebo", "gazebo"])

    def test_invalid_cli_pid_duration_or_missing_required_report(self):
        for extra in (("--duration", "nan"), ("--duration", "inf"), ("--duration", "-1"),
                      ("--supervisor-pid", "0"), ("--supervisor-pid", "-1")):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                probe.main(["--supervisor-pid", "10", "--report", "unused", *extra])
            self.assertEqual(exc.exception.code, 2)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            probe.main(["--supervisor-pid", "10"])

    def test_inspect_writes_only_json_no_ros_pose_or_screenshots(self):
        helper = NS(find_owned_windows=lambda pid: windows(), ping=lambda window, timeout: {"pass": True})
        context = mock.MagicMock()
        context.__enter__.return_value = helper

        def inspection(command, deadline):
            req = "Pose" if probe.POSE_SERVICE in command else "StringMsg"
            return result(f"Service providers [Address, Request Message Type, Response Message Type]:\n"
                          f"tcp://local, ignition.msgs.{req}, ignition.msgs.Boolean\n")

        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, ENV, clear=True), \
                mock.patch.object(probe, "process_snapshot", side_effect=tree), \
                mock.patch.object(probe, "bounded_command", side_effect=inspection), \
                import_overrides({"gui_x11_probe": NS(X11Probe=lambda: context)}), \
                mock.patch.object(probe, "run_ros") as ros, \
                mock.patch.object(probe, "service_call") as writes, contextlib.redirect_stdout(io.StringIO()):
            path = Path(tmp)/"evidence.json"
            self.assertEqual(probe.main(["--supervisor-pid", "10", "--inspect", "--report", str(path)]), 0)
            report = json.loads(path.read_text())
            self.assertEqual(report["scope"], "INSPECT")
            self.assertTrue(report["pass"])
            self.assertEqual(len(report["service_inspection"]), 2)
            self.assertEqual(list(Path(tmp).iterdir()), [path])
            ros.assert_not_called()
            writes.assert_not_called()

    def test_wrong_domain_fails_before_any_runtime_with_partial_report(self):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, {"ROS_DOMAIN_ID": "44"}, clear=True), \
                mock.patch.object(probe, "process_snapshot") as snapshot, contextlib.redirect_stdout(io.StringIO()):
            path = Path(tmp)/"evidence.json"
            self.assertEqual(probe.main(["--supervisor-pid", "10", "--report", str(path), "--duration", "0"]), 1)
            snapshot.assert_not_called()
            report = json.loads(path.read_text())
            self.assertEqual(report["scope"], "PILOT")
            self.assertEqual(report["status"], "FAIL")
            self.assertIn("ROS_DOMAIN_ID", report["errors"][0])

    def test_cleanup_destroy_and_shutdown_without_poisoning_sys_modules(self):
        for init_failure in (False, True):
            node = NS(create_subscription=mock.Mock(), destroy_node=mock.Mock())
            ros = NS(init=mock.Mock(side_effect=RuntimeError("init") if init_failure else None),
                     ok=lambda: True, shutdown=mock.Mock())
            stubs = {"rclpy": ros, "rclpy.node": NS(Node=lambda *a, **k: node),
                     "rclpy.qos": NS(qos_profile_sensor_data=object()),
                     "rclpy.signals": NS(SignalHandlerOptions=NS(NO=0)),
                     "rosgraph_msgs.msg": NS(Clock=object()),
                     "sensor_msgs.msg": NS(CameraInfo=object(), Image=object(), Imu=object(), LaserScan=object())}
            before = sys.modules.get("rclpy")
            with import_overrides(stubs), mock.patch.object(probe.RosRuntime, "warmup", side_effect=RuntimeError("stop")), \
                    self.assertRaises(RuntimeError):
                probe.run_ros(None, None, {}, 0, lambda: None, 60)
            self.assertIs(sys.modules.get("rclpy"), before)
            if init_failure:
                node.destroy_node.assert_not_called()
                ros.shutdown.assert_not_called()
            else:
                ros.init.assert_called_once_with(args=[], signal_handler_options=0)
                self.assertEqual(node.create_subscription.call_count, 5)
                node.destroy_node.assert_called_once()
                ros.shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()