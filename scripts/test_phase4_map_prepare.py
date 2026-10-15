"""ROS-free tests for the verified-start-corridor map preparation utility."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location(
    "phase4_map_prepare_under_test", Path(__file__).with_name("prepare_phase4_map.py"))
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)
goal_spec = importlib.util.spec_from_file_location(
    "phase4_goal_under_test", Path(__file__).with_name("verify_phase4_goal.py"))
goal_probe = importlib.util.module_from_spec(goal_spec)
goal_spec.loader.exec_module(goal_probe)


class PreparedMapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.width, self.height = 62, 29
        self.resolution = 0.05
        self.origin = (0.02, -0.951, 0.0)
        self.yaml_path = self.root / "raw.yaml"
        self.pgm_path = self.root / "raw.pgm"
        self.mapping_path = self.root / "mapping.json"
        self.output_prefix = self.root / "nav_map"
        pixels = bytearray([prepare.MAP_SAVER_UNKNOWN_PIXEL]*(self.width*self.height))

        def index(x, y):
            col = int((x-self.origin[0])//self.resolution)
            row_from_bottom = int((y-self.origin[1])//self.resolution)
            return (self.height-1-row_from_bottom)*self.width+col

        # SLAM observed the free line from the approach endpoint toward the goal.
        for col in range(4, 29):
            for row_from_bottom in range(7, 21):
                pixels[(self.height-1-row_from_bottom)*self.width+col] = prepare.FREE_PIXEL
        self.box_index = index(1.495, -0.426)
        pixels[self.box_index] = 0
        self.south_raster_obstacle_index = index(2.145, -0.698)
        pixels[self.south_raster_obstacle_index] = 0
        self.pgm_path.write_bytes(f"P5\n{self.width} {self.height}\n255\n".encode()+pixels)
        self.raw_pgm_bytes = self.pgm_path.read_bytes()
        self.yaml_path.write_text(
            "image: raw.pgm\nmode: trinary\nresolution: 0.05\n"
            "origin: [0.02, -0.951, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
        self.mapping_path.write_text(json.dumps({
            "status": "PASS", "scope": "G4_SIMULATION_ONLY",
            "map": {"width": self.width, "height": self.height,
                    "resolution": 0.05, "origin": [0.02, -0.950852, 0.0],
                    "frame_id": "map"},
            "test_box_occupied_cell_near_left_face": {
                "x_cell": 29, "y_cell": 10, "occupancy": 100},
            "segments": [
                {"name": "cover_west_of_start", "pass": True,
                 "min_obstacle_clearance_m": 1.7,
                 "ground_truth_start": {"position": [0.0, 0.0, 0.0]},
                 "ground_truth_end": {"position": [-0.547, 0.0, 0.0]}},
                {"name": "approach_box", "pass": True,
                 "min_obstacle_clearance_m": 0.9,
                 "ground_truth_start": {"position": [-0.547, 0.0, 0.0]},
                 "ground_truth_end": {"position": [0.255, 0.0, 0.0]}},
                {"name": "pass_south_of_box", "pass": True,
                 "min_obstacle_clearance_m": 0.3,
                 "ground_truth_start": {"position": [0.255, 0.0, 0.0]},
                 "ground_truth_end": {"position": [0.255, -1.1, 0.0]},
                 "ground_truth_samples": [
                     {"pose": {"position": [0.255, -0.275*i, 0.0]},
                      "obstacle_clearance_m": 0.3} for i in range(5)]},
                {"name": "map_far_side", "pass": True,
                 "min_obstacle_clearance_m": 0.2,
                 "ground_truth_start": {"position": [0.255, -1.0, 0.0]},
                 "ground_truth_end": {"position": [3.0, -1.0, 0.0]},
                 "ground_truth_samples": [
                     {"pose": {"position": [0.255+0.25*i, -1.0, 0.0]},
                      "obstacle_clearance_m": 0.2} for i in range(12)]}]}))

    def tearDown(self):
        self.temp.cleanup()

    def test_pads_map_and_fills_only_verified_free_corridor(self):
        report = prepare.prepare_map(self.yaml_path, self.mapping_path,
                                     self.output_prefix)
        width, height, pixels = prepare.read_pgm(report["prepared_map_pgm"])
        self.assertEqual((width, height), (self.width+6, self.height+9))
        self.assertAlmostEqual(report["map_origin_after"][0], -0.28)
        self.assertAlmostEqual(report["map_origin_after"][1], -1.401)
        self.assertTrue(report["raw_slam_map_preserved"])
        self.assertEqual(self.pgm_path.read_bytes(), self.raw_pgm_bytes)
        self.assertEqual(report["nav2_unknown_pixel_value"], 127)
        self.assertEqual(report["source_unknown_cells_normalized"],
                         self.width*self.height-25*14-2)
        for x in (0.0, 0.1, 0.25, 0.4, 0.55):
            index = prepare.grid_pixel_index(width, height, self.resolution,
                                             tuple(report["map_origin_after"]), x, 0.0)
            self.assertIsNotNone(index)
            self.assertEqual(prepare.classify_pixel(pixels[index], 0.25, 0.65), 0, x)
        unknown_index = prepare.grid_pixel_index(
            width, height, self.resolution, tuple(report["map_origin_after"]), -0.25, 0.4)
        self.assertEqual(pixels[unknown_index], 127)
        self.assertEqual(prepare.classify_pixel(pixels[unknown_index], 0.25, 0.65), -1)
        outside_corridor_index = prepare.grid_pixel_index(
            width, height, self.resolution, tuple(report["map_origin_after"]), 1.0, 0.4)
        self.assertEqual(prepare.classify_pixel(pixels[outside_corridor_index], 0.25, 0.65), -1)
        box_index = prepare.grid_pixel_index(
            width, height, self.resolution, tuple(report["map_origin_after"]), 1.495, -0.426)
        self.assertEqual(prepare.classify_pixel(pixels[box_index], 0.25, 0.65), 100)
        south_raster_obstacle_index = prepare.grid_pixel_index(
            width, height, self.resolution, tuple(report["map_origin_after"]), 2.145, -0.698)
        self.assertEqual(prepare.classify_pixel(
            pixels[south_raster_obstacle_index], 0.25, 0.65), 100)
        bypass_index = prepare.grid_pixel_index(
            width, height, self.resolution, tuple(report["map_origin_after"]), 2.85, -0.9)
        self.assertEqual(prepare.classify_pixel(pixels[bypass_index], 0.25, 0.65), 0)
        bypass_outside_index = prepare.grid_pixel_index(
            width, height, self.resolution, tuple(report["map_origin_after"]), 2.85, -0.60)
        self.assertEqual(prepare.classify_pixel(pixels[bypass_outside_index], 0.25, 0.65), -1)
        nav_metadata = {"width": width, "height": height, "resolution": self.resolution,
                        "origin": report["map_origin_after"], "frame_id": "map"}
        nav_data = [prepare.classify_pixel(value, 0.25, 0.65)
                for row in range(height-1, -1, -1)
                for value in pixels[row*width:(row+1)*width]]
        self.assertEqual(goal_probe.select_known_free_avoidance_goal(
            nav_metadata, nav_data, start=(0.55, 0.0)), (2.80, -1.05, 0))
        self.assertGreater(report["carved_free_cells"], 0)

    def test_sensor_seeded_map_preserves_cells_and_only_seeds_start_footprint(self):
        output_prefix = self.root/"sensor_only_map"
        report = prepare.prepare_sensor_seeded_map(self.yaml_path, output_prefix)
        width, height, pixels = prepare.read_pgm(report["prepared_map_pgm"])
        pad = 5  # ceil(robot_radius / SLAM resolution) = ceil(0.22 / 0.05)
        self.assertEqual((width, height), (self.width+2*pad, self.height+2*pad))
        self.assertEqual(report["scope"], "G4_SENSOR_DERIVED_MAP_WITH_START_FOOTPRINT_SEED")
        self.assertFalse(report["ground_truth_used_for_navigation_map"])
        self.assertGreater(report["start_footprint_seeded_cells"], 0)
        self.assertEqual(report["free_cells_added"], report["start_footprint_seeded_cells"])
        self.assertEqual(report["occupied_cells_removed"], 0)
        self.assertTrue(report["raw_slam_map_preserved"])
        self.assertEqual(self.pgm_path.read_bytes(), self.raw_pgm_bytes)
        self.assertEqual(report["source_map_yaml_sha256"],
                         __import__("hashlib").sha256(self.yaml_path.read_bytes()).hexdigest())
        self.assertEqual(report["source_map_pgm_sha256"],
                         __import__("hashlib").sha256(self.raw_pgm_bytes).hexdigest())

        nav_origin = report["origin_after"]
        self.assertAlmostEqual(nav_origin[0], self.origin[0]-pad*self.resolution)
        self.assertAlmostEqual(nav_origin[1], self.origin[1]-pad*self.resolution)
        unknown_at_start = prepare.grid_pixel_index(
            width, height, self.resolution, nav_origin, 0.0, 0.0)
        self.assertEqual(prepare.classify_pixel(
            pixels[unknown_at_start], 0.25, 0.65), 0)
        seeded_at_start = prepare.grid_pixel_index(
            width, height, self.resolution, nav_origin, 0.025, 0.0)
        self.assertEqual(prepare.classify_pixel(
            pixels[seeded_at_start], 0.25, 0.65), 0)

        observed_free = prepare.grid_pixel_index(
            width, height, self.resolution, nav_origin, 0.30, 0.0)
        self.assertEqual(prepare.classify_pixel(
            pixels[observed_free], 0.25, 0.65), 0)
        observed_occupied = prepare.grid_pixel_index(
            width, height, self.resolution, nav_origin, 1.495, -0.426)
        self.assertEqual(prepare.classify_pixel(
            pixels[observed_occupied], 0.25, 0.65), 100)

        outside_seed = prepare.grid_pixel_index(
            width, height, self.resolution, nav_origin, 0.30, 0.30)
        self.assertEqual(prepare.classify_pixel(
            pixels[outside_seed], 0.25, 0.65), -1)

        for row in range(height):
            for column in range(width):
                if (pad <= row < pad+self.height
                        and pad <= column < pad+self.width):
                    continue
                x = nav_origin[0]+(column+0.5)*self.resolution
                y = nav_origin[1]+(height-1-row+0.5)*self.resolution
                if math.hypot(x, y) <= prepare.ROBOT_RADIUS:
                    self.assertEqual(prepare.classify_pixel(
                        pixels[row*width+column], 0.25, 0.65), 0)
                else:
                    self.assertEqual(pixels[row*width+column], prepare.UNKNOWN_PIXEL)

    def test_sensor_seeded_mode_does_not_accept_mapping_report_or_carve_occupied_seed(self):
        from contextlib import redirect_stderr
        from io import StringIO
        import inspect

        self.assertNotIn("mapping_report_path",
                         inspect.signature(prepare.prepare_sensor_seeded_map).parameters)
        with redirect_stderr(StringIO()):
            result = prepare.main([
                "--yaml", str(self.yaml_path), "--mode", "sensor-seeded",
                "--mapping-report", str(self.mapping_path),
                "--output-prefix", str(self.root/"rejected_sensor_map")])
        self.assertEqual(result, 1)

        content = self.pgm_path.read_bytes()
        header_end = content.index(b"255\n")+4
        pixels = bytearray(content[header_end:])
        col = int((0.05-self.origin[0])//self.resolution)
        row_from_bottom = int((0.0-self.origin[1])//self.resolution)
        pixels[(self.height-1-row_from_bottom)*self.width+col] = 0
        self.pgm_path.write_bytes(content[:header_end]+pixels)
        with self.assertRaisesRegex(ValueError, "initial footprint overlaps occupied"):
            prepare.prepare_sensor_seeded_map(
                self.yaml_path, self.root/"rejected_occupied_seed")

    def test_initial_footprint_seed_matches_fortress_nav2_prior(self):
        package = Path(__file__).parents[1]/"linorobot2_navigation"
        common = (package/"config/navigation_common.yaml").read_text()
        config = (package/"config/navigation_fortress.yaml").read_text()
        self.assertEqual(prepare.ROBOT_RADIUS, 0.22)
        self.assertEqual(common.count(f"robot_radius: {prepare.ROBOT_RADIUS}"), 2)
        self.assertIn("initial_pose.x: 0.0", config)
        self.assertIn("initial_pose.y: 0.0", config)
        self.assertIn("initial_pose.yaw: 0.0", config)

    def test_g4_harness_defaults_to_sensor_only_and_keeps_fixture_opt_in(self):
        gate = Path(__file__).with_name("test_phase4_gate.sh").read_text()
        self.assertIn('MAP_MODE="sensor-seeded"', gate)
        self.assertIn("--mode sensor-seeded", gate)
        self.assertIn("--fixture-map", gate)
        self.assertIn('--mode fixture', gate)

    def test_refuses_to_carve_an_occupied_obstacle(self):
        content = self.pgm_path.read_bytes()
        header_end = content.index(b"255\n")+4
        pixels = bytearray(content[header_end:])
        col = int((0.15-self.origin[0])//self.resolution)
        row_from_bottom = int((0.0-self.origin[1])//self.resolution)
        pixels[(self.height-1-row_from_bottom)*self.width+col] = 0
        self.pgm_path.write_bytes(content[:header_end]+pixels)
        with self.assertRaisesRegex(ValueError, "intersects occupied"):
            prepare.prepare_map(self.yaml_path, self.mapping_path, self.output_prefix)

    def test_requires_a_passing_measured_start_route(self):
        report = json.loads(self.mapping_path.read_text())
        report["segments"][0]["pass"] = False
        self.mapping_path.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, "passing start-corridor"):
            prepare.prepare_map(self.yaml_path, self.mapping_path, self.output_prefix)


if __name__ == "__main__":
    unittest.main()